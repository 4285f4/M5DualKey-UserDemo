/**
 * @file diag_power.c
 * @brief BLE 档功耗诊断实现。见 diag_power.h 的说明。
 *
 * 采样策略（开销优先）：
 *   - 每 5s 只做两个计数器的累加（CPU cycle 数 + 墙上时间），几乎不占 CPU；
 *   - 每 5min 才构造一次文本快照并写入 NVS —— 因为 esp_pm_dump_locks /
 *     esp_timer_dump 会格式化较多文本，不适合高频调用；
 *   - esp_pm 的电源模式分布（Mode: SLEEP/APB_MIN/...）是自开机起的累计统计，
 *     所以低频抓取不会漏掉历史信息。
 */
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "diag_power.h"

#include "esp_attr.h"
#include "esp_cpu.h"
#include "esp_log.h"
#include "esp_pm.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs.h"

static const char *TAG = "DIAGP";

#define NVS_NS   "diagp"
#define NVS_KEY  "rec"

/*!< 采样周期。取 5s 有两个考虑：① 累加本身极轻；② CCOUNT 是 32 位，
 *   在 160MHz 下约 26.8s 回绕一次，5s 的间隔确保差值绝不会跨圈。 */
#define SAMPLE_PERIOD_MS   5000
/*!< 文本快照 + 落盘周期。 */
#define FLUSH_PERIOD_MS    (5 * 60 * 1000)
/*!< CCOUNT 是 per-core 的，任务跨核迁移会让前后两次读数相减得到垃圾值，
 *   所以把诊断任务钉在核 0。PM 的 DFS 是全局的，核 0 的有效频率足以代表整体。 */
#define DIAG_TASK_CORE     0

static bool     s_started    = false;
static uint64_t s_cyc_total  = 0;   /*!< 累计 CPU 周期数 */
static uint64_t s_us_total   = 0;   /*!< 累计墙上时间 (us) */
static uint32_t s_cyc_prev   = 0;
static int64_t  s_us_prev    = 0;
static int64_t  s_t_start    = 0;
static int64_t  s_last_flush = 0;
static char    *s_snapshot   = NULL; /*!< 最近一次快照文本（堆，由 open_memstream 分配） */

/*!< --- CENTER 误读的 NVS 日志节流（2026-09-15）---
 *   运行期读到 CENTER 档 = 0 是**确定性的误读**（见 main.cpp `adc_switch_task` 里的
 *   守卫；light sleep 下被驱动为高的那一路会系统性塌陷到阈值以下），且每约 33s 复现一次。
 *   上一版每条都落 NVS，把"单次开机 60 行"的追加配额与事件日志总量瞬间吃光，反而
 *   挤掉真正有价值的 BOOT# / 重启记录（实测 BLE 档静置时会看到满屏 DECIDE）。
 *   改为：**首次 + 之后每 30 分钟一条**，行内带累计次数 —— 既不丢观测也不污染日志。
 *   ⚠️ 0 == DIP_SWITCH_POS_CENTER，该宏定义在 main.cpp:105；本文件不包含 main 的头，
 *      故此处用本地别名，若将来改档位编码，两边必须一起改。 */
#define DIAG_POS_CENTER                  0
#define DIAG_CENTER_LOG_MIN_INTERVAL_MS  (30 * 60 * 1000)
static uint32_t s_center_misreads    = 0;  /*!< 本次上电累计的 CENTER 误读次数 */
static int64_t  s_center_last_log_ms = 0;  /*!< 上次为误读落 NVS 的时刻（本机 uptime ms） */

static void build_snapshot(void)
{
    char  *locks  = NULL;
    char  *timers = NULL;
    size_t locks_len  = 0;
    size_t timers_len = 0;

    /*!< esp_pm_dump_locks 只在 CONFIG_PM_PROFILING 下才带持有次数/时长/模式占比，
     *   未开启时仍会列出当前存在的锁。 */
    FILE *fl = open_memstream(&locks, &locks_len);
    if (fl != NULL) {
        esp_pm_dump_locks(fl);
        fclose(fl);
    }

    /*!< esp_timer_dump 列出当前活跃的 esp_timer；CONFIG_ESP_TIMER_PROFILING
     *   开启后会额外带上名字、启动次数与回调总耗时。 */
    FILE *ft = open_memstream(&timers, &timers_len);
    if (ft != NULL) {
        esp_timer_dump(ft);
        fclose(ft);
    }

    char  *out     = NULL;
    size_t out_len = 0;
    FILE  *fo      = open_memstream(&out, &out_len);
    if (fo == NULL) {
        free(locks);
        free(timers);
        return;
    }

    const int64_t up_s    = (esp_timer_get_time() - s_t_start) / 1000000;
    const double  eff_mhz = (s_us_total != 0) ? ((double)s_cyc_total / (double)s_us_total) : 0.0;

    fprintf(fo, "=== DualKey BLE-only power diag ===\n");
    fprintf(fo, "uptime        : %lld s\n", (long long)up_s);
    fprintf(fo, "sample window : %llu us\n", (unsigned long long)s_us_total);
    fprintf(fo, "cpu cycles    : %llu\n", (unsigned long long)s_cyc_total);
    fprintf(fo, "effective cpu : %.2f MHz   (40 = lowest DFS step, 160 = never downscaled)\n", eff_mhz);
    fprintf(fo, "\n--- PM locks / power mode distribution ---\n");
    if (locks != NULL && locks_len > 0) {
        fwrite(locks, 1, locks_len, fo);
    } else {
        fprintf(fo, "(unavailable)\n");
    }
    fprintf(fo, "\n--- active esp_timers ---\n");
    if (timers != NULL && timers_len > 0) {
        fwrite(timers, 1, timers_len, fo);
    } else {
        fprintf(fo, "(unavailable)\n");
    }

    fclose(fo);
    free(locks);
    free(timers);

    free(s_snapshot);
    s_snapshot = out;
}

void diag_power_flush(void)
{
    /*!< ⚠️ 关键守卫：只有本次上电真的跑过采样任务时才允许落盘。
     *   WiFi 档不会调用 diag_power_start()，但 adc_switch_task 在跨档重启前会无条件
     *   调用本函数 —— 若不做这个判断，它会用一份全 0 的静止快照（s_cyc_total 等
     *   均为 0）覆盖掉刚在 BLE 档测到的有效数据，导致读数永远为 0。 */
    if (!s_started) {
        return;
    }

    if (s_snapshot == NULL) {
        build_snapshot();
    }
    if (s_snapshot == NULL) {
        return;
    }

    nvs_handle_t h;
    if (nvs_open(NVS_NS, NVS_READWRITE, &h) != ESP_OK) {
        ESP_LOGW(TAG, "nvs_open failed");
        return;
    }
    if (nvs_set_str(h, NVS_KEY, s_snapshot) == ESP_OK) {
        nvs_commit(h);
    }
    nvs_close(h);
}

char *diag_power_load(void)
{
    nvs_handle_t h;
    if (nvs_open(NVS_NS, NVS_READONLY, &h) != ESP_OK) {
        return NULL;
    }

    size_t len = 0;
    char  *buf = NULL;
    if (nvs_get_str(h, NVS_KEY, NULL, &len) == ESP_OK && len > 0) {
        buf = malloc(len);
        if (buf != NULL && nvs_get_str(h, NVS_KEY, buf, &len) != ESP_OK) {
            free(buf);
            buf = NULL;
        }
    }
    nvs_close(h);
    return buf;
}

void diag_power_clear(void)
{
    nvs_handle_t h;
    if (nvs_open(NVS_NS, NVS_READWRITE, &h) != ESP_OK) {
        return;
    }
    nvs_erase_key(h, NVS_KEY);
    nvs_commit(h);
    nvs_close(h);
}

/* ===========================================================================
 * 事件日志（NVS）：扛得住 OFF 档断电
 * =========================================================================== */

#define LOG_KEY        "log"
/*!< 单条字符串的上限。NVS 单个 string value 上限是 4000 字节，留足余量。 */
#define LOG_MAX_CHARS  1500
/*!< 单次开机内最多追加多少行。重启风暴时每轮会追加 3~4 行，设上限是为了
 *   不让 flash 在一个诊断周期里被反复擦写（NVS 每次 set_str 都要写一整条）。 */
#define LOG_MAX_APPENDS 60

static int s_log_appends = 0; /*!< 本次开机已追加行数（每次开机清零） */

void diag_log_event(const char *fmt, ...)
{
    if (s_log_appends >= LOG_MAX_APPENDS) {
        return;
    }
    s_log_appends++;

    char line[220];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(line, sizeof(line), fmt, ap);
    va_end(ap);

    nvs_handle_t h;
    if (nvs_open(NVS_NS, NVS_READWRITE, &h) != ESP_OK) {
        return;
    }

    size_t old_len = 0;
    char  *buf     = NULL;
    if (nvs_get_str(h, LOG_KEY, NULL, &old_len) == ESP_OK && old_len > 1) {
        buf = malloc(old_len);
        if (buf != NULL && nvs_get_str(h, LOG_KEY, buf, &old_len) != ESP_OK) {
            free(buf);
            buf = NULL;
        }
    }
    if (buf == NULL) {
        buf     = malloc(1);
        old_len = 0;
    }
    /*!< 统一按最大容量重新分配，后面的拼接就不必再关心旧长度，
     *   也不会出现"截断后剩余空间算错"的越界。 */
    char *bigger = realloc(buf, LOG_MAX_CHARS + 1);
    if (bigger == NULL) {
        free(buf);
        nvs_close(h);
        return;
    }
    buf = bigger;
    if (old_len == 0) {
        buf[0] = '\0'; /*!< 新建（或旧值无效）：从空串开始 */
    }

    const size_t line_len = strlen(line);
    /*!< 日志满了就从最前面整行地丢（可能需要丢多行）—— 保证尾部最新事件完整。 */
    while (strlen(buf) + line_len + 2 > LOG_MAX_CHARS) {
        char *nl = strchr(buf, '\n');
        if (nl == NULL) {
            buf[0] = '\0';
            break;
        }
        const size_t drop = (size_t)(nl - buf) + 1;
        memmove(buf, buf + drop, strlen(buf) - drop + 1);
    }

    if (line_len + 2 <= LOG_MAX_CHARS) {
        const size_t used = strlen(buf);
        memcpy(buf + used, line, line_len);
        buf[used + line_len]     = '\n';
        buf[used + line_len + 1] = '\0';
        if (nvs_set_str(h, LOG_KEY, buf) == ESP_OK) {
            nvs_commit(h);
        }
    }
    nvs_close(h);
    free(buf);
}

char *diag_log_load(void)
{
    nvs_handle_t h;
    if (nvs_open(NVS_NS, NVS_READONLY, &h) != ESP_OK) {
        return NULL;
    }

    size_t len = 0;
    char  *buf = NULL;
    if (nvs_get_str(h, LOG_KEY, NULL, &len) == ESP_OK && len > 0) {
        buf = malloc(len);
        if (buf != NULL && nvs_get_str(h, LOG_KEY, buf, &len) != ESP_OK) {
            free(buf);
            buf = NULL;
        }
    }
    nvs_close(h);
    return buf;
}

void diag_log_clear(void)
{
    nvs_handle_t h;
    if (nvs_open(NVS_NS, NVS_READWRITE, &h) != ESP_OK) {
        return;
    }
    nvs_erase_key(h, LOG_KEY);
    nvs_commit(h);
    nvs_close(h);
}

void diag_note_dip_view(int switch_pos, int derived, int raw_ble, int raw_wifi)
{
    static int last_switch  = -999;
    static int last_derived = -999;

    /*!< update_device_status() 以 ~2Hz 运行，只在"结论发生变化"时落盘：
     *   不加节流会立刻吃满单次开机的追加配额（LOG_MAX_APPENDS=60），
     *   而且每次 set_str 都是对 flash 的写入。结论稳定时最多记一两行，
     *   这一两行恰好就是判断"switch_pos 与硬件实际档位是否分歧"所需的证据。 */
    if (switch_pos == last_switch && derived == last_derived) {
        return;
    }
    last_switch  = switch_pos;
    last_derived = derived;

    diag_log_event("VIEW  sw_pos=%d derived=%d raw(%d,%d) uptime=%llds %s", switch_pos, derived, raw_ble, raw_wifi,
                   (long long)(esp_timer_get_time() / 1000000), (switch_pos == derived) ? "MATCH" : "MISMATCH");
}

static void diag_task(void *arg)
{
    (void)arg;

    s_t_start    = esp_timer_get_time();
    s_us_prev    = s_t_start;
    s_cyc_prev   = esp_cpu_get_cycle_count();
    s_last_flush = s_t_start;

    ESP_LOGI(TAG, "sampling started (sample=%d ms, flush=%d s)", SAMPLE_PERIOD_MS, FLUSH_PERIOD_MS / 1000);

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(SAMPLE_PERIOD_MS));

        const uint32_t cyc = esp_cpu_get_cycle_count();
        const int64_t  us  = esp_timer_get_time();

        /*!< 无符号 32 位相减天然处理回绕。 */
        s_cyc_total += (uint32_t)(cyc - s_cyc_prev);
        s_us_total  += (uint64_t)(us - s_us_prev);
        s_cyc_prev   = cyc;
        s_us_prev    = us;

        if ((us - s_last_flush) >= ((int64_t)FLUSH_PERIOD_MS * 1000)) {
            build_snapshot();
            diag_power_flush();
            s_last_flush = us;
        }
    }
}

void diag_power_start(void)
{
    if (s_started) {
        return;
    }
    s_started = true;
    if (xTaskCreatePinnedToCore(diag_task, "diag_power", 5120, NULL, 3, NULL, DIAG_TASK_CORE) != pdPASS) {
        s_started = false;
        ESP_LOGW(TAG, "failed to start diag task");
    }
}

/* ===========================================================================
 * 重启 / 跨档取证（RTC 内存）
 *
 * 为什么用 RTC 内存而不是 NVS：
 *   - 误读循环里设备每几秒重启一次，每次都写 NVS 会白白磨损 flash；
 *   - RTC 内存在 esp_restart()（软复位）之后原样保留，正好覆盖"跨档重启"路径；
 *   - 唯一会丢的场景是彻底断电 —— 取证期间不要让用户拔电池。
 * =========================================================================== */

#define BOOT_REC_MAGIC       0xD1A6B007u
/*!< "上一次软复位前存活时间 ≤ 该值"即视为异常短命，据此判定重启风暴。 */
#define BOOT_STORM_UPTIME_MS 15000

typedef struct {
    uint32_t magic;
    uint32_t boot_seq;          /*!< 上电以来第几次启动 */
    int      reset_reason;      /*!< 本次启动的原因 (= 上一次重启的类型) */
    int      boot_pos;          /*!< 本次启动判定的档位 */
    int      boot_ble_raw;      /*!< 开机瞬间 BLE 通道原始值（light sleep 未开，可信基准） */
    int      boot_wifi_raw;     /*!< 开机瞬间 WiFi 通道原始值 */
    int64_t  uptime_ms;         /*!< 本次运行已存活时长，由 tick 刷新 */
    int64_t  last_sw_uptime_ms; /*!< 上一次软复位前存活了多久（0 = 尚无记录） */
    /* 最近一次运行期档位判定背后的原始样本 */
    int      dip_r0, dip_r1;    /*!< adc_oneshot_read 返回码 (0 = ESP_OK) */
    int      dip_v0, dip_v1;    /*!< ADC 原始读数，阈值 2000 */
    int      dip_from, dip_to;  /*!< 判定前后的档位 */
    int      dip_hits;          /*!< 提交时的去抖命中数 */
    int64_t  dip_at_ms;         /*!< 判定发生的时刻（开机后 ms） */
    /* 自证统计 */
    uint32_t verify_total;
    uint32_t verify_rejected;   /*!< 被判为休眠误读的次数 */
    uint32_t storm_suppressed;  /*!< 熔断次数 */
    int      last_recheck;      /*!< 最近一次复读得到的档位 */
} diag_boot_rec_t;

RTC_DATA_ATTR static diag_boot_rec_t s_boot_rec;

static void boot_rec_ensure(void)
{
    if (s_boot_rec.magic == BOOT_REC_MAGIC) {
        return;
    }
    memset(&s_boot_rec, 0, sizeof(s_boot_rec));
    s_boot_rec.magic        = BOOT_REC_MAGIC;
    s_boot_rec.reset_reason = -1;
    s_boot_rec.boot_pos     = -1;
    s_boot_rec.boot_ble_raw = s_boot_rec.boot_wifi_raw = -1;
    s_boot_rec.dip_r0 = s_boot_rec.dip_r1 = -1;
    s_boot_rec.dip_v0 = s_boot_rec.dip_v1 = -1;
    s_boot_rec.dip_from = s_boot_rec.dip_to = -1;
    s_boot_rec.last_recheck = -1;
}

void diag_boot_note_boot(int boot_pos, int boot_ble_raw, int boot_wifi_raw)
{
    boot_rec_ensure();
    s_boot_rec.boot_seq++;
    s_boot_rec.reset_reason = (int)esp_reset_reason();
    s_boot_rec.boot_pos     = boot_pos;
    s_boot_rec.boot_ble_raw = boot_ble_raw;
    s_boot_rec.boot_wifi_raw = boot_wifi_raw;
    s_boot_rec.uptime_ms    = 0;

    /*!< 新的一次开机：重置本次的追加配额，并把启动事件落 NVS。
     *   RTC 记录会被 OFF 档断电清零，所以这一行是唯一能跨断电的证据。 */
    s_log_appends = 0;
    diag_log_event("BOOT#%u reason=%d pos=%d raw(ble=%d wifi=%d) prev_sw_up=%lldms", (unsigned)s_boot_rec.boot_seq,
                   s_boot_rec.reset_reason, boot_pos, boot_ble_raw, boot_wifi_raw,
                   (long long)s_boot_rec.last_sw_uptime_ms);
}

void diag_boot_note_uptime_ms(int64_t ms)
{
    boot_rec_ensure();
    s_boot_rec.uptime_ms = ms;
}

void diag_boot_note_dip_sample(int r0, int r1, int v0, int v1, int from, int to, int hits)
{
    boot_rec_ensure();
    s_boot_rec.dip_r0    = r0;
    s_boot_rec.dip_r1    = r1;
    s_boot_rec.dip_v0    = v0;
    s_boot_rec.dip_v1    = v1;
    s_boot_rec.dip_from  = from;
    s_boot_rec.dip_to    = to;
    s_boot_rec.dip_hits  = hits;
    s_boot_rec.dip_at_ms = s_boot_rec.uptime_ms;

    if (to == DIAG_POS_CENTER) {
        /*!< 误读路径：节流落盘（首次 + 之后每 30 分钟一次），行内带累计次数。 */
        s_center_misreads++;
        const int64_t up_ms = s_boot_rec.uptime_ms;
        if ((s_center_misreads == 1) || (up_ms - s_center_last_log_ms >= DIAG_CENTER_LOG_MIN_INTERVAL_MS)) {
            s_center_last_log_ms = up_ms;
            diag_log_event("DIPCENTER@%lldms x%u raw(%d,%d) hits=%d rc(%d,%d)", (long long)up_ms,
                           (unsigned)s_center_misreads, v0, v1, hits, r0, r1);
        }
    } else {
        diag_log_event("DECIDE@%lldms %d->%d hits=%d rc(%d,%d) raw(%d,%d)", (long long)s_boot_rec.uptime_ms, from, to,
                       hits, r0, r1, v0, v1);
    }
}

void diag_boot_note_verify(int from, int to, int recheck, bool confirmed)
{
    boot_rec_ensure();
    (void)from;
    (void)to;
    s_boot_rec.verify_total++;
    s_boot_rec.last_recheck = recheck;
    if (!confirmed) {
        s_boot_rec.verify_rejected++;
    }

    diag_log_event("VERIFY cand=%d recheck=%d boot_raw(ble=%d wifi=%d) -> %s", to, recheck,
                   s_boot_rec.boot_ble_raw, s_boot_rec.boot_wifi_raw, confirmed ? "CONFIRMED" : "FAKE");
}

void diag_boot_note_confirmed_restart(void)
{
    boot_rec_ensure();
    s_boot_rec.last_sw_uptime_ms = s_boot_rec.uptime_ms;
    diag_log_event("RESTART@%lldms (cross-position, restarting to apply)", (long long)s_boot_rec.uptime_ms);
}

bool diag_boot_is_restart_storm(void)
{
    boot_rec_ensure();
    /*!< 判据只用"上一次是不是我们自己重启的、且活得极短"。
     *   ⚠️ 特意不要求 reset reason == ESP_RST_SW：实测风暴期间拿到过非 SW 的
     *   复位原因，若把它写进前置条件，熔断就永远不会生效（这正是第一版取证固件
     *   没能兜住重启循环的原因）。last_sw_uptime_ms 只有 diag_boot_note_confirmed_restart()
     *   会写，而 RTC 内存被断电清零，所以"非 0 且很短"就足以认定循环。 */
    if (s_boot_rec.last_sw_uptime_ms <= 0) {
        return false;
    }
    return s_boot_rec.last_sw_uptime_ms <= BOOT_STORM_UPTIME_MS;
}

void diag_boot_note_storm_suppressed(void)
{
    boot_rec_ensure();
    s_boot_rec.storm_suppressed++;
    diag_log_event("FUSE   storm suppressed (reason=%d prev_up=%lldms)", s_boot_rec.reset_reason,
                   (long long)s_boot_rec.last_sw_uptime_ms);
    /*!< 清掉短命标记，避免把之后正常的用户拨档（此时 uptime 已经很长）也拦下来。 */
    s_boot_rec.last_sw_uptime_ms = 0;
}

static const char *reset_reason_str(int r)
{
    switch (r) {
        case ESP_RST_UNKNOWN:
            return "UNKNOWN";
        case ESP_RST_POWERON:
            return "POWERON";
        case ESP_RST_EXT:
            return "EXT_PIN";
        case ESP_RST_SW:
            return "SW (esp_restart)";
        case ESP_RST_PANIC:
            return "PANIC";
        case ESP_RST_INT_WDT:
            return "INT_WDT";
        case ESP_RST_TASK_WDT:
            return "TASK_WDT";
        case ESP_RST_WDT:
            return "WDT";
        case ESP_RST_DEEPSLEEP:
            return "DEEPSLEEP";
        case ESP_RST_BROWNOUT:
            return "BROWNOUT";
        case ESP_RST_SDIO:
            return "SDIO";
        default:
            return "?";
    }
}

char *diag_boot_load(void)
{
    boot_rec_ensure();

    char  *out = NULL;
    size_t n   = 0;
    FILE  *fo  = open_memstream(&out, &n);
    if (fo == NULL) {
        return NULL;
    }

    fprintf(fo, "=== boot / reset / DIP forensics (RTC memory) ===\n");
    fprintf(fo, "boot count     : %u\n", (unsigned)s_boot_rec.boot_seq);
    fprintf(fo, "reset reason   : %d (%s)   <- why THIS boot started\n", s_boot_rec.reset_reason,
            reset_reason_str(s_boot_rec.reset_reason));
    fprintf(fo, "boot dip pos   : %d   (0=center 1=wifi 2=ble)\n", s_boot_rec.boot_pos);
    fprintf(fo, "boot adc raw   : ch_ble=%d ch_wifi=%d   <- taken at boot, light sleep OFF (trustworthy baseline)\n",
            s_boot_rec.boot_ble_raw, s_boot_rec.boot_wifi_raw);
    fprintf(fo, "uptime now     : %lld s\n", (long long)(s_boot_rec.uptime_ms / 1000));
    fprintf(fo, "prev sw uptime : %lld ms   (<= %d ms => restart storm)\n", (long long)s_boot_rec.last_sw_uptime_ms,
            BOOT_STORM_UPTIME_MS);
    fprintf(fo, "\n--- last runtime DIP decision sample ---\n");
    fprintf(fo, "at uptime      : %lld s\n", (long long)(s_boot_rec.dip_at_ms / 1000));
    fprintf(fo, "adc read rc    : ch_ble=%d ch_wifi=%d  (0 = ESP_OK)\n", s_boot_rec.dip_r0, s_boot_rec.dip_r1);
    fprintf(fo, "adc raw value  : ch_ble=%d ch_wifi=%d  (threshold = 2000)\n", s_boot_rec.dip_v0, s_boot_rec.dip_v1);
    fprintf(fo, "pos decision   : %d -> %d  (debounce hits = %d)\n", s_boot_rec.dip_from, s_boot_rec.dip_to,
            s_boot_rec.dip_hits);
    fprintf(fo, "\n--- light-sleep re-verify ---\n");
    fprintf(fo, "verify runs    : %u\n", (unsigned)s_boot_rec.verify_total);
    fprintf(fo, "rejected(fake) : %u   <- judged as light-sleep misread\n", (unsigned)s_boot_rec.verify_rejected);
    fprintf(fo, "storm fuse     : %u   <- restarts suppressed\n", (unsigned)s_boot_rec.storm_suppressed);
    fprintf(fo, "last recheck   : %d\n", s_boot_rec.last_recheck);

    fclose(fo);
    return out;
}
