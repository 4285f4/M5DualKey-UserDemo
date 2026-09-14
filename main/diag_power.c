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
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "diag_power.h"

#include "esp_cpu.h"
#include "esp_log.h"
#include "esp_pm.h"
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
