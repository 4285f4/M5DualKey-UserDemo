/**
 * @file diag_power.h
 * @brief BLE 档功耗诊断：运行期电源数据落 NVS，切到 WiFi 档后经 HTTP /diag 读回。
 *
 * 为什么非得这么绕：
 *   - BLE 档跳过了 TinyUSB，设备在 USB 上完全不可见 -> 拿不到控制台日志；
 *   - WiFi 档走 TinyUSB（303A:8000），而 CDC 被注释掉了 -> 同样读不到日志。
 * 于是唯一可用的观测通道是「BLE 档把数据写进 NVS -> 切档重启 -> WiFi 档用网页读」。
 * 这也是为什么 adc_switch_task 在跨档 esp_restart() 之前必须先调 diag_power_flush()。
 *
 * 采样只做两个计数器累加，开销可忽略；只有落盘时才构造文本快照。
 */
#pragma once

#include <stdbool.h>

/** 启动后台采样任务（BLE 档调用）。幂等；重复调用无副作用。 */
void diag_power_start(void);

/** 把当前统计快照写入 NVS。跨档 esp_restart() 之前必须调用，否则本段数据丢失。 */
void diag_power_flush(void);

/**
 * @brief 读取上一次落盘的诊断文本。
 * @return 以 '\0' 结尾的堆字符串，调用方负责 free()；无记录时返回 NULL。
 */
char *diag_power_load(void);

/** 删除 NVS 中的诊断记录（下一次 BLE 档运行会重新写入）。 */
void diag_power_clear(void);

/* ---------------------------------------------------------------------------
 * 重启 / 跨档取证
 *
 * 背景：BLE 档（light sleep 生效后）USB 与 WiFi 双静默，一旦出现"反复重启"
 * 就完全看不到现场。而 esp_restart() 是软复位 —— RTC 内存在软复位后保留 ——
 * 于是把现场写进 RTC 内存，等设备被切到 WiFi 档后再经 /diag 读回。
 *
 * 这套记录同时用于判断"反复重启"的真凶：
 *   - reset_reason == ESP_RST_SW  -> 是 adc_switch_task 判定跨档后主动重启；
 *   - reset_reason == PANIC/WDT   -> 与档位判定无关，另有崩溃点；
 *   - dip_v0/dip_v1               -> 误读时的 ADC 原始值，用来区分"读不到"
 *                                    （≈0，pad/通道失效）还是"读偏了"（阈值问题）。
 * ------------------------------------------------------------------------- */

/**
 * @brief 记录本次启动（reset reason + 开机判定档位 + 开机瞬间的原始 ADC 采样）。
 *
 * ⚠️ 为什么必须带上开机 ADC 原始值：开机时 light sleep 尚未开启，此时的读数
 *    （8 点平均后的通道原始值）是"可信基准"。运行期出现档位误判时，把它与
 *    dip_v0/dip_v1 对比，就能一眼区分是"通道整个失效（都掉到 0）"还是
 *    "读数漂移（只是跌到阈值以下）"。
 *
 * @param boot_pos      开机判定的档位 (0=center 1=wifi 2=ble)
 * @param boot_ble_raw  BLE 通道原始值 (ADC raw, -1 = 读取失败)
 * @param boot_wifi_raw WiFi 通道原始值
 */
void diag_boot_note_boot(int boot_pos, int boot_ble_raw, int boot_wifi_raw);

/** 刷新存活时长（建议每秒一次）。崩溃重启后可据此推算崩溃发生在开机后多久。 */
void diag_boot_note_uptime_ms(int64_t ms);

/** 记录一次运行期"档位判定"背后的原始 ADC 样本（无论最终是否重启）。
 *  ⚠️ 判定为 CENTER(0) 时视为休眠误读：NVS 事件日志**节流**为"首次 + 之后每 30 分钟
 *  一条"（行内带累计次数），避免每约 33s 一条把日志刷爆；RTC 字段仍逐次更新。 */
void diag_boot_note_dip_sample(int r0, int r1, int v0, int v1, int from, int to, int hits);

/** 记录一次"跨档自证"的结果。confirmed=false 表示复读后判为休眠导致的误读。 */
void diag_boot_note_verify(int from, int to, int recheck, bool confirmed);

/** 在真正执行跨档 esp_restart() 前调用，记录本次存活时长供下次熔断判定。 */
void diag_boot_note_confirmed_restart(void);

/**
 * @brief 是否处于"重启风暴"（短时间内反复软复位）。
 *
 * 判定依据：本次启动的原因是软件复位，且上一次运行只存活了很短时间 ——
 * 正常用户拨档时设备已运行较久，只有"误读循环"才会出现连续短命重启。
 * 命中后调用方应放弃本次重启（熔断），否则会无限循环。
 */
bool diag_boot_is_restart_storm(void);

/** 标记熔断已生效，避免后续把正常的用户拨档也误判成风暴。 */
void diag_boot_note_storm_suppressed(void);

/** 把取证记录格式化成文本（堆字符串，调用方 free()）。 */
char *diag_boot_load(void);

/* ---------------------------------------------------------------------------
 * 事件日志（NVS，扛得住断电）
 *
 * 为什么不能只用 RTC 内存：**中间 OFF 档会切断供电** —— 实测用户从 BLE 档拨到
 * WiFi 档后，本次启动的 reset reason 是 POWERON 而不是 SW，说明拨档过程必然
 * 经过一次真实断电，RTC_DATA_ATTR 里的取证记录会被整块清零（实测 boot_seq=1、
 * 所有 ADC 样本为 -1）。所以关键事件必须落到 flash。
 *
 * 写入策略：单条字符串、每次追加一行、超出上限就丢弃最旧的行；同时限制单次
 * 开机内的追加次数（重启风暴时每轮都会追加，不能让 flash 被反复擦写）。
 * ------------------------------------------------------------------------- */

/** 追加一行事件到 NVS 日志（printf 风格）。重启风暴期间有次数上限，超出即静默丢弃。 */
void diag_log_event(const char *fmt, ...) __attribute__((format(printf, 1, 2)));

/** 读回 NVS 事件日志（堆字符串，调用方 free()；无记录时返回 NULL）。 */
char *diag_log_load(void);

/** 清空 NVS 事件日志。 */
void diag_log_clear(void);

/**
 * @brief 记录一次"网页显示用档位"的派生结果，用于排查显示与实际档位不一致。
 *
 * 背景（2026-09-15 实测）：设备实物在 WiFi 档，网页收到的 dip_switch_pos 却是 0
 * （中间档），而同一时刻两路原始采样明确显示 WiFi 通道越过了阈值。为把
 * "档位判定值 switch_pos"与"由原始 ADC 派生的显示值"的分歧留住证据，在
 * update_device_status() 里调用本函数。
 *
 * 内部按"结论是否变化"节流：该函数会被 ~2Hz 调用，不加节流会瞬间吃满单次
 * 开机的追加配额（60 行）并白白磨损 flash。
 *
 * @param switch_pos  全局档位判定值 (0=center 1=wifi 2=ble)
 * @param derived     由 switch_1_value / switch_2_value 派生出的档位
 * @param raw_ble     BLE 通道原始值
 * @param raw_wifi    WiFi 通道原始值
 */
void diag_note_dip_view(int switch_pos, int derived, int raw_ble, int raw_wifi);

/* ---------------------------------------------------------------------------
 * 运行期即时取证（纯 RAM，绝不写 NVS）
 *
 * 动机（2026-09-15，排查"蓝牙连上约 1s 又断、反复循环"）：
 *   第一反应是在 BT 回调里用 diag_log_event 落 NVS，但那有两个副作用 ——
 *     ① BT 回调里做 NVS 写 = flash 擦写，会停 cache、阻塞协议栈几十 ms，
 *        本身就可能把连接搞断（"记录问题"变成"制造问题"）；
 *     ② NVS 分区只有 24KB，事件日志又是"整条 blob 重写"，高频事件会迅速磨损。
 *   所以运行期事件一律只压 RAM 环形缓冲，由 /diag 现场读出。
 *
 * 为什么这样也够用：**WiFi 档同样会初始化 BLE**（main.cpp: `if (!usb_only_mode)
 *   ble_hid_init();`），因此可以在 WiFi 档一边复现重连、一边实时拉 /diag ——
 *   完全不需要 NVS 中转。
 *
 * ⚠️ 代价：RAM 记录扛不住断电/跨档，只对"本次运行"有效。需要跨档保留的
 *    关键事件仍走 diag_log_event（NVS）。
 * ------------------------------------------------------------------------- */

/** 运行期事件种类（diag_rt_push 的 kind 参数）。 */
enum {
    DIAG_RT_BLE_CONNECT   = 1,  /*!< 连接建立 */
    DIAG_RT_BLE_DISCONN   = 2,  /*!< a = HCI disconnect reason（0x08=超时 0x13/0x05=认证类） */
    DIAG_RT_BLE_AUTH      = 3,  /*!< a = success(0/1), b = fail_reason */
    DIAG_RT_BLE_ADV_START = 4,  /*!< a = status */
    DIAG_RT_BLE_CONNPARAM = 5,  /*!< a = status, b = min_int, c = max_int（单位 1.25ms） */
    DIAG_RT_BLE_SECREQ    = 6,  /*!< 对端发起安全请求 */
    DIAG_RT_LED_WAKEIND   = 10, /*!< a = ws2812_enable, b = strip_ok */
    DIAG_RT_LED_KEYFLASH  = 11, /*!< a = led, b = level, c = ws2812_enable, d = strip_ok */
    DIAG_RT_LED_SLEEPIND  = 12, /*!< a = ws2812_enable, b = strip_ok */
    DIAG_RT_KEYPRESS      = 13, /*!< a = change_num, b = pressed_num, c = rgb_en, d = suspend */
};

/** 压入一条运行期取证（RAM，可从中断/回调上下文调用；无阻塞、不写 flash）。 */
void diag_rt_push(int kind, int a, int b, int c, int d);

/** 把运行期取证 + NVS 占用统计格式化成文本（堆字符串，调用方 free()；失败返回 NULL）。 */
char *diag_rt_report(void);
