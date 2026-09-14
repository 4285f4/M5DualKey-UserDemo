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

/** 记录一次运行期"档位判定"背后的原始 ADC 样本（无论最终是否重启）。 */
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
