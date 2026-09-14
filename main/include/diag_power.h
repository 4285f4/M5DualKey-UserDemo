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

/** 记录本次启动（reset reason + 开机判定档位）。在 app_main 判完档位后调用。 */
void diag_boot_note_boot(int boot_pos);

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
