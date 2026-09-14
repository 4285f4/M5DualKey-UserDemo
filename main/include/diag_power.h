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
