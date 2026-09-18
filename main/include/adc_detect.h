/* SPDX-FileCopyrightText: 2025 M5Stack Technology CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include "esp_err.h"
#include "esp_sleep.h"
#include "esp_pm.h"
#include "led_strip.h"
#include "esp_adc/adc_oneshot.h"
#ifdef __cplusplus
extern "C" {
#endif

void test_light_sleep_button_wakeup(led_strip_handle_t led_strip);
void test_deep_sleep_button_wakeup(led_strip_handle_t led_strip);
void test_uart_wakeup(led_strip_handle_t led_strip);

// UART通信测试函数
void test_uart_communication(led_strip_handle_t led_strip);

// ADC检测测试函数
void test_adc_detection(led_strip_handle_t led_strip, adc_oneshot_unit_handle_t adc_handle);

// 低功耗测试函数
void low_power_test(void);

// 更新电源状态函数
void update_power_status(led_strip_handle_t led_strip);

// 轻量插线探测：只读 VBUS 一路（16 样本 ≈16ms），用于未插线态发现插线事件。
// 与 update_power_status() 共用同一把 NO_LIGHT_SLEEP 锁，所以读数可信。
bool adc_vbus_present(void);

// 电量等级 -> 低亮度颜色（红 <20% / 琥珀 <60% / 绿 >=60%）。
// 全工程唯一的"电量指示色"实现，充电指示与开机自检共用。
uint32_t power_indicator_color(int percentage);

// 全局电池状态变量
extern float g_battery_voltage;
extern int g_charging_status;
extern int g_battery_percentage;
extern float g_usb_voltage;
extern bool g_usb_connected;

#ifdef __cplusplus
}
#endif