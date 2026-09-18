/*
 *SPDX-FileCopyrightText: 2025 M5Stack Technology CO LTD
 *
 *SPDX-License-Identifier: MIT
 */

#include "adc_detect.h"
#include "memory.h"
#include "esp_log.h"
#include "esp_sleep.h"
#include "esp_timer.h"
#include "esp_pm.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "bsp/esp-bsp.h"
#include "rgb_matrix.h"
#include "settings.h"
#include "bsp/esp_duo.h"
#include "driver/uart.h"
#include "esp_crc.h"
#include "string.h"
#include "driver/rtc_io.h"
#include "driver/adc.h"

static const char *TAG = "test_case";

// LED互斥锁
static SemaphoreHandle_t led_mutex = NULL;

// ADC检测配置
#define ADC_BAT_GPIO  10  // 电池电压检测GPIO
#define ADC_CHRG_GPIO 9   // 充电状态检测GPIO
#define ADC_VBUS_GPIO 2   // USB电压检测GPIO

#define ADC_BAT_CHANNEL  ADC_CHANNEL_9  // GPIO10对应ADC1_CH9
#define ADC_CHRG_CHANNEL ADC_CHANNEL_8  // GPIO9对应ADC1_CH8
#define ADC_VBUS_CHANNEL ADC_CHANNEL_1  // GPIO2对应ADC1_CH1

#define ADC_ATTEN        ADC_ATTEN_DB_11  // 衰减11dB, 测量范围0-3.3V
#define ADC_BITWIDTH     ADC_BITWIDTH_12  // 12位精度
#define ADC_SAMPLE_COUNT 64               // 采样次数，用于平均滤波

// 分压比例定义
#define VOLTAGE_DIVIDER_RATIO_BAT  1.51f  // 电池检测: (51+100)/100 = 1.51
#define VOLTAGE_DIVIDER_RATIO_VBUS 1.51f  // VBUS检测: (51+100)/100 = 1.51
#define VREF_VOLTAGE               3333   // 参考电压3.3V (mV) Vbat*0.662
#define ADC_MAX_VALUE              4095   // 12位ADC最大值

// 充电状态电压阈值 (mV)
#define CHRG_NOT_CHARGING 3000  // 没充电：>3.0V
#define CHRG_CHARGING     1400  // 充电中：1.4V-1.9V
#define CHRG_CHARGED      1900  // 充满电：1.9V-3.0V

// ADC检测结果结构
typedef struct {
    float battery_voltage;      // 电池电压 (V)
    float vbus_voltage;         // USB电压 (V)
    float chrg_voltage;         // 充电检测电压 (V)
    const char *charge_status;  // 充电状态字符串
    bool usb_connected;         // USB连接状态
    bool battery_low;           // 电池低电量标志
} adc_result_t;

// ADC句柄
static adc_oneshot_unit_handle_t adc_handle = NULL;

/*!< ── ADC 采样期间的 light sleep 禁用锁（2026-09-18）────────────────────────
 *
 *   问题：light sleep 会切断数字外设的电源域，`adc_oneshot_read()` 在休眠期间
 *   返回无意义的值。本工程已经为拨码通道栽过一次（REFERENCE §J：休眠下读数从
 *   ~3026 塌到 ~1275，蓝牙档被读成中间档 ⇒ 跨档重启死循环），为此专门写了
 *   `dip_switch_verify_crossing()` 做"关休眠 + 重建通道 + 复读"。
 *
 *   但那把补丁**只打在拨码的两路通道上**，电池/充电/VBUS 这三路一直是裸奔的：
 *   `adc_read_average()` 是 64 次采样、每次 `vTaskDelay(1)`，而 tick=1000Hz，
 *   也就是每个样本点都让出 CPU 一次 —— 蓝牙档开了 light sleep，这些间隙正好
 *   被 tickless idle 用来入睡，醒来后 ADC 数字接口已经失效 ⇒ 读数系统性虚低。
 *
 *   实测后果：WiFi 档（无休眠）读 4.159V/87%，蓝牙档（有休眠）读出的值低到
 *   落进琥珀段（<60%）。同一块电池、同一份固件，只因为档位不同就变了个颜色。
 *
 *   ⇒ 采样全程持 `ESP_PM_NO_LIGHT_SLEEP`。
 *     代价说明：**插线时这把锁是空操作** —— USB-Serial-JTAG 连接监控一旦检测到
 *     主机，本身就持有 `ESP_PM_NO_LIGHT_SLEEP`（REFERENCE §0），light sleep
 *     已经 100% 被禁。所以"插线态高频采样"不影响功耗；真正有成本的是"未插线态
 *     的插线探测"，那条路走 `adc_vbus_present()`，只读一路、样本数也降到 16。 */
static esp_pm_lock_handle_t s_adc_pm_lock = NULL;

static void adc_pm_lock_acquire(void)
{
#if CONFIG_PM_ENABLE
    if (s_adc_pm_lock != NULL) {
        esp_pm_lock_acquire(s_adc_pm_lock);
    }
#endif
}

static void adc_pm_lock_release(void)
{
#if CONFIG_PM_ENABLE
    if (s_adc_pm_lock != NULL) {
        esp_pm_lock_release(s_adc_pm_lock);
    }
#endif
}

// 全局变量，供其他模块访问
float g_battery_voltage  = 3.7f;
int g_charging_status    = 0;
int g_battery_percentage = 75;
float g_usb_voltage      = 0.0f;
bool g_usb_connected     = false;

// 安全的LED操作函数
static void safe_led_set_and_refresh(led_strip_handle_t led_strip, int index, uint8_t r, uint8_t g, uint8_t b,
                                     int delay_ms)
{
    if (led_mutex == NULL || led_strip == NULL) {
        return;
    }

    if (xSemaphoreTake(led_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        led_strip_set_pixel(led_strip, index, r, g, b);
        led_strip_refresh(led_strip);
        if (delay_ms > 0) {
            vTaskDelay(pdMS_TO_TICKS(delay_ms));
            led_strip_set_pixel(led_strip, index, 0, 0, 0);
            led_strip_refresh(led_strip);
        }
        xSemaphoreGive(led_mutex);
    }
}

// 初始化ADC
static esp_err_t init_adc(adc_oneshot_unit_handle_t handle)
{
    adc_handle = handle;

    // 配置ADC通道
    adc_oneshot_chan_cfg_t config = {
        .bitwidth = ADC_BITWIDTH,
        .atten    = ADC_ATTEN,
    };

    // 配置电池电压检测通道
    esp_err_t ret = adc_oneshot_config_channel(adc_handle, ADC_BAT_CHANNEL, &config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to config battery ADC channel: %s", esp_err_to_name(ret));
        return ret;
    }

    // 配置充电状态检测通道
    ret = adc_oneshot_config_channel(adc_handle, ADC_CHRG_CHANNEL, &config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to config charge ADC channel: %s", esp_err_to_name(ret));
        return ret;
    }

    // 配置USB电压检测通道
    ret = adc_oneshot_config_channel(adc_handle, ADC_VBUS_CHANNEL, &config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to config VBUS ADC channel: %s", esp_err_to_name(ret));
        return ret;
    }

#if CONFIG_PM_ENABLE
    /*!< 幂等：只建一次。允许在 esp_pm_configure() 之前创建。 */
    if (s_adc_pm_lock == NULL) {
        const esp_err_t lk = esp_pm_lock_create(ESP_PM_NO_LIGHT_SLEEP, 0, "adcsample", &s_adc_pm_lock);
        if (lk != ESP_OK) {
            ESP_LOGW(TAG, "ADC PM lock create failed: %s (读数可能被休眠污染)", esp_err_to_name(lk));
        }
    }
#endif

    ESP_LOGI(TAG, "ADC initialized successfully");
    return ESP_OK;
}

// 多次采样并平均
static int adc_read_average(adc_channel_t channel, int sample_count)
{
    int sum           = 0;
    int valid_samples = 0;

    for (int i = 0; i < sample_count; i++) {
        int adc_raw   = 0;
        esp_err_t ret = adc_oneshot_read(adc_handle, channel, &adc_raw);
        if (ret == ESP_OK) {
            sum += adc_raw;
            valid_samples++;
        }
        vTaskDelay(pdMS_TO_TICKS(1));  // 1ms延时
    }

    if (valid_samples > 0) {
        return sum / valid_samples;
    }
    return 0;
}

// 将ADC原始值转换为电压 (mV)
static float adc_raw_to_voltage(int adc_raw)
{
    return (float)adc_raw * VREF_VOLTAGE / ADC_MAX_VALUE;
}

// 获取充电状态字符串
static const char *get_charge_status_string(float chrg_voltage_mv)
{
    if (chrg_voltage_mv > CHRG_NOT_CHARGING) {
        return "Not Charging";
    } else if (chrg_voltage_mv > CHRG_CHARGED) {
        return "Fully Charged";
    } else if (chrg_voltage_mv > CHRG_CHARGING) {
        return "Charging";
    } else {
        return "Unknown";
    }
}

/*!< 轻量"插线探测"：只读 VBUS 一路、样本数 16。
 *
 *   用途：未插线态下 `power_indicator_task` 必须先"发现插线"才能点亮电量指示，
 *   而这条探测是蓝牙档**未插线时唯一还在跑的 ADC 采样** ⇒ 它的开销直接落在静置
 *   续航上。完整采样是 3 路 × 64 样本 ≈ 192ms（tick=1000Hz，每样本 1ms）；这里
 *   1 路 × 16 样本 ≈ 16ms，约 1/12。
 *   VBUS 只需要跨过 4.0V 这个门限、结果就是布尔量，噪声容忍度高，16 点足够。
 *
 *   同样持 NO_LIGHT_SLEEP 锁 —— 读数不可信的话，探测本身就没有意义。 */
bool adc_vbus_present(void)
{
    if (adc_handle == NULL) {
        return false;
    }

    adc_pm_lock_acquire();
    const int raw       = adc_read_average(ADC_VBUS_CHANNEL, 16);
    const float voltage = adc_raw_to_voltage(raw) * VOLTAGE_DIVIDER_RATIO_VBUS / 1000.0f;
    adc_pm_lock_release();

    return voltage > 4.0f;
}

// 执行ADC检测
static adc_result_t perform_adc_detection(void)
{
    adc_result_t result = {0};

    /*!< 采样全程禁 light sleep —— 见 s_adc_pm_lock 的长注释。 */
    adc_pm_lock_acquire();

    // 读取电池电压
    int bat_raw            = adc_read_average(ADC_BAT_CHANNEL, ADC_SAMPLE_COUNT);
    float bat_voltage_mv   = adc_raw_to_voltage(bat_raw);
    result.battery_voltage = bat_voltage_mv * VOLTAGE_DIVIDER_RATIO_BAT / 1000.0f;  // 转换为V
    result.battery_low     = (result.battery_voltage < 3.2f);                       // 低于3.2V认为低电量

    // 读取USB电压
    int vbus_raw          = adc_read_average(ADC_VBUS_CHANNEL, ADC_SAMPLE_COUNT);
    float vbus_voltage_mv = adc_raw_to_voltage(vbus_raw);
    result.vbus_voltage   = vbus_voltage_mv * VOLTAGE_DIVIDER_RATIO_VBUS / 1000.0f;  // 转换为V
    result.usb_connected  = (result.vbus_voltage > 4.0f);                            // 高于4.0V认为USB连接

    // 读取充电状态
    int chrg_raw         = adc_read_average(ADC_CHRG_CHANNEL, ADC_SAMPLE_COUNT);
    result.chrg_voltage  = adc_raw_to_voltage(chrg_raw) / 1000.0f;  // 转换为V
    result.charge_status = get_charge_status_string(result.chrg_voltage * 1000);

    adc_pm_lock_release();

    return result;
}

/*!< ── 电量等级色（全工程唯一实现，2026-09-18 从 main.cpp 迁入）─────────────
 *
 *   之所以要收敛成一份：这套"低亮度常亮表示电量等级"的语义原本只在 main.cpp 的
 *   `power_indicator_task` 里，而 `update_status_leds()`（开机自检 / 长按 4~7s）
 *   用的是厂方遗留的另一套 —— LED0 显示电池**电压**色、LED1 显示**充电状态**色。
 *   两套并存的结果是开机瞬间两颗灯语义不同、颜色不一样，用户会读成"灯效错乱"
 *   （实机已反馈过）。现在两边共用这一个函数，三档 + 全路径语义统一。
 *
 *   亮度刻意压到 ~10%（POWER_LED_BRIGHTNESS=26）：边充边用时不刺眼。
 *   绿色通道人眼更敏感，故取值更低以保持观感一致。 */
#define POWER_LED_BRIGHTNESS 26

uint32_t power_indicator_color(int percentage)
{
    if (percentage < 20) {
        return (uint32_t)POWER_LED_BRIGHTNESS << 16;  // 红
    } else if (percentage < 60) {
        return ((uint32_t)POWER_LED_BRIGHTNESS << 16) | ((uint32_t)(POWER_LED_BRIGHTNESS / 2) << 8);  // 琥珀
    }
    return (uint32_t)(POWER_LED_BRIGHTNESS * 3 / 4) << 8;  // 绿
}

// LED状态指示
static void update_status_leds(led_strip_handle_t led_strip, const adc_result_t *result)
{
    if (led_mutex == NULL || led_strip == NULL) {
        return;
    }

    /*!< 与 power_indicator_task 完全相同的语义：两颗灯**同色**常亮表示电量等级。
     *
     *   充满 → 两颗都熄灭。判定必须带 `usb_connected` 校验：TP4057 在"未充电"与
     *   "充满"两种状态下 CHRG 引脚都可能落在电压表的同一侧（REFERENCE §P.3），
     *   不校验会把"没插线"读成"充满了"。
     *
     *   ⚠️ g_battery_percentage 由调用方 update_power_status() 在本函数之前更新。 */
    const bool full      = result->usb_connected && (strcmp(result->charge_status, "Fully Charged") == 0);
    const uint32_t color = full ? 0u : power_indicator_color(g_battery_percentage);

    const uint8_t R = (color >> 16) & 0xFF;
    const uint8_t G = (color >> 8) & 0xFF;
    const uint8_t B = color & 0xFF;

    if (xSemaphoreTake(led_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        led_strip_set_pixel(led_strip, 0, R, G, B);
        led_strip_set_pixel(led_strip, 1, R, G, B);
        led_strip_refresh(led_strip);
        xSemaphoreGive(led_mutex);
    }
}

// ADC检测任务
static void adc_detection_task(void *arg)
{
    led_strip_handle_t led_strip = (led_strip_handle_t)arg;
    TickType_t last_wake_time    = xTaskGetTickCount();

    while (1) {
        // 执行ADC检测
        adc_result_t result = perform_adc_detection();

        // 打印检测结果
        ESP_LOGI(TAG, "=== ADC Detection Results ===");
        ESP_LOGI(TAG, "Battery Voltage: %.2fV %s", result.battery_voltage, result.battery_low ? "(LOW!)" : "(OK)");
        ESP_LOGI(TAG, "USB Voltage: %.2fV %s", result.vbus_voltage,
                 result.usb_connected ? "(Connected)" : "(Disconnected)");
        ESP_LOGI(TAG, "Charge Status: %s (%.2fV)", result.charge_status, result.chrg_voltage);
        ESP_LOGI(TAG, "=============================");

        // 更新LED状态
        update_status_leds(led_strip, &result);

        // 每2秒检测一次
        vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(2000));
    }
}

// ADC检测测试主函数
void test_adc_detection(led_strip_handle_t led_strip, adc_oneshot_unit_handle_t adc_handle)
{
    ESP_LOGI(TAG, "=== ADC Detection Test ===");

    // 创建LED互斥锁（如果还没有创建）
    if (led_mutex == NULL) {
        led_mutex = xSemaphoreCreateMutex();
        if (led_mutex == NULL) {
            ESP_LOGE(TAG, "Failed to create LED mutex");
            return;
        }
    }

    // 显示测试开始状态 - 白色闪烁
    for (int i = 0; i < 3; i++) {
        safe_led_set_and_refresh(led_strip, 0, 255, 255, 255, 0);
        safe_led_set_and_refresh(led_strip, 1, 255, 255, 255, 0);
        vTaskDelay(pdMS_TO_TICKS(200));
        safe_led_set_and_refresh(led_strip, 0, 0, 0, 0, 0);
        safe_led_set_and_refresh(led_strip, 1, 0, 0, 0, 0);
        vTaskDelay(pdMS_TO_TICKS(200));
    }

    // 初始化ADC
    esp_err_t ret = init_adc(adc_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "ADC initialization failed");
        // 错误指示 - 红色闪烁
        for (int i = 0; i < 5; i++) {
            safe_led_set_and_refresh(led_strip, 0, 255, 0, 0, 100);
            safe_led_set_and_refresh(led_strip, 1, 255, 0, 0, 100);
        }
        return;
    }

    ESP_LOGI(TAG, "ADC Detection Configuration:");
    ESP_LOGI(TAG, "- GPIO%d: Battery voltage (VBAT)", ADC_BAT_GPIO);
    ESP_LOGI(TAG, "- GPIO%d: Charge status (TP4057)", ADC_CHRG_GPIO);
    ESP_LOGI(TAG, "- GPIO%d: USB voltage (VBUS)", ADC_VBUS_GPIO);
    ESP_LOGI(TAG, "- Voltage divider ratio: %.2f", VOLTAGE_DIVIDER_RATIO_BAT);
    ESP_LOGI(TAG, "- Sample count: %d", ADC_SAMPLE_COUNT);

    ESP_LOGI(TAG, "Charge Status Thresholds:");
    ESP_LOGI(TAG, "- Not Charging: >%.1fV", CHRG_NOT_CHARGING / 1000.0f);
    ESP_LOGI(TAG, "- Fully Charged: %.1fV-%.1fV", CHRG_CHARGED / 1000.0f, CHRG_NOT_CHARGING / 1000.0f);
    ESP_LOGI(TAG, "- Charging: %.1fV-%.1fV", CHRG_CHARGING / 1000.0f, CHRG_CHARGED / 1000.0f);

    ESP_LOGI(TAG, "LED Status Indicators (both keys share one colour):");
    ESP_LOGI(TAG, "    Red: battery <20%%");
    ESP_LOGI(TAG, "    Amber: battery 20%%-59%%");
    ESP_LOGI(TAG, "    Green: battery >=60%%");
    ESP_LOGI(TAG, "    Off: fully charged while USB connected");

    // 创建ADC检测任务
    // xTaskCreate(adc_detection_task, "adc_detection_task", 4096, led_strip, 5, NULL);
    update_power_status(led_strip);
}

static int calculate_smooth_percentage(float voltage, bool is_charging)
{
    // 定义查找表结构
    typedef struct {
        float v;
        int p;
    } point_t;

    // 充电曲线表 (电压从小到大)
    const point_t chrg_table[] = {{3.40f, 0}, {3.61f, 25}, {3.88f, 50}, {4.12f, 75}, {4.20f, 100}};
    // 放电曲线表 (电压从小到大)
    const point_t dischrg_table[] = {{3.33f, 0}, {3.55f, 25}, {3.81f, 50}, {4.07f, 75}, {4.20f, 100}};

    const point_t *table = is_charging ? chrg_table : dischrg_table;
    int count            = 5;

    if (voltage <= table[0].v) return 0;
    if (voltage >= table[count - 1].v) return 100;

    // 线性插值
    for (int i = 0; i < count - 1; i++) {
        if (voltage >= table[i].v && voltage < table[i + 1].v) {
            float range_v = table[i + 1].v - table[i].v;
            float range_p = table[i + 1].p - table[i].p;
            float offset  = voltage - table[i].v;
            return (int)(table[i].p + (offset / range_v) * range_p);
        }
    }
    return 100;
}

void update_power_status(led_strip_handle_t led_strip)
{
    adc_result_t result = perform_adc_detection();

    // 更新全局变量
    g_battery_voltage = result.battery_voltage;

    g_usb_voltage   = result.vbus_voltage;
    g_usb_connected = result.usb_connected;

    /*!< 判定充电状态: 0=未充电, 1=充电中, 2=充满。
     *   ⚠️ "充满"必须同时满足 USB 在位 —— TP4057 在"未插线"时 CHRG 是高阻态，
     *   电压会落在与"充满"相同的区间里（REFERENCE §P.3），不校验就会被读成
     *   "充满了"并进一步把电量硬拉成 100%。 */
    const bool chrg_charging = (strcmp(result.charge_status, "Charging") == 0);
    const bool chrg_full     = (strcmp(result.charge_status, "Fully Charged") == 0) && result.usb_connected;
    g_charging_status        = chrg_charging ? 1 : (chrg_full ? 2 : 0);

    // --- 优化后的电量计算逻辑开始 ---
    if (g_charging_status == 2) {
        // 硬件指示已充满，强制100%
        g_battery_percentage = 100;
    } else {
        // 根据充电/放电状态选择不同的曲线
        bool is_charging_curve = (g_charging_status == 1);
        g_battery_percentage   = calculate_smooth_percentage(g_battery_voltage, is_charging_curve);
    }
    // --- 优化后的电量计算逻辑结束 ---

    // 边界保护
    if (g_battery_percentage > 100) g_battery_percentage = 100;
    if (g_battery_percentage < 0) g_battery_percentage = 0;

    // 更新LED状态
    if (led_strip != NULL) {
        // 打印检测结果，增加百分比显示
        ESP_LOGI(TAG, "=== ADC Detection Results ===");
        ESP_LOGI(TAG, "Battery: %.2fV [%d%%] %s", result.battery_voltage, g_battery_percentage,
                 result.battery_low ? "(LOW!)" : "(OK)");
        ESP_LOGI(TAG, "USB Voltage: %.2fV %s", result.vbus_voltage,
                 result.usb_connected ? "(Connected)" : "(Disconnected)");
        ESP_LOGI(TAG, "Charge Status: %s (%.2fV)", result.charge_status, result.chrg_voltage);
        ESP_LOGI(TAG, "=============================");
        update_status_leds(led_strip, &result);
    }
}

void low_power_test(void)
{
    // gpio_reset_pin((gpio_num_t)21);
    // gpio_set_direction((gpio_num_t)21, GPIO_MODE_INPUT);
    // // gpio_set_level((gpio_num_t)21, 1); //拉高
    // gpio_set_pull_mode((gpio_num_t)21, GPIO_PULLDOWN_ONLY);

    gpio_deep_sleep_hold_en();
    esp_deep_sleep_start();
}