/*
 * SPDX-FileCopyrightText: 2021-2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>
#include "esp_log.h"
#include "esp_check.h"
#include "keyboard_button.h"
#include "bsp/keyboard.h"
#include "bsp/lightmap.h"
#include "led_strip.h"
#include "rgb_matrix_drivers.h"
#include "rgb_matrix.h"
#include "esp_adc/adc_oneshot.h"

static const char *TAG = "kbd_kit";

esp_err_t bsp_keyboard_init(keyboard_btn_handle_t *kbd_handle, keyboard_btn_config_t *ex_cfg)
{
    ESP_RETURN_ON_FALSE(kbd_handle != NULL, ESP_ERR_INVALID_ARG, TAG, "kbd_handle is NULL");

    if (ex_cfg != NULL) {
        keyboard_button_create(ex_cfg, kbd_handle);
    } else {
        keyboard_btn_config_t cfg = {
            .output_gpios = NULL,
            .output_gpio_num = 0,
            .input_gpios = (int[])KBD_INPUT_IOS,
            .input_gpio_num = KBD_COL_NUM,
            .active_level = KBD_ATTIVE_LEVEL,
            .debounce_ticks = 3,
            .ticks_interval = KBD_TICKS_INTERVAL_US,
            .enable_power_save = true,
        };
        keyboard_button_create(&cfg, kbd_handle);
    }

    return ESP_OK;
}

static led_strip_handle_t s_led_strip = NULL;
static bool s_led_enable = false;

esp_err_t bsp_ws2812_init(led_strip_handle_t *led_strip)
{
    if (s_led_strip) {
        if (led_strip) {
            *led_strip = s_led_strip;
        }
        return ESP_OK;
    }

    gpio_config_t io_conf = {
        .pin_bit_mask = 1ULL << KBD_WS2812_POWER_IO,
                             .mode = GPIO_MODE_OUTPUT_OD,
                             .pull_down_en = 0,
                             .pull_up_en = 0,
    };
    gpio_config(&io_conf);

    /* LED strip initialization with the GPIO and pixels number*/
    led_strip_config_t strip_config = {
        .strip_gpio_num = LIGHTMAP_GPIO, // The GPIO that connected to the LED strip's data line
        .max_leds = LIGHTMAP_NUM, // The number of LEDs in the strip,
        .led_pixel_format = LED_PIXEL_FORMAT_GRB, // Pixel format of your LED strip
        .led_model = LED_MODEL_WS2812, // LED strip model
        .flags.invert_out = false, // whether to invert the output signal (useful when your hardware has a level inverter)
    };

    // LED strip backend configuration: SPI
    led_strip_spi_config_t spi_config = {
        .clk_src = SPI_CLK_SRC_XTAL, // different clock source can lead to different power consumption
        .flags.with_dma = true,         // Using DMA can improve performance and help drive more LEDs
        .spi_bus = SPI2_HOST,           // SPI bus ID
    };

    // LED Strip object handle
    ESP_ERROR_CHECK(led_strip_new_spi_device(&strip_config, &spi_config, &s_led_strip));

    if (led_strip) {
        *led_strip = s_led_strip;
    }
    return ESP_OK;
}

esp_err_t bsp_ws2812_enable(bool enable)
{
    /*!< ⚠️ 顺序要紧：hold 生效期间 gpio_set_level() 是**静默无效**的
     *   （ESP-IDF: 调用 gpio_hold_en 后 "changing the output level ... will not
     *   take effect"，必须先 gpio_hold_dis 才能再改这个 pad）。
     *   原实现只在 !enable 分支里解 hold，于是本函数**不可重复调用** ——
     *   第二次 enable(true) 时 hold 还在，gpio_set_level 会被丢掉，
     *   引脚维持旧值。修好后它才是幂等的，才能在 light sleep 唤醒后
     *   当作"重新声明供电"来用（见 bsp_ws2812_resync）。 */
    gpio_hold_dis(KBD_WS2812_POWER_IO);
    gpio_set_level(KBD_WS2812_POWER_IO, !enable);
    /*!< Make output stable in light sleep */
    if (enable) {
        gpio_hold_en(KBD_WS2812_POWER_IO);
    }
    s_led_enable = enable;
    return ESP_OK;
}

int bsp_ws2812_resync(void)
{
    /*!< ── 唤醒后重新声明 LED 供电通路（2026-09-16 加）──────────────────────
     *
     *   官方依据（ESP-IDF v5.5 esp_pm/Kconfig 中 PM_SLP_DISABLE_GPIO 的 help）：
     *     "Disable all GPIO when chip at sleep ... chips will disable all GPIO pins
     *      at automantic sleep to reduce about 200~300 uA current.
     *      If you want to specifically use some pins normally ... you can call
     *      'gpio_sleep_sel_dis' to disable this feature on those pins."
     *   实现见 esp_hw_support/sleep_gpio.c:57 esp_sleep_config_gpio_isolate() ——
     *   它对**每一个**有效 GPIO 调 gpio_sleep_set_direction(GPIO_MODE_DISABLE) +
     *   gpio_sleep_set_pull_mode(GPIO_FLOATING)，并由 esp_sleep_enable_gpio_switch()
     *   给每个 pad 置 SleepSelEn=1。也就是说：**每次 light sleep，所有 GPIO 都会
     *   被切到"睡眠配置"（禁用 + 浮空）**，本工程 sdkconfig 里该项为 y。
     *
     *   GPIO40 是 WS2812 的供电使能（产品文档管脚映射：G40 = WS2812_PWR / PWR_EN），
     *   睡眠期间被隔离 ⇒ LED 断电；唤醒后数据虽然发得出去（led_strip_refresh()
     *   只管 SPI，**不知道灯有没有电**，照样返回 ESP_OK），但供电时序没恢复时
     *   灯就是完全不亮 —— 症状与"灯效根本没触发"完全一致，这正是本轮最难分辨处。
     *
     *   本函数把"上电 + hold"重放一遍，返回读回的引脚电平供取证。 */
    bsp_ws2812_enable(true);
    return gpio_get_level(KBD_WS2812_POWER_IO);
}

esp_err_t bsp_ws2812_clear(void)
{
    return led_strip_clear(s_led_strip);
}

bool bsp_ws2812_is_enable(void)
{
    return s_led_enable;
}

esp_err_t bsp_rgb_matrix_init(void)
{
    if (!s_led_strip) {
        bsp_ws2812_init(NULL);
    }
    rgb_matrix_driver_init(s_led_strip, LIGHTMAP_NUM);
    rgb_matrix_init();
    return ESP_OK;
}

static adc_oneshot_unit_handle_t adc1_handle;

esp_err_t bsp_get_adc_handle(adc_oneshot_unit_handle_t *handle)
{
    if (handle) {
        *handle = adc1_handle;
        return ESP_OK;
    }
    return ESP_FAIL;
}

esp_err_t bsp_adc_switch_init(void)
{
    if (!adc1_handle) {
        adc_oneshot_unit_init_cfg_t init_config = {
            .unit_id = ADC_UNIT_1,
        };
        ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config, &adc1_handle));
    }

    adc_oneshot_chan_cfg_t config = {
        .bitwidth = ADC_BITWIDTH_DEFAULT,
        .atten = ADC_ATTEN_DB_12,
    };

    adc_oneshot_config_channel(adc1_handle, KBD_ADC_SWITCH_BLE_CHAN, &config);
    adc_oneshot_config_channel(adc1_handle, KBD_ADC_SWITCH_RAINMAKER_CHAN, &config);
    return ESP_OK;
}

esp_err_t bsp_adc_charge_monitor_init(void)
{
    if (!adc1_handle) {
        adc_oneshot_unit_init_cfg_t init_config = {
            .unit_id = ADC_UNIT_1,
        };
        ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config, &adc1_handle));
    }

    adc_oneshot_chan_cfg_t config = {
        .bitwidth = ADC_BITWIDTH_DEFAULT,
        .atten = ADC_ATTEN_DB_12,
    };

    adc_oneshot_config_channel(adc1_handle, KBD_ADC_BATTERY_MONITOR_CHAN, &config);
    adc_oneshot_config_channel(adc1_handle, KBD_ADC_CHARGE_MONITOR_CHAN, &config);
    return ESP_OK;
}
