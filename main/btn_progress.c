/*
 *SPDX-FileCopyrightText: 2025 M5Stack Technology CO LTD
 *
 *SPDX-License-Identifier: MIT
 */

#include "btn_progress.h"
#include "tinyusb_hid.h"
#include "ble_hid.h"
#include "tinyusb_cdc.h"
#include "bsp/keycodes.h"
#include "bsp/keymap.h"
#include "usb_descriptors.h"
#include "rgb_matrix.h"
#include "settings.h"
#include "esp_system.h"
#include "esp_pm.h"
#include "esp_timer.h"
#include "bsp/esp-bsp.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include <stdio.h>
#include <string.h>

// 声明外部变量来获取设备状态
extern bool g_usb_mapping_enabled;
extern bool g_ble_mapping_enabled;

static btn_report_type_t report_type = USB_CDC_REPORT;

// 当前按键映射索引，默认为10（翻页模式）
static int current_key_mapping_index = 10;

// ============ 自定义映射状态 ============
static bool custom_mapping_enabled = false;

static custom_key_action_t custom_left_action = {
    .type     = CUSTOM_ACTION_KEY,
    .modifier = 0,
    .keycode  = 0x4B,  // HID_KEY_PAGE_UP
    .text     = "",
};

static custom_key_action_t custom_right_action = {
    .type     = CUSTOM_ACTION_KEY,
    .modifier = 0,
    .keycode  = 0x4E,  // HID_KEY_PAGE_DOWN
    .text     = "",
};

// 跟踪上次按键状态，用于检测上升沿（避免文本多次触发）
static bool custom_left_prev_pressed  = false;
static bool custom_right_prev_pressed = false;

// 当前是否正被按住。btn_progress() 在按键回调里更新，btn_progress_tick() 在周期任务里读取
static volatile bool custom_left_down  = false;
static volatile bool custom_right_down = false;

// ---- 长按动作（方案B：松手时按按压时长判定短按/长按）----
// type == CUSTOM_ACTION_NONE 表示该键未配置长按。此时该键完全不做长按判定，
// 走"按下即上报"的原路径，延迟为零。
static custom_key_action_t custom_left_long_action = {
    .type     = CUSTOM_ACTION_NONE,
    .modifier = 0,
    .keycode  = 0,
    .text     = "",
};

static custom_key_action_t custom_right_long_action = {
    .type     = CUSTOM_ACTION_NONE,
    .modifier = 0,
    .keycode  = 0,
    .text     = "",
};

// 长按判定阈值 (ms)，可在网页调整
static uint16_t custom_long_press_ms = CUSTOM_LONG_PRESS_DEFAULT_MS;

// 长按判定用的按下时刻 (us)，esp_timer_get_time() 单调递增
static int64_t custom_left_press_us  = 0;
static int64_t custom_right_press_us = 0;

// 本次按住期间长按是否已经触发过（方案A：到点即触发，之后松手不再重复）
static volatile bool custom_left_long_fired  = false;
static volatile bool custom_right_long_fired = false;

// ---- 修饰键位掩码 ----
#define MODIFIER_LEFT_CTRL  0x01
#define MODIFIER_LEFT_SHIFT 0x02
#define MODIFIER_LEFT_ALT   0x04
#define MODIFIER_LEFT_GUI   0x08

// ---- US QWERTY: ASCII → (modifier, HID keycode) ----
static bool char_to_hid(char c, uint8_t *modifier, uint8_t *keycode)
{
    if (c >= 'a' && c <= 'z') {
        *modifier = 0;
        *keycode  = 0x04 + (c - 'a');  // HID_KEY_A = 0x04
        return true;
    }
    if (c >= 'A' && c <= 'Z') {
        *modifier = MODIFIER_LEFT_SHIFT;
        *keycode  = 0x04 + (c - 'A');
        return true;
    }
    if (c >= '1' && c <= '9') {
        *modifier = 0;
        *keycode  = 0x1E + (c - '1');  // HID_KEY_1 = 0x1E
        return true;
    }
    switch (c) {
        case '0':
            *modifier = 0;
            *keycode  = 0x27;
            return true;  // HID_KEY_0
        case ' ':
            *modifier = 0;
            *keycode  = 0x2C;
            return true;  // HID_KEY_SPACE
        case '\n':
            *modifier = 0;
            *keycode  = 0x28;
            return true;  // HID_KEY_ENTER
        case '\t':
            *modifier = 0;
            *keycode  = 0x2B;
            return true;  // HID_KEY_TAB
        case '-':
            *modifier = 0;
            *keycode  = 0x2D;
            return true;  // HID_KEY_MINUS
        case '_':
            *modifier = MODIFIER_LEFT_SHIFT;
            *keycode  = 0x2D;
            return true;
        case '=':
            *modifier = 0;
            *keycode  = 0x2E;
            return true;  // HID_KEY_EQUAL
        case '+':
            *modifier = MODIFIER_LEFT_SHIFT;
            *keycode  = 0x2E;
            return true;
        case '[':
            *modifier = 0;
            *keycode  = 0x2F;
            return true;
        case '{':
            *modifier = MODIFIER_LEFT_SHIFT;
            *keycode  = 0x2F;
            return true;
        case ']':
            *modifier = 0;
            *keycode  = 0x30;
            return true;
        case '}':
            *modifier = MODIFIER_LEFT_SHIFT;
            *keycode  = 0x30;
            return true;
        case '\\':
            *modifier = 0;
            *keycode  = 0x31;
            return true;
        case '|':
            *modifier = MODIFIER_LEFT_SHIFT;
            *keycode  = 0x31;
            return true;
        case ';':
            *modifier = 0;
            *keycode  = 0x33;
            return true;
        case ':':
            *modifier = MODIFIER_LEFT_SHIFT;
            *keycode  = 0x33;
            return true;
        case '\'':
            *modifier = 0;
            *keycode  = 0x34;
            return true;
        case '"':
            *modifier = MODIFIER_LEFT_SHIFT;
            *keycode  = 0x34;
            return true;
        case '`':
            *modifier = 0;
            *keycode  = 0x35;
            return true;
        case '~':
            *modifier = MODIFIER_LEFT_SHIFT;
            *keycode  = 0x35;
            return true;
        case ',':
            *modifier = 0;
            *keycode  = 0x36;
            return true;
        case '<':
            *modifier = MODIFIER_LEFT_SHIFT;
            *keycode  = 0x36;
            return true;
        case '.':
            *modifier = 0;
            *keycode  = 0x37;
            return true;
        case '>':
            *modifier = MODIFIER_LEFT_SHIFT;
            *keycode  = 0x37;
            return true;
        case '/':
            *modifier = 0;
            *keycode  = 0x38;
            return true;
        case '?':
            *modifier = MODIFIER_LEFT_SHIFT;
            *keycode  = 0x38;
            return true;
        case '!':
            *modifier = MODIFIER_LEFT_SHIFT;
            *keycode  = 0x1E;
            return true;
        case '@':
            *modifier = MODIFIER_LEFT_SHIFT;
            *keycode  = 0x1F;
            return true;
        case '#':
            *modifier = MODIFIER_LEFT_SHIFT;
            *keycode  = 0x20;
            return true;
        case '$':
            *modifier = MODIFIER_LEFT_SHIFT;
            *keycode  = 0x21;
            return true;
        case '%':
            *modifier = MODIFIER_LEFT_SHIFT;
            *keycode  = 0x22;
            return true;
        case '^':
            *modifier = MODIFIER_LEFT_SHIFT;
            *keycode  = 0x23;
            return true;
        case '&':
            *modifier = MODIFIER_LEFT_SHIFT;
            *keycode  = 0x24;
            return true;
        case '*':
            *modifier = MODIFIER_LEFT_SHIFT;
            *keycode  = 0x25;
            return true;
        case '(':
            *modifier = MODIFIER_LEFT_SHIFT;
            *keycode  = 0x26;
            return true;
        case ')':
            *modifier = MODIFIER_LEFT_SHIFT;
            *keycode  = 0x27;
            return true;
        default:
            break;
    }
    return false;
}

// 统一的上报出口（按 report_type 分发并串行化发送），定义见下方
static void _report(hid_report_t report);

// 逐字符输出文本字符串（阻塞，每个字符约 10ms press + 10ms release）
static void type_text_string(const char *text)
{
    for (int i = 0; text[i] != '\0' && i < CUSTOM_TEXT_MAX_LEN; i++) {
        uint8_t mod = 0, kc = 0;
        if (!char_to_hid(text[i], &mod, &kc)) continue;

        hid_report_t press               = {0};
        press.report_id                  = REPORT_ID_KEYBOARD;
        press.keyboard_report.modifier   = mod;
        press.keyboard_report.keycode[0] = kc;

        hid_report_t release = {0};
        release.report_id    = REPORT_ID_KEYBOARD;

        // 统一走 _report()：由它按 report_type 分发，并保证发送串行化
        _report(press);
        vTaskDelay(pdMS_TO_TICKS(10));
        _report(release);
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

#define HSV_MAX 18
static uint8_t hsv_index             = 0;
static uint8_t hsv_color[HSV_MAX][3] = {
    {132, 102, 255},  // HSV_AZURE
    {170, 255, 255},  // HSV_BLUE
    {64, 255, 255},   // HSV_CHARTREUSE
    {11, 176, 255},   // HSV_CORAL
    {128, 255, 255},  // HSV_CYAN
    {36, 255, 255},   // HSV_GOLD
    {30, 218, 218},   // HSV_GOLDENROD
    {85, 255, 255},   // HSV_GREEN
    {213, 255, 255},  // HSV_MAGENTA
    {21, 255, 255},   // HSV_ORANGE
    {234, 128, 255},  // HSV_PINK
    {191, 255, 255},  // HSV_PURPLE
    {0, 255, 255},    // HSV_RED
    {106, 255, 255},  // HSV_SPRINGGREEN
    {128, 255, 128},  // HSV_TEAL
    {123, 90, 112},   // HSV_TURQUOISE
    {0, 0, 255},      // HSV_WHITE
    {43, 255, 255},   // HSV_YELLOW
};

// HID 键盘报告是"全量状态"，且底层发送接口（尤其 BLE 的 esp_hidd_dev_input_set）并非线程安全。
// 自定义映射下有两个上下文会发报告：按键回调所在的 kbd_task，与负责长按到点触发的周期任务。
// 所有发送统一经由这把锁串行化；锁只包住发送本身，绝不跨越 vTaskDelay。
static SemaphoreHandle_t custom_emit_mutex = NULL;

static inline void custom_emit_lock(void)
{
    if (custom_emit_mutex) {
        xSemaphoreTake(custom_emit_mutex, portMAX_DELAY);
    }
}

static inline void custom_emit_unlock(void)
{
    if (custom_emit_mutex) {
        xSemaphoreGive(custom_emit_mutex);
    }
}

void btn_progress_init(void)
{
    if (custom_emit_mutex == NULL) {
        custom_emit_mutex = xSemaphoreCreateMutex();
        if (custom_emit_mutex == NULL) {
            ESP_LOGE("btn_progress", "创建 HID 发送互斥锁失败");
        }
    }
}

static void _report(hid_report_t report)
{
    custom_emit_lock();
    switch (report_type) {
        case TINYUSB_HID_REPORT:
            tinyusb_hid_keyboard_report(report);
            break;
        case BLE_HID_REPORT:
            ble_hid_keyboard_report(report);
            break;
        case USB_CDC_REPORT:
            // CDC 模式不发送 HID 报告，在 btn_progress 中直接处理
            break;
        case ALL_REPORT:
            if (g_usb_mapping_enabled) {
                tinyusb_hid_keyboard_report(report);
            }
            if (g_ble_mapping_enabled) {
                ble_hid_keyboard_report(report);
            }
            break;
        default:
            break;
    }
    custom_emit_unlock();
}

// ============ 自定义映射的按键状态与长按辅助 ============
// HID 键盘报告是"全量状态"（不是增量），所以必须整体维护当前按下的按键集合，
// 否则在补发长按/短按点按时会把另一个正在按住的键一起"放开"。
typedef struct {
    uint8_t modifier;
    uint8_t keycode[6];
    int count;
} custom_kbd_state_t;

static void custom_state_clear(custom_kbd_state_t *s)
{
    s->modifier = 0;
    memset(s->keycode, 0, sizeof(s->keycode));
    s->count = 0;
}

static void custom_state_add(custom_kbd_state_t *s, uint8_t modifier, uint8_t keycode)
{
    s->modifier |= modifier;
    if (keycode != 0 && s->count < 6) {
        s->keycode[s->count++] = keycode;
    }
}

static void custom_state_send(const custom_kbd_state_t *s)
{
    hid_report_t report             = {0};
    report.report_id                = REPORT_ID_KEYBOARD;
    report.keyboard_report.modifier = s->modifier;
    for (int i = 0; i < s->count && i < 6; i++) {
        report.keyboard_report.keycode[i] = s->keycode[i];
    }
    _report(report);
}

static bool custom_action_has_long(const custom_key_action_t *action)
{
    return action->type != CUSTOM_ACTION_NONE;
}

// 方案B：选定动作后做一次"点按"（按下 + 松开），结束后把状态恢复成 base
// （base = 其它未配长按、且当前正被按住的键）
static void custom_emit_tap(const custom_key_action_t *action, const custom_kbd_state_t *base)
{
    if (action->type == CUSTOM_ACTION_TEXT) {
        // 输出文本前先把已按下的键放开，输出完再恢复
        custom_kbd_state_t empty;
        custom_state_clear(&empty);
        custom_state_send(&empty);
        type_text_string(action->text);
        custom_state_send(base);
        return;
    }

    custom_kbd_state_t press = *base;
    custom_state_add(&press, action->modifier, action->keycode);
    custom_state_send(&press);
    vTaskDelay(pdMS_TO_TICKS(10));
    custom_state_send(base);
}

void btn_progress(keyboard_btn_report_t kbd_report)
{
    static uint8_t layer         = 1;
    uint8_t mo_action_layer      = layer;
    uint8_t keycode[120]         = {0};
    uint8_t keynum               = 0;
    uint8_t modify               = 0;
    hid_report_t kbd_hid_report  = {0};
    hid_report_t consumer_report = {0};
    bool if_consumer_report      = false;
    bool release_consumer_report = false;
    sys_param_t *sys_param       = settings_get_parameter();

    if (sys_param->report_type == USB_CDC_REPORT) {
        // USB CDC 模式：发送原始键盘数据
        // tinyusb_cdc_send_keyboard_report(kbd_report);
        return;
    }

    // ---- 自定义映射模式 ----
    if (custom_mapping_enabled) {
        bool left_now  = false;
        bool right_now = false;
        for (int i = 0; i < kbd_report.key_pressed_num; i++) {
            if (kbd_report.key_data[i].input_index == 0) left_now = true;
            if (kbd_report.key_data[i].input_index == 1) right_now = true;
        }

        const bool left_edge_down  = left_now && !custom_left_prev_pressed;
        const bool right_edge_down = right_now && !custom_right_prev_pressed;
        const bool left_edge_up    = !left_now && custom_left_prev_pressed;
        const bool right_edge_up   = !right_now && custom_right_prev_pressed;

        // 是否配置了长按。未配置的键完全跳过长按判定，保持"按下即上报"的零延迟手感
        const bool left_has_long  = custom_action_has_long(&custom_left_long_action);
        const bool right_has_long = custom_action_has_long(&custom_right_long_action);

        const int64_t now_us = esp_timer_get_time();

        // 方案A（到点触发）：按下只记时间戳并清掉"已触发"标志，
        // 真正的长按触发交给 btn_progress_tick() 在按住期间检查
        if (left_has_long && left_edge_down) {
            custom_left_press_us   = now_us;
            custom_left_long_fired = false;
        }
        if (right_has_long && right_edge_down) {
            custom_right_press_us   = now_us;
            custom_right_long_fired = false;
        }

        // 未配长按的键：文本仍在按下瞬间触发（保持原行为）
        if (!left_has_long && left_edge_down && custom_left_action.type == CUSTOM_ACTION_TEXT) {
            type_text_string(custom_left_action.text);
        }
        if (!right_has_long && right_edge_down && custom_right_action.type == CUSTOM_ACTION_TEXT) {
            type_text_string(custom_right_action.text);
        }

        custom_left_prev_pressed  = left_now;
        custom_right_prev_pressed = right_now;
        custom_left_down          = left_now;
        custom_right_down         = right_now;

        // 未配长按的键的"按住"状态（保持按住期间持续发送）
        custom_kbd_state_t held;
        custom_state_clear(&held);
        if (!left_has_long && left_now && custom_left_action.type == CUSTOM_ACTION_KEY) {
            custom_state_add(&held, custom_left_action.modifier, custom_left_action.keycode);
        }
        if (!right_has_long && right_now && custom_right_action.type == CUSTOM_ACTION_KEY) {
            custom_state_add(&held, custom_right_action.modifier, custom_right_action.keycode);
        }

        // 配了长按的键：长按若已在按住期间触发过，松手不再动作；否则判为短按
        bool tapped = false;
        if (left_has_long && left_edge_up) {
            if (custom_left_long_fired) {
                custom_left_long_fired = false;  // 长按已触发，松手不重复
                ESP_LOGI("btn_progress", "左键长按已触发, 松手不重复");
            } else {
                ESP_LOGI("btn_progress", "左键短按: %lld ms", (long long)((now_us - custom_left_press_us) / 1000));
                custom_emit_tap(&custom_left_action, &held);
                tapped = true;
            }
        }
        if (right_has_long && right_edge_up) {
            if (custom_right_long_fired) {
                custom_right_long_fired = false;
                ESP_LOGI("btn_progress", "右键长按已触发, 松手不重复");
            } else {
                ESP_LOGI("btn_progress", "右键短按: %lld ms", (long long)((now_us - custom_right_press_us) / 1000));
                custom_emit_tap(&custom_right_action, &held);
                tapped = true;
            }
        }

        // 点按已经把最终状态（其它键的按住状态）发出去了，无需重复发送
        if (!tapped) {
            custom_state_send(&held);
        }
        return;
    }

    /*!< Report with key pressed */
    for (int i = 0; i < kbd_report.key_pressed_num; i++) {
        uint8_t row = kbd_report.key_data[i].output_index;
        uint8_t col = kbd_report.key_data[i].input_index;
        // 使用当前按键映射索引而不是固定的层
        uint16_t kc = keymaps[current_key_mapping_index][row][col];

        switch (kc) {
            case QK_MOMENTARY ... QK_MOMENTARY_MAX:
                // Momentary action_layer
                mo_action_layer = QK_MOMENTARY_GET_LAYER(kc);
                printf("Momentary action_layer: %d\n", mo_action_layer);
                break;

            case QK_LCTL ... QK_RSFT:  // 扩展范围到所有修饰键组合
                // 提取修饰键位掩码（高字节）
                modify |= (kc >> 8) & 0xFF;
                // 提取基础按键码（低字节）
                keycode[keynum++] = QK_MODS_GET_BASIC_KEYCODE(kc);
                break;

            case HID_KEY_CONTROL_LEFT ... HID_KEY_GUI_RIGHT:
                // Modifier key
                modify |= 1 << (kc - HID_KEY_CONTROL_LEFT);
                break;
                ;

            case KC_KB_MUTE:
                consumer_report.consumer_report.keycode = HID_USAGE_CONSUMER_MUTE;
                if_consumer_report                      = true;
                break;
            case KC_KB_VOLUME_UP:
                consumer_report.consumer_report.keycode = HID_USAGE_CONSUMER_VOLUME_INCREMENT;
                if_consumer_report                      = true;
                break;
            case KC_KB_VOLUME_DOWN:
                consumer_report.consumer_report.keycode = HID_USAGE_CONSUMER_VOLUME_DECREMENT;
                if_consumer_report                      = true;
                break;

            case KC_MEDIA_PREV_TRACK:
                consumer_report.consumer_report.keycode = HID_USAGE_CONSUMER_SCAN_PREVIOUS;
                if_consumer_report                      = true;
                break;

            case KC_MEDIA_NEXT_TRACK:
                consumer_report.consumer_report.keycode = HID_USAGE_CONSUMER_SCAN_NEXT;
                if_consumer_report                      = true;
                break;

            case KC_MEDIA_PLAY_PAUSE:
                consumer_report.consumer_report.keycode = HID_USAGE_CONSUMER_PLAY_PAUSE;
                if_consumer_report                      = true;
                break;

            case KC_MEDIA_STOP:
                consumer_report.consumer_report.keycode = HID_USAGE_CONSUMER_STOP;
                if_consumer_report                      = true;
                break;

            case QK_BACKLIGHT_UP:
                rgb_matrix_increase_val();
                break;

            case QK_BACKLIGHT_DOWN:
                rgb_matrix_decrease_val();
                break;

            case RGB_SPD:
                rgb_matrix_decrease_speed();
                break;

            case RGB_SPI:
                rgb_matrix_increase_speed();
                break;

            case QK_BACKLIGHT_TOGGLE:
                rgb_matrix_toggle();
                if (!rgb_matrix_is_enabled()) {
                    bsp_ws2812_clear();
                }
                bsp_ws2812_enable(rgb_matrix_is_enabled());
                break;

            case RGB_MODE_FORWARD: {
                uint16_t index = (rgb_matrix_get_mode() + 1) % RGB_MATRIX_EFFECT_MAX;
                rgb_matrix_mode(index);
                break;
            }

            case RGB_MODE_REVERSE: {
                uint16_t index = rgb_matrix_get_mode() - 1;
                if (index < 1) {
                    index = RGB_MATRIX_EFFECT_MAX - 1;
                }
                rgb_matrix_mode(index);
                break;
            }

            case RGB_TOG:
                rgb_matrix_sethsv(hsv_color[hsv_index][0], hsv_color[hsv_index][1], hsv_color[hsv_index][2]);
                hsv_index = (hsv_index + 1) % HSV_MAX;
                break;

            default:
                if (kc != HID_KEY_NONE) {
                    keycode[keynum++] = kc;
                }
                break;
        }
    }

    /*!< Find if consumer key release */
    for (int i = 0; i < kbd_report.key_release_num; i++) {
        uint8_t row = kbd_report.key_release_data[i].output_index;
        uint8_t col = kbd_report.key_release_data[i].input_index;
        // 使用当前按键映射索引
        uint16_t kc = keymaps[current_key_mapping_index][row][col];
        switch (kc) {
            case KC_KB_MUTE:
                release_consumer_report = true;
                break;

            case KC_KB_VOLUME_UP:
                release_consumer_report = true;
                break;

            case KC_KB_VOLUME_DOWN:
                release_consumer_report = true;
                break;

            case KC_MEDIA_PREV_TRACK:
                release_consumer_report = true;
                break;

            case KC_MEDIA_NEXT_TRACK:
                release_consumer_report = true;
                break;

            case KC_MEDIA_PLAY_PAUSE:
                release_consumer_report = true;
                break;

            case KC_MEDIA_STOP:
                release_consumer_report = true;
                break;
        }
    }

    if (keynum <= 6) {
        kbd_hid_report.report_id                = REPORT_ID_KEYBOARD;
        kbd_hid_report.keyboard_report.modifier = modify;
        for (int i = 0; i < keynum; i++) {
            kbd_hid_report.keyboard_report.keycode[i] = keycode[i];
        }
    } else {
        kbd_hid_report.report_id                         = REPORT_ID_FULL_KEY_KEYBOARD;
        kbd_hid_report.keyboard_full_key_report.modifier = modify;
        for (int i = 0; i < keynum; i++) {
            // USAGE ID for keyboard starts from 4
            uint8_t key       = keycode[i] - 3;
            uint8_t byteIndex = (key - 1) / 8;
            uint8_t bitIndex  = (key - 1) % 8;
            kbd_hid_report.keyboard_full_key_report.keycode[byteIndex] |= (1 << bitIndex);
        }
    }

    _report(kbd_hid_report);

    if (if_consumer_report && !release_consumer_report) {
        consumer_report.report_id = REPORT_ID_CONSUMER;
        _report(consumer_report);
    } else if (release_consumer_report) {
        consumer_report.report_id               = REPORT_ID_CONSUMER;
        consumer_report.consumer_report.keycode = 0;
        _report(consumer_report);
    }
}

/*!< 长按到点触发检查，由周期任务（约 10ms）调用
 *
 *   方案A：配了长按的键被按住后，一旦按住时长达到阈值就立刻执行长按动作，
 *   不需要等松手；触发过之后本次按住不再重复，松手时也不会再补发短按。
 *   未配长按的键不参与这里的任何判定，行为与改动前完全一致。
 */
void btn_progress_tick(void)
{
    if (!custom_mapping_enabled) {
        return;
    }
    if (!custom_left_down && !custom_right_down) {
        return;
    }

    const bool left_has_long  = custom_action_has_long(&custom_left_long_action);
    const bool right_has_long = custom_action_has_long(&custom_right_long_action);
    if (!left_has_long && !right_has_long) {
        return;
    }

    const int64_t now_us = esp_timer_get_time();

    // 其它"未配长按且正被按住"的键，补发点按时要保持按住，避免被一起放开
    custom_kbd_state_t held;
    custom_state_clear(&held);
    if (!left_has_long && custom_left_down && custom_left_action.type == CUSTOM_ACTION_KEY) {
        custom_state_add(&held, custom_left_action.modifier, custom_left_action.keycode);
    }
    if (!right_has_long && custom_right_down && custom_right_action.type == CUSTOM_ACTION_KEY) {
        custom_state_add(&held, custom_right_action.modifier, custom_right_action.keycode);
    }

    if (left_has_long && custom_left_down && !custom_left_long_fired &&
        (now_us - custom_left_press_us) / 1000 >= (int64_t)custom_long_press_ms) {
        ESP_LOGI("btn_progress", "左键长按触发: %lld ms", (long long)((now_us - custom_left_press_us) / 1000));
        custom_emit_tap(&custom_left_long_action, &held);
        custom_left_long_fired = true;
    }

    if (right_has_long && custom_right_down && !custom_right_long_fired &&
        (now_us - custom_right_press_us) / 1000 >= (int64_t)custom_long_press_ms) {
        ESP_LOGI("btn_progress", "右键长按触发: %lld ms", (long long)((now_us - custom_right_press_us) / 1000));
        custom_emit_tap(&custom_right_long_action, &held);
        custom_right_long_fired = true;
    }
}

void light_progress(void)
{
    rgb_matrix_task();

    // TODO: for Rainmaker light
    // rgb_matrix_set_suspend_state(false);
}

void btn_progress_set_report_type(btn_report_type_t type)
{
    report_type = type;
}

void btn_progress_set_key_mapping(int mapping_index)
{
    if (mapping_index >= 0 && mapping_index < 17) {
        current_key_mapping_index = mapping_index;
        ESP_LOGI("btn_progress", "按键映射已设置为: %d", mapping_index);
    } else {
        ESP_LOGW("btn_progress", "无效的按键映射索引: %d", mapping_index);
    }
}

int btn_progress_get_key_mapping(void)
{
    return current_key_mapping_index;
}

// ============ 自定义映射接口 ============

void btn_progress_enable_custom_mapping(bool enabled)
{
    custom_mapping_enabled    = enabled;
    custom_left_prev_pressed  = false;
    custom_right_prev_pressed = false;
    custom_left_press_us      = 0;
    custom_right_press_us     = 0;
    custom_left_down          = false;
    custom_right_down         = false;
    custom_left_long_fired    = false;
    custom_right_long_fired   = false;
    ESP_LOGI("btn_progress", "自定义映射模式: %s", enabled ? "启用" : "禁用");
}

bool btn_progress_is_custom_mapping_enabled(void)
{
    return custom_mapping_enabled;
}

void btn_progress_set_custom_left_action(const custom_key_action_t *action)
{
    if (action) {
        memcpy(&custom_left_action, action, sizeof(custom_key_action_t));
        // 确保文本以 '\0' 结尾
        custom_left_action.text[CUSTOM_TEXT_MAX_LEN - 1] = '\0';
        ESP_LOGI("btn_progress", "左键自定义: type=%d modifier=0x%02X keycode=0x%02X", action->type, action->modifier,
                 action->keycode);
    }
}

void btn_progress_set_custom_right_action(const custom_key_action_t *action)
{
    if (action) {
        memcpy(&custom_right_action, action, sizeof(custom_key_action_t));
        custom_right_action.text[CUSTOM_TEXT_MAX_LEN - 1] = '\0';
        ESP_LOGI("btn_progress", "右键自定义: type=%d modifier=0x%02X keycode=0x%02X", action->type, action->modifier,
                 action->keycode);
    }
}

const custom_key_action_t *btn_progress_get_custom_left_action(void)
{
    return &custom_left_action;
}

const custom_key_action_t *btn_progress_get_custom_right_action(void)
{
    return &custom_right_action;
}

// ============ 长按动作 ============

void btn_progress_set_custom_long_left_action(const custom_key_action_t *action)
{
    if (action) {
        memcpy(&custom_left_long_action, action, sizeof(custom_key_action_t));
        custom_left_long_action.text[CUSTOM_TEXT_MAX_LEN - 1] = '\0';
        ESP_LOGI("btn_progress", "左键长按: type=%d modifier=0x%02X keycode=0x%02X", action->type, action->modifier,
                 action->keycode);
    }
}

void btn_progress_set_custom_long_right_action(const custom_key_action_t *action)
{
    if (action) {
        memcpy(&custom_right_long_action, action, sizeof(custom_key_action_t));
        custom_right_long_action.text[CUSTOM_TEXT_MAX_LEN - 1] = '\0';
        ESP_LOGI("btn_progress", "右键长按: type=%d modifier=0x%02X keycode=0x%02X", action->type, action->modifier,
                 action->keycode);
    }
}

const custom_key_action_t *btn_progress_get_custom_long_left_action(void)
{
    return &custom_left_long_action;
}

const custom_key_action_t *btn_progress_get_custom_long_right_action(void)
{
    return &custom_right_long_action;
}

void btn_progress_set_long_press_ms(uint16_t ms)
{
    if (ms < CUSTOM_LONG_PRESS_MIN_MS) {
        ms = CUSTOM_LONG_PRESS_MIN_MS;
    } else if (ms > CUSTOM_LONG_PRESS_MAX_MS) {
        ms = CUSTOM_LONG_PRESS_MAX_MS;
    }
    custom_long_press_ms = ms;
    ESP_LOGI("btn_progress", "长按判定阈值: %u ms", (unsigned)custom_long_press_ms);
}

uint16_t btn_progress_get_long_press_ms(void)
{
    return custom_long_press_ms;
}
