/*
 * SPDX-FileCopyrightText: 2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#pragma once

#include "esp_err.h"
#include "keyboard_button.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    TINYUSB_HID_REPORT = 0,
    BLE_HID_REPORT,
    USB_CDC_REPORT,
    ALL_REPORT,
    BTN_REPORT_TYPE_MAX,
} btn_report_type_t;

typedef struct {
    uint32_t report_id;  // Report identifier
    union {
        struct {
            uint8_t modifier;        // Modifier keys
            uint8_t reserved;        // Reserved byte
            uint8_t keycode[15];     // Keycode
        } keyboard_full_key_report;  // Keyboard full key report
        struct {
            uint8_t modifier;    // Modifier keys
            uint8_t reserved;    // Reserved byte
            uint8_t keycode[6];  // Keycodes
        } keyboard_report;       // Keyboard report
        struct {
            uint16_t keycode;  // Keycode
        } consumer_report;
        struct {
            uint8_t buttons; /**< buttons mask for currently pressed buttons in the mouse. */
            int8_t x;        /**< Current delta x movement of the mouse. */
            int8_t y;        /**< Current delta y movement on the mouse. */
            int8_t wheel;    /**< Current delta wheel movement on the mouse. */
            int8_t pan;      // using AC Pan
        } mouse_report;
    };
} hid_report_t;

/**
 * @brief Button progress function.
 *
 * @param kbd_report Keyboard button report.
 */
void btn_progress(keyboard_btn_report_t kbd_report);

/**
 * @brief 初始化（创建 HID 发送互斥锁）。应在按键扫描任务启动前调用一次。
 */
void btn_progress_init(void);

/**
 * @brief 长按到点触发检查，需要由一个约 10ms 周期的任务持续调用。
 *
 * 只有"自定义映射 + 配了长按 + 该键正被按住"时才真正做事，其余情况立即返回。
 */
void btn_progress_tick(void);

/**
 * @brief Set the report type for button progress.
 *
 * @param type Button report type.
 */
void btn_progress_set_report_type(btn_report_type_t type);

/**
 * @brief Function to handle light progress.
 */
void light_progress(void);

/**
 * @brief Set the key mapping index for button progress.
 *
 * @param mapping_index Key mapping index (0-3)
 */
void btn_progress_set_key_mapping(int mapping_index);

/**
 * @brief Get the current key mapping index.
 *
 * @return Current key mapping index
 */
int btn_progress_get_key_mapping(void);

// ============ 自定义映射 ============

#define CUSTOM_TEXT_MAX_LEN 64

/** 自定义动作类型 */
typedef enum {
    CUSTOM_ACTION_KEY  = 0,  // 单键 / 组合键 (modifier + keycode)
    CUSTOM_ACTION_TEXT = 1,  // 文本字符串逐字输出
    CUSTOM_ACTION_NONE = 2,  // 未配置。仅用于"长按动作"槽，表示该键不做长按判定
} custom_action_type_t;

/** 长按判定默认阈值与允许范围 (ms) */
#define CUSTOM_LONG_PRESS_DEFAULT_MS 500
#define CUSTOM_LONG_PRESS_MIN_MS     100
#define CUSTOM_LONG_PRESS_MAX_MS     2000

/** 自定义按键动作 */
typedef struct {
    custom_action_type_t type;  // 动作类型
    uint8_t modifier;  //  bit0=LCtrl bit1=LShift bit2=LAlt bit3=LGUI bit4=RCtrl bit5=RShift bit6=RAlt bit7=RGUI
    uint8_t keycode;   // HID 键码 (key/combo 有效)
    char text[CUSTOM_TEXT_MAX_LEN];  // 文本字符串 (text 有效)
} custom_key_action_t;

/**
 * @brief 启用/禁用自定义映射模式（禁用时回退到预设 keymap 模式）
 */
void btn_progress_enable_custom_mapping(bool enabled);

/**
 * @brief 获取自定义映射模式状态
 */
bool btn_progress_is_custom_mapping_enabled(void);

/**
 * @brief 设置左键自定义动作
 */
void btn_progress_set_custom_left_action(const custom_key_action_t *action);

/**
 * @brief 设置右键自定义动作
 */
void btn_progress_set_custom_right_action(const custom_key_action_t *action);

/**
 * @brief 获取左键自定义动作
 */
const custom_key_action_t *btn_progress_get_custom_left_action(void);

/**
 * @brief 获取右键自定义动作
 */
const custom_key_action_t *btn_progress_get_custom_right_action(void);

// ============ 自定义映射：长按（方案A：按住到阈值即触发） ============

/**
 * @brief 设置左键长按动作。action->type 为 CUSTOM_ACTION_NONE 表示取消长按。
 *
 * 配了长按的键：按下后先不发报告，按住到阈值（默认 500ms，见 btn_progress_tick()）
 * 时立即执行长按动作；未到阈值就松手则判为短按。未配置长按的键保持
 * "按下即上报"的原行为，零额外延迟。
 */
void btn_progress_set_custom_long_left_action(const custom_key_action_t *action);

/**
 * @brief 设置右键长按动作。action->type 为 CUSTOM_ACTION_NONE 表示取消长按。
 */
void btn_progress_set_custom_long_right_action(const custom_key_action_t *action);

/**
 * @brief 获取左键长按动作
 */
const custom_key_action_t *btn_progress_get_custom_long_left_action(void);

/**
 * @brief 获取右键长按动作
 */
const custom_key_action_t *btn_progress_get_custom_long_right_action(void);

/**
 * @brief 设置长按判定阈值 (ms)，会被夹到 [CUSTOM_LONG_PRESS_MIN_MS, CUSTOM_LONG_PRESS_MAX_MS]
 */
void btn_progress_set_long_press_ms(uint16_t ms);

/**
 * @brief 获取长按判定阈值 (ms)
 */
uint16_t btn_progress_get_long_press_ms(void);

#ifdef __cplusplus
}
#endif
