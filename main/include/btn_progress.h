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

/* ---------- 按键时延取证（纯 RAM，仅本次运行有效）----------
 *
 * 2026-09-15 实测现象：OFF/WiFi 档插线连电脑时按键"将近一秒才反应"，而蓝牙档正常。
 * 静态分析覆盖了从扫描到上报的整条链路都推不出 1 秒，因此改用实测分辨：
 *
 *   - 快速点按（物理按住约 100ms）若被量成约 1000ms → 按下/松手边沿被晚检出，
 *     问题在扫描 / GPIO 唤醒路径；
 *   - 若被量成约 100ms → 边沿是准的，问题在报告送达（USB 栈 / 多通道竞争）。
 *
 * `cb_gap_ms_max` 是同一结论的旁证：按住期间相邻回调间隔若远大于扫描周期（1ms），
 * 说明扫描任务被长时间饿过。
 *
 * 只写 RAM、不落盘 —— 复现场景与读 /diag 都在 WiFi 档，不需要跨重启保留。
 */
typedef struct {
    uint32_t tap_hold_ms_last;  /**< 最近一次短按的实测按住时长 */
    uint32_t tap_hold_ms_max;   /**< 全部短按里的最大值 */
    uint32_t tap_count;         /**< 短按次数 */
    uint32_t long_fire_ms_last; /**< 最近一次长按触发时的按住时长 */
    uint32_t long_count;        /**< 长按触发次数 */
    uint32_t cb_gap_ms_max;     /**< 按住期间相邻按键回调间隔的最大值 */
    uint32_t cb_down_count;     /**< 参与统计的回调次数 */
} btn_latency_stats_t;

/**
 * @brief 取出按键时延取证快照（纯 RAM 读取，可随时调用）。
 *
 * @param out 输出结构体，调用方分配。
 */
void btn_progress_get_latency_stats(btn_latency_stats_t *out);

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

/* ---------- 板载 LED 渲染协调 ----------
 *
 * bsp_ws2812 与 rgb_matrix 使用同一个 led_strip 句柄（同一份像素缓冲），而 rgb_matrix
 * 每次渲染都会刷新整条灯带。主程序若要直接写像素（静态按键色、充电指示），必须与它串行，
 * 否则"写左灯 + 写右灯"两次调用之间可能被 rgb_matrix 的清屏冲掉（表现为某个键的灯熄灭）。
 */

/**
 * @brief 获取板载 LED 访问锁。仅用于直接写像素，不要跨越 vTaskDelay 持有。
 */
void light_progress_lock(void);

/**
 * @brief 释放板载 LED 访问锁。
 */
void light_progress_unlock(void);

/**
 * @brief 接管/释放板载灯带渲染。
 *
 * on = true 时暂停 rgb_matrix 渲染，调用方可自行画像素（如充电电量常亮指示）；
 * 期间按键热力灯效不会显示。置回 false 后 rgb_matrix 恢复正常渲染。
 * 注意：内部自行加锁，调用时不要持有 light_progress_lock()。
 */
void light_progress_set_overlay(bool on);

/**
 * @brief 当前是否处于 overlay 接管状态。
 */
bool light_progress_is_overlay_active(void);

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

/**
 * @brief 最近一次上报中是否有键处于按下状态。
 *        灯效任务用它决定心跳周期（空闲 200ms / 有键 10ms）以配合 light sleep。
 */
bool btn_progress_has_pressed_key(void);

/**
 * @brief 最近是否发生过按键事件（含刚松手的那一次），用于让灯效保持快渲染。
 *
 *        热力图淡出是"每次渲染衰减一个定值"，空闲 200ms 的渲染周期会让淡出慢 20 倍
 *        （按一下就常亮）。灯效任务据此在按键后 LED_ANIM_SETTLE_MS 内继续用 10ms 周期。
 *        无按键活动时返回 false，省电行为不变。
 */
bool btn_progress_led_anim_active(void);

#ifdef __cplusplus
}
#endif
