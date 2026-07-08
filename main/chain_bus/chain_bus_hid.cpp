#include "chain_bus_hid.h"
#include "esp_log.h"
#include "tinyusb_hid.h"
#include "ble_hid.h"
#include <string.h>
#include <stdlib.h>

static const char* TAG = "chain_hid";

extern bool g_usb_mapping_enabled;
extern bool g_ble_mapping_enabled;

// HID功能表定义
const hid_func_desc_t hid_func_table[] = {
    // 按照结构体成员顺序: func_type, name, description
    {HID_FUNC_NONE, "无功能", "不执行任何操作"},

    // 键盘功能
    {HID_FUNC_COPY, "复制", "Ctrl+C复制"},
    {HID_FUNC_PASTE, "粘贴", "Ctrl+V粘贴"},
    {HID_FUNC_CUT, "剪切", "Ctrl+X剪切"},
    {HID_FUNC_UNDO, "撤销", "Ctrl+Z撤销"},
    {HID_FUNC_REDO, "重做", "Ctrl+Y重做"},
    {HID_FUNC_SELECT_ALL, "全选", "Ctrl+A全选"},
    {HID_FUNC_DELETE, "删除", "Delete删除"},
    {HID_FUNC_BACKSPACE, "退格", "Backspace退格"},
    {HID_FUNC_ENTER, "回车", "Enter回车"},
    {HID_FUNC_ESC, "Esc", "Esc"},
    {HID_FUNC_TAB, "Tab", "Tab"},
    {HID_FUNC_PAGE_UP, "上翻页", "Page Up"},
    {HID_FUNC_PAGE_DOWN, "下翻页", "Page Down"},

    // 鼠标功能
    {HID_FUNC_MOUSE_BUTTON_LEFT_DOWN, "鼠标左键按下", "鼠标左键按下"},
    {HID_FUNC_MOUSE_BUTTON_LEFT_UP, "鼠标左键释放", "鼠标左键释放"},
    {HID_FUNC_MOUSE_BUTTON_RIGHT_DOWN, "鼠标右键按下", "鼠标右键按下"},
    {HID_FUNC_MOUSE_BUTTON_RIGHT_UP, "鼠标右键释放", "鼠标右键释放"},
    {HID_FUNC_MOUSE_MOVE, "鼠标移动", "移动鼠标指针"},
    {HID_FUNC_MOUSE_MOVE_LEFT, "鼠标左移", "鼠标指针向左移动"},
    {HID_FUNC_MOUSE_MOVE_RIGHT, "鼠标右移", "鼠标指针向右移动"},
    {HID_FUNC_MOUSE_MOVE_UP, "鼠标上移", "鼠标指针向上移动"},
    {HID_FUNC_MOUSE_MOVE_DOWN, "鼠标下移", "鼠标指针向下移动"},
    {HID_FUNC_MOUSE_SCROLL, "鼠标竖向滚轮", "鼠标竖向滚轮"},
    {HID_FUNC_MOUSE_SCROLL_UP, "鼠标竖向滚轮向上", "鼠标竖向滚轮向上"},
    {HID_FUNC_MOUSE_SCROLL_DOWN, "鼠标竖向滚轮向下", "鼠标竖向滚轮向下"},
    {HID_FUNC_MOUSE_PAN, "鼠标横向滚轮", "鼠标横向滚轮"},
    {HID_FUNC_MOUSE_PAN_LEFT, "鼠标横向滚轮向左", "鼠标横向滚轮向左"},
    {HID_FUNC_MOUSE_PAN_RIGHT, "鼠标横向滚轮向右", "鼠标横向滚轮向右"},

    // 音量功能
    {HID_FUNC_VOLUME_UP, "音量增加", "增加系统音量"},
    {HID_FUNC_VOLUME_DOWN, "音量减少", "减少系统音量"},
    {HID_FUNC_VOLUME_MUTE, "静音切换", "切换静音状态"},

    // 媒体控制功能
    {HID_FUNC_MEDIA_PLAY_PAUSE, "播放暂停", "播放/暂停媒体"},
    {HID_FUNC_MEDIA_NEXT, "下一首", "播放下一首"},
    {HID_FUNC_MEDIA_PREV, "上一首", "播放上一首"},
    {HID_FUNC_MEDIA_STOP, "停止播放", "停止媒体播放"},

    {HID_FUNC_JOYSTICK_WASD, "摇杆WASD", "摇杆映射WASD八方向"},
    {HID_FUNC_JOYSTICK_ARROWS, "摇杆方向键", "摇杆映射方向键八方向"},
};

// 鼠标移动中枢状态
typedef struct {
    bool need_report;      // 需要上报（按键释放）
    int16_t target_x;      // 目标X移动量
    int16_t target_y;      // 目标Y移动量
    int16_t current_x;     // 当前X位置
    int16_t current_y;     // 当前Y位置
    uint8_t buttons;       // 按键状态
    int8_t wheel;          // 滚轮状态
    int8_t pan;            // 平移状态
    uint8_t smooth_steps;  // 平滑步数
    uint8_t current_step;  // 当前步数
} mouse_hub_t;

static mouse_hub_t g_mouse_hub = {
    false,  // need_report
    0,      // target_x
    0,      // target_y
    0,      // current_x
    0,      // current_y
    0,      // buttons
    0,      // wheel
    0,      // pan
    24,     // smooth_steps
    0       // current_step
};

// 鼠标中枢任务控制
static TaskHandle_t g_mouse_hub_task_handle = NULL;

// 鼠标按键位掩码定义
#define MOUSE_BUTTON_LEFT   0x01
#define MOUSE_BUTTON_RIGHT  0x02
#define MOUSE_BUTTON_MIDDLE 0x04

// 修饰键位掩码定义
#define MODIFIER_LEFT_CTRL  0x01
#define MODIFIER_LEFT_SHIFT 0x02
#define MODIFIER_LEFT_ALT   0x04
#define MODIFIER_LEFT_GUI   0x08

// 内部函数声明
static esp_err_t send_keyboard_report(uint8_t modifier, uint8_t keycode);
static esp_err_t send_keyboard_keys_report(uint8_t modifier, const uint8_t* keycodes, uint8_t key_count);
static esp_err_t send_consumer_report(uint16_t keycode);
static esp_err_t send_mouse_report(uint8_t buttons, int8_t x, int8_t y, int8_t wheel, int8_t pan);

// 鼠标中枢内部函数
static void mouse_hub_update_movement(void);

// 发送键盘按键（带自动释放）
static esp_err_t send_keyboard_key_with_release(uint8_t modifier, uint8_t keycode);
// 发送消费者按键（带自动释放）
static esp_err_t send_consumer_key_with_release(uint16_t keycode);
// 更新鼠标竖向滚轮
static esp_err_t update_mouse_scroll(int8_t wheel_delta);
// 更新鼠标移动
static esp_err_t update_mouse_movement(int16_t x, int16_t y);
// 更新鼠标按键
static esp_err_t update_mouse_button(uint8_t buttons);
// 更新鼠标横向滚轮
static esp_err_t update_mouse_pan(int8_t pan);

// 鼠标中枢任务函数
static void mouse_hub_task(void* pvParameters);

// API函数实现

esp_err_t chain_bus_hid_init(void)
{
    ESP_LOGI(TAG, "初始化Chain HID系统");

    // 启动鼠标中枢任务
    xTaskCreate(mouse_hub_task, "mouse_hub", 4096, NULL, 8, &g_mouse_hub_task_handle);

    ESP_LOGI(TAG, "Chain HID系统初始化完成");
    return ESP_OK;
}

esp_err_t chain_bus_hid_set_enabled(bool enabled)
{
    ESP_LOGI(TAG, "HID映射%s", enabled ? "启用" : "禁用");
    return ESP_OK;
}

bool chain_bus_hid_is_enabled(void)
{
    return true;
}

esp_err_t chain_bus_hid_trigger_function(hid_func_type_t func_type, hid_func_params_t* params)
{
    esp_err_t ret = ESP_OK;

    switch (func_type) {
        case HID_FUNC_NONE:
            // 无操作
            break;

        // 键盘功能
        case HID_FUNC_COPY:
            ret = send_keyboard_key_with_release(MODIFIER_LEFT_CTRL, KC_C);
            break;
        case HID_FUNC_PASTE:
            ret = send_keyboard_key_with_release(MODIFIER_LEFT_CTRL, KC_V);
            break;
        case HID_FUNC_CUT:
            ret = send_keyboard_key_with_release(MODIFIER_LEFT_CTRL, KC_X);
            break;
        case HID_FUNC_UNDO:
            ret = send_keyboard_key_with_release(MODIFIER_LEFT_CTRL, KC_Z);
            break;
        case HID_FUNC_REDO:
            ret = send_keyboard_key_with_release(MODIFIER_LEFT_CTRL, KC_Y);
            break;
        case HID_FUNC_SELECT_ALL:
            ret = send_keyboard_key_with_release(MODIFIER_LEFT_CTRL, KC_A);
            break;
        case HID_FUNC_DELETE:
            ret = send_keyboard_key_with_release(0, KC_DELETE);
            break;
        case HID_FUNC_BACKSPACE:
            ret = send_keyboard_key_with_release(0, KC_BACKSPACE);
            break;
        case HID_FUNC_ENTER:
            ret = send_keyboard_key_with_release(0, KC_ENTER);
            break;
        case HID_FUNC_ESC:
            ret = send_keyboard_key_with_release(0, KC_ESC);
            break;
        case HID_FUNC_TAB:
            ret = send_keyboard_key_with_release(0, KC_TAB);
            break;
        case HID_FUNC_PAGE_UP:
            ret = send_keyboard_key_with_release(0, KC_PAGE_UP);
            break;
        case HID_FUNC_PAGE_DOWN:
            ret = send_keyboard_key_with_release(0, KC_PAGE_DOWN);
            break;

        // 鼠标功能
        case HID_FUNC_MOUSE_BUTTON_LEFT_DOWN:
            ret = update_mouse_button(MOUSE_BUTTON_LEFT);
            break;
        case HID_FUNC_MOUSE_BUTTON_LEFT_UP:
            ret = update_mouse_button(0);
            break;
        case HID_FUNC_MOUSE_BUTTON_RIGHT_DOWN:
            ret = update_mouse_button(MOUSE_BUTTON_RIGHT);
            break;
        case HID_FUNC_MOUSE_BUTTON_RIGHT_UP:
            ret = update_mouse_button(0);
            break;
        case HID_FUNC_MOUSE_MOVE:
            if (params) {
                ret = update_mouse_movement(params->mouse_value.x, params->mouse_value.y);
            }
            break;
        case HID_FUNC_MOUSE_MOVE_LEFT:
            ret = update_mouse_movement(-10, 0);
            break;
        case HID_FUNC_MOUSE_MOVE_RIGHT:
            ret = update_mouse_movement(10, 0);
            break;
        case HID_FUNC_MOUSE_MOVE_UP:
            ret = update_mouse_movement(0, -10);
            break;
        case HID_FUNC_MOUSE_MOVE_DOWN:
            ret = update_mouse_movement(0, 10);
            break;
        case HID_FUNC_MOUSE_SCROLL:
            if (params) {
                if (params->mouse_value.wheel != 0) {
                    ret = update_mouse_scroll(params->mouse_value.wheel);
                }
                if (params->mouse_value.pan != 0) {
                    ret = update_mouse_pan(params->mouse_value.pan);
                }
            }
            break;
        case HID_FUNC_MOUSE_PAN:
            if (params) {
                if (params->mouse_value.pan != 0) {
                    ret = update_mouse_pan(params->mouse_value.pan);
                }
                if (params->mouse_value.wheel != 0) {
                    ret = update_mouse_scroll(params->mouse_value.wheel);
                }
            }
            break;
        case HID_FUNC_MOUSE_SCROLL_UP:
            ret = update_mouse_scroll(10);
            break;
        case HID_FUNC_MOUSE_SCROLL_DOWN:
            ret = update_mouse_scroll(-10);
            break;
        case HID_FUNC_MOUSE_PAN_LEFT:
            ret = update_mouse_pan(-10);
            break;
        case HID_FUNC_MOUSE_PAN_RIGHT:
            ret = update_mouse_pan(10);
            break;
        // 音量功能
        case HID_FUNC_VOLUME_UP:
            ret = send_consumer_key_with_release(HID_USAGE_CONSUMER_VOLUME_INCREMENT);
            break;
        case HID_FUNC_VOLUME_DOWN:
            ret = send_consumer_key_with_release(HID_USAGE_CONSUMER_VOLUME_DECREMENT);
            break;
        case HID_FUNC_VOLUME_MUTE:
            ret = send_consumer_key_with_release(HID_USAGE_CONSUMER_MUTE);
            break;

        // 媒体控制功能
        case HID_FUNC_MEDIA_PLAY_PAUSE:
            ret = send_consumer_key_with_release(HID_USAGE_CONSUMER_PLAY_PAUSE);
            break;
        case HID_FUNC_MEDIA_NEXT:
            ret = send_consumer_key_with_release(HID_USAGE_CONSUMER_SCAN_NEXT);
            break;
        case HID_FUNC_MEDIA_PREV:
            ret = send_consumer_key_with_release(HID_USAGE_CONSUMER_SCAN_PREVIOUS);
            break;
        case HID_FUNC_MEDIA_STOP:
            ret = send_consumer_key_with_release(HID_USAGE_CONSUMER_STOP);
            break;

        default:
            ESP_LOGW(TAG, "未知的HID功能类型: %d", func_type);
            ret = ESP_ERR_INVALID_ARG;
            break;
    }

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "HID功能触发失败: %d, 错误: %s", func_type, esp_err_to_name(ret));
    } else {
        ESP_LOGD(TAG, "HID功能触发成功: %d", func_type);
    }

    return ret;
}

const hid_func_desc_t* chain_bus_hid_get_func_desc(hid_func_type_t func_type)
{
    if (func_type >= HID_FUNC_MAX) {
        return NULL;
    }

    for (size_t i = 0; i < HID_FUNC_TABLE_SIZE; i++) {
        if (hid_func_table[i].func_type == func_type) {
            return &hid_func_table[i];
        }
    }

    return NULL;
}

const hid_func_desc_t* chain_bus_hid_get_all_functions(size_t* count)
{
    if (count) {
        *count = HID_FUNC_TABLE_SIZE;
    }
    return hid_func_table;
}

// 摇杆键盘映射状态
typedef struct {
    uint8_t keys[2];
    uint8_t count;
    hid_func_type_t mode;
} joystick_key_hub_t;

static joystick_key_hub_t g_joy_key_hub = {{0}, 0, HID_FUNC_NONE};

static bool joystick_keys_equal(const uint8_t* a, uint8_t ac, const uint8_t* b, uint8_t bc)
{
    if (ac != bc) {
        return false;
    }
    for (uint8_t i = 0; i < ac; i++) {
        if (a[i] != b[i]) {
            return false;
        }
    }
    return true;
}

esp_err_t chain_bus_hid_update_joystick_keys(hid_func_type_t func_type, int8_t x, int8_t y)
{
    if (func_type != HID_FUNC_JOYSTICK_WASD && func_type != HID_FUNC_JOYSTICK_ARROWS) {
        return ESP_ERR_INVALID_ARG;
    }

    const int8_t deadzone = 20;
    uint8_t new_keys[2]   = {0};
    uint8_t new_count     = 0;
    const bool use_wasd   = (func_type == HID_FUNC_JOYSTICK_WASD);

    if (abs(x) > deadzone || abs(y) > deadzone) {
        const bool up    = y > deadzone;
        const bool down  = y < -deadzone;
        const bool left  = x < -deadzone;
        const bool right = x > deadzone;

        if (use_wasd) {
            if (up) {
                new_keys[new_count++] = KC_W;
            }
            if (down) {
                new_keys[new_count++] = KC_S;
            }
            if (left) {
                new_keys[new_count++] = KC_A;
            }
            if (right) {
                new_keys[new_count++] = KC_D;
            }
        } else {
            if (up) {
                new_keys[new_count++] = KC_UP;
            }
            if (down) {
                new_keys[new_count++] = KC_DOWN;
            }
            if (left) {
                new_keys[new_count++] = KC_LEFT;
            }
            if (right) {
                new_keys[new_count++] = KC_RIGHT;
            }
        }
    }

    if (g_joy_key_hub.mode != func_type ||
        !joystick_keys_equal(g_joy_key_hub.keys, g_joy_key_hub.count, new_keys, new_count)) {
        g_joy_key_hub.mode  = func_type;
        g_joy_key_hub.count = new_count;
        memcpy(g_joy_key_hub.keys, new_keys, sizeof(new_keys));
        return send_keyboard_keys_report(0, new_keys, new_count);
    }

    return ESP_OK;
}

// 鼠标中枢任务实现
static void mouse_hub_task(void* pvParameters)
{
    ESP_LOGI(TAG, "鼠标中枢任务启动");

    while (1) {
        // 更新鼠标移动
        if (g_mouse_hub.need_report || g_mouse_hub.target_x != 0 || g_mouse_hub.target_y != 0 ||
            g_mouse_hub.wheel != 0 || g_mouse_hub.pan != 0) {
            mouse_hub_update_movement();
        }

        // 延时一段时间，避免占用过多CPU
        vTaskDelay(pdMS_TO_TICKS(5));
    }

    ESP_LOGI(TAG, "鼠标中枢任务退出");
    vTaskDelete(NULL);
}

// 鼠标中枢功能实现

static void mouse_hub_update_movement(void)
{
    g_mouse_hub.need_report = false;
    int8_t send_x           = 0;
    int8_t send_y           = 0;
    int8_t send_wheel       = 0;
    int8_t send_pan         = 0;

    // 处理XY移动的平滑插值
    if (g_mouse_hub.target_x != 0 || g_mouse_hub.target_y != 0) {
        // 计算当前步的移动量
        if (g_mouse_hub.current_step < g_mouse_hub.smooth_steps) {
            // 使用线性插值计算当前步的移动量
            int16_t total_x = g_mouse_hub.target_x - g_mouse_hub.current_x;
            int16_t total_y = g_mouse_hub.target_y - g_mouse_hub.current_y;

            // 计算每步的移动量
            uint8_t remaining_steps = g_mouse_hub.smooth_steps - g_mouse_hub.current_step;
            if (remaining_steps == 0) {
                remaining_steps = 1;  // 防止除零
            }

            int16_t step_x = total_x / remaining_steps;
            int16_t step_y = total_y / remaining_steps;

            // 确保至少移动1个单位（如果还有剩余）
            if (total_x > 0 && step_x == 0) step_x = 1;
            if (total_x < 0 && step_x == 0) step_x = -1;
            if (total_y > 0 && step_y == 0) step_y = 1;
            if (total_y < 0 && step_y == 0) step_y = -1;

            // 限制在有效范围内
            send_x = (step_x > 127) ? 127 : ((step_x < -127) ? -127 : (int8_t)step_x);
            send_y = (step_y > 127) ? 127 : ((step_y < -127) ? -127 : (int8_t)step_y);

            // 更新当前位置
            g_mouse_hub.current_x += step_x;
            g_mouse_hub.current_y += step_y;

            // 更新步数
            g_mouse_hub.current_step++;

            // 检查是否完成移动
            if (g_mouse_hub.current_step >= g_mouse_hub.smooth_steps ||
                (abs(g_mouse_hub.target_x - g_mouse_hub.current_x) <= 1 &&
                 abs(g_mouse_hub.target_y - g_mouse_hub.current_y) <= 1)) {
                // 移动完成，重置状态
                g_mouse_hub.current_x    = g_mouse_hub.target_x;
                g_mouse_hub.current_y    = g_mouse_hub.target_y;
                g_mouse_hub.current_step = 0;
                g_mouse_hub.target_x     = 0;
                g_mouse_hub.target_y     = 0;

                // 发送最终位置报告
                step_x = 0;
                step_y = 0;
            }
        }
    }

    // 检查是否有wheel或pan数据需要上报
    if (g_mouse_hub.wheel != 0 || g_mouse_hub.pan != 0) {
        send_wheel = g_mouse_hub.wheel;
        send_pan   = g_mouse_hub.pan;

        // 上报后清零
        g_mouse_hub.wheel       = 0;
        g_mouse_hub.pan         = 0;
        g_mouse_hub.need_report = true;
    }

    send_mouse_report(g_mouse_hub.buttons, send_x, send_y, send_wheel, send_pan);
}

// 内部函数实现

static esp_err_t send_keyboard_report(uint8_t modifier, uint8_t keycode)
{
    hid_report_t report;
    memset(&report, 0, sizeof(report));

    report.report_id                = REPORT_ID_KEYBOARD;
    report.keyboard_report.modifier = modifier;
    report.keyboard_report.reserved = 0;
    memset(report.keyboard_report.keycode, 0, sizeof(report.keyboard_report.keycode));
    if (keycode != 0) {
        report.keyboard_report.keycode[0] = keycode;
    }

    if (g_usb_mapping_enabled) {
        tinyusb_hid_keyboard_report(report);
    }
    if (g_ble_mapping_enabled) {
        ble_hid_keyboard_report(report);
    }

    return ESP_OK;
}

static esp_err_t send_keyboard_keys_report(uint8_t modifier, const uint8_t* keycodes, uint8_t key_count)
{
    hid_report_t report;
    memset(&report, 0, sizeof(report));

    report.report_id                = REPORT_ID_KEYBOARD;
    report.keyboard_report.modifier = modifier;
    report.keyboard_report.reserved = 0;
    memset(report.keyboard_report.keycode, 0, sizeof(report.keyboard_report.keycode));
    if (keycodes && key_count > 0) {
        uint8_t copy_count = (key_count > 6) ? 6 : key_count;
        memcpy(report.keyboard_report.keycode, keycodes, copy_count);
    }

    if (g_usb_mapping_enabled) {
        tinyusb_hid_keyboard_report(report);
    }
    if (g_ble_mapping_enabled) {
        ble_hid_keyboard_report(report);
    }

    return ESP_OK;
}

static esp_err_t send_consumer_report(uint16_t keycode)
{
    hid_report_t report;
    memset(&report, 0, sizeof(report));

    report.report_id               = REPORT_ID_CONSUMER;
    report.consumer_report.keycode = keycode;

    if (g_usb_mapping_enabled) {
        tinyusb_hid_keyboard_report(report);
    }
    if (g_ble_mapping_enabled) {
        ble_hid_keyboard_report(report);
    }

    return ESP_OK;
}

static esp_err_t send_mouse_report(uint8_t buttons, int8_t x, int8_t y, int8_t wheel, int8_t pan)
{
    // cost too much, 2048 not enough
    // ESP_LOGI(TAG, "鼠标报告: 按键=0x%02X, X=%d, Y=%d, 滚轮=%d, 平移=%d", buttons, x, y, wheel, pan);

    hid_report_t report;
    memset(&report, 0, sizeof(report));

    report.report_id            = REPORT_ID_MOUSE;
    report.mouse_report.buttons = buttons;
    report.mouse_report.x       = x;
    report.mouse_report.y       = y;
    report.mouse_report.wheel   = wheel;
    report.mouse_report.pan     = pan;

    if (g_usb_mapping_enabled) {
        tinyusb_hid_mouse_report(report);
    }
    if (g_ble_mapping_enabled) {
        ble_hid_keyboard_report(report);
    }

    return ESP_OK;
}

esp_err_t chain_bus_hid_send_mouse_delta(int8_t dx, int8_t dy)
{
    if (dx == 0 && dy == 0) {
        return ESP_OK;
    }
    return send_mouse_report(g_mouse_hub.buttons, dx, dy, 0, 0);
}

// 发送键盘按键（带自动释放）
static esp_err_t send_keyboard_key_with_release(uint8_t modifier, uint8_t keycode)
{
    esp_err_t ret = ESP_OK;

    // 发送按下报告
    ret = send_keyboard_report(modifier, keycode);
    if (ret != ESP_OK) {
        return ret;
    }

    // 短暂延时确保按下报告被处理
    vTaskDelay(pdMS_TO_TICKS(5));

    // 发送释放报告（清空修饰键和按键码）
    ret = send_keyboard_report(0, 0);

    return ret;
}

// 发送消费者按键（带自动释放）
static esp_err_t send_consumer_key_with_release(uint16_t keycode)
{
    esp_err_t ret = ESP_OK;

    // 发送按下报告
    ret = send_consumer_report(keycode);
    if (ret != ESP_OK) {
        return ret;
    }

    // 短暂延时确保按下报告被处理
    vTaskDelay(pdMS_TO_TICKS(5));

    // 发送释放报告（按键码清零）
    ret = send_consumer_report(0);

    return ret;
}

// 发送鼠标竖向滚轮（带自动释放）
static esp_err_t update_mouse_scroll(int8_t wheel_delta)
{
    esp_err_t ret = ESP_OK;

    g_mouse_hub.wheel       = wheel_delta;
    g_mouse_hub.need_report = true;

    return ret;
}

static esp_err_t update_mouse_movement(int16_t x, int16_t y)
{
    if (x == 0 && y == 0) {
        return ESP_OK;
    }

    // 摇杆等小增量：每帧直接上报，避免平滑中枢导致跟手迟滞
    if (abs(x) <= 63 && abs(y) <= 63) {
        return send_mouse_report(g_mouse_hub.buttons, (int8_t)x, (int8_t)y, 0, 0);
    }

    // 大位移（编码器等）：走平滑中枢
    if (x != g_mouse_hub.target_x || y != g_mouse_hub.target_y) {
        g_mouse_hub.current_step = 0;
        g_mouse_hub.current_x    = 0;
        g_mouse_hub.current_y    = 0;
        g_mouse_hub.target_x     = x;
        g_mouse_hub.target_y     = y;
        g_mouse_hub.need_report  = true;
    }

    return ESP_OK;
}

static esp_err_t update_mouse_button(uint8_t buttons)
{
    g_mouse_hub.buttons     = buttons;
    g_mouse_hub.need_report = true;
    return ESP_OK;
}

static esp_err_t update_mouse_pan(int8_t pan)
{
    g_mouse_hub.pan         = pan;
    g_mouse_hub.need_report = true;
    return ESP_OK;
}
