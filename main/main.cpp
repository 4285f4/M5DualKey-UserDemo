/*
 *SPDX-FileCopyrightText: 2025 M5Stack Technology CO LTD
 *
 *SPDX-License-Identifier: MIT
 */

extern "C" {

#include "esp_log.h"
#include "esp_pm.h"
#include "esp_sleep.h"
#include "esp_system.h" /*!< esp_reset_reason() / ESP_RST_DEEPSLEEP（深睡取证用） */
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/timers.h"
#include "nvs_flash.h"
#include "bsp/esp-bsp.h"
#include "settings.h"
#include "btn_progress.h"
#include "rgb_matrix.h"
#include "esp_timer.h"
#include "tinyusb_hid.h"
#include "ble_hid.h"
#include "dual_button.h"
#include "iot_button.h"
#include "tinyusb_cdc.h"
#include "adc_detect.h"
#include "diag_power.h"
#include "driver/gpio.h"
#include "driver/rtc_io.h"
#include "chain_bus.h"
#include "esp_wifi.h"
#include "esp_netif.h"
#include "esp_event.h"
#include "esp_http_server.h"
#include "lwip/inet.h"
#include "string.h"
#include <stdlib.h>
#include <time.h>
#include "esp_attr.h"
#include "esp_sleep.h"
#include "esp_mac.h"
#include "cJSON.h"
#include "esp_crc.h"
#include "web_assets.h"

// Web文件嵌入
extern "C" {
extern const char index_html_start[] asm("_binary_index_html_start");
extern const char index_html_end[] asm("_binary_index_html_end");
extern const char styles_css_start[] asm("_binary_styles_css_start");
extern const char styles_css_end[] asm("_binary_styles_css_end");
extern const char script_js_start[] asm("_binary_script_js_start");
extern const char script_js_end[] asm("_binary_script_js_end");
extern const char favicon_ico_start[] asm("_binary_favicon_ico_start");
extern const char favicon_ico_end[] asm("_binary_favicon_ico_end");
}

static const char *TAG = "main";

// WiFi配置
#define AP_SSID       "DualKey_mac"
#define AP_PASSWORD   "12345678"
#define MAXIMUM_RETRY 5

// WiFi配置数据结构
typedef struct {
    char ssid[32];       // SSID
    char password[64];   // 密码
    bool use_static_ip;  // 是否使用静态IP
    char static_ip[16];  // 静态IP地址
    char netmask[16];    // 子网掩码
    char gateway[16];    // 网关
    uint32_t crc32;      // CRC校验
} wifi_config_saved_t;

static keyboard_btn_handle_t kbd_handle        = NULL;
static TaskHandle_t light_progress_task_handle = NULL;
static led_strip_handle_t led_strip            = NULL;
uint64_t last_time                             = 0;
sys_param_t *sys_param;

// WiFi相关变量
static int s_retry_num       = 0;
static httpd_handle_t server = NULL;
static bool wifi_connected   = false;
static bool wifi_ap_mode     = false;  // WiFi运行模式标志

// WebSocket相关变量
static int websocket_fd                   = -1;
static TaskHandle_t websocket_task_handle = NULL;
QueueHandle_t status_refresh_queue        = NULL;  // 状态刷新消息队列
static bool send_bus_all_data             = false;

int switch_pos             = 0;  // 0:center, 1:left, 2:right
/*!< 是否已开启自动 light sleep（仅蓝牙档）。运行期判档的"自证"逻辑依赖它：
 *   只有休眠生效期间 ADC 才可能读到无意义值；未开休眠时读数可信，无需自证。 */
static bool g_light_sleep_on = false;
bool g_usb_mapping_enabled = true;
bool g_ble_mapping_enabled = true;
uint8_t g_connect_status   = 0;      // 0:未连接, 1:连接中, 2:已连接
bool g_ble_adv_status      = false;  // 蓝牙广播状态: false=未广播, true=正在广播

/*!< ========================= 自动关机（仅蓝牙档） =========================
 *
 *   用户诉求：蓝牙档长时间没有主机连接时，不能一直广播耗电。
 *   方案（2026-09-15 用户拍板）：无连接持续超过设定时长 → 闪灯提示 → 深度睡眠。
 *   · 超时可网页设置（1~120 分钟，0 = 关闭），默认 15 分钟；
 *   · **只在这一档生效**：唯一调用点 auto_off_tick() 挂在 device_status_task 上，
 *     而该任务只在蓝牙档创建；
 *   · 插着 USB 时不算空闲 —— 那时没有耗电之忧，且要保住充电电量指示；
 *   · 唤醒 = 按任一键（GPIO0/17 拉低），或拨档（经中间档必然断电，等于重新上电）。
 *     深睡唤醒即重启，这是用户已知并接受的语义。
 *
 *   ⚠️ NVS 用独立 namespace/key，不动 settings.c 的 sys_param blob —— 那是个定长
 *   blob，往里加字段会让已烧录设备上的旧 blob 直接读取失败
 *   （nvs_get_blob 返回 ESP_ERR_NVS_INVALID_LENGTH）。
 *   ⚠️ 写 NVS 只发生在网页下发配置时（用户触发、极低频），不走任何周期性路径 ——
 *   本项目已两次因"高频路径写 flash"翻车。
 */
#define AUTO_OFF_NVS_NS      "power_cfg"
#define AUTO_OFF_NVS_KEY     "auto_off_min"
#define AUTO_OFF_DEFAULT_MIN 15
#define AUTO_OFF_MAX_MIN     120

static uint16_t g_auto_off_min = AUTO_OFF_DEFAULT_MIN;

/*!< 当前"无连接"已持续秒数（file scope 便于 /diag 观测；判定逻辑见 auto_off_tick）。 */
static uint32_t s_auto_off_idle_s = 0;

/*!< 深睡取证（放 RTC 域，跨深睡保留；只有"经过中间 OFF 档断电"才会丢）。
 *
 *   为什么需要（2026-09-15 实机）：蓝牙档两次自动关机都记到了 BOOT#reason=8
 *   （= 深睡唤醒），而用户只按过一次键 —— 说明其中一次是**假唤醒**（睡下即被拉起来）。
 *   可 BLE 档没有控制台，EXT1 细节读不到，所以把这几项落 NVS 事件日志，
 *   切到 WiFi 档拉 /diag 即可事后取证：
 *     · ext1 mask  → 究竟哪颗键/哪个脚触发（bit0=Key1/GPIO0, bit17=Key2/GPIO17）
 *     · 睡了多久   → ≈0s = 睡下即醒（上拉没保持住，引脚浮空被判成低）；
 *                    数十秒 = 确实睡住了，之后才被误触发
 *     · 入睡前电平 → 若入睡瞬间该脚已是低，根因直接就是上拉，而不是"睡后漂移"
 *   ⚠️ 墙钟 time() 在本工程由 RTC 计时器提供（CONFIG_ESP_TIME_FUNCS_USE_RTC_TIMER=y，
 *   见 build/config/sdkconfig.h），跨深睡连续 —— 直接用它量时长即可，不必碰私有的
 *   rtc_time_get() / esp_clk_slowclk_cal_get()。 */
RTC_DATA_ATTR static int64_t  s_ds_enter_epoch = -1; /*!< 入睡瞬间的墙钟秒 */
RTC_DATA_ATTR static uint32_t s_ds_seq         = 0;  /*!< 累计进入深睡的次序 */
RTC_DATA_ATTR static int      s_ds_pre_lvl0    = -1; /*!< 入睡瞬间 GPIO0 电平(-1=没采到) */
RTC_DATA_ATTR static int      s_ds_pre_lvl17   = -1; /*!< 入睡瞬间 GPIO17 电平(-1=没采到) */

/*!< DIP switch (BLE / OFF / WIFI)
 *   蓝牙档(网页显示 right, 即 SWITCH_1/GPIO8)只开启蓝牙以省电, 其余档位维持蓝牙 + WiFi
 *   (WiFi 档为 switch_pos 1, 中间 OFF 档为 0)
 *   DIP_SWITCH_RESTART_ON_BOUNDARY_CHANGE: 档位跨越蓝牙档/非蓝牙档边界时自动重启,
 *   使 WiFi 启停立即生效。电池供电时切换档位必经中间 OFF 档会自动断电重启,
 *   该逻辑主要针对 USB 供电场景 */
#define DIP_SWITCH_POS_CENTER                 0
#define DIP_SWITCH_POS_WIFI                   1
#define DIP_SWITCH_POS_BLE                    2
#define DIP_SWITCH_ADC_THRESHOLD              2000
#define DIP_SWITCH_READ_SAMPLE_NUM            8
#define DIP_SWITCH_RESTART_ON_BOUNDARY_CHANGE 1

/*!< 运行期档位判定的去抖次数（配合 adc_switch_task 的 1s 周期）。
 *
 *   为什么必须去抖（2026-09-13 实锤的"电池态周期性重启"真凶）：
 *   light sleep 生效后 adc_oneshot_read() 会偶发失败/给出无意义值，而原实现
 *   ① 不查返回值、② 单次样本即提交 switch_pos。读失败时 adc_value 保持上一次的
 *   值或 0，BLE 档(2) 会被判成中间档(0)，于是跨越 dip_switch_wifi_enabled 的边界
 *   → 执行 esp_restart()（rst=3 软件复位、不写 coredump）→ 表现为"每隔几分钟自己
 *   重启、闪一次开机灯效"。
 *
 *   对策：① 读取失败的样本直接丢弃；② 新档位需连续 N 次一致才提交。
 *   代价：真实拨码变化需要约 N 秒才生效（拨档本来就要跨档重启，用户可接受）。 */
#define DIP_SWITCH_DEBOUNCE_HITS              3

/*!< 跨档自证参数（见 dip_switch_verify_crossing）。
 *   SETTLE: 关掉 light sleep 后等待 ADC 恢复的时间；COOLDOWN: 判定为误读后的静默期，
 *   避免每秒都重复走一遍"关休眠 -> 复读 -> 恢复休眠"。 */
#define DIP_VERIFY_SETTLE_MS                  1200
/*!< 关闭 light sleep 后，连续复读多少轮都指向候选档位才采信（每轮内部已 8 点平均）。 */
#define DIP_VERIFY_ROUNDS                     5
/*!< 判定被否决/熔断后的静默时长：这段时间内不再做档位判定。 */
#define DIP_VERIFY_COOLDOWN_MS                30000

/*!< 是否需要启用 WiFi(蓝牙档只开蓝牙) */
static bool dip_switch_wifi_enabled(int pos)
{
    return pos != DIP_SWITCH_POS_BLE;
}

/*!< iot_button 定时器按需启停。
 *
 *   espressif__button 组件的 20ms 周期 esp_timer（CONFIG_BUTTON_PERIOD_TIME_MS）
 *   只在 driver->enable_power_save 为真时才会自动停止，而 dual_button 驱动没有
 *   实现该能力（无 enable_power_save 字段、无 get_gpio_num），定时器因此永不停止
 *   → 50Hz 常驻唤醒，会持续打断 tickless idle。这里自行跟踪状态并跟随按键启停：
 *   有键按下才运行，全部松开立即停。
 *   本地状态镜像不可省：iot_button_stop() 在定时器已停时会 ESP_LOGE 并返回
 *   ESP_ERR_INVALID_STATE，不做跟踪会刷屏日志。 */
static bool s_iot_btn_timer_running = false;

static void iot_button_timer_enable(bool enable)
{
    if (enable) {
        if (!s_iot_btn_timer_running && iot_button_resume() == ESP_OK) {
            s_iot_btn_timer_running = true;
        }
    } else if (s_iot_btn_timer_running && iot_button_stop() == ESP_OK) {
        s_iot_btn_timer_running = false;
    }
}

/*!< 阻塞读取拨码开关档位(多次采样取平均, 返回 0=中间 / 1=WiFi档 / 2=蓝牙档) */
static int dip_switch_read_position(adc_oneshot_unit_handle_t handle)
{
    int ble_value  = 0;
    int wifi_value = 0;

    for (int i = 0; i < DIP_SWITCH_READ_SAMPLE_NUM; i++) {
        int value = 0;
        if (adc_oneshot_read(handle, KBD_ADC_SWITCH_BLE_CHAN, &value) == ESP_OK) {
            ble_value += value;
        }
        value = 0;
        if (adc_oneshot_read(handle, KBD_ADC_SWITCH_RAINMAKER_CHAN, &value) == ESP_OK) {
            wifi_value += value;
        }
        vTaskDelay(pdMS_TO_TICKS(2));
    }

    ble_value /= DIP_SWITCH_READ_SAMPLE_NUM;
    wifi_value /= DIP_SWITCH_READ_SAMPLE_NUM;

    if (ble_value > DIP_SWITCH_ADC_THRESHOLD) {
        return DIP_SWITCH_POS_BLE;
    }
    if (wifi_value > DIP_SWITCH_ADC_THRESHOLD) {
        return DIP_SWITCH_POS_WIFI;
    }
    return DIP_SWITCH_POS_CENTER;
}

#define enable_web_cache 1  // 1:启用Web缓存, 60分钟不变,ctrl+F5/cmd+shift+R刷新, 0:禁用Web缓存

// 状态刷新消息类型
typedef enum {
    STATUS_REFRESH_NORMAL,    // 正常刷新
    STATUS_REFRESH_IMMEDIATE  // 立即刷新
} status_refresh_type_t;

// DualKey配置数据结构
typedef struct {
    uint32_t left_key_color;                  // 左键颜色
    uint32_t right_key_color;                 // 右键颜色
    int current_key_mapping;                  // 当前按键映射索引
    bool usb_mapping_enabled;                 // USB映射开关
    bool ble_mapping_enabled;                 // 蓝牙映射开关
    bool custom_mapping_enabled;              // 自定义映射开关
    custom_key_action_t custom_left_action;   // 左键自定义动作
    custom_key_action_t custom_right_action;  // 右键自定义动作
    uint32_t crc32;                           // CRC校验
} dualkey_saved_config_t;

// 长按配置：单独存一个 NVS key。
// 不并入 dualkey_saved_config_t，是为了避免结构扩容导致老用户配置的 blob 长度/CRC
// 校验失败而被重置为默认值。
typedef struct {
    custom_key_action_t left_long_action;   // 左键长按动作，type==CUSTOM_ACTION_NONE 表示未配置
    custom_key_action_t right_long_action;  // 右键长按动作
    uint16_t long_press_ms;                 // 长按判定阈值 (ms)
    uint16_t reserved;                      // 显式填充，保证结构布局稳定
    uint32_t crc32;                         // CRC校验
} dualkey_longpress_saved_t;

// 状态数据结构
typedef struct {
    bool left_key_pressed;
    bool right_key_pressed;
    uint32_t left_key_color;
    uint32_t right_key_color;
    float usb_voltage;
    bool usb_connected;
    int usb_mode;        // 0:HID, 1:BLE, 2:CDC
    int dip_switch_pos;  // 0:center, 1:left, 2:right
    float battery_voltage;
    int charge_status;  // 0:未充电, 1:充电中, 2:充满
    int battery_percentage;
    int switch_1_value;  // 拨码开关1值
    int switch_2_value;  // 拨码开关2值
    // 蓝牙状态
    bool bluetooth_connected;
    char bluetooth_device_name[32];
    int bluetooth_pairing_status;  // 0:未配对, 1:配对中, 2:已配对
    bool bluetooth_adv_status;     // 蓝牙广播状态: false=未广播, true=正在广播
    // HID按键映射
    int current_key_mapping;  // 当前按键映射索引
    // WIFI状态
    char wifi_ssid[32];   // WiFi网络名称
    char wifi_ip[16];     // IP地址
    int wifi_rssi;        // 信号强度
    bool wifi_connected;  // 连接状态
} device_status_t;

device_status_t g_device_status = {.left_key_pressed   = false,
                                   .right_key_pressed  = false,
                                   .left_key_color     = 0x000000,  // 红色
                                   .right_key_color    = 0x000000,  // 绿色
                                   .usb_voltage        = 0.0f,
                                   .usb_connected      = true,
                                   .usb_mode           = 0,  // HID
                                   .dip_switch_pos     = 0,  // center
                                   .battery_voltage    = 3.7f,
                                   .charge_status      = 0,
                                   .battery_percentage = 75,
                                   .switch_1_value     = 0,
                                   .switch_2_value     = 0,
                                   // 蓝牙状态初始化
                                   .bluetooth_connected      = false,
                                   .bluetooth_device_name    = USB_HID_PRODUCT,
                                   .bluetooth_pairing_status = 0,      // 未配对
                                   .bluetooth_adv_status     = false,  // 未广播
                                   // HID按键映射初始化
                                   .current_key_mapping = 9,  // 默认为翻页模式
                                   // WIFI状态初始化
                                   .wifi_ssid      = "",
                                   .wifi_ip        = "",
                                   .wifi_rssi      = 0,
                                   .wifi_connected = false};

static void wifi_start_ap_mode(void);
static httpd_handle_t start_webserver(void);

// WebSocket函数声明
static esp_err_t websocket_handler(httpd_req_t *req);
static void websocket_send_status(void);
static void websocket_task(void *pvParameters);
static void device_status_task(void *pvParameters);
static void power_indicator_task(void *pvParameters);
static void update_device_status(void);

// 自动关机（仅蓝牙档）：超时配置 + 深睡入口
static void auto_off_config_load(void);
static void auto_off_config_save(uint16_t minutes);
static void auto_off_tick(void);
static void power_enter_deep_sleep(void) __attribute__((noreturn));

// RGB颜色控制函数声明
static void set_key_rgb_color_locked(int key_index, uint32_t rgb_color);
static void apply_key_colors(uint32_t left_color, uint32_t right_color);
static void refresh_key_colors_from_status(void);

// DualKey配置保存和加载函数声明
static esp_err_t dualkey_config_save(void);
static esp_err_t dualkey_config_load(void);
static uint32_t dualkey_config_calculate_crc(const dualkey_saved_config_t *config);

// 长按配置保存和加载函数声明（独立 NVS key）
static esp_err_t dualkey_longpress_config_save(void);
static esp_err_t dualkey_longpress_config_load(void);
static uint32_t dualkey_longpress_calculate_crc(const dualkey_longpress_saved_t *config);

// WiFi配置保存和加载函数声明
static esp_err_t wifi_config_save(const wifi_config_saved_t *config);
static esp_err_t wifi_config_load(wifi_config_saved_t *config);
static esp_err_t wifi_config_reset(void);
static uint32_t wifi_config_calculate_crc(const wifi_config_saved_t *config);

// WiFi事件处理
static void wifi_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        wifi_connected = false;
        if (s_retry_num < MAXIMUM_RETRY) {
            esp_wifi_connect();
            s_retry_num++;
            ESP_LOGI(TAG, "retry to connect to the AP");
        } else {
            ESP_LOGI(TAG, "Failed to connect to WiFi, starting AP mode");
            // 停止STA模式
            esp_wifi_stop();
            // 切换到AP模式
            wifi_start_ap_mode();
        }
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
        ESP_LOGI(TAG, "got ip:" IPSTR, IP2STR(&event->ip_info.ip));
        s_retry_num    = 0;
        wifi_connected = true;
        wifi_ap_mode   = false;
        start_webserver();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_AP_STACONNECTED) {
        wifi_event_ap_staconnected_t *event = (wifi_event_ap_staconnected_t *)event_data;
        ESP_LOGI(TAG, "station " MACSTR " join, AID=%d", MAC2STR(event->mac), event->aid);
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_AP_STADISCONNECTED) {
        wifi_event_ap_stadisconnected_t *event = (wifi_event_ap_stadisconnected_t *)event_data;
        ESP_LOGI(TAG, "station " MACSTR " leave, AID=%d, reason=%d", MAC2STR(event->mac), event->aid, event->reason);
    }
}

// 启动AP模式
static void wifi_start_ap_mode(void)
{
    ESP_LOGI(TAG, "Starting AP mode");

    wifi_config_t wifi_config = {};
    uint8_t mac[6];
    esp_efuse_mac_get_default(mac);
    char ap_ssid[32] = {0};
    sprintf(ap_ssid, "DualKey_%02X%02X", mac[4], mac[5]);
    strcpy((char *)wifi_config.ap.ssid, ap_ssid);
    wifi_config.ap.ssid_len = strlen(ap_ssid);
    strcpy((char *)wifi_config.ap.password, AP_PASSWORD);
    wifi_config.ap.max_connection = 4;
    wifi_config.ap.authmode       = WIFI_AUTH_WPA_WPA2_PSK;

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    esp_wifi_set_max_tx_power(40);  // 设置最大发射功率为40dBm

    wifi_ap_mode   = true;
    wifi_connected = false;

    esp_netif_ip_info_t ip_info;
    esp_netif_get_ip_info(esp_netif_get_handle_from_ifkey("WIFI_AP_DEF"), &ip_info);

    char ip_addr[16];
    inet_ntoa_r(ip_info.ip.addr, ip_addr, 16);
    ESP_LOGI(TAG, "AP started with IP: %s", ip_addr);

    start_webserver();
}

// 初始化WiFi
static void wifi_init_sta(void)
{
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_t *sta_netif = esp_netif_create_default_wifi_sta();
    esp_netif_create_default_wifi_ap();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL, NULL));
    ESP_ERROR_CHECK(
        esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL, NULL));

    // 从flash加载WiFi配置
    wifi_config_saved_t saved_config;
    esp_err_t ret = wifi_config_load(&saved_config);

    if (ret == ESP_OK && strlen(saved_config.ssid) > 0) {
        // 有保存的配置，尝试连接
        ESP_LOGI(TAG, "Found saved WiFi config, SSID: %s", saved_config.ssid);

        // 配置静态IP（如果启用）
        if (saved_config.use_static_ip) {
            ESP_LOGI(TAG, "Using static IP: %s", saved_config.static_ip);
            esp_netif_dhcpc_stop(sta_netif);

            esp_netif_ip_info_t ip_info;
            memset(&ip_info, 0, sizeof(esp_netif_ip_info_t));
            ip_info.ip.addr      = esp_ip4addr_aton(saved_config.static_ip);
            ip_info.netmask.addr = esp_ip4addr_aton(saved_config.netmask);
            ip_info.gw.addr      = esp_ip4addr_aton(saved_config.gateway);

            esp_netif_set_ip_info(sta_netif, &ip_info);
        }

        wifi_config_t wifi_config = {};
        strcpy((char *)wifi_config.sta.ssid, saved_config.ssid);
        strcpy((char *)wifi_config.sta.password, saved_config.password);
        wifi_config.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;
        wifi_config.sta.pmf_cfg.capable    = true;
        wifi_config.sta.pmf_cfg.required   = false;

        wifi_ap_mode   = false;
        wifi_connected = false;

        ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
        ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
        ESP_ERROR_CHECK(esp_wifi_start());

        ESP_LOGI(TAG, "wifi_init_sta finished, connecting to %s", saved_config.ssid);
    } else {
        // 没有保存的配置，直接启动AP模式
        ESP_LOGI(TAG, "No saved WiFi config found, starting AP mode");
        wifi_ap_mode   = false;
        wifi_connected = false;
        ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_NULL));
        ESP_ERROR_CHECK(esp_wifi_start());
        wifi_start_ap_mode();
    }
}

// HTTP GET处理器 - 主页
static esp_err_t index_get_handler(httpd_req_t *req)
{
    const uint32_t index_len = index_html_end - index_html_start;
    ESP_LOGI(TAG, "Serve index.html");
    httpd_resp_set_type(req, "text/html; charset=utf-8");
    httpd_resp_set_hdr(req, "X-Content-Type-Options", "nosniff");
    // #if enable_web_cache
    //     httpd_resp_set_hdr(req, "Cache-Control", "public, max-age=3600, immutable");
    // #else
    httpd_resp_set_hdr(req, "Cache-Control", "no-cache");
    // #endif
    httpd_resp_send(req, index_html_start, index_len);
    return ESP_OK;
}

// HTTP GET处理器 - CSS样式
static esp_err_t styles_get_handler(httpd_req_t *req)
{
    const uint32_t styles_len = styles_css_end - styles_css_start;
    ESP_LOGI(TAG, "Serve styles.css");
    httpd_resp_set_type(req, "text/css; charset=utf-8");
    httpd_resp_set_hdr(req, "X-Content-Type-Options", "nosniff");
    // #if enable_web_cache
    //     httpd_resp_set_hdr(req, "Cache-Control", "public, max-age=3600, immutable");
    // #else
    httpd_resp_set_hdr(req, "Cache-Control", "no-cache");
    // #endif
    httpd_resp_send(req, styles_css_start, styles_len);
    return ESP_OK;
}

// HTTP GET处理器 - JavaScript脚本
static esp_err_t script_get_handler(httpd_req_t *req)
{
    const uint32_t script_len = script_js_end - script_js_start;
    ESP_LOGI(TAG, "Serve script.js");
    httpd_resp_set_type(req, "application/javascript; charset=utf-8");
    httpd_resp_set_hdr(req, "X-Content-Type-Options", "nosniff");
    // #if enable_web_cache
    //     httpd_resp_set_hdr(req, "Cache-Control", "public, max-age=3600, immutable");
    // #else
    httpd_resp_set_hdr(req, "Cache-Control", "no-cache");
    // #endif
    httpd_resp_send(req, script_js_start, script_len);
    return ESP_OK;
}

// HTTP 404错误处理器 - 重定向到主页
static esp_err_t http_404_error_handler(httpd_req_t *req, httpd_err_code_t err)
{
    httpd_resp_set_status(req, "302 Temporary Redirect");
    httpd_resp_set_hdr(req, "Location", "/");
    httpd_resp_send(req, "Redirect to the main page", HTTPD_RESP_USE_STRLEN);
    ESP_LOGI(TAG, "Redirecting to index");
    return ESP_OK;
}

// HTTP GET处理器 - Favicon
static esp_err_t favicon_get_handler(httpd_req_t *req)
{
    const uint32_t favicon_len = favicon_ico_end - favicon_ico_start;
    ESP_LOGI(TAG, "Serve favicon.ico");
    httpd_resp_set_type(req, "image/x-icon");
    httpd_resp_set_hdr(req, "X-Content-Type-Options", "nosniff");
#if enable_web_cache
    httpd_resp_set_hdr(req, "Cache-Control", "public, max-age=3600, immutable");
#else
    httpd_resp_set_hdr(req, "Cache-Control", "no-cache");
#endif
    httpd_resp_send(req, favicon_ico_start, favicon_len);
    return ESP_OK;
}

/*!< 功耗诊断读数端点。
 *   数据来源是 BLE 档运行期间由 diag_power 写进 NVS 的快照：BLE 档既没有网页、
 *   也没有可用的 USB 控制台，只能先把数据存起来，等切到 WiFi 档再从这儿读回。
 *   用 GET 而不是 WebSocket 命令，是因为它只需要一个纯文本快照，浏览器直接
 *   打开 /diag 就能看。加 ?clear=1 可在读完后清空记录，方便开始下一轮采样。 */
static esp_err_t diag_get_handler(httpd_req_t *req)
{
    httpd_resp_set_type(req, "text/plain; charset=utf-8");

    /*!< 注意 httpd_req::uri 是定长数组而非指针，不能做 NULL 判断
     *   （`req->uri != NULL` 恒为真，会被 -Werror=address 拦下）。 */
    if (strstr(req->uri, "clear=1") != NULL) {
        diag_power_clear();
        diag_log_clear();
        return httpd_resp_send(req, "diag record cleared\n", HTTPD_RESP_USE_STRLEN);
    }

    char *txt  = diag_power_load();
    char *boot = diag_boot_load();
    /*!< NVS 事件日志：唯一能扛住"OFF 档断电"的证据（RTC 内存会被清零）。 */
    char *log = diag_log_load();

    /*!< 按键时延取证（纯 RAM，只对本次运行有效）。判据见 btn_progress.h：
     *   tap 量的是两个边沿之差（测不出"整体平移"）；send 量的是把报告交给协议栈的耗时，
     *   两者合起来才能分辨"边沿晚检出"与"送达侧阻塞"。 */
    btn_latency_stats_t lat;
    btn_progress_get_latency_stats(&lat);
    char *lat_txt = (char *)malloc(640);
    if (lat_txt != NULL) {
        snprintf(lat_txt, 640,
                 "\n--- key latency (RAM, this session only) ---\n"
                 "short tap   : last %u ms   max %u ms   count %u\n"
                 "long  fire  : last %u ms   count %u\n"
                 "hold cb gap : max %u ms    (%u callbacks while key was down)\n"
                 "hid send usb: last %u ms   max %u ms   count %u\n"
                 "hid send ble: last %u ms   max %u ms   count %u\n"
                 "auto off    : %u min (0=off)   idle %u s   (BLE 档无连接判定)\n",
                 (unsigned)lat.tap_hold_ms_last, (unsigned)lat.tap_hold_ms_max, (unsigned)lat.tap_count,
                 (unsigned)lat.long_fire_ms_last, (unsigned)lat.long_count, (unsigned)lat.cb_gap_ms_max,
                 (unsigned)lat.cb_down_count, (unsigned)lat.usb_send_ms_last, (unsigned)lat.usb_send_ms_max,
                 (unsigned)lat.usb_send_cnt, (unsigned)lat.ble_send_ms_last, (unsigned)lat.ble_send_ms_max,
                 (unsigned)lat.ble_send_cnt, (unsigned)g_auto_off_min, (unsigned)s_auto_off_idle_s);
    }

    if (txt == NULL && boot == NULL && log == NULL && lat_txt == NULL) {
        return httpd_resp_send(req,
                               "no diag record.\n"
                               "Run the device in BLE position first, then flip the DIP switch to WiFi\n"
                               "position (this restarts the chip and flushes the record), and reload here.\n",
                               HTTPD_RESP_USE_STRLEN);
    }

    /*!< 三段文本用 chunked 依次送出，省掉一次拼接与额外的堆拷贝。 */
    esp_err_t err = ESP_OK;
    if (log != NULL) {
        err = httpd_resp_send_chunk(req, "=== event log (NVS, survives power cycle) ===\n", HTTPD_RESP_USE_STRLEN);
        if (err == ESP_OK) {
            err = httpd_resp_send_chunk(req, log, HTTPD_RESP_USE_STRLEN);
        }
    }
    free(log);
    if (err == ESP_OK && boot != NULL) {
        err = httpd_resp_send_chunk(req, "\n", 1);
        if (err == ESP_OK) {
            err = httpd_resp_send_chunk(req, boot, HTTPD_RESP_USE_STRLEN);
        }
        free(boot);
    }
    if (err == ESP_OK && txt != NULL) {
        err = httpd_resp_send_chunk(req, txt, HTTPD_RESP_USE_STRLEN);
    }
    free(txt);
    if (err == ESP_OK && lat_txt != NULL) {
        err = httpd_resp_send_chunk(req, lat_txt, HTTPD_RESP_USE_STRLEN);
    }
    free(lat_txt);
    if (err == ESP_OK) {
        err = httpd_resp_send_chunk(req, NULL, 0); /*!< 结束 chunked 响应 */
    }
    return err;
}

// WebSocket处理器
static esp_err_t websocket_handler(httpd_req_t *req)
{
    if (req->method == HTTP_GET) {
        ESP_LOGI(TAG, "WebSocket握手请求");
        return ESP_OK;
    }

    httpd_ws_frame_t ws_pkt;
    uint8_t *buf = NULL;
    memset(&ws_pkt, 0, sizeof(httpd_ws_frame_t));
    ws_pkt.type = HTTPD_WS_TYPE_TEXT;

    // 接收WebSocket数据
    esp_err_t ret = httpd_ws_recv_frame(req, &ws_pkt, 0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "httpd_ws_recv_frame failed to get frame len with %d", ret);
        return ret;
    }

    if (ws_pkt.len) {
        buf = (uint8_t *)malloc(ws_pkt.len + 1);
        if (buf == NULL) {
            ESP_LOGE(TAG, "Failed to allocate memory for buf");
            return ESP_ERR_NO_MEM;
        }
        ws_pkt.payload = buf;
        ret            = httpd_ws_recv_frame(req, &ws_pkt, ws_pkt.len);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "httpd_ws_recv_frame failed with %d", ret);
            free(buf);
            return ret;
        }
        buf[ws_pkt.len] = '\0';

        // 处理接收到的JSON数据
        cJSON *json = cJSON_Parse((char *)buf);
        if (json) {
            cJSON *type = cJSON_GetObjectItem(json, "type");
            if (type && cJSON_IsString(type)) {
                if (strcmp(type->valuestring, "set_rgb") == 0) {
                    // 设置RGB颜色
                    if (rgb_matrix_get_suspend_state() == false) {
                        rgb_matrix_set_suspend_state(true);
                        vTaskDelay(1 / portTICK_PERIOD_MS);
                    }
                    cJSON *key   = cJSON_GetObjectItem(json, "key");
                    cJSON *color = cJSON_GetObjectItem(json, "color");
                    if (key && color && cJSON_IsString(key) && cJSON_IsNumber(color)) {
                        uint32_t rgb_color = (uint32_t)color->valueint;
                        bool recognized    = true;
                        if (strcmp(key->valuestring, "left") == 0) {
                            g_device_status.left_key_color = rgb_color;
                            ESP_LOGI(TAG, "设置左键颜色: 0x%06lX", rgb_color);
                        } else if (strcmp(key->valuestring, "right") == 0) {
                            g_device_status.right_key_color = rgb_color;
                            ESP_LOGI(TAG, "设置右键颜色: 0x%06lX", rgb_color);
                        } else {
                            recognized = false;  // 未识别的 key 字段
                        }
                        if (recognized) {
                            // 成对原子刷新，避免单颗写入被 rgb_matrix 的整带刷新冲掉
                            refresh_key_colors_from_status();
                            // 自动保存配置
                            dualkey_config_save();
                        }
                    }
                } else if (strcmp(type->valuestring, "set_key_led_effect") == 0) {
                    // 按键灯效开关（网页文案：按键灯效）。作用对象是 project 侧的按键脉冲
                    // 灯效（key_flash_*）—— 它用 rgb_matrix 的 enable 位当开关，所以这里
                    // 仍然直接操作该位：rgb_matrix_enable()/disable() 内部会写 NVS 持久化，
                    // 而 key_led_effect_enabled() 读的就是它，两边永远一致。
                    // （原先是打字热力图 TYPING_HEATMAP，2026-09-15 已换成单次脉冲灯效。）
                    cJSON *enabled = cJSON_GetObjectItem(json, "enabled");
                    if (enabled && cJSON_IsBool(enabled)) {
                        const bool en = cJSON_IsTrue(enabled);
                        if (en) {
                            rgb_matrix_enable();
                        } else {
                            rgb_matrix_disable();
                        }
                        ESP_LOGI(TAG, "按键灯效: %s", en ? "启用" : "禁用");
                        // enable 位一变，rgb_matrix 下一次渲染会带 init 标记从而清一次整条灯带。
                        // 先等它落地，再把用户配置的静态色成对写回，否则会有一颗灯的灯被抹掉。
                        if (g_device_status.left_key_color != 0x000000 || g_device_status.right_key_color != 0x000000) {
                            vTaskDelay(80 / portTICK_PERIOD_MS);
                            refresh_key_colors_from_status();
                            vTaskDelay(20 / portTICK_PERIOD_MS);
                            refresh_key_colors_from_status();
                        }
                    }
                } else if (strcmp(type->valuestring, "set_auto_off") == 0) {
                    /*!< 自动关机超时（仅蓝牙档生效）。minutes 单位分钟，0 = 关闭，
                     *   上限 AUTO_OFF_MAX_MIN(120)。这里写 NVS 是安全的：只在用户
                     *   操作网页时发生，不是周期性路径。 */
                    cJSON *minutes_item = cJSON_GetObjectItem(json, "minutes");
                    if (minutes_item != NULL && cJSON_IsNumber(minutes_item)) {
                        int minutes = minutes_item->valueint;
                        if (minutes < 0) {
                            minutes = 0;
                        }
                        if (minutes > AUTO_OFF_MAX_MIN) {
                            minutes = AUTO_OFF_MAX_MIN;
                        }
                        auto_off_config_save((uint16_t)minutes);
                        /*!< 立刻回推一次状态，让网页那个数字框尽快拿到落定值，
                         *   不必等下一个 500ms 周期（减少"改了又跳回去"的观感）。 */
                        if (status_refresh_queue != NULL) {
                            status_refresh_type_t refresh_msg = STATUS_REFRESH_IMMEDIATE;
                            xQueueSend(status_refresh_queue, &refresh_msg, 0);
                        }
                    }
                } else if (strcmp(type->valuestring, "get_status") == 0) {
                    // 避免在 httpd 栈上构建 JSON
                    websocket_fd      = httpd_req_to_sockfd(req);
                    send_bus_all_data = true;
                    if (status_refresh_queue != NULL) {
                        status_refresh_type_t refresh_msg = STATUS_REFRESH_IMMEDIATE;
                        xQueueSend(status_refresh_queue, &refresh_msg, 0);
                    }
                } else if (strcmp(type->valuestring, "enumerate_bus") == 0) {
                    // 枚举Chain Bus设备
                    cJSON *bus = cJSON_GetObjectItem(json, "bus");
                    if (bus && cJSON_IsString(bus)) {
                        int bus_idx = (strcmp(bus->valuestring, "left") == 0) ? 0 : 1;
                        ESP_LOGI(TAG, "枚举 %s Bus 设备", bus->valuestring);
                        // 触发Chain Bus扫描
                        chain_bus_scan_single(bus_idx);
                        send_bus_all_data = true;
                    }
                } else if (strcmp(type->valuestring, "set_chain_rgb") == 0) {
                    // 设置Chain设备RGB颜色
                    cJSON *bus       = cJSON_GetObjectItem(json, "bus");
                    cJSON *device_id = cJSON_GetObjectItem(json, "device_id");
                    cJSON *color     = cJSON_GetObjectItem(json, "color");

                    if (bus && device_id && color && cJSON_IsString(bus) && cJSON_IsNumber(device_id) &&
                        cJSON_IsNumber(color)) {
                        int bus_idx        = (strcmp(bus->valuestring, "left") == 0) ? 0 : 1;
                        uint8_t dev_id     = (uint8_t)device_id->valueint;
                        uint32_t rgb_color = (uint32_t)color->valueint;

                        ESP_LOGI(TAG, "设置 %s Bus 设备 %d RGB颜色: 0x%06lX", bus->valuestring, dev_id, rgb_color);

                        // 调用Chain Bus RGB设置函数
                        esp_err_t ret = chain_bus_set_device_rgb(bus_idx, dev_id, rgb_color);
                        if (ret != ESP_OK) {
                            ESP_LOGE(TAG, "设置Chain设备RGB失败: %s", esp_err_to_name(ret));
                        }
                    }
                } else if (strcmp(type->valuestring, "set_chain_bus_rgb") == 0) {
                    // 设置Chain Bus RGB颜色
                    cJSON *bus   = cJSON_GetObjectItem(json, "bus");
                    cJSON *color = cJSON_GetObjectItem(json, "color");
                    if (bus && color && cJSON_IsString(bus) && cJSON_IsNumber(color)) {
                        int bus_idx        = (strcmp(bus->valuestring, "left") == 0) ? 0 : 1;
                        uint32_t rgb_color = (uint32_t)color->valueint;
                        for (int i = 0; i < bus_status[bus_idx].device_count; i++) {
                            if (bus_status[bus_idx].device_status[i].connected) {
                                chain_bus_set_device_rgb(bus_idx, bus_status[bus_idx].device_status[i].id, rgb_color);
                            }
                        }
                    }
                } else if (strcmp(type->valuestring, "set_hid_mapping") == 0) {
                    // 设置HID按键映射
                    cJSON *mapping_index = cJSON_GetObjectItem(json, "mapping_index");
                    if (mapping_index && cJSON_IsNumber(mapping_index)) {
                        // 更新全局状态
                        g_device_status.current_key_mapping = mapping_index->valueint;
                        // 调用按键映射设置函数
                        btn_progress_set_key_mapping(mapping_index->valueint);
                        ESP_LOGI(TAG, "设置HID按键映射为: %d", mapping_index->valueint);
                        // 自动保存配置
                        dualkey_config_save();
                    }
                } else if (strcmp(type->valuestring, "set_key_mapping_switch") == 0) {
                    // 设置按键映射开关
                    cJSON *mapping_type = cJSON_GetObjectItem(json, "mapping_type");
                    cJSON *enabled      = cJSON_GetObjectItem(json, "enabled");
                    if (mapping_type && enabled && cJSON_IsString(mapping_type) && cJSON_IsBool(enabled)) {
                        if (strcmp(mapping_type->valuestring, "usb") == 0) {
                            g_usb_mapping_enabled = cJSON_IsTrue(enabled);
                            ESP_LOGI(TAG, "设置USB映射开关为: %s", g_usb_mapping_enabled ? "启用" : "禁用");
                            // 自动保存配置
                            dualkey_config_save();
                        } else if (strcmp(mapping_type->valuestring, "ble") == 0) {
                            g_ble_mapping_enabled = cJSON_IsTrue(enabled);
                            ESP_LOGI(TAG, "设置蓝牙映射开关为: %s", g_ble_mapping_enabled ? "启用" : "禁用");
                            // 自动保存配置
                            dualkey_config_save();
                        }
                    }
                } else if (strcmp(type->valuestring, "set_device_hid_config") == 0) {
                    // 设置Chain设备HID配置
                    cJSON *bus       = cJSON_GetObjectItem(json, "bus");
                    cJSON *device_id = cJSON_GetObjectItem(json, "device_id");
                    cJSON *config    = cJSON_GetObjectItem(json, "config");

                    if (bus && device_id && config && cJSON_IsString(bus) && cJSON_IsNumber(device_id) &&
                        cJSON_IsObject(config)) {
                        int bus_idx    = (strcmp(bus->valuestring, "left") == 0) ? 0 : 1;
                        uint8_t dev_id = (uint8_t)device_id->valueint;

                        ESP_LOGI(TAG, "设置 %s Bus 设备 %d HID配置", bus->valuestring, dev_id);

                        // 查找设备
                        int device_index = -1;
                        esp_err_t ret    = chain_bus_find_device_index(bus_idx, dev_id, &device_index);
                        if (ret == ESP_OK && device_index >= 0) {
                            chain_device_status_t *dev_status    = &bus_status[bus_idx].device_status[device_index];
                            chain_device_hid_config_t hid_config = {};

                            // 根据设备类型解析配置
                            switch (dev_status->type) {
                                case CHAIN_KEY_TYPE_CODE: {
                                    cJSON *single_click  = cJSON_GetObjectItem(config, "single_click");
                                    cJSON *double_click  = cJSON_GetObjectItem(config, "double_click");
                                    cJSON *long_press    = cJSON_GetObjectItem(config, "long_press");
                                    cJSON *press_down    = cJSON_GetObjectItem(config, "press_down");
                                    cJSON *press_release = cJSON_GetObjectItem(config, "press_release");

                                    if (single_click && cJSON_IsNumber(single_click))
                                        hid_config.key_config.single_click = (hid_func_type_t)single_click->valueint;
                                    if (double_click && cJSON_IsNumber(double_click))
                                        hid_config.key_config.double_click = (hid_func_type_t)double_click->valueint;
                                    if (long_press && cJSON_IsNumber(long_press))
                                        hid_config.key_config.long_press = (hid_func_type_t)long_press->valueint;
                                    if (press_down && cJSON_IsNumber(press_down))
                                        hid_config.key_config.press_down = (hid_func_type_t)press_down->valueint;
                                    if (press_release && cJSON_IsNumber(press_release))
                                        hid_config.key_config.press_release = (hid_func_type_t)press_release->valueint;

                                    chain_bus_set_device_hid_config(bus_idx, dev_id, &hid_config);
                                    break;
                                }
                                case CHAIN_JOYSTICK_TYPE_CODE: {
                                    cJSON *single_click    = cJSON_GetObjectItem(config, "single_click");
                                    cJSON *double_click    = cJSON_GetObjectItem(config, "double_click");
                                    cJSON *long_press      = cJSON_GetObjectItem(config, "long_press");
                                    cJSON *press_down      = cJSON_GetObjectItem(config, "press_down");
                                    cJSON *press_release   = cJSON_GetObjectItem(config, "press_release");
                                    cJSON *xy_move_func    = cJSON_GetObjectItem(config, "xy_move_func");
                                    cJSON *xy_move_reverse = cJSON_GetObjectItem(config, "xy_move_reverse");

                                    if (single_click && cJSON_IsNumber(single_click))
                                        hid_config.joystick_config.single_click =
                                            (hid_func_type_t)single_click->valueint;
                                    if (double_click && cJSON_IsNumber(double_click))
                                        hid_config.joystick_config.double_click =
                                            (hid_func_type_t)double_click->valueint;
                                    if (long_press && cJSON_IsNumber(long_press))
                                        hid_config.joystick_config.long_press = (hid_func_type_t)long_press->valueint;
                                    if (press_down && cJSON_IsNumber(press_down))
                                        hid_config.joystick_config.press_down = (hid_func_type_t)press_down->valueint;
                                    if (xy_move_reverse && cJSON_IsBool(xy_move_reverse))
                                        hid_config.joystick_config.xy_move_reverse = cJSON_IsTrue(xy_move_reverse);
                                    if (press_release && cJSON_IsNumber(press_release))
                                        hid_config.joystick_config.press_release =
                                            (hid_func_type_t)press_release->valueint;
                                    if (xy_move_func && cJSON_IsNumber(xy_move_func))
                                        hid_config.joystick_config.xy_move_func =
                                            (hid_func_type_t)xy_move_func->valueint;

                                    chain_bus_set_device_hid_config(bus_idx, dev_id, &hid_config);
                                    break;
                                }
                                case CHAIN_ENCODER_TYPE_CODE: {
                                    cJSON *single_click    = cJSON_GetObjectItem(config, "single_click");
                                    cJSON *double_click    = cJSON_GetObjectItem(config, "double_click");
                                    cJSON *long_press      = cJSON_GetObjectItem(config, "long_press");
                                    cJSON *press_down      = cJSON_GetObjectItem(config, "press_down");
                                    cJSON *press_release   = cJSON_GetObjectItem(config, "press_release");
                                    cJSON *rotate_cw_func  = cJSON_GetObjectItem(config, "rotate_cw_func");
                                    cJSON *rotate_ccw_func = cJSON_GetObjectItem(config, "rotate_ccw_func");

                                    if (single_click && cJSON_IsNumber(single_click))
                                        hid_config.encoder_config.single_click =
                                            (hid_func_type_t)single_click->valueint;
                                    if (double_click && cJSON_IsNumber(double_click))
                                        hid_config.encoder_config.double_click =
                                            (hid_func_type_t)double_click->valueint;
                                    if (long_press && cJSON_IsNumber(long_press))
                                        hid_config.encoder_config.long_press = (hid_func_type_t)long_press->valueint;
                                    if (press_down && cJSON_IsNumber(press_down))
                                        hid_config.encoder_config.press_down = (hid_func_type_t)press_down->valueint;
                                    if (press_release && cJSON_IsNumber(press_release))
                                        hid_config.encoder_config.press_release =
                                            (hid_func_type_t)press_release->valueint;
                                    if (rotate_cw_func && cJSON_IsNumber(rotate_cw_func))
                                        hid_config.encoder_config.rotate_cw_func =
                                            (hid_func_type_t)rotate_cw_func->valueint;
                                    if (rotate_ccw_func && cJSON_IsNumber(rotate_ccw_func))
                                        hid_config.encoder_config.rotate_ccw_func =
                                            (hid_func_type_t)rotate_ccw_func->valueint;

                                    chain_bus_set_device_hid_config(bus_idx, dev_id, &hid_config);
                                    break;
                                }
                                case CHAIN_ANGLE_TYPE_CODE: {
                                    cJSON *angle_func = cJSON_GetObjectItem(config, "angle_func");

                                    if (angle_func && cJSON_IsNumber(angle_func))
                                        hid_config.angle_config.angle_func = (hid_func_type_t)angle_func->valueint;

                                    chain_bus_set_device_hid_config(bus_idx, dev_id, &hid_config);
                                    break;
                                }
                                case CHAIN_PEDAL_TYPE_CODE: {
                                    cJSON *single_click  = cJSON_GetObjectItem(config, "single_click");
                                    cJSON *press_down    = cJSON_GetObjectItem(config, "press_down");
                                    cJSON *press_release = cJSON_GetObjectItem(config, "press_release");
                                    if (single_click && cJSON_IsNumber(single_click))
                                        hid_config.pedal_config.single_click = (hid_func_type_t)single_click->valueint;
                                    if (press_down && cJSON_IsNumber(press_down))
                                        hid_config.pedal_config.press_down = (hid_func_type_t)press_down->valueint;
                                    if (press_release && cJSON_IsNumber(press_release))
                                        hid_config.pedal_config.press_release =
                                            (hid_func_type_t)press_release->valueint;
                                    chain_bus_set_device_hid_config(bus_idx, dev_id, &hid_config);
                                    break;
                                }
                                case CHAIN_MIC_TYPE_CODE: {
                                    cJSON *high_func = cJSON_GetObjectItem(config, "high_threshold_func");
                                    cJSON *low_func  = cJSON_GetObjectItem(config, "low_threshold_func");
                                    cJSON *threshold = cJSON_GetObjectItem(config, "threshold");
                                    cJSON *interval  = cJSON_GetObjectItem(config, "trigger_interval_ms");
                                    if (high_func && cJSON_IsNumber(high_func))
                                        hid_config.mic_config.high_threshold_func =
                                            (hid_func_type_t)high_func->valueint;
                                    if (low_func && cJSON_IsNumber(low_func))
                                        hid_config.mic_config.low_threshold_func = (hid_func_type_t)low_func->valueint;
                                    if (threshold && cJSON_IsNumber(threshold))
                                        hid_config.mic_config.threshold = (uint16_t)threshold->valueint;
                                    if (interval && cJSON_IsNumber(interval))
                                        hid_config.mic_config.trigger_interval_ms = (uint16_t)interval->valueint;
                                    chain_bus_set_device_hid_config(bus_idx, dev_id, &hid_config);
                                    break;
                                }
                                case CHAIN_SWITCH_TYPE_CODE: {
                                    cJSON *open_func  = cJSON_GetObjectItem(config, "open_func");
                                    cJSON *close_func = cJSON_GetObjectItem(config, "close_func");
                                    if (open_func && cJSON_IsNumber(open_func))
                                        hid_config.switch_config.open_func = (hid_func_type_t)open_func->valueint;
                                    if (close_func && cJSON_IsNumber(close_func))
                                        hid_config.switch_config.close_func = (hid_func_type_t)close_func->valueint;
                                    chain_bus_set_device_hid_config(bus_idx, dev_id, &hid_config);
                                    break;
                                }
                                case CHAIN_DLight_TYPE_CODE: {
                                    cJSON *high_func = cJSON_GetObjectItem(config, "lux_high_func");
                                    cJSON *low_func  = cJSON_GetObjectItem(config, "lux_low_func");
                                    cJSON *high_thr  = cJSON_GetObjectItem(config, "high_threshold");
                                    cJSON *low_thr   = cJSON_GetObjectItem(config, "low_threshold");
                                    if (high_func && cJSON_IsNumber(high_func))
                                        hid_config.dlight_config.lux_high_func = (hid_func_type_t)high_func->valueint;
                                    if (low_func && cJSON_IsNumber(low_func))
                                        hid_config.dlight_config.lux_low_func = (hid_func_type_t)low_func->valueint;
                                    if (high_thr && cJSON_IsNumber(high_thr))
                                        hid_config.dlight_config.high_threshold = (uint32_t)high_thr->valueint;
                                    if (low_thr && cJSON_IsNumber(low_thr))
                                        hid_config.dlight_config.low_threshold = (uint32_t)low_thr->valueint;
                                    chain_bus_set_device_hid_config(bus_idx, dev_id, &hid_config);
                                    break;
                                }
                                case CHAIN_PIR_TYPE_CODE: {
                                    cJSON *come_func  = cJSON_GetObjectItem(config, "person_come_func");
                                    cJSON *leave_func = cJSON_GetObjectItem(config, "person_leave_func");
                                    if (come_func && cJSON_IsNumber(come_func))
                                        hid_config.pir_config.person_come_func = (hid_func_type_t)come_func->valueint;
                                    if (leave_func && cJSON_IsNumber(leave_func))
                                        hid_config.pir_config.person_leave_func = (hid_func_type_t)leave_func->valueint;
                                    chain_bus_set_device_hid_config(bus_idx, dev_id, &hid_config);
                                    break;
                                }
                                default:
                                    ESP_LOGW(TAG, "设备类型 %s 不支持HID配置",
                                             chain_device_type_name(dev_status->type));
                                    break;
                            }

                            // 保存设备配置到NVS
                            esp_err_t save_ret = chain_bus_save_device_config(bus_idx, dev_id);
                            if (save_ret == ESP_OK) {
                                ESP_LOGI(TAG, "设备 %s Bus ID:%d 配置已保存", bus->valuestring, dev_id);
                            } else {
                                ESP_LOGE(TAG, "设备 %s Bus ID:%d 配置保存失败: %s", bus->valuestring, dev_id,
                                         esp_err_to_name(save_ret));
                            }

                            // 标记状态更新，使配置变更立即发送到前端
                            dev_status->status_updated = true;

                            // 立即触发状态刷新，确保配置更新后前端能收到最新状态
                            status_refresh_type_t refresh_msg = STATUS_REFRESH_IMMEDIATE;
                            if (status_refresh_queue != NULL) {
                                xQueueSend(status_refresh_queue, &refresh_msg, 0);
                            }
                        } else {
                            ESP_LOGE(TAG, "找不到设备 %s Bus ID: %d", bus->valuestring, dev_id);
                        }
                    }
                } else if (strcmp(type->valuestring, "reset_device_hid_config") == 0) {
                    // 恢复Chain设备HID配置为默认值
                    cJSON *bus       = cJSON_GetObjectItem(json, "bus");
                    cJSON *device_id = cJSON_GetObjectItem(json, "device_id");

                    if (bus && device_id && cJSON_IsString(bus) && cJSON_IsNumber(device_id)) {
                        int bus_idx    = (strcmp(bus->valuestring, "left") == 0) ? 0 : 1;
                        uint8_t dev_id = (uint8_t)device_id->valueint;

                        ESP_LOGI(TAG, "恢复 %s Bus 设备 %d HID配置为默认值", bus->valuestring, dev_id);

                        // 查找设备
                        int device_index = -1;
                        esp_err_t ret    = chain_bus_find_device_index(bus_idx, dev_id, &device_index);
                        if (ret == ESP_OK && device_index >= 0) {
                            chain_device_status_t *dev_status = &bus_status[bus_idx].device_status[device_index];

                            // 删除保存的配置
                            esp_err_t delete_ret = chain_bus_config_delete(dev_status->uid);
                            if (delete_ret == ESP_OK) {
                                ESP_LOGI(TAG, "设备 %s Bus ID:%d 保存的配置已删除", bus->valuestring, dev_id);
                            } else {
                                ESP_LOGW(TAG, "设备 %s Bus ID:%d 配置删除失败: %s", bus->valuestring, dev_id,
                                         esp_err_to_name(delete_ret));
                            }

                            // 重新初始化设备HID配置为默认值
                            chain_bus_init_device_hid_config(dev_status, bus_idx);

                            // 标记状态更新，使配置变更立即发送到前端
                            dev_status->status_updated = true;

                            // 立即触发状态刷新，确保配置更新后前端能收到最新状态
                            status_refresh_type_t refresh_msg = STATUS_REFRESH_IMMEDIATE;
                            if (status_refresh_queue != NULL) {
                                xQueueSend(status_refresh_queue, &refresh_msg, 0);
                            }

                            ESP_LOGI(TAG, "设备 %s Bus ID:%d HID配置已恢复为默认值", bus->valuestring, dev_id);
                        } else {
                            ESP_LOGE(TAG, "找不到设备 %s Bus ID: %d", bus->valuestring, dev_id);
                        }
                    }
                } else if (strcmp(type->valuestring, "buzzer_play") == 0) {
                    cJSON *bus       = cJSON_GetObjectItem(json, "bus");
                    cJSON *device_id = cJSON_GetObjectItem(json, "device_id");
                    cJSON *note      = cJSON_GetObjectItem(json, "note");
                    cJSON *freq      = cJSON_GetObjectItem(json, "freq");
                    cJSON *duty      = cJSON_GetObjectItem(json, "duty");
                    cJSON *duration  = cJSON_GetObjectItem(json, "duration");
                    if (bus && device_id && cJSON_IsString(bus) && cJSON_IsNumber(device_id)) {
                        int bus_idx    = (strcmp(bus->valuestring, "left") == 0) ? 0 : 1;
                        uint8_t dev_id = (uint8_t)device_id->valueint;
                        uint16_t dur   = (duration && cJSON_IsNumber(duration)) ? (uint16_t)duration->valueint : 500;
                        esp_err_t ret  = ESP_FAIL;
                        if (note && cJSON_IsNumber(note)) {
                            ret = chain_bus_buzzer_note_play(bus_idx, dev_id, (uint8_t)note->valueint, dur);
                        } else if (freq && cJSON_IsNumber(freq)) {
                            uint8_t d = (duty && cJSON_IsNumber(duty)) ? (uint8_t)duty->valueint : 50;
                            ret       = chain_bus_buzzer_auto_play(bus_idx, dev_id, (uint16_t)freq->valueint, d, dur);
                        }
                        if (ret != ESP_OK) {
                            ESP_LOGE(TAG, "buzzer_play failed");
                        }
                    }
                } else if (strcmp(type->valuestring, "buzzer_seq_play") == 0) {
                    cJSON *bus       = cJSON_GetObjectItem(json, "bus");
                    cJSON *device_id = cJSON_GetObjectItem(json, "device_id");
                    cJSON *steps     = cJSON_GetObjectItem(json, "steps");
                    cJSON *gap       = cJSON_GetObjectItem(json, "gap_ms");
                    cJSON *loop      = cJSON_GetObjectItem(json, "loop");
                    cJSON *preset    = cJSON_GetObjectItem(json, "preset");
                    if (bus && device_id && cJSON_IsString(bus) && cJSON_IsNumber(device_id)) {
                        int bus_idx           = (strcmp(bus->valuestring, "left") == 0) ? 0 : 1;
                        uint8_t dev_id        = (uint8_t)device_id->valueint;
                        buzzer_sequence_t seq = {};
                        if (preset && cJSON_IsNumber(preset)) {
                            const buzzer_sequence_t *p = chain_bus_buzzer_seq_get_preset((uint8_t)preset->valueint);
                            if (p) {
                                memcpy(&seq, p, sizeof(seq));
                                if (loop) {
                                    seq.loop = cJSON_IsTrue(loop);
                                }
                            }
                        } else if (steps && cJSON_IsArray(steps)) {
                            int count = cJSON_GetArraySize(steps);
                            if (count > BUZZER_SEQ_MAX_STEPS) {
                                count = BUZZER_SEQ_MAX_STEPS;
                            }
                            for (int i = 0; i < count; i++) {
                                cJSON *step = cJSON_GetArrayItem(steps, i);
                                cJSON *note = cJSON_GetObjectItem(step, "note");
                                cJSON *dur  = cJSON_GetObjectItem(step, "duration_ms");
                                if (note && cJSON_IsNumber(note)) {
                                    seq.steps[i].note = (uint8_t)note->valueint;
                                }
                                if (dur && cJSON_IsNumber(dur)) {
                                    seq.steps[i].duration_ms = (uint16_t)dur->valueint;
                                }
                            }
                            seq.step_count = (uint8_t)count;
                            seq.gap_ms     = (gap && cJSON_IsNumber(gap)) ? (uint16_t)gap->valueint : 30;
                            seq.loop       = !loop || cJSON_IsTrue(loop);
                        }
                        chain_bus_buzzer_seq_play(bus_idx, dev_id, &seq);
                    }
                } else if (strcmp(type->valuestring, "buzzer_seq_stop") == 0) {
                    chain_bus_buzzer_seq_stop();
                } else if (strcmp(type->valuestring, "buzzer_stop") == 0) {
                    cJSON *bus       = cJSON_GetObjectItem(json, "bus");
                    cJSON *device_id = cJSON_GetObjectItem(json, "device_id");
                    if (bus && device_id && cJSON_IsString(bus) && cJSON_IsNumber(device_id)) {
                        int bus_idx = (strcmp(bus->valuestring, "left") == 0) ? 0 : 1;
                        chain_bus_buzzer_stop(bus_idx, (uint8_t)device_id->valueint);
                    }
                } else if (strcmp(type->valuestring, "set_mic_config") == 0) {
                    cJSON *bus       = cJSON_GetObjectItem(json, "bus");
                    cJSON *device_id = cJSON_GetObjectItem(json, "device_id");
                    cJSON *threshold = cJSON_GetObjectItem(json, "threshold");
                    cJSON *interval  = cJSON_GetObjectItem(json, "trigger_interval_ms");
                    if (bus && device_id && cJSON_IsString(bus) && cJSON_IsNumber(device_id)) {
                        int bus_idx  = (strcmp(bus->valuestring, "left") == 0) ? 0 : 1;
                        uint16_t thr = (threshold && cJSON_IsNumber(threshold)) ? (uint16_t)threshold->valueint : 2000;
                        uint16_t iv  = (interval && cJSON_IsNumber(interval)) ? (uint16_t)interval->valueint : 500;
                        chain_bus_set_mic_params(bus_idx, (uint8_t)device_id->valueint, thr, iv);
                        chain_bus_save_device_config(bus_idx, (uint8_t)device_id->valueint);
                    }
                } else if (strcmp(type->valuestring, "set_dlight_config") == 0) {
                    cJSON *bus       = cJSON_GetObjectItem(json, "bus");
                    cJSON *device_id = cJSON_GetObjectItem(json, "device_id");
                    cJSON *high_thr  = cJSON_GetObjectItem(json, "high_threshold");
                    cJSON *low_thr   = cJSON_GetObjectItem(json, "low_threshold");
                    if (bus && device_id && cJSON_IsString(bus) && cJSON_IsNumber(device_id)) {
                        int bus_idx   = (strcmp(bus->valuestring, "left") == 0) ? 0 : 1;
                        uint32_t high = (high_thr && cJSON_IsNumber(high_thr)) ? (uint32_t)high_thr->valueint : 500;
                        uint32_t low  = (low_thr && cJSON_IsNumber(low_thr)) ? (uint32_t)low_thr->valueint : 100;
                        chain_bus_set_dlight_params(bus_idx, (uint8_t)device_id->valueint, high, low);
                        chain_bus_save_device_config(bus_idx, (uint8_t)device_id->valueint);
                    }
                } else if (strcmp(type->valuestring, "mono_scroll_all") == 0) {
                    cJSON *bus      = cJSON_GetObjectItem(json, "bus");
                    cJSON *text     = cJSON_GetObjectItem(json, "text");
                    cJSON *dir      = cJSON_GetObjectItem(json, "dir");
                    cJSON *mode     = cJSON_GetObjectItem(json, "mode");
                    cJSON *interval = cJSON_GetObjectItem(json, "interval");
                    if (bus && text && cJSON_IsString(bus) && cJSON_IsString(text)) {
                        int bus_idx = (strcmp(bus->valuestring, "left") == 0) ? 0 : 1;
                        uint8_t d   = (dir && cJSON_IsNumber(dir)) ? (uint8_t)dir->valueint : 0;
                        uint8_t m   = (mode && cJSON_IsNumber(mode)) ? (uint8_t)mode->valueint : 1;
                        uint16_t iv = (interval && cJSON_IsNumber(interval)) ? (uint16_t)interval->valueint : 200;
                        chain_bus_mono_scroll_all(bus_idx, text->valuestring, d, m, iv);
                    }
                } else if (strcmp(type->valuestring, "mono_draw") == 0 ||
                           strcmp(type->valuestring, "mono_clear") == 0 ||
                           strcmp(type->valuestring, "mono_scroll") == 0) {
                    cJSON *bus       = cJSON_GetObjectItem(json, "bus");
                    cJSON *device_id = cJSON_GetObjectItem(json, "device_id");
                    if (bus && device_id && cJSON_IsString(bus) && cJSON_IsNumber(device_id)) {
                        int bus_idx    = (strcmp(bus->valuestring, "left") == 0) ? 0 : 1;
                        uint8_t dev_id = (uint8_t)device_id->valueint;
                        if (strcmp(type->valuestring, "mono_clear") == 0) {
                            chain_bus_mono_clear(bus_idx, dev_id);
                        } else if (strcmp(type->valuestring, "mono_draw") == 0) {
                            cJSON *x     = cJSON_GetObjectItem(json, "x");
                            cJSON *y     = cJSON_GetObjectItem(json, "y");
                            cJSON *state = cJSON_GetObjectItem(json, "state");
                            if (x && y && state && cJSON_IsNumber(x) && cJSON_IsNumber(y) && cJSON_IsBool(state)) {
                                chain_bus_mono_set_pixel(bus_idx, dev_id, (uint8_t)x->valueint, (uint8_t)y->valueint,
                                                         cJSON_IsTrue(state));
                            }
                        } else {
                            cJSON *text     = cJSON_GetObjectItem(json, "text");
                            cJSON *dir      = cJSON_GetObjectItem(json, "dir");
                            cJSON *mode     = cJSON_GetObjectItem(json, "mode");
                            cJSON *interval = cJSON_GetObjectItem(json, "interval");
                            if (text && cJSON_IsString(text)) {
                                uint8_t d = (dir && cJSON_IsNumber(dir)) ? (uint8_t)dir->valueint : 0;
                                uint8_t m = (mode && cJSON_IsNumber(mode)) ? (uint8_t)mode->valueint : 1;
                                uint16_t iv =
                                    (interval && cJSON_IsNumber(interval)) ? (uint16_t)interval->valueint : 200;
                                chain_bus_mono_scroll(bus_idx, dev_id, text->valuestring, d, m, iv);
                            }
                        }
                    }
                } else if (strcmp(type->valuestring, "rgb_scroll_all") == 0) {
                    cJSON *bus      = cJSON_GetObjectItem(json, "bus");
                    cJSON *text     = cJSON_GetObjectItem(json, "text");
                    cJSON *dir      = cJSON_GetObjectItem(json, "dir");
                    cJSON *mode     = cJSON_GetObjectItem(json, "mode");
                    cJSON *interval = cJSON_GetObjectItem(json, "interval");
                    cJSON *color    = cJSON_GetObjectItem(json, "color");
                    if (bus && text && cJSON_IsString(bus) && cJSON_IsString(text)) {
                        int bus_idx = (strcmp(bus->valuestring, "left") == 0) ? 0 : 1;
                        uint8_t d   = (dir && cJSON_IsNumber(dir)) ? (uint8_t)dir->valueint : 0;
                        uint8_t m   = (mode && cJSON_IsNumber(mode)) ? (uint8_t)mode->valueint : 1;
                        uint16_t iv = (interval && cJSON_IsNumber(interval)) ? (uint16_t)interval->valueint : 200;
                        uint16_t c  = (color && cJSON_IsNumber(color)) ? (uint16_t)color->valueint : 0xF800;
                        chain_bus_rgb_scroll_all(bus_idx, text->valuestring, d, m, iv, c);
                    }
                } else if (strcmp(type->valuestring, "rgb_draw") == 0 || strcmp(type->valuestring, "rgb_clear") == 0 ||
                           strcmp(type->valuestring, "rgb_scroll") == 0) {
                    cJSON *bus       = cJSON_GetObjectItem(json, "bus");
                    cJSON *device_id = cJSON_GetObjectItem(json, "device_id");
                    if (bus && device_id && cJSON_IsString(bus) && cJSON_IsNumber(device_id)) {
                        int bus_idx    = (strcmp(bus->valuestring, "left") == 0) ? 0 : 1;
                        uint8_t dev_id = (uint8_t)device_id->valueint;
                        if (strcmp(type->valuestring, "rgb_clear") == 0) {
                            chain_bus_rgb_clear(bus_idx, dev_id);
                        } else if (strcmp(type->valuestring, "rgb_draw") == 0) {
                            cJSON *x     = cJSON_GetObjectItem(json, "x");
                            cJSON *y     = cJSON_GetObjectItem(json, "y");
                            cJSON *color = cJSON_GetObjectItem(json, "color");
                            if (x && y && color && cJSON_IsNumber(x) && cJSON_IsNumber(y) && cJSON_IsNumber(color)) {
                                chain_bus_rgb_set_pixel(bus_idx, dev_id, (uint8_t)x->valueint, (uint8_t)y->valueint,
                                                        (uint16_t)color->valueint);
                            }
                        } else {
                            cJSON *text     = cJSON_GetObjectItem(json, "text");
                            cJSON *dir      = cJSON_GetObjectItem(json, "dir");
                            cJSON *mode     = cJSON_GetObjectItem(json, "mode");
                            cJSON *interval = cJSON_GetObjectItem(json, "interval");
                            cJSON *color    = cJSON_GetObjectItem(json, "color");
                            if (text && cJSON_IsString(text)) {
                                uint8_t d = (dir && cJSON_IsNumber(dir)) ? (uint8_t)dir->valueint : 0;
                                uint8_t m = (mode && cJSON_IsNumber(mode)) ? (uint8_t)mode->valueint : 1;
                                uint16_t iv =
                                    (interval && cJSON_IsNumber(interval)) ? (uint16_t)interval->valueint : 200;
                                uint16_t c = (color && cJSON_IsNumber(color)) ? (uint16_t)color->valueint : 0xF800;
                                chain_bus_rgb_scroll(bus_idx, dev_id, text->valuestring, d, m, iv, c);
                            }
                        }
                    }
                } else if (strcmp(type->valuestring, "servo_set_angle") == 0) {
                    cJSON *bus       = cJSON_GetObjectItem(json, "bus");
                    cJSON *device_id = cJSON_GetObjectItem(json, "device_id");
                    cJSON *gpio      = cJSON_GetObjectItem(json, "gpio");
                    cJSON *angle     = cJSON_GetObjectItem(json, "angle");
                    if (bus && device_id && gpio && angle && cJSON_IsString(bus) && cJSON_IsNumber(device_id) &&
                        cJSON_IsNumber(gpio) && cJSON_IsNumber(angle)) {
                        int bus_idx = (strcmp(bus->valuestring, "left") == 0) ? 0 : 1;
                        chain_bus_servo_set_angle(bus_idx, (uint8_t)device_id->valueint, (uint8_t)gpio->valueint,
                                                  (uint8_t)angle->valueint);
                    }
                } else if (strcmp(type->valuestring, "unitbus_i2c_scan") == 0) {
                    cJSON *bus       = cJSON_GetObjectItem(json, "bus");
                    cJSON *device_id = cJSON_GetObjectItem(json, "device_id");
                    if (bus && device_id && cJSON_IsString(bus) && cJSON_IsNumber(device_id)) {
                        int bus_idx = (strcmp(bus->valuestring, "left") == 0) ? 0 : 1;
                        chain_bus_unitbus_i2c_scan(bus_idx, (uint8_t)device_id->valueint);
                        send_bus_all_data = true;
                    }
                } else if (strcmp(type->valuestring, "bluetooth_start_adv") == 0) {
                    // 开始蓝牙广播
                    ESP_LOGI(TAG, "启动蓝牙广播");
                    esp_err_t ret = ble_hid_adv_start();
                    if (ret == ESP_OK) {
                        g_ble_adv_status = true;
                        ESP_LOGI(TAG, "蓝牙广播已启动");
                    } else {
                        ESP_LOGE(TAG, "蓝牙广播启动失败: %s", esp_err_to_name(ret));
                    }

                } else if (strcmp(type->valuestring, "bluetooth_disconnect") == 0) {
                    // 断开蓝牙连接
                    ESP_LOGI(TAG, "断开蓝牙连接");
                    esp_err_t ret = ble_hid_adv_stop();
                    if (ret == ESP_OK) {
                        g_ble_adv_status = false;

                        ESP_LOGI(TAG, "蓝牙连接已断开");
                    } else {
                        ESP_LOGE(TAG, "断开蓝牙连接失败: %s", esp_err_to_name(ret));
                    }

                } else if (strcmp(type->valuestring, "bluetooth_start_pairing") == 0) {
                    // 开始蓝牙配对 TODO
                    ESP_LOGI(TAG, "开始蓝牙配对");

                } else if (strcmp(type->valuestring, "save_dualkey_config") == 0) {
                    // 保存DualKey配置
                    ESP_LOGI(TAG, "保存DualKey配置");
                    esp_err_t ret = dualkey_config_save();
                    if (ret == ESP_OK) {
                        ESP_LOGI(TAG, "DualKey配置保存成功");
                    } else {
                        ESP_LOGE(TAG, "DualKey配置保存失败: %s", esp_err_to_name(ret));
                    }

                } else if (strcmp(type->valuestring, "load_dualkey_config") == 0) {
                    // 加载DualKey配置
                    ESP_LOGI(TAG, "加载DualKey配置");
                    esp_err_t ret = dualkey_config_load();
                    if (ret == ESP_OK) {
                        ESP_LOGI(TAG, "DualKey配置加载成功");
                        send_bus_all_data = true;
                        if (status_refresh_queue != NULL) {
                            status_refresh_type_t refresh_msg = STATUS_REFRESH_IMMEDIATE;
                            xQueueSend(status_refresh_queue, &refresh_msg, 0);
                        }
                    } else if (ret == ESP_ERR_NOT_FOUND) {
                        ESP_LOGI(TAG, "DualKey配置未找到，使用默认配置");
                    } else {
                        ESP_LOGE(TAG, "DualKey配置加载失败: %s", esp_err_to_name(ret));
                    }

                } else if (strcmp(type->valuestring, "set_wifi_config") == 0) {
                    // 设置WiFi配置
                    cJSON *config = cJSON_GetObjectItem(json, "config");
                    if (config && cJSON_IsObject(config)) {
                        wifi_config_saved_t wifi_cfg;
                        memset(&wifi_cfg, 0, sizeof(wifi_cfg));

                        cJSON *ssid          = cJSON_GetObjectItem(config, "ssid");
                        cJSON *password      = cJSON_GetObjectItem(config, "password");
                        cJSON *use_static_ip = cJSON_GetObjectItem(config, "use_static_ip");

                        if (ssid && password && cJSON_IsString(ssid) && cJSON_IsString(password)) {
                            strncpy(wifi_cfg.ssid, ssid->valuestring, sizeof(wifi_cfg.ssid) - 1);
                            strncpy(wifi_cfg.password, password->valuestring, sizeof(wifi_cfg.password) - 1);
                            wifi_cfg.use_static_ip = (use_static_ip && cJSON_IsTrue(use_static_ip));

                            if (wifi_cfg.use_static_ip) {
                                cJSON *static_ip = cJSON_GetObjectItem(config, "static_ip");
                                cJSON *netmask   = cJSON_GetObjectItem(config, "netmask");
                                cJSON *gateway   = cJSON_GetObjectItem(config, "gateway");

                                if (static_ip && netmask && gateway && cJSON_IsString(static_ip) &&
                                    cJSON_IsString(netmask) && cJSON_IsString(gateway)) {
                                    strncpy(wifi_cfg.static_ip, static_ip->valuestring, sizeof(wifi_cfg.static_ip) - 1);
                                    strncpy(wifi_cfg.netmask, netmask->valuestring, sizeof(wifi_cfg.netmask) - 1);
                                    strncpy(wifi_cfg.gateway, gateway->valuestring, sizeof(wifi_cfg.gateway) - 1);
                                }
                            }

                            // 保存配置
                            esp_err_t save_ret = wifi_config_save(&wifi_cfg);
                            if (save_ret == ESP_OK) {
                                ESP_LOGI(TAG, "WiFi配置已保存，准备重启...");
                                // 延迟2秒后重启
                                vTaskDelay(pdMS_TO_TICKS(2000));
                                esp_restart();
                            } else {
                                ESP_LOGE(TAG, "WiFi配置保存失败: %s", esp_err_to_name(save_ret));
                            }
                        }
                    }
                } else if (strcmp(type->valuestring, "set_custom_mapping") == 0) {
                    // 设置自定义按键映射
                    cJSON *enabled_json = cJSON_GetObjectItem(json, "enabled");
                    cJSON *left_key     = cJSON_GetObjectItem(json, "left_key");
                    cJSON *right_key    = cJSON_GetObjectItem(json, "right_key");
                    cJSON *long_left    = cJSON_GetObjectItem(json, "long_left");
                    cJSON *long_right   = cJSON_GetObjectItem(json, "long_right");
                    cJSON *long_ms      = cJSON_GetObjectItem(json, "long_press_ms");

                    if (enabled_json && cJSON_IsBool(enabled_json)) {
                        bool custom_en = cJSON_IsTrue(enabled_json);
                        btn_progress_enable_custom_mapping(custom_en);
                        ESP_LOGI(TAG, "自定义映射: %s", custom_en ? "启用" : "禁用");
                    }

                    auto parse_key_action = [](cJSON *key_obj, custom_key_action_t *out) {
                        if (!key_obj || !cJSON_IsObject(key_obj)) return;
                        memset(out, 0, sizeof(custom_key_action_t));
                        cJSON *action_type = cJSON_GetObjectItem(key_obj, "action_type");
                        cJSON *modifier    = cJSON_GetObjectItem(key_obj, "modifier");
                        cJSON *keycode     = cJSON_GetObjectItem(key_obj, "keycode");
                        cJSON *text        = cJSON_GetObjectItem(key_obj, "text");
                        if (action_type && cJSON_IsNumber(action_type))
                            out->type = (custom_action_type_t)(int)action_type->valueint;
                        if (modifier && cJSON_IsNumber(modifier)) out->modifier = (uint8_t)modifier->valueint;
                        if (keycode && cJSON_IsNumber(keycode)) out->keycode = (uint8_t)keycode->valueint;
                        if (text && cJSON_IsString(text))
                            strncpy(out->text, text->valuestring, CUSTOM_TEXT_MAX_LEN - 1);
                    };

                    // 长按动作：缺省时默认为"未配置"(CUSTOM_ACTION_NONE)，
                    // 网页用 action_type=2 显式取消某个键的长按
                    auto parse_long_key_action = [](cJSON *key_obj, custom_key_action_t *out) {
                        if (!key_obj || !cJSON_IsObject(key_obj)) return;
                        memset(out, 0, sizeof(custom_key_action_t));
                        out->type          = CUSTOM_ACTION_NONE;
                        cJSON *action_type = cJSON_GetObjectItem(key_obj, "action_type");
                        cJSON *modifier    = cJSON_GetObjectItem(key_obj, "modifier");
                        cJSON *keycode     = cJSON_GetObjectItem(key_obj, "keycode");
                        cJSON *text        = cJSON_GetObjectItem(key_obj, "text");
                        if (action_type && cJSON_IsNumber(action_type)) {
                            int t = (int)action_type->valueint;
                            out->type =
                                (t >= 0 && t <= (int)CUSTOM_ACTION_NONE) ? (custom_action_type_t)t : CUSTOM_ACTION_NONE;
                        }
                        if (modifier && cJSON_IsNumber(modifier)) out->modifier = (uint8_t)modifier->valueint;
                        if (keycode && cJSON_IsNumber(keycode)) out->keycode = (uint8_t)keycode->valueint;
                        if (text && cJSON_IsString(text))
                            strncpy(out->text, text->valuestring, CUSTOM_TEXT_MAX_LEN - 1);
                    };

                    if (left_key) {
                        custom_key_action_t action = {};
                        parse_key_action(left_key, &action);
                        btn_progress_set_custom_left_action(&action);
                    }
                    if (right_key) {
                        custom_key_action_t action = {};
                        parse_key_action(right_key, &action);
                        btn_progress_set_custom_right_action(&action);
                    }
                    // 长按字段缺省表示"保持不变"，避免只切映射开关时清掉已配好的长按
                    if (long_left) {
                        custom_key_action_t action = {};
                        parse_long_key_action(long_left, &action);
                        btn_progress_set_custom_long_left_action(&action);
                    }
                    if (long_right) {
                        custom_key_action_t action = {};
                        parse_long_key_action(long_right, &action);
                        btn_progress_set_custom_long_right_action(&action);
                    }
                    if (long_ms && cJSON_IsNumber(long_ms)) {
                        btn_progress_set_long_press_ms((uint16_t)long_ms->valueint);
                    }

                    // 自动保存配置
                    dualkey_config_save();

                } else if (strcmp(type->valuestring, "reset_wifi_config") == 0) {
                    // 重置WiFi配置
                    ESP_LOGI(TAG, "重置WiFi配置");
                    esp_err_t reset_ret = wifi_config_reset();
                    if (reset_ret == ESP_OK) {
                        ESP_LOGI(TAG, "WiFi配置已重置，准备重启...");
                        // 延迟2秒后重启
                        vTaskDelay(pdMS_TO_TICKS(2000));
                        esp_restart();
                    } else {
                        ESP_LOGE(TAG, "WiFi配置重置失败: %s", esp_err_to_name(reset_ret));
                    }
                }
            }
            cJSON_Delete(json);
        }

        free(buf);
    }

    websocket_fd = httpd_req_to_sockfd(req);
    return ESP_OK;
}

// 发送状态数据到WebSocket客户端
static void websocket_send_status(void)
{
    if (websocket_fd < 0 || server == NULL) {
        return;
    }

    cJSON *json      = cJSON_CreateObject();
    cJSON *dualkey   = cJSON_CreateObject();
    cJSON *chainbus  = cJSON_CreateObject();
    cJSON *left_bus  = cJSON_CreateObject();
    cJSON *right_bus = cJSON_CreateObject();

    // DualKey状态
    cJSON_AddBoolToObject(dualkey, "left_key_pressed", g_device_status.left_key_pressed);
    cJSON_AddBoolToObject(dualkey, "right_key_pressed", g_device_status.right_key_pressed);
    cJSON_AddNumberToObject(dualkey, "left_key_color", g_device_status.left_key_color);
    cJSON_AddNumberToObject(dualkey, "right_key_color", g_device_status.right_key_color);
    cJSON_AddNumberToObject(dualkey, "usb_voltage", g_device_status.usb_voltage);
    cJSON_AddBoolToObject(dualkey, "usb_connected", g_device_status.usb_connected);
    cJSON_AddNumberToObject(dualkey, "usb_mode", g_device_status.usb_mode);
    cJSON_AddNumberToObject(dualkey, "dip_switch_pos", g_device_status.dip_switch_pos);
    cJSON_AddNumberToObject(dualkey, "battery_voltage", g_device_status.battery_voltage);
    cJSON_AddNumberToObject(dualkey, "charge_status", g_device_status.charge_status);
    cJSON_AddNumberToObject(dualkey, "battery_percentage", g_device_status.battery_percentage);
    cJSON_AddNumberToObject(dualkey, "switch_1_value", g_device_status.switch_1_value);
    cJSON_AddNumberToObject(dualkey, "switch_2_value", g_device_status.switch_2_value);
    // 蓝牙状态
    cJSON_AddBoolToObject(dualkey, "bluetooth_connected", g_device_status.bluetooth_connected);
    cJSON_AddStringToObject(dualkey, "bluetooth_device_name", g_device_status.bluetooth_device_name);
    cJSON_AddNumberToObject(dualkey, "bluetooth_pairing_status", g_device_status.bluetooth_pairing_status);
    cJSON_AddBoolToObject(dualkey, "bluetooth_adv_status", g_device_status.bluetooth_adv_status);

    // HID按键映射状态
    cJSON_AddNumberToObject(dualkey, "current_key_mapping", g_device_status.current_key_mapping);
    cJSON_AddBoolToObject(dualkey, "usb_mapping_enabled", g_usb_mapping_enabled);
    cJSON_AddBoolToObject(dualkey, "ble_mapping_enabled", g_ble_mapping_enabled);
    // 按键灯效开关（读 rgb_matrix 的 enable 位；该值本身已由 rgb_matrix 持久化在 NVS）
    cJSON_AddBoolToObject(dualkey, "key_led_effect_enabled", rgb_matrix_is_enabled());
    // 自动关机超时（分钟，0 = 关闭）；只在蓝牙档真正生效
    cJSON_AddNumberToObject(dualkey, "auto_off_min", g_auto_off_min);

    // 自定义映射状态
    cJSON_AddBoolToObject(dualkey, "custom_mapping_enabled", btn_progress_is_custom_mapping_enabled());
    const custom_key_action_t *left_act  = btn_progress_get_custom_left_action();
    const custom_key_action_t *right_act = btn_progress_get_custom_right_action();
    cJSON *custom_left_json              = cJSON_CreateObject();
    cJSON *custom_right_json             = cJSON_CreateObject();
    cJSON_AddNumberToObject(custom_left_json, "action_type", left_act->type);
    cJSON_AddNumberToObject(custom_left_json, "modifier", left_act->modifier);
    cJSON_AddNumberToObject(custom_left_json, "keycode", left_act->keycode);
    cJSON_AddStringToObject(custom_left_json, "text", left_act->text);
    cJSON_AddNumberToObject(custom_right_json, "action_type", right_act->type);
    cJSON_AddNumberToObject(custom_right_json, "modifier", right_act->modifier);
    cJSON_AddNumberToObject(custom_right_json, "keycode", right_act->keycode);
    cJSON_AddStringToObject(custom_right_json, "text", right_act->text);
    cJSON_AddItemToObject(dualkey, "custom_left_action", custom_left_json);
    cJSON_AddItemToObject(dualkey, "custom_right_action", custom_right_json);

    // 长按配置（type==CUSTOM_ACTION_NONE 表示该键未配置长按）
    const custom_key_action_t *left_long_act  = btn_progress_get_custom_long_left_action();
    const custom_key_action_t *right_long_act = btn_progress_get_custom_long_right_action();
    cJSON *custom_left_long_json              = cJSON_CreateObject();
    cJSON *custom_right_long_json             = cJSON_CreateObject();
    cJSON_AddNumberToObject(custom_left_long_json, "action_type", left_long_act->type);
    cJSON_AddNumberToObject(custom_left_long_json, "modifier", left_long_act->modifier);
    cJSON_AddNumberToObject(custom_left_long_json, "keycode", left_long_act->keycode);
    cJSON_AddStringToObject(custom_left_long_json, "text", left_long_act->text);
    cJSON_AddNumberToObject(custom_right_long_json, "action_type", right_long_act->type);
    cJSON_AddNumberToObject(custom_right_long_json, "modifier", right_long_act->modifier);
    cJSON_AddNumberToObject(custom_right_long_json, "keycode", right_long_act->keycode);
    cJSON_AddStringToObject(custom_right_long_json, "text", right_long_act->text);
    cJSON_AddItemToObject(dualkey, "custom_left_long_action", custom_left_long_json);
    cJSON_AddItemToObject(dualkey, "custom_right_long_action", custom_right_long_json);
    cJSON_AddNumberToObject(dualkey, "long_press_ms", btn_progress_get_long_press_ms());

    // WIFI状态
    cJSON_AddStringToObject(dualkey, "wifi_ssid", g_device_status.wifi_ssid);
    cJSON_AddStringToObject(dualkey, "wifi_ip", g_device_status.wifi_ip);
    cJSON_AddNumberToObject(dualkey, "wifi_rssi", g_device_status.wifi_rssi);
    cJSON_AddBoolToObject(dualkey, "wifi_connected", g_device_status.wifi_connected);

    // Chain Bus状态 - 只发送有更新的信息
    bool left_connected  = bus_status[0].initialized && (bus_status[0].device_count > 0);
    bool right_connected = bus_status[1].initialized && (bus_status[1].device_count > 0);

    cJSON_AddBoolToObject(left_bus, "connected", left_connected);
    cJSON_AddNumberToObject(left_bus, "device_count", bus_status[0].device_count);
    cJSON_AddBoolToObject(right_bus, "connected", right_connected);
    cJSON_AddNumberToObject(right_bus, "device_count", bus_status[1].device_count);

    // 添加当前连接设备的ID列表（用于前端判断设备离线）
    cJSON *left_device_ids = cJSON_CreateArray();
    for (int i = 0; i < bus_status[0].device_count; i++) {
        if (bus_status[0].device_status[i].connected) {
            cJSON_AddItemToArray(left_device_ids, cJSON_CreateNumber(bus_status[0].device_status[i].id));
        }
    }
    cJSON_AddItemToObject(left_bus, "connected_device_ids", left_device_ids);

    cJSON *right_device_ids = cJSON_CreateArray();
    for (int i = 0; i < bus_status[1].device_count; i++) {
        if (bus_status[1].device_status[i].connected) {
            cJSON_AddItemToArray(right_device_ids, cJSON_CreateNumber(bus_status[1].device_status[i].id));
        }
    }
    cJSON_AddItemToObject(right_bus, "connected_device_ids", right_device_ids);

    // 添加设备信息 - 只发送有更新的设备
    cJSON *left_devices  = cJSON_CreateArray();
    cJSON *right_devices = cJSON_CreateArray();

    // 处理Bus设备（左Bus和右Bus）
    for (int bus_idx = 0; bus_idx < 2; bus_idx++) {
        cJSON *devices_array = (bus_idx == 0) ? left_devices : right_devices;

        for (int i = 0; i < bus_status[bus_idx].device_count; i++) {
            if (bus_status[bus_idx].device_status[i].status_updated || send_bus_all_data ||
                !bus_status[bus_idx].device_status[i].communication_flag) {
                cJSON *device                     = cJSON_CreateObject();
                chain_device_status_t *dev_status = &bus_status[bus_idx].device_status[i];

                // 基本设备信息
                cJSON_AddNumberToObject(device, "id", dev_status->id);
                cJSON_AddStringToObject(device, "type", chain_device_type_name(dev_status->type));
                cJSON_AddBoolToObject(device, "connected", dev_status->connected);
                if (dev_status->uid_valid) {
                    cJSON *uid_array = cJSON_CreateArray();
                    for (int j = 0; j < 4; j++) {
                        cJSON_AddItemToArray(uid_array, cJSON_CreateNumber(dev_status->uid[j]));
                    }
                    cJSON_AddItemToObject(device, "uid", uid_array);
                }
                cJSON_AddBoolToObject(device, "communication_flag", dev_status->communication_flag);

                // RGB状态
                cJSON_AddNumberToObject(device, "rgb_color", dev_status->rgb_color);
                cJSON_AddBoolToObject(device, "rgb_setting", dev_status->rgb_setting);

                // 设备数据
                cJSON *device_data = cJSON_CreateObject();
                switch (dev_status->type) {
                    case CHAIN_KEY_TYPE_CODE:
                        cJSON_AddNumberToObject(device_data, "button_status",
                                                dev_status->device_data.key_data.button_status);
                        break;
                    case CHAIN_PIR_TYPE_CODE:
                        cJSON_AddNumberToObject(device_data, "detect_status",
                                                dev_status->device_data.pir_data.detect_status);
                        cJSON_AddNumberToObject(device_data, "trigger_count",
                                                dev_status->device_data.pir_data.trigger_count);
                        break;
                    case CHAIN_JOYSTICK_TYPE_CODE:
                        cJSON_AddNumberToObject(device_data, "x_value", dev_status->device_data.joystick_data.x_value);
                        cJSON_AddNumberToObject(device_data, "y_value", dev_status->device_data.joystick_data.y_value);
                        cJSON_AddNumberToObject(device_data, "button_status",
                                                dev_status->device_data.joystick_data.button_status);
                        break;
                    case CHAIN_ENCODER_TYPE_CODE:
                        cJSON_AddNumberToObject(device_data, "encoder_value",
                                                dev_status->device_data.encoder_data.encoder_value);
                        cJSON_AddNumberToObject(device_data, "last_encoder_value",
                                                dev_status->device_data.encoder_data.last_encoder_value);
                        cJSON_AddNumberToObject(device_data, "button_status",
                                                dev_status->device_data.encoder_data.button_status);
                        break;
                    case CHAIN_TOF_TYPE_CODE:
                        cJSON_AddNumberToObject(device_data, "distance", dev_status->device_data.tof_data.distance);
                        cJSON_AddNumberToObject(device_data, "last_distance",
                                                dev_status->device_data.tof_data.last_distance);
                        break;
                    case CHAIN_ANGLE_TYPE_CODE:
                        cJSON_AddNumberToObject(device_data, "angle_value",
                                                dev_status->device_data.angle_data.angle_value);
                        cJSON_AddNumberToObject(device_data, "last_angle_value",
                                                dev_status->device_data.angle_data.last_angle_value);
                        break;
                    case CHAIN_SWITCH_TYPE_CODE:
                        cJSON_AddNumberToObject(device_data, "switch_status",
                                                dev_status->device_data.switch_data.switch_status);
                        cJSON_AddNumberToObject(device_data, "switch_count",
                                                dev_status->device_data.switch_data.switch_count);
                        break;
                    case CHAIN_PEDAL_TYPE_CODE:
                        cJSON_AddNumberToObject(device_data, "button_status",
                                                dev_status->device_data.pedal_data.button_status);
                        cJSON_AddNumberToObject(device_data, "switch_status",
                                                dev_status->device_data.pedal_data.switch_status);
                        break;
                    case CHAIN_MIC_TYPE_CODE:
                        cJSON_AddNumberToObject(device_data, "adc_value", dev_status->device_data.mic_data.adc_value);
                        cJSON_AddNumberToObject(device_data, "threshold", dev_status->device_data.mic_data.threshold);
                        break;
                    case CHAIN_ENV_TYPE_CODE:
                        cJSON_AddNumberToObject(device_data, "temperature",
                                                dev_status->device_data.env_data.temperature);
                        cJSON_AddNumberToObject(device_data, "humidity", dev_status->device_data.env_data.humidity);
                        cJSON_AddNumberToObject(device_data, "pressure", dev_status->device_data.env_data.pressure);
                        cJSON_AddNumberToObject(device_data, "altitude", dev_status->device_data.env_data.altitude);
                        cJSON_AddBoolToObject(device_data, "spa_ok", dev_status->device_data.env_data.spa_ok);
                        break;
                    case CHAIN_IMU_TYPE_CODE:
                        cJSON_AddNumberToObject(device_data, "ax", dev_status->device_data.imu_data.ax);
                        cJSON_AddNumberToObject(device_data, "ay", dev_status->device_data.imu_data.ay);
                        cJSON_AddNumberToObject(device_data, "az", dev_status->device_data.imu_data.az);
                        cJSON_AddNumberToObject(device_data, "gx", dev_status->device_data.imu_data.gx);
                        cJSON_AddNumberToObject(device_data, "gy", dev_status->device_data.imu_data.gy);
                        cJSON_AddNumberToObject(device_data, "gz", dev_status->device_data.imu_data.gz);
                        cJSON_AddNumberToObject(device_data, "temperature",
                                                dev_status->device_data.imu_data.temperature);
                        break;
                    case CHAIN_DLight_TYPE_CODE:
                        cJSON_AddNumberToObject(device_data, "lux", dev_status->device_data.dlight_data.lux);
                        break;
                    case UNIT_8SERVOS2_CHAIN_TYPE_CODE: {
                        cJSON *angles = cJSON_CreateArray();
                        for (int j = 0; j < 8; j++) {
                            cJSON_AddItemToArray(angles,
                                                 cJSON_CreateNumber(dev_status->device_data.servos_data.angles[j]));
                        }
                        cJSON_AddItemToObject(device_data, "angles", angles);
                        cJSON_AddNumberToObject(device_data, "dc_voltage",
                                                dev_status->device_data.servos_data.dc_voltage);
                        cJSON_AddNumberToObject(device_data, "grove_voltage",
                                                dev_status->device_data.servos_data.grove_voltage);
                        break;
                    }
                    case UNIT_CHAIN_BUS_TYPE_CODE: {
                        cJSON *addrs = cJSON_CreateArray();
                        for (int j = 0; j < dev_status->device_data.chain_bus_data.i2c_addr_count; j++) {
                            cJSON_AddItemToArray(
                                addrs, cJSON_CreateNumber(dev_status->device_data.chain_bus_data.i2c_addrs[j]));
                        }
                        cJSON_AddItemToObject(device_data, "i2c_addrs", addrs);
                        cJSON_AddNumberToObject(device_data, "i2c_addr_count",
                                                dev_status->device_data.chain_bus_data.i2c_addr_count);
                        break;
                    }
                    default:
                        break;
                }
                cJSON_AddItemToObject(device, "device_data", device_data);

                // 添加HID配置信息（根据设备类型）
                cJSON *hid_config = cJSON_CreateObject();
                switch (dev_status->type) {
                    case CHAIN_KEY_TYPE_CODE: {
                        cJSON_AddNumberToObject(hid_config, "single_click",
                                                dev_status->hid_config.key_config.single_click);
                        cJSON_AddNumberToObject(hid_config, "double_click",
                                                dev_status->hid_config.key_config.double_click);
                        cJSON_AddNumberToObject(hid_config, "long_press", dev_status->hid_config.key_config.long_press);
                        cJSON_AddNumberToObject(hid_config, "press_down", dev_status->hid_config.key_config.press_down);
                        cJSON_AddNumberToObject(hid_config, "press_release",
                                                dev_status->hid_config.key_config.press_release);
                        break;
                    }
                    case CHAIN_JOYSTICK_TYPE_CODE: {
                        cJSON_AddNumberToObject(hid_config, "single_click",
                                                dev_status->hid_config.joystick_config.single_click);
                        cJSON_AddNumberToObject(hid_config, "double_click",
                                                dev_status->hid_config.joystick_config.double_click);
                        cJSON_AddNumberToObject(hid_config, "long_press",
                                                dev_status->hid_config.joystick_config.long_press);
                        cJSON_AddNumberToObject(hid_config, "press_down",
                                                dev_status->hid_config.joystick_config.press_down);
                        cJSON_AddNumberToObject(hid_config, "press_release",
                                                dev_status->hid_config.joystick_config.press_release);
                        cJSON_AddBoolToObject(hid_config, "xy_move_reverse",
                                              dev_status->hid_config.joystick_config.xy_move_reverse);
                        cJSON_AddNumberToObject(hid_config, "xy_move_func",
                                                dev_status->hid_config.joystick_config.xy_move_func);
                        break;
                    }
                    case CHAIN_ENCODER_TYPE_CODE: {
                        cJSON_AddNumberToObject(hid_config, "single_click",
                                                dev_status->hid_config.encoder_config.single_click);
                        cJSON_AddNumberToObject(hid_config, "double_click",
                                                dev_status->hid_config.encoder_config.double_click);
                        cJSON_AddNumberToObject(hid_config, "long_press",
                                                dev_status->hid_config.encoder_config.long_press);
                        cJSON_AddNumberToObject(hid_config, "press_down",
                                                dev_status->hid_config.encoder_config.press_down);
                        cJSON_AddNumberToObject(hid_config, "press_release",
                                                dev_status->hid_config.encoder_config.press_release);
                        cJSON_AddNumberToObject(hid_config, "rotate_cw_func",
                                                dev_status->hid_config.encoder_config.rotate_cw_func);
                        cJSON_AddNumberToObject(hid_config, "rotate_ccw_func",
                                                dev_status->hid_config.encoder_config.rotate_ccw_func);
                        break;
                    }
                    case CHAIN_ANGLE_TYPE_CODE: {
                        cJSON_AddNumberToObject(hid_config, "angle_func",
                                                dev_status->hid_config.angle_config.angle_func);
                        break;
                    }
                    case CHAIN_PEDAL_TYPE_CODE: {
                        cJSON_AddNumberToObject(hid_config, "single_click",
                                                dev_status->hid_config.pedal_config.single_click);
                        cJSON_AddNumberToObject(hid_config, "double_click",
                                                dev_status->hid_config.pedal_config.double_click);
                        cJSON_AddNumberToObject(hid_config, "long_press",
                                                dev_status->hid_config.pedal_config.long_press);
                        cJSON_AddNumberToObject(hid_config, "press_down",
                                                dev_status->hid_config.pedal_config.press_down);
                        cJSON_AddNumberToObject(hid_config, "press_release",
                                                dev_status->hid_config.pedal_config.press_release);
                        break;
                    }
                    case CHAIN_MIC_TYPE_CODE: {
                        cJSON_AddNumberToObject(hid_config, "high_threshold_func",
                                                dev_status->hid_config.mic_config.high_threshold_func);
                        cJSON_AddNumberToObject(hid_config, "low_threshold_func",
                                                dev_status->hid_config.mic_config.low_threshold_func);
                        cJSON_AddNumberToObject(hid_config, "threshold", dev_status->hid_config.mic_config.threshold);
                        cJSON_AddNumberToObject(hid_config, "trigger_interval_ms",
                                                dev_status->hid_config.mic_config.trigger_interval_ms);
                        break;
                    }
                    case CHAIN_SWITCH_TYPE_CODE: {
                        cJSON_AddNumberToObject(hid_config, "open_func",
                                                dev_status->hid_config.switch_config.open_func);
                        cJSON_AddNumberToObject(hid_config, "close_func",
                                                dev_status->hid_config.switch_config.close_func);
                        break;
                    }
                    case CHAIN_DLight_TYPE_CODE: {
                        cJSON_AddNumberToObject(hid_config, "lux_high_func",
                                                dev_status->hid_config.dlight_config.lux_high_func);
                        cJSON_AddNumberToObject(hid_config, "lux_low_func",
                                                dev_status->hid_config.dlight_config.lux_low_func);
                        cJSON_AddNumberToObject(hid_config, "high_threshold",
                                                dev_status->hid_config.dlight_config.high_threshold);
                        cJSON_AddNumberToObject(hid_config, "low_threshold",
                                                dev_status->hid_config.dlight_config.low_threshold);
                        break;
                    }
                    case CHAIN_PIR_TYPE_CODE: {
                        cJSON_AddNumberToObject(hid_config, "person_come_func",
                                                dev_status->hid_config.pir_config.person_come_func);
                        cJSON_AddNumberToObject(hid_config, "person_leave_func",
                                                dev_status->hid_config.pir_config.person_leave_func);
                        break;
                    }
                    default:
                        // 其他设备类型不支持HID配置
                        break;
                }
                cJSON_AddItemToObject(device, "hid_config", hid_config);

                // 如果有新事件，添加事件信息
                if (dev_status->event_updated) {
                    cJSON_AddStringToObject(device, "last_event", dev_status->last_event_desc);
                    cJSON_AddNumberToObject(device, "event_time", dev_status->last_event_time);
                    dev_status->event_updated = false;  // 重置事件更新标记
                }

                cJSON_AddItemToArray(devices_array, device);
                dev_status->status_updated = false;  // 重置状态更新标记
                dev_status->rgb_updated    = false;  // 重置RGB更新标记
            }
        }
    }

    send_bus_all_data = false;
    cJSON_AddItemToObject(left_bus, "devices", left_devices);
    cJSON_AddItemToObject(right_bus, "devices", right_devices);

    cJSON_AddItemToObject(chainbus, "left_bus", left_bus);
    cJSON_AddItemToObject(chainbus, "right_bus", right_bus);

    cJSON_AddStringToObject(json, "type", "status_update");
    cJSON_AddItemToObject(json, "dualkey", dualkey);
    cJSON_AddItemToObject(json, "chainbus", chainbus);

    char *json_str = cJSON_PrintUnformatted(json);
    if (json_str) {
        httpd_ws_frame_t ws_pkt;
        memset(&ws_pkt, 0, sizeof(httpd_ws_frame_t));
        ws_pkt.type    = HTTPD_WS_TYPE_TEXT;
        ws_pkt.payload = (uint8_t *)json_str;
        ws_pkt.len     = strlen(json_str);

        esp_err_t ret = httpd_ws_send_frame_async(server, websocket_fd, &ws_pkt);
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "WebSocket发送失败: %s", esp_err_to_name(ret));
        }

        free(json_str);
    }

    cJSON_Delete(json);
}

// 更新设备状态
static void update_device_status(void)
{
    /*!< 更新拨码开关位置 —— 由两路原始 ADC 通道直接派生，不再采信 switch_pos。
     *
     *   为什么改（2026-09-15 实测）：设备实物在 WiFi 档时，网页收到的
     *   dip_switch_pos 却是 0（中间档），而同一次报文里的原始采样是
     *   switch_1=92 / switch_2=2983 —— WiFi 通道明显越过阈值 2000。
     *   即 switch_pos 与硬件实际状态可以长期不一致，而这两路原始值是
     *   adc_switch_task 每秒直读的，永远跟硬件一致。
     *
     *   只在休眠未开启时采信原始值：light sleep 生效后（仅蓝牙档启用）ADC 会
     *   系统性塌陷到 ~1V，读数不可信。蓝牙档不启动网页服务，这段代码在那时
     *   本来也不可达 —— 加这道判断只是为了让上报值在任何情况下都不撒谎。
     *
     *   功能不受影响：设备行为由开机快照 ble_only_mode 与 dip_switch_wifi_enabled()
     *   决定，本函数只负责网页显示用的那个字段。 */
    if (!g_light_sleep_on) {
        if (g_device_status.switch_1_value > DIP_SWITCH_ADC_THRESHOLD) {
            g_device_status.dip_switch_pos = DIP_SWITCH_POS_BLE;
        } else if (g_device_status.switch_2_value > DIP_SWITCH_ADC_THRESHOLD) {
            g_device_status.dip_switch_pos = DIP_SWITCH_POS_WIFI;
        } else if (g_device_status.switch_1_value > 0 || g_device_status.switch_2_value > 0) {
            /*!< 两路都低于阈值、但至少一路有真实采样（不是全 0 的"尚未采样"）
             *   → 这才是真正的中间 OFF 档。 */
            g_device_status.dip_switch_pos = DIP_SWITCH_POS_CENTER;
        }
        /*!< ⚠️ 这条路径上刻意不做任何 NVS / flash 写入。
         *
         *   2026-09-15：上一版在这里调用 diag_note_dip_view() 往 NVS 追加一行取证，
         *   烧录后实机立刻出现"疯狂反复重启"（USB 在两档 PID 之间反复重枚举）。
         *   本函数由 websocket_task 以 ~2Hz 调用，在这条高频路径上做 flash 写属于
         *   高风险动作（cache 停摆、IPC 停核、NVS GC 时长不可控），而它换来的只是
         *   "switch_pos 是否与硬件分歧"这一条诊断信息 —— 不值得用设备可用性去换。
         *
         *   显示值本身已经由原始 ADC 派生，与硬件一致，不需要 switch_pos 的旁证；
         *   真要做档位分歧取证，应放在 adc_switch_task（1Hz、且本来就承担该职责）。 */
    }

    // 更新电池状态 (使用test_case.c中的真实数据)
    g_device_status.battery_voltage    = g_battery_voltage;
    g_device_status.charge_status      = g_charging_status;
    g_device_status.battery_percentage = g_battery_percentage;
    g_device_status.usb_connected      = g_usb_connected;
    g_device_status.usb_voltage        = g_usb_voltage;

    // 更新蓝牙状态
    g_device_status.bluetooth_connected = (g_connect_status == 2);
    const char *device_name             = ble_hid_get_device_name();
    if (device_name != NULL) {
        strncpy(g_device_status.bluetooth_device_name, device_name, sizeof(g_device_status.bluetooth_device_name) - 1);
        g_device_status.bluetooth_device_name[sizeof(g_device_status.bluetooth_device_name) - 1] = '\0';
    }

    // 更新配对状态 (基于连接状态推断)
    g_device_status.bluetooth_pairing_status = g_connect_status;

    // 更新广播状态
    g_ble_adv_status                     = ble_hid_is_advertising();
    g_device_status.bluetooth_adv_status = g_ble_adv_status;

    // 同步按键映射状态
    g_device_status.current_key_mapping = btn_progress_get_key_mapping();

    // 更新WIFI状态
    if (wifi_ap_mode) {
        // AP模式：显示AP的信息
        g_device_status.wifi_connected = true;

        // 获取AP配置
        wifi_config_t wifi_config;
        esp_err_t ret = esp_wifi_get_config(WIFI_IF_AP, &wifi_config);
        if (ret == ESP_OK) {
            strncpy(g_device_status.wifi_ssid, (char *)wifi_config.ap.ssid, sizeof(g_device_status.wifi_ssid) - 1);
            g_device_status.wifi_ssid[sizeof(g_device_status.wifi_ssid) - 1] = '\0';
        } else {
            strcpy(g_device_status.wifi_ssid, "");
        }

        // 获取AP IP地址
        esp_netif_t *netif = esp_netif_get_handle_from_ifkey("WIFI_AP_DEF");
        if (netif != NULL) {
            esp_netif_ip_info_t ip_info;
            ret = esp_netif_get_ip_info(netif, &ip_info);
            if (ret == ESP_OK) {
                inet_ntoa_r(ip_info.ip.addr, g_device_status.wifi_ip, sizeof(g_device_status.wifi_ip));
            } else {
                strcpy(g_device_status.wifi_ip, "0.0.0.0");
            }
        } else {
            strcpy(g_device_status.wifi_ip, "0.0.0.0");
        }

        g_device_status.wifi_rssi = 0;
    } else if (wifi_connected) {
        // STA模式已连接：显示连接的AP信息
        g_device_status.wifi_connected = true;

        // 获取SSID
        wifi_ap_record_t ap_info;
        esp_err_t ret = esp_wifi_sta_get_ap_info(&ap_info);
        if (ret == ESP_OK) {
            strncpy(g_device_status.wifi_ssid, (char *)ap_info.ssid, sizeof(g_device_status.wifi_ssid) - 1);
            g_device_status.wifi_ssid[sizeof(g_device_status.wifi_ssid) - 1] = '\0';
            g_device_status.wifi_rssi                                        = ap_info.rssi;
        } else {
            strcpy(g_device_status.wifi_ssid, "");
            g_device_status.wifi_rssi = 0;
        }

        // 获取IP地址
        esp_netif_t *netif = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");
        if (netif != NULL) {
            esp_netif_ip_info_t ip_info;
            ret = esp_netif_get_ip_info(netif, &ip_info);
            if (ret == ESP_OK) {
                inet_ntoa_r(ip_info.ip.addr, g_device_status.wifi_ip, sizeof(g_device_status.wifi_ip));
            } else {
                strcpy(g_device_status.wifi_ip, "0.0.0.0");
            }
        } else {
            strcpy(g_device_status.wifi_ip, "0.0.0.0");
        }
    } else {
        // WiFi未连接
        g_device_status.wifi_connected = false;
        strcpy(g_device_status.wifi_ssid, "");
        strcpy(g_device_status.wifi_ip, "0.0.0.0");
        g_device_status.wifi_rssi = 0;
    }
}

/*!< 直接写单颗 LED 像素。调用者须已持有 light_progress_lock()，
 *   原因见 apply_key_colors()。 */
static void set_key_rgb_color_locked(int key_index, uint32_t rgb_color)
{
    uint8_t R = (rgb_color >> 16) & 0xFF;
    uint8_t G = (rgb_color >> 8) & 0xFF;
    uint8_t B = rgb_color & 0xFF;
    led_strip_set_pixel(led_strip, key_index, R, G, B);
    led_strip_refresh(led_strip);
}

/*!< 原子地写两颗灯：左键 = LED1，右键 = LED0。
 *
 *   必须成对写入 —— rgb_matrix 与这里共用同一份像素缓冲且会整条刷新，
 *   若分成两次独立写，中间可能被它的清屏冲掉一颗（表现为某个键的灯熄灭）。 */
static void apply_key_colors(uint32_t left_color, uint32_t right_color)
{
    light_progress_lock();
    set_key_rgb_color_locked(1, left_color);
    set_key_rgb_color_locked(0, right_color);
    light_progress_unlock();
}

/*!< 按 g_device_status 里的静态色刷新两颗灯。
 *   overlay 接管期间（充电电量指示）不动灯，释放后由接管方负责恢复。 */
static void refresh_key_colors_from_status(void)
{
    if (light_progress_is_overlay_active()) {
        return;
    }
    apply_key_colors(g_device_status.left_key_color, g_device_status.right_key_color);
}

// WebSocket任务
static void websocket_task(void *pvParameters)
{
    status_refresh_type_t refresh_msg;
    TickType_t timeout = pdMS_TO_TICKS(500);  // 500ms超时

    while (1) {
        // 等待状态刷新指令，超时时间为500ms
        if (xQueueReceive(status_refresh_queue, &refresh_msg, timeout) == pdTRUE) {
            // 收到刷新指令，立即刷新状态
            // ESP_LOGI(TAG, "Status refresh requested: %s", refresh_msg == STATUS_REFRESH_IMMEDIATE ? "immediate" :
            // "normal");
        } else {
            // 超时，执行正常的状态更新
            // ESP_LOGD(TAG, "Status update timeout, performing normal refresh");
        }

        // 更新设备状态
        update_device_status();
        websocket_send_status();

        // 更新电源状态
        update_power_status(NULL);
    }
}

/*!< 蓝牙档(仅蓝牙)下的设备状态刷新任务
 *   蓝牙档不启动 HTTP/WebSocket 服务, 但 g_device_status.bluetooth_connected 依赖
 *   update_device_status() 更新, app_main 主循环据此上报 BLE 电量, 因此不可省略 */
static void device_status_task(void *pvParameters)
{
    int tick = 0;

    while (1) {
        update_device_status();
        // 电量变化缓慢, 降低 ADC 采样频率(单次采样约 200ms)以省电；
        // 5s 一次是为了让"插入 USB"能在数秒内触发充电电量指示
        if (tick % 5 == 0) {
            update_power_status(NULL);
        }
        tick++;
        /*!< 自动关机检查（仅本任务存在，即仅蓝牙档）。周期约 1s，
         *   所以超时精度是 ±1s 量级 —— 对"分钟"级超时完全够用。 */
        auto_off_tick();
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

// 启动web服务器
static httpd_handle_t start_webserver(void)
{
    if (server != NULL) {
        return server;
    }

    httpd_config_t config   = HTTPD_DEFAULT_CONFIG();
    config.stack_size       = 1024 * 8;
    config.max_open_sockets = 6;
    config.max_uri_handlers = 32;
    config.lru_purge_enable = true;
    config.uri_match_fn     = httpd_uri_match_wildcard;

    ESP_LOGI(TAG, "Starting server on port: '%d'", config.server_port);
    if (httpd_start(&server, &config) == ESP_OK) {
        ESP_LOGI(TAG, "Registering URI handlers");

        // 注册URI处理器
        httpd_uri_t index_uri = {.uri = "/", .method = HTTP_GET, .handler = index_get_handler};
        httpd_register_uri_handler(server, &index_uri);

        httpd_uri_t styles_uri = {.uri = "/styles.css", .method = HTTP_GET, .handler = styles_get_handler};
        httpd_register_uri_handler(server, &styles_uri);

        httpd_uri_t script_uri = {.uri = "/script.js", .method = HTTP_GET, .handler = script_get_handler};
        httpd_register_uri_handler(server, &script_uri);

        // 注册WebSocket处理器
        httpd_uri_t websocket_uri = {
            .uri = "/ws", .method = HTTP_GET, .handler = websocket_handler, .user_ctx = NULL, .is_websocket = true};
        httpd_register_uri_handler(server, &websocket_uri);

        httpd_uri_t favicon_uri = {.uri = "/favicon.ico", .method = HTTP_GET, .handler = favicon_get_handler};
        httpd_register_uri_handler(server, &favicon_uri);

        // 功耗诊断读数端点（数据由 BLE 档写入 NVS，此处只读回）
        httpd_uri_t diag_uri = {.uri = "/diag", .method = HTTP_GET, .handler = diag_get_handler};
        httpd_register_uri_handler(server, &diag_uri);

        web_assets_register_handlers(server);

        // 注册404错误处理器
        httpd_register_err_handler(server, HTTPD_404_NOT_FOUND, http_404_error_handler);
    }
    return server;
}

/*!< ==================== 按键灯效（替代原 TYPING_HEATMAP 热力图）====================
 *
 *   为什么换掉热力图（2026-09-15，用户实测反馈）：
 *   热力值是"每次按键 +32、每渲染帧 -2"的**累积**模型，衰减速率还完全绑在渲染周期上。
 *   本机只有 2 颗灯，同一个键会被反复加热 —— 连点超过约 6 次/秒，累积就压过衰减，
 *   一路冲到 val>=85，于是亮度到顶、色相从蓝被推向绿/红，观感是"越来越亮、还会变色、
 *   刺眼"；停止后又要等热力慢慢排掉，"残留很久"。
 *
 *   新模型：按下 = 一次**脉冲**。该灯立刻亮到峰值，然后在 KEY_FLASH_DURATION_MS 内
 *   线性淡到 0。连点只是不断重置同一颗灯的起始时刻，**结构上不可能叠加**。
 *
 *   与 rgb_matrix 的关系：效果本身仍在（mode=2），但不再喂热力 ⇒ 它的输出恒为全灭，
 *   空闲观感与改动前一致。脉冲期间用 overlay 暂停 rgb_matrix 渲染，直接写灯带像素
 *   （与充电电量指示同一套已验证机制）。 */
#define KEY_FLASH_PEAK_LEVEL  100  /*!< 峰值亮度 0~255；刻意压低以免刺眼 */
#define KEY_FLASH_DURATION_MS 1000 /*!< 从峰值线性淡到 0 的时长 */

/*!< 每颗灯各自的脉冲起始时刻；INT64_MIN/4 表示未激活（避免与 uptime 相减溢出）。 */
static int64_t s_key_flash_start_us[2] = {INT64_MIN / 4, INT64_MIN / 4};
static bool    s_key_flash_overlay     = false;

/*!< 按键灯效是否开启：沿用网页那个开关，也就是 rgb_matrix 的 enable 位 ——
 *   这样上报给网页的状态（main.cpp 里 cJSON "key_led_effect_enabled"）与这里的实际
 *   行为永远一致，不会出现"网页显示开着但灯不亮"。 */
static bool key_led_effect_enabled(void)
{
    return rgb_matrix_is_enabled();
}

/*!< input_index：0 = 左键，1 = 右键。LED 映射与之交叉：左键 = LED1，右键 = LED0
 *   （与 set_key_rgb_color_locked / power_indicator_task 的既有约定一致）。 */
#define KEY_FLASH_LED_OF_INPUT(input_index) ((input_index) == 0 ? 1 : 0)

static void key_flash_trigger(int input_index)
{
    if (!key_led_effect_enabled()) {
        return;
    }
    s_key_flash_start_us[KEY_FLASH_LED_OF_INPUT(input_index)] = esp_timer_get_time();

    /*!< 立刻唤醒渲染任务：空闲周期是 200ms，若只等它自己醒来，第一次点亮最多晚 200ms
     *   —— 那正是"按下去灯半天才亮"的观感来源。 */
    if (light_progress_task_handle != NULL) {
        xTaskNotifyGive(light_progress_task_handle);
    }
}

/*!< 是否还有灯处于脉冲中。渲染任务据此选周期，充电指示也据此让位。 */
static bool key_flash_any_active(void)
{
    if (!key_led_effect_enabled()) {
        return false;
    }
    const int64_t now = esp_timer_get_time();
    for (int led = 0; led < 2; led++) {
        const int64_t t = now - s_key_flash_start_us[led];
        if (t >= 0 && t < (int64_t)KEY_FLASH_DURATION_MS * 1000) {
            return true;
        }
    }
    return false;
}

/*!< 渲染一帧按键灯效。
 *   只写"仍在脉冲中"的那些灯，其余像素保持原样不碰 —— 这样充电电量指示画的那颗灯
 *   不会被我们抹掉，也就不需要在这里重建"静息色"（少一份与业务状态的耦合）。 */
static void key_flash_render(void)
{
    const int64_t now  = esp_timer_get_time();
    const int64_t span = (int64_t)KEY_FLASH_DURATION_MS * 1000;
    bool          wrote = false;

    light_progress_lock();
    if (led_strip != NULL) {
        for (int led = 0; led < 2; led++) {
            const int64_t t = now - s_key_flash_start_us[led];
            if (t < 0 || t >= span) {
                continue;
            }
            const uint8_t level = (uint8_t)((int64_t)KEY_FLASH_PEAK_LEVEL * (span - t) / span);
            led_strip_set_pixel(led_strip, led, 0, 0, level); /*!< 纯蓝，与原来按键反馈的色相一致 */
            wrote = true;
        }
        if (wrote) {
            led_strip_refresh(led_strip);
        }
    }
    light_progress_unlock();

    if (wrote) {
        if (!s_key_flash_overlay) {
            s_key_flash_overlay = true;
            light_progress_set_flash_overlay(true);
        }
    } else if (s_key_flash_overlay) {
        s_key_flash_overlay = false;
        light_progress_set_flash_overlay(false);
    }
}

static void keyboard_cb(keyboard_btn_handle_t kbd_handle, keyboard_btn_report_t kbd_report, void *user_data)
{
    ESP_LOGI(TAG, "keyboard_cb: pressed=%ld, released=%ld, changed=%d", kbd_report.key_pressed_num,
             kbd_report.key_release_num, kbd_report.key_change_num);

    if (rgb_matrix_get_suspend_state() == true && g_device_status.left_key_color == 0x000000 &&
        g_device_status.right_key_color == 0x000000) {
        rgb_matrix_set_suspend_state(false);
        vTaskDelay(1 / portTICK_PERIOD_MS);
    }

    // 更新按键状态
    g_device_status.left_key_pressed  = false;
    g_device_status.right_key_pressed = false;

    // 检查当前按下的按键
    for (int i = 0; i < kbd_report.key_pressed_num; i++) {
        uint8_t input_index = kbd_report.key_data[i].input_index;
        // 假设output_index 0对应左键，1对应右键
        if (input_index == 0) {
            g_device_status.left_key_pressed = true;
        } else if (input_index == 1) {
            g_device_status.right_key_pressed = true;
        }
    }

    btn_progress(kbd_report);

    /*!< 按键灯效：只认"按下"边沿。
     *   ⚠️ 这里必须用 key_change_num（按下 +1），不能用 key_pressed_num ——
     *   松手时 key_change_num = 0 - 上一次的 1（uint32 回绕）→ 存进 int 字段是 -1，
     *   `> 0` 自然跳过，所以不会在松手时再触发一次脉冲，也不会出现负索引
     *   （key_pressed_num - i 只在按下时才被求值，落在 0 .. n-1 内）。 */
    if (kbd_report.key_change_num > 0) {
        for (int i = 1; i <= kbd_report.key_change_num; i++) {
            key_flash_trigger(kbd_report.key_data[kbd_report.key_pressed_num - i].input_index);
        }
    }

    // 按键状态变更时，发送立即刷新指令
    if (status_refresh_queue && kbd_report.key_change_num != 0) {
        status_refresh_type_t refresh_msg = STATUS_REFRESH_IMMEDIATE;
        xQueueSend(status_refresh_queue, &refresh_msg, 0);
        // ESP_LOGI(TAG, "Sent immediate status refresh request due to key state change");
    }

    // iot_button 仅用于双键组合长按（复位/配网）检测。
    // 该组件定时器不会自行停止，这里跟随按键状态启停：有键按下才跑，全部松开立即停，
    // 避免 20ms(50Hz) 常驻唤醒拖住 tickless idle。
    iot_button_timer_enable(kbd_report.key_pressed_num > 0);
}

/*!< 灯效任务心跳：空闲 200ms / 有键按下或灯效淡出中 10ms。
 *   原为固定 10ms 无条件唤醒，会持续把 CPU 从 light sleep 拉起来。
 *   按键上报不依赖本任务轮询（keyboard_cb 由 GPIO 中断边沿驱动，s_keys_down 在
 *   回调里同步更新），所以空闲期拉长不影响按键手感；空闲 200ms 只是让长按"到点
 *   触发"最多晚一个心跳（长按阈值本身是数百 ms 级，长度按绝对时间戳计算，不会
 *   累积误差）；有键按下、或按键灯效脉冲仍未淡完时回到 10ms，
 *   保证长按判定精度与淡出的平滑度。
 *   在 light sleep 下本任务的每个心跳周期都对应一次唤醒，200ms 把这里的唤醒
 *   频率从 20/s 降到 5/s。 */
#define LIGHT_TASK_PERIOD_IDLE_MS 200
#define LIGHT_TASK_PERIOD_ACTIVE_MS 10

static void light_progress_task(void *pvParameters)
{
    while (1) {
        if (bsp_ws2812_is_enable()) {
            /*!< 按键灯效需要自己画像素并暂停 rgb_matrix，故它优先；没有脉冲时才走常规渲染。
             *   注意条件里带上 s_key_flash_overlay：脉冲结束的那一帧仍要进来一次，
             *   由 key_flash_render() 负责把 overlay 放掉，否则灯带会一直停在被接管的画面。 */
            if (key_flash_any_active() || s_key_flash_overlay) {
                key_flash_render();
            } else {
                light_progress();
            }
        }
        /*!< 顺便作为心跳：检查长按是否已到阈值（方案A：到点即触发） */
        btn_progress_tick();
        /*!< 快节奏的条件：键还按着，或按键灯效正在淡出（淡出要靠每帧重画才平滑）。
         *   其余时间回到 200ms，把 light sleep 下的唤醒从 20/s 降到 5/s —— 按键是用户
         *   主动行为，其后 1s 内的多几次唤醒不影响功耗大局。 */
        const bool fast_period = btn_progress_has_pressed_key() || key_flash_any_active();
        /*!< 用"通知 + 超时"等待，而不是纯 vTaskDelay：按键回调会 xTaskNotifyGive()
         *   立刻唤醒本任务，否则空闲档下第一次点亮最多要等一个 200ms 周期。 */
        ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(fast_period ? LIGHT_TASK_PERIOD_ACTIVE_MS : LIGHT_TASK_PERIOD_IDLE_MS));
    }
}

/*!< 充电指示亮度：刻意压到 ~10%，边充边用时不刺眼 */
#define POWER_LED_BRIGHTNESS 26

/*!< 电量等级 -> 低亮度颜色：红 <20%，琥珀 <60%，绿 >=60%。
 *   绿色通道人眼更敏感，故用更低的值保持观感一致。 */
static uint32_t power_indicator_color(int percentage)
{
    if (percentage < 20) {
        return (uint32_t)POWER_LED_BRIGHTNESS << 16;  // 红
    } else if (percentage < 60) {
        return ((uint32_t)POWER_LED_BRIGHTNESS << 16) | ((uint32_t)(POWER_LED_BRIGHTNESS / 2) << 8);  // 琥珀
    }
    return (uint32_t)(POWER_LED_BRIGHTNESS * 3 / 4) << 8;  // 绿
}

/*!< 充电电量指示：USB 连接期间以低亮度常亮显示电量等级。
 *
 *   rgb_matrix 每 10ms 就会重绘整条灯带，直接写像素会被立刻覆盖，因此必须用 overlay
 *   接管渲染；断开 USB 后释放 overlay 并恢复按键显示（静态色或按键灯效）。
 */
static void power_indicator_task(void *pvParameters)
{
    bool overlay_on = false;

    while (1) {
        if (g_usb_connected) {
            if (!overlay_on) {
                light_progress_set_charge_overlay(true);
                overlay_on = true;
            }
            /*!< 按键灯效脉冲期间让位：它正在那两颗灯上画淡出，这里若照旧写电量色会把
             *   淡出画面刷回去（表现为按键反馈被"吃掉"、或闪烁）。1s 后本任务会再刷一次，
             *   脉冲最长为 1s，所以最多漏刷一轮，观感无影响。 */
            if (!key_flash_any_active()) {
                const uint32_t color = power_indicator_color(g_battery_percentage);
                light_progress_lock();
                set_key_rgb_color_locked(0, color);  // 右键 LED index 0
                set_key_rgb_color_locked(1, color);  // 左键 LED index 1
                light_progress_unlock();
            }
            vTaskDelay(pdMS_TO_TICKS(1000));  // 常亮；每秒刷新一次电量等级
        } else if (overlay_on) {
            overlay_on = false;
            // 先放开渲染，等可能的一次性清屏落地，再成对恢复按键显示
            light_progress_set_charge_overlay(false);
            vTaskDelay(pdMS_TO_TICKS(30));
            refresh_key_colors_from_status();
            vTaskDelay(pdMS_TO_TICKS(20));
            refresh_key_colors_from_status();
        } else {
            /*!< 电池态空闲：只等 USB 插入事件，1s 足够（插线后最多 1s 内点亮电量指示）。
             *   原为 300ms，在 light sleep 下等于每秒多 2.3 次无用唤醒。 */
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }
}

/*!< 自动关机：超时配置与深睡入口。常量与 g_auto_off_min 见文件头部全局区。 */

static void auto_off_config_load(void)
{
    nvs_handle_t h = 0;
    if (nvs_open(AUTO_OFF_NVS_NS, NVS_READONLY, &h) != ESP_OK) {
        ESP_LOGI(TAG, "自动关机: 无存档，用默认 %d 分钟", AUTO_OFF_DEFAULT_MIN);
        return;
    }
    uint16_t v = AUTO_OFF_DEFAULT_MIN;
    if (nvs_get_u16(h, AUTO_OFF_NVS_KEY, &v) == ESP_OK && v <= AUTO_OFF_MAX_MIN) {
        g_auto_off_min = v;
    }
    nvs_close(h);
    ESP_LOGI(TAG, "自动关机超时: %u 分钟 (0=关闭)", (unsigned)g_auto_off_min);
}

static void auto_off_config_save(uint16_t minutes)
{
    if (minutes > AUTO_OFF_MAX_MIN) {
        minutes = AUTO_OFF_MAX_MIN;
    }
    g_auto_off_min = minutes;

    nvs_handle_t h = 0;
    if (nvs_open(AUTO_OFF_NVS_NS, NVS_READWRITE, &h) != ESP_OK) {
        ESP_LOGW(TAG, "自动关机: NVS 打开失败，本次仅内存生效");
        return;
    }
    nvs_set_u16(h, AUTO_OFF_NVS_KEY, minutes);
    nvs_commit(h);
    nvs_close(h);
    ESP_LOGI(TAG, "自动关机超时已保存: %u 分钟 (0=关闭)", (unsigned)minutes);
}

/*!< ---- 自动关机的灯效：睡前"呼吸 3 次" / 唤醒"呼吸 1 次" ----
 *
 *   2026-09-15 用户两次反馈后定下的观感：
 *   ① 首版 130ms 硬闪 3 次 —— "闪得很快，不是很好看"；
 *   ② 二版呼吸（14ms/步，约 1.6s 三段）—— 仍"闪得很快，完全没看出呼吸效果"，
 *      而且"灯还是蓝色的"。
 *
 *   **为什么暗灰看着是蓝的**：二版用的是等值 RGB `0x2A2A2A`。WS2812 的三色
 *   LED 在低电流下发光效率并不一致（蓝/绿通道比红更早被点亮），等值低亮度
 *   在实物上会被看成偏蓝的白。所以这次**直接给纯红** —— 观感明确、不会串色，
 *   和充电指示（红/琥珀/绿，但常亮）、按键灯效（纯蓝脉冲）也区分得开。
 *   红色通道单色发光，亮度爬升过程肉眼最容易分辨"渐亮渐暗"。
 *
 *   **为什么放慢**：14ms/步 时一次呼吸只有约 0.41s，人眼会当成"闪一下"而不是
 *   "呼吸一下"。这次把步长放到 22ms、并把渐亮/渐暗步数加上去：
 *     单次呼吸 ≈ 14×22（渐亮 308ms）+ 150（保持）+ 18×22（渐暗 396ms） ≈ 854ms，
 *     三次 + 两次间隔(260ms) ≈ **3.1s** —— 明显是一段"呼吸"，不是眨眼。
 *   两种信号仍用"次数"区分，用户凭灯就知道当前是睡还是醒：
 *     · 睡前 = 呼吸 **3 次**（约 3.1s）——"要睡了"
 *     · 唤醒 = 呼吸 **1 次**（约 0.9s，单次更长）——"醒了"
 *   不受"按键灯效"开关约束（那是按键反馈的开关），但尊重板级 bsp_ws2812 使能。 */
#define AUTO_OFF_BREATH_PULSES 3
#define AUTO_OFF_BREATH_COLOR  0x600000 /*!< 纯红，峰值 R=96/255：亮但不刺眼 */
#define AUTO_OFF_BREATH_UP     14       /*!< 渐亮步数 */
#define AUTO_OFF_BREATH_DOWN   18       /*!< 渐暗步数 */
#define AUTO_OFF_BREATH_STEP   22       /*!< ms/步（由 14 放慢到 22） */
#define AUTO_OFF_BREATH_HOLD   150      /*!< 峰值保持 */
#define AUTO_OFF_BREATH_GAP    260      /*!< 两次呼吸之间的间隔 */

/*!< 按 0~255 比例缩放一个 24bit RGB（纯整数，不用浮点） */
static uint32_t scale_rgb_brightness(uint32_t rgb, uint32_t scale)
{
    const uint32_t r = ((rgb >> 16) & 0xFF) * scale / 255u;
    const uint32_t g = ((rgb >> 8) & 0xFF) * scale / 255u;
    const uint32_t b = (rgb & 0xFF) * scale / 255u;
    return (r << 16) | (g << 8) | b;
}

/*!< 一次呼吸：渐亮 -> 保持 -> 渐暗 -> 回到全黑。返回后灯是灭的。
 *   ⚠️ 锁只包住"写像素"，绝不跨 vTaskDelay（本项目铁律：否则会饿死渲染任务）。 */
static void auto_off_breath_once(int up_steps, int down_steps, int step_ms, int hold_ms)
{
    for (int i = 1; i <= up_steps; i++) {
        const uint32_t c = scale_rgb_brightness(AUTO_OFF_BREATH_COLOR, (uint32_t)i * 255u / (uint32_t)up_steps);
        light_progress_lock();
        set_key_rgb_color_locked(0, c);
        set_key_rgb_color_locked(1, c);
        light_progress_unlock();
        vTaskDelay(pdMS_TO_TICKS(step_ms));
    }
    vTaskDelay(pdMS_TO_TICKS(hold_ms));
    for (int i = down_steps; i >= 0; i--) {
        const uint32_t c = scale_rgb_brightness(AUTO_OFF_BREATH_COLOR, (uint32_t)i * 255u / (uint32_t)down_steps);
        light_progress_lock();
        set_key_rgb_color_locked(0, c);
        set_key_rgb_color_locked(1, c);
        light_progress_unlock();
        vTaskDelay(pdMS_TO_TICKS(step_ms));
    }
}

/*!< 深睡前提示：三次呼吸。
 *   必须先接管渲染：light_progress() 见到 overlay 才会跳过 rgb_matrix_task()，
 *   否则它每 10~200ms 整条刷新一次，会把这里写的像素冲掉。
 *   结束时故意不释放 overlay —— 紧接着就 deep sleep 了。 */
static void auto_off_blink_warning(void)
{
    if (!bsp_ws2812_is_enable() || led_strip == NULL) {
        return;
    }
    light_progress_set_flash_overlay(true);
    for (int p = 0; p < AUTO_OFF_BREATH_PULSES; p++) {
        auto_off_breath_once(AUTO_OFF_BREATH_UP, AUTO_OFF_BREATH_DOWN, AUTO_OFF_BREATH_STEP,
                             AUTO_OFF_BREATH_HOLD);
        if (p + 1 < AUTO_OFF_BREATH_PULSES) {
            vTaskDelay(pdMS_TO_TICKS(AUTO_OFF_BREATH_GAP));
        }
    }
}

/*!< 深睡唤醒提示：**一次**更慢的呼吸，与睡前的三次区分开。
 *   2026-09-15 用户问"唤醒时没有灯效吗" —— 之前确实没有。电池态下两颗灯本来就常灭
 *   （默认键色 0x000000），所以"睡着"和"醒着"从灯上完全分不出来。
 *   ⚠️ 调用点必须在 bsp_ws2812_init() + bsp_ws2812_enable(true) 之后，否则 led_strip 还是空。
 *   ⚠️ 上一轮这段**根本没执行**：唤醒来源被误报成 ESP_SLEEP_WAKEUP_GPIO，
 *      而这里（以及 WAKE1 取证）都挂在 `cause == ESP_SLEEP_WAKEUP_EXT1` 上。
 *      修掉那个误武装的唤醒源之后，这段才会真正跑到。 */
static void auto_off_wake_indication(void)
{
    if (!bsp_ws2812_is_enable() || led_strip == NULL) {
        return;
    }
    light_progress_set_flash_overlay(true);
    auto_off_breath_once(16, 22, 22, 90); /*!< 约 0.35s 渐亮 + 0.09s 保持 + 约 0.48s 渐暗 ≈ 0.9s */
    light_progress_set_flash_overlay(false);
    refresh_key_colors_from_status(); /*!< 把灯交还给常规渲染（充电指示/按键灯效） */
}

/*!< 进入深度睡眠；唤醒源 = 两个按键拉低（ext1, ANY_LOW）。
 *
 *   ⚠️ ESP32-S3 上**没有** SOC_GPIO_SUPPORT_DEEPSLEEP_WAKEUP ——
 *   esp_deep_sleep_enable_gpio_wakeup() 那个 API 只有 C2/C3/C5/C6/C61/H4/P4 有，
 *   在 S3 上连声明都进不来（见 esp_sleep.h 的平台条件编译）。S3 只能走 ext0/ext1。
 *   GPIO0 与 GPIO17 都在 S3 的 RTC 域（0~21）内。
 *
 *   ⚠️ 按键的上拉是**软件内部上拉**（components/keyboard_button/src/kbd_gpio.c 的
 *   GPIO_PULLUP_ENABLE），休眠期 RTC_PERIPH 断电后必须靠 RTC IO 的上拉（+ HOLD 特性）
 *   维持高电平；否则引脚浮空会被 ext1 判成"已拉低"而立刻唤醒（表现为"睡下马上就醒"）。 */
static void power_enter_deep_sleep(void)
{
    ESP_LOGW(TAG, "无主机连接已超过 %u 分钟：闪灯提示后进入深度睡眠（按任一键唤醒）",
             (unsigned)g_auto_off_min);

    auto_off_blink_warning();

    /*!< 取证：必须在 rtc_gpio_init() 把引脚切到 RTC 复用**之前**采，
     *   否则 gpio_get_level() 读的是已断开的数字输入。 */
    s_ds_pre_lvl0    = gpio_get_level(GPIO_NUM_0);
    s_ds_pre_lvl17   = gpio_get_level(GPIO_NUM_17);
    s_ds_enter_epoch = (int64_t)time(NULL);
    s_ds_seq++;

    /*!< ---- 【本轮核心修复①】关掉"GPIO 唤醒源" ----
     *   app_main 里为 light sleep 调过一次 esp_sleep_enable_gpio_wakeup()，
     *   而 IDF 的实现（esp_hw_support/sleep_modes.c:2191）**没有任何平台条件编译**：
     *       s_config.wakeup_triggers |= RTC_GPIO_TRIG_EN;   // 无条件
     *       return ESP_OK;                                  // S3 上也照样 OK
     *   启动日志里 `gpio_wakeup=ESP_OK` 就是它在 S3 上"成功"武装的证据。
     *   而 esp_sleep_get_wakeup_cause()（:2300）里 **RTC_GPIO_TRIG_EN 的判断排在
     *   EXT1 之前** —— 于是深睡醒来时 cause 报的是 ESP_SLEEP_WAKEUP_GPIO，不是 EXT1。
     *   一个根因同时解释了两个现场现象：
     *     · 没按任何按键却自己醒了（S3 的 GPIO 深睡唤醒本就不支持，
     *       SOC_GPIO_SUPPORT_DEEPSLEEP_WAKEUP 未定义，这个位属于被误武装）；
     *     · WAKE1 取证与唤醒灯效**从不执行**（它们挂在 cause==EXT1 上）。
     *   深睡前必须显式关掉，让 EXT1 成为唯一唤醒源。 */
    esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_GPIO);

    /*!< ---- 【本轮核心修复②】保活 RTC_PERIPH 电源域 ----
     *   RTC IO 的上下拉属于 RTC_PERIPH 域。IDF 只对 EXT0/GPIO 唤醒自动保活它
     *   （sleep_modes.c:2624-2630；官方 ext_wakeup.c:26-28 的注释原文：
     *    "No need to keep that power domain explicitly, **unlike EXT1**"），
     *   EXT1 不在保活名单里 → 深睡时该域掉电 → 上拉消失、引脚浮空被判低。
     *   代价：深睡电流增加几十 uA。相对"能可靠睡下去、不被假唤醒打断"完全值得。 */
    esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_ON);

    /*!< ---- 按键上拉改走 RTC IO ----
     *   休眠期数字域断电，只有 RTC 上拉能维持高电平。
     *   ⚠️ 必须显式 rtc_gpio_set_direction(INPUT_ONLY)：rtc_gpio_init() 只切功能
     *   （rtc_io.c:54 function_select(RTC)），**不开输入通路**；上一轮的 SLEEP# 取证
     *   就是漏了这一步，第一次读回来的 rtc(0,0) 是无效值（第二次读对了，因为
     *   IDF 的 ext1_wakeup_prepare() 已经点过 input_enable 且该位在 RTC 域留存）。 */
    rtc_gpio_init(GPIO_NUM_0);
    rtc_gpio_set_direction(GPIO_NUM_0, RTC_GPIO_MODE_INPUT_ONLY);
    rtc_gpio_pulldown_dis(GPIO_NUM_0);
    rtc_gpio_pullup_en(GPIO_NUM_0);
    rtc_gpio_init(GPIO_NUM_17);
    rtc_gpio_set_direction(GPIO_NUM_17, RTC_GPIO_MODE_INPUT_ONLY);
    rtc_gpio_pulldown_dis(GPIO_NUM_17);
    rtc_gpio_pullup_en(GPIO_NUM_17);

    /*!< 等上拉把焊盘电容充上去，再读数 —— 这样 SLEEP# 里的电平才是可信的。 */
    vTaskDelay(pdMS_TO_TICKS(30));

    const uint64_t  wake_mask = (1ULL << GPIO_NUM_0) | (1ULL << GPIO_NUM_17);
    const esp_err_t err       = esp_sleep_enable_ext1_wakeup_io(wake_mask, ESP_EXT1_WAKEUP_ANY_LOW);
    if (err != ESP_OK) {
        /*!< 不 abort：即使按键唤醒没配上，拨档经过中间档会断电，等于重新上电，
         *   设备不会变砖。日志留证据即可。 */
        ESP_LOGW(TAG, "ext1 唤醒配置失败 (%s)，将只能靠拨档重新上电唤醒", esp_err_to_name(err));
    }

    /*!< 把"入睡瞬间"的状态也落 NVS：假唤醒会让设备重启，而蓝牙档没有控制台 ——
     *   这一行是唯一能证明"睡下去时引脚到底是高还是低"的证据。
     *   rtc(lvl…) 是**配好 RTC 上拉、并使能输入通路、且延时之后**读的，值可信。 */
    diag_log_event("SLEEP#%u pre(lvl0=%d lvl17=%d) rtc(lvl0=%u lvl17=%u) mask=0x%llx err=%d",
                   (unsigned)s_ds_seq, s_ds_pre_lvl0, s_ds_pre_lvl17,
                   (unsigned)rtc_gpio_get_level(GPIO_NUM_0), (unsigned)rtc_gpio_get_level(GPIO_NUM_17),
                   (unsigned long long)wake_mask, (int)err);

    vTaskDelay(pdMS_TO_TICKS(50)); /*!< 让最后两条日志落出去 */

    esp_deep_sleep_start();
    __builtin_unreachable();
}

/*!< 空闲判定：只有"既没有主机连接、也没插 USB"才算空闲。
 *
 *   计数按 device_status_task 的循环走（约 1s 一次）；连接建立或插入 USB 都会清零。
 *   **按键不重置计数**：这一档没连主机时按键本身不产生任何动作，按了也不代表在
 *   使用设备，重置只会让"一直按着玩"永远不睡 —— 恰好背离要解决的问题。
 *   （若将来想要"按键也算活动"，在这里加一个 btn_progress_has_pressed_key() 分支即可。） */
static void auto_off_tick(void)
{
    if (g_auto_off_min == 0) { /*!< 网页里填 0 = 关闭 */
        s_auto_off_idle_s = 0;
        return;
    }
    if (g_connect_status == 2 || g_usb_connected) {
        s_auto_off_idle_s = 0;
        return;
    }
    s_auto_off_idle_s++;
    if (s_auto_off_idle_s >= (uint32_t)g_auto_off_min * 60u) {
        power_enter_deep_sleep();
    }
}

/*!< 跨档自证。
 *
 *   为什么需要它（2026-09-14 实机复现"拨到蓝牙档就每隔几秒闪一次开机灯效"）：
 *   light sleep 生效后 adc_oneshot_read() 会给出无意义值 —— 蓝牙档被读成"两路
 *   都低于阈值"的中间档，而中间档与蓝牙档跨过了 dip_switch_wifi_enabled 的边界
 *   → 触发 esp_restart()。重启后 light sleep 立刻再次生效 → 误读重现 → 数秒后
 *   又重启，形成死循环。去抖（连续 3 次一致）挡不住，因为误读是持续性的而非偶发。
 *
 *   所以在真正重启前先自证：关掉 light sleep、等 ADC 恢复，重建档位通道，再用与
 *   开机相同的 8 点平均复读。只有复读仍指向候选档位，才承认用户真的拨了档。
 *   非蓝牙档（未开 light sleep）读数可信，直接采信。 */
static bool dip_switch_verify_crossing(adc_oneshot_unit_handle_t handle, int prev_pos, int cand)
{
    /*!< ⚠️ 这里特意不用 g_light_sleep_on 做前置判断：它在 adc_switch_task 创建之后
     *   才被赋值（esp_pm_configure 在 app_main 很靠后），存在竞态 —— 一旦取到 false
     *   就会直接放行一次误读重启。跨档判定本身极少发生，多花 ~2s 换取确定性是划算的。 */
    const bool was_on = g_light_sleep_on;

    esp_pm_config_t pm_no_ls = {
        .max_freq_mhz       = CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ,
        .min_freq_mhz       = CONFIG_XTAL_FREQ,
        .light_sleep_enable = false,
    };
    const esp_err_t off_ret = esp_pm_configure(&pm_no_ls);
    vTaskDelay(pdMS_TO_TICKS(DIP_VERIFY_SETTLE_MS));

    /*!< light sleep 会切断数字外设电源，唤醒后 ADC 通道配置可能已经失效 —— 这正是
     *   误读的成因。bsp_adc_switch_init() 是幂等的（unit 已存在时只重配通道），
     *   重调一次按原参数重建两个档位通道，然后才复读。 */
    bsp_adc_switch_init();
    vTaskDelay(pdMS_TO_TICKS(200));

    /*!< 连续复读 DIP_VERIFY_ROUNDS 轮（每轮内部已经是 8 点平均），任何一轮不指向
     *   候选档位就否决。单轮巧合容易骗过判定，连续多轮同时巧合的概率可以忽略。 */
    int  recheck   = -1;
    bool confirmed = true;
    for (int i = 0; i < DIP_VERIFY_ROUNDS; i++) {
        recheck = dip_switch_read_position(handle);
        if (recheck != cand) {
            confirmed = false;
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(200));
    }
    diag_boot_note_verify(prev_pos, cand, recheck, confirmed);

    ESP_LOGW(TAG, "cross-position verify: cand=%d recheck=%d -> %s (pm_off=%s, was_on=%d)", cand, recheck,
             confirmed ? "CONFIRMED" : "FAKE(misread)", esp_err_to_name(off_ret), (int)was_on);

    if (confirmed) {
        return true; /*!< 即将重启，无需恢复休眠设置 */
    }

    if (was_on) {
        esp_pm_config_t pm_ls = {
            .max_freq_mhz       = CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ,
            .min_freq_mhz       = CONFIG_XTAL_FREQ,
            .light_sleep_enable = true,
        };
        esp_pm_configure(&pm_ls);
    }
    return false;
}

void adc_switch_task(void *pvParameters)
{
    adc_oneshot_unit_handle_t handle = (adc_oneshot_unit_handle_t)pvParameters;
    int adc_value[2]                 = {0};
    sys_param_t *sys_param           = settings_get_parameter();
    btn_report_type_t report_type    = sys_param->report_type;
    report_type                      = ALL_REPORT;  // 默认打印按键 不report kb功能
    int last_pos                     = switch_pos;  // 开机时已判定, 避免上电误判为档位变化
    int cand_pos                     = -1;          /*!< 去抖：候选新档位 */
    int cand_hits                    = 0;           /*!< 候选档位连续命中次数 */
    int64_t cooldown_until_ms        = 0;           /*!< 自证否决后的静默截止时刻 */

    while (1) {
        esp_err_t r0 = adc_oneshot_read(handle, KBD_ADC_SWITCH_BLE_CHAN, &adc_value[0]);
        esp_err_t r1 = adc_oneshot_read(handle, KBD_ADC_SWITCH_RAINMAKER_CHAN, &adc_value[1]);
        g_device_status.switch_1_value = adc_value[0];
        g_device_status.switch_2_value = adc_value[1];

        /*!< 读取失败的样本一律丢弃：light sleep 下 ADC 偶发失败，而失败时 adc_value
         *   会保留上一次的值，若照旧参与判档就会把 BLE 档误判成中间档并触发重启。
         *   详见 DIP_SWITCH_DEBOUNCE_HITS 的说明。 */
        int new_pos = -1;
        if (r0 == ESP_OK && r1 == ESP_OK) {
            if (adc_value[0] > DIP_SWITCH_ADC_THRESHOLD) {
                // report_type = BLE_HID_REPORT;
                new_pos = DIP_SWITCH_POS_BLE;
            } else if (adc_value[1] > DIP_SWITCH_ADC_THRESHOLD) {
                // report_type = USB_CDC_REPORT;
                new_pos = DIP_SWITCH_POS_WIFI;
            } else {
                // report_type = TINYUSB_HID_REPORT;
                new_pos = DIP_SWITCH_POS_CENTER;
            }
        }

        /*!< 静默期内不做判定：自证否决后下一轮必然还是同样的误读，
         *   重复走一遍"关休眠-复读"只会白费电。 */
        if (new_pos >= 0 && (esp_timer_get_time() / 1000) >= cooldown_until_ms) {
            if (new_pos == last_pos) {
                cand_pos  = -1;
                cand_hits = 0;
            } else if (new_pos == cand_pos) {
                cand_hits++;
            } else {
                cand_pos  = new_pos;
                cand_hits = 1;
            }

            if (cand_hits >= DIP_SWITCH_DEBOUNCE_HITS) {
                const int hits = cand_hits;
                cand_pos   = -1;
                cand_hits  = 0;
                switch_pos = new_pos;

                ESP_LOGW(TAG, "DIP switch position: %d -> %d (hits=%d, adc raw %d/%d, rc %d/%d)", last_pos,
                         switch_pos, hits, adc_value[0], adc_value[1], (int)r0, (int)r1);
                /*!< 取证：不论最终是否重启，先把这次判定背后的原始样本存进 RTC 内存。 */
                diag_boot_note_dip_sample((int)r0, (int)r1, adc_value[0], adc_value[1], last_pos, new_pos, hits);
#if DIP_SWITCH_RESTART_ON_BOUNDARY_CHANGE
                /*!< ⚠️ 读到 CENTER 一律忽略，绝不据此重启。
                 *
                 *   CENTER 是"两路 ADC 都没有超过阈值"反推出来的（absence 证据），
                 *   而任何 ADC 失效 —— 通道配置被休眠清掉、采样发生在唤醒瞬间等 ——
                 *   都会得到这个结果。更要命的是：**中间 OFF 档会切断供电**。实测用户
                 *   从 BLE 档拨到 WiFi 档后，本次启动的 reset reason 是 POWERON 而不是
                 *   SW，说明拨档过程必然经历一次真实断电重启 —— 于是运行期读到 CENTER
                 *   永远只可能是误读（或拨档途中一闪而过，反正马上就会断电）。
                 *   据此重启毫无意义，只会把一次 ADC 抽风放大成"每秒闪一次开机灯效"的
                 *   重启循环（2026-09-14 实机现象）。 */
                if (new_pos == DIP_SWITCH_POS_CENTER) {
                    ESP_LOGW(TAG, "DIP reads CENTER (adc raw %d/%d) - absence-based, ignored", adc_value[0],
                             adc_value[1]);
                    switch_pos        = last_pos;
                    cooldown_until_ms = (esp_timer_get_time() / 1000) + DIP_VERIFY_COOLDOWN_MS;
                } else if (dip_switch_wifi_enabled(last_pos) != dip_switch_wifi_enabled(switch_pos)) {
                    if (!dip_switch_verify_crossing(handle, last_pos, new_pos)) {
                        /*!< 复读推翻了这次判定 -> 判为休眠误读，撤销并静默一段时间。 */
                        ESP_LOGW(TAG, "DIP change rejected after re-verify (light-sleep misread), cooldown %d s",
                                 DIP_VERIFY_COOLDOWN_MS / 1000);
                        switch_pos        = last_pos;
                        cooldown_until_ms = (esp_timer_get_time() / 1000) + DIP_VERIFY_COOLDOWN_MS;
                    } else if (diag_boot_is_restart_storm()) {
                        /*!< 短时间内反复软复位 = 误读循环。熔断：宁可放弃这次切档，
                         *   也不能让设备一直重启（重启期间用户根本没法用）。 */
                        ESP_LOGE(TAG, "restart storm detected -> suppressing cross-position restart");
                        diag_boot_note_storm_suppressed();
                        switch_pos        = last_pos;
                        cooldown_until_ms = (esp_timer_get_time() / 1000) + DIP_VERIFY_COOLDOWN_MS;
                    } else {
                        ESP_LOGW(TAG, "DIP switch crossed BLE/WiFi boundary, restarting to apply");
                        /*!< 重启会清空 RAM 里的统计，跨档前必须先把诊断数据落盘到 NVS，
                         *   否则刚从 BLE 档跑出来的那一段数据就白测了。 */
                        diag_boot_note_confirmed_restart();
                        diag_power_flush();
                        vTaskDelay(pdMS_TO_TICKS(100));
                        esp_restart();
                    }
                }
#endif
                last_pos = switch_pos;
            }
        }

        // ESP_LOGI(TAG, "adc_value[0]: %d, adc_value[1]: %d", g_device_status.switch_1_value,
        // g_device_status.switch_2_value);

        if (report_type != sys_param->report_type) {
            sys_param->report_type = report_type;
            settings_write_parameter_to_nvs();

            // 拨码开关位置改变时，发送立即刷新指令
            if (status_refresh_queue) {
                status_refresh_type_t refresh_msg = STATUS_REFRESH_IMMEDIATE;
                xQueueSend(status_refresh_queue, &refresh_msg, 0);
                ESP_LOGI(TAG, "Sent immediate status refresh request due to DIP switch change");
            }

            // esp_restart();
        }

        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }

    vTaskDelete(NULL);
}

/**
 * @brief Press and hold for 4 seconds to start blinking.
 *        Release between 4s-7s to enter pairing/reset WiFi mode.
 *        If held for more than 7 seconds, ignore this long press.
 */
static void button_event_cb(void *button_handle, void *usr_data)
{
    button_handle_t btn  = (button_handle_t)button_handle;
    button_event_t event = iot_button_get_event(btn);
    uint32_t ticks       = iot_button_get_ticks_time(btn);
    if (event == BUTTON_LONG_PRESS_HOLD) {
        if (ticks >= 4000 && ticks <= 7000) {
            ESP_LOGI(TAG, "button_long_press_hold");
            update_power_status(led_strip);

            vTaskDelay(1000 / portTICK_PERIOD_MS);
        }

    } else if (event == BUTTON_LONG_PRESS_UP) {
        if (ticks >= 4000 && ticks <= 7000) {
            // 重置WiFi配置并重启
            ESP_LOGI(TAG, "重置WiFi配置");
            esp_err_t ret = wifi_config_reset();
            if (ret == ESP_OK) {
                ESP_LOGI(TAG, "WiFi配置已重置，即将重启...");
                // 闪烁LED指示成功
                for (int i = 0; i < 3; i++) {
                    led_strip_set_pixel(led_strip, 0, 255, 0, 0);
                    led_strip_set_pixel(led_strip, 1, 255, 0, 0);
                    led_strip_refresh(led_strip);
                    vTaskDelay(200 / portTICK_PERIOD_MS);
                    led_strip_clear(led_strip);
                    vTaskDelay(200 / portTICK_PERIOD_MS);
                }
                // 重启设备
                esp_restart();
            } else {
                ESP_LOGE(TAG, "WiFi配置重置失败");
            }
            rgb_matrix_set_suspend_state(false);
            iot_button_stop();
        }
        ESP_LOGI(TAG, "button_long_press_up");
    } else if (event == BUTTON_LONG_PRESS_START) {
        if (ticks >= 6900) {
            rgb_matrix_set_suspend_state(false);
            iot_button_stop();
        } else {
            rgb_matrix_set_suspend_state(true);
        }
        ESP_LOGI(TAG, "button_press_start");
    }
}

void app_dual_button_init(void)
{
    /*!< Init reset/reconfig event */
    const button_config_t btn_cfg = {
        .long_press_time  = 4000,
        .short_press_time = 0,
    };
    button_dual_config_t dual_cfg = {
        .active_level   = KBD_ATTIVE_LEVEL,
        .gpio_num       = KBD_INPUT_IOS,
        .disable_pull   = 0,
        .skip_gpio_init = true,
    };
    button_handle_t btn = NULL;
    iot_button_new_dual_button_device(&btn_cfg, &dual_cfg, &btn);
    iot_button_register_cb(btn, BUTTON_LONG_PRESS_START, NULL, button_event_cb, NULL);
    iot_button_register_cb(btn, BUTTON_LONG_PRESS_HOLD, NULL, button_event_cb, NULL);
    iot_button_register_cb(btn, BUTTON_LONG_PRESS_UP, NULL, button_event_cb, NULL);
    button_event_args_t btn_event_cfg{};
    btn_event_cfg.long_press.press_time = 7000;
    iot_button_register_cb(btn, BUTTON_LONG_PRESS_START, &btn_event_cfg, button_event_cb, NULL);
}

// 计算DualKey配置的CRC32
static uint32_t dualkey_config_calculate_crc(const dualkey_saved_config_t *config)
{
    return esp_crc32_le(0, (const uint8_t *)config, sizeof(dualkey_saved_config_t) - sizeof(uint32_t));
}

// 保存DualKey配置到NVS
static esp_err_t dualkey_config_save(void)
{
    nvs_handle_t nvs_handle;
    esp_err_t ret = nvs_open("dualkey_cfg", NVS_READWRITE, &nvs_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "打开DualKey配置NVS失败: %s", esp_err_to_name(ret));
        return ret;
    }

    dualkey_saved_config_t config;
    memset(&config, 0, sizeof(config));

    // 填充配置数据
    config.left_key_color         = g_device_status.left_key_color;
    config.right_key_color        = g_device_status.right_key_color;
    config.current_key_mapping    = g_device_status.current_key_mapping;
    config.usb_mapping_enabled    = g_usb_mapping_enabled;
    config.ble_mapping_enabled    = g_ble_mapping_enabled;
    config.custom_mapping_enabled = btn_progress_is_custom_mapping_enabled();
    memcpy(&config.custom_left_action, btn_progress_get_custom_left_action(), sizeof(custom_key_action_t));
    memcpy(&config.custom_right_action, btn_progress_get_custom_right_action(), sizeof(custom_key_action_t));
    config.crc32 = dualkey_config_calculate_crc(&config);

    // 保存到NVS
    ret = nvs_set_blob(nvs_handle, "config", &config, sizeof(config));
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "保存DualKey配置失败: %s", esp_err_to_name(ret));
        nvs_close(nvs_handle);
        return ret;
    }

    ret = nvs_commit(nvs_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "提交DualKey配置失败: %s", esp_err_to_name(ret));
        nvs_close(nvs_handle);
        return ret;
    }

    nvs_close(nvs_handle);
    ESP_LOGI(TAG, "DualKey配置已保存: 左键颜色=0x%06lX, 右键颜色=0x%06lX, 映射=%d", config.left_key_color,
             config.right_key_color, config.current_key_mapping);

    // 长按配置使用独立的 NVS key，随配置一起落盘
    dualkey_longpress_config_save();

    return ESP_OK;
}

// 从NVS加载DualKey配置
static esp_err_t dualkey_config_load(void)
{
    nvs_handle_t nvs_handle;
    esp_err_t ret = nvs_open("dualkey_cfg", NVS_READONLY, &nvs_handle);
    if (ret != ESP_OK) {
        if (ret == ESP_ERR_NVS_NOT_FOUND) {
            ESP_LOGI(TAG, "DualKey配置未找到，使用默认配置");
            return ESP_ERR_NOT_FOUND;
        }
        ESP_LOGE(TAG, "打开DualKey配置NVS失败: %s", esp_err_to_name(ret));
        return ret;
    }

    dualkey_saved_config_t config;
    size_t required_size = sizeof(config);

    ret = nvs_get_blob(nvs_handle, "config", &config, &required_size);
    if (ret == ESP_ERR_NVS_NOT_FOUND) {
        ESP_LOGI(TAG, "DualKey配置未找到，使用默认配置");
        nvs_close(nvs_handle);
        return ESP_ERR_NOT_FOUND;
    }

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "读取DualKey配置失败: %s", esp_err_to_name(ret));
        nvs_close(nvs_handle);
        return ret;
    }

    // 验证CRC
    uint32_t calculated_crc = dualkey_config_calculate_crc(&config);
    if (calculated_crc != config.crc32) {
        ESP_LOGE(TAG, "DualKey配置CRC校验失败");
        nvs_close(nvs_handle);
        return ESP_ERR_INVALID_CRC;
    }

    // 应用配置
    g_device_status.left_key_color      = config.left_key_color;
    g_device_status.right_key_color     = config.right_key_color;
    g_device_status.current_key_mapping = config.current_key_mapping;
    g_usb_mapping_enabled               = config.usb_mapping_enabled;
    g_ble_mapping_enabled               = config.ble_mapping_enabled;

    // 应用按键映射
    btn_progress_set_key_mapping(config.current_key_mapping);

    // 应用自定义映射
    btn_progress_enable_custom_mapping(config.custom_mapping_enabled);
    btn_progress_set_custom_left_action(&config.custom_left_action);
    btn_progress_set_custom_right_action(&config.custom_right_action);

    // 应用长按配置（独立 NVS key；不存在时保持默认，即所有按键都不做长按判定）
    dualkey_longpress_config_load();

    // 应用RGB颜色
    vTaskDelay(10 / portTICK_PERIOD_MS);
    if (config.left_key_color != 0x000000 || config.right_key_color != 0x000000) {
        if (rgb_matrix_get_suspend_state() == false) {
            rgb_matrix_set_suspend_state(true);
            vTaskDelay(100 / portTICK_PERIOD_MS);
        }
        // 成对原子写入；连写两遍躲避 rgb_matrix 可能的一次性清屏
        apply_key_colors(config.left_key_color, config.right_key_color);
        vTaskDelay(20 / portTICK_PERIOD_MS);
        apply_key_colors(config.left_key_color, config.right_key_color);
    }

    nvs_close(nvs_handle);
    ESP_LOGI(TAG, "DualKey配置已加载: 左键颜色=0x%06lX, 右键颜色=0x%06lX, 映射=%d", config.left_key_color,
             config.right_key_color, config.current_key_mapping);

    return ESP_OK;
}

// 计算长按配置的CRC32
static uint32_t dualkey_longpress_calculate_crc(const dualkey_longpress_saved_t *config)
{
    return esp_crc32_le(0, (const uint8_t *)config, sizeof(dualkey_longpress_saved_t) - sizeof(uint32_t));
}

// 保存长按配置到NVS（独立 key "longpress"）
static esp_err_t dualkey_longpress_config_save(void)
{
    nvs_handle_t nvs_handle;
    esp_err_t ret = nvs_open("dualkey_cfg", NVS_READWRITE, &nvs_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "打开DualKey配置NVS失败: %s", esp_err_to_name(ret));
        return ret;
    }

    dualkey_longpress_saved_t config;
    memset(&config, 0, sizeof(config));
    memcpy(&config.left_long_action, btn_progress_get_custom_long_left_action(), sizeof(custom_key_action_t));
    memcpy(&config.right_long_action, btn_progress_get_custom_long_right_action(), sizeof(custom_key_action_t));
    config.long_press_ms = btn_progress_get_long_press_ms();
    config.crc32         = dualkey_longpress_calculate_crc(&config);

    ret = nvs_set_blob(nvs_handle, "longpress", &config, sizeof(config));
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "保存长按配置失败: %s", esp_err_to_name(ret));
        nvs_close(nvs_handle);
        return ret;
    }

    ret = nvs_commit(nvs_handle);
    nvs_close(nvs_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "提交长按配置失败: %s", esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(TAG, "长按配置已保存: 左type=%d 右type=%d 阈值=%ums", config.left_long_action.type,
             config.right_long_action.type, (unsigned)config.long_press_ms);

    return ESP_OK;
}

// 从NVS加载长按配置。不存在或校验失败时不做长按判定（保持默认），不影响主配置
static esp_err_t dualkey_longpress_config_load(void)
{
    nvs_handle_t nvs_handle;
    esp_err_t ret = nvs_open("dualkey_cfg", NVS_READONLY, &nvs_handle);
    if (ret != ESP_OK) {
        return ret;
    }

    dualkey_longpress_saved_t config;
    size_t required_size = sizeof(config);

    ret = nvs_get_blob(nvs_handle, "longpress", &config, &required_size);
    if (ret != ESP_OK) {
        nvs_close(nvs_handle);
        if (ret == ESP_ERR_NVS_NOT_FOUND) {
            ESP_LOGI(TAG, "长按配置未找到，所有按键均不做长按判定");
        } else {
            ESP_LOGW(TAG, "读取长按配置失败: %s", esp_err_to_name(ret));
        }
        return ret;
    }

    if (required_size != sizeof(config) || dualkey_longpress_calculate_crc(&config) != config.crc32) {
        nvs_close(nvs_handle);
        ESP_LOGW(TAG, "长按配置长度或CRC校验失败，忽略该配置");
        return ESP_ERR_INVALID_CRC;
    }

    btn_progress_set_custom_long_left_action(&config.left_long_action);
    btn_progress_set_custom_long_right_action(&config.right_long_action);
    btn_progress_set_long_press_ms(config.long_press_ms);

    nvs_close(nvs_handle);
    ESP_LOGI(TAG, "长按配置已加载: 左type=%d 右type=%d 阈值=%ums", config.left_long_action.type,
             config.right_long_action.type, (unsigned)config.long_press_ms);

    return ESP_OK;
}

// 计算WiFi配置的CRC32
static uint32_t wifi_config_calculate_crc(const wifi_config_saved_t *config)
{
    return esp_crc32_le(0, (const uint8_t *)config, sizeof(wifi_config_saved_t) - sizeof(uint32_t));
}

// 保存WiFi配置到NVS
static esp_err_t wifi_config_save(const wifi_config_saved_t *config)
{
    nvs_handle_t nvs_handle;
    esp_err_t ret = nvs_open("wifi_cfg", NVS_READWRITE, &nvs_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "打开WiFi配置NVS失败: %s", esp_err_to_name(ret));
        return ret;
    }

    wifi_config_saved_t cfg_to_save;
    memcpy(&cfg_to_save, config, sizeof(wifi_config_saved_t));
    cfg_to_save.crc32 = wifi_config_calculate_crc(&cfg_to_save);

    // 保存到NVS
    ret = nvs_set_blob(nvs_handle, "config", &cfg_to_save, sizeof(cfg_to_save));
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "保存WiFi配置失败: %s", esp_err_to_name(ret));
        nvs_close(nvs_handle);
        return ret;
    }

    ret = nvs_commit(nvs_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "提交WiFi配置失败: %s", esp_err_to_name(ret));
        nvs_close(nvs_handle);
        return ret;
    }

    nvs_close(nvs_handle);
    ESP_LOGI(TAG, "WiFi配置已保存: SSID=%s, 静态IP=%s", cfg_to_save.ssid, cfg_to_save.use_static_ip ? "启用" : "禁用");

    return ESP_OK;
}

// 从NVS加载WiFi配置
static esp_err_t wifi_config_load(wifi_config_saved_t *config)
{
    nvs_handle_t nvs_handle;
    esp_err_t ret = nvs_open("wifi_cfg", NVS_READONLY, &nvs_handle);
    if (ret != ESP_OK) {
        if (ret == ESP_ERR_NVS_NOT_FOUND) {
            ESP_LOGI(TAG, "WiFi配置未找到");
            return ESP_ERR_NOT_FOUND;
        }
        ESP_LOGE(TAG, "打开WiFi配置NVS失败: %s", esp_err_to_name(ret));
        return ret;
    }

    size_t required_size = sizeof(wifi_config_saved_t);

    ret = nvs_get_blob(nvs_handle, "config", config, &required_size);
    if (ret == ESP_ERR_NVS_NOT_FOUND) {
        ESP_LOGI(TAG, "WiFi配置未找到");
        nvs_close(nvs_handle);
        return ESP_ERR_NOT_FOUND;
    }

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "读取WiFi配置失败: %s", esp_err_to_name(ret));
        nvs_close(nvs_handle);
        return ret;
    }

    // 验证CRC
    uint32_t calculated_crc = wifi_config_calculate_crc(config);
    if (calculated_crc != config->crc32) {
        ESP_LOGE(TAG, "WiFi配置CRC校验失败");
        nvs_close(nvs_handle);
        return ESP_ERR_INVALID_CRC;
    }

    nvs_close(nvs_handle);
    ESP_LOGI(TAG, "WiFi配置已加载: SSID=%s", config->ssid);

    return ESP_OK;
}

// 重置WiFi配置
static esp_err_t wifi_config_reset(void)
{
    nvs_handle_t nvs_handle;
    esp_err_t ret = nvs_open("wifi_cfg", NVS_READWRITE, &nvs_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "打开WiFi配置NVS失败: %s", esp_err_to_name(ret));
        return ret;
    }

    // 删除配置
    ret = nvs_erase_key(nvs_handle, "config");
    if (ret != ESP_OK && ret != ESP_ERR_NVS_NOT_FOUND) {
        ESP_LOGE(TAG, "删除WiFi配置失败: %s", esp_err_to_name(ret));
        nvs_close(nvs_handle);
        return ret;
    }

    ret = nvs_commit(nvs_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "提交WiFi配置删除失败: %s", esp_err_to_name(ret));
        nvs_close(nvs_handle);
        return ret;
    }

    nvs_close(nvs_handle);
    ESP_LOGI(TAG, "WiFi配置已重置");

    return ESP_OK;
}

void app_main(void)
{
    // 创建状态刷新消息队列
    status_refresh_queue = xQueueCreate(10, sizeof(status_refresh_type_t));
    if (status_refresh_queue == NULL) {
        ESP_LOGE(TAG, "Failed to create status refresh queue");
    } else {
        ESP_LOGI(TAG, "Status refresh queue created successfully");
    }

    // Initialize NVS.
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    /*!< Read System config */
    settings_read_parameter_from_nvs();
    sys_param = settings_get_parameter();

    /*!< 自动关机超时（独立 NVS namespace，见文件头部常量区）。
     *   三档都读：虽然只有蓝牙档会用它，但只读一次的成本可以忽略，
     *   而且这样以后要在别的档位启用不需要再动这里。 */
    auto_off_config_load();

    /*!< 唤醒原因取证：区分"冷启动"与"从自动关机深睡被唤醒"。
     *   蓝牙档没有控制台，NVS 是唯一的事后通道；所以这里**无条件**落一行 BOOTRAW ——
     *   上一轮只留下 reason=8，看不出唤醒源到底是谁（WAKE1 从未落盘），
     *   这轮把 cause / reset reason / 全部唤醒位 / ext1 掩码 / 睡眠时长一次记全。 */
    bool woke_from_deep_sleep = false;
    {
        const esp_sleep_wakeup_cause_t cause  = esp_sleep_get_wakeup_cause();
        const esp_reset_reason_t       rst    = esp_reset_reason();
        const uint32_t                 causes = esp_sleep_get_wakeup_causes();
        const uint64_t                 st =
            (cause == ESP_SLEEP_WAKEUP_EXT1) ? esp_sleep_get_ext1_wakeup_status() : 0;
        /*!< 墙钟由 RTC 计时器提供，跨深睡连续，可直接作差（CONFIG_ESP_TIME_FUNCS_USE_RTC_TIMER=y）。 */
        const int64_t now_epoch = (int64_t)time(NULL);
        const int64_t slept_s   = (s_ds_enter_epoch > 0) ? (now_epoch - s_ds_enter_epoch) : -1;

        diag_log_event("BOOTRAW cause=%d rst=%d causes=0x%lx ext1=0x%llx seq=%u slept=%llds", (int)cause,
                       (int)rst, (unsigned long)causes, (unsigned long long)st, (unsigned)s_ds_seq,
                       (long long)slept_s);

        if (cause == ESP_SLEEP_WAKEUP_EXT1) {
            woke_from_deep_sleep = true;
            ESP_LOGW(TAG, "从深度睡眠唤醒: EXT1, mask=0x%llx (bit0=Key1/GPIO0, bit17=Key2/GPIO17), 睡了 %llds",
                     (unsigned long long)st, (long long)slept_s);
            /*!< ⚠️ 这里用 rtc_gpio_get_level()：引脚此刻仍停在入睡前设的 RTC 复用上，
             *   数字输入是断开的，gpio_get_level() 读不到真实电平。
             *   （后面 KEYMUX 那段会把复用切回数字并复读一次，两边正好互为印证。） */
            diag_log_event("WAKE1 seq=%u ext1=0x%llx slept=%llds pre(lvl0=%d lvl17=%d) now(lvl0=%u lvl17=%u)",
                           (unsigned)s_ds_seq, (unsigned long long)st, (long long)slept_s, s_ds_pre_lvl0,
                           s_ds_pre_lvl17, (unsigned)rtc_gpio_get_level(GPIO_NUM_0),
                           (unsigned)rtc_gpio_get_level(GPIO_NUM_17));
        } else {
            ESP_LOGI(TAG, "启动/唤醒原因: cause=%d rst=%d causes=0x%lx (rst: 1=上电, 8=深睡唤醒)", (int)cause,
                     (int)rst, (unsigned long)causes);
        }
    }

    /*!< Monitor adc switch */
    bsp_adc_switch_init();
    bsp_adc_charge_monitor_init();
    adc_oneshot_unit_handle_t handle = NULL;
    ret                              = bsp_get_adc_handle(&handle);
    assert(ret == ESP_OK);

    /*!< Read the DIP switch before wireless init: BLE position runs BLE only */
    switch_pos = dip_switch_read_position(handle);
    /*!< 取证：记录本次启动原因、开机判定档位，以及**开机瞬间的原始 ADC 采样**。
     *   此刻 light sleep 尚未开启，这份读数是"可信基准"，用于和运行期误判时的 raw
     *   值对比：都掉到 0 = 通道失效；只是跌到阈值以下 = 读数漂移。
     *   同时会落一份到 NVS —— 中间 OFF 档会切断供电，RTC 内存扛不住。 */
    int boot_ble_raw = -1, boot_wifi_raw = -1;
    adc_oneshot_read(handle, KBD_ADC_SWITCH_BLE_CHAN, &boot_ble_raw);
    adc_oneshot_read(handle, KBD_ADC_SWITCH_RAINMAKER_CHAN, &boot_wifi_raw);
    diag_boot_note_boot(switch_pos, boot_ble_raw, boot_wifi_raw);
    /*!< 蓝牙档 = 纯蓝牙翻页器，关掉一切用不上的常驻负载。
     *   开机时判定一次即可：跨越蓝牙档/非蓝牙档边界会 esp_restart()（见 adc_switch_task）。 */
    const bool ble_only_mode = !dip_switch_wifi_enabled(switch_pos);
    /*!< OFF(中间) 档 = 有线档：只启用 USB-HID。
     *   2026-09-15 用户实测定下的三档分工：
     *     中间 OFF 档 → 仅 USB-HID（禁 WiFi、禁蓝牙）
     *     WiFi 档     → WiFi + 蓝牙（禁 USB-HID）
     *     BLE 档      → 仅蓝牙
     *   动机是实测到的"双通道同时上报"：USB-HID 与蓝牙同时连到同一台主机时，一次按键
     *   会被两条通道各送一次（光标跳两格）。只要任一档位保证"同时只有一条通道在工作"，
     *   这个冲突就不可能发生 —— 这比在按键逻辑里做仲裁可靠得多。
     *   OFF 档由 USB 供电（中间档切断电池），所以那一档不需要考虑省电。 */
    const bool usb_only_mode = (switch_pos == DIP_SWITCH_POS_CENTER);
    ESP_LOGI(TAG, "DIP switch position at boot: %d (ble_only=%d usb_only=%d)", switch_pos, (int)ble_only_mode,
             (int)usb_only_mode);

    xTaskCreate(adc_switch_task, "adc_switch_task", 4096, handle, 5, NULL);

    /*!< Init LED and clear WS2812's status */
    bsp_ws2812_init(&led_strip);
    if (led_strip) {
        led_strip_clear(led_strip);
    }

    /*!< Chain 扩展总线：蓝牙档下不启动。
     *   chain_bus_task 每 3s 会对两条总线各做一次 isDeviceConnected()，其内部是
     *   1ms 周期的忙等轮询（50ms 超时 × 3 次重试 = 150ms/总线/次）；未插任何 Chain
     *   子设备时永远等不到回应，每次都会跑满超时。停用后可省掉双 UART 时钟与这部分
     *   空转，也让系统有机会进入更长的空闲。
     *   翻页器场景用不到扩展口；需要时把拨码拨到 WiFi 档即可恢复。 */
    if (ble_only_mode || usb_only_mode) {
        ESP_LOGI(TAG, "Chain 扩展总线不初始化 (只有 WiFi 档保留双 UART / 扫描任务)");
    } else {
        chain_bus_init();
    }

    /* ADC detection test */
    test_adc_detection(led_strip, handle);

    /*!< 创建 HID 发送互斥锁：长按到点触发与按键回调都会发报告，需要串行化 */
    btn_progress_init();

    bsp_rgb_matrix_init();
    bsp_ws2812_enable(true);
    xTaskCreate(light_progress_task, "light_progress_task", 4096, NULL, 5, &light_progress_task_handle);

    /*!< 从自动关机深睡被按键唤醒时给一次灯效反馈（呼吸一次，与睡前的三次区分开）。
     *   放在这里而不是更早：此刻 WS2812 与渲染任务才就绪，led_strip 才有值。 */
    if (woke_from_deep_sleep) {
        auto_off_wake_indication();
    }

    /*!< ---- 【本轮核心修复③】把两个键脚从 RTC 复用里放回数字复用 ----
     *   现象：深睡被唤醒后按键**全部失灵**（无法触发任何映射），而冷启动那一档按键是好的。
     *   机理（源码已核实）：rtc_gpio_init() 只做 function_select(…, RTCIO_LL_FUNC_RTC)
     *   （esp_driver_gpio/src/rtc_io.c:54）把焊盘切到 RTC 功能；IDF 自己的
     *   ext1_wakeup_prepare() 也会做同样的事（sleep_modes.c:2063）。而该选择位位于 RTC 域，
     *   **深睡时 RTC 域不掉电、唤醒后也不复位** → 复位之后键脚仍挂在 RTC 上，数字 GPIO
     *   （也就是 keyboard_button 驱动）根本读不到它 → 按键彻底失灵。
     *   冷启动是上电复位，RTC 域一并复位，所以那一档按键正常 —— 与实测现象完全吻合。
     *   修复：rtc_gpio_deinit()（rtc_io.c:60，切回 RTCIO_LL_FUNC_DIGITAL）。
     *   无条件调用（冷启动时等价于空操作）；顺便把"切之前 / 切之后"的数字电平落盘，
     *   给上面的机理留一份直接证据。 */
    {
        gpio_config_t key_cfg = {
            .pin_bit_mask = (1ULL << GPIO_NUM_0) | (1ULL << GPIO_NUM_17),
            .mode         = GPIO_MODE_INPUT,
            .pull_up_en   = GPIO_PULLUP_ENABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type    = GPIO_INTR_DISABLE,
        };
        gpio_config(&key_cfg);
        const int pre0  = gpio_get_level(GPIO_NUM_0);
        const int pre17 = gpio_get_level(GPIO_NUM_17);
        rtc_gpio_deinit(GPIO_NUM_0);
        rtc_gpio_deinit(GPIO_NUM_17);
        vTaskDelay(pdMS_TO_TICKS(5));
        const int post0  = gpio_get_level(GPIO_NUM_0);
        const int post17 = gpio_get_level(GPIO_NUM_17);
        /*!< pre 为 0 而 post 为 1（键没按着时）就坐实了上面的机理。 */
        diag_log_event("KEYMUX pre(%d,%d) post(%d,%d) rst=%d", pre0, pre17, post0, post17,
                       (int)esp_reset_reason());
    }

    /*!< Init keyboard key monitor */
    bsp_keyboard_init(&kbd_handle, NULL);
    keyboard_btn_cb_config_t cb_cfg = {
        .event    = KBD_EVENT_PRESSED,
        .callback = keyboard_cb,
    };
    keyboard_button_register_cb(kbd_handle, cb_cfg, NULL);
    btn_progress_set_report_type(sys_param->report_type);

    app_dual_button_init();
    /*!< Init report */
    // switch (sys_param->report_type) {
    //     case TINYUSB_HID_REPORT:
    //         tinyusb_hid_init();
    //         break;
    //     case BLE_HID_REPORT:
    //         ble_hid_init();
    //         break;
    //     case USB_CDC_REPORT:
    //         tinyusb_cdc_init(NULL, NULL);
    //         ESP_LOGI(TAG, "USB CDC mode initialized");
    //         break;
    //     default:
    //         break;
    // }

    /*!< WiFi + 网页服务：只有 WiFi 档开启。
     *   蓝牙档跳过是为了省电；OFF 档跳过是因为那一档的职责只是 USB 键盘。
     *   连带效果：WiFi 档不再初始化 TinyUSB（见下方），原生 USB-Serial-JTAG 就能重新
     *   拿到 USB PHY —— 那一档的插线日志因此是可读的，调试时很有用。 */
    if (!ble_only_mode && !usb_only_mode) {
        // 初始化WiFi
        wifi_init_sta();

        // 启动web服务器
        start_webserver();

        // 启动WebSocket状态更新任务
        xTaskCreate(websocket_task, "websocket_task", 1024 * 10, NULL, 5, &websocket_task_handle);
    } else if (ble_only_mode) {
        ESP_LOGI(TAG, "DIP switch is in BLE position: WiFi/HTTP disabled, BLE only");
        // 无网页服务时仍需刷新设备状态, 否则 BLE 电量上报不会触发
        xTaskCreate(device_status_task, "device_status_task", 4096, NULL, 5, NULL);
        /*!< 只在 BLE 档启动功耗采样任务。WiFi 档故意不启动：它每 5min 会把快照
         *   写进 NVS，若在 WiFi 档也跑，切回 BLE 档时就会把刚测到的数据覆盖掉。 */
        diag_power_start();
    } else {
        ESP_LOGI(TAG, "DIP switch is in OFF position: USB-HID only (WiFi/HTTP/BLE disabled)");
    }

    /*!< USB-OTG / TinyUSB：只有 OFF(中间) 档启用，即"有线键盘"形态。
     *   tinyusb_hid_init() 会拉起 dwc2 驱动并使能 USB-OTG PHY，而该驱动在未接主机时
     *   也不会门控 PHY 时钟（dwc2_common.c 清掉 STOPPCLK/GATEHCLK），PHY 会持续耗电；
     *   同时 USB 协议栈要求 48MHz 时钟域常开，会阻止系统进入更低的功耗状态。
     *   WiFi / 蓝牙档因此都不启动它 —— 这两档的键盘要走无线，正是本次要消除的
     *   "USB-HID 与蓝牙同时上报同一台主机"的冲突源。
     *   日志与烧录走原生 USB-Serial-JTAG（PID 303A:1001），与 TinyUSB（303A:8000）
     *   是两条独立通道：不初始化 TinyUSB 时 USJ 就能独占 PHY，插线日志反而可读。 */
    if (usb_only_mode) {
        tinyusb_hid_init();
        ESP_LOGI(TAG, "USB-only: TinyUSB HID 启用 (WiFi / 蓝牙均不启动)");
    } else {
        ESP_LOGI(TAG, "TinyUSB 与 USB-OTG PHY 不初始化 (WiFi/BLE 档禁 USB-HID)");
    }
    // tinyusb_cdc_init(NULL, NULL);
    /*!< 蓝牙 HID：BLE 档 + WiFi 档。OFF 档不启动，避免与 USB-HID 同时向同一台主机上报。 */
    if (!usb_only_mode) {
        ble_hid_init();
    } else {
        ESP_LOGI(TAG, "OFF position: BLE HID 不初始化");
    }

    rgb_matrix_mode(2);

    /*!< Load DualKey config */
    esp_err_t dualkey_ret = dualkey_config_load();
    if (dualkey_ret == ESP_OK) {
        ESP_LOGI(TAG, "DualKey配置加载成功");
    } else if (dualkey_ret == ESP_ERR_NOT_FOUND) {
        ESP_LOGI(TAG, "DualKey配置未找到，使用默认配置");
    } else {
        ESP_LOGW(TAG, "DualKey配置加载失败: %s，使用默认配置", esp_err_to_name(dualkey_ret));
    }

    /*!< 充电电量指示（低亮度常亮）：放在配置加载之后，确保静态色已就绪 */
    xTaskCreate(power_indicator_task, "power_indicator_task", 3072, NULL, 4, NULL);

    /*!< 阶段 3：只对蓝牙档开启自动 light sleep —— 这是把静置电流从 ~45mA 压到个位数
     *   的唯一途径。诊断实测（BLE 连接态静置 119min）：有效 CPU 49.5MHz、各 PM 锁
     *   均正常、而 SLEEP 占比 0%。即硬件没毛病、任务也没乱醒，45~50mA 就是
     *   "CPU 常驻 40MHz + BLE 维持"这条基线的代价 —— 数字域从头到尾没停过。
     *
     *   为什么必须由应用层补这一刀：IDF 的 CONFIG_PM_DFS_INIT_AUTO 确实会自动调用
     *   esp_pm_configure()，但它在构造 esp_pm_config_t 时没有填 light_sleep_enable
     *   （零值 = false），应用层也从未主动调用过它。结果是"DFS 开着、却永远不进休眠"。
     *
     *   Kconfig 侧无需改动，本就已就绪：
     *     - BT_CTRL_MODEM_SLEEP + MODE_1：射频每周期入睡，并释放 bt 的 APB 锁
     *       （该锁若不释放，pm_impl.c:554 会把最低可达模式卡在 APB_MAX）；
     *     - PM_POWER_DOWN_CPU_IN_LIGHT_SLEEP=y：休眠时 CPU 断电，省约 650uA；
     *     - PM_SLP_DISABLE_GPIO=y：休眠时隔离全部 GPIO，省 200~300uA。
     *       用作唤醒的键脚不会被它切断 —— esp_driver_gpio/gpio.c:672-674 在
     *       gpio_wakeup_enable() 内部会自动 gpio_sleep_sel_dis() 予以豁免。
     *
     *   唤醒源：两个键脚的低电平唤醒已由 keyboard_button 组件在 bsp_keyboard_init()
     *   里注册（其内部把 gpio_mode 显式设为 INPUT、enable_power_save 透传 true）；
     *   这里再补一次 esp_sleep_enable_gpio_wakeup() 把该唤醒源挂上。
     *   定时唤醒由 tickless idle 按"下一个到期事件"自动设置，无需手工干预。
     *
     *   只对蓝牙档开启：WiFi 档要保证网页/WebSocket 实时响应，而且读诊断数据也在
     *   那一档，不适合引入休眠。 */
    if (ble_only_mode) {
        const esp_err_t gpio_wk = esp_sleep_enable_gpio_wakeup();
        esp_pm_config_t pm_cfg  = {
            .max_freq_mhz       = CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ,
            .min_freq_mhz       = CONFIG_XTAL_FREQ, /*!< S3 最低档即晶振 40MHz */
            .light_sleep_enable = true,
        };
        const esp_err_t pm_ret = esp_pm_configure(&pm_cfg);
        g_light_sleep_on       = (pm_ret == ESP_OK);
        ESP_LOGI(TAG, "BLE-only: auto light sleep %s (pm=%s gpio_wakeup=%s, DFS %d/%d MHz)",
                 (pm_ret == ESP_OK) ? "ENABLED" : "FAILED", esp_err_to_name(pm_ret), esp_err_to_name(gpio_wk),
                 pm_cfg.max_freq_mhz, pm_cfg.min_freq_mhz);
    }

    static uint8_t last_battery_percentage = 255;
    while (1) {
        if (g_device_status.bluetooth_connected) {
            // 只在电量变化时发送，减少蓝牙通信负担
            if (last_battery_percentage != g_battery_percentage) {
                esp_err_t ret = ble_hid_set_battery(g_battery_percentage);
                if (ret == ESP_OK) {
                    last_battery_percentage = g_battery_percentage;
                    ESP_LOGI(TAG, "BLE battery updated: %d%%", g_battery_percentage);
                }
            }
        } else {
            // 连接断开时重置，确保下次连接会发送电量
            last_battery_percentage = 255;
        }
        /*!< 取证：持续刷新存活时长，重启后据此推算它这次活了多久。 */
        diag_boot_note_uptime_ms(esp_timer_get_time() / 1000);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}
}