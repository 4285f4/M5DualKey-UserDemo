#include "web_assets.h"
#include "esp_log.h"
#include <string.h>

static const char* TAG = "web_assets";

#define EMBED_SYM(name)                          \
    extern const char _binary_##name##_start[];  \
    extern const char _binary_##name##_end[]

EMBED_SYM(Chain_8Servos_2_jpg);
EMBED_SYM(Chain_Angle_jpg);
EMBED_SYM(Chain_Blank_jpg);
EMBED_SYM(Chain_Buzzer_jpg);
EMBED_SYM(Chain_DLight_jpg);
EMBED_SYM(Chain_DualKey_png);
EMBED_SYM(Chain_Encoder_jpg);
EMBED_SYM(Chain_ENV_jpg);
EMBED_SYM(Chain_IMU_jpg);
EMBED_SYM(Chain_Joystick_jpg);
EMBED_SYM(Chain_Key_jpg);
EMBED_SYM(Chain_MIC_jpg);
EMBED_SYM(Chain_Mono_jpg);
EMBED_SYM(Chain_Mount_jpg);
EMBED_SYM(Chain_PIR_jpg);
EMBED_SYM(Chain_RGB_jpg);
EMBED_SYM(Chain_Switch_jpg);
EMBED_SYM(Chain_ToF_jpg);
EMBED_SYM(Chain_Uart_jpg);

typedef struct {
    const char* uri;
    const char* content_type;
    const char* start;
    const char* end;
} web_asset_t;

static const web_asset_t s_assets[] = {
    {"/Chain_8Servos-2.jpg", "image/jpeg", _binary_Chain_8Servos_2_jpg_start, _binary_Chain_8Servos_2_jpg_end},
    {"/Chain_Angle.jpg", "image/jpeg", _binary_Chain_Angle_jpg_start, _binary_Chain_Angle_jpg_end},
    {"/Chain_Blank.jpg", "image/jpeg", _binary_Chain_Blank_jpg_start, _binary_Chain_Blank_jpg_end},
    {"/Chain_Buzzer.jpg", "image/jpeg", _binary_Chain_Buzzer_jpg_start, _binary_Chain_Buzzer_jpg_end},
    {"/Chain_DLight.jpg", "image/jpeg", _binary_Chain_DLight_jpg_start, _binary_Chain_DLight_jpg_end},
    {"/Chain_DualKey.png", "image/png", _binary_Chain_DualKey_png_start, _binary_Chain_DualKey_png_end},
    {"/Chain_Encoder.jpg", "image/jpeg", _binary_Chain_Encoder_jpg_start, _binary_Chain_Encoder_jpg_end},
    {"/Chain_ENV.jpg", "image/jpeg", _binary_Chain_ENV_jpg_start, _binary_Chain_ENV_jpg_end},
    {"/Chain_IMU.jpg", "image/jpeg", _binary_Chain_IMU_jpg_start, _binary_Chain_IMU_jpg_end},
    {"/Chain_Joystick.jpg", "image/jpeg", _binary_Chain_Joystick_jpg_start, _binary_Chain_Joystick_jpg_end},
    {"/Chain_Key.jpg", "image/jpeg", _binary_Chain_Key_jpg_start, _binary_Chain_Key_jpg_end},
    {"/Chain_MIC.jpg", "image/jpeg", _binary_Chain_MIC_jpg_start, _binary_Chain_MIC_jpg_end},
    {"/Chain_Mono.jpg", "image/jpeg", _binary_Chain_Mono_jpg_start, _binary_Chain_Mono_jpg_end},
    {"/Chain_Mount.jpg", "image/jpeg", _binary_Chain_Mount_jpg_start, _binary_Chain_Mount_jpg_end},
    {"/Chain_PIR.jpg", "image/jpeg", _binary_Chain_PIR_jpg_start, _binary_Chain_PIR_jpg_end},
    {"/Chain_RGB.jpg", "image/jpeg", _binary_Chain_RGB_jpg_start, _binary_Chain_RGB_jpg_end},
    {"/Chain_Switch.jpg", "image/jpeg", _binary_Chain_Switch_jpg_start, _binary_Chain_Switch_jpg_end},
    {"/Chain_ToF.jpg", "image/jpeg", _binary_Chain_ToF_jpg_start, _binary_Chain_ToF_jpg_end},
    {"/Chain_Uart.jpg", "image/jpeg", _binary_Chain_Uart_jpg_start, _binary_Chain_Uart_jpg_end},
};

static esp_err_t web_asset_get_handler(httpd_req_t* req)
{
    for (size_t i = 0; i < sizeof(s_assets) / sizeof(s_assets[0]); i++) {
        if (strcmp(req->uri, s_assets[i].uri) == 0) {
            const uint32_t len = (uint32_t)(s_assets[i].end - s_assets[i].start);
            httpd_resp_set_type(req, s_assets[i].content_type);
            httpd_resp_set_hdr(req, "X-Content-Type-Options", "nosniff");
            httpd_resp_set_hdr(req, "Cache-Control", "public, max-age=3600, immutable");
            httpd_resp_send(req, s_assets[i].start, len);
            return ESP_OK;
        }
    }
    httpd_resp_send_404(req);
    return ESP_FAIL;
}

esp_err_t web_assets_register_handlers(httpd_handle_t server)
{
    httpd_uri_t uri = {
        .uri     = "/Chain_*",
        .method  = HTTP_GET,
        .handler = web_asset_get_handler,
    };
    esp_err_t ret = httpd_register_uri_handler(server, &uri);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to register chain image handler: %s", esp_err_to_name(ret));
        return ret;
    }
    ESP_LOGI(TAG, "Registered chain image wildcard handler (%d assets)", (int)(sizeof(s_assets) / sizeof(s_assets[0])));
    return ESP_OK;
}
