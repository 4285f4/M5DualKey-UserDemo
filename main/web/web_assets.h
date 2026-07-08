#pragma once

#include "esp_http_server.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t web_assets_register_handlers(httpd_handle_t server);

#ifdef __cplusplus
}
#endif
