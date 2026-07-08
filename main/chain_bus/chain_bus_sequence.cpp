#include "chain_bus_sequence.h"
#include "chain_bus.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <stdlib.h>
#include <string.h>

static const char* TAG = "buzzer_seq";

typedef struct {
    int bus_idx;
    uint8_t device_id;
    buzzer_sequence_t seq;
} buzzer_seq_ctx_t;

static TaskHandle_t s_seq_task  = NULL;
static volatile bool s_seq_stop = false;
static int s_last_bus_idx       = -1;
static uint8_t s_last_device_id = 0;

static const buzzer_sequence_t s_presets[] = {
    {
        // Super Mario Bros. intro (BPM 200, quarter = 300ms, eighth = 150ms)
        .steps =
            {
                {29, 150},
                {29, 150},
                {0, 150},
                {29, 150},
                {0, 150},
                {25, 150},
                {29, 300},
                {32, 300},
            },
        .step_count = 8,
        .gap_ms     = 20,
        .loop       = false,
    },
    {
        .steps =
            {
                {20, 150},
                {24, 150},
                {29, 150},
                {25, 300},
                {0, 100},
            },
        .step_count = 5,
        .gap_ms     = 40,
        .loop       = false,
    },
    {
        .steps =
            {
                {13, 120},
                {15, 120},
                {17, 120},
                {18, 120},
                {20, 120},
                {22, 120},
                {24, 120},
                {25, 240},
            },
        .step_count = 8,
        .gap_ms     = 20,
        .loop       = false,
    },
    {
        .steps =
            {
                {29, 80},
                {29, 80},
                {29, 80},
                {0, 80},
                {25, 200},
                {0, 200},
                {29, 80},
                {29, 80},
            },
        .step_count = 8,
        .gap_ms     = 10,
        .loop       = false,
    },
};

static void buzzer_seq_interruptible_delay(uint32_t wait_ms)
{
    while (wait_ms > 0 && !s_seq_stop) {
        uint32_t chunk = (wait_ms > 10) ? 10 : wait_ms;
        vTaskDelay(pdMS_TO_TICKS(chunk));
        wait_ms -= chunk;
    }
}

static void buzzer_seq_task(void* arg)
{
    buzzer_seq_ctx_t* ctx = (buzzer_seq_ctx_t*)arg;

    do {
        for (uint8_t i = 0; i < ctx->seq.step_count && !s_seq_stop; i++) {
            if (ctx->seq.steps[i].note != 0) {
                chain_bus_buzzer_note_play(ctx->bus_idx, ctx->device_id, ctx->seq.steps[i].note,
                                           ctx->seq.steps[i].duration_ms);
            }
            uint32_t wait_ms = ctx->seq.steps[i].duration_ms + ctx->seq.gap_ms;
            buzzer_seq_interruptible_delay(wait_ms);
        }
    } while (ctx->seq.loop && !s_seq_stop);

    if (s_seq_stop) {
        chain_bus_buzzer_stop(ctx->bus_idx, ctx->device_id);
    }

    free(ctx);
    s_seq_task = NULL;
    vTaskDelete(NULL);
}

esp_err_t chain_bus_buzzer_seq_stop(void)
{
    s_seq_stop = true;
    for (int i = 0; i < 100 && s_seq_task != NULL; i++) {
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    if (s_last_bus_idx >= 0 && s_last_device_id != 0) {
        chain_bus_buzzer_stop(s_last_bus_idx, s_last_device_id);
    }
    return ESP_OK;
}

bool chain_bus_buzzer_seq_is_playing(void)
{
    return s_seq_task != NULL;
}

esp_err_t chain_bus_buzzer_seq_play(int bus_idx, uint8_t device_id, const buzzer_sequence_t* seq)
{
    if (!seq || seq->step_count == 0 || seq->step_count > BUZZER_SEQ_MAX_STEPS) {
        return ESP_ERR_INVALID_ARG;
    }

    chain_bus_buzzer_seq_stop();

    buzzer_seq_ctx_t* ctx = (buzzer_seq_ctx_t*)malloc(sizeof(buzzer_seq_ctx_t));
    if (!ctx) {
        return ESP_ERR_NO_MEM;
    }
    ctx->bus_idx   = bus_idx;
    ctx->device_id = device_id;
    memcpy(&ctx->seq, seq, sizeof(buzzer_sequence_t));

    s_last_bus_idx   = bus_idx;
    s_last_device_id = device_id;
    s_seq_stop       = false;
    if (xTaskCreate(buzzer_seq_task, "buzzer_seq", 4096, ctx, 4, &s_seq_task) != pdPASS) {
        free(ctx);
        return ESP_FAIL;
    }
    return ESP_OK;
}

const buzzer_sequence_t* chain_bus_buzzer_seq_get_preset(uint8_t preset_index)
{
    if (preset_index >= (sizeof(s_presets) / sizeof(s_presets[0]))) {
        return NULL;
    }
    return &s_presets[preset_index];
}

uint8_t chain_bus_buzzer_seq_preset_count(void)
{
    return sizeof(s_presets) / sizeof(s_presets[0]);
}
