#ifndef CHAIN_BUS_SEQUENCE_H
#define CHAIN_BUS_SEQUENCE_H

#include "esp_err.h"
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

#define BUZZER_SEQ_MAX_STEPS 32

typedef struct {
    uint8_t note;
    uint16_t duration_ms;
} buzzer_seq_step_t;

typedef struct {
    buzzer_seq_step_t steps[BUZZER_SEQ_MAX_STEPS];
    uint8_t step_count;
    uint16_t gap_ms;
    bool loop;
} buzzer_sequence_t;

esp_err_t chain_bus_buzzer_seq_play(int bus_idx, uint8_t device_id, const buzzer_sequence_t* seq);
esp_err_t chain_bus_buzzer_seq_stop(void);
bool chain_bus_buzzer_seq_is_playing(void);
const buzzer_sequence_t* chain_bus_buzzer_seq_get_preset(uint8_t preset_index);
uint8_t chain_bus_buzzer_seq_preset_count(void);

#ifdef __cplusplus
}
#endif

#endif
