#ifndef __DENDROMETER_H__
#define __DENDROMETER_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#define DENDROMETER_SETTLE_TIME_MS      50U
#define DENDROMETER_SAMPLE_COUNT        50U
#define DENDROMETER_SAMPLE_DELAY_MS     1U
#define DENDROMETER_REF_MIN_RAW         128U
#define DENDROMETER_MOD3_PAYLOAD_LEN    8U

#define DENDROMETER_CHANNEL_SIGNAL      0U
#define DENDROMETER_CHANNEL_REFERENCE   1U

#define DENDROMETER_FLAG_VALID              0x01U
#define DENDROMETER_FLAG_REFERENCE_TOO_LOW  0x02U
#define DENDROMETER_FLAG_DIVISION_SKIPPED   0x04U

typedef struct {
    void *ctx;
    void (*set_power_enabled)(void *ctx, uint8_t enabled);
    uint16_t (*read_adc_raw)(void *ctx, uint8_t channel_id);
    void (*delay_ms)(void *ctx, uint16_t delay_ms);
} dendrometer_board_ops_t;

typedef struct {
    uint16_t adc_signal_avg_raw;
    uint16_t adc_reference_avg_raw;
    uint8_t valid;
    uint8_t flags;
} dendrometer_measurement_t;

uint16_t read_adc_avg(uint32_t accumulated_sum, uint16_t sample_count);
int measure_dendrometer(const dendrometer_board_ops_t *ops, dendrometer_measurement_t *measurement);
uint8_t dendrometer_pack_mod3_payload(uint16_t battery_mv, uint8_t status_mode_byte, const dendrometer_measurement_t *measurement, uint8_t *payload, uint8_t payload_len);

#ifdef __cplusplus
}
#endif

#endif
