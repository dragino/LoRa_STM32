#include <string.h>

#include "dendrometer.h"

uint16_t read_adc_avg(uint32_t accumulated_sum, uint16_t sample_count)
{
    if (sample_count == 0U) {
        return 0U;
    }

    return (uint16_t)(accumulated_sum / sample_count);
}

int measure_dendrometer(const dendrometer_board_ops_t *ops, dendrometer_measurement_t *measurement)
{
    uint16_t i;
    uint32_t signal_sum = 0U;
    uint32_t reference_sum = 0U;

    if (ops == 0 || measurement == 0 || ops->set_power_enabled == 0 || ops->read_adc_raw == 0 || ops->delay_ms == 0) {
        return -1;
    }

    memset(measurement, 0, sizeof(*measurement));

    ops->set_power_enabled(ops->ctx, 1U);
    ops->delay_ms(ops->ctx, DENDROMETER_SETTLE_TIME_MS);

    for (i = 0U; i < DENDROMETER_SAMPLE_COUNT; ++i) {
        signal_sum += ops->read_adc_raw(ops->ctx, DENDROMETER_CHANNEL_SIGNAL);
        reference_sum += ops->read_adc_raw(ops->ctx, DENDROMETER_CHANNEL_REFERENCE);

        if ((i + 1U) < DENDROMETER_SAMPLE_COUNT) {
            ops->delay_ms(ops->ctx, DENDROMETER_SAMPLE_DELAY_MS);
        }
    }

    ops->set_power_enabled(ops->ctx, 0U);

    measurement->adc_signal_avg_raw = read_adc_avg(signal_sum, DENDROMETER_SAMPLE_COUNT);
    measurement->adc_reference_avg_raw = read_adc_avg(reference_sum, DENDROMETER_SAMPLE_COUNT);

    if (measurement->adc_reference_avg_raw < DENDROMETER_REF_MIN_RAW) {
        measurement->valid = 0U;
        measurement->flags = DENDROMETER_FLAG_REFERENCE_TOO_LOW | DENDROMETER_FLAG_DIVISION_SKIPPED;
        return 0;
    }

    measurement->valid = 1U;
    measurement->flags = DENDROMETER_FLAG_VALID;
    return 0;
}

uint8_t dendrometer_pack_mod3_payload(uint16_t battery_mv, uint8_t status_mode_byte, const dendrometer_measurement_t *measurement, uint8_t *payload, uint8_t payload_len)
{
    if (measurement == 0 || payload == 0 || payload_len < DENDROMETER_MOD3_PAYLOAD_LEN) {
        return 0U;
    }

    payload[0] = (uint8_t)(battery_mv >> 8);
    payload[1] = (uint8_t)battery_mv;
    payload[2] = (uint8_t)(measurement->adc_signal_avg_raw >> 8);
    payload[3] = (uint8_t)measurement->adc_signal_avg_raw;
    payload[4] = (uint8_t)(measurement->adc_reference_avg_raw >> 8);
    payload[5] = (uint8_t)measurement->adc_reference_avg_raw;
    payload[6] = status_mode_byte;
    payload[7] = measurement->flags;

    return DENDROMETER_MOD3_PAYLOAD_LEN;
}
