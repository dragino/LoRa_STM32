#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "dendrometer.h"

typedef struct {
    uint8_t power_events[4];
    uint8_t power_event_count;
    uint8_t channel_trace[128];
    uint8_t channel_trace_count;
    uint16_t delay_trace[128];
    uint8_t delay_trace_count;
    uint16_t signal_samples[DENDROMETER_SAMPLE_COUNT];
    uint16_t reference_samples[DENDROMETER_SAMPLE_COUNT];
    uint16_t signal_index;
    uint16_t reference_index;
} fake_board_t;

static void fake_set_power_enabled(void *ctx, uint8_t enabled)
{
    fake_board_t *board = (fake_board_t *)ctx;
    board->power_events[board->power_event_count++] = enabled;
}

static uint16_t fake_read_adc_raw(void *ctx, uint8_t channel_id)
{
    fake_board_t *board = (fake_board_t *)ctx;
    board->channel_trace[board->channel_trace_count++] = channel_id;

    if (channel_id == DENDROMETER_CHANNEL_SIGNAL) {
        return board->signal_samples[board->signal_index++];
    }

    if (channel_id == DENDROMETER_CHANNEL_REFERENCE) {
        return board->reference_samples[board->reference_index++];
    }

    return 0;
}

static void fake_delay_ms(void *ctx, uint16_t delay_ms)
{
    fake_board_t *board = (fake_board_t *)ctx;
    board->delay_trace[board->delay_trace_count++] = delay_ms;
}

static void seed_samples(fake_board_t *board, uint16_t signal, uint16_t reference)
{
    uint16_t i;

    memset(board, 0, sizeof(*board));
    for (i = 0; i < DENDROMETER_SAMPLE_COUNT; ++i) {
        board->signal_samples[i] = signal;
        board->reference_samples[i] = reference;
    }
}

static void test_read_adc_avg_returns_integer_mean(void)
{
    assert(read_adc_avg(5000U, 5U) == 1000U);
    assert(read_adc_avg(999U, 0U) == 0U);
}

static void test_measure_dendrometer_pairs_channels_and_sets_valid_flag(void)
{
    fake_board_t board;
    dendrometer_board_ops_t ops;
    dendrometer_measurement_t measurement = {0};

    seed_samples(&board, 1024U, 2048U);

    ops.ctx = &board;
    ops.set_power_enabled = fake_set_power_enabled;
    ops.read_adc_raw = fake_read_adc_raw;
    ops.delay_ms = fake_delay_ms;

    assert(measure_dendrometer(&ops, &measurement) == 0);
    assert(measurement.adc_signal_avg_raw == 1024U);
    assert(measurement.adc_reference_avg_raw == 2048U);
    assert(measurement.valid == 1U);
    assert(measurement.flags == DENDROMETER_FLAG_VALID);
    assert(board.power_event_count == 2U);
    assert(board.power_events[0] == 1U);
    assert(board.power_events[1] == 0U);
    assert(board.delay_trace_count == DENDROMETER_SAMPLE_COUNT);
    assert(board.delay_trace[0] == DENDROMETER_SETTLE_TIME_MS);
    assert(board.delay_trace[1] == DENDROMETER_SAMPLE_DELAY_MS);
    assert(board.channel_trace_count == (uint8_t)(DENDROMETER_SAMPLE_COUNT * 2U));
    assert(board.channel_trace[0] == DENDROMETER_CHANNEL_SIGNAL);
    assert(board.channel_trace[1] == DENDROMETER_CHANNEL_REFERENCE);
}

static void test_measure_dendrometer_marks_low_reference_invalid(void)
{
    fake_board_t board;
    dendrometer_board_ops_t ops;
    dendrometer_measurement_t measurement = {0};

    seed_samples(&board, 900U, 64U);

    ops.ctx = &board;
    ops.set_power_enabled = fake_set_power_enabled;
    ops.read_adc_raw = fake_read_adc_raw;
    ops.delay_ms = fake_delay_ms;

    assert(measure_dendrometer(&ops, &measurement) == 0);
    assert(measurement.adc_signal_avg_raw == 900U);
    assert(measurement.adc_reference_avg_raw == 64U);
    assert(measurement.valid == 0U);
    assert(measurement.flags == (DENDROMETER_FLAG_REFERENCE_TOO_LOW | DENDROMETER_FLAG_DIVISION_SKIPPED));
    assert(board.power_event_count == 2U);
    assert(board.power_events[1] == 0U);
}

static void test_pack_mod3_payload_emits_new_8_byte_frame(void)
{
    const dendrometer_measurement_t measurement = {
        1024U,
        2048U,
        1U,
        DENDROMETER_FLAG_VALID
    };
    uint8_t payload[DENDROMETER_MOD3_PAYLOAD_LEN] = {0};
    const uint8_t expected[DENDROMETER_MOD3_PAYLOAD_LEN] = {0x0F, 0xA0, 0x04, 0x00, 0x08, 0x00, 0x08, 0x01};

    assert(dendrometer_pack_mod3_payload(4000U, 0x08U, &measurement, payload, sizeof(payload)) == DENDROMETER_MOD3_PAYLOAD_LEN);
    assert(memcmp(payload, expected, sizeof(expected)) == 0);
}

int main(void)
{
    test_read_adc_avg_returns_integer_mean();
    test_measure_dendrometer_pairs_channels_and_sets_valid_flag();
    test_measure_dendrometer_marks_low_reference_invalid();
    test_pack_mod3_payload_emits_new_8_byte_frame();
    puts("All dendrometer tests passed");
    return 0;
}
