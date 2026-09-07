#include <assert.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "c2_pwm.h"
#include "c2_timer.h"

typedef struct {
    volatile uint32_t *address;
    uint32_t value;
} write_record_t;

static uint32_t timer_storage[3];
static uint32_t pwm_storage[9];
static C2_TIM_TypeDef *const test_timer = (C2_TIM_TypeDef *)(void *)timer_storage;
static C2_PWM_TypeDef *const test_pwm = (C2_PWM_TypeDef *)(void *)pwm_storage;
static write_record_t writes[64];
static size_t write_count;
static uint32_t timer_reads[16];
static size_t timer_read_count;
static size_t timer_read_index;
static size_t stat_read_count;

static void reset_mock(void)
{
    memset(timer_storage, 0, sizeof(timer_storage));
    memset(pwm_storage, 0, sizeof(pwm_storage));
    memset(writes, 0, sizeof(writes));
    memset(timer_reads, 0, sizeof(timer_reads));
    write_count = 0U;
    timer_read_count = 0U;
    timer_read_index = 0U;
    stat_read_count = 0U;
}

static void set_timer_reads(const uint32_t *values, size_t count)
{
    assert(count <= (sizeof(timer_reads) / sizeof(timer_reads[0])));
    memcpy(timer_reads, values, count * sizeof(values[0]));
    timer_read_count = count;
    timer_read_index = 0U;
}

uint32_t c2_test_mmio_read32(volatile const uint32_t *address)
{
    if (address == &test_timer->DATA.WORD) {
        if (timer_read_index < timer_read_count) {
            return timer_reads[timer_read_index++];
        }
    }
    if (address == &test_pwm->STAT.WORD) {
        uint32_t value = pwm_storage[8];
        ++stat_read_count;
        pwm_storage[8] = 0U;
        return value;
    }
    return *address;
}

void c2_test_mmio_write32(volatile uint32_t *address, uint32_t value)
{
    assert(write_count < (sizeof(writes) / sizeof(writes[0])));
    writes[write_count].address = address;
    writes[write_count].value = value;
    ++write_count;
    *address = value;
}

static void test_timer_parameters_and_sequence(void)
{
    uint32_t current;

    reset_mock();
    assert(c2_timer_start(NULL, 1U) == C2_ERROR_INVALID_ARGUMENT);
    assert(c2_timer_current(test_timer, NULL) == C2_ERROR_INVALID_ARGUMENT);
    assert(c2_timer_delay_us(NULL, 0U, 0U, 0U) == C2_OK);
    assert(c2_timer_delay_ms(NULL, 0U, 0U, 0U) == C2_OK);
    assert(write_count == 0U);
    assert(c2_timer_delay_us(test_timer, 1000000U, 1U, 0U) ==
           C2_ERROR_INVALID_ARGUMENT);
    assert(write_count == 0U);

    assert(c2_timer_start(test_timer, 71U) == C2_OK);
    assert(write_count == 3U);
    assert(writes[0].address == &test_timer->CONFIG.WORD);
    assert(writes[0].value == C2_TIMER_CONFIG_LOAD);
    assert(writes[1].address == &test_timer->DATA.WORD);
    assert(writes[1].value == 71U);
    assert(writes[2].address == &test_timer->CONFIG.WORD);
    assert(writes[2].value == C2_TIMER_CONFIG_RUN);

    timer_storage[2] = 55U;
    assert(c2_timer_current(test_timer, &current) == C2_OK);
    assert(current == 55U);
}

static void test_timer_systick_modulo(void)
{
    uint32_t now;
    uint32_t elapsed;

    reset_mock();
    assert(c2_timer_systick_init(test_timer) == C2_OK);
    assert(writes[1].value == UINT32_MAX);

    timer_storage[2] = UINT32_MAX - 5U;
    assert(c2_timer_systick_now(test_timer, &now) == C2_OK);
    assert(now == 5U);
    assert(c2_timer_systick_elapsed(test_timer, UINT32_MAX - 2U,
                                    &elapsed) == C2_OK);
    assert(elapsed == 8U);
}

static void test_timer_delay_completion_timeout_and_chunks(void)
{
    static const uint32_t wrap_reads[] = {5U, 7U};
    static const uint32_t timeout_reads[] = {5U, 4U};
    static const uint32_t chunk_reads[] = {0U, 0U};

    reset_mock();
    set_timer_reads(wrap_reads, 2U);
    assert(c2_timer_delay_us(test_timer, 1000000U, 8U, 2U) == C2_OK);
    assert(writes[1].value == 7U);
    assert(writes[write_count - 1U].value == C2_TIMER_CONFIG_LOAD);

    reset_mock();
    set_timer_reads(timeout_reads, 2U);
    assert(c2_timer_delay_us(test_timer, 1000000U, 8U, 2U) ==
           C2_ERROR_TIMEOUT);
    assert(timer_read_index == 2U);
    assert(writes[write_count - 1U].address == &test_timer->CONFIG.WORD);
    assert(writes[write_count - 1U].value == C2_TIMER_CONFIG_LOAD);

    reset_mock();
    set_timer_reads(chunk_reads, 2U);
    assert(c2_timer_delay_us(test_timer, 2000000U, UINT32_MAX, 2U) == C2_OK);
    assert(write_count == 7U);
    assert(writes[1].value == UINT32_MAX);
    assert(writes[4].value == UINT32_MAX - 2U);
}

static void test_pwm_raw_and_calculation(void)
{
    const c2_pwm_config_t raw = {
        71U, 1000U, {1000U, 750U, 500U, 0U}
    };
    const c2_pwm_config_t bad_prescaler = {
        65536U, 1000U, {0U, 0U, 0U, 0U}
    };
    const c2_pwm_config_t bad_period = {
        0U, 0U, {0U, 0U, 0U, 0U}
    };
    const c2_pwm_config_t bad_compare = {
        0U, 1000U, {1001U, 0U, 0U, 0U}
    };
    const uint32_t duties[4] = {0U, 10000U, 5000U, 2500U};
    const uint32_t bad_duties[4] = {0U, 10001U, 0U, 0U};
    c2_pwm_config_t calculated;
    uint32_t actual;

    reset_mock();
    assert(c2_pwm_init_raw(test_pwm, &raw) == C2_OK);
    assert(write_count == 6U);
    assert(writes[0].address == &test_pwm->PSCR.WORD);
    assert(writes[0].value == 71U);
    assert(writes[1].address == &test_pwm->CMP.WORD);
    assert(writes[1].value == 1000U);
    assert(writes[5].address == &test_pwm->CR3.WORD);
    assert(writes[5].value == 0U);

    reset_mock();
    assert(c2_pwm_init_raw(test_pwm, &bad_prescaler) ==
           C2_ERROR_INVALID_ARGUMENT);
    assert(c2_pwm_init_raw(test_pwm, &bad_period) == C2_ERROR_INVALID_ARGUMENT);
    assert(c2_pwm_init_raw(test_pwm, &bad_compare) ==
           C2_ERROR_INVALID_ARGUMENT);
    assert(write_count == 0U);

    assert(c2_pwm_calculate_config(72000000U, 1000U, duties, &calculated,
                                   &actual) == C2_OK);
    assert(calculated.prescaler == 1U);
    assert(calculated.period == 36000U);
    assert(calculated.compare[0] == 36000U);
    assert(calculated.compare[1] == 0U);
    assert(calculated.compare[2] == 18000U);
    assert(calculated.compare[3] == 27000U);
    assert(actual == 1000U);

    assert(c2_pwm_calculate_config(72000000U, 72000001U, duties,
                                   &calculated, NULL) ==
           C2_ERROR_INVALID_ARGUMENT);
    assert(c2_pwm_calculate_config(72000000U, 1000U, bad_duties,
                                   &calculated, NULL) ==
           C2_ERROR_INVALID_ARGUMENT);
}

static void test_pwm_boundaries_control_and_snapshot(void)
{
    c2_pwm_irq_snapshot_t snapshot;

    reset_mock();
    pwm_storage[3] = 1000U;
    assert(c2_pwm_set_duty(test_pwm, C2_PWM_CHANNEL_0, 0U) == C2_OK);
    assert(pwm_storage[4] == 1000U);
    assert(c2_pwm_set_duty(test_pwm, C2_PWM_CHANNEL_1,
                           C2_PWM_DUTY_SCALE) == C2_OK);
    assert(pwm_storage[5] == 0U);
    assert(c2_pwm_set_compare(test_pwm, (c2_pwm_channel_t)4, 1U) ==
           C2_ERROR_INVALID_ARGUMENT);
    assert(c2_pwm_set_compare(test_pwm, C2_PWM_CHANNEL_2, 1001U) ==
           C2_ERROR_INVALID_ARGUMENT);

    pwm_storage[0] = C2_PWM_CTRL_OVIE;
    assert(c2_pwm_enable(test_pwm) == C2_OK);
    assert(pwm_storage[0] == (C2_PWM_CTRL_OVIE | C2_PWM_CTRL_EN));
    assert(c2_pwm_clear(test_pwm) == C2_OK);
    assert(writes[write_count - 2U].value ==
           (C2_PWM_CTRL_OVIE | C2_PWM_CTRL_EN | C2_PWM_CTRL_CLR));
    assert(writes[write_count - 1U].value ==
           (C2_PWM_CTRL_OVIE | C2_PWM_CTRL_EN));
    assert(c2_pwm_irq_enable(test_pwm, false) == C2_OK);
    assert(pwm_storage[0] == C2_PWM_CTRL_EN);
    assert(c2_pwm_disable(test_pwm) == C2_OK);
    assert(pwm_storage[0] == 0U);

    pwm_storage[8] = 1U;
    assert(c2_pwm_irq_snapshot(test_pwm, &snapshot) == C2_OK);
    assert(snapshot.raw == 1U);
    assert(snapshot.overflow);
    assert(stat_read_count == 1U);
    assert(pwm_storage[8] == 0U);
    assert(c2_pwm_irq_snapshot(test_pwm, &snapshot) == C2_OK);
    assert(!snapshot.overflow);
    assert(stat_read_count == 2U);
}

int main(void)
{
    test_timer_parameters_and_sequence();
    test_timer_systick_modulo();
    test_timer_delay_completion_timeout_and_chunks();
    test_pwm_raw_and_calculation();
    test_pwm_boundaries_control_and_snapshot();
    puts("timer/pwm tests passed");
    return 0;
}
