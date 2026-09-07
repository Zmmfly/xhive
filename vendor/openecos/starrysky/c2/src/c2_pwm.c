#include "c2_pwm.h"

static bool c2_pwm_channel_valid(c2_pwm_channel_t channel)
{
    return ((uint32_t)channel < C2_PWM_CHANNEL_COUNT);
}

static void c2_pwm_write_compare(C2_PWM_TypeDef *pwm,
                                 c2_pwm_channel_t channel,
                                 uint32_t compare)
{
    switch (channel) {
    case C2_PWM_CHANNEL_0:
        c2_mmio_write32(&pwm->CR0.WORD, compare);
        break;
    case C2_PWM_CHANNEL_1:
        c2_mmio_write32(&pwm->CR1.WORD, compare);
        break;
    case C2_PWM_CHANNEL_2:
        c2_mmio_write32(&pwm->CR2.WORD, compare);
        break;
    case C2_PWM_CHANNEL_3:
        c2_mmio_write32(&pwm->CR3.WORD, compare);
        break;
    default:
        break;
    }
}

static c2_status_t c2_pwm_validate_config(const c2_pwm_config_t *config)
{
    uint32_t channel;

    if (config == NULL) {
        return C2_ERROR_INVALID_ARGUMENT;
    }
    if ((config->prescaler > C2_PWM_FIELD_MAX) ||
        (config->period == 0U) ||
        (config->period > C2_PWM_FIELD_MAX)) {
        return C2_ERROR_INVALID_ARGUMENT;
    }

    for (channel = 0U; channel < C2_PWM_CHANNEL_COUNT; ++channel) {
        if ((config->compare[channel] > C2_PWM_FIELD_MAX) ||
            (config->compare[channel] > config->period)) {
            return C2_ERROR_INVALID_ARGUMENT;
        }
    }

    return C2_OK;
}

static uint32_t c2_pwm_control_state(C2_PWM_TypeDef *pwm)
{
    return c2_mmio_read32(&pwm->CTRL.WORD) &
           (C2_PWM_CTRL_OVIE | C2_PWM_CTRL_EN);
}

c2_status_t c2_pwm_init_raw(C2_PWM_TypeDef *pwm,
                            const c2_pwm_config_t *config)
{
    c2_status_t status;

    if (pwm == NULL) {
        return C2_ERROR_INVALID_ARGUMENT;
    }

    status = c2_pwm_validate_config(config);
    if (status != C2_OK) {
        return status;
    }

    c2_mmio_write32(&pwm->PSCR.WORD, config->prescaler);
    c2_mmio_write32(&pwm->CMP.WORD, config->period);
    c2_mmio_write32(&pwm->CR0.WORD, config->compare[0]);
    c2_mmio_write32(&pwm->CR1.WORD, config->compare[1]);
    c2_mmio_write32(&pwm->CR2.WORD, config->compare[2]);
    c2_mmio_write32(&pwm->CR3.WORD, config->compare[3]);
    return C2_OK;
}

c2_status_t c2_pwm_calculate_config(uint32_t clock_hz,
                                    uint32_t frequency_hz,
                                    const uint32_t duty[C2_PWM_CHANNEL_COUNT],
                                    c2_pwm_config_t *config,
                                    uint32_t *actual_frequency_hz)
{
    uint64_t divider;
    uint64_t period;
    uint64_t frequency_denominator;
    uint32_t channel;

    if ((clock_hz == 0U) || (frequency_hz == 0U) ||
        (frequency_hz > clock_hz) || (duty == NULL) || (config == NULL)) {
        return C2_ERROR_INVALID_ARGUMENT;
    }

    for (channel = 0U; channel < C2_PWM_CHANNEL_COUNT; ++channel) {
        if (duty[channel] > C2_PWM_DUTY_SCALE) {
            return C2_ERROR_INVALID_ARGUMENT;
        }
    }

    frequency_denominator = (uint64_t)frequency_hz * C2_PWM_FIELD_MAX;
    divider = ((uint64_t)clock_hz + frequency_denominator - 1U) /
              frequency_denominator;
    if (divider == 0U) {
        divider = 1U;
    }
    if (divider > (C2_PWM_FIELD_MAX + 1U)) {
        divider = C2_PWM_FIELD_MAX + 1U;
    }

    frequency_denominator = (uint64_t)frequency_hz * divider;
    period = ((uint64_t)clock_hz + (frequency_denominator / 2U)) /
             frequency_denominator;
    if (period == 0U) {
        period = 1U;
    } else if (period > C2_PWM_FIELD_MAX) {
        period = C2_PWM_FIELD_MAX;
    }

    config->prescaler = (uint32_t)(divider - 1U);
    config->period = (uint32_t)period;
    for (channel = 0U; channel < C2_PWM_CHANNEL_COUNT; ++channel) {
        const uint64_t high_clocks =
            (period * duty[channel] + (C2_PWM_DUTY_SCALE / 2U)) /
            C2_PWM_DUTY_SCALE;
        config->compare[channel] = (uint32_t)(period - high_clocks);
    }

    if (actual_frequency_hz != NULL) {
        const uint64_t hardware_divisor = divider * period;
        *actual_frequency_hz = (uint32_t)((uint64_t)clock_hz /
                                          hardware_divisor);
    }

    return C2_OK;
}

c2_status_t c2_pwm_configure(C2_PWM_TypeDef *pwm,
                             uint32_t clock_hz,
                             uint32_t frequency_hz,
                             const uint32_t duty[C2_PWM_CHANNEL_COUNT],
                             c2_pwm_config_t *config_out,
                             uint32_t *actual_frequency_hz)
{
    c2_pwm_config_t config;
    uint32_t actual;
    c2_status_t status;

    if (pwm == NULL) {
        return C2_ERROR_INVALID_ARGUMENT;
    }

    status = c2_pwm_calculate_config(clock_hz, frequency_hz, duty, &config,
                                     &actual);
    if (status != C2_OK) {
        return status;
    }

    status = c2_pwm_init_raw(pwm, &config);
    if (status != C2_OK) {
        return status;
    }

    if (config_out != NULL) {
        *config_out = config;
    }
    if (actual_frequency_hz != NULL) {
        *actual_frequency_hz = actual;
    }
    return C2_OK;
}

c2_status_t c2_pwm_set_compare(C2_PWM_TypeDef *pwm,
                               c2_pwm_channel_t channel,
                               uint32_t compare)
{
    uint32_t period;

    if ((pwm == NULL) || !c2_pwm_channel_valid(channel)) {
        return C2_ERROR_INVALID_ARGUMENT;
    }

    period = c2_mmio_read32(&pwm->CMP.WORD) & C2_PWM_FIELD_MAX;
    if ((period == 0U) || (compare > period) ||
        (compare > C2_PWM_FIELD_MAX)) {
        return C2_ERROR_INVALID_ARGUMENT;
    }

    c2_pwm_write_compare(pwm, channel, compare);
    return C2_OK;
}

c2_status_t c2_pwm_set_duty(C2_PWM_TypeDef *pwm,
                            c2_pwm_channel_t channel,
                            uint32_t duty)
{
    uint32_t period;
    uint64_t high_clocks;
    uint32_t compare;

    if ((pwm == NULL) || !c2_pwm_channel_valid(channel) ||
        (duty > C2_PWM_DUTY_SCALE)) {
        return C2_ERROR_INVALID_ARGUMENT;
    }

    period = c2_mmio_read32(&pwm->CMP.WORD) & C2_PWM_FIELD_MAX;
    if (period == 0U) {
        return C2_ERROR_INVALID_ARGUMENT;
    }

    high_clocks = ((uint64_t)period * duty +
                   (C2_PWM_DUTY_SCALE / 2U)) /
                  C2_PWM_DUTY_SCALE;
    compare = period - (uint32_t)high_clocks;
    c2_pwm_write_compare(pwm, channel, compare);
    return C2_OK;
}

c2_status_t c2_pwm_enable(C2_PWM_TypeDef *pwm)
{
    uint32_t control;

    if (pwm == NULL) {
        return C2_ERROR_INVALID_ARGUMENT;
    }

    control = c2_pwm_control_state(pwm) | C2_PWM_CTRL_EN;
    c2_mmio_write32(&pwm->CTRL.WORD, control);
    return C2_OK;
}

c2_status_t c2_pwm_disable(C2_PWM_TypeDef *pwm)
{
    uint32_t control;

    if (pwm == NULL) {
        return C2_ERROR_INVALID_ARGUMENT;
    }

    control = c2_pwm_control_state(pwm) & ~C2_PWM_CTRL_EN;
    c2_mmio_write32(&pwm->CTRL.WORD, control);
    return C2_OK;
}

c2_status_t c2_pwm_clear(C2_PWM_TypeDef *pwm)
{
    uint32_t control;

    if (pwm == NULL) {
        return C2_ERROR_INVALID_ARGUMENT;
    }

    control = c2_pwm_control_state(pwm);
    c2_mmio_write32(&pwm->CTRL.WORD, control | C2_PWM_CTRL_CLR);
    c2_mmio_write32(&pwm->CTRL.WORD, control);
    return C2_OK;
}

c2_status_t c2_pwm_irq_enable(C2_PWM_TypeDef *pwm, bool enable)
{
    uint32_t control;

    if (pwm == NULL) {
        return C2_ERROR_INVALID_ARGUMENT;
    }

    control = c2_pwm_control_state(pwm);
    if (enable) {
        control |= C2_PWM_CTRL_OVIE;
    } else {
        control &= ~C2_PWM_CTRL_OVIE;
    }
    c2_mmio_write32(&pwm->CTRL.WORD, control);
    return C2_OK;
}

c2_status_t c2_pwm_irq_snapshot(C2_PWM_TypeDef *pwm,
                                c2_pwm_irq_snapshot_t *snapshot)
{
    uint32_t raw;

    if ((pwm == NULL) || (snapshot == NULL)) {
        return C2_ERROR_INVALID_ARGUMENT;
    }

    raw = c2_mmio_read32(&pwm->STAT.WORD);
    snapshot->raw = raw;
    snapshot->overflow = ((raw & UINT32_C(1)) != 0U);
    return C2_OK;
}
