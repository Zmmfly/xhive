#include "c2_timer.h"

#include <limits.h>

#define C2_TIMER_CLOCKS_PER_US UINT32_C(1000000)
#define C2_TIMER_CLOCKS_PER_MS UINT32_C(1000)
#define C2_TIMER_CHUNK_CLOCKS  (UINT64_C(1) << 32)

static c2_status_t c2_timer_delay_clocks(C2_TIM_TypeDef *timer,
                                         uint64_t clocks,
                                         uint32_t poll_limit)
{
    uint32_t polls_left = poll_limit;

    while (clocks != 0U) {
        const uint64_t chunk = clocks > C2_TIMER_CHUNK_CLOCKS
                                   ? C2_TIMER_CHUNK_CLOCKS
                                   : clocks;
        const uint32_t reload = chunk == C2_TIMER_CHUNK_CLOCKS
                                    ? UINT32_MAX
                                    : (uint32_t)(chunk - 1U);
        uint32_t previous = reload;
        bool complete = false;
        c2_status_t status = c2_timer_start(timer, reload);

        if (status != C2_OK) {
            return status;
        }

        while (polls_left != 0U) {
            uint32_t current;

            status = c2_timer_current(timer, &current);
            if (status != C2_OK) {
                (void)c2_timer_stop(timer);
                return status;
            }
            --polls_left;

            if ((current == 0U) || (current > previous)) {
                complete = true;
                break;
            }
            previous = current;
        }

        if (!complete) {
            (void)c2_timer_stop(timer);
            return C2_ERROR_TIMEOUT;
        }

        clocks -= chunk;
    }

    return c2_timer_stop(timer);
}

static c2_status_t c2_timer_convert_clocks(uint32_t clock_hz,
                                           uint32_t duration,
                                           uint32_t units_per_second,
                                           uint64_t *clocks)
{
    const uint64_t whole = (uint64_t)(clock_hz / units_per_second) * duration;
    const uint64_t fraction =
        (uint64_t)(clock_hz % units_per_second) * duration;
    uint64_t rounded_fraction = fraction / units_per_second;

    if ((fraction % units_per_second) != 0U) {
        ++rounded_fraction;
    }
    if (whole > (UINT64_MAX - rounded_fraction)) {
        return C2_ERROR_INVALID_ARGUMENT;
    }

    *clocks = whole + rounded_fraction;
    return C2_OK;
}

c2_status_t c2_timer_start(C2_TIM_TypeDef *timer, uint32_t reload)
{
    if (timer == NULL) {
        return C2_ERROR_INVALID_ARGUMENT;
    }

    c2_mmio_write32(&timer->CONFIG.WORD, C2_TIMER_CONFIG_LOAD);
    c2_mmio_write32(&timer->DATA.WORD, reload);
    c2_mmio_write32(&timer->CONFIG.WORD, C2_TIMER_CONFIG_RUN);
    return C2_OK;
}

c2_status_t c2_timer_stop(C2_TIM_TypeDef *timer)
{
    if (timer == NULL) {
        return C2_ERROR_INVALID_ARGUMENT;
    }

    c2_mmio_write32(&timer->CONFIG.WORD, C2_TIMER_CONFIG_LOAD);
    return C2_OK;
}

c2_status_t c2_timer_current(const C2_TIM_TypeDef *timer, uint32_t *current)
{
    if ((timer == NULL) || (current == NULL)) {
        return C2_ERROR_INVALID_ARGUMENT;
    }

    *current = c2_mmio_read32(&timer->DATA.WORD);
    return C2_OK;
}

c2_status_t c2_timer_systick_init(C2_TIM_TypeDef *timer)
{
    return c2_timer_start(timer, UINT32_MAX);
}

c2_status_t c2_timer_systick_now(const C2_TIM_TypeDef *timer, uint32_t *tick)
{
    uint32_t current;
    c2_status_t status;

    if (tick == NULL) {
        return C2_ERROR_INVALID_ARGUMENT;
    }

    status = c2_timer_current(timer, &current);
    if (status != C2_OK) {
        return status;
    }

    *tick = UINT32_MAX - current;
    return C2_OK;
}

c2_status_t c2_timer_systick_elapsed(const C2_TIM_TypeDef *timer,
                                     uint32_t start,
                                     uint32_t *elapsed)
{
    uint32_t now;
    c2_status_t status;

    if (elapsed == NULL) {
        return C2_ERROR_INVALID_ARGUMENT;
    }

    status = c2_timer_systick_now(timer, &now);
    if (status != C2_OK) {
        return status;
    }

    *elapsed = now - start;
    return C2_OK;
}

c2_status_t c2_timer_delay_us(C2_TIM_TypeDef *timer,
                              uint32_t clock_hz,
                              uint32_t delay_us,
                              uint32_t poll_limit)
{
    uint64_t clocks;
    c2_status_t status;

    if (delay_us == 0U) {
        return C2_OK;
    }
    if ((timer == NULL) || (clock_hz == 0U) || (poll_limit == 0U)) {
        return C2_ERROR_INVALID_ARGUMENT;
    }

    status = c2_timer_convert_clocks(clock_hz, delay_us,
                                     C2_TIMER_CLOCKS_PER_US, &clocks);
    if (status != C2_OK) {
        return status;
    }

    return c2_timer_delay_clocks(timer, clocks, poll_limit);
}

c2_status_t c2_timer_delay_ms(C2_TIM_TypeDef *timer,
                              uint32_t clock_hz,
                              uint32_t delay_ms,
                              uint32_t poll_limit)
{
    uint64_t clocks;
    c2_status_t status;

    if (delay_ms == 0U) {
        return C2_OK;
    }
    if ((timer == NULL) || (clock_hz == 0U) || (poll_limit == 0U)) {
        return C2_ERROR_INVALID_ARGUMENT;
    }

    status = c2_timer_convert_clocks(clock_hz, delay_ms,
                                     C2_TIMER_CLOCKS_PER_MS, &clocks);
    if (status != C2_OK) {
        return status;
    }

    return c2_timer_delay_clocks(timer, clocks, poll_limit);
}
