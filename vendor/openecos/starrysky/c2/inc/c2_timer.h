/* StarrySky C2 timers. Source: works/docs/periphs/timer.md.
 * Do not substitute the reference APB timer's CTRL/PSCR/CNT layout.
 * C2 has no documented CMP or interrupt status register.
 */
#ifndef STARRYSKY_C2_TIMER_H
#define STARRYSKY_C2_TIMER_H

#include "c2_common.h"
#include "c2_register.h"

/* ENABLE and LOAD are driver-inferred, not authoritative bit definitions. */
typedef union {
    uint32_t WORD;
    struct {
        uint32_t ENABLE : 1; /* [0] */
        uint32_t UNKNOWN0 : 7; /* [7:1] */
        uint32_t LOAD : 1; /* [8] */
        uint32_t UNKNOWN1 : 23; /* [31:9] */
    } BITS;
} C2_TIM_CONFIG_TypeDef;
C2_REG_ASSERT_SIZE(C2_TIM_CONFIG_TypeDef, 4);

/* Count format and actual read behavior are not verified. */
typedef union {
    uint32_t WORD;
    struct {
        uint32_t VALUE : 32; /* [31:0] */
    } BITS;
} C2_TIM_VALUE_TypeDef;
C2_REG_ASSERT_SIZE(C2_TIM_VALUE_TypeDef, 4);

typedef union {
    uint32_t WORD;
    struct {
        uint32_t DATA : 32; /* [31:0] */
    } BITS;
} C2_TIM_DATA_TypeDef;
C2_REG_ASSERT_SIZE(C2_TIM_DATA_TypeDef, 4);

typedef struct {
    volatile C2_TIM_CONFIG_TypeDef CONFIG;      /* 0x00: configuration; bit meanings
                                   * are only inferred from the driver. */
    volatile const C2_TIM_VALUE_TypeDef VALUE; /* 0x04: documented as count; actual
                                   * behavior unverified by the driver. */
    volatile C2_TIM_DATA_TypeDef DATA;        /* 0x08: write reload / read live down-count. */
} C2_TIM_TypeDef;

#define C2_TIM0_BASE UINT32_C(0x10002000)
#define C2_TIM1_BASE UINT32_C(0x10003000)
#define C2_TIM0 ((C2_TIM_TypeDef *)(uintptr_t)C2_TIM0_BASE)
#define C2_TIM1 ((C2_TIM_TypeDef *)(uintptr_t)C2_TIM1_BASE)

#define C2_TIMER_CONFIG_LOAD UINT32_C(0x00000100)
#define C2_TIMER_CONFIG_RUN  UINT32_C(0x00000101)

#ifdef __cplusplus
extern "C" {
#endif

/* Load reload and start the verified CONFIG=0x100, DATA, CONFIG=0x101
 * sequence. reload is the register value, so it represents reload + 1
 * timer clocks. Timer_0 and Timer_1 share this register type.
 */
c2_status_t c2_timer_start(C2_TIM_TypeDef *timer, uint32_t reload);

/* Stop through the SDK-observed load/stopped state. This does not promise to
 * preserve a prior count for later resume; resume behavior is undocumented.
 */
c2_status_t c2_timer_stop(C2_TIM_TypeDef *timer);

/* Read DATA once. VALUE is deliberately unused because its behavior has not
 * been verified. current receives the live hardware down-count snapshot.
 */
c2_status_t c2_timer_current(const C2_TIM_TypeDef *timer, uint32_t *current);

/* Run the timer from UINT32_MAX as a software-visible modulo-2^32 up-counter.
 * A timer dedicated to this timebase must not also be used for delays/reloads.
 */
c2_status_t c2_timer_systick_init(C2_TIM_TypeDef *timer);
c2_status_t c2_timer_systick_now(const C2_TIM_TypeDef *timer, uint32_t *tick);

/* Read the current systick and return (now - start) modulo 2^32. The result is
 * valid only when the interval spans at most one hardware wrap.
 */
c2_status_t c2_timer_systick_elapsed(const C2_TIM_TypeDef *timer,
                                     uint32_t start,
                                     uint32_t *elapsed);

/* Blocking, polling delays. clock_hz is the timer input clock, not MHz.
 * delay==0 returns C2_OK immediately without validating other arguments or
 * touching MMIO. For nonzero delay, clock_hz and poll_limit must both be
 * nonzero; poll_limit is the total DATA-read budget across all 32-bit chunks.
 * Exhaustion stops the timer and returns C2_ERROR_TIMEOUT. Conversion rounds
 * up to a timer clock. Long delays are safely split into <=2^32-clock chunks.
 * Completion observes DATA==0 or an observed down-counter reload transition,
 * so a sampled wrap cannot create an unbounded wait. A wrap wholly between
 * samples can only consume the finite budget and end in timeout.
 *
 * These calls take exclusive ownership of the selected timer until return.
 * All APIs require external serialization; no OS, heap, IRQ, or global clock
 * is used. Practical accuracy also requires a poll budget large enough for
 * the requested time and platform MMIO speed.
 */
c2_status_t c2_timer_delay_us(C2_TIM_TypeDef *timer,
                              uint32_t clock_hz,
                              uint32_t delay_us,
                              uint32_t poll_limit);
c2_status_t c2_timer_delay_ms(C2_TIM_TypeDef *timer,
                              uint32_t clock_hz,
                              uint32_t delay_ms,
                              uint32_t poll_limit);

#ifdef __cplusplus
}
#endif

#endif /* STARRYSKY_C2_TIMER_H */
