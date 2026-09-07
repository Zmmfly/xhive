/* StarrySky C2 PWM registers. Source: works/docs/periphs/pwm.md.
 * 32-bit accesses; PSCR/CMP/CR0..3 have 16-bit effective values.
 */
#ifndef STARRYSKY_C2_PWM_H
#define STARRYSKY_C2_PWM_H

#include "c2_common.h"
#include "c2_register.h"

typedef union {
    uint32_t WORD;
    struct {
        uint32_t OVIE : 1; /* [0] */
        uint32_t EN : 1; /* [1] */
        uint32_t CLR : 1; /* [2] */
        uint32_t RESERVED0 : 29; /* [31:3] */
    } BITS;
} C2_PWM_CTRL_TypeDef;
C2_REG_ASSERT_SIZE(C2_PWM_CTRL_TypeDef, 4);

typedef union {
    uint32_t WORD;
    struct {
        uint32_t PSCR : 16; /* [15:0] */
        uint32_t RESERVED0 : 16; /* [31:16] */
    } BITS;
} C2_PWM_PSCR_TypeDef;
C2_REG_ASSERT_SIZE(C2_PWM_PSCR_TypeDef, 4);

/* Documentation-only internal counter field; no software counter access at this address. */
typedef union {
    uint32_t WORD;
    struct {
        uint32_t CNT : 16; /* [15:0] */
        uint32_t RESERVED0 : 16; /* [31:16] */
    } BITS;
} C2_PWM_CNT_TypeDef;
C2_REG_ASSERT_SIZE(C2_PWM_CNT_TypeDef, 4);

typedef union {
    uint32_t WORD;
    struct {
        uint32_t CMP : 16; /* [15:0] */
        uint32_t RESERVED0 : 16; /* [31:16] */
    } BITS;
} C2_PWM_CMP_TypeDef;
C2_REG_ASSERT_SIZE(C2_PWM_CMP_TypeDef, 4);

typedef union {
    uint32_t WORD;
    struct {
        uint32_t CR0 : 16; /* [15:0] */
        uint32_t RESERVED0 : 16; /* [31:16] */
    } BITS;
} C2_PWM_CR0_TypeDef;
C2_REG_ASSERT_SIZE(C2_PWM_CR0_TypeDef, 4);

typedef union {
    uint32_t WORD;
    struct {
        uint32_t CR1 : 16; /* [15:0] */
        uint32_t RESERVED0 : 16; /* [31:16] */
    } BITS;
} C2_PWM_CR1_TypeDef;
C2_REG_ASSERT_SIZE(C2_PWM_CR1_TypeDef, 4);

typedef union {
    uint32_t WORD;
    struct {
        uint32_t CR2 : 16; /* [15:0] */
        uint32_t RESERVED0 : 16; /* [31:16] */
    } BITS;
} C2_PWM_CR2_TypeDef;
C2_REG_ASSERT_SIZE(C2_PWM_CR2_TypeDef, 4);

typedef union {
    uint32_t WORD;
    struct {
        uint32_t CR3 : 16; /* [15:0] */
        uint32_t RESERVED0 : 16; /* [31:16] */
    } BITS;
} C2_PWM_CR3_TypeDef;
C2_REG_ASSERT_SIZE(C2_PWM_CR3_TypeDef, 4);

typedef union {
    uint32_t WORD;
    struct {
        uint32_t OVIF : 1; /* [0] */
        uint32_t RESERVED0 : 31; /* [31:1] */
    } BITS;
} C2_PWM_STAT_TypeDef;
C2_REG_ASSERT_SIZE(C2_PWM_STAT_TypeDef, 4);

typedef struct {
    volatile C2_PWM_CTRL_TypeDef CTRL;      /* 0x00: enable, counter clear, IRQ enable. */
    volatile C2_PWM_PSCR_TypeDef PSCR;      /* 0x04: divider = PSCR + 1. */
    const C2_PWM_CNT_TypeDef RESERVED_CNT; /* 0x08: CNT address exists in SDK, but
                                 * hardware exposes no counter access. */
    volatile C2_PWM_CMP_TypeDef CMP;       /* 0x0C: period in counter cycles. */
    volatile C2_PWM_CR0_TypeDef CR0;       /* 0x10: channel 0 compare. */
    volatile C2_PWM_CR1_TypeDef CR1;       /* 0x14: channel 1 compare. */
    volatile C2_PWM_CR2_TypeDef CR2;       /* 0x18: channel 2 compare. */
    volatile C2_PWM_CR3_TypeDef CR3;       /* 0x1C: channel 3 compare. */
    volatile const C2_PWM_STAT_TypeDef STAT; /* 0x20: overflow status; read clears. */
} C2_PWM_TypeDef;

#define C2_PWM0_BASE UINT32_C(0x20004000)
#define C2_PWM0 ((C2_PWM_TypeDef *)(uintptr_t)C2_PWM0_BASE)

#define C2_PWM_CTRL_OVIE UINT32_C(0x00000001)
#define C2_PWM_CTRL_EN   UINT32_C(0x00000002)
#define C2_PWM_CTRL_CLR  UINT32_C(0x00000004)
#define C2_PWM_FIELD_MAX UINT32_C(0x0000FFFF)
#define C2_PWM_DUTY_SCALE UINT32_C(10000)
#define C2_PWM_CHANNEL_COUNT 4U

typedef enum {
    C2_PWM_CHANNEL_0 = 0,
    C2_PWM_CHANNEL_1 = 1,
    C2_PWM_CHANNEL_2 = 2,
    C2_PWM_CHANNEL_3 = 3
} c2_pwm_channel_t;

/* Raw register values. period is CMP and must be 1..65535. prescaler is PSCR
 * and must be 0..65535; the divider is prescaler + 1. compare[channel] may be
 * 0..period. Since output is CNT >= CR, high clocks = period - compare:
 * compare=period is 0%, compare=0 is 100%.
 */
typedef struct {
    uint32_t prescaler;
    uint32_t period;
    uint32_t compare[C2_PWM_CHANNEL_COUNT];
} c2_pwm_config_t;

typedef struct {
    uint32_t raw;
    bool overflow;
} c2_pwm_irq_snapshot_t;

#ifdef __cplusplus
extern "C" {
#endif

/* Program PSCR, CMP, then CR0..CR3 using full-width writes. No implicit
 * enable/disable/clear is performed. Running reconfiguration is non-atomic:
 * disable first when a coherent multi-register transition is required.
 */
c2_status_t c2_pwm_init_raw(C2_PWM_TypeDef *pwm,
                            const c2_pwm_config_t *config);

/* Calculate raw values for f = clock_hz / (PSCR + 1) / CMP and four high-level
 * duties in units of 1/10000 (0..10000 inclusive). The smallest divider that
 * permits a 16-bit nonzero CMP is chosen (or the maximum divider when the
 * ideal product exceeds both fields), then CMP is rounded to nearest for
 * useful duty resolution. actual_frequency_hz may be NULL and, when present,
 * receives integer-truncated hardware frequency. No MMIO is touched.
 */
c2_status_t c2_pwm_calculate_config(uint32_t clock_hz,
                                    uint32_t frequency_hz,
                                    const uint32_t duty[C2_PWM_CHANNEL_COUNT],
                                    c2_pwm_config_t *config,
                                    uint32_t *actual_frequency_hz);

/* Calculate and write a complete configuration. config_out and
 * actual_frequency_hz are optional. PWM remains in its existing run state;
 * therefore a running update has the same non-atomic limitation as raw init.
 */
c2_status_t c2_pwm_configure(C2_PWM_TypeDef *pwm,
                             uint32_t clock_hz,
                             uint32_t frequency_hz,
                             const uint32_t duty[C2_PWM_CHANNEL_COUNT],
                             c2_pwm_config_t *config_out,
                             uint32_t *actual_frequency_hz);

/* Raw compare accepts 0..current CMP. Duty uses 1/10000 units and converts
 * according to high clocks = CMP - CR, including exact 0% and 100% endpoints.
 * Each operation updates one channel only and is not synchronized to rollover.
 */
c2_status_t c2_pwm_set_compare(C2_PWM_TypeDef *pwm,
                               c2_pwm_channel_t channel,
                               uint32_t compare);
c2_status_t c2_pwm_set_duty(C2_PWM_TypeDef *pwm,
                            c2_pwm_channel_t channel,
                            uint32_t duty);

/* CTRL operations preserve only the documented EN/OVIE state and use whole
 * 32-bit accesses. They are read-modify-write transactions and require
 * external serialization. clear pulses CLR then restores EN/OVIE; CNT itself
 * is deliberately never accessed because its APB read path is not implemented.
 */
c2_status_t c2_pwm_enable(C2_PWM_TypeDef *pwm);
c2_status_t c2_pwm_disable(C2_PWM_TypeDef *pwm);
c2_status_t c2_pwm_clear(C2_PWM_TypeDef *pwm);
c2_status_t c2_pwm_irq_enable(C2_PWM_TypeDef *pwm, bool enable);

/* STAT is read exactly once. Hardware clears OVIF as a side effect of that
 * read, so all consumers must use this returned snapshot. This only manages
 * the peripheral source; it does not install or claim a CPU interrupt route.
 */
c2_status_t c2_pwm_irq_snapshot(C2_PWM_TypeDef *pwm,
                                c2_pwm_irq_snapshot_t *snapshot);

#ifdef __cplusplus
}
#endif

#endif /* STARRYSKY_C2_PWM_H */
