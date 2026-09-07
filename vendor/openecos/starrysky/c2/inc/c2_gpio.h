/* StarrySky C2 GPIO registers. Source: works/docs/periphs/gpio.md.
 * Only the low 16 bits are valid. DDR: 1=input, 0=output.
 * Reads have hardware hazards: maintain software shadows of output and
 * direction values; do not use read-modify-write on DR or DDR.
 */
#ifndef STARRYSKY_C2_GPIO_H
#define STARRYSKY_C2_GPIO_H

#include "c2_common.h"
#include "c2_register.h"

/* PIN0..PIN15 map directly to GPIO channels; high bits are undefined. Use software shadows, not MMIO bit-field writes. */
typedef union {
    uint32_t WORD;
    struct {
        uint32_t PIN0 : 1; /* [0] */
        uint32_t PIN1 : 1; /* [1] */
        uint32_t PIN2 : 1; /* [2] */
        uint32_t PIN3 : 1; /* [3] */
        uint32_t PIN4 : 1; /* [4] */
        uint32_t PIN5 : 1; /* [5] */
        uint32_t PIN6 : 1; /* [6] */
        uint32_t PIN7 : 1; /* [7] */
        uint32_t PIN8 : 1; /* [8] */
        uint32_t PIN9 : 1; /* [9] */
        uint32_t PIN10 : 1; /* [10] */
        uint32_t PIN11 : 1; /* [11] */
        uint32_t PIN12 : 1; /* [12] */
        uint32_t PIN13 : 1; /* [13] */
        uint32_t PIN14 : 1; /* [14] */
        uint32_t PIN15 : 1; /* [15] */
        uint32_t UNKNOWN0 : 16; /* [31:16] */
    } BITS;
} C2_GPIO_DR_TypeDef;
C2_REG_ASSERT_SIZE(C2_GPIO_DR_TypeDef, 4);

/* PIN0..PIN15 map directly to GPIO channels; high bits are undefined. Use software shadows, not MMIO bit-field writes. */
typedef union {
    uint32_t WORD;
    struct {
        uint32_t PIN0 : 1; /* [0] */
        uint32_t PIN1 : 1; /* [1] */
        uint32_t PIN2 : 1; /* [2] */
        uint32_t PIN3 : 1; /* [3] */
        uint32_t PIN4 : 1; /* [4] */
        uint32_t PIN5 : 1; /* [5] */
        uint32_t PIN6 : 1; /* [6] */
        uint32_t PIN7 : 1; /* [7] */
        uint32_t PIN8 : 1; /* [8] */
        uint32_t PIN9 : 1; /* [9] */
        uint32_t PIN10 : 1; /* [10] */
        uint32_t PIN11 : 1; /* [11] */
        uint32_t PIN12 : 1; /* [12] */
        uint32_t PIN13 : 1; /* [13] */
        uint32_t PIN14 : 1; /* [14] */
        uint32_t PIN15 : 1; /* [15] */
        uint32_t UNKNOWN0 : 16; /* [31:16] */
    } BITS;
} C2_GPIO_DDR_TypeDef;
C2_REG_ASSERT_SIZE(C2_GPIO_DDR_TypeDef, 4);

typedef struct {
    volatile C2_GPIO_DR_TypeDef DR;  /* 0x00: write output / read sampled input. */
    volatile C2_GPIO_DDR_TypeDef DDR; /* 0x04: direction; reads are unreliable. */
} C2_GPIO_TypeDef;

#define C2_GPIO0_BASE UINT32_C(0x10000000)
#define C2_GPIO0 ((C2_GPIO_TypeDef *)(uintptr_t)C2_GPIO0_BASE)

#define C2_GPIO_PIN_MASK UINT32_C(0x0000ffff)

typedef enum {
    C2_GPIO_DIRECTION_OUTPUT = 0,
    C2_GPIO_DIRECTION_INPUT = 1
} c2_gpio_direction_t;

/* One context must have one exclusive owner. Zero-initialize its storage before
 * first use. The shadows are authoritative: output and direction changes never
 * read DR or DDR. */
typedef struct {
    C2_GPIO_TypeDef *regs;
    uint32_t output_shadow;
    uint32_t input_mask_shadow;
    bool initialized;
} c2_gpio_t;

#ifdef __cplusplus
extern "C" {
#endif

/* Safe startup sequence: make all pins inputs, program the output latch, then
 * apply output_mask. initial_output_bits may seed latches for input pins too. */
c2_status_t c2_gpio_init(c2_gpio_t *gpio,
                         C2_GPIO_TypeDef *regs,
                         uint32_t output_mask,
                         uint32_t initial_output_bits);

/* Applies direction to every selected pin. A zero pin_mask is a successful
 * no-op. Switching to output exposes the already-seeded output shadow. */
c2_status_t c2_gpio_set_direction_mask(c2_gpio_t *gpio,
                                       uint32_t pin_mask,
                                       c2_gpio_direction_t direction);

/* Masked output-latch operations. They may also seed an input pin's latch for
 * a later glitch-minimized switch to output. No operation reads GPIO MMIO. */
c2_status_t c2_gpio_write_mask(c2_gpio_t *gpio,
                               uint32_t pin_mask,
                               uint32_t value_bits);
c2_status_t c2_gpio_set_mask(c2_gpio_t *gpio, uint32_t pin_mask);
c2_status_t c2_gpio_clear_mask(c2_gpio_t *gpio, uint32_t pin_mask);
c2_status_t c2_gpio_toggle_mask(c2_gpio_t *gpio, uint32_t pin_mask);

/* Reads DR exactly once and returns only requested input pins. Requesting a
 * pin currently configured as output is invalid; use the output shadow. */
c2_status_t c2_gpio_read_inputs(c2_gpio_t *gpio,
                                uint32_t pin_mask,
                                uint32_t *value_bits);

/* Returns authoritative software shadows without touching MMIO. */
c2_status_t c2_gpio_get_shadow(const c2_gpio_t *gpio,
                               uint32_t *input_mask,
                               uint32_t *output_bits);

#ifdef __cplusplus
}
#endif

#endif /* STARRYSKY_C2_GPIO_H */
