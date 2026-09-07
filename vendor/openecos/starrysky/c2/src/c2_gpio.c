#include "c2_gpio.h"

static c2_status_t c2_gpio_validate(const c2_gpio_t *gpio)
{
    if (gpio == NULL) {
        return C2_ERROR_INVALID_ARGUMENT;
    }
    if (!gpio->initialized || gpio->regs == NULL) {
        return C2_ERROR_NOT_INITIALIZED;
    }
    return C2_OK;
}

static bool c2_gpio_mask_is_valid(uint32_t pin_mask)
{
    return (pin_mask & ~C2_GPIO_PIN_MASK) == 0u;
}

c2_status_t c2_gpio_init(c2_gpio_t *gpio,
                         C2_GPIO_TypeDef *regs,
                         uint32_t output_mask,
                         uint32_t initial_output_bits)
{
    uint32_t input_mask;

    if (gpio == NULL || regs == NULL || !c2_gpio_mask_is_valid(output_mask) ||
        !c2_gpio_mask_is_valid(initial_output_bits)) {
        return C2_ERROR_INVALID_ARGUMENT;
    }

    input_mask = (~output_mask) & C2_GPIO_PIN_MASK;

    gpio->regs = regs;
    gpio->output_shadow = initial_output_bits;
    gpio->input_mask_shadow = C2_GPIO_PIN_MASK;
    gpio->initialized = false;

    /* Avoid output glitches: disconnect every output, seed its latch, then
     * expose only the requested output pins. */
    c2_mmio_write32(&regs->DDR.WORD, C2_GPIO_PIN_MASK);
    c2_mmio_write32(&regs->DR.WORD, initial_output_bits);
    c2_mmio_write32(&regs->DDR.WORD, input_mask);

    gpio->input_mask_shadow = input_mask;
    gpio->initialized = true;
    return C2_OK;
}

c2_status_t c2_gpio_set_direction_mask(c2_gpio_t *gpio,
                                       uint32_t pin_mask,
                                       c2_gpio_direction_t direction)
{
    c2_status_t status = c2_gpio_validate(gpio);

    if (status != C2_OK) {
        return status;
    }
    if (!c2_gpio_mask_is_valid(pin_mask) ||
        (direction != C2_GPIO_DIRECTION_INPUT &&
         direction != C2_GPIO_DIRECTION_OUTPUT)) {
        return C2_ERROR_INVALID_ARGUMENT;
    }
    if (pin_mask == 0u) {
        return C2_OK;
    }

    if (direction == C2_GPIO_DIRECTION_INPUT) {
        gpio->input_mask_shadow |= pin_mask;
    } else {
        gpio->input_mask_shadow &= ~pin_mask;
    }
    gpio->input_mask_shadow &= C2_GPIO_PIN_MASK;
    c2_mmio_write32(&gpio->regs->DDR.WORD, gpio->input_mask_shadow);
    return C2_OK;
}

c2_status_t c2_gpio_write_mask(c2_gpio_t *gpio,
                               uint32_t pin_mask,
                               uint32_t value_bits)
{
    c2_status_t status = c2_gpio_validate(gpio);

    if (status != C2_OK) {
        return status;
    }
    if (!c2_gpio_mask_is_valid(pin_mask) ||
        !c2_gpio_mask_is_valid(value_bits)) {
        return C2_ERROR_INVALID_ARGUMENT;
    }
    if (pin_mask == 0u) {
        return C2_OK;
    }

    gpio->output_shadow = (gpio->output_shadow & ~pin_mask) |
                          (value_bits & pin_mask);
    gpio->output_shadow &= C2_GPIO_PIN_MASK;
    c2_mmio_write32(&gpio->regs->DR.WORD, gpio->output_shadow);
    return C2_OK;
}

c2_status_t c2_gpio_set_mask(c2_gpio_t *gpio, uint32_t pin_mask)
{
    return c2_gpio_write_mask(gpio, pin_mask, pin_mask);
}

c2_status_t c2_gpio_clear_mask(c2_gpio_t *gpio, uint32_t pin_mask)
{
    return c2_gpio_write_mask(gpio, pin_mask, 0u);
}

c2_status_t c2_gpio_toggle_mask(c2_gpio_t *gpio, uint32_t pin_mask)
{
    c2_status_t status = c2_gpio_validate(gpio);

    if (status != C2_OK) {
        return status;
    }
    if (!c2_gpio_mask_is_valid(pin_mask)) {
        return C2_ERROR_INVALID_ARGUMENT;
    }
    if (pin_mask == 0u) {
        return C2_OK;
    }

    gpio->output_shadow ^= pin_mask;
    gpio->output_shadow &= C2_GPIO_PIN_MASK;
    c2_mmio_write32(&gpio->regs->DR.WORD, gpio->output_shadow);
    return C2_OK;
}

c2_status_t c2_gpio_read_inputs(c2_gpio_t *gpio,
                                uint32_t pin_mask,
                                uint32_t *value_bits)
{
    c2_status_t status = c2_gpio_validate(gpio);
    uint32_t sample;

    if (status != C2_OK) {
        return status;
    }
    if (value_bits == NULL || !c2_gpio_mask_is_valid(pin_mask) ||
        (pin_mask & ~gpio->input_mask_shadow) != 0u) {
        return C2_ERROR_INVALID_ARGUMENT;
    }
    if (pin_mask == 0u) {
        *value_bits = 0u;
        return C2_OK;
    }

    sample = c2_mmio_read32(&gpio->regs->DR.WORD);
    *value_bits = sample & pin_mask & C2_GPIO_PIN_MASK;
    return C2_OK;
}

c2_status_t c2_gpio_get_shadow(const c2_gpio_t *gpio,
                               uint32_t *input_mask,
                               uint32_t *output_bits)
{
    c2_status_t status = c2_gpio_validate(gpio);

    if (status != C2_OK) {
        return status;
    }
    if (input_mask == NULL || output_bits == NULL) {
        return C2_ERROR_INVALID_ARGUMENT;
    }

    *input_mask = gpio->input_mask_shadow;
    *output_bits = gpio->output_shadow;
    return C2_OK;
}
