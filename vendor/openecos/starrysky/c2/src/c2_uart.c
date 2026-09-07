#include "c2_uart.h"

#define C2_SYS_UART_RX_INVALID_MASK UINT32_C(0xffffff00)
#define C2_HP_UART_LCR_FRAME_MASK   UINT32_C(0x1f8)
#define C2_HP_UART_FCR_FLUSH_MASK   UINT32_C(0x003)
#define C2_HP_UART_FCR_TRIGGER_MASK UINT32_C(0x00c)

static c2_status_t c2_sys_uart_validate(const c2_sys_uart_t *uart)
{
    if (uart == NULL) {
        return C2_ERROR_INVALID_ARGUMENT;
    }
    if (!uart->initialized || uart->regs == NULL) {
        return C2_ERROR_NOT_INITIALIZED;
    }
    return C2_OK;
}

static c2_status_t c2_hp_uart_validate(const c2_hp_uart_t *uart)
{
    if (uart == NULL) {
        return C2_ERROR_INVALID_ARGUMENT;
    }
    if (!uart->initialized || uart->regs == NULL) {
        return C2_ERROR_NOT_INITIALIZED;
    }
    return C2_OK;
}

static c2_status_t c2_sys_uart_divider(uint32_t clock_hz,
                                       uint32_t baud_rate,
                                       uint32_t *divider)
{
    uint32_t value;

    if (clock_hz == 0u || baud_rate == 0u || divider == NULL) {
        return C2_ERROR_INVALID_ARGUMENT;
    }
    value = clock_hz / baud_rate;
    if (value == 0u) {
        return C2_ERROR_INVALID_ARGUMENT;
    }
    *divider = value;
    return C2_OK;
}

static c2_status_t c2_hp_uart_encode(uint32_t clock_hz,
                                     const c2_hp_uart_config_t *config,
                                     uint32_t *divider,
                                     uint32_t *lcr,
                                     uint32_t *fcr_trigger)
{
    uint32_t quotient;
    uint32_t encoded_lcr;

    if (config == NULL || divider == NULL || lcr == NULL ||
        fcr_trigger == NULL || clock_hz == 0u || config->baud_rate == 0u ||
        config->data_bits < 5u || config->data_bits > 8u ||
        (config->stop_bits != C2_UART_STOP_BITS_1 &&
         config->stop_bits != C2_UART_STOP_BITS_2) ||
        (config->parity != C2_UART_PARITY_NONE &&
         config->parity != C2_UART_PARITY_ODD &&
         config->parity != C2_UART_PARITY_EVEN) ||
        (uint32_t)config->rx_trigger >
            (uint32_t)C2_HP_UART_RX_TRIGGER_14_BYTES ||
        (config->irq_enable_mask & ~C2_HP_UART_IRQ_ALL) != 0u) {
        return C2_ERROR_INVALID_ARGUMENT;
    }

    quotient = clock_hz / config->baud_rate;
    /* RTL documents DIV >= 2 and a 16-bit DIV field. */
    if (quotient < 3u || quotient > UINT32_C(65536)) {
        return C2_ERROR_INVALID_ARGUMENT;
    }

    encoded_lcr = ((uint32_t)(config->data_bits - 5u) << 3);
    if (config->stop_bits == C2_UART_STOP_BITS_2) {
        encoded_lcr |= UINT32_C(1) << 5;
    }
    if (config->parity != C2_UART_PARITY_NONE) {
        encoded_lcr |= UINT32_C(1) << 6;
        if (config->parity == C2_UART_PARITY_EVEN) {
            encoded_lcr |= UINT32_C(1) << 7;
        }
    }
    encoded_lcr |= config->irq_enable_mask;

    *divider = quotient - 1u;
    *lcr = encoded_lcr;
    *fcr_trigger = ((uint32_t)config->rx_trigger << 2) &
                   C2_HP_UART_FCR_TRIGGER_MASK;
    return C2_OK;
}

static c2_status_t c2_hp_uart_apply_config(c2_hp_uart_t *uart,
                                            uint32_t clock_hz,
                                            const c2_hp_uart_config_t *config)
{
    uint32_t divider;
    uint32_t lcr;
    uint32_t fcr_trigger;
    c2_status_t status;

    status = c2_hp_uart_encode(clock_hz, config, &divider, &lcr,
                               &fcr_trigger);
    if (status != C2_OK) {
        return status;
    }

    c2_mmio_write32(&uart->regs->LCR.WORD, 0u);
    c2_mmio_write32(&uart->regs->DIV.WORD, divider);
    c2_mmio_write32(&uart->regs->FCR.WORD,
                    fcr_trigger | C2_HP_UART_FCR_FLUSH_MASK);
    c2_mmio_write32(&uart->regs->FCR.WORD, fcr_trigger);
    c2_mmio_write32(&uart->regs->LCR.WORD, lcr);

    uart->lcr_shadow = lcr;
    uart->fcr_trigger_shadow = fcr_trigger;
    uart->initialized = true;
    return C2_OK;
}

c2_status_t c2_sys_uart_init(c2_sys_uart_t *uart,
                             C2_SYS_UART_TypeDef *regs,
                             uint32_t clock_hz,
                             uint32_t baud_rate)
{
    uint32_t divider;
    c2_status_t status;

    if (uart == NULL || regs == NULL) {
        return C2_ERROR_INVALID_ARGUMENT;
    }
    status = c2_sys_uart_divider(clock_hz, baud_rate, &divider);
    if (status != C2_OK) {
        return status;
    }

    uart->regs = regs;
    uart->initialized = false;
    c2_mmio_write32(&regs->CLKDIV.WORD, divider);
    uart->initialized = true;
    return C2_OK;
}

c2_status_t c2_sys_uart_configure(c2_sys_uart_t *uart,
                                  uint32_t clock_hz,
                                  uint32_t baud_rate)
{
    uint32_t divider;
    c2_status_t status = c2_sys_uart_validate(uart);

    if (status != C2_OK) {
        return status;
    }
    status = c2_sys_uart_divider(clock_hz, baud_rate, &divider);
    if (status != C2_OK) {
        return status;
    }
    c2_mmio_write32(&uart->regs->CLKDIV.WORD, divider);
    return C2_OK;
}

c2_status_t c2_sys_uart_disable(c2_sys_uart_t *uart)
{
    c2_status_t status = c2_sys_uart_validate(uart);

    if (status != C2_OK) {
        return status;
    }
    uart->initialized = false;
    return C2_OK;
}

c2_status_t c2_sys_uart_write_byte(c2_sys_uart_t *uart, uint8_t byte)
{
    c2_status_t status = c2_sys_uart_validate(uart);

    if (status != C2_OK) {
        return status;
    }
    c2_mmio_write32(&uart->regs->DATA.WORD, (uint32_t)byte);
    return C2_OK;
}

c2_status_t c2_sys_uart_write(c2_sys_uart_t *uart,
                              const uint8_t *data,
                              size_t length,
                              size_t *written)
{
    c2_status_t status = c2_sys_uart_validate(uart);
    size_t index;

    if (status != C2_OK) {
        return status;
    }
    if (written == NULL || (data == NULL && length != 0u)) {
        return C2_ERROR_INVALID_ARGUMENT;
    }

    *written = 0u;
    for (index = 0u; index < length; ++index) {
        c2_mmio_write32(&uart->regs->DATA.WORD, (uint32_t)data[index]);
        *written = index + 1u;
    }
    return C2_OK;
}

c2_status_t c2_sys_uart_read_byte_nonblocking(c2_sys_uart_t *uart,
                                              uint8_t *byte)
{
    c2_status_t status = c2_sys_uart_validate(uart);
    uint32_t raw;

    if (status != C2_OK) {
        return status;
    }
    if (byte == NULL) {
        return C2_ERROR_INVALID_ARGUMENT;
    }

    raw = c2_mmio_read32(&uart->regs->DATA.WORD);
    if ((raw & C2_SYS_UART_RX_INVALID_MASK) != 0u) {
        return C2_ERROR_BUSY;
    }
    *byte = (uint8_t)raw;
    return C2_OK;
}

c2_status_t c2_sys_uart_read_byte(c2_sys_uart_t *uart,
                                  uint8_t *byte,
                                  uint32_t poll_limit)
{
    c2_status_t status = c2_sys_uart_validate(uart);
    uint32_t poll;

    if (status != C2_OK) {
        return status;
    }
    if (byte == NULL) {
        return C2_ERROR_INVALID_ARGUMENT;
    }

    for (poll = 0u; poll < poll_limit; ++poll) {
        uint32_t raw = c2_mmio_read32(&uart->regs->DATA.WORD);
        if ((raw & C2_SYS_UART_RX_INVALID_MASK) == 0u) {
            *byte = (uint8_t)raw;
            return C2_OK;
        }
    }
    return C2_ERROR_TIMEOUT;
}

c2_status_t c2_sys_uart_read(c2_sys_uart_t *uart,
                             uint8_t *data,
                             size_t length,
                             size_t *read_count,
                             uint32_t poll_limit)
{
    c2_status_t status = c2_sys_uart_validate(uart);
    uint32_t polls = 0u;

    if (status != C2_OK) {
        return status;
    }
    if (read_count == NULL || (data == NULL && length != 0u)) {
        return C2_ERROR_INVALID_ARGUMENT;
    }

    *read_count = 0u;
    while (*read_count < length && polls < poll_limit) {
        uint32_t raw = c2_mmio_read32(&uart->regs->DATA.WORD);
        ++polls;
        if ((raw & C2_SYS_UART_RX_INVALID_MASK) == 0u) {
            data[*read_count] = (uint8_t)raw;
            ++(*read_count);
        }
    }
    return (*read_count == length) ? C2_OK : C2_ERROR_TIMEOUT;
}

c2_status_t c2_hp_uart_init(c2_hp_uart_t *uart,
                            C2_HP_UART_TypeDef *regs,
                            uint32_t clock_hz,
                            const c2_hp_uart_config_t *config)
{
    c2_hp_uart_t candidate;
    c2_status_t status;

    if (uart == NULL || regs == NULL) {
        return C2_ERROR_INVALID_ARGUMENT;
    }

    /* Configure through a temporary context so invalid arguments do not
     * destroy a previously initialized caller context. */
    candidate.regs = regs;
    candidate.lcr_shadow = 0u;
    candidate.fcr_trigger_shadow = 0u;
    candidate.initialized = false;
    status = c2_hp_uart_apply_config(&candidate, clock_hz, config);
    if (status == C2_OK) {
        *uart = candidate;
    }
    return status;
}

c2_status_t c2_hp_uart_configure(c2_hp_uart_t *uart,
                                 uint32_t clock_hz,
                                 const c2_hp_uart_config_t *config)
{
    c2_status_t status = c2_hp_uart_validate(uart);

    if (status != C2_OK) {
        return status;
    }
    return c2_hp_uart_apply_config(uart, clock_hz, config);
}

c2_status_t c2_hp_uart_disable(c2_hp_uart_t *uart)
{
    c2_status_t status = c2_hp_uart_validate(uart);

    if (status != C2_OK) {
        return status;
    }
    uart->lcr_shadow &= C2_HP_UART_LCR_FRAME_MASK;
    c2_mmio_write32(&uart->regs->LCR.WORD, uart->lcr_shadow);
    uart->initialized = false;
    return C2_OK;
}

c2_status_t c2_hp_uart_get_status(c2_hp_uart_t *uart, uint32_t *status_word)
{
    c2_status_t status = c2_hp_uart_validate(uart);

    if (status != C2_OK) {
        return status;
    }
    if (status_word == NULL) {
        return C2_ERROR_INVALID_ARGUMENT;
    }
    *status_word = c2_mmio_read32(&uart->regs->LSR.WORD) &
                   C2_HP_UART_STATUS_ALL;
    return C2_OK;
}

c2_status_t c2_hp_uart_set_irq_mask(c2_hp_uart_t *uart,
                                    uint32_t irq_enable_mask)
{
    c2_status_t status = c2_hp_uart_validate(uart);

    if (status != C2_OK) {
        return status;
    }
    if ((irq_enable_mask & ~C2_HP_UART_IRQ_ALL) != 0u) {
        return C2_ERROR_INVALID_ARGUMENT;
    }

    uart->lcr_shadow = (uart->lcr_shadow & ~C2_HP_UART_IRQ_ALL) |
                       irq_enable_mask;
    c2_mmio_write32(&uart->regs->LCR.WORD, uart->lcr_shadow);
    return C2_OK;
}

c2_status_t c2_hp_uart_flush(c2_hp_uart_t *uart,
                             c2_hp_uart_flush_t kind)
{
    c2_status_t status = c2_hp_uart_validate(uart);
    uint32_t flush_bits = (uint32_t)kind;

    if (status != C2_OK) {
        return status;
    }
    if (flush_bits == 0u ||
        (flush_bits & ~C2_HP_UART_FCR_FLUSH_MASK) != 0u) {
        return C2_ERROR_INVALID_ARGUMENT;
    }

    c2_mmio_write32(&uart->regs->FCR.WORD,
                    uart->fcr_trigger_shadow | flush_bits);
    c2_mmio_write32(&uart->regs->FCR.WORD, uart->fcr_trigger_shadow);
    return C2_OK;
}

c2_status_t c2_hp_uart_write_byte_nonblocking(c2_hp_uart_t *uart,
                                              uint8_t byte)
{
    c2_status_t status = c2_hp_uart_validate(uart);
    uint32_t lsr;

    if (status != C2_OK) {
        return status;
    }
    lsr = c2_mmio_read32(&uart->regs->LSR.WORD);
    if ((lsr & C2_HP_UART_STATUS_TX_FIFO_FULL) != 0u) {
        return C2_ERROR_BUSY;
    }
    c2_mmio_write32(&uart->regs->TRX.WORD, (uint32_t)byte);
    return C2_OK;
}

c2_status_t c2_hp_uart_write_byte(c2_hp_uart_t *uart,
                                  uint8_t byte,
                                  uint32_t poll_limit)
{
    c2_status_t status = c2_hp_uart_validate(uart);
    uint32_t poll;

    if (status != C2_OK) {
        return status;
    }
    for (poll = 0u; poll < poll_limit; ++poll) {
        uint32_t lsr = c2_mmio_read32(&uart->regs->LSR.WORD);
        if ((lsr & C2_HP_UART_STATUS_TX_FIFO_FULL) == 0u) {
            c2_mmio_write32(&uart->regs->TRX.WORD, (uint32_t)byte);
            return C2_OK;
        }
    }
    return C2_ERROR_TIMEOUT;
}

c2_status_t c2_hp_uart_wait_tx_complete(c2_hp_uart_t *uart,
                                        uint32_t poll_limit)
{
    c2_status_t status = c2_hp_uart_validate(uart);
    uint32_t poll;

    if (status != C2_OK) {
        return status;
    }
    for (poll = 0u; poll < poll_limit; ++poll) {
        uint32_t lsr = c2_mmio_read32(&uart->regs->LSR.WORD);
        if ((lsr & C2_HP_UART_STATUS_TX_IDLE) != 0u) {
            return C2_OK;
        }
    }
    return C2_ERROR_TIMEOUT;
}

c2_status_t c2_hp_uart_write(c2_hp_uart_t *uart,
                             const uint8_t *data,
                             size_t length,
                             size_t *written,
                             uint32_t poll_limit)
{
    c2_status_t status = c2_hp_uart_validate(uart);
    uint32_t polls = 0u;

    if (status != C2_OK) {
        return status;
    }
    if (written == NULL || (data == NULL && length != 0u)) {
        return C2_ERROR_INVALID_ARGUMENT;
    }

    *written = 0u;
    while (*written < length && polls < poll_limit) {
        uint32_t lsr = c2_mmio_read32(&uart->regs->LSR.WORD);
        ++polls;
        if ((lsr & C2_HP_UART_STATUS_TX_FIFO_FULL) == 0u) {
            c2_mmio_write32(&uart->regs->TRX.WORD,
                            (uint32_t)data[*written]);
            ++(*written);
        }
    }
    return (*written == length) ? C2_OK : C2_ERROR_TIMEOUT;
}

static c2_status_t c2_hp_uart_read_ready(c2_hp_uart_t *uart,
                                         uint8_t *byte,
                                         uint32_t lsr,
                                         c2_status_t unavailable_status)
{
    if ((lsr & C2_HP_UART_STATUS_RX_FIFO_EMPTY) != 0u) {
        return unavailable_status;
    }
    *byte = (uint8_t)c2_mmio_read32(&uart->regs->TRX.WORD);
    return ((lsr & C2_HP_UART_STATUS_PARITY_ERROR) != 0u) ?
           C2_ERROR_IO : C2_OK;
}

c2_status_t c2_hp_uart_read_byte_nonblocking(c2_hp_uart_t *uart,
                                             uint8_t *byte)
{
    c2_status_t status = c2_hp_uart_validate(uart);
    uint32_t lsr;

    if (status != C2_OK) {
        return status;
    }
    if (byte == NULL) {
        return C2_ERROR_INVALID_ARGUMENT;
    }

    lsr = c2_mmio_read32(&uart->regs->LSR.WORD);
    return c2_hp_uart_read_ready(uart, byte, lsr, C2_ERROR_BUSY);
}

c2_status_t c2_hp_uart_read_byte(c2_hp_uart_t *uart,
                                 uint8_t *byte,
                                 uint32_t poll_limit)
{
    c2_status_t status = c2_hp_uart_validate(uart);
    uint32_t poll;

    if (status != C2_OK) {
        return status;
    }
    if (byte == NULL) {
        return C2_ERROR_INVALID_ARGUMENT;
    }

    for (poll = 0u; poll < poll_limit; ++poll) {
        uint32_t lsr = c2_mmio_read32(&uart->regs->LSR.WORD);
        if ((lsr & C2_HP_UART_STATUS_RX_FIFO_EMPTY) == 0u) {
            return c2_hp_uart_read_ready(uart, byte, lsr,
                                         C2_ERROR_TIMEOUT);
        }
    }
    return C2_ERROR_TIMEOUT;
}

c2_status_t c2_hp_uart_read(c2_hp_uart_t *uart,
                            uint8_t *data,
                            size_t length,
                            size_t *read_count,
                            uint32_t poll_limit)
{
    c2_status_t status = c2_hp_uart_validate(uart);
    uint32_t polls = 0u;

    if (status != C2_OK) {
        return status;
    }
    if (read_count == NULL || (data == NULL && length != 0u)) {
        return C2_ERROR_INVALID_ARGUMENT;
    }

    *read_count = 0u;
    while (*read_count < length && polls < poll_limit) {
        uint32_t lsr = c2_mmio_read32(&uart->regs->LSR.WORD);
        ++polls;
        if ((lsr & C2_HP_UART_STATUS_RX_FIFO_EMPTY) == 0u) {
            data[*read_count] =
                (uint8_t)c2_mmio_read32(&uart->regs->TRX.WORD);
            ++(*read_count);
            if ((lsr & C2_HP_UART_STATUS_PARITY_ERROR) != 0u) {
                return C2_ERROR_IO;
            }
        }
    }
    return (*read_count == length) ? C2_OK : C2_ERROR_TIMEOUT;
}
