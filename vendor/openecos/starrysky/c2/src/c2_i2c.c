#include "c2_i2c.h"

static bool c2_i2c_regs_valid(const C2_I2C_TypeDef *regs)
{
    return (regs != NULL) && ((((uintptr_t)regs) & (sizeof(uint32_t) - 1U)) == 0U);
}

static uint32_t c2_i2c_read_status(const C2_I2C_TypeDef *regs)
{
    return c2_mmio_read32(&regs->SR.WORD);
}

static c2_status_t c2_i2c_clear_if(C2_I2C_TypeDef *regs,
                                    uint32_t status,
                                    uint32_t poll_limit)
{
    uint32_t poll;

    if ((status & C2_I2C_STATUS_IF) == 0U) {
        return C2_OK;
    }
    if ((status & C2_I2C_STATUS_TIP) != 0U) {
        return C2_ERROR_BUSY;
    }

    c2_mmio_write32(&regs->CMD.WORD, C2_I2C_CMD_IACK);
    for (poll = 0U; poll < poll_limit; ++poll) {
        if ((c2_i2c_read_status(regs) & C2_I2C_STATUS_IF) == 0U) {
            return C2_OK;
        }
    }

    return C2_ERROR_TIMEOUT;
}

static c2_status_t c2_i2c_issue_command(C2_I2C_TypeDef *regs,
                                         uint32_t command,
                                         bool check_nack,
                                         uint32_t poll_limit,
                                         uint32_t *completion_status)
{
    c2_status_t result;
    uint32_t status;
    uint32_t poll;

    status = c2_i2c_read_status(regs);
    if ((status & C2_I2C_STATUS_TIP) != 0U) {
        return C2_ERROR_BUSY;
    }
    if (((status & C2_I2C_STATUS_AL) != 0U) &&
        ((command & C2_I2C_CMD_STA) == 0U)) {
        return C2_ERROR_ARBITRATION;
    }

    result = c2_i2c_clear_if(regs, status, poll_limit);
    if (result != C2_OK) {
        return result;
    }

    c2_mmio_write32(&regs->CMD.WORD, command);
    for (poll = 0U; poll < poll_limit; ++poll) {
        status = c2_i2c_read_status(regs);
        if ((status & C2_I2C_STATUS_AL) != 0U) {
            return C2_ERROR_ARBITRATION;
        }
        /* IF is latched by cmd_done, so it cannot be missed even if TIP rose
         * and fell entirely between two MMIO reads.
         */
        if ((status & C2_I2C_STATUS_IF) != 0U) {
            if (completion_status != NULL) {
                *completion_status = status;
            }
            if (check_nack && ((status & C2_I2C_STATUS_RXK) != 0U)) {
                return C2_ERROR_NACK;
            }
            return C2_OK;
        }
    }

    return C2_ERROR_TIMEOUT;
}

static c2_status_t c2_i2c_stop(C2_I2C_TypeDef *regs, uint32_t poll_limit)
{
    c2_status_t result;
    uint32_t status = 0U;
    uint32_t poll;

    result = c2_i2c_issue_command(regs, C2_I2C_CMD_STO, false,
                                  poll_limit, &status);
    if (result != C2_OK) {
        status = c2_i2c_read_status(regs);
        if (((status & C2_I2C_STATUS_IF) != 0U) &&
            ((status & C2_I2C_STATUS_TIP) == 0U)) {
            (void)c2_i2c_clear_if(regs, status, poll_limit);
        }
        return result;
    }

    if ((status & C2_I2C_STATUS_BSY) != 0U) {
        for (poll = 0U; poll < poll_limit; ++poll) {
            status = c2_i2c_read_status(regs);
            if ((status & C2_I2C_STATUS_AL) != 0U) {
                if (((status & C2_I2C_STATUS_IF) != 0U) &&
                    ((status & C2_I2C_STATUS_TIP) == 0U)) {
                    (void)c2_i2c_clear_if(regs, status, poll_limit);
                }
                return C2_ERROR_ARBITRATION;
            }
            if ((status & C2_I2C_STATUS_BSY) == 0U) {
                break;
            }
        }
        if ((status & C2_I2C_STATUS_BSY) != 0U) {
            (void)c2_i2c_clear_if(regs, status, poll_limit);
            return C2_ERROR_TIMEOUT;
        }
    }

    return c2_i2c_clear_if(regs, status, poll_limit);
}

static void c2_i2c_cleanup_failure(C2_I2C_TypeDef *regs,
                                    c2_status_t failure,
                                    uint32_t poll_limit)
{
    uint32_t status;
    uint32_t poll;

    status = c2_i2c_read_status(regs);
    if ((failure == C2_ERROR_ARBITRATION) ||
        ((status & C2_I2C_STATUS_AL) != 0U)) {
        /* Losing arbitration releases the output drivers in RTL. A STOP here
         * could corrupt the winning master's transaction; only clear IF.
         */
        if (((status & C2_I2C_STATUS_IF) != 0U) &&
            ((status & C2_I2C_STATUS_TIP) == 0U)) {
            (void)c2_i2c_clear_if(regs, status, poll_limit);
        }
        return;
    }

    /* Do not overwrite CMD while the timed-out command is still active. Give
     * it one additional bounded completion window before attempting STOP.
     */
    for (poll = 0U;
         ((status & C2_I2C_STATUS_TIP) != 0U) && (poll < poll_limit);
         ++poll) {
        status = c2_i2c_read_status(regs);
        if ((status & C2_I2C_STATUS_AL) != 0U) {
            if (((status & C2_I2C_STATUS_IF) != 0U) &&
                ((status & C2_I2C_STATUS_TIP) == 0U)) {
                (void)c2_i2c_clear_if(regs, status, poll_limit);
            }
            return;
        }
    }

    if ((status & C2_I2C_STATUS_TIP) == 0U) {
        (void)c2_i2c_stop(regs, poll_limit);
    }
}

static c2_status_t c2_i2c_begin(C2_I2C_TypeDef *regs)
{
    uint32_t control;
    uint32_t status;

    control = c2_mmio_read32(&regs->CTRL.WORD);
    if ((control & C2_I2C_CTRL_EN) == 0U) {
        return C2_ERROR_NOT_INITIALIZED;
    }

    status = c2_i2c_read_status(regs);
    if ((status & (C2_I2C_STATUS_TIP | C2_I2C_STATUS_BSY)) != 0U) {
        return C2_ERROR_BUSY;
    }

    return C2_OK;
}

static c2_status_t c2_i2c_send_byte(C2_I2C_TypeDef *regs,
                                     uint8_t byte,
                                     uint32_t command,
                                     uint32_t poll_limit)
{
    c2_mmio_write32(&regs->TXR.WORD, (uint32_t)byte);
    return c2_i2c_issue_command(regs, command, true, poll_limit, NULL);
}

static c2_status_t c2_i2c_receive_byte(C2_I2C_TypeDef *regs,
                                        uint8_t *byte,
                                        bool acknowledge,
                                        uint32_t poll_limit)
{
    c2_status_t result;
    uint32_t command = C2_I2C_CMD_RD;

    if (acknowledge) {
        command |= C2_I2C_CMD_ACK;
    }
    result = c2_i2c_issue_command(regs, command, false, poll_limit, NULL);
    if (result == C2_OK) {
        *byte = (uint8_t)c2_mmio_read32(&regs->RXR.WORD);
    }
    return result;
}

c2_status_t c2_i2c_configure_prescaler(C2_I2C_TypeDef *regs,
                                         uint32_t prescaler)
{
    uint32_t status;
    uint32_t control;

    if (!c2_i2c_regs_valid(regs) ||
        (prescaler > C2_I2C_PRESCALER_MAX)) {
        return C2_ERROR_INVALID_ARGUMENT;
    }

    status = c2_i2c_read_status(regs);
    if ((status & (C2_I2C_STATUS_TIP | C2_I2C_STATUS_BSY)) != 0U) {
        return C2_ERROR_BUSY;
    }

    control = c2_mmio_read32(&regs->CTRL.WORD);
    c2_mmio_write32(&regs->CTRL.WORD, control & C2_I2C_CTRL_IEN);
    c2_mmio_write32(&regs->PSCR.WORD, prescaler);
    return C2_OK;
}

c2_status_t c2_i2c_configure(C2_I2C_TypeDef *regs,
                              uint32_t clock_hz,
                              uint32_t bus_hz,
                              uint16_t *prescaler_out)
{
    c2_status_t result;
    uint64_t denominator;
    uint64_t divider;
    uint32_t prescaler;

    if (!c2_i2c_regs_valid(regs) || (clock_hz == 0U) || (bus_hz == 0U)) {
        return C2_ERROR_INVALID_ARGUMENT;
    }

    denominator = (uint64_t)bus_hz * UINT64_C(5);
    divider = ((uint64_t)clock_hz + denominator - UINT64_C(1)) / denominator;
    if (divider == 0U) {
        divider = 1U;
    }
    if (divider > (UINT64_C(1) << 16)) {
        return C2_ERROR_INVALID_ARGUMENT;
    }
    prescaler = (uint32_t)(divider - UINT64_C(1));

    result = c2_i2c_configure_prescaler(regs, prescaler);
    if ((result == C2_OK) && (prescaler_out != NULL)) {
        *prescaler_out = (uint16_t)prescaler;
    }
    return result;
}

c2_status_t c2_i2c_enable(C2_I2C_TypeDef *regs)
{
    uint32_t control;

    if (!c2_i2c_regs_valid(regs)) {
        return C2_ERROR_INVALID_ARGUMENT;
    }

    control = c2_mmio_read32(&regs->CTRL.WORD);
    if ((control & C2_I2C_CTRL_EN) == 0U) {
        c2_mmio_write32(&regs->CTRL.WORD, control | C2_I2C_CTRL_EN);
    }
    return C2_OK;
}

c2_status_t c2_i2c_disable(C2_I2C_TypeDef *regs)
{
    uint32_t status;
    uint32_t control;

    if (!c2_i2c_regs_valid(regs)) {
        return C2_ERROR_INVALID_ARGUMENT;
    }

    status = c2_i2c_read_status(regs);
    if ((status & (C2_I2C_STATUS_TIP | C2_I2C_STATUS_BSY)) != 0U) {
        return C2_ERROR_BUSY;
    }
    control = c2_mmio_read32(&regs->CTRL.WORD);
    if ((control & C2_I2C_CTRL_EN) != 0U) {
        c2_mmio_write32(&regs->CTRL.WORD, control & ~C2_I2C_CTRL_EN);
    }
    return C2_OK;
}

c2_status_t c2_i2c_set_interrupt_enabled(C2_I2C_TypeDef *regs,
                                          bool enabled)
{
    uint32_t control;
    uint32_t new_control;

    if (!c2_i2c_regs_valid(regs)) {
        return C2_ERROR_INVALID_ARGUMENT;
    }

    control = c2_mmio_read32(&regs->CTRL.WORD);
    if (enabled) {
        new_control = control | C2_I2C_CTRL_IEN;
    } else {
        new_control = control & ~C2_I2C_CTRL_IEN;
    }
    if (new_control != control) {
        c2_mmio_write32(&regs->CTRL.WORD, new_control);
    }
    return C2_OK;
}

c2_status_t c2_i2c_get_status(const C2_I2C_TypeDef *regs,
                               c2_i2c_status_snapshot_t *snapshot)
{
    uint32_t status;

    if (!c2_i2c_regs_valid(regs) || (snapshot == NULL)) {
        return C2_ERROR_INVALID_ARGUMENT;
    }

    status = c2_i2c_read_status(regs);
    snapshot->raw = status;
    snapshot->interrupt_pending = (status & C2_I2C_STATUS_IF) != 0U;
    snapshot->transfer_in_progress = (status & C2_I2C_STATUS_TIP) != 0U;
    snapshot->arbitration_lost = (status & C2_I2C_STATUS_AL) != 0U;
    snapshot->bus_busy = (status & C2_I2C_STATUS_BSY) != 0U;
    snapshot->nack_received = (status & C2_I2C_STATUS_RXK) != 0U;
    return C2_OK;
}

c2_status_t c2_i2c_ack_interrupt(C2_I2C_TypeDef *regs,
                                  uint32_t poll_limit)
{
    uint32_t status;

    if (!c2_i2c_regs_valid(regs) || (poll_limit == 0U)) {
        return C2_ERROR_INVALID_ARGUMENT;
    }

    status = c2_i2c_read_status(regs);
    return c2_i2c_clear_if(regs, status, poll_limit);
}

c2_status_t c2_i2c_master_write(C2_I2C_TypeDef *regs,
                                 uint8_t address,
                                 const uint8_t *data,
                                 size_t length,
                                 uint32_t poll_limit)
{
    c2_status_t result;
    size_t index;

    if (!c2_i2c_regs_valid(regs) || (address > UINT8_C(0x7F)) ||
        (data == NULL) || (length == 0U) || (poll_limit == 0U)) {
        return C2_ERROR_INVALID_ARGUMENT;
    }

    result = c2_i2c_begin(regs);
    if (result != C2_OK) {
        return result;
    }

    result = c2_i2c_send_byte(regs, (uint8_t)(address << 1),
                              C2_I2C_CMD_STA | C2_I2C_CMD_WR, poll_limit);
    for (index = 0U; (result == C2_OK) && (index < length); ++index) {
        result = c2_i2c_send_byte(regs, data[index], C2_I2C_CMD_WR,
                                  poll_limit);
    }
    if (result == C2_OK) {
        result = c2_i2c_stop(regs, poll_limit);
    } else {
        c2_i2c_cleanup_failure(regs, result, poll_limit);
    }
    return result;
}

c2_status_t c2_i2c_master_read(C2_I2C_TypeDef *regs,
                                uint8_t address,
                                uint8_t *data,
                                size_t length,
                                uint32_t poll_limit)
{
    c2_status_t result;
    size_t index;

    if (!c2_i2c_regs_valid(regs) || (address > UINT8_C(0x7F)) ||
        (data == NULL) || (length == 0U) || (poll_limit == 0U)) {
        return C2_ERROR_INVALID_ARGUMENT;
    }

    result = c2_i2c_begin(regs);
    if (result != C2_OK) {
        return result;
    }

    result = c2_i2c_send_byte(regs, (uint8_t)((address << 1) | 1U),
                              C2_I2C_CMD_STA | C2_I2C_CMD_WR, poll_limit);
    for (index = 0U; (result == C2_OK) && (index < length); ++index) {
        result = c2_i2c_receive_byte(regs, &data[index],
                                     index + 1U < length, poll_limit);
    }
    if (result == C2_OK) {
        result = c2_i2c_stop(regs, poll_limit);
    } else {
        c2_i2c_cleanup_failure(regs, result, poll_limit);
    }
    return result;
}

c2_status_t c2_i2c_master_write_read(C2_I2C_TypeDef *regs,
                                      uint8_t address,
                                      const uint8_t *write_data,
                                      size_t write_length,
                                      uint8_t *read_data,
                                      size_t read_length,
                                      uint32_t poll_limit)
{
    c2_status_t result;
    size_t index;

    if (!c2_i2c_regs_valid(regs) || (address > UINT8_C(0x7F)) ||
        (write_data == NULL) || (write_length == 0U) ||
        (read_data == NULL) || (read_length == 0U) ||
        (poll_limit == 0U)) {
        return C2_ERROR_INVALID_ARGUMENT;
    }

    result = c2_i2c_begin(regs);
    if (result != C2_OK) {
        return result;
    }

    result = c2_i2c_send_byte(regs, (uint8_t)(address << 1),
                              C2_I2C_CMD_STA | C2_I2C_CMD_WR, poll_limit);
    for (index = 0U; (result == C2_OK) && (index < write_length); ++index) {
        result = c2_i2c_send_byte(regs, write_data[index], C2_I2C_CMD_WR,
                                  poll_limit);
    }
    if (result == C2_OK) {
        result = c2_i2c_send_byte(regs,
                                  (uint8_t)((address << 1) | 1U),
                                  C2_I2C_CMD_STA | C2_I2C_CMD_WR,
                                  poll_limit);
    }
    for (index = 0U; (result == C2_OK) && (index < read_length); ++index) {
        result = c2_i2c_receive_byte(regs, &read_data[index],
                                     index + 1U < read_length, poll_limit);
    }
    if (result == C2_OK) {
        result = c2_i2c_stop(regs, poll_limit);
    } else {
        c2_i2c_cleanup_failure(regs, result, poll_limit);
    }
    return result;
}
