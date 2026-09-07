/* Evidence-bounded C2 QSPI driver. See c2_qspi.h for the two status modes. */
#include "c2_qspi.h"

static bool c2_qspi_cs_valid(c2_qspi_chip_select_t cs)
{
    return cs == C2_QSPI_CS0 || cs == C2_QSPI_CS1 ||
           cs == C2_QSPI_CS2 || cs == C2_QSPI_CS3;
}

static c2_status_t c2_qspi_poll_sdk_idle(C2_QSPI_TypeDef *reg,
                                         uint32_t *remaining)
{
    while (*remaining != 0u) {
        uint32_t status;
        --(*remaining);
        status = c2_mmio_read32(&reg->STATUS.WORD);
        if ((status & C2_QSPI_STATUS_SDK_BUSY_MASK) == 0u) {
            return C2_OK;
        }
    }
    return C2_ERROR_TIMEOUT;
}

c2_status_t c2_qspi_init(C2_QSPI_TypeDef *reg, uint32_t clkdiv)
{
    if (reg == NULL) {
        return C2_ERROR_INVALID_ARGUMENT;
    }
    c2_mmio_write32(&reg->CLKDIV.WORD, clkdiv);
    c2_mmio_write32(&reg->CMD.WORD, 0u);
    c2_mmio_write32(&reg->ADR.WORD, 0u);
    c2_mmio_write32(&reg->LEN.WORD, 0u);
    return C2_OK;
}

c2_status_t c2_qspi_set_clkdiv(C2_QSPI_TypeDef *reg, uint32_t clkdiv)
{
    if (reg == NULL) {
        return C2_ERROR_INVALID_ARGUMENT;
    }
    c2_mmio_write32(&reg->CLKDIV.WORD, clkdiv);
    return C2_OK;
}

c2_status_t c2_qspi_reset(C2_QSPI_TypeDef *reg)
{
    if (reg == NULL) {
        return C2_ERROR_INVALID_ARGUMENT;
    }
    c2_mmio_write32(&reg->STATUS.WORD, C2_QSPI_CONTROL_RESET_ASSERT);
    c2_mmio_write32(&reg->STATUS.WORD, 0u);
    return C2_OK;
}

c2_status_t c2_qspi_select(C2_QSPI_TypeDef *reg,
                           c2_qspi_chip_select_t cs,
                           uint32_t poll_limit)
{
    c2_status_t status;
    if (reg == NULL || !c2_qspi_cs_valid(cs)) {
        return C2_ERROR_INVALID_ARGUMENT;
    }
    status = c2_qspi_poll_sdk_idle(reg, &poll_limit);
    if (status != C2_OK) {
        return status;
    }
    c2_mmio_write32(&reg->CMD.WORD, (uint32_t)cs);
    return C2_OK;
}

c2_status_t c2_qspi_read_status(C2_QSPI_TypeDef *reg, uint32_t *status)
{
    if (reg == NULL || status == NULL) {
        return C2_ERROR_INVALID_ARGUMENT;
    }
    *status = c2_mmio_read32(&reg->STATUS.WORD);
    return C2_OK;
}

c2_status_t c2_qspi_wait_sdk_idle(C2_QSPI_TypeDef *reg, uint32_t poll_limit)
{
    if (reg == NULL) {
        return C2_ERROR_INVALID_ARGUMENT;
    }
    return c2_qspi_poll_sdk_idle(reg, &poll_limit);
}

c2_status_t c2_qspi_fifo_write_word(C2_QSPI_TypeDef *reg,
                                    uint32_t word,
                                    uint32_t poll_limit)
{
    c2_status_t status;
    if (reg == NULL) {
        return C2_ERROR_INVALID_ARGUMENT;
    }
    status = c2_qspi_poll_sdk_idle(reg, &poll_limit);
    if (status != C2_OK) {
        return status;
    }
    c2_mmio_write32(&reg->TXFIFO.WORD, word);
    return C2_OK;
}

c2_status_t c2_qspi_fifo_read_word_raw(C2_QSPI_TypeDef *reg, uint32_t *word)
{
    if (reg == NULL || word == NULL) {
        return C2_ERROR_INVALID_ARGUMENT;
    }
    *word = c2_mmio_read32(&reg->RXFIFO.WORD);
    return C2_OK;
}

c2_status_t c2_qspi_write_bytes(C2_QSPI_TypeDef *reg,
                                const uint8_t *data,
                                size_t length,
                                uint32_t poll_limit,
                                size_t *bytes_submitted)
{
    uint32_t remaining = poll_limit;
    size_t submitted = 0u;
    c2_status_t status;

    if (bytes_submitted != NULL) {
        *bytes_submitted = 0u;
    }
    if (reg == NULL || (length != 0u && data == NULL)) {
        return C2_ERROR_INVALID_ARGUMENT;
    }
    if (length == 0u) {
        return C2_OK;
    }
    /* At least one successful status read is needed before every byte and one
     * more after the final byte. Reject known-impossible budgets atomically. */
    if (length > (size_t)UINT32_MAX - 1u ||
        (size_t)poll_limit < length + 1u) {
        return C2_ERROR_INVALID_ARGUMENT;
    }

    while (submitted != length) {
        status = c2_qspi_poll_sdk_idle(reg, &remaining);
        if (status != C2_OK) {
            if (bytes_submitted != NULL) {
                *bytes_submitted = submitted;
            }
            return status;
        }
        c2_mmio_write32(&reg->TXFIFO.WORD, (uint32_t)data[submitted]);
        ++submitted;
    }

    status = c2_qspi_poll_sdk_idle(reg, &remaining);
    if (bytes_submitted != NULL) {
        *bytes_submitted = submitted;
    }
    return status;
}

c2_status_t c2_qspi_transaction_raw(C2_QSPI_TypeDef *reg,
                                    const c2_qspi_raw_transaction_t *transaction,
                                    uint32_t poll_limit,
                                    uint32_t *last_status)
{
    uint32_t i;
    uint32_t status = 0u;

    if (last_status != NULL) {
        *last_status = 0u;
    }
    if (reg == NULL || transaction == NULL || poll_limit == 0u ||
        transaction->completion_mask == 0u ||
        (transaction->write_mask & ~C2_QSPI_RAW_WRITE_ALL_MASK) != 0u) {
        return C2_ERROR_INVALID_ARGUMENT;
    }

    if ((transaction->write_mask & C2_QSPI_RAW_WRITE_CMD) != 0u) {
        c2_mmio_write32(&reg->CMD.WORD, transaction->command_word);
    }
    if ((transaction->write_mask & C2_QSPI_RAW_WRITE_ADR) != 0u) {
        c2_mmio_write32(&reg->ADR.WORD, transaction->address_word);
    }
    if ((transaction->write_mask & C2_QSPI_RAW_WRITE_LEN) != 0u) {
        c2_mmio_write32(&reg->LEN.WORD, transaction->length_word);
    }
    if ((transaction->write_mask & C2_QSPI_RAW_WRITE_DUM) != 0u) {
        c2_mmio_write32(&reg->DUM.WORD, transaction->dummy_word);
    }
    if ((transaction->write_mask & C2_QSPI_RAW_WRITE_TXFIFO) != 0u) {
        c2_mmio_write32(&reg->TXFIFO.WORD, transaction->tx_word);
    }
    c2_mmio_write32(&reg->STATUS.WORD, transaction->trigger_word);

    for (i = 0u; i < poll_limit; ++i) {
        status = c2_mmio_read32(&reg->STATUS.WORD);
        if ((status & transaction->completion_mask) ==
            (transaction->completion_value & transaction->completion_mask)) {
            if (last_status != NULL) {
                *last_status = status;
            }
            return C2_OK;
        }
    }
    if (last_status != NULL) {
        *last_status = status;
    }
    return C2_ERROR_TIMEOUT;
}
