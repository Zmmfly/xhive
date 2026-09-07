/* C translation of the observed C2 PSRAM boot sequence.
 * No mapped-PSRAM access, global state, heap or OS service is used here.
 */
#include "c2_psram.h"

#define C2_PSRAM_COMMAND_RESET_ENABLE UINT32_C(0x66)
#define C2_PSRAM_COMMAND_RESET_DEVICE UINT32_C(0x99)
#define C2_PSRAM_COMMAND_ENTER_QPI UINT32_C(0x35)

static c2_status_t c2_psram_delay_call(c2_psram_delay_fn delay,
                                       void *context,
                                       c2_psram_delay_stage_t stage)
{
    if (delay == NULL) {
        return C2_OK;
    }
    return delay(context, stage);
}

static c2_status_t c2_psram_raw_command(C2_QSPI_TypeDef *qspi,
                                        uint32_t command,
                                        uint32_t *remaining_polls)
{
    uint32_t status;

    /* Do not launch a command that cannot be observed at least once. */
    if (*remaining_polls == 0u) {
        return C2_ERROR_TIMEOUT;
    }

    c2_mmio_write32(&qspi->LEN.WORD, C2_QSPI_RAW_PSRAM_BYTE_LENGTH);
    c2_mmio_write32(&qspi->TXFIFO.WORD, command << 24);
    c2_mmio_write32(&qspi->STATUS.WORD, C2_QSPI_RAW_PSRAM_TRIGGER);

    while (*remaining_polls != 0u) {
        --(*remaining_polls);
        status = c2_mmio_read32(&qspi->STATUS.WORD);
        if (status == C2_QSPI_RAW_PSRAM_DONE) {
            return C2_OK;
        }
    }
    return C2_ERROR_TIMEOUT;
}

c2_status_t c2_psram_init(C2_PSRAM_TypeDef *psram,
                          C2_QSPI_TypeDef *qspi,
                          c2_gpio_t *gpio,
                          uint32_t poll_limit,
                          c2_psram_delay_fn delay,
                          void *delay_context)
{
    uint32_t remaining_polls = poll_limit;
    c2_status_t status;

    /* Validate every dependency before the first destructive MMIO write. */
    if (psram == NULL || qspi == NULL || gpio == NULL) {
        return C2_ERROR_INVALID_ARGUMENT;
    }
    if (!gpio->initialized || gpio->regs == NULL) {
        return C2_ERROR_NOT_INITIALIZED;
    }
    if (poll_limit < 3u) {
        return C2_ERROR_INVALID_ARGUMENT;
    }
    if ((gpio->output_shadow & C2_PSRAM_QPI_MODE_GPIO_MASK) != 0u) {
        return C2_ERROR_BUSY;
    }

    /* Exact QSPI reset/config order from StarrySkyC2 start.S. Deliberately do
     * not clear CMD/ADR/LEN or touch INTSTA: the official raw path does not. */
    c2_mmio_write32(&qspi->STATUS.WORD, C2_QSPI_CONTROL_RESET_ASSERT);
    c2_mmio_write32(&qspi->STATUS.WORD, 0u);
    c2_mmio_write32(&qspi->INTCFG.WORD, 0u);
    c2_mmio_write32(&qspi->DUM.WORD, 0u);
    c2_mmio_write32(&qspi->CLKDIV.WORD, 1u);

    /* Shadow-safe adaptation of the official whole-port DDR=0/DR=0 boot
     * writes: latch bit15 low before exposing it as an output, preserving all
     * other GPIO shadow bits and never reading DR/DDR. */
    status = c2_gpio_clear_mask(gpio, C2_PSRAM_QPI_MODE_GPIO_MASK);
    if (status != C2_OK) {
        return status;
    }
    status = c2_gpio_set_direction_mask(gpio,
                                        C2_PSRAM_QPI_MODE_GPIO_MASK,
                                        C2_GPIO_DIRECTION_OUTPUT);
    if (status != C2_OK) {
        return status;
    }

    c2_mmio_write32(&psram->WC.WORD, C2_PSRAM_INIT_WC);
    c2_mmio_write32(&psram->CHD.WORD, C2_PSRAM_INIT_CHD);

    status = c2_psram_delay_call(delay, delay_context,
                                 C2_PSRAM_DELAY_BEFORE_RESET_ENABLE);
    if (status != C2_OK) {
        return status;
    }

    status = c2_psram_raw_command(qspi, C2_PSRAM_COMMAND_RESET_ENABLE,
                                  &remaining_polls);
    if (status != C2_OK) {
        return status;
    }
    status = c2_psram_delay_call(delay, delay_context,
                                 C2_PSRAM_DELAY_AFTER_RESET_ENABLE);
    if (status != C2_OK) {
        return status;
    }

    status = c2_psram_raw_command(qspi, C2_PSRAM_COMMAND_RESET_DEVICE,
                                  &remaining_polls);
    if (status != C2_OK) {
        return status;
    }
    status = c2_psram_delay_call(delay, delay_context,
                                 C2_PSRAM_DELAY_AFTER_DEVICE_RESET);
    if (status != C2_OK) {
        return status;
    }

    status = c2_psram_raw_command(qspi, C2_PSRAM_COMMAND_ENTER_QPI,
                                  &remaining_polls);
    if (status != C2_OK) {
        return status;
    }
    status = c2_psram_delay_call(delay, delay_context,
                                 C2_PSRAM_DELAY_AFTER_ENTER_QPI);
    if (status != C2_OK) {
        return status;
    }

    /* The bridge is switched to mapped QPI only after every command and guard
     * has completed. Running timing is the final write pair. */
    status = c2_gpio_set_mask(gpio, C2_PSRAM_QPI_MODE_GPIO_MASK);
    if (status != C2_OK) {
        return status;
    }
    c2_mmio_write32(&psram->WC.WORD, C2_PSRAM_RUN_WC);
    c2_mmio_write32(&psram->CHD.WORD, C2_PSRAM_RUN_CHD);
    return C2_OK;
}
