/* See works/docs/periphs/ps2.md: EN=bit1, ITN=bit0, STAT read clears. */
#include "c2_ps2.h"

#define PS2_ITN UINT32_C(1)
#define PS2_EN  UINT32_C(2)

c2_status_t c2_ps2_init(C2_PS2_TypeDef *reg, bool irq_enable)
{
    if (reg == NULL) {
        return C2_ERROR_INVALID_ARGUMENT;
    }
    c2_mmio_write32(&reg->CTRL.WORD, PS2_EN | (irq_enable ? PS2_ITN : 0u));
    return C2_OK;
}

c2_status_t c2_ps2_deinit(C2_PS2_TypeDef *reg)
{
    if (reg == NULL) {
        return C2_ERROR_INVALID_ARGUMENT;
    }
    c2_mmio_write32(&reg->CTRL.WORD, 0u);
    return C2_OK;
}

c2_status_t c2_ps2_enable(C2_PS2_TypeDef *reg, bool enable)
{
    uint32_t ctrl;
    if (reg == NULL) {
        return C2_ERROR_INVALID_ARGUMENT;
    }
    ctrl = c2_mmio_read32(&reg->CTRL.WORD) & PS2_ITN;
    c2_mmio_write32(&reg->CTRL.WORD, ctrl | (enable ? PS2_EN : 0u));
    return C2_OK;
}

c2_status_t c2_ps2_irq_enable(C2_PS2_TypeDef *reg, bool enable)
{
    uint32_t ctrl;
    if (reg == NULL) {
        return C2_ERROR_INVALID_ARGUMENT;
    }
    ctrl = c2_mmio_read32(&reg->CTRL.WORD) & PS2_EN;
    c2_mmio_write32(&reg->CTRL.WORD, ctrl | (enable ? PS2_ITN : 0u));
    return C2_OK;
}

c2_status_t c2_ps2_status_ack(C2_PS2_TypeDef *reg, bool *pending)
{
    if (reg == NULL || pending == NULL) {
        return C2_ERROR_INVALID_ARGUMENT;
    }
    *pending = (c2_mmio_read32(&reg->STAT.WORD) & 1u) != 0u;
    return C2_OK;
}

c2_status_t c2_ps2_read_raw(C2_PS2_TypeDef *reg, uint8_t *data)
{
    if (reg == NULL || data == NULL) {
        return C2_ERROR_INVALID_ARGUMENT;
    }
    *data = (uint8_t)c2_mmio_read32(&reg->DATA.WORD);
    return C2_OK;
}

c2_status_t c2_ps2_try_read(C2_PS2_TypeDef *reg, uint8_t *data)
{
    uint8_t value;
    if (reg == NULL || data == NULL) {
        return C2_ERROR_INVALID_ARGUMENT;
    }
    value = (uint8_t)c2_mmio_read32(&reg->DATA.WORD);
    if (value == 0u) {
        return C2_ERROR_BUSY;
    }
    *data = value;
    return C2_OK;
}

c2_status_t c2_ps2_receive(C2_PS2_TypeDef *reg, uint8_t *data, size_t length,
                           size_t *received, uint32_t poll_limit)
{
    size_t done = 0u;
    if (received == NULL) {
        return C2_ERROR_INVALID_ARGUMENT;
    }
    *received = 0u;
    if (reg == NULL || (length != 0u && data == NULL)) {
        return C2_ERROR_INVALID_ARGUMENT;
    }
    while (done < length && poll_limit != 0u) {
        uint8_t value = (uint8_t)c2_mmio_read32(&reg->DATA.WORD);
        --poll_limit;
        if (value != 0u) {
            data[done++] = value;
        }
    }
    *received = done;
    return done == length ? C2_OK : C2_ERROR_TIMEOUT;
}
