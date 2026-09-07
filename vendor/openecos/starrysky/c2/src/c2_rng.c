/* See works/docs/periphs/rng.md sections 3.1-3.3. */
#include "c2_rng.h"

c2_status_t c2_rng_init(C2_RNG_TypeDef *reg, uint32_t seed)
{
    if (reg == NULL || seed == 0u) {
        return C2_ERROR_INVALID_ARGUMENT;
    }
    c2_mmio_write32(&reg->CTRL.WORD, 1u);
    c2_mmio_write32(&reg->SEED.WORD, seed);
    c2_mmio_write32(&reg->CTRL.WORD, 0u);
    return C2_OK;
}

c2_status_t c2_rng_seed_write_enable(C2_RNG_TypeDef *reg, bool enable)
{
    if (reg == NULL) {
        return C2_ERROR_INVALID_ARGUMENT;
    }
    c2_mmio_write32(&reg->CTRL.WORD, enable ? 1u : 0u);
    return C2_OK;
}

c2_status_t c2_rng_seed(C2_RNG_TypeDef *reg, uint32_t seed)
{
    uint32_t control;
    if (reg == NULL) {
        return C2_ERROR_INVALID_ARGUMENT;
    }
    control = c2_mmio_read32(&reg->CTRL.WORD) & 1u;
    c2_mmio_write32(&reg->CTRL.WORD, 1u);
    c2_mmio_write32(&reg->SEED.WORD, seed);
    c2_mmio_write32(&reg->CTRL.WORD, control);
    return C2_OK;
}

c2_status_t c2_rng_stop(C2_RNG_TypeDef *reg)
{
    if (reg == NULL) {
        return C2_ERROR_INVALID_ARGUMENT;
    }
    /* EN=0 alone does not stop the generator. Zero is an absorbing state. */
    c2_mmio_write32(&reg->CTRL.WORD, 1u);
    c2_mmio_write32(&reg->SEED.WORD, 0u);
    c2_mmio_write32(&reg->CTRL.WORD, 0u);
    return C2_OK;
}

c2_status_t c2_rng_read(C2_RNG_TypeDef *reg, uint32_t *value)
{
    if (reg == NULL || value == NULL) {
        return C2_ERROR_INVALID_ARGUMENT;
    }
    *value = c2_mmio_read32(&reg->VAL.WORD);
    return C2_OK;
}

c2_status_t c2_rng_fill(C2_RNG_TypeDef *reg, void *data, size_t length)
{
    uint8_t *bytes = (uint8_t *)data;
    if (reg == NULL || (length != 0u && data == NULL)) {
        return C2_ERROR_INVALID_ARGUMENT;
    }
    while (length != 0u) {
        uint32_t sample = c2_mmio_read32(&reg->VAL.WORD);
        unsigned int i;
        for (i = 0u; i < 4u && length != 0u; ++i) {
            *bytes++ = (uint8_t)sample;
            sample >>= 8;
            --length;
        }
    }
    return C2_OK;
}
