/* StarrySky C2 RNG registers. Source: works/docs/periphs/rng.md.
 * LFSR pseudo-random generator; not a cryptographic entropy source.
 * C cannot enforce write-only access: do not read SEED.
 */
#ifndef STARRYSKY_C2_RNG_H
#define STARRYSKY_C2_RNG_H

#include "c2_register.h"

typedef union {
    uint32_t WORD;
    struct {
        uint32_t EN : 1; /* [0] */
        uint32_t RESERVED0 : 31; /* [31:1] */
    } BITS;
} C2_RNG_CTRL_TypeDef;
C2_REG_ASSERT_SIZE(C2_RNG_CTRL_TypeDef, 4);

typedef union {
    uint32_t WORD;
    struct {
        uint32_t SEED : 32; /* [31:0] */
    } BITS;
} C2_RNG_SEED_TypeDef;
C2_REG_ASSERT_SIZE(C2_RNG_SEED_TypeDef, 4);

typedef union {
    uint32_t WORD;
    struct {
        uint32_t VAL : 32; /* [31:0] */
    } BITS;
} C2_RNG_VAL_TypeDef;
C2_REG_ASSERT_SIZE(C2_RNG_VAL_TypeDef, 4);

typedef struct {
    volatile C2_RNG_CTRL_TypeDef CTRL;      /* 0x00: seed write enable. */
    volatile C2_RNG_SEED_TypeDef SEED;      /* 0x04: WO; seed write requires CTRL.EN. */
    volatile const C2_RNG_VAL_TypeDef VAL; /* 0x08: RO; current LFSR value. */
} C2_RNG_TypeDef;

#define C2_RNG0_BASE UINT32_C(0x20002000)
#define C2_RNG0 ((C2_RNG_TypeDef *)(uintptr_t)C2_RNG0_BASE)

#include "c2_common.h"

#ifdef __cplusplus
extern "C" {
#endif

/* EN gates seed writes only; it does NOT stop the running LFSR.
 * init requires a nonzero seed and leaves seed writes disabled.
 * seed() preserves the prior seed-write gate; seed=0 intentionally locks
 * the LFSR at zero (stop() does this and disables subsequent seed writes).
 * Samples are not guaranteed unique or cryptographically random. */
c2_status_t c2_rng_init(C2_RNG_TypeDef *reg, uint32_t seed);
c2_status_t c2_rng_seed_write_enable(C2_RNG_TypeDef *reg, bool enable);
c2_status_t c2_rng_seed(C2_RNG_TypeDef *reg, uint32_t seed);
c2_status_t c2_rng_stop(C2_RNG_TypeDef *reg);
c2_status_t c2_rng_read(C2_RNG_TypeDef *reg, uint32_t *value);
/* Fill bytes in little-endian sample order. The last partial sample is
 * discarded. length=0 permits data=NULL and performs no MMIO. */
c2_status_t c2_rng_fill(C2_RNG_TypeDef *reg, void *data, size_t length);

#ifdef __cplusplus
}
#endif

#endif /* STARRYSKY_C2_RNG_H */
