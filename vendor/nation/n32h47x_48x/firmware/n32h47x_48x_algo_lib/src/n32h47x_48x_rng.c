#include <stddef.h>
#include <string.h>

#include "n32h47x_48x_rng.h"
#include "n32h47x_48x_algo_common.h"

/*
 * Reconstructed from n32h47x_48x_algo_lib v1.2.0 binary (n32h47x_48x_rng.a,
 * GCC arm with DWARF; cross-checked against the armclang
 * n32h47x_48x_rng_1.o). Both RNG paths run inside the shared Security
 * Algorithm Controller (SAC) block at 0x4002A000; see docs/registers_rng.md
 * for the register map and the disassembly evidence behind every access.
 *
 * Register names follow the DWARF SAC_TypeDef layout:
 *   SACCR@0x00 SACSR@0x04 SACSFR1@0x0C SACSFR2@0x10 SACSFR3@0x14
 * The RNG library never touches the SAC SRAM window (0x4002A400) or the
 * P1ADR/P2ADR/P3ADR/KADR/IADR/OADR pointer registers at 0x30..0x44.
 */

#define SAC_BASE 0x4002A000UL

#define SAC_CR_REG    (*(volatile uint32_t *)(SAC_BASE + 0x00)) /* Control */
#define SAC_SR_REG    (*(volatile uint32_t *)(SAC_BASE + 0x04)) /* Command/status */
#define SAC_SFR1_REG  (*(volatile uint32_t *)(SAC_BASE + 0x0C)) /* RNG status/trigger */
#define SAC_SFR2_REG  (*(volatile uint32_t *)(SAC_BASE + 0x10)) /* Seed write port */
#define SAC_SFR3_REG  (*(volatile uint32_t *)(SAC_BASE + 0x14)) /* Data read port */

/* SACCR: the library writes the same whole value 0x16 for TRNG and PRNG. */
#define SAC_CR_MODE_RNG 0x00000016UL /* bits 1,2,4; per-algorithm meaning unknown */

/* SACSR bits (RNG library only ever writes this register). */
#define SAC_SR_START 0x00000080UL /* issue one round after SACSFR1 idle */
#define SAC_SR_FLUSH 0x00000800UL /* TRNG init flush; wait idle afterwards */

/* SACSFR1 bits. During TRNG init the library also sets bits 1 and 8 (see
 * the three separate read-modify-writes in GetTrueRand_U32, net 0x182). */
#define SAC_SFR1_BUSY 0x00000080UL /* read: sampling busy; write: kick a round */
#define SAC_SFR1_SEED 0x00000001UL /* PRNG: latch the seed written to SACSFR2 */

/*
 * Attack error code actually returned by the v1.2.0 binary: 0x7A9DB86D,
 * which also matches the auto-incremented enum value in the vendor header
 * (ADDRNULL = 0x7A9DB86C followed by RNGATTACKED, RNG_TimeOutError). The
 * inc/ header previously carried a hand-expanded 0x7A9DB86F typo; it has
 * been corrected to 0x7A9DB86D, so the header enum is authoritative again.
 * See docs/registers_rng.md section 6.
 */

/* RCC (base 0x40021000) bits used by both paths, set in this fixed order. */
#define RCC_AHBPCLKEN_REG  (*(volatile uint32_t *)0x40021018UL) /* AHBPCLKEN */
#define RCC_CFG3_REG        (*(volatile uint32_t *)0x4002100CUL) /* CFG3 */
#define RCC_AHB1PCLKEN_REG  (*(volatile uint32_t *)0x4002103CUL) /* AHB1PCLKEN */

#define RCC_AHBPCLKEN_RNGCEN  0x00000010UL /* RNGC clock enable (CMSIS name) */
#define RCC_CFG3_TRNG1MEN     0x00000004UL /* TRNG 1M entropy clock enable */
#define RCC_AHB1PCLKEN_SACEN  0x00008000UL /* SAC clock enable */

/*
 * Library state (symbol names and initial values from the binary):
 *  - seedbuf: .data, 8 bytes, initial 64-bit default seed from the vendor;
 *  - upseedcount: 16-bit counter, wraps to 0 above 1000 (TRNG seeding once);
 *  - counterInner: run the TRNG entropy health check on the first call only.
 */
static uint32_t seedbuf[2] = {0x17B03B21UL, 0x346963FDUL};
static uint16_t upseedcount = 0U;
static uint32_t counterInner = 0U;

/* Kick one sampling round, wait until SACSFR1 busy clears, then START and
 * read one 32-bit word from SACSFR3. Budget is Alg_TimeOut_Counter polls;
 * on timeout the caller returns without restoring SACCR/SACSFR1, matching
 * the binary (error paths never restore). */
static uint32_t RNG_ReadWord(uint32_t *word)
{
    uint32_t i;

    SAC_SFR1_REG |= SAC_SFR1_BUSY;
    for (i = 0U; (SAC_SFR1_REG & SAC_SFR1_BUSY) != 0U; i++)
    {
        if (i > Alg_TimeOut_Counter)
        {
            return RNG_TimeOutError;
        }
    }
    SAC_SR_REG |= SAC_SR_START;
    *word = SAC_SFR3_REG;
    return RNG_OK;
}

/* Kick/wait/START rounds without reading SACSFR3: reproduced verbatim from
 * the last health-check stage, where the binary performs two blind rounds
 * (no data read) before testing tempBuf[1]. */
static uint32_t RNG_KickIdleStart(void)
{
    uint32_t i;

    SAC_SFR1_REG |= SAC_SFR1_BUSY;
    for (i = 0U; (SAC_SFR1_REG & SAC_SFR1_BUSY) != 0U; i++)
    {
        if (i > Alg_TimeOut_Counter)
        {
            return RNG_TimeOutError;
        }
    }
    SAC_SR_REG |= SAC_SR_START;
    return RNG_OK;
}

uint32_t GetTrueRand_U32(uint32_t *rand, uint32_t wordLen)
{
    uint32_t tempBuf[16];
    uint32_t i;
    uint32_t ret;
    uint32_t rRNGCTRL_back; /* saved SACCR */
    uint32_t rSACCRback;    /* saved SACSFR1 */

    memset(tempBuf, 0, sizeof(tempBuf));

    if (rand == NULL)
    {
        return ADDRNULL;
    }
    if (wordLen == 0U)
    {
        return LENError;
    }

    /* Clock setup: RNGC, TRNG 1M entropy clock, SAC. */
    RCC_AHBPCLKEN_REG |= RCC_AHBPCLKEN_RNGCEN;
    RCC_CFG3_REG |= RCC_CFG3_TRNG1MEN;
    RCC_AHB1PCLKEN_REG |= RCC_AHB1PCLKEN_SACEN;

    rRNGCTRL_back = SAC_CR_REG;
    SAC_CR_REG = SAC_CR_MODE_RNG;
    rSACCRback = SAC_SFR1_REG;
    SAC_SFR1_REG = 0U;
    SAC_SFR1_REG |= 0x02UL;
    SAC_SFR1_REG |= 0x100UL;
    SAC_SFR1_REG |= SAC_SFR1_BUSY;

    /* Flush, wait for idle, then one START round. */
    SAC_SR_REG |= SAC_SR_FLUSH;
    for (i = 0U; (SAC_SFR1_REG & SAC_SFR1_BUSY) != 0U; i++)
    {
        if (i > Alg_TimeOut_Counter)
        {
            return RNG_TimeOutError;
        }
    }
    SAC_SR_REG |= SAC_SR_START;

    if (counterInner == 0U)
    {
        /* First-call entropy health check: 16 -> 8 -> 4 -> 2 word stages.
         * Each stage passes as soon as its last collected word is nonzero. */
        for (i = 0U; i < 16U; i++)
        {
            ret = RNG_ReadWord(&tempBuf[i]);
            if (ret != RNG_OK)
            {
                return ret;
            }
        }
        if (tempBuf[15] == 0U)
        {
            SetZero_U32(tempBuf, 8U);
            for (i = 0U; i < 8U; i++)
            {
                ret = RNG_ReadWord(&tempBuf[i]);
                if (ret != RNG_OK)
                {
                    return ret;
                }
            }
            if (tempBuf[7] == 0U)
            {
                SetZero_U32(tempBuf, 4U);
                for (i = 0U; i < 4U; i++)
                {
                    ret = RNG_ReadWord(&tempBuf[i]);
                    if (ret != RNG_OK)
                    {
                        return ret;
                    }
                }
                if (tempBuf[3] == 0U)
                {
                    SetZero_U32(tempBuf, 2U);
                    /* Vendor binary quirk (kept for alignment): two blind
                     * rounds with no SACSFR3 read, then a test of tempBuf[1],
                     * which SetZero_U32 just cleared. Reaching this stage
                     * therefore always reports the attack code. */
                    for (i = 0U; i < 2U; i++)
                    {
                        ret = RNG_KickIdleStart();
                        if (ret != RNG_OK)
                        {
                            return ret;
                        }
                    }
                    if (tempBuf[1] == 0U)
                    {
                        return RNGATTACKED;
                    }
                }
            }
        }
        counterInner = 1U;
    }

    for (i = 0U; i < wordLen; i++)
    {
        ret = RNG_ReadWord(&rand[i]);
        if (ret != RNG_OK)
        {
            return ret;
        }
    }

    /* Restore in the binary's order: SACSFR1 first, then SACCR. */
    SAC_SFR1_REG = rSACCRback;
    SAC_CR_REG = rRNGCTRL_back;
    return RNG_OK;
}

uint32_t GetPseudoRand_U32(uint32_t *rand, uint32_t wordLen, uint32_t seed[2])
{
    uint32_t i;
    uint32_t word;
    uint32_t ret;
    uint32_t rRNGCTRL_back; /* saved SACCR */
    uint32_t rSACCRback;    /* saved SACSFR1 */

    if (rand == NULL)
    {
        return ADDRNULL;
    }
    if (wordLen == 0U)
    {
        return LENError;
    }

    /* Clock setup: RNGC, TRNG 1M entropy clock, SAC. */
    RCC_AHBPCLKEN_REG |= RCC_AHBPCLKEN_RNGCEN;
    RCC_CFG3_REG |= RCC_CFG3_TRNG1MEN;
    RCC_AHB1PCLKEN_REG |= RCC_AHB1PCLKEN_SACEN;

    rRNGCTRL_back = SAC_CR_REG;
    SAC_CR_REG = SAC_CR_MODE_RNG;
    rSACCRback = SAC_SFR1_REG;
    SAC_SFR1_REG = 0U;

    /* Seed policy (binary order):
     *  - first call ever (upseedcount == 0) with seed == NULL: pull a fresh
     *    TRNG seed into seedbuf; only RNG_TimeOutError propagates (other
     *    errors, including the attack code, are ignored and the previous
     *    seedbuf content is kept);
     *  - seed != NULL: explicit reseed via Cpy_U32;
     *  - otherwise: reuse the internal seedbuf (XOR-reseeded each call). */
    if (upseedcount == 0U)
    {
        if (seed == NULL)
        {
            ret = GetTrueRand_U32(seedbuf, 2U);
            if (ret == RNG_TimeOutError)
            {
                return RNG_TimeOutError;
            }
        }
        else
        {
            Cpy_U32(seedbuf, seed, 2U);
        }
    }
    else if (seed != NULL)
    {
        Cpy_U32(seedbuf, seed, 2U);
    }
    else
    {
        /* reuse seedbuf */
    }

    SAC_SFR1_REG |= SAC_SFR1_SEED;
    SAC_SFR2_REG = seedbuf[0]; /* 64-bit seed: two consecutive writes */
    SAC_SFR2_REG = seedbuf[1];

    upseedcount = (uint16_t)(upseedcount + 1U);
    if (upseedcount > 1000U)
    {
        upseedcount = 0U;
    }

    for (i = 0U; i < wordLen; i++)
    {
        ret = RNG_ReadWord(&rand[i]);
        if (ret != RNG_OK)
        {
            return ret;
        }
    }

    /* Reseed seedbuf for the next seedless call: XOR two fresh words. */
    for (i = 0U; i < 2U; i++)
    {
        ret = RNG_ReadWord(&word);
        if (ret != RNG_OK)
        {
            return ret;
        }
        seedbuf[i] ^= word;
    }

    /* Restore in the binary's order: SACSFR1 first, then SACCR. */
    SAC_SFR1_REG = rSACCRback;
    SAC_CR_REG = rRNGCTRL_back;
    return RNG_OK;
}

void RNG_Version(uint8_t *type, uint8_t *customer, uint8_t date[3], uint8_t *version)
{
    /* Unconditional writes, no NULL checks, identical in GCC and armclang
     * images. Version 0x10 encodes v1.0 per the header's BCD-ish scheme,
     * although the library ships as v1.2.0. */
    *type = 1U;
    *customer = 0U;
    date[0] = 25U; /* 2025 */
    date[1] = 3U;  /* March */
    date[2] = 19U;
    *version = 16U; /* 0x10 */
}
