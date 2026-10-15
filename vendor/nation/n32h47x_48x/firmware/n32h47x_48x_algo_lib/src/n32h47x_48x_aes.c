#include <stddef.h>

#include "n32h47x_48x_aes.h"
#include "n32h47x_48x_algo_common.h"

/*
 * Reconstructed from n32h47x_48x_algo_lib v1.2.0 binary (n32h47x_48x_aes.a,
 * GCC arm with DWARF). The AES engine lives in the shared Security Algorithm
 * Controller (SAC) block at 0x4002A000; see docs/registers_aes.md for the
 * register map and the disassembly evidence behind every access below.
 *
 * Register names follow the DWARF SAC_TypeDef layout:
 *   SACCR@0x00 SACSR@0x04 SACSFR1@0x0C P1ADR@0x30 KADR@0x3C IADR@0x40 OADR@0x44
 */

#define SAC_BASE 0x4002A000UL

#define SAC_CR_REG   (*(volatile uint32_t *)(SAC_BASE + 0x00)) /* Control */
#define SAC_SR_REG   (*(volatile uint32_t *)(SAC_BASE + 0x04)) /* Status flags */
#define SAC_SFR1_REG (*(volatile uint32_t *)(SAC_BASE + 0x0C)) /* AES config */

/* Data ports: one 32-bit little-endian word per access, FIFO ordered. */
#define SAC_P1_FIFO_REG (*(volatile uint32_t *)(SAC_BASE + 0x30)) /* IV in, 4 words */
#define SAC_K_FIFO_REG  (*(volatile uint32_t *)(SAC_BASE + 0x3C)) /* key in, keyWordLen words */
#define SAC_I_FIFO_REG  (*(volatile uint32_t *)(SAC_BASE + 0x40)) /* data in, 4 words per block */
#define SAC_O_FIFO_REG  (*(volatile uint32_t *)(SAC_BASE + 0x44)) /* data out, 4 words per block */

/* SACCR bits. */
#define SAC_CR_ALGSEL_MSK 0x0000000FUL /* 0=AES, 1=DES, 4=SM4 (DES/SM4 seen in sibling libs) */
#define SAC_CR_ENABLE     0x00000010UL /* algorithm enable; read back as "initialized" */
#define SAC_CR_CFG_BIT6   0x00000040UL /* set by every algorithm Init; purpose unknown */
#define SAC_CR_ARAM_CLR   0x00000080UL /* write 1 to start ARAM clear; self-clearing busy */
#define SAC_CR_DEINIT     0x00000100UL /* set in AES_Close before clearing ENABLE (suspected) */

/* SACSR bits; both flags are cleared by writing 1 (library uses RMW OR). */
#define SAC_SR_ARAM_DONE 0x00000001UL
#define SAC_SR_DONE      0x00000002UL

/* SACSFR1 bits. */
#define SAC_SFR1_ENCRYPT 0x00000001UL /* DIR = 01: encrypt (also forced for CTR) */
#define SAC_SFR1_DECRYPT 0x00000002UL /* DIR = 10: decrypt */
#define SAC_SFR1_KEY192  0x00000008UL
#define SAC_SFR1_KEY256  0x00000010UL
#define SAC_SFR1_CBC     0x00000020UL /* chaining mode: 1=CBC, 0=ECB/CTR */
#define SAC_SFR1_START   0x00000080UL /* trigger key load / one block operation */

/* SACCR value written by AES_Init: AES select, ENABLE, common bit6, ARAM clear. */
#define SAC_CR_INIT_AES (SAC_CR_ARAM_CLR | SAC_CR_CFG_BIT6 | SAC_CR_ENABLE)

/* RCC AHB1PCLKEN (RCC_Module.AHB1PCLKEN at 0x4002103C per DWARF), bit15 gates SAC. */
#define RCC_AHB1PCLKEN_REG    (*(volatile uint32_t *)0x4002103CUL)
#define RCC_AHB1PCLKEN_SACEN  0x00008000UL

/*
 * Copy length words (length * 4 bytes) into a SAC FIFO write port.
 * Declared in the common library object; not exposed by its public header.
 */
extern uint32_t BufToFIFO(volatile uint32_t *fifo_addr, const void *buffer,
                           uint32_t wordLen);

/* Busy-wait until SACSR_DONE is set; budget is Alg_TimeOut_Counter polls. */
static uint32_t SAC_WaitDone(void)
{
    uint32_t i;

    for (i = 0U; (SAC_SR_REG & SAC_SR_DONE) == 0U; i++)
    {
        if (i > Alg_TimeOut_Counter)
        {
            return AES_TimeOutError;
        }
    }
    return AES_Crypto_OK;
}

uint32_t AES_Init(AES_PARM *parm)
{
    uint32_t i;

    /* Enable the SAC clock, select AES and kick off an ARAM clear. */
    RCC_AHB1PCLKEN_REG |= RCC_AHB1PCLKEN_SACEN;
    SAC_CR_REG = SAC_CR_INIT_AES; /* 0xD0, direct store */

    if (CLEAR_ARAM_DONE_CHECK() == Time_Out)
    {
        return AES_TimeOutError;
    }

    /* Key length select; the direct store also resets every SACSFR1 field. */
    switch (parm->keyWordLen)
    {
    case 4:
        SAC_SFR1_REG = 0U; /* AES-128 */
        break;
    case 6:
        SAC_SFR1_REG = SAC_SFR1_KEY192;
        break;
    case 8:
        SAC_SFR1_REG = SAC_SFR1_KEY256;
        break;
    default:
        return AES_Crypto_KeyLengthError;
    }

    /* Parameter validation, in the same order as the binary. */
    if (parm->key == NULL)
    {
        return AES_Crypto_ParaNull;
    }
    if ((parm->Mode == AES_CBC || parm->Mode == AES_CTR) && parm->iv == NULL)
    {
        return AES_Crypto_ParaNull;
    }
    if (parm->Mode != AES_ECB && parm->Mode != AES_CBC && parm->Mode != AES_CTR)
    {
        return AES_Crypto_ModeError;
    }
    if (parm->En_De != AES_ENC && parm->En_De != AES_DEC)
    {
        return AES_Crypto_EnOrDeError;
    }

    /* Load the key through KADR and trigger the key expansion. */
    BufToFIFO(&SAC_K_FIFO_REG, parm->key, parm->keyWordLen);
    SAC_SFR1_REG |= SAC_SFR1_START;

    for (i = 0U; (SAC_SR_REG & SAC_SR_DONE) == 0U; i++)
    {
        if (i > Alg_TimeOut_Counter)
        {
            return AES_TimeOutError;
        }
    }
    SAC_SR_REG |= SAC_SR_DONE; /* write 1 to clear */

    /* Direction: CTR always runs the cipher in encryption direction. */
    if (parm->En_De == AES_ENC || parm->Mode == AES_CTR)
    {
        SAC_SFR1_REG = (SAC_SFR1_REG & ~0x3UL) | SAC_SFR1_ENCRYPT;
    }
    else
    {
        SAC_SFR1_REG = (SAC_SFR1_REG & 0xFFUL) | SAC_SFR1_DECRYPT;
    }

    /* Chaining mode; CBC additionally seeds the IV through P1ADR. */
    if (parm->Mode == AES_ECB || parm->Mode == AES_CTR)
    {
        SAC_SFR1_REG &= ~SAC_SFR1_CBC;
    }
    else /* AES_CBC */
    {
        SAC_SFR1_REG = (SAC_SFR1_REG & 0xFFUL) | SAC_SFR1_CBC;
        BufToFIFO(&SAC_P1_FIFO_REG, parm->iv, 4U);
    }

    return AES_Init_OK;
}

uint32_t AES_Crypto(AES_PARM *parm)
{
    uint32_t inWordLen, remaind, i, counter;
    uint32_t *in, *out;
    uint32_t iv_buf[4];

    if (parm->in == NULL || parm->out == NULL)
    {
        return AES_Crypto_ParaNull;
    }

    inWordLen = parm->inWordLen;
    if (inWordLen == 0U)
    {
        return AES_Crypto_LengthError;
    }
    /* ECB/CBC need a whole number of 16-byte blocks; CTR accepts any length. */
    if ((inWordLen & 0x3U) != 0U && parm->Mode != AES_CTR)
    {
        return AES_Crypto_LengthError;
    }

    if ((RCC_AHB1PCLKEN_REG & RCC_AHB1PCLKEN_SACEN) == 0U ||
        (SAC_CR_REG & SAC_CR_ENABLE) == 0U)
    {
        return AES_Crypto_UnInitError;
    }

    if (parm->Mode == AES_CTR)
    {
        iv_buf[0] = parm->iv[0];
        iv_buf[1] = parm->iv[1];
        iv_buf[2] = parm->iv[2];
        iv_buf[3] = parm->iv[3];
    }

    in       = parm->in;
    out      = parm->out;
    remaind  = inWordLen & 0x3U;
    inWordLen &= ~0x3U;
    counter   = 0U;

    while (inWordLen != 0U)
    {
        if (parm->Mode == AES_CTR)
        {
            /* Encrypt the counter block, then increment it as a 128-bit
               big-endian integer before the next round. */
            SAC_I_FIFO_REG = iv_buf[0];
            SAC_I_FIFO_REG = iv_buf[1];
            SAC_I_FIFO_REG = iv_buf[2];
            SAC_I_FIFO_REG = iv_buf[3];
            for (i = 16U; i != 0U; i--)
            {
                uint8_t *byte = ((uint8_t *)iv_buf) + i - 1U;
                if (*byte != 0xFFU)
                {
                    (*byte)++;
                    break;
                }
                *byte = (uint8_t)counter;
            }
        }
        else
        {
            SAC_I_FIFO_REG = in[0];
            SAC_I_FIFO_REG = in[1];
            SAC_I_FIFO_REG = in[2];
            SAC_I_FIFO_REG = in[3];
        }

        SAC_SFR1_REG |= SAC_SFR1_START;
        if (SAC_WaitDone() != AES_Crypto_OK)
        {
            return AES_TimeOutError;
        }
        SAC_SR_REG |= SAC_SR_DONE; /* write 1 to clear */

        if (parm->Mode == AES_CTR)
        {
            out[0] = SAC_O_FIFO_REG ^ in[0];
            out[1] = SAC_O_FIFO_REG ^ in[1];
            out[2] = SAC_O_FIFO_REG ^ in[2];
            out[3] = SAC_O_FIFO_REG ^ in[3];
        }
        else
        {
            out[0] = SAC_O_FIFO_REG;
            out[1] = SAC_O_FIFO_REG;
            out[2] = SAC_O_FIFO_REG;
            out[3] = SAC_O_FIFO_REG;
        }

        in += 4;
        out += 4;
        inWordLen -= 4U;
    }

    /* CTR tail: the trailing 1..3 words are XORed word-wise with fresh
       keystream words read from OADR. */
    if (remaind != 0U && parm->Mode == AES_CTR)
    {
        SAC_I_FIFO_REG = iv_buf[0];
        SAC_I_FIFO_REG = iv_buf[1];
        SAC_I_FIFO_REG = iv_buf[2];
        SAC_I_FIFO_REG = iv_buf[3];

        SAC_SFR1_REG |= SAC_SFR1_START;
        if (SAC_WaitDone() != AES_Crypto_OK)
        {
            return AES_TimeOutError;
        }
        SAC_SR_REG |= SAC_SR_DONE; /* write 1 to clear */

        for (i = 0U; i < remaind; i++)
        {
            out[i] = SAC_O_FIFO_REG ^ in[i];
        }
    }

    return AES_Crypto_OK;
}

void AES_Close(void)
{
    if ((RCC_AHB1PCLKEN_REG & RCC_AHB1PCLKEN_SACEN) != 0U &&
        (SAC_CR_REG & SAC_CR_ENABLE) != 0U)
    {
        SAC_CR_REG |= SAC_CR_DEINIT;
        SAC_CR_REG &= ~SAC_CR_ENABLE;
        (void)CLEAR_ARAM_DONE_CHECK();
        RCC_AHB1PCLKEN_REG &= ~RCC_AHB1PCLKEN_SACEN;
    }
}

void AES_Version(uint8_t *type, uint8_t *customer, uint8_t date[3], uint8_t *version)
{
    /* No NULL checks in the binary; callers must pass valid pointers. */
    *type = 0x05;     /* Commercial + Normal */
    *customer = 0x00; /* standard version */
    date[0] = 25;     /* year 2025 */
    date[1] = 3;      /* month */
    date[2] = 19;     /* day */
    *version = 0x10;  /* v1.0 */
}
