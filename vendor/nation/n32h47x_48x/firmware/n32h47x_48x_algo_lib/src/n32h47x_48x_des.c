/**
 * \file n32h47x_48x_des.c
 * \brief DES/TDES driver for the SAC accelerator, realigned to vendor
 *        n32h47x_48x_algo_lib v1.2.0 binary behavior.
 *
 * Reconstructed from:
 *   - lib/n32h47x_48x_des.a (GCC arm, -g, GNU Arm Embedded 10.2.1) disassembly
 *     and DWARF (SAC_TypeDef layout, enum values, call sites);
 *   - .lib armclang object n32h47x_48x_des_1.o (cross validation);
 *   - inc/n32h47x_48x_des.h (API contract, error code values).
 * See docs/registers_des.md for the register map and evidence citations.
 */

#include "n32h47x_48x_des.h"
#include "n32h47x_48x_algo_common.h"

#include <stddef.h>

/*
 * Hardware bases.
 * SAC (Security Algorithm Coprocessor) base matches SAC_BASE in the CMSIS
 * device header (AHBPERIPH_BASE + 0xA000 = 0x4002A000); literals confirmed in
 * the DES_Init/DES_Crypto/DES_Close literal pools.
 */
#define SAC_BASE                 0x4002A000U
#define RCC_BASE                 0x40021000U

/*
 * SAC register offsets, names taken from the DWARF SAC_TypeDef of the vendor
 * des object. All FIFO-style ports (P1ADR/P2ADR/P3ADR/KADR/IADR/OADR) are
 * 32-bit push/pop windows: every write pushes one word, every read pops one
 * word.
 */
#define SAC_SACCR_OFFSET         0x00U /* SACCR:  control register */
#define SAC_SACSR_OFFSET         0x04U /* SACSR:  status register */
#define SAC_SACSFR1_OFFSET       0x0CU /* SACSFR1: DES/TDES operating config */
#define SAC_P1ADR_OFFSET         0x30U /* P1ADR:  CBC initial vector port */
#define SAC_P2ADR_OFFSET         0x34U /* P2ADR:  key words [1:0] port */
#define SAC_P3ADR_OFFSET         0x38U /* P3ADR:  key words [3:2] port (TDES) */
#define SAC_KADR_OFFSET          0x3CU /* KADR:   key words [5:4] port (TDES 3key) */
#define SAC_IADR_OFFSET          0x40U /* IADR:   data input FIFO */
#define SAC_OADR_OFFSET          0x44U /* OADR:   data output FIFO */

/*
 * SACCR bits used by the DES library.
 * DES_Init writes 0xD1 (bits 0+4+6+7); the AES library writes 0xD0 and the
 * SM4 library 0xD4, so bits[2:0] select the algorithm (1 = DES, inferred).
 * Bit 4 together with bit 0 forms the 0x11 "configured" signature checked by
 * DES_Crypto/DES_Close. Bit 7 starts the ARAM clear sequenced with
 * CLEAR_ARAM_DONE_CHECK(). Bit 8 is set once by DES_Close before the enable
 * bits are dropped.
 */
#define SAC_SACCR_INIT_DES       0x000000D1U /* reset sequence + DES select  */
#define SAC_SACCR_CONFIG_MASK    0x00000011U /* bits 0 and 4, init signature */
#define SAC_SACCR_CLOSE_FLAG     0x00000100U /* pulsed by DES_Close          */

/*
 * SACSFR1 bits (DES/TDES configuration register, cleared to 0 by DES_Init).
 * Key length encoding: DES_KEY -> 0x00, TDES_2KEY -> 0x20, TDES_3KEY -> 0x30
 * (bit 5 marks TDES, bit 4 adds the third key, inferred from the pair).
 */
#define SAC_SACSFR1_CBC          0x00000002U /* CBC chaining (else ECB)      */
#define SAC_SACSFR1_TDES_3KEY    0x00000010U /* third key present            */
#define SAC_SACSFR1_TDES         0x00000020U /* TDES (two or three keys)     */
#define SAC_SACSFR1_DECRYPT      0x00000040U /* decrypt direction            */
#define SAC_SACSFR1_GO           0x00000080U /* start one 64-bit block       */
#define KEYMODE_BITS_2KEY        SAC_SACSFR1_TDES
#define KEYMODE_BITS_3KEY        (SAC_SACSFR1_TDES | SAC_SACSFR1_TDES_3KEY)

/*
 * SACSR bits. OFNE is set by hardware when the output FIFO holds a result;
 * the library clears it with a read-modify-write of 1 into the bit before
 * draining the two result words.
 */
#define SAC_SACSR_OFNE           0x00000004U /* output FIFO not empty        */

/*
 * RCC AHB1PCLKEN (RCC_BASE + 0x3C) bit 15 gates the SAC clock
 * (RCC_AHB1PCLKEN_SACEN in the CMSIS/RCC headers).
 */
#define RCC_AHB1PCLKEN_OFFSET    0x3CU
#define RCC_AHB1PCLKEN_SACEN     0x00008000U

/* One DES/TDES block = 64 bits = 2 words; key/IV ports take 2 words each. */
#define DES_BLOCK_WORDS          2U

/*
 * Common-library FIFO helper (defined in n32h47x_48x_algo_common.c, declared
 * in the vendor common header but not in the current one). wordLen is counted
 * in 32-bit words (common lib final RE conclusion, DWARF parameter name):
 * DES passes 2 for every key/IV block and AES passes keyWordLen (4/6/8).
 */
extern uint32_t BufToFIFO(volatile uint32_t *fifo_addr, const void *buffer,
                           uint32_t wordLen);

/**
 * @brief Hardware register access helper functions.
 */
static inline void WriteHardwareRegister(uint32_t offset, uint32_t value)
{
    *((volatile uint32_t *)(SAC_BASE + offset)) = value;
}

static inline uint32_t ReadHardwareRegister(uint32_t offset)
{
    return *((volatile uint32_t *)(SAC_BASE + offset));
}

static inline void WriteRCCRegister(uint32_t offset, uint32_t value)
{
    *((volatile uint32_t *)(RCC_BASE + offset)) = value;
}

static inline uint32_t ReadRCCRegister(uint32_t offset)
{
    return *((volatile uint32_t *)(RCC_BASE + offset));
}

/**
 * @brief Enable the SAC clock on RCC AHB1PCLKEN.
 */
static void EnableDESHardwareClock(void)
{
    WriteRCCRegister(RCC_AHB1PCLKEN_OFFSET,
                     ReadRCCRegister(RCC_AHB1PCLKEN_OFFSET) | RCC_AHB1PCLKEN_SACEN);
}

/**
 * @brief Disable the SAC clock on RCC AHB1PCLKEN.
 */
static void DisableDESHardwareClock(void)
{
    WriteRCCRegister(RCC_AHB1PCLKEN_OFFSET,
                     ReadRCCRegister(RCC_AHB1PCLKEN_OFFSET) & ~RCC_AHB1PCLKEN_SACEN);
}

/**
 * @brief Check whether the accelerator is initialized by DES_Init.
 * @return 1 when the SAC clock is on and SACCR carries the 0x11 signature,
 *         0 otherwise.
 */
static uint32_t IsDESHardwareReady(void)
{
    if ((ReadRCCRegister(RCC_AHB1PCLKEN_OFFSET) & RCC_AHB1PCLKEN_SACEN) == 0U)
    {
        return 0U;
    }
    if ((ReadHardwareRegister(SAC_SACCR_OFFSET) & SAC_SACCR_CONFIG_MASK) != SAC_SACCR_CONFIG_MASK)
    {
        return 0U;
    }
    return 1U;
}

/**
 * @brief Wait until one processed block is available in the output FIFO.
 * @return DES_Crypto_OK when OFNE is observed; DES_TimeOutError after more
 *         than Alg_TimeOut_Counter busy polls.
 * @note On success the OFNE flag is cleared (write 1) before the caller
 *       drains the two result words from OADR.
 */
static uint32_t WaitForDESOutput(void)
{
    uint32_t counter = 0U;

    while ((ReadHardwareRegister(SAC_SACSR_OFFSET) & SAC_SACSR_OFNE) == 0U)
    {
        counter++;
        if (counter > Alg_TimeOut_Counter)
        {
            return DES_TimeOutError;
        }
    }

    WriteHardwareRegister(SAC_SACSR_OFFSET,
                          ReadHardwareRegister(SAC_SACSR_OFFSET) | SAC_SACSR_OFNE);
    return DES_Crypto_OK;
}

/**
 * @brief Initialize the SAC accelerator for one DES/TDES session.
 * @param[in] parm Pointer to the session parameters; key must be non-NULL,
 *                 iv must be non-NULL for CBC mode.
 * @return DES_Init_OK on success; otherwise a DES enum error code.
 * @note The accelerator must be closed with DES_Close() after the session.
 */
uint32_t DES_Init(DES_PARM *parm)
{
    /* Enable SAC clock, then trigger the ARAM clear sequence (SACCR = 0xD1). */
    EnableDESHardwareClock();
    WriteHardwareRegister(SAC_SACCR_OFFSET, SAC_SACCR_INIT_DES);

    if (CLEAR_ARAM_DONE_CHECK() == Time_Out)
    {
        return DES_TimeOutError;
    }

    /* Reset the DES/TDES operating configuration. */
    WriteHardwareRegister(SAC_SACSFR1_OFFSET, 0U);

    if (parm->key == NULL)
    {
        return DES_Crypto_ParaNull;
    }
    if ((parm->Mode == DES_CBC) && (parm->iv == NULL))
    {
        return DES_Crypto_ParaNull;
    }
    if ((parm->Mode != DES_ECB) && (parm->Mode != DES_CBC))
    {
        return DES_Crypto_ModeError;
    }
    if ((parm->En_De != DES_ENC) && (parm->En_De != DES_DEC))
    {
        return DES_Crypto_EnOrDeError;
    }

    /* Push the key words, then select single DES / TDES 2-key / TDES 3-key. */
    switch (parm->keyMode)
    {
        case DES_KEY:
            BufToFIFO((volatile uint32_t *)(SAC_BASE + SAC_P2ADR_OFFSET), parm->key, DES_BLOCK_WORDS);
            break;

        case TDES_2KEY:
            BufToFIFO((volatile uint32_t *)(SAC_BASE + SAC_P2ADR_OFFSET), parm->key, DES_BLOCK_WORDS);
            BufToFIFO((volatile uint32_t *)(SAC_BASE + SAC_P3ADR_OFFSET), parm->key + 2, DES_BLOCK_WORDS);
            WriteHardwareRegister(SAC_SACSFR1_OFFSET,
                                  ReadHardwareRegister(SAC_SACSFR1_OFFSET) | KEYMODE_BITS_2KEY);
            break;

        case TDES_3KEY:
            BufToFIFO((volatile uint32_t *)(SAC_BASE + SAC_P2ADR_OFFSET), parm->key, DES_BLOCK_WORDS);
            BufToFIFO((volatile uint32_t *)(SAC_BASE + SAC_P3ADR_OFFSET), parm->key + 2, DES_BLOCK_WORDS);
            BufToFIFO((volatile uint32_t *)(SAC_BASE + SAC_KADR_OFFSET), parm->key + 4, DES_BLOCK_WORDS);
            WriteHardwareRegister(SAC_SACSFR1_OFFSET,
                                  ReadHardwareRegister(SAC_SACSFR1_OFFSET) | KEYMODE_BITS_3KEY);
            break;

        default:
            return DES_Crypto_KeyError;
    }

    if (parm->En_De == DES_DEC)
    {
        WriteHardwareRegister(SAC_SACSFR1_OFFSET,
                              ReadHardwareRegister(SAC_SACSFR1_OFFSET) | SAC_SACSFR1_DECRYPT);
    }

    if (parm->Mode == DES_CBC)
    {
        WriteHardwareRegister(SAC_SACSFR1_OFFSET,
                              ReadHardwareRegister(SAC_SACSFR1_OFFSET) | SAC_SACSFR1_CBC);
        BufToFIFO((volatile uint32_t *)(SAC_BASE + SAC_P1ADR_OFFSET), parm->iv, DES_BLOCK_WORDS);
    }

    return DES_Init_OK;
}

/**
 * @brief Encrypt or decrypt a buffer block by block (64 bits per block).
 * @param[in] parm Pointer to the session parameters; in and out must be
 *                 non-NULL, inWordLen must be non-zero and even (words).
 *                 in and out may alias.
 * @return DES_Crypto_OK on success; otherwise a DES enum error code.
 * @note Each block is pushed to IADR, started with SACSFR1.GO, polled on
 *       SACSR.OFNE (bounded by Alg_TimeOut_Counter) and drained from OADR.
 */
uint32_t DES_Crypto(DES_PARM *parm)
{
    uint32_t i;

    if ((parm->in == NULL) || (parm->out == NULL))
    {
        return DES_Crypto_ParaNull;
    }

    if ((parm->inWordLen == 0U) || ((parm->inWordLen & 1U) != 0U))
    {
        return DES_Crypto_LengthError;
    }

    if (IsDESHardwareReady() == 0U)
    {
        return DES_Crypto_UnInitError;
    }

    for (i = 0U; i < parm->inWordLen; i += DES_BLOCK_WORDS)
    {
        /* Push one 64-bit block, then start the engine. */
        WriteHardwareRegister(SAC_IADR_OFFSET, parm->in[i]);
        WriteHardwareRegister(SAC_IADR_OFFSET, parm->in[i + 1]);
        WriteHardwareRegister(SAC_SACSFR1_OFFSET,
                              ReadHardwareRegister(SAC_SACSFR1_OFFSET) | SAC_SACSFR1_GO);

        /* Wait for the result block and drain it in push order. */
        uint32_t status = WaitForDESOutput();
        if (status != DES_Crypto_OK)
        {
            return status;
        }
        parm->out[i] = ReadHardwareRegister(SAC_OADR_OFFSET);
        parm->out[i + 1] = ReadHardwareRegister(SAC_OADR_OFFSET);
    }

    return DES_Crypto_OK;
}

/**
 * @brief Close the DES/TDES session and stop the SAC accelerator.
 * @note Does nothing unless the clock is on and SACCR still carries the
 *       0x11 signature written by DES_Init. The return value of
 *       CLEAR_ARAM_DONE_CHECK() is ignored, exactly as in the vendor binary.
 */
void DES_Close(void)
{
    if ((ReadRCCRegister(RCC_AHB1PCLKEN_OFFSET) & RCC_AHB1PCLKEN_SACEN) == 0U)
    {
        return;
    }
    if ((ReadHardwareRegister(SAC_SACCR_OFFSET) & SAC_SACCR_CONFIG_MASK) != SAC_SACCR_CONFIG_MASK)
    {
        return;
    }

    WriteHardwareRegister(SAC_SACCR_OFFSET,
                          ReadHardwareRegister(SAC_SACCR_OFFSET) | SAC_SACCR_CLOSE_FLAG);
    WriteHardwareRegister(SAC_SACCR_OFFSET,
                          ReadHardwareRegister(SAC_SACCR_OFFSET) & ~SAC_SACCR_CONFIG_MASK);

    (void)CLEAR_ARAM_DONE_CHECK();

    DisableDESHardwareClock();
}

/**
 * @brief Report the DES/TDES library version information.
 * @param[out] type Library type bits (0x05).
 * @param[out] customer Customer id (0x00, standard version).
 * @param[out] date Three version date bytes {0x19, 0x04, 0x12}; following the
 *                  header convention (year, month, day) this is 2025-04-18.
 * @param[out] version Library version (0x11, i.e. 1.1).
 */
void DES_Version(uint8_t *type, uint8_t *customer, uint8_t date[3], uint8_t *version)
{
    *type = 0x05U;
    *customer = 0x00U;

    date[0] = 0x19U;
    date[1] = 0x04U;
    date[2] = 0x12U;

    *version = 0x11U;
}
