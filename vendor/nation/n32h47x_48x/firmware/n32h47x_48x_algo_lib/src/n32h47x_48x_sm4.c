#include "n32h47x_48x_sm4.h"
#include "n32h47x_48x_algo_common.h"

#include <stddef.h>

/*
 * Reconstruction of the vendor closed-source SM4 driver, aligned with the
 * binary library v1.2.0 (lib/n32h47x_48x_sm4.a, arm-none-eabi-gcc 10.2.1 -Os,
 * cross-checked against the armclang .lib build).
 *
 * Register names below come from the DWARF debug info of the vendor archive
 * (SAC_TypeDef / RCC_Module members). The SDK CMSIS header only provides
 * SAC_BASE and RCC_AHB1PCLKEN_SACEN; it does not document the SAC register
 * layout. See docs/registers_sm4.md for the full register map, bit fields
 * and disassembly evidence offsets.
 */

/* Algorithm accelerator peripheral (SAC), shared by SM4/AES/DES/HASH/RNG. */
#define SAC_BASE                 0x4002A000UL

/* RCC: the SAC clock enable bit lives in AHB1PCLKEN. */
#define RCC_BASE                 0x40021000UL
#define RCC_AHB1PCLKEN_OFFSET    0x3C
#define RCC_AHB1PCLKEN_SACEN     0x00008000UL

/* SAC register offsets. */
#define SAC_SACCR_OFFSET         0x00    /* Control register */
#define SAC_SACSR_OFFSET         0x04    /* Status register */
#define SAC_SACSFR1_OFFSET       0x0C    /* Function reg 1: direction/mode/start */
#define SAC_SACSFR2_OFFSET       0x10    /* Function reg 2: algorithm select */
#define SAC_P1ADR_OFFSET         0x30    /* Parameter FIFO 1: IV (CBC only) */
#define SAC_P2ADR_OFFSET         0x34    /* Parameter FIFO 2: FK constants */
#define SAC_P3ADR_OFFSET         0x38    /* Parameter FIFO 3: user key */
#define SAC_KADR_OFFSET          0x3C    /* Parameter FIFO: CK constants */
#define SAC_IADR_OFFSET          0x40    /* Data input FIFO */
#define SAC_OADR_OFFSET          0x44    /* Data output FIFO */

/* SACCR values observed in the vendor driver (field split per the common/AES
 * library conclusions): 0xD4 = ALGO_SEL=SM4(0x04) | ENABLE(0x10) | 0x40 |
 * ARAM_CLEAR(0x80). */
#define SAC_SACCR_CFG            0xD4

/* Readiness mask used by SM4_Crypto/SM4_Close: ENABLE(bit4) plus the SM4
 * algorithm-select bit(bit2) inside the low nibble. */
#define SAC_SACCR_SM4_READY      0x14

/* Bit 8: set by SM4_Close before waiting for the ARAM clear. */
#define SAC_SACCR_ARAM_CLR       0x100

/* SACSR bits. */
#define SAC_SACSR_DONE           0x20        /* Bit 5: operation done flag */

/* SACSFR1 bits. */
#define SAC_SACSFR1_DEC          0x01        /* Bit 0: 1 = decrypt, 0 = encrypt */
#define SAC_SACSFR1_CBC          0x02        /* Bit 1: 1 = CBC, 0 = ECB */
#define SAC_SACSFR1_KEYEXP_BASE  0x40        /* Base value before key expansion start */
#define SAC_SACSFR1_START        0x80        /* Bit 7: start key expansion / block op */

/* SACSFR2: SM4 algorithm select value. */
#define SAC_SACSFR2_ALGO_SM4     0x7E

/* Vendor keeps both tables in writable .data as externally visible symbols
 * (sections .data.SM4_FK / .data.SM4_CK), not in .rodata. The values below
 * are the exact little-endian memory image of the vendor .data sections:
 * each 32-bit word is the byte-reversed form of the GB/T 32907-2016
 * notation, i.e. the hardware consumes the FIFO word's memory bytes in
 * ascending order as the big-endian SM4 constant (byte-level board evidence:
 * roundtrip passed with swapped tables while standard vectors failed).
 * Do NOT "normalise" these to the GB/T printed order. */
uint32_t SM4_FK[4] = {
    0xC6BAB1A3,
    0x5033AA56,
    0x97917D67,
    0xDC2270B2
};

uint32_t SM4_CK[32] = {
    0x150E0700, 0x312A231C, 0x4D463F38, 0x69625B54,
    0x857E7770, 0xA19A938C, 0xBDB6AFA8, 0xD9D2CBC4,
    0xF5EEE7E0, 0x110A03FC, 0x2D261F18, 0x49423B34,
    0x655E5750, 0x817A736C, 0x9D968F88, 0xB9B2ABA4,
    0xD5CEC7C0, 0xF1EAE3DC, 0x0D06FFF8, 0x29221B14,
    0x453E3730, 0x615A534C, 0x7D766F68, 0x99928B84,
    0xB5AEA7A0, 0xD1CAC3BC, 0xEDE6DFD8, 0x0902FBF4,
    0x251E1710, 0x413A332C, 0x5D564F48, 0x79726B64
};

/**
 * @brief Write a word buffer to a SAC parameter FIFO port.
 * @param[in] fifo_offset Register offset of the FIFO write port.
 * @param[in] buffer Word-aligned source buffer (uint32_t pointers per API contract).
 * @param[in] word_count Number of 32-bit words to push.
 * @note Thin wrapper over the common-library BufToFIFO helper, matching the
 *       vendor binary call structure; its length unit is 32-bit words.
 */
static void BufferToFIFO_Write(uint32_t fifo_offset, const uint32_t *buffer, uint32_t word_count)
{
    extern uint32_t BufToFIFO(volatile uint32_t *fifo_addr, const void *src,
                              uint32_t wordLen);

    BufToFIFO((volatile uint32_t *)(SAC_BASE + fifo_offset), buffer, word_count);
}

/**
 * @brief Hardware register access helper functions.
 */
static inline void WriteSACRegister(uint32_t offset, uint32_t value)
{
    *((volatile uint32_t *)(SAC_BASE + offset)) = value;
}

static inline uint32_t ReadSACRegister(uint32_t offset)
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
 * @brief Poll the SAC operation-done flag with the shared timeout budget.
 * @param[in,out] counter Running poll counter; callers keep one counter per
 *                        logical operation so the Alg_TimeOut_Counter budget
 *                        accumulates across data blocks like the vendor code.
 * @return Poll result.
 * @retval 0 SACSR bit5 was observed set and has been cleared again.
 * @retval 1 Poll count exceeded Alg_TimeOut_Counter (flag not cleared).
 * @note The done flag is cleared by a read-modify-write of 0x20, matching the
 *       vendor sequence of clear-then-read-OADR.
 */
static uint32_t WaitSM4DoneFlag(uint32_t *counter)
{
    while ((ReadSACRegister(SAC_SACSR_OFFSET) & SAC_SACSR_DONE) == 0) {
        (*counter)++;
        if (*counter > Alg_TimeOut_Counter) {
            return 1;
        }
    }

    WriteSACRegister(SAC_SACSR_OFFSET,
                     ReadSACRegister(SAC_SACSR_OFFSET) | SAC_SACSR_DONE);
    return 0;
}

/**
 * @brief Initialize SM4 algorithm with parameters
 * @param[in] param Pointer to SM4 parameter structure
 * @return SM4_Init_OK if success, other error codes if fail
 * @note param itself is not NULL-checked, matching the vendor library.
 */
uint32_t SM4_Init(SM4_PARM *param)
{
    uint32_t counter = 0;
    uint32_t *iv_ptr = param->iv;

    // Enable the SAC clock (RCC AHB1PCLKEN, bit 15).
    uint32_t rcc_value = ReadRCCRegister(RCC_AHB1PCLKEN_OFFSET);
    WriteRCCRegister(RCC_AHB1PCLKEN_OFFSET, rcc_value | RCC_AHB1PCLKEN_SACEN);

    // Select SM4, enable the accelerator, trigger the ARAM clear and wait
    // for it to finish.
    WriteSACRegister(SAC_SACCR_OFFSET, SAC_SACCR_CFG);
    if (CLEAR_ARAM_DONE_CHECK() == Time_Out) {
        return SM4_TimeOutError;
    }

    // Validate working mode (ECB or CBC).
    if (param->workingMode != SM4_ECB && param->workingMode != SM4_CBC) {
        return SM4_ModeErr;
    }

    // Validate encryption/decryption mode.
    if (param->EnDeMode != SM4_ENC && param->EnDeMode != SM4_DEC) {
        return SM4_EnDeErr;
    }

    // Validate key pointer.
    if (param->key == NULL) {
        return SM4_ADRNULL;
    }

    // For CBC mode, validate IV pointer.
    if (param->workingMode == SM4_CBC && iv_ptr == NULL) {
        return SM4_ADRNULL;
    }

    // Load key, CK round constants, then FK system parameters.
    BufferToFIFO_Write(SAC_P3ADR_OFFSET, param->key, 4);
    BufferToFIFO_Write(SAC_KADR_OFFSET, SM4_CK, 32);
    BufferToFIFO_Write(SAC_P2ADR_OFFSET, SM4_FK, 4);

    // Select the SM4 algorithm and start the key expansion.
    WriteSACRegister(SAC_SACSFR2_OFFSET, SAC_SACSFR2_ALGO_SM4);
    WriteSACRegister(SAC_SACSFR1_OFFSET, SAC_SACSFR1_KEYEXP_BASE);
    WriteSACRegister(SAC_SACSFR1_OFFSET,
                     ReadSACRegister(SAC_SACSFR1_OFFSET) | SAC_SACSFR1_START);

    if (WaitSM4DoneFlag(&counter) != 0) {
        return SM4_TimeOutError;
    }

    // Configure encryption/decryption direction (bit 0).
    if (param->EnDeMode == SM4_ENC) {
        WriteSACRegister(SAC_SACSFR1_OFFSET,
                         ReadSACRegister(SAC_SACSFR1_OFFSET) & ~SAC_SACSFR1_DEC);
    } else if (param->EnDeMode == SM4_DEC) {
        WriteSACRegister(SAC_SACSFR1_OFFSET,
                         (ReadSACRegister(SAC_SACSFR1_OFFSET) & 0xFFu) | SAC_SACSFR1_DEC);
    }

    // Configure block mode (bit 1); CBC additionally loads the IV.
    if (param->workingMode == SM4_ECB) {
        WriteSACRegister(SAC_SACSFR1_OFFSET,
                         ReadSACRegister(SAC_SACSFR1_OFFSET) & ~SAC_SACSFR1_CBC);
    } else if (param->workingMode == SM4_CBC) {
        WriteSACRegister(SAC_SACSFR1_OFFSET,
                         (ReadSACRegister(SAC_SACSFR1_OFFSET) & 0xFFu) | SAC_SACSFR1_CBC);
        BufferToFIFO_Write(SAC_P1ADR_OFFSET, iv_ptr, 4);
    }

    return SM4_Init_OK;
}

/**
 * @brief Perform SM4 encryption/decryption
 * @param[in] param Pointer to SM4 parameter structure
 * @return SM4_Crypto_OK if success, other error codes if fail
 * @note The timeout counter accumulates across all blocks of one call, like
 *       the vendor driver; on timeout the blocks already finished stay in the
 *       output buffer (partial completion).
 */
uint32_t SM4_Crypto(SM4_PARM *param)
{
    uint32_t counter = 0;
    uint32_t *input_ptr = param->in;
    uint32_t *output_ptr = param->out;
    uint32_t remaining_words = param->inWordLen;

    // Validate pointers.
    if (output_ptr == NULL || input_ptr == NULL) {
        return SM4_ADRNULL;
    }

    // Validate input length.
    if (remaining_words == 0 || (remaining_words & 0x03) != 0) {
        return SM4_LengthErr;
    }

    // Check that the accelerator is clocked and SM4-selected.
    if ((ReadRCCRegister(RCC_AHB1PCLKEN_OFFSET) & RCC_AHB1PCLKEN_SACEN) == 0) {
        return SM4_UnInitError;
    }
    if ((ReadSACRegister(SAC_SACCR_OFFSET) & SAC_SACCR_SM4_READY) !=
        SAC_SACCR_SM4_READY) {
        return SM4_UnInitError;
    }

    // Process data in blocks of 4 words (16 bytes).
    while (remaining_words > 0) {
        // Write 4 words to the input FIFO.
        WriteSACRegister(SAC_IADR_OFFSET, input_ptr[0]);
        WriteSACRegister(SAC_IADR_OFFSET, input_ptr[1]);
        WriteSACRegister(SAC_IADR_OFFSET, input_ptr[2]);
        WriteSACRegister(SAC_IADR_OFFSET, input_ptr[3]);

        // Start the block operation (keep only the low byte, set the start bit).
        WriteSACRegister(SAC_SACSFR1_OFFSET,
                         (ReadSACRegister(SAC_SACSFR1_OFFSET) & 0xFFu) | SAC_SACSFR1_START);

        // Wait for the block to complete.
        if (WaitSM4DoneFlag(&counter) != 0) {
            return SM4_TimeOutError;
        }

        // Read 4 words from the output FIFO.
        output_ptr[0] = ReadSACRegister(SAC_OADR_OFFSET);
        output_ptr[1] = ReadSACRegister(SAC_OADR_OFFSET);
        output_ptr[2] = ReadSACRegister(SAC_OADR_OFFSET);
        output_ptr[3] = ReadSACRegister(SAC_OADR_OFFSET);

        // Move to next block.
        input_ptr += 4;
        output_ptr += 4;
        remaining_words -= 4;
    }

    return SM4_Crypto_OK;
}

/**
 * @brief Close SM4 algorithm and disable hardware
 * @note Clean up and disable SM4 hardware module
 */
void SM4_Close(void)
{
    if ((ReadRCCRegister(RCC_AHB1PCLKEN_OFFSET) & RCC_AHB1PCLKEN_SACEN) == 0) {
        return;
    }

    uint32_t crypto_config = ReadSACRegister(SAC_SACCR_OFFSET);
    if ((crypto_config & SAC_SACCR_SM4_READY) != SAC_SACCR_SM4_READY) {
        return;
    }

    // Trigger the ARAM clear, then drop ENABLE and the SM4 algorithm-select
    // bit. The register is re-read between writes, like the vendor code.
    WriteSACRegister(SAC_SACCR_OFFSET, crypto_config | SAC_SACCR_ARAM_CLR);
    WriteSACRegister(SAC_SACCR_OFFSET,
                     ReadSACRegister(SAC_SACCR_OFFSET) & ~SAC_SACCR_SM4_READY);

    // Wait for the ARAM clear to finish (result ignored by the vendor code).
    (void)CLEAR_ARAM_DONE_CHECK();

    // Disable the SAC clock.
    WriteRCCRegister(RCC_AHB1PCLKEN_OFFSET,
                     ReadRCCRegister(RCC_AHB1PCLKEN_OFFSET) & ~RCC_AHB1PCLKEN_SACEN);
}

/**
 * @brief Get SM4 library version information
 * @param[out] type Library type information
 * @param[out] customer Customer information
 * @param[out] date Library date (3 bytes: day, month, year)
 * @param[out] version Library version information
 * @note Values match the v1.2.0 binary image: the vendor never bumped the
 *       version bytes (still 1.0, dated 2019-03-25) in either toolchain build.
 */
void SM4_Version(uint8_t *type, uint8_t *customer, uint8_t date[3], uint8_t *version)
{
    *type = 0x01;        // Commercial version
    *customer = 0x00;    // Standard version

    date[0] = 0x19;      // Day: 25
    date[1] = 0x03;      // Month: 3
    date[2] = 0x13;      // Year: 19 (2019)

    *version = 0x10;     // Version 1.0
}
