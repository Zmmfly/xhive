#include "n32h47x_48x_hash.h"
#include "n32h47x_48x_algo_common.h"

#include <stddef.h>

// Hardware register addresses for HASH peripheral
#define HASH_RCC_BASE           (0x40021000UL)    // RCC base address
#define HASH_RCC_AHBENR         (0x0000003CUL)    // AHB enable register offset
#define HASH_AHBENR_HASHEN      (0x00008000UL)    // HASH peripheral enable bit
#define HASH_BASE               (0x4002A000UL)    // HASH peripheral base address
#define HASH_CR                 (0x00000000UL)    // HASH control register
#define HASH_DIN                (0x00000004UL)    // HASH data input register
#define HASH_STR                (0x00000008UL)    // HASH start register
#define HASH_IMR                (0x0000000CUL)    // HASH interrupt mask register
#define HASH_SR                 (0x00000010UL)    // HASH status register
#define HASH_SR_BUSY            (0x00000001UL)    // HASH busy flag
#define HASH_SR_DINIS           (0x00000008UL)    // HASH data input interrupt status
#define HASH_SR_DCIS            (0x00000100UL)    // HASH digest calculation interrupt status
#define HASH_CSR                (0x0000000FUL)    // HASH context swap registers
#define HASH_HR0                (0x0000010CUL)    // HASH hash result registers
#define HASH_HR1                (0x00000110UL)
#define HASH_HR2                (0x00000114UL)
#define HASH_HR3                (0x00000118UL)
#define HASH_HR4                (0x0000011CUL)
#define HASH_HR5                (0x00000120UL)
#define HASH_HR6                (0x00000124UL)
#define HASH_HR7                (0x00000128UL)
#define HASH_FIFO_DEPTH         (0x00000044UL)    // HASH FIFO depth register

// Control register bits
#define HASH_CR_INIT            (0x00000001UL)    // HASH initialization
#define HASH_CR_DMAE            (0x00000002UL)    // DMA enable
#define HASH_CR_DATATYPE        (0x00000008UL)    // Data type
#define HASH_CR_MODE            (0x00000010UL)    // Mode selection
#define HASH_CR_ALGO            (0x00000020UL)    // Algorithm selection
#define HASH_CR_NBW             (0x00010000UL)    // Number of bits already written
#define HASH_CR_DINNE           (0x00000080UL)    // DIN not empty

// MD5-specific constants (S rotation schedule)
static const uint32_t MD5_S[64] = {
    7, 12, 17, 22, 7, 12, 17, 22, 7, 12, 17, 22, 7, 12, 17, 22,
    5,  9, 14, 20, 5,  9, 14, 20, 5,  9, 14, 20, 5,  9, 14, 20,
    4, 11, 16, 23, 4, 11, 16, 23, 4, 11, 16, 23, 4, 11, 16, 23,
    6, 10, 15, 21, 6, 10, 15, 21, 6, 10, 15, 21, 6, 10, 15, 21
};

// SHA-1 Initial Vector
static const uint32_t SHA1_IV[5] = {
    0x67452301UL, 0xEFCDAB89UL, 0x98BADCFEUL, 0x10325476UL, 0xC3D2E1F0UL
};

// MD5 Initial Vector
static const uint32_t MD5_IV[4] = {
    0x67452301UL, 0xEFCDAB89UL, 0x98BADCFEUL, 0x10325476UL
};

// SHA-256 Initial Vector (First 8 words from extracted data at 0x948)
static const uint32_t SHA256_IV[8] = {
    0x5A827999UL, 0x6ED9EBA1UL, 0x8F1BBCDCUL, 0xCA62C1D6UL,
    0x67452301UL, 0xEFCDAB89UL, 0x98BADCFEUL, 0x10325476UL
};

// SM3 Initial Vector (Next 8 words from extracted data at 0x948)  
static const uint32_t SM3_IV[8] = {
    0xC3D2E1F0UL, 0x428A2F98UL, 0x71374491UL, 0xB5C0FBCFUL,
    0xE9B5DBA5UL, 0x3956C25BUL, 0x59F111F1UL, 0x923F82A4UL
};

// SHA-224 Initial Vector (Extracted from data at 0x998)
static const uint32_t SHA224_IV[8] = {
    0xC1059ED8UL, 0x367CD507UL, 0x3070DD17UL, 0xF70E5939UL,
    0xFFC00B31UL, 0x68581511UL, 0x64F98FA7UL, 0xBEFA4FA4UL
};

// SHA-256 Round Constants (K) - Extracted from data section
static const uint32_t SHA256_K[64] = {
    0x428A2F98UL, 0x71374491UL, 0xB5C0FBCFUL, 0xE9B5DBA5UL,
    0x3956C25BUL, 0x59F111F1UL, 0x923F82A4UL, 0xAB1C5ED5UL,
    0xD807AA98UL, 0x12835B01UL, 0x243185BEUL, 0x550C7DC3UL,
    0x72BE5D74UL, 0x80DEB1FEUL, 0x9BDC06A7UL, 0xC19BF174UL,
    0xE49B69C1UL, 0xEFBE4786UL, 0x0FC19DC6UL, 0x240CA1CCUL,
    0x2DE92C6FUL, 0x4A7484AAUL, 0x5CB0A9DCUL, 0x76F988DAUL,
    0x983E5152UL, 0xA831C66DUL, 0xB00327C8UL, 0xBF597FC7UL,
    0xC6E00BF3UL, 0xD5A79147UL, 0x06CA6351UL, 0x14292967UL,
    0x27B70A85UL, 0x2E1B2138UL, 0x4D2C6DFCUL, 0x53380D13UL,
    0x650A7354UL, 0x766A0ABBUL, 0x81C2C92EUL, 0x92722C85UL,
    0xA2BFE8A1UL, 0xA81A664BUL, 0xC24B8B70UL, 0xC76C51A3UL,
    0xD192E819UL, 0xD6990624UL, 0xF40E3585UL, 0x106AA070UL,
    0x19A4C116UL, 0x1E376C08UL, 0x2748774CUL, 0x34B0BCB5UL,
    0x391C0CB3UL, 0x4ED8AA4AUL, 0x5B9CCA4FUL, 0x682E6FF3UL,
    0x748F82EEUL, 0x78A5636FUL, 0x84C87814UL, 0x8CC70208UL,
    0x90BEFFFAUL, 0xA4506CEBUL, 0xBEF9A3F7UL, 0xC67178F2UL
};

// Function prototypes for hash operations
static uint32_t HASH_ByteLenPlus1(uint32_t* msgByteLen, uint32_t byteLen);
static uint32_t HASH_PadMsg1(HASH_CTX* ctx);
static void BufToFIFO(uint32_t addr, const uint32_t* src, uint32_t wordLen);
static void FIFOToBuf(uint32_t* dst, uint32_t addr, uint32_t wordLen);

// Complete algorithm structure definitions (extracted from reverse engineering)
const HASH_ALG HASH_ALG_SHA1[1] = {{
    .HashAlgID     = ALG_SHA1,                                              // 0x0004
    .K             = NULL,                                                  // No K array for SHA1 in hardware
    .KLen          = 0,
    .IV            = SHA1_IV,                                               // Points to SHA1_IV[0] 
    .IVLen         = 5,                                                     // 5 words (20 bytes)
    .HASH_SACCR    = 0x948,                                                 // SHA1 accelerator control register
    .HASH_HASHCTRL = 0x000000D2UL,                                          // HASH control register value
    .BlockByteLen  = 64,                                                    // 64 bytes per block
    .BlockWordLen  = 16,                                                    // 16 words per block  
    .DigestByteLen = 20,                                                    // 20 bytes digest
    .DigestWordLen = 5,                                                     // 5 words digest
    .Cycle         = 80,                                                    // 80 rounds
    .ByteLenPlus   = (uint32_t(*)(uint32_t*, uint32_t))HASH_ByteLenPlus1,
    .PadMsg        = (uint32_t(*)(HASH_CTX*))HASH_PadMsg1
}};

const HASH_ALG HASH_ALG_SHA224[1] = {{
    .HashAlgID     = ALG_SHA224,                                            // 0x000A
    .K             = SHA256_K,                                              // Uses same K as SHA256
    .KLen          = 64,                                                    // 64 round constants
    .IV            = SHA224_IV,                                             // Points to SHA224_IV[0]
    .IVLen         = 8,                                                     // 8 words (32 bytes)
    .HASH_SACCR    = 0x96C,                                                 // SHA224 accelerator control register
    .HASH_HASHCTRL = 0x00000040UL,                                          // HASH control register value
    .BlockByteLen  = 64,                                                    // 64 bytes per block
    .BlockWordLen  = 16,                                                    // 16 words per block
    .DigestByteLen = 28,                                                    // 28 bytes digest
    .DigestWordLen = 7,                                                     // 7 words digest
    .Cycle         = 64,                                                    // 64 rounds
    .ByteLenPlus   = (uint32_t(*)(uint32_t*, uint32_t))HASH_ByteLenPlus1,
    .PadMsg        = (uint32_t(*)(HASH_CTX*))HASH_PadMsg1
}};

const HASH_ALG HASH_ALG_SHA256[1] = {{
    .HashAlgID     = ALG_SHA256,                                            // 0x000B
    .K             = SHA256_K,                                              // SHA256 round constants
    .KLen          = 64,                                                    // 64 round constants
    .IV            = SHA256_IV,                                             // Points to SHA256_IV[0]
    .IVLen         = 8,                                                     // 8 words (32 bytes)
    .HASH_SACCR    = 0x9E4,                                                 // SHA256 accelerator control register
    .HASH_HASHCTRL = 0x00000040UL,                                          // HASH control register value
    .BlockByteLen  = 64,                                                    // 64 bytes per block
    .BlockWordLen  = 16,                                                    // 16 words per block
    .DigestByteLen = 32,                                                    // 32 bytes digest
    .DigestWordLen = 8,                                                     // 8 words digest
    .Cycle         = 64,                                                    // 64 rounds
    .ByteLenPlus   = (uint32_t(*)(uint32_t*, uint32_t))HASH_ByteLenPlus1,
    .PadMsg        = (uint32_t(*)(HASH_CTX*))HASH_PadMsg1
}};

const HASH_ALG HASH_ALG_MD5[1] = {{
    .HashAlgID     = ALG_MD5,                                               // 0x000C
    .K             = NULL,                                                  // No K array for MD5 in hardware
    .KLen          = 0,
    .IV            = MD5_IV,                                                // Points to MD5_IV[0]
    .IVLen         = 4,                                                     // 4 words (16 bytes)
    .HASH_SACCR    = 0x9D4,                                                 // MD5 accelerator control register
    .HASH_HASHCTRL = 0x00000092UL,                                          // HASH control register value
    .BlockByteLen  = 64,                                                    // 64 bytes per block
    .BlockWordLen  = 16,                                                    // 16 words per block
    .DigestByteLen = 16,                                                    // 16 bytes digest
    .DigestWordLen = 4,                                                     // 4 words digest
    .Cycle         = 64,                                                    // 64 rounds
    .ByteLenPlus   = (uint32_t(*)(uint32_t*, uint32_t))HASH_ByteLenPlus1,
    .PadMsg        = (uint32_t(*)(HASH_CTX*))HASH_PadMsg1
}};

const HASH_ALG HASH_ALG_SM3[1] = {{
    .HashAlgID     = ALG_SM3,                                               // 0x0012
    .K             = NULL,                                                  // No K array for SM3 in hardware
    .KLen          = 0,
    .IV            = SM3_IV,                                                // Points to SM3_IV[0]
    .IVLen         = 8,                                                     // 8 words (32 bytes)
    .HASH_SACCR    = 0x9AC,                                                 // SM3 accelerator control register
    .HASH_HASHCTRL = 0x00000002UL,                                          // HASH control register value
    .BlockByteLen  = 64,                                                    // 64 bytes per block
    .BlockWordLen  = 16,                                                    // 16 words per block
    .DigestByteLen = 32,                                                    // 32 bytes digest
    .DigestWordLen = 8,                                                     // 8 words digest
    .Cycle         = 64,                                                    // 64 rounds
    .ByteLenPlus   = (uint32_t(*)(uint32_t*, uint32_t))HASH_ByteLenPlus1,
    .PadMsg        = (uint32_t(*)(HASH_CTX*))HASH_PadMsg1
}};

/**
 * @brief Copy data from buffer to HASH FIFO
 * @param[in] addr FIFO register address
 * @param[in] src pointer to source data
 * @param[in] wordLen length in words
 * @note Hardware-specific function for data transfer to HASH accelerator
 */
static void BufToFIFO(uint32_t addr, const uint32_t* src, uint32_t wordLen)
{
    volatile uint32_t* fifo_reg = (volatile uint32_t*)addr;
    
    for (uint32_t i = 0; i < wordLen; i++) {
        *fifo_reg = src[i];
    }
}

/**
 * @brief Copy data from HASH FIFO to buffer
 * @param[in] dst pointer to destination buffer
 * @param[in] addr FIFO register address
 * @param[in] wordLen length in words
 * @note Hardware-specific function for data transfer from HASH accelerator
 */
static void FIFOToBuf(uint32_t* dst, uint32_t addr, uint32_t wordLen)
{
    volatile uint32_t* fifo_reg = (volatile uint32_t*)addr;
    
    for (uint32_t i = 0; i < wordLen; i++) {
        dst[i] = *fifo_reg;
    }
}


/**
 * @brief Process message buffer through hardware
 * @param[in] ctx pointer to HASH_CTX struct
 * @return HASH_ProcMsgBuf_OK, success; others: fail
 * @note Internal function used by HASH_Update
 */
uint32_t HASH_ProcMsgBuf(HASH_CTX* ctx)
{
    // Send message to hardware
    BufToFIFO(HASH_BASE + HASH_DIN, (uint32_t*)ctx->msgBuf, ctx->hashAlg->BlockWordLen);
    
    // Start processing
    *(volatile uint32_t*)(HASH_BASE + HASH_IMR) |= 0x80;
    
    // Wait for completion
    uint32_t timeout = 0;
    while ((*(volatile uint32_t*)(HASH_BASE + HASH_SR) & HASH_SR_DINIS) == 0) {
        if (timeout++ > Alg_TimeOut_Counter) {
            return HASH_TimeOut_ERROR;
        }
    }
    
    // Clear interrupt
    *(volatile uint32_t*)(HASH_BASE + HASH_SR) |= HASH_SR_DINIS;
    
    // Reset message buffer index
    ctx->msgIdx = 0;
    
    return HASH_ProcMsgBuf_OK;
}

/**
 * @brief Hash init
 * @param[in] ctx pointer to HASH_CTX struct
 * @return HASH_Init_OK, Hash init success; others: Hash init fail
 * @note 1.Please refer to the demo in user guidance before using this function 
 */
uint32_t HASH_Init(HASH_CTX* ctx)
{
    if (!ctx) {
        return HASH_Init_ERROR;
    }
    
    // Validate algorithm
    if (ctx->hashAlg != &HASH_ALG_SHA1[0] &&
        ctx->hashAlg != &HASH_ALG_SHA224[0] &&
        ctx->hashAlg != &HASH_ALG_SHA256[0] &&
        ctx->hashAlg != &HASH_ALG_SM3[0] &&
        ctx->hashAlg != &HASH_ALG_MD5[0]) {
        return HASH_Init_ERROR;
    }
    
    // Enable HASH peripheral clock
    *(volatile uint32_t*)(HASH_RCC_BASE + HASH_RCC_AHBENR) |= HASH_AHBENR_HASHEN;
    
    // Initialize HASH control register
    uint32_t ctrl_value = ctx->hashAlg->HASH_HASHCTRL | HASH_CR_INIT;
    *(volatile uint32_t*)(HASH_BASE + HASH_CR) = ctrl_value;
    
    // Wait for initialization to complete
    if (CLEAR_ARAM_DONE_CHECK() == Time_Out) {
        return HASH_TimeOut_ERROR;
    }
    
    // Set algorithm-specific configuration
    *(volatile uint32_t*)(HASH_BASE + HASH_IMR) = ctx->hashAlg->HASH_SACCR;
    
    // Load initial values
    BufToFIFO(HASH_BASE + HASH_CSR + 0x3C, ctx->hashAlg->IV, ctx->hashAlg->IVLen);
    
    // Special handling for MD5 algorithm
    if (ctx->hashAlg->HashAlgID == ALG_MD5) {
        BufToFIFO(HASH_BASE + HASH_CSR + 0x34, MD5_S, 16);
    }
    
    return HASH_Init_OK;
}

/**
 * @brief Hash start
 * @param[in] ctx pointer to HASH_CTX struct
 * @return HASH_Start_OK, Hash start success; others: Hash start fail
 * @note 1.Please refer to the demo in user guidance before using this function 
 *         2.HASH_Init() should be recalled before use this function 
 */
uint32_t HASH_Start(HASH_CTX* ctx)
{
    if (!ctx) {
        return HASH_Start_ERROR;
    }
    
    // Validate algorithm
    if (ctx->hashAlg != &HASH_ALG_SHA1[0] &&
        ctx->hashAlg != &HASH_ALG_SHA224[0] &&
        ctx->hashAlg != &HASH_ALG_SHA256[0] &&
        ctx->hashAlg != &HASH_ALG_SM3[0] &&
        ctx->hashAlg != &HASH_ALG_MD5[0]) {
        return HASH_Start_ERROR;
    }
    
    // Clear message length
    SetZero_U32(ctx->msgByteLen, 4);
    ctx->msgIdx = 0;
    
    // Initialize IV based on sequence type
    if (ctx->sequence == HASH_SEQUENCE_TRUE) {
        // Save IV in context
        Cpy_U32(ctx->IV, ctx->hashAlg->IV, ctx->hashAlg->IVLen);
    } else if (ctx->sequence == HASH_SEQUENCE_FALSE) {
        // Load IV to hardware
        BufToFIFO(HASH_BASE + HASH_CSR + 0x30, 
                 ctx->hashAlg->IV, ctx->hashAlg->IVLen);
    } else {
        return HASH_Start_ERROR;
    }
    
    return HASH_Start_OK;
}

/**
 * @brief Hash update
 * @param[in] ctx pointer to HASH_CTX struct
 * @param[in] in pointer to message
 * @param[in] byteLen length of message in bytes
 * @return HASH_Update_OK, Hash update success; others: Hash update fail
 * @note 1.Please refer to the demo in user guidance before using this function 
 *         2.HASH_Init() and HASH_Start() should be recalled before use this function 
 */
uint32_t HASH_Update(HASH_CTX* ctx, uint8_t* in, uint32_t byteLen)
{
    if (!ctx || !in || !byteLen) {
        return (!ctx || !in) ? HASH_Update_ERROR : HASH_Update_OK;
    }
    
    // Validate algorithm
    if (ctx->hashAlg != &HASH_ALG_SHA1[0] &&
        ctx->hashAlg != &HASH_ALG_SHA224[0] &&
        ctx->hashAlg != &HASH_ALG_SHA256[0] &&
        ctx->hashAlg != &HASH_ALG_SM3[0] &&
        ctx->hashAlg != &HASH_ALG_MD5[0]) {
        return HASH_Update_ERROR;
    }
    
    // Check hardware status
    if ((*(volatile uint32_t*)(HASH_RCC_BASE + HASH_RCC_AHBENR) & HASH_AHBENR_HASHEN) != HASH_AHBENR_HASHEN ||
        (*(volatile uint32_t*)(HASH_BASE + HASH_CR) & 0x12) != 0x12) {
        return HASH_Update_ERROR;
    }
    
    uint32_t newMsgIdx = ctx->msgIdx + byteLen;
    uint32_t fullBlocks = newMsgIdx >> ctx->hashAlg->BlockByteLen;
    uint32_t blockSize = ctx->hashAlg->BlockWordLen;
    uint32_t msgBlockSize = ctx->hashAlg->BlockByteLen;
    
    // Update byte length counter
    uint32_t result = ctx->hashAlg->ByteLenPlus(ctx->msgByteLen, byteLen);
    if (result != HASH_ByteLenPlus_OK) {
        return HASH_Update_ERROR;
    }
    
    // If we don't have a full block yet
    if (newMsgIdx < msgBlockSize) {
        Cpy_U8(&ctx->msgBuf[ctx->msgIdx], in, byteLen);
        ctx->msgIdx = newMsgIdx;
        return HASH_Update_OK;
    }
    
    // Save current IV if in sequence mode
    if (ctx->sequence == HASH_SEQUENCE_TRUE) {
        BufToFIFO(HASH_BASE + HASH_CSR + 0x30, ctx->IV, ctx->hashAlg->IVLen);
    }
    
    // Process partial block first
    if (ctx->msgIdx > 0) {
        uint32_t remaining = msgBlockSize - ctx->msgIdx;
        Cpy_U8(&ctx->msgBuf[ctx->msgIdx], in, remaining);
        
        uint32_t procResult = HASH_ProcMsgBuf(ctx);
        if (procResult != HASH_ProcMsgBuf_OK) {
            return procResult;
        }
        
        in += remaining;
        byteLen -= remaining;
        fullBlocks--;
    }
    
    // Process full blocks
    for (uint32_t i = 0; i < fullBlocks; i++) {
        if (((uint32_t)in & 0x3) != 0) {
            // Unaligned access - copy to buffer first
            Cpy_U8(ctx->msgBuf, in, msgBlockSize);
            uint32_t procResult = HASH_ProcMsgBuf(ctx);
            if (procResult != HASH_ProcMsgBuf_OK) {
                return procResult;
            }
        } else {
            // Aligned access - direct to FIFO
            BufToFIFO(HASH_BASE + HASH_DIN, (uint32_t*)in, blockSize);
            
            // Start processing
            *(volatile uint32_t*)(HASH_BASE + HASH_IMR) |= 0x80;
            
            // Wait for completion
            uint32_t timeout = 0;
            while ((*(volatile uint32_t*)(HASH_BASE + HASH_SR) & HASH_SR_DINIS) == 0) {
                if (timeout++ > Alg_TimeOut_Counter) {
                    return HASH_TimeOut_ERROR;
                }
            }
            
            // Clear interrupt
            *(volatile uint32_t*)(HASH_BASE + HASH_SR) |= HASH_SR_DINIS;
        }
        in += msgBlockSize;
    }
    
    // Store remaining data in buffer
    ctx->msgIdx = (msgBlockSize - 1) & newMsgIdx;
    if (ctx->msgIdx > 0) {
        Cpy_U8(ctx->msgBuf, in, ctx->msgIdx);
    }
    
    // Restore IV if in sequence mode
    if (ctx->sequence == HASH_SEQUENCE_TRUE) {
        FIFOToBuf(ctx->IV, HASH_BASE + HASH_CSR + 0x44, ctx->hashAlg->IVLen);
    }
    
    return HASH_Update_OK;
}

/**
 * @brief Hash complete
 * @param[in] ctx pointer to HASH_CTX struct
 * @param[out] out pointer to hash result, digest
 * @return HASH_Complete_OK, Hash complete success; others: Hash complete fail
 * @note 1.Please refer to the demo in user guidance before using this function 
 *         2.HASH_Init(), HASH_Start() and HASH_Update() should be recalled before use this function 
 */
uint32_t HASH_Complete(HASH_CTX* ctx, uint8_t* out)
{
    uint8_t tempDigest[64];
    uint32_t digestSize = ctx->hashAlg->DigestWordLen;
    
    if (!ctx || !out) {
        return HASH_Complete_ERROR;
    }
    
    // Validate algorithm
    if (ctx->hashAlg != &HASH_ALG_SHA1[0] &&
        ctx->hashAlg != &HASH_ALG_SHA224[0] &&
        ctx->hashAlg != &HASH_ALG_SHA256[0] &&
        ctx->hashAlg != &HASH_ALG_SM3[0] &&
        ctx->hashAlg != &HASH_ALG_MD5[0]) {
        return HASH_Complete_ERROR;
    }
    
    // Check hardware status
    if ((*(volatile uint32_t*)(HASH_RCC_BASE + HASH_RCC_AHBENR) & HASH_AHBENR_HASHEN) != HASH_AHBENR_HASHEN ||
        (*(volatile uint32_t*)(HASH_BASE + HASH_CR) & 0x12) != 0x12) {
        return HASH_Complete_ERROR;
    }
    
    // Save current IV if in sequence mode
    if (ctx->sequence == HASH_SEQUENCE_TRUE) {
        BufToFIFO(HASH_BASE + HASH_CSR + 0x30, ctx->IV, ctx->hashAlg->IVLen);
    }
    
    // Perform message padding
    uint32_t result = ctx->hashAlg->PadMsg(ctx);
    if (result != HASH_PadMsg_OK) {
        return result;
    }
    
    // Read digest from hardware
    if (((uint32_t)out & 0x3) != 0) {
        // Unaligned output buffer
        FIFOToBuf((uint32_t*)tempDigest, HASH_BASE + HASH_HR0, digestSize);
        Cpy_U8(out, tempDigest, 4 * digestSize);
    } else {
        // Aligned output buffer
        FIFOToBuf((uint32_t*)out, HASH_BASE + HASH_HR0, digestSize);
    }
    
    return HASH_Complete_OK;
}

/**
 * @brief Hash close
 * @return HASH_Close_OK, Hash close success; others: Hash close fail
 * @note 1.Please refer to the demo in user guidance before using this function  
 */
uint32_t HASH_Close(void)
{
    // Check if HASH peripheral is enabled
    if ((*(volatile uint32_t*)(HASH_RCC_BASE + HASH_RCC_AHBENR) & HASH_AHBENR_HASHEN) == HASH_AHBENR_HASHEN &&
        (*(volatile uint32_t*)(HASH_BASE + HASH_CR) & 0x12) == 0x12) {
        
        // Disable HASH peripheral
        *(volatile uint32_t*)(HASH_BASE + HASH_CR) |= 0x100;
        *(volatile uint32_t*)(HASH_BASE + HASH_CR) &= 0xFFFFFFED;
        
        if (CLEAR_ARAM_DONE_CHECK() == Time_Out) {
            return HASH_Close_ERROR;
        }
        
        // Disable clock
        *(volatile uint32_t*)(HASH_RCC_BASE + HASH_RCC_AHBENR) &= ~HASH_AHBENR_HASHEN;
    }
    
    return HASH_Close_OK;
}

/**
 * @brief Add byte length with carry handling
 * @param[in,out] msgByteLen pointer to message length array (4 words)
 * @param[in] byteLen bytes to add
 * @return HASH_ByteLenPlus_OK, success; HASH_ByteLenPlus_ERROR, overflow
 * @note Internal function used for 128-bit message length tracking
 */
static uint32_t HASH_ByteLenPlus1(uint32_t* msgByteLen, uint32_t byteLen)
{
    msgByteLen[1] += byteLen;
    if (msgByteLen[1] < byteLen) {
        msgByteLen[0]++;
    }
    
    // Check for overflow (max 2^29 bits supported by hardware)
    if (msgByteLen[0] >= 0x20000000UL) {
        return HASH_ByteLenPlus_ERROR;
    }
    
    return HASH_ByteLenPlus_OK;
}

/**
 * @brief Pad message for hash completion
 * @param[in,out] ctx pointer to HASH_CTX struct
 * @return HASH_PadMsg_OK, success; HASH_PadMsg_ERROR, error
 * @note Internal function implementing standard hash message padding
 */
static uint32_t HASH_PadMsg1(HASH_CTX* ctx)
{
    uint32_t wordsToPad = (ctx->msgIdx + 4) >> 2;
    
    // Add padding bit
    ctx->msgBuf[ctx->msgIdx] = 0x80;
    ctx->msgBuf[ctx->msgIdx + 1] = 0;
    ctx->msgBuf[ctx->msgIdx + 2] = 0;
    ctx->msgBuf[ctx->msgIdx + 3] = 0;
    
    // Convert bit length from byte length
    ctx->msgByteLen[2] = (ctx->msgByteLen[2] << 3) | (ctx->msgByteLen[3] >> 29);
    ctx->msgByteLen[3] <<= 3;
    
    // Handle MD5 little-endian format
    if (ctx->hashAlg->HashAlgID == ALG_MD5) {
        uint32_t temp = ctx->msgByteLen[2];
        ctx->msgByteLen[2] = ctx->msgByteLen[3];
        ctx->msgByteLen[3] = temp;
    } else {
        // Big-endian format for other algorithms
        ReverseBytesInWord_U32(&ctx->msgByteLen[2], &ctx->msgByteLen[2], 2);
    }
    
    // Process current block if not enough space
    if (ctx->msgIdx > 0x37) {
        BufToFIFO(HASH_BASE + HASH_DIN, (uint32_t*)ctx->msgBuf, wordsToPad);
        
        // Fill rest of block
        for (uint32_t i = wordsToPad; i < 0x10; i++) {
            *(volatile uint32_t*)(HASH_BASE + HASH_DIN) = 0;
        }
        
        // Start processing
        *(volatile uint32_t*)(HASH_BASE + HASH_IMR) |= 0x80;
        
        uint32_t timeout = 0;
        while ((*(volatile uint32_t*)(HASH_BASE + HASH_SR) & HASH_SR_DINIS) == 0) {
            if (timeout++ > Alg_TimeOut_Counter) {
                return HASH_TimeOut_ERROR;
            }
        }
        
        *(volatile uint32_t*)(HASH_BASE + HASH_SR) |= HASH_SR_DINIS;
        wordsToPad = 0;
    }
    
    // Final block with message length
    BufToFIFO(HASH_BASE + HASH_DIN, (uint32_t*)ctx->msgBuf, wordsToPad);
    
    // Fill padding area
    for (uint32_t i = wordsToPad; i < 0xE; i++) {
        *(volatile uint32_t*)(HASH_BASE + HASH_DIN) = 0;
    }
    
    // Add message length
    *(volatile uint32_t*)(HASH_BASE + HASH_DIN) = ctx->msgByteLen[2];
    *(volatile uint32_t*)(HASH_BASE + HASH_DIN) = ctx->msgByteLen[3];
    
    // Final processing
    *(volatile uint32_t*)(HASH_BASE + HASH_IMR) |= 0x80;
    
    uint32_t timeout = 0;
    while ((*(volatile uint32_t*)(HASH_BASE + HASH_SR) & HASH_SR_DINIS) == 0) {
        if (timeout++ > Alg_TimeOut_Counter) {
            return HASH_TimeOut_ERROR;
        }
    }
    
    *(volatile uint32_t*)(HASH_BASE + HASH_SR) |= HASH_SR_DINIS;
    
    return HASH_PadMsg_OK;
}

/**
 * @brief SM3 Hash for 256bits digest
 * @param[in] in pointer to message
 * @param[in] byteLen length of in
 * @param[out] out pointer to hash result, digest
 * @return SM3_Hash_OK, SM3 hash success; others: SM3 hash fail
 * @note 1.Please refer to the demo in user guidance before using this function  
 */
uint32_t SM3_Hash(uint8_t* in, uint32_t byteLen, uint8_t* out)
{
    HASH_CTX ctx[56];  // Ensure sufficient stack space
    
    // Initialize context
    ctx[0].hashAlg = &HASH_ALG_SM3[0];
    ctx[1].sequence = HASH_SEQUENCE_FALSE;
    
    // Process hash
    if (HASH_Init((HASH_CTX*)ctx) != HASH_Init_OK) {
        return SM3_Hash_ERROR;
    }
    
    if (HASH_Start((HASH_CTX*)ctx) != HASH_Start_OK) {
        return SM3_Hash_ERROR;
    }
    
    if (HASH_Update((HASH_CTX*)ctx, in, byteLen) != HASH_Update_OK) {
        return SM3_Hash_ERROR;
    }
    
    if (HASH_Complete((HASH_CTX*)ctx, out) != HASH_Complete_OK) {
        return SM3_Hash_ERROR;
    }
    
    if (HASH_Close() != HASH_Close_OK) {
        return SM3_Hash_ERROR;
    }
    
    return SM3_Hash_OK;
}

/**
 * @brief SHA1 Hash
 * @param[in] in pointer to message
 * @param[in] byteLen length of in
 * @param[out] out pointer to hash result, digest
 * @return SHA1_Hash_OK, SHA1 hash success; others: SHA1 hash fail
 * @note 1.Please refer to the demo in user guidance before using this function  
 */
uint32_t SHA1_Hash(uint8_t* in, uint32_t byteLen, uint8_t* out)
{
    HASH_CTX ctx[56];  // Ensure sufficient stack space
    
    // Initialize context
    ctx[0].hashAlg = &HASH_ALG_SHA1[0];
    ctx[1].sequence = HASH_SEQUENCE_FALSE;
    
    // Process hash
    if (HASH_Init((HASH_CTX*)ctx) != HASH_Init_OK) {
        return SHA1_Hash_ERROR;
    }
    
    if (HASH_Start((HASH_CTX*)ctx) != HASH_Start_OK) {
        return SHA1_Hash_ERROR;
    }
    
    if (HASH_Update((HASH_CTX*)ctx, in, byteLen) != HASH_Update_OK) {
        return SHA1_Hash_ERROR;
    }
    
    if (HASH_Complete((HASH_CTX*)ctx, out) != HASH_Complete_OK) {
        return SHA1_Hash_ERROR;
    }
    
    if (HASH_Close() != HASH_Close_OK) {
        return SHA1_Hash_ERROR;
    }
    
    return SHA1_Hash_OK;
}

/**
 * @brief SHA224 Hash
 * @param[in] in pointer to message
 * @param[in] byteLen length of in
 * @param[out] out pointer to hash result, digest
 * @return SHA224_Hash_OK, SHA224 hash success; others: SHA224 hash fail
 * @note 1.Please refer to the demo in user guidance before using this function  
 */
uint32_t SHA224_Hash(uint8_t* in, uint32_t byteLen, uint8_t* out)
{
    HASH_CTX ctx[56];  // Ensure sufficient stack space
    
    // Initialize context
    ctx[0].hashAlg = &HASH_ALG_SHA224[0];
    ctx[1].sequence = HASH_SEQUENCE_FALSE;
    
    // Process hash
    if (HASH_Init((HASH_CTX*)ctx) != HASH_Init_OK) {
        return SHA224_Hash_ERROR;
    }
    
    if (HASH_Start((HASH_CTX*)ctx) != HASH_Start_OK) {
        return SHA224_Hash_ERROR;
    }
    
    if (HASH_Update((HASH_CTX*)ctx, in, byteLen) != HASH_Update_OK) {
        return SHA224_Hash_ERROR;
    }
    
    if (HASH_Complete((HASH_CTX*)ctx, out) != HASH_Complete_OK) {
        return SHA224_Hash_ERROR;
    }
    
    if (HASH_Close() != HASH_Close_OK) {
        return SHA224_Hash_ERROR;
    }
    
    return SHA224_Hash_OK;
}

/**
 * @brief SHA256 Hash
 * @param[in] in pointer to message
 * @param[in] byteLen length of in
 * @param[out] out pointer to hash result, digest
 * @return SHA256_Hash_OK, SHA256 hash success; others: SHA256 hash fail
 * @note 1.Please refer to the demo in user guidance before using this function  
 */
uint32_t SHA256_Hash(uint8_t* in, uint32_t byteLen, uint8_t* out)
{
    HASH_CTX ctx[56];  // Ensure sufficient stack space
    
    // Initialize context
    ctx[0].hashAlg = &HASH_ALG_SHA256[0];
    ctx[1].sequence = HASH_SEQUENCE_FALSE;
    
    // Process hash
    if (HASH_Init((HASH_CTX*)ctx) != HASH_Init_OK) {
        return SHA256_Hash_ERROR;
    }
    
    if (HASH_Start((HASH_CTX*)ctx) != HASH_Start_OK) {
        return SHA256_Hash_ERROR;
    }
    
    if (HASH_Update((HASH_CTX*)ctx, in, byteLen) != HASH_Update_OK) {
        return SHA256_Hash_ERROR;
    }
    
    if (HASH_Complete((HASH_CTX*)ctx, out) != HASH_Complete_OK) {
        return SHA256_Hash_ERROR;
    }
    
    if (HASH_Close() != HASH_Close_OK) {
        return SHA256_Hash_ERROR;
    }
    
    return SHA256_Hash_OK;
}

/**
 * @brief MD5 Hash
 * @param[in] in pointer to message
 * @param[in] byteLen length of in
 * @param[in] out pointer to hash result, digest
 * @return MD5_Hash_OK, MD5 hash success; others: MD5 hash fail
 * @note 1.Please refer to the demo in user guidance before using this function  
 */
uint32_t MD5_Hash(uint8_t* in, uint32_t byteLen, uint8_t* out)
{
    HASH_CTX ctx[56];  // Ensure sufficient stack space
    
    // Initialize context
    ctx[0].hashAlg = &HASH_ALG_MD5[0];
    ctx[1].sequence = HASH_SEQUENCE_FALSE;
    
    // Process hash
    if (HASH_Init((HASH_CTX*)ctx) != HASH_Init_OK) {
        return MD5_Hash_ERROR;
    }
    
    if (HASH_Start((HASH_CTX*)ctx) != HASH_Start_OK) {
        return MD5_Hash_ERROR;
    }
    
    if (HASH_Update((HASH_CTX*)ctx, in, byteLen) != HASH_Update_OK) {
        return MD5_Hash_ERROR;
    }
    
    if (HASH_Complete((HASH_CTX*)ctx, out) != HASH_Complete_OK) {
        return MD5_Hash_ERROR;
    }
    
    if (HASH_Close() != HASH_Close_OK) {
        return MD5_Hash_ERROR;
    }
    
    return MD5_Hash_OK;
}

/**
 * @brief Get HASH lib version
 * @param[out] type pointer one byte type information represents the type of the lib, like Commercial version.
 * @Bits 0~4 stands for Commercial (C), Security (S), Normal (N), Evaluation (E), Test (T), Bits 5~7 are reserved. e.g. 0x09 stands for CE version.
 * @param[out] customer pointer one byte customer information represents customer ID. for example, 0x00 stands for standard version, 0x01 is for Tianyu customized version...
 * @param[out] date pointer array which include three bytes date information. If the returned bytes are 18,9,13,this denotes September 13,2018 
 * @param[out] version pointer one byte version information represents develop version of the lib. e.g. 0x12 denotes version 1.2.
 * @return none
 * @1.You can recall this function to get HASH lib information
 */
void HASH_Version(uint8_t* type, uint8_t* customer, uint8_t date[3], uint8_t* version)
{
    if (type) *type = 1;        // Normal version
    if (customer) *customer = 0; // Standard customer
    if (date) {
        date[0] = 25;  // Year 2025 (25 + 2000)
        date[1] = 3;   // March
        date[2] = 19;  // Day 19
    }
    if (version) *version = 16; // Version 1.6
}
