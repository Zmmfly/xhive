/**
 * Reconstruction of the vendor closed-source library
 * n32h47x_48x_algo_lib v1.2.0, module n32h47x_48x_hash.c.
 *
 * Behavior is aligned with the shipped binary
 * lib/n32h47x_48x_hash.a (GNU Arm Embedded 10.2.1, -Os, DWARF-2) and
 * cross-checked against the armclang build (.lib member
 * n32h47x_48x_hash_1.o). Evidence tags below cite "function+offset"
 * inside those objects; register names come from the vendor-private
 * SAC_Common.h SAC_TypeDef layout recovered from DWARF.
 *
 * Confirmed facts from the binary:
 * - Exported symbols (11 functions + 7 HASH_ALG tables): HASH_Init,
 *   HASH_Start, HASH_Update, HASH_Complete, HASH_Close, SHA1_Hash,
 *   SHA224_Hash, SHA256_Hash, MD5_Hash, SM3_Hash, HASH_Version; tables
 *   HASH_ALG_{SHA1,SHA224,SHA256,MD5,SM3} plus undocumented
 *   HASH_ALG_SM3_160 / HASH_ALG_SM3_192 (truncated-digest SM3 variants
 *   that no public function references). Internal statics:
 *   HASH_ByteLenPlus1, HASH_ProcMsgBuf, HASH_PadMsg1.
 * - Vendor source line numbers from DWARF: ByteLenPlus1@186,
 *   ProcMsgBuf@208, PadMsg1@234, Init@322, Start@361, Update@401,
 *   Complete@526, Close@568, SM3_Hash@594, SHA1_Hash@616,
 *   SHA224_Hash@637, SHA256_Hash@658, MD5_Hash@679, Version@702.
 * - Register map (base 0x4002A000): SACCR@+0x00, SACSR@+0x04,
 *   SACSFR1@+0x0C, P1ADR@+0x30, P2ADR@+0x34, KADR@+0x3C, IADR@+0x40,
 *   OADR@+0x44. HASH uses P1ADR for the IV/state upload, P2ADR only
 *   for the MD5 rotation table, KADR for round constants, IADR for
 *   message words, OADR for digest/state readout.
 * - HASH_Init writes SACCR = HASH_SACCR|0x80 (0xD2 for SHA1/SHA224/
 *   SHA256/SM3, 0x92 for MD5), then CLEAR_ARAM_DONE_CHECK(), then
 *   SACSFR1 = HASH_HASHCTRL (SHA1 0x00, SHA224/SHA256 0x02, MD5 0x10,
 *   SM3 0x0F), then uploads K via KADR and, for MD5, MD5_S via P2ADR
 *   (GCC HASH_Init+0x28..+0x62).
 * - Every block compression is kicked by SACSFR1 |= 0x80 and completed
 *   when SACSR bit3 reads 1; bit3 is then set again through a
 *   read-modify-write (write-1-to-clear), with Alg_TimeOut_Counter as
 *   the polling budget (GCC HASH_ProcMsgBuf+0x14..+0x3c).
 * - The 64-bit message length is kept as msgByteLen[0]=high word,
 *   msgByteLen[1]=low word (HASH_ByteLenPlus1 adds at [1] and carries
 *   into [0]); PadMsg1 pushes [0] first, then [1].
 * - Word-lane convention (see docs/registers_hash.md): SHA1/SHA224/
 *   SHA256/SM3 tables are stored byte-swapped relative to the standard
 *   constants and the length words are byte-reversed before the push;
 *   MD5 tables are stored standard and its length words are pushed
 *   low-word first. The vendor .data bytes are replicated exactly.
 * - Vendor quirks kept on purpose (suspected defects, documented only):
 *   HASH_Update returns OK for in==NULL with byteLen>0; HASH_Complete
 *   ignores the PadMsg return value; one-shot wrappers discard every
 *   step result and always return 0; HASH_Complete never returns
 *   HASH_Complete_ERROR (it uses HASH_Update_ERROR / HASH_Init_ERROR).
 */

#include <stddef.h>
#include <string.h>
#include "n32h47x_48x_hash.h"
#include "n32h47x_48x_algo_common.h"

/*
 * FIFO helpers are exported by the common module but not declared in
 * its public header; signatures follow the DWARF of
 * n32h47x_48x_common.a (vendor source lines 680/742). The third
 * parameter of both helpers is a 32-bit word count.
 */
extern uint32_t BufToFIFO(volatile uint32_t *fifo_addr, const void *buffer, uint32_t wordLen);
extern uint32_t FIFOToBuf(void *buffer, volatile uint32_t *fifo_addr, uint32_t wordLen);

/* SAC (Security Algorithm Coprocessor) register block at 0x4002A000.
 * Names follow the vendor-private SAC_TypeDef recovered from DWARF. */
#define SAC_REG_BASE           0x4002A000U
#define SAC_SACCR_REG          (*(volatile uint32_t *)(SAC_REG_BASE + 0x00U))
#define SAC_SACSR_REG          (*(volatile uint32_t *)(SAC_REG_BASE + 0x04U))
#define SAC_SACSFR1_REG        (*(volatile uint32_t *)(SAC_REG_BASE + 0x0CU))
#define SAC_P1ADR_REG          (*(volatile uint32_t *)(SAC_REG_BASE + 0x30U))
#define SAC_P2ADR_REG          (*(volatile uint32_t *)(SAC_REG_BASE + 0x34U))
#define SAC_KADR_REG           (*(volatile uint32_t *)(SAC_REG_BASE + 0x3CU))
#define SAC_IADR_REG           (*(volatile uint32_t *)(SAC_REG_BASE + 0x40U))
#define SAC_OADR_REG           (*(volatile uint32_t *)(SAC_REG_BASE + 0x44U))

/* SAC clock gate: RCC AHB1PCLKEN (0x40021000+0x3C) bit15. */
#define RCC_AHB1PCLKEN_REG     (*(volatile uint32_t *)0x4002103CU)
#define RCC_AHB1PCLKEN_SACEN   0x00008000U

/* Observed SACCR bits (see docs/registers_hash.md section 4.1). */
#define SAC_SACCR_HASH_READY   0x12U   /* bits 1|4: hash configured signature */
#define SAC_SACCR_ARAM_CLR     0x80U   /* bit 7: ARAM clear start, self-clearing */
#define SAC_SACCR_CLOSE        0x100U  /* bit 8: set by HASH_Close before deconfig */

/* Observed SACSR bit (see docs/registers_hash.md section 4.2). */
#define SAC_SACSR_HASH_DONE    0x08U   /* bit 3: block done, write-1-to-clear */

/* Observed SACSFR1 bit (see docs/registers_hash.md section 4.3). */
#define SAC_SACSFR1_START      0x80U   /* bit 7: start one block compression */

/*
 * Constant tables below replicate the vendor .data bytes exactly.
 * SHA/SM3 tables are stored byte-swapped relative to the textbook
 * constants (big-endian word lanes); MD5 tables are stored standard.
 * MD5_K = standard T table; MD5_V = standard MD5 IV.
 */
static const uint32_t MD5_K[64] = {
    0xD76AA478, 0xE8C7B756, 0x242070DB, 0xC1BDCEEE, 0xF57C0FAF, 0x4787C62A,
    0xA8304613, 0xFD469501, 0x698098D8, 0x8B44F7AF, 0xFFFF5BB1, 0x895CD7BE,
    0x6B901122, 0xFD987193, 0xA679438E, 0x49B40821, 0xF61E2562, 0xC040B340,
    0x265E5A51, 0xE9B6C7AA, 0xD62F105D, 0x02441453, 0xD8A1E681, 0xE7D3FBC8,
    0x21E1CDE6, 0xC33707D6, 0xF4D50D87, 0x455A14ED, 0xA9E3E905, 0xFCEFA3F8,
    0x676F02D9, 0x8D2A4C8A, 0xFFFA3942, 0x8771F681, 0x6D9D6122, 0xFDE5380C,
    0xA4BEEA44, 0x4BDECFA9, 0xF6BB4B60, 0xBEBFBC70, 0x289B7EC6, 0xEAA127FA,
    0xD4EF3085, 0x04881D05, 0xD9D4D039, 0xE6DB99E5, 0x1FA27CF8, 0xC4AC5665,
    0xF4292244, 0x432AFF97, 0xAB9423A7, 0xFC93A039, 0x655B59C3, 0x8F0CCC92,
    0xFFEFF47D, 0x85845DD1, 0x6FA87E4F, 0xFE2CE6E0, 0xA3014314, 0x4E0811A1,
    0xF7537E82, 0xBD3AF235, 0x2AD7D2BB, 0xEB86D391
};

/* MD5 rotation schedule (first 16 standard shifts; rounds repeat). */
static const uint32_t MD5_S[16] = {
    0x00000007, 0x0000000C, 0x00000011, 0x00000016,
    0x00000005, 0x00000009, 0x0000000E, 0x00000014,
    0x00000004, 0x0000000B, 0x00000010, 0x00000017,
    0x00000006, 0x0000000A, 0x0000000F, 0x00000015
};

static const uint32_t MD5_V[4] = {
    0x67452301, 0xEFCDAB89, 0x98BADCFE, 0x10325476
};

/* bswap() of the standard SHA-1 IV. */
static const uint32_t SHA1_IV[5] = {
    0x01234567, 0x89ABCDEF, 0xFEDCBA98, 0x76543210, 0xF0E1D2C3
};

/* bswap() of the standard SHA-1 round constants. */
static const uint32_t SHA1_K[4] = {
    0x9979825A, 0xA1EBD96E, 0xDCBC1B8F, 0xD6C162CA
};

/* bswap() of the standard SHA-224 IV. */
static const uint32_t SHA224_IV[8] = {
    0xD89E05C1, 0x07D57C36, 0x17DD7030, 0x39590EF7,
    0x310BC0FF, 0x11155868, 0xA78FF964, 0xA44FFABE
};

/* bswap() of the standard SHA-256 IV. */
static const uint32_t SHA256_IV[8] = {
    0x67E6096A, 0x85AE67BB, 0x72F36E3C, 0x3AF54FA5,
    0x7F520E51, 0x8C68059B, 0xABD9831F, 0x19CDE05B
};

/* bswap() of the standard SHA-256 round constants. */
static const uint32_t SHA256_K[64] = {
    0x982F8A42, 0x91443771, 0xCFFBC0B5, 0xA5DBB5E9, 0x5BC25639, 0xF111F159,
    0xA4823F92, 0xD55E1CAB, 0x98AA07D8, 0x015B8312, 0xBE853124, 0xC37D0C55,
    0x745DBE72, 0xFEB1DE80, 0xA706DC9B, 0x74F19BC1, 0xC1699BE4, 0x8647BEEF,
    0xC69DC10F, 0xCCA10C24, 0x6F2CE92D, 0xAA84744A, 0xDCA9B05C, 0xDA88F976,
    0x52513E98, 0x6DC631A8, 0xC82703B0, 0xC77F59BF, 0xF30BE0C6, 0x4791A7D5,
    0x5163CA06, 0x67292914, 0x850AB727, 0x38211B2E, 0xFC6D2C4D, 0x130D3853,
    0x54730A65, 0xBB0A6A76, 0x2EC9C281, 0x852C7292, 0xA1E8BFA2, 0x4B661AA8,
    0x708B4BC2, 0xA3516CC7, 0x19E892D1, 0x240699D6, 0x85350EF4, 0x70A06A10,
    0x16C1A419, 0x086C371E, 0x4C774827, 0xB5BCB034, 0xB30C1C39, 0x4AAAD84E,
    0x4FCA9C5B, 0xF36F2E68, 0xEE828F74, 0x6F63A578, 0x1478C884, 0x0802C78C,
    0xFAFFBE90, 0xEB6C50A4, 0xF7A3F9BE, 0xF27871C6
};

/* bswap() of the standard SM3 T constants. */
static const uint32_t SM3_K[2] = {
    0x1945CC79, 0x8A9D877A
};

/* bswap() of the standard SM3 IV. */
static const uint32_t SM3_V[8] = {
    0x6F168073, 0xB9B21449, 0xD7422417, 0x00068ADA,
    0xBC306FA9, 0xAA383116, 0x4DEE8DE3, 0x4E0EFBB0
};

static uint32_t HASH_ByteLenPlus1(uint32_t *msgByteLen, uint32_t byteLen);
static uint32_t HASH_ProcMsgBuf(HASH_CTX *ctx);
static uint32_t HASH_PadMsg1(HASH_CTX *ctx);

/*
 * Cycle is 6 = log2(BlockByteLen): HASH_Update uses it as a shift to
 * compute the number of complete blocks (GCC HASH_Update+0xae), not as
 * a round count. All supported algorithms use 64-byte blocks.
 */
const HASH_ALG HASH_ALG_SHA1[1] = {{
    ALG_SHA1,
    SHA1_K,   4,
    SHA1_IV,  5,
    0x000000D2UL, 0x00000000UL,
    64U, 16U, 20U, 5U, 6U,
    HASH_ByteLenPlus1, HASH_PadMsg1
}};

const HASH_ALG HASH_ALG_SHA224[1] = {{
    ALG_SHA224,
    SHA256_K, 64,
    SHA224_IV, 8,
    0x000000D2UL, 0x00000002UL,
    64U, 16U, 28U, 7U, 6U,
    HASH_ByteLenPlus1, HASH_PadMsg1
}};

const HASH_ALG HASH_ALG_SHA256[1] = {{
    ALG_SHA256,
    SHA256_K, 64,
    SHA256_IV, 8,
    0x000000D2UL, 0x00000002UL,
    64U, 16U, 32U, 8U, 6U,
    HASH_ByteLenPlus1, HASH_PadMsg1
}};

const HASH_ALG HASH_ALG_MD5[1] = {{
    ALG_MD5,
    MD5_K,    64,
    MD5_V,    4,
    0x00000092UL, 0x00000010UL,
    64U, 16U, 16U, 4U, 6U,
    HASH_ByteLenPlus1, HASH_PadMsg1
}};

const HASH_ALG HASH_ALG_SM3[1] = {{
    ALG_SM3,
    SM3_K,    2,
    SM3_V,    8,
    0x000000D2UL, 0x0000000FUL,
    64U, 16U, 32U, 8U, 6U,
    HASH_ByteLenPlus1, HASH_PadMsg1
}};

/*
 * Undocumented truncated-digest SM3 variants exported by the vendor
 * binary (digest truncated to 160/192 bits). They are not declared in
 * the public header and are rejected by the ctx->hashAlg validity
 * checks, which compare against the five public tables only.
 */
const HASH_ALG HASH_ALG_SM3_160[1] = {{
    ALG_SM3,
    SM3_K,    2,
    SM3_V,    8,
    0x000000D2UL, 0x0000000FUL,
    64U, 16U, 20U, 5U, 6U,
    HASH_ByteLenPlus1, HASH_PadMsg1
}};

const HASH_ALG HASH_ALG_SM3_192[1] = {{
    ALG_SM3,
    SM3_K,    2,
    SM3_V,    8,
    0x000000D2UL, 0x0000000FUL,
    64U, 16U, 24U, 6U, 6U,
    HASH_ByteLenPlus1, HASH_PadMsg1
}};

/**
 * @brief Check whether hashAlg points to one of the five public tables.
 * @param[in] hashAlg Algorithm table pointer to validate.
 * @return Nonzero when the pointer is one of HASH_ALG_{SHA1,SHA224,
 *         SHA256,SM3,MD5}; zero otherwise.
 * @discovery GCC HASH_Init+0x08..+0x26 inlines this five-way compare
 *            chain; SM3_160/SM3_192 are deliberately not accepted.
 */
static int HASH_IsKnownAlg(const HASH_ALG *hashAlg)
{
    return (hashAlg == &HASH_ALG_SHA1[0]) ||
           (hashAlg == &HASH_ALG_SHA224[0]) ||
           (hashAlg == &HASH_ALG_SHA256[0]) ||
           (hashAlg == &HASH_ALG_SM3[0]) ||
           (hashAlg == &HASH_ALG_MD5[0]);
}

/**
 * @brief Wait for one block compression and acknowledge it.
 * @return 0 on success; HASH_TimeOut_ERROR after the polling budget.
 * @discovery Poll loop shape from GCC HASH_ProcMsgBuf+0x1c..+0x3c:
 *            counter increments after each failed poll and the loop
 *            continues while Alg_TimeOut_Counter >= counter. On
 *            success SACSR bit3 is set through read-modify-write
 *            (write-1-to-clear acknowledge).
 */
static uint32_t HASH_WaitHashDone(void)
{
    uint32_t counter = 0U;

    while ((SAC_SACSR_REG & SAC_SACSR_HASH_DONE) == 0U) {
        counter++;
        if (Alg_TimeOut_Counter < counter) {
            return HASH_TimeOut_ERROR;
        }
    }
    SAC_SACSR_REG |= SAC_SACSR_HASH_DONE;
    return 0U;
}

/**
 * @brief Add a byte count to the 64-bit message length counter.
 * @param[in,out] msgByteLen Counter words: [0]=high 32 bits, [1]=low
 *                           32 bits.
 * @param[in] byteLen Number of bytes to add.
 * @return HASH_ByteLenPlus_OK on success; HASH_ByteLenPlus_ERROR when
 *         the high word reached 0x20000000 (message >= 2^61 bytes).
 * @discovery GCC HASH_ByteLenPlus1+0x00..+0x1a: adds at [1], carries
 *            into [0], rejects when [0] >= 0x20000000 (SHA-2 message
 *            limit).
 */
static uint32_t HASH_ByteLenPlus1(uint32_t *msgByteLen, uint32_t byteLen)
{
    msgByteLen[1] += byteLen;
    if (msgByteLen[1] < byteLen) {
        msgByteLen[0]++;
    }
    if (msgByteLen[0] < 0x20000000UL) {
        return HASH_ByteLenPlus_OK;
    }
    return HASH_ByteLenPlus_ERROR;
}

/**
 * @brief Push one complete message block to the accelerator.
 * @param[in,out] ctx Hash context whose msgBuf holds one full block.
 * @return HASH_ProcMsgBuf_OK on success; HASH_TimeOut_ERROR when the
 *         accelerator does not finish in time.
 * @post On success ctx->msgIdx is reset to 0 (not on timeout).
 * @discovery GCC HASH_ProcMsgBuf+0x04..+0x3c: BufToFIFO(IADR, msgBuf,
 *            BlockWordLen), SACSFR1 |= 0x80, wait for SACSR bit3,
 *            acknowledge bit3, msgIdx = 0.
 */
static uint32_t HASH_ProcMsgBuf(HASH_CTX *ctx)
{
    BufToFIFO(&SAC_IADR_REG, (const uint32_t *)(const void *)ctx->msgBuf,
              ctx->hashAlg->BlockWordLen);

    SAC_SACSFR1_REG |= SAC_SACSFR1_START;

    if (HASH_WaitHashDone() != 0U) {
        return HASH_TimeOut_ERROR;
    }

    ctx->msgIdx = 0U;
    return HASH_ProcMsgBuf_OK;
}

/**
 * @brief Append hash padding and the 64-bit bit length, then process
 *        the final block(s).
 * @param[in,out] ctx Hash context; msgBuf/msgIdx/msgByteLen are
 *                   consumed and modified.
 * @return HASH_PadMsg_OK on success; HASH_TimeOut_ERROR on timeout.
 * @discovery GCC HASH_PadMsg1+0x00..+0xe4 and armclang
 *            HASH_PadMsg1+0x00..+0x114. Writes 0x80 plus three zero
 *            bytes at msgBuf[msgIdx] (the slack in msgBuf[128+4]
 *            absorbs msgIdx up to 63+3); converts the byte length to
 *            a bit length kept as msgByteLen[0]=high/[1]=low; for
 *            MD5 the words are swapped to low-first order, otherwise
 *            both words are byte-reversed for a big-endian length;
 *            msgIdx > 55 forces a first block padded with zeros;
 *            the final block is padded to 14 words and followed by
 *            the two length words, then compressed.
 */
static uint32_t HASH_PadMsg1(HASH_CTX *ctx)
{
    uint32_t wordLen = (ctx->msgIdx + 4U) >> 2U;
    uint32_t i;
    uint32_t tmp;

    ctx->msgBuf[ctx->msgIdx] = 0x80U;
    ctx->msgBuf[ctx->msgIdx + 1U] = 0U;
    ctx->msgBuf[ctx->msgIdx + 2U] = 0U;
    ctx->msgBuf[ctx->msgIdx + 3U] = 0U;

    /* 64-bit bit length: new high = (high<<3)|(low>>29), new low =
     * low<<3, stored as msgByteLen[0]=high, [1]=low. */
    tmp = ctx->msgByteLen[1];
    ctx->msgByteLen[0] = (ctx->msgByteLen[0] << 3) | (tmp >> 29);
    ctx->msgByteLen[1] = tmp << 3;

    if (ctx->hashAlg->HashAlgID == ALG_MD5) {
        /* Little-endian length field: low word pushed first. */
        tmp = ctx->msgByteLen[0];
        ctx->msgByteLen[0] = ctx->msgByteLen[1];
        ctx->msgByteLen[1] = tmp;
    } else {
        /* Big-endian length field: byte-swap both words in place. */
        (void)ReverseBytesInWord_U32(ctx->msgByteLen, ctx->msgByteLen, 2U);
    }

    if (ctx->msgIdx > 55U) {
        /* The 0x80 word leaves no room for the length: compress this
         * block first (zero-filled to 16 words). */
        BufToFIFO(&SAC_IADR_REG, (const uint32_t *)(const void *)ctx->msgBuf, wordLen);
        for (i = wordLen; i < 16U; i++) {
            SAC_IADR_REG = 0U;
        }
        SAC_SACSFR1_REG |= SAC_SACSFR1_START;
        if (HASH_WaitHashDone() != 0U) {
            return HASH_TimeOut_ERROR;
        }
        wordLen = 0U;
    }

    BufToFIFO(&SAC_IADR_REG, (const uint32_t *)(const void *)ctx->msgBuf, wordLen);
    for (i = wordLen; i < 14U; i++) {
        SAC_IADR_REG = 0U;
    }
    SAC_IADR_REG = ctx->msgByteLen[0];
    SAC_IADR_REG = ctx->msgByteLen[1];

    SAC_SACSFR1_REG |= SAC_SACSFR1_START;
    if (HASH_WaitHashDone() != 0U) {
        return HASH_TimeOut_ERROR;
    }
    return HASH_PadMsg_OK;
}

/**
 * @brief Hash init
 * @param[in] ctx pointer to HASH_CTX struct
 * @return HASH_Init_OK,  Hash init success; othets:  Hash init fail
 * @note   1.Please refer to the demo in user guidance before using this function
 * @discovery GCC HASH_Init+0x28..+0x62: RCC_AHB1PCLKEN |= SACEN;
 *            SACCR = HASH_SACCR | 0x80; CLEAR_ARAM_DONE_CHECK();
 *            SACSFR1 = HASH_HASHCTRL; K constants uploaded to KADR;
 *            MD5 additionally uploads MD5_S (16 words) to P2ADR. The
 *            IV is NOT uploaded here (HASH_Start does that) and no
 *            context counter is reset.
 */
uint32_t HASH_Init(HASH_CTX *ctx)
{
    if ((ctx == NULL) || !HASH_IsKnownAlg(ctx->hashAlg)) {
        return HASH_Init_ERROR;
    }

    RCC_AHB1PCLKEN_REG |= RCC_AHB1PCLKEN_SACEN;

    SAC_SACCR_REG = ctx->hashAlg->HASH_SACCR | SAC_SACCR_ARAM_CLR;
    if (CLEAR_ARAM_DONE_CHECK() == Time_Out) {
        return HASH_TimeOut_ERROR;
    }

    SAC_SACSFR1_REG = ctx->hashAlg->HASH_HASHCTRL;

    BufToFIFO(&SAC_KADR_REG, ctx->hashAlg->K, ctx->hashAlg->KLen);

    if (ctx->hashAlg->HashAlgID == ALG_MD5) {
        BufToFIFO(&SAC_P2ADR_REG, MD5_S, 16U);
    }

    return HASH_Init_OK;
}

/**
 * @brief Hash start
 * @param[in] ctx pointer to HASH_CTX struct
 * @return HASH_Start_OK,  Hash start success; othets:  Hash start fail
 * @note   1.Please refer to the demo in user guidance before using this function
 *         2.HASH_Init() should be recalled before use this function
 * @discovery GCC HASH_Start+0x2a..+0x62: zeroes msgByteLen[4], sets
 *            msgIdx = 0; HASH_SEQUENCE_TRUE copies the table IV into
 *            ctx->IV (context-save mode); HASH_SEQUENCE_FALSE pushes
 *            the table IV to P1ADR (hardware-only state); any other
 *            sequence value fails with HASH_Start_ERROR.
 */
uint32_t HASH_Start(HASH_CTX *ctx)
{
    if ((ctx == NULL) || !HASH_IsKnownAlg(ctx->hashAlg)) {
        return HASH_Start_ERROR;
    }

    (void)SetZero_U32(ctx->msgByteLen, 4U);
    ctx->msgIdx = 0U;

    if (ctx->sequence == HASH_SEQUENCE_TRUE) {
        (void)Cpy_U32(ctx->IV, ctx->hashAlg->IV, ctx->hashAlg->IVLen);
    } else if (ctx->sequence == HASH_SEQUENCE_FALSE) {
        (void)BufToFIFO(&SAC_P1ADR_REG, ctx->hashAlg->IV, ctx->hashAlg->IVLen);
    } else {
        return HASH_Start_ERROR;
    }

    return HASH_Start_OK;
}

/**
 * @brief Hash update
 * @param[in] ctx pointer to HASH_CTX struct
 * @param[in] in pointer to message
 * @param[out] out pointer tohash result,digest
 * @return HASH_Update_OK,  Hash update success; othets:  Hash update fail
 * @note   1.Please refer to the demo in user guidance before using this function
 *         2.HASH_Init() and HASH_Start() should be recalled before use this function
 * @discovery GCC HASH_Update+0x0a..+0x1bc. Validation order: ctx NULL
 *            and unknown table -> HASH_Update_ERROR; byteLen == 0 ->
 *            success; in == NULL -> success (silent drop, suspected
 *            vendor defect); clock off or SACCR & 0x12 != 0x12 ->
 *            HASH_Init_ERROR. ByteLenPlus failure returns its own
 *            code (HASH_ByteLenPlus_ERROR). Inputs not spanning a
 *            block are buffered; otherwise, in context-save mode the
 *            saved IV is pushed to P1ADR first, a partial block is
 *            filled and compressed, remaining complete blocks are
 *            pushed either directly (4-byte aligned input) or through
 *            msgBuf, the tail is rebuffered, and in context-save mode
 *            the resulting state is read back from OADR into ctx->IV.
 */
uint32_t HASH_Update(HASH_CTX *ctx, uint8_t *in, uint32_t byteLen)
{
    const HASH_ALG *alg;
    uint32_t cycle;
    uint32_t begin;
    uint32_t remainder;
    uint32_t BlockByteLen;
    uint32_t i;
    uint32_t ret;

    if ((ctx == NULL) || !HASH_IsKnownAlg(ctx->hashAlg)) {
        return HASH_Update_ERROR;
    }

    if (byteLen == 0U) {
        return HASH_Update_OK;
    }
    if (in == NULL) {
        /* v1.2.0 returns success without hashing the data. */
        return HASH_Update_OK;
    }

    if (((RCC_AHB1PCLKEN_REG & RCC_AHB1PCLKEN_SACEN) == 0U) ||
        ((SAC_SACCR_REG & SAC_SACCR_HASH_READY) != SAC_SACCR_HASH_READY)) {
        return HASH_Init_ERROR;
    }

    alg = ctx->hashAlg;
    BlockByteLen = alg->BlockByteLen;

    ret = alg->ByteLenPlus(ctx->msgByteLen, byteLen);
    if (ret != HASH_ByteLenPlus_OK) {
        return ret;
    }

    begin = ctx->msgIdx + byteLen;
    if (begin < BlockByteLen) {
        (void)Cpy_U8(&ctx->msgBuf[ctx->msgIdx], in, byteLen);
        ctx->msgIdx = begin;
        return HASH_Update_OK;
    }

    if (ctx->sequence == HASH_SEQUENCE_TRUE) {
        (void)BufToFIFO(&SAC_P1ADR_REG, ctx->IV, alg->IVLen);
    }

    cycle = begin >> alg->Cycle; /* complete blocks in this update */

    if (ctx->msgIdx != 0U) {
        remainder = BlockByteLen - ctx->msgIdx;
        (void)Cpy_U8(&ctx->msgBuf[ctx->msgIdx], in, remainder);
        ret = HASH_ProcMsgBuf(ctx);
        if (ret == HASH_TimeOut_ERROR) {
            return HASH_TimeOut_ERROR;
        }
        cycle--;
        in += remainder;
    }

    for (i = 0U; i < cycle; i++) {
        if (((uint32_t)(uintptr_t)in & 3U) != 0U) {
            (void)Cpy_U8(ctx->msgBuf, in, BlockByteLen);
            ret = HASH_ProcMsgBuf(ctx);
            if (ret == HASH_TimeOut_ERROR) {
                return HASH_TimeOut_ERROR;
            }
        } else {
            (void)BufToFIFO(&SAC_IADR_REG, (const uint32_t *)(const void *)in,
                            alg->BlockWordLen);
            SAC_SACSFR1_REG |= SAC_SACSFR1_START;
            if (HASH_WaitHashDone() != 0U) {
                return HASH_TimeOut_ERROR;
            }
        }
        in += BlockByteLen;
    }

    ctx->msgIdx = (BlockByteLen - 1U) & begin;
    (void)Cpy_U8(ctx->msgBuf, in, ctx->msgIdx);

    if (ctx->sequence == HASH_SEQUENCE_TRUE) {
        (void)FIFOToBuf(ctx->IV, &SAC_OADR_REG, alg->IVLen);
    }

    return HASH_Update_OK;
}

/**
 * @brief Hash complete
 * @param[in] ctx pointer to HASH_CTX struct
 * @param[out] out pointer tohash result,digest
 * @return HASH_Complete_OK,  Hash complete success; othets:  Hash complete fail
 * @note   1.Please refer to the demo in user guidance before using this function
 *         2.HASH_Init(), HASH_Start() and HASH_Update() should be recalled before use this function
 * @discovery GCC HASH_Complete+0x08..+0x90. Zeroes a 64-byte stack
 *            digest buffer first, then dereferences ctx without a
 *            NULL check (vendor behavior). out NULL or unknown table
 *            -> HASH_Update_ERROR; hardware not ready ->
 *            HASH_Init_ERROR (HASH_Complete_ERROR is never returned).
 *            In context-save mode the saved IV is pushed to P1ADR
 *            before padding. The PadMsg return value is ignored
 *            (suspected vendor defect): the digest is read even if
 *            padding timed out. Aligned out goes through the temp
 *            buffer plus Cpy_U8; unaligned out is filled directly by
 *            FIFOToBuf.
 */
uint32_t HASH_Complete(HASH_CTX *ctx, uint8_t *out)
{
    uint8_t tmpBuf[64];
    uint32_t wordLen;

    (void)memset(tmpBuf, 0, sizeof(tmpBuf));

    if ((out == NULL) || !HASH_IsKnownAlg(ctx->hashAlg)) {
        return HASH_Update_ERROR;
    }

    if (((RCC_AHB1PCLKEN_REG & RCC_AHB1PCLKEN_SACEN) == 0U) ||
        ((SAC_SACCR_REG & SAC_SACCR_HASH_READY) != SAC_SACCR_HASH_READY)) {
        return HASH_Init_ERROR;
    }

    wordLen = ctx->hashAlg->DigestWordLen;

    if (ctx->sequence == HASH_SEQUENCE_TRUE) {
        (void)BufToFIFO(&SAC_P1ADR_REG, ctx->IV, ctx->hashAlg->IVLen);
    }

    (void)ctx->hashAlg->PadMsg(ctx);

    if (((uint32_t)(uintptr_t)out & 3U) == 0U) {
        (void)FIFOToBuf(tmpBuf, &SAC_OADR_REG, wordLen);
        (void)Cpy_U8(out, tmpBuf, wordLen * 4U);
    } else {
        (void)FIFOToBuf(out, &SAC_OADR_REG, wordLen);
    }

    return HASH_Complete_OK;
}

/**
 * @brief Hash close
 * @return HASH_Close_OK, Hash close success; othets: Hash close fail
 * @note   1.Please refer to the demo in user guidance before using this function
 * @discovery GCC HASH_Close+0x02..+0x3e: returns OK immediately when
 *            the SAC clock is off or SACCR & 0x12 != 0x12; otherwise
 *            sets SACCR bit8, clears the 0x12 configuration bits, runs
 *            CLEAR_ARAM_DONE_CHECK (timeout -> HASH_TimeOut_ERROR,
 *            not HASH_Close_ERROR) and finally gates the clock off.
 */
uint32_t HASH_Close(void)
{
    if ((RCC_AHB1PCLKEN_REG & RCC_AHB1PCLKEN_SACEN) == 0U) {
        return HASH_Close_OK;
    }
    if ((SAC_SACCR_REG & SAC_SACCR_HASH_READY) != SAC_SACCR_HASH_READY) {
        return HASH_Close_OK;
    }

    SAC_SACCR_REG |= SAC_SACCR_CLOSE;
    SAC_SACCR_REG &= ~SAC_SACCR_HASH_READY;

    if (CLEAR_ARAM_DONE_CHECK() == Time_Out) {
        return HASH_TimeOut_ERROR;
    }

    RCC_AHB1PCLKEN_REG &= ~RCC_AHB1PCLKEN_SACEN;
    return HASH_Close_OK;
}

/**
 * @brief One-shot hash driver shared by all five wrappers.
 * @param[in] alg Algorithm table.
 * @param[in] in Message buffer.
 * @param[in] byteLen Message length in bytes.
 * @param[out] out Digest output buffer.
 * @return Always 0; every step result is discarded, matching v1.2.0.
 * @discovery GCC SM3_Hash/SHA1_Hash/SHA224_Hash/SHA256_Hash/MD5_Hash
 *            are identical 0x44-byte wrappers over one 224-byte stack
 *            HASH_CTX (relocations resolve to HASH_Init, HASH_Start,
 *            HASH_Update, HASH_Complete, HASH_Close) with
 *            sequence = HASH_SEQUENCE_FALSE and a hard-coded
 *            "return 0".
 */
static uint32_t HASH_OneShot(const HASH_ALG *alg, uint8_t *in, uint32_t byteLen, uint8_t *out)
{
    HASH_CTX ctx;

    ctx.hashAlg = alg;
    ctx.sequence = HASH_SEQUENCE_FALSE;

    (void)HASH_Init(&ctx);
    (void)HASH_Start(&ctx);
    (void)HASH_Update(&ctx, in, byteLen);
    (void)HASH_Complete(&ctx, out);
    (void)HASH_Close();

    return 0U;
}

/**
 * @brief  SM3 Hash for 256bits digest
 * @param[in] in pointer to message
 * @param[in] byte length of in
 * @param[out] out pointer tohash result,digest
 * @return SM3_Hash_OK, SM3 hash success; othets: SM3 hash fail
 * @note   1.Please refer to the demo in user guidance before using this function
 */
uint32_t SM3_Hash(uint8_t *in, uint32_t byteLen, uint8_t *out)
{
    return HASH_OneShot(&HASH_ALG_SM3[0], in, byteLen, out);
}

/**
 * @brief  SHA1 Hash
 * @param[in] in pointer to message
 * @param[in] byte length of in
 * @param[out] out pointer tohash result,digest
 * @return SHA1_Hash_OK, SHA1 hash success; othets: SHA1 hash fail
 * @note   1.Please refer to the demo in user guidance before using this function
 */
uint32_t SHA1_Hash(uint8_t *in, uint32_t byteLen, uint8_t *out)
{
    return HASH_OneShot(&HASH_ALG_SHA1[0], in, byteLen, out);
}

/**
 * @brief  SHA224 Hash
 * @param[in] in pointer to message
 * @param[in] byte length of in
 * @param[out] out pointer tohash result,digest
 * @return SHA224_Hash_OK, SHA224 hash success; othets: SHA224 hash fail
 * @note   1.Please refer to the demo in user guidance before using this function
 */
uint32_t SHA224_Hash(uint8_t *in, uint32_t byteLen, uint8_t *out)
{
    return HASH_OneShot(&HASH_ALG_SHA224[0], in, byteLen, out);
}

/**
 * @brief  SHA256 Hash
 * @param[in] in pointer to message
 * @param[in] byte length of in
 * @param[out] out pointer tohash result,digest
 * @return SHA256_Hash_OK, SHA256 hash success; othets: SHA256 hash fail
 * @note   1.Please refer to the demo in user guidance before using this function
 */
uint32_t SHA256_Hash(uint8_t *in, uint32_t byteLen, uint8_t *out)
{
    return HASH_OneShot(&HASH_ALG_SHA256[0], in, byteLen, out);
}

/**
 * @brief  MD5 Hash
 * @param[in] in pointer to message
 * @param[in] byte length of in
 * @param[in] out pointer tohash result,digest
 * @return MD5_Hash_OK, MD5 hash success; othets: MD5 hash fail
 * @note   1.Please refer to the demo in user guidance before using this function
 */
uint32_t MD5_Hash(uint8_t *in, uint32_t byteLen, uint8_t *out)
{
    return HASH_OneShot(&HASH_ALG_MD5[0], in, byteLen, out);
}

/**
 * @brief Get HASH lib version
 * @param[out] type pointer one byte type information represents the type of the lib, like Commercial version.\
 * @Bits 0~4 stands for Commercial (C), Security (S), Normal (N), Evaluation (E), Test (T), Bits 5~7 are reserved. e.g. 0x09 stands for CE version.
 * @param[out] customer pointer one byte customer information represents customer ID. for example, 0x00 stands for standard version, 0x01 is for Tianyu customized version...
 * @param[out] date pointer array which include three bytes date information. If the returned bytes are 18,9,13,this denotes September 13,2018
 * @param[out] version pointer one byte version information represents develop version of the lib. e.g. 0x12 denotes version 1.2.
 * @return none
 * @1.You can recall this function to get RSA lib information
 * @discovery GCC/armclang HASH_Version+0x02..+0x18: writes 1, 0,
 *            {25,3,19} (2025-03-19, yy/mm/dd) and 0x10 (version 1.0
 *            by the header convention) without NULL checks. Note the
 *            v1.2.0 library reports an internal version of 0x10.
 */
void HASH_Version(uint8_t *type, uint8_t *customer, uint8_t date[3], uint8_t *version)
{
    *type = 1U;
    *customer = 0U;
    date[0] = 25U;
    date[1] = 3U;
    date[2] = 19U;
    *version = 0x10U;
}
