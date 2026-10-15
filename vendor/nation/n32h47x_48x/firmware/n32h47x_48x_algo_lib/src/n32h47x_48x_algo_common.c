/**
 * Reconstruction of the vendor closed-source library
 * n32h47x_48x_algo_lib v1.2.0, module n32h47x_48x_common.c.
 *
 * Behavior is aligned with the shipped binary
 * lib/n32h47x_48x_common.a (GNU ARM 10.2.1, -Os, DWARF-2) and cross-checked
 * against the armclang build (.lib member n32h47x_48x_common.o).
 * Evidence tags below cite "function+offset" inside those objects.
 *
 * Confirmed facts from the binary:
 * - Exported symbols (15): Alg_TimeOut_Counter (.data), CLEAR_ARAM_DONE_CHECK,
 *   RandomSort, Cpy_U8, Cpy_U32, XOR_U8, XOR_U32, SetZero_U8, SetZero_U32,
 *   ReverseBytesInWord_U8, ReverseBytesInWord_U32, Cmp_U32, Cmp_U8,
 *   BufToFIFO, FIFOToBuf. BufToFIFO/FIFOToBuf/ReverseBytesInWord_U8 are
 *   exported but not declared in the public header.
 * - DWARF signatures: BufToFIFO(volatile uint32_t *dst, const void *src,
 *   uint32_t wordLen) at vendor src line 680; FIFOToBuf(void *dst,
 *   volatile uint32_t *src, uint32_t wordLen) at vendor src line 742. The
 *   third parameter of both FIFO helpers is a 32-bit word count, not bytes.
 * - Internal calls resolved by relocations: Cpy_U32 unaligned path tail-jumps
 *   to Cpy_U8 (Cpy_U32+0x16, R_ARM_THM_JUMP24); SetZero_U8 and SetZero_U32
 *   call memset (SetZero_U8+0x06, SetZero_U32+0x1c); ReverseBytesInWord_U32
 *   calls ReverseBytesInWord_U8 (ReverseBytesInWord_U32+0x02); BufToFIFO and
 *   FIFOToBuf call Cpy_U8 for stack staging and SetZero_U32(tmp, 4) to wipe
 *   the 16-byte temp buffer (BufToFIFO+0x36/+0x62/+0x6e,
 *   FIFOToBuf+0x4c/+0x6e/+0x76).
 * - CLEAR_ARAM_DONE_CHECK polls SACCR bit 7 and performs a read-modify-write
 *   of SACSR bit 0 on success (CLEAR_ARAM_DONE_CHECK+0x08..+0x16), confirmed
 *   identical in the armclang object.
 */

#include <stdint.h>
#include <string.h>
#include "n32h47x_48x_algo_common.h"

/* Global timeout budget for hardware polling loops.
 * Initial value 0x000186A0 (100000) per .data.Alg_TimeOut_Counter. */
uint32_t Alg_TimeOut_Counter = 100000U;

/* SAC (Security Algorithm Coprocessor) register block.
 * CMSIS n32h47x_48x.h: SAC_BASE = AHBPERIPH_BASE + 0xA000 = 0x4002A000.
 * Register names follow the vendor-private SAC_Common.h layout recovered
 * from DWARF (SAC_TypeDef: SACCR@+0x00, SACSR@+0x04). */
#define SAC_REG_BASE          0x4002A000U
#define SAC_SACCR_REG         (*(volatile uint32_t *)(SAC_REG_BASE + 0x00U))
#define SAC_SACSR_REG         (*(volatile uint32_t *)(SAC_REG_BASE + 0x04U))

#define SAC_SACCR_BUSY_MASK   0x80U /* Bit 7: ARAM clear operation busy. */
#define SAC_SACSR_CLRDONE     0x01U /* Bit 0: clear-done acknowledge. */

/**
 * @brief Copy data byte by byte.
 * @param[in] dst Pointer to destination buffer.
 * @param[in] src Pointer to source buffer.
 * @param[in] byteLen Number of bytes to copy.
 * @return Cpy_OK: success; others: fail.
 * @note Any alignment is accepted.
 * @discovery Cpy_U8+0x00..+0x12: simple byte loop, returns 0.
 */
uint32_t Cpy_U8(uint8_t *dst, uint8_t *src, uint32_t byteLen)
{
    uint32_t i;

    for (i = 0U; i < byteLen; i++)
    {
        dst[i] = src[i];
    }

    return Cpy_OK;
}

/**
 * @brief Copy data word by word with byte fallback for unaligned pointers.
 * @param[in] dst Pointer to destination buffer.
 * @param[in] src Pointer to source buffer.
 * @param[in] wordLen Number of 32-bit words to copy.
 * @return Cpy_OK: success; others: fail.
 * @note If either pointer is not 4-byte aligned the whole region is copied
 *       byte by byte; otherwise words are copied in groups of four.
 * @discovery Cpy_U32+0x02..+0x10 checks (dst|src)&3; Cpy_U32+0x14..+0x16
 *            tail-jumps to Cpy_U8 with byteLen = wordLen*4; aligned path
 *            copies 4 words per iteration then the wordLen&3 remainder.
 */
uint32_t Cpy_U32(uint32_t *dst, const uint32_t *src, uint32_t wordLen)
{
    uint32_t i;

    if ((((uint32_t)dst | (uint32_t)src) & 0x3U) != 0U)
    {
        return Cpy_U8((uint8_t *)dst, (uint8_t *)src, wordLen * 4U);
    }

    for (i = wordLen >> 2U; i > 0U; i--)
    {
        dst[0] = src[0];
        dst[1] = src[1];
        dst[2] = src[2];
        dst[3] = src[3];
        dst += 4;
        src += 4;
    }

    for (i = wordLen & 0x3U; i > 0U; i--)
    {
        *dst++ = *src++;
    }

    return Cpy_OK;
}

/**
 * @brief XOR two byte arrays.
 * @param[in] a Pointer to one input array.
 * @param[in] b Pointer to another input array.
 * @param[out] c Pointer to the output array, may alias a or b.
 * @param[in] byteLen Number of bytes to process.
 * @return XOR_OK: operation success; others: operation fail.
 * @discovery XOR_U8+0x0c..+0x16: c[i] = a[i]^b[i], returns 0.
 */
uint32_t XOR_U8(uint8_t *a, uint8_t *b, uint8_t *c, uint32_t byteLen)
{
    uint32_t i;

    for (i = 0U; i < byteLen; i++)
    {
        c[i] = a[i] ^ b[i];
    }

    return XOR_OK;
}

/**
 * @brief XOR two 32-bit word arrays.
 * @param[in] a Pointer to one input array.
 * @param[in] b Pointer to another input array.
 * @param[out] c Pointer to the output array, may alias a or b.
 * @param[in] wordLen Number of 32-bit words to process.
 * @return XOR_OK: operation success; others: operation fail.
 * @discovery XOR_U32+0x0c..+0x1a: c[i] = a[i]^b[i] word-wise, returns 0.
 */
uint32_t XOR_U32(uint32_t *a, uint32_t *b, uint32_t *c, uint32_t wordLen)
{
    uint32_t i;

    for (i = 0U; i < wordLen; i++)
    {
        c[i] = a[i] ^ b[i];
    }

    return XOR_OK;
}

/**
 * @brief Set a byte region to zero.
 * @param[in] dst Pointer to the region to clear.
 * @param[in] byteLen Number of bytes to clear.
 * @return SetZero_OK: success; others: fail.
 * @discovery SetZero_U8+0x02..+0x0a: memset(dst, 0, byteLen), returns 0
 *            (relocation SetZero_U8+0x06 -> memset).
 */
uint32_t SetZero_U8(uint8_t *dst, uint32_t byteLen)
{
    (void)memset(dst, 0, byteLen);

    return SetZero_OK;
}

/**
 * @brief Set a word region to zero.
 * @param[in] dst Pointer to the region to clear.
 * @param[in] wordLen Number of 32-bit words to clear.
 * @return SetZero_OK: success; others: fail.
 * @discovery SetZero_U32+0x24..+0x2c clears wordLen>>2 groups of four words
 *            (strd pairs), then memsets the wordLen&3 tail words
 *            (SetZero_U32+0x1c -> memset), returns 0.
 */
uint32_t SetZero_U32(uint32_t *dst, uint32_t wordLen)
{
    uint32_t i;

    for (i = wordLen >> 2U; i > 0U; i--)
    {
        dst[0] = 0U;
        dst[1] = 0U;
        dst[2] = 0U;
        dst[3] = 0U;
        dst += 4;
    }

    (void)memset(dst, 0, (wordLen & 0x3U) * sizeof(uint32_t));

    return SetZero_OK;
}

/**
 * @brief Wait until the SAC ARAM clear operation finishes, then acknowledge it.
 * @return SetZero_OK: the busy flag cleared and the acknowledge bit was set.
 * @retval Time_Out The poll count exceeded Alg_TimeOut_Counter; no
 *                  acknowledgement was written and the accelerator may still
 *                  be busy.
 * @note Blocks while polling SACCR bit 7. With Alg_TimeOut_Counter = N the
 *       status register is read at most N+1 times before Time_Out is
 *       returned. The success path reads SACSR back and writes value|1
 *       (read-modify-write), it does not write a plain 1.
 * @discovery CLEAR_ARAM_DONE_CHECK+0x00 loads Alg_TimeOut_Counter once (GCC;
 *            the armclang build reloads it every iteration), +0x08..+0x0e
 *            polls SACCR&0x80, +0x10..+0x16 does SACSR |= 1, +0x20 returns
 *            Time_Out(2).
 */
uint32_t CLEAR_ARAM_DONE_CHECK(void)
{
    uint32_t i = 0U;

    while ((SAC_SACCR_REG & SAC_SACCR_BUSY_MASK) != 0U)
    {
        i++;
        if (i > Alg_TimeOut_Counter)
        {
            return Time_Out;
        }
    }

    SAC_SACSR_REG |= SAC_SACSR_CLRDONE;

    return SetZero_OK;
}

/**
 * @brief Shuffle a byte sequence with the Fisher-Yates algorithm.
 * @param[in] order Pointer to the sequence; overwritten with 0..len-1 in
 *                  shuffled order.
 * @param[in] rand Pointer to the random byte source, must supply at least
 *                 len bytes.
 * @param[in] len Length of the sequence in bytes.
 * @return Cpy_OK (0): the header documents a RandomSort_OK result that is
 *         not defined by the public header; the binary always returns 0.
 * @discovery RandomSort+0x14..+0x18 initializes order[i]=i; +0x1a..+0x34
 *            computes index = i + rand[i] % (len - i) (udiv/mls) and swaps
 *            order[i]/order[index].
 */
uint32_t RandomSort(uint8_t *order, const uint8_t *rand, uint32_t len)
{
    uint32_t i;
    uint32_t index;
    uint8_t tmp;

    for (i = 0U; i < len; i++)
    {
        order[i] = (uint8_t)i;
    }

    for (i = 0U; i < len; i++)
    {
        index = i + (rand[i] % (len - i));

        tmp = order[i];
        order[i] = order[index];
        order[index] = tmp;
    }

    return Cpy_OK;
}

/**
 * @brief Reverse the byte order inside each 32-bit word.
 * @param[in] dst Pointer to the destination buffer.
 * @param[in] src Pointer to the source buffer, may equal dst.
 * @param[in] wordLen Number of 32-bit words to reverse; every word occupies
 *                    four consecutive bytes.
 * @return Reverse_OK: success; others: fail.
 * @note dst==src takes the in-place path swapping bytes 0<->3 and 1<->2;
 *       otherwise dst[4i+k] = src[4i+3-k].
 * @discovery ReverseBytesInWord_U8+0x00..+0x08 compares src/dst;
 *            +0x16..+0x32 out-of-place mapping; +0x3a..+0x56 in-place swaps.
 */
uint32_t ReverseBytesInWord_U8(uint8_t *dst, const uint8_t *src, uint32_t wordLen)
{
    uint32_t i;
    uint8_t temp;

    if (dst == src)
    {
        for (i = 0U; i < wordLen; i++)
        {
            temp = dst[i * 4U];
            dst[i * 4U] = dst[i * 4U + 3U];
            dst[i * 4U + 3U] = temp;

            temp = dst[i * 4U + 1U];
            dst[i * 4U + 1U] = dst[i * 4U + 2U];
            dst[i * 4U + 2U] = temp;
        }
    }
    else
    {
        for (i = 0U; i < wordLen; i++)
        {
            dst[i * 4U] = src[i * 4U + 3U];
            dst[i * 4U + 1U] = src[i * 4U + 2U];
            dst[i * 4U + 2U] = src[i * 4U + 1U];
            dst[i * 4U + 3U] = src[i * 4U];
        }
    }

    return Reverse_OK;
}

/**
 * @brief Reverse the byte order inside each 32-bit word (word-typed wrapper).
 * @param[in] dst Pointer to the destination array.
 * @param[in] src Pointer to the source array, may equal dst.
 * @param[in] wordLen Number of 32-bit words to reverse.
 * @return Reverse_OK: success; others: fail.
 * @discovery ReverseBytesInWord_U32+0x02 calls ReverseBytesInWord_U8 with
 *            the arguments passed through, discards its return value and
 *            returns 0.
 */
uint32_t ReverseBytesInWord_U32(uint32_t *dst, const uint32_t *src, uint32_t wordLen)
{
    (void)ReverseBytesInWord_U8((uint8_t *)dst, (const uint8_t *)src, wordLen);

    return Reverse_OK;
}

/**
 * @brief Compare two big numbers held in 32-bit word arrays.
 * @param[in] a Pointer to the first number.
 * @param[in] aWordLen Word length of a.
 * @param[in] b Pointer to the second number.
 * @param[in] bWordLen Word length of b.
 * @return Cmp_EQUAL if lengths and all words match, Cmp_UNEQUAL otherwise.
 * @discovery Cmp_U32+0x00..+0x04 compares lengths first; +0x10..+0x1e walks
 *            the words; returns 1 on any mismatch.
 */
int32_t Cmp_U32(const uint32_t *a, uint32_t aWordLen, const uint32_t *b, uint32_t bWordLen)
{
    uint32_t i;

    if (aWordLen != bWordLen)
    {
        return Cmp_UNEQUAL;
    }

    for (i = 0U; i < aWordLen; i++)
    {
        if (a[i] != b[i])
        {
            return Cmp_UNEQUAL;
        }
    }

    return Cmp_EQUAL;
}

/**
 * @brief Compare two big numbers held in byte arrays.
 * @param[in] a Pointer to the first number.
 * @param[in] aByteLen Byte length of a.
 * @param[in] b Pointer to the second number.
 * @param[in] bByteLen Byte length of b.
 * @return Cmp_EQUAL if lengths and all bytes match, Cmp_UNEQUAL otherwise.
 * @discovery Cmp_U8+0x00..+0x04 compares lengths first; +0x10..+0x1a walks
 *            the bytes; returns 1 on any mismatch.
 */
int32_t Cmp_U8(const uint8_t *a, uint32_t aByteLen, const uint8_t *b, uint32_t bByteLen)
{
    uint32_t i;

    if (aByteLen != bByteLen)
    {
        return Cmp_UNEQUAL;
    }

    for (i = 0U; i < aByteLen; i++)
    {
        if (a[i] != b[i])
        {
            return Cmp_UNEQUAL;
        }
    }

    return Cmp_EQUAL;
}

/**
 * @brief Push a memory buffer into a hardware FIFO write port, word by word.
 * @param[in] fifo_addr Pointer to the 32-bit FIFO write register; every
 *                      store enqueues one little-endian word.
 * @param[in] buffer Pointer to the source data; wordLen*4 readable bytes
 *                   are required.
 * @param[in] wordLen Number of 32-bit words to transfer (NOT bytes).
 * @return Cpy_OK: success; others: fail.
 * @note If buffer is not 4-byte aligned the data is staged through a
 *       16-byte stack buffer copied by Cpy_U8, four words per round; the
 *       staging buffer is wiped with SetZero_U32(tmp, 4) afterwards.
 *       Exactly wordLen words are written to the FIFO in both paths.
 * @discovery BufToFIFO+0x08 checks buffer alignment; +0x88..+0xa2 aligned
 *            4-word rounds; +0x32..+0x54 staged rounds; +0x56..+0x6e tail
 *            words and SetZero_U32; +0x72 returns 0. The armclang object
 *            (i.BufToFIFO+0x0c..+0x14) matches.
 */
uint32_t BufToFIFO(volatile uint32_t *fifo_addr, const void *buffer, uint32_t wordLen)
{
    uint32_t i;
    uint32_t tmp[4];
    const uint8_t *s = (const uint8_t *)buffer;

    if (((uint32_t)s & 0x3U) != 0U)
    {
        for (i = wordLen >> 2U; i > 0U; i--)
        {
            (void)Cpy_U8((uint8_t *)tmp, (uint8_t *)s, 16U);
            *fifo_addr = tmp[0];
            *fifo_addr = tmp[1];
            *fifo_addr = tmp[2];
            *fifo_addr = tmp[3];
            s += 16U;
        }

        i = wordLen & 0x3U;
        (void)Cpy_U8((uint8_t *)tmp, (uint8_t *)s, i * 4U);
        while (i > 0U)
        {
            *fifo_addr = tmp[(wordLen & 0x3U) - i];
            i--;
        }

        (void)SetZero_U32(tmp, 4U);
    }
    else
    {
        const uint32_t *w = (const uint32_t *)(const void *)s;

        for (i = wordLen >> 2U; i > 0U; i--)
        {
            *fifo_addr = w[0];
            *fifo_addr = w[1];
            *fifo_addr = w[2];
            *fifo_addr = w[3];
            w += 4;
        }

        for (i = wordLen & 0x3U; i > 0U; i--)
        {
            *fifo_addr = *w++;
        }
    }

    return Cpy_OK;
}

/**
 * @brief Pull words from a hardware FIFO read port into a memory buffer.
 * @param[out] buffer Pointer to the destination buffer with room for
 *                    wordLen words. NOTE: this is the FIRST parameter,
 *                    unlike BufToFIFO where the FIFO comes first.
 * @param[in] fifo_addr Pointer to the 32-bit FIFO read register; every
 *                      load dequeues one word.
 * @param[in] wordLen Number of 32-bit words to transfer (NOT bytes).
 * @return Cpy_OK: success; others: fail.
 * @note The alignment gate in the binary is applied to fifo_addr (the
 *       source), not to the destination buffer; because FIFO register
 *       addresses are always 4-byte aligned the staging path is effectively
 *       unreachable and words are always stored directly into buffer.
 *       The quirk is reproduced verbatim from v1.2.0.
 * @discovery FIFOToBuf+0x04 checks src(fifo) alignment (GCC and armclang);
 *            +0x94..+0xac aligned 4-word rounds; +0x36..+0x5a staged rounds
 *            with Cpy_U8 and SetZero_U32(tmp, 4); +0x7a returns 0. DWARF
 *            names the parameters dst(void*)/src(volatile uint32_t*).
 */
uint32_t FIFOToBuf(void *buffer, volatile uint32_t *fifo_addr, uint32_t wordLen)
{
    uint32_t i;
    uint32_t tmp[4];
    uint32_t *d = (uint32_t *)buffer;

    if (((uint32_t)fifo_addr & 0x3U) != 0U)
    {
        for (i = wordLen >> 2U; i > 0U; i--)
        {
            tmp[0] = *fifo_addr;
            tmp[1] = *fifo_addr;
            tmp[2] = *fifo_addr;
            tmp[3] = *fifo_addr;
            (void)Cpy_U8((uint8_t *)d, (uint8_t *)tmp, 16U);
            d += 4;
        }

        i = 0U;
        while (i < (wordLen & 0x3U))
        {
            tmp[i] = *fifo_addr;
            i++;
        }
        (void)Cpy_U8((uint8_t *)d, (uint8_t *)tmp, i * 4U);

        (void)SetZero_U32(tmp, 4U);
    }
    else
    {
        for (i = wordLen >> 2U; i > 0U; i--)
        {
            d[0] = *fifo_addr;
            d[1] = *fifo_addr;
            d[2] = *fifo_addr;
            d[3] = *fifo_addr;
            d += 4;
        }

        for (i = wordLen & 0x3U; i > 0U; i--)
        {
            *d++ = *fifo_addr;
        }
    }

    return Cpy_OK;
}
