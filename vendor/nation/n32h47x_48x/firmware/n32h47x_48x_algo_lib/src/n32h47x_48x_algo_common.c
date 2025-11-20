#include <stdint.h>
#include "n32h47x_48x_algo_common.h"

// Global timeout counter for hardware operations
uint32_t Alg_TimeOut_Counter = 0;

/**
 * @brief Hardware FIFO write operation - Buffer to FIFO
 * @param[in] fifo_addr Pointer to hardware FIFO register address
 * @param[in] buffer Pointer to source data buffer
 * @param[in] byte_len Length of data to transfer in bytes
 * @param[in] channel FIFO channel identifier (likely hardware-specific)
 * @return Cpy_OK: success; others: fail
 * @note This function handles both aligned and unaligned memory accesses
 *       and performs word-aligned transfers to hardware FIFO registers
 * @discovery Address 0x0E: This appears to be a hardware FIFO interface function
 *           that transfers data from memory to cryptographic hardware FIFO
 */
uint32_t BufToFIFO(uint32_t *fifo_addr, uint32_t *buffer, uint32_t byte_len, uint32_t channel)
{
    uint32_t word_count;
    uint32_t remaining_bytes;
    uint32_t i;
    uint32_t temp_data[4] = {0};
    uint32_t *temp_ptr = temp_data;

    // Store parameters for potential debugging
    uint32_t *dest_fifo = fifo_addr;
    uint32_t *src_buffer = buffer;
    uint32_t length = byte_len;
    uint32_t fifo_channel = channel;

    // Check if buffer address is not word-aligned
    if (((uint32_t)src_buffer & 0x3) != 0)
    {
        // Unaligned transfer path - use byte-wise copy with temporary buffer
        word_count = length >> 2;  // Number of complete words

        for (i = word_count; i > 0; i--)
        {
            // Copy 16 bytes (4 words) using byte copy function
            Cpy_U8((uint8_t *)&temp_ptr, (uint8_t *)src_buffer, 16);

            // Write to hardware FIFO (assuming *fifo_addr writes to FIFO)
            *dest_fifo = temp_ptr[0];  // Write first word
            *dest_fifo = temp_ptr[1];  // Write second word
            *dest_fifo = temp_ptr[2];  // Write third word
            *dest_fifo = temp_ptr[3];  // Write fourth word

            src_buffer += 4;  // Advance by 4 words
        }

        // Handle remaining bytes (less than 4 words)
        remaining_bytes = length & 0x3;

        // Copy remaining bytes to temporary buffer
        Cpy_U8((uint8_t *)&temp_ptr, (uint8_t *)src_buffer, 4 * remaining_bytes);

        // Write remaining words to FIFO
        for (i = 0; i < remaining_bytes; i++)
        {
            *dest_fifo = temp_ptr[i];
        }

        // Clear temporary buffer for security
        SetZero_U32(temp_ptr, 4);
    }
    else
    {
        // Aligned transfer path - direct word transfers
        word_count = length >> 2;  // Number of complete words

        for (i = word_count; i > 0; i--)
        {
            // Direct word-aligned transfer to FIFO
            *dest_fifo = src_buffer[0];
            *dest_fifo = src_buffer[1];
            *dest_fifo = src_buffer[2];
            *dest_fifo = src_buffer[3];

            src_buffer += 4;
        }

        // Handle remaining bytes
        for (i = length & 0x3; i > 0; i--)
        {
            *dest_fifo = *src_buffer++;
        }
    }

    return Cpy_OK;
}

/**
 * @brief Hardware FIFO read operation - FIFO to Buffer
 * @param[in] fifo_addr Pointer to hardware FIFO register address
 * @param[in] buffer Pointer to destination data buffer
 * @param[in] byte_len Length of data to transfer in bytes
 * @param[in] channel FIFO channel identifier (likely hardware-specific)
 * @return Cpy_OK: success; others: fail
 * @note This function reads data from hardware FIFO registers to memory
 * @discovery Address 0x196: Complementary function to BufToFIFO, reads from FIFO
 */
uint32_t FIFOToBuf(uint32_t *fifo_addr, uint32_t *buffer, uint32_t byte_len, uint32_t channel)
{
    uint32_t word_count;
    uint32_t remaining_bytes;
    uint32_t i, j;
    uint32_t temp_data[4] = {0};
    uint32_t *temp_ptr = temp_data;

    // Store parameters
    uint32_t *dest_buffer = buffer;
    uint32_t *src_fifo = fifo_addr;
    uint32_t length = byte_len;
    uint32_t fifo_channel = channel;

    // Check if buffer address is not word-aligned
    if (((uint32_t)buffer & 0x3) != 0)
    {
        // Unaligned receive path
        word_count = length >> 2;

        for (i = word_count; i > 0; i--)
        {
            // Read 4 words from hardware FIFO
            temp_ptr[0] = *src_fifo;  // Read first word
            temp_ptr[1] = *src_fifo;  // Read second word
            temp_ptr[2] = *src_fifo;  // Read third word
            temp_ptr[3] = *src_fifo;  // Read fourth word

            // Copy to destination buffer using byte-wise copy
            Cpy_U8((uint8_t *)dest_buffer, (uint8_t *)&temp_ptr, 16);

            dest_buffer += 4;
        }

        // Handle remaining words from FIFO
        i = 0;
        while ((length & 0x3) > i)
        {
            temp_ptr[i++] = *src_fifo;
        }

        // Copy remaining bytes
        Cpy_U8((uint8_t *)dest_buffer, (uint8_t *)&temp_ptr, 4 * i);

        // Clear temporary buffer
        SetZero_U32(temp_ptr, 4);
    }
    else
    {
        // Aligned receive path - direct transfers
        word_count = length >> 2;

        for (i = word_count; i > 0; i--)
        {
            // Direct word-aligned read from FIFO
            dest_buffer[0] = *src_fifo;
            dest_buffer[1] = *src_fifo;
            dest_buffer[2] = *src_fifo;
            dest_buffer[3] = *src_fifo;

            dest_buffer += 4;
        }

        // Handle remaining bytes
        for (i = length & 0x3; i > 0; i--)
        {
            *dest_buffer++ = *src_fifo;
        }
    }

    return Cpy_OK;
}

// Hardware register definitions for N32H474/75 cryptographic accelerator
#define CRYPTO_BASE_ADDR           0x4002A000U
#define CRYPTO_STATUS_REG          (*(volatile uint32_t *)(CRYPTO_BASE_ADDR + 0x00))
#define CRYPTO_CONTROL_REG         (*(volatile uint32_t *)(CRYPTO_BASE_ADDR + 0x04))

// Status register bit definitions
#define CRYPTO_STATUS_BUSY_FLAG    0x80U    // Bit 7: Cryptographic operation busy flag
#define CRYPTO_CTRL_CLEAR_DONE     0x01U    // Bit 0: Clear operation done flag

/**
 * @brief Clear ARAM completion check and acknowledge
 * @return SetZero_OK: operation completed successfully; Time_Out: timeout occurred
 * @note This function polls the cryptographic accelerator status register to wait
 *       for ARAM (Access RAM) clear operations to complete, then acknowledges
 *       the completion by setting the control register's clear done flag.
 * @discovery Address 0x0A0: Hardware polling loop with timeout mechanism
 * @hardware Access to cryptographic accelerator registers:
 *         - 0x4002A000: Status register (reads busy flag at bit 7)
 *         - 0x4002A004: Control register (writes clear done flag at bit 0)
 */
uint32_t CLEAR_ARAM_DONE_CHECK(void)
{
    uint32_t timeout_counter = 0;

    // Poll the cryptographic accelerator status register
    // Check if the BUSY flag (bit 7) is set, indicating ARAM clear in progress
    while ((CRYPTO_STATUS_REG & CRYPTO_STATUS_BUSY_FLAG) != 0)
    {
        timeout_counter++;

        // Check if timeout has been exceeded
        if (timeout_counter > Alg_TimeOut_Counter)
        {
            return Time_Out;  // Return timeout error
        }
    }

    // ARAM clear operation completed, acknowledge by setting control bit
    CRYPTO_CONTROL_REG |= CRYPTO_CTRL_CLEAR_DONE;

    return SetZero_OK;  // Return success
}

/**
 * @brief Shuffle sequence order using Fisher-Yates algorithm with provided randomness
 * @param[in] order Pointer to sequence array to be shuffled
 * @param[in] rand Pointer to random number array for shuffling
 * @param[in] len Length of the sequence
 * @return RandomSort_OK: shuffle success; others: shuffle fail
 * @note Implements Fisher-Yates shuffle using provided random bytes
 * @discovery Address 0x22A: Standard Fisher-Yates shuffle implementation
 */
uint32_t RandomSort(uint8_t *order, const uint8_t *rand, uint32_t len)
{
    uint32_t i, j;
    uint32_t swap_index;
    uint8_t temp;

    // Initialize sequence with sequential values 0,1,2,...len-1
    for (i = 0; i < len; i++)
    {
        order[i] = (uint8_t)i;
    }

    // Fisher-Yates shuffle using provided random bytes
    for (j = 0; j < len; j++)
    {
        // Use random byte modulo remaining elements to get swap index
        swap_index = (rand[j] % (len - j)) + j;

        // Swap elements
        temp = order[j];
        order[j] = order[swap_index];
        order[swap_index] = temp;
    }

    return Cpy_OK;
}

/**
 * @brief Copy data by byte with basic memory transfer
 * @param[in] dst Pointer to destination memory address
 * @param[in] src Pointer to source memory address
 * @param[in] byteLen Length of data to copy in bytes
 * @return Cpy_OK: success; others: fail
 * @note Simple byte-wise memory copy, handles any alignment
 * @discovery Address 0x17E: Basic memcpy implementation
 */
uint32_t Cpy_U8(uint8_t *dst, uint8_t *src, uint32_t byteLen)
{
    uint32_t i;

    // Basic byte-wise copy loop
    for (i = 0; i < byteLen; i++)
    {
        dst[i] = src[i];
    }

    return Cpy_OK;
}

/**
 * @brief Copy data by word with optimization for aligned memory
 * @param[in] dst Pointer to destination memory address
 * @param[in] src Pointer to source memory address
 * @param[in] wordLen Length of data to copy in 32-bit words
 * @return Cpy_OK: success; others: fail
 * @note Optimized word-wise copy with fallback to byte copy for unaligned addresses
 * @discovery Address 0x12C: Optimized memcpy with alignment checking
 */
uint32_t Cpy_U32(uint32_t *dst, const uint32_t *src, uint32_t wordLen)
{
    uint32_t *dest_ptr = dst;
    const uint32_t *src_ptr = src;
    uint32_t word_groups;
    uint32_t i;
    uint32_t remaining_words;

    // Check if both addresses are word-aligned
    if (((uint32_t)dest_ptr & 0x3) != 0 || ((uint32_t)src_ptr & 0x3) != 0)
    {
        // Fallback to byte copy for unaligned addresses
        return Cpy_U8((uint8_t *)dst, (uint8_t *)src, 4 * wordLen);
    }

    // Optimized copy using 4-word groups for better performance
    word_groups = wordLen >> 2;  // Number of 4-word groups

    for (i = word_groups; i > 0; i--)
    {
        // Copy 4 words at once for better cache utilization
        dest_ptr[0] = src_ptr[0];
        dest_ptr[1] = src_ptr[1];
        dest_ptr[2] = src_ptr[2];
        dest_ptr[3] = src_ptr[3];

        dest_ptr += 4;
        src_ptr += 4;
    }

    // Handle remaining words
    for (remaining_words = wordLen & 0x3; remaining_words > 0; remaining_words--)
    {
        *dest_ptr++ = *src_ptr++;
    }

    return Cpy_OK;
}

/**
 * @brief XOR operation on byte arrays
 * @param[in] a Pointer to first input array
 * @param[in] b Pointer to second input array
 * @param[out] result Pointer to output array for XOR result
 * @param[in] byteLen Length of arrays in bytes
 * @return XOR_OK: operation success; others: operation fail
 * @note Performs a[i] ^ b[i] and stores in result[i]
 * @discovery Address 0x370: Basic byte-wise XOR operation
 */
uint32_t XOR_U8(uint8_t *a, uint8_t *b, uint8_t *result, uint32_t byteLen)
{
    uint32_t i;

    // Byte-wise XOR operation
    for (i = 0; i < byteLen; i++)
    {
        result[i] = a[i] ^ b[i];
    }

    return XOR_OK;
}

/**
 * @brief XOR operation on 32-bit word arrays
 * @param[in] a Pointer to first input array
 * @param[in] b Pointer to second input array
 * @param[out] result Pointer to output array for XOR result
 * @param[in] wordLen Length of arrays in 32-bit words
 * @return XOR_OK: operation success; others: operation fail
 * @note Performs word-wise XOR for better performance on aligned data
 * @discovery Address 0x34E: Optimized word-wise XOR operation
 */
uint32_t XOR_U32(uint32_t *a, uint32_t *b, uint32_t *result, uint32_t wordLen)
{
    uint32_t i;

    // Word-wise XOR operation
    for (i = 0; i < wordLen; i++)
    {
        result[i] = a[i] ^ b[i];
    }

    return XOR_OK;
}

/**
 * @brief Set memory to zero by byte
 * @param[in] dst Pointer to memory address to be zeroed
 * @param[in] byteLen Length in bytes to zero
 * @return SetZero_OK: success; others: fail
 * @note Secure memory zeroization for cryptographic key material
 * @discovery Address 0x338: Basic memset implementation
 */
uint32_t SetZero_U8(uint8_t *dst, uint32_t byteLen)
{
    uint32_t i;

    // Byte-wise zeroization
    for (i = 0; i < byteLen; i++)
    {
        dst[i] = 0;
    }

    return SetZero_OK;
}

/**
 * @brief Set memory to zero by word
 * @param[in] dst Pointer to memory address to be zeroed
 * @param[in] wordLen Length in 32-bit words to zero
 * @return SetZero_OK: success; others: fail
 * @note Optimized word-wise zeroization for aligned memory
 * @discovery Address 0x30A: Optimized memset with word groups
 */
uint32_t SetZero_U32(uint32_t *dst, uint32_t wordLen)
{
    uint32_t *dest_ptr = dst;
    uint32_t word_groups;
    uint32_t remaining_words;
    uint32_t i;

    // Optimized zeroization using 4-word groups
    word_groups = wordLen >> 2;  // Number of 4-word groups

    for (i = word_groups; i > 0; i--)
    {
        // Zero 4 words at once for better performance
        dest_ptr[0] = 0;
        dest_ptr[1] = 0;
        dest_ptr[2] = 0;
        dest_ptr[3] = 0;

        dest_ptr += 4;
    }

    // Handle remaining words
    for (remaining_words = wordLen & 0x3; remaining_words > 0; remaining_words--)
    {
        *dest_ptr++ = 0;
    }

    return SetZero_OK;
}

/**
 * @brief Reverse byte order within each 32-bit word
 * @param[in] dst Pointer to destination array
 * @param[in] src Pointer to source array
 * @param[in] wordLen Length in 32-bit words
 * @return Reverse_OK: success; others: fail
 * @note Reverses bytes within each word: ABCD -> DCBA
 * @discovery Address 0x276: Core byte reversal implementation
 */
uint32_t ReverseBytesInWord_U8(uint8_t *dst, const uint8_t *src, uint32_t wordLen)
{
    uint32_t i;
    uint8_t temp1, temp2;

    if (src == dst)
    {
        // In-place reversal - swap bytes within each word
        for (i = 0; i < wordLen; i++)
        {
            // Swap byte 0 with byte 3
            temp1 = dst[i * 4];
            dst[i * 4] = dst[i * 4 + 3];
            dst[i * 4 + 3] = temp1;

            // Swap byte 1 with byte 2
            temp2 = dst[i * 4 + 1];
            dst[i * 4 + 1] = dst[i * 4 + 2];
            dst[i * 4 + 2] = temp2;
        }
    }
    else
    {
        // Out-of-place reversal - copy with byte swapping
        for (i = 0; i < wordLen; i++)
        {
            dst[i * 4] = src[i * 4 + 3];     // Byte 3 -> Byte 0
            dst[i * 4 + 1] = src[i * 4 + 2]; // Byte 2 -> Byte 1
            dst[i * 4 + 2] = src[i * 4 + 1]; // Byte 1 -> Byte 2
            dst[i * 4 + 3] = src[i * 4];     // Byte 0 -> Byte 3
        }
    }

    return Reverse_OK;
}

/**
 * @brief Reverse byte order within 32-bit words (big-endian to little-endian)
 * @param[in] dst Pointer to destination array
 * @param[in] src Pointer to source array
 * @param[in] wordLen Length in 32-bit words
 * @return Reverse_OK: success; others: fail
 * @note dst and src can be same for in-place reversal
 * @discovery Address 0x260: Wrapper function for byte reversal
 */
uint32_t ReverseBytesInWord_U32(uint32_t *dst, const uint32_t *src, uint32_t wordLen)
{
    return ReverseBytesInWord_U8((uint8_t *)dst, (uint8_t *)src, wordLen);
}

/**
 * @brief Compare two 32-bit big number arrays
 * @param[in] a Pointer to first big number array
 * @param[in] aWordLen Length of first array in 32-bit words
 * @param[in] b Pointer to second big number array
 * @param[in] bWordLen Length of second array in 32-bit words
 * @return Cmp_EQUAL if equal, Cmp_UNEQUAL if not equal
 * @note First checks length equality, then compares word by word
 * @discovery Address 0x0D8: Big number comparison for cryptographic operations
 */
int32_t Cmp_U32(const uint32_t *a, uint32_t aWordLen, const uint32_t *b, uint32_t bWordLen)
{
    uint32_t i;

    // First check if lengths are equal
    if (aWordLen != bWordLen)
    {
        return Cmp_UNEQUAL;
    }

    // Compare each word
    for (i = 0; i < aWordLen; i++)
    {
        if (a[i] != b[i])
        {
            return Cmp_UNEQUAL;
        }
    }

    return Cmp_EQUAL;
}

/**
 * @brief Compare two 8-bit big number arrays
 * @param[in] a Pointer to first big number array
 * @param[in] aByteLen Length of first array in bytes
 * @param[in] b Pointer to second big number array
 * @param[in] bByteLen Length of second array in bytes
 * @return Cmp_EQUAL if equal, Cmp_UNEQUAL if not equal
 * @note First checks length equality, then compares byte by byte
 * @discovery Address 0x104: Byte-wise big number comparison
 */
int32_t Cmp_U8(const uint8_t *a, uint32_t aByteLen, const uint8_t *b, uint32_t bByteLen)
{
    uint32_t i;

    // First check if lengths are equal
    if (aByteLen != bByteLen)
    {
        return Cmp_UNEQUAL;
    }

    // Compare each byte
    for (i = 0; i < aByteLen; i++)
    {
        if (a[i] != b[i])
        {
            return Cmp_UNEQUAL;
        }
    }

    return Cmp_EQUAL;
}