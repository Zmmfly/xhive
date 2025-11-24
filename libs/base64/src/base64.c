/**
 * @file base64.c
 * @author Zmmfly
 * @brief Base64 encoding and decoding functions
 * @version 0.1
 * @date 2025-11-22
 *
 * @copyright Copyright (c) 2025, License under GPLv3
 *
 */
#include <base64.h>
#include <string.h>

static const char base64_lut[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/=";

// Uncomment to use optimized range-check decoder (less memory, slightly slower)
// #define BASE64_USE_RANGE_CHECK_DECODER

#ifdef BASE64_USE_RANGE_CHECK_DECODER
// Optimized base64 character decoder using range checks + offsets
// Memory efficient: only a few comparisons, no lookup table
static inline int8_t base64_char_decode(char c)
{
    if (c >= 'A' && c <= 'Z') {
        return c - 'A';                    // 0-25
    } else if (c >= 'a' && c <= 'z') {
        return c - 'a' + 26;               // 26-51
    } else if (c >= '0' && c <= '9') {
        return c - '0' + 52;               // 52-61
    } else if (c == '+') {
        return 62;                         // 62
    } else if (c == '/') {
        return 63;                         // 63
    } else {
        return -1;                         // Invalid character
    }
}
#else
// Fast decode table method (more memory, O(1) lookup)
// Static decode table for better performance and no stack usage
static const int8_t decode_table[256] = {
    -1,-1,-1,-1, -1,-1,-1,-1, -1,-1,-1,-1, -1,-1,-1,-1,
    -1,-1,-1,-1, -1,-1,-1,-1, -1,-1,-1,-1, -1,-1,-1,-1,
    -1,-1,-1,-1, -1,-1,-1,-1, -1,-1,-1,62,  -1,-1,-1,63,
    52,53,54,55, 56,57,58,59, 60,61,-1,-1, -1,-1,-1,-1,
    -1, 0, 1, 2,  3, 4, 5, 6,  7, 8, 9,10, 11,12,13,14,
    15,16,17,18, 19,20,21,22, 23,24,25,-1, -1,-1,-1,-1,
    -1,26,27,28, 29,30,31,32, 33,34,35,36, 37,38,39,40,
    41,42,43,44, 45,46,47,48, 49,50,51,-1, -1,-1,-1,-1,
    -1,-1,-1,-1, -1,-1,-1,-1, -1,-1,-1,-1, -1,-1,-1,-1,
    -1,-1,-1,-1, -1,-1,-1,-1, -1,-1,-1,-1, -1,-1,-1,-1,
    -1,-1,-1,-1, -1,-1,-1,-1, -1,-1,-1,-1, -1,-1,-1,-1,
    -1,-1,-1,-1, -1,-1,-1,-1, -1,-1,-1,-1, -1,-1,-1,-1,
    -1,-1,-1,-1, -1,-1,-1,-1, -1,-1,-1,-1, -1,-1,-1,-1,
    -1,-1,-1,-1, -1,-1,-1,-1, -1,-1,-1,-1, -1,-1,-1,-1,
    -1,-1,-1,-1, -1,-1,-1,-1, -1,-1,-1,-1, -1,-1,-1,-1,
    -1,-1,-1,-1, -1,-1,-1,-1, -1,-1,-1,-1, -1,-1,-1,-1
};

static inline int8_t base64_char_decode(char c)
{
    return decode_table[(unsigned char)c];
}
#endif

bool is_base64(const char* str, size_t len)
{
    if (!str || len == 0) return false;

    for (size_t i = 0; i < len; i++) {
        if (memchr(base64_lut, str[i], sizeof(base64_lut)) == NULL)return false;
    }
    return true;
}

size_t base64_decode(const char* base64, void* output)
{
    if (!base64) return 0;

    size_t len = strlen(base64);
    if (len == 0) return 0;

    // Remove padding and calculate output size
    size_t padding = 0;
    if (len > 0 && base64[len - 1] == '=') padding++;
    if (len > 1 && base64[len - 2] == '=') padding++;

    size_t output_len = (len * 3) / 4 - padding;

    // If output is NULL, just return the size
    if (!output) {
        return output_len;
    }

    uint8_t* out_ptr     = (uint8_t*)output;
    uint32_t buffer      = 0;
    size_t   buffer_bits = 0;
    size_t   out_index   = 0;

    for (size_t i = 0; i < len; i++) {
        char c = base64[i];
        if (c == '=') break; // Stop at padding

        int8_t value = base64_char_decode(c);
        if (value == -1) continue; // Skip invalid characters (including newlines)

        buffer = (buffer << 6) | value;
        buffer_bits += 6;

        if (buffer_bits >= 8) {
            out_ptr[out_index++] = (uint8_t)((buffer >> (buffer_bits - 8)) & 0xFF);
            buffer_bits -= 8;
        }
    }

    return out_index;
}

size_t base64_encode(const void* data, size_t len, char* output)
{
    if (!data || len == 0) return 0;

    const uint8_t* bytes = (const uint8_t*)data;
    size_t output_len = ((len + 2) / 3) * 4;
    // If output is NULL, just return the size
    if (!output) {
        return output_len;
    }

    for (size_t i = 0; i < len; i += 3) {
        uint32_t buffer = 0;
        size_t bytes_to_process = (len - i >= 3) ? 3 : (len - i);

        // Pack bytes into buffer
        for (size_t j = 0; j < bytes_to_process; j++) {
            buffer = (buffer << 8) | bytes[i + j];
        }

        // Pad buffer to 24 bits
        for (size_t j = bytes_to_process; j < 3; j++) {
            buffer <<= 8;
        }

        // Extract 6-bit groups
        for (size_t j = 0; j < 4; j++) {
            size_t index = (i / 3) * 4 + j;
            if (index >= output_len) break;

            if (j < (bytes_to_process + 1)) {
                output[index] = base64_lut[(buffer >> (18 - j * 6)) & 0x3F];
            } else {
                output[index] = '=';
            }
        }
    }
    return output_len;
}
