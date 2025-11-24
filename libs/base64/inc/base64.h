/**
 * @file base64.h
 * @author Zmmfly
 * @brief Base64 encoding and decoding functions
 * @version 0.1
 * @date 2025-11-22
 * 
 * @copyright Copyright (c) 2025, License under GPLv3
 * 
 */
#ifndef __BASE64_H__
#define __BASE64_H__

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif // __cplusplus

/**
 * @brief Checks if a string is base64 encoded.
 *
 * @param str The string to check
 * @param len The length of the string
 * @return bool true if the string is base64 encoded, false otherwise
 */
bool is_base64(const char* str, size_t len);

/**
 * @brief Decodes a base64 encoded string.
 *
 * @param base64 Base64 encoded string
 * @param output NULL if only size is needed
 * @return size_t return decoded binary size
 */
size_t base64_decode(const char* base64, void* output);

/**
 * @brief Encodes binary data to base64 string.
 * @note The the end of the output will not set null terminator, caller need to set it manually.
 *
 * @param data Binary data to encode
 * @param len Length of binary data
 * @param output Output buffer (if NULL, only returns size), must be large enough: ((len + 2) / 3) * 4 + 1
 * @return size_t Length of encoded string (excluding null terminator)
 */
size_t base64_encode(const void* data, size_t len, char* output);

#ifdef __cplusplus
}
#endif // __cplusplus
#endif // __BASE64_H__
