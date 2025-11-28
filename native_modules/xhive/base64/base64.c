/**
 * @file base64.c
 * @author Zmmfly
 * @brief Base64 encoding and decoding native module for xmake using
 * @version 0.1
 * @date 2025-11-22
 * @copyright Copyright (c) 2025, License under GPLv3
 * 
 */
#include <xmi.h>
#include <base64.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <ctype.h>

#ifdef _MSC_VER
#define DYN_EXPORT    __declspec(dllexport)
#else
#define DYN_EXPORT    extern
#endif

/**
 * @brief Validates if the input is a proper string (not NULL, not empty)
 * @param str The string to validate
 * @param len Length of the string
 * @return 1 if valid string, 0 otherwise
 */
static int is_valid_string(const char* str, size_t len) {
    return (str != NULL && len > 0);
}

/**
 * @brief Validates if a string contains only valid Base64 characters
 * @param str The string to validate
 * @param len Length of the string
 * @return 1 if valid Base64 string, 0 otherwise
 */
static int is_valid_base64_string(const char* str, size_t len) {
    if (!is_valid_string(str, len)) {
        return 0;
    }
    
    // Base64 strings should have length divisible by 4 (except for URL-safe variants)
    // Allow for padding characters '=' at the end
    size_t i;
    for (i = 0; i < len; i++) {
        char c = str[i];
        
        // Check for valid Base64 characters
        if ((c >= 'A' && c <= 'Z') ||
            (c >= 'a' && c <= 'z') ||
            (c >= '0' && c <= '9') ||
            c == '+' || c == '/' || c == '-' || c == '_') {
            continue;
        }
        
        // Allow padding '=' only at the end
        if (c == '=') {
            // Check that remaining characters are all '='
            for (size_t j = i; j < len; j++) {
                if (str[j] != '=') {
                    return 0;
                }
            }
            break;
        }
        
        // Invalid character found
        return 0;
    }
    
    // Check if length is reasonable (not too short)
    // Minimum valid Base64 string length is 4 (representing 3 bytes)
    if (len < 4) {
        return 0;
    }
    
    // Check if padding is correct (max 2 '=' characters)
    size_t padding_count = 0;
    for (i = len - 1; i >= 0 && str[i] == '='; i--) {
        padding_count++;
    }
    
    if (padding_count > 2) {
        return 0;
    }
    
    // For standard Base64, length should be divisible by 4
    // But we'll be lenient to handle different Base64 variants
    return 1;
}

/**
 * @brief Encodes a string to Base64 format
 * @param input_str (string): The input string to encode (must be string type)
 * @return encoded_str (string): The Base64 encoded string
 */
static int encode(lua_State* lua)
{
    // Get the input string
    size_t input_len;
    const char* input_str = luaL_checklstring(lua, 1, &input_len);
    
    // Validate that input is a proper string
    if (!is_valid_string(input_str, input_len)) {
        lua_pushstring(lua, "Input must be a non-empty string");
        lua_error(lua);
        return 0;
    }
    
    // Calculate the required buffer size for Base64 encoding
    // Base64 encoding expands data by ~33%: output_size = ((input_size + 2) / 3) * 4
    size_t encoded_len = base64_encode((const uint8_t*)input_str, input_len, NULL);
    
    if (encoded_len == 0) {
        lua_pushstring(lua, "Failed to calculate encoded size");
        lua_error(lua);
        return 0;
    }
    
    // Allocate buffer for encoded data (+1 for null terminator)
    char* encoded_str = (char*)malloc(encoded_len + 1);
    if (!encoded_str) {
        lua_pushstring(lua, "Memory allocation failed");
        lua_error(lua);
        return 0;
    }
    
    // Perform the actual encoding
    size_t actual_len = base64_encode((const uint8_t*)input_str, input_len, encoded_str);
    
    if (actual_len == 0) {
        free(encoded_str);
        lua_pushstring(lua, "Base64 encoding failed");
        lua_error(lua);
        return 0;
    }
    
    // Ensure null termination
    encoded_str[actual_len] = '\0';
    
    // Push the encoded string to Lua stack
    lua_pushlstring(lua, encoded_str, actual_len);
    
    // Clean up
    free(encoded_str);
    
    return 1; // Return the encoded string
}

/**
 * @brief Decodes a Base64 encoded string back to original string
 * @param base64_str (string): The Base64 encoded string to decode
 * @return decoded_str (string): The decoded string
 */
static int decode(lua_State* lua)
{
    // Get the Base64 string
    size_t base64_len;
    const char* base64_str = luaL_checklstring(lua, 1, &base64_len);
    
    // Validate that input is a proper Base64 string
    if (!is_valid_base64_string(base64_str, base64_len)) {
        lua_pushstring(lua, "Input must be a valid Base64 encoded string");
        lua_error(lua);
        return 0;
    }
    
    // Calculate the required buffer size for decoded data
    size_t decoded_len = base64_decode(base64_str, NULL);
    
    if (decoded_len == 0) {
        lua_pushstring(lua, "Invalid Base64 format or decode failed");
        lua_error(lua);
        return 0;
    }
    
    // Allocate buffer for decoded data (+1 for null terminator)
    char* decoded_str = (char*)malloc(decoded_len + 1);
    if (!decoded_str) {
        lua_pushstring(lua, "Memory allocation failed");
        lua_error(lua);
        return 0;
    }
    
    // Perform the actual decoding
    size_t actual_len = base64_decode(base64_str, (uint8_t*)decoded_str);
    
    if (actual_len == 0) {
        free(decoded_str);
        lua_pushstring(lua, "Base64 decoding failed");
        lua_error(lua);
        return 0;
    }
    
    // Ensure null termination
    decoded_str[actual_len] = '\0';
    
    // Push the decoded string to Lua stack
    lua_pushlstring(lua, decoded_str, actual_len);
    
    // Clean up
    free(decoded_str);
    
    return 1; // Return the decoded string
}

/**
 * @brief Validates if a string is in valid Base64 format
 * @param input_str (string): The string to validate
 * @return is_valid (boolean): True if valid Base64, false otherwise
 */
static int is_valid(lua_State* lua)
{
    // Get the string to validate
    size_t input_len;
    const char* input_str = luaL_checklstring(lua, 1, &input_len);
    
    // Check if it's a valid Base64 string
    int valid = is_valid_base64_string(input_str, input_len);
    
    // Push the boolean result
    lua_pushboolean(lua, valid);
    
    return 1; // Return the boolean result
}

DYN_EXPORT int luaopen(base64, lua_State* lua)
{
    static const luaL_Reg funcs[] = {
        {"encode", encode},
        {"decode", decode},
        {"is_valid", is_valid},
        {NULL, NULL}
    };
    lua_newtable(lua);
    luaL_setfuncs(lua, funcs, 0);
    return 1;
}
