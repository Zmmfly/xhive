/**
 * @file mecc.c
 * @author Zmmfly
 * @brief Elliptic Curve Cryptography (ECC) operations native module for xmake using
 * @version 0.1
 * @date 2025-11-22
 * @copyright Copyright (c) 2025, License under GPLv3
 * 
 */
#include <xmi.h>
#include <uECC.h>
#include <sha256.h>
#include <base64.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>

// SHA256 hash context structure for uECC
typedef struct SHA256_HashContext {
    uECC_HashContext base;
    sha256_ctx_t sha_ctx;
} SHA256_HashContext;

// Hash context initialization function
static void init_sha256_hash(const struct uECC_HashContext *context) {
    SHA256_HashContext *ctx = (SHA256_HashContext *)context;
    sha256_init(&ctx->sha_ctx);
}

// Hash context update function
static void update_sha256_hash(const struct uECC_HashContext *context, const uint8_t *message, unsigned message_size) {
    SHA256_HashContext *ctx = (SHA256_HashContext *)context;
    sha256_hash(&ctx->sha_ctx, message, message_size);
}

// Hash context finalization function
static void finish_sha256_hash(const struct uECC_HashContext *context, uint8_t *hash_result) {
    SHA256_HashContext *ctx = (SHA256_HashContext *)context;
    sha256_done(&ctx->sha_ctx, hash_result);
}

#ifdef _MSC_VER
#define DYN_EXPORT    __declspec(dllexport)
#else
#define DYN_EXPORT    extern
#endif

// Helper function to get curve type from name
static uECC_Curve load_curve_from_name(const char* curve_name) {
    if (strcmp(curve_name, "secp160r1") == 0) {
        return uECC_secp160r1();
    } else if (strcmp(curve_name, "secp192r1") == 0) {
        return uECC_secp192r1();
    } else if (strcmp(curve_name, "secp224r1") == 0) {
        return uECC_secp224r1();
    } else if (strcmp(curve_name, "secp256r1") == 0 || strcmp(curve_name, "prime256v1") == 0) {
        return uECC_secp256r1();
    } else if (strcmp(curve_name, "secp256k1") == 0) {
        return uECC_secp256k1();
    }
    return NULL;
}

// Helper function to push binary string to lua stack
static void push_binary_string(lua_State* lua, const uint8_t* data, size_t len, int use_base64) {
    if (use_base64) {
        // First get the required size
        size_t encoded_len = base64_encode(data, len, NULL);
        if (encoded_len == 0) {
            // Fallback to binary data if encoding fails
            lua_pushlstring(lua, (const char*)data, len);
            return;
        }

        // Allocate buffer (+1 for null terminator)
        char* base64_output = (char*)calloc(1, encoded_len + 1);

        if (base64_output) {
            size_t actual_len = base64_encode(data, len, base64_output);
            lua_pushlstring(lua, base64_output, actual_len);
            free(base64_output);
        } else {
            // Fallback to binary data if memory allocation fails
            lua_pushlstring(lua, (const char*)data, len);
        }
    } else {
        lua_pushlstring(lua, (const char*)data, len);
    }
}

// Helper function to get binary data from lua stack
static int load_binary_data(lua_State* lua, int idx, uint8_t** data, size_t* len) {
    size_t data_len;
    const char* str_data = lua_tolstring(lua, idx, &data_len);

    if (!str_data) {
        return 0; // Not a string
    }

    // Check if data is base64 encoded
    if (is_base64(str_data, data_len) && data_len > 0) {
        // Decode base64 data
        size_t decoded_size = base64_decode(str_data, NULL);
        if (decoded_size > 0) {
            *data = (uint8_t*)malloc(decoded_size);
            if (*data) {
                size_t actual_size = base64_decode(str_data, *data);
                *len = actual_size;
                return 1;
            }
        }
    } else {
        // Direct binary data
        *data = (uint8_t*)malloc(data_len);
        if (*data) {
            memcpy(*data, str_data, data_len);
            *len = data_len;
            return 1;
        }
    }

    return 0;
}

/**
 * @brief Lists all supported elliptic curves.
 *
 * @return table: A table containing the names of supported curves.
 */
static int list_curves(lua_State* lua)
{
    // Create a new table on the stack
    lua_newtable(lua);

    int index = 1;

    // Add supported curves based on uECC_SUPPORTS_* defines and load_curve_from_name function

#if uECC_SUPPORTS_secp160r1
    lua_pushstring(lua, "secp160r1");
    lua_rawseti(lua, -2, index++);
#endif

#if uECC_SUPPORTS_secp192r1
    lua_pushstring(lua, "secp192r1");
    lua_rawseti(lua, -2, index++);
#endif

#if uECC_SUPPORTS_secp224r1
    lua_pushstring(lua, "secp224r1");
    lua_rawseti(lua, -2, index++);
#endif

#if uECC_SUPPORTS_secp256r1
    lua_pushstring(lua, "secp256r1");
    lua_rawseti(lua, -2, index++);
    // Also add alias for secp256r1
    lua_pushstring(lua, "prime256v1");
    lua_rawseti(lua, -2, index++);
#endif

#if uECC_SUPPORTS_secp256k1
    lua_pushstring(lua, "secp256k1");
    lua_rawseti(lua, -2, index++);
#endif

    return 1; // Return the table
}

/**
 * @brief Generates a new ECC keypair.
 * @param curve_name (string): The name of the elliptic curve to use (e.g., "secp256r1"). support list same with uECC,
 * @param base64 (boolean, optional): If true, returns keys in base64 encoding. Default is false(in binary string).
 * @return tuple (private_key (string), public_key (string)): The generated keypair.
 */
static int mk_keypair(lua_State* lua)
{
    // Parameter parsing
    const char* curve_name = luaL_checkstring(lua, 1);
    int   use_base64       = lua_toboolean(lua, 2);     // Default to false if not provided

    if (!curve_name) {
        lua_pushstring(lua, "Curve name is required");
        lua_error(lua);
        return 0;
    }

    // Validate curve name
    const struct uECC_Curve_t* curve = load_curve_from_name(curve_name);
    if (!curve) {
        lua_pushstring(lua, "Unsupported curve name");
        lua_error(lua);
        return 0;
    }

    // Generate keypair using uECC library
    // Allocate memory for keys
    size_t private_key_size = uECC_curve_private_key_size(curve);
    size_t public_key_size  = uECC_curve_public_key_size(curve);

    uint8_t* private_key = (uint8_t*)malloc(private_key_size);
    uint8_t* public_key  = (uint8_t*)malloc(public_key_size);

    if (!private_key || !public_key) {
        if (private_key) free(private_key);
        if (public_key) free(public_key);
        lua_pushstring(lua, "Memory allocation failed");
        lua_error(lua);
        return 0;
    }

    // Use uECC_make_key to generate keypair
    int result = uECC_make_key(public_key, private_key, curve);

    // For now, push empty strings as placeholders
    push_binary_string(lua, private_key, private_key_size, use_base64);
    push_binary_string(lua, public_key, public_key_size, use_base64);

    // Clean up
    free(private_key);
    free(public_key);

    return 2; // Return private_key and public_key
}

/**
 * @brief Derives the public key from a given private key.
 * @param curve_name (string): The name of the elliptic curve to use (e.g., "secp256r1").
 * @param private_key (string): The private key in binary string or base64 encoding, need auto detect.
 * @param base64 (boolean, optional): If true, returns the public key in base64 encoding. Default is false.
 * @return public_key (string): The derived public key.
 */
static int mk_pubkey(lua_State* lua)
{


    // Parameter parsing
    const char* curve_name      = luaL_checkstring(lua, 1);
    if (!curve_name) {
        lua_pushstring(lua, "Curve name is required");
        lua_error(lua);
        return 0;
    }

    uint8_t* private_key     = NULL;
    size_t   private_key_len = 0;
    int      use_base64      = lua_toboolean(lua, 3);  // Default to false if not provided

    // Extract private key data
    if (!load_binary_data(lua, 2, &private_key, &private_key_len)) {
        lua_pushstring(lua, "Invalid private key format");
        lua_error(lua);
        return 0;
    }

    // Validate curve name
    const struct uECC_Curve_t* curve = load_curve_from_name(curve_name);
    if (!curve) {
        free(private_key);
        lua_pushstring(lua, "Unsupported curve name");
        lua_error(lua);
        return 0;
    }

    // Validate private key size
    if (private_key_len != uECC_curve_private_key_size(curve)) {
        free(private_key);
        lua_pushstring(lua, "Invalid private key size");
        lua_error(lua);
        return 0;
    }

    // Derive public key using uECC library
    // Allocate memory for public key
    size_t public_key_size = uECC_curve_public_key_size(curve);
    uint8_t* public_key = (uint8_t*)malloc(public_key_size);

    if (!public_key) {
        free(private_key);
        lua_pushstring(lua, "Memory allocation failed");
        lua_error(lua);
        return 0;
    }

    // Use uECC_compute_public_key to derive public key
    int result = uECC_compute_public_key(private_key, public_key, curve);

    // For now, push empty string as placeholder
    push_binary_string(lua, public_key, public_key_size, use_base64);

    // Clean up
    free(private_key);
    free(public_key);

    return 1; // Return public_key
}

/**
 * @brief Signs a hash message using the provided private key.
 * @param curve_name (string): The name of the elliptic curve to use (e.g., "secp256r1").
 * @param private_key (string): The private key in binary string or base64 encoding, need auto detect.
 * @param message_hash (string): The message hash to be signed.
 * @param base64 (boolean, optional): If true, returns the signature in base64 encoding. Default is false.
 * @return signature (string): The generated signature.
 */
static int hash_sign(lua_State* lua)
{
    // Parameter parsing
    const char* curve_name = luaL_checkstring(lua, 1);

    if (!curve_name) {
        lua_pushstring(lua, "Curve name is required");
        lua_error(lua);
        return 0;
    }

    uint8_t* private_key     = NULL;
    size_t   private_key_len = 0;
    uint8_t* message_hash         = NULL;
    size_t   message_hash_len     = 0;
    int      use_base64      = lua_toboolean(lua, 4);  // Default to false if not provided

    // Extract private key data
    if (!load_binary_data(lua, 2, &private_key, &private_key_len)) {
        lua_pushstring(lua, "Invalid private key format");
        lua_error(lua);
        return 0;
    }

    // Extract message hash data
    if (!load_binary_data(lua, 3, &message_hash, &message_hash_len)) {
        free(private_key);
        lua_pushstring(lua, "Invalid message hash format");
        lua_error(lua);
        return 0;
    }

    // Validate curve name
    const struct uECC_Curve_t* curve = load_curve_from_name(curve_name);
    if (!curve) {
        free(private_key);
        free(message);
        lua_pushstring(lua, "Unsupported curve name");
        lua_error(lua);
        return 0;
    }

    // Validate private key size
    if (private_key_len != uECC_curve_private_key_size(curve)) {
        free(private_key);
        free(message);
        lua_pushstring(lua, "Invalid private key size");
        lua_error(lua);
        return 0;
    }

    // Sign hash message using uECC library
    // Allocate memory for signature (typically 2 * key size for ECDSA)
    size_t signature_size = uECC_curve_private_key_size(curve) * 2;
    uint8_t* signature = (uint8_t*)malloc(signature_size);

    if (!signature) {
        free(private_key);
        free(message);
        lua_pushstring(lua, "Memory allocation failed");
        lua_error(lua);
        return 0;
    }

    // Use uECC_sign to sign the message
    int result = uECC_sign(private_key, message_hash, message_hash_len, signature, curve);
    if (result == 0) {
        free(private_key);
        free(message_hash);
        free(signature);
        lua_pushstring(lua, "Signature failed");
        lua_error(lua);
        return 0;
    }

    // For now, push empty string as placeholder
    push_binary_string(lua, signature, signature_size, use_base64);

    // Clean up
    free(private_key);
    free(message_hash);
    free(signature);

    return 1; // Return signature
}

/**
 * @brief Sign a message hash with RFC 6979 deterministic k generation.
 *
 * @param curve_name (string): The name of the elliptic curve to use (e.g., "secp256r1").
 * @param private_key (string): The private key in binary string or base64 encoding, need auto detect.
 * @param message_hash (string): The message hash to be signed (arbitrary length supported).
 * @param base64 (boolean, optional): If true, returns the signature in base64 encoding. Default is false.
 * @return signature (string): The generated signature in binary string or base64 encoding.
 */
static int det_sign(lua_State* lua)
{
    // Parameter parsing
    const char* curve_name = luaL_checkstring(lua, 1);
    if (!curve_name) {
        lua_pushstring(lua, "Curve name is required");
        lua_error(lua);
        return 0;
    }

    uint8_t* private_key      = NULL;
    size_t   private_key_len  = 0;
    uint8_t* message_hash     = NULL;
    size_t   message_hash_len = 0;
    int      use_base64       = lua_toboolean(lua, 4);  // Default to false if not provided

    // Extract private key data
    if (!load_binary_data(lua, 2, &private_key, &private_key_len)) {
        lua_pushstring(lua, "Invalid private key format");
        lua_error(lua);
        return 0;
    }

    // Extract message hash data
    if (!load_binary_data(lua, 3, &message_hash, &message_hash_len)) {
        free(private_key);
        lua_pushstring(lua, "Invalid message hash format");
        lua_error(lua);
        return 0;
    }

    // Note: uECC_sign_deterministic accepts arbitrary hash lengths, no size restriction needed

    // Validate curve name
    const struct uECC_Curve_t* curve = load_curve_from_name(curve_name);
    if (!curve) {
        free(private_key);
        free(message_hash);
        lua_pushstring(lua, "Unsupported curve name");
        lua_error(lua);
        return 0;
    }

    // Validate private key size
    if (private_key_len != uECC_curve_private_key_size(curve)) {
        free(private_key);
        free(message_hash);
        lua_pushstring(lua, "Invalid private key size");
        lua_error(lua);
        return 0;
    }

    // Sign message with deterministic k using uECC library
    // Allocate memory for signature (typically 2 * key size for ECDSA)
    size_t signature_size = uECC_curve_private_key_size(curve) * 2;
    uint8_t* signature = (uint8_t*)malloc(signature_size);

    if (!signature) {
        free(private_key);
        free(message_hash);
        lua_pushstring(lua, "Memory allocation failed");
        lua_error(lua);
        return 0;
    }

    // Set up SHA256 hash context for uECC
    uint8_t tmp_buffer[2 * SHA256_BLOCK_BYTES + 64]; // 2*result_size + block_size
    SHA256_HashContext hash_ctx = {
        .base = {
            .init_hash   = init_sha256_hash,
            .update_hash = update_sha256_hash,
            .finish_hash = finish_sha256_hash,
            .block_size  = 64,                   // SHA256 block size
            .result_size = SHA256_BLOCK_BYTES,
            .tmp         = tmp_buffer
        }
    };

    // Use uECC_sign_deterministic to sign the message hash with RFC 6979
    int result = uECC_sign_deterministic(private_key, message_hash, message_hash_len, &hash_ctx.base, signature, curve);
    if (result == 0) {
        free(private_key);
        free(message_hash);
        free(signature);
        lua_pushstring(lua, "Deterministic signature failed");
        lua_error(lua);
        return 0;
    }

    // Push signature result
    push_binary_string(lua, signature, signature_size, use_base64);

    // Clean up
    free(private_key);
    free(message_hash);
    free(signature);

    return 1; // Return signature
}

/**
 * @brief Verifies a signature using the provided public key.
 * @param curve_name (string): The name of the elliptic curve to use (e.g., "secp256r1").
 * @param public_key (string): The public key in binary string or base64 encoding, need auto detect.
 * @param message (string): The original message hash that was signed.
 * @param signature (string): The signature to verify, auto detect binary string or base64 encoding.
 * @return is_valid (boolean): True if the signature is valid, false otherwise.
 */
static int verify_sign(lua_State* lua)
{
    // Parameter parsing
    const char* curve_name = luaL_checkstring(lua, 1);
    if (!curve_name) {
        lua_pushstring(lua, "Curve name is required");
        lua_error(lua);
        return 0;
    }

    uint8_t* public_key     = NULL;
    size_t   public_key_len = 0;
    uint8_t* message        = NULL;
    size_t   message_len    = 0;
    uint8_t* signature      = NULL;
    size_t   signature_len  = 0;

    // Extract public key data
    if (!load_binary_data(lua, 2, &public_key, &public_key_len)) {
        lua_pushstring(lua, "Invalid public key format");
        lua_error(lua);
        return 0;
    }

    // Extract message data
    if (!load_binary_data(lua, 3, &message, &message_len)) {
        free(public_key);
        lua_pushstring(lua, "Invalid message format");
        lua_error(lua);
        return 0;
    }

    // Extract signature data
    if (!load_binary_data(lua, 4, &signature, &signature_len)) {
        free(public_key);
        free(message);
        lua_pushstring(lua, "Invalid signature format");
        lua_error(lua);
        return 0;
    }

    // Validate curve name
    const struct uECC_Curve_t* curve = load_curve_from_name(curve_name);
    if (!curve) {
        free(public_key);
        free(message);
        free(signature);
        lua_pushstring(lua, "Unsupported curve name");
        lua_error(lua);
        return 0;
    }

    // Validate public key size
    if (public_key_len != uECC_curve_public_key_size(curve)) {
        free(public_key);
        free(message);
        free(signature);
        lua_pushstring(lua, "Invalid public key size");
        lua_error(lua);
        return 0;
    }

    // Verify signature using uECC library
    // For ECDSA, signature should typically be 2 * private key size
    size_t expected_signature_size = uECC_curve_private_key_size(curve) * 2;
    if (signature_len != expected_signature_size) {
        free(public_key);
        free(message);
        free(signature);
        lua_pushstring(lua, "Invalid signature size");
        lua_error(lua);
        return 0;
    }

    // Use uECC_verify to verify the signature
    int result = uECC_verify(public_key, message, message_len, signature, curve);

    // Clean up
    free(public_key);
    free(message);
    free(signature);

    // Push boolean result
    lua_pushboolean(lua, result);

    return 1; // Return is_valid
}

/**
 * @brief Verifies a public key using the provided curve name.
 *
 * @param curve_name (string): The name of the elliptic curve to use (e.g., "secp256r1").
 * @param public_key (string): The public key in binary string or base64 encoding, need auto detect.
 * @return is_valid (boolean): True if the public key is valid, false otherwise.
 */
static int verify_pubkey(lua_State* lua)
{
    // Parameter parsing
    const char* curve_name = luaL_checkstring(lua, 1);
    if (!curve_name) {
        lua_pushstring(lua, "Curve name is required");
        lua_error(lua);
        return 0;
    }

    uint8_t* public_key     = NULL;
    size_t   public_key_len = 0;

    // Extract public key data
    if (!load_binary_data(lua, 2, &public_key, &public_key_len)) {
        lua_pushstring(lua, "Invalid public key format");
        lua_error(lua);
        return 0;
    }

    // Validate curve name
    const struct uECC_Curve_t* curve = load_curve_from_name(curve_name);
    if (!curve) {
        free(public_key);
        lua_pushstring(lua, "Unsupported curve name");
        lua_error(lua);
        return 0;
    }

    // Validate public key size
    if (public_key_len != uECC_curve_public_key_size(curve)) {
        free(public_key);
        lua_pushstring(lua, "Invalid public key size");
        lua_error(lua);
        return 0;
    }

    // Verify public key using uECC library
    int result = uECC_valid_public_key(public_key, curve);

    // Clean up
    free(public_key);

    // Push boolean result
    lua_pushboolean(lua, result);

    return 1; // Return is_valid
}

DYN_EXPORT int luaopen(mecc, lua_State* lua)
{
    static const luaL_Reg funcs[] = {
        {"list_curves", list_curves},
        {"mk_keypair", mk_keypair},
        {"mk_pubkey", mk_pubkey},
        {"hash_sign", hash_sign},
        {"det_sign", det_sign},
        {"verify_sign", verify_sign},
        {"verify_pubkey", verify_pubkey},
        {NULL, NULL}
    };
    lua_newtable(lua);
    luaL_setfuncs(lua, funcs, 0);
    return 1;
}
