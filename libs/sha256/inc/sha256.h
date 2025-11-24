/*
*   SHA-256 implementation, Mark 2
*
*   Copyright (c) 2010,2014 Ilya O. Levin, http://www.literatecode.com
*
*   Permission to use, copy, modify, and distribute this software for any
*   purpose with or without fee is hereby granted, provided that the above
*   copyright notice and this permission notice appear in all copies.
*
*   THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
*   WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
*   MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
*   ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
*   WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
*   ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
*   OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
*/
#ifndef __SHA256_H__
#define __SHA256_H__

#include <stddef.h>
#ifdef _MSC_VER
	#ifndef uint8_t
	typedef unsigned __int8 uint8_t;
	#endif

	#ifndef uint32_t
	typedef unsigned __int32 uint32_t;
	#endif
#else
#include <stdint.h>
#endif

#define SHA256_BLOCK_BYTES    32	// SHA256 block size in bytes

#ifdef __cplusplus
extern "C"
{
#endif

typedef struct sha256_ctx_t{
	uint8_t  buf[64]; /* 512 bits block buffer */
	uint32_t hash[8]; /* 256 bits hash value */
	uint32_t bits[2]; /* 64 bits bit count */
	uint32_t len;     /* Current length of the buffer */
} sha256_ctx_t;
typedef sha256_ctx_t* sha256_ctx_p;

/**
 * @brief Initialize the SHA-256 context.
 * 
 * @param ctx Pointer to the SHA-256 context.
 */
void sha256_init(sha256_ctx_p ctx);

/**
 * @brief Update the SHA-256 context with new data.
 * 
 * @param ctx Pointer to the SHA-256 context.
 * @param data Pointer to the input data.
 * @param len Length of the input data.
 */
void sha256_hash(sha256_ctx_p ctx, const void *data, size_t len);

/**
 * @brief Finalize the SHA-256 context and produce the hash.
 * 
 * @param ctx Pointer to the SHA-256 context.
 * @param hash Pointer to the output hash buffer (must be at least 32 bytes).
 */
void sha256_done(sha256_ctx_p ctx, void *hash);

/**
 * @brief Compute the SHA-256 hash of the input data.
 * 
 * @param data Pointer to the input data.
 * @param len Length of the input data.
 * @param hash Pointer to the output hash buffer (must be at least 32 bytes).
 */
void sha256(const void *data, size_t len, void *hash);

#ifdef __cplusplus
}
#endif

#endif // __SHA256_H__
