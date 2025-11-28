#ifndef __XHIVE_BITMAP_H__
#define __XHIVE_BITMAP_H__

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdio.h>
#include <stdlib.h> 
#include <string.h>

#ifndef __INLINE
    #ifdef __GNUC__
        #define __INLINE static inline __attribute__((always_inline))
    #elif defined(_MSC_VER)
        #define __INLINE static __forceinline
    #else
        #define __INLINE static inline
    #endif
#endif /* __INLINE */

#ifndef NDEBUG
    #define bitmap_assert(expr)                                             \
        do {                                                                \
            if (!(expr)) {                                                  \
                printf("Bitmap assertion failed: %s, file %s, line %d\n",   \
                        #expr, __FILE__, __LINE__);                         \
                abort();                                                    \
            }                                                               \
        } while (0)
#else
    #define bitmap_assert(expr) ((void)0)
#endif /* NDEBUG */

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

typedef uint64_t bitdat_t;

#define BITDAT_BITS (sizeof(bitdat_t) * 8)

#define BITDAT_MAX  ((bitdat_t)(~((bitdat_t)0)))

/**
 * @brief Bitmap structure
 * 
 */
typedef struct xh_bitmap_t
{
    bitdat_t* data;  // pointer to bitmap data
    uint64_t  bits;  // number of bits
} xh_bitmap_t;
typedef xh_bitmap_t* xh_bitmap_p;

/**
 * @brief Create a bitmap
 * 
 * @param bits 
 * @return xh_bitmap_p 
 */
__INLINE xh_bitmap_p xh_bitmap_create(uint64_t bits)
{
    xh_bitmap_p bitmap = (xh_bitmap_p)malloc(sizeof(xh_bitmap_t));
    if (bitmap)
    {
        bitmap->data = (bitdat_t*)calloc((bits + BITDAT_BITS - 1) / BITDAT_BITS, sizeof(bitdat_t));
        bitmap->bits = bits;
        if (!bitmap->data)
        {
            free(bitmap);
            bitmap = NULL;
        }
    }
    return bitmap;
}

/**
 * @brief Destroy a bitmap
 * 
 * @param bitmap  
 */
__INLINE void xh_bitmap_destroy(xh_bitmap_p bitmap)
{
    if (bitmap)
    {
        free(bitmap->data);
        free(bitmap);
    }
}

/**
 * @brief Set all bits in the bitmap to a value
 * 
 * @param bitmap 
 * @param value true to set, false to clear
 */
__INLINE void xh_bitmap_all_to(xh_bitmap_p bitmap, bool value)
{
    bitmap_assert(bitmap != NULL);
    uint64_t qty = bitmap->bits / BITDAT_BITS;
    uint64_t rem = bitmap->bits % BITDAT_BITS;
    for (uint64_t i = 0; i < qty; i++)
    {
        bitmap->data[i] = value ? BITDAT_MAX : 0;
    }
    if (rem)
    {
        bitmap->data[qty] = value ? ((~((bitdat_t)0) << (BITDAT_BITS - rem)) >> (BITDAT_BITS - rem)) : (bitdat_t)0;
    }
}

/**
 * @brief Set a bit in the bitmap
 * 
 * @param bitmap 
 * @param index 
 */
__INLINE void xh_bitmap_set(xh_bitmap_p bitmap, size_t index)
{
    bitmap_assert(bitmap != NULL);
    bitmap_assert(index < bitmap->bits);
    bitmap->data[index / BITDAT_BITS] |= ((bitdat_t)1 << (index % BITDAT_BITS));
}

/**
 * @brief Clear a bit in the bitmap
 * 
 * @param bitmap 
 * @param index 
 */
__INLINE void xh_bitmap_clear(xh_bitmap_p bitmap, size_t index)
{
    bitmap_assert(bitmap != NULL);
    bitmap_assert(index < bitmap->bits);
    bitmap->data[index / BITDAT_BITS] &= ~((bitdat_t)1 << (index % BITDAT_BITS));
}

/**
 * @brief Test a bit in the bitmap
 * 
 * @param bitmap 
 * @param index 
 * @return true if set
 * @return false if not set
 */
__INLINE bool xh_bitmap_test(xh_bitmap_p bitmap, size_t index)
{
    bitmap_assert(bitmap != NULL);
    bitmap_assert(index < bitmap->bits);
    return (bitmap->data[index / BITDAT_BITS] & ((bitdat_t)1 << (index % BITDAT_BITS))) != 0;
}

#if defined(__GNUC__) || defined(__clang__)  // GCC/Clang
    #define POPCOUNT64 __builtin_popcountll
#else
    inline uint64_t popcount_swar64(uint64_t x) {
        x = x - ((x >> 1) & 0x5555555555555555);
        x = (x & 0x3333333333333333) + ((x >> 2) & 0x3333333333333333);
        x = (x + (x >> 4)) & 0x0F0F0F0F0F0F0F0F;
        return (x * 0x0101010101010101) >> 56;
    }
    #define POPCOUNT64 popcount_swar64
#endif

/**
 * @brief Count the number of set bits in the bitmap
 * 
 * @param bitmap 
 * @return uint64_t 
 */
__INLINE uint64_t xh_bitmap_count_set_bits(xh_bitmap_p bitmap)
{
    bitmap_assert(bitmap != NULL);
    uint64_t count = 0;
    uint64_t len   = (bitmap->bits + BITDAT_BITS - 1) / BITDAT_BITS;
    for (uint64_t i = 0; i < len; i++)
    {
        uint64_t val = bitmap->data[i];
        count += POPCOUNT64(val);
    }
    return count;
}

/**
 * @brief Get the index of the leftmost set bit starting from start_index
 * @note MSB at left side, LSB at right side, index decrease from start_index
 * 
 * @param bitmap 
 * @param start_index Start index at left, include start_index, will auto decrease to 0
 * @return uint64_t return BITDAT_MAX if not found
 */
__INLINE uint64_t xh_bitmap_index_of_left_set(xh_bitmap_p bitmap, uint64_t start_index)
{
    bitmap_assert(bitmap != NULL);
    bitmap_assert(start_index < bitmap->bits);

    uint64_t start_qty = start_index / BITDAT_BITS;
    uint64_t start_rem = start_index % BITDAT_BITS;

    for (uint64_t i = start_qty; i != BITDAT_MAX; i--)
    {
        // skip unset data
        if (bitmap->data[i] == 0) continue;

        // bits from j down to 0
        for (uint64_t j = (i == start_qty ? start_rem : BITDAT_BITS-1); j != BITDAT_MAX; j--)
        {
            // skip unset bits
            if (!(bitmap->data[i] & (1U << j))) continue;
            return (i * BITDAT_BITS) | j;
        }
    }
    return BITDAT_MAX;
}

/**
 * @brief Get the index of the leftmost bit starting from start_index with specified bit_value
 * 
 * @param bitmap 
 * @param start_index 
 * @param bit_value true for 1 bit, false for 0 bit
 * @return uint64_t return BITDAT_MAX if not found
 */
__INLINE uint64_t xh_bitmap_index_of_left(xh_bitmap_p bitmap, uint64_t start_index, bool bit_value)
{
    bitmap_assert(bitmap != NULL);
    bitmap_assert(start_index < bitmap->bits);

    uint64_t start_qty = start_index / BITDAT_BITS;
    uint64_t start_rem = start_index % BITDAT_BITS;

    for (uint64_t i = start_qty; i != BITDAT_MAX; i--)
    {
        if ( (bit_value && bitmap->data[i] == 0) || (!bit_value && bitmap->data[i] != 0) ) continue;
        // Check bits from j down to 0
        for (uint64_t j = (i == start_qty ? start_rem : BITDAT_BITS-1); j != BITDAT_MAX; j--)
        {
            // clang-format off
            if (   !(  bit_value &&  (bitmap->data[i] & (1U << j)) ) 
                && !( !bit_value && !(bitmap->data[i] & (1U << j)) ) )
            {
                continue;
            }
            return (i * BITDAT_BITS) | j;
            // clang-format on
        }
    }
    return BITDAT_MAX;
}

/**
 * @brief Get the index of the rightmost set bit starting from start_index
 * @note MSB at left side, LSB at right side, index increase from start_index
 * 
 * @param bitmap 
 * @param start_index Start index at right, will auto increase to bits-1
 * @return uint64_t return BITDAT_MAX if not found
 */
__INLINE uint64_t xh_bitmap_index_of_right_set(xh_bitmap_p bitmap, uint64_t start_index)
{
    bitmap_assert(bitmap != NULL);
    bitmap_assert(start_index < bitmap->bits);
    uint64_t qty       = bitmap->bits / BITDAT_BITS;
    uint64_t rem       = bitmap->bits % BITDAT_BITS;
    uint64_t start_qty = start_index / BITDAT_BITS;
    uint64_t start_rem = start_index % BITDAT_BITS;

    for (uint64_t i = start_qty; i < qty + (rem ? 1 : 0); i++)
    {
        // Skip empty words
        if (bitmap->data[i] == 0) continue;

        for (uint64_t j = (i == start_qty ? start_rem : 0); j < BITDAT_BITS; j++)
        {
            // Skip unset bits
            if ( ( bitmap->data[i] & ((bitdat_t)1 << j) ) == 0 ) continue;

            uint64_t index = (i * BITDAT_BITS) | j;
            if (index < bitmap->bits) return index;
            else return BITDAT_MAX;
        }
    }
    return BITDAT_MAX;
}

#ifdef __cplusplus
}
#endif /* __cplusplus */
#endif /* __XHIVE_BITMAP_H__ */
