#ifndef FILTERS_Q_INTEGER_WIDE_H
#define FILTERS_Q_INTEGER_WIDE_H

#include "q_integer_bits.h"

/**
 * @brief Unsigned scalar product or accumulator, not a stored signal format.
 */
typedef struct {
    uint64_t lo; /**< Least significant 64 bits. */
    uint64_t hi; /**< Most significant 64 bits. */
} filters_q_u128_t;

/**
 * @brief Rounded scalar quotient and whether its integer input lost any bits.
 */
typedef struct {
    uint64_t value;   /**< Nearest quotient, with halfway values rounded up. */
    unsigned inexact; /**< One if any discarded input bit was nonzero. */
} filters_q_rounded_u64_t;

/**
 * @brief Form the full unsigned 64-by-64 product using 32-bit limbs.
 *
 * @param[in] a First unsigned factor.
 * @param[in] b Second unsigned factor.
 * @return filters_q_u128_t Exact product, including both limbs.
 */
static inline filters_q_u128_t filters_q_multiply_u64(uint64_t a, uint64_t b)
{
    const uint64_t mask = UINT64_C(0xffffffff);
    uint64_t a0 = a & mask, a1 = a >> 32, b0 = b & mask, b1 = b >> 32;
    uint64_t low = a0 * b0, middle = a1 * b0 + (low >> 32), carry = middle >> 32;
    filters_q_u128_t result;
    middle = (middle & mask) + a0 * b1;
    result.lo = (middle << 32) | (low & mask);
    result.hi = a1 * b1 + carry + (middle >> 32);
    return result;
}

/**
 * @brief Count the significant bits of a two-limb unsigned scalar.
 *
 * @param[in] value Any unsigned 128-bit value.
 * @return unsigned Zero for zero, otherwise the significant bit count (1 to 128).
 */
static inline unsigned filters_q_u128_bit_length(filters_q_u128_t value)
{
    return value.hi != 0 ? 64u + filters_q_bit_length(value.hi) : filters_q_bit_length(value.lo);
}

/**
 * @brief Round an unsigned scalar divided by a power of two to uint64_t.
 *
 * @param[in] value Unsigned scalar numerator.
 * @param[in] shift Nonnegative binary divisor exponent. Values above 128
 *                  produce zero, while still reporting discarded nonzero bits.
 * @return filters_q_rounded_u64_t Nearest quotient (halfway up) and exactness.
 *
 * @pre The rounded mathematical quotient fits uint64_t. In particular, shift
 *      zero requires value.hi == 0. Callers establish this from product widths;
 *      this primitive does not clip or check a second time.
 * @note Exactness describes division before rounding, not whether the rounded
 *       result is zero. No floating-point environment is consulted.
 */
static inline filters_q_rounded_u64_t filters_q_round_u128(filters_q_u128_t value, unsigned shift)
{
    filters_q_rounded_u64_t result;
    uint64_t rounding;
    if (shift == 0) {
        result.value = value.lo;
        result.inexact = 0;
        return result;
    }
    if (shift >= 128) {
        result.value = shift == 128 ? value.hi >> 63 : 0;
        result.inexact = (value.lo != 0 || value.hi != 0);
        return result;
    }
    if (shift < 64) {
        uint64_t discarded = value.lo & ((UINT64_C(1) << shift) - 1);
        result.value = (value.lo >> shift) | (value.hi << (64 - shift));
        rounding = (value.lo >> (shift - 1)) & 1;
        result.inexact = discarded != 0;
    } else if (shift == 64) {
        result.value = value.hi;
        rounding = value.lo >> 63;
        result.inexact = value.lo != 0;
    } else {
        unsigned tail = shift - 64;
        uint64_t discarded = value.hi & ((UINT64_C(1) << tail) - 1);
        result.value = value.hi >> tail;
        rounding = (value.hi >> (tail - 1)) & 1;
        result.inexact = (discarded != 0 || value.lo != 0);
    }
    result.value += rounding;
    return result;
}

#endif
