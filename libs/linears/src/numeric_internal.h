/* Private bounded integer operations shared by numeric.c and raw-Q kernels.
 * All limbs are unsigned; signed values use explicit two's-complement encoding.
 * No function below infers or stores a dynamic binary point. */
#ifndef LINEARS_NUMERIC_INTERNAL_H
#define LINEARS_NUMERIC_INTERNAL_H

#include "linears_numeric.h"
#include <limits.h>

static inline uint64_t ln_magnitude(int64_t value) {
    return value < 0 ? (uint64_t)(-(value + 1)) + UINT64_C(1) : (uint64_t)value;
}

static inline linears_accum_t ln_negate(linears_accum_t value) {
    linears_accum_t result;
    result.low = ~value.low + UINT64_C(1);
    result.high = ~value.high + (result.low == 0);
    return result;
}

static inline int ln_negative(linears_accum_t value) {
    return (int)(value.high >> 63);
}

static inline linears_accum_t ln_absolute(linears_accum_t value) {
    return ln_negative(value) ? ln_negate(value) : value;
}

static inline int ln_compare_unsigned(linears_accum_t a, linears_accum_t b) {
    if (a.high != b.high) return a.high > b.high ? 1 : -1;
    return (a.low > b.low) - (a.low < b.low);
}

/* amount is a compile-time format shift in 1..63 at every call site. */
static inline linears_accum_t ln_shift_u64(uint64_t value, unsigned amount) {
    linears_accum_t result = {value << amount, value >> (64 - amount)};
    return result;
}

static inline linears_accum_t ln_shift_i64(int64_t value, unsigned amount) {
    linears_accum_t result = ln_shift_u64(ln_magnitude(value), amount);
    return value < 0 ? ln_negate(result) : result;
}

/* 64x64 -> 128 schoolbook multiplication in radix 2^32. The common raw-Q
 * operand path needs only one 32x32 -> 64 multiply. All partial sums fit. */
static inline linears_accum_t ln_multiply_unsigned(uint64_t a, uint64_t b) {
    linears_accum_t result;
    if ((a | b) <= UINT32_MAX) {
        result.low = (uint64_t)(uint32_t)a * (uint32_t)b;
        result.high = 0;
        return result;
    }
    uint64_t a0 = (uint32_t)a, a1 = a >> 32;
    uint64_t b0 = (uint32_t)b, b1 = b >> 32;
    uint64_t p00 = a0 * b0;
    uint64_t t = a1 * b0 + (p00 >> 32);
    uint64_t carry = t >> 32;
    t = (uint32_t)t + a0 * b1;
    result.low = (t << 32) | (uint32_t)p00;
    result.high = a1 * b1 + carry + (t >> 32);
    return result;
}

static inline linears_accum_t ln_product(int64_t a, int64_t b) {
    linears_accum_t result = ln_multiply_unsigned(ln_magnitude(a), ln_magnitude(b));
    return (a < 0) != (b < 0) ? ln_negate(result) : result;
}

/* Caller proves that the signed sum fits; raw-Q matrix bounds provide this. */
static inline linears_accum_t ln_add_unchecked(linears_accum_t a, linears_accum_t b) {
    linears_accum_t result;
    result.low = a.low + b.low;
    result.high = a.high + b.high + (result.low < a.low);
    return result;
}

/* Fixed format alignment only; caller proves the shifted signed value fits. */
static inline linears_accum_t ln_shift_accum(linears_accum_t value, unsigned amount) {
    linears_accum_t result = {value.low << amount,
                             (value.high << amount) | (value.low >> (64 - amount))};
    return result;
}

static inline linears_status_t ln_add_checked(linears_accum_t a, linears_accum_t b,
                                            linears_accum_t *out) {
    linears_accum_t result = ln_add_unchecked(a, b);
    if (ln_negative(a) == ln_negative(b) && ln_negative(result) != ln_negative(a))
        return LINEARS_ERROR_RANGE;
    *out = result;
    return LINEARS_OK;
}

/* Raw q31 dot products have |term| <= 2^62 and at most SIZE_MAX terms.
 * On 32/64-bit size_t targets their entire sum fits signed 128 bits. */
static inline void ln_add_product64(linears_accum_t *sum, int64_t product) {
    uint64_t old_low = sum->low;
    sum->low += (uint64_t)product;
    sum->high += (product < 0 ? UINT64_MAX : UINT64_C(0)) + (sum->low < old_low);
}

/* Binary restoring square root: Hacker's Delight, 2nd ed., section 11-1.
 * Returns the exact floor root and leaves the nonnegative remainder. */
static inline uint64_t ln_sqrt_floor64(uint64_t value, uint64_t *remainder) {
    uint64_t root = 0, bit = UINT64_C(1) << 62;
    while (bit > value) bit >>= 2;
    while (bit != 0) {
        if (value >= root + bit) {
            value -= root + bit;
            root = (root >> 1) + bit;
        } else root >>= 1;
        bit >>= 2;
    }
    *remainder = value;
    return root;
}

static inline uint64_t ln_sqrt_nearest64(uint64_t value) {
    uint64_t remainder;
    uint64_t root = ln_sqrt_floor64(value, &remainder);
    return root + (remainder >= root + 1);
}

#endif
