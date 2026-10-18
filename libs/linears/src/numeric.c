#include "linears_numeric.h"
#include "numeric_internal.h"

struct alignment_i16 { char padding; int16_t value; };
struct alignment_i32 { char padding; int32_t value; };
struct alignment_fixed { char padding; linears_fixed_t value; };

static linears_status_t signed_result(uint64_t magnitude, int negative, int64_t *out) {
    uint64_t limit = negative ? (UINT64_C(1) << 63) : (uint64_t)INT64_MAX;
    if (magnitude > limit) return LINEARS_ERROR_RANGE;
    *out = negative ? (magnitude == (UINT64_C(1) << 63) ? INT64_MIN : -(int64_t)magnitude)
                    : (int64_t)magnitude;
    return LINEARS_OK;
}

/* Discard only the fixed-format fractional bits. No signed right shift is
 * used. The high-limb check prevents truncating any integer result bits. */
static linears_status_t round_shift(linears_accum_t magnitude, unsigned shift,
                                    int negative, int64_t *out) {
    if ((magnitude.high >> shift) != 0) return LINEARS_ERROR_RANGE;
    uint64_t quotient = (magnitude.low >> shift) | (magnitude.high << (64 - shift));
    uint64_t remainder = magnitude.low & ((UINT64_C(1) << shift) - 1);
    if (remainder >= (UINT64_C(1) << (shift - 1))) {
        if (quotient == UINT64_MAX) return LINEARS_ERROR_RANGE;
        ++quotient;
    }
    return signed_result(quotient, negative, out);
}

linears_status_t linears_fixed_from_int(int64_t integer, linears_fixed_t *out) {
    if (out == NULL) return LINEARS_ERROR_ARGUMENT;
    if (integer < -INT64_C(4294967296) || integer > INT64_C(4294967295))
        return LINEARS_ERROR_RANGE;
    *out = integer * LINEARS_FIXED_ONE;
    return LINEARS_OK;
}

linears_status_t linears_fixed_add(linears_fixed_t a, linears_fixed_t b, linears_fixed_t *out) {
    if (out == NULL) return LINEARS_ERROR_ARGUMENT;
    if ((b > 0 && a > INT64_MAX - b) || (b < 0 && a < INT64_MIN - b))
        return LINEARS_ERROR_RANGE;
    *out = a + b;
    return LINEARS_OK;
}

linears_status_t linears_fixed_sub(linears_fixed_t a, linears_fixed_t b, linears_fixed_t *out) {
    if (out == NULL) return LINEARS_ERROR_ARGUMENT;
    if ((b > 0 && a < INT64_MIN + b) || (b < 0 && a > INT64_MAX + b))
        return LINEARS_ERROR_RANGE;
    *out = a - b;
    return LINEARS_OK;
}

linears_status_t linears_fixed_mul(linears_fixed_t a, linears_fixed_t b, linears_fixed_t *out) {
    if (out == NULL) return LINEARS_ERROR_ARGUMENT;
    return round_shift(ln_multiply_unsigned(ln_magnitude(a), ln_magnitude(b)), 31,
                       (a < 0) != (b < 0), out);
}

/* Restoring two-limb / one-limb division. high < divisor guarantees a 64-bit
 * quotient. Keeping the carry bit separately avoids overflow in 2*remainder.
 * Common products have high == 0 and use the ordinary uint64_t fast path. */
static uint64_t divide_magnitude(uint64_t high, uint64_t low, uint64_t divisor,
                                 uint64_t *remainder) {
    if (high == 0) {
        *remainder = low % divisor;
        return low / divisor;
    }
    uint64_t quotient = 0, rest = high;
    for (unsigned i = 64; i != 0; --i) {
        uint64_t carry = rest >> 63;
        rest = (rest << 1) | ((low >> (i - 1)) & 1);
        quotient <<= 1;
        if (carry != 0 || rest >= divisor) {
            rest -= divisor;
            quotient |= 1;
        }
    }
    *remainder = rest;
    return quotient;
}

static linears_status_t rounded_division(linears_accum_t numerator, uint64_t divisor,
                                        int negative, linears_fixed_t *out) {
    if (numerator.high >= divisor) return LINEARS_ERROR_RANGE;
    uint64_t remainder;
    uint64_t quotient = divide_magnitude(numerator.high, numerator.low, divisor, &remainder);
    /* remainder*2 could overflow for wide divisors. */
    if (remainder >= divisor - remainder) {
        if (quotient == UINT64_MAX) return LINEARS_ERROR_RANGE;
        ++quotient;
    }
    return signed_result(quotient, negative, out);
}

linears_status_t linears_fixed_div(linears_fixed_t a, linears_fixed_t b, linears_fixed_t *out) {
    if (out == NULL) return LINEARS_ERROR_ARGUMENT;
    if (b == 0) return LINEARS_ERROR_DOMAIN;
    return rounded_division(ln_shift_u64(ln_magnitude(a), 31), ln_magnitude(b),
                            (a < 0) != (b < 0), out);
}

linears_status_t linears_fixed_muldiv(linears_fixed_t a, linears_fixed_t b,
                                    linears_fixed_t divisor, linears_fixed_t *out) {
    if (out == NULL) return LINEARS_ERROR_ARGUMENT;
    if (divisor == 0) return LINEARS_ERROR_DOMAIN;
    return rounded_division(ln_multiply_unsigned(ln_magnitude(a), ln_magnitude(b)),
                            ln_magnitude(divisor), (a < 0) ^ (b < 0) ^ (divisor < 0), out);
}

linears_status_t linears_fixed_sqrt(linears_fixed_t value, linears_fixed_t *out) {
    if (out == NULL) return LINEARS_ERROR_ARGUMENT;
    if (value < 0) return LINEARS_ERROR_DOMAIN;
    linears_accum_t radicand = ln_shift_u64((uint64_t)value, 31);
    if (radicand.high == 0) {
        *out = (int64_t)ln_sqrt_nearest64(radicand.low);
        return LINEARS_OK;
    }
    /* A positive int64_t F31 input gives at most 94 radicand bits and a
     * 47-bit floor root. Base-4 restoring digits need only uint64_t remainder
     * and root registers; 48 fixed iterations also cover leading zero bits. */
    uint64_t root = 0, remainder = 0;
    for (unsigned pair = 48; pair != 0; --pair) {
        unsigned position = 2 * (pair - 1);
        uint64_t digit = position >= 64 ? (radicand.high >> (position - 64)) & 3
                                        : (radicand.low >> position) & 3;
        remainder = (remainder << 2) | digit;
        root <<= 1;
        uint64_t trial = (root << 1) | 1;
        if (remainder >= trial) {
            remainder -= trial;
            ++root;
        }
    }
    *out = (int64_t)(root + (remainder >= root + 1));
    return LINEARS_OK;
}

linears_status_t linears_fixed_neg(linears_fixed_t value, linears_fixed_t *out) {
    if (out == NULL) return LINEARS_ERROR_ARGUMENT;
    if (value == INT64_MIN) return LINEARS_ERROR_RANGE;
    *out = -value;
    return LINEARS_OK;
}

linears_status_t linears_fixed_abs(linears_fixed_t value, linears_fixed_t *out) {
    if (out == NULL) return LINEARS_ERROR_ARGUMENT;
    if (value == INT64_MIN) return LINEARS_ERROR_RANGE;
    *out = value < 0 ? -value : value;
    return LINEARS_OK;
}

linears_status_t linears_fixed_to_q15(linears_fixed_t value, linears_q15_t *out) {
    if (out == NULL) return LINEARS_ERROR_ARGUMENT;
    if (value < -LINEARS_FIXED_ONE || value > LINEARS_FIXED_ONE) return LINEARS_ERROR_RANGE;
    uint64_t magnitude = ln_magnitude(value);
    uint64_t raw = (magnitude >> 16) + ((magnitude & UINT64_C(65535)) >= UINT64_C(32768));
    if (value >= 0 && raw == UINT64_C(32768)) raw = INT16_MAX;
    *out = (int16_t)(value < 0 ? -(int32_t)raw : (int32_t)raw);
    return LINEARS_OK;
}

linears_status_t linears_fixed_to_q31(linears_fixed_t value, linears_q31_t *out) {
    if (out == NULL) return LINEARS_ERROR_ARGUMENT;
    if (value < -LINEARS_FIXED_ONE || value > LINEARS_FIXED_ONE) return LINEARS_ERROR_RANGE;
    *out = value == LINEARS_FIXED_ONE ? INT32_MAX : (int32_t)value;
    return LINEARS_OK;
}

linears_status_t linears_fixed_mac(linears_accum_t *accumulator,
                                 linears_fixed_t a, linears_fixed_t b) {
    if (accumulator == NULL) return LINEARS_ERROR_ARGUMENT;
    return ln_add_checked(*accumulator, ln_product(a, b), accumulator);
}

linears_status_t linears_fixed_msu(linears_accum_t *accumulator,
                                 linears_fixed_t a, linears_fixed_t b) {
    if (accumulator == NULL) return LINEARS_ERROR_ARGUMENT;
    return ln_add_checked(*accumulator, ln_negate(ln_product(a, b)), accumulator);
}

linears_status_t linears_accum_to_fixed(linears_accum_t value, linears_fixed_t *out) {
    if (out == NULL) return LINEARS_ERROR_ARGUMENT;
    return round_shift(ln_absolute(value), 31, ln_negative(value), out);
}

/* magnitude is already checked to be <= 2^62 by the calling boundary. */
static int64_t quantize_unit(uint64_t magnitude, int negative, unsigned bits) {
    unsigned shift = 62 - bits;
    uint64_t raw = (magnitude >> shift) +
        ((magnitude & ((UINT64_C(1) << shift) - 1)) >= (UINT64_C(1) << (shift - 1)));
    if (!negative && raw == (UINT64_C(1) << bits)) --raw;
    return negative ? -(int64_t)raw : (int64_t)raw;
}

static linears_status_t quantize_accumulator(linears_accum_t value, unsigned bits, int64_t *out) {
    linears_accum_t magnitude = ln_absolute(value);
    if (magnitude.high != 0 || magnitude.low > (UINT64_C(1) << 62))
        return LINEARS_ERROR_RANGE;
    *out = quantize_unit(magnitude.low, ln_negative(value), bits);
    return LINEARS_OK;
}

linears_status_t linears_accum_to_q15(linears_accum_t value, linears_q15_t *out) {
    if (out == NULL) return LINEARS_ERROR_ARGUMENT;
    int64_t raw;
    linears_status_t status = quantize_accumulator(value, 15, &raw);
    if (status == LINEARS_OK) *out = (int16_t)raw;
    return status;
}

linears_status_t linears_accum_to_q31(linears_accum_t value, linears_q31_t *out) {
    if (out == NULL) return LINEARS_ERROR_ARGUMENT;
    int64_t raw;
    linears_status_t status = quantize_accumulator(value, 31, &raw);
    if (status == LINEARS_OK) *out = (int32_t)raw;
    return status;
}

static linears_status_t quantize_mean(linears_accum_t sum, uint64_t count,
                                     unsigned bits, int64_t *out) {
    if (count == 0) return LINEARS_ERROR_DOMAIN;
    linears_accum_t magnitude = ln_absolute(sum);
    linears_accum_t bound = ln_shift_u64(count, 62);
    if (ln_compare_unsigned(magnitude, bound) > 0) return LINEARS_ERROR_RANGE;
    /* The range bound proves magnitude.high < count and quotient <= 2^62.
     * Floor the division, not round it: discarding its remainder (<1 raw unit)
     * cannot cross the subsequent power-of-two halfway threshold, since that
     * threshold is an integer (the final shift is 31 or 47). Exact halfway
     * already rounds away from zero, so this is one correctly rounded result. */
    uint64_t remainder;
    uint64_t quotient = divide_magnitude(magnitude.high, magnitude.low, count, &remainder);
    *out = quantize_unit(quotient, ln_negative(sum), bits);
    return LINEARS_OK;
}

linears_status_t linears_accum_mean_q15(linears_accum_t sum, uint64_t count, linears_q15_t *out) {
    if (out == NULL) return LINEARS_ERROR_ARGUMENT;
    int64_t raw;
    linears_status_t status = quantize_mean(sum, count, 15, &raw);
    if (status == LINEARS_OK) *out = (int16_t)raw;
    return status;
}

linears_status_t linears_accum_mean_q31(linears_accum_t sum, uint64_t count, linears_q31_t *out) {
    if (out == NULL) return LINEARS_ERROR_ARGUMENT;
    int64_t raw;
    linears_status_t status = quantize_mean(sum, count, 31, &raw);
    if (status == LINEARS_OK) *out = (int32_t)raw;
    return status;
}

static int overlaps(const void *a, size_t asize, const void *b, size_t bsize) {
    if (asize == 0 || bsize == 0) return 0;
    uintptr_t x = (uintptr_t)a, y = (uintptr_t)b;
    return x <= y ? y - x < asize : x - y < bsize;
}

static linears_status_t export_values(const linears_fixed_t *values, size_t count,
                                     void *out, unsigned bits) {
    size_t width = bits == 15 ? sizeof(int16_t) : sizeof(int32_t);
    size_t alignment = bits == 15 ? offsetof(struct alignment_i16, value)
                                 : offsetof(struct alignment_i32, value);
    if (count != 0 && (values == NULL || out == NULL ||
        (uintptr_t)values % offsetof(struct alignment_fixed, value) != 0 ||
        (uintptr_t)out % alignment != 0)) return LINEARS_ERROR_ARGUMENT;
    if (count > SIZE_MAX / sizeof(*values) || count > SIZE_MAX / width)
        return LINEARS_ERROR_SIZE;
    if (count * sizeof(*values) > UINTPTR_MAX - (uintptr_t)values ||
        count * width > UINTPTR_MAX - (uintptr_t)out) return LINEARS_ERROR_SIZE;
    if (overlaps(values, count * sizeof(*values), out, count * width)) return LINEARS_ERROR_ALIAS;
    for (size_t i = 0; i < count; ++i)
        if (values[i] < -LINEARS_FIXED_ONE || values[i] > LINEARS_FIXED_ONE)
            return LINEARS_ERROR_RANGE;
    for (size_t i = 0; i < count; ++i) {
        if (bits == 15) (void)linears_fixed_to_q15(values[i], &((int16_t *)out)[i]);
        else (void)linears_fixed_to_q31(values[i], &((int32_t *)out)[i]);
    }
    return LINEARS_OK;
}

linears_status_t linears_fixed_export_q15(const linears_fixed_t *values, size_t count,
                                        linears_q15_t *out) {
    return export_values(values, count, out, 15);
}

linears_status_t linears_fixed_export_q31(const linears_fixed_t *values, size_t count,
                                        linears_q31_t *out) {
    return export_values(values, count, out, 31);
}
