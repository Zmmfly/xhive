#include "linears_numeric.h"

#include <limits.h>

/* C99 alignment probes avoid a dependency on C11 _Alignof. */
struct alignment_i16 { char padding; int16_t value; };
struct alignment_i32 { char padding; int32_t value; };
struct alignment_int { char padding; int value; };
struct alignment_fixed { char padding; linears_fixed_t value; };

/* Work on unsigned magnitudes: signed negative shifts and INT64_MIN negation
 * are deliberately never used. Every variable shift is bounded before use. */
static uint64_t magnitude64(int64_t value) {
    return value < 0 ? (uint64_t)(-(value + 1)) + UINT64_C(1) : (uint64_t)value;
}

static unsigned bit_length(uint64_t value) {
    unsigned bits = 0;
    while (value != 0) {
        value >>= 1;
        ++bits;
    }
    return bits;
}

static uint64_t rounded_right(uint64_t value, unsigned shift) {
    if (shift == 0) return value;
    if (shift > 64) return 0;
    if (shift == 64) return value >= (UINT64_C(1) << 63);
    return (value >> shift) + ((value & ((UINT64_C(1) << shift) - 1)) >=
                              (UINT64_C(1) << (shift - 1)));
}

/* Normalize the exact signed integer times 2^power, then round only once. */
static linears_status_t pack(int64_t integer, int64_t power, linears_fixed_t *out) {
    linears_fixed_t result = {0, 0};
    uint64_t magnitude = magnitude64(integer);
    if (magnitude != 0) {
        unsigned bits = bit_length(magnitude);
        int64_t exponent = power + bits;
        magnitude = bits > 31 ? rounded_right(magnitude, bits - 31)
                              : magnitude << (31 - bits);
        if (magnitude == (UINT64_C(1) << 31)) {
            magnitude >>= 1;
            ++exponent;
        }
        if (exponent < LINEARS_EXPONENT_MIN || exponent > LINEARS_EXPONENT_MAX)
            return LINEARS_ERROR_RANGE;
        result.mantissa = integer < 0 ? -(int32_t)magnitude : (int32_t)magnitude;
        result.exponent = (int)exponent;
    }
    *out = result;
    return LINEARS_OK;
}

int linears_fixed_valid(linears_fixed_t value) {
    if (value.mantissa == 0) return value.exponent == 0;
    return value.exponent >= LINEARS_EXPONENT_MIN &&
           value.exponent <= LINEARS_EXPONENT_MAX &&
           value.mantissa != INT32_MIN &&
           magnitude64(value.mantissa) >= (UINT64_C(1) << 30);
}

linears_status_t linears_fixed_from_i64(int64_t mantissa, int fractional_bits,
                                      int exponent, linears_fixed_t *out) {
    if (out == NULL || fractional_bits < 0 || fractional_bits > 63)
        return LINEARS_ERROR_ARGUMENT;
    if (exponent < LINEARS_EXPONENT_MIN || exponent > LINEARS_EXPONENT_MAX)
        return LINEARS_ERROR_RANGE;
    return pack(mantissa, (int64_t)exponent - fractional_bits, out);
}

linears_status_t linears_fixed_from_q15(linears_q15_t value, linears_fixed_t *out) {
    return linears_fixed_from_i64(value.mantissa, 15, value.exponent, out);
}

linears_status_t linears_fixed_from_q31(linears_q31_t value, linears_fixed_t *out) {
    return linears_fixed_from_i64(value.mantissa, 31, value.exponent, out);
}

linears_status_t linears_fixed_add(linears_fixed_t a, linears_fixed_t b,
                                 linears_fixed_t *out) {
    if (out == NULL || !linears_fixed_valid(a) || !linears_fixed_valid(b))
        return LINEARS_ERROR_ARGUMENT;
    if (a.mantissa == 0) { *out = b; return LINEARS_OK; }
    if (b.mantissa == 0) { *out = a; return LINEARS_OK; }
    if (a.exponent < b.exponent) {
        linears_fixed_t temporary = a;
        a = b;
        b = temporary;
    }
    unsigned distance = (unsigned)(a.exponent - b.exponent);
    /* At distance >32 the smaller operand is strictly below half an ulp,
     * including at the lower normalization boundary. Otherwise the exact
     * aligned sum fits int64_t: (2^31-1)*2^32 + (2^31-1) < INT64_MAX. */
    if (distance > 32) { *out = a; return LINEARS_OK; }
    int64_t aligned = (int64_t)a.mantissa * (INT64_C(1) << distance);
    return pack(aligned + b.mantissa, (int64_t)b.exponent - 31, out);
}

linears_status_t linears_fixed_sub(linears_fixed_t a, linears_fixed_t b,
                                 linears_fixed_t *out) {
    if (!linears_fixed_valid(b)) return LINEARS_ERROR_ARGUMENT;
    b.mantissa = -b.mantissa;
    return linears_fixed_add(a, b, out);
}

linears_status_t linears_fixed_mul(linears_fixed_t a, linears_fixed_t b,
                                 linears_fixed_t *out) {
    if (out == NULL || !linears_fixed_valid(a) || !linears_fixed_valid(b))
        return LINEARS_ERROR_ARGUMENT;
    return pack((int64_t)a.mantissa * b.mantissa,
                (int64_t)a.exponent + b.exponent - 62, out);
}

linears_status_t linears_fixed_div(linears_fixed_t a, linears_fixed_t b,
                                 linears_fixed_t *out) {
    if (out == NULL || !linears_fixed_valid(a) || !linears_fixed_valid(b))
        return LINEARS_ERROR_ARGUMENT;
    if (b.mantissa == 0) return LINEARS_ERROR_DOMAIN;
    if (a.mantissa == 0) { *out = a; return LINEARS_OK; }
    uint64_t numerator = magnitude64(a.mantissa);
    uint64_t denominator = magnitude64(b.mantissa);
    unsigned shift = numerator >= denominator ? 30 : 31;
    numerator <<= shift;
    uint64_t quotient = numerator / denominator;
    uint64_t remainder = numerator % denominator;
    quotient += remainder * 2 >= denominator;
    int64_t signed_quotient = (a.mantissa < 0) != (b.mantissa < 0)
                                 ? -(int64_t)quotient : (int64_t)quotient;
    return pack(signed_quotient, (int64_t)a.exponent - b.exponent - shift, out);
}

/* Binary restoring square root (base-4 digit recurrence), as described in
 * Hacker's Delight, 2nd ed., section 11-1, "Integer Square Root". The loop
 * consumes at most 32 pairs of bits; it does not use Newton floating seeds. */
static uint64_t integer_sqrt(uint64_t value) {
    uint64_t root = 0;
    uint64_t bit = UINT64_C(1) << 62;
    while (bit > value) bit >>= 2;
    while (bit != 0) {
        if (value >= root + bit) {
            value -= root + bit;
            root = (root >> 1) + bit;
        } else {
            root >>= 1;
        }
        bit >>= 2;
    }
    return root;
}

linears_status_t linears_fixed_sqrt(linears_fixed_t value, linears_fixed_t *out) {
    if (out == NULL || !linears_fixed_valid(value)) return LINEARS_ERROR_ARGUMENT;
    if (value.mantissa < 0) return LINEARS_ERROR_DOMAIN;
    if (value.mantissa == 0) { *out = value; return LINEARS_OK; }
    int odd = value.exponent % 2 != 0;
    uint64_t radicand = (uint64_t)value.mantissa << (31 - odd);
    uint64_t root = integer_sqrt(radicand);
    /* N >= (r + 1/2)^2 iff N-r*r >= r+1, since N is an integer. */
    root += radicand - root * root >= root + 1;
    int exponent = odd ? (value.exponent + 1) / 2 : value.exponent / 2;
    return pack((int64_t)root, (int64_t)exponent - 31, out);
}

linears_status_t linears_fixed_neg(linears_fixed_t value, linears_fixed_t *out) {
    if (out == NULL || !linears_fixed_valid(value)) return LINEARS_ERROR_ARGUMENT;
    value.mantissa = -value.mantissa;
    *out = value;
    return LINEARS_OK;
}

linears_status_t linears_fixed_abs(linears_fixed_t value, linears_fixed_t *out) {
    if (out == NULL || !linears_fixed_valid(value)) return LINEARS_ERROR_ARGUMENT;
    if (value.mantissa < 0) value.mantissa = -value.mantissa;
    *out = value;
    return LINEARS_OK;
}

int linears_fixed_compare(linears_fixed_t a, linears_fixed_t b) {
    if (a.mantissa < 0 && b.mantissa >= 0) return -1;
    if (a.mantissa >= 0 && b.mantissa < 0) return 1;
    if (a.mantissa == 0) return b.mantissa == 0 ? 0 : -1;
    if (b.mantissa == 0) return 1;
    if (a.exponent != b.exponent) {
        int magnitude_order = a.exponent < b.exponent ? -1 : 1;
        return a.mantissa < 0 ? -magnitude_order : magnitude_order;
    }
    return (a.mantissa > b.mantissa) - (a.mantissa < b.mantissa);
}

static linears_status_t quantize(linears_fixed_t value, int exponent,
                                  unsigned bits, int64_t *out) {
    if (!linears_fixed_valid(value)) return LINEARS_ERROR_ARGUMENT;
    if (exponent < LINEARS_EXPONENT_MIN || exponent > LINEARS_EXPONENT_MAX)
        return LINEARS_ERROR_RANGE;
    int shift = value.exponent - exponent + (int)bits - 31;
    uint64_t magnitude = magnitude64(value.mantissa);
    uint64_t limit = (UINT64_C(1) << bits) - (value.mantissa >= 0);
    if (magnitude != 0) {
        if (shift >= 0) {
            if (shift > (int)bits || magnitude > (limit >> shift))
                return LINEARS_ERROR_RANGE;
            magnitude <<= shift;
        } else {
            magnitude = rounded_right(magnitude, (unsigned)-shift);
        }
    }
    if (magnitude > limit) return LINEARS_ERROR_RANGE;
    *out = value.mantissa < 0 ? -(int64_t)magnitude : (int64_t)magnitude;
    return LINEARS_OK;
}

linears_status_t linears_fixed_to_q15(linears_fixed_t value, int exponent,
                                    int16_t *out) {
    if (out == NULL) return LINEARS_ERROR_ARGUMENT;
    int64_t result;
    linears_status_t status = quantize(value, exponent, 15, &result);
    if (status == LINEARS_OK) *out = (int16_t)result;
    return status;
}

linears_status_t linears_fixed_to_q31(linears_fixed_t value, int exponent,
                                    int32_t *out) {
    if (out == NULL) return LINEARS_ERROR_ARGUMENT;
    int64_t result;
    linears_status_t status = quantize(value, exponent, 31, &result);
    if (status == LINEARS_OK) *out = (int32_t)result;
    return status;
}

static int overlaps(const void *a, size_t asize, const void *b, size_t bsize) {
    if (asize == 0 || bsize == 0) return 0;
    uintptr_t x = (uintptr_t)a;
    uintptr_t y = (uintptr_t)b;
    return x <= y ? y - x < asize : x - y < bsize;
}

/* Validate every element and quantization before the first output write. */
static linears_status_t export_block(const linears_fixed_t *values, size_t count,
                                    void *out, int *exponent, unsigned bits) {
    size_t width = bits == 15 ? sizeof(int16_t) : sizeof(int32_t);
    size_t alignment = bits == 15 ? offsetof(struct alignment_i16, value)
                                 : offsetof(struct alignment_i32, value);
    if (exponent == NULL || (uintptr_t)exponent % offsetof(struct alignment_int, value) != 0 ||
        (count != 0 && (values == NULL || out == NULL ||
         (uintptr_t)values % offsetof(struct alignment_fixed, value) != 0 ||
         (uintptr_t)out % alignment != 0))) return LINEARS_ERROR_ARGUMENT;
    if (count > SIZE_MAX / sizeof(*values) || count > SIZE_MAX / width)
        return LINEARS_ERROR_SIZE;
    if (overlaps(values, count * sizeof(*values), out, count * width) ||
        overlaps(values, count * sizeof(*values), exponent, sizeof(*exponent)) ||
        overlaps(out, count * width, exponent, sizeof(*exponent)))
        return LINEARS_ERROR_ALIAS;
    int block_exponent = LINEARS_EXPONENT_MIN;
    int any = 0;
    for (size_t i = 0; i < count; ++i) {
        if (!linears_fixed_valid(values[i])) return LINEARS_ERROR_ARGUMENT;
        if (values[i].mantissa != 0) {
            any = 1;
            if (values[i].exponent > block_exponent) block_exponent = values[i].exponent;
        }
    }
    if (!any) block_exponent = 0;
    /* q15 can carry a positive rounding bit into the shared exponent. q31
     * canonical mantissas fit at their maximum exponent without a carry. */
    for (size_t i = 0; i < count; ++i) {
        int64_t ignored;
        if (quantize(values[i], block_exponent, bits, &ignored) != LINEARS_OK) {
            if (block_exponent == LINEARS_EXPONENT_MAX) return LINEARS_ERROR_RANGE;
            ++block_exponent;
            break;
        }
    }
    for (size_t i = 0; i < count; ++i) {
        int64_t mantissa;
        (void)quantize(values[i], block_exponent, bits, &mantissa);
        if (bits == 15) ((int16_t *)out)[i] = (int16_t)mantissa;
        else ((int32_t *)out)[i] = (int32_t)mantissa;
    }
    *exponent = block_exponent;
    return LINEARS_OK;
}

linears_status_t linears_fixed_export_q15(const linears_fixed_t *values,
                                        size_t count, int16_t *out, int *exponent) {
    return export_block(values, count, out, exponent, 15);
}

linears_status_t linears_fixed_export_q31(const linears_fixed_t *values,
                                        size_t count, int32_t *out, int *exponent) {
    return export_block(values, count, out, exponent, 31);
}
