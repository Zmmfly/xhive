/**
 * @file linears_numeric.h
 * @brief Portable integer-only arithmetic with an invariant 31-bit fraction.
 *
 * linears_fixed_t is int64_t raw / 2^31, not a normalized mantissa. Every raw
 * value is valid. There is no exponent, floating conversion, or dynamic shift
 * metadata. Products round once to nearest, with halfway cases away from zero;
 * successful fixed results have absolute rounding error <= 2^-32. Relative
 * accuracy is not constant near zero. Add/sub/neg/abs are exact when successful.
 * Wide multiplication/division use two uint64_t limbs, never __int128.
 *
 * Q export first checks the actual supplied value against the closed interval
 * [-1,+1]. Values outside it return RANGE, including values just above +1.
 * Within that interval, nearest-away rounding that reaches the unavailable
 * positive endpoint maps to INT16_MAX/INT32_MAX. Thus +1 is represented with
 * one signal LSB error; -1 is represented exactly. The q15 upper rounding cell
 * has error <= one q15 LSB, not half an LSB. Other cells have half-LSB error.
 *
 * Every failed operation leaves its output unchanged. Scalar inputs are passed
 * by value, so out may alias them. Typed pointers must have natural alignment.
 * Array export additionally checks alignment, byte-size arithmetic and overlap.
 * Calls allocate nothing, use bounded scalar stack storage, retain no pointers,
 * and are reentrant when caller-owned output storage is disjoint.
 */
#ifndef LINEARS_NUMERIC_H
#define LINEARS_NUMERIC_H

#include "linears.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Signed 128-bit two's-complement accumulator with a fixed 62-bit fraction.
 *
 * The raw signed integer is encoded by high:low and divided by 2^62. Initialize
 * both limbs to zero. There is no exponent and no normalization invariant.
 * This format retains exact products of two linears_fixed_t raw integers.
 */
typedef struct {
    uint64_t low;  /**< Least-significant 64 bits of the two's-complement integer. */
    uint64_t high; /**< Most-significant 64 bits; bit 63 is the sign bit. */
} linears_accum_t;

/**
 * @brief Convert a q15 signal to the wide F31 format exactly.
 *
 * @param[in] value Raw q15 signal.
 * @return linears_fixed_t value multiplied by 65536; every q15 input is valid.
 */
static inline linears_fixed_t linears_fixed_from_q15(linears_q15_t value) {
    return (int64_t)value * INT64_C(65536);
}

/**
 * @brief Convert a q31 signal to the wide F31 format exactly.
 *
 * @param[in] value Raw q31 signal.
 * @return linears_fixed_t Sign-extended raw value; every q31 input is valid.
 */
static inline linears_fixed_t linears_fixed_from_q31(linears_q31_t value) {
    return value;
}

/**
 * @brief Compare two wide F31 raw integers without conversion.
 *
 * @param[in] a First working value.
 * @param[in] b Second working value.
 * @return int -1 if a < b, zero if equal, or +1 if a > b.
 */
static inline int linears_fixed_compare(linears_fixed_t a, linears_fixed_t b) {
    return (a > b) - (a < b);
}

/**
 * @brief Convert an integer to the wide F31 format exactly.
 *
 * @param[in] integer Integer value, representable only in [-2^32, 2^32-1].
 * @param[out] out Non-NULL destination, unchanged on failure.
 * @return linears_status_t OK, ARGUMENT for NULL out, or RANGE on overflow.
 */
linears_status_t linears_fixed_from_int(int64_t integer, linears_fixed_t *out);

/**
 * @brief Add two wide F31 values exactly with signed-overflow detection.
 *
 * @param[in] a First operand.
 * @param[in] b Second operand.
 * @param[out] out Non-NULL destination, unchanged on failure.
 * @return linears_status_t OK, ARGUMENT for NULL out, or RANGE on overflow.
 */
linears_status_t linears_fixed_add(linears_fixed_t a, linears_fixed_t b, linears_fixed_t *out);

/**
 * @brief Subtract b from a exactly with signed-overflow detection.
 *
 * @param[in] a Minuend.
 * @param[in] b Subtrahend.
 * @param[out] out Non-NULL destination, unchanged on failure.
 * @return linears_status_t OK, ARGUMENT for NULL out, or RANGE on overflow.
 */
linears_status_t linears_fixed_sub(linears_fixed_t a, linears_fixed_t b, linears_fixed_t *out);

/**
 * @brief Multiply wide F31 values with one final nearest-away rounding.
 *
 * @param[in] a First factor.
 * @param[in] b Second factor.
 * @param[out] out Non-NULL F31 destination, unchanged on failure.
 * @return linears_status_t OK, ARGUMENT for NULL out, or RANGE on rounded overflow.
 * @note The raw product is exact up to 128 bits before the fixed 31-bit shift.
 */
linears_status_t linears_fixed_mul(linears_fixed_t a, linears_fixed_t b, linears_fixed_t *out);

/**
 * @brief Divide wide F31 values with one final nearest-away rounding.
 *
 * @param[in] a Numerator.
 * @param[in] b Denominator, nonzero.
 * @param[out] out Non-NULL F31 destination, unchanged on failure.
 * @return linears_status_t OK, ARGUMENT for NULL out, DOMAIN for zero b, or RANGE.
 * @note The exact unsigned numerator magnitude is scaled by 2^31 in two limbs.
 */
linears_status_t linears_fixed_div(linears_fixed_t a, linears_fixed_t b, linears_fixed_t *out);

/**
 * @brief Compute (a * b) / divisor without rounding the intermediate product.
 *
 * @param[in] a First F31 factor.
 * @param[in] b Second F31 factor.
 * @param[in] divisor Nonzero F31 divisor.
 * @param[out] out Non-NULL F31 destination, unchanged on failure.
 * @return linears_status_t OK, ARGUMENT for NULL out, DOMAIN for zero divisor,
 *         or RANGE when the nearest-away rounded quotient does not fit int64_t.
 * @note Equivalently computes nearest-away(a_raw * b_raw / divisor_raw).
 */
linears_status_t linears_fixed_muldiv(linears_fixed_t a, linears_fixed_t b,
                                    linears_fixed_t divisor, linears_fixed_t *out);

/**
 * @brief Compute the nonnegative square root using bounded integer restoring steps.
 *
 * @param[in] value Nonnegative F31 input, including tiny raw values such as 1.
 * @param[out] out Non-NULL F31 result, unchanged on failure.
 * @return linears_status_t OK, ARGUMENT for NULL out, or DOMAIN for negative value.
 * @note The radicand is value_raw * 2^31; no floating-point seed is used.
 */
linears_status_t linears_fixed_sqrt(linears_fixed_t value, linears_fixed_t *out);

/**
 * @brief Negate a wide F31 value exactly.
 *
 * @param[in] value Operand.
 * @param[out] out Non-NULL result, unchanged on failure.
 * @return linears_status_t OK, ARGUMENT for NULL out, or RANGE for INT64_MIN.
 */
linears_status_t linears_fixed_neg(linears_fixed_t value, linears_fixed_t *out);

/**
 * @brief Take the absolute value of a wide F31 value exactly.
 *
 * @param[in] value Operand.
 * @param[out] out Non-NULL result, unchanged on failure.
 * @return linears_status_t OK, ARGUMENT for NULL out, or RANGE for INT64_MIN.
 */
linears_status_t linears_fixed_abs(linears_fixed_t value, linears_fixed_t *out);

/**
 * @brief Quantize a wide F31 value to q15 using the documented endpoint policy.
 *
 * @param[in] value F31 working value; the accepted interval is [-1,+1].
 * @param[out] out Non-NULL raw q15 destination, unchanged on failure.
 * @return linears_status_t OK, ARGUMENT for NULL out, or RANGE outside [-1,+1].
 */
linears_status_t linears_fixed_to_q15(linears_fixed_t value, linears_q15_t *out);

/**
 * @brief Quantize a wide F31 value to q31 using the documented endpoint policy.
 *
 * @param[in] value F31 working value; the accepted interval is [-1,+1].
 * @param[out] out Non-NULL raw q31 destination, unchanged on failure.
 * @return linears_status_t OK, ARGUMENT for NULL out, or RANGE outside [-1,+1].
 */
linears_status_t linears_fixed_to_q31(linears_fixed_t value, linears_q31_t *out);

/**
 * @brief Add an exact product to a signed 128-bit F62 accumulator.
 *
 * @param[in,out] accumulator Non-NULL two-limb accumulator, unchanged on failure.
 * @param[in] a First F31 factor.
 * @param[in] b Second F31 factor.
 * @return linears_status_t OK, ARGUMENT for NULL accumulator, or RANGE for
 *         signed 128-bit addition overflow. No product bits are discarded.
 */
linears_status_t linears_fixed_mac(linears_accum_t *accumulator,
                                 linears_fixed_t a, linears_fixed_t b);

/**
 * @brief Subtract an exact product from a signed 128-bit F62 accumulator.
 *
 * @param[in,out] accumulator Non-NULL two-limb accumulator, unchanged on failure.
 * @param[in] a First F31 factor.
 * @param[in] b Second F31 factor.
 * @return linears_status_t OK, ARGUMENT for NULL accumulator, or RANGE for
 *         signed 128-bit subtraction overflow. No product bits are discarded.
 */
linears_status_t linears_fixed_msu(linears_accum_t *accumulator,
                                 linears_fixed_t a, linears_fixed_t b);

/**
 * @brief Round an F62 accumulator once to a wide F31 working value.
 *
 * @param[in] value Signed two-limb F62 value.
 * @param[out] out Non-NULL destination, unchanged on failure.
 * @return linears_status_t OK, ARGUMENT for NULL out, or RANGE on rounded overflow.
 */
linears_status_t linears_accum_to_fixed(linears_accum_t value, linears_fixed_t *out);

/**
 * @brief Quantize an exact F62 accumulator directly to q15.
 *
 * @param[in] value Signed two-limb F62 value, checked against [-1,+1] before rounding.
 * @param[out] out Non-NULL q15 destination, unchanged on failure.
 * @return linears_status_t OK, ARGUMENT for NULL out, or RANGE outside [-1,+1].
 * @note No intermediate F31 quantization can hide a result just outside range.
 */
linears_status_t linears_accum_to_q15(linears_accum_t value, linears_q15_t *out);

/**
 * @brief Quantize an exact F62 accumulator directly to q31.
 *
 * @param[in] value Signed two-limb F62 value, checked against [-1,+1] before rounding.
 * @param[out] out Non-NULL q31 destination, unchanged on failure.
 * @return linears_status_t OK, ARGUMENT for NULL out, or RANGE outside [-1,+1].
 */
linears_status_t linears_accum_to_q31(linears_accum_t value, linears_q31_t *out);

/**
 * @brief Divide an exact F62 sum by a positive count and quantize directly to q15.
 *
 * @param[in] sum Exact signed two-limb F62 sum.
 * @param[in] count Positive unsigned integer divisor, including UINT64_MAX.
 * @param[out] out Non-NULL q15 destination, unchanged on failure.
 * @return linears_status_t OK, ARGUMENT for NULL out, DOMAIN for zero count,
 *         or RANGE when the exact mean is outside [-1,+1].
 * @note Range is checked against count * 2^62 before division. There is only
 *       one nearest-away rounding, and the usual positive endpoint policy applies.
 */
linears_status_t linears_accum_mean_q15(linears_accum_t sum, uint64_t count, linears_q15_t *out);

/**
 * @brief Divide an exact F62 sum by a positive count and quantize directly to q31.
 *
 * @param[in] sum Exact signed two-limb F62 sum.
 * @param[in] count Positive unsigned integer divisor, including UINT64_MAX.
 * @param[out] out Non-NULL q31 destination, unchanged on failure.
 * @return linears_status_t OK, ARGUMENT for NULL out, DOMAIN for zero count,
 *         or RANGE when the exact mean is outside [-1,+1].
 * @note No conversion of count to F31 and no intermediate rounded mean is used.
 */
linears_status_t linears_accum_mean_q31(linears_accum_t sum, uint64_t count, linears_q31_t *out);

/**
 * @brief Export wide F31 values to q15 without partial output on failure.
 *
 * @param[in] values Naturally aligned array of count F31 values; NULL only for zero count.
 * @param[in] count Element count; byte-size or address overflow is rejected.
 * @param[out] out Storage for count q15 elements; NULL only for zero count.
 *                 Must not overlap values. Unchanged on failure.
 * @return linears_status_t OK, ARGUMENT for invalid pointers/alignment, SIZE for
 *         storage arithmetic overflow, ALIAS for overlap, or RANGE for any value.
 */
linears_status_t linears_fixed_export_q15(const linears_fixed_t *values, size_t count,
                                        linears_q15_t *out);

/**
 * @brief Export wide F31 values to q31 without partial output on failure.
 *
 * @param[in] values Naturally aligned array of count F31 values; NULL only for zero count.
 * @param[in] count Element count; byte-size or address overflow is rejected.
 * @param[out] out Storage for count q31 elements; NULL only for zero count.
 *                 Must not overlap values. Unchanged on failure.
 * @return linears_status_t OK, ARGUMENT for invalid pointers/alignment, SIZE for
 *         storage arithmetic overflow, ALIAS for overlap, or RANGE for any value.
 */
linears_status_t linears_fixed_export_q31(const linears_fixed_t *values, size_t count,
                                        linears_q31_t *out);

#ifdef __cplusplus
}
#endif
#endif
