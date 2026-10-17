/**
 * @file linears_numeric.h
 * @brief Reentrant, integer-only working arithmetic for block fixed point.
 *
 * No function in this interface evaluates floating-point expressions. A nonzero
 * working value has 2^30 <= abs(mantissa) < 2^31 and represents
 * mantissa * 2^(exponent - 31). Zero is always {0, 0}. There are 31 significant
 * magnitude bits. Each arithmetic result is rounded to nearest, with exact
 * halfway cases rounded away from zero. No saturation or wraparound is used.
 * Values outside LINEARS_EXPONENT_MIN..LINEARS_EXPONENT_MAX fail with RANGE.
 *
 * Inputs to arithmetic and comparison must be canonical working values produced
 * by this interface (or equivalent constants). Import q15/q31 storage first.
 * Arithmetic checks this condition; comparison has it as a precondition.
 * Scalar out parameters may alias an input object, since inputs are passed by
 * value. On every scalar failure the output object is unchanged. No function
 * allocates, uses variable-sized stack arrays, or reads mutable global state.
 */
#ifndef LINEARS_NUMERIC_H
#define LINEARS_NUMERIC_H

#include "linears.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Normalized 31-significant-bit integer working scalar.
 */
typedef linears_q31_t linears_fixed_t;

/**
 * @brief Test whether a scalar is canonical and its exponent is in range.
 *
 * @param[in] value Scalar to inspect.
 * @return int Nonzero for a canonical scalar, zero otherwise.
 */
int linears_fixed_valid(linears_fixed_t value);

/**
 * @brief Import an integer times an explicit binary scale.
 *
 * @param[in] mantissa Signed integer numerator, including INT64_MIN.
 * @param[in] fractional_bits Denominator power of two, in the range 0..63.
 * @param[in] exponent Binary scale in the accepted exponent range.
 * @param[out] out Non-NULL normalized destination, unchanged on failure.
 * @return linears_status_t Import status.
 * @retval LINEARS_OK The rounded value was stored.
 * @retval LINEARS_ERROR_ARGUMENT out is NULL or fractional_bits is invalid.
 * @retval LINEARS_ERROR_RANGE The input or normalized exponent is out of range.
 */
linears_status_t linears_fixed_from_i64(int64_t mantissa, int fractional_bits,
                                      int exponent, linears_fixed_t *out);

/**
 * @brief Import a q15 scalar without floating-point conversion.
 *
 * @param[in] value Scalar with a raw q15 mantissa and explicit scale.
 * @param[out] out Non-NULL normalized destination, unchanged on failure.
 * @return linears_status_t OK, ARGUMENT for NULL out, or RANGE for an invalid
 *         input exponent or an unrepresentable normalized exponent.
 */
linears_status_t linears_fixed_from_q15(linears_q15_t value, linears_fixed_t *out);

/**
 * @brief Import a q31 scalar without floating-point conversion.
 *
 * @param[in] value Scalar with a raw q31 mantissa and explicit scale.
 * @param[out] out Non-NULL normalized destination, unchanged on failure.
 * @return linears_status_t OK, ARGUMENT for NULL out, or RANGE for an invalid
 *         input exponent or an unrepresentable normalized exponent.
 */
linears_status_t linears_fixed_from_q31(linears_q31_t value, linears_fixed_t *out);

/**
 * @brief Add two normalized working scalars.
 *
 * @param[in] a First canonical operand.
 * @param[in] b Second canonical operand.
 * @param[out] out Non-NULL result; unchanged on failure.
 * @return linears_status_t OK, ARGUMENT for noncanonical operands or NULL out,
 *         or RANGE if the normalized result exponent cannot be represented.
 */
linears_status_t linears_fixed_add(linears_fixed_t a, linears_fixed_t b,
                                 linears_fixed_t *out);

/**
 * @brief Subtract normalized b from normalized a.
 *
 * @param[in] a Canonical minuend.
 * @param[in] b Canonical subtrahend.
 * @param[out] out Non-NULL result; unchanged on failure.
 * @return linears_status_t OK, ARGUMENT for invalid operands/out, or RANGE.
 */
linears_status_t linears_fixed_sub(linears_fixed_t a, linears_fixed_t b,
                                 linears_fixed_t *out);

/**
 * @brief Multiply normalized scalars using a signed 64-bit product.
 *
 * @param[in] a First canonical factor.
 * @param[in] b Second canonical factor.
 * @param[out] out Non-NULL result; unchanged on failure.
 * @return linears_status_t OK, ARGUMENT for invalid operands/out, or RANGE.
 */
linears_status_t linears_fixed_mul(linears_fixed_t a, linears_fixed_t b,
                                 linears_fixed_t *out);

/**
 * @brief Divide normalized scalars using unsigned 64-bit integer division.
 *
 * @param[in] a Canonical numerator.
 * @param[in] b Canonical denominator; zero is rejected.
 * @param[out] out Non-NULL result; unchanged on failure.
 * @return linears_status_t OK, ARGUMENT for invalid operands/out, RANGE for an
 *         unrepresentable exponent, or DOMAIN for a zero denominator.
 */
linears_status_t linears_fixed_div(linears_fixed_t a, linears_fixed_t b,
                                 linears_fixed_t *out);

/**
 * @brief Compute the nonnegative square root by integer restoring square root.
 *
 * @param[in] value Canonical nonnegative scalar.
 * @param[out] out Non-NULL nearest-rounded root; unchanged on failure.
 * @return linears_status_t OK, ARGUMENT for invalid value/out, DOMAIN for a
 *         negative value, or RANGE for an unrepresentable result exponent.
 */
linears_status_t linears_fixed_sqrt(linears_fixed_t value, linears_fixed_t *out);

/**
 * @brief Negate a normalized scalar exactly.
 *
 * @param[in] value Canonical operand.
 * @param[out] out Non-NULL result; unchanged on failure.
 * @return linears_status_t OK, or ARGUMENT for invalid value/out.
 */
linears_status_t linears_fixed_neg(linears_fixed_t value, linears_fixed_t *out);

/**
 * @brief Take the absolute value of a normalized scalar exactly.
 *
 * @param[in] value Canonical operand.
 * @param[out] out Non-NULL result; unchanged on failure.
 * @return linears_status_t OK, or ARGUMENT for invalid value/out.
 */
linears_status_t linears_fixed_abs(linears_fixed_t value, linears_fixed_t *out);

/**
 * @brief Compare canonical working scalars without subtraction or rescaling.
 *
 * @param[in] a First canonical operand.
 * @param[in] b Second canonical operand.
 * @return int -1 if a < b, 0 if a == b, or 1 if a > b.
 * @pre Both operands satisfy linears_fixed_valid(). No validation is repeated.
 */
int linears_fixed_compare(linears_fixed_t a, linears_fixed_t b);

/**
 * @brief Quantize one working scalar at a caller-selected q15 block exponent.
 *
 * @param[in] value Canonical scalar.
 * @param[in] exponent Desired block exponent in the accepted range.
 * @param[out] out Non-NULL raw mantissa destination; unchanged on failure.
 * @return linears_status_t OK, ARGUMENT for invalid value/out, or RANGE when
 *         exponent is invalid or the rounded mantissa cannot fit int16_t.
 * @note Values smaller than half an output unit round to zero.
 */
linears_status_t linears_fixed_to_q15(linears_fixed_t value, int exponent,
                                    int16_t *out);

/**
 * @brief Quantize one working scalar at a caller-selected q31 block exponent.
 *
 * @param[in] value Canonical scalar.
 * @param[in] exponent Desired block exponent in the accepted range.
 * @param[out] out Non-NULL raw mantissa destination; unchanged on failure.
 * @return linears_status_t OK, ARGUMENT for invalid value/out, or RANGE when
 *         exponent is invalid or the rounded mantissa cannot fit int32_t.
 * @note Values smaller than half an output unit round to zero.
 */
linears_status_t linears_fixed_to_q31(linears_fixed_t value, int exponent,
                                    int32_t *out);

/**
 * @brief Export a canonical array as one automatically normalized q15 block.
 *
 * The largest nonzero magnitude selects the common exponent. If positive
 * rounding would overflow, the common exponent is increased by one and every
 * element is rounded at that scale. An all-zero or empty block uses exponent 0.
 * Individual small elements can round to zero because the scale is shared.
 *
 * @param[in] values Canonical contiguous array of count values; NULL only if
 *                   count is zero. Must not overlap out or exponent.
 * @param[in] count Number of elements; counts with overflowing byte sizes fail.
 * @param[out] out Contiguous storage for count int16_t values; NULL only if
 *                 count is zero. Must not overlap exponent.
 * @param[out] exponent Non-NULL common block exponent destination.
 * @return linears_status_t OK, ARGUMENT for invalid pointers/values/alignment,
 *         SIZE for byte-size overflow, ALIAS for overlapping ranges, or RANGE
 *         for a block exponent that cannot be represented.
 * @note All output elements and exponent remain unchanged on any failure.
 */
linears_status_t linears_fixed_export_q15(const linears_fixed_t *values,
                                        size_t count, int16_t *out, int *exponent);

/**
 * @brief Export a canonical array as one automatically normalized q31 block.
 *
 * @param[in] values Canonical contiguous array of count values; NULL only for
 *                   count zero. Must not overlap out or exponent.
 * @param[in] count Number of elements; overflowing byte sizes fail.
 * @param[out] out Storage for count int32_t values; NULL only for count zero.
 *                 Must not overlap exponent.
 * @param[out] exponent Non-NULL common exponent destination.
 * @return linears_status_t OK, ARGUMENT for invalid pointers/values/alignment,
 *         SIZE for byte-size overflow, ALIAS for overlap, or RANGE.
 * @note Normalization, rounding and atomic output commit match export_q15.
 */
linears_status_t linears_fixed_export_q31(const linears_fixed_t *values,
                                        size_t count, int32_t *out, int *exponent);

#ifdef __cplusplus
}
#endif
#endif
