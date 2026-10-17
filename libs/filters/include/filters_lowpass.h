/**
 * @file filters_lowpass.h
 * @brief First-order low-pass filtering with explicit caller-owned state.
 *
 * A successful step computes y[n] = (1-alpha)*y[n-1] + alpha*x[n].
 * Alpha is dimensionless in [0,1]; one means passthrough, zero holds state.
 * For a backward-Euler RC low-pass, select alpha = dt/(tau+dt), with dt and
 * tau in the same time unit. This API does not perform that conversion.
 * Fixed-point values use linears' signed mantissa and binary exponent.
 *
 * Contexts must be initialized before step/reset and must not be edited by
 * callers. Calls on one context require serialization, including interrupts.
 * Distinct contexts are reentrant. No allocation, locks, I/O or MMIO occurs.
 * On every failure the context and output retain their prior values.
 */
#ifndef FILTERS_LOWPASS_H
#define FILTERS_LOWPASS_H

#include "linears.h"
#include "linears_numeric.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Single-precision low-pass state, owned and kept alive by the caller.
 */
typedef struct {
    float alpha; /**< Dimensionless input weight in [0,1]; immutable after init. */
    float value; /**< Last output, in the same physical unit as the input. */
} filters_lowpass_f32_t;

/**
 * @brief Initialize a single-precision low-pass without external storage.
 *
 * @param[out] ctx Non-NULL caller-owned context; no prior initialization needed.
 * @param[in] alpha Dimensionless input weight in [0,1].
 * @param[in] initial Initial output, in input units; must be finite.
 *
 * @return linears_status_t Status of the initialization.
 *
 * @retval LINEARS_OK Context initialized.
 * @retval LINEARS_ERROR_ARGUMENT ctx is NULL.
 * @retval LINEARS_ERROR_DOMAIN alpha is outside [0,1].
 * @retval LINEARS_ERROR_RANGE A scalar input is nonfinite.
 *
 * @note On failure ctx is unchanged; serialize concurrent use of ctx.
 */
linears_status_t filters_lowpass_f32_init(filters_lowpass_f32_t *ctx, float alpha, float initial);

/**
 * @brief Replace a single-precision low-pass output history, preserving alpha.
 *
 * @param[in,out] ctx Non-NULL successfully initialized context.
 * @param[in] initial New initial output in input units; must be finite.
 *
 * @return linears_status_t Status of the reset.
 *
 * @retval LINEARS_OK History replaced.
 * @retval LINEARS_ERROR_ARGUMENT ctx is NULL.
 * @retval LINEARS_ERROR_RANGE initial is nonfinite.
 *
 * @note Failure leaves ctx unchanged. Serialize concurrent use of ctx.
 */
linears_status_t filters_lowpass_f32_reset(filters_lowpass_f32_t *ctx, float initial);

/**
 * @brief Filter one single-precision sample and atomically commit its output.
 *
 * @param[in,out] ctx Non-NULL successfully initialized context.
 * @param[in] input Finite input sample in the same units as the initial output.
 * @param[out] output Non-NULL scalar destination; must not overlap ctx.
 *
 * @return linears_status_t Status of the sample operation.
 *
 * @retval LINEARS_OK State and output committed.
 * @retval LINEARS_ERROR_ARGUMENT ctx or output is NULL.
 * @retval LINEARS_ERROR_ALIAS output overlaps ctx.
 * @retval LINEARS_ERROR_RANGE input or an intermediate result is nonfinite.
 *
 * @note Failure leaves state and output unchanged. Serialize use of ctx.
 */
linears_status_t filters_lowpass_f32_step(filters_lowpass_f32_t *ctx, float input, float *output);

/**
 * @brief Double-precision low-pass state, owned and kept alive by the caller.
 */
typedef struct {
    double alpha; /**< Dimensionless input weight in [0,1]; immutable after init. */
    double value; /**< Last output, in the same physical unit as the input. */
} filters_lowpass_f64_t;

/**
 * @brief Initialize a double-precision low-pass without external storage.
 *
 * @param[out] ctx Non-NULL caller-owned context; no prior initialization needed.
 * @param[in] alpha Dimensionless input weight in [0,1].
 * @param[in] initial Initial output, in input units; must be finite.
 *
 * @return linears_status_t Status of the initialization.
 *
 * @retval LINEARS_OK Context initialized.
 * @retval LINEARS_ERROR_ARGUMENT ctx is NULL.
 * @retval LINEARS_ERROR_DOMAIN alpha is outside [0,1].
 * @retval LINEARS_ERROR_RANGE A scalar input is nonfinite.
 *
 * @note On failure ctx is unchanged; serialize concurrent use of ctx.
 */
linears_status_t filters_lowpass_f64_init(filters_lowpass_f64_t *ctx, double alpha, double initial);

/**
 * @brief Replace a double-precision low-pass output history, preserving alpha.
 *
 * @param[in,out] ctx Non-NULL successfully initialized context.
 * @param[in] initial New initial output in input units; must be finite.
 *
 * @return linears_status_t Status of the reset.
 *
 * @retval LINEARS_OK History replaced.
 * @retval LINEARS_ERROR_ARGUMENT ctx is NULL.
 * @retval LINEARS_ERROR_RANGE initial is nonfinite.
 *
 * @note Failure leaves ctx unchanged. Serialize concurrent use of ctx.
 */
linears_status_t filters_lowpass_f64_reset(filters_lowpass_f64_t *ctx, double initial);

/**
 * @brief Filter one double-precision sample and atomically commit its output.
 *
 * @param[in,out] ctx Non-NULL successfully initialized context.
 * @param[in] input Finite input sample in the same units as the initial output.
 * @param[out] output Non-NULL scalar destination; must not overlap ctx.
 *
 * @return linears_status_t Status of the sample operation.
 *
 * @retval LINEARS_OK State and output committed.
 * @retval LINEARS_ERROR_ARGUMENT ctx or output is NULL.
 * @retval LINEARS_ERROR_ALIAS output overlaps ctx.
 * @retval LINEARS_ERROR_RANGE input or an intermediate result is nonfinite.
 *
 * @note Failure leaves state and output unchanged. Serialize use of ctx.
 */
linears_status_t filters_lowpass_f64_step(filters_lowpass_f64_t *ctx, double input, double *output);

/**
 * @brief Q15 low-pass state using value = mantissa * 2^(exponent-15).
 *
 * @note Arithmetic uses linears' normalized Q31 integer core, then rounds the
 *       committed state to Q15. No floating-point conversion is performed.
 */
typedef struct {
    linears_q15_t alpha; /**< Dimensionless input weight in [0,1]. */
    linears_q15_t value; /**< Last output in input units, with its own exponent. */
} filters_lowpass_q15_t;

/**
 * @brief Initialize a Q15 low-pass without external storage.
 *
 * @param[out] ctx Non-NULL caller-owned context; no prior initialization needed.
 * @param[in] alpha Dimensionless input weight in [0,1], with a valid exponent.
 * @param[in] initial Initial output in input units, with a valid exponent.
 *
 * @return linears_status_t Status of the initialization.
 *
 * @retval LINEARS_OK Context initialized.
 * @retval LINEARS_ERROR_ARGUMENT ctx is NULL.
 * @retval LINEARS_ERROR_DOMAIN alpha is outside [0,1].
 * @retval LINEARS_ERROR_RANGE An exponent or normalized value is out of range.
 *
 * @note Failure leaves ctx unchanged. Serialize concurrent use of ctx.
 */
linears_status_t filters_lowpass_q15_init(filters_lowpass_q15_t *ctx, linears_q15_t alpha, linears_q15_t initial);

/**
 * @brief Replace Q15 low-pass output history, preserving alpha.
 *
 * @param[in,out] ctx Non-NULL successfully initialized context.
 * @param[in] initial New initial output in input units, with a valid exponent.
 *
 * @return linears_status_t Status of the reset.
 *
 * @retval LINEARS_OK History replaced.
 * @retval LINEARS_ERROR_ARGUMENT ctx is NULL.
 * @retval LINEARS_ERROR_RANGE initial cannot be normalized in the exponent range.
 *
 * @note Failure leaves ctx unchanged. Serialize concurrent use of ctx.
 */
linears_status_t filters_lowpass_q15_reset(filters_lowpass_q15_t *ctx, linears_q15_t initial);

/**
 * @brief Filter one Q15 sample using pure integer arithmetic.
 *
 * @param[in,out] ctx Non-NULL successfully initialized context.
 * @param[in] input Input sample in initial-output units, with a valid exponent.
 * @param[out] output Non-NULL Q15 scalar; must not overlap ctx.
 *
 * @return linears_status_t Status of the sample operation.
 *
 * @retval LINEARS_OK Q15 state and output committed with a normalized exponent.
 * @retval LINEARS_ERROR_ARGUMENT ctx or output is NULL.
 * @retval LINEARS_ERROR_ALIAS output overlaps ctx.
 * @retval LINEARS_ERROR_RANGE An input, intermediate or output exponent is out of range.
 *
 * @note Failure leaves state and output unchanged. Serialize use of ctx.
 */
linears_status_t filters_lowpass_q15_step(filters_lowpass_q15_t *ctx, linears_q15_t input, linears_q15_t *output);

/**
 * @brief Q31 low-pass state using value = mantissa * 2^(exponent-31).
 *
 * @note Arithmetic uses the shared linears pure-integer normalized Q31 core.
 */
typedef struct {
    linears_q31_t alpha; /**< Dimensionless input weight in [0,1]. */
    linears_q31_t value; /**< Last output in input units, with its own exponent. */
} filters_lowpass_q31_t;

/**
 * @brief Initialize a Q31 low-pass without external storage.
 *
 * @param[out] ctx Non-NULL caller-owned context; no prior initialization needed.
 * @param[in] alpha Dimensionless input weight in [0,1], with a valid exponent.
 * @param[in] initial Initial output in input units, with a valid exponent.
 *
 * @return linears_status_t Status of the initialization.
 *
 * @retval LINEARS_OK Context initialized.
 * @retval LINEARS_ERROR_ARGUMENT ctx is NULL.
 * @retval LINEARS_ERROR_DOMAIN alpha is outside [0,1].
 * @retval LINEARS_ERROR_RANGE An exponent or normalized value is out of range.
 *
 * @note Failure leaves ctx unchanged. Serialize concurrent use of ctx.
 */
linears_status_t filters_lowpass_q31_init(filters_lowpass_q31_t *ctx, linears_q31_t alpha, linears_q31_t initial);

/**
 * @brief Replace Q31 low-pass output history, preserving alpha.
 *
 * @param[in,out] ctx Non-NULL successfully initialized context.
 * @param[in] initial New initial output in input units, with a valid exponent.
 *
 * @return linears_status_t Status of the reset.
 *
 * @retval LINEARS_OK History replaced.
 * @retval LINEARS_ERROR_ARGUMENT ctx is NULL.
 * @retval LINEARS_ERROR_RANGE initial cannot be normalized in the exponent range.
 *
 * @note Failure leaves ctx unchanged. Serialize concurrent use of ctx.
 */
linears_status_t filters_lowpass_q31_reset(filters_lowpass_q31_t *ctx, linears_q31_t initial);

/**
 * @brief Filter one Q31 sample using pure integer arithmetic.
 *
 * @param[in,out] ctx Non-NULL successfully initialized context.
 * @param[in] input Input sample in initial-output units, with a valid exponent.
 * @param[out] output Non-NULL Q31 scalar; must not overlap ctx.
 *
 * @return linears_status_t Status of the sample operation.
 *
 * @retval LINEARS_OK State and output committed with a normalized exponent.
 * @retval LINEARS_ERROR_ARGUMENT ctx or output is NULL.
 * @retval LINEARS_ERROR_ALIAS output overlaps ctx.
 * @retval LINEARS_ERROR_RANGE An input, intermediate or output exponent is out of range.
 *
 * @note Failure leaves state and output unchanged. Serialize use of ctx.
 */
linears_status_t filters_lowpass_q31_step(filters_lowpass_q31_t *ctx, linears_q31_t input, linears_q31_t *output);

#ifdef __cplusplus
}
#endif
#endif
