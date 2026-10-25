/**
 * @file filters_lowpass.h
 * @brief First-order low-pass filtering with explicit caller-owned state.
 *
 * A successful step computes y[n] = (1-alpha)*y[n-1] + alpha*x[n].
 * Floating alpha is dimensionless in [0,1]; one passes through and zero holds.
 * Fixed alpha is raw Q1.15/Q1.31 in [0,1); MAX is 1-LSB, not exact one.
 * For a backward-Euler RC low-pass, select alpha = dt/(tau+dt), with dt and
 * tau in the same time unit. This API does not perform that conversion.
 * Fixed signals are int16_t/F15 or int32_t/F31. Recursive state is int64_t/F31
 * with an invariant binary point; Q15 outputs do not truncate that state.
 *
 * Contexts must be initialized before step/reset. Do not manually edit fields;
 * filters_lowpass_work_step may explicitly advance a Q context's value field.
 * Calls on one context require serialization, including interrupts.
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
 * @brief Caller-owned Q1.15 low-pass with a fixed F31 recursive state.
 *
 * Signals and alpha are raw signed integers divided by 2^15. The state is
 * int64_t divided by 2^31; no scaling metadata or floating conversion is used.
 * Output is rounded separately, nearest with ties away from zero. Exact +1
 * maps to INT16_MAX under the shared linears endpoint policy.
 */
typedef struct {
    linears_q15_t alpha; /**< Input weight raw/2^15, in [0,1); MAX is not one. */
    linears_fixed_t value; /**< Private recursive value, int64_t raw/2^31 in input units. */
} filters_lowpass_q15_t;

/**
 * @brief Initialize a Q1.15 low-pass with caller-selected history.
 *
 * @param[out] ctx Non-NULL caller-owned context; unchanged on failure.
 * @param[in] alpha Nonnegative raw Q1.15 input weight; MAX represents 1-2^-15.
 * @param[in] initial Raw Q1.15 initial output in signal units.
 *
 * @return linears_status_t Initialization status.
 * @retval LINEARS_OK Context initialized; history converted exactly to F31.
 * @retval LINEARS_ERROR_ARGUMENT ctx is NULL.
 * @retval LINEARS_ERROR_DOMAIN alpha is negative.
 *
 * @note Retain ctx for its lifetime and serialize all calls, including interrupts.
 */
linears_status_t filters_lowpass_q15_init(filters_lowpass_q15_t *ctx, linears_q15_t alpha, linears_q15_t initial);

/**
 * @brief Replace Q1.15 low-pass history without changing alpha.
 *
 * @param[in,out] ctx Non-NULL successfully initialized context.
 * @param[in] initial Raw Q1.15 initial signal, converted exactly to F31 state.
 *
 * @return linears_status_t Reset status.
 * @retval LINEARS_OK History replaced.
 * @retval LINEARS_ERROR_ARGUMENT ctx is NULL.
 *
 * @note Caller serializes shared use; no allocation, locking or hardware access.
 */
linears_status_t filters_lowpass_q15_reset(filters_lowpass_q15_t *ctx, linears_q15_t initial);

/**
 * @brief Filter a Q1.15 sample with F31 state and pure integer arithmetic.
 *
 * @param[in,out] ctx Non-NULL initialized context; history may also be advanced by work_step.
 * @param[in] input Raw Q1.15 sample in the same physical units as the state.
 * @param[out] output Non-NULL raw Q1.15 destination; must not overlap ctx.
 *
 * @return linears_status_t Sample status.
 * @retval LINEARS_OK F31 state and separately quantized output committed.
 * @retval LINEARS_ERROR_ARGUMENT ctx or output is NULL.
 * @retval LINEARS_ERROR_ALIAS output overlaps ctx.
 * @retval LINEARS_ERROR_RANGE A wider work-step history produces output outside [-1,+1].
 *
 * @note Failures preserve context and output. Serialize calls sharing ctx.
 *       Raw init/reset/step alone keep the convex state in the signal interval;
 *       work_step may retain wider history. MAX alpha is not exact one.
 */
linears_status_t filters_lowpass_q15_step(filters_lowpass_q15_t *ctx, linears_q15_t input, linears_q15_t *output);

/**
 * @brief Caller-owned Q1.31 low-pass with a fixed F31 recursive state.
 *
 * Signals and alpha are raw signed integers divided by 2^31. The state is
 * int64_t divided by 2^31; no scaling metadata or floating conversion is used.
 * Output is rounded separately, nearest with ties away from zero. Exact +1
 * maps to INT32_MAX under the shared linears endpoint policy.
 */
typedef struct {
    linears_q31_t alpha; /**< Input weight raw/2^31, in [0,1); MAX is not one. */
    linears_fixed_t value; /**< Private recursive value, int64_t raw/2^31 in input units. */
} filters_lowpass_q31_t;

/**
 * @brief Initialize a Q1.31 low-pass with caller-selected history.
 *
 * @param[out] ctx Non-NULL caller-owned context; unchanged on failure.
 * @param[in] alpha Nonnegative raw Q1.31 input weight; MAX represents 1-2^-31.
 * @param[in] initial Raw Q1.31 initial output in signal units.
 *
 * @return linears_status_t Initialization status.
 * @retval LINEARS_OK Context initialized; history converted exactly to F31.
 * @retval LINEARS_ERROR_ARGUMENT ctx is NULL.
 * @retval LINEARS_ERROR_DOMAIN alpha is negative.
 *
 * @note Retain ctx for its lifetime and serialize all calls, including interrupts.
 */
linears_status_t filters_lowpass_q31_init(filters_lowpass_q31_t *ctx, linears_q31_t alpha, linears_q31_t initial);

/**
 * @brief Replace Q1.31 low-pass history without changing alpha.
 *
 * @param[in,out] ctx Non-NULL successfully initialized context.
 * @param[in] initial Raw Q1.31 initial signal, converted exactly to F31 state.
 *
 * @return linears_status_t Reset status.
 * @retval LINEARS_OK History replaced.
 * @retval LINEARS_ERROR_ARGUMENT ctx is NULL.
 *
 * @note Caller serializes shared use; no allocation, locking or hardware access.
 */
linears_status_t filters_lowpass_q31_reset(filters_lowpass_q31_t *ctx, linears_q31_t initial);

/**
 * @brief Filter a Q1.31 sample with F31 state and pure integer arithmetic.
 *
 * @param[in,out] ctx Non-NULL initialized context; history may also be advanced by work_step.
 * @param[in] input Raw Q1.31 sample in the same physical units as the state.
 * @param[out] output Non-NULL raw Q1.31 destination; must not overlap ctx.
 *
 * @return linears_status_t Sample status.
 * @retval LINEARS_OK F31 state and separately quantized output committed.
 * @retval LINEARS_ERROR_ARGUMENT ctx or output is NULL.
 * @retval LINEARS_ERROR_ALIAS output overlaps ctx.
 * @retval LINEARS_ERROR_RANGE A wider work-step history produces output outside [-1,+1].
 *
 * @note Failures preserve context and output. Serialize calls sharing ctx.
 *       Raw init/reset/step alone keep the convex state in the signal interval;
 *       work_step may retain wider history. MAX alpha is not exact one.
 */
linears_status_t filters_lowpass_q31_step(filters_lowpass_q31_t *ctx, linears_q31_t input, linears_q31_t *output);

/**
 * @brief Advance a wide F31 low-pass history without narrowing it to Q1.
 *
 * This is the common integer recurrence used by both Q low-pass APIs and
 * observer working states. It computes (1-alpha)*state + alpha*input with an
 * exact F62 product sum, then rounds once to the invariant F31 binary point.
 *
 * @param[in,out] state Non-NULL caller-owned int64_t/F31 history; initialize it
 *                     before calling. Values outside the Q1 signal interval
 *                     are allowed and remain F31, not dynamically rescaled.
 * @param[in] alpha Nonnegative raw Q1.31 weight. INT32_MAX means 1-2^-31, not 1.
 * @param[in] input int64_t/F31 sample in the same physical units as state.
 *
 * @return linears_status_t Update status.
 * @retval LINEARS_OK Rounded F31 history committed.
 * @retval LINEARS_ERROR_ARGUMENT state is NULL.
 * @retval LINEARS_ERROR_DOMAIN alpha is negative.
 *
 * @note Failure leaves state unchanged. Serialize use of the same state,
 *       including interrupt callers; disjoint states are reentrant. No I/O,
 *       allocation, lock or hardware access occurs. Input is passed by value.
 */
linears_status_t filters_lowpass_work_step(linears_fixed_t *state,
                                          linears_q31_t alpha,
                                          linears_fixed_t input);

#ifdef __cplusplus
}
#endif
#endif
