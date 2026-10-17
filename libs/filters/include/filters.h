/**
 * @file filters.h
 * @brief Allocation-free C99 finite-state signal filters in four numeric formats.
 *
 * All contexts and retained arrays are caller-owned and naturally aligned.
 * Init stores pointers but never allocates: arrays and immutable coefficients
 * must remain alive until the context is no longer used. Only the required
 * prefix of a larger capacity is used. Do not edit an initialized context,
 * coefficients, or its retained state buffers; use reset to clear history.
 *
 * Every non-OK return preserves persistent state and output. Temporary scratch
 * may change after a failed step. Output must not overlap any retained buffer
 * or context. Exact and partial overlaps are both rejected. Single-context
 * calls require caller serialization, including interrupts; disjoint contexts
 * are reentrant. There is no MMIO, I/O, allocation, recursion or mutable static
 * state. The call completes synchronously with bounded, size-dependent work.
 *
 * f32/f64 use their native arithmetic and reject NaN/Inf. q15/q31 scalars are
 * linears_q15_t/linears_q31_t, with value mantissa*2^(exponent-15/31). The shared
 * pure-integer linears_fixed_t core performs intermediate arithmetic, then
 * rounds persistent values to the selected storage precision. Coefficients
 * may be negative or larger than one unless a specific range is documented.
 */
#ifndef FILTERS_H
#define FILTERS_H

#include "filters_lowpass.h"
#include "filters_kalman.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Required FIR state elements for taps coefficients.
 *
 * @param[in] taps Positive tap count, without expression side effects.
 *
 * @return size_t State count; init also checks multiplication by element size.
 */
#define FILTERS_FIR_STATE_COUNT(taps) ((size_t)(taps))

/**
 * @brief Required transposed-DFII state elements for sections biquads.
 *
 * @param[in] sections Positive section count, without expression side effects.
 *
 * @return size_t Twice sections, or SIZE_MAX for size overflow (invalid).
 */
#define FILTERS_BIQUAD_STATE_COUNT(sections) LINEARS_SIZE_MUL(2, (sections))

/**
 * @brief Required tentative-state elements for sections cascaded biquads.
 *
 * @param[in] sections Positive section count, without expression side effects.
 *
 * @return size_t Twice sections, or SIZE_MAX for size overflow (invalid).
 */
#define FILTERS_BIQUAD_WORKSPACE_COUNT(sections) FILTERS_BIQUAD_STATE_COUNT(sections)

/**
 * @brief Required moving-average ring elements for a window of samples.
 *
 * @param[in] window Positive sample count, without expression side effects.
 *
 * @return size_t State count; init also checks the element byte count.
 */
#define FILTERS_MOVING_AVERAGE_STATE_COUNT(window) ((size_t)(window))

/**
 * @brief Required median ring elements for a window of samples.
 *
 * @param[in] window Positive sample count, without expression side effects.
 *
 * @return size_t State count; init also checks the element byte count.
 */
#define FILTERS_MEDIAN_STATE_COUNT(window) ((size_t)(window))

/**
 * @brief Required median sorting scratch elements for a window of samples.
 *
 * @param[in] window Positive sample count, without expression side effects.
 *
 * @return size_t Scratch count; init also checks the element byte count.
 */
#define FILTERS_MEDIAN_WORKSPACE_COUNT(window) ((size_t)(window))

/* F32 interface. */

/**
 * @brief Caller-owned FIR delay ring and immutable coefficient reference.
 */
typedef struct {
    const float *coefficients; /**< h[0] multiplies the current input; length taps. */
    float *state; /**< Private zero-padded delay ring; length taps. */
    size_t taps; /**< Positive coefficient and history length, in samples. */
    size_t head; /**< Next replaced ring index; maintained by the API. */
} filters_fir_f32_t;

/**
 * @brief Normalized biquad coefficients: denominator 1 + a1*z^-1 + a2*z^-2.
 */
typedef struct {
    float b0; /**< Current-input feedforward coefficient, dimensionless. */
    float b1; /**< One-delay feedforward coefficient, dimensionless. */
    float b2; /**< Two-delay feedforward coefficient, dimensionless. */
    float a1; /**< One-delay denominator coefficient, subtracted in recurrence. */
    float a2; /**< Two-delay denominator coefficient, subtracted in recurrence. */
} filters_biquad_f32_coefficients_t;

/**
 * @brief Caller-owned SOS cascade with transactional tentative-state scratch.
 */
typedef struct {
    const filters_biquad_f32_coefficients_t *coefficients; /**< Immutable sections array. */
    float *state; /**< Private pairs (s1,s2), length 2*sections. */
    float *workspace; /**< Tentative states, length 2*sections; not persistent. */
    size_t sections; /**< Positive number of cascaded second-order sections. */
} filters_biquad_f32_t;

/**
 * @brief Caller-owned first-order high-pass context.
 */
typedef struct {
    float alpha; /**< Dimensionless recurrence multiplier in [0,1]. */
    float previous_input; /**< Previous sample in input units. */
    float value; /**< Last output in input units. */
} filters_highpass_f32_t;

/**
 * @brief Caller-owned zero-padded moving-average delay ring.
 */
typedef struct {
    float *state; /**< Private sample ring, length window. */
    size_t window; /**< Positive averaging-window length, in samples. */
    size_t head; /**< Next replaced ring index; maintained by the API. */
} filters_moving_average_f32_t;

/**
 * @brief Caller-owned median delay ring and sorting scratch.
 */
typedef struct {
    float *state; /**< Private sample ring, length window. */
    float *workspace; /**< Sorting scratch, length window. */
    size_t window; /**< Positive window length, in samples; even lengths allowed. */
    size_t head; /**< Next replaced ring index; maintained by the API. */
} filters_median_f32_t;

/**
 * @brief Caller-owned linear complementary rate/measurement filter.
 */
typedef struct {
    float alpha; /**< Dimensionless prediction weight in [0,1]. */
    float sample_period; /**< Fixed positive interval between steps, in seconds. */
    float value; /**< Last estimate in the measurement physical unit. */
} filters_complementary_f32_t;

/**
 * @brief F32 exponential moving average; identical recurrence to low-pass.
 */
typedef filters_lowpass_f32_t filters_ema_f32_t;

/**
 * @brief Initialize a F32 fir context.
 *
 * Coefficients, state and context must be pairwise disjoint. Arrays remain
 * valid for the context lifetime. Coefficient signs and gains are unrestricted.
 *
 * @param[out] ctx Non-NULL caller-owned context; unchanged on failure.
 * @param[in] coefficients Non-NULL immutable tap array in increasing delay order.
 * @param[in] taps Positive number of coefficients, in samples.
 * @param[out] state Non-NULL history array, zero-filled in its required prefix.
 * @param[in] state_capacity Available state elements; at least FILTERS_FIR_STATE_COUNT(taps).
 *
 * @return linears_status_t Operation status.
 *
 * @retval LINEARS_OK Context initialized and documented history cleared.
 * @retval LINEARS_ERROR_ARGUMENT A required pointer is NULL or a count is zero.
 * @retval LINEARS_ERROR_RANGE A scalar is nonfinite or an exponent cannot be represented.
 * @retval LINEARS_ERROR_SIZE A required byte/count size overflows.
 * @retval LINEARS_ERROR_WORKSPACE A supplied element capacity is too small.
 * @retval LINEARS_ERROR_ALIAS Live context/array prefixes overlap.
 *
 * @note All numeric values must be valid for the chosen format. Serialize use of ctx.
 */
linears_status_t filters_fir_f32_init(filters_fir_f32_t *ctx, const float *coefficients, size_t taps, float *state, size_t state_capacity);

/**
 * @brief Reset a F32 fir history.
 *
 * @param[in,out] ctx Non-NULL initialized context; coefficients and storage are preserved.
 *
 * @return linears_status_t Operation status.
 *
 * @retval LINEARS_OK History replaced (zero-filled for window/cascade filters).
 * @retval LINEARS_ERROR_ARGUMENT ctx is NULL.
 *
 * @note Failure preserves state. Caller must serialize use of ctx; scratch is not cleared.
 */
linears_status_t filters_fir_f32_reset(filters_fir_f32_t *ctx);

/**
 * @brief Process one F32 fir sample.
 *
 * @param[in,out] ctx Non-NULL successfully initialized context.
 * @param[in] input One sample in the physical units used for the state.
 * @param[out] output Non-NULL scalar; must not overlap ctx or any retained live array prefix.
 *
 * @return linears_status_t Operation status.
 *
 * @retval LINEARS_OK Output and complete persistent state committed.
 * @retval LINEARS_ERROR_ARGUMENT ctx or output is NULL.
 * @retval LINEARS_ERROR_ALIAS output overlaps a retained live buffer or ctx.
 * @retval LINEARS_ERROR_RANGE A scalar/intermediate is nonfinite or outside the exponent range.
 *
 * @note Failure preserves all persistent state and output; scratch may change. Serialize ctx.
 */
linears_status_t filters_fir_f32_step(filters_fir_f32_t *ctx, float input, float *output);

/**
 * @brief Initialize a F32 biquad context.
 *
 * All context/array live prefixes must be pairwise disjoint. Arrays remain
 * valid for the context lifetime. Stability and scaling are caller choices.
 * Each section computes y=b0*x+s1, s1=b1*x-a1*y+s2, s2=b2*x-a2*y.
 * The section output and state are rounded to this API storage precision.
 *
 * @param[out] ctx Non-NULL caller-owned context; unchanged on failure.
 * @param[in] coefficients Non-NULL immutable normalized SOS array; a0 is implicitly one.
 * @param[in] sections Positive number of cascaded sections.
 * @param[out] state Non-NULL state array, zero-filled; 2*sections elements are used.
 * @param[in] state_capacity Available state elements; at least FILTERS_BIQUAD_STATE_COUNT(sections).
 * @param[in,out] workspace Non-NULL tentative-state array; contents are unspecified after step.
 * @param[in] workspace_capacity Scratch elements; at least FILTERS_BIQUAD_WORKSPACE_COUNT(sections).
 *
 * @return linears_status_t Operation status.
 *
 * @retval LINEARS_OK Context initialized and documented history cleared.
 * @retval LINEARS_ERROR_ARGUMENT A required pointer is NULL or a count is zero.
 * @retval LINEARS_ERROR_RANGE A scalar is nonfinite or an exponent cannot be represented.
 * @retval LINEARS_ERROR_SIZE A required byte/count size overflows.
 * @retval LINEARS_ERROR_WORKSPACE A supplied element capacity is too small.
 * @retval LINEARS_ERROR_ALIAS Live context/array prefixes overlap.
 *
 * @note All numeric values must be valid for the chosen format. Serialize use of ctx.
 */
linears_status_t filters_biquad_f32_init(filters_biquad_f32_t *ctx, const filters_biquad_f32_coefficients_t *coefficients, size_t sections, float *state, size_t state_capacity, float *workspace, size_t workspace_capacity);

/**
 * @brief Reset a F32 biquad history.
 *
 * @param[in,out] ctx Non-NULL initialized context; coefficients and storage are preserved.
 *
 * @return linears_status_t Operation status.
 *
 * @retval LINEARS_OK History replaced (zero-filled for window/cascade filters).
 * @retval LINEARS_ERROR_ARGUMENT ctx is NULL.
 *
 * @note Failure preserves state. Caller must serialize use of ctx; scratch is not cleared.
 */
linears_status_t filters_biquad_f32_reset(filters_biquad_f32_t *ctx);

/**
 * @brief Process one F32 biquad sample.
 *
 * @param[in,out] ctx Non-NULL successfully initialized context.
 * @param[in] input One sample in the physical units used for the state.
 * @param[out] output Non-NULL scalar; must not overlap ctx or any retained live array prefix.
 *
 * @return linears_status_t Operation status.
 *
 * @retval LINEARS_OK Output and complete persistent state committed.
 * @retval LINEARS_ERROR_ARGUMENT ctx or output is NULL.
 * @retval LINEARS_ERROR_ALIAS output overlaps a retained live buffer or ctx.
 * @retval LINEARS_ERROR_RANGE A scalar/intermediate is nonfinite or outside the exponent range.
 *
 * @note Failure preserves all persistent state and output; scratch may change. Serialize ctx.
 */
linears_status_t filters_biquad_f32_step(filters_biquad_f32_t *ctx, float input, float *output);

/**
 * @brief Initialize a F32 highpass context.
 *
 * The recurrence is y=alpha*(previous_y+input-previous_input).
 * For a backward-Euler RC high-pass choose alpha=tau/(tau+dt).
 *
 * @param[out] ctx Non-NULL caller-owned context; unchanged on failure.
 * @param[in] alpha Dimensionless weight in [0,1].
 * @param[in] initial_input Previous input sample in signal units.
 * @param[in] initial_output Previous output sample in signal units.
 *
 * @return linears_status_t Operation status.
 *
 * @retval LINEARS_OK Context initialized and documented history cleared.
 * @retval LINEARS_ERROR_ARGUMENT A required pointer is NULL or a count is zero.
 * @retval LINEARS_ERROR_RANGE A scalar is nonfinite or an exponent cannot be represented.
 * @retval LINEARS_ERROR_DOMAIN alpha is outside [0,1].
 *
 * @note All numeric values must be valid for the chosen format. Serialize use of ctx.
 */
linears_status_t filters_highpass_f32_init(filters_highpass_f32_t *ctx, float alpha, float initial_input, float initial_output);

/**
 * @brief Reset a F32 highpass history.
 *
 * @param[in,out] ctx Non-NULL initialized context; coefficients and storage are preserved.
 * @param[in] initial_input New previous input, in signal units.
 * @param[in] initial_output New previous output, in signal units.
 *
 * @return linears_status_t Operation status.
 *
 * @retval LINEARS_OK History replaced (zero-filled for window/cascade filters).
 * @retval LINEARS_ERROR_ARGUMENT ctx is NULL.
 * @retval LINEARS_ERROR_RANGE An initial value is nonfinite or outside the exponent range.
 *
 * @note Failure preserves state. Caller must serialize use of ctx; scratch is not cleared.
 */
linears_status_t filters_highpass_f32_reset(filters_highpass_f32_t *ctx, float initial_input, float initial_output);

/**
 * @brief Process one F32 highpass sample.
 *
 * @param[in,out] ctx Non-NULL successfully initialized context.
 * @param[in] input One sample in the physical units used for the state.
 * @param[out] output Non-NULL scalar; must not overlap ctx or any retained live array prefix.
 *
 * @return linears_status_t Operation status.
 *
 * @retval LINEARS_OK Output and complete persistent state committed.
 * @retval LINEARS_ERROR_ARGUMENT ctx or output is NULL.
 * @retval LINEARS_ERROR_ALIAS output overlaps a retained live buffer or ctx.
 * @retval LINEARS_ERROR_RANGE A scalar/intermediate is nonfinite or outside the exponent range.
 *
 * @note Failure preserves all persistent state and output; scratch may change. Serialize ctx.
 */
linears_status_t filters_highpass_f32_step(filters_highpass_f32_t *ctx, float input, float *output);

/**
 * @brief Initialize a F32 moving average context.
 *
 * The context and all live array prefixes must be pairwise disjoint.
 * Arrays remain valid for the context lifetime. Startup samples outside
 * the available history are zeros; the divisor/window length stays fixed.
 *
 * @param[out] ctx Non-NULL caller-owned context; unchanged on failure.
 * @param[in] window Positive window length, in samples.
 * @param[out] state Non-NULL sample ring; its window-element prefix is zero-filled.
 * @param[in] state_capacity Available state elements; must be at least window.
 *
 * @return linears_status_t Operation status.
 *
 * @retval LINEARS_OK Context initialized and documented history cleared.
 * @retval LINEARS_ERROR_ARGUMENT A required pointer is NULL or a count is zero.
 * @retval LINEARS_ERROR_SIZE A required byte/count size overflows.
 * @retval LINEARS_ERROR_WORKSPACE A supplied element capacity is too small.
 * @retval LINEARS_ERROR_ALIAS Live context/array prefixes overlap.
 *
 * @note All numeric values must be valid for the chosen format. Serialize use of ctx.
 */
linears_status_t filters_moving_average_f32_init(filters_moving_average_f32_t *ctx, size_t window, float *state, size_t state_capacity);

/**
 * @brief Reset a F32 moving average history.
 *
 * @param[in,out] ctx Non-NULL initialized context; coefficients and storage are preserved.
 *
 * @return linears_status_t Operation status.
 *
 * @retval LINEARS_OK History replaced (zero-filled for window/cascade filters).
 * @retval LINEARS_ERROR_ARGUMENT ctx is NULL.
 *
 * @note Failure preserves state. Caller must serialize use of ctx; scratch is not cleared.
 */
linears_status_t filters_moving_average_f32_reset(filters_moving_average_f32_t *ctx);

/**
 * @brief Process one F32 moving average sample.
 *
 * Recomputes the window mean in O(window) work to avoid accumulated
 * running-sum drift. Each term is divided before the sum is formed.
 *
 * @param[in,out] ctx Non-NULL successfully initialized context.
 * @param[in] input One sample in the physical units used for the state.
 * @param[out] output Non-NULL scalar; must not overlap ctx or any retained live array prefix.
 *
 * @return linears_status_t Operation status.
 *
 * @retval LINEARS_OK Output and complete persistent state committed.
 * @retval LINEARS_ERROR_ARGUMENT ctx or output is NULL.
 * @retval LINEARS_ERROR_ALIAS output overlaps a retained live buffer or ctx.
 * @retval LINEARS_ERROR_RANGE A scalar/intermediate is nonfinite or outside the exponent range.
 *
 * @note Failure preserves all persistent state and output; scratch may change. Serialize ctx.
 */
linears_status_t filters_moving_average_f32_step(filters_moving_average_f32_t *ctx, float input, float *output);

/**
 * @brief Initialize a F32 median context.
 *
 * The context and all live array prefixes must be pairwise disjoint.
 * Arrays remain valid for the context lifetime. Startup samples outside
 * the available history are zeros; the divisor/window length stays fixed.
 *
 * @param[out] ctx Non-NULL caller-owned context; unchanged on failure.
 * @param[in] window Positive window length, in samples.
 * @param[out] state Non-NULL sample ring; its window-element prefix is zero-filled.
 * @param[in] state_capacity Available state elements; must be at least window.
 * @param[in,out] workspace Non-NULL window-element sorting scratch, not initialized here.
 * @param[in] workspace_capacity Available scratch elements; must be at least window.
 *
 * @return linears_status_t Operation status.
 *
 * @retval LINEARS_OK Context initialized and documented history cleared.
 * @retval LINEARS_ERROR_ARGUMENT A required pointer is NULL or a count is zero.
 * @retval LINEARS_ERROR_SIZE A required byte/count size overflows.
 * @retval LINEARS_ERROR_WORKSPACE A supplied element capacity is too small.
 * @retval LINEARS_ERROR_ALIAS Live context/array prefixes overlap.
 *
 * @note All numeric values must be valid for the chosen format. Serialize use of ctx.
 */
linears_status_t filters_median_f32_init(filters_median_f32_t *ctx, size_t window, float *state, size_t state_capacity, float *workspace, size_t workspace_capacity);

/**
 * @brief Reset a F32 median history.
 *
 * @param[in,out] ctx Non-NULL initialized context; coefficients and storage are preserved.
 *
 * @return linears_status_t Operation status.
 *
 * @retval LINEARS_OK History replaced (zero-filled for window/cascade filters).
 * @retval LINEARS_ERROR_ARGUMENT ctx is NULL.
 *
 * @note Failure preserves state. Caller must serialize use of ctx; scratch is not cleared.
 */
linears_status_t filters_median_f32_reset(filters_median_f32_t *ctx);

/**
 * @brief Process one F32 median sample.
 *
 * Returns the central sorted sample, or the mean of both central values
 * for an even window. Sorting is O(window^2) with caller-owned scratch.
 *
 * @param[in,out] ctx Non-NULL successfully initialized context.
 * @param[in] input One sample in the physical units used for the state.
 * @param[out] output Non-NULL scalar; must not overlap ctx or any retained live array prefix.
 *
 * @return linears_status_t Operation status.
 *
 * @retval LINEARS_OK Output and complete persistent state committed.
 * @retval LINEARS_ERROR_ARGUMENT ctx or output is NULL.
 * @retval LINEARS_ERROR_ALIAS output overlaps a retained live buffer or ctx.
 * @retval LINEARS_ERROR_RANGE A scalar/intermediate is nonfinite or outside the exponent range.
 *
 * @note Failure preserves all persistent state and output; scratch may change. Serialize ctx.
 */
linears_status_t filters_median_f32_step(filters_median_f32_t *ctx, float input, float *output);

/**
 * @brief Initialize a F32 ema context.
 *
 * The recurrence is y=(1-alpha)*previous_y+alpha*input.
 * This API delegates to the same implementation as the first-order low-pass.
 *
 * @param[out] ctx Non-NULL caller-owned context; unchanged on failure.
 * @param[in] alpha Dimensionless weight in [0,1].
 * @param[in] initial Initial estimate/output in signal units.
 *
 * @return linears_status_t Operation status.
 *
 * @retval LINEARS_OK Context initialized and documented history cleared.
 * @retval LINEARS_ERROR_ARGUMENT A required pointer is NULL or a count is zero.
 * @retval LINEARS_ERROR_RANGE A scalar is nonfinite or an exponent cannot be represented.
 * @retval LINEARS_ERROR_DOMAIN alpha is outside [0,1].
 *
 * @note All numeric values must be valid for the chosen format. Serialize use of ctx.
 */
linears_status_t filters_ema_f32_init(filters_ema_f32_t *ctx, float alpha, float initial);

/**
 * @brief Reset a F32 ema history.
 *
 * @param[in,out] ctx Non-NULL initialized context; coefficients and storage are preserved.
 * @param[in] initial New output history, in signal units.
 *
 * @return linears_status_t Operation status.
 *
 * @retval LINEARS_OK History replaced (zero-filled for window/cascade filters).
 * @retval LINEARS_ERROR_ARGUMENT ctx is NULL.
 * @retval LINEARS_ERROR_RANGE An initial value is nonfinite or outside the exponent range.
 *
 * @note Failure preserves state. Caller must serialize use of ctx; scratch is not cleared.
 */
linears_status_t filters_ema_f32_reset(filters_ema_f32_t *ctx, float initial);

/**
 * @brief Process one F32 ema sample.
 *
 * @param[in,out] ctx Non-NULL successfully initialized context.
 * @param[in] input One sample in the physical units used for the state.
 * @param[out] output Non-NULL scalar; must not overlap ctx or any retained live array prefix.
 *
 * @return linears_status_t Operation status.
 *
 * @retval LINEARS_OK Output and complete persistent state committed.
 * @retval LINEARS_ERROR_ARGUMENT ctx or output is NULL.
 * @retval LINEARS_ERROR_ALIAS output overlaps a retained live buffer or ctx.
 * @retval LINEARS_ERROR_RANGE A scalar/intermediate is nonfinite or outside the exponent range.
 *
 * @note Failure preserves all persistent state and output; scratch may change. Serialize ctx.
 */
linears_status_t filters_ema_f32_step(filters_ema_f32_t *ctx, float input, float *output);

/**
 * @brief Initialize a F32 complementary context.
 *
 * The recurrence is y=alpha*(previous_y+rate*sample_period)
 * +(1-alpha)*measurement. No angle wrapping or nonlinear state model is used.
 *
 * @param[out] ctx Non-NULL caller-owned context; unchanged on failure.
 * @param[in] alpha Dimensionless weight in [0,1].
 * @param[in] sample_period Positive fixed sample period, in seconds.
 * @param[in] initial Initial estimate/output in signal units.
 *
 * @return linears_status_t Operation status.
 *
 * @retval LINEARS_OK Context initialized and documented history cleared.
 * @retval LINEARS_ERROR_ARGUMENT A required pointer is NULL or a count is zero.
 * @retval LINEARS_ERROR_RANGE A scalar is nonfinite or an exponent cannot be represented.
 * @retval LINEARS_ERROR_DOMAIN alpha is outside [0,1] or sample_period is not positive.
 *
 * @note All numeric values must be valid for the chosen format. Serialize use of ctx.
 */
linears_status_t filters_complementary_f32_init(filters_complementary_f32_t *ctx, float alpha, float sample_period, float initial);

/**
 * @brief Reset a F32 complementary history.
 *
 * @param[in,out] ctx Non-NULL initialized context; coefficients and storage are preserved.
 * @param[in] initial New output history, in signal units.
 *
 * @return linears_status_t Operation status.
 *
 * @retval LINEARS_OK History replaced (zero-filled for window/cascade filters).
 * @retval LINEARS_ERROR_ARGUMENT ctx is NULL.
 * @retval LINEARS_ERROR_RANGE An initial value is nonfinite or outside the exponent range.
 *
 * @note Failure preserves state. Caller must serialize use of ctx; scratch is not cleared.
 */
linears_status_t filters_complementary_f32_reset(filters_complementary_f32_t *ctx, float initial);

/**
 * @brief Process one F32 complementary sample.
 *
 * @param[in,out] ctx Non-NULL successfully initialized context.
 * @param[in] rate Measured derivative in measurement units per second.
 * @param[in] measurement Absolute observation in the estimate physical unit.
 * @param[out] output Non-NULL scalar; must not overlap ctx or any retained live array prefix.
 *
 * @return linears_status_t Operation status.
 *
 * @retval LINEARS_OK Output and complete persistent state committed.
 * @retval LINEARS_ERROR_ARGUMENT ctx or output is NULL.
 * @retval LINEARS_ERROR_ALIAS output overlaps a retained live buffer or ctx.
 * @retval LINEARS_ERROR_RANGE A scalar/intermediate is nonfinite or outside the exponent range.
 *
 * @note Failure preserves all persistent state and output; scratch may change. Serialize ctx.
 */
linears_status_t filters_complementary_f32_step(filters_complementary_f32_t *ctx, float rate, float measurement, float *output);

/* F64 interface. */

/**
 * @brief Caller-owned FIR delay ring and immutable coefficient reference.
 */
typedef struct {
    const double *coefficients; /**< h[0] multiplies the current input; length taps. */
    double *state; /**< Private zero-padded delay ring; length taps. */
    size_t taps; /**< Positive coefficient and history length, in samples. */
    size_t head; /**< Next replaced ring index; maintained by the API. */
} filters_fir_f64_t;

/**
 * @brief Normalized biquad coefficients: denominator 1 + a1*z^-1 + a2*z^-2.
 */
typedef struct {
    double b0; /**< Current-input feedforward coefficient, dimensionless. */
    double b1; /**< One-delay feedforward coefficient, dimensionless. */
    double b2; /**< Two-delay feedforward coefficient, dimensionless. */
    double a1; /**< One-delay denominator coefficient, subtracted in recurrence. */
    double a2; /**< Two-delay denominator coefficient, subtracted in recurrence. */
} filters_biquad_f64_coefficients_t;

/**
 * @brief Caller-owned SOS cascade with transactional tentative-state scratch.
 */
typedef struct {
    const filters_biquad_f64_coefficients_t *coefficients; /**< Immutable sections array. */
    double *state; /**< Private pairs (s1,s2), length 2*sections. */
    double *workspace; /**< Tentative states, length 2*sections; not persistent. */
    size_t sections; /**< Positive number of cascaded second-order sections. */
} filters_biquad_f64_t;

/**
 * @brief Caller-owned first-order high-pass context.
 */
typedef struct {
    double alpha; /**< Dimensionless recurrence multiplier in [0,1]. */
    double previous_input; /**< Previous sample in input units. */
    double value; /**< Last output in input units. */
} filters_highpass_f64_t;

/**
 * @brief Caller-owned zero-padded moving-average delay ring.
 */
typedef struct {
    double *state; /**< Private sample ring, length window. */
    size_t window; /**< Positive averaging-window length, in samples. */
    size_t head; /**< Next replaced ring index; maintained by the API. */
} filters_moving_average_f64_t;

/**
 * @brief Caller-owned median delay ring and sorting scratch.
 */
typedef struct {
    double *state; /**< Private sample ring, length window. */
    double *workspace; /**< Sorting scratch, length window. */
    size_t window; /**< Positive window length, in samples; even lengths allowed. */
    size_t head; /**< Next replaced ring index; maintained by the API. */
} filters_median_f64_t;

/**
 * @brief Caller-owned linear complementary rate/measurement filter.
 */
typedef struct {
    double alpha; /**< Dimensionless prediction weight in [0,1]. */
    double sample_period; /**< Fixed positive interval between steps, in seconds. */
    double value; /**< Last estimate in the measurement physical unit. */
} filters_complementary_f64_t;

/**
 * @brief F64 exponential moving average; identical recurrence to low-pass.
 */
typedef filters_lowpass_f64_t filters_ema_f64_t;

/**
 * @brief Initialize a F64 fir context.
 *
 * Coefficients, state and context must be pairwise disjoint. Arrays remain
 * valid for the context lifetime. Coefficient signs and gains are unrestricted.
 *
 * @param[out] ctx Non-NULL caller-owned context; unchanged on failure.
 * @param[in] coefficients Non-NULL immutable tap array in increasing delay order.
 * @param[in] taps Positive number of coefficients, in samples.
 * @param[out] state Non-NULL history array, zero-filled in its required prefix.
 * @param[in] state_capacity Available state elements; at least FILTERS_FIR_STATE_COUNT(taps).
 *
 * @return linears_status_t Operation status.
 *
 * @retval LINEARS_OK Context initialized and documented history cleared.
 * @retval LINEARS_ERROR_ARGUMENT A required pointer is NULL or a count is zero.
 * @retval LINEARS_ERROR_RANGE A scalar is nonfinite or an exponent cannot be represented.
 * @retval LINEARS_ERROR_SIZE A required byte/count size overflows.
 * @retval LINEARS_ERROR_WORKSPACE A supplied element capacity is too small.
 * @retval LINEARS_ERROR_ALIAS Live context/array prefixes overlap.
 *
 * @note All numeric values must be valid for the chosen format. Serialize use of ctx.
 */
linears_status_t filters_fir_f64_init(filters_fir_f64_t *ctx, const double *coefficients, size_t taps, double *state, size_t state_capacity);

/**
 * @brief Reset a F64 fir history.
 *
 * @param[in,out] ctx Non-NULL initialized context; coefficients and storage are preserved.
 *
 * @return linears_status_t Operation status.
 *
 * @retval LINEARS_OK History replaced (zero-filled for window/cascade filters).
 * @retval LINEARS_ERROR_ARGUMENT ctx is NULL.
 *
 * @note Failure preserves state. Caller must serialize use of ctx; scratch is not cleared.
 */
linears_status_t filters_fir_f64_reset(filters_fir_f64_t *ctx);

/**
 * @brief Process one F64 fir sample.
 *
 * @param[in,out] ctx Non-NULL successfully initialized context.
 * @param[in] input One sample in the physical units used for the state.
 * @param[out] output Non-NULL scalar; must not overlap ctx or any retained live array prefix.
 *
 * @return linears_status_t Operation status.
 *
 * @retval LINEARS_OK Output and complete persistent state committed.
 * @retval LINEARS_ERROR_ARGUMENT ctx or output is NULL.
 * @retval LINEARS_ERROR_ALIAS output overlaps a retained live buffer or ctx.
 * @retval LINEARS_ERROR_RANGE A scalar/intermediate is nonfinite or outside the exponent range.
 *
 * @note Failure preserves all persistent state and output; scratch may change. Serialize ctx.
 */
linears_status_t filters_fir_f64_step(filters_fir_f64_t *ctx, double input, double *output);

/**
 * @brief Initialize a F64 biquad context.
 *
 * All context/array live prefixes must be pairwise disjoint. Arrays remain
 * valid for the context lifetime. Stability and scaling are caller choices.
 * Each section computes y=b0*x+s1, s1=b1*x-a1*y+s2, s2=b2*x-a2*y.
 * The section output and state are rounded to this API storage precision.
 *
 * @param[out] ctx Non-NULL caller-owned context; unchanged on failure.
 * @param[in] coefficients Non-NULL immutable normalized SOS array; a0 is implicitly one.
 * @param[in] sections Positive number of cascaded sections.
 * @param[out] state Non-NULL state array, zero-filled; 2*sections elements are used.
 * @param[in] state_capacity Available state elements; at least FILTERS_BIQUAD_STATE_COUNT(sections).
 * @param[in,out] workspace Non-NULL tentative-state array; contents are unspecified after step.
 * @param[in] workspace_capacity Scratch elements; at least FILTERS_BIQUAD_WORKSPACE_COUNT(sections).
 *
 * @return linears_status_t Operation status.
 *
 * @retval LINEARS_OK Context initialized and documented history cleared.
 * @retval LINEARS_ERROR_ARGUMENT A required pointer is NULL or a count is zero.
 * @retval LINEARS_ERROR_RANGE A scalar is nonfinite or an exponent cannot be represented.
 * @retval LINEARS_ERROR_SIZE A required byte/count size overflows.
 * @retval LINEARS_ERROR_WORKSPACE A supplied element capacity is too small.
 * @retval LINEARS_ERROR_ALIAS Live context/array prefixes overlap.
 *
 * @note All numeric values must be valid for the chosen format. Serialize use of ctx.
 */
linears_status_t filters_biquad_f64_init(filters_biquad_f64_t *ctx, const filters_biquad_f64_coefficients_t *coefficients, size_t sections, double *state, size_t state_capacity, double *workspace, size_t workspace_capacity);

/**
 * @brief Reset a F64 biquad history.
 *
 * @param[in,out] ctx Non-NULL initialized context; coefficients and storage are preserved.
 *
 * @return linears_status_t Operation status.
 *
 * @retval LINEARS_OK History replaced (zero-filled for window/cascade filters).
 * @retval LINEARS_ERROR_ARGUMENT ctx is NULL.
 *
 * @note Failure preserves state. Caller must serialize use of ctx; scratch is not cleared.
 */
linears_status_t filters_biquad_f64_reset(filters_biquad_f64_t *ctx);

/**
 * @brief Process one F64 biquad sample.
 *
 * @param[in,out] ctx Non-NULL successfully initialized context.
 * @param[in] input One sample in the physical units used for the state.
 * @param[out] output Non-NULL scalar; must not overlap ctx or any retained live array prefix.
 *
 * @return linears_status_t Operation status.
 *
 * @retval LINEARS_OK Output and complete persistent state committed.
 * @retval LINEARS_ERROR_ARGUMENT ctx or output is NULL.
 * @retval LINEARS_ERROR_ALIAS output overlaps a retained live buffer or ctx.
 * @retval LINEARS_ERROR_RANGE A scalar/intermediate is nonfinite or outside the exponent range.
 *
 * @note Failure preserves all persistent state and output; scratch may change. Serialize ctx.
 */
linears_status_t filters_biquad_f64_step(filters_biquad_f64_t *ctx, double input, double *output);

/**
 * @brief Initialize a F64 highpass context.
 *
 * The recurrence is y=alpha*(previous_y+input-previous_input).
 * For a backward-Euler RC high-pass choose alpha=tau/(tau+dt).
 *
 * @param[out] ctx Non-NULL caller-owned context; unchanged on failure.
 * @param[in] alpha Dimensionless weight in [0,1].
 * @param[in] initial_input Previous input sample in signal units.
 * @param[in] initial_output Previous output sample in signal units.
 *
 * @return linears_status_t Operation status.
 *
 * @retval LINEARS_OK Context initialized and documented history cleared.
 * @retval LINEARS_ERROR_ARGUMENT A required pointer is NULL or a count is zero.
 * @retval LINEARS_ERROR_RANGE A scalar is nonfinite or an exponent cannot be represented.
 * @retval LINEARS_ERROR_DOMAIN alpha is outside [0,1].
 *
 * @note All numeric values must be valid for the chosen format. Serialize use of ctx.
 */
linears_status_t filters_highpass_f64_init(filters_highpass_f64_t *ctx, double alpha, double initial_input, double initial_output);

/**
 * @brief Reset a F64 highpass history.
 *
 * @param[in,out] ctx Non-NULL initialized context; coefficients and storage are preserved.
 * @param[in] initial_input New previous input, in signal units.
 * @param[in] initial_output New previous output, in signal units.
 *
 * @return linears_status_t Operation status.
 *
 * @retval LINEARS_OK History replaced (zero-filled for window/cascade filters).
 * @retval LINEARS_ERROR_ARGUMENT ctx is NULL.
 * @retval LINEARS_ERROR_RANGE An initial value is nonfinite or outside the exponent range.
 *
 * @note Failure preserves state. Caller must serialize use of ctx; scratch is not cleared.
 */
linears_status_t filters_highpass_f64_reset(filters_highpass_f64_t *ctx, double initial_input, double initial_output);

/**
 * @brief Process one F64 highpass sample.
 *
 * @param[in,out] ctx Non-NULL successfully initialized context.
 * @param[in] input One sample in the physical units used for the state.
 * @param[out] output Non-NULL scalar; must not overlap ctx or any retained live array prefix.
 *
 * @return linears_status_t Operation status.
 *
 * @retval LINEARS_OK Output and complete persistent state committed.
 * @retval LINEARS_ERROR_ARGUMENT ctx or output is NULL.
 * @retval LINEARS_ERROR_ALIAS output overlaps a retained live buffer or ctx.
 * @retval LINEARS_ERROR_RANGE A scalar/intermediate is nonfinite or outside the exponent range.
 *
 * @note Failure preserves all persistent state and output; scratch may change. Serialize ctx.
 */
linears_status_t filters_highpass_f64_step(filters_highpass_f64_t *ctx, double input, double *output);

/**
 * @brief Initialize a F64 moving average context.
 *
 * The context and all live array prefixes must be pairwise disjoint.
 * Arrays remain valid for the context lifetime. Startup samples outside
 * the available history are zeros; the divisor/window length stays fixed.
 *
 * @param[out] ctx Non-NULL caller-owned context; unchanged on failure.
 * @param[in] window Positive window length, in samples.
 * @param[out] state Non-NULL sample ring; its window-element prefix is zero-filled.
 * @param[in] state_capacity Available state elements; must be at least window.
 *
 * @return linears_status_t Operation status.
 *
 * @retval LINEARS_OK Context initialized and documented history cleared.
 * @retval LINEARS_ERROR_ARGUMENT A required pointer is NULL or a count is zero.
 * @retval LINEARS_ERROR_SIZE A required byte/count size overflows.
 * @retval LINEARS_ERROR_WORKSPACE A supplied element capacity is too small.
 * @retval LINEARS_ERROR_ALIAS Live context/array prefixes overlap.
 *
 * @note All numeric values must be valid for the chosen format. Serialize use of ctx.
 */
linears_status_t filters_moving_average_f64_init(filters_moving_average_f64_t *ctx, size_t window, double *state, size_t state_capacity);

/**
 * @brief Reset a F64 moving average history.
 *
 * @param[in,out] ctx Non-NULL initialized context; coefficients and storage are preserved.
 *
 * @return linears_status_t Operation status.
 *
 * @retval LINEARS_OK History replaced (zero-filled for window/cascade filters).
 * @retval LINEARS_ERROR_ARGUMENT ctx is NULL.
 *
 * @note Failure preserves state. Caller must serialize use of ctx; scratch is not cleared.
 */
linears_status_t filters_moving_average_f64_reset(filters_moving_average_f64_t *ctx);

/**
 * @brief Process one F64 moving average sample.
 *
 * Recomputes the window mean in O(window) work to avoid accumulated
 * running-sum drift. Each term is divided before the sum is formed.
 *
 * @param[in,out] ctx Non-NULL successfully initialized context.
 * @param[in] input One sample in the physical units used for the state.
 * @param[out] output Non-NULL scalar; must not overlap ctx or any retained live array prefix.
 *
 * @return linears_status_t Operation status.
 *
 * @retval LINEARS_OK Output and complete persistent state committed.
 * @retval LINEARS_ERROR_ARGUMENT ctx or output is NULL.
 * @retval LINEARS_ERROR_ALIAS output overlaps a retained live buffer or ctx.
 * @retval LINEARS_ERROR_RANGE A scalar/intermediate is nonfinite or outside the exponent range.
 *
 * @note Failure preserves all persistent state and output; scratch may change. Serialize ctx.
 */
linears_status_t filters_moving_average_f64_step(filters_moving_average_f64_t *ctx, double input, double *output);

/**
 * @brief Initialize a F64 median context.
 *
 * The context and all live array prefixes must be pairwise disjoint.
 * Arrays remain valid for the context lifetime. Startup samples outside
 * the available history are zeros; the divisor/window length stays fixed.
 *
 * @param[out] ctx Non-NULL caller-owned context; unchanged on failure.
 * @param[in] window Positive window length, in samples.
 * @param[out] state Non-NULL sample ring; its window-element prefix is zero-filled.
 * @param[in] state_capacity Available state elements; must be at least window.
 * @param[in,out] workspace Non-NULL window-element sorting scratch, not initialized here.
 * @param[in] workspace_capacity Available scratch elements; must be at least window.
 *
 * @return linears_status_t Operation status.
 *
 * @retval LINEARS_OK Context initialized and documented history cleared.
 * @retval LINEARS_ERROR_ARGUMENT A required pointer is NULL or a count is zero.
 * @retval LINEARS_ERROR_SIZE A required byte/count size overflows.
 * @retval LINEARS_ERROR_WORKSPACE A supplied element capacity is too small.
 * @retval LINEARS_ERROR_ALIAS Live context/array prefixes overlap.
 *
 * @note All numeric values must be valid for the chosen format. Serialize use of ctx.
 */
linears_status_t filters_median_f64_init(filters_median_f64_t *ctx, size_t window, double *state, size_t state_capacity, double *workspace, size_t workspace_capacity);

/**
 * @brief Reset a F64 median history.
 *
 * @param[in,out] ctx Non-NULL initialized context; coefficients and storage are preserved.
 *
 * @return linears_status_t Operation status.
 *
 * @retval LINEARS_OK History replaced (zero-filled for window/cascade filters).
 * @retval LINEARS_ERROR_ARGUMENT ctx is NULL.
 *
 * @note Failure preserves state. Caller must serialize use of ctx; scratch is not cleared.
 */
linears_status_t filters_median_f64_reset(filters_median_f64_t *ctx);

/**
 * @brief Process one F64 median sample.
 *
 * Returns the central sorted sample, or the mean of both central values
 * for an even window. Sorting is O(window^2) with caller-owned scratch.
 *
 * @param[in,out] ctx Non-NULL successfully initialized context.
 * @param[in] input One sample in the physical units used for the state.
 * @param[out] output Non-NULL scalar; must not overlap ctx or any retained live array prefix.
 *
 * @return linears_status_t Operation status.
 *
 * @retval LINEARS_OK Output and complete persistent state committed.
 * @retval LINEARS_ERROR_ARGUMENT ctx or output is NULL.
 * @retval LINEARS_ERROR_ALIAS output overlaps a retained live buffer or ctx.
 * @retval LINEARS_ERROR_RANGE A scalar/intermediate is nonfinite or outside the exponent range.
 *
 * @note Failure preserves all persistent state and output; scratch may change. Serialize ctx.
 */
linears_status_t filters_median_f64_step(filters_median_f64_t *ctx, double input, double *output);

/**
 * @brief Initialize a F64 ema context.
 *
 * The recurrence is y=(1-alpha)*previous_y+alpha*input.
 * This API delegates to the same implementation as the first-order low-pass.
 *
 * @param[out] ctx Non-NULL caller-owned context; unchanged on failure.
 * @param[in] alpha Dimensionless weight in [0,1].
 * @param[in] initial Initial estimate/output in signal units.
 *
 * @return linears_status_t Operation status.
 *
 * @retval LINEARS_OK Context initialized and documented history cleared.
 * @retval LINEARS_ERROR_ARGUMENT A required pointer is NULL or a count is zero.
 * @retval LINEARS_ERROR_RANGE A scalar is nonfinite or an exponent cannot be represented.
 * @retval LINEARS_ERROR_DOMAIN alpha is outside [0,1].
 *
 * @note All numeric values must be valid for the chosen format. Serialize use of ctx.
 */
linears_status_t filters_ema_f64_init(filters_ema_f64_t *ctx, double alpha, double initial);

/**
 * @brief Reset a F64 ema history.
 *
 * @param[in,out] ctx Non-NULL initialized context; coefficients and storage are preserved.
 * @param[in] initial New output history, in signal units.
 *
 * @return linears_status_t Operation status.
 *
 * @retval LINEARS_OK History replaced (zero-filled for window/cascade filters).
 * @retval LINEARS_ERROR_ARGUMENT ctx is NULL.
 * @retval LINEARS_ERROR_RANGE An initial value is nonfinite or outside the exponent range.
 *
 * @note Failure preserves state. Caller must serialize use of ctx; scratch is not cleared.
 */
linears_status_t filters_ema_f64_reset(filters_ema_f64_t *ctx, double initial);

/**
 * @brief Process one F64 ema sample.
 *
 * @param[in,out] ctx Non-NULL successfully initialized context.
 * @param[in] input One sample in the physical units used for the state.
 * @param[out] output Non-NULL scalar; must not overlap ctx or any retained live array prefix.
 *
 * @return linears_status_t Operation status.
 *
 * @retval LINEARS_OK Output and complete persistent state committed.
 * @retval LINEARS_ERROR_ARGUMENT ctx or output is NULL.
 * @retval LINEARS_ERROR_ALIAS output overlaps a retained live buffer or ctx.
 * @retval LINEARS_ERROR_RANGE A scalar/intermediate is nonfinite or outside the exponent range.
 *
 * @note Failure preserves all persistent state and output; scratch may change. Serialize ctx.
 */
linears_status_t filters_ema_f64_step(filters_ema_f64_t *ctx, double input, double *output);

/**
 * @brief Initialize a F64 complementary context.
 *
 * The recurrence is y=alpha*(previous_y+rate*sample_period)
 * +(1-alpha)*measurement. No angle wrapping or nonlinear state model is used.
 *
 * @param[out] ctx Non-NULL caller-owned context; unchanged on failure.
 * @param[in] alpha Dimensionless weight in [0,1].
 * @param[in] sample_period Positive fixed sample period, in seconds.
 * @param[in] initial Initial estimate/output in signal units.
 *
 * @return linears_status_t Operation status.
 *
 * @retval LINEARS_OK Context initialized and documented history cleared.
 * @retval LINEARS_ERROR_ARGUMENT A required pointer is NULL or a count is zero.
 * @retval LINEARS_ERROR_RANGE A scalar is nonfinite or an exponent cannot be represented.
 * @retval LINEARS_ERROR_DOMAIN alpha is outside [0,1] or sample_period is not positive.
 *
 * @note All numeric values must be valid for the chosen format. Serialize use of ctx.
 */
linears_status_t filters_complementary_f64_init(filters_complementary_f64_t *ctx, double alpha, double sample_period, double initial);

/**
 * @brief Reset a F64 complementary history.
 *
 * @param[in,out] ctx Non-NULL initialized context; coefficients and storage are preserved.
 * @param[in] initial New output history, in signal units.
 *
 * @return linears_status_t Operation status.
 *
 * @retval LINEARS_OK History replaced (zero-filled for window/cascade filters).
 * @retval LINEARS_ERROR_ARGUMENT ctx is NULL.
 * @retval LINEARS_ERROR_RANGE An initial value is nonfinite or outside the exponent range.
 *
 * @note Failure preserves state. Caller must serialize use of ctx; scratch is not cleared.
 */
linears_status_t filters_complementary_f64_reset(filters_complementary_f64_t *ctx, double initial);

/**
 * @brief Process one F64 complementary sample.
 *
 * @param[in,out] ctx Non-NULL successfully initialized context.
 * @param[in] rate Measured derivative in measurement units per second.
 * @param[in] measurement Absolute observation in the estimate physical unit.
 * @param[out] output Non-NULL scalar; must not overlap ctx or any retained live array prefix.
 *
 * @return linears_status_t Operation status.
 *
 * @retval LINEARS_OK Output and complete persistent state committed.
 * @retval LINEARS_ERROR_ARGUMENT ctx or output is NULL.
 * @retval LINEARS_ERROR_ALIAS output overlaps a retained live buffer or ctx.
 * @retval LINEARS_ERROR_RANGE A scalar/intermediate is nonfinite or outside the exponent range.
 *
 * @note Failure preserves all persistent state and output; scratch may change. Serialize ctx.
 */
linears_status_t filters_complementary_f64_step(filters_complementary_f64_t *ctx, double rate, double measurement, double *output);

/* Q15 interface. */

/**
 * @brief Caller-owned FIR delay ring and immutable coefficient reference.
 */
typedef struct {
    const linears_q15_t *coefficients; /**< h[0] multiplies the current input; length taps. */
    linears_q15_t *state; /**< Private zero-padded delay ring; length taps. */
    size_t taps; /**< Positive coefficient and history length, in samples. */
    size_t head; /**< Next replaced ring index; maintained by the API. */
} filters_fir_q15_t;

/**
 * @brief Normalized biquad coefficients: denominator 1 + a1*z^-1 + a2*z^-2.
 */
typedef struct {
    linears_q15_t b0; /**< Current-input feedforward coefficient, dimensionless. */
    linears_q15_t b1; /**< One-delay feedforward coefficient, dimensionless. */
    linears_q15_t b2; /**< Two-delay feedforward coefficient, dimensionless. */
    linears_q15_t a1; /**< One-delay denominator coefficient, subtracted in recurrence. */
    linears_q15_t a2; /**< Two-delay denominator coefficient, subtracted in recurrence. */
} filters_biquad_q15_coefficients_t;

/**
 * @brief Caller-owned SOS cascade with transactional tentative-state scratch.
 */
typedef struct {
    const filters_biquad_q15_coefficients_t *coefficients; /**< Immutable sections array. */
    linears_q15_t *state; /**< Private pairs (s1,s2), length 2*sections. */
    linears_q15_t *workspace; /**< Tentative states, length 2*sections; not persistent. */
    size_t sections; /**< Positive number of cascaded second-order sections. */
} filters_biquad_q15_t;

/**
 * @brief Caller-owned first-order high-pass context.
 */
typedef struct {
    linears_q15_t alpha; /**< Dimensionless recurrence multiplier in [0,1]. */
    linears_q15_t previous_input; /**< Previous sample in input units. */
    linears_q15_t value; /**< Last output in input units. */
} filters_highpass_q15_t;

/**
 * @brief Caller-owned zero-padded moving-average delay ring.
 */
typedef struct {
    linears_q15_t *state; /**< Private sample ring, length window. */
    size_t window; /**< Positive averaging-window length, in samples. */
    size_t head; /**< Next replaced ring index; maintained by the API. */
} filters_moving_average_q15_t;

/**
 * @brief Caller-owned median delay ring and sorting scratch.
 */
typedef struct {
    linears_q15_t *state; /**< Private sample ring, length window. */
    linears_q15_t *workspace; /**< Sorting scratch, length window. */
    size_t window; /**< Positive window length, in samples; even lengths allowed. */
    size_t head; /**< Next replaced ring index; maintained by the API. */
} filters_median_q15_t;

/**
 * @brief Caller-owned linear complementary rate/measurement filter.
 */
typedef struct {
    linears_q15_t alpha; /**< Dimensionless prediction weight in [0,1]. */
    linears_q15_t sample_period; /**< Fixed positive interval between steps, in seconds. */
    linears_q15_t value; /**< Last estimate in the measurement physical unit. */
} filters_complementary_q15_t;

/**
 * @brief Q15 exponential moving average; identical recurrence to low-pass.
 */
typedef filters_lowpass_q15_t filters_ema_q15_t;

/**
 * @brief Initialize a Q15 fir context.
 *
 * Coefficients, state and context must be pairwise disjoint. Arrays remain
 * valid for the context lifetime. Coefficient signs and gains are unrestricted.
 *
 * @param[out] ctx Non-NULL caller-owned context; unchanged on failure.
 * @param[in] coefficients Non-NULL immutable tap array in increasing delay order.
 * @param[in] taps Positive number of coefficients, in samples.
 * @param[out] state Non-NULL history array, zero-filled in its required prefix.
 * @param[in] state_capacity Available state elements; at least FILTERS_FIR_STATE_COUNT(taps).
 *
 * @return linears_status_t Operation status.
 *
 * @retval LINEARS_OK Context initialized and documented history cleared.
 * @retval LINEARS_ERROR_ARGUMENT A required pointer is NULL or a count is zero.
 * @retval LINEARS_ERROR_RANGE A scalar is nonfinite or an exponent cannot be represented.
 * @retval LINEARS_ERROR_SIZE A required byte/count size overflows.
 * @retval LINEARS_ERROR_WORKSPACE A supplied element capacity is too small.
 * @retval LINEARS_ERROR_ALIAS Live context/array prefixes overlap.
 *
 * @note All numeric values must be valid for the chosen format. Serialize use of ctx.
 */
linears_status_t filters_fir_q15_init(filters_fir_q15_t *ctx, const linears_q15_t *coefficients, size_t taps, linears_q15_t *state, size_t state_capacity);

/**
 * @brief Reset a Q15 fir history.
 *
 * @param[in,out] ctx Non-NULL initialized context; coefficients and storage are preserved.
 *
 * @return linears_status_t Operation status.
 *
 * @retval LINEARS_OK History replaced (zero-filled for window/cascade filters).
 * @retval LINEARS_ERROR_ARGUMENT ctx is NULL.
 *
 * @note Failure preserves state. Caller must serialize use of ctx; scratch is not cleared.
 */
linears_status_t filters_fir_q15_reset(filters_fir_q15_t *ctx);

/**
 * @brief Process one Q15 fir sample.
 *
 * @param[in,out] ctx Non-NULL successfully initialized context.
 * @param[in] input One sample in the physical units used for the state.
 * @param[out] output Non-NULL scalar; must not overlap ctx or any retained live array prefix.
 *
 * @return linears_status_t Operation status.
 *
 * @retval LINEARS_OK Output and complete persistent state committed.
 * @retval LINEARS_ERROR_ARGUMENT ctx or output is NULL.
 * @retval LINEARS_ERROR_ALIAS output overlaps a retained live buffer or ctx.
 * @retval LINEARS_ERROR_RANGE A scalar/intermediate is nonfinite or outside the exponent range.
 *
 * @note Failure preserves all persistent state and output; scratch may change. Serialize ctx.
 */
linears_status_t filters_fir_q15_step(filters_fir_q15_t *ctx, linears_q15_t input, linears_q15_t *output);

/**
 * @brief Initialize a Q15 biquad context.
 *
 * All context/array live prefixes must be pairwise disjoint. Arrays remain
 * valid for the context lifetime. Stability and scaling are caller choices.
 * Each section computes y=b0*x+s1, s1=b1*x-a1*y+s2, s2=b2*x-a2*y.
 * The section output and state are rounded to this API storage precision.
 *
 * @param[out] ctx Non-NULL caller-owned context; unchanged on failure.
 * @param[in] coefficients Non-NULL immutable normalized SOS array; a0 is implicitly one.
 * @param[in] sections Positive number of cascaded sections.
 * @param[out] state Non-NULL state array, zero-filled; 2*sections elements are used.
 * @param[in] state_capacity Available state elements; at least FILTERS_BIQUAD_STATE_COUNT(sections).
 * @param[in,out] workspace Non-NULL tentative-state array; contents are unspecified after step.
 * @param[in] workspace_capacity Scratch elements; at least FILTERS_BIQUAD_WORKSPACE_COUNT(sections).
 *
 * @return linears_status_t Operation status.
 *
 * @retval LINEARS_OK Context initialized and documented history cleared.
 * @retval LINEARS_ERROR_ARGUMENT A required pointer is NULL or a count is zero.
 * @retval LINEARS_ERROR_RANGE A scalar is nonfinite or an exponent cannot be represented.
 * @retval LINEARS_ERROR_SIZE A required byte/count size overflows.
 * @retval LINEARS_ERROR_WORKSPACE A supplied element capacity is too small.
 * @retval LINEARS_ERROR_ALIAS Live context/array prefixes overlap.
 *
 * @note All numeric values must be valid for the chosen format. Serialize use of ctx.
 */
linears_status_t filters_biquad_q15_init(filters_biquad_q15_t *ctx, const filters_biquad_q15_coefficients_t *coefficients, size_t sections, linears_q15_t *state, size_t state_capacity, linears_q15_t *workspace, size_t workspace_capacity);

/**
 * @brief Reset a Q15 biquad history.
 *
 * @param[in,out] ctx Non-NULL initialized context; coefficients and storage are preserved.
 *
 * @return linears_status_t Operation status.
 *
 * @retval LINEARS_OK History replaced (zero-filled for window/cascade filters).
 * @retval LINEARS_ERROR_ARGUMENT ctx is NULL.
 *
 * @note Failure preserves state. Caller must serialize use of ctx; scratch is not cleared.
 */
linears_status_t filters_biquad_q15_reset(filters_biquad_q15_t *ctx);

/**
 * @brief Process one Q15 biquad sample.
 *
 * @param[in,out] ctx Non-NULL successfully initialized context.
 * @param[in] input One sample in the physical units used for the state.
 * @param[out] output Non-NULL scalar; must not overlap ctx or any retained live array prefix.
 *
 * @return linears_status_t Operation status.
 *
 * @retval LINEARS_OK Output and complete persistent state committed.
 * @retval LINEARS_ERROR_ARGUMENT ctx or output is NULL.
 * @retval LINEARS_ERROR_ALIAS output overlaps a retained live buffer or ctx.
 * @retval LINEARS_ERROR_RANGE A scalar/intermediate is nonfinite or outside the exponent range.
 *
 * @note Failure preserves all persistent state and output; scratch may change. Serialize ctx.
 */
linears_status_t filters_biquad_q15_step(filters_biquad_q15_t *ctx, linears_q15_t input, linears_q15_t *output);

/**
 * @brief Initialize a Q15 highpass context.
 *
 * The recurrence is y=alpha*(previous_y+input-previous_input).
 * For a backward-Euler RC high-pass choose alpha=tau/(tau+dt).
 *
 * @param[out] ctx Non-NULL caller-owned context; unchanged on failure.
 * @param[in] alpha Dimensionless weight in [0,1].
 * @param[in] initial_input Previous input sample in signal units.
 * @param[in] initial_output Previous output sample in signal units.
 *
 * @return linears_status_t Operation status.
 *
 * @retval LINEARS_OK Context initialized and documented history cleared.
 * @retval LINEARS_ERROR_ARGUMENT A required pointer is NULL or a count is zero.
 * @retval LINEARS_ERROR_RANGE A scalar is nonfinite or an exponent cannot be represented.
 * @retval LINEARS_ERROR_DOMAIN alpha is outside [0,1].
 *
 * @note All numeric values must be valid for the chosen format. Serialize use of ctx.
 */
linears_status_t filters_highpass_q15_init(filters_highpass_q15_t *ctx, linears_q15_t alpha, linears_q15_t initial_input, linears_q15_t initial_output);

/**
 * @brief Reset a Q15 highpass history.
 *
 * @param[in,out] ctx Non-NULL initialized context; coefficients and storage are preserved.
 * @param[in] initial_input New previous input, in signal units.
 * @param[in] initial_output New previous output, in signal units.
 *
 * @return linears_status_t Operation status.
 *
 * @retval LINEARS_OK History replaced (zero-filled for window/cascade filters).
 * @retval LINEARS_ERROR_ARGUMENT ctx is NULL.
 * @retval LINEARS_ERROR_RANGE An initial value is nonfinite or outside the exponent range.
 *
 * @note Failure preserves state. Caller must serialize use of ctx; scratch is not cleared.
 */
linears_status_t filters_highpass_q15_reset(filters_highpass_q15_t *ctx, linears_q15_t initial_input, linears_q15_t initial_output);

/**
 * @brief Process one Q15 highpass sample.
 *
 * @param[in,out] ctx Non-NULL successfully initialized context.
 * @param[in] input One sample in the physical units used for the state.
 * @param[out] output Non-NULL scalar; must not overlap ctx or any retained live array prefix.
 *
 * @return linears_status_t Operation status.
 *
 * @retval LINEARS_OK Output and complete persistent state committed.
 * @retval LINEARS_ERROR_ARGUMENT ctx or output is NULL.
 * @retval LINEARS_ERROR_ALIAS output overlaps a retained live buffer or ctx.
 * @retval LINEARS_ERROR_RANGE A scalar/intermediate is nonfinite or outside the exponent range.
 *
 * @note Failure preserves all persistent state and output; scratch may change. Serialize ctx.
 */
linears_status_t filters_highpass_q15_step(filters_highpass_q15_t *ctx, linears_q15_t input, linears_q15_t *output);

/**
 * @brief Initialize a Q15 moving average context.
 *
 * The context and all live array prefixes must be pairwise disjoint.
 * Arrays remain valid for the context lifetime. Startup samples outside
 * the available history are zeros; the divisor/window length stays fixed.
 *
 * @param[out] ctx Non-NULL caller-owned context; unchanged on failure.
 * @param[in] window Positive window length, in samples.
 * @param[out] state Non-NULL sample ring; its window-element prefix is zero-filled.
 * @param[in] state_capacity Available state elements; must be at least window.
 *
 * @return linears_status_t Operation status.
 *
 * @retval LINEARS_OK Context initialized and documented history cleared.
 * @retval LINEARS_ERROR_ARGUMENT A required pointer is NULL or a count is zero.
 * @retval LINEARS_ERROR_SIZE A required byte/count size overflows.
 * @retval LINEARS_ERROR_WORKSPACE A supplied element capacity is too small.
 * @retval LINEARS_ERROR_ALIAS Live context/array prefixes overlap.
 *
 * @note All numeric values must be valid for the chosen format. Serialize use of ctx.
 */
linears_status_t filters_moving_average_q15_init(filters_moving_average_q15_t *ctx, size_t window, linears_q15_t *state, size_t state_capacity);

/**
 * @brief Reset a Q15 moving average history.
 *
 * @param[in,out] ctx Non-NULL initialized context; coefficients and storage are preserved.
 *
 * @return linears_status_t Operation status.
 *
 * @retval LINEARS_OK History replaced (zero-filled for window/cascade filters).
 * @retval LINEARS_ERROR_ARGUMENT ctx is NULL.
 *
 * @note Failure preserves state. Caller must serialize use of ctx; scratch is not cleared.
 */
linears_status_t filters_moving_average_q15_reset(filters_moving_average_q15_t *ctx);

/**
 * @brief Process one Q15 moving average sample.
 *
 * Recomputes the window mean in O(window) work to avoid accumulated
 * running-sum drift. Each term is divided before the sum is formed.
 *
 * @param[in,out] ctx Non-NULL successfully initialized context.
 * @param[in] input One sample in the physical units used for the state.
 * @param[out] output Non-NULL scalar; must not overlap ctx or any retained live array prefix.
 *
 * @return linears_status_t Operation status.
 *
 * @retval LINEARS_OK Output and complete persistent state committed.
 * @retval LINEARS_ERROR_ARGUMENT ctx or output is NULL.
 * @retval LINEARS_ERROR_ALIAS output overlaps a retained live buffer or ctx.
 * @retval LINEARS_ERROR_RANGE A scalar/intermediate is nonfinite or outside the exponent range.
 *
 * @note Failure preserves all persistent state and output; scratch may change. Serialize ctx.
 */
linears_status_t filters_moving_average_q15_step(filters_moving_average_q15_t *ctx, linears_q15_t input, linears_q15_t *output);

/**
 * @brief Initialize a Q15 median context.
 *
 * The context and all live array prefixes must be pairwise disjoint.
 * Arrays remain valid for the context lifetime. Startup samples outside
 * the available history are zeros; the divisor/window length stays fixed.
 *
 * @param[out] ctx Non-NULL caller-owned context; unchanged on failure.
 * @param[in] window Positive window length, in samples.
 * @param[out] state Non-NULL sample ring; its window-element prefix is zero-filled.
 * @param[in] state_capacity Available state elements; must be at least window.
 * @param[in,out] workspace Non-NULL window-element sorting scratch, not initialized here.
 * @param[in] workspace_capacity Available scratch elements; must be at least window.
 *
 * @return linears_status_t Operation status.
 *
 * @retval LINEARS_OK Context initialized and documented history cleared.
 * @retval LINEARS_ERROR_ARGUMENT A required pointer is NULL or a count is zero.
 * @retval LINEARS_ERROR_SIZE A required byte/count size overflows.
 * @retval LINEARS_ERROR_WORKSPACE A supplied element capacity is too small.
 * @retval LINEARS_ERROR_ALIAS Live context/array prefixes overlap.
 *
 * @note All numeric values must be valid for the chosen format. Serialize use of ctx.
 */
linears_status_t filters_median_q15_init(filters_median_q15_t *ctx, size_t window, linears_q15_t *state, size_t state_capacity, linears_q15_t *workspace, size_t workspace_capacity);

/**
 * @brief Reset a Q15 median history.
 *
 * @param[in,out] ctx Non-NULL initialized context; coefficients and storage are preserved.
 *
 * @return linears_status_t Operation status.
 *
 * @retval LINEARS_OK History replaced (zero-filled for window/cascade filters).
 * @retval LINEARS_ERROR_ARGUMENT ctx is NULL.
 *
 * @note Failure preserves state. Caller must serialize use of ctx; scratch is not cleared.
 */
linears_status_t filters_median_q15_reset(filters_median_q15_t *ctx);

/**
 * @brief Process one Q15 median sample.
 *
 * Returns the central sorted sample, or the mean of both central values
 * for an even window. Sorting is O(window^2) with caller-owned scratch.
 *
 * @param[in,out] ctx Non-NULL successfully initialized context.
 * @param[in] input One sample in the physical units used for the state.
 * @param[out] output Non-NULL scalar; must not overlap ctx or any retained live array prefix.
 *
 * @return linears_status_t Operation status.
 *
 * @retval LINEARS_OK Output and complete persistent state committed.
 * @retval LINEARS_ERROR_ARGUMENT ctx or output is NULL.
 * @retval LINEARS_ERROR_ALIAS output overlaps a retained live buffer or ctx.
 * @retval LINEARS_ERROR_RANGE A scalar/intermediate is nonfinite or outside the exponent range.
 *
 * @note Failure preserves all persistent state and output; scratch may change. Serialize ctx.
 */
linears_status_t filters_median_q15_step(filters_median_q15_t *ctx, linears_q15_t input, linears_q15_t *output);

/**
 * @brief Initialize a Q15 ema context.
 *
 * The recurrence is y=(1-alpha)*previous_y+alpha*input.
 * This API delegates to the same implementation as the first-order low-pass.
 *
 * @param[out] ctx Non-NULL caller-owned context; unchanged on failure.
 * @param[in] alpha Dimensionless weight in [0,1].
 * @param[in] initial Initial estimate/output in signal units.
 *
 * @return linears_status_t Operation status.
 *
 * @retval LINEARS_OK Context initialized and documented history cleared.
 * @retval LINEARS_ERROR_ARGUMENT A required pointer is NULL or a count is zero.
 * @retval LINEARS_ERROR_RANGE A scalar is nonfinite or an exponent cannot be represented.
 * @retval LINEARS_ERROR_DOMAIN alpha is outside [0,1].
 *
 * @note All numeric values must be valid for the chosen format. Serialize use of ctx.
 */
linears_status_t filters_ema_q15_init(filters_ema_q15_t *ctx, linears_q15_t alpha, linears_q15_t initial);

/**
 * @brief Reset a Q15 ema history.
 *
 * @param[in,out] ctx Non-NULL initialized context; coefficients and storage are preserved.
 * @param[in] initial New output history, in signal units.
 *
 * @return linears_status_t Operation status.
 *
 * @retval LINEARS_OK History replaced (zero-filled for window/cascade filters).
 * @retval LINEARS_ERROR_ARGUMENT ctx is NULL.
 * @retval LINEARS_ERROR_RANGE An initial value is nonfinite or outside the exponent range.
 *
 * @note Failure preserves state. Caller must serialize use of ctx; scratch is not cleared.
 */
linears_status_t filters_ema_q15_reset(filters_ema_q15_t *ctx, linears_q15_t initial);

/**
 * @brief Process one Q15 ema sample.
 *
 * @param[in,out] ctx Non-NULL successfully initialized context.
 * @param[in] input One sample in the physical units used for the state.
 * @param[out] output Non-NULL scalar; must not overlap ctx or any retained live array prefix.
 *
 * @return linears_status_t Operation status.
 *
 * @retval LINEARS_OK Output and complete persistent state committed.
 * @retval LINEARS_ERROR_ARGUMENT ctx or output is NULL.
 * @retval LINEARS_ERROR_ALIAS output overlaps a retained live buffer or ctx.
 * @retval LINEARS_ERROR_RANGE A scalar/intermediate is nonfinite or outside the exponent range.
 *
 * @note Failure preserves all persistent state and output; scratch may change. Serialize ctx.
 */
linears_status_t filters_ema_q15_step(filters_ema_q15_t *ctx, linears_q15_t input, linears_q15_t *output);

/**
 * @brief Initialize a Q15 complementary context.
 *
 * The recurrence is y=alpha*(previous_y+rate*sample_period)
 * +(1-alpha)*measurement. No angle wrapping or nonlinear state model is used.
 *
 * @param[out] ctx Non-NULL caller-owned context; unchanged on failure.
 * @param[in] alpha Dimensionless weight in [0,1].
 * @param[in] sample_period Positive fixed sample period, in seconds.
 * @param[in] initial Initial estimate/output in signal units.
 *
 * @return linears_status_t Operation status.
 *
 * @retval LINEARS_OK Context initialized and documented history cleared.
 * @retval LINEARS_ERROR_ARGUMENT A required pointer is NULL or a count is zero.
 * @retval LINEARS_ERROR_RANGE A scalar is nonfinite or an exponent cannot be represented.
 * @retval LINEARS_ERROR_DOMAIN alpha is outside [0,1] or sample_period is not positive.
 *
 * @note All numeric values must be valid for the chosen format. Serialize use of ctx.
 */
linears_status_t filters_complementary_q15_init(filters_complementary_q15_t *ctx, linears_q15_t alpha, linears_q15_t sample_period, linears_q15_t initial);

/**
 * @brief Reset a Q15 complementary history.
 *
 * @param[in,out] ctx Non-NULL initialized context; coefficients and storage are preserved.
 * @param[in] initial New output history, in signal units.
 *
 * @return linears_status_t Operation status.
 *
 * @retval LINEARS_OK History replaced (zero-filled for window/cascade filters).
 * @retval LINEARS_ERROR_ARGUMENT ctx is NULL.
 * @retval LINEARS_ERROR_RANGE An initial value is nonfinite or outside the exponent range.
 *
 * @note Failure preserves state. Caller must serialize use of ctx; scratch is not cleared.
 */
linears_status_t filters_complementary_q15_reset(filters_complementary_q15_t *ctx, linears_q15_t initial);

/**
 * @brief Process one Q15 complementary sample.
 *
 * @param[in,out] ctx Non-NULL successfully initialized context.
 * @param[in] rate Measured derivative in measurement units per second.
 * @param[in] measurement Absolute observation in the estimate physical unit.
 * @param[out] output Non-NULL scalar; must not overlap ctx or any retained live array prefix.
 *
 * @return linears_status_t Operation status.
 *
 * @retval LINEARS_OK Output and complete persistent state committed.
 * @retval LINEARS_ERROR_ARGUMENT ctx or output is NULL.
 * @retval LINEARS_ERROR_ALIAS output overlaps a retained live buffer or ctx.
 * @retval LINEARS_ERROR_RANGE A scalar/intermediate is nonfinite or outside the exponent range.
 *
 * @note Failure preserves all persistent state and output; scratch may change. Serialize ctx.
 */
linears_status_t filters_complementary_q15_step(filters_complementary_q15_t *ctx, linears_q15_t rate, linears_q15_t measurement, linears_q15_t *output);

/* Q31 interface. */

/**
 * @brief Caller-owned FIR delay ring and immutable coefficient reference.
 */
typedef struct {
    const linears_q31_t *coefficients; /**< h[0] multiplies the current input; length taps. */
    linears_q31_t *state; /**< Private zero-padded delay ring; length taps. */
    size_t taps; /**< Positive coefficient and history length, in samples. */
    size_t head; /**< Next replaced ring index; maintained by the API. */
} filters_fir_q31_t;

/**
 * @brief Normalized biquad coefficients: denominator 1 + a1*z^-1 + a2*z^-2.
 */
typedef struct {
    linears_q31_t b0; /**< Current-input feedforward coefficient, dimensionless. */
    linears_q31_t b1; /**< One-delay feedforward coefficient, dimensionless. */
    linears_q31_t b2; /**< Two-delay feedforward coefficient, dimensionless. */
    linears_q31_t a1; /**< One-delay denominator coefficient, subtracted in recurrence. */
    linears_q31_t a2; /**< Two-delay denominator coefficient, subtracted in recurrence. */
} filters_biquad_q31_coefficients_t;

/**
 * @brief Caller-owned SOS cascade with transactional tentative-state scratch.
 */
typedef struct {
    const filters_biquad_q31_coefficients_t *coefficients; /**< Immutable sections array. */
    linears_q31_t *state; /**< Private pairs (s1,s2), length 2*sections. */
    linears_q31_t *workspace; /**< Tentative states, length 2*sections; not persistent. */
    size_t sections; /**< Positive number of cascaded second-order sections. */
} filters_biquad_q31_t;

/**
 * @brief Caller-owned first-order high-pass context.
 */
typedef struct {
    linears_q31_t alpha; /**< Dimensionless recurrence multiplier in [0,1]. */
    linears_q31_t previous_input; /**< Previous sample in input units. */
    linears_q31_t value; /**< Last output in input units. */
} filters_highpass_q31_t;

/**
 * @brief Caller-owned zero-padded moving-average delay ring.
 */
typedef struct {
    linears_q31_t *state; /**< Private sample ring, length window. */
    size_t window; /**< Positive averaging-window length, in samples. */
    size_t head; /**< Next replaced ring index; maintained by the API. */
} filters_moving_average_q31_t;

/**
 * @brief Caller-owned median delay ring and sorting scratch.
 */
typedef struct {
    linears_q31_t *state; /**< Private sample ring, length window. */
    linears_q31_t *workspace; /**< Sorting scratch, length window. */
    size_t window; /**< Positive window length, in samples; even lengths allowed. */
    size_t head; /**< Next replaced ring index; maintained by the API. */
} filters_median_q31_t;

/**
 * @brief Caller-owned linear complementary rate/measurement filter.
 */
typedef struct {
    linears_q31_t alpha; /**< Dimensionless prediction weight in [0,1]. */
    linears_q31_t sample_period; /**< Fixed positive interval between steps, in seconds. */
    linears_q31_t value; /**< Last estimate in the measurement physical unit. */
} filters_complementary_q31_t;

/**
 * @brief Q31 exponential moving average; identical recurrence to low-pass.
 */
typedef filters_lowpass_q31_t filters_ema_q31_t;

/**
 * @brief Initialize a Q31 fir context.
 *
 * Coefficients, state and context must be pairwise disjoint. Arrays remain
 * valid for the context lifetime. Coefficient signs and gains are unrestricted.
 *
 * @param[out] ctx Non-NULL caller-owned context; unchanged on failure.
 * @param[in] coefficients Non-NULL immutable tap array in increasing delay order.
 * @param[in] taps Positive number of coefficients, in samples.
 * @param[out] state Non-NULL history array, zero-filled in its required prefix.
 * @param[in] state_capacity Available state elements; at least FILTERS_FIR_STATE_COUNT(taps).
 *
 * @return linears_status_t Operation status.
 *
 * @retval LINEARS_OK Context initialized and documented history cleared.
 * @retval LINEARS_ERROR_ARGUMENT A required pointer is NULL or a count is zero.
 * @retval LINEARS_ERROR_RANGE A scalar is nonfinite or an exponent cannot be represented.
 * @retval LINEARS_ERROR_SIZE A required byte/count size overflows.
 * @retval LINEARS_ERROR_WORKSPACE A supplied element capacity is too small.
 * @retval LINEARS_ERROR_ALIAS Live context/array prefixes overlap.
 *
 * @note All numeric values must be valid for the chosen format. Serialize use of ctx.
 */
linears_status_t filters_fir_q31_init(filters_fir_q31_t *ctx, const linears_q31_t *coefficients, size_t taps, linears_q31_t *state, size_t state_capacity);

/**
 * @brief Reset a Q31 fir history.
 *
 * @param[in,out] ctx Non-NULL initialized context; coefficients and storage are preserved.
 *
 * @return linears_status_t Operation status.
 *
 * @retval LINEARS_OK History replaced (zero-filled for window/cascade filters).
 * @retval LINEARS_ERROR_ARGUMENT ctx is NULL.
 *
 * @note Failure preserves state. Caller must serialize use of ctx; scratch is not cleared.
 */
linears_status_t filters_fir_q31_reset(filters_fir_q31_t *ctx);

/**
 * @brief Process one Q31 fir sample.
 *
 * @param[in,out] ctx Non-NULL successfully initialized context.
 * @param[in] input One sample in the physical units used for the state.
 * @param[out] output Non-NULL scalar; must not overlap ctx or any retained live array prefix.
 *
 * @return linears_status_t Operation status.
 *
 * @retval LINEARS_OK Output and complete persistent state committed.
 * @retval LINEARS_ERROR_ARGUMENT ctx or output is NULL.
 * @retval LINEARS_ERROR_ALIAS output overlaps a retained live buffer or ctx.
 * @retval LINEARS_ERROR_RANGE A scalar/intermediate is nonfinite or outside the exponent range.
 *
 * @note Failure preserves all persistent state and output; scratch may change. Serialize ctx.
 */
linears_status_t filters_fir_q31_step(filters_fir_q31_t *ctx, linears_q31_t input, linears_q31_t *output);

/**
 * @brief Initialize a Q31 biquad context.
 *
 * All context/array live prefixes must be pairwise disjoint. Arrays remain
 * valid for the context lifetime. Stability and scaling are caller choices.
 * Each section computes y=b0*x+s1, s1=b1*x-a1*y+s2, s2=b2*x-a2*y.
 * The section output and state are rounded to this API storage precision.
 *
 * @param[out] ctx Non-NULL caller-owned context; unchanged on failure.
 * @param[in] coefficients Non-NULL immutable normalized SOS array; a0 is implicitly one.
 * @param[in] sections Positive number of cascaded sections.
 * @param[out] state Non-NULL state array, zero-filled; 2*sections elements are used.
 * @param[in] state_capacity Available state elements; at least FILTERS_BIQUAD_STATE_COUNT(sections).
 * @param[in,out] workspace Non-NULL tentative-state array; contents are unspecified after step.
 * @param[in] workspace_capacity Scratch elements; at least FILTERS_BIQUAD_WORKSPACE_COUNT(sections).
 *
 * @return linears_status_t Operation status.
 *
 * @retval LINEARS_OK Context initialized and documented history cleared.
 * @retval LINEARS_ERROR_ARGUMENT A required pointer is NULL or a count is zero.
 * @retval LINEARS_ERROR_RANGE A scalar is nonfinite or an exponent cannot be represented.
 * @retval LINEARS_ERROR_SIZE A required byte/count size overflows.
 * @retval LINEARS_ERROR_WORKSPACE A supplied element capacity is too small.
 * @retval LINEARS_ERROR_ALIAS Live context/array prefixes overlap.
 *
 * @note All numeric values must be valid for the chosen format. Serialize use of ctx.
 */
linears_status_t filters_biquad_q31_init(filters_biquad_q31_t *ctx, const filters_biquad_q31_coefficients_t *coefficients, size_t sections, linears_q31_t *state, size_t state_capacity, linears_q31_t *workspace, size_t workspace_capacity);

/**
 * @brief Reset a Q31 biquad history.
 *
 * @param[in,out] ctx Non-NULL initialized context; coefficients and storage are preserved.
 *
 * @return linears_status_t Operation status.
 *
 * @retval LINEARS_OK History replaced (zero-filled for window/cascade filters).
 * @retval LINEARS_ERROR_ARGUMENT ctx is NULL.
 *
 * @note Failure preserves state. Caller must serialize use of ctx; scratch is not cleared.
 */
linears_status_t filters_biquad_q31_reset(filters_biquad_q31_t *ctx);

/**
 * @brief Process one Q31 biquad sample.
 *
 * @param[in,out] ctx Non-NULL successfully initialized context.
 * @param[in] input One sample in the physical units used for the state.
 * @param[out] output Non-NULL scalar; must not overlap ctx or any retained live array prefix.
 *
 * @return linears_status_t Operation status.
 *
 * @retval LINEARS_OK Output and complete persistent state committed.
 * @retval LINEARS_ERROR_ARGUMENT ctx or output is NULL.
 * @retval LINEARS_ERROR_ALIAS output overlaps a retained live buffer or ctx.
 * @retval LINEARS_ERROR_RANGE A scalar/intermediate is nonfinite or outside the exponent range.
 *
 * @note Failure preserves all persistent state and output; scratch may change. Serialize ctx.
 */
linears_status_t filters_biquad_q31_step(filters_biquad_q31_t *ctx, linears_q31_t input, linears_q31_t *output);

/**
 * @brief Initialize a Q31 highpass context.
 *
 * The recurrence is y=alpha*(previous_y+input-previous_input).
 * For a backward-Euler RC high-pass choose alpha=tau/(tau+dt).
 *
 * @param[out] ctx Non-NULL caller-owned context; unchanged on failure.
 * @param[in] alpha Dimensionless weight in [0,1].
 * @param[in] initial_input Previous input sample in signal units.
 * @param[in] initial_output Previous output sample in signal units.
 *
 * @return linears_status_t Operation status.
 *
 * @retval LINEARS_OK Context initialized and documented history cleared.
 * @retval LINEARS_ERROR_ARGUMENT A required pointer is NULL or a count is zero.
 * @retval LINEARS_ERROR_RANGE A scalar is nonfinite or an exponent cannot be represented.
 * @retval LINEARS_ERROR_DOMAIN alpha is outside [0,1].
 *
 * @note All numeric values must be valid for the chosen format. Serialize use of ctx.
 */
linears_status_t filters_highpass_q31_init(filters_highpass_q31_t *ctx, linears_q31_t alpha, linears_q31_t initial_input, linears_q31_t initial_output);

/**
 * @brief Reset a Q31 highpass history.
 *
 * @param[in,out] ctx Non-NULL initialized context; coefficients and storage are preserved.
 * @param[in] initial_input New previous input, in signal units.
 * @param[in] initial_output New previous output, in signal units.
 *
 * @return linears_status_t Operation status.
 *
 * @retval LINEARS_OK History replaced (zero-filled for window/cascade filters).
 * @retval LINEARS_ERROR_ARGUMENT ctx is NULL.
 * @retval LINEARS_ERROR_RANGE An initial value is nonfinite or outside the exponent range.
 *
 * @note Failure preserves state. Caller must serialize use of ctx; scratch is not cleared.
 */
linears_status_t filters_highpass_q31_reset(filters_highpass_q31_t *ctx, linears_q31_t initial_input, linears_q31_t initial_output);

/**
 * @brief Process one Q31 highpass sample.
 *
 * @param[in,out] ctx Non-NULL successfully initialized context.
 * @param[in] input One sample in the physical units used for the state.
 * @param[out] output Non-NULL scalar; must not overlap ctx or any retained live array prefix.
 *
 * @return linears_status_t Operation status.
 *
 * @retval LINEARS_OK Output and complete persistent state committed.
 * @retval LINEARS_ERROR_ARGUMENT ctx or output is NULL.
 * @retval LINEARS_ERROR_ALIAS output overlaps a retained live buffer or ctx.
 * @retval LINEARS_ERROR_RANGE A scalar/intermediate is nonfinite or outside the exponent range.
 *
 * @note Failure preserves all persistent state and output; scratch may change. Serialize ctx.
 */
linears_status_t filters_highpass_q31_step(filters_highpass_q31_t *ctx, linears_q31_t input, linears_q31_t *output);

/**
 * @brief Initialize a Q31 moving average context.
 *
 * The context and all live array prefixes must be pairwise disjoint.
 * Arrays remain valid for the context lifetime. Startup samples outside
 * the available history are zeros; the divisor/window length stays fixed.
 *
 * @param[out] ctx Non-NULL caller-owned context; unchanged on failure.
 * @param[in] window Positive window length, in samples.
 * @param[out] state Non-NULL sample ring; its window-element prefix is zero-filled.
 * @param[in] state_capacity Available state elements; must be at least window.
 *
 * @return linears_status_t Operation status.
 *
 * @retval LINEARS_OK Context initialized and documented history cleared.
 * @retval LINEARS_ERROR_ARGUMENT A required pointer is NULL or a count is zero.
 * @retval LINEARS_ERROR_SIZE A required byte/count size overflows.
 * @retval LINEARS_ERROR_WORKSPACE A supplied element capacity is too small.
 * @retval LINEARS_ERROR_ALIAS Live context/array prefixes overlap.
 *
 * @note All numeric values must be valid for the chosen format. Serialize use of ctx.
 */
linears_status_t filters_moving_average_q31_init(filters_moving_average_q31_t *ctx, size_t window, linears_q31_t *state, size_t state_capacity);

/**
 * @brief Reset a Q31 moving average history.
 *
 * @param[in,out] ctx Non-NULL initialized context; coefficients and storage are preserved.
 *
 * @return linears_status_t Operation status.
 *
 * @retval LINEARS_OK History replaced (zero-filled for window/cascade filters).
 * @retval LINEARS_ERROR_ARGUMENT ctx is NULL.
 *
 * @note Failure preserves state. Caller must serialize use of ctx; scratch is not cleared.
 */
linears_status_t filters_moving_average_q31_reset(filters_moving_average_q31_t *ctx);

/**
 * @brief Process one Q31 moving average sample.
 *
 * Recomputes the window mean in O(window) work to avoid accumulated
 * running-sum drift. Each term is divided before the sum is formed.
 *
 * @param[in,out] ctx Non-NULL successfully initialized context.
 * @param[in] input One sample in the physical units used for the state.
 * @param[out] output Non-NULL scalar; must not overlap ctx or any retained live array prefix.
 *
 * @return linears_status_t Operation status.
 *
 * @retval LINEARS_OK Output and complete persistent state committed.
 * @retval LINEARS_ERROR_ARGUMENT ctx or output is NULL.
 * @retval LINEARS_ERROR_ALIAS output overlaps a retained live buffer or ctx.
 * @retval LINEARS_ERROR_RANGE A scalar/intermediate is nonfinite or outside the exponent range.
 *
 * @note Failure preserves all persistent state and output; scratch may change. Serialize ctx.
 */
linears_status_t filters_moving_average_q31_step(filters_moving_average_q31_t *ctx, linears_q31_t input, linears_q31_t *output);

/**
 * @brief Initialize a Q31 median context.
 *
 * The context and all live array prefixes must be pairwise disjoint.
 * Arrays remain valid for the context lifetime. Startup samples outside
 * the available history are zeros; the divisor/window length stays fixed.
 *
 * @param[out] ctx Non-NULL caller-owned context; unchanged on failure.
 * @param[in] window Positive window length, in samples.
 * @param[out] state Non-NULL sample ring; its window-element prefix is zero-filled.
 * @param[in] state_capacity Available state elements; must be at least window.
 * @param[in,out] workspace Non-NULL window-element sorting scratch, not initialized here.
 * @param[in] workspace_capacity Available scratch elements; must be at least window.
 *
 * @return linears_status_t Operation status.
 *
 * @retval LINEARS_OK Context initialized and documented history cleared.
 * @retval LINEARS_ERROR_ARGUMENT A required pointer is NULL or a count is zero.
 * @retval LINEARS_ERROR_SIZE A required byte/count size overflows.
 * @retval LINEARS_ERROR_WORKSPACE A supplied element capacity is too small.
 * @retval LINEARS_ERROR_ALIAS Live context/array prefixes overlap.
 *
 * @note All numeric values must be valid for the chosen format. Serialize use of ctx.
 */
linears_status_t filters_median_q31_init(filters_median_q31_t *ctx, size_t window, linears_q31_t *state, size_t state_capacity, linears_q31_t *workspace, size_t workspace_capacity);

/**
 * @brief Reset a Q31 median history.
 *
 * @param[in,out] ctx Non-NULL initialized context; coefficients and storage are preserved.
 *
 * @return linears_status_t Operation status.
 *
 * @retval LINEARS_OK History replaced (zero-filled for window/cascade filters).
 * @retval LINEARS_ERROR_ARGUMENT ctx is NULL.
 *
 * @note Failure preserves state. Caller must serialize use of ctx; scratch is not cleared.
 */
linears_status_t filters_median_q31_reset(filters_median_q31_t *ctx);

/**
 * @brief Process one Q31 median sample.
 *
 * Returns the central sorted sample, or the mean of both central values
 * for an even window. Sorting is O(window^2) with caller-owned scratch.
 *
 * @param[in,out] ctx Non-NULL successfully initialized context.
 * @param[in] input One sample in the physical units used for the state.
 * @param[out] output Non-NULL scalar; must not overlap ctx or any retained live array prefix.
 *
 * @return linears_status_t Operation status.
 *
 * @retval LINEARS_OK Output and complete persistent state committed.
 * @retval LINEARS_ERROR_ARGUMENT ctx or output is NULL.
 * @retval LINEARS_ERROR_ALIAS output overlaps a retained live buffer or ctx.
 * @retval LINEARS_ERROR_RANGE A scalar/intermediate is nonfinite or outside the exponent range.
 *
 * @note Failure preserves all persistent state and output; scratch may change. Serialize ctx.
 */
linears_status_t filters_median_q31_step(filters_median_q31_t *ctx, linears_q31_t input, linears_q31_t *output);

/**
 * @brief Initialize a Q31 ema context.
 *
 * The recurrence is y=(1-alpha)*previous_y+alpha*input.
 * This API delegates to the same implementation as the first-order low-pass.
 *
 * @param[out] ctx Non-NULL caller-owned context; unchanged on failure.
 * @param[in] alpha Dimensionless weight in [0,1].
 * @param[in] initial Initial estimate/output in signal units.
 *
 * @return linears_status_t Operation status.
 *
 * @retval LINEARS_OK Context initialized and documented history cleared.
 * @retval LINEARS_ERROR_ARGUMENT A required pointer is NULL or a count is zero.
 * @retval LINEARS_ERROR_RANGE A scalar is nonfinite or an exponent cannot be represented.
 * @retval LINEARS_ERROR_DOMAIN alpha is outside [0,1].
 *
 * @note All numeric values must be valid for the chosen format. Serialize use of ctx.
 */
linears_status_t filters_ema_q31_init(filters_ema_q31_t *ctx, linears_q31_t alpha, linears_q31_t initial);

/**
 * @brief Reset a Q31 ema history.
 *
 * @param[in,out] ctx Non-NULL initialized context; coefficients and storage are preserved.
 * @param[in] initial New output history, in signal units.
 *
 * @return linears_status_t Operation status.
 *
 * @retval LINEARS_OK History replaced (zero-filled for window/cascade filters).
 * @retval LINEARS_ERROR_ARGUMENT ctx is NULL.
 * @retval LINEARS_ERROR_RANGE An initial value is nonfinite or outside the exponent range.
 *
 * @note Failure preserves state. Caller must serialize use of ctx; scratch is not cleared.
 */
linears_status_t filters_ema_q31_reset(filters_ema_q31_t *ctx, linears_q31_t initial);

/**
 * @brief Process one Q31 ema sample.
 *
 * @param[in,out] ctx Non-NULL successfully initialized context.
 * @param[in] input One sample in the physical units used for the state.
 * @param[out] output Non-NULL scalar; must not overlap ctx or any retained live array prefix.
 *
 * @return linears_status_t Operation status.
 *
 * @retval LINEARS_OK Output and complete persistent state committed.
 * @retval LINEARS_ERROR_ARGUMENT ctx or output is NULL.
 * @retval LINEARS_ERROR_ALIAS output overlaps a retained live buffer or ctx.
 * @retval LINEARS_ERROR_RANGE A scalar/intermediate is nonfinite or outside the exponent range.
 *
 * @note Failure preserves all persistent state and output; scratch may change. Serialize ctx.
 */
linears_status_t filters_ema_q31_step(filters_ema_q31_t *ctx, linears_q31_t input, linears_q31_t *output);

/**
 * @brief Initialize a Q31 complementary context.
 *
 * The recurrence is y=alpha*(previous_y+rate*sample_period)
 * +(1-alpha)*measurement. No angle wrapping or nonlinear state model is used.
 *
 * @param[out] ctx Non-NULL caller-owned context; unchanged on failure.
 * @param[in] alpha Dimensionless weight in [0,1].
 * @param[in] sample_period Positive fixed sample period, in seconds.
 * @param[in] initial Initial estimate/output in signal units.
 *
 * @return linears_status_t Operation status.
 *
 * @retval LINEARS_OK Context initialized and documented history cleared.
 * @retval LINEARS_ERROR_ARGUMENT A required pointer is NULL or a count is zero.
 * @retval LINEARS_ERROR_RANGE A scalar is nonfinite or an exponent cannot be represented.
 * @retval LINEARS_ERROR_DOMAIN alpha is outside [0,1] or sample_period is not positive.
 *
 * @note All numeric values must be valid for the chosen format. Serialize use of ctx.
 */
linears_status_t filters_complementary_q31_init(filters_complementary_q31_t *ctx, linears_q31_t alpha, linears_q31_t sample_period, linears_q31_t initial);

/**
 * @brief Reset a Q31 complementary history.
 *
 * @param[in,out] ctx Non-NULL initialized context; coefficients and storage are preserved.
 * @param[in] initial New output history, in signal units.
 *
 * @return linears_status_t Operation status.
 *
 * @retval LINEARS_OK History replaced (zero-filled for window/cascade filters).
 * @retval LINEARS_ERROR_ARGUMENT ctx is NULL.
 * @retval LINEARS_ERROR_RANGE An initial value is nonfinite or outside the exponent range.
 *
 * @note Failure preserves state. Caller must serialize use of ctx; scratch is not cleared.
 */
linears_status_t filters_complementary_q31_reset(filters_complementary_q31_t *ctx, linears_q31_t initial);

/**
 * @brief Process one Q31 complementary sample.
 *
 * @param[in,out] ctx Non-NULL successfully initialized context.
 * @param[in] rate Measured derivative in measurement units per second.
 * @param[in] measurement Absolute observation in the estimate physical unit.
 * @param[out] output Non-NULL scalar; must not overlap ctx or any retained live array prefix.
 *
 * @return linears_status_t Operation status.
 *
 * @retval LINEARS_OK Output and complete persistent state committed.
 * @retval LINEARS_ERROR_ARGUMENT ctx or output is NULL.
 * @retval LINEARS_ERROR_ALIAS output overlaps a retained live buffer or ctx.
 * @retval LINEARS_ERROR_RANGE A scalar/intermediate is nonfinite or outside the exponent range.
 *
 * @note Failure preserves all persistent state and output; scratch may change. Serialize ctx.
 */
linears_status_t filters_complementary_q31_step(filters_complementary_q31_t *ctx, linears_q31_t rate, linears_q31_t measurement, linears_q31_t *output);

#ifdef __cplusplus
}
#endif
#endif
