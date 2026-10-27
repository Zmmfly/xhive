/**
 * @file filters.h
 * @brief Allocation-free C99 finite-state signal filters in four numeric formats.
 *
 * This umbrella header includes every family: EMA (filters_ema.h), FIR
 * (filters_fir.h), biquad/SOS (filters_iir.h), frequency-designed LPF/HPF/BPF
 * (filters_lpf.h, filters_hpf.h, filters_bpf.h), linear Kalman
 * (filters_kalman.h) and unscented Kalman (filters_ukf.h).
 * It declares the moving-average, median and complementary families itself.
 *
 * All contexts and retained arrays are caller-owned and naturally aligned.
 * Init stores pointers but never allocates: arrays and immutable coefficients
 * must remain alive until the context is no longer used. Only the required
 * prefix of a larger capacity is used. Do not edit an initialized context,
 * coefficients, or its retained state buffers; use reset to clear history.
 * filters_ema_work_step may explicitly advance a Q EMA context's value field.
 *
 * Every non-OK return preserves persistent state and output. Temporary scratch
 * may change after a failed step. Output must not overlap any retained buffer
 * or context. Exact and partial overlaps are both rejected. Single-context
 * calls require caller serialization, including interrupts; disjoint contexts
 * are reentrant. There is no MMIO, I/O, allocation, recursion or mutable static
 * state. The call completes synchronously with bounded, size-dependent work.
 *
 * f32/f64 use their native arithmetic and reject NaN/Inf. q15/q31 signals are
 * raw int16_t/2^15 and int32_t/2^31. FIR/SOS coefficients and fixed-point sample
 * periods use linears_fixed_t, an int64_t with invariant 31 fraction bits, so
 * negative feedback near -2 and gains above one remain expressible. Q EMA,
 * LPF/HPF/BPF, complementary and SOS histories retain F31 precision; signal rings
 * retain raw Q1 storage. Kalman-specific state/workspace contracts are defined
 * by filters_kalman.h and filters_ukf.h.
 * Q export checks its supplied working value against [-1,+1] before narrowing.
 * Direct final MAC paths check F62; compound matrix Kalman checks its computed
 * F31 result, not an ideal unrounded chain. Exact +1 maps to MAX under the shared
 * endpoint policy; MAX means 1-LSB. No rescaling or floating conversion occurs.
 */
#ifndef FILTERS_H
#define FILTERS_H

#include "filters_ema.h"
#include "filters_fir.h"
#include "filters_iir.h"
#include "filters_lpf.h"
#include "filters_hpf.h"
#include "filters_bpf.h"
#include "filters_kalman.h"
#include "filters_ukf.h"

#ifdef __cplusplus
extern "C" {
#endif

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
 * @retval LINEARS_ERROR_RANGE A scalar input or intermediate is nonfinite.
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
 * @retval LINEARS_ERROR_RANGE A scalar input or intermediate is nonfinite.
 *
 * @note Failure preserves all persistent state and output; scratch may change. Serialize ctx.
 */
linears_status_t filters_median_f32_step(filters_median_f32_t *ctx, float input, float *output);

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
 * @retval LINEARS_ERROR_RANGE A scalar input is nonfinite.
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
 * @retval LINEARS_ERROR_RANGE A scalar input or intermediate is nonfinite.
 *
 * @note Failure preserves all persistent state and output; scratch may change. Serialize ctx.
 */
linears_status_t filters_complementary_f32_step(filters_complementary_f32_t *ctx, float rate, float measurement, float *output);

/* F64 interface. */

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
 * @retval LINEARS_ERROR_RANGE A scalar input or intermediate is nonfinite.
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
 * @retval LINEARS_ERROR_RANGE A scalar input or intermediate is nonfinite.
 *
 * @note Failure preserves all persistent state and output; scratch may change. Serialize ctx.
 */
linears_status_t filters_median_f64_step(filters_median_f64_t *ctx, double input, double *output);

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
 * @retval LINEARS_ERROR_RANGE A scalar input is nonfinite.
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
 * @retval LINEARS_ERROR_RANGE A scalar input or intermediate is nonfinite.
 *
 * @note Failure preserves all persistent state and output; scratch may change. Serialize ctx.
 */
linears_status_t filters_complementary_f64_step(filters_complementary_f64_t *ctx, double rate, double measurement, double *output);

/* Q15 interface. */

/**
 * @brief Caller-owned zero-padded moving-average delay ring.
 */
typedef struct {
    linears_q15_t *state; /**< Private sample ring, length window. */
    size_t window; /**< Positive averaging-window length, in samples. */
    size_t head; /**< Next replaced ring index; maintained by the API. */
    linears_accum_t sum; /**< Exact rolling F62 sum; maintained transactionally. */
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
    linears_q15_t alpha; /**< Dimensionless prediction weight raw/2^15, in [0,1); MAX is not one. */
    linears_fixed_t sample_period; /**< Positive F31 raw/2^31 interval, in seconds. */
    linears_fixed_t value; /**< Private F31 estimate in measurement units. */
} filters_complementary_q15_t;

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
 * @note All raw integers have a fixed binary point. Serialize use of ctx.
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
 * Updates an exact 128-bit F62 rolling sum in O(1) work. Division by the
 * window and output rounding happen once; integer accumulation has no drift.
 * Persistent storage includes the sample ring and the 16-byte running sum.
 *
 * @param[in,out] ctx Non-NULL successfully initialized context.
 * @param[in] input Raw Q1.15 sample in the physical units used for the state.
 * @param[out] output Non-NULL raw Q1.15 destination; disjoint from ctx and retained live prefixes.
 *
 * @return linears_status_t Operation status.
 *
 * @retval LINEARS_OK Output and complete persistent state committed.
 * @retval LINEARS_ERROR_ARGUMENT ctx or output is NULL.
 * @retval LINEARS_ERROR_ALIAS output overlaps a retained live buffer or ctx.
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
 * @note All raw integers have a fixed binary point. Serialize use of ctx.
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
 * @param[in] input Raw Q1.15 sample in the physical units used for the state.
 * @param[out] output Non-NULL raw Q1.15 destination; disjoint from ctx and retained live prefixes.
 *
 * @return linears_status_t Operation status.
 *
 * @retval LINEARS_OK Output and complete persistent state committed.
 * @retval LINEARS_ERROR_ARGUMENT ctx or output is NULL.
 * @retval LINEARS_ERROR_ALIAS output overlaps a retained live buffer or ctx.
 *
 * @note Failure preserves all persistent state and output; scratch may change. Serialize ctx.
 */
linears_status_t filters_median_q15_step(filters_median_q15_t *ctx, linears_q15_t input, linears_q15_t *output);

/**
 * @brief Initialize a Q15 complementary context.
 *
 * The recurrence is y=alpha*(previous_y+rate*sample_period)
 * +(1-alpha)*measurement. No angle wrapping or nonlinear state model is used.
 *
 * @param[out] ctx Non-NULL caller-owned context; unchanged on failure.
 * @param[in] alpha Dimensionless raw Q1.15 weight, nonnegative; MAX is 1-2^-15.
 * @param[in] sample_period Positive wide F31 raw/2^31 sample period, in seconds.
 * @param[in] initial Raw Q1.15 initial signal, converted exactly to F31 history.
 *
 * @return linears_status_t Operation status.
 *
 * @retval LINEARS_OK Context initialized and documented history cleared.
 * @retval LINEARS_ERROR_ARGUMENT A required pointer is NULL or a count is zero.
 * @retval LINEARS_ERROR_DOMAIN alpha is negative or sample_period is not positive.
 *
 * @note All raw integers have a fixed binary point. Serialize use of ctx.
 */
linears_status_t filters_complementary_q15_init(filters_complementary_q15_t *ctx, linears_q15_t alpha, linears_fixed_t sample_period, linears_q15_t initial);

/**
 * @brief Reset a Q15 complementary history.
 *
 * @param[in,out] ctx Non-NULL initialized context; coefficients and storage are preserved.
 * @param[in] initial Raw Q1.15 new signal history, converted exactly to F31.
 *
 * @return linears_status_t Operation status.
 *
 * @retval LINEARS_OK History replaced (zero-filled for window/cascade filters).
 * @retval LINEARS_ERROR_ARGUMENT ctx is NULL.
 *
 * @note Failure preserves state. Caller must serialize use of ctx; scratch is not cleared.
 */
linears_status_t filters_complementary_q15_reset(filters_complementary_q15_t *ctx, linears_q15_t initial);

/**
 * @brief Process one Q15 complementary sample.
 *
 * @param[in,out] ctx Non-NULL successfully initialized context.
 * @param[in] rate Raw Q1.15 derivative in measurement units per second.
 * @param[in] measurement Raw Q1.15 absolute observation in estimate units.
 * @param[out] output Non-NULL raw Q1.15 destination; disjoint from ctx and retained live prefixes.
 *
 * @return linears_status_t Operation status.
 *
 * @retval LINEARS_OK Output and complete persistent state committed.
 * @retval LINEARS_ERROR_ARGUMENT ctx or output is NULL.
 * @retval LINEARS_ERROR_ALIAS output overlaps a retained live buffer or ctx.
 * @retval LINEARS_ERROR_RANGE A working accumulator overflows or the true output exceeds [-1,+1].
 *
 * @note Failure preserves all persistent state and output; scratch may change. Serialize ctx.
 */
linears_status_t filters_complementary_q15_step(filters_complementary_q15_t *ctx, linears_q15_t rate, linears_q15_t measurement, linears_q15_t *output);

/* Q31 interface. */

/**
 * @brief Caller-owned zero-padded moving-average delay ring.
 */
typedef struct {
    linears_q31_t *state; /**< Private sample ring, length window. */
    size_t window; /**< Positive averaging-window length, in samples. */
    size_t head; /**< Next replaced ring index; maintained by the API. */
    linears_accum_t sum; /**< Exact rolling F62 sum; maintained transactionally. */
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
    linears_q31_t alpha; /**< Dimensionless prediction weight raw/2^31, in [0,1); MAX is not one. */
    linears_fixed_t sample_period; /**< Positive F31 raw/2^31 interval, in seconds. */
    linears_fixed_t value; /**< Private F31 estimate in measurement units. */
} filters_complementary_q31_t;

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
 * @note All raw integers have a fixed binary point. Serialize use of ctx.
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
 * Updates an exact 128-bit F62 rolling sum in O(1) work. Division by the
 * window and output rounding happen once; integer accumulation has no drift.
 * Persistent storage includes the sample ring and the 16-byte running sum.
 *
 * @param[in,out] ctx Non-NULL successfully initialized context.
 * @param[in] input Raw Q1.31 sample in the physical units used for the state.
 * @param[out] output Non-NULL raw Q1.31 destination; disjoint from ctx and retained live prefixes.
 *
 * @return linears_status_t Operation status.
 *
 * @retval LINEARS_OK Output and complete persistent state committed.
 * @retval LINEARS_ERROR_ARGUMENT ctx or output is NULL.
 * @retval LINEARS_ERROR_ALIAS output overlaps a retained live buffer or ctx.
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
 * @note All raw integers have a fixed binary point. Serialize use of ctx.
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
 * @param[in] input Raw Q1.31 sample in the physical units used for the state.
 * @param[out] output Non-NULL raw Q1.31 destination; disjoint from ctx and retained live prefixes.
 *
 * @return linears_status_t Operation status.
 *
 * @retval LINEARS_OK Output and complete persistent state committed.
 * @retval LINEARS_ERROR_ARGUMENT ctx or output is NULL.
 * @retval LINEARS_ERROR_ALIAS output overlaps a retained live buffer or ctx.
 *
 * @note Failure preserves all persistent state and output; scratch may change. Serialize ctx.
 */
linears_status_t filters_median_q31_step(filters_median_q31_t *ctx, linears_q31_t input, linears_q31_t *output);

/**
 * @brief Initialize a Q31 complementary context.
 *
 * The recurrence is y=alpha*(previous_y+rate*sample_period)
 * +(1-alpha)*measurement. No angle wrapping or nonlinear state model is used.
 *
 * @param[out] ctx Non-NULL caller-owned context; unchanged on failure.
 * @param[in] alpha Dimensionless raw Q1.31 weight, nonnegative; MAX is 1-2^-31.
 * @param[in] sample_period Positive wide F31 raw/2^31 sample period, in seconds.
 * @param[in] initial Raw Q1.31 initial signal, converted exactly to F31 history.
 *
 * @return linears_status_t Operation status.
 *
 * @retval LINEARS_OK Context initialized and documented history cleared.
 * @retval LINEARS_ERROR_ARGUMENT A required pointer is NULL or a count is zero.
 * @retval LINEARS_ERROR_DOMAIN alpha is negative or sample_period is not positive.
 *
 * @note All raw integers have a fixed binary point. Serialize use of ctx.
 */
linears_status_t filters_complementary_q31_init(filters_complementary_q31_t *ctx, linears_q31_t alpha, linears_fixed_t sample_period, linears_q31_t initial);

/**
 * @brief Reset a Q31 complementary history.
 *
 * @param[in,out] ctx Non-NULL initialized context; coefficients and storage are preserved.
 * @param[in] initial Raw Q1.31 new signal history, converted exactly to F31.
 *
 * @return linears_status_t Operation status.
 *
 * @retval LINEARS_OK History replaced (zero-filled for window/cascade filters).
 * @retval LINEARS_ERROR_ARGUMENT ctx is NULL.
 *
 * @note Failure preserves state. Caller must serialize use of ctx; scratch is not cleared.
 */
linears_status_t filters_complementary_q31_reset(filters_complementary_q31_t *ctx, linears_q31_t initial);

/**
 * @brief Process one Q31 complementary sample.
 *
 * @param[in,out] ctx Non-NULL successfully initialized context.
 * @param[in] rate Raw Q1.31 derivative in measurement units per second.
 * @param[in] measurement Raw Q1.31 absolute observation in estimate units.
 * @param[out] output Non-NULL raw Q1.31 destination; disjoint from ctx and retained live prefixes.
 *
 * @return linears_status_t Operation status.
 *
 * @retval LINEARS_OK Output and complete persistent state committed.
 * @retval LINEARS_ERROR_ARGUMENT ctx or output is NULL.
 * @retval LINEARS_ERROR_ALIAS output overlaps a retained live buffer or ctx.
 * @retval LINEARS_ERROR_RANGE A working accumulator overflows or the true output exceeds [-1,+1].
 *
 * @note Failure preserves all persistent state and output; scratch may change. Serialize ctx.
 */
linears_status_t filters_complementary_q31_step(filters_complementary_q31_t *ctx, linears_q31_t rate, linears_q31_t measurement, linears_q31_t *output);

#ifdef __cplusplus
}
#endif
#endif
