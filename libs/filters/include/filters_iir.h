/**
 * @file filters_iir.h
 * @brief Allocation-free biquad/SOS cascades and shared IIR section types.
 *
 * filters_biquad_* runs caller-designed second-order sections in transposed
 * direct form II with caller-owned coefficient, state and scratch arrays. The
 * filters_order_t and filters_iir_section_*_t types are embedded by the
 * frequency-designed filters in filters_lpf.h, filters_hpf.h and filters_bpf.h.
 *
 * Init stores pointers but never allocates: arrays and immutable coefficients
 * must remain alive until the context is no longer used. Only the required
 * prefix of a larger capacity is used. Do not edit an initialized context,
 * coefficients, or retained state; use reset to clear history.
 *
 * f32/f64 use native arithmetic and reject NaN/Inf. q15/q31 signals are raw
 * int16_t/2^15 and int32_t/2^31; Q coefficients and states use
 * linears_fixed_t (int64_t/2^31), so feedback near -2 remains expressible.
 * Every non-OK return preserves persistent state and output; scratch may
 * change. Single-context calls require caller serialization, including
 * interrupts; disjoint contexts are reentrant. No MMIO, I/O, allocation,
 * recursion or mutable static state is used.
 */
#ifndef FILTERS_IIR_H
#define FILTERS_IIR_H

#include "linears.h"
#include "linears_numeric.h"

#ifdef __cplusplus
extern "C" {
#endif

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
 * @brief Roll-off order of a frequency-designed LPF, HPF or BPF.
 *
 * Each transition band falls by 20 dB/decade per order. Order two is a
 * Butterworth (maximally flat) response; order one is the bilinear-transformed
 * RC (LPF/HPF) or series-RLC (BPF) response.
 */
typedef enum {
    FILTERS_ORDER_1 = 1, /**< 20 dB/decade per transition band. */
    FILTERS_ORDER_2 = 2  /**< 40 dB/decade per transition band. */
} filters_order_t;

/* F32 interface. */

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
 * @brief Direct-form-I second-order section with embedded coefficients and history.
 *
 * Computes y = b0*x + b1*x1 + b2*x2 - a1*y1 - a2*y2. The history holds actual
 * past section inputs and outputs, so replacing coefficients never
 * reinterprets state. Owned by LPF/HPF/BPF contexts; do not edit fields.
 */
typedef struct {
    float b0; /**< Current-input feedforward coefficient, dimensionless. */
    float b1; /**< One-delay feedforward coefficient, dimensionless. */
    float b2; /**< Two-delay feedforward coefficient, dimensionless. */
    float a1; /**< One-delay feedback coefficient, subtracted in the recurrence. */
    float a2; /**< Two-delay feedback coefficient, subtracted in the recurrence. */
    float x1; /**< Previous section input, in signal units. */
    float x2; /**< Section input two samples ago, in signal units. */
    float y1; /**< Previous section output, in signal units. */
    float y2; /**< Section output two samples ago, in signal units. */
} filters_iir_section_f32_t;

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
 * @retval LINEARS_ERROR_RANGE A scalar input is nonfinite.
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
 * @retval LINEARS_ERROR_RANGE A scalar input or intermediate is nonfinite.
 *
 * @note Failure preserves all persistent state and output; scratch may change. Serialize ctx.
 */
linears_status_t filters_biquad_f32_step(filters_biquad_f32_t *ctx, float input, float *output);

/* F64 interface. */

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
 * @brief Direct-form-I second-order section with embedded coefficients and history.
 *
 * Computes y = b0*x + b1*x1 + b2*x2 - a1*y1 - a2*y2. The history holds actual
 * past section inputs and outputs, so replacing coefficients never
 * reinterprets state. Owned by LPF/HPF/BPF contexts; do not edit fields.
 */
typedef struct {
    double b0; /**< Current-input feedforward coefficient, dimensionless. */
    double b1; /**< One-delay feedforward coefficient, dimensionless. */
    double b2; /**< Two-delay feedforward coefficient, dimensionless. */
    double a1; /**< One-delay feedback coefficient, subtracted in the recurrence. */
    double a2; /**< Two-delay feedback coefficient, subtracted in the recurrence. */
    double x1; /**< Previous section input, in signal units. */
    double x2; /**< Section input two samples ago, in signal units. */
    double y1; /**< Previous section output, in signal units. */
    double y2; /**< Section output two samples ago, in signal units. */
} filters_iir_section_f64_t;

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
 * @retval LINEARS_ERROR_RANGE A scalar input is nonfinite.
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
 * @retval LINEARS_ERROR_RANGE A scalar input or intermediate is nonfinite.
 *
 * @note Failure preserves all persistent state and output; scratch may change. Serialize ctx.
 */
linears_status_t filters_biquad_f64_step(filters_biquad_f64_t *ctx, double input, double *output);

/* Q15 interface. */

/**
 * @brief Normalized biquad coefficients: denominator 1 + a1*z^-1 + a2*z^-2.
 */
typedef struct {
    linears_fixed_t b0; /**< Current-input feedforward coefficient, dimensionless F31 raw/2^31. */
    linears_fixed_t b1; /**< One-delay feedforward coefficient, dimensionless F31 raw/2^31. */
    linears_fixed_t b2; /**< Two-delay feedforward coefficient, dimensionless F31 raw/2^31. */
    linears_fixed_t a1; /**< One-delay denominator coefficient, F31 raw/2^31, subtracted in recurrence. */
    linears_fixed_t a2; /**< Two-delay denominator coefficient, F31 raw/2^31, subtracted in recurrence. */
} filters_biquad_q15_coefficients_t;

/**
 * @brief Caller-owned SOS cascade with transactional tentative-state scratch.
 */
typedef struct {
    const filters_biquad_q15_coefficients_t *coefficients; /**< Immutable sections array. */
    linears_fixed_t *state; /**< Private F31 pairs (s1,s2), length 2*sections. */
    linears_fixed_t *workspace; /**< Tentative F31 states, length 2*sections; not persistent. */
    size_t sections; /**< Positive number of cascaded second-order sections. */
} filters_biquad_q15_t;

/**
 * @brief Direct-form-I second-order section with embedded coefficients and history.
 *
 * Computes y = b0*x + b1*x1 + b2*x2 - a1*y1 - a2*y2. The history holds actual
 * past section inputs and outputs, so replacing coefficients never
 * reinterprets state. Owned by LPF/HPF/BPF contexts; do not edit fields.
 * Q histories keep F31 precision; an intermediate cascade signal may
 * exceed the Q1 interval without being narrowed.
 */
typedef struct {
    linears_fixed_t b0; /**< Current-input feedforward coefficient, dimensionless, F31 raw/2^31. */
    linears_fixed_t b1; /**< One-delay feedforward coefficient, dimensionless, F31 raw/2^31. */
    linears_fixed_t b2; /**< Two-delay feedforward coefficient, dimensionless, F31 raw/2^31. */
    linears_fixed_t a1; /**< One-delay feedback coefficient, F31 raw/2^31, subtracted in the recurrence. */
    linears_fixed_t a2; /**< Two-delay feedback coefficient, F31 raw/2^31, subtracted in the recurrence. */
    linears_fixed_t x1; /**< Previous section input, in F31 raw/2^31 signal units. */
    linears_fixed_t x2; /**< Section input two samples ago, in F31 raw/2^31 signal units. */
    linears_fixed_t y1; /**< Previous section output, in F31 raw/2^31 signal units. */
    linears_fixed_t y2; /**< Section output two samples ago, in F31 raw/2^31 signal units. */
} filters_iir_section_q15_t;

/**
 * @brief Initialize a Q15 biquad context.
 *
 * All context/array live prefixes must be pairwise disjoint. Arrays remain
 * valid for the context lifetime. Stability and scaling are caller choices.
 * Each section computes y=b0*x+s1, s1=b1*x-a1*y+s2, s2=b2*x-a2*y.
 * States retain F31 precision and headroom; only each section output is Q1.15.
 *
 * @param[out] ctx Non-NULL caller-owned context; unchanged on failure.
 * @param[in] coefficients Non-NULL immutable wide F31 SOS array; a0 is exactly one.
 * @param[in] sections Positive number of cascaded sections.
 * @param[out] state Non-NULL F31 state array, zero-filled; 2*sections int64_t elements used.
 * @param[in] state_capacity Available F31 state elements; at least FILTERS_BIQUAD_STATE_COUNT(sections).
 * @param[in,out] workspace Non-NULL F31 tentative-state array; contents unspecified after step.
 * @param[in] workspace_capacity F31 scratch elements; at least FILTERS_BIQUAD_WORKSPACE_COUNT(sections).
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
linears_status_t filters_biquad_q15_init(filters_biquad_q15_t *ctx, const filters_biquad_q15_coefficients_t *coefficients, size_t sections, linears_fixed_t *state, size_t state_capacity, linears_fixed_t *workspace, size_t workspace_capacity);

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
 * @param[in] input Raw Q1.15 sample in the physical units used for the state.
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
linears_status_t filters_biquad_q15_step(filters_biquad_q15_t *ctx, linears_q15_t input, linears_q15_t *output);

/* Q31 interface. */

/**
 * @brief Normalized biquad coefficients: denominator 1 + a1*z^-1 + a2*z^-2.
 */
typedef struct {
    linears_fixed_t b0; /**< Current-input feedforward coefficient, dimensionless F31 raw/2^31. */
    linears_fixed_t b1; /**< One-delay feedforward coefficient, dimensionless F31 raw/2^31. */
    linears_fixed_t b2; /**< Two-delay feedforward coefficient, dimensionless F31 raw/2^31. */
    linears_fixed_t a1; /**< One-delay denominator coefficient, F31 raw/2^31, subtracted in recurrence. */
    linears_fixed_t a2; /**< Two-delay denominator coefficient, F31 raw/2^31, subtracted in recurrence. */
} filters_biquad_q31_coefficients_t;

/**
 * @brief Caller-owned SOS cascade with transactional tentative-state scratch.
 */
typedef struct {
    const filters_biquad_q31_coefficients_t *coefficients; /**< Immutable sections array. */
    linears_fixed_t *state; /**< Private F31 pairs (s1,s2), length 2*sections. */
    linears_fixed_t *workspace; /**< Tentative F31 states, length 2*sections; not persistent. */
    size_t sections; /**< Positive number of cascaded second-order sections. */
} filters_biquad_q31_t;

/**
 * @brief Direct-form-I second-order section with embedded coefficients and history.
 *
 * Computes y = b0*x + b1*x1 + b2*x2 - a1*y1 - a2*y2. The history holds actual
 * past section inputs and outputs, so replacing coefficients never
 * reinterprets state. Owned by LPF/HPF/BPF contexts; do not edit fields.
 * Q histories keep F31 precision; an intermediate cascade signal may
 * exceed the Q1 interval without being narrowed.
 */
typedef struct {
    linears_fixed_t b0; /**< Current-input feedforward coefficient, dimensionless, F31 raw/2^31. */
    linears_fixed_t b1; /**< One-delay feedforward coefficient, dimensionless, F31 raw/2^31. */
    linears_fixed_t b2; /**< Two-delay feedforward coefficient, dimensionless, F31 raw/2^31. */
    linears_fixed_t a1; /**< One-delay feedback coefficient, F31 raw/2^31, subtracted in the recurrence. */
    linears_fixed_t a2; /**< Two-delay feedback coefficient, F31 raw/2^31, subtracted in the recurrence. */
    linears_fixed_t x1; /**< Previous section input, in F31 raw/2^31 signal units. */
    linears_fixed_t x2; /**< Section input two samples ago, in F31 raw/2^31 signal units. */
    linears_fixed_t y1; /**< Previous section output, in F31 raw/2^31 signal units. */
    linears_fixed_t y2; /**< Section output two samples ago, in F31 raw/2^31 signal units. */
} filters_iir_section_q31_t;

/**
 * @brief Initialize a Q31 biquad context.
 *
 * All context/array live prefixes must be pairwise disjoint. Arrays remain
 * valid for the context lifetime. Stability and scaling are caller choices.
 * Each section computes y=b0*x+s1, s1=b1*x-a1*y+s2, s2=b2*x-a2*y.
 * States retain F31 precision and headroom; only each section output is Q1.31.
 *
 * @param[out] ctx Non-NULL caller-owned context; unchanged on failure.
 * @param[in] coefficients Non-NULL immutable wide F31 SOS array; a0 is exactly one.
 * @param[in] sections Positive number of cascaded sections.
 * @param[out] state Non-NULL F31 state array, zero-filled; 2*sections int64_t elements used.
 * @param[in] state_capacity Available F31 state elements; at least FILTERS_BIQUAD_STATE_COUNT(sections).
 * @param[in,out] workspace Non-NULL F31 tentative-state array; contents unspecified after step.
 * @param[in] workspace_capacity F31 scratch elements; at least FILTERS_BIQUAD_WORKSPACE_COUNT(sections).
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
linears_status_t filters_biquad_q31_init(filters_biquad_q31_t *ctx, const filters_biquad_q31_coefficients_t *coefficients, size_t sections, linears_fixed_t *state, size_t state_capacity, linears_fixed_t *workspace, size_t workspace_capacity);

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
 * @param[in] input Raw Q1.31 sample in the physical units used for the state.
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
linears_status_t filters_biquad_q31_step(filters_biquad_q31_t *ctx, linears_q31_t input, linears_q31_t *output);

#ifdef __cplusplus
}
#endif
#endif
