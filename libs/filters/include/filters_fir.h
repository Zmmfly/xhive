/**
 * @file filters_fir.h
 * @brief Allocation-free FIR filters and windowed-sinc tap design in four formats.
 *
 * A FIR context references a caller-owned immutable coefficient array and a
 * caller-owned delay ring; init stores the pointers and never allocates. Keep
 * both alive and unmodified while the context is used; only the required
 * prefix of a larger state capacity is used. The design helpers fill such a
 * coefficient array from frequencies in Hz and are independent of any context.
 *
 * f32/f64 use native arithmetic and reject NaN/Inf. q15/q31 signals are raw
 * int16_t/2^15 and int32_t/2^31; their coefficients are linears_fixed_t
 * (int64_t/2^31), so gains above one remain expressible. Q steps accumulate
 * exact F62 products and round once; an output outside [-1,+1] returns RANGE.
 *
 * Every non-OK return preserves persistent state and output. Single-context
 * calls require caller serialization, including interrupts; disjoint
 * contexts are reentrant. No MMIO, I/O, allocation, recursion or mutable
 * static state is used.
 */
#ifndef FILTERS_FIR_H
#define FILTERS_FIR_H

#include "linears.h"
#include "linears_numeric.h"

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
 * @retval LINEARS_ERROR_RANGE A scalar input is nonfinite.
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
 * @retval LINEARS_ERROR_RANGE A scalar input or intermediate is nonfinite.
 *
 * @note Failure preserves all persistent state and output; scratch may change. Serialize ctx.
 */
linears_status_t filters_fir_f32_step(filters_fir_f32_t *ctx, float input, float *output);

/**
 * @brief Design F32 low-pass FIR taps with a Hamming-windowed sinc.
 *
 * Writes h[n] = w[n]*sin(2*pi*r*k)/(pi*k) with k=n-(taps-1)/2,
 * r=cutoff_hz/sample_rate_hz and w the Hamming window, normalized to unit DC
 * gain. The response has linear phase, about -6 dB at cutoff_hz, and a
 * transition about 3.3*sample_rate_hz/taps wide.
 *
 * Designed in float arithmetic; the center tap absorbs normalization rounding.
 *
 * @param[out] coefficients Non-NULL array of taps elements in increasing delay order;
 *                          unchanged on failure.
 * @param[in] taps Odd positive tap count; the group delay is (taps-1)/2 samples.
 * @param[in] cutoff_hz Transition-center frequency in Hz, in (0, sample_rate_hz/2).
 * @param[in] sample_rate_hz Positive sample rate in Hz.
 *
 * @return linears_status_t Operation status.
 *
 * @retval LINEARS_OK All taps written.
 * @retval LINEARS_ERROR_ARGUMENT coefficients is NULL or taps is zero.
 * @retval LINEARS_ERROR_RANGE A frequency is nonfinite.
 * @retval LINEARS_ERROR_DOMAIN taps is even, sample_rate_hz is not positive, or
 *                              cutoff_hz is outside (0, sample_rate_hz/2).
 *
 * @note O(taps) sinf/cosf calls. Pass the taps to filters_fir_f32_init.
 */
linears_status_t filters_fir_f32_design_lpf(float *coefficients, size_t taps, float cutoff_hz, float sample_rate_hz);

/**
 * @brief Design F32 high-pass FIR taps by spectral inversion.
 *
 * Writes the impulse delayed by (taps-1)/2 minus the matching
 * filters_fir_f32_design_lpf taps, so the DC gain is zero.
 *
 * Designed in float arithmetic; the center tap absorbs normalization rounding.
 *
 * @param[out] coefficients Non-NULL array of taps elements in increasing delay order;
 *                          unchanged on failure.
 * @param[in] taps Odd positive tap count; the group delay is (taps-1)/2 samples.
 * @param[in] cutoff_hz Transition-center frequency in Hz, in (0, sample_rate_hz/2).
 * @param[in] sample_rate_hz Positive sample rate in Hz.
 *
 * @return linears_status_t Operation status.
 *
 * @retval LINEARS_OK All taps written.
 * @retval LINEARS_ERROR_ARGUMENT coefficients is NULL or taps is zero.
 * @retval LINEARS_ERROR_RANGE A frequency is nonfinite.
 * @retval LINEARS_ERROR_DOMAIN taps is even, sample_rate_hz is not positive, or
 *                              cutoff_hz is outside (0, sample_rate_hz/2).
 *
 * @note O(taps) sinf/cosf calls. Pass the taps to filters_fir_f32_init.
 */
linears_status_t filters_fir_f32_design_hpf(float *coefficients, size_t taps, float cutoff_hz, float sample_rate_hz);

/**
 * @brief Design F32 band-pass FIR taps as a difference of low-pass designs.
 *
 * Writes the unit-DC low-pass taps at high_hz minus those at low_hz, so the
 * DC gain is zero. The passband gain approaches one only when high_hz-low_hz
 * is well above the transition width, about 3.3*sample_rate_hz/taps.
 *
 * Designed in float arithmetic; the center tap absorbs normalization rounding.
 *
 * @param[out] coefficients Non-NULL array of taps elements in increasing delay order;
 *                          unchanged on failure.
 * @param[in] taps Odd positive tap count; the group delay is (taps-1)/2 samples.
 * @param[in] low_hz Lower transition-center frequency in Hz, above zero.
 * @param[in] high_hz Upper transition-center frequency in Hz, above low_hz and below
 *                    sample_rate_hz/2.
 * @param[in] sample_rate_hz Positive sample rate in Hz.
 *
 * @return linears_status_t Operation status.
 *
 * @retval LINEARS_OK All taps written.
 * @retval LINEARS_ERROR_ARGUMENT coefficients is NULL or taps is zero.
 * @retval LINEARS_ERROR_RANGE A frequency is nonfinite.
 * @retval LINEARS_ERROR_DOMAIN taps is even, sample_rate_hz is not positive, or
 *                              0 < low_hz < high_hz < sample_rate_hz/2 does not hold.
 *
 * @note O(taps) sinf/cosf calls. Pass the taps to filters_fir_f32_init.
 */
linears_status_t filters_fir_f32_design_bpf(float *coefficients, size_t taps, float low_hz, float high_hz, float sample_rate_hz);

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
 * @retval LINEARS_ERROR_RANGE A scalar input is nonfinite.
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
 * @retval LINEARS_ERROR_RANGE A scalar input or intermediate is nonfinite.
 *
 * @note Failure preserves all persistent state and output; scratch may change. Serialize ctx.
 */
linears_status_t filters_fir_f64_step(filters_fir_f64_t *ctx, double input, double *output);

/**
 * @brief Design F64 low-pass FIR taps with a Hamming-windowed sinc.
 *
 * Writes h[n] = w[n]*sin(2*pi*r*k)/(pi*k) with k=n-(taps-1)/2,
 * r=cutoff_hz/sample_rate_hz and w the Hamming window, normalized to unit DC
 * gain. The response has linear phase, about -6 dB at cutoff_hz, and a
 * transition about 3.3*sample_rate_hz/taps wide.
 *
 * Designed in double arithmetic; the center tap absorbs normalization rounding.
 *
 * @param[out] coefficients Non-NULL array of taps elements in increasing delay order;
 *                          unchanged on failure.
 * @param[in] taps Odd positive tap count; the group delay is (taps-1)/2 samples.
 * @param[in] cutoff_hz Transition-center frequency in Hz, in (0, sample_rate_hz/2).
 * @param[in] sample_rate_hz Positive sample rate in Hz.
 *
 * @return linears_status_t Operation status.
 *
 * @retval LINEARS_OK All taps written.
 * @retval LINEARS_ERROR_ARGUMENT coefficients is NULL or taps is zero.
 * @retval LINEARS_ERROR_RANGE A frequency is nonfinite.
 * @retval LINEARS_ERROR_DOMAIN taps is even, sample_rate_hz is not positive, or
 *                              cutoff_hz is outside (0, sample_rate_hz/2).
 *
 * @note O(taps) sin/cos calls. Pass the taps to filters_fir_f64_init.
 */
linears_status_t filters_fir_f64_design_lpf(double *coefficients, size_t taps, double cutoff_hz, double sample_rate_hz);

/**
 * @brief Design F64 high-pass FIR taps by spectral inversion.
 *
 * Writes the impulse delayed by (taps-1)/2 minus the matching
 * filters_fir_f64_design_lpf taps, so the DC gain is zero.
 *
 * Designed in double arithmetic; the center tap absorbs normalization rounding.
 *
 * @param[out] coefficients Non-NULL array of taps elements in increasing delay order;
 *                          unchanged on failure.
 * @param[in] taps Odd positive tap count; the group delay is (taps-1)/2 samples.
 * @param[in] cutoff_hz Transition-center frequency in Hz, in (0, sample_rate_hz/2).
 * @param[in] sample_rate_hz Positive sample rate in Hz.
 *
 * @return linears_status_t Operation status.
 *
 * @retval LINEARS_OK All taps written.
 * @retval LINEARS_ERROR_ARGUMENT coefficients is NULL or taps is zero.
 * @retval LINEARS_ERROR_RANGE A frequency is nonfinite.
 * @retval LINEARS_ERROR_DOMAIN taps is even, sample_rate_hz is not positive, or
 *                              cutoff_hz is outside (0, sample_rate_hz/2).
 *
 * @note O(taps) sin/cos calls. Pass the taps to filters_fir_f64_init.
 */
linears_status_t filters_fir_f64_design_hpf(double *coefficients, size_t taps, double cutoff_hz, double sample_rate_hz);

/**
 * @brief Design F64 band-pass FIR taps as a difference of low-pass designs.
 *
 * Writes the unit-DC low-pass taps at high_hz minus those at low_hz, so the
 * DC gain is zero. The passband gain approaches one only when high_hz-low_hz
 * is well above the transition width, about 3.3*sample_rate_hz/taps.
 *
 * Designed in double arithmetic; the center tap absorbs normalization rounding.
 *
 * @param[out] coefficients Non-NULL array of taps elements in increasing delay order;
 *                          unchanged on failure.
 * @param[in] taps Odd positive tap count; the group delay is (taps-1)/2 samples.
 * @param[in] low_hz Lower transition-center frequency in Hz, above zero.
 * @param[in] high_hz Upper transition-center frequency in Hz, above low_hz and below
 *                    sample_rate_hz/2.
 * @param[in] sample_rate_hz Positive sample rate in Hz.
 *
 * @return linears_status_t Operation status.
 *
 * @retval LINEARS_OK All taps written.
 * @retval LINEARS_ERROR_ARGUMENT coefficients is NULL or taps is zero.
 * @retval LINEARS_ERROR_RANGE A frequency is nonfinite.
 * @retval LINEARS_ERROR_DOMAIN taps is even, sample_rate_hz is not positive, or
 *                              0 < low_hz < high_hz < sample_rate_hz/2 does not hold.
 *
 * @note O(taps) sin/cos calls. Pass the taps to filters_fir_f64_init.
 */
linears_status_t filters_fir_f64_design_bpf(double *coefficients, size_t taps, double low_hz, double high_hz, double sample_rate_hz);

/* Q15 interface. */

/**
 * @brief Caller-owned FIR delay ring and immutable coefficient reference.
 */
typedef struct {
    const linears_fixed_t *coefficients; /**< h[0] multiplies the current input; length taps. */
    linears_q15_t *state; /**< Private zero-padded delay ring; length taps. */
    size_t taps; /**< Positive coefficient and history length, in samples. */
    size_t head; /**< Next replaced ring index; maintained by the API. */
} filters_fir_q15_t;

/**
 * @brief Initialize a Q15 fir context.
 *
 * Coefficients, state and context must be pairwise disjoint. Arrays remain
 * valid for the context lifetime. Coefficient signs and gains are unrestricted.
 *
 * @param[out] ctx Non-NULL caller-owned context; unchanged on failure.
 * @param[in] coefficients Non-NULL immutable wide F31 (int64_t/2^31) taps, in increasing delay order.
 * @param[in] taps Positive number of coefficients, in samples.
 * @param[out] state Non-NULL history array, zero-filled in its required prefix.
 * @param[in] state_capacity Available state elements; at least FILTERS_FIR_STATE_COUNT(taps).
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
linears_status_t filters_fir_q15_init(filters_fir_q15_t *ctx, const linears_fixed_t *coefficients, size_t taps, linears_q15_t *state, size_t state_capacity);

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
linears_status_t filters_fir_q15_step(filters_fir_q15_t *ctx, linears_q15_t input, linears_q15_t *output);

/**
 * @brief Design Q15 low-pass FIR taps with a Hamming-windowed sinc.
 *
 * Writes h[n] = w[n]*sin(2*pi*r*k)/(pi*k) with k=n-(taps-1)/2,
 * r=cutoff_hz/sample_rate_hz and w the Hamming window, normalized to unit DC
 * gain. The response has linear phase, about -6 dB at cutoff_hz, and a
 * transition about 3.3*sample_rate_hz/taps wide.
 *
 * Designed in double from float frequencies. Each tap is rounded to F31 and
 * the center tap absorbs the rounding, so the taps sum exactly to LINEARS_FIXED_ONE.
 *
 * @param[out] coefficients Non-NULL F31 (int64_t/2^31) array of taps elements in increasing
 *                          delay order; unchanged on failure.
 * @param[in] taps Odd positive tap count; the group delay is (taps-1)/2 samples.
 * @param[in] cutoff_hz Transition-center frequency in Hz, in (0, sample_rate_hz/2).
 * @param[in] sample_rate_hz Positive sample rate in Hz.
 *
 * @return linears_status_t Operation status.
 *
 * @retval LINEARS_OK All taps written.
 * @retval LINEARS_ERROR_ARGUMENT coefficients is NULL or taps is zero.
 * @retval LINEARS_ERROR_RANGE A frequency is nonfinite.
 * @retval LINEARS_ERROR_DOMAIN taps is even, sample_rate_hz is not positive, or
 *                              cutoff_hz is outside (0, sample_rate_hz/2).
 *
 * @note O(taps) sin/cos calls. Pass the taps to filters_fir_q15_init.
 */
linears_status_t filters_fir_q15_design_lpf(linears_fixed_t *coefficients, size_t taps, float cutoff_hz, float sample_rate_hz);

/**
 * @brief Design Q15 high-pass FIR taps by spectral inversion.
 *
 * Writes the impulse delayed by (taps-1)/2 minus the matching
 * filters_fir_q15_design_lpf taps, so the DC gain is zero.
 *
 * Designed in double from float frequencies. Each tap is rounded to F31 and
 * the center tap absorbs the rounding, so the taps sum exactly to zero.
 *
 * @param[out] coefficients Non-NULL F31 (int64_t/2^31) array of taps elements in increasing
 *                          delay order; unchanged on failure.
 * @param[in] taps Odd positive tap count; the group delay is (taps-1)/2 samples.
 * @param[in] cutoff_hz Transition-center frequency in Hz, in (0, sample_rate_hz/2).
 * @param[in] sample_rate_hz Positive sample rate in Hz.
 *
 * @return linears_status_t Operation status.
 *
 * @retval LINEARS_OK All taps written.
 * @retval LINEARS_ERROR_ARGUMENT coefficients is NULL or taps is zero.
 * @retval LINEARS_ERROR_RANGE A frequency is nonfinite.
 * @retval LINEARS_ERROR_DOMAIN taps is even, sample_rate_hz is not positive, or
 *                              cutoff_hz is outside (0, sample_rate_hz/2).
 *
 * @note O(taps) sin/cos calls. Pass the taps to filters_fir_q15_init.
 */
linears_status_t filters_fir_q15_design_hpf(linears_fixed_t *coefficients, size_t taps, float cutoff_hz, float sample_rate_hz);

/**
 * @brief Design Q15 band-pass FIR taps as a difference of low-pass designs.
 *
 * Writes the unit-DC low-pass taps at high_hz minus those at low_hz, so the
 * DC gain is zero. The passband gain approaches one only when high_hz-low_hz
 * is well above the transition width, about 3.3*sample_rate_hz/taps.
 *
 * Designed in double from float frequencies. Each tap is rounded to F31 and
 * the center tap absorbs the rounding, so the taps sum exactly to zero.
 *
 * @param[out] coefficients Non-NULL F31 (int64_t/2^31) array of taps elements in increasing
 *                          delay order; unchanged on failure.
 * @param[in] taps Odd positive tap count; the group delay is (taps-1)/2 samples.
 * @param[in] low_hz Lower transition-center frequency in Hz, above zero.
 * @param[in] high_hz Upper transition-center frequency in Hz, above low_hz and below
 *                    sample_rate_hz/2.
 * @param[in] sample_rate_hz Positive sample rate in Hz.
 *
 * @return linears_status_t Operation status.
 *
 * @retval LINEARS_OK All taps written.
 * @retval LINEARS_ERROR_ARGUMENT coefficients is NULL or taps is zero.
 * @retval LINEARS_ERROR_RANGE A frequency is nonfinite.
 * @retval LINEARS_ERROR_DOMAIN taps is even, sample_rate_hz is not positive, or
 *                              0 < low_hz < high_hz < sample_rate_hz/2 does not hold.
 *
 * @note O(taps) sin/cos calls. Pass the taps to filters_fir_q15_init.
 */
linears_status_t filters_fir_q15_design_bpf(linears_fixed_t *coefficients, size_t taps, float low_hz, float high_hz, float sample_rate_hz);

/* Q31 interface. */

/**
 * @brief Caller-owned FIR delay ring and immutable coefficient reference.
 */
typedef struct {
    const linears_fixed_t *coefficients; /**< h[0] multiplies the current input; length taps. */
    linears_q31_t *state; /**< Private zero-padded delay ring; length taps. */
    size_t taps; /**< Positive coefficient and history length, in samples. */
    size_t head; /**< Next replaced ring index; maintained by the API. */
} filters_fir_q31_t;

/**
 * @brief Initialize a Q31 fir context.
 *
 * Coefficients, state and context must be pairwise disjoint. Arrays remain
 * valid for the context lifetime. Coefficient signs and gains are unrestricted.
 *
 * @param[out] ctx Non-NULL caller-owned context; unchanged on failure.
 * @param[in] coefficients Non-NULL immutable wide F31 (int64_t/2^31) taps, in increasing delay order.
 * @param[in] taps Positive number of coefficients, in samples.
 * @param[out] state Non-NULL history array, zero-filled in its required prefix.
 * @param[in] state_capacity Available state elements; at least FILTERS_FIR_STATE_COUNT(taps).
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
linears_status_t filters_fir_q31_init(filters_fir_q31_t *ctx, const linears_fixed_t *coefficients, size_t taps, linears_q31_t *state, size_t state_capacity);

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
linears_status_t filters_fir_q31_step(filters_fir_q31_t *ctx, linears_q31_t input, linears_q31_t *output);

/**
 * @brief Design Q31 low-pass FIR taps with a Hamming-windowed sinc.
 *
 * Writes h[n] = w[n]*sin(2*pi*r*k)/(pi*k) with k=n-(taps-1)/2,
 * r=cutoff_hz/sample_rate_hz and w the Hamming window, normalized to unit DC
 * gain. The response has linear phase, about -6 dB at cutoff_hz, and a
 * transition about 3.3*sample_rate_hz/taps wide.
 *
 * Designed in double from float frequencies. Each tap is rounded to F31 and
 * the center tap absorbs the rounding, so the taps sum exactly to LINEARS_FIXED_ONE.
 *
 * @param[out] coefficients Non-NULL F31 (int64_t/2^31) array of taps elements in increasing
 *                          delay order; unchanged on failure.
 * @param[in] taps Odd positive tap count; the group delay is (taps-1)/2 samples.
 * @param[in] cutoff_hz Transition-center frequency in Hz, in (0, sample_rate_hz/2).
 * @param[in] sample_rate_hz Positive sample rate in Hz.
 *
 * @return linears_status_t Operation status.
 *
 * @retval LINEARS_OK All taps written.
 * @retval LINEARS_ERROR_ARGUMENT coefficients is NULL or taps is zero.
 * @retval LINEARS_ERROR_RANGE A frequency is nonfinite.
 * @retval LINEARS_ERROR_DOMAIN taps is even, sample_rate_hz is not positive, or
 *                              cutoff_hz is outside (0, sample_rate_hz/2).
 *
 * @note O(taps) sin/cos calls. Pass the taps to filters_fir_q31_init.
 */
linears_status_t filters_fir_q31_design_lpf(linears_fixed_t *coefficients, size_t taps, float cutoff_hz, float sample_rate_hz);

/**
 * @brief Design Q31 high-pass FIR taps by spectral inversion.
 *
 * Writes the impulse delayed by (taps-1)/2 minus the matching
 * filters_fir_q31_design_lpf taps, so the DC gain is zero.
 *
 * Designed in double from float frequencies. Each tap is rounded to F31 and
 * the center tap absorbs the rounding, so the taps sum exactly to zero.
 *
 * @param[out] coefficients Non-NULL F31 (int64_t/2^31) array of taps elements in increasing
 *                          delay order; unchanged on failure.
 * @param[in] taps Odd positive tap count; the group delay is (taps-1)/2 samples.
 * @param[in] cutoff_hz Transition-center frequency in Hz, in (0, sample_rate_hz/2).
 * @param[in] sample_rate_hz Positive sample rate in Hz.
 *
 * @return linears_status_t Operation status.
 *
 * @retval LINEARS_OK All taps written.
 * @retval LINEARS_ERROR_ARGUMENT coefficients is NULL or taps is zero.
 * @retval LINEARS_ERROR_RANGE A frequency is nonfinite.
 * @retval LINEARS_ERROR_DOMAIN taps is even, sample_rate_hz is not positive, or
 *                              cutoff_hz is outside (0, sample_rate_hz/2).
 *
 * @note O(taps) sin/cos calls. Pass the taps to filters_fir_q31_init.
 */
linears_status_t filters_fir_q31_design_hpf(linears_fixed_t *coefficients, size_t taps, float cutoff_hz, float sample_rate_hz);

/**
 * @brief Design Q31 band-pass FIR taps as a difference of low-pass designs.
 *
 * Writes the unit-DC low-pass taps at high_hz minus those at low_hz, so the
 * DC gain is zero. The passband gain approaches one only when high_hz-low_hz
 * is well above the transition width, about 3.3*sample_rate_hz/taps.
 *
 * Designed in double from float frequencies. Each tap is rounded to F31 and
 * the center tap absorbs the rounding, so the taps sum exactly to zero.
 *
 * @param[out] coefficients Non-NULL F31 (int64_t/2^31) array of taps elements in increasing
 *                          delay order; unchanged on failure.
 * @param[in] taps Odd positive tap count; the group delay is (taps-1)/2 samples.
 * @param[in] low_hz Lower transition-center frequency in Hz, above zero.
 * @param[in] high_hz Upper transition-center frequency in Hz, above low_hz and below
 *                    sample_rate_hz/2.
 * @param[in] sample_rate_hz Positive sample rate in Hz.
 *
 * @return linears_status_t Operation status.
 *
 * @retval LINEARS_OK All taps written.
 * @retval LINEARS_ERROR_ARGUMENT coefficients is NULL or taps is zero.
 * @retval LINEARS_ERROR_RANGE A frequency is nonfinite.
 * @retval LINEARS_ERROR_DOMAIN taps is even, sample_rate_hz is not positive, or
 *                              0 < low_hz < high_hz < sample_rate_hz/2 does not hold.
 *
 * @note O(taps) sin/cos calls. Pass the taps to filters_fir_q31_init.
 */
linears_status_t filters_fir_q31_design_bpf(linears_fixed_t *coefficients, size_t taps, float low_hz, float high_hz, float sample_rate_hz);

#ifdef __cplusplus
}
#endif
#endif
