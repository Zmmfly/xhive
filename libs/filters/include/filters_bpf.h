/**
 * @file filters_bpf.h
 * @brief Band-pass filters designed from band edges or a center frequency and Q.
 *
 * Order one is the series-RLC response H(s)=B*s/(s^2+B*s+w0^2). Order two
 * applies the low-pass-to-band-pass transform to the second-order Butterworth
 * prototype and is stored as two stagger-tuned sections. Frequencies are
 * prewarped with w=tan(pi*f/sample_rate_hz) before the bilinear transform:
 * - band form: w0=sqrt(wl*wh) and B=wh-wl, so the designed response is exactly
 *   -3 dB at low_hz and high_hz;
 * - center form: w0=tan(pi*center_hz/sample_rate_hz) and B=w0/q.
 * Both forms have a designed peak gain of one at w0 and zero gain at DC and
 * sample_rate_hz/2. center_hz is the geometric center of the prewarped edges,
 * not their arithmetic mean.
 *
 * Each context embeds its sections; no external arrays are needed. Call step
 * once per sample at the configured rate. The set functions replace the
 * coefficients and keep the input/output history; either form may retune a
 * context initialized with the other. f32 designs in float and f64 in double.
 * Q15/Q31 accept float parameters, design in double, round coefficients to
 * F31 once, and step with integer arithmetic only; Q history is F31, the
 * signal between cascaded sections is not narrowed, and the Q output is
 * rounded once from the exact F62 sum.
 *
 * Contexts are caller-owned; do not edit fields. Serialize calls on one
 * context, including interrupts; distinct contexts are reentrant. No
 * allocation, locks, I/O or MMIO occurs. Every failure leaves the context
 * and output unchanged.
 */
#ifndef FILTERS_BPF_H
#define FILTERS_BPF_H

#include "filters_iir.h"

#ifdef __cplusplus
extern "C" {
#endif

/* F32 interface. */

/**
 * @brief Caller-owned F32 band-pass filter with embedded coefficients and history.
 */
typedef struct {
    filters_order_t order; /**< Roll-off order; fixed at init. */
    filters_iir_section_f32_t sections[2]; /**< Cascade; sections[1] is zero and unused for order one. */
} filters_bpf_f32_t;

/**
 * @brief Initialize a F32 band-pass filter from band edges.
 *
 * The history represents initial held forever with a settled zero output, so a
 * DC offset in the first samples causes no startup transient.
 *
 * @param[out] ctx Non-NULL caller-owned context; unchanged on failure.
 * @param[in] order FILTERS_ORDER_1 (one section) or FILTERS_ORDER_2 (two sections).
 * @param[in] low_hz Lower -3 dB frequency in Hz, above zero.
 * @param[in] high_hz Upper -3 dB frequency in Hz, above low_hz and below sample_rate_hz/2.
 * @param[in] sample_rate_hz Positive step rate in Hz.
 * @param[in] initial Input level held before the first step, in signal units.
 *
 * @return linears_status_t Operation status.
 *
 * @retval LINEARS_OK Coefficients designed and history set.
 * @retval LINEARS_ERROR_ARGUMENT ctx is NULL.
 * @retval LINEARS_ERROR_RANGE A frequency or initial is nonfinite.
 * @retval LINEARS_ERROR_DOMAIN order is invalid, sample_rate_hz is not positive, or the
 *                              frequencies do not satisfy 0 < low_hz < high_hz <
 *                              sample_rate_hz/2 with distinct prewarped edges.
 *
 * @note Designs with tanf. Serialize use of ctx.
 */
linears_status_t filters_bpf_f32_init_band(filters_bpf_f32_t *ctx, filters_order_t order, float low_hz, float high_hz, float sample_rate_hz, float initial);

/**
 * @brief Initialize a F32 band-pass filter from a center frequency and Q.
 *
 * The history represents initial held forever with a settled zero output, so a
 * DC offset in the first samples causes no startup transient.
 *
 * @param[out] ctx Non-NULL caller-owned context; unchanged on failure.
 * @param[in] order FILTERS_ORDER_1 (one section) or FILTERS_ORDER_2 (two sections).
 * @param[in] center_hz Peak frequency in Hz, in (0, sample_rate_hz/2).
 * @param[in] q Positive quality factor w0/B of the prewarped design.
 * @param[in] sample_rate_hz Positive step rate in Hz.
 * @param[in] initial Input level held before the first step, in signal units.
 *
 * @return linears_status_t Operation status.
 *
 * @retval LINEARS_OK Coefficients designed and history set.
 * @retval LINEARS_ERROR_ARGUMENT ctx is NULL.
 * @retval LINEARS_ERROR_RANGE A frequency, q or initial is nonfinite, or a tiny q
 *                             makes a designed coefficient unrepresentable.
 * @retval LINEARS_ERROR_DOMAIN order is invalid, sample_rate_hz is not positive, center_hz
 *                              is outside (0, sample_rate_hz/2), q is not positive, or
 *                              the prewarped bandwidth underflows to zero.
 *
 * @note Designs with tanf. Serialize use of ctx.
 */
linears_status_t filters_bpf_f32_init_center(filters_bpf_f32_t *ctx, filters_order_t order, float center_hz, float q, float sample_rate_hz, float initial);

/**
 * @brief Retune a F32 band-pass filter to new band edges.
 *
 * The order and input/output history are kept.
 *
 * @param[in,out] ctx Non-NULL initialized context; unchanged on failure.
 * @param[in] low_hz New lower -3 dB frequency in Hz, above zero.
 * @param[in] high_hz New upper -3 dB frequency in Hz, above low_hz and below sample_rate_hz/2.
 * @param[in] sample_rate_hz New positive step rate in Hz.
 *
 * @return linears_status_t Operation status.
 *
 * @retval LINEARS_OK Coefficients replaced; history unchanged.
 * @retval LINEARS_ERROR_ARGUMENT ctx is NULL.
 * @retval LINEARS_ERROR_RANGE A frequency is nonfinite.
 * @retval LINEARS_ERROR_DOMAIN sample_rate_hz is not positive, or the
 *                              frequencies do not satisfy 0 < low_hz < high_hz <
 *                              sample_rate_hz/2 with distinct prewarped edges.
 *
 * @note Designs with tanf. Serialize use of ctx.
 */
linears_status_t filters_bpf_f32_set_band(filters_bpf_f32_t *ctx, float low_hz, float high_hz, float sample_rate_hz);

/**
 * @brief Retune a F32 band-pass filter to new center frequency and Q.
 *
 * The order and input/output history are kept.
 *
 * @param[in,out] ctx Non-NULL initialized context; unchanged on failure.
 * @param[in] center_hz New peak frequency in Hz, in (0, sample_rate_hz/2).
 * @param[in] q New positive quality factor w0/B of the prewarped design.
 * @param[in] sample_rate_hz New positive step rate in Hz.
 *
 * @return linears_status_t Operation status.
 *
 * @retval LINEARS_OK Coefficients replaced; history unchanged.
 * @retval LINEARS_ERROR_ARGUMENT ctx is NULL.
 * @retval LINEARS_ERROR_RANGE A frequency or q is nonfinite, or a tiny q
 *                             makes a designed coefficient unrepresentable.
 * @retval LINEARS_ERROR_DOMAIN sample_rate_hz is not positive, center_hz
 *                              is outside (0, sample_rate_hz/2), q is not positive, or
 *                              the prewarped bandwidth underflows to zero.
 *
 * @note Designs with tanf. Serialize use of ctx.
 */
linears_status_t filters_bpf_f32_set_center(filters_bpf_f32_t *ctx, float center_hz, float q, float sample_rate_hz);

/**
 * @brief Restart a F32 band-pass filter at a steady state.
 *
 * The history represents initial held forever with a settled zero output.
 *
 * @param[in,out] ctx Non-NULL initialized context; coefficients and order are preserved.
 * @param[in] initial Input level held before the next step, in signal units.
 *
 * @return linears_status_t Operation status.
 *
 * @retval LINEARS_OK History replaced.
 * @retval LINEARS_ERROR_ARGUMENT ctx is NULL.
 * @retval LINEARS_ERROR_RANGE initial is nonfinite.
 *
 * @note Failure leaves ctx unchanged. Serialize use of ctx.
 */
linears_status_t filters_bpf_f32_reset(filters_bpf_f32_t *ctx, float initial);

/**
 * @brief Filter one sample through a F32 band-pass filter.
 *
 * Call once per period of the configured sample rate.
 *
 * @param[in,out] ctx Non-NULL initialized context.
 * @param[in] input One sample in signal units.
 * @param[out] output Non-NULL destination; must not overlap ctx.
 *
 * @return linears_status_t Operation status.
 *
 * @retval LINEARS_OK Output and history committed.
 * @retval LINEARS_ERROR_ARGUMENT ctx or output is NULL.
 * @retval LINEARS_ERROR_ALIAS output overlaps ctx.
 * @retval LINEARS_ERROR_RANGE input or an intermediate result is nonfinite.
 *
 * @note Failure leaves ctx and output unchanged. Serialize use of ctx.
 */
linears_status_t filters_bpf_f32_step(filters_bpf_f32_t *ctx, float input, float *output);

/* F64 interface. */

/**
 * @brief Caller-owned F64 band-pass filter with embedded coefficients and history.
 */
typedef struct {
    filters_order_t order; /**< Roll-off order; fixed at init. */
    filters_iir_section_f64_t sections[2]; /**< Cascade; sections[1] is zero and unused for order one. */
} filters_bpf_f64_t;

/**
 * @brief Initialize a F64 band-pass filter from band edges.
 *
 * The history represents initial held forever with a settled zero output, so a
 * DC offset in the first samples causes no startup transient.
 *
 * @param[out] ctx Non-NULL caller-owned context; unchanged on failure.
 * @param[in] order FILTERS_ORDER_1 (one section) or FILTERS_ORDER_2 (two sections).
 * @param[in] low_hz Lower -3 dB frequency in Hz, above zero.
 * @param[in] high_hz Upper -3 dB frequency in Hz, above low_hz and below sample_rate_hz/2.
 * @param[in] sample_rate_hz Positive step rate in Hz.
 * @param[in] initial Input level held before the first step, in signal units.
 *
 * @return linears_status_t Operation status.
 *
 * @retval LINEARS_OK Coefficients designed and history set.
 * @retval LINEARS_ERROR_ARGUMENT ctx is NULL.
 * @retval LINEARS_ERROR_RANGE A frequency or initial is nonfinite.
 * @retval LINEARS_ERROR_DOMAIN order is invalid, sample_rate_hz is not positive, or the
 *                              frequencies do not satisfy 0 < low_hz < high_hz <
 *                              sample_rate_hz/2 with distinct prewarped edges.
 *
 * @note Designs with tan. Serialize use of ctx.
 */
linears_status_t filters_bpf_f64_init_band(filters_bpf_f64_t *ctx, filters_order_t order, double low_hz, double high_hz, double sample_rate_hz, double initial);

/**
 * @brief Initialize a F64 band-pass filter from a center frequency and Q.
 *
 * The history represents initial held forever with a settled zero output, so a
 * DC offset in the first samples causes no startup transient.
 *
 * @param[out] ctx Non-NULL caller-owned context; unchanged on failure.
 * @param[in] order FILTERS_ORDER_1 (one section) or FILTERS_ORDER_2 (two sections).
 * @param[in] center_hz Peak frequency in Hz, in (0, sample_rate_hz/2).
 * @param[in] q Positive quality factor w0/B of the prewarped design.
 * @param[in] sample_rate_hz Positive step rate in Hz.
 * @param[in] initial Input level held before the first step, in signal units.
 *
 * @return linears_status_t Operation status.
 *
 * @retval LINEARS_OK Coefficients designed and history set.
 * @retval LINEARS_ERROR_ARGUMENT ctx is NULL.
 * @retval LINEARS_ERROR_RANGE A frequency, q or initial is nonfinite, or a tiny q
 *                             makes a designed coefficient unrepresentable.
 * @retval LINEARS_ERROR_DOMAIN order is invalid, sample_rate_hz is not positive, center_hz
 *                              is outside (0, sample_rate_hz/2), q is not positive, or
 *                              the prewarped bandwidth underflows to zero.
 *
 * @note Designs with tan. Serialize use of ctx.
 */
linears_status_t filters_bpf_f64_init_center(filters_bpf_f64_t *ctx, filters_order_t order, double center_hz, double q, double sample_rate_hz, double initial);

/**
 * @brief Retune a F64 band-pass filter to new band edges.
 *
 * The order and input/output history are kept.
 *
 * @param[in,out] ctx Non-NULL initialized context; unchanged on failure.
 * @param[in] low_hz New lower -3 dB frequency in Hz, above zero.
 * @param[in] high_hz New upper -3 dB frequency in Hz, above low_hz and below sample_rate_hz/2.
 * @param[in] sample_rate_hz New positive step rate in Hz.
 *
 * @return linears_status_t Operation status.
 *
 * @retval LINEARS_OK Coefficients replaced; history unchanged.
 * @retval LINEARS_ERROR_ARGUMENT ctx is NULL.
 * @retval LINEARS_ERROR_RANGE A frequency is nonfinite.
 * @retval LINEARS_ERROR_DOMAIN sample_rate_hz is not positive, or the
 *                              frequencies do not satisfy 0 < low_hz < high_hz <
 *                              sample_rate_hz/2 with distinct prewarped edges.
 *
 * @note Designs with tan. Serialize use of ctx.
 */
linears_status_t filters_bpf_f64_set_band(filters_bpf_f64_t *ctx, double low_hz, double high_hz, double sample_rate_hz);

/**
 * @brief Retune a F64 band-pass filter to new center frequency and Q.
 *
 * The order and input/output history are kept.
 *
 * @param[in,out] ctx Non-NULL initialized context; unchanged on failure.
 * @param[in] center_hz New peak frequency in Hz, in (0, sample_rate_hz/2).
 * @param[in] q New positive quality factor w0/B of the prewarped design.
 * @param[in] sample_rate_hz New positive step rate in Hz.
 *
 * @return linears_status_t Operation status.
 *
 * @retval LINEARS_OK Coefficients replaced; history unchanged.
 * @retval LINEARS_ERROR_ARGUMENT ctx is NULL.
 * @retval LINEARS_ERROR_RANGE A frequency or q is nonfinite, or a tiny q
 *                             makes a designed coefficient unrepresentable.
 * @retval LINEARS_ERROR_DOMAIN sample_rate_hz is not positive, center_hz
 *                              is outside (0, sample_rate_hz/2), q is not positive, or
 *                              the prewarped bandwidth underflows to zero.
 *
 * @note Designs with tan. Serialize use of ctx.
 */
linears_status_t filters_bpf_f64_set_center(filters_bpf_f64_t *ctx, double center_hz, double q, double sample_rate_hz);

/**
 * @brief Restart a F64 band-pass filter at a steady state.
 *
 * The history represents initial held forever with a settled zero output.
 *
 * @param[in,out] ctx Non-NULL initialized context; coefficients and order are preserved.
 * @param[in] initial Input level held before the next step, in signal units.
 *
 * @return linears_status_t Operation status.
 *
 * @retval LINEARS_OK History replaced.
 * @retval LINEARS_ERROR_ARGUMENT ctx is NULL.
 * @retval LINEARS_ERROR_RANGE initial is nonfinite.
 *
 * @note Failure leaves ctx unchanged. Serialize use of ctx.
 */
linears_status_t filters_bpf_f64_reset(filters_bpf_f64_t *ctx, double initial);

/**
 * @brief Filter one sample through a F64 band-pass filter.
 *
 * Call once per period of the configured sample rate.
 *
 * @param[in,out] ctx Non-NULL initialized context.
 * @param[in] input One sample in signal units.
 * @param[out] output Non-NULL destination; must not overlap ctx.
 *
 * @return linears_status_t Operation status.
 *
 * @retval LINEARS_OK Output and history committed.
 * @retval LINEARS_ERROR_ARGUMENT ctx or output is NULL.
 * @retval LINEARS_ERROR_ALIAS output overlaps ctx.
 * @retval LINEARS_ERROR_RANGE input or an intermediate result is nonfinite.
 *
 * @note Failure leaves ctx and output unchanged. Serialize use of ctx.
 */
linears_status_t filters_bpf_f64_step(filters_bpf_f64_t *ctx, double input, double *output);

/* Q15 interface. */

/**
 * @brief Caller-owned Q15 band-pass filter with embedded coefficients and history.
 */
typedef struct {
    filters_order_t order; /**< Roll-off order; fixed at init. */
    filters_iir_section_q15_t sections[2]; /**< Cascade; sections[1] is zero and unused for order one. */
} filters_bpf_q15_t;

/**
 * @brief Initialize a Q15 band-pass filter from band edges.
 *
 * The history represents initial held forever with a settled zero output, so a
 * DC offset in the first samples causes no startup transient.
 *
 * @param[out] ctx Non-NULL caller-owned context; unchanged on failure.
 * @param[in] order FILTERS_ORDER_1 (one section) or FILTERS_ORDER_2 (two sections).
 * @param[in] low_hz Lower -3 dB frequency in Hz, above zero.
 * @param[in] high_hz Upper -3 dB frequency in Hz, above low_hz and below sample_rate_hz/2.
 * @param[in] sample_rate_hz Positive step rate in Hz.
 * @param[in] initial Raw Q1.15 input level held before the first step; converted exactly to F31.
 *
 * @return linears_status_t Operation status.
 *
 * @retval LINEARS_OK Coefficients designed and history set.
 * @retval LINEARS_ERROR_ARGUMENT ctx is NULL.
 * @retval LINEARS_ERROR_RANGE A frequency is nonfinite.
 * @retval LINEARS_ERROR_DOMAIN order is invalid, sample_rate_hz is not positive, or the
 *                              frequencies do not satisfy 0 < low_hz < high_hz <
 *                              sample_rate_hz/2 with distinct prewarped edges.
 *
 * @note Designs in double (tan) and rounds coefficients to F31; slow without a
 *       double-precision FPU. Serialize use of ctx.
 */
linears_status_t filters_bpf_q15_init_band(filters_bpf_q15_t *ctx, filters_order_t order, float low_hz, float high_hz, float sample_rate_hz, linears_q15_t initial);

/**
 * @brief Initialize a Q15 band-pass filter from a center frequency and Q.
 *
 * The history represents initial held forever with a settled zero output, so a
 * DC offset in the first samples causes no startup transient.
 *
 * @param[out] ctx Non-NULL caller-owned context; unchanged on failure.
 * @param[in] order FILTERS_ORDER_1 (one section) or FILTERS_ORDER_2 (two sections).
 * @param[in] center_hz Peak frequency in Hz, in (0, sample_rate_hz/2).
 * @param[in] q Positive quality factor w0/B of the prewarped design.
 * @param[in] sample_rate_hz Positive step rate in Hz.
 * @param[in] initial Raw Q1.15 input level held before the first step; converted exactly to F31.
 *
 * @return linears_status_t Operation status.
 *
 * @retval LINEARS_OK Coefficients designed and history set.
 * @retval LINEARS_ERROR_ARGUMENT ctx is NULL.
 * @retval LINEARS_ERROR_RANGE A frequency or q is nonfinite, or a tiny q
 *                             makes a designed coefficient unrepresentable.
 * @retval LINEARS_ERROR_DOMAIN order is invalid, sample_rate_hz is not positive, center_hz
 *                              is outside (0, sample_rate_hz/2), q is not positive, or
 *                              the prewarped bandwidth underflows to zero.
 *
 * @note Designs in double (tan) and rounds coefficients to F31; slow without a
 *       double-precision FPU. Serialize use of ctx.
 */
linears_status_t filters_bpf_q15_init_center(filters_bpf_q15_t *ctx, filters_order_t order, float center_hz, float q, float sample_rate_hz, linears_q15_t initial);

/**
 * @brief Retune a Q15 band-pass filter to new band edges.
 *
 * The order and input/output history are kept.
 *
 * @param[in,out] ctx Non-NULL initialized context; unchanged on failure.
 * @param[in] low_hz New lower -3 dB frequency in Hz, above zero.
 * @param[in] high_hz New upper -3 dB frequency in Hz, above low_hz and below sample_rate_hz/2.
 * @param[in] sample_rate_hz New positive step rate in Hz.
 *
 * @return linears_status_t Operation status.
 *
 * @retval LINEARS_OK Coefficients replaced; history unchanged.
 * @retval LINEARS_ERROR_ARGUMENT ctx is NULL.
 * @retval LINEARS_ERROR_RANGE A frequency is nonfinite.
 * @retval LINEARS_ERROR_DOMAIN sample_rate_hz is not positive, or the
 *                              frequencies do not satisfy 0 < low_hz < high_hz <
 *                              sample_rate_hz/2 with distinct prewarped edges.
 *
 * @note Designs in double (tan) and rounds coefficients to F31; slow without a
 *       double-precision FPU. Serialize use of ctx.
 */
linears_status_t filters_bpf_q15_set_band(filters_bpf_q15_t *ctx, float low_hz, float high_hz, float sample_rate_hz);

/**
 * @brief Retune a Q15 band-pass filter to new center frequency and Q.
 *
 * The order and input/output history are kept.
 *
 * @param[in,out] ctx Non-NULL initialized context; unchanged on failure.
 * @param[in] center_hz New peak frequency in Hz, in (0, sample_rate_hz/2).
 * @param[in] q New positive quality factor w0/B of the prewarped design.
 * @param[in] sample_rate_hz New positive step rate in Hz.
 *
 * @return linears_status_t Operation status.
 *
 * @retval LINEARS_OK Coefficients replaced; history unchanged.
 * @retval LINEARS_ERROR_ARGUMENT ctx is NULL.
 * @retval LINEARS_ERROR_RANGE A frequency or q is nonfinite, or a tiny q
 *                             makes a designed coefficient unrepresentable.
 * @retval LINEARS_ERROR_DOMAIN sample_rate_hz is not positive, center_hz
 *                              is outside (0, sample_rate_hz/2), q is not positive, or
 *                              the prewarped bandwidth underflows to zero.
 *
 * @note Designs in double (tan) and rounds coefficients to F31; slow without a
 *       double-precision FPU. Serialize use of ctx.
 */
linears_status_t filters_bpf_q15_set_center(filters_bpf_q15_t *ctx, float center_hz, float q, float sample_rate_hz);

/**
 * @brief Restart a Q15 band-pass filter at a steady state.
 *
 * The history represents initial held forever with a settled zero output.
 *
 * @param[in,out] ctx Non-NULL initialized context; coefficients and order are preserved.
 * @param[in] initial Raw Q1.15 input level held before the next step; converted exactly to F31.
 *
 * @return linears_status_t Operation status.
 *
 * @retval LINEARS_OK History replaced.
 * @retval LINEARS_ERROR_ARGUMENT ctx is NULL.
 *
 * @note Failure leaves ctx unchanged. Serialize use of ctx.
 */
linears_status_t filters_bpf_q15_reset(filters_bpf_q15_t *ctx, linears_q15_t initial);

/**
 * @brief Filter one sample through a Q15 band-pass filter.
 *
 * Call once per period of the configured sample rate.
 *
 * @param[in,out] ctx Non-NULL initialized context.
 * @param[in] input Raw Q1.15 sample.
 * @param[out] output Non-NULL destination; must not overlap ctx.
 *
 * @return linears_status_t Operation status.
 *
 * @retval LINEARS_OK Output and history committed.
 * @retval LINEARS_ERROR_ARGUMENT ctx or output is NULL.
 * @retval LINEARS_ERROR_ALIAS output overlaps ctx.
 * @retval LINEARS_ERROR_RANGE The exact F62 output sum lies outside [-1,+1], or a
 *                             working value does not fit F31.
 *
 * @note Failure leaves ctx and output unchanged. Serialize use of ctx.
 */
linears_status_t filters_bpf_q15_step(filters_bpf_q15_t *ctx, linears_q15_t input, linears_q15_t *output);

/* Q31 interface. */

/**
 * @brief Caller-owned Q31 band-pass filter with embedded coefficients and history.
 */
typedef struct {
    filters_order_t order; /**< Roll-off order; fixed at init. */
    filters_iir_section_q31_t sections[2]; /**< Cascade; sections[1] is zero and unused for order one. */
} filters_bpf_q31_t;

/**
 * @brief Initialize a Q31 band-pass filter from band edges.
 *
 * The history represents initial held forever with a settled zero output, so a
 * DC offset in the first samples causes no startup transient.
 *
 * @param[out] ctx Non-NULL caller-owned context; unchanged on failure.
 * @param[in] order FILTERS_ORDER_1 (one section) or FILTERS_ORDER_2 (two sections).
 * @param[in] low_hz Lower -3 dB frequency in Hz, above zero.
 * @param[in] high_hz Upper -3 dB frequency in Hz, above low_hz and below sample_rate_hz/2.
 * @param[in] sample_rate_hz Positive step rate in Hz.
 * @param[in] initial Raw Q1.31 input level held before the first step; converted exactly to F31.
 *
 * @return linears_status_t Operation status.
 *
 * @retval LINEARS_OK Coefficients designed and history set.
 * @retval LINEARS_ERROR_ARGUMENT ctx is NULL.
 * @retval LINEARS_ERROR_RANGE A frequency is nonfinite.
 * @retval LINEARS_ERROR_DOMAIN order is invalid, sample_rate_hz is not positive, or the
 *                              frequencies do not satisfy 0 < low_hz < high_hz <
 *                              sample_rate_hz/2 with distinct prewarped edges.
 *
 * @note Designs in double (tan) and rounds coefficients to F31; slow without a
 *       double-precision FPU. Serialize use of ctx.
 */
linears_status_t filters_bpf_q31_init_band(filters_bpf_q31_t *ctx, filters_order_t order, float low_hz, float high_hz, float sample_rate_hz, linears_q31_t initial);

/**
 * @brief Initialize a Q31 band-pass filter from a center frequency and Q.
 *
 * The history represents initial held forever with a settled zero output, so a
 * DC offset in the first samples causes no startup transient.
 *
 * @param[out] ctx Non-NULL caller-owned context; unchanged on failure.
 * @param[in] order FILTERS_ORDER_1 (one section) or FILTERS_ORDER_2 (two sections).
 * @param[in] center_hz Peak frequency in Hz, in (0, sample_rate_hz/2).
 * @param[in] q Positive quality factor w0/B of the prewarped design.
 * @param[in] sample_rate_hz Positive step rate in Hz.
 * @param[in] initial Raw Q1.31 input level held before the first step; converted exactly to F31.
 *
 * @return linears_status_t Operation status.
 *
 * @retval LINEARS_OK Coefficients designed and history set.
 * @retval LINEARS_ERROR_ARGUMENT ctx is NULL.
 * @retval LINEARS_ERROR_RANGE A frequency or q is nonfinite, or a tiny q
 *                             makes a designed coefficient unrepresentable.
 * @retval LINEARS_ERROR_DOMAIN order is invalid, sample_rate_hz is not positive, center_hz
 *                              is outside (0, sample_rate_hz/2), q is not positive, or
 *                              the prewarped bandwidth underflows to zero.
 *
 * @note Designs in double (tan) and rounds coefficients to F31; slow without a
 *       double-precision FPU. Serialize use of ctx.
 */
linears_status_t filters_bpf_q31_init_center(filters_bpf_q31_t *ctx, filters_order_t order, float center_hz, float q, float sample_rate_hz, linears_q31_t initial);

/**
 * @brief Retune a Q31 band-pass filter to new band edges.
 *
 * The order and input/output history are kept.
 *
 * @param[in,out] ctx Non-NULL initialized context; unchanged on failure.
 * @param[in] low_hz New lower -3 dB frequency in Hz, above zero.
 * @param[in] high_hz New upper -3 dB frequency in Hz, above low_hz and below sample_rate_hz/2.
 * @param[in] sample_rate_hz New positive step rate in Hz.
 *
 * @return linears_status_t Operation status.
 *
 * @retval LINEARS_OK Coefficients replaced; history unchanged.
 * @retval LINEARS_ERROR_ARGUMENT ctx is NULL.
 * @retval LINEARS_ERROR_RANGE A frequency is nonfinite.
 * @retval LINEARS_ERROR_DOMAIN sample_rate_hz is not positive, or the
 *                              frequencies do not satisfy 0 < low_hz < high_hz <
 *                              sample_rate_hz/2 with distinct prewarped edges.
 *
 * @note Designs in double (tan) and rounds coefficients to F31; slow without a
 *       double-precision FPU. Serialize use of ctx.
 */
linears_status_t filters_bpf_q31_set_band(filters_bpf_q31_t *ctx, float low_hz, float high_hz, float sample_rate_hz);

/**
 * @brief Retune a Q31 band-pass filter to new center frequency and Q.
 *
 * The order and input/output history are kept.
 *
 * @param[in,out] ctx Non-NULL initialized context; unchanged on failure.
 * @param[in] center_hz New peak frequency in Hz, in (0, sample_rate_hz/2).
 * @param[in] q New positive quality factor w0/B of the prewarped design.
 * @param[in] sample_rate_hz New positive step rate in Hz.
 *
 * @return linears_status_t Operation status.
 *
 * @retval LINEARS_OK Coefficients replaced; history unchanged.
 * @retval LINEARS_ERROR_ARGUMENT ctx is NULL.
 * @retval LINEARS_ERROR_RANGE A frequency or q is nonfinite, or a tiny q
 *                             makes a designed coefficient unrepresentable.
 * @retval LINEARS_ERROR_DOMAIN sample_rate_hz is not positive, center_hz
 *                              is outside (0, sample_rate_hz/2), q is not positive, or
 *                              the prewarped bandwidth underflows to zero.
 *
 * @note Designs in double (tan) and rounds coefficients to F31; slow without a
 *       double-precision FPU. Serialize use of ctx.
 */
linears_status_t filters_bpf_q31_set_center(filters_bpf_q31_t *ctx, float center_hz, float q, float sample_rate_hz);

/**
 * @brief Restart a Q31 band-pass filter at a steady state.
 *
 * The history represents initial held forever with a settled zero output.
 *
 * @param[in,out] ctx Non-NULL initialized context; coefficients and order are preserved.
 * @param[in] initial Raw Q1.31 input level held before the next step; converted exactly to F31.
 *
 * @return linears_status_t Operation status.
 *
 * @retval LINEARS_OK History replaced.
 * @retval LINEARS_ERROR_ARGUMENT ctx is NULL.
 *
 * @note Failure leaves ctx unchanged. Serialize use of ctx.
 */
linears_status_t filters_bpf_q31_reset(filters_bpf_q31_t *ctx, linears_q31_t initial);

/**
 * @brief Filter one sample through a Q31 band-pass filter.
 *
 * Call once per period of the configured sample rate.
 *
 * @param[in,out] ctx Non-NULL initialized context.
 * @param[in] input Raw Q1.31 sample.
 * @param[out] output Non-NULL destination; must not overlap ctx.
 *
 * @return linears_status_t Operation status.
 *
 * @retval LINEARS_OK Output and history committed.
 * @retval LINEARS_ERROR_ARGUMENT ctx or output is NULL.
 * @retval LINEARS_ERROR_ALIAS output overlaps ctx.
 * @retval LINEARS_ERROR_RANGE The exact F62 output sum lies outside [-1,+1], or a
 *                             working value does not fit F31.
 *
 * @note Failure leaves ctx and output unchanged. Serialize use of ctx.
 */
linears_status_t filters_bpf_q31_step(filters_bpf_q31_t *ctx, linears_q31_t input, linears_q31_t *output);

#ifdef __cplusplus
}
#endif
#endif
