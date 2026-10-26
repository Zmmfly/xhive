/**
 * @file filters_lpf.h
 * @brief Low-pass filters designed from a cutoff frequency and sample rate.
 *
 * Order one is H(s)=wc/(s+wc); order two is the Butterworth
 * H(s)=wc^2/(s^2+sqrt(2)*wc*s+wc^2). With the bilinear transform and the
 * prewarped wc=tan(pi*cutoff_hz/sample_rate_hz), the designed response is
 * exactly -3 dB at cutoff_hz and zero at sample_rate_hz/2. The numerator
 * taps are derived from the stored feedback taps, so the DC gain is exactly
 * one in the stored coefficients; a held Q input reproduces itself exactly.
 *
 * Each context embeds one direct-form-I section; no external arrays are
 * needed. Call step once per sample at the configured rate. set replaces the
 * coefficients and keeps the input/output history, so retuning does not
 * restart the filter. f32 designs in float and f64 in double. Q15/Q31 accept
 * float frequencies, design in double, round coefficients to F31 once, and
 * step with integer arithmetic only; Q history is F31 and the Q output is
 * rounded once from the exact F62 sum.
 *
 * Contexts are caller-owned; do not edit fields. Serialize calls on one
 * context, including interrupts; distinct contexts are reentrant. No
 * allocation, locks, I/O or MMIO occurs. Every failure leaves the context
 * and output unchanged.
 */
#ifndef FILTERS_LPF_H
#define FILTERS_LPF_H

#include "filters_iir.h"

#ifdef __cplusplus
extern "C" {
#endif

/* F32 interface. */

/**
 * @brief Caller-owned F32 low-pass filter with embedded coefficients and history.
 */
typedef struct {
    filters_order_t order; /**< Roll-off order; fixed at init. */
    filters_iir_section_f32_t section; /**< Designed coefficients and DF-I history. */
} filters_lpf_f32_t;

/**
 * @brief Initialize a F32 low-pass filter at a steady state.
 *
 * The history represents initial held forever, so a constant input equal to
 * initial is reproduced without a startup transient.
 *
 * @param[out] ctx Non-NULL caller-owned context; unchanged on failure.
 * @param[in] order FILTERS_ORDER_1 or FILTERS_ORDER_2; fixed for the context lifetime.
 * @param[in] cutoff_hz -3 dB frequency in Hz, in (0, sample_rate_hz/2).
 * @param[in] sample_rate_hz Positive step rate in Hz.
 * @param[in] initial Input level held before the first step, in signal units.
 *
 * @return linears_status_t Operation status.
 *
 * @retval LINEARS_OK Coefficients designed and history set.
 * @retval LINEARS_ERROR_ARGUMENT ctx is NULL.
 * @retval LINEARS_ERROR_RANGE A frequency or initial is nonfinite.
 * @retval LINEARS_ERROR_DOMAIN order is invalid, sample_rate_hz is not positive,
 *                              or cutoff_hz is outside (0, sample_rate_hz/2).
 *
 * @note Designs with tanf. Serialize use of ctx.
 */
linears_status_t filters_lpf_f32_init(filters_lpf_f32_t *ctx, filters_order_t order, float cutoff_hz, float sample_rate_hz, float initial);

/**
 * @brief Retune a F32 low-pass filter, keeping its order and history.
 *
 * @param[in,out] ctx Non-NULL initialized context; unchanged on failure.
 * @param[in] cutoff_hz New -3 dB frequency in Hz, in (0, sample_rate_hz/2).
 * @param[in] sample_rate_hz New positive step rate in Hz.
 *
 * @return linears_status_t Operation status.
 *
 * @retval LINEARS_OK Coefficients replaced; history unchanged.
 * @retval LINEARS_ERROR_ARGUMENT ctx is NULL.
 * @retval LINEARS_ERROR_RANGE A frequency is nonfinite.
 * @retval LINEARS_ERROR_DOMAIN sample_rate_hz is not positive or cutoff_hz is
 *                              outside (0, sample_rate_hz/2).
 *
 * @note Designs with tanf. Serialize use of ctx.
 */
linears_status_t filters_lpf_f32_set(filters_lpf_f32_t *ctx, float cutoff_hz, float sample_rate_hz);

/**
 * @brief Restart a F32 low-pass filter at a steady state.
 *
 * The history represents initial held forever; the next output starts from
 * initial.
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
linears_status_t filters_lpf_f32_reset(filters_lpf_f32_t *ctx, float initial);

/**
 * @brief Filter one sample through a F32 low-pass filter.
 *
 * Call once per period of the configured sample rate. Order-two step
 * responses overshoot by about 4%.
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
linears_status_t filters_lpf_f32_step(filters_lpf_f32_t *ctx, float input, float *output);

/* F64 interface. */

/**
 * @brief Caller-owned F64 low-pass filter with embedded coefficients and history.
 */
typedef struct {
    filters_order_t order; /**< Roll-off order; fixed at init. */
    filters_iir_section_f64_t section; /**< Designed coefficients and DF-I history. */
} filters_lpf_f64_t;

/**
 * @brief Initialize a F64 low-pass filter at a steady state.
 *
 * The history represents initial held forever, so a constant input equal to
 * initial is reproduced without a startup transient.
 *
 * @param[out] ctx Non-NULL caller-owned context; unchanged on failure.
 * @param[in] order FILTERS_ORDER_1 or FILTERS_ORDER_2; fixed for the context lifetime.
 * @param[in] cutoff_hz -3 dB frequency in Hz, in (0, sample_rate_hz/2).
 * @param[in] sample_rate_hz Positive step rate in Hz.
 * @param[in] initial Input level held before the first step, in signal units.
 *
 * @return linears_status_t Operation status.
 *
 * @retval LINEARS_OK Coefficients designed and history set.
 * @retval LINEARS_ERROR_ARGUMENT ctx is NULL.
 * @retval LINEARS_ERROR_RANGE A frequency or initial is nonfinite.
 * @retval LINEARS_ERROR_DOMAIN order is invalid, sample_rate_hz is not positive,
 *                              or cutoff_hz is outside (0, sample_rate_hz/2).
 *
 * @note Designs with tan. Serialize use of ctx.
 */
linears_status_t filters_lpf_f64_init(filters_lpf_f64_t *ctx, filters_order_t order, double cutoff_hz, double sample_rate_hz, double initial);

/**
 * @brief Retune a F64 low-pass filter, keeping its order and history.
 *
 * @param[in,out] ctx Non-NULL initialized context; unchanged on failure.
 * @param[in] cutoff_hz New -3 dB frequency in Hz, in (0, sample_rate_hz/2).
 * @param[in] sample_rate_hz New positive step rate in Hz.
 *
 * @return linears_status_t Operation status.
 *
 * @retval LINEARS_OK Coefficients replaced; history unchanged.
 * @retval LINEARS_ERROR_ARGUMENT ctx is NULL.
 * @retval LINEARS_ERROR_RANGE A frequency is nonfinite.
 * @retval LINEARS_ERROR_DOMAIN sample_rate_hz is not positive or cutoff_hz is
 *                              outside (0, sample_rate_hz/2).
 *
 * @note Designs with tan. Serialize use of ctx.
 */
linears_status_t filters_lpf_f64_set(filters_lpf_f64_t *ctx, double cutoff_hz, double sample_rate_hz);

/**
 * @brief Restart a F64 low-pass filter at a steady state.
 *
 * The history represents initial held forever; the next output starts from
 * initial.
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
linears_status_t filters_lpf_f64_reset(filters_lpf_f64_t *ctx, double initial);

/**
 * @brief Filter one sample through a F64 low-pass filter.
 *
 * Call once per period of the configured sample rate. Order-two step
 * responses overshoot by about 4%.
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
linears_status_t filters_lpf_f64_step(filters_lpf_f64_t *ctx, double input, double *output);

/* Q15 interface. */

/**
 * @brief Caller-owned Q15 low-pass filter with embedded coefficients and history.
 */
typedef struct {
    filters_order_t order; /**< Roll-off order; fixed at init. */
    filters_iir_section_q15_t section; /**< Designed coefficients and DF-I history. */
} filters_lpf_q15_t;

/**
 * @brief Initialize a Q15 low-pass filter at a steady state.
 *
 * The history represents initial held forever, so a constant input equal to
 * initial is reproduced without a startup transient.
 *
 * @param[out] ctx Non-NULL caller-owned context; unchanged on failure.
 * @param[in] order FILTERS_ORDER_1 or FILTERS_ORDER_2; fixed for the context lifetime.
 * @param[in] cutoff_hz -3 dB frequency in Hz, in (0, sample_rate_hz/2).
 * @param[in] sample_rate_hz Positive step rate in Hz.
 * @param[in] initial Raw Q1.15 input level held before the first step; converted exactly to F31.
 *
 * @return linears_status_t Operation status.
 *
 * @retval LINEARS_OK Coefficients designed and history set.
 * @retval LINEARS_ERROR_ARGUMENT ctx is NULL.
 * @retval LINEARS_ERROR_RANGE A frequency is nonfinite.
 * @retval LINEARS_ERROR_DOMAIN order is invalid, sample_rate_hz is not positive,
 *                              or cutoff_hz is outside (0, sample_rate_hz/2).
 *
 * @note Designs in double (tan) and rounds coefficients to F31; slow without a
 *       double-precision FPU. Serialize use of ctx.
 */
linears_status_t filters_lpf_q15_init(filters_lpf_q15_t *ctx, filters_order_t order, float cutoff_hz, float sample_rate_hz, linears_q15_t initial);

/**
 * @brief Retune a Q15 low-pass filter, keeping its order and history.
 *
 * @param[in,out] ctx Non-NULL initialized context; unchanged on failure.
 * @param[in] cutoff_hz New -3 dB frequency in Hz, in (0, sample_rate_hz/2).
 * @param[in] sample_rate_hz New positive step rate in Hz.
 *
 * @return linears_status_t Operation status.
 *
 * @retval LINEARS_OK Coefficients replaced; history unchanged.
 * @retval LINEARS_ERROR_ARGUMENT ctx is NULL.
 * @retval LINEARS_ERROR_RANGE A frequency is nonfinite.
 * @retval LINEARS_ERROR_DOMAIN sample_rate_hz is not positive or cutoff_hz is
 *                              outside (0, sample_rate_hz/2).
 *
 * @note Designs in double (tan) and rounds coefficients to F31; slow without a
 *       double-precision FPU. Serialize use of ctx.
 */
linears_status_t filters_lpf_q15_set(filters_lpf_q15_t *ctx, float cutoff_hz, float sample_rate_hz);

/**
 * @brief Restart a Q15 low-pass filter at a steady state.
 *
 * The history represents initial held forever; the next output starts from
 * initial.
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
linears_status_t filters_lpf_q15_reset(filters_lpf_q15_t *ctx, linears_q15_t initial);

/**
 * @brief Filter one sample through a Q15 low-pass filter.
 *
 * Call once per period of the configured sample rate. Order-two step
 * responses overshoot by about 4%; leave Q headroom.
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
linears_status_t filters_lpf_q15_step(filters_lpf_q15_t *ctx, linears_q15_t input, linears_q15_t *output);

/* Q31 interface. */

/**
 * @brief Caller-owned Q31 low-pass filter with embedded coefficients and history.
 */
typedef struct {
    filters_order_t order; /**< Roll-off order; fixed at init. */
    filters_iir_section_q31_t section; /**< Designed coefficients and DF-I history. */
} filters_lpf_q31_t;

/**
 * @brief Initialize a Q31 low-pass filter at a steady state.
 *
 * The history represents initial held forever, so a constant input equal to
 * initial is reproduced without a startup transient.
 *
 * @param[out] ctx Non-NULL caller-owned context; unchanged on failure.
 * @param[in] order FILTERS_ORDER_1 or FILTERS_ORDER_2; fixed for the context lifetime.
 * @param[in] cutoff_hz -3 dB frequency in Hz, in (0, sample_rate_hz/2).
 * @param[in] sample_rate_hz Positive step rate in Hz.
 * @param[in] initial Raw Q1.31 input level held before the first step; converted exactly to F31.
 *
 * @return linears_status_t Operation status.
 *
 * @retval LINEARS_OK Coefficients designed and history set.
 * @retval LINEARS_ERROR_ARGUMENT ctx is NULL.
 * @retval LINEARS_ERROR_RANGE A frequency is nonfinite.
 * @retval LINEARS_ERROR_DOMAIN order is invalid, sample_rate_hz is not positive,
 *                              or cutoff_hz is outside (0, sample_rate_hz/2).
 *
 * @note Designs in double (tan) and rounds coefficients to F31; slow without a
 *       double-precision FPU. Serialize use of ctx.
 */
linears_status_t filters_lpf_q31_init(filters_lpf_q31_t *ctx, filters_order_t order, float cutoff_hz, float sample_rate_hz, linears_q31_t initial);

/**
 * @brief Retune a Q31 low-pass filter, keeping its order and history.
 *
 * @param[in,out] ctx Non-NULL initialized context; unchanged on failure.
 * @param[in] cutoff_hz New -3 dB frequency in Hz, in (0, sample_rate_hz/2).
 * @param[in] sample_rate_hz New positive step rate in Hz.
 *
 * @return linears_status_t Operation status.
 *
 * @retval LINEARS_OK Coefficients replaced; history unchanged.
 * @retval LINEARS_ERROR_ARGUMENT ctx is NULL.
 * @retval LINEARS_ERROR_RANGE A frequency is nonfinite.
 * @retval LINEARS_ERROR_DOMAIN sample_rate_hz is not positive or cutoff_hz is
 *                              outside (0, sample_rate_hz/2).
 *
 * @note Designs in double (tan) and rounds coefficients to F31; slow without a
 *       double-precision FPU. Serialize use of ctx.
 */
linears_status_t filters_lpf_q31_set(filters_lpf_q31_t *ctx, float cutoff_hz, float sample_rate_hz);

/**
 * @brief Restart a Q31 low-pass filter at a steady state.
 *
 * The history represents initial held forever; the next output starts from
 * initial.
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
linears_status_t filters_lpf_q31_reset(filters_lpf_q31_t *ctx, linears_q31_t initial);

/**
 * @brief Filter one sample through a Q31 low-pass filter.
 *
 * Call once per period of the configured sample rate. Order-two step
 * responses overshoot by about 4%; leave Q headroom.
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
linears_status_t filters_lpf_q31_step(filters_lpf_q31_t *ctx, linears_q31_t input, linears_q31_t *output);

#ifdef __cplusplus
}
#endif
#endif
