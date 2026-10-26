/**
 * @file fieldctrl_smo.h
 * @brief Boundary-layer sliding-mode back-EMF observer for isotropic PMSMs.
 *
 * The observer predicts alpha/beta current using v-R*i_hat-z and estimates
 * back-EMF by first-order low-pass filtering the switching injection z.
 * Angles are electrical turns in [0,1), positive from alpha toward beta.
 * Floating frequencies are electrical turns/s, not mechanical speed or rad/s.
 * Q signals are raw int16_t/F15 or int32_t/F31; working state is int64_t/F31.
 * Fixed arithmetic never uses an exponent, scale metadata or floating conversion.
 * Q units are caller-normalized: Ipu=I/Ibase, Vpu=V/Vbase, tpu=t/Tbase and
 * fpu=f*Tbase. The caller's base values are not stored in any context.
 * Mathematical +1 signal output maps to MAX, -1 to MIN; values actually
 * outside [-1,+1] return RANGE. Q MAX alpha is less than one, not passthrough.
 *
 * Contexts belong to callers and must be initialized and not edited directly.
 * Distinct contexts are reentrant; serialize all access to a shared context,
 * including interrupt calls. No dynamic allocation, I/O or MMIO is performed.
 * On error the context and output retain their prior values. A successful
 * sample with observable/locked equal to zero is not an arithmetic error.
 *
 * Back-EMF is absent at standstill. This observer does not provide startup,
 * zero-speed position, automatic reversal, or proof of physical rotor lock.
 * The caller must implement open-loop startup and safe handover/recovery.
 */
#ifndef FIELDCTRL_SMO_H
#define FIELDCTRL_SMO_H

#include "fieldctrl_types.h"
#include "filters_ema.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Known rotation direction resolving the back-EMF half-turn ambiguity.
 * @note Reinitialize after changing direction; cross zero using a separate
 *       startup/reversal controller, not an unlocked SMO angle.
 */
typedef enum {
    FIELDCTRL_SMO_FORWARD = 1, /**< Positive electrical frequency, alpha to beta. */
    FIELDCTRL_SMO_REVERSE = -1 /**< Negative electrical frequency, beta to alpha. */
} fieldctrl_smo_direction_t;

/**
 * @brief single-precision SMO tuning for a fixed sampling period and known direction.
 *
 * Require dt*(resistance+sliding_gain/boundary)/inductance < 2, the explicit
 * Euler local boundary-layer stability limit. This is not a guarantee under
 * model mismatch. max_speed*dt must be less than 1/4 turn to reject ambiguous
 * angle steps. Gains/thresholds require application-specific noise margins.
 */
typedef struct {
    float resistance; /**< Nonnegative stator phase resistance, ohm. */
    float inductance; /**< Positive isotropic phase inductance Ld=Lq, H. */
    float dt; /**< Positive fixed sampling interval, s. */
    float sliding_gain; /**< Positive injection magnitude K, V; exceed peak back-EMF. */
    float boundary; /**< Positive current-error saturation boundary, A. */
    float emf_alpha; /**< EMF low-pass input weight in (0,1], dimensionless. */
    float speed_alpha; /**< Speed low-pass input weight in (0,1], dimensionless. */
    float min_emf; /**< Positive filtered EMF magnitude threshold, V. */
    float min_speed; /**< Positive minimum observable electrical speed magnitude, turn/s. */
    float max_speed; /**< Maximum electrical speed magnitude, turn/s; greater than min_speed. */
    uint32_t lock_samples; /**< Positive consecutive observable-sample count for lock. */
    fieldctrl_smo_direction_t direction; /**< Known fixed rotation direction. */
} fieldctrl_f32_smo_config_t;

/**
 * @brief single-precision caller-owned SMO state with independent scalar filters.
 * @note Floating state uses single precision throughout.
 */
typedef struct {
    fieldctrl_f32_smo_config_t config; /**< Immutable copied motor/observer tuning. */
    struct {
        float alpha; /**< Predicted next-sample alpha current, A. */
        float beta; /**< Predicted next-sample beta current, A. */
    } current_hat; /**< Current predictor at working precision. */
    filters_ema_f32_t emf_alpha_filter; /**< Alpha switching-injection history, V. */
    filters_ema_f32_t emf_beta_filter; /**< Beta switching-injection history, V. */
    filters_ema_f32_t speed_filter; /**< Electrical-frequency history, turn/s. */
    float previous_angle; /**< Previous uncorrected EMF-derived angle, turns. */
    float angle; /**< Last compensated angle, turns in [0,1). */
    uint32_t lock_count; /**< Saturating count of consecutive observable samples. */
    int has_angle; /**< A previous above-threshold EMF angle is available. */
} fieldctrl_f32_smo_t;

/**
 * @brief single-precision observer sample and explicit observability/lock indicators.
 */
typedef struct {
    fieldctrl_f32_ab_t current_hat; /**< Predicted current at the next sample, A. */
    fieldctrl_f32_ab_t emf; /**< Low-pass switching injection, V; not de-attenuated. */
    float electrical_angle; /**< LP-phase-compensated flux angle, turns in [0,1). */
    float electrical_speed; /**< Filtered signed electrical speed, turns/s. */
    int observable; /**< EMF, raw/filtered speed and sliding-region tests all passed. */
    int locked; /**< observable held for config.lock_samples consecutive samples. */
} fieldctrl_f32_smo_output_t;

/**
 * @brief Initialize a single-precision SMO with zero EMF/speed and no lock.
 *
 * @param[out] ctx Non-NULL caller-owned context; no prior initialization needed.
 * @param[in] config Non-NULL tuning copied by value; limits as documented above.
 * @param[in] initial_current Initial simultaneous alpha/beta current estimate, A.
 *
 * @return linears_status_t Initialization status.
 * @retval LINEARS_OK Predictor and filters initialized, lock cleared.
 * @retval LINEARS_ERROR_ARGUMENT ctx or config is NULL.
 * @retval LINEARS_ERROR_DOMAIN A tuning, direction, stability or alias bound is invalid.
 * @retval LINEARS_ERROR_RANGE A scalar/intermediate is nonfinite or unrepresentable.
 * @note Error preserves ctx. Serialize concurrent access to ctx.
 */
linears_status_t fieldctrl_f32_smo_init(fieldctrl_f32_smo_t *ctx,
    const fieldctrl_f32_smo_config_t *config, fieldctrl_f32_ab_t initial_current);

/**
 * @brief Predict current and estimate rotor motion using a single-precision SMO.
 *
 * At sample n, z=K*sat((i_hat[n]-i[n])/boundary), componentwise. The predictor
 * advances i_hat[n+1]=i_hat[n]+dt/L*(v[n]-R*i_hat[n]-z). Injection z passes
 * through the shared filters low-pass. Speed uses wrapped angle differences.
 * LP phase is compensated using the exact discrete first-order frequency
 * response at the estimated speed; observer/ADC/PWM delay and acceleration
 * transients are NOT compensated. Negative direction removes the EMF pi shift.
 *
 * Below min_emf, angle is held (zero before first acquisition), speed/history
 * and lock are reset. Otherwise an angle is returned even when unlocked and
 * must not drive a closed-loop handover. observable additionally requires a
 * previous angle, expected raw-speed sign, both raw and filtered speed within
 * [min_speed,max_speed], and both current errors within boundary. A failed
 * observability test clears lock immediately, without being an API error.
 *
 * @param[in,out] ctx Non-NULL successfully initialized caller-owned context.
 * @param[in] current Simultaneous measured alpha/beta current at sample n, A.
 * @param[in] voltage Applied average alpha/beta voltage for [n,n+1), V; use
 *                    reconstructed applied voltage, not an unlimited PI request.
 * @param[out] output Non-NULL result, disjoint from ctx; flags govern validity.
 *
 * @return linears_status_t Numeric execution status, distinct from rotor lock.
 * @retval LINEARS_OK Sample committed; inspect observable and locked separately.
 * @retval LINEARS_ERROR_ARGUMENT ctx or output is NULL.
 * @retval LINEARS_ERROR_ALIAS output overlaps ctx.
 * @retval LINEARS_ERROR_RANGE An input/intermediate/output is nonfinite or unrepresentable.
 * @pre Use exactly config.dt between calls; motor model is isotropic and direction known.
 * @note Nonblocking; errors preserve ctx/output. Serialize use of ctx.
 * @warning No standstill observability, startup, automatic reversal or hardware protection.
 */
linears_status_t fieldctrl_f32_smo_step(fieldctrl_f32_smo_t *ctx,
    fieldctrl_f32_ab_t current, fieldctrl_f32_ab_t voltage,
    fieldctrl_f32_smo_output_t *output);

/**
 * @brief double-precision SMO tuning for a fixed sampling period and known direction.
 *
 * Require dt*(resistance+sliding_gain/boundary)/inductance < 2, the explicit
 * Euler local boundary-layer stability limit. This is not a guarantee under
 * model mismatch. max_speed*dt must be less than 1/4 turn to reject ambiguous
 * angle steps. Gains/thresholds require application-specific noise margins.
 */
typedef struct {
    double resistance; /**< Nonnegative stator phase resistance, ohm. */
    double inductance; /**< Positive isotropic phase inductance Ld=Lq, H. */
    double dt; /**< Positive fixed sampling interval, s. */
    double sliding_gain; /**< Positive injection magnitude K, V; exceed peak back-EMF. */
    double boundary; /**< Positive current-error saturation boundary, A. */
    double emf_alpha; /**< EMF low-pass input weight in (0,1], dimensionless. */
    double speed_alpha; /**< Speed low-pass input weight in (0,1], dimensionless. */
    double min_emf; /**< Positive filtered EMF magnitude threshold, V. */
    double min_speed; /**< Positive minimum observable electrical speed magnitude, turn/s. */
    double max_speed; /**< Maximum electrical speed magnitude, turn/s; greater than min_speed. */
    uint32_t lock_samples; /**< Positive consecutive observable-sample count for lock. */
    fieldctrl_smo_direction_t direction; /**< Known fixed rotation direction. */
} fieldctrl_f64_smo_config_t;

/**
 * @brief double-precision caller-owned SMO state with independent scalar filters.
 * @note Floating state uses double precision throughout.
 */
typedef struct {
    fieldctrl_f64_smo_config_t config; /**< Immutable copied motor/observer tuning. */
    struct {
        double alpha; /**< Predicted next-sample alpha current, A. */
        double beta; /**< Predicted next-sample beta current, A. */
    } current_hat; /**< Current predictor at working precision. */
    filters_ema_f64_t emf_alpha_filter; /**< Alpha switching-injection history, V. */
    filters_ema_f64_t emf_beta_filter; /**< Beta switching-injection history, V. */
    filters_ema_f64_t speed_filter; /**< Electrical-frequency history, turn/s. */
    double previous_angle; /**< Previous uncorrected EMF-derived angle, turns. */
    double angle; /**< Last compensated angle, turns in [0,1). */
    uint32_t lock_count; /**< Saturating count of consecutive observable samples. */
    int has_angle; /**< A previous above-threshold EMF angle is available. */
} fieldctrl_f64_smo_t;

/**
 * @brief double-precision observer sample and explicit observability/lock indicators.
 */
typedef struct {
    fieldctrl_f64_ab_t current_hat; /**< Predicted current at the next sample, A. */
    fieldctrl_f64_ab_t emf; /**< Low-pass switching injection, V; not de-attenuated. */
    double electrical_angle; /**< LP-phase-compensated flux angle, turns in [0,1). */
    double electrical_speed; /**< Filtered signed electrical speed, turns/s. */
    int observable; /**< EMF, raw/filtered speed and sliding-region tests all passed. */
    int locked; /**< observable held for config.lock_samples consecutive samples. */
} fieldctrl_f64_smo_output_t;

/**
 * @brief Initialize a double-precision SMO with zero EMF/speed and no lock.
 *
 * @param[out] ctx Non-NULL caller-owned context; no prior initialization needed.
 * @param[in] config Non-NULL tuning copied by value; limits as documented above.
 * @param[in] initial_current Initial simultaneous alpha/beta current estimate, A.
 *
 * @return linears_status_t Initialization status.
 * @retval LINEARS_OK Predictor and filters initialized, lock cleared.
 * @retval LINEARS_ERROR_ARGUMENT ctx or config is NULL.
 * @retval LINEARS_ERROR_DOMAIN A tuning, direction, stability or alias bound is invalid.
 * @retval LINEARS_ERROR_RANGE A scalar/intermediate is nonfinite or unrepresentable.
 * @note Error preserves ctx. Serialize concurrent access to ctx.
 */
linears_status_t fieldctrl_f64_smo_init(fieldctrl_f64_smo_t *ctx,
    const fieldctrl_f64_smo_config_t *config, fieldctrl_f64_ab_t initial_current);

/**
 * @brief Predict current and estimate rotor motion using a double-precision SMO.
 *
 * At sample n, z=K*sat((i_hat[n]-i[n])/boundary), componentwise. The predictor
 * advances i_hat[n+1]=i_hat[n]+dt/L*(v[n]-R*i_hat[n]-z). Injection z passes
 * through the shared filters low-pass. Speed uses wrapped angle differences.
 * LP phase is compensated using the exact discrete first-order frequency
 * response at the estimated speed; observer/ADC/PWM delay and acceleration
 * transients are NOT compensated. Negative direction removes the EMF pi shift.
 *
 * Below min_emf, angle is held (zero before first acquisition), speed/history
 * and lock are reset. Otherwise an angle is returned even when unlocked and
 * must not drive a closed-loop handover. observable additionally requires a
 * previous angle, expected raw-speed sign, both raw and filtered speed within
 * [min_speed,max_speed], and both current errors within boundary. A failed
 * observability test clears lock immediately, without being an API error.
 *
 * @param[in,out] ctx Non-NULL successfully initialized caller-owned context.
 * @param[in] current Simultaneous measured alpha/beta current at sample n, A.
 * @param[in] voltage Applied average alpha/beta voltage for [n,n+1), V; use
 *                    reconstructed applied voltage, not an unlimited PI request.
 * @param[out] output Non-NULL result, disjoint from ctx; flags govern validity.
 *
 * @return linears_status_t Numeric execution status, distinct from rotor lock.
 * @retval LINEARS_OK Sample committed; inspect observable and locked separately.
 * @retval LINEARS_ERROR_ARGUMENT ctx or output is NULL.
 * @retval LINEARS_ERROR_ALIAS output overlaps ctx.
 * @retval LINEARS_ERROR_RANGE An input/intermediate/output is nonfinite or unrepresentable.
 * @pre Use exactly config.dt between calls; motor model is isotropic and direction known.
 * @note Nonblocking; errors preserve ctx/output. Serialize use of ctx.
 * @warning No standstill observability, startup, automatic reversal or hardware protection.
 */
linears_status_t fieldctrl_f64_smo_step(fieldctrl_f64_smo_t *ctx,
    fieldctrl_f64_ab_t current, fieldctrl_f64_ab_t voltage,
    fieldctrl_f64_smo_output_t *output);

/**
 * @brief Q1.15 per-unit SMO tuning for fixed sampling and known direction.
 *
 * Require dt*(resistance+sliding_gain/boundary)/inductance < 2 in working
 * arithmetic and max_speed*dt < 1/4 turn. This local Euler stability check
 * does not guarantee convergence under model mismatch, noise or extra delays.
 * Positive Q1 parameters are at most 1-LSB; choose external bases accordingly.
 */
typedef struct {
    linears_q15_t resistance; /**< Nonnegative Rpu=R*Ibase/Vbase. */
    linears_q15_t inductance; /**< Positive Lpu=L*Ibase/(Vbase*Tbase), Ld=Lq. */
    linears_q15_t dt; /**< Positive tpu=sample_period/Tbase. */
    linears_q15_t sliding_gain; /**< Positive Kpu=K/Vbase; cover peak back-EMF. */
    linears_q15_t boundary; /**< Positive current-error boundary/Ibase. */
    linears_q15_t emf_alpha; /**< Q1.15 dimensionless input weight in (0,1); MAX<1. */
    linears_q15_t speed_alpha; /**< Q1.15 dimensionless input weight in (0,1); MAX<1. */
    linears_q15_t min_emf; /**< Positive filtered-EMF threshold/Vbase. */
    linears_q15_t min_speed; /**< Positive minimum electrical frequency f*Tbase. */
    linears_q15_t max_speed; /**< Maximum electrical f*Tbase, greater than min_speed. */
    uint32_t lock_samples; /**< Positive consecutive observable samples needed for lock. */
    fieldctrl_smo_direction_t direction; /**< Known fixed rotation direction. */
} fieldctrl_q15_smo_config_t;

/**
 * @brief Caller-owned Q1.15 observer with constant-binary-point F31 histories.
 *
 * The Q31 filter contexts store raw Q1.31 alpha and int64_t/F31 value. Updates
 * call the shared filters working-state recurrence, not a floating wrapper.
 */
typedef struct {
    fieldctrl_q15_smo_config_t config; /**< Immutable copied pu tuning; no base metadata. */
    struct {
        linears_fixed_t alpha; /**< int64_t/F31 next-sample current prediction, Ipu. */
        linears_fixed_t beta; /**< int64_t/F31 next-sample current prediction, Ipu. */
    } current_hat; /**< Fixed F31 predictor; successful exported components fit Q1. */
    filters_ema_q31_t emf_alpha_filter; /**< Alpha injection history, F31 Vpu. */
    filters_ema_q31_t emf_beta_filter; /**< Beta injection history, F31 Vpu. */
    filters_ema_q31_t speed_filter; /**< Frequency history, F31 f*Tbase; may exceed Q1. */
    linears_fixed_t previous_angle; /**< Uncorrected previous angle, F31 electrical turns. */
    linears_fixed_t angle; /**< Compensated angle, F31 electrical turns in [0,1). */
    uint32_t lock_count; /**< Saturating consecutive observable-sample counter. */
    int has_angle; /**< A previous above-threshold EMF angle is available. */
} fieldctrl_q15_smo_t;

/**
 * @brief Q1.15 observer result with a wide unlocked-frequency diagnostic.
 */
typedef struct {
    fieldctrl_q15_ab_t current_hat; /**< Q1.15 current prediction at the next sample, Ipu. */
    fieldctrl_q15_ab_t emf; /**< Q1.15 filtered injection, Vpu; not de-attenuated. */
    linears_q15_t electrical_angle; /**< Q1.15 electrical turns [0,1), cyclic rounding. */
    linears_fixed_t electrical_speed; /**< int64_t/F31 signed f*Tbase, possibly outside Q1 while unlocked. */
    int observable; /**< EMF, direction, speed and sliding-region criteria all passed. */
    int locked; /**< observable held for config.lock_samples consecutive samples. */
} fieldctrl_q15_smo_output_t;

/**
 * @brief Initialize a Q1.15 SMO with zero EMF/frequency and no lock.
 *
 * @param[out] ctx Non-NULL caller-owned context; no prior initialization required.
 * @param[in] config Non-NULL per-unit tuning copied by value; bounds documented above.
 * @param[in] initial_current Q1.15 alpha/beta initial current estimate, Ipu.
 *
 * @return linears_status_t Initialization status.
 * @retval LINEARS_OK Predictor and filters initialized, lock cleared.
 * @retval LINEARS_ERROR_ARGUMENT ctx or config is NULL.
 * @retval LINEARS_ERROR_DOMAIN Tuning, direction, stability or alias bound is invalid.
 * @retval LINEARS_ERROR_RANGE An intermediate exceeds the int64_t/F31 working range.
 *
 * @note Failure preserves ctx. Serialize shared calls, including interrupt calls.
 */
linears_status_t fieldctrl_q15_smo_init(fieldctrl_q15_smo_t *ctx,
    const fieldctrl_q15_smo_config_t *config, fieldctrl_q15_ab_t initial_current);

/**
 * @brief Predict current and estimate motion with the integer Q1.15 SMO.
 *
 * At sample n, z=K*sat((i_hat[n]-i[n])/boundary). Prediction advances as
 * i_hat[n+1]=i_hat[n]+dt/L*(v[n]-R*i_hat[n]-z). Shared low-pass filters extract
 * EMF and filter wrapped angle differences. Their exact discrete steady-state
 * phase response is compensated at the signed estimated frequency. Observer,
 * ADC/PWM delay and acceleration transients are not compensated.
 *
 * Below min_emf, hold the last angle (zero before acquisition), clear speed
 * history and lock. Otherwise observable requires a previous angle, expected
 * raw/filtered speed signs, both speed magnitudes in [min_speed,max_speed],
 * and both current errors within boundary. Any failed criterion clears lock.
 * A wide unlocked frequency is diagnostic only; check flags before exporting
 * it to a Q1 controller input. Q15 cyclic rounding that reaches one wraps to zero.
 *
 * @param[in,out] ctx Non-NULL successfully initialized caller-owned context.
 * @param[in] current Q1.15 measured alpha/beta current at n, Ipu.
 * @param[in] voltage Q1.15 applied average voltage over [n,n+1), Vpu;
 *                    reconstruct actual switching voltage, not an unlimited request.
 * @param[out] output Non-NULL result storage, disjoint from ctx; inspect validity flags.
 *
 * @return linears_status_t Numeric execution status, distinct from rotor lock.
 * @retval LINEARS_OK Sample committed, including unlocked observations.
 * @retval LINEARS_ERROR_ARGUMENT ctx or output is NULL.
 * @retval LINEARS_ERROR_ALIAS output overlaps ctx.
 * @retval LINEARS_ERROR_RANGE F31 arithmetic overflows or a Q1 output exceeds [-1,+1].
 *
 * @pre Call at config.dt, use the selected pu bases, isotropic model and known direction.
 * @note Nonblocking; errors preserve ctx/output. Serialize shared access.
 * @warning No standstill observability, startup, automatic reversal or hardware protection.
 */
linears_status_t fieldctrl_q15_smo_step(fieldctrl_q15_smo_t *ctx,
    fieldctrl_q15_ab_t current, fieldctrl_q15_ab_t voltage,
    fieldctrl_q15_smo_output_t *output);

/**
 * @brief Q1.31 per-unit SMO tuning for fixed sampling and known direction.
 *
 * Require dt*(resistance+sliding_gain/boundary)/inductance < 2 in working
 * arithmetic and max_speed*dt < 1/4 turn. This local Euler stability check
 * does not guarantee convergence under model mismatch, noise or extra delays.
 * Positive Q1 parameters are at most 1-LSB; choose external bases accordingly.
 */
typedef struct {
    linears_q31_t resistance; /**< Nonnegative Rpu=R*Ibase/Vbase. */
    linears_q31_t inductance; /**< Positive Lpu=L*Ibase/(Vbase*Tbase), Ld=Lq. */
    linears_q31_t dt; /**< Positive tpu=sample_period/Tbase. */
    linears_q31_t sliding_gain; /**< Positive Kpu=K/Vbase; cover peak back-EMF. */
    linears_q31_t boundary; /**< Positive current-error boundary/Ibase. */
    linears_q31_t emf_alpha; /**< Q1.31 dimensionless input weight in (0,1); MAX<1. */
    linears_q31_t speed_alpha; /**< Q1.31 dimensionless input weight in (0,1); MAX<1. */
    linears_q31_t min_emf; /**< Positive filtered-EMF threshold/Vbase. */
    linears_q31_t min_speed; /**< Positive minimum electrical frequency f*Tbase. */
    linears_q31_t max_speed; /**< Maximum electrical f*Tbase, greater than min_speed. */
    uint32_t lock_samples; /**< Positive consecutive observable samples needed for lock. */
    fieldctrl_smo_direction_t direction; /**< Known fixed rotation direction. */
} fieldctrl_q31_smo_config_t;

/**
 * @brief Caller-owned Q1.31 observer with constant-binary-point F31 histories.
 *
 * The Q31 filter contexts store raw Q1.31 alpha and int64_t/F31 value. Updates
 * call the shared filters working-state recurrence, not a floating wrapper.
 */
typedef struct {
    fieldctrl_q31_smo_config_t config; /**< Immutable copied pu tuning; no base metadata. */
    struct {
        linears_fixed_t alpha; /**< int64_t/F31 next-sample current prediction, Ipu. */
        linears_fixed_t beta; /**< int64_t/F31 next-sample current prediction, Ipu. */
    } current_hat; /**< Fixed F31 predictor; successful exported components fit Q1. */
    filters_ema_q31_t emf_alpha_filter; /**< Alpha injection history, F31 Vpu. */
    filters_ema_q31_t emf_beta_filter; /**< Beta injection history, F31 Vpu. */
    filters_ema_q31_t speed_filter; /**< Frequency history, F31 f*Tbase; may exceed Q1. */
    linears_fixed_t previous_angle; /**< Uncorrected previous angle, F31 electrical turns. */
    linears_fixed_t angle; /**< Compensated angle, F31 electrical turns in [0,1). */
    uint32_t lock_count; /**< Saturating consecutive observable-sample counter. */
    int has_angle; /**< A previous above-threshold EMF angle is available. */
} fieldctrl_q31_smo_t;

/**
 * @brief Q1.31 observer result with a wide unlocked-frequency diagnostic.
 */
typedef struct {
    fieldctrl_q31_ab_t current_hat; /**< Q1.31 current prediction at the next sample, Ipu. */
    fieldctrl_q31_ab_t emf; /**< Q1.31 filtered injection, Vpu; not de-attenuated. */
    linears_q31_t electrical_angle; /**< Q1.31 electrical turns [0,1), cyclic rounding. */
    linears_fixed_t electrical_speed; /**< int64_t/F31 signed f*Tbase, possibly outside Q1 while unlocked. */
    int observable; /**< EMF, direction, speed and sliding-region criteria all passed. */
    int locked; /**< observable held for config.lock_samples consecutive samples. */
} fieldctrl_q31_smo_output_t;

/**
 * @brief Initialize a Q1.31 SMO with zero EMF/frequency and no lock.
 *
 * @param[out] ctx Non-NULL caller-owned context; no prior initialization required.
 * @param[in] config Non-NULL per-unit tuning copied by value; bounds documented above.
 * @param[in] initial_current Q1.31 alpha/beta initial current estimate, Ipu.
 *
 * @return linears_status_t Initialization status.
 * @retval LINEARS_OK Predictor and filters initialized, lock cleared.
 * @retval LINEARS_ERROR_ARGUMENT ctx or config is NULL.
 * @retval LINEARS_ERROR_DOMAIN Tuning, direction, stability or alias bound is invalid.
 * @retval LINEARS_ERROR_RANGE An intermediate exceeds the int64_t/F31 working range.
 *
 * @note Failure preserves ctx. Serialize shared calls, including interrupt calls.
 */
linears_status_t fieldctrl_q31_smo_init(fieldctrl_q31_smo_t *ctx,
    const fieldctrl_q31_smo_config_t *config, fieldctrl_q31_ab_t initial_current);

/**
 * @brief Predict current and estimate motion with the integer Q1.31 SMO.
 *
 * At sample n, z=K*sat((i_hat[n]-i[n])/boundary). Prediction advances as
 * i_hat[n+1]=i_hat[n]+dt/L*(v[n]-R*i_hat[n]-z). Shared low-pass filters extract
 * EMF and filter wrapped angle differences. Their exact discrete steady-state
 * phase response is compensated at the signed estimated frequency. Observer,
 * ADC/PWM delay and acceleration transients are not compensated.
 *
 * Below min_emf, hold the last angle (zero before acquisition), clear speed
 * history and lock. Otherwise observable requires a previous angle, expected
 * raw/filtered speed signs, both speed magnitudes in [min_speed,max_speed],
 * and both current errors within boundary. Any failed criterion clears lock.
 * A wide unlocked frequency is diagnostic only; check flags before exporting
 * it to a Q1 controller input. Q15 cyclic rounding that reaches one wraps to zero.
 *
 * @param[in,out] ctx Non-NULL successfully initialized caller-owned context.
 * @param[in] current Q1.31 measured alpha/beta current at n, Ipu.
 * @param[in] voltage Q1.31 applied average voltage over [n,n+1), Vpu;
 *                    reconstruct actual switching voltage, not an unlimited request.
 * @param[out] output Non-NULL result storage, disjoint from ctx; inspect validity flags.
 *
 * @return linears_status_t Numeric execution status, distinct from rotor lock.
 * @retval LINEARS_OK Sample committed, including unlocked observations.
 * @retval LINEARS_ERROR_ARGUMENT ctx or output is NULL.
 * @retval LINEARS_ERROR_ALIAS output overlaps ctx.
 * @retval LINEARS_ERROR_RANGE F31 arithmetic overflows or a Q1 output exceeds [-1,+1].
 *
 * @pre Call at config.dt, use the selected pu bases, isotropic model and known direction.
 * @note Nonblocking; errors preserve ctx/output. Serialize shared access.
 * @warning No standstill observability, startup, automatic reversal or hardware protection.
 */
linears_status_t fieldctrl_q31_smo_step(fieldctrl_q31_smo_t *ctx,
    fieldctrl_q31_ab_t current, fieldctrl_q31_ab_t voltage,
    fieldctrl_q31_smo_output_t *output);

#ifdef __cplusplus
}
#endif
#endif
