/**
 * @file fieldctrl_smo.h
 * @brief Boundary-layer sliding-mode back-EMF observer for isotropic PMSMs.
 *
 * The observer predicts alpha/beta current using v-R*i_hat-z and estimates
 * back-EMF by first-order low-pass filtering the switching injection z.
 * Angles are electrical turns in [0,1), positive from alpha toward beta.
 * Frequencies are signed electrical turns/s, not mechanical speed or rad/s.
 * Q15/Q31 computations use the linears pure-integer exponent-bearing core.
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
#include "filters_lowpass.h"

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
    filters_lowpass_f32_t emf_alpha_filter; /**< Alpha switching-injection history, V. */
    filters_lowpass_f32_t emf_beta_filter; /**< Beta switching-injection history, V. */
    filters_lowpass_f32_t speed_filter; /**< Electrical-frequency history, turn/s. */
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
 * @retval LINEARS_ERROR_RANGE A scalar/intermediate is nonfinite or outside exponent range.
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
 * @retval LINEARS_ERROR_RANGE An input/intermediate/output is nonfinite or out of exponent range.
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
    filters_lowpass_f64_t emf_alpha_filter; /**< Alpha switching-injection history, V. */
    filters_lowpass_f64_t emf_beta_filter; /**< Beta switching-injection history, V. */
    filters_lowpass_f64_t speed_filter; /**< Electrical-frequency history, turn/s. */
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
 * @retval LINEARS_ERROR_RANGE A scalar/intermediate is nonfinite or outside exponent range.
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
 * @retval LINEARS_ERROR_RANGE An input/intermediate/output is nonfinite or out of exponent range.
 * @pre Use exactly config.dt between calls; motor model is isotropic and direction known.
 * @note Nonblocking; errors preserve ctx/output. Serialize use of ctx.
 * @warning No standstill observability, startup, automatic reversal or hardware protection.
 */
linears_status_t fieldctrl_f64_smo_step(fieldctrl_f64_smo_t *ctx,
    fieldctrl_f64_ab_t current, fieldctrl_f64_ab_t voltage,
    fieldctrl_f64_smo_output_t *output);

/**
 * @brief Q15 SMO tuning for a fixed sampling period and known direction.
 *
 * Require dt*(resistance+sliding_gain/boundary)/inductance < 2, the explicit
 * Euler local boundary-layer stability limit. This is not a guarantee under
 * model mismatch. max_speed*dt must be less than 1/4 turn to reject ambiguous
 * angle steps. Gains/thresholds require application-specific noise margins.
 */
typedef struct {
    linears_q15_t resistance; /**< Nonnegative stator phase resistance, ohm. */
    linears_q15_t inductance; /**< Positive isotropic phase inductance Ld=Lq, H. */
    linears_q15_t dt; /**< Positive fixed sampling interval, s. */
    linears_q15_t sliding_gain; /**< Positive injection magnitude K, V; exceed peak back-EMF. */
    linears_q15_t boundary; /**< Positive current-error saturation boundary, A. */
    linears_q15_t emf_alpha; /**< EMF low-pass input weight in (0,1], dimensionless. */
    linears_q15_t speed_alpha; /**< Speed low-pass input weight in (0,1], dimensionless. */
    linears_q15_t min_emf; /**< Positive filtered EMF magnitude threshold, V. */
    linears_q15_t min_speed; /**< Positive minimum observable electrical speed magnitude, turn/s. */
    linears_q15_t max_speed; /**< Maximum electrical speed magnitude, turn/s; greater than min_speed. */
    uint32_t lock_samples; /**< Positive consecutive observable-sample count for lock. */
    fieldctrl_smo_direction_t direction; /**< Known fixed rotation direction. */
} fieldctrl_q15_smo_config_t;

/**
 * @brief Q15 caller-owned SMO state with independent scalar filters.
 * @note Q15 inputs/outputs use Q31 integer predictor and filter state to retain small increments.
 */
typedef struct {
    fieldctrl_q15_smo_config_t config; /**< Immutable copied motor/observer tuning. */
    struct {
        linears_fixed_t alpha; /**< Predicted next-sample alpha current, A. */
        linears_fixed_t beta; /**< Predicted next-sample beta current, A. */
    } current_hat; /**< Current predictor at working precision. */
    filters_lowpass_q31_t emf_alpha_filter; /**< Alpha switching-injection history, V. */
    filters_lowpass_q31_t emf_beta_filter; /**< Beta switching-injection history, V. */
    filters_lowpass_q31_t speed_filter; /**< Electrical-frequency history, turn/s. */
    linears_fixed_t previous_angle; /**< Previous uncorrected EMF-derived angle, turns. */
    linears_fixed_t angle; /**< Last compensated angle, turns in [0,1). */
    uint32_t lock_count; /**< Saturating count of consecutive observable samples. */
    int has_angle; /**< A previous above-threshold EMF angle is available. */
} fieldctrl_q15_smo_t;

/**
 * @brief Q15 observer sample and explicit observability/lock indicators.
 */
typedef struct {
    fieldctrl_q15_ab_t current_hat; /**< Predicted current at the next sample, A. */
    fieldctrl_q15_ab_t emf; /**< Low-pass switching injection, V; not de-attenuated. */
    linears_q15_t electrical_angle; /**< LP-phase-compensated flux angle, turns in [0,1). */
    linears_q15_t electrical_speed; /**< Filtered signed electrical speed, turns/s. */
    int observable; /**< EMF, raw/filtered speed and sliding-region tests all passed. */
    int locked; /**< observable held for config.lock_samples consecutive samples. */
} fieldctrl_q15_smo_output_t;

/**
 * @brief Initialize a Q15 SMO with zero EMF/speed and no lock.
 *
 * @param[out] ctx Non-NULL caller-owned context; no prior initialization needed.
 * @param[in] config Non-NULL tuning copied by value; limits as documented above.
 * @param[in] initial_current Initial simultaneous alpha/beta current estimate, A.
 *
 * @return linears_status_t Initialization status.
 * @retval LINEARS_OK Predictor and filters initialized, lock cleared.
 * @retval LINEARS_ERROR_ARGUMENT ctx or config is NULL.
 * @retval LINEARS_ERROR_DOMAIN A tuning, direction, stability or alias bound is invalid.
 * @retval LINEARS_ERROR_RANGE A scalar/intermediate is nonfinite or outside exponent range.
 * @note Error preserves ctx. Serialize concurrent access to ctx.
 */
linears_status_t fieldctrl_q15_smo_init(fieldctrl_q15_smo_t *ctx,
    const fieldctrl_q15_smo_config_t *config, fieldctrl_q15_ab_t initial_current);

/**
 * @brief Predict current and estimate rotor motion using a Q15 SMO.
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
 * @retval LINEARS_ERROR_RANGE An input/intermediate/output is nonfinite or out of exponent range.
 * @pre Use exactly config.dt between calls; motor model is isotropic and direction known.
 * @note Nonblocking; errors preserve ctx/output. Serialize use of ctx.
 * @warning No standstill observability, startup, automatic reversal or hardware protection.
 */
linears_status_t fieldctrl_q15_smo_step(fieldctrl_q15_smo_t *ctx,
    fieldctrl_q15_ab_t current, fieldctrl_q15_ab_t voltage,
    fieldctrl_q15_smo_output_t *output);

/**
 * @brief Q31 SMO tuning for a fixed sampling period and known direction.
 *
 * Require dt*(resistance+sliding_gain/boundary)/inductance < 2, the explicit
 * Euler local boundary-layer stability limit. This is not a guarantee under
 * model mismatch. max_speed*dt must be less than 1/4 turn to reject ambiguous
 * angle steps. Gains/thresholds require application-specific noise margins.
 */
typedef struct {
    linears_q31_t resistance; /**< Nonnegative stator phase resistance, ohm. */
    linears_q31_t inductance; /**< Positive isotropic phase inductance Ld=Lq, H. */
    linears_q31_t dt; /**< Positive fixed sampling interval, s. */
    linears_q31_t sliding_gain; /**< Positive injection magnitude K, V; exceed peak back-EMF. */
    linears_q31_t boundary; /**< Positive current-error saturation boundary, A. */
    linears_q31_t emf_alpha; /**< EMF low-pass input weight in (0,1], dimensionless. */
    linears_q31_t speed_alpha; /**< Speed low-pass input weight in (0,1], dimensionless. */
    linears_q31_t min_emf; /**< Positive filtered EMF magnitude threshold, V. */
    linears_q31_t min_speed; /**< Positive minimum observable electrical speed magnitude, turn/s. */
    linears_q31_t max_speed; /**< Maximum electrical speed magnitude, turn/s; greater than min_speed. */
    uint32_t lock_samples; /**< Positive consecutive observable-sample count for lock. */
    fieldctrl_smo_direction_t direction; /**< Known fixed rotation direction. */
} fieldctrl_q31_smo_config_t;

/**
 * @brief Q31 caller-owned SMO state with independent scalar filters.
 * @note Predictor and low-pass histories use the shared normalized Q31 integer core.
 */
typedef struct {
    fieldctrl_q31_smo_config_t config; /**< Immutable copied motor/observer tuning. */
    struct {
        linears_fixed_t alpha; /**< Predicted next-sample alpha current, A. */
        linears_fixed_t beta; /**< Predicted next-sample beta current, A. */
    } current_hat; /**< Current predictor at working precision. */
    filters_lowpass_q31_t emf_alpha_filter; /**< Alpha switching-injection history, V. */
    filters_lowpass_q31_t emf_beta_filter; /**< Beta switching-injection history, V. */
    filters_lowpass_q31_t speed_filter; /**< Electrical-frequency history, turn/s. */
    linears_fixed_t previous_angle; /**< Previous uncorrected EMF-derived angle, turns. */
    linears_fixed_t angle; /**< Last compensated angle, turns in [0,1). */
    uint32_t lock_count; /**< Saturating count of consecutive observable samples. */
    int has_angle; /**< A previous above-threshold EMF angle is available. */
} fieldctrl_q31_smo_t;

/**
 * @brief Q31 observer sample and explicit observability/lock indicators.
 */
typedef struct {
    fieldctrl_q31_ab_t current_hat; /**< Predicted current at the next sample, A. */
    fieldctrl_q31_ab_t emf; /**< Low-pass switching injection, V; not de-attenuated. */
    linears_q31_t electrical_angle; /**< LP-phase-compensated flux angle, turns in [0,1). */
    linears_q31_t electrical_speed; /**< Filtered signed electrical speed, turns/s. */
    int observable; /**< EMF, raw/filtered speed and sliding-region tests all passed. */
    int locked; /**< observable held for config.lock_samples consecutive samples. */
} fieldctrl_q31_smo_output_t;

/**
 * @brief Initialize a Q31 SMO with zero EMF/speed and no lock.
 *
 * @param[out] ctx Non-NULL caller-owned context; no prior initialization needed.
 * @param[in] config Non-NULL tuning copied by value; limits as documented above.
 * @param[in] initial_current Initial simultaneous alpha/beta current estimate, A.
 *
 * @return linears_status_t Initialization status.
 * @retval LINEARS_OK Predictor and filters initialized, lock cleared.
 * @retval LINEARS_ERROR_ARGUMENT ctx or config is NULL.
 * @retval LINEARS_ERROR_DOMAIN A tuning, direction, stability or alias bound is invalid.
 * @retval LINEARS_ERROR_RANGE A scalar/intermediate is nonfinite or outside exponent range.
 * @note Error preserves ctx. Serialize concurrent access to ctx.
 */
linears_status_t fieldctrl_q31_smo_init(fieldctrl_q31_smo_t *ctx,
    const fieldctrl_q31_smo_config_t *config, fieldctrl_q31_ab_t initial_current);

/**
 * @brief Predict current and estimate rotor motion using a Q31 SMO.
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
 * @retval LINEARS_ERROR_RANGE An input/intermediate/output is nonfinite or out of exponent range.
 * @pre Use exactly config.dt between calls; motor model is isotropic and direction known.
 * @note Nonblocking; errors preserve ctx/output. Serialize use of ctx.
 * @warning No standstill observability, startup, automatic reversal or hardware protection.
 */
linears_status_t fieldctrl_q31_smo_step(fieldctrl_q31_smo_t *ctx,
    fieldctrl_q31_ab_t current, fieldctrl_q31_ab_t voltage,
    fieldctrl_q31_smo_output_t *output);


#ifdef __cplusplus
}
#endif
#endif
