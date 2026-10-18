/**
 * @file fieldctrl_base.h
 * @brief Coordinate transforms, bounded PI, modulation and open-loop VVVF.
 *
 * Every successful scalar output is finite or representable in its fixed format.
 * Every failure leaves outputs and contexts unchanged. Scalar inputs are by
 * value; vector outputs may replace their input object. Multi-output pointers
 * must be disjoint, and outputs must not overlap mutable contexts. Callers
 * keep initialized contexts alive and serialize their use, also in interrupts.
 * Independent contexts are reentrant. There is no I/O, allocation or locking.
 */
#ifndef FIELDCTRL_BASE_H
#define FIELDCTRL_BASE_H

#include "fieldctrl_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief single-precision PI state with conditional-integration anti-windup.
 *
 * The integral uses native float precision. Fields are library-owned after successful
 * initialization. Saturation is intentional and returns OK, not RANGE.
 */
typedef struct {
    float kp; /**< Nonnegative proportional gain, output units per error unit. */
    float ki; /**< Nonnegative integral gain, output/(error*second). */
    float lower; /**< Inclusive lower output and integral bound. */
    float upper; /**< Inclusive upper output and integral bound. */
    float integral; /**< Accumulated output contribution; not caller-modifiable. */
} fieldctrl_f32_pi_t;

/**
 * @brief single-precision open-loop rotating voltage-generator state.
 */
typedef struct {
    float angle; /**< Electrical turns in [0,1), retained at working precision. */
} fieldctrl_f32_vvvf_t;

/**
 * @brief single-precision VVVF outputs at the newly advanced electrical angle.
 */
typedef struct {
    float angle; /**< Electrical turns in [0,1). */
    fieldctrl_f32_ab_t voltage; /**< Limited alpha/beta voltage in bus-voltage units. */
    fieldctrl_f32_abc_t duty; /**< Three high-side duty ratios, each in [0,1]. */
    int saturated; /**< Nonzero if requested amplitude exceeded Vdc/sqrt(3). */
} fieldctrl_f32_vvvf_output_t;

/**
 * @brief Reduce a single-precision electrical angle modulo one turn.
 *
 * @param[in] angle Electrical turns, of either sign; finite.
 * @param[out] output Non-NULL wrapped angle in [0,1).
 * @return linears_status_t OK, ARGUMENT for NULL output, or RANGE for an invalid
 *         nonfinite input or unrepresentable working result.
 * @note A value rounding to one turn is represented by zero; negative zero
 *       is canonicalized to positive zero. Output is unchanged on failure.
 */
linears_status_t fieldctrl_f32_wrap(float angle, float *output);

/**
 * @brief Compute sine and cosine of a single-precision electrical angle.
 *
 * @param[in] angle Electrical turns; any finite signed value.
 * @param[out] sine Non-NULL dimensionless sine in [-1,1].
 * @param[out] cosine Non-NULL dimensionless cosine in [-1,1], disjoint from sine.
 * @return linears_status_t OK, ARGUMENT for NULL, ALIAS for overlap, or RANGE
 *         for an nonfinite input or unrepresentable result.
 * @note q variants use exactly 32 CORDIC rotations, except exact cardinal
 *       angles. Both outputs are unchanged on failure; no hardware is touched.
 */
linears_status_t fieldctrl_f32_sincos(float angle, float *sine, float *cosine);

/**
 * @brief Obtain a single-precision vector angle in electrical turns.
 *
 * @param[in] y Beta component in the same units as x; finite.
 * @param[in] x Alpha component; finite.
 * @param[out] angle Non-NULL electrical angle in [0,1).
 * @return linears_status_t OK, ARGUMENT for NULL, DOMAIN for x=y=0, or RANGE
 *         for nonfinite inputs or an unrepresentable result.
 * @note q variants use at most 32 CORDIC vectoring rotations. Failure preserves
 *       angle. Signed floating zero does not create a distinct physical angle.
 */
linears_status_t fieldctrl_f32_atan2(float y, float x, float *angle);

/**
 * @brief Apply the amplitude-invariant single-precision Clarke transform.
 *
 * @param[in] phase Phase a,b,c currents or voltages in identical units.
 * @param[out] stationary Non-NULL alpha/beta output in the same units.
 * @return linears_status_t OK, ARGUMENT for NULL, or RANGE for invalid scalars
 *         or unrepresentable arithmetic. Output is unchanged on failure.
 * @note alpha=(2*a-b-c)/3 and beta=(b-c)/sqrt(3); zero sequence is discarded.
 */
linears_status_t fieldctrl_f32_clarke(fieldctrl_f32_abc_t phase,
                                    fieldctrl_f32_ab_t *stationary);

/**
 * @brief Convert a single-precision stationary vector to balanced phase quantities.
 *
 * @param[in] stationary Alpha/beta input, in common current or voltage units.
 * @param[out] phase Non-NULL a,b,c output in those units, with zero sum.
 * @return linears_status_t OK, ARGUMENT for NULL, or RANGE for invalid scalars
 *         or unrepresentable arithmetic. Output is unchanged on failure.
 */
linears_status_t fieldctrl_f32_inverse_clarke(fieldctrl_f32_ab_t stationary,
                                            fieldctrl_f32_abc_t *phase);

/**
 * @brief Rotate a single-precision stationary vector into the rotor frame.
 *
 * @param[in] stationary Alpha/beta quantities with common units.
 * @param[in] angle Electrical rotor angle in turns, positive alpha toward beta.
 * @param[out] rotating Non-NULL d/q output with the same units.
 * @return linears_status_t OK, ARGUMENT for NULL, or RANGE for invalid scalars
 *         or unrepresentable arithmetic; failure preserves rotating.
 * @note d=alpha*cos+beta*sin, q=-alpha*sin+beta*cos.
 */
linears_status_t fieldctrl_f32_park(fieldctrl_f32_ab_t stationary, float angle,
                                   fieldctrl_f32_dq_t *rotating);

/**
 * @brief Rotate a single-precision rotor-frame vector into the stationary frame.
 *
 * @param[in] rotating Direct and quadrature components in common units.
 * @param[in] angle Electrical rotor angle in turns, positive alpha toward beta.
 * @param[out] stationary Non-NULL alpha/beta output in those units.
 * @return linears_status_t OK, ARGUMENT for NULL, or RANGE for invalid scalars
 *         or unrepresentable arithmetic; failure preserves stationary.
 */
linears_status_t fieldctrl_f32_inverse_park(fieldctrl_f32_dq_t rotating, float angle,
                                           fieldctrl_f32_ab_t *stationary);

/**
 * @brief Limit single-precision voltage magnitude radially without changing direction.
 *
 * @param[in] voltage Alpha/beta voltage vector.
 * @param[in] limit Nonnegative peak vector magnitude in voltage units.
 * @param[out] limited Non-NULL limited voltage; zero limit yields zero.
 * @return linears_status_t OK (also when clipped), ARGUMENT for NULL, DOMAIN
 *         for a negative limit, or RANGE for invalid/unrepresentable numbers.
 * @note Rounding is to the selected output precision; the bound can differ
 *       by final-output rounding. Failure preserves limited.
 */
linears_status_t fieldctrl_f32_limit_voltage(fieldctrl_f32_ab_t voltage, float limit,
                                            fieldctrl_f32_ab_t *limited);

/**
 * @brief Compute single-precision center-aligned SVPWM duty ratios.
 *
 * @param[in] voltage Requested alpha/beta voltage in bus-voltage units.
 * @param[in] dc_bus Positive DC bus voltage in the same units.
 * @param[out] duty Non-NULL high-side duty ratios in [0,1], a,b,c order.
 * @return linears_status_t OK, ARGUMENT for NULL, DOMAIN for dc_bus<=0, or
 *         RANGE for invalid scalars/unrepresentable arithmetic.
 * @note First limit magnitude to dc_bus/sqrt(3), then inject -(max+min)/2
 *       common-mode voltage. Duty is 1/2+phase/dc_bus. This is not overmodulation.
 *       No timer configuration, dead-time, minimum pulse or gate action occurs.
 *       Failure preserves duty. Clipping is a successful command calculation.
 */
linears_status_t fieldctrl_f32_svpwm(fieldctrl_f32_ab_t voltage, float dc_bus,
                                    fieldctrl_f32_abc_t *duty);

/**
 * @brief Initialize a single-precision PI without allocating state.
 *
 * @param[out] ctx Non-NULL caller-owned context, unchanged on failure.
 * @param[in] kp Nonnegative gain in output/error units.
 * @param[in] ki Nonnegative gain in output/(error*second).
 * @param[in] lower Inclusive lower output and integral bound.
 * @param[in] upper Inclusive upper bound, strictly greater than lower.
 * @param[in] integral Initial integral contribution in [lower,upper].
 * @return linears_status_t OK, ARGUMENT for NULL, DOMAIN for invalid bounds or
 *         negative gains, or RANGE for invalid/unrepresentable numeric values.
 * @note Context fields are library-owned after initialization. Calls sharing
 *       ctx require serialization; separate contexts are reentrant.
 */
linears_status_t fieldctrl_f32_pi_init(fieldctrl_f32_pi_t *ctx, float kp, float ki,
                                      float lower, float upper, float integral);

/**
 * @brief Advance a single-precision conditional-integration PI by one sample.
 *
 * @param[in,out] ctx Non-NULL successfully initialized context.
 * @param[in] error Reference minus feedback in controller error units.
 * @param[in] dt Positive sample period in seconds; no upper bound is imposed
 *               beyond representable arithmetic. The caller ensures stability.
 * @param[out] output Non-NULL command, disjoint from ctx, in [lower,upper].
 * @return linears_status_t OK including saturation, ARGUMENT for NULL, ALIAS
 *         for overlap, DOMAIN for dt<=0, or RANGE for invalid numeric values.
 * @note Candidate I=clamp(I+ki*dt*error). If its unconstrained PI output exceeds
 *       a limit and the integral increment drives further into saturation,
 *       reject that increment, then recompute/clamp the output using old I.
 *       Both context and output remain unchanged on failure.
 */
linears_status_t fieldctrl_f32_pi_step(fieldctrl_f32_pi_t *ctx, float error, float dt,
                                      float *output);

/**
 * @brief Initialize a single-precision VVVF generator at a selected field angle.
 *
 * @param[out] ctx Non-NULL caller-owned state, unchanged on failure.
 * @param[in] angle Initial electrical turns, reduced modulo one turn.
 * @return linears_status_t OK, ARGUMENT for NULL, or RANGE for invalid or
 *         unrepresentable values. Context fields must not be edited directly.
 */
linears_status_t fieldctrl_f32_vvvf_init(fieldctrl_f32_vvvf_t *ctx, float angle);

/**
 * @brief Advance a single-precision open-loop variable-frequency, variable-amplitude field.
 *
 * @param[in,out] ctx Non-NULL initialized context.
 * @param[in] frequency Signed electrical turns/second; negative reverses the
 *                      field. Zero holds angle without implying rotor lock.
 * @param[in] amplitude Nonnegative requested peak phase/vector voltage.
 * @param[in] dc_bus Positive bus voltage in amplitude's units.
 * @param[in] dt Positive seconds, with abs(frequency)*dt <= 1/2 turn; this
 *               sampling bound prevents aliasing of the commanded field.
 * @param[out] output Non-NULL result, disjoint from ctx; no hardware action.
 * @return linears_status_t OK including clipping, ARGUMENT for NULL, ALIAS for
 *         overlap, DOMAIN for invalid signs/sample advance, or RANGE for
 *         invalid/unrepresentable numbers. Failure preserves ctx and output.
 * @note New angle=wrap(old+frequency*dt); voltage=amplitude*(cos,sin), limited
 *       to dc_bus/sqrt(3). Frequency and amplitude may vary every call. A V/f
 *       law, ramps, rotor synchronization and current limits are caller policy.
 */
linears_status_t fieldctrl_f32_vvvf_step(fieldctrl_f32_vvvf_t *ctx,
                                        float frequency, float amplitude,
                                        float dc_bus, float dt,
                                        fieldctrl_f32_vvvf_output_t *output);

/**
 * @brief double-precision PI state with conditional-integration anti-windup.
 *
 * The integral uses native double precision. Fields are library-owned after successful
 * initialization. Saturation is intentional and returns OK, not RANGE.
 */
typedef struct {
    double kp; /**< Nonnegative proportional gain, output units per error unit. */
    double ki; /**< Nonnegative integral gain, output/(error*second). */
    double lower; /**< Inclusive lower output and integral bound. */
    double upper; /**< Inclusive upper output and integral bound. */
    double integral; /**< Accumulated output contribution; not caller-modifiable. */
} fieldctrl_f64_pi_t;

/**
 * @brief double-precision open-loop rotating voltage-generator state.
 */
typedef struct {
    double angle; /**< Electrical turns in [0,1), retained at working precision. */
} fieldctrl_f64_vvvf_t;

/**
 * @brief double-precision VVVF outputs at the newly advanced electrical angle.
 */
typedef struct {
    double angle; /**< Electrical turns in [0,1). */
    fieldctrl_f64_ab_t voltage; /**< Limited alpha/beta voltage in bus-voltage units. */
    fieldctrl_f64_abc_t duty; /**< Three high-side duty ratios, each in [0,1]. */
    int saturated; /**< Nonzero if requested amplitude exceeded Vdc/sqrt(3). */
} fieldctrl_f64_vvvf_output_t;

/**
 * @brief Reduce a double-precision electrical angle modulo one turn.
 *
 * @param[in] angle Electrical turns, of either sign; finite.
 * @param[out] output Non-NULL wrapped angle in [0,1).
 * @return linears_status_t OK, ARGUMENT for NULL output, or RANGE for an invalid
 *         nonfinite input or unrepresentable working result.
 * @note A value rounding to one turn is represented by zero; negative zero
 *       is canonicalized to positive zero. Output is unchanged on failure.
 */
linears_status_t fieldctrl_f64_wrap(double angle, double *output);

/**
 * @brief Compute sine and cosine of a double-precision electrical angle.
 *
 * @param[in] angle Electrical turns; any finite signed value.
 * @param[out] sine Non-NULL dimensionless sine in [-1,1].
 * @param[out] cosine Non-NULL dimensionless cosine in [-1,1], disjoint from sine.
 * @return linears_status_t OK, ARGUMENT for NULL, ALIAS for overlap, or RANGE
 *         for an nonfinite input or unrepresentable result.
 * @note q variants use exactly 32 CORDIC rotations, except exact cardinal
 *       angles. Both outputs are unchanged on failure; no hardware is touched.
 */
linears_status_t fieldctrl_f64_sincos(double angle, double *sine, double *cosine);

/**
 * @brief Obtain a double-precision vector angle in electrical turns.
 *
 * @param[in] y Beta component in the same units as x; finite.
 * @param[in] x Alpha component; finite.
 * @param[out] angle Non-NULL electrical angle in [0,1).
 * @return linears_status_t OK, ARGUMENT for NULL, DOMAIN for x=y=0, or RANGE
 *         for nonfinite inputs or an unrepresentable result.
 * @note q variants use at most 32 CORDIC vectoring rotations. Failure preserves
 *       angle. Signed floating zero does not create a distinct physical angle.
 */
linears_status_t fieldctrl_f64_atan2(double y, double x, double *angle);

/**
 * @brief Apply the amplitude-invariant double-precision Clarke transform.
 *
 * @param[in] phase Phase a,b,c currents or voltages in identical units.
 * @param[out] stationary Non-NULL alpha/beta output in the same units.
 * @return linears_status_t OK, ARGUMENT for NULL, or RANGE for invalid scalars
 *         or unrepresentable arithmetic. Output is unchanged on failure.
 * @note alpha=(2*a-b-c)/3 and beta=(b-c)/sqrt(3); zero sequence is discarded.
 */
linears_status_t fieldctrl_f64_clarke(fieldctrl_f64_abc_t phase,
                                    fieldctrl_f64_ab_t *stationary);

/**
 * @brief Convert a double-precision stationary vector to balanced phase quantities.
 *
 * @param[in] stationary Alpha/beta input, in common current or voltage units.
 * @param[out] phase Non-NULL a,b,c output in those units, with zero sum.
 * @return linears_status_t OK, ARGUMENT for NULL, or RANGE for invalid scalars
 *         or unrepresentable arithmetic. Output is unchanged on failure.
 */
linears_status_t fieldctrl_f64_inverse_clarke(fieldctrl_f64_ab_t stationary,
                                            fieldctrl_f64_abc_t *phase);

/**
 * @brief Rotate a double-precision stationary vector into the rotor frame.
 *
 * @param[in] stationary Alpha/beta quantities with common units.
 * @param[in] angle Electrical rotor angle in turns, positive alpha toward beta.
 * @param[out] rotating Non-NULL d/q output with the same units.
 * @return linears_status_t OK, ARGUMENT for NULL, or RANGE for invalid scalars
 *         or unrepresentable arithmetic; failure preserves rotating.
 * @note d=alpha*cos+beta*sin, q=-alpha*sin+beta*cos.
 */
linears_status_t fieldctrl_f64_park(fieldctrl_f64_ab_t stationary, double angle,
                                   fieldctrl_f64_dq_t *rotating);

/**
 * @brief Rotate a double-precision rotor-frame vector into the stationary frame.
 *
 * @param[in] rotating Direct and quadrature components in common units.
 * @param[in] angle Electrical rotor angle in turns, positive alpha toward beta.
 * @param[out] stationary Non-NULL alpha/beta output in those units.
 * @return linears_status_t OK, ARGUMENT for NULL, or RANGE for invalid scalars
 *         or unrepresentable arithmetic; failure preserves stationary.
 */
linears_status_t fieldctrl_f64_inverse_park(fieldctrl_f64_dq_t rotating, double angle,
                                           fieldctrl_f64_ab_t *stationary);

/**
 * @brief Limit double-precision voltage magnitude radially without changing direction.
 *
 * @param[in] voltage Alpha/beta voltage vector.
 * @param[in] limit Nonnegative peak vector magnitude in voltage units.
 * @param[out] limited Non-NULL limited voltage; zero limit yields zero.
 * @return linears_status_t OK (also when clipped), ARGUMENT for NULL, DOMAIN
 *         for a negative limit, or RANGE for invalid/unrepresentable numbers.
 * @note Rounding is to the selected output precision; the bound can differ
 *       by final-output rounding. Failure preserves limited.
 */
linears_status_t fieldctrl_f64_limit_voltage(fieldctrl_f64_ab_t voltage, double limit,
                                            fieldctrl_f64_ab_t *limited);

/**
 * @brief Compute double-precision center-aligned SVPWM duty ratios.
 *
 * @param[in] voltage Requested alpha/beta voltage in bus-voltage units.
 * @param[in] dc_bus Positive DC bus voltage in the same units.
 * @param[out] duty Non-NULL high-side duty ratios in [0,1], a,b,c order.
 * @return linears_status_t OK, ARGUMENT for NULL, DOMAIN for dc_bus<=0, or
 *         RANGE for invalid scalars/unrepresentable arithmetic.
 * @note First limit magnitude to dc_bus/sqrt(3), then inject -(max+min)/2
 *       common-mode voltage. Duty is 1/2+phase/dc_bus. This is not overmodulation.
 *       No timer configuration, dead-time, minimum pulse or gate action occurs.
 *       Failure preserves duty. Clipping is a successful command calculation.
 */
linears_status_t fieldctrl_f64_svpwm(fieldctrl_f64_ab_t voltage, double dc_bus,
                                    fieldctrl_f64_abc_t *duty);

/**
 * @brief Initialize a double-precision PI without allocating state.
 *
 * @param[out] ctx Non-NULL caller-owned context, unchanged on failure.
 * @param[in] kp Nonnegative gain in output/error units.
 * @param[in] ki Nonnegative gain in output/(error*second).
 * @param[in] lower Inclusive lower output and integral bound.
 * @param[in] upper Inclusive upper bound, strictly greater than lower.
 * @param[in] integral Initial integral contribution in [lower,upper].
 * @return linears_status_t OK, ARGUMENT for NULL, DOMAIN for invalid bounds or
 *         negative gains, or RANGE for invalid/unrepresentable numeric values.
 * @note Context fields are library-owned after initialization. Calls sharing
 *       ctx require serialization; separate contexts are reentrant.
 */
linears_status_t fieldctrl_f64_pi_init(fieldctrl_f64_pi_t *ctx, double kp, double ki,
                                      double lower, double upper, double integral);

/**
 * @brief Advance a double-precision conditional-integration PI by one sample.
 *
 * @param[in,out] ctx Non-NULL successfully initialized context.
 * @param[in] error Reference minus feedback in controller error units.
 * @param[in] dt Positive sample period in seconds; no upper bound is imposed
 *               beyond representable arithmetic. The caller ensures stability.
 * @param[out] output Non-NULL command, disjoint from ctx, in [lower,upper].
 * @return linears_status_t OK including saturation, ARGUMENT for NULL, ALIAS
 *         for overlap, DOMAIN for dt<=0, or RANGE for invalid numeric values.
 * @note Candidate I=clamp(I+ki*dt*error). If its unconstrained PI output exceeds
 *       a limit and the integral increment drives further into saturation,
 *       reject that increment, then recompute/clamp the output using old I.
 *       Both context and output remain unchanged on failure.
 */
linears_status_t fieldctrl_f64_pi_step(fieldctrl_f64_pi_t *ctx, double error, double dt,
                                      double *output);

/**
 * @brief Initialize a double-precision VVVF generator at a selected field angle.
 *
 * @param[out] ctx Non-NULL caller-owned state, unchanged on failure.
 * @param[in] angle Initial electrical turns, reduced modulo one turn.
 * @return linears_status_t OK, ARGUMENT for NULL, or RANGE for invalid or
 *         unrepresentable values. Context fields must not be edited directly.
 */
linears_status_t fieldctrl_f64_vvvf_init(fieldctrl_f64_vvvf_t *ctx, double angle);

/**
 * @brief Advance a double-precision open-loop variable-frequency, variable-amplitude field.
 *
 * @param[in,out] ctx Non-NULL initialized context.
 * @param[in] frequency Signed electrical turns/second; negative reverses the
 *                      field. Zero holds angle without implying rotor lock.
 * @param[in] amplitude Nonnegative requested peak phase/vector voltage.
 * @param[in] dc_bus Positive bus voltage in amplitude's units.
 * @param[in] dt Positive seconds, with abs(frequency)*dt <= 1/2 turn; this
 *               sampling bound prevents aliasing of the commanded field.
 * @param[out] output Non-NULL result, disjoint from ctx; no hardware action.
 * @return linears_status_t OK including clipping, ARGUMENT for NULL, ALIAS for
 *         overlap, DOMAIN for invalid signs/sample advance, or RANGE for
 *         invalid/unrepresentable numbers. Failure preserves ctx and output.
 * @note New angle=wrap(old+frequency*dt); voltage=amplitude*(cos,sin), limited
 *       to dc_bus/sqrt(3). Frequency and amplitude may vary every call. A V/f
 *       law, ramps, rotor synchronization and current limits are caller policy.
 */
linears_status_t fieldctrl_f64_vvvf_step(fieldctrl_f64_vvvf_t *ctx,
                                        double frequency, double amplitude,
                                        double dc_bus, double dt,
                                        fieldctrl_f64_vvvf_output_t *output);

/**
 * @brief Q15 signal PI with wide int64 F31 gains and integral state.
 *
 * Signals are raw int16_t/2^15; gains and integral are int64_t/2^31.
 * Gains are not Q1 signals, even for this Q15 entry point. All physical
 * quantities use consistent caller-selected per-unit bases; time is t/Tbase.
 * State is library-owned after initialization, and shared calls are serialized.
 */
typedef struct {
    linears_fixed_t kp; /**< Nonnegative F31 output-pu/error-pu gain. */
    linears_fixed_t ki; /**< Nonnegative F31 output-pu/(error-pu*time-pu) gain. */
    linears_q15_t lower; /**< Inclusive Q1 output and integral lower bound. */
    linears_q15_t upper; /**< Inclusive Q1 upper bound, greater than lower. */
    linears_fixed_t integral; /**< Output-pu integral, int64 raw/2^31. */
} fieldctrl_q15_pi_t;

/**
 * @brief Q15 rotating-field state with a constant F31 electrical phase.
 */
typedef struct {
    linears_fixed_t angle; /**< Electrical turns, int64 raw/2^31, in [0,1). */
} fieldctrl_q15_vvvf_t;

/**
 * @brief Q15 VVVF software commands at the newly advanced field angle.
 */
typedef struct {
    linears_q15_t angle; /**< Electrical turns in [0,1), with circular rounding. */
    fieldctrl_q15_ab_t voltage; /**< Alpha/beta voltage divided by caller Vbase. */
    fieldctrl_q15_abc_t duty; /**< High-side duty ratios; mathematical +1 maps to INT_MAX. */
    int saturated; /**< Nonzero if the requested amplitude exceeded Vdc/sqrt(3). */
} fieldctrl_q15_vvvf_output_t;

/**
 * @brief Reduce a Q15 electrical signal angle modulo one turn.
 * @param[in] angle Raw Q15 electrical turns, including the negative endpoint.
 * @param[out] output Non-NULL wrapped angle in [0,1), unchanged on failure.
 * @return linears_status_t OK, or ARGUMENT for NULL output.
 * @note Every raw input is valid. A calculated angle that rounds to one turn
 *       wraps to zero, unlike the nonperiodic +1 signal endpoint policy.
 */
linears_status_t fieldctrl_q15_wrap(linears_q15_t angle, linears_q15_t *output);

/**
 * @brief Compute Q15 sine/cosine using bounded integer CORDIC rotations.
 * @param[in] angle Raw electrical turns; every raw input is valid.
 * @param[out] sine Non-NULL Q15 sine; mathematical +1 is INT_MAX and -1 is INT_MIN.
 * @param[out] cosine Non-NULL Q15 cosine destination, disjoint from sine.
 * @return linears_status_t OK, ARGUMENT for NULL, or ALIAS for overlap.
 * @note Both outputs are unchanged on failure. Rotations are limited to 32;
 *       cardinal angles are exact internally before endpoint quantization.
 */
linears_status_t fieldctrl_q15_sincos(linears_q15_t angle, linears_q15_t *sine,
                                    linears_q15_t *cosine);

/**
 * @brief Obtain a Q15 vector angle with integer CORDIC vectoring.
 * @param[in] y Raw beta component in the same per-unit scale as x.
 * @param[in] x Raw alpha component.
 * @param[out] angle Non-NULL electrical turns in [0,1), with circular rounding.
 * @return linears_status_t OK, ARGUMENT for NULL, or DOMAIN for x=y=0.
 * @note At most 32 iterations; failure preserves angle. Q15 raw values have
 *       no negative zero and no nonfinite bit patterns.
 */
linears_status_t fieldctrl_q15_atan2(linears_q15_t y, linears_q15_t x, linears_q15_t *angle);

/**
 * @brief Apply the amplitude-invariant Q15 Clarke transform.
 * @param[in] phase Raw a,b,c currents/Ibase or voltages/Vbase.
 * @param[out] stationary Non-NULL alpha/beta output in the same per-unit scale.
 * @return linears_status_t OK, ARGUMENT for NULL, or RANGE if an output exceeds
 *         the true [-1,+1] signal domain. Failure preserves the output.
 * @note alpha=(2*a-b-c)/3, beta=(b-c)/sqrt(3); zero sequence is discarded.
 */
linears_status_t fieldctrl_q15_clarke(fieldctrl_q15_abc_t phase, fieldctrl_q15_ab_t *stationary);

/**
 * @brief Convert a Q15 stationary vector to balanced phase quantities.
 * @param[in] stationary Alpha/beta currents/Ibase or voltages/Vbase.
 * @param[out] phase Non-NULL phase output in the same per-unit scale.
 * @return linears_status_t OK, ARGUMENT for NULL, or RANGE outside the output
 *         [-1,+1] domain. Failure preserves phase; no clipping is performed.
 */
linears_status_t fieldctrl_q15_inverse_clarke(fieldctrl_q15_ab_t stationary, fieldctrl_q15_abc_t *phase);

/**
 * @brief Rotate a Q15 stationary vector into the rotor frame.
 * @param[in] stationary Alpha/beta input in a common per-unit scale.
 * @param[in] angle Electrical rotor turns, positive alpha toward beta.
 * @param[out] rotating Non-NULL d/q output in the same per-unit scale.
 * @return linears_status_t OK, ARGUMENT for NULL, or RANGE outside the output
 *         [-1,+1] domain. Failure preserves rotating; no clipping is performed.
 * @note d=alpha*cos+beta*sin, q=-alpha*sin+beta*cos.
 */
linears_status_t fieldctrl_q15_park(fieldctrl_q15_ab_t stationary, linears_q15_t angle,
                                   fieldctrl_q15_dq_t *rotating);

/**
 * @brief Rotate a Q15 rotor-frame vector into the stationary frame.
 * @param[in] rotating d/q quantities in a common per-unit scale.
 * @param[in] angle Electrical rotor turns, positive alpha toward beta.
 * @param[out] stationary Non-NULL alpha/beta output in the same scale.
 * @return linears_status_t OK, ARGUMENT for NULL, or RANGE outside the output
 *         [-1,+1] domain. Failure preserves stationary.
 */
linears_status_t fieldctrl_q15_inverse_park(fieldctrl_q15_dq_t rotating, linears_q15_t angle,
                                           fieldctrl_q15_ab_t *stationary);

/**
 * @brief Limit a Q15 voltage vector radially, retaining its direction.
 * @param[in] voltage Alpha/beta voltage divided by Vbase.
 * @param[in] limit Nonnegative peak magnitude divided by the same Vbase.
 * @param[out] limited Non-NULL result; zero limit yields zero.
 * @return linears_status_t OK including clipping, ARGUMENT for NULL, or DOMAIN
 *         for negative limit. Failure preserves limited.
 * @note Final signal quantization can perturb the radius by one output unit.
 */
linears_status_t fieldctrl_q15_limit_voltage(fieldctrl_q15_ab_t voltage, linears_q15_t limit,
                                            fieldctrl_q15_ab_t *limited);

/**
 * @brief Compute Q15 centered SVPWM duty ratios with circular voltage limiting.
 * @param[in] voltage Alpha/beta voltage divided by Vbase.
 * @param[in] dc_bus Strictly positive bus voltage divided by the same Vbase.
 * @param[out] duty Non-NULL three raw duties; mathematical full duty maps to INT_MAX.
 * @return linears_status_t OK including clipping, ARGUMENT for NULL, or DOMAIN
 *         for a nonpositive bus. Failure preserves duty.
 * @note Uses the radius dc_bus/sqrt(3) and max/min common-mode injection. Ratios
 *       are evaluated before final voltage quantization. No overmodulation,
 *       timer configuration, dead-time insertion or hardware action occurs.
 */
linears_status_t fieldctrl_q15_svpwm(fieldctrl_q15_ab_t voltage, linears_q15_t dc_bus,
                                    fieldctrl_q15_abc_t *duty);

/**
 * @brief Initialize a Q15 PI with explicitly wide F31 coefficients.
 * @param[out] ctx Non-NULL caller-owned context, unchanged on failure.
 * @param[in] kp Nonnegative int64 raw/2^31 gain, output-pu/error-pu.
 * @param[in] ki Nonnegative int64 raw/2^31 gain, output-pu/(error-pu*time-pu).
 * @param[in] lower Raw Q15 lower output/integral bound.
 * @param[in] upper Raw Q15 upper bound, strictly greater than lower.
 * @param[in] integral Initial Q15 output-pu contribution in [lower,upper].
 * @return linears_status_t OK, ARGUMENT for NULL, or DOMAIN for invalid bounds,
 *         initial value or negative gains. Failure preserves ctx.
 * @note Do not pass a Q15 raw coefficient as kp/ki: their binary point is F31.
 *       Fields are library-owned after init; serialize calls sharing ctx.
 */
linears_status_t fieldctrl_q15_pi_init(fieldctrl_q15_pi_t *ctx,
                                      linears_fixed_t kp, linears_fixed_t ki,
                                      linears_q15_t lower, linears_q15_t upper, linears_q15_t integral);

/**
 * @brief Advance a Q15 conditional-integration PI by one per-unit sample.
 * @param[in,out] ctx Non-NULL successfully initialized caller-owned context.
 * @param[in] error Reference minus feedback in the caller's error per-unit scale;
 *                  this public input must itself fit Q15.
 * @param[in] dt Strictly positive t/Tbase, represented as raw Q15; choose bases
 *               and a stable sample period so this value is nonzero and <1.
 * @param[out] output Non-NULL bounded raw command, disjoint from ctx.
 * @return linears_status_t OK including saturation, ARGUMENT for NULL, ALIAS
 *         for overlap, DOMAIN for dt<=0, or RANGE for int64 F31 work overflow.
 * @note Candidate I=clamp(I+ki*dt*error). Reject an increment that drives the
 *       unconstrained command further past a bound, then recompute/clamp.
 *       Integral state has constant F31 precision; sub-half-F31 increments can
 *       round away. There is no hidden remainder accumulator. Failure preserves
 *       context/output; independent contexts are reentrant.
 */
linears_status_t fieldctrl_q15_pi_step(fieldctrl_q15_pi_t *ctx, linears_q15_t error,
                                      linears_q15_t dt, linears_q15_t *output);

/**
 * @brief Initialize a Q15 generator at a selected electrical field angle.
 * @param[out] ctx Non-NULL caller-owned state, unchanged on failure.
 * @param[in] angle Raw electrical turns, reduced modulo one turn.
 * @return linears_status_t OK, or ARGUMENT for NULL ctx.
 * @note State is retained as int64 F31; do not edit it between calls.
 */
linears_status_t fieldctrl_q15_vvvf_init(fieldctrl_q15_vvvf_t *ctx, linears_q15_t angle);

/**
 * @brief Advance a Q15 open-loop variable-frequency/amplitude field.
 * @param[in,out] ctx Non-NULL initialized caller-owned state.
 * @param[in] frequency Electrical f*Tbase, raw Q15; negative reverses the field.
 * @param[in] amplitude Nonnegative peak voltage/Vbase, raw Q15.
 * @param[in] dc_bus Strictly positive bus voltage/Vbase, raw Q15.
 * @param[in] dt Strictly positive t/Tbase, raw Q15; require the working product
 *               abs(frequency)*dt <= 1/2 electrical turn to prevent aliasing.
 * @param[out] output Non-NULL result disjoint from ctx; no hardware action.
 * @return linears_status_t OK including amplitude clipping, ARGUMENT for NULL,
 *         ALIAS for overlap, or DOMAIN for invalid signs/sample advance.
 * @note All signal inputs must be scaled before encoding; e.g. 50 Hz with
 *       Tbase=0.005 s is frequency=0.25, not an unrepresentable Q value of 50.
 *       State is F31; output angles use circular nearest rounding to [0,1).
 *       Voltage is limited to dc_bus/sqrt(3). Both state/output stay unchanged
 *       on failure. Ramps, V/f laws and rotor synchronization remain caller policy.
 */
linears_status_t fieldctrl_q15_vvvf_step(fieldctrl_q15_vvvf_t *ctx,
                                        linears_q15_t frequency, linears_q15_t amplitude,
                                        linears_q15_t dc_bus, linears_q15_t dt,
                                        fieldctrl_q15_vvvf_output_t *output);

/**
 * @brief Q31 signal PI with wide int64 F31 gains and integral state.
 *
 * Signals are raw int32_t/2^31; gains and integral are int64_t/2^31.
 * Gains are not Q1 signals, even for this Q31 entry point. All physical
 * quantities use consistent caller-selected per-unit bases; time is t/Tbase.
 * State is library-owned after initialization, and shared calls are serialized.
 */
typedef struct {
    linears_fixed_t kp; /**< Nonnegative F31 output-pu/error-pu gain. */
    linears_fixed_t ki; /**< Nonnegative F31 output-pu/(error-pu*time-pu) gain. */
    linears_q31_t lower; /**< Inclusive Q1 output and integral lower bound. */
    linears_q31_t upper; /**< Inclusive Q1 upper bound, greater than lower. */
    linears_fixed_t integral; /**< Output-pu integral, int64 raw/2^31. */
} fieldctrl_q31_pi_t;

/**
 * @brief Q31 rotating-field state with a constant F31 electrical phase.
 */
typedef struct {
    linears_fixed_t angle; /**< Electrical turns, int64 raw/2^31, in [0,1). */
} fieldctrl_q31_vvvf_t;

/**
 * @brief Q31 VVVF software commands at the newly advanced field angle.
 */
typedef struct {
    linears_q31_t angle; /**< Electrical turns in [0,1), with circular rounding. */
    fieldctrl_q31_ab_t voltage; /**< Alpha/beta voltage divided by caller Vbase. */
    fieldctrl_q31_abc_t duty; /**< High-side duty ratios; mathematical +1 maps to INT_MAX. */
    int saturated; /**< Nonzero if the requested amplitude exceeded Vdc/sqrt(3). */
} fieldctrl_q31_vvvf_output_t;

/**
 * @brief Reduce a Q31 electrical signal angle modulo one turn.
 * @param[in] angle Raw Q31 electrical turns, including the negative endpoint.
 * @param[out] output Non-NULL wrapped angle in [0,1), unchanged on failure.
 * @return linears_status_t OK, or ARGUMENT for NULL output.
 * @note Every raw input is valid. A calculated angle that rounds to one turn
 *       wraps to zero, unlike the nonperiodic +1 signal endpoint policy.
 */
linears_status_t fieldctrl_q31_wrap(linears_q31_t angle, linears_q31_t *output);

/**
 * @brief Compute Q31 sine/cosine using bounded integer CORDIC rotations.
 * @param[in] angle Raw electrical turns; every raw input is valid.
 * @param[out] sine Non-NULL Q31 sine; mathematical +1 is INT_MAX and -1 is INT_MIN.
 * @param[out] cosine Non-NULL Q31 cosine destination, disjoint from sine.
 * @return linears_status_t OK, ARGUMENT for NULL, or ALIAS for overlap.
 * @note Both outputs are unchanged on failure. Rotations are limited to 32;
 *       cardinal angles are exact internally before endpoint quantization.
 */
linears_status_t fieldctrl_q31_sincos(linears_q31_t angle, linears_q31_t *sine,
                                    linears_q31_t *cosine);

/**
 * @brief Obtain a Q31 vector angle with integer CORDIC vectoring.
 * @param[in] y Raw beta component in the same per-unit scale as x.
 * @param[in] x Raw alpha component.
 * @param[out] angle Non-NULL electrical turns in [0,1), with circular rounding.
 * @return linears_status_t OK, ARGUMENT for NULL, or DOMAIN for x=y=0.
 * @note At most 32 iterations; failure preserves angle. Q31 raw values have
 *       no negative zero and no nonfinite bit patterns.
 */
linears_status_t fieldctrl_q31_atan2(linears_q31_t y, linears_q31_t x, linears_q31_t *angle);

/**
 * @brief Apply the amplitude-invariant Q31 Clarke transform.
 * @param[in] phase Raw a,b,c currents/Ibase or voltages/Vbase.
 * @param[out] stationary Non-NULL alpha/beta output in the same per-unit scale.
 * @return linears_status_t OK, ARGUMENT for NULL, or RANGE if an output exceeds
 *         the true [-1,+1] signal domain. Failure preserves the output.
 * @note alpha=(2*a-b-c)/3, beta=(b-c)/sqrt(3); zero sequence is discarded.
 */
linears_status_t fieldctrl_q31_clarke(fieldctrl_q31_abc_t phase, fieldctrl_q31_ab_t *stationary);

/**
 * @brief Convert a Q31 stationary vector to balanced phase quantities.
 * @param[in] stationary Alpha/beta currents/Ibase or voltages/Vbase.
 * @param[out] phase Non-NULL phase output in the same per-unit scale.
 * @return linears_status_t OK, ARGUMENT for NULL, or RANGE outside the output
 *         [-1,+1] domain. Failure preserves phase; no clipping is performed.
 */
linears_status_t fieldctrl_q31_inverse_clarke(fieldctrl_q31_ab_t stationary, fieldctrl_q31_abc_t *phase);

/**
 * @brief Rotate a Q31 stationary vector into the rotor frame.
 * @param[in] stationary Alpha/beta input in a common per-unit scale.
 * @param[in] angle Electrical rotor turns, positive alpha toward beta.
 * @param[out] rotating Non-NULL d/q output in the same per-unit scale.
 * @return linears_status_t OK, ARGUMENT for NULL, or RANGE outside the output
 *         [-1,+1] domain. Failure preserves rotating; no clipping is performed.
 * @note d=alpha*cos+beta*sin, q=-alpha*sin+beta*cos.
 */
linears_status_t fieldctrl_q31_park(fieldctrl_q31_ab_t stationary, linears_q31_t angle,
                                   fieldctrl_q31_dq_t *rotating);

/**
 * @brief Rotate a Q31 rotor-frame vector into the stationary frame.
 * @param[in] rotating d/q quantities in a common per-unit scale.
 * @param[in] angle Electrical rotor turns, positive alpha toward beta.
 * @param[out] stationary Non-NULL alpha/beta output in the same scale.
 * @return linears_status_t OK, ARGUMENT for NULL, or RANGE outside the output
 *         [-1,+1] domain. Failure preserves stationary.
 */
linears_status_t fieldctrl_q31_inverse_park(fieldctrl_q31_dq_t rotating, linears_q31_t angle,
                                           fieldctrl_q31_ab_t *stationary);

/**
 * @brief Limit a Q31 voltage vector radially, retaining its direction.
 * @param[in] voltage Alpha/beta voltage divided by Vbase.
 * @param[in] limit Nonnegative peak magnitude divided by the same Vbase.
 * @param[out] limited Non-NULL result; zero limit yields zero.
 * @return linears_status_t OK including clipping, ARGUMENT for NULL, or DOMAIN
 *         for negative limit. Failure preserves limited.
 * @note Final signal quantization can perturb the radius by one output unit.
 */
linears_status_t fieldctrl_q31_limit_voltage(fieldctrl_q31_ab_t voltage, linears_q31_t limit,
                                            fieldctrl_q31_ab_t *limited);

/**
 * @brief Compute Q31 centered SVPWM duty ratios with circular voltage limiting.
 * @param[in] voltage Alpha/beta voltage divided by Vbase.
 * @param[in] dc_bus Strictly positive bus voltage divided by the same Vbase.
 * @param[out] duty Non-NULL three raw duties; mathematical full duty maps to INT_MAX.
 * @return linears_status_t OK including clipping, ARGUMENT for NULL, or DOMAIN
 *         for a nonpositive bus. Failure preserves duty.
 * @note Uses the radius dc_bus/sqrt(3) and max/min common-mode injection. Ratios
 *       are evaluated before final voltage quantization. No overmodulation,
 *       timer configuration, dead-time insertion or hardware action occurs.
 */
linears_status_t fieldctrl_q31_svpwm(fieldctrl_q31_ab_t voltage, linears_q31_t dc_bus,
                                    fieldctrl_q31_abc_t *duty);

/**
 * @brief Initialize a Q31 PI with explicitly wide F31 coefficients.
 * @param[out] ctx Non-NULL caller-owned context, unchanged on failure.
 * @param[in] kp Nonnegative int64 raw/2^31 gain, output-pu/error-pu.
 * @param[in] ki Nonnegative int64 raw/2^31 gain, output-pu/(error-pu*time-pu).
 * @param[in] lower Raw Q31 lower output/integral bound.
 * @param[in] upper Raw Q31 upper bound, strictly greater than lower.
 * @param[in] integral Initial Q31 output-pu contribution in [lower,upper].
 * @return linears_status_t OK, ARGUMENT for NULL, or DOMAIN for invalid bounds,
 *         initial value or negative gains. Failure preserves ctx.
 * @note Do not pass a Q31 raw coefficient as kp/ki: their binary point is F31.
 *       Fields are library-owned after init; serialize calls sharing ctx.
 */
linears_status_t fieldctrl_q31_pi_init(fieldctrl_q31_pi_t *ctx,
                                      linears_fixed_t kp, linears_fixed_t ki,
                                      linears_q31_t lower, linears_q31_t upper, linears_q31_t integral);

/**
 * @brief Advance a Q31 conditional-integration PI by one per-unit sample.
 * @param[in,out] ctx Non-NULL successfully initialized caller-owned context.
 * @param[in] error Reference minus feedback in the caller's error per-unit scale;
 *                  this public input must itself fit Q31.
 * @param[in] dt Strictly positive t/Tbase, represented as raw Q31; choose bases
 *               and a stable sample period so this value is nonzero and <1.
 * @param[out] output Non-NULL bounded raw command, disjoint from ctx.
 * @return linears_status_t OK including saturation, ARGUMENT for NULL, ALIAS
 *         for overlap, DOMAIN for dt<=0, or RANGE for int64 F31 work overflow.
 * @note Candidate I=clamp(I+ki*dt*error). Reject an increment that drives the
 *       unconstrained command further past a bound, then recompute/clamp.
 *       Integral state has constant F31 precision; sub-half-F31 increments can
 *       round away. There is no hidden remainder accumulator. Failure preserves
 *       context/output; independent contexts are reentrant.
 */
linears_status_t fieldctrl_q31_pi_step(fieldctrl_q31_pi_t *ctx, linears_q31_t error,
                                      linears_q31_t dt, linears_q31_t *output);

/**
 * @brief Initialize a Q31 generator at a selected electrical field angle.
 * @param[out] ctx Non-NULL caller-owned state, unchanged on failure.
 * @param[in] angle Raw electrical turns, reduced modulo one turn.
 * @return linears_status_t OK, or ARGUMENT for NULL ctx.
 * @note State is retained as int64 F31; do not edit it between calls.
 */
linears_status_t fieldctrl_q31_vvvf_init(fieldctrl_q31_vvvf_t *ctx, linears_q31_t angle);

/**
 * @brief Advance a Q31 open-loop variable-frequency/amplitude field.
 * @param[in,out] ctx Non-NULL initialized caller-owned state.
 * @param[in] frequency Electrical f*Tbase, raw Q31; negative reverses the field.
 * @param[in] amplitude Nonnegative peak voltage/Vbase, raw Q31.
 * @param[in] dc_bus Strictly positive bus voltage/Vbase, raw Q31.
 * @param[in] dt Strictly positive t/Tbase, raw Q31; require the working product
 *               abs(frequency)*dt <= 1/2 electrical turn to prevent aliasing.
 * @param[out] output Non-NULL result disjoint from ctx; no hardware action.
 * @return linears_status_t OK including amplitude clipping, ARGUMENT for NULL,
 *         ALIAS for overlap, or DOMAIN for invalid signs/sample advance.
 * @note All signal inputs must be scaled before encoding; e.g. 50 Hz with
 *       Tbase=0.005 s is frequency=0.25, not an unrepresentable Q value of 50.
 *       State is F31; output angles use circular nearest rounding to [0,1).
 *       Voltage is limited to dc_bus/sqrt(3). Both state/output stay unchanged
 *       on failure. Ramps, V/f laws and rotor synchronization remain caller policy.
 */
linears_status_t fieldctrl_q31_vvvf_step(fieldctrl_q31_vvvf_t *ctx,
                                        linears_q31_t frequency, linears_q31_t amplitude,
                                        linears_q31_t dc_bus, linears_q31_t dt,
                                        fieldctrl_q31_vvvf_output_t *output);

#ifdef __cplusplus
}
#endif
#endif
