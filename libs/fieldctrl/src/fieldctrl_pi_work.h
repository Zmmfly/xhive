/* Internal PI entry points shared by the public signal API and speed FOC.
 * Their error is a working value, so subtracting two valid Q1 signals does
 * not impose an extra Q1 range limit before the PI's actuator saturation. */
#ifndef FIELDCTRL_PI_WORK_H
#define FIELDCTRL_PI_WORK_H

#include "fieldctrl_base.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Advance initialized f32 PI using native working inputs.
 * @param[in,out] ctx Valid initialized private candidate context.
 * @param[in] error Finite error in the controller's error units.
 * @param[in] dt Positive finite sample period in seconds.
 * @param[out] output Valid destination disjoint from ctx, in output units.
 * @return linears_status_t OK, DOMAIN for dt<=0, or RANGE for arithmetic overflow.
 * @note Caller validates storage and inputs; failures preserve ctx/output.
 */
linears_status_t fieldctrl_f32_pi_step_work(fieldctrl_f32_pi_t *ctx,
    float error, float dt, float *output);

/**
 * @brief Advance initialized f64 PI using native working inputs.
 * @param[in,out] ctx Valid initialized private candidate context.
 * @param[in] error Finite error in the controller's error units.
 * @param[in] dt Positive finite sample period in seconds.
 * @param[out] output Valid destination disjoint from ctx, in output units.
 * @return linears_status_t OK, DOMAIN for dt<=0, or RANGE for arithmetic overflow.
 * @note Caller validates storage and inputs; failures preserve ctx/output.
 */
linears_status_t fieldctrl_f64_pi_step_work(fieldctrl_f64_pi_t *ctx,
    double error, double dt, double *output);

/**
 * @brief Advance a Q15-signal PI using int64 F31 per-unit inputs.
 * @param[in,out] ctx Valid initialized private candidate context.
 * @param[in] error Wide F31 error, allowed beyond the Q1 signal interval.
 * @param[in] dt Positive F31 t/Tbase; this entry does not narrow it to Q15.
 * @param[out] output Valid F31 bounded command destination, disjoint from ctx.
 * @return linears_status_t OK, DOMAIN for dt<=0, or RANGE for work overflow.
 * @note Storage validation belongs to the caller. Failure preserves ctx/output.
 */
linears_status_t fieldctrl_q15_pi_step_work(fieldctrl_q15_pi_t *ctx,
    linears_fixed_t error, linears_fixed_t dt, linears_fixed_t *output);

/**
 * @brief Advance a Q31-signal PI using int64 F31 per-unit inputs.
 * @param[in,out] ctx Valid initialized private candidate context.
 * @param[in] error Wide F31 error, allowed beyond the Q1 signal interval.
 * @param[in] dt Positive F31 t/Tbase.
 * @param[out] output Valid F31 bounded command destination, disjoint from ctx.
 * @return linears_status_t OK, DOMAIN for dt<=0, or RANGE for work overflow.
 * @note Storage validation belongs to the caller. Failure preserves ctx/output.
 */
linears_status_t fieldctrl_q31_pi_step_work(fieldctrl_q31_pi_t *ctx,
    linears_fixed_t error, linears_fixed_t dt, linears_fixed_t *output);

#ifdef __cplusplus
}
#endif
#endif
