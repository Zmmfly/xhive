/* Internal PID entry points shared by the public signal API and speed FOC.
 * Setpoint and measurement are working values, so the speed cascade passes its
 * unrounded mechanical speed and the error is never narrowed to Q1 before the
 * PID's actuator saturation. */
#ifndef FIELDCTRL_PID_WORK_H
#define FIELDCTRL_PID_WORK_H

#include "fieldctrl_base.h"

/* Every option bit accepted by fieldctrl_S_pid_init(). */
#define FIELDCTRL_PID_OPTIONS (FIELDCTRL_PID_OPT_INTEGRAL_LIMIT | \
    FIELDCTRL_PID_OPT_INTEGRAL_SEPARATION | FIELDCTRL_PID_OPT_SETPOINT_WEIGHT)

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Advance initialized f32 PID using native working inputs.
 * @param[in,out] ctx Valid initialized context.
 * @param[in] setpoint Finite reference in the controller's error units.
 * @param[in] measurement Finite feedback in the same units.
 * @param[in] dt Finite sample period in seconds.
 * @param[out] output Valid destination disjoint from ctx.
 * @return linears_status_t OK, DOMAIN for dt<=0, or RANGE for arithmetic overflow.
 * @note Caller validates storage and inputs; failures preserve ctx/output.
 */
linears_status_t fieldctrl_f32_pid_step_work(fieldctrl_f32_pid_t *ctx,
    float setpoint, float measurement, float dt, fieldctrl_f32_pid_output_t *output);

/**
 * @brief Advance initialized f64 PID using native working inputs.
 * @param[in,out] ctx Valid initialized context.
 * @param[in] setpoint Finite reference in the controller's error units.
 * @param[in] measurement Finite feedback in the same units.
 * @param[in] dt Finite sample period in seconds.
 * @param[out] output Valid destination disjoint from ctx.
 * @return linears_status_t OK, DOMAIN for dt<=0, or RANGE for arithmetic overflow.
 * @note Caller validates storage and inputs; failures preserve ctx/output.
 */
linears_status_t fieldctrl_f64_pid_step_work(fieldctrl_f64_pid_t *ctx,
    double setpoint, double measurement, double dt, fieldctrl_f64_pid_output_t *output);

/**
 * @brief Advance a Q15-signal PID using int64 F31 per-unit inputs.
 * @param[in,out] ctx Valid initialized context.
 * @param[in] setpoint Wide F31 reference.
 * @param[in] measurement Wide F31 feedback; the difference may leave Q1.
 * @param[in] dt F31 t/Tbase; this entry does not narrow it to Q15.
 * @param[out] output Valid destination disjoint from ctx.
 * @return linears_status_t OK, DOMAIN for dt<=0, or RANGE for work overflow.
 * @note Storage validation belongs to the caller. Failure preserves ctx/output.
 */
linears_status_t fieldctrl_q15_pid_step_work(fieldctrl_q15_pid_t *ctx,
    linears_fixed_t setpoint, linears_fixed_t measurement, linears_fixed_t dt,
    fieldctrl_q15_pid_output_t *output);

/**
 * @brief Advance a Q31-signal PID using int64 F31 per-unit inputs.
 * @param[in,out] ctx Valid initialized context.
 * @param[in] setpoint Wide F31 reference.
 * @param[in] measurement Wide F31 feedback; the difference may leave Q1.
 * @param[in] dt F31 t/Tbase.
 * @param[out] output Valid destination disjoint from ctx.
 * @return linears_status_t OK, DOMAIN for dt<=0, or RANGE for work overflow.
 * @note Storage validation belongs to the caller. Failure preserves ctx/output.
 */
linears_status_t fieldctrl_q31_pid_step_work(fieldctrl_q31_pid_t *ctx,
    linears_fixed_t setpoint, linears_fixed_t measurement, linears_fixed_t dt,
    fieldctrl_q31_pid_output_t *output);

#ifdef __cplusplus
}
#endif
#endif
