/**
 * @file fieldctrl_foc.h
 * @brief Current and cascaded speed FOC for balanced, isotropic PMSMs.
 *
 * Current and voltage vectors use amperes and volts. Angles are electrical
 * turns in [0,1), positive from alpha toward beta. No decoupling, field
 * weakening, startup sequencing, inverter dead-time or hardware protection is
 * performed. A step returns a software duty command, not completed switching.
 * All scalar inputs must be finite (floating APIs) or have valid linears binary
 * exponents (fixed APIs). Q15/Q31 paths perform pure integer arithmetic.
 *
 * Initialize each caller-owned context before use and do not edit its fields.
 * Serialize calls sharing a context, including interrupt calls; distinct
 * contexts are reentrant. No allocation, I/O, MMIO, locks or global state.
 * Every error preserves the context and output. Output storage must not
 * overlap its context. Configurations are copied during initialization.
 */
#ifndef FIELDCTRL_FOC_H
#define FIELDCTRL_FOC_H

#include "fieldctrl_types.h"
#include "fieldctrl_base.h"
#include "linears_numeric.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief single-precision d/q current-controller gains and radial voltage ceiling.
 */
typedef struct {
    float kp_d; /**< Nonnegative d-axis proportional gain, V/A. */
    float ki_d; /**< Nonnegative d-axis integral gain, V/(A s). */
    float kp_q; /**< Nonnegative q-axis proportional gain, V/A. */
    float ki_q; /**< Nonnegative q-axis integral gain, V/(A s). */
    float voltage_limit; /**< Positive alpha/beta vector magnitude ceiling, V. */
} fieldctrl_f32_foc_config_t;

/**
 * @brief single-precision caller-owned current-controller state.
 */
typedef struct {
    fieldctrl_f32_foc_config_t config; /**< Immutable copied tuning. */
    fieldctrl_f32_dq_t integral; /**< d/q integral contributions, V. */
} fieldctrl_f32_foc_t;

/**
 * @brief single-precision current-loop result for one successful control sample.
 */
typedef struct {
    fieldctrl_f32_dq_t current; /**< Measured rotor-frame current, A. */
    fieldctrl_f32_dq_t requested_voltage; /**< PI voltage before radial limiting, V. */
    fieldctrl_f32_dq_t voltage; /**< Limited rotor-frame voltage, V. */
    fieldctrl_f32_ab_t stationary_voltage; /**< Limited stator-frame voltage, V. */
    fieldctrl_f32_abc_t duty; /**< Centered SVPWM duty ratios, each in [0,1]. */
    int saturated; /**< Nonzero when the final voltage request was limited. */
} fieldctrl_f32_foc_output_t;

/**
 * @brief Initialize a single-precision current loop with zero integral state.
 *
 * @param[out] ctx Non-NULL caller-owned context; no prior initialization needed.
 * @param[in] config Non-NULL tuning copied by value; gains must be nonnegative
 *                   and voltage_limit must be strictly positive, in SI units.
 *
 * @return linears_status_t Initialization status.
 * @retval LINEARS_OK Context initialized.
 * @retval LINEARS_ERROR_ARGUMENT ctx or config is NULL.
 * @retval LINEARS_ERROR_DOMAIN A gain is negative or the voltage limit is nonpositive.
 * @retval LINEARS_ERROR_RANGE A scalar is nonfinite or its exponent is invalid.
 * @note On error ctx is unchanged. Serialize access to ctx.
 */
linears_status_t fieldctrl_f32_foc_init(fieldctrl_f32_foc_t *ctx,
    const fieldctrl_f32_foc_config_t *config);

/**
 * @brief Run single-precision d/q PI control, radial anti-windup and centered SVPWM.
 *
 * The voltage radius is min(config.voltage_limit, dc_bus/sqrt(3)). Both
 * integral increments are rejected together if their dot product with the
 * radial saturation residual is positive; the voltage is then recomputed
 * using the prior integral. There is no independent d/q voltage clipping.
 * Final scalar rounding can perturb the voltage-radius bound; SVPWM performs
 * its own bus-limit check at output precision.
 *
 * @param[in,out] ctx Non-NULL initialized context; retained by the caller.
 * @param[in] current Simultaneous measured alpha/beta currents, A.
 * @param[in] reference Requested d/q currents, A.
 * @param[in] electrical_angle Rotor flux angle, electrical turns in [0,1).
 * @param[in] dc_bus Positive measured DC-link voltage, V.
 * @param[in] dt Positive elapsed control period, seconds.
 * @param[out] output Non-NULL result storage, disjoint from ctx.
 *
 * @return linears_status_t Control-sample status.
 * @retval LINEARS_OK State and software voltage/duty command committed.
 * @retval LINEARS_ERROR_ARGUMENT ctx or output is NULL.
 * @retval LINEARS_ERROR_ALIAS output overlaps ctx.
 * @retval LINEARS_ERROR_DOMAIN dt/DC bus is nonpositive or angle is outside [0,1).
 * @retval LINEARS_ERROR_RANGE An input/intermediate/output is nonfinite or out of exponent range.
 * @pre Initialize ctx successfully and provide coherent current/angle samples.
 * @note Nonblocking; errors preserve state and output. Serialize use of ctx.
 * @warning Duty limits are not a substitute for board-level current/voltage protection.
 */
linears_status_t fieldctrl_f32_foc_step(fieldctrl_f32_foc_t *ctx,
    fieldctrl_f32_ab_t current, fieldctrl_f32_dq_t reference,
    float electrical_angle, float dc_bus, float dt,
    fieldctrl_f32_foc_output_t *output);

/**
 * @brief single-precision speed/current cascade tuning for zero d-axis reference.
 */
typedef struct {
    fieldctrl_f32_foc_config_t current; /**< Inner current-loop tuning. */
    float kp; /**< Nonnegative mechanical-speed proportional gain, A/(turn/s). */
    float ki; /**< Nonnegative mechanical-speed integral gain, A/turn. */
    float current_limit; /**< Positive magnitude limit of the q-axis reference, A. */
    uint32_t pole_pairs; /**< Nonzero motor pole-pair count. */
} fieldctrl_f32_speed_foc_config_t;

/**
 * @brief single-precision caller-owned cascaded speed/current controller.
 */
typedef struct {
    fieldctrl_f32_foc_t current; /**< Inner current-controller state. */
    fieldctrl_f32_pi_t speed; /**< Mechanical-speed PI state; output in A. */
    uint32_t pole_pairs; /**< Immutable electrical/mechanical frequency ratio. */
} fieldctrl_f32_speed_foc_t;

/**
 * @brief single-precision cascaded-loop result, including the speed-unit conversion.
 */
typedef struct {
    fieldctrl_f32_foc_output_t current; /**< Inner-loop result. */
    fieldctrl_f32_dq_t reference; /**< Generated current reference, A; d is zero. */
    float mechanical_speed; /**< Electrical feedback divided by pole_pairs, turn/s. */
} fieldctrl_f32_speed_foc_output_t;

/**
 * @brief Initialize a single-precision mechanical-speed/current cascade.
 *
 * @param[out] ctx Non-NULL caller-owned context; no prior initialization needed.
 * @param[in] config Non-NULL copied tuning. Gains are nonnegative, current and
 *                   voltage limits positive, pole_pairs nonzero.
 *
 * @return linears_status_t Initialization status.
 * @retval LINEARS_OK Both integral states initialized to zero.
 * @retval LINEARS_ERROR_ARGUMENT ctx or config is NULL.
 * @retval LINEARS_ERROR_DOMAIN A gain/limit/pole-pair count violates its range.
 * @retval LINEARS_ERROR_RANGE A scalar is nonfinite or out of exponent range.
 * @note Error leaves ctx unchanged; serialize access to ctx.
 */
linears_status_t fieldctrl_f32_speed_foc_init(fieldctrl_f32_speed_foc_t *ctx,
    const fieldctrl_f32_speed_foc_config_t *config);

/**
 * @brief Run a single-precision speed PI followed by the d/q current loop.
 *
 * Current-reference clipping uses conditional speed-PI anti-windup. Inner
 * radial voltage saturation also cancels this sample's speed-integral update
 * if speed error and q-voltage saturation residual have the same sign. The
 * current sample's q reference is retained; the restored integral affects the
 * next sample. The speed and current loops share dt; no implicit decimation.
 *
 * @param[in,out] ctx Non-NULL successfully initialized caller-owned context.
 * @param[in] current Measured alpha/beta currents, A.
 * @param[in] mechanical_reference Signed mechanical speed request, turns/s.
 * @param[in] electrical_speed Signed electrical speed feedback, turns/s.
 * @param[in] electrical_angle Rotor flux angle, electrical turns in [0,1).
 * @param[in] dc_bus Positive measured DC-link voltage, V.
 * @param[in] dt Positive common sample period, seconds.
 * @param[out] output Non-NULL result storage, disjoint from ctx.
 *
 * @return linears_status_t Cascaded-sample status.
 * @retval LINEARS_OK Both controller states and output committed.
 * @retval LINEARS_ERROR_ARGUMENT ctx or output is NULL.
 * @retval LINEARS_ERROR_ALIAS output overlaps ctx.
 * @retval LINEARS_ERROR_DOMAIN dt/DC bus is nonpositive or angle is outside [0,1).
 * @retval LINEARS_ERROR_RANGE Numeric input/intermediate/output is out of range.
 * @note Nonblocking; errors preserve both loops and output. Serialize ctx access.
 * @warning This API neither starts a sensorless motor nor validates a rotor observer.
 */
linears_status_t fieldctrl_f32_speed_foc_step(fieldctrl_f32_speed_foc_t *ctx,
    fieldctrl_f32_ab_t current, float mechanical_reference,
    float electrical_speed, float electrical_angle, float dc_bus, float dt,
    fieldctrl_f32_speed_foc_output_t *output);

/**
 * @brief double-precision d/q current-controller gains and radial voltage ceiling.
 */
typedef struct {
    double kp_d; /**< Nonnegative d-axis proportional gain, V/A. */
    double ki_d; /**< Nonnegative d-axis integral gain, V/(A s). */
    double kp_q; /**< Nonnegative q-axis proportional gain, V/A. */
    double ki_q; /**< Nonnegative q-axis integral gain, V/(A s). */
    double voltage_limit; /**< Positive alpha/beta vector magnitude ceiling, V. */
} fieldctrl_f64_foc_config_t;

/**
 * @brief double-precision caller-owned current-controller state.
 */
typedef struct {
    fieldctrl_f64_foc_config_t config; /**< Immutable copied tuning. */
    fieldctrl_f64_dq_t integral; /**< d/q integral contributions, V. */
} fieldctrl_f64_foc_t;

/**
 * @brief double-precision current-loop result for one successful control sample.
 */
typedef struct {
    fieldctrl_f64_dq_t current; /**< Measured rotor-frame current, A. */
    fieldctrl_f64_dq_t requested_voltage; /**< PI voltage before radial limiting, V. */
    fieldctrl_f64_dq_t voltage; /**< Limited rotor-frame voltage, V. */
    fieldctrl_f64_ab_t stationary_voltage; /**< Limited stator-frame voltage, V. */
    fieldctrl_f64_abc_t duty; /**< Centered SVPWM duty ratios, each in [0,1]. */
    int saturated; /**< Nonzero when the final voltage request was limited. */
} fieldctrl_f64_foc_output_t;

/**
 * @brief Initialize a double-precision current loop with zero integral state.
 *
 * @param[out] ctx Non-NULL caller-owned context; no prior initialization needed.
 * @param[in] config Non-NULL tuning copied by value; gains must be nonnegative
 *                   and voltage_limit must be strictly positive, in SI units.
 *
 * @return linears_status_t Initialization status.
 * @retval LINEARS_OK Context initialized.
 * @retval LINEARS_ERROR_ARGUMENT ctx or config is NULL.
 * @retval LINEARS_ERROR_DOMAIN A gain is negative or the voltage limit is nonpositive.
 * @retval LINEARS_ERROR_RANGE A scalar is nonfinite or its exponent is invalid.
 * @note On error ctx is unchanged. Serialize access to ctx.
 */
linears_status_t fieldctrl_f64_foc_init(fieldctrl_f64_foc_t *ctx,
    const fieldctrl_f64_foc_config_t *config);

/**
 * @brief Run double-precision d/q PI control, radial anti-windup and centered SVPWM.
 *
 * The voltage radius is min(config.voltage_limit, dc_bus/sqrt(3)). Both
 * integral increments are rejected together if their dot product with the
 * radial saturation residual is positive; the voltage is then recomputed
 * using the prior integral. There is no independent d/q voltage clipping.
 * Final scalar rounding can perturb the voltage-radius bound; SVPWM performs
 * its own bus-limit check at output precision.
 *
 * @param[in,out] ctx Non-NULL initialized context; retained by the caller.
 * @param[in] current Simultaneous measured alpha/beta currents, A.
 * @param[in] reference Requested d/q currents, A.
 * @param[in] electrical_angle Rotor flux angle, electrical turns in [0,1).
 * @param[in] dc_bus Positive measured DC-link voltage, V.
 * @param[in] dt Positive elapsed control period, seconds.
 * @param[out] output Non-NULL result storage, disjoint from ctx.
 *
 * @return linears_status_t Control-sample status.
 * @retval LINEARS_OK State and software voltage/duty command committed.
 * @retval LINEARS_ERROR_ARGUMENT ctx or output is NULL.
 * @retval LINEARS_ERROR_ALIAS output overlaps ctx.
 * @retval LINEARS_ERROR_DOMAIN dt/DC bus is nonpositive or angle is outside [0,1).
 * @retval LINEARS_ERROR_RANGE An input/intermediate/output is nonfinite or out of exponent range.
 * @pre Initialize ctx successfully and provide coherent current/angle samples.
 * @note Nonblocking; errors preserve state and output. Serialize use of ctx.
 * @warning Duty limits are not a substitute for board-level current/voltage protection.
 */
linears_status_t fieldctrl_f64_foc_step(fieldctrl_f64_foc_t *ctx,
    fieldctrl_f64_ab_t current, fieldctrl_f64_dq_t reference,
    double electrical_angle, double dc_bus, double dt,
    fieldctrl_f64_foc_output_t *output);

/**
 * @brief double-precision speed/current cascade tuning for zero d-axis reference.
 */
typedef struct {
    fieldctrl_f64_foc_config_t current; /**< Inner current-loop tuning. */
    double kp; /**< Nonnegative mechanical-speed proportional gain, A/(turn/s). */
    double ki; /**< Nonnegative mechanical-speed integral gain, A/turn. */
    double current_limit; /**< Positive magnitude limit of the q-axis reference, A. */
    uint32_t pole_pairs; /**< Nonzero motor pole-pair count. */
} fieldctrl_f64_speed_foc_config_t;

/**
 * @brief double-precision caller-owned cascaded speed/current controller.
 */
typedef struct {
    fieldctrl_f64_foc_t current; /**< Inner current-controller state. */
    fieldctrl_f64_pi_t speed; /**< Mechanical-speed PI state; output in A. */
    uint32_t pole_pairs; /**< Immutable electrical/mechanical frequency ratio. */
} fieldctrl_f64_speed_foc_t;

/**
 * @brief double-precision cascaded-loop result, including the speed-unit conversion.
 */
typedef struct {
    fieldctrl_f64_foc_output_t current; /**< Inner-loop result. */
    fieldctrl_f64_dq_t reference; /**< Generated current reference, A; d is zero. */
    double mechanical_speed; /**< Electrical feedback divided by pole_pairs, turn/s. */
} fieldctrl_f64_speed_foc_output_t;

/**
 * @brief Initialize a double-precision mechanical-speed/current cascade.
 *
 * @param[out] ctx Non-NULL caller-owned context; no prior initialization needed.
 * @param[in] config Non-NULL copied tuning. Gains are nonnegative, current and
 *                   voltage limits positive, pole_pairs nonzero.
 *
 * @return linears_status_t Initialization status.
 * @retval LINEARS_OK Both integral states initialized to zero.
 * @retval LINEARS_ERROR_ARGUMENT ctx or config is NULL.
 * @retval LINEARS_ERROR_DOMAIN A gain/limit/pole-pair count violates its range.
 * @retval LINEARS_ERROR_RANGE A scalar is nonfinite or out of exponent range.
 * @note Error leaves ctx unchanged; serialize access to ctx.
 */
linears_status_t fieldctrl_f64_speed_foc_init(fieldctrl_f64_speed_foc_t *ctx,
    const fieldctrl_f64_speed_foc_config_t *config);

/**
 * @brief Run a double-precision speed PI followed by the d/q current loop.
 *
 * Current-reference clipping uses conditional speed-PI anti-windup. Inner
 * radial voltage saturation also cancels this sample's speed-integral update
 * if speed error and q-voltage saturation residual have the same sign. The
 * current sample's q reference is retained; the restored integral affects the
 * next sample. The speed and current loops share dt; no implicit decimation.
 *
 * @param[in,out] ctx Non-NULL successfully initialized caller-owned context.
 * @param[in] current Measured alpha/beta currents, A.
 * @param[in] mechanical_reference Signed mechanical speed request, turns/s.
 * @param[in] electrical_speed Signed electrical speed feedback, turns/s.
 * @param[in] electrical_angle Rotor flux angle, electrical turns in [0,1).
 * @param[in] dc_bus Positive measured DC-link voltage, V.
 * @param[in] dt Positive common sample period, seconds.
 * @param[out] output Non-NULL result storage, disjoint from ctx.
 *
 * @return linears_status_t Cascaded-sample status.
 * @retval LINEARS_OK Both controller states and output committed.
 * @retval LINEARS_ERROR_ARGUMENT ctx or output is NULL.
 * @retval LINEARS_ERROR_ALIAS output overlaps ctx.
 * @retval LINEARS_ERROR_DOMAIN dt/DC bus is nonpositive or angle is outside [0,1).
 * @retval LINEARS_ERROR_RANGE Numeric input/intermediate/output is out of range.
 * @note Nonblocking; errors preserve both loops and output. Serialize ctx access.
 * @warning This API neither starts a sensorless motor nor validates a rotor observer.
 */
linears_status_t fieldctrl_f64_speed_foc_step(fieldctrl_f64_speed_foc_t *ctx,
    fieldctrl_f64_ab_t current, double mechanical_reference,
    double electrical_speed, double electrical_angle, double dc_bus, double dt,
    fieldctrl_f64_speed_foc_output_t *output);

/**
 * @brief Q15 d/q current-controller gains and radial voltage ceiling.
 */
typedef struct {
    linears_q15_t kp_d; /**< Nonnegative d-axis proportional gain, V/A. */
    linears_q15_t ki_d; /**< Nonnegative d-axis integral gain, V/(A s). */
    linears_q15_t kp_q; /**< Nonnegative q-axis proportional gain, V/A. */
    linears_q15_t ki_q; /**< Nonnegative q-axis integral gain, V/(A s). */
    linears_q15_t voltage_limit; /**< Positive alpha/beta vector magnitude ceiling, V. */
} fieldctrl_q15_foc_config_t;

/**
 * @brief Q15 caller-owned current-controller state.
 */
typedef struct {
    fieldctrl_q15_foc_config_t config; /**< Immutable copied tuning. */
    struct {
        linears_fixed_t d; /**< Q31 working d-axis integral, V. */
        linears_fixed_t q; /**< Q31 working q-axis integral, V. */
    } integral; /**< Wide state preserves sub-Q15 integral increments. */
} fieldctrl_q15_foc_t;

/**
 * @brief Q15 current-loop result for one successful control sample.
 */
typedef struct {
    fieldctrl_q15_dq_t current; /**< Measured rotor-frame current, A. */
    fieldctrl_q15_dq_t requested_voltage; /**< PI voltage before radial limiting, V. */
    fieldctrl_q15_dq_t voltage; /**< Limited rotor-frame voltage, V. */
    fieldctrl_q15_ab_t stationary_voltage; /**< Limited stator-frame voltage, V. */
    fieldctrl_q15_abc_t duty; /**< Centered SVPWM duty ratios, each in [0,1]. */
    int saturated; /**< Nonzero when the final voltage request was limited. */
} fieldctrl_q15_foc_output_t;

/**
 * @brief Initialize a Q15 current loop with zero integral state.
 *
 * @param[out] ctx Non-NULL caller-owned context; no prior initialization needed.
 * @param[in] config Non-NULL tuning copied by value; gains must be nonnegative
 *                   and voltage_limit must be strictly positive, in SI units.
 *
 * @return linears_status_t Initialization status.
 * @retval LINEARS_OK Context initialized.
 * @retval LINEARS_ERROR_ARGUMENT ctx or config is NULL.
 * @retval LINEARS_ERROR_DOMAIN A gain is negative or the voltage limit is nonpositive.
 * @retval LINEARS_ERROR_RANGE A scalar is nonfinite or its exponent is invalid.
 * @note On error ctx is unchanged. Serialize access to ctx.
 */
linears_status_t fieldctrl_q15_foc_init(fieldctrl_q15_foc_t *ctx,
    const fieldctrl_q15_foc_config_t *config);

/**
 * @brief Run Q15 d/q PI control, radial anti-windup and centered SVPWM.
 *
 * The voltage radius is min(config.voltage_limit, dc_bus/sqrt(3)). Both
 * integral increments are rejected together if their dot product with the
 * radial saturation residual is positive; the voltage is then recomputed
 * using the prior integral. There is no independent d/q voltage clipping.
 * Final scalar rounding can perturb the voltage-radius bound; SVPWM performs
 * its own bus-limit check at output precision.
 *
 * @param[in,out] ctx Non-NULL initialized context; retained by the caller.
 * @param[in] current Simultaneous measured alpha/beta currents, A.
 * @param[in] reference Requested d/q currents, A.
 * @param[in] electrical_angle Rotor flux angle, electrical turns in [0,1).
 * @param[in] dc_bus Positive measured DC-link voltage, V.
 * @param[in] dt Positive elapsed control period, seconds.
 * @param[out] output Non-NULL result storage, disjoint from ctx.
 *
 * @return linears_status_t Control-sample status.
 * @retval LINEARS_OK State and software voltage/duty command committed.
 * @retval LINEARS_ERROR_ARGUMENT ctx or output is NULL.
 * @retval LINEARS_ERROR_ALIAS output overlaps ctx.
 * @retval LINEARS_ERROR_DOMAIN dt/DC bus is nonpositive or angle is outside [0,1).
 * @retval LINEARS_ERROR_RANGE An input/intermediate/output is nonfinite or out of exponent range.
 * @pre Initialize ctx successfully and provide coherent current/angle samples.
 * @note Nonblocking; errors preserve state and output. Serialize use of ctx.
 * @warning Duty limits are not a substitute for board-level current/voltage protection.
 */
linears_status_t fieldctrl_q15_foc_step(fieldctrl_q15_foc_t *ctx,
    fieldctrl_q15_ab_t current, fieldctrl_q15_dq_t reference,
    linears_q15_t electrical_angle, linears_q15_t dc_bus, linears_q15_t dt,
    fieldctrl_q15_foc_output_t *output);

/**
 * @brief Q15 speed/current cascade tuning for zero d-axis reference.
 */
typedef struct {
    fieldctrl_q15_foc_config_t current; /**< Inner current-loop tuning. */
    linears_q15_t kp; /**< Nonnegative mechanical-speed proportional gain, A/(turn/s). */
    linears_q15_t ki; /**< Nonnegative mechanical-speed integral gain, A/turn. */
    linears_q15_t current_limit; /**< Positive magnitude limit of the q-axis reference, A. */
    uint32_t pole_pairs; /**< Nonzero motor pole-pair count. */
} fieldctrl_q15_speed_foc_config_t;

/**
 * @brief Q15 caller-owned cascaded speed/current controller.
 */
typedef struct {
    fieldctrl_q15_foc_t current; /**< Inner current-controller state. */
    fieldctrl_q15_pi_t speed; /**< Mechanical-speed PI state; output in A. */
    uint32_t pole_pairs; /**< Immutable electrical/mechanical frequency ratio. */
} fieldctrl_q15_speed_foc_t;

/**
 * @brief Q15 cascaded-loop result, including the speed-unit conversion.
 */
typedef struct {
    fieldctrl_q15_foc_output_t current; /**< Inner-loop result. */
    fieldctrl_q15_dq_t reference; /**< Generated current reference, A; d is zero. */
    linears_q15_t mechanical_speed; /**< Electrical feedback divided by pole_pairs, turn/s. */
} fieldctrl_q15_speed_foc_output_t;

/**
 * @brief Initialize a Q15 mechanical-speed/current cascade.
 *
 * @param[out] ctx Non-NULL caller-owned context; no prior initialization needed.
 * @param[in] config Non-NULL copied tuning. Gains are nonnegative, current and
 *                   voltage limits positive, pole_pairs nonzero.
 *
 * @return linears_status_t Initialization status.
 * @retval LINEARS_OK Both integral states initialized to zero.
 * @retval LINEARS_ERROR_ARGUMENT ctx or config is NULL.
 * @retval LINEARS_ERROR_DOMAIN A gain/limit/pole-pair count violates its range.
 * @retval LINEARS_ERROR_RANGE A scalar is nonfinite or out of exponent range.
 * @note Error leaves ctx unchanged; serialize access to ctx.
 */
linears_status_t fieldctrl_q15_speed_foc_init(fieldctrl_q15_speed_foc_t *ctx,
    const fieldctrl_q15_speed_foc_config_t *config);

/**
 * @brief Run a Q15 speed PI followed by the d/q current loop.
 *
 * Current-reference clipping uses conditional speed-PI anti-windup. Inner
 * radial voltage saturation also cancels this sample's speed-integral update
 * if speed error and q-voltage saturation residual have the same sign. The
 * current sample's q reference is retained; the restored integral affects the
 * next sample. The speed and current loops share dt; no implicit decimation.
 *
 * @param[in,out] ctx Non-NULL successfully initialized caller-owned context.
 * @param[in] current Measured alpha/beta currents, A.
 * @param[in] mechanical_reference Signed mechanical speed request, turns/s.
 * @param[in] electrical_speed Signed electrical speed feedback, turns/s.
 * @param[in] electrical_angle Rotor flux angle, electrical turns in [0,1).
 * @param[in] dc_bus Positive measured DC-link voltage, V.
 * @param[in] dt Positive common sample period, seconds.
 * @param[out] output Non-NULL result storage, disjoint from ctx.
 *
 * @return linears_status_t Cascaded-sample status.
 * @retval LINEARS_OK Both controller states and output committed.
 * @retval LINEARS_ERROR_ARGUMENT ctx or output is NULL.
 * @retval LINEARS_ERROR_ALIAS output overlaps ctx.
 * @retval LINEARS_ERROR_DOMAIN dt/DC bus is nonpositive or angle is outside [0,1).
 * @retval LINEARS_ERROR_RANGE Numeric input/intermediate/output is out of range.
 * @note Nonblocking; errors preserve both loops and output. Serialize ctx access.
 * @warning This API neither starts a sensorless motor nor validates a rotor observer.
 */
linears_status_t fieldctrl_q15_speed_foc_step(fieldctrl_q15_speed_foc_t *ctx,
    fieldctrl_q15_ab_t current, linears_q15_t mechanical_reference,
    linears_q15_t electrical_speed, linears_q15_t electrical_angle, linears_q15_t dc_bus, linears_q15_t dt,
    fieldctrl_q15_speed_foc_output_t *output);

/**
 * @brief Q31 d/q current-controller gains and radial voltage ceiling.
 */
typedef struct {
    linears_q31_t kp_d; /**< Nonnegative d-axis proportional gain, V/A. */
    linears_q31_t ki_d; /**< Nonnegative d-axis integral gain, V/(A s). */
    linears_q31_t kp_q; /**< Nonnegative q-axis proportional gain, V/A. */
    linears_q31_t ki_q; /**< Nonnegative q-axis integral gain, V/(A s). */
    linears_q31_t voltage_limit; /**< Positive alpha/beta vector magnitude ceiling, V. */
} fieldctrl_q31_foc_config_t;

/**
 * @brief Q31 caller-owned current-controller state.
 */
typedef struct {
    fieldctrl_q31_foc_config_t config; /**< Immutable copied tuning. */
    struct {
        linears_fixed_t d; /**< Q31 working d-axis integral, V. */
        linears_fixed_t q; /**< Q31 working q-axis integral, V. */
    } integral; /**< Normalized working integral contributions. */
} fieldctrl_q31_foc_t;

/**
 * @brief Q31 current-loop result for one successful control sample.
 */
typedef struct {
    fieldctrl_q31_dq_t current; /**< Measured rotor-frame current, A. */
    fieldctrl_q31_dq_t requested_voltage; /**< PI voltage before radial limiting, V. */
    fieldctrl_q31_dq_t voltage; /**< Limited rotor-frame voltage, V. */
    fieldctrl_q31_ab_t stationary_voltage; /**< Limited stator-frame voltage, V. */
    fieldctrl_q31_abc_t duty; /**< Centered SVPWM duty ratios, each in [0,1]. */
    int saturated; /**< Nonzero when the final voltage request was limited. */
} fieldctrl_q31_foc_output_t;

/**
 * @brief Initialize a Q31 current loop with zero integral state.
 *
 * @param[out] ctx Non-NULL caller-owned context; no prior initialization needed.
 * @param[in] config Non-NULL tuning copied by value; gains must be nonnegative
 *                   and voltage_limit must be strictly positive, in SI units.
 *
 * @return linears_status_t Initialization status.
 * @retval LINEARS_OK Context initialized.
 * @retval LINEARS_ERROR_ARGUMENT ctx or config is NULL.
 * @retval LINEARS_ERROR_DOMAIN A gain is negative or the voltage limit is nonpositive.
 * @retval LINEARS_ERROR_RANGE A scalar is nonfinite or its exponent is invalid.
 * @note On error ctx is unchanged. Serialize access to ctx.
 */
linears_status_t fieldctrl_q31_foc_init(fieldctrl_q31_foc_t *ctx,
    const fieldctrl_q31_foc_config_t *config);

/**
 * @brief Run Q31 d/q PI control, radial anti-windup and centered SVPWM.
 *
 * The voltage radius is min(config.voltage_limit, dc_bus/sqrt(3)). Both
 * integral increments are rejected together if their dot product with the
 * radial saturation residual is positive; the voltage is then recomputed
 * using the prior integral. There is no independent d/q voltage clipping.
 * Final scalar rounding can perturb the voltage-radius bound; SVPWM performs
 * its own bus-limit check at output precision.
 *
 * @param[in,out] ctx Non-NULL initialized context; retained by the caller.
 * @param[in] current Simultaneous measured alpha/beta currents, A.
 * @param[in] reference Requested d/q currents, A.
 * @param[in] electrical_angle Rotor flux angle, electrical turns in [0,1).
 * @param[in] dc_bus Positive measured DC-link voltage, V.
 * @param[in] dt Positive elapsed control period, seconds.
 * @param[out] output Non-NULL result storage, disjoint from ctx.
 *
 * @return linears_status_t Control-sample status.
 * @retval LINEARS_OK State and software voltage/duty command committed.
 * @retval LINEARS_ERROR_ARGUMENT ctx or output is NULL.
 * @retval LINEARS_ERROR_ALIAS output overlaps ctx.
 * @retval LINEARS_ERROR_DOMAIN dt/DC bus is nonpositive or angle is outside [0,1).
 * @retval LINEARS_ERROR_RANGE An input/intermediate/output is nonfinite or out of exponent range.
 * @pre Initialize ctx successfully and provide coherent current/angle samples.
 * @note Nonblocking; errors preserve state and output. Serialize use of ctx.
 * @warning Duty limits are not a substitute for board-level current/voltage protection.
 */
linears_status_t fieldctrl_q31_foc_step(fieldctrl_q31_foc_t *ctx,
    fieldctrl_q31_ab_t current, fieldctrl_q31_dq_t reference,
    linears_q31_t electrical_angle, linears_q31_t dc_bus, linears_q31_t dt,
    fieldctrl_q31_foc_output_t *output);

/**
 * @brief Q31 speed/current cascade tuning for zero d-axis reference.
 */
typedef struct {
    fieldctrl_q31_foc_config_t current; /**< Inner current-loop tuning. */
    linears_q31_t kp; /**< Nonnegative mechanical-speed proportional gain, A/(turn/s). */
    linears_q31_t ki; /**< Nonnegative mechanical-speed integral gain, A/turn. */
    linears_q31_t current_limit; /**< Positive magnitude limit of the q-axis reference, A. */
    uint32_t pole_pairs; /**< Nonzero motor pole-pair count. */
} fieldctrl_q31_speed_foc_config_t;

/**
 * @brief Q31 caller-owned cascaded speed/current controller.
 */
typedef struct {
    fieldctrl_q31_foc_t current; /**< Inner current-controller state. */
    fieldctrl_q31_pi_t speed; /**< Mechanical-speed PI state; output in A. */
    uint32_t pole_pairs; /**< Immutable electrical/mechanical frequency ratio. */
} fieldctrl_q31_speed_foc_t;

/**
 * @brief Q31 cascaded-loop result, including the speed-unit conversion.
 */
typedef struct {
    fieldctrl_q31_foc_output_t current; /**< Inner-loop result. */
    fieldctrl_q31_dq_t reference; /**< Generated current reference, A; d is zero. */
    linears_q31_t mechanical_speed; /**< Electrical feedback divided by pole_pairs, turn/s. */
} fieldctrl_q31_speed_foc_output_t;

/**
 * @brief Initialize a Q31 mechanical-speed/current cascade.
 *
 * @param[out] ctx Non-NULL caller-owned context; no prior initialization needed.
 * @param[in] config Non-NULL copied tuning. Gains are nonnegative, current and
 *                   voltage limits positive, pole_pairs nonzero.
 *
 * @return linears_status_t Initialization status.
 * @retval LINEARS_OK Both integral states initialized to zero.
 * @retval LINEARS_ERROR_ARGUMENT ctx or config is NULL.
 * @retval LINEARS_ERROR_DOMAIN A gain/limit/pole-pair count violates its range.
 * @retval LINEARS_ERROR_RANGE A scalar is nonfinite or out of exponent range.
 * @note Error leaves ctx unchanged; serialize access to ctx.
 */
linears_status_t fieldctrl_q31_speed_foc_init(fieldctrl_q31_speed_foc_t *ctx,
    const fieldctrl_q31_speed_foc_config_t *config);

/**
 * @brief Run a Q31 speed PI followed by the d/q current loop.
 *
 * Current-reference clipping uses conditional speed-PI anti-windup. Inner
 * radial voltage saturation also cancels this sample's speed-integral update
 * if speed error and q-voltage saturation residual have the same sign. The
 * current sample's q reference is retained; the restored integral affects the
 * next sample. The speed and current loops share dt; no implicit decimation.
 *
 * @param[in,out] ctx Non-NULL successfully initialized caller-owned context.
 * @param[in] current Measured alpha/beta currents, A.
 * @param[in] mechanical_reference Signed mechanical speed request, turns/s.
 * @param[in] electrical_speed Signed electrical speed feedback, turns/s.
 * @param[in] electrical_angle Rotor flux angle, electrical turns in [0,1).
 * @param[in] dc_bus Positive measured DC-link voltage, V.
 * @param[in] dt Positive common sample period, seconds.
 * @param[out] output Non-NULL result storage, disjoint from ctx.
 *
 * @return linears_status_t Cascaded-sample status.
 * @retval LINEARS_OK Both controller states and output committed.
 * @retval LINEARS_ERROR_ARGUMENT ctx or output is NULL.
 * @retval LINEARS_ERROR_ALIAS output overlaps ctx.
 * @retval LINEARS_ERROR_DOMAIN dt/DC bus is nonpositive or angle is outside [0,1).
 * @retval LINEARS_ERROR_RANGE Numeric input/intermediate/output is out of range.
 * @note Nonblocking; errors preserve both loops and output. Serialize ctx access.
 * @warning This API neither starts a sensorless motor nor validates a rotor observer.
 */
linears_status_t fieldctrl_q31_speed_foc_step(fieldctrl_q31_speed_foc_t *ctx,
    fieldctrl_q31_ab_t current, linears_q31_t mechanical_reference,
    linears_q31_t electrical_speed, linears_q31_t electrical_angle, linears_q31_t dc_bus, linears_q31_t dt,
    fieldctrl_q31_speed_foc_output_t *output);

#ifdef __cplusplus
}
#endif
#endif
