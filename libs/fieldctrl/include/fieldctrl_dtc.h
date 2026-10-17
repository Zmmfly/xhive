/**
 * @file fieldctrl_dtc.h
 * @brief Voltage-model, six-sector direct torque control for a two-level inverter.
 *
 * Coordinates use an amplitude-invariant Clarke transform. Current is in A,
 * voltage in V, resistance in ohm, stator flux linkage in Wb, torque in N*m,
 * and time in s. A consistent per-unit system must preserve these equations:
 * psi[k+1] = psi[k] + dt * (applied_voltage - R * current),
 * torque = (3/2) * pole_pairs * (psi_alpha * i_beta - psi_beta * i_alpha).
 * Positive electrical rotation is from alpha toward beta.
 *
 * This is the basic switching table (BST) discussed for PMSM in Nasr et al.,
 * Energies 13(8), 1907 (2020), DOI: 10.3390/en13081907, Sections 2.1--2.3.
 * It is not a guarantee of stable control over every PMSM operating point.
 * See README_dtc.md for its switching table, tuning and observer limitations.
 *
 * Every context belongs to the caller. Independent contexts are reentrant;
 * serialize initialization and steps that share a context or output storage.
 * There is no hardware access, allocation, blocking wait or implicit clock.
 * Interrupt use is permitted if the caller supplies exclusive storage and
 * meets its execution-time budget. Public context members are inspectable but
 * must not be modified between successful initialization and subsequent steps.
 */
#ifndef FIELDCTRL_DTC_H
#define FIELDCTRL_DTC_H

#include "fieldctrl_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Single-precision DTC configuration, copied into a caller-owned context.
 */
typedef struct {
    float resistance; /**< Stator phase resistance in ohm, finite and >= 0. */
    float flux_band; /**< Flux-error hysteresis half-band in Wb, finite and > 0. */
    float torque_band; /**< Torque-error outer hysteresis threshold in N*m, finite and > 0. */
    uint16_t pole_pairs; /**< Mechanical-to-electrical pole-pair count, 1..65535. */
} fieldctrl_f32_dtc_config_t;

/**
 * @brief Single-precision, explicitly owned DTC state; initialize before use.
 */
typedef struct {
    fieldctrl_f32_dtc_config_t config; /**< Validated parameters; reinitialize to change. */
    float flux_alpha; /**< Integrated stator alpha flux linkage in Wb. */
    float flux_beta; /**< Integrated stator beta flux linkage in Wb. */
    int8_t flux_state; /**< Flux hysteresis memory: -1 decrease, +1 increase. */
    int8_t torque_state; /**< Torque memory: -1 decrease, 0 neutral, +1 increase. */
    uint8_t last_switch; /**< Previously requested inverter state, 0..7; not hardware feedback. */
} fieldctrl_f32_dtc_t;

/**
 * @brief Single-precision estimates and the next requested inverter state.
 */
typedef struct {
    fieldctrl_f32_ab_t stator_flux; /**< Integrated flux linkage at the updated sample, Wb. */
    float flux_magnitude; /**< Nonnegative magnitude of stator_flux, Wb. */
    float torque; /**< Signed electromagnetic torque estimate, N*m. */
    uint8_t sector; /**< Flux sector 1..6, centered at (sector-1)/6 electrical turns. */
    uint8_t switch_state; /**< Bit 2=a, bit 1=b, bit 0=c upper switch; 0 and 7 are zero vectors. */
    int8_t flux_state; /**< Flux comparator output, -1 or +1. */
    int8_t torque_state; /**< Torque comparator output, -1, 0 or +1. */
} fieldctrl_f32_dtc_result_t;

/**
 * @brief Initialize single-precision DTC from a known nonzero stator flux.
 *
 * @param[out] ctx Non-NULL caller-owned context; unchanged on failure.
 * @param[in] config Finite parameters: resistance >= 0, both bands > 0,
 *                   and pole_pairs in 1..65535.
 * @param[in] initial_flux Nonzero, finite alpha/beta stator flux linkage in Wb.
 *                         For PMSM obtain it from known rotor position and
 *                         current, not by assuming the initial flux is zero.
 * @return linears_status_t Initialization status.
 * @retval LINEARS_OK State initialized with flux_state=+1, torque_state=0,
 *                    and last_switch=0; no inverter command is executed.
 * @retval LINEARS_ERROR_ARGUMENT ctx is NULL or config violates its bounds.
 * @retval LINEARS_ERROR_RANGE A scalar is nonfinite.
 * @retval LINEARS_ERROR_DOMAIN initial_flux has both components zero.
 * @note Serialize access to ctx. Reinitialization resets the voltage integrator
 *       and is the caller's explicit mechanism for correcting its initial bias.
 */
linears_status_t fieldctrl_f32_dtc_init(fieldctrl_f32_dtc_t *ctx,
                                      fieldctrl_f32_dtc_config_t config,
                                      fieldctrl_f32_ab_t initial_flux);

/**
 * @brief Integrate actual applied voltage, estimate torque, and select a state.
 *
 * Flux error >= +flux_band selects +1; error <= -flux_band selects -1;
 * otherwise the previous flux state is retained. Torque error >= +torque_band
 * selects +1 and <= -torque_band selects -1. Inside the outer band an active
 * torque state persists only until the error reaches/crosses zero; otherwise
 * torque_state is 0. Only torque_state=0 selects a zero vector, choosing 0 or 7
 * to minimize bit changes from last_switch (including retaining a zero vector).
 *
 * @param[in,out] ctx Non-NULL initialized context, unchanged on any failure.
 * @param[in] current Finite measured alpha/beta current in A, time-aligned with
 *                    applied_voltage; held constant for this Euler interval.
 * @param[in] applied_voltage Finite actual average alpha/beta terminal voltage
 *                            in V over the elapsed dt, supplied by the caller.
 *                            Never substitute an unexecuted switching request.
 * @param[in] flux_reference Finite desired stator flux magnitude in Wb, strictly
 *                           greater than ctx->config.flux_band.
 * @param[in] torque_reference Finite signed torque reference in N*m.
 * @param[in] dt Finite elapsed sample time in s, strictly > 0; all intermediate
 *               values must be representable. It is not a blocking timeout.
 * @param[out] out Non-NULL result, disjoint from ctx; unchanged on any failure.
 * @return linears_status_t Step status.
 * @retval LINEARS_OK Both ctx and out committed; switch_state is a request only.
 * @retval LINEARS_ERROR_ARGUMENT NULL storage or invalid dt/reference bounds.
 * @retval LINEARS_ERROR_ALIAS ctx and out overlap.
 * @retval LINEARS_ERROR_RANGE Nonfinite scalar or unrepresentable arithmetic.
 * @retval LINEARS_ERROR_DOMAIN Updated stator flux is zero, so its sector is
 *                             undefined. No partial integration is committed.
 * @pre Initialize ctx and serialize calls sharing ctx or out. Use sufficiently
 *      short dt and suitable bands for the motor, voltage and switching delays.
 * @warning A pure voltage integrator drifts with voltage/current offsets and R
 *          error, particularly at low speed. There is no startup alignment,
 *          current limiting, dead-time insertion, gate drive or safety shutdown.
 */
linears_status_t fieldctrl_f32_dtc_step(fieldctrl_f32_dtc_t *ctx,
                                      fieldctrl_f32_ab_t current,
                                      fieldctrl_f32_ab_t applied_voltage,
                                      float flux_reference,
                                      float torque_reference,
                                      float dt,
                                      fieldctrl_f32_dtc_result_t *out);

/**
 * @brief Double-precision DTC configuration in SI or consistent per-unit units.
 */
typedef struct {
    double resistance; /**< Stator phase resistance in ohm, finite and >= 0. */
    double flux_band; /**< Flux-error half-band in Wb, finite and > 0. */
    double torque_band; /**< Torque-error outer threshold in N*m, finite and > 0. */
    uint16_t pole_pairs; /**< Pole-pair count, 1..65535. */
} fieldctrl_f64_dtc_config_t;

/**
 * @brief Double-precision DTC state, owned and serialized by the caller.
 */
typedef struct {
    fieldctrl_f64_dtc_config_t config; /**< Validated parameters; reinitialize to change. */
    double flux_alpha; /**< Integrated alpha flux linkage in Wb. */
    double flux_beta; /**< Integrated beta flux linkage in Wb. */
    int8_t flux_state; /**< Hysteresis memory: -1 or +1. */
    int8_t torque_state; /**< Hysteresis memory: -1, 0 or +1. */
    uint8_t last_switch; /**< Last requested inverter state 0..7, not applied-voltage feedback. */
} fieldctrl_f64_dtc_t;

/**
 * @brief Double-precision estimates and next requested switching state.
 */
typedef struct {
    fieldctrl_f64_ab_t stator_flux; /**< Updated alpha/beta stator flux linkage in Wb. */
    double flux_magnitude; /**< Updated stator flux magnitude in Wb. */
    double torque; /**< Signed electromagnetic torque estimate in N*m. */
    uint8_t sector; /**< Sector 1..6 centered on (sector-1)/6 electrical turns. */
    uint8_t switch_state; /**< Upper-switch bits abc in bits 2,1,0; 0/7 are zero vectors. */
    int8_t flux_state; /**< Flux decision, -1 or +1. */
    int8_t torque_state; /**< Torque decision, -1, 0 or +1. */
} fieldctrl_f64_dtc_result_t;

/**
 * @brief Initialize double-precision DTC with a known nonzero flux vector.
 *
 * @param[out] ctx Non-NULL caller context, unchanged on failure.
 * @param[in] config Finite resistance >= 0 ohm, positive bands in Wb and N*m,
 *                   and pole_pairs in 1..65535.
 * @param[in] initial_flux Finite, nonzero alpha/beta stator flux linkage in Wb;
 *                         derive it from a known initial motor condition.
 * @return linears_status_t Initialization status.
 * @retval LINEARS_OK Context initialized to flux_state=+1, torque_state=0 and
 *                    last_switch=0 without any hardware action.
 * @retval LINEARS_ERROR_ARGUMENT NULL ctx or invalid config bounds.
 * @retval LINEARS_ERROR_RANGE A scalar is nonfinite.
 * @retval LINEARS_ERROR_DOMAIN initial_flux is the zero vector.
 * @note Serialize ctx access; reinitialization explicitly resets its integrator.
 */
linears_status_t fieldctrl_f64_dtc_init(fieldctrl_f64_dtc_t *ctx,
                                      fieldctrl_f64_dtc_config_t config,
                                      fieldctrl_f64_ab_t initial_flux);

/**
 * @brief Run one double-precision Euler voltage-model and BST DTC step.
 *
 * The comparator memory, centered sectors and zero-vector selection are exactly
 * those of fieldctrl_f32_dtc_step(). Estimates use the newly integrated flux.
 *
 * @param[in,out] ctx Non-NULL initialized context; unchanged on failure.
 * @param[in] current Finite measured alpha/beta current in A, time-aligned with
 *                    applied_voltage and held constant over dt.
 * @param[in] applied_voltage Actual average alpha/beta terminal voltage in V
 *                            over elapsed dt, not an assumed executed request.
 * @param[in] flux_reference Finite desired flux magnitude in Wb, greater than
 *                           ctx->config.flux_band.
 * @param[in] torque_reference Finite signed desired torque in N*m.
 * @param[in] dt Finite elapsed time in s, > 0; intermediates must be representable.
 * @param[out] out Non-NULL result, disjoint from ctx; unchanged on failure.
 * @return linears_status_t Step status.
 * @retval LINEARS_OK ctx and out committed; no hardware action occurs.
 * @retval LINEARS_ERROR_ARGUMENT NULL storage or invalid dt/reference bounds.
 * @retval LINEARS_ERROR_ALIAS ctx and out overlap.
 * @retval LINEARS_ERROR_RANGE Nonfinite input or unrepresentable arithmetic.
 * @retval LINEARS_ERROR_DOMAIN Updated flux vector is zero; no update commits.
 * @pre Initialize ctx and serialize shared storage. Select dt and bands to
 *      respect motor dynamics, inverter voltage and sampling/switching delay.
 * @warning Voltage-model drift and low-speed R/offset sensitivity are not
 *          corrected. Alignment, current limits and board protection are external.
 */
linears_status_t fieldctrl_f64_dtc_step(fieldctrl_f64_dtc_t *ctx,
                                      fieldctrl_f64_ab_t current,
                                      fieldctrl_f64_ab_t applied_voltage,
                                      double flux_reference,
                                      double torque_reference,
                                      double dt,
                                      fieldctrl_f64_dtc_result_t *out);

/**
 * @brief q15 DTC parameters; each scalar has its own explicit binary exponent.
 */
typedef struct {
    linears_q15_t resistance; /**< Phase resistance in ohm, >= 0. */
    linears_q15_t flux_band; /**< Flux-error half-band in Wb, > 0. */
    linears_q15_t torque_band; /**< Torque-error outer threshold in N*m, > 0. */
    uint16_t pole_pairs; /**< Pole-pair count, 1..65535. */
} fieldctrl_q15_dtc_config_t;

/**
 * @brief q15-interface DTC state with 31-significant-bit integer flux accumulators.
 *
 * Wide flux state avoids rounding every small integration increment to q15.
 * It is still integer-only block fixed point, not a floating-point observer.
 */
typedef struct {
    fieldctrl_q15_dtc_config_t config; /**< Validated q15 parameters, unchanged between init calls. */
    linears_q31_t flux_alpha; /**< Canonical wide alpha flux accumulator in Wb. */
    linears_q31_t flux_beta; /**< Canonical wide beta flux accumulator in Wb. */
    int8_t flux_state; /**< Hysteresis memory: -1 or +1. */
    int8_t torque_state; /**< Hysteresis memory: -1, 0 or +1. */
    uint8_t last_switch; /**< Last requested switch bits 0..7, not measured hardware state. */
} fieldctrl_q15_dtc_t;

/**
 * @brief q15 estimates, independently normalized, and a switching request.
 */
typedef struct {
    fieldctrl_q15_ab_t stator_flux; /**< Updated flux linkage in Wb, each component separately scaled. */
    linears_q15_t flux_magnitude; /**< Nonnegative flux magnitude in Wb. */
    linears_q15_t torque; /**< Signed electromagnetic torque in N*m. */
    uint8_t sector; /**< Sector 1..6 centered on (sector-1)/6 electrical turns. */
    uint8_t switch_state; /**< Upper-switch bits abc in bits 2,1,0; 0/7 are zero vectors. */
    int8_t flux_state; /**< Flux decision, -1 or +1. */
    int8_t torque_state; /**< Torque decision, -1, 0 or +1. */
} fieldctrl_q15_dtc_result_t;

/**
 * @brief Initialize integer-only q15 DTC using a known nonzero flux vector.
 *
 * @param[out] ctx Non-NULL caller context, unchanged on failure.
 * @param[in] config Resistance >= 0 ohm, bands > 0 Wb/N*m, pole_pairs 1..65535.
 *                   Every scalar's input and normalized exponent must lie in
 *                   LINEARS_EXPONENT_MIN..LINEARS_EXPONENT_MAX.
 * @param[in] initial_flux Nonzero initial flux linkage in Wb, q15 values
 *                         mantissa * 2^(exponent-15), with the same exponent
 *                         bounds. Obtain it from a known initial motor condition.
 * @return linears_status_t Initialization status.
 * @retval LINEARS_OK Wide flux state initialized; flux_state=+1, torque_state=0,
 *                    last_switch=0. No switching request is executed.
 * @retval LINEARS_ERROR_ARGUMENT NULL ctx or invalid configuration bounds.
 * @retval LINEARS_ERROR_RANGE Invalid input/normalized exponent.
 * @retval LINEARS_ERROR_DOMAIN initial_flux is the zero vector.
 * @note Serialize ctx access. Initialization/reinitialization resets the observer;
 *       independent instances have no shared mutable state.
 */
linears_status_t fieldctrl_q15_dtc_init(fieldctrl_q15_dtc_t *ctx,
                                      fieldctrl_q15_dtc_config_t config,
                                      fieldctrl_q15_ab_t initial_flux);

/**
 * @brief Run an integer-only q15-interface voltage-model and BST DTC step.
 *
 * Uses 31-significant-bit integer working arithmetic and flux accumulation.
 * Outputs are individually normalized q15 values, mantissa * 2^(exponent-15).
 * Comparator/sector decisions use wide estimates before output quantization;
 * their hysteresis and zero-vector semantics match fieldctrl_f32_dtc_step().
 *
 * @param[in,out] ctx Non-NULL initialized context; unchanged on any failure.
 * @param[in] current Time-aligned measured alpha/beta current in A, held over dt.
 * @param[in] applied_voltage Actual average terminal alpha/beta voltage in V
 *                            over elapsed dt, never assumed from last_switch.
 * @param[in] flux_reference Desired magnitude in Wb, > ctx->config.flux_band.
 * @param[in] torque_reference Signed desired electromagnetic torque in N*m.
 * @param[in] dt Elapsed time in s, strictly > 0, not a timeout. All input,
 *               normalized, intermediate and output exponents must remain in
 *               LINEARS_EXPONENT_MIN..LINEARS_EXPONENT_MAX. Resolution must be
 *               sufficient for the integration increment and hysteresis bands.
 * @param[out] out Non-NULL result, disjoint from ctx; unchanged on any failure.
 * @return linears_status_t Step status.
 * @retval LINEARS_OK Both objects committed; only a switching request is returned.
 * @retval LINEARS_ERROR_ARGUMENT NULL storage or invalid dt/reference bounds.
 * @retval LINEARS_ERROR_ALIAS ctx and out overlap.
 * @retval LINEARS_ERROR_RANGE Invalid exponent or unrepresentable arithmetic/output.
 * @retval LINEARS_ERROR_DOMAIN Updated flux is zero; no partial integration commits.
 * @pre Initialize ctx; serialize shared ctx/out. Choose dt/bands for motor and
 *      inverter dynamics and explicit fixed-point resolution.
 * @warning No drift compensation, low-speed observability guarantee, startup
 *          alignment, current limit, dead-time insertion or hardware protection.
 */
linears_status_t fieldctrl_q15_dtc_step(fieldctrl_q15_dtc_t *ctx,
                                      fieldctrl_q15_ab_t current,
                                      fieldctrl_q15_ab_t applied_voltage,
                                      linears_q15_t flux_reference,
                                      linears_q15_t torque_reference,
                                      linears_q15_t dt,
                                      fieldctrl_q15_dtc_result_t *out);

/**
 * @brief q31 DTC parameters with independent explicit binary exponents.
 */
typedef struct {
    linears_q31_t resistance; /**< Phase resistance in ohm, >= 0. */
    linears_q31_t flux_band; /**< Flux-error half-band in Wb, > 0. */
    linears_q31_t torque_band; /**< Torque-error outer threshold in N*m, > 0. */
    uint16_t pole_pairs; /**< Pole-pair count, 1..65535. */
} fieldctrl_q31_dtc_config_t;

/**
 * @brief Caller-owned integer-only q31 DTC state; no floating-point members.
 */
typedef struct {
    fieldctrl_q31_dtc_config_t config; /**< Validated parameters; reinitialize to change. */
    linears_q31_t flux_alpha; /**< Canonical wide alpha stator flux linkage in Wb. */
    linears_q31_t flux_beta; /**< Canonical wide beta stator flux linkage in Wb. */
    int8_t flux_state; /**< Flux memory: -1 or +1. */
    int8_t torque_state; /**< Torque memory: -1, 0 or +1. */
    uint8_t last_switch; /**< Last requested inverter state, 0..7, not voltage feedback. */
} fieldctrl_q31_dtc_t;

/**
 * @brief q31 flux/torque estimates and a discrete inverter switching request.
 */
typedef struct {
    fieldctrl_q31_ab_t stator_flux; /**< Updated flux linkage in Wb, independent component exponents. */
    linears_q31_t flux_magnitude; /**< Nonnegative flux magnitude in Wb. */
    linears_q31_t torque; /**< Signed electromagnetic torque in N*m. */
    uint8_t sector; /**< Sector 1..6 centered on (sector-1)/6 electrical turns. */
    uint8_t switch_state; /**< Upper-switch bits abc in bits 2,1,0; 0/7 are zero vectors. */
    int8_t flux_state; /**< Flux decision, -1 or +1. */
    int8_t torque_state; /**< Torque decision, -1, 0 or +1. */
} fieldctrl_q31_dtc_result_t;

/**
 * @brief Initialize q31 DTC from a known initial flux without floating arithmetic.
 *
 * @param[out] ctx Non-NULL caller context; unchanged on failure.
 * @param[in] config Resistance >= 0 ohm, bands > 0 Wb/N*m, pole_pairs 1..65535.
 *                   Input and normalized exponents must remain in
 *                   LINEARS_EXPONENT_MIN..LINEARS_EXPONENT_MAX.
 * @param[in] initial_flux Nonzero flux linkage in Wb represented by independent
 *                         q31 scalars, mantissa * 2^(exponent-31), with the same
 *                         exponent bounds. Requires a known initial condition.
 * @return linears_status_t Initialization status.
 * @retval LINEARS_OK State initialized with flux_state=+1, torque_state=0,
 *                    last_switch=0; no hardware action occurs.
 * @retval LINEARS_ERROR_ARGUMENT NULL ctx or invalid config bounds.
 * @retval LINEARS_ERROR_RANGE Invalid input/normalized exponent.
 * @retval LINEARS_ERROR_DOMAIN initial_flux is zero.
 * @note Serialize ctx access. Reinitialization explicitly resets the integrator.
 */
linears_status_t fieldctrl_q31_dtc_init(fieldctrl_q31_dtc_t *ctx,
                                      fieldctrl_q31_dtc_config_t config,
                                      fieldctrl_q31_ab_t initial_flux);

/**
 * @brief Run one integer-only q31 Euler observer and six-sector BST step.
 *
 * Uses 31-significant-bit working arithmetic and independently normalized
 * outputs. Hysteresis, sector and zero-vector rules match fieldctrl_f32_dtc_step().
 *
 * @param[in,out] ctx Non-NULL initialized context, unchanged on any failure.
 * @param[in] current Time-aligned measured alpha/beta current in A, held over dt.
 * @param[in] applied_voltage Actual average alpha/beta terminal voltage in V
 *                            during dt; not inferred from a requested state.
 * @param[in] flux_reference Desired flux magnitude in Wb, > ctx->config.flux_band.
 * @param[in] torque_reference Signed desired torque in N*m.
 * @param[in] dt Elapsed time in s, strictly > 0. Input, normalized, intermediate
 *               and output exponents must be within LINEARS_EXPONENT_MIN through
 *               LINEARS_EXPONENT_MAX. Integration increments and hysteresis
 *               bands must be resolvable at the working precision.
 * @param[out] out Non-NULL result, disjoint from ctx, unchanged on any failure.
 * @return linears_status_t Step status.
 * @retval LINEARS_OK ctx and out committed; switch_state is not executed.
 * @retval LINEARS_ERROR_ARGUMENT NULL storage or invalid dt/reference bounds.
 * @retval LINEARS_ERROR_ALIAS ctx and out overlap.
 * @retval LINEARS_ERROR_RANGE Invalid exponent or unrepresentable intermediate/output.
 * @retval LINEARS_ERROR_DOMAIN Updated flux is zero; no integration is committed.
 * @pre Initialize ctx and serialize shared storage. Tune dt and bands for the
 *      motor, inverter and sampling/switching delays, including quantization.
 * @warning The voltage integrator has no offset/drift correction or low-speed
 *          guarantee. Startup alignment and all board protection are external.
 */
linears_status_t fieldctrl_q31_dtc_step(fieldctrl_q31_dtc_t *ctx,
                                      fieldctrl_q31_ab_t current,
                                      fieldctrl_q31_ab_t applied_voltage,
                                      linears_q31_t flux_reference,
                                      linears_q31_t torque_reference,
                                      linears_q31_t dt,
                                      fieldctrl_q31_dtc_result_t *out);

#ifdef __cplusplus
}
#endif
#endif
