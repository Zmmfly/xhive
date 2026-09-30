/**
 * @file pid.h
 * @author Zmmfly
 * @brief Generic PID controller of the ezctrl control library
 * @version 0.1
 * @date 2026-09-29
 *
 * @copyright Copyright (c) 2026, License under GPLv3
 *
 * Scalar variants
 * ---------------
 * The controller is declared once per scalar type, and the type is part of
 * every symbolic name:
 *
 * - `float`: `ezc_pid_f32_*`
 * - `double`: `ezc_pid_f64_*`
 *
 * Fixed-point variants are expected to follow the same pattern with their own
 * suffix, so application code keeps a single style across all variants.
 *
 * Update semantics
 * ----------------
 * One update step `n` computes
 *
 *     e(n) = setpoint - measure
 *     b    = weight_p, or 1 while the setpoint weight option is off
 *     P(n) = kp * (b * setpoint - measure)
 *     I(n) = I(n-1) + ki * e(n) * dt
 *     D(n) = kd * (e(n) - e(n-1)) / dt              (derivative on error)
 *     D(n) = kd * (measure(n-1) - measure(n)) / dt  (derivative on measurement)
 *     u(n) = P(n) + I(n) + D(n)
 *
 * - Gains are used as given: negative gains implement a reverse-acting loop and
 *   are not rejected by any entry point.
 * - `dt` is the elapsed time in seconds since the previous update step. It is
 *   passed to every step, so one configuration serves constant, jittering and
 *   event-driven sample rates alike.
 * - `ki` is the integral gain per second and `kd` the derivative gain per
 *   second. The reciprocal of `dt` is cached in the context and reused while
 *   the caller repeats the same value, which keeps the division out of
 *   fixed-rate loops.
 * - A `dt` that is not a finite positive number (zero, negative, NaN or
 *   infinite) is a zero-time step: the integral does not advance, the
 *   derivative term is zero, the derivative filter keeps its state,
 *   `prev_error` and `prev_measure` still track this step, and the returned
 *   output is the proportional term plus the unchanged integral, clamped and
 *   reported as usual.
 * - The derivative term is derived from the error by default, so a setpoint
 *   step also produces a derivative kick. The derivative source option moves it
 *   to the measurement and removes the kick; the derivative filter option
 *   suppresses the noise that a raw derivative would pass on.
 * - The first step after initialization or reset uses `e(n-1) = 0` and
 *   `measure(n-1) = 0`, so the derivative term starts from the full rate of
 *   the first sample.
 * - The output limit is optional. While it is disabled, `u(n)` is returned as
 *   computed.
 * - Anti-windup is optional and only effective together with the output limit.
 *   While it is enabled, the integral term may not advance past the value that
 *   places the unclamped output exactly on a limit. The limiter only stops the
 *   integral from moving further into saturation; it never moves the integral
 *   back towards its previous value and it never saturates the integral by
 *   itself.
 * - The callback is optional. The output is returned either way, and the
 *   callback additionally receives the callback context pointer and the full
 *   result of the update step.
 *
 * Optional features
 * -----------------
 * Every optional feature is off in a zero-initialized context, so enabling one
 * never changes behavior implicitly:
 *
 * - Setpoint weight (`EZC_PID_OPT_SETPOINT_WEIGHT`): the proportional term uses
 *   `b * setpoint - measure` with `b` in `[0, 1]`. `b = 0` is
 *   proportional-on-measurement, `b = 1` restores the default term.
 * - Derivative source (@ref ezc_pid_deriv_source_t): the derivative is computed
 *   from the error by default, or from the measurement to remove the setpoint
 *   derivative kick.
 * - Derivative filter (`deriv_filter_t` greater than zero): the derivative term
 *   passes a first-order low pass `1 / (1 + s * deriv_filter_t)`, discretized
 *   as `D += a * (D_raw - D)` with `a = dt / (deriv_filter_t + dt)`. Zero
 *   disables the filter; its state then tracks the raw derivative, so the
 *   filter can be enabled later without a transient.
 * - Integral limit (`EZC_PID_OPT_INTEGRAL_LIMIT`): the integral term is held
 *   inside `[integral_min, integral_max]`, independently of the output limit
 *   and of anti-windup.
 * - Integral separation (`EZC_PID_OPT_INTEGRAL_SEPARATION`): integration is
 *   suspended while `|e| > integral_separation` and resumes below it.
 *
 * Minimal usage
 * -------------
 * The shortest form needs no configuration structure: a zero-initialized
 * context is a valid disabled controller, and the setters configure it in
 * place.
 *
 * @code
 * #include "ezctrl/pid.h"
 *
 * ezc_pid_f32_t pid = {0};                       // zero-initialized: disabled
 * ezc_pid_f32_set_gains(&pid, 1.0f, 0.5f, 0.0f); // kp, ki per second, kd per second
 *
 * // once per control period, dt = seconds since the previous call
 * float duty = ezc_pid_f32_update(&pid, setpoint, measure, dt);
 * @endcode
 *
 * A controller that is configured in one place, clamped to the actuator range
 * and observable through a callback looks like this:
 *
 * @code
 * static void log_output(void *user_ctx, const ezc_pid_f32_result_t *result);
 *
 * ezc_pid_f32_t pid;
 * ezc_pid_f32_cfg_t cfg = {0};
 * cfg.kp = 2.0f;
 * cfg.ki = 0.5f;
 * cfg.kd = 0.01f;
 * cfg.limit_enabled = true;
 * cfg.out_min = 0.0f;
 * cfg.out_max = 1.0f;
 * cfg.anti_windup_enabled = true;
 *
 * if (ezc_pid_f32_init(&pid, &cfg) != EZC_OK) {
 *     // rejected parameters; the context is zeroed and stays usable
 * }
 * ezc_pid_f32_set_callback(&pid, log_output, app);
 *
 * float duty = ezc_pid_f32_update(&pid, target, measured, dt);
 * @endcode
 *
 * The host test suites are further executable examples: `test/pid_update.cc`
 * for the step semantics, `test/pid_options.cc` for the optional features, and
 * `test/compile/consumer.c` for plain C consumption.
 *
 * A context must be initialized by the matching `*_init()` entry point before
 * it is used. A zero-initialized context is a valid disabled controller (all
 * gains zero, no limit, no callback, no optional feature) and produces a zero
 * output.
 */
#ifndef __EZCTRL_PID_H__
#define __EZCTRL_PID_H__

#include "ezctrl/common.h"

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/**
 * @brief Derivative source of the PID derivative term.
 */
typedef enum ezc_pid_deriv_source_e {
    EZC_PID_DERIV_ON_ERROR = 0,       /**< Derive the error; setpoint steps kick it. */
    EZC_PID_DERIV_ON_MEASUREMENT = 1, /**< Derive the measurement; setpoint steps do not. */
} ezc_pid_deriv_source_t;

/**
 * @brief Optional feature flags of the controller `options` field.
 *
 * A zero-initialized context has none of them set, which keeps every optional
 * feature disabled.
 */
#define EZC_PID_OPT_INTEGRAL_LIMIT      (1u << 0) /**< Hold the integral inside its limits. */
#define EZC_PID_OPT_INTEGRAL_SEPARATION (1u << 1) /**< Suspend integration above a |e| threshold. */
#define EZC_PID_OPT_SETPOINT_WEIGHT     (1u << 2) /**< Use weight_p in the proportional term. */

/**
 * @brief Result of a single float PID update step.
 *
 * The structure reports the inputs that were used, the terms that were
 * computed, and the output that was returned to the caller. It is produced by
 * @ref ezc_pid_f32_update and handed to @ref ezc_pid_f32_cb_t. When `limited`
 * is true, the returned output sits exactly on the limit it crossed.
 */
typedef struct ezc_pid_f32_result_s {
    float output;   /**< Output after clamping; equal to the update return value. */
    float setpoint; /**< Setpoint of this update step. */
    float measure;  /**< Measurement of this update step. */
    float error;    /**< Error `e = setpoint - measure`. */
    float p_term;   /**< Proportional term, `kp * e` or `kp * (b * setpoint - measure)`. */
    float i_term;   /**< Integral term `I(n)` after separation, limiting and anti-windup. */
    float d_term;   /**< Derivative term that was applied, including the optional filter. */
    bool limited;   /**< true when the unclamped output crossed a limit. */
} ezc_pid_f32_result_t;

/**
 * @brief Optional callback invoked after every completed float PID update.
 *
 * @param[in] user_ctx Callback context stored in the controller context; it is
 *                     the `callback_ctx` value passed to
 *                     @ref ezc_pid_f32_init or @ref ezc_pid_f32_set_callback.
 * @param[in] result   Result of the completed update step. The pointer stays
 *                     valid for the duration of the call only.
 */
typedef void (*ezc_pid_f32_cb_t)(void *user_ctx, const ezc_pid_f32_result_t *result);

/**
 * @brief float PID controller context.
 *
 * Gains, limits, options and callback are configuration; `integral`,
 * `prev_error`, `prev_measure` and `d_filtered` are controller state owned by
 * @ref ezc_pid_f32_update, while `dt_cached` and `dt_inv_cached` only memoize
 * the caller's sample period. Initialize the context with
 * @ref ezc_pid_f32_init, or zero-initialize it for a disabled controller.
 */
typedef struct ezc_pid_f32_s {
    float kp;                 /**< Proportional gain. */
    float ki;                 /**< Integral gain per second. */
    float kd;                 /**< Derivative gain per second. */
    uint32_t options;         /**< Optional features, see `EZC_PID_OPT_*`. */
    ezc_pid_deriv_source_t deriv_source; /**< Derivative source. */
    float deriv_filter_t;     /**< Derivative filter time constant in seconds; 0 disables it. */
    float weight_p;           /**< Setpoint weight of the proportional term; 0..1. */
    float integral_min;       /**< Lower integral limit. */
    float integral_max;       /**< Upper integral limit. */
    float integral_separation; /**< Error magnitude above which integration stops. */
    float integral;           /**< Integral term state `I(n-1)`. */
    float prev_error;         /**< Error state `e(n-1)`. */
    float prev_measure;       /**< Measurement state `measure(n-1)`. */
    float d_filtered;         /**< Derivative filter state; follows the raw derivative when off. */
    float dt_cached;          /**< Sample period whose reciprocal is cached; 0 means none. */
    float dt_inv_cached;      /**< Cached `1 / dt_cached`, reused while dt is unchanged. */
    float out_min;            /**< Lower output limit; used while limit_enabled. */
    float out_max;            /**< Upper output limit; used while limit_enabled. */
    bool limit_enabled;       /**< Clamp the output to `[out_min, out_max]`. */
    bool anti_windup_enabled; /**< Hold the integral at the saturation boundary. */
    ezc_pid_f32_cb_t callback; /**< Optional update callback; NULL disables it. */
    void *callback_ctx;       /**< Context pointer passed to callback. */
} ezc_pid_f32_t;

/**
 * @brief Initialization parameters of a float PID controller.
 *
 * Value-initialize the structure (`{0}`) to get a disabled controller and
 * enable only the parts the application needs.
 */
typedef struct ezc_pid_f32_cfg_s {
    float kp;                   /**< Proportional gain. */
    float ki;                   /**< Integral gain per second. */
    float kd;                   /**< Derivative gain per second. */
    bool limit_enabled;         /**< Enable output clamping. */
    float out_min;              /**< Lower output limit; must be <= out_max while limit_enabled. */
    float out_max;              /**< Upper output limit; must be >= out_min while limit_enabled. */
    bool anti_windup_enabled;   /**< Enable anti-windup; only effective with limit_enabled. */
    uint32_t options;           /**< Optional features, see `EZC_PID_OPT_*`. */
    ezc_pid_deriv_source_t deriv_source; /**< Derivative source. */
    float deriv_filter_t;       /**< Derivative filter time constant in seconds; 0 disables it. */
    float weight_p;             /**< Setpoint weight of the proportional term; 0..1. */
    float integral_min;         /**< Lower integral limit. */
    float integral_max;         /**< Upper integral limit. */
    float integral_separation;  /**< Error magnitude above which integration stops. */
    ezc_pid_f32_cb_t callback;  /**< Optional update callback; NULL disables it. */
    void *callback_ctx;         /**< Context pointer passed to callback. */
} ezc_pid_f32_cfg_t;

/**
 * @brief Initialize a float PID controller.
 *
 * The whole context is overwritten: gains, limits, options, callback,
 * controller state and the cached sample period are taken from @p cfg.
 *
 * @param[out] ctx Controller context to initialize. Must not be NULL.
 * @param[in]  cfg Initialization parameters. Must not be NULL. Every enabled
 *                 option is validated: `cfg->out_min` must not exceed
 *                 `cfg->out_max` while `limit_enabled` is set, the integral
 *                 limits must not be inverted while
 *                 `EZC_PID_OPT_INTEGRAL_LIMIT` is set, `integral_separation`
 *                 must be greater than zero while
 *                 `EZC_PID_OPT_INTEGRAL_SEPARATION` is set, `weight_p` must be
 *                 inside `[0, 1]` while `EZC_PID_OPT_SETPOINT_WEIGHT` is set,
 *                 `deriv_filter_t` must not be negative and `deriv_source`
 *                 must be a known value. Ranges that are not comparable (for
 *                 example NaN limits) are rejected as well.
 * @return ezc_status_t Operation status.
 * @retval EZC_OK The context was configured from @p cfg.
 * @retval EZC_ERR_INVALID_ARG ctx or cfg is NULL, or an enabled option, limit
 *                             or filter parameter is out of range. The context
 *                             is zeroed and stays usable as a disabled
 *                             controller.
 *
 * @pre @p ctx references writable storage that outlives its use as a controller
 *      context.
 * @post The controller state, the derivative filter state and the sample
 *       period cache are cleared: the first update step starts from
 *       `I(0) = e(0) = D(0) = 0`.
 * @note Not reentrant: do not initialize the same context concurrently.
 */
ezc_status_t ezc_pid_f32_init(ezc_pid_f32_t *ctx, const ezc_pid_f32_cfg_t *cfg);

/**
 * @brief Change the float PID gains without touching the controller state.
 *
 * @param[in,out] ctx Controller context. Must not be NULL.
 * @param[in] kp Proportional gain.
 * @param[in] ki Integral gain per second.
 * @param[in] kd Derivative gain per second.
 * @return ezc_status_t Operation status.
 * @retval EZC_OK The gains were stored.
 * @retval EZC_ERR_INVALID_ARG ctx is NULL; the context is left unchanged.
 *
 * @note Gains may be changed between update steps, for example by a tuning
 *       command. The integral and error state are preserved.
 */
ezc_status_t ezc_pid_f32_set_gains(ezc_pid_f32_t *ctx, float kp, float ki, float kd);

/**
 * @brief Enable float output clamping and set its range.
 *
 * @param[in,out] ctx Controller context. Must not be NULL.
 * @param[in] out_min Lower output limit.
 * @param[in] out_max Upper output limit; must be greater than or equal to
 *                    @p out_min, otherwise the call is rejected. A range that
 *                    is not comparable (for example NaN limits) is rejected
 *                    as well.
 * @return ezc_status_t Operation status.
 * @retval EZC_OK Clamping is enabled with the given range.
 * @retval EZC_ERR_INVALID_ARG ctx is NULL or the range is invalid; the context
 *                             is left unchanged.
 *
 * @note An equal range is valid and pins the output to that value.
 * @note Enabling clamping does not clear the controller state. Resetting the
 *       controller after a large setpoint change remains the caller's choice.
 */
ezc_status_t ezc_pid_f32_set_limit(ezc_pid_f32_t *ctx, float out_min, float out_max);

/**
 * @brief Disable float output clamping.
 *
 * @param[in,out] ctx Controller context. Must not be NULL.
 * @return ezc_status_t Operation status.
 * @retval EZC_OK Clamping is disabled; `out_min` and `out_max` are retained but
 *               ignored until clamping is enabled again.
 * @retval EZC_ERR_INVALID_ARG ctx is NULL; the context is left unchanged.
 *
 * @note Disabling clamping also disables the effect of anti-windup, which needs
 *       a limit to detect saturation.
 */
ezc_status_t ezc_pid_f32_clear_limit(ezc_pid_f32_t *ctx);

/**
 * @brief Enable or disable float anti-windup.
 *
 * @param[in,out] ctx Controller context. Must not be NULL.
 * @param[in] enable true to enable anti-windup, false to disable it.
 * @return ezc_status_t Operation status.
 * @retval EZC_OK The requested mode was stored.
 * @retval EZC_ERR_INVALID_ARG ctx is NULL; the context is left unchanged.
 *
 * @note The setting is stored independently of the output limit and takes
 *       effect once clamping is enabled.
 */
ezc_status_t ezc_pid_f32_set_anti_windup(ezc_pid_f32_t *ctx, bool enable);

/**
 * @brief Install or remove the float update callback.
 *
 * @param[in,out] ctx Controller context. Must not be NULL.
 * @param[in] callback Function invoked after each completed update step, or
 *                     NULL to remove the current callback.
 * @param[in] user_ctx Context pointer handed to @p callback. May be NULL, and
 *                     may differ per callback installation.
 * @return ezc_status_t Operation status.
 * @retval EZC_OK The callback and its context were stored.
 * @retval EZC_ERR_INVALID_ARG ctx is NULL; the context is left unchanged.
 *
 * @note The callback receives the same output that the update call returns. It
 *       runs in the update caller's context and must not call back into the
 *       same controller context.
 */
ezc_status_t ezc_pid_f32_set_callback(ezc_pid_f32_t *ctx, ezc_pid_f32_cb_t callback,
                                      void *user_ctx);

/**
 * @brief Select which signal feeds the float derivative term.
 *
 * @param[in,out] ctx Controller context. Must not be NULL.
 * @param[in] source EZC_PID_DERIV_ON_ERROR (default) derives the error and adds
 *                   a kick on setpoint steps, EZC_PID_DERIV_ON_MEASUREMENT
 *                   derives the measurement and removes that kick.
 * @return ezc_status_t Operation status.
 * @retval EZC_OK The source was stored.
 * @retval EZC_ERR_INVALID_ARG ctx is NULL or source is not a known value; the
 *                             context is left unchanged.
 *
 * @note `prev_error` and `prev_measure` are both tracked on every step, so
 *       switching the source between steps does not add a transient.
 */
ezc_status_t ezc_pid_f32_set_derivative_source(ezc_pid_f32_t *ctx,
                                               ezc_pid_deriv_source_t source);

/**
 * @brief Enable or disable the float derivative filter.
 *
 * @param[in,out] ctx Controller context. Must not be NULL.
 * @param[in] deriv_filter_t Filter time constant in seconds, greater than zero
 *                           to filter the derivative term, or zero to pass the
 *                           raw derivative through.
 * @return ezc_status_t Operation status.
 * @retval EZC_OK The time constant was stored.
 * @retval EZC_ERR_INVALID_ARG ctx is NULL, or deriv_filter_t is negative or not
 *                             comparable; the context is left unchanged.
 *
 * @note While the filter is off, its state keeps tracking the raw derivative,
 *       so enabling it later starts from the current derivative instead of
 *       from zero.
 */
ezc_status_t ezc_pid_f32_set_derivative_filter(ezc_pid_f32_t *ctx, float deriv_filter_t);

/**
 * @brief Enable the float integral limit and set its range.
 *
 * @param[in,out] ctx Controller context. Must not be NULL.
 * @param[in] integral_min Lower integral limit.
 * @param[in] integral_max Upper integral limit; must be greater than or equal
 *                         to @p integral_min, otherwise the call is rejected.
 * @return ezc_status_t Operation status.
 * @retval EZC_OK The integral limit is enabled with the given range.
 * @retval EZC_ERR_INVALID_ARG ctx is NULL or the range is invalid; the context
 *                             is left unchanged.
 *
 * @note The limit bounds the integral term itself and works next to the output
 *       limit and anti-windup. A state that is already outside the range, for
 *       example after the limits were changed at run time, is clamped on the
 *       next integrating step.
 */
ezc_status_t ezc_pid_f32_set_integral_limit(ezc_pid_f32_t *ctx, float integral_min,
                                            float integral_max);

/**
 * @brief Disable the float integral limit.
 *
 * @param[in,out] ctx Controller context. Must not be NULL.
 * @return ezc_status_t Operation status.
 * @retval EZC_OK The limit is disabled; `integral_min` and `integral_max` are
 *               retained but ignored until it is enabled again.
 * @retval EZC_ERR_INVALID_ARG ctx is NULL; the context is left unchanged.
 */
ezc_status_t ezc_pid_f32_clear_integral_limit(ezc_pid_f32_t *ctx);

/**
 * @brief Enable float integral separation and set its threshold.
 *
 * @param[in,out] ctx Controller context. Must not be NULL.
 * @param[in] threshold Error magnitude above which integration stops; must be
 *                      greater than zero, otherwise the call is rejected.
 * @return ezc_status_t Operation status.
 * @retval EZC_OK Integral separation is enabled with the given threshold.
 * @retval EZC_ERR_INVALID_ARG ctx is NULL or threshold is not greater than
 *                             zero; the context is left unchanged.
 *
 * @note While `|e| > threshold` the integral keeps its value and the output
 *       follows the proportional and derivative terms only. Integration
 *       resumes as soon as the error is inside the threshold, which avoids
 *       wind-up during large setpoint steps.
 */
ezc_status_t ezc_pid_f32_set_integral_separation(ezc_pid_f32_t *ctx, float threshold);

/**
 * @brief Disable float integral separation.
 *
 * @param[in,out] ctx Controller context. Must not be NULL.
 * @return ezc_status_t Operation status.
 * @retval EZC_OK Integration runs again on every step.
 * @retval EZC_ERR_INVALID_ARG ctx is NULL; the context is left unchanged.
 */
ezc_status_t ezc_pid_f32_clear_integral_separation(ezc_pid_f32_t *ctx);

/**
 * @brief Set the float setpoint weight of the proportional term.
 *
 * @param[in,out] ctx Controller context. Must not be NULL.
 * @param[in] weight Setpoint weight, `0` to `1`; must be inside that range,
 *                   otherwise the call is rejected. `1` stores the default
 *                   behavior (`kp * error`), `0` is proportional on the
 *                   measurement.
 * @return ezc_status_t Operation status.
 * @retval EZC_OK The weight is stored and the option is enabled.
 * @retval EZC_ERR_INVALID_ARG ctx is NULL or weight is outside `[0, 1]`; the
 *                             context is left unchanged.
 */
ezc_status_t ezc_pid_f32_set_setpoint_weight(ezc_pid_f32_t *ctx, float weight);

/**
 * @brief Reset the float controller state, keeping configuration and cache.
 *
 * @param[in,out] ctx Controller context. NULL is ignored.
 *
 * @post `integral`, `prev_error`, `prev_measure` and `d_filtered` are zero, so
 *       the next update step behaves like the first step after initialization.
 *       Gains, limits, options, callback and the cached sample period are
 *       kept.
 * @note Use this after the controller was held in saturation or when the
 *       control loop is re-enabled, so stale state is not reused.
 */
void ezc_pid_f32_reset(ezc_pid_f32_t *ctx);

/**
 * @brief Run one float PID update step.
 *
 * Computes the output from @p setpoint, @p measure and the elapsed time @p dt,
 * updates the controller state, applies the optional features, the output limit
 * and anti-windup as configured, invokes the callback if one is installed, and
 * returns the output. The result that the callback receives is the same value
 * that is returned here.
 *
 * @param[in,out] ctx Controller context. Must not be NULL for a meaningful
 *                    result.
 * @param[in] setpoint Target value of this update step.
 * @param[in] measure Measured value of this update step.
 * @param[in] dt Elapsed time since the previous update step, in seconds. Its
 *               reciprocal is cached in the context and reused while the same
 *               value is passed again.
 * @return float Controller output `u(n)` after clamping.
 * @retval 0.0 ctx is NULL.
 *
 * @pre @p ctx was initialized by @ref ezc_pid_f32_init or zero-initialized.
 * @post `integral`, `prev_error`, `prev_measure`, `d_filtered` and the cached
 *       reciprocal hold the state of this step, and the callback has been
 *       invoked when one is installed.
 * @note A @p dt that is not a finite positive number is a zero-time step: the
 *       integral does not advance and the derivative term is zero, while
 *       `prev_error` still tracks @p measure so the next valid step sees the
 *       correct error change.
 * @note Not reentrant and not interrupt safe with respect to the same context:
 *       serialize calls that share one context. Calls on different contexts are
 *       independent.
 * @warning Non-finite inputs are not rejected: they propagate into the state
 *          and the output, and keep affecting later results until the context
 *          is reset. Validate sensor data before it reaches the controller.
 */
float ezc_pid_f32_update(ezc_pid_f32_t *ctx, float setpoint, float measure, float dt);

/**
 * @brief Result of a single double PID update step.
 *
 * The structure reports the inputs that were used, the terms that were
 * computed, and the output that was returned to the caller. It is produced by
 * @ref ezc_pid_f64_update and handed to @ref ezc_pid_f64_cb_t. When `limited`
 * is true, the returned output sits exactly on the limit it crossed.
 */
typedef struct ezc_pid_f64_result_s {
    double output;   /**< Output after clamping; equal to the update return value. */
    double setpoint; /**< Setpoint of this update step. */
    double measure;  /**< Measurement of this update step. */
    double error;    /**< Error `e = setpoint - measure`. */
    double p_term;   /**< Proportional term, `kp * e` or `kp * (b * setpoint - measure)`. */
    double i_term;   /**< Integral term `I(n)` after separation, limiting and anti-windup. */
    double d_term;   /**< Derivative term that was applied, including the optional filter. */
    bool limited;    /**< true when the unclamped output crossed a limit. */
} ezc_pid_f64_result_t;

/**
 * @brief Optional callback invoked after every completed double PID update.
 *
 * @param[in] user_ctx Callback context stored in the controller context; it is
 *                     the `callback_ctx` value passed to
 *                     @ref ezc_pid_f64_init or @ref ezc_pid_f64_set_callback.
 * @param[in] result   Result of the completed update step. The pointer stays
 *                     valid for the duration of the call only.
 */
typedef void (*ezc_pid_f64_cb_t)(void *user_ctx, const ezc_pid_f64_result_t *result);

/**
 * @brief double PID controller context.
 *
 * Gains, limits, options and callback are configuration; `integral`,
 * `prev_error`, `prev_measure` and `d_filtered` are controller state owned by
 * @ref ezc_pid_f64_update, while `dt_cached` and `dt_inv_cached` only memoize
 * the caller's sample period. Initialize the context with
 * @ref ezc_pid_f64_init, or zero-initialize it for a disabled controller.
 */
typedef struct ezc_pid_f64_s {
    double kp;                 /**< Proportional gain. */
    double ki;                 /**< Integral gain per second. */
    double kd;                 /**< Derivative gain per second. */
    uint32_t options;          /**< Optional features, see `EZC_PID_OPT_*`. */
    ezc_pid_deriv_source_t deriv_source; /**< Derivative source. */
    double deriv_filter_t;     /**< Derivative filter time constant in seconds; 0 disables it. */
    double weight_p;           /**< Setpoint weight of the proportional term; 0..1. */
    double integral_min;       /**< Lower integral limit. */
    double integral_max;       /**< Upper integral limit. */
    double integral_separation; /**< Error magnitude above which integration stops. */
    double integral;           /**< Integral term state `I(n-1)`. */
    double prev_error;         /**< Error state `e(n-1)`. */
    double prev_measure;       /**< Measurement state `measure(n-1)`. */
    double d_filtered;         /**< Derivative filter state; follows the raw derivative when off. */
    double dt_cached;          /**< Sample period whose reciprocal is cached; 0 means none. */
    double dt_inv_cached;      /**< Cached `1 / dt_cached`, reused while dt is unchanged. */
    double out_min;            /**< Lower output limit; used while limit_enabled. */
    double out_max;            /**< Upper output limit; used while limit_enabled. */
    bool limit_enabled;        /**< Clamp the output to `[out_min, out_max]`. */
    bool anti_windup_enabled;  /**< Hold the integral at the saturation boundary. */
    ezc_pid_f64_cb_t callback; /**< Optional update callback; NULL disables it. */
    void *callback_ctx;        /**< Context pointer passed to callback. */
} ezc_pid_f64_t;

/**
 * @brief Initialization parameters of a double PID controller.
 *
 * Value-initialize the structure (`{0}`) to get a disabled controller and
 * enable only the parts the application needs.
 */
typedef struct ezc_pid_f64_cfg_s {
    double kp;                 /**< Proportional gain. */
    double ki;                 /**< Integral gain per second. */
    double kd;                 /**< Derivative gain per second. */
    bool limit_enabled;        /**< Enable output clamping. */
    double out_min;            /**< Lower output limit; must be <= out_max while limit_enabled. */
    double out_max;            /**< Upper output limit; must be >= out_min while limit_enabled. */
    bool anti_windup_enabled;  /**< Enable anti-windup; only effective with limit_enabled. */
    uint32_t options;          /**< Optional features, see `EZC_PID_OPT_*`. */
    ezc_pid_deriv_source_t deriv_source; /**< Derivative source. */
    double deriv_filter_t;     /**< Derivative filter time constant in seconds; 0 disables it. */
    double weight_p;           /**< Setpoint weight of the proportional term; 0..1. */
    double integral_min;       /**< Lower integral limit. */
    double integral_max;       /**< Upper integral limit. */
    double integral_separation; /**< Error magnitude above which integration stops. */
    ezc_pid_f64_cb_t callback; /**< Optional update callback; NULL disables it. */
    void *callback_ctx;        /**< Context pointer passed to callback. */
} ezc_pid_f64_cfg_t;

/**
 * @brief Initialize a double PID controller.
 *
 * The whole context is overwritten: gains, limits, options, callback,
 * controller state and the cached sample period are taken from @p cfg.
 *
 * @param[out] ctx Controller context to initialize. Must not be NULL.
 * @param[in]  cfg Initialization parameters. Must not be NULL. Every enabled
 *                 option is validated: `cfg->out_min` must not exceed
 *                 `cfg->out_max` while `limit_enabled` is set, the integral
 *                 limits must not be inverted while
 *                 `EZC_PID_OPT_INTEGRAL_LIMIT` is set, `integral_separation`
 *                 must be greater than zero while
 *                 `EZC_PID_OPT_INTEGRAL_SEPARATION` is set, `weight_p` must be
 *                 inside `[0, 1]` while `EZC_PID_OPT_SETPOINT_WEIGHT` is set,
 *                 `deriv_filter_t` must not be negative and `deriv_source`
 *                 must be a known value. Ranges that are not comparable (for
 *                 example NaN limits) are rejected as well.
 * @return ezc_status_t Operation status.
 * @retval EZC_OK The context was configured from @p cfg.
 * @retval EZC_ERR_INVALID_ARG ctx or cfg is NULL, or an enabled option, limit
 *                             or filter parameter is out of range. The context
 *                             is zeroed and stays usable as a disabled
 *                             controller.
 *
 * @pre @p ctx references writable storage that outlives its use as a controller
 *      context.
 * @post The controller state, the derivative filter state and the sample
 *       period cache are cleared: the first update step starts from
 *       `I(0) = e(0) = D(0) = 0`.
 * @note Not reentrant: do not initialize the same context concurrently.
 */
ezc_status_t ezc_pid_f64_init(ezc_pid_f64_t *ctx, const ezc_pid_f64_cfg_t *cfg);

/**
 * @brief Change the double PID gains without touching the controller state.
 *
 * @param[in,out] ctx Controller context. Must not be NULL.
 * @param[in] kp Proportional gain.
 * @param[in] ki Integral gain per second.
 * @param[in] kd Derivative gain per second.
 * @return ezc_status_t Operation status.
 * @retval EZC_OK The gains were stored.
 * @retval EZC_ERR_INVALID_ARG ctx is NULL; the context is left unchanged.
 *
 * @note Gains may be changed between update steps, for example by a tuning
 *       command. The integral and error state are preserved.
 */
ezc_status_t ezc_pid_f64_set_gains(ezc_pid_f64_t *ctx, double kp, double ki, double kd);

/**
 * @brief Enable double output clamping and set its range.
 *
 * @param[in,out] ctx Controller context. Must not be NULL.
 * @param[in] out_min Lower output limit.
 * @param[in] out_max Upper output limit; must be greater than or equal to
 *                    @p out_min, otherwise the call is rejected. A range that
 *                    is not comparable (for example NaN limits) is rejected
 *                    as well.
 * @return ezc_status_t Operation status.
 * @retval EZC_OK Clamping is enabled with the given range.
 * @retval EZC_ERR_INVALID_ARG ctx is NULL or the range is invalid; the context
 *                             is left unchanged.
 *
 * @note An equal range is valid and pins the output to that value.
 * @note Enabling clamping does not clear the controller state. Resetting the
 *       controller after a large setpoint change remains the caller's choice.
 */
ezc_status_t ezc_pid_f64_set_limit(ezc_pid_f64_t *ctx, double out_min, double out_max);

/**
 * @brief Disable double output clamping.
 *
 * @param[in,out] ctx Controller context. Must not be NULL.
 * @return ezc_status_t Operation status.
 * @retval EZC_OK Clamping is disabled; `out_min` and `out_max` are retained but
 *               ignored until clamping is enabled again.
 * @retval EZC_ERR_INVALID_ARG ctx is NULL; the context is left unchanged.
 *
 * @note Disabling clamping also disables the effect of anti-windup, which needs
 *       a limit to detect saturation.
 */
ezc_status_t ezc_pid_f64_clear_limit(ezc_pid_f64_t *ctx);

/**
 * @brief Enable or disable double anti-windup.
 *
 * @param[in,out] ctx Controller context. Must not be NULL.
 * @param[in] enable true to enable anti-windup, false to disable it.
 * @return ezc_status_t Operation status.
 * @retval EZC_OK The requested mode was stored.
 * @retval EZC_ERR_INVALID_ARG ctx is NULL; the context is left unchanged.
 *
 * @note The setting is stored independently of the output limit and takes
 *       effect once clamping is enabled.
 */
ezc_status_t ezc_pid_f64_set_anti_windup(ezc_pid_f64_t *ctx, bool enable);

/**
 * @brief Install or remove the double update callback.
 *
 * @param[in,out] ctx Controller context. Must not be NULL.
 * @param[in] callback Function invoked after each completed update step, or
 *                     NULL to remove the current callback.
 * @param[in] user_ctx Context pointer handed to @p callback. May be NULL, and
 *                     may differ per callback installation.
 * @return ezc_status_t Operation status.
 * @retval EZC_OK The callback and its context were stored.
 * @retval EZC_ERR_INVALID_ARG ctx is NULL; the context is left unchanged.
 *
 * @note The callback receives the same output that the update call returns. It
 *       runs in the update caller's context and must not call back into the
 *       same controller context.
 */
ezc_status_t ezc_pid_f64_set_callback(ezc_pid_f64_t *ctx, ezc_pid_f64_cb_t callback,
                                      void *user_ctx);

/**
 * @brief Select which signal feeds the double derivative term.
 *
 * @param[in,out] ctx Controller context. Must not be NULL.
 * @param[in] source EZC_PID_DERIV_ON_ERROR (default) derives the error and adds
 *                   a kick on setpoint steps, EZC_PID_DERIV_ON_MEASUREMENT
 *                   derives the measurement and removes that kick.
 * @return ezc_status_t Operation status.
 * @retval EZC_OK The source was stored.
 * @retval EZC_ERR_INVALID_ARG ctx is NULL or source is not a known value; the
 *                             context is left unchanged.
 *
 * @note `prev_error` and `prev_measure` are both tracked on every step, so
 *       switching the source between steps does not add a transient.
 */
ezc_status_t ezc_pid_f64_set_derivative_source(ezc_pid_f64_t *ctx,
                                               ezc_pid_deriv_source_t source);

/**
 * @brief Enable or disable the double derivative filter.
 *
 * @param[in,out] ctx Controller context. Must not be NULL.
 * @param[in] deriv_filter_t Filter time constant in seconds, greater than zero
 *                           to filter the derivative term, or zero to pass the
 *                           raw derivative through.
 * @return ezc_status_t Operation status.
 * @retval EZC_OK The time constant was stored.
 * @retval EZC_ERR_INVALID_ARG ctx is NULL, or deriv_filter_t is negative or not
 *                             comparable; the context is left unchanged.
 *
 * @note While the filter is off, its state keeps tracking the raw derivative,
 *       so enabling it later starts from the current derivative instead of
 *       from zero.
 */
ezc_status_t ezc_pid_f64_set_derivative_filter(ezc_pid_f64_t *ctx, double deriv_filter_t);

/**
 * @brief Enable the double integral limit and set its range.
 *
 * @param[in,out] ctx Controller context. Must not be NULL.
 * @param[in] integral_min Lower integral limit.
 * @param[in] integral_max Upper integral limit; must be greater than or equal
 *                         to @p integral_min, otherwise the call is rejected.
 * @return ezc_status_t Operation status.
 * @retval EZC_OK The integral limit is enabled with the given range.
 * @retval EZC_ERR_INVALID_ARG ctx is NULL or the range is invalid; the context
 *                             is left unchanged.
 *
 * @note The limit bounds the integral term itself and works next to the output
 *       limit and anti-windup. A state that is already outside the range, for
 *       example after the limits were changed at run time, is clamped on the
 *       next integrating step.
 */
ezc_status_t ezc_pid_f64_set_integral_limit(ezc_pid_f64_t *ctx, double integral_min,
                                            double integral_max);

/**
 * @brief Disable the double integral limit.
 *
 * @param[in,out] ctx Controller context. Must not be NULL.
 * @return ezc_status_t Operation status.
 * @retval EZC_OK The limit is disabled; `integral_min` and `integral_max` are
 *               retained but ignored until it is enabled again.
 * @retval EZC_ERR_INVALID_ARG ctx is NULL; the context is left unchanged.
 */
ezc_status_t ezc_pid_f64_clear_integral_limit(ezc_pid_f64_t *ctx);

/**
 * @brief Enable double integral separation and set its threshold.
 *
 * @param[in,out] ctx Controller context. Must not be NULL.
 * @param[in] threshold Error magnitude above which integration stops; must be
 *                      greater than zero, otherwise the call is rejected.
 * @return ezc_status_t Operation status.
 * @retval EZC_OK Integral separation is enabled with the given threshold.
 * @retval EZC_ERR_INVALID_ARG ctx is NULL or threshold is not greater than
 *                             zero; the context is left unchanged.
 *
 * @note While `|e| > threshold` the integral keeps its value and the output
 *       follows the proportional and derivative terms only. Integration
 *       resumes as soon as the error is inside the threshold, which avoids
 *       wind-up during large setpoint steps.
 */
ezc_status_t ezc_pid_f64_set_integral_separation(ezc_pid_f64_t *ctx, double threshold);

/**
 * @brief Disable double integral separation.
 *
 * @param[in,out] ctx Controller context. Must not be NULL.
 * @return ezc_status_t Operation status.
 * @retval EZC_OK Integration runs again on every step.
 * @retval EZC_ERR_INVALID_ARG ctx is NULL; the context is left unchanged.
 */
ezc_status_t ezc_pid_f64_clear_integral_separation(ezc_pid_f64_t *ctx);

/**
 * @brief Set the double setpoint weight of the proportional term.
 *
 * @param[in,out] ctx Controller context. Must not be NULL.
 * @param[in] weight Setpoint weight, `0` to `1`; must be inside that range,
 *                   otherwise the call is rejected. `1` stores the default
 *                   behavior (`kp * error`), `0` is proportional on the
 *                   measurement.
 * @return ezc_status_t Operation status.
 * @retval EZC_OK The weight is stored and the option is enabled.
 * @retval EZC_ERR_INVALID_ARG ctx is NULL or weight is outside `[0, 1]`; the
 *                             context is left unchanged.
 */
ezc_status_t ezc_pid_f64_set_setpoint_weight(ezc_pid_f64_t *ctx, double weight);

/**
 * @brief Reset the double controller state, keeping configuration and cache.
 *
 * @param[in,out] ctx Controller context. NULL is ignored.
 *
 * @post `integral`, `prev_error`, `prev_measure` and `d_filtered` are zero, so
 *       the next update step behaves like the first step after initialization.
 *       Gains, limits, options, callback and the cached sample period are
 *       kept.
 * @note Use this after the controller was held in saturation or when the
 *       control loop is re-enabled, so stale state is not reused.
 */
void ezc_pid_f64_reset(ezc_pid_f64_t *ctx);

/**
 * @brief Run one double PID update step.
 *
 * Computes the output from @p setpoint, @p measure and the elapsed time @p dt,
 * updates the controller state, applies the optional features, the output limit
 * and anti-windup as configured, invokes the callback if one is installed, and
 * returns the output. The result that the callback receives is the same value
 * that is returned here.
 *
 * @param[in,out] ctx Controller context. Must not be NULL for a meaningful
 *                    result.
 * @param[in] setpoint Target value of this update step.
 * @param[in] measure Measured value of this update step.
 * @param[in] dt Elapsed time since the previous update step, in seconds. Its
 *               reciprocal is cached in the context and reused while the same
 *               value is passed again.
 * @return double Controller output `u(n)` after clamping.
 * @retval 0.0 ctx is NULL.
 *
 * @pre @p ctx was initialized by @ref ezc_pid_f64_init or zero-initialized.
 * @post `integral`, `prev_error`, `prev_measure`, `d_filtered` and the cached
 *       reciprocal hold the state of this step, and the callback has been
 *       invoked when one is installed.
 * @note A @p dt that is not a finite positive number is a zero-time step: the
 *       integral does not advance and the derivative term is zero, while
 *       `prev_error` still tracks @p measure so the next valid step sees the
 *       correct error change.
 * @note Not reentrant and not interrupt safe with respect to the same context:
 *       serialize calls that share one context. Calls on different contexts are
 *       independent.
 * @warning Non-finite inputs are not rejected: they propagate into the state
 *          and the output, and keep affecting later results until the context
 *          is reset. Validate sensor data before it reaches the controller.
 */
double ezc_pid_f64_update(ezc_pid_f64_t *ctx, double setpoint, double measure, double dt);

#ifdef __cplusplus
}
#endif /* __cplusplus */
#endif /* __EZCTRL_PID_H__ */
