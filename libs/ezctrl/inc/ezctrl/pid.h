/**
 * @file pid.h
 * @author Zmmfly
 * @brief Generic PID controller of the ezctrl control library
 * @version 0.2
 * @date 2026-09-29
 *
 * @copyright Copyright (c) 2026, License under GPLv3
 *
 * Scalar variants
 * ---------------
 * The controller is declared once per scalar type, and the type is part of
 * every symbolic name:
 *
 * - `float`: `ezc_pid_f32_*`, computed by `fieldctrl_f32_pid_*`
 * - `double`: `ezc_pid_f64_*`, computed by `fieldctrl_f64_pid_*`
 *
 * Relation to fieldctrl
 * ---------------------
 * ezctrl contains no PID arithmetic of its own. Every update step is one call
 * of the fieldctrl PID declared in `fieldctrl_base.h`, whose file comment is
 * the normative definition of the step. This layer adds what fieldctrl leaves
 * to its callers: a zero-initialized context that works without any
 * configuration, an optional output limit, setters that change one parameter
 * at a time, an update callback and a plain scalar return value.
 *
 * Update semantics
 * ----------------
 * One update step `n` computes, with `b = weight_p` while
 * `EZC_PID_OPT_SETPOINT_WEIGHT` is set and `b = 1` otherwise:
 *
 *     e(n) = setpoint - measure
 *     P(n) = kp * (b * setpoint - measure)
 *     I(n) = I(n-1) + ki * dt * e(n), held inside the integral bounds
 *     D(n) = kd * (e(n) - e(n-1)) / dt              (derivative on error)
 *     D(n) = kd * (measure(n-1) - measure(n)) / dt  (derivative on measurement)
 *     u(n) = P(n) + I(n) + D(n), clamped to the output range
 *
 * - `kp`, `ki` and `kd` share one sign: all nonnegative for a direct-acting
 *   loop (more output raises the measurement, for example heating), all
 *   nonpositive for a reverse-acting loop (more output lowers it, for example
 *   cooling). Mixed signs are rejected, because the terms would then work
 *   against each other.
 * - `dt` is the elapsed time in seconds since the previous step and is passed
 *   to every step, so one configuration serves constant, jittering and
 *   event-driven sample rates. `ki` is the integral gain per second and `kd`
 *   the derivative gain in seconds. fieldctrl caches `ki * dt`, `kd / dt` and
 *   the filter weight and recomputes them only when `dt` changes.
 * - The first step after initialization, reset or a configuration change only
 *   records `e(n)` and `measure(n)`: its derivative term is zero, so neither a
 *   nonzero start value nor a reconfiguration causes a derivative kick.
 * - The output limit is optional. While it is disabled the output range is the
 *   whole finite range of the scalar type (`[-FLT_MAX, FLT_MAX]` or
 *   `[-DBL_MAX, DBL_MAX]`).
 * - The integral bounds are the output range, or `[integral_min,
 *   integral_max]` while `EZC_PID_OPT_INTEGRAL_LIMIT` is set.
 * - Anti-windup is always active: when `P + I + D` crosses an output limit,
 *   the integral advances at most to the value that puts the unclamped output
 *   exactly on that limit, and never moves back past `I(n-1)`.
 * - A step is rejected when `dt` is not a finite positive number, when the
 *   setpoint or the measurement is not finite, or when an intermediate value
 *   overflows. A rejected step changes no state, does not invoke the callback
 *   and returns the output of the last successful step (zero before the first
 *   one).
 * - The callback is optional. It receives the callback context pointer and the
 *   full result of every successful step, including the returned output.
 *
 * Configuration
 * -------------
 * The configuration in effect is a whole `ezc_pid_*_cfg_t` stored in the
 * context: `*_init()` replaces it and each setter changes one part of it. The
 * fieldctrl tuning is immutable, so every change except the callback rebuilds
 * the fieldctrl controller: the integral is kept, clamped into the new integral
 * bounds, while the derivative history and the `dt` cache restart. A rejected
 * change leaves the whole context unchanged.
 *
 * Optional features
 * -----------------
 * Every optional feature is off in a zero-initialized configuration, so
 * enabling one never changes behavior implicitly:
 *
 * - Setpoint weight (`EZC_PID_OPT_SETPOINT_WEIGHT`): the proportional term uses
 *   `b * setpoint - measure` with `b` in `[0, 1]`. `b = 0` is
 *   proportional-on-measurement, `b = 1` restores the default term.
 * - Derivative source (@ref ezc_pid_deriv_source_t): the derivative is computed
 *   from the error by default, or from the measurement to remove the setpoint
 *   derivative kick.
 * - Derivative filter (`deriv_filter_t` greater than zero): the derivative term
 *   passes a first-order low pass, discretized as `D += a * (D_raw - D)` with
 *   `a = dt / (deriv_filter_t + dt)`. Zero passes the raw derivative.
 * - Integral limit (`EZC_PID_OPT_INTEGRAL_LIMIT`): the integral term is held
 *   inside `[integral_min, integral_max]` instead of the output range.
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
 * ezc_pid_f32_set_gains(&pid, 1.0f, 0.5f, 0.0f); // kp, ki per second, kd in seconds
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
 * `test/compile/consumer.c` for plain C consumption. A complete runnable
 * program is checked in at `libs/ezctrl/examples/pid_c.c`, with its C++
 * counterpart next to it; the files under `examples/` are deliberately not part
 * of the build.
 */
#ifndef __EZCTRL_PID_H__
#define __EZCTRL_PID_H__

#include "ezctrl/common.h"
#include "fieldctrl_base.h"

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/**
 * @brief Derivative source of the PID derivative term.
 *
 * The values are those of the matching `fieldctrl_pid_derivative_t`
 * enumerators.
 */
typedef enum ezc_pid_deriv_source_e {
    EZC_PID_DERIV_ON_ERROR = FIELDCTRL_PID_DERIVATIVE_ON_ERROR, /**< Derive the error; setpoint steps kick it. */
    EZC_PID_DERIV_ON_MEASUREMENT = FIELDCTRL_PID_DERIVATIVE_ON_MEASUREMENT, /**< Derive the measurement; setpoint steps do not. */
} ezc_pid_deriv_source_t;

/**
 * @brief Optional feature flags of the configuration `options` field.
 *
 * The flags are the fieldctrl `FIELDCTRL_PID_OPT_*` bits and are passed to
 * fieldctrl unchanged; any other bit is rejected. A zero-initialized
 * configuration has none of them set.
 */
#define EZC_PID_OPT_INTEGRAL_LIMIT      FIELDCTRL_PID_OPT_INTEGRAL_LIMIT      /**< Bound the integral by its own limits. */
#define EZC_PID_OPT_INTEGRAL_SEPARATION FIELDCTRL_PID_OPT_INTEGRAL_SEPARATION /**< Suspend integration above a |e| threshold. */
#define EZC_PID_OPT_SETPOINT_WEIGHT     FIELDCTRL_PID_OPT_SETPOINT_WEIGHT     /**< Use weight_p in the proportional term. */

/**
 * @brief Result of a single successful float PID update step.
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
    float i_term;   /**< Integral term `I(n)` after separation, bounds and anti-windup. */
    float d_term;   /**< Derivative term that was applied, including the optional filter. */
    bool limited;   /**< true when the unclamped output crossed a limit. */
} ezc_pid_f32_result_t;

/**
 * @brief Optional callback invoked after every successful float PID update.
 *
 * @param[in] user_ctx Callback context stored in the configuration; it is the
 *                     `callback_ctx` value passed to @ref ezc_pid_f32_init or
 *                     @ref ezc_pid_f32_set_callback.
 * @param[in] result   Result of the completed update step. The pointer stays
 *                     valid for the duration of the call only.
 */
typedef void (*ezc_pid_f32_cb_t)(void *user_ctx, const ezc_pid_f32_result_t *result);

/**
 * @brief Configuration of a float PID controller.
 *
 * Value-initialize the structure (`{0}`) to get a disabled controller and
 * enable only the parts the application needs. Every numeric field that is
 * passed to fieldctrl must be finite, also the parameters of disabled options;
 * `out_min` and `out_max` are passed only while `limit_enabled` is set.
 */
typedef struct ezc_pid_f32_cfg_s {
    float kp;                   /**< Proportional gain; same sign as ki and kd. */
    float ki;                   /**< Integral gain per second; same sign as kp and kd. */
    float kd;                   /**< Derivative gain in seconds; same sign as kp and ki. */
    bool limit_enabled;         /**< Clamp the output to `[out_min, out_max]`. */
    float out_min;              /**< Lower output limit; must be below out_max while limit_enabled. */
    float out_max;              /**< Upper output limit; must be above out_min while limit_enabled. */
    uint32_t options;           /**< Optional features, see `EZC_PID_OPT_*`. */
    ezc_pid_deriv_source_t deriv_source; /**< Derivative source. */
    float deriv_filter_t;       /**< Derivative filter time constant in seconds; 0 disables it. */
    float weight_p;             /**< Setpoint weight of the proportional term; 0..1. */
    float integral_min;         /**< Lower integral limit. */
    float integral_max;         /**< Upper integral limit; must not be below integral_min. */
    float integral_separation;  /**< Error magnitude above which integration stops; > 0. */
    ezc_pid_f32_cb_t callback;  /**< Optional update callback; NULL disables it. */
    void *callback_ctx;         /**< Context pointer passed to callback. */
} ezc_pid_f32_cfg_t;

/**
 * @brief float PID controller context.
 *
 * `cfg` is the configuration in effect and `pid` the fieldctrl controller
 * built from it, which owns the integral, the derivative history and the `dt`
 * cache. A zero-initialized context is a valid disabled controller: its `pid`
 * is all zero until the first setter or update step builds it. Read the fields
 * freely, but change them only through the `ezc_pid_f32_*` entry points.
 */
typedef struct ezc_pid_f32_s {
    ezc_pid_f32_cfg_t cfg;   /**< Configuration in effect. */
    fieldctrl_f32_pid_t pid; /**< fieldctrl controller built from cfg; all zero until built. */
    float output;            /**< Output of the last successful update step; 0 before the first. */
} ezc_pid_f32_t;

/**
 * @brief Initialize a float PID controller.
 *
 * The whole context is replaced: the configuration is copied from @p cfg and
 * the fieldctrl controller is built with no derivative history, an empty `dt`
 * cache and an integral of zero clamped into the integral bounds.
 *
 * @param[out] ctx Controller context to initialize. Must not be NULL.
 * @param[in]  cfg Configuration. Must not be NULL. fieldctrl_f32_pid_init()
 *                 validates it: `kp`, `ki` and `kd` must not have mixed
 *                 signs, `deriv_filter_t` must not be negative, `out_min`
 *                 must be below `out_max` while
 *                 `limit_enabled` is set, `options` must contain only
 *                 `EZC_PID_OPT_*` bits, `deriv_source` must be a known value,
 *                 enabled options need `integral_min <= integral_max`,
 *                 `integral_separation > 0` and `weight_p` inside `[0, 1]`,
 *                 and every numeric field passed to fieldctrl must be finite.
 * @return ezc_status_t Operation status.
 * @retval EZC_OK The context was configured from @p cfg.
 * @retval EZC_ERR_INVALID_ARG ctx or cfg is NULL, or @p cfg was rejected. A
 *                             non-NULL ctx is zeroed and stays usable as a
 *                             disabled controller.
 *
 * @pre @p ctx references writable storage that outlives its use as a controller
 *      context.
 * @post `output` is zero and the first update step has no derivative term.
 * @note An output range that excludes zero starts the integral at the nearer
 *       bound, so the first outputs already include that integral.
 * @note Not reentrant: do not initialize the same context concurrently.
 */
ezc_status_t ezc_pid_f32_init(ezc_pid_f32_t *ctx, const ezc_pid_f32_cfg_t *cfg);

/**
 * @brief Change the float PID gains.
 *
 * @param[in,out] ctx Controller context. Must not be NULL.
 * @param[in] kp Proportional gain; finite, with the sign of @p ki and @p kd.
 * @param[in] ki Integral gain per second; finite, with the sign of @p kp and @p kd.
 * @param[in] kd Derivative gain in seconds; finite, with the sign of @p kp and @p ki.
 * @return ezc_status_t Operation status.
 * @retval EZC_OK The gains are in effect.
 * @retval EZC_ERR_INVALID_ARG ctx is NULL, a gain is not finite or the gains have
 *                             mixed signs; the context is left unchanged.
 *
 * @note Rebuilds the fieldctrl controller: the integral is kept, and the next
 *       update step has no derivative term.
 */
ezc_status_t ezc_pid_f32_set_gains(ezc_pid_f32_t *ctx, float kp, float ki, float kd);

/**
 * @brief Enable float output clamping and set its range.
 *
 * @param[in,out] ctx Controller context. Must not be NULL.
 * @param[in] out_min Lower output limit; must be finite.
 * @param[in] out_max Upper output limit; must be finite and strictly greater
 *                    than @p out_min.
 * @return ezc_status_t Operation status.
 * @retval EZC_OK Clamping is enabled with the given range.
 * @retval EZC_ERR_INVALID_ARG ctx is NULL or the range was rejected; the context
 *                             is left unchanged.
 *
 * @note Rebuilds the fieldctrl controller. Without the integral limit option
 *       the integral is clamped into the new range, and anti-windup acts on
 *       it from the next step.
 */
ezc_status_t ezc_pid_f32_set_limit(ezc_pid_f32_t *ctx, float out_min, float out_max);

/**
 * @brief Disable float output clamping.
 *
 * @param[in,out] ctx Controller context. Must not be NULL.
 * @return ezc_status_t Operation status.
 * @retval EZC_OK The output range is `[-FLT_MAX, FLT_MAX]`; `out_min` and
 *               `out_max` are retained in `cfg` but ignored.
 * @retval EZC_ERR_INVALID_ARG ctx is NULL.
 *
 * @note Rebuilds the fieldctrl controller. Without the integral limit option
 *       the integral bounds widen to the same range.
 */
ezc_status_t ezc_pid_f32_clear_limit(ezc_pid_f32_t *ctx);

/**
 * @brief Install or remove the float update callback.
 *
 * @param[in,out] ctx Controller context. Must not be NULL.
 * @param[in] callback Function invoked after each successful update step, or
 *                     NULL to remove the current callback.
 * @param[in] user_ctx Context pointer handed to @p callback. May be NULL, and
 *                     may differ per callback installation.
 * @return ezc_status_t Operation status.
 * @retval EZC_OK The callback and its context were stored.
 * @retval EZC_ERR_INVALID_ARG ctx is NULL; the context is left unchanged.
 *
 * @note Only `cfg` changes; the fieldctrl controller is not rebuilt. The
 *       callback runs in the update caller's context and must not call back
 *       into the same controller context.
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
 * @retval EZC_OK The source is in effect.
 * @retval EZC_ERR_INVALID_ARG ctx is NULL or source is not a known value; the
 *                             context is left unchanged.
 *
 * @note Rebuilds the fieldctrl controller: the next update step has no
 *       derivative term.
 */
ezc_status_t ezc_pid_f32_set_derivative_source(ezc_pid_f32_t *ctx,
                                               ezc_pid_deriv_source_t source);

/**
 * @brief Enable or disable the float derivative filter.
 *
 * @param[in,out] ctx Controller context. Must not be NULL.
 * @param[in] deriv_filter_t Filter time constant in seconds, greater than zero
 *                           to filter the derivative term, or zero to pass the
 *                           raw derivative through. Must be finite.
 * @return ezc_status_t Operation status.
 * @retval EZC_OK The time constant is in effect.
 * @retval EZC_ERR_INVALID_ARG ctx is NULL, or deriv_filter_t is negative or not
 *                             finite; the context is left unchanged.
 *
 * @note Rebuilds the fieldctrl controller: the filter state restarts from zero
 *       and the next update step has no derivative term.
 */
ezc_status_t ezc_pid_f32_set_derivative_filter(ezc_pid_f32_t *ctx, float deriv_filter_t);

/**
 * @brief Enable the float integral limit and set its range.
 *
 * @param[in,out] ctx Controller context. Must not be NULL.
 * @param[in] integral_min Lower integral limit; must be finite.
 * @param[in] integral_max Upper integral limit; must be finite and not below
 *                         @p integral_min.
 * @return ezc_status_t Operation status.
 * @retval EZC_OK The integral limit is enabled with the given range.
 * @retval EZC_ERR_INVALID_ARG ctx is NULL or the range was rejected; the context
 *                             is left unchanged.
 *
 * @note The range replaces the output range as integral bounds and may be wider
 *       or narrower than it. Rebuilding the fieldctrl controller clamps the
 *       current integral into the range immediately.
 */
ezc_status_t ezc_pid_f32_set_integral_limit(ezc_pid_f32_t *ctx, float integral_min,
                                            float integral_max);

/**
 * @brief Disable the float integral limit.
 *
 * @param[in,out] ctx Controller context. Must not be NULL.
 * @return ezc_status_t Operation status.
 * @retval EZC_OK The integral is bounded by the output range again;
 *               `integral_min` and `integral_max` are retained but ignored.
 * @retval EZC_ERR_INVALID_ARG ctx is NULL.
 *
 * @note Rebuilds the fieldctrl controller and clamps the integral into the
 *       output range.
 */
ezc_status_t ezc_pid_f32_clear_integral_limit(ezc_pid_f32_t *ctx);

/**
 * @brief Enable float integral separation and set its threshold.
 *
 * @param[in,out] ctx Controller context. Must not be NULL.
 * @param[in] threshold Error magnitude above which integration stops; must be
 *                      finite and greater than zero.
 * @return ezc_status_t Operation status.
 * @retval EZC_OK Integral separation is enabled with the given threshold.
 * @retval EZC_ERR_INVALID_ARG ctx is NULL or threshold was rejected; the context
 *                             is left unchanged.
 *
 * @note While `|e| > threshold` the integral keeps its value and the output
 *       follows the proportional and derivative terms only. Integration
 *       resumes as soon as the error is inside the threshold.
 */
ezc_status_t ezc_pid_f32_set_integral_separation(ezc_pid_f32_t *ctx, float threshold);

/**
 * @brief Disable float integral separation.
 *
 * @param[in,out] ctx Controller context. Must not be NULL.
 * @return ezc_status_t Operation status.
 * @retval EZC_OK Integration runs again on every step.
 * @retval EZC_ERR_INVALID_ARG ctx is NULL.
 */
ezc_status_t ezc_pid_f32_clear_integral_separation(ezc_pid_f32_t *ctx);

/**
 * @brief Set the float setpoint weight of the proportional term.
 *
 * @param[in,out] ctx Controller context. Must not be NULL.
 * @param[in] weight Setpoint weight inside `[0, 1]`. `1` keeps the default
 *                   term (`kp * error`), `0` is proportional on the
 *                   measurement.
 * @return ezc_status_t Operation status.
 * @retval EZC_OK The weight is stored and the option is enabled.
 * @retval EZC_ERR_INVALID_ARG ctx is NULL or weight is outside `[0, 1]`; the
 *                             context is left unchanged.
 */
ezc_status_t ezc_pid_f32_set_setpoint_weight(ezc_pid_f32_t *ctx, float weight);

/**
 * @brief Restart the float controller, keeping configuration and `dt` cache.
 *
 * @param[in,out] ctx Controller context. NULL, and a zero-initialized context
 *                    that has not been built yet, are ignored.
 *
 * @post The integral is zero clamped into the integral bounds and the
 *       derivative history is cleared, so the next update step behaves like
 *       the first step after initialization. `cfg` and `output` are kept.
 * @note Use this after the controller was held in saturation or when the
 *       control loop is re-enabled, so stale state is not reused.
 */
void ezc_pid_f32_reset(ezc_pid_f32_t *ctx);

/**
 * @brief Run one float PID update step.
 *
 * Builds the fieldctrl controller of a zero-initialized context, advances it by
 * one fieldctrl_f32_pid_step() and, on success, stores the output, invokes the
 * callback if one is installed and returns the output.
 *
 * @param[in,out] ctx Controller context. NULL returns zero.
 * @param[in] setpoint Target value of this update step; must be finite.
 * @param[in] measure Measured value of this update step; must be finite.
 * @param[in] dt Elapsed time since the previous update step, in seconds; must
 *               be finite and positive.
 * @return float Output `u(n)` of this step inside the output range, or
 *         `ctx->output` when the step was rejected.
 * @retval 0.0 ctx is NULL, or the step was rejected before any step succeeded.
 *
 * @pre @p ctx was initialized by @ref ezc_pid_f32_init or zero-initialized.
 * @post A successful step updates the fieldctrl state and `output`, and the
 *       callback has been invoked when one is installed. A rejected step (see
 *       the file comment) changes nothing and invokes no callback.
 * @note Not reentrant and not interrupt safe with respect to the same context:
 *       serialize calls that share one context. Calls on different contexts are
 *       independent.
 * @warning The return value alone does not distinguish a rejected step from a
 *          held output. Validate `dt` and sensor data before the call when the
 *          actuator must not keep the previous command.
 */
float ezc_pid_f32_update(ezc_pid_f32_t *ctx, float setpoint, float measure, float dt);

/**
 * @brief Result of a single successful double PID update step.
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
    double i_term;   /**< Integral term `I(n)` after separation, bounds and anti-windup. */
    double d_term;   /**< Derivative term that was applied, including the optional filter. */
    bool limited;    /**< true when the unclamped output crossed a limit. */
} ezc_pid_f64_result_t;

/**
 * @brief Optional callback invoked after every successful double PID update.
 *
 * @param[in] user_ctx Callback context stored in the configuration; it is the
 *                     `callback_ctx` value passed to @ref ezc_pid_f64_init or
 *                     @ref ezc_pid_f64_set_callback.
 * @param[in] result   Result of the completed update step. The pointer stays
 *                     valid for the duration of the call only.
 */
typedef void (*ezc_pid_f64_cb_t)(void *user_ctx, const ezc_pid_f64_result_t *result);

/**
 * @brief Configuration of a double PID controller.
 *
 * Value-initialize the structure (`{0}`) to get a disabled controller and
 * enable only the parts the application needs. Every numeric field that is
 * passed to fieldctrl must be finite, also the parameters of disabled options;
 * `out_min` and `out_max` are passed only while `limit_enabled` is set.
 */
typedef struct ezc_pid_f64_cfg_s {
    double kp;                 /**< Proportional gain; same sign as ki and kd. */
    double ki;                 /**< Integral gain per second; same sign as kp and kd. */
    double kd;                 /**< Derivative gain in seconds; same sign as kp and ki. */
    bool limit_enabled;        /**< Clamp the output to `[out_min, out_max]`. */
    double out_min;            /**< Lower output limit; must be below out_max while limit_enabled. */
    double out_max;            /**< Upper output limit; must be above out_min while limit_enabled. */
    uint32_t options;          /**< Optional features, see `EZC_PID_OPT_*`. */
    ezc_pid_deriv_source_t deriv_source; /**< Derivative source. */
    double deriv_filter_t;     /**< Derivative filter time constant in seconds; 0 disables it. */
    double weight_p;           /**< Setpoint weight of the proportional term; 0..1. */
    double integral_min;       /**< Lower integral limit. */
    double integral_max;       /**< Upper integral limit; must not be below integral_min. */
    double integral_separation; /**< Error magnitude above which integration stops; > 0. */
    ezc_pid_f64_cb_t callback; /**< Optional update callback; NULL disables it. */
    void *callback_ctx;        /**< Context pointer passed to callback. */
} ezc_pid_f64_cfg_t;

/**
 * @brief double PID controller context.
 *
 * `cfg` is the configuration in effect and `pid` the fieldctrl controller
 * built from it, which owns the integral, the derivative history and the `dt`
 * cache. A zero-initialized context is a valid disabled controller: its `pid`
 * is all zero until the first setter or update step builds it. Read the fields
 * freely, but change them only through the `ezc_pid_f64_*` entry points.
 */
typedef struct ezc_pid_f64_s {
    ezc_pid_f64_cfg_t cfg;   /**< Configuration in effect. */
    fieldctrl_f64_pid_t pid; /**< fieldctrl controller built from cfg; all zero until built. */
    double output;           /**< Output of the last successful update step; 0 before the first. */
} ezc_pid_f64_t;

/**
 * @brief Initialize a double PID controller.
 *
 * The whole context is replaced: the configuration is copied from @p cfg and
 * the fieldctrl controller is built with no derivative history, an empty `dt`
 * cache and an integral of zero clamped into the integral bounds.
 *
 * @param[out] ctx Controller context to initialize. Must not be NULL.
 * @param[in]  cfg Configuration. Must not be NULL. fieldctrl_f64_pid_init()
 *                 validates it: `kp`, `ki` and `kd` must not have mixed
 *                 signs, `deriv_filter_t` must not be negative, `out_min`
 *                 must be below `out_max` while
 *                 `limit_enabled` is set, `options` must contain only
 *                 `EZC_PID_OPT_*` bits, `deriv_source` must be a known value,
 *                 enabled options need `integral_min <= integral_max`,
 *                 `integral_separation > 0` and `weight_p` inside `[0, 1]`,
 *                 and every numeric field passed to fieldctrl must be finite.
 * @return ezc_status_t Operation status.
 * @retval EZC_OK The context was configured from @p cfg.
 * @retval EZC_ERR_INVALID_ARG ctx or cfg is NULL, or @p cfg was rejected. A
 *                             non-NULL ctx is zeroed and stays usable as a
 *                             disabled controller.
 *
 * @pre @p ctx references writable storage that outlives its use as a controller
 *      context.
 * @post `output` is zero and the first update step has no derivative term.
 * @note An output range that excludes zero starts the integral at the nearer
 *       bound, so the first outputs already include that integral.
 * @note Not reentrant: do not initialize the same context concurrently.
 */
ezc_status_t ezc_pid_f64_init(ezc_pid_f64_t *ctx, const ezc_pid_f64_cfg_t *cfg);

/**
 * @brief Change the double PID gains.
 *
 * @param[in,out] ctx Controller context. Must not be NULL.
 * @param[in] kp Proportional gain; finite, with the sign of @p ki and @p kd.
 * @param[in] ki Integral gain per second; finite, with the sign of @p kp and @p kd.
 * @param[in] kd Derivative gain in seconds; finite, with the sign of @p kp and @p ki.
 * @return ezc_status_t Operation status.
 * @retval EZC_OK The gains are in effect.
 * @retval EZC_ERR_INVALID_ARG ctx is NULL, a gain is not finite or the gains have
 *                             mixed signs; the context is left unchanged.
 *
 * @note Rebuilds the fieldctrl controller: the integral is kept, and the next
 *       update step has no derivative term.
 */
ezc_status_t ezc_pid_f64_set_gains(ezc_pid_f64_t *ctx, double kp, double ki, double kd);

/**
 * @brief Enable double output clamping and set its range.
 *
 * @param[in,out] ctx Controller context. Must not be NULL.
 * @param[in] out_min Lower output limit; must be finite.
 * @param[in] out_max Upper output limit; must be finite and strictly greater
 *                    than @p out_min.
 * @return ezc_status_t Operation status.
 * @retval EZC_OK Clamping is enabled with the given range.
 * @retval EZC_ERR_INVALID_ARG ctx is NULL or the range was rejected; the context
 *                             is left unchanged.
 *
 * @note Rebuilds the fieldctrl controller. Without the integral limit option
 *       the integral is clamped into the new range, and anti-windup acts on
 *       it from the next step.
 */
ezc_status_t ezc_pid_f64_set_limit(ezc_pid_f64_t *ctx, double out_min, double out_max);

/**
 * @brief Disable double output clamping.
 *
 * @param[in,out] ctx Controller context. Must not be NULL.
 * @return ezc_status_t Operation status.
 * @retval EZC_OK The output range is `[-DBL_MAX, DBL_MAX]`; `out_min` and
 *               `out_max` are retained in `cfg` but ignored.
 * @retval EZC_ERR_INVALID_ARG ctx is NULL.
 *
 * @note Rebuilds the fieldctrl controller. Without the integral limit option
 *       the integral bounds widen to the same range.
 */
ezc_status_t ezc_pid_f64_clear_limit(ezc_pid_f64_t *ctx);

/**
 * @brief Install or remove the double update callback.
 *
 * @param[in,out] ctx Controller context. Must not be NULL.
 * @param[in] callback Function invoked after each successful update step, or
 *                     NULL to remove the current callback.
 * @param[in] user_ctx Context pointer handed to @p callback. May be NULL, and
 *                     may differ per callback installation.
 * @return ezc_status_t Operation status.
 * @retval EZC_OK The callback and its context were stored.
 * @retval EZC_ERR_INVALID_ARG ctx is NULL; the context is left unchanged.
 *
 * @note Only `cfg` changes; the fieldctrl controller is not rebuilt. The
 *       callback runs in the update caller's context and must not call back
 *       into the same controller context.
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
 * @retval EZC_OK The source is in effect.
 * @retval EZC_ERR_INVALID_ARG ctx is NULL or source is not a known value; the
 *                             context is left unchanged.
 *
 * @note Rebuilds the fieldctrl controller: the next update step has no
 *       derivative term.
 */
ezc_status_t ezc_pid_f64_set_derivative_source(ezc_pid_f64_t *ctx,
                                               ezc_pid_deriv_source_t source);

/**
 * @brief Enable or disable the double derivative filter.
 *
 * @param[in,out] ctx Controller context. Must not be NULL.
 * @param[in] deriv_filter_t Filter time constant in seconds, greater than zero
 *                           to filter the derivative term, or zero to pass the
 *                           raw derivative through. Must be finite.
 * @return ezc_status_t Operation status.
 * @retval EZC_OK The time constant is in effect.
 * @retval EZC_ERR_INVALID_ARG ctx is NULL, or deriv_filter_t is negative or not
 *                             finite; the context is left unchanged.
 *
 * @note Rebuilds the fieldctrl controller: the filter state restarts from zero
 *       and the next update step has no derivative term.
 */
ezc_status_t ezc_pid_f64_set_derivative_filter(ezc_pid_f64_t *ctx, double deriv_filter_t);

/**
 * @brief Enable the double integral limit and set its range.
 *
 * @param[in,out] ctx Controller context. Must not be NULL.
 * @param[in] integral_min Lower integral limit; must be finite.
 * @param[in] integral_max Upper integral limit; must be finite and not below
 *                         @p integral_min.
 * @return ezc_status_t Operation status.
 * @retval EZC_OK The integral limit is enabled with the given range.
 * @retval EZC_ERR_INVALID_ARG ctx is NULL or the range was rejected; the context
 *                             is left unchanged.
 *
 * @note The range replaces the output range as integral bounds and may be wider
 *       or narrower than it. Rebuilding the fieldctrl controller clamps the
 *       current integral into the range immediately.
 */
ezc_status_t ezc_pid_f64_set_integral_limit(ezc_pid_f64_t *ctx, double integral_min,
                                            double integral_max);

/**
 * @brief Disable the double integral limit.
 *
 * @param[in,out] ctx Controller context. Must not be NULL.
 * @return ezc_status_t Operation status.
 * @retval EZC_OK The integral is bounded by the output range again;
 *               `integral_min` and `integral_max` are retained but ignored.
 * @retval EZC_ERR_INVALID_ARG ctx is NULL.
 *
 * @note Rebuilds the fieldctrl controller and clamps the integral into the
 *       output range.
 */
ezc_status_t ezc_pid_f64_clear_integral_limit(ezc_pid_f64_t *ctx);

/**
 * @brief Enable double integral separation and set its threshold.
 *
 * @param[in,out] ctx Controller context. Must not be NULL.
 * @param[in] threshold Error magnitude above which integration stops; must be
 *                      finite and greater than zero.
 * @return ezc_status_t Operation status.
 * @retval EZC_OK Integral separation is enabled with the given threshold.
 * @retval EZC_ERR_INVALID_ARG ctx is NULL or threshold was rejected; the context
 *                             is left unchanged.
 *
 * @note While `|e| > threshold` the integral keeps its value and the output
 *       follows the proportional and derivative terms only. Integration
 *       resumes as soon as the error is inside the threshold.
 */
ezc_status_t ezc_pid_f64_set_integral_separation(ezc_pid_f64_t *ctx, double threshold);

/**
 * @brief Disable double integral separation.
 *
 * @param[in,out] ctx Controller context. Must not be NULL.
 * @return ezc_status_t Operation status.
 * @retval EZC_OK Integration runs again on every step.
 * @retval EZC_ERR_INVALID_ARG ctx is NULL.
 */
ezc_status_t ezc_pid_f64_clear_integral_separation(ezc_pid_f64_t *ctx);

/**
 * @brief Set the double setpoint weight of the proportional term.
 *
 * @param[in,out] ctx Controller context. Must not be NULL.
 * @param[in] weight Setpoint weight inside `[0, 1]`. `1` keeps the default
 *                   term (`kp * error`), `0` is proportional on the
 *                   measurement.
 * @return ezc_status_t Operation status.
 * @retval EZC_OK The weight is stored and the option is enabled.
 * @retval EZC_ERR_INVALID_ARG ctx is NULL or weight is outside `[0, 1]`; the
 *                             context is left unchanged.
 */
ezc_status_t ezc_pid_f64_set_setpoint_weight(ezc_pid_f64_t *ctx, double weight);

/**
 * @brief Restart the double controller, keeping configuration and `dt` cache.
 *
 * @param[in,out] ctx Controller context. NULL, and a zero-initialized context
 *                    that has not been built yet, are ignored.
 *
 * @post The integral is zero clamped into the integral bounds and the
 *       derivative history is cleared, so the next update step behaves like
 *       the first step after initialization. `cfg` and `output` are kept.
 * @note Use this after the controller was held in saturation or when the
 *       control loop is re-enabled, so stale state is not reused.
 */
void ezc_pid_f64_reset(ezc_pid_f64_t *ctx);

/**
 * @brief Run one double PID update step.
 *
 * Builds the fieldctrl controller of a zero-initialized context, advances it by
 * one fieldctrl_f64_pid_step() and, on success, stores the output, invokes the
 * callback if one is installed and returns the output.
 *
 * @param[in,out] ctx Controller context. NULL returns zero.
 * @param[in] setpoint Target value of this update step; must be finite.
 * @param[in] measure Measured value of this update step; must be finite.
 * @param[in] dt Elapsed time since the previous update step, in seconds; must
 *               be finite and positive.
 * @return double Output `u(n)` of this step inside the output range, or
 *         `ctx->output` when the step was rejected.
 * @retval 0.0 ctx is NULL, or the step was rejected before any step succeeded.
 *
 * @pre @p ctx was initialized by @ref ezc_pid_f64_init or zero-initialized.
 * @post A successful step updates the fieldctrl state and `output`, and the
 *       callback has been invoked when one is installed. A rejected step (see
 *       the file comment) changes nothing and invokes no callback.
 * @note Not reentrant and not interrupt safe with respect to the same context:
 *       serialize calls that share one context. Calls on different contexts are
 *       independent.
 * @warning The return value alone does not distinguish a rejected step from a
 *          held output. Validate `dt` and sensor data before the call when the
 *          actuator must not keep the previous command.
 */
double ezc_pid_f64_update(ezc_pid_f64_t *ctx, double setpoint, double measure, double dt);

#ifdef __cplusplus
}
#endif /* __cplusplus */
#endif /* __EZCTRL_PID_H__ */
