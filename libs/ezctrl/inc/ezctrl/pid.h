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
 *     P(n) = kp * e(n)
 *     I(n) = I(n-1) + ki * e(n) * dt
 *     D(n) = kd * (e(n) - e(n-1)) / dt
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
 *   derivative term is zero, `prev_error` still tracks the measurement of this
 *   step, and the returned output is the proportional term plus the unchanged
 *   integral, clamped and reported as usual.
 * - The derivative term is derived from the error, so a setpoint step also
 *   produces a derivative kick. Use `kd = 0` when the setpoint changes in
 *   steps and no derivative action is wanted.
 * - The first step after initialization or reset uses `e(n-1) = 0`, so it
 *   contributes `kd * e(0) / dt` to the output.
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
 * A context must be initialized by the matching `*_init()` entry point before
 * it is used. A zero-initialized context is a valid disabled controller (all
 * gains zero, no limit, no callback) and produces a zero output.
 */
#ifndef __EZCTRL_PID_H__
#define __EZCTRL_PID_H__

#include "ezctrl/common.h"

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

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
    float p_term;   /**< Proportional term `kp * e`. */
    float i_term;   /**< Integral term `I(n)` after the anti-windup decision. */
    float d_term;   /**< Derivative term `kd * (e - e(n-1))`. */
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
 * Gains, limits, callback and the cached sample period are configuration;
 * `integral` and `prev_error` are controller state owned by
 * @ref ezc_pid_f32_update, while `dt_cached` and `dt_inv_cached` only memoize
 * the caller's sample period. Initialize the context with
 * @ref ezc_pid_f32_init, or zero-initialize it for a disabled controller.
 */
typedef struct ezc_pid_f32_s {
    float kp;                 /**< Proportional gain. */
    float ki;                 /**< Integral gain per second. */
    float kd;                 /**< Derivative gain per second. */
    float integral;           /**< Integral term state `I(n-1)`. */
    float prev_error;         /**< Error state `e(n-1)`. */
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
    ezc_pid_f32_cb_t callback;  /**< Optional update callback; NULL disables it. */
    void *callback_ctx;         /**< Context pointer passed to callback. */
} ezc_pid_f32_cfg_t;

/**
 * @brief Initialize a float PID controller.
 *
 * The whole context is overwritten: gains, output limit, anti-windup, callback,
 * controller state and the cached sample period are taken from @p cfg.
 *
 * @param[out] ctx Controller context to initialize. Must not be NULL.
 * @param[in]  cfg Initialization parameters. Must not be NULL. While
 *                 `cfg->limit_enabled` is set, `cfg->out_min` must be less
 *                 than or equal to `cfg->out_max`; a range that is not
 *                 comparable (for example NaN limits) is rejected.
 * @return ezc_status_t Operation status.
 * @retval EZC_OK The context was configured from @p cfg.
 * @retval EZC_ERR_INVALID_ARG ctx or cfg is NULL, or an enabled output limit
 *                             range is inverted. The context is zeroed and
 *                             stays usable as a disabled controller.
 *
 * @pre @p ctx references writable storage that outlives its use as a controller
 *      context.
 * @post The controller state is zeroed and no sample period is cached: the
 *       first update step starts from `I(0) = e(0) = 0`.
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
 * @brief Reset the float controller state, keeping configuration and cache.
 *
 * @param[in,out] ctx Controller context. NULL is ignored.
 *
 * @post `integral` and `prev_error` are zero, so the next update step behaves
 *       like the first step after initialization. Gains, limits, callback and
 *       the cached sample period are kept.
 * @note Use this after the controller was held in saturation or when the
 *       control loop is re-enabled, so stale state is not reused.
 */
void ezc_pid_f32_reset(ezc_pid_f32_t *ctx);

/**
 * @brief Run one float PID update step.
 *
 * Computes the output from @p setpoint, @p measure and the elapsed time @p dt,
 * updates the controller state, applies clamping and anti-windup as
 * configured, invokes the callback if one is installed, and returns the
 * output. The result that the callback receives is the same value that is
 * returned here.
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
 * @post `integral`, `prev_error` and the cached reciprocal hold the state of
 *       this step, and the callback has been invoked when one is installed.
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
    double p_term;   /**< Proportional term `kp * e`. */
    double i_term;   /**< Integral term `I(n)` after the anti-windup decision. */
    double d_term;   /**< Derivative term `kd * (e - e(n-1))`. */
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
 * Gains, limits, callback and the cached sample period are configuration;
 * `integral` and `prev_error` are controller state owned by
 * @ref ezc_pid_f64_update, while `dt_cached` and `dt_inv_cached` only memoize
 * the caller's sample period. Initialize the context with
 * @ref ezc_pid_f64_init, or zero-initialize it for a disabled controller.
 */
typedef struct ezc_pid_f64_s {
    double kp;                 /**< Proportional gain. */
    double ki;                 /**< Integral gain per second. */
    double kd;                 /**< Derivative gain per second. */
    double integral;           /**< Integral term state `I(n-1)`. */
    double prev_error;         /**< Error state `e(n-1)`. */
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
    ezc_pid_f64_cb_t callback; /**< Optional update callback; NULL disables it. */
    void *callback_ctx;        /**< Context pointer passed to callback. */
} ezc_pid_f64_cfg_t;

/**
 * @brief Initialize a double PID controller.
 *
 * The whole context is overwritten: gains, output limit, anti-windup, callback,
 * controller state and the cached sample period are taken from @p cfg.
 *
 * @param[out] ctx Controller context to initialize. Must not be NULL.
 * @param[in]  cfg Initialization parameters. Must not be NULL. While
 *                 `cfg->limit_enabled` is set, `cfg->out_min` must be less
 *                 than or equal to `cfg->out_max`; a range that is not
 *                 comparable (for example NaN limits) is rejected.
 * @return ezc_status_t Operation status.
 * @retval EZC_OK The context was configured from @p cfg.
 * @retval EZC_ERR_INVALID_ARG ctx or cfg is NULL, or an enabled output limit
 *                             range is inverted. The context is zeroed and
 *                             stays usable as a disabled controller.
 *
 * @pre @p ctx references writable storage that outlives its use as a controller
 *      context.
 * @post The controller state is zeroed and no sample period is cached: the
 *       first update step starts from `I(0) = e(0) = 0`.
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
 * @brief Reset the double controller state, keeping configuration and cache.
 *
 * @param[in,out] ctx Controller context. NULL is ignored.
 *
 * @post `integral` and `prev_error` are zero, so the next update step behaves
 *       like the first step after initialization. Gains, limits, callback and
 *       the cached sample period are kept.
 * @note Use this after the controller was held in saturation or when the
 *       control loop is re-enabled, so stale state is not reused.
 */
void ezc_pid_f64_reset(ezc_pid_f64_t *ctx);

/**
 * @brief Run one double PID update step.
 *
 * Computes the output from @p setpoint, @p measure and the elapsed time @p dt,
 * updates the controller state, applies clamping and anti-windup as
 * configured, invokes the callback if one is installed, and returns the
 * output. The result that the callback receives is the same value that is
 * returned here.
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
 * @post `integral`, `prev_error` and the cached reciprocal hold the state of
 *       this step, and the callback has been invoked when one is installed.
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
