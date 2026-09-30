/**
 * @file pid.hpp
 * @author Zmmfly
 * @brief C++ wrapper around the typed PID controller of ezctrl
 * @version 0.1
 * @date 2026-09-29
 *
 * @copyright Copyright (c) 2026, License under GPLv3
 *
 * The wrapper is header-only and holds the C context by value, so it adds no
 * allocation and no virtual dispatch. The shortest form needs no configuration
 * structure at all:
 *
 * @code
 * #include "ezctrl.hpp"
 *
 * ezc::pid<float> pid;              // default-constructed: disabled controller
 * pid.set_params(1.0f, 0.5f, 0.0f); // kp, ki per second, kd per second
 *
 * // once per control period, dt = seconds since the previous call
 * float duty = pid.update(setpoint, measure, dt);
 * @endcode
 *
 * Every setter enables what it configures, so the optional features need
 * neither a bitmask nor a C configuration structure:
 *
 * @code
 * ezc::pid<float> ctrl;
 * ctrl.set_params(2.0f, 0.5f, 0.01f); // kp, ki per second, kd per second
 * ctrl.limit(0.0f, 1.0f);             // clamp the output to the actuator range
 * ctrl.anti_windup(true);
 * ctrl.integral_limit(-1.0f, 1.0f);   // enables EZC_PID_OPT_INTEGRAL_LIMIT
 * ctrl.integral_separation(5.0f);     // enables EZC_PID_OPT_INTEGRAL_SEPARATION
 * ctrl.setpoint_weight(0.5f);         // enables EZC_PID_OPT_SETPOINT_WEIGHT
 * ctrl.derivative_source(EZC_PID_DERIV_ON_MEASUREMENT);
 * ctrl.derivative_filter(0.01f);      // first-order filter, 10 ms
 *
 * float duty = ctrl.update(setpoint, measure, dt_seconds);
 * @endcode
 *
 * Configuration methods are named after what they set (`limit`, `anti_windup`,
 * `integral_limit`, `integral_separation`, `setpoint_weight`,
 * `derivative_source`, `derivative_filter`), except `set_params` and
 * `set_callback`; all of them return `ezc_status_t` like their C counterparts.
 * `init()` is still there for bulk configuration from `ezc_pid_f32_cfg_t` (or
 * its f64 variant), and `context()` exposes the C state when it is needed. The
 * scalar type selects the C API through the @ref ezc::pid_traits specialization
 * for that type.
 *
 * The host test suites are further executable examples: `test/pid_cpp.cc` for
 * the wrapper and `test/compile/headers.cpp` for mixed-language consumption. A
 * complete runnable program is checked in at
 * `libs/ezctrl/examples/pid_cpp.cpp`, with its C counterpart next to it; the
 * files under `examples/` are deliberately not part of the build.
 *
 * Include `ezctrl.hpp` to pull in this layer together with every other C++
 * wrapper of the library.
 *
 * Requires C++11 or later and the C API of the module; no other dependency is
 * added.
 */
#ifndef __EZCTRL_PID_HPP__
#define __EZCTRL_PID_HPP__

#include "ezctrl/pid.h"

namespace ezc {

/**
 * @brief Maps a scalar type to the matching C PID API.
 *
 * @tparam T Scalar type of the controller. Specializations are provided for
 *           `float` and `double`; an unspecialized type fails to compile, which
 *           keeps unsupported types out of the API on purpose.
 */
template <typename T>
struct pid_traits;

/**
 * @brief float specialization of @ref ezc::pid_traits.
 */
template <>
struct pid_traits<float> {
    using value_type = float;               /**< Scalar type of the controller. */
    using context_type = ezc_pid_f32_t;     /**< C context of the controller. */
    using config_type = ezc_pid_f32_cfg_t;  /**< C initialization parameters. */
    using result_type = ezc_pid_f32_result_t; /**< C result of one update step. */
    using callback_type = ezc_pid_f32_cb_t; /**< C update callback type. */

    /**
     * @brief Initialize the controller; forwards to ezc_pid_f32_init().
     *
     * @param[out] ctx Controller context to initialize. Must not be NULL.
     * @param[in] cfg Initialization parameters. Must not be NULL.
     * @return ezc_status_t Result of ezc_pid_f32_init(); a rejected
     *         configuration leaves a zeroed context.
     */
    static ezc_status_t init(context_type *ctx, const config_type *cfg) noexcept {
        return ezc_pid_f32_init(ctx, cfg);
    }

    /**
     * @brief Change the gains; forwards to ezc_pid_f32_set_gains().
     *
     * @param[in,out] ctx Controller context. Must not be NULL.
     * @param[in] kp Proportional gain.
     * @param[in] ki Integral gain applied per sample.
     * @param[in] kd Derivative gain applied per sample.
     * @return ezc_status_t Result of ezc_pid_f32_set_gains(); the controller
     *         state is preserved.
     */
    static ezc_status_t set_gains(context_type *ctx, value_type kp, value_type ki,
                                  value_type kd) noexcept {
        return ezc_pid_f32_set_gains(ctx, kp, ki, kd);
    }

    /**
     * @brief Enable output clamping; forwards to ezc_pid_f32_set_limit().
     *
     * @param[in,out] ctx Controller context. Must not be NULL.
     * @param[in] out_min Lower output limit.
     * @param[in] out_max Upper output limit; must not be below @p out_min.
     * @return ezc_status_t Result of ezc_pid_f32_set_limit().
     */
    static ezc_status_t set_limit(context_type *ctx, value_type out_min,
                                  value_type out_max) noexcept {
        return ezc_pid_f32_set_limit(ctx, out_min, out_max);
    }

    /**
     * @brief Disable output clamping; forwards to ezc_pid_f32_clear_limit().
     *
     * @param[in,out] ctx Controller context. Must not be NULL.
     * @return ezc_status_t Result of ezc_pid_f32_clear_limit().
     */
    static ezc_status_t clear_limit(context_type *ctx) noexcept {
        return ezc_pid_f32_clear_limit(ctx);
    }

    /**
     * @brief Enable or disable anti-windup; forwards to
     *        ezc_pid_f32_set_anti_windup().
     *
     * @param[in,out] ctx Controller context. Must not be NULL.
     * @param[in] enable true to enable anti-windup, false to disable it.
     * @return ezc_status_t Result of ezc_pid_f32_set_anti_windup().
     */
    static ezc_status_t set_anti_windup(context_type *ctx, bool enable) noexcept {
        return ezc_pid_f32_set_anti_windup(ctx, enable);
    }

    /**
     * @brief Install or remove the update callback; forwards to
     *        ezc_pid_f32_set_callback().
     *
     * @param[in,out] ctx Controller context. Must not be NULL.
     * @param[in] callback Function invoked after each completed update step, or
     *                     nullptr to remove the current callback.
     * @param[in] user_ctx Context pointer handed to @p callback; may be
     *                     nullptr.
     * @return ezc_status_t Result of ezc_pid_f32_set_callback().
     */
    static ezc_status_t set_callback(context_type *ctx, callback_type callback,
                                     void *user_ctx) noexcept {
        return ezc_pid_f32_set_callback(ctx, callback, user_ctx);
    }

    /**
     * @brief Select the derivative source; forwards to
     *        ezc_pid_f32_set_derivative_source().
     *
     * @param[in,out] ctx Controller context. Must not be NULL.
     * @param[in] source Derivative source, see @ref ezc_pid_deriv_source_t.
     * @return ezc_status_t Result of ezc_pid_f32_set_derivative_source().
     */
    static ezc_status_t set_derivative_source(context_type *ctx,
                                              ezc_pid_deriv_source_t source) noexcept {
        return ezc_pid_f32_set_derivative_source(ctx, source);
    }

    /**
     * @brief Enable or disable the derivative filter; forwards to
     *        ezc_pid_f32_set_derivative_filter().
     *
     * @param[in,out] ctx Controller context. Must not be NULL.
     * @param[in] deriv_filter_t Filter time constant in seconds; 0 disables it.
     * @return ezc_status_t Result of ezc_pid_f32_set_derivative_filter().
     */
    static ezc_status_t set_derivative_filter(context_type *ctx,
                                              value_type deriv_filter_t) noexcept {
        return ezc_pid_f32_set_derivative_filter(ctx, deriv_filter_t);
    }

    /**
     * @brief Enable the integral limit; forwards to
     *        ezc_pid_f32_set_integral_limit().
     *
     * @param[in,out] ctx Controller context. Must not be NULL.
     * @param[in] integral_min Lower integral limit.
     * @param[in] integral_max Upper integral limit; must not be below the
     *                         lower one.
     * @return ezc_status_t Result of ezc_pid_f32_set_integral_limit().
     */
    static ezc_status_t set_integral_limit(context_type *ctx, value_type integral_min,
                                           value_type integral_max) noexcept {
        return ezc_pid_f32_set_integral_limit(ctx, integral_min, integral_max);
    }

    /**
     * @brief Disable the integral limit; forwards to
     *        ezc_pid_f32_clear_integral_limit().
     *
     * @param[in,out] ctx Controller context. Must not be NULL.
     * @return ezc_status_t Result of ezc_pid_f32_clear_integral_limit().
     */
    static ezc_status_t clear_integral_limit(context_type *ctx) noexcept {
        return ezc_pid_f32_clear_integral_limit(ctx);
    }

    /**
     * @brief Enable integral separation; forwards to
     *        ezc_pid_f32_set_integral_separation().
     *
     * @param[in,out] ctx Controller context. Must not be NULL.
     * @param[in] threshold Error magnitude above which integration stops.
     * @return ezc_status_t Result of ezc_pid_f32_set_integral_separation().
     */
    static ezc_status_t set_integral_separation(context_type *ctx,
                                                value_type threshold) noexcept {
        return ezc_pid_f32_set_integral_separation(ctx, threshold);
    }

    /**
     * @brief Disable integral separation; forwards to
     *        ezc_pid_f32_clear_integral_separation().
     *
     * @param[in,out] ctx Controller context. Must not be NULL.
     * @return ezc_status_t Result of ezc_pid_f32_clear_integral_separation().
     */
    static ezc_status_t clear_integral_separation(context_type *ctx) noexcept {
        return ezc_pid_f32_clear_integral_separation(ctx);
    }

    /**
     * @brief Set the proportional setpoint weight; forwards to
     *        ezc_pid_f32_set_setpoint_weight().
     *
     * @param[in,out] ctx Controller context. Must not be NULL.
     * @param[in] weight Setpoint weight, 0 to 1.
     * @return ezc_status_t Result of ezc_pid_f32_set_setpoint_weight().
     */
    static ezc_status_t set_setpoint_weight(context_type *ctx, value_type weight) noexcept {
        return ezc_pid_f32_set_setpoint_weight(ctx, weight);
    }

    /**
     * @brief Reset the controller state; forwards to ezc_pid_f32_reset().
     *
     * @param[in,out] ctx Controller context. NULL is ignored.
     */
    static void reset(context_type *ctx) noexcept {
        ezc_pid_f32_reset(ctx);
    }

    /**
     * @brief Run one update step; forwards to ezc_pid_f32_update().
     *
     * @param[in,out] ctx Controller context. Must not be NULL.
     * @param[in] setpoint Target value of this update step.
     * @param[in] measure Measured value of this update step.
     * @param[in] dt Elapsed time since the previous update step, in seconds.
     * @return value_type Controller output after clamping; also handed to the
     *         callback when one is installed.
     */
    static value_type update(context_type *ctx, value_type setpoint,
                             value_type measure, value_type dt) noexcept {
        return ezc_pid_f32_update(ctx, setpoint, measure, dt);
    }
};

/**
 * @brief double specialization of @ref ezc::pid_traits.
 */
template <>
struct pid_traits<double> {
    using value_type = double;                /**< Scalar type of the controller. */
    using context_type = ezc_pid_f64_t;       /**< C context of the controller. */
    using config_type = ezc_pid_f64_cfg_t;    /**< C initialization parameters. */
    using result_type = ezc_pid_f64_result_t; /**< C result of one update step. */
    using callback_type = ezc_pid_f64_cb_t;   /**< C update callback type. */

    /**
     * @brief Initialize the controller; forwards to ezc_pid_f64_init().
     *
     * @param[out] ctx Controller context to initialize. Must not be NULL.
     * @param[in] cfg Initialization parameters. Must not be NULL.
     * @return ezc_status_t Result of ezc_pid_f64_init(); a rejected
     *         configuration leaves a zeroed context.
     */
    static ezc_status_t init(context_type *ctx, const config_type *cfg) noexcept {
        return ezc_pid_f64_init(ctx, cfg);
    }

    /**
     * @brief Change the gains; forwards to ezc_pid_f64_set_gains().
     *
     * @param[in,out] ctx Controller context. Must not be NULL.
     * @param[in] kp Proportional gain.
     * @param[in] ki Integral gain applied per sample.
     * @param[in] kd Derivative gain applied per sample.
     * @return ezc_status_t Result of ezc_pid_f64_set_gains(); the controller
     *         state is preserved.
     */
    static ezc_status_t set_gains(context_type *ctx, value_type kp, value_type ki,
                                  value_type kd) noexcept {
        return ezc_pid_f64_set_gains(ctx, kp, ki, kd);
    }

    /**
     * @brief Enable output clamping; forwards to ezc_pid_f64_set_limit().
     *
     * @param[in,out] ctx Controller context. Must not be NULL.
     * @param[in] out_min Lower output limit.
     * @param[in] out_max Upper output limit; must not be below @p out_min.
     * @return ezc_status_t Result of ezc_pid_f64_set_limit().
     */
    static ezc_status_t set_limit(context_type *ctx, value_type out_min,
                                  value_type out_max) noexcept {
        return ezc_pid_f64_set_limit(ctx, out_min, out_max);
    }

    /**
     * @brief Disable output clamping; forwards to ezc_pid_f64_clear_limit().
     *
     * @param[in,out] ctx Controller context. Must not be NULL.
     * @return ezc_status_t Result of ezc_pid_f64_clear_limit().
     */
    static ezc_status_t clear_limit(context_type *ctx) noexcept {
        return ezc_pid_f64_clear_limit(ctx);
    }

    /**
     * @brief Enable or disable anti-windup; forwards to
     *        ezc_pid_f64_set_anti_windup().
     *
     * @param[in,out] ctx Controller context. Must not be NULL.
     * @param[in] enable true to enable anti-windup, false to disable it.
     * @return ezc_status_t Result of ezc_pid_f64_set_anti_windup().
     */
    static ezc_status_t set_anti_windup(context_type *ctx, bool enable) noexcept {
        return ezc_pid_f64_set_anti_windup(ctx, enable);
    }

    /**
     * @brief Install or remove the update callback; forwards to
     *        ezc_pid_f64_set_callback().
     *
     * @param[in,out] ctx Controller context. Must not be NULL.
     * @param[in] callback Function invoked after each completed update step, or
     *                     nullptr to remove the current callback.
     * @param[in] user_ctx Context pointer handed to @p callback; may be
     *                     nullptr.
     * @return ezc_status_t Result of ezc_pid_f64_set_callback().
     */
    static ezc_status_t set_callback(context_type *ctx, callback_type callback,
                                     void *user_ctx) noexcept {
        return ezc_pid_f64_set_callback(ctx, callback, user_ctx);
    }

    /**
     * @brief Select the derivative source; forwards to
     *        ezc_pid_f64_set_derivative_source().
     *
     * @param[in,out] ctx Controller context. Must not be NULL.
     * @param[in] source Derivative source, see @ref ezc_pid_deriv_source_t.
     * @return ezc_status_t Result of ezc_pid_f64_set_derivative_source().
     */
    static ezc_status_t set_derivative_source(context_type *ctx,
                                              ezc_pid_deriv_source_t source) noexcept {
        return ezc_pid_f64_set_derivative_source(ctx, source);
    }

    /**
     * @brief Enable or disable the derivative filter; forwards to
     *        ezc_pid_f64_set_derivative_filter().
     *
     * @param[in,out] ctx Controller context. Must not be NULL.
     * @param[in] deriv_filter_t Filter time constant in seconds; 0 disables it.
     * @return ezc_status_t Result of ezc_pid_f64_set_derivative_filter().
     */
    static ezc_status_t set_derivative_filter(context_type *ctx,
                                              value_type deriv_filter_t) noexcept {
        return ezc_pid_f64_set_derivative_filter(ctx, deriv_filter_t);
    }

    /**
     * @brief Enable the integral limit; forwards to
     *        ezc_pid_f64_set_integral_limit().
     *
     * @param[in,out] ctx Controller context. Must not be NULL.
     * @param[in] integral_min Lower integral limit.
     * @param[in] integral_max Upper integral limit; must not be below the
     *                         lower one.
     * @return ezc_status_t Result of ezc_pid_f64_set_integral_limit().
     */
    static ezc_status_t set_integral_limit(context_type *ctx, value_type integral_min,
                                           value_type integral_max) noexcept {
        return ezc_pid_f64_set_integral_limit(ctx, integral_min, integral_max);
    }

    /**
     * @brief Disable the integral limit; forwards to
     *        ezc_pid_f64_clear_integral_limit().
     *
     * @param[in,out] ctx Controller context. Must not be NULL.
     * @return ezc_status_t Result of ezc_pid_f64_clear_integral_limit().
     */
    static ezc_status_t clear_integral_limit(context_type *ctx) noexcept {
        return ezc_pid_f64_clear_integral_limit(ctx);
    }

    /**
     * @brief Enable integral separation; forwards to
     *        ezc_pid_f64_set_integral_separation().
     *
     * @param[in,out] ctx Controller context. Must not be NULL.
     * @param[in] threshold Error magnitude above which integration stops.
     * @return ezc_status_t Result of ezc_pid_f64_set_integral_separation().
     */
    static ezc_status_t set_integral_separation(context_type *ctx,
                                                value_type threshold) noexcept {
        return ezc_pid_f64_set_integral_separation(ctx, threshold);
    }

    /**
     * @brief Disable integral separation; forwards to
     *        ezc_pid_f64_clear_integral_separation().
     *
     * @param[in,out] ctx Controller context. Must not be NULL.
     * @return ezc_status_t Result of ezc_pid_f64_clear_integral_separation().
     */
    static ezc_status_t clear_integral_separation(context_type *ctx) noexcept {
        return ezc_pid_f64_clear_integral_separation(ctx);
    }

    /**
     * @brief Set the proportional setpoint weight; forwards to
     *        ezc_pid_f64_set_setpoint_weight().
     *
     * @param[in,out] ctx Controller context. Must not be NULL.
     * @param[in] weight Setpoint weight, 0 to 1.
     * @return ezc_status_t Result of ezc_pid_f64_set_setpoint_weight().
     */
    static ezc_status_t set_setpoint_weight(context_type *ctx, value_type weight) noexcept {
        return ezc_pid_f64_set_setpoint_weight(ctx, weight);
    }

    /**
     * @brief Reset the controller state; forwards to ezc_pid_f64_reset().
     *
     * @param[in,out] ctx Controller context. NULL is ignored.
     */
    static void reset(context_type *ctx) noexcept {
        ezc_pid_f64_reset(ctx);
    }

    /**
     * @brief Run one update step; forwards to ezc_pid_f64_update().
     *
     * @param[in,out] ctx Controller context. Must not be NULL.
     * @param[in] setpoint Target value of this update step.
     * @param[in] measure Measured value of this update step.
     * @param[in] dt Elapsed time since the previous update step, in seconds.
     * @return value_type Controller output after clamping; also handed to the
     *         callback when one is installed.
     */
    static value_type update(context_type *ctx, value_type setpoint,
                             value_type measure, value_type dt) noexcept {
        return ezc_pid_f64_update(ctx, setpoint, measure, dt);
    }
};

/**
 * @brief C++ PID controller bound to the C implementation of one scalar type.
 *
 * The object owns the C context by value and is freely copyable, which makes it
 * usable as a member of a control-loop structure or as a template-instantiated
 * loop state. Every member forwards to the C API of the matching scalar type
 * and returns its status unchanged.
 *
 * @tparam T Scalar type, `float` or `double`.
 *
 * @note The wrapper adds no synchronization. As with the C API, update steps
 *       that share one controller must be serialized.
 * @note The setters configure and enable in one call: `limit()` sets
 *       `limit_enabled`, `anti_windup()` stores the flag, `integral_limit()`,
 *       `integral_separation()` and `setpoint_weight()` set their
 *       `EZC_PID_OPT_*` bit, and `derivative_filter()` filters while its time
 *       constant is greater than zero. Nothing has to be enabled twice, and
 *       `init()` is only needed for bulk configuration.
 */
template <typename T>
class pid {
public:
    using traits_type = pid_traits<T>;                          /**< C API binding of `T`. */
    using value_type = T;                                       /**< Scalar type. */
    using context_type = typename traits_type::context_type;    /**< C context type. */
    using config_type = typename traits_type::config_type;      /**< C configuration type. */
    using result_type = typename traits_type::result_type;      /**< C update result type. */
    using callback_type = typename traits_type::callback_type;  /**< C callback type. */

    /**
     * @brief Construct a disabled controller.
     *
     * @post The context is zero-initialized: all gains zero, no limit, no
     *       callback. Update steps return zero until the controller is
     *       configured by @ref init or by the individual setters.
     */
    pid() noexcept : ctx_{} {}

    /**
     * @brief Initialize the controller, see ezc_pid_*_init().
     *
     * @param[in] cfg Initialization parameters.
     * @return ezc_status_t EZC_OK on success, EZC_ERR_INVALID_ARG when @p cfg is
     *         rejected; a rejected configuration leaves a zeroed context.
     */
    ezc_status_t init(const config_type &cfg) noexcept {
        return traits_type::init(&ctx_, &cfg);
    }

    /**
     * @brief Set the gains, see ezc_pid_*_set_gains().
     *
     * @param[in] kp Proportional gain.
     * @param[in] ki Integral gain per second.
     * @param[in] kd Derivative gain per second.
     * @return ezc_status_t EZC_OK on success.
     */
    ezc_status_t set_params(value_type kp, value_type ki, value_type kd) noexcept {
        return traits_type::set_gains(&ctx_, kp, ki, kd);
    }

    /**
     * @brief Enable output clamping, see ezc_pid_*_set_limit().
     *
     * @param[in] out_min Lower output limit.
     * @param[in] out_max Upper output limit; must not be below @p out_min.
     * @return ezc_status_t EZC_OK on success, EZC_ERR_INVALID_ARG for an
     *         inverted or non-comparable range.
     */
    ezc_status_t limit(value_type out_min, value_type out_max) noexcept {
        return traits_type::set_limit(&ctx_, out_min, out_max);
    }

    /**
     * @brief Disable output clamping, see ezc_pid_*_clear_limit().
     *
     * @return ezc_status_t EZC_OK on success.
     */
    ezc_status_t clear_limit() noexcept {
        return traits_type::clear_limit(&ctx_);
    }

    /**
     * @brief Enable or disable anti-windup, see ezc_pid_*_set_anti_windup().
     *
     * @param[in] enable true to enable anti-windup, false to disable it.
     * @return ezc_status_t EZC_OK on success.
     */
    ezc_status_t anti_windup(bool enable) noexcept {
        return traits_type::set_anti_windup(&ctx_, enable);
    }

    /**
     * @brief Install or remove the update callback, see ezc_pid_*_set_callback().
     *
     * @param[in] callback Function invoked after each completed update step, or
     *                     nullptr to remove the current callback. A captureless
     *                     lambda converts to this function pointer.
     * @param[in] user_ctx Context pointer handed to @p callback; may be nullptr.
     * @return ezc_status_t EZC_OK on success.
     */
    ezc_status_t set_callback(callback_type callback, void *user_ctx = nullptr) noexcept {
        return traits_type::set_callback(&ctx_, callback, user_ctx);
    }

    /**
     * @brief Select the derivative source.
     *
     * @param[in] source Derivative source; deriving the error is the default.
     * @return ezc_status_t EZC_OK on success.
     */
    ezc_status_t derivative_source(ezc_pid_deriv_source_t source) noexcept {
        return traits_type::set_derivative_source(&ctx_, source);
    }

    /**
     * @brief Enable or disable the derivative filter.
     *
     * @param[in] deriv_filter_t Filter time constant in seconds; 0 disables it.
     * @return ezc_status_t EZC_OK on success, EZC_ERR_INVALID_ARG for a negative
     *         or non-comparable time constant.
     */
    ezc_status_t derivative_filter(value_type deriv_filter_t) noexcept {
        return traits_type::set_derivative_filter(&ctx_, deriv_filter_t);
    }

    /**
     * @brief Enable the integral limit and its `EZC_PID_OPT_INTEGRAL_LIMIT` bit.
     *
     * @param[in] integral_min Lower integral limit.
     * @param[in] integral_max Upper integral limit; must not be below
     *                         @p integral_min.
     * @return ezc_status_t EZC_OK on success.
     */
    ezc_status_t integral_limit(value_type integral_min, value_type integral_max) noexcept {
        return traits_type::set_integral_limit(&ctx_, integral_min, integral_max);
    }

    /**
     * @brief Disable the integral limit.
     *
     * @return ezc_status_t EZC_OK on success.
     */
    ezc_status_t clear_integral_limit() noexcept {
        return traits_type::clear_integral_limit(&ctx_);
    }

    /**
     * @brief Enable integral separation and its option bit.
     *
     * @param[in] threshold Error magnitude above which integration stops; must
     *                      be greater than zero.
     * @return ezc_status_t EZC_OK on success.
     */
    ezc_status_t integral_separation(value_type threshold) noexcept {
        return traits_type::set_integral_separation(&ctx_, threshold);
    }

    /**
     * @brief Disable integral separation.
     *
     * @return ezc_status_t EZC_OK on success.
     */
    ezc_status_t clear_integral_separation() noexcept {
        return traits_type::clear_integral_separation(&ctx_);
    }

    /**
     * @brief Set the proportional setpoint weight and its option bit.
     *
     * @param[in] weight Setpoint weight inside `[0, 1]`; 1 keeps the default
     *                   term and 0 makes the term follow the measurement only.
     * @return ezc_status_t EZC_OK on success.
     */
    ezc_status_t setpoint_weight(value_type weight) noexcept {
        return traits_type::set_setpoint_weight(&ctx_, weight);
    }

    /**
     * @brief Reset the controller state, see ezc_pid_*_reset().
     *
     * @post Integral and error state are zero while gains, limits and callback
     *       are preserved.
     */
    void reset() noexcept {
        traits_type::reset(&ctx_);
    }

    /**
     * @brief Run one update step, see ezc_pid_*_update().
     *
     * @param[in] setpoint Target value of this update step.
     * @param[in] measure Measured value of this update step.
     * @param[in] dt Elapsed time since the previous update step, in seconds; a
     *               value that is not a finite positive number makes this a
     *               zero-time step.
     * @return value_type Controller output after clamping; also handed to the
     *         callback when one is installed.
     */
    value_type update(value_type setpoint, value_type measure, value_type dt) noexcept {
        return traits_type::update(&ctx_, setpoint, measure, dt);
    }

    /**
     * @brief Access the underlying C context.
     * @return context_type& Mutable context, for example to inspect gains or
     *         state, or to pass it to C code.
     */
    context_type &context() noexcept {
        return ctx_;
    }

    /**
     * @brief Access the underlying C context read-only.
     * @return const context_type& Immutable context.
     */
    const context_type &context() const noexcept {
        return ctx_;
    }

private:
    context_type ctx_; /**< C controller context owned by this object. */
};

} /* namespace ezc */

#endif /* __EZCTRL_PID_HPP__ */
