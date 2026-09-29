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
 * allocation and no virtual dispatch. The scalar type selects the C API through
 * the @ref ezc::pid_traits specialization for that type:
 *
 * @code
 * ezc::pid<float> controller;
 * ezc_pid_f32_cfg_t cfg{};
 * cfg.kp = 1.0f;
 * cfg.ki = 0.1f;
 * cfg.limit_enabled = true;
 * cfg.out_min = -1.0f;
 * cfg.out_max = 1.0f;
 * cfg.anti_windup_enabled = true;
 * if (controller.init(cfg) == EZC_OK) {
 *     float duty = controller.update(setpoint, measure, dt_seconds);
 * }
 * @endcode
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
     * @brief Change the gains, see ezc_pid_*_set_gains().
     *
     * @param[in] kp Proportional gain.
     * @param[in] ki Integral gain applied per sample.
     * @param[in] kd Derivative gain applied per sample.
     * @return ezc_status_t EZC_OK on success.
     */
    ezc_status_t set_gains(value_type kp, value_type ki, value_type kd) noexcept {
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
    ezc_status_t set_limit(value_type out_min, value_type out_max) noexcept {
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
    ezc_status_t set_anti_windup(bool enable) noexcept {
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
