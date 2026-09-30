/*
 * Shared typed-test harness for the ezctrl PID C API.
 *
 * Every test translation unit of this library is registered as its own binary,
 * so the fixture and the type-parameter list may live in this header and be
 * used by all suites. The type parameter runs each suite once per scalar
 * variant, which keeps the expectations of f32 and f64 identical.
 */
#ifndef EZCTRL_TEST_PID_TEST_TYPES_H
#define EZCTRL_TEST_PID_TEST_TYPES_H

#include <gtest/gtest.h>

#include <cstddef>

#include "ezctrl/pid.h"

namespace ezc_test {

/*
 * Absolute tolerance of the scalar type under test. The expectations use
 * exactly representable values, so this only absorbs the rounding of the
 * floating-point arithmetic itself.
 */
template <typename T>
constexpr T pid_epsilon() {
    return sizeof(T) == sizeof(float) ? static_cast<T>(1e-5) : static_cast<T>(1e-12);
}

/**
 * Mapping from a scalar type to the typed C PID API under test.
 */
template <typename T>
struct pid_api;

template <>
struct pid_api<float> {
    using value_type = float;
    using context_type = ezc_pid_f32_t;
    using config_type = ezc_pid_f32_cfg_t;
    using result_type = ezc_pid_f32_result_t;
    using callback_type = ezc_pid_f32_cb_t;

    /* Records what the callback receives during one test. */
    struct callback_sink {
        int calls = 0;
        value_type output = value_type(0);
        value_type setpoint = value_type(0);
        value_type measure = value_type(0);
        value_type error = value_type(0);
        value_type p_term = value_type(0);
        value_type i_term = value_type(0);
        value_type d_term = value_type(0);
        bool limited = false;
        void *last_user = nullptr;
    };

    static void callback(void *user_ctx, const result_type *result) {
        auto *sink = static_cast<callback_sink *>(user_ctx);
        sink->calls += 1;
        sink->output = result->output;
        sink->setpoint = result->setpoint;
        sink->measure = result->measure;
        sink->error = result->error;
        sink->p_term = result->p_term;
        sink->i_term = result->i_term;
        sink->d_term = result->d_term;
        sink->limited = result->limited;
        sink->last_user = user_ctx;
    }

    static const char *name() { return "f32"; }

    /* Value-initialized configuration: disabled controller. */
    static config_type make_config() { return config_type{}; }

    static ezc_status_t init(context_type *ctx, const config_type *cfg) {
        return ezc_pid_f32_init(ctx, cfg);
    }
    static ezc_status_t set_gains(context_type *ctx, value_type kp, value_type ki,
                                  value_type kd) {
        return ezc_pid_f32_set_gains(ctx, kp, ki, kd);
    }
    static ezc_status_t set_limit(context_type *ctx, value_type out_min, value_type out_max) {
        return ezc_pid_f32_set_limit(ctx, out_min, out_max);
    }
    static ezc_status_t clear_limit(context_type *ctx) {
        return ezc_pid_f32_clear_limit(ctx);
    }
    static ezc_status_t set_anti_windup(context_type *ctx, bool enable) {
        return ezc_pid_f32_set_anti_windup(ctx, enable);
    }
    static ezc_status_t set_callback(context_type *ctx, callback_type callback, void *user_ctx) {
        return ezc_pid_f32_set_callback(ctx, callback, user_ctx);
    }
    static ezc_status_t set_derivative_source(context_type *ctx, ezc_pid_deriv_source_t source) {
        return ezc_pid_f32_set_derivative_source(ctx, source);
    }
    static ezc_status_t set_derivative_filter(context_type *ctx, value_type deriv_filter_t) {
        return ezc_pid_f32_set_derivative_filter(ctx, deriv_filter_t);
    }
    static ezc_status_t set_integral_limit(context_type *ctx, value_type integral_min,
                                           value_type integral_max) {
        return ezc_pid_f32_set_integral_limit(ctx, integral_min, integral_max);
    }
    static ezc_status_t clear_integral_limit(context_type *ctx) {
        return ezc_pid_f32_clear_integral_limit(ctx);
    }
    static ezc_status_t set_integral_separation(context_type *ctx, value_type threshold) {
        return ezc_pid_f32_set_integral_separation(ctx, threshold);
    }
    static ezc_status_t clear_integral_separation(context_type *ctx) {
        return ezc_pid_f32_clear_integral_separation(ctx);
    }
    static ezc_status_t set_setpoint_weight(context_type *ctx, value_type weight) {
        return ezc_pid_f32_set_setpoint_weight(ctx, weight);
    }
    static void reset(context_type *ctx) { ezc_pid_f32_reset(ctx); }

    /*
     * One update step with a one-second sample period. The per-step arithmetic
     * tests use it, so their expectations read as gains per second; the
     * sample-period tests call update() with an explicit dt.
     */
    static value_type step(context_type *ctx, value_type setpoint, value_type measure) {
        return ezc_pid_f32_update(ctx, setpoint, measure, value_type(1));
    }

    static value_type update(context_type *ctx, value_type setpoint, value_type measure,
                             value_type dt) {
        return ezc_pid_f32_update(ctx, setpoint, measure, dt);
    }
};

template <>
struct pid_api<double> {
    using value_type = double;
    using context_type = ezc_pid_f64_t;
    using config_type = ezc_pid_f64_cfg_t;
    using result_type = ezc_pid_f64_result_t;
    using callback_type = ezc_pid_f64_cb_t;

    /* Records what the callback receives during one test. */
    struct callback_sink {
        int calls = 0;
        value_type output = value_type(0);
        value_type setpoint = value_type(0);
        value_type measure = value_type(0);
        value_type error = value_type(0);
        value_type p_term = value_type(0);
        value_type i_term = value_type(0);
        value_type d_term = value_type(0);
        bool limited = false;
        void *last_user = nullptr;
    };

    static void callback(void *user_ctx, const result_type *result) {
        auto *sink = static_cast<callback_sink *>(user_ctx);
        sink->calls += 1;
        sink->output = result->output;
        sink->setpoint = result->setpoint;
        sink->measure = result->measure;
        sink->error = result->error;
        sink->p_term = result->p_term;
        sink->i_term = result->i_term;
        sink->d_term = result->d_term;
        sink->limited = result->limited;
        sink->last_user = user_ctx;
    }

    static const char *name() { return "f64"; }

    /* Value-initialized configuration: disabled controller. */
    static config_type make_config() { return config_type{}; }

    static ezc_status_t init(context_type *ctx, const config_type *cfg) {
        return ezc_pid_f64_init(ctx, cfg);
    }
    static ezc_status_t set_gains(context_type *ctx, value_type kp, value_type ki,
                                  value_type kd) {
        return ezc_pid_f64_set_gains(ctx, kp, ki, kd);
    }
    static ezc_status_t set_limit(context_type *ctx, value_type out_min, value_type out_max) {
        return ezc_pid_f64_set_limit(ctx, out_min, out_max);
    }
    static ezc_status_t clear_limit(context_type *ctx) {
        return ezc_pid_f64_clear_limit(ctx);
    }
    static ezc_status_t set_anti_windup(context_type *ctx, bool enable) {
        return ezc_pid_f64_set_anti_windup(ctx, enable);
    }
    static ezc_status_t set_callback(context_type *ctx, callback_type callback, void *user_ctx) {
        return ezc_pid_f64_set_callback(ctx, callback, user_ctx);
    }
    static ezc_status_t set_derivative_source(context_type *ctx, ezc_pid_deriv_source_t source) {
        return ezc_pid_f64_set_derivative_source(ctx, source);
    }
    static ezc_status_t set_derivative_filter(context_type *ctx, value_type deriv_filter_t) {
        return ezc_pid_f64_set_derivative_filter(ctx, deriv_filter_t);
    }
    static ezc_status_t set_integral_limit(context_type *ctx, value_type integral_min,
                                           value_type integral_max) {
        return ezc_pid_f64_set_integral_limit(ctx, integral_min, integral_max);
    }
    static ezc_status_t clear_integral_limit(context_type *ctx) {
        return ezc_pid_f64_clear_integral_limit(ctx);
    }
    static ezc_status_t set_integral_separation(context_type *ctx, value_type threshold) {
        return ezc_pid_f64_set_integral_separation(ctx, threshold);
    }
    static ezc_status_t clear_integral_separation(context_type *ctx) {
        return ezc_pid_f64_clear_integral_separation(ctx);
    }
    static ezc_status_t set_setpoint_weight(context_type *ctx, value_type weight) {
        return ezc_pid_f64_set_setpoint_weight(ctx, weight);
    }
    static void reset(context_type *ctx) { ezc_pid_f64_reset(ctx); }

    /*
     * One update step with a one-second sample period. The per-step arithmetic
     * tests use it, so their expectations read as gains per second; the
     * sample-period tests call update() with an explicit dt.
     */
    static value_type step(context_type *ctx, value_type setpoint, value_type measure) {
        return ezc_pid_f64_update(ctx, setpoint, measure, value_type(1));
    }

    static value_type update(context_type *ctx, value_type setpoint, value_type measure,
                             value_type dt) {
        return ezc_pid_f64_update(ctx, setpoint, measure, dt);
    }
};

} /* namespace ezc_test */

using PidTypes = ::testing::Types<float, double>;

template <typename T>
class PidTest : public ::testing::Test {};

TYPED_TEST_SUITE(PidTest, PidTypes);

#endif /* EZCTRL_TEST_PID_TEST_TYPES_H */
