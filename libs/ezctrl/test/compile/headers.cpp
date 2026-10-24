/*
 * C++ side of the ezctrl consumer fixture.
 *
 * It includes the single C++ entry point (`ezctrl.hpp`) together with the C
 * headers of the module, statically checks the C API signatures as they are
 * seen from C++, drives the wrapper of both scalar variants, and exports its
 * verdict with C linkage so consumer.c can link against it. A successful
 * mixed-language link and a zero exit code are the pass condition.
 *
 * Build requirements: C++17, warnings-as-errors. No GoogleTest is used.
 */

#include "ezctrl.hpp"

#include "ezctrl/common.h"
#include "ezctrl/pid.h"

#include <type_traits>
#include <utility>

/* The C declarations stay C declarations when they are consumed from C++. */
static_assert(std::is_same<decltype(ezc_pid_f32_init(nullptr, nullptr)), ezc_status_t>::value,
              "ezc_pid_f32_init must return ezc_status_t");
static_assert(std::is_same<decltype(ezc_pid_f32_update(nullptr, 0.0f, 0.0f, 0.0f)), float>::value,
              "ezc_pid_f32_update must return float");
static_assert(std::is_same<decltype(ezc_pid_f64_update(nullptr, 0.0, 0.0, 0.0)), double>::value,
              "ezc_pid_f64_update must return double");
static_assert(std::is_same<decltype(ezc_pid_f64_set_limit(nullptr, 0.0, 0.0)), ezc_status_t>::value,
              "ezc_pid_f64_set_limit must return ezc_status_t");
static_assert(std::is_same<decltype(ezc_pid_f32_try_update(nullptr, 0.0f, 0.0f, 0.0f, nullptr)),
                           ezc_status_t>::value,
              "ezc_pid_f32_try_update must return ezc_status_t");
static_assert(std::is_same<decltype(ezc_pid_f64_reset_to(nullptr, 0.0)), ezc_status_t>::value,
              "ezc_pid_f64_reset_to must return ezc_status_t");

/* The scalar variants share names but never types. */
static_assert(!std::is_same<ezc_pid_f32_t, ezc_pid_f64_t>::value,
              "the f32 and f64 contexts must stay distinct");
static_assert(!std::is_same<ezc_pid_f32_t, ezc_pid_f32_cfg_t>::value,
              "a context and its configuration must stay distinct");
static_assert(!std::is_same<ezc_pid_f32_cb_t, ezc_pid_f64_cb_t>::value,
              "the f32 and f64 callback types must stay distinct");

/* The wrapper binds to the typed C API of its scalar type. */
static_assert(std::is_same<ezc::pid<float>::context_type, ezc_pid_f32_t>::value,
              "ezc::pid<float> must wrap the f32 C API");
static_assert(std::is_same<ezc::pid<double>::context_type, ezc_pid_f64_t>::value,
              "ezc::pid<double> must wrap the f64 C API");
static_assert(std::is_same<ezc::pid<double>::callback_type, ezc_pid_f64_cb_t>::value,
              "ezc::pid<double> must reuse the f64 callback type");

/* Each C context embeds the fieldctrl controller of the same scalar type. */
static_assert(std::is_same<decltype(ezc_pid_f32_t::pid), fieldctrl_f32_pid_t>::value,
              "the f32 context must be computed by fieldctrl_f32_pid");
static_assert(std::is_same<decltype(ezc_pid_f64_t::pid), fieldctrl_f64_pid_t>::value,
              "the f64 context must be computed by fieldctrl_f64_pid");

static_assert(std::is_same<decltype(std::declval<ezc::pid<float> &>().update(1.0f, 0.0f, 1.0f)),
                           float>::value,
              "the wrapper update must return the scalar type");
static_assert(std::is_same<decltype(std::declval<ezc::pid<float> &>().setpoint_weight(0.0f)),
                           ezc_status_t>::value,
              "the wrapper setters must return ezc_status_t");
static_assert(std::is_same<decltype(std::declval<ezc::pid<double> &>().derivative_source(
                           EZC_PID_DERIV_ON_ERROR)),
                           ezc_status_t>::value,
              "the wrapper must accept the C derivative source enum");

extern "C" int ezctrl_cpp_consumer_check(void);

extern "C" int ezctrl_cpp_consumer_check(void) {
    ezc::pid<float> f32;
    ezc::pid<double> f64;
    float f32_callback_output = 0.0f;
    double f64_callback_output = 0.0;

    ezc_pid_f32_cfg_t cfg32{};
    cfg32.kp = 2.0f;
    cfg32.ki = 0.5f;
    ezc_pid_f64_cfg_t cfg64{};
    cfg64.kp = 2.0;
    cfg64.ki = 0.5;

    if (f32.init(cfg32) != EZC_OK || f64.init(cfg64) != EZC_OK) {
        return 1;
    }

    const auto f32_callback = [](void *user_ctx, const ezc_pid_f32_result_t *result) {
        *static_cast<float *>(user_ctx) = result->output;
    };
    const auto f64_callback = [](void *user_ctx, const ezc_pid_f64_result_t *result) {
        *static_cast<double *>(user_ctx) = result->output;
    };
    if (f32.set_callback(f32_callback, &f32_callback_output) != EZC_OK ||
        f64.set_callback(f64_callback, &f64_callback_output) != EZC_OK) {
        return 1;
    }

    /* P = 2, I = 0.5 * 1 s: the wrapper returns what the callback receives. */
    const float f32_output = f32.update(1.0f, 0.0f, 1.0f);
    const double f64_output = f64.update(1.0, 0.0, 1.0);
    if (f32_output != f32_callback_output || f64_output != f64_callback_output) {
        return 1;
    }
    if (f32_output != 2.5f || f64_output != 2.5) {
        return 1;
    }

    /* The C API stays directly callable next to the wrapper. */
    if (ezc_pid_f32_set_limit(&f32.context(), -1.0f, 1.0f) != EZC_OK) {
        return 1;
    }
    if (ezc_pid_f64_clear_limit(&f64.context()) != EZC_OK) {
        return 1;
    }

    /* The optional features are reachable through the wrapper as well. */
    if (f32.derivative_source(EZC_PID_DERIV_ON_MEASUREMENT) != EZC_OK ||
        f32.derivative_filter(0.05f) != EZC_OK ||
        f32.integral_limit(-1.0f, 1.0f) != EZC_OK ||
        f32.integral_separation(1.0f) != EZC_OK ||
        f32.setpoint_weight(0.0f) != EZC_OK) {
        return 1;
    }
    if (f64.derivative_source(EZC_PID_DERIV_ON_MEASUREMENT) != EZC_OK ||
        f64.derivative_filter(0.1) != EZC_OK ||
        f64.integral_limit(-1.0, 1.0) != EZC_OK ||
        f64.integral_separation(1.0) != EZC_OK ||
        f64.setpoint_weight(0.0) != EZC_OK) {
        return 1;
    }
    if (f32.clear_integral_limit() != EZC_OK ||
        f32.clear_integral_separation() != EZC_OK ||
        f64.clear_integral_limit() != EZC_OK ||
        f64.clear_integral_separation() != EZC_OK) {
        return 1;
    }

    /* Proportional on the measurement keeps a setpoint step out of the output. */
    if (f32.set_params(2.0f, 0.0f, 0.0f) != EZC_OK) {
        return 1;
    }
    f32.reset();
    if (f32.update(10.0f, 0.0f, 1.0f) != 0.0f) {
        return 1;
    }

    /* Status-returning step and bumpless restart through the wrapper. */
    double f64_output_checked = 0.0;
    if (f64.reset_to(0.5) != EZC_OK ||
        f64.try_update(0.0, 0.0, 1.0, f64_output_checked) != EZC_OK ||
        f64_output_checked != 0.5) {
        return 1;
    }
    if (f64.try_update(0.0, 0.0, 0.0, f64_output_checked) != EZC_ERR_INVALID_ARG) {
        return 1;
    }
    return 0;
}
