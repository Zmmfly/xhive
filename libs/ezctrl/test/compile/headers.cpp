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

/* The C declarations stay C declarations when they are consumed from C++. */
static_assert(std::is_same<decltype(ezc_pid_f32_init(nullptr, nullptr)), ezc_status_t>::value,
              "ezc_pid_f32_init must return ezc_status_t");
static_assert(std::is_same<decltype(ezc_pid_f32_update(nullptr, 0.0f, 0.0f, 0.0f)), float>::value,
              "ezc_pid_f32_update must return float");
static_assert(std::is_same<decltype(ezc_pid_f64_update(nullptr, 0.0, 0.0, 0.0)), double>::value,
              "ezc_pid_f64_update must return double");
static_assert(std::is_same<decltype(ezc_pid_f64_set_limit(nullptr, 0.0, 0.0)), ezc_status_t>::value,
              "ezc_pid_f64_set_limit must return ezc_status_t");

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
    if (f32.update(1.0f, 0.0f, 1.0f) != 1.0f) {
        return 1;
    }
    return 0;
}
