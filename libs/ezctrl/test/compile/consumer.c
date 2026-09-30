/*
 * C consumer fixture for the ezctrl public headers.
 *
 * The fixture is built as its own binary together with headers.cpp, uses only
 * the documented public include path (`ezctrl/<module>.h`), and is compiled as
 * C99 with warnings-as-errors. It drives every entry point of both PID scalar
 * variants so the library objects are really linked, and treats a zero exit
 * code as the pass condition. Detailed behavior is covered by the GoogleTest
 * suites under test/.
 */

#include "ezctrl/common.h"
#include "ezctrl/pid.h"

#include <stdio.h>

/* The scalar suffixes promise 32-bit and 64-bit types on every supported
 * platform; a mismatch must fail the build instead of failing at run time. */
typedef char ezctrl_consumer_float_width[(sizeof(float) == 4) ? 1 : -1];
typedef char ezctrl_consumer_double_width[(sizeof(double) == 8) ? 1 : -1];

/* Defined in headers.cpp with C linkage: it includes ezctrl.hpp and checks the
 * C++ wrapper layer. It returns nonzero when the wrapper or the C linkage of
 * the API is broken. */
extern int ezctrl_cpp_consumer_check(void);

static void f32_callback(void *user_ctx, const ezc_pid_f32_result_t *result)
{
    float *slot = (float *)user_ctx;
    *slot = result->output;
}

static void f64_callback(void *user_ctx, const ezc_pid_f64_result_t *result)
{
    double *slot = (double *)user_ctx;
    *slot = result->output;
}

static int check_f32(void)
{
    ezc_pid_f32_t ctx = {0};
    ezc_pid_f32_cfg_t cfg = {0};
    float callback_output = -1.0f;
    float out;

    cfg.kp = 1.0f;
    cfg.ki = 0.5f;
    if (ezc_pid_f32_init(&ctx, &cfg) != EZC_OK) {
        return 1;
    }
    if (ezc_pid_f32_set_callback(&ctx, f32_callback, &callback_output) != EZC_OK) {
        return 1;
    }
    /* P = 1, I = 0.5: the callback sees the returned output. */
    out = ezc_pid_f32_update(&ctx, 1.0f, 0.0f, 1.0f);
    if (out != 1.5f || callback_output != out) {
        return 1;
    }
    /* The limiter clips the output while the integral keeps accumulating. */
    if (ezc_pid_f32_set_limit(&ctx, -1.0f, 1.0f) != EZC_OK) {
        return 1;
    }
    out = ezc_pid_f32_update(&ctx, 10.0f, 0.0f, 1.0f);
    if (out != 1.0f || ctx.integral != 5.5f) {
        return 1;
    }
    /* Anti-windup holds the integral at the saturation boundary. */
    if (ezc_pid_f32_set_anti_windup(&ctx, true) != EZC_OK) {
        return 1;
    }
    if (ezc_pid_f32_set_gains(&ctx, 0.0f, 1.0f, 0.0f) != EZC_OK) {
        return 1;
    }
    ezc_pid_f32_reset(&ctx);
    out = ezc_pid_f32_update(&ctx, 10.0f, 0.0f, 1.0f);
    if (out != 1.0f || ctx.integral != 1.0f) {
        return 1;
    }
    if (ezc_pid_f32_clear_limit(&ctx) != EZC_OK) {
        return 1;
    }
    out = ezc_pid_f32_update(&ctx, 0.0f, 0.0f, 1.0f);
    if (out != 1.0f) {
        return 1;
    }
    /* The optional features are reachable from C as well. */
    if (ezc_pid_f32_set_derivative_source(&ctx, EZC_PID_DERIV_ON_MEASUREMENT) != EZC_OK) {
        return 1;
    }
    if (ezc_pid_f32_set_derivative_filter(&ctx, 0.05f) != EZC_OK) {
        return 1;
    }
    if (ezc_pid_f32_set_integral_limit(&ctx, -5.0f, 5.0f) != EZC_OK) {
        return 1;
    }
    if (ezc_pid_f32_set_integral_separation(&ctx, 2.0f) != EZC_OK) {
        return 1;
    }
    if (ezc_pid_f32_set_setpoint_weight(&ctx, 0.0f) != EZC_OK) {
        return 1;
    }
    /* Proportional on the measurement: a setpoint step cannot move the output. */
    if (ezc_pid_f32_set_gains(&ctx, 2.0f, 0.0f, 0.0f) != EZC_OK) {
        return 1;
    }
    ezc_pid_f32_reset(&ctx);
    if (ezc_pid_f32_update(&ctx, 10.0f, 0.0f, 1.0f) != 0.0f) {
        return 1;
    }
    if (ezc_pid_f32_clear_integral_limit(&ctx) != EZC_OK) {
        return 1;
    }
    if (ezc_pid_f32_clear_integral_separation(&ctx) != EZC_OK) {
        return 1;
    }
    return 0;
}

static int check_f64(void)
{
    ezc_pid_f64_t ctx = {0};
    ezc_pid_f64_cfg_t cfg = {0};
    double callback_output = -1.0;
    double out;

    cfg.kp = 2.0;
    cfg.ki = 1.0;
    cfg.limit_enabled = true;
    cfg.out_min = -4.0;
    cfg.out_max = 4.0;
    if (ezc_pid_f64_init(&ctx, &cfg) != EZC_OK) {
        return 1;
    }
    if (ezc_pid_f64_set_callback(&ctx, f64_callback, &callback_output) != EZC_OK) {
        return 1;
    }
    if (ezc_pid_f64_set_anti_windup(&ctx, true) != EZC_OK) {
        return 1;
    }
    if (ezc_pid_f64_set_gains(&ctx, 2.0, 1.0, 0.0) != EZC_OK) {
        return 1;
    }
    if (ezc_pid_f64_set_limit(&ctx, -4.0, 4.0) != EZC_OK) {
        return 1;
    }
    /* P = 2, I = 1: the callback sees the returned output. */
    out = ezc_pid_f64_update(&ctx, 1.0, 0.0, 1.0);
    if (out != 3.0 || callback_output != out) {
        return 1;
    }
    /* Integrator-only saturation, held at the upper limit. */
    if (ezc_pid_f64_set_gains(&ctx, 0.0, 1.0, 0.0) != EZC_OK) {
        return 1;
    }
    out = ezc_pid_f64_update(&ctx, 100.0, 0.0, 1.0);
    if (out != 4.0 || ctx.integral != 4.0) {
        return 1;
    }
    ezc_pid_f64_reset(&ctx);
    if (ctx.integral != 0.0 || ctx.prev_error != 0.0) {
        return 1;
    }
    if (ezc_pid_f64_clear_limit(&ctx) != EZC_OK) {
        return 1;
    }
    /* Half a second advances half the integral and returns P + I. */
    out = ezc_pid_f64_update(&ctx, 2.0, 0.0, 0.5);
    if (out != 1.0) {
        return 1;
    }
    /* The optional features are reachable from the double API as well. */
    if (ezc_pid_f64_set_derivative_source(&ctx, EZC_PID_DERIV_ON_MEASUREMENT) != EZC_OK) {
        return 1;
    }
    if (ezc_pid_f64_set_derivative_filter(&ctx, 0.1) != EZC_OK) {
        return 1;
    }
    if (ezc_pid_f64_set_integral_limit(&ctx, -5.0, 5.0) != EZC_OK) {
        return 1;
    }
    if (ezc_pid_f64_clear_integral_limit(&ctx) != EZC_OK) {
        return 1;
    }
    if (ezc_pid_f64_set_integral_separation(&ctx, 2.0) != EZC_OK) {
        return 1;
    }
    if (ezc_pid_f64_clear_integral_separation(&ctx) != EZC_OK) {
        return 1;
    }
    if (ezc_pid_f64_set_setpoint_weight(&ctx, 1.0) != EZC_OK) {
        return 1;
    }
    return 0;
}

int main(void)
{
    int failures = 0;

    failures += check_f32();
    failures += check_f64();
    failures += ezctrl_cpp_consumer_check();

    if (failures != 0) {
        (void)fprintf(stderr, "ezctrl consumer fixture failed: %d check(s)\n", failures);
        return 1;
    }
    (void)printf("ezctrl consumer fixture passed\n");
    return 0;
}
