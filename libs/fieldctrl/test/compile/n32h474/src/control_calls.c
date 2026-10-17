/* Compile/link fixture only. These calls never configure or touch an inverter. */
#include "fieldctrl.h"
#include "fieldctrl_foc.h"
#include "fieldctrl_smo.h"
#include "fieldctrl_dtc.h"

#define PROBE_CONTROL(variant, scalar_type, zero_value, one_value, half_value, two_value, small_value, dt_value) \
static int probe_##variant(void) { \
    const scalar_type zero = zero_value, one = one_value, half = half_value; \
    const scalar_type two = two_value, small = small_value, dt = dt_value; \
    fieldctrl_##variant##_ab_t stationary = {zero, zero}; \
    fieldctrl_##variant##_ab_t flux = {one, zero}; \
    fieldctrl_##variant##_dq_t reference = {zero, half}; \
    fieldctrl_##variant##_vvvf_t vvvf; \
    fieldctrl_##variant##_vvvf_output_t vvvf_output; \
    fieldctrl_##variant##_foc_config_t foc_config = {one, one, one, one, one}; \
    fieldctrl_##variant##_foc_t foc; \
    fieldctrl_##variant##_foc_output_t foc_output; \
    fieldctrl_##variant##_speed_foc_config_t speed_config = {{one, one, one, one, one}, one, one, one, 2}; \
    fieldctrl_##variant##_speed_foc_t speed; \
    fieldctrl_##variant##_speed_foc_output_t speed_output; \
    fieldctrl_##variant##_smo_config_t smo_config = {one, one, dt, one, one, half, half, small, small, two, 2, FIELDCTRL_SMO_FORWARD}; \
    fieldctrl_##variant##_smo_t smo; \
    fieldctrl_##variant##_smo_output_t smo_output; \
    fieldctrl_##variant##_dtc_config_t dtc_config = {one, small, small, 2}; \
    fieldctrl_##variant##_dtc_t dtc; \
    fieldctrl_##variant##_dtc_result_t dtc_output; \
    int result = fieldctrl_##variant##_vvvf_init(&vvvf, zero); \
    result |= fieldctrl_##variant##_vvvf_step(&vvvf, one, half, two, dt, &vvvf_output); \
    result |= fieldctrl_##variant##_foc_init(&foc, &foc_config); \
    result |= fieldctrl_##variant##_foc_step(&foc, stationary, reference, zero, two, dt, &foc_output); \
    result |= fieldctrl_##variant##_speed_foc_init(&speed, &speed_config); \
    result |= fieldctrl_##variant##_speed_foc_step(&speed, stationary, one, zero, zero, two, dt, &speed_output); \
    result |= fieldctrl_##variant##_smo_init(&smo, &smo_config, stationary); \
    result |= fieldctrl_##variant##_smo_step(&smo, stationary, stationary, &smo_output); \
    result |= fieldctrl_##variant##_dtc_init(&dtc, dtc_config, flux); \
    result |= fieldctrl_##variant##_dtc_step(&dtc, stationary, stationary, one, zero, dt, &dtc_output); \
    return result; \
}

PROBE_CONTROL(f32, float, 0.0f, 1.0f, 0.5f, 2.0f, 0.0625f, 0.0009765625f)
PROBE_CONTROL(f64, double, 0.0, 1.0, 0.5, 2.0, 0.0625, 0.0009765625)
PROBE_CONTROL(q15, linears_q15_t,
              ((linears_q15_t){0, 0}), ((linears_q15_t){16384, 1}),
              ((linears_q15_t){16384, 0}), ((linears_q15_t){16384, 2}),
              ((linears_q15_t){16384, -3}), ((linears_q15_t){16384, -9}))
PROBE_CONTROL(q31, linears_q31_t,
              ((linears_q31_t){0, 0}), ((linears_q31_t){1073741824, 1}),
              ((linears_q31_t){1073741824, 0}), ((linears_q31_t){1073741824, 2}),
              ((linears_q31_t){1073741824, -3}), ((linears_q31_t){1073741824, -9}))
#undef PROBE_CONTROL

int fieldctrl_probe_control(void) {
    int result = probe_f32();
    result |= probe_f64();
    result |= probe_q15();
    result |= probe_q31();
    return result;
}
