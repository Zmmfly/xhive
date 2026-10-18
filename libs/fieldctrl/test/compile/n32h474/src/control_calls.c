/* Compile/link fixture only. These calls never configure or touch an inverter. */
#include "fieldctrl.h"
#include "fieldctrl_foc.h"
#include "fieldctrl_smo.h"
#include "fieldctrl_dtc.h"

/* Q signals are per-unit raw integers; only the PI coefficients use wide F31.
 * Small binary-fraction commands leave headroom for every exported Q1 value. */
#define PROBE_CONTROL(variant, scalar_type, coefficient_type, half_value, quarter_value, eighth_value, bus_value, small_value, dt_value, gain_value) \
static int probe_##variant(void) { \
    const scalar_type zero = 0, half = half_value, quarter = quarter_value; \
    const scalar_type eighth = eighth_value, bus = bus_value, small = small_value, dt = dt_value; \
    const coefficient_type gain = gain_value; \
    fieldctrl_##variant##_ab_t stationary = {zero, zero}; \
    fieldctrl_##variant##_ab_t flux = {quarter, zero}; \
    fieldctrl_##variant##_dq_t reference = {zero, eighth}; \
    fieldctrl_##variant##_vvvf_t vvvf; \
    fieldctrl_##variant##_vvvf_output_t vvvf_output; \
    fieldctrl_##variant##_foc_config_t foc_config = {gain, gain, gain, gain, quarter}; \
    fieldctrl_##variant##_foc_t foc; \
    fieldctrl_##variant##_foc_output_t foc_output; \
    fieldctrl_##variant##_speed_foc_config_t speed_config = {{gain, gain, gain, gain, quarter}, gain, gain, quarter, 2}; \
    fieldctrl_##variant##_speed_foc_t speed; \
    fieldctrl_##variant##_speed_foc_output_t speed_output; \
    fieldctrl_##variant##_smo_config_t smo_config = {quarter, half, dt, half, quarter, half, half, small, small, half, 2, FIELDCTRL_SMO_FORWARD}; \
    fieldctrl_##variant##_smo_t smo; \
    fieldctrl_##variant##_smo_output_t smo_output; \
    fieldctrl_##variant##_dtc_config_t dtc_config = {quarter, small, small, 2}; \
    fieldctrl_##variant##_dtc_t dtc; \
    fieldctrl_##variant##_dtc_result_t dtc_output; \
    int result = fieldctrl_##variant##_vvvf_init(&vvvf, zero); \
    result |= fieldctrl_##variant##_vvvf_step(&vvvf, eighth, quarter, bus, dt, &vvvf_output); \
    result |= fieldctrl_##variant##_foc_init(&foc, &foc_config); \
    result |= fieldctrl_##variant##_foc_step(&foc, stationary, reference, zero, bus, dt, &foc_output); \
    result |= fieldctrl_##variant##_speed_foc_init(&speed, &speed_config); \
    result |= fieldctrl_##variant##_speed_foc_step(&speed, stationary, eighth, zero, zero, bus, dt, &speed_output); \
    result |= fieldctrl_##variant##_smo_init(&smo, &smo_config, stationary); \
    result |= fieldctrl_##variant##_smo_step(&smo, stationary, stationary, &smo_output); \
    result |= fieldctrl_##variant##_dtc_init(&dtc, dtc_config, flux); \
    result |= fieldctrl_##variant##_dtc_step(&dtc, stationary, stationary, quarter, zero, dt, &dtc_output); \
    return result; \
}

PROBE_CONTROL(f32, float, float, 0.5f, 0.25f, 0.125f, 0.75f, 0.0625f, 0.0009765625f, 0.5f)
PROBE_CONTROL(f64, double, double, 0.5, 0.25, 0.125, 0.75, 0.0625, 0.0009765625, 0.5)
PROBE_CONTROL(q15, linears_q15_t, linears_fixed_t,
              16384, 8192, 4096, 24576, 2048, 32, INT64_C(1073741824))
PROBE_CONTROL(q31, linears_q31_t, linears_fixed_t,
              1073741824, 536870912, 268435456, 1610612736, 134217728, 2097152,
              INT64_C(1073741824))
#undef PROBE_CONTROL

int fieldctrl_probe_control(void) {
    int result = probe_f32();
    result |= probe_f64();
    result |= probe_q15();
    result |= probe_q31();
    return result;
}
