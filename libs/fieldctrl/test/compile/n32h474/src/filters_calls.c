/* Compile/link fixture only; all state and finite scratch belong to this caller. */
#include "filters.h"
#include "filters_kalman.h"

#define PROBE_FILTERS(suffix, raw_type, work_type, scalar_type, raw_one, exponent, zero_value, one_value, half_value) \
static int probe_##suffix(void) { \
    const scalar_type zero = zero_value, one = one_value, half = half_value; \
    filters_lowpass_##suffix##_t lowpass; \
    filters_kalman_scalar_##suffix##_t scalar; \
    scalar_type filtered = zero; \
    raw_type estimate[1] = {0}, covariance[1] = {0}; \
    raw_type initial_estimate[1] = {raw_one}, initial_covariance[1] = {raw_one}; \
    raw_type transition_data[1] = {raw_one}, process_data[1] = {0}; \
    raw_type measurement_data[1] = {raw_one}, observation_data[1] = {raw_one}; \
    raw_type noise_data[1] = {raw_one}; \
    linears_matrix_t x0 = {initial_estimate, 1, 1, exponent}; \
    linears_matrix_t p0 = {initial_covariance, 1, 1, exponent}; \
    linears_matrix_t transition = {transition_data, 1, 1, exponent}; \
    linears_matrix_t process = {process_data, 1, 1, 0}; \
    linears_matrix_t measurement = {measurement_data, 1, 1, exponent}; \
    linears_matrix_t observation = {observation_data, 1, 1, exponent}; \
    linears_matrix_t noise = {noise_data, 1, 1, exponent}; \
    filters_kalman_matrix_t matrix = {{estimate, 1, 1, 0}, {covariance, 1, 1, 0}, 1, 1}; \
    raw_type raw[FILTERS_KALMAN_UPDATE_STORAGE(1, 1)]; \
    linears_workspace_unit_t basic[LINEARS_WORKSPACE_UNITS(FILTERS_KALMAN_UPDATE_BASIC_BYTES(1, 1))]; \
    work_type decomp[FILTERS_KALMAN_UPDATE_DECOMP_STORAGE(1, 1)]; \
    filters_kalman_matrix_##suffix##_workspace_t workspace = {raw, FILTERS_KALMAN_UPDATE_STORAGE(1, 1), basic, sizeof(basic), {decomp, FILTERS_KALMAN_UPDATE_DECOMP_STORAGE(1, 1)}}; \
    int result = filters_lowpass_##suffix##_init(&lowpass, half, zero); \
    result |= filters_lowpass_##suffix##_step(&lowpass, one, &filtered); \
    result |= filters_kalman_scalar_##suffix##_reset(&scalar, zero, one); \
    result |= filters_kalman_scalar_##suffix##_predict(&scalar, one, zero); \
    result |= filters_kalman_scalar_##suffix##_update(&scalar, one, one, one); \
    result |= filters_kalman_matrix_##suffix##_reset(&matrix, &x0, &p0); \
    result |= filters_kalman_matrix_##suffix##_predict(&matrix, &transition, &process, workspace); \
    result |= filters_kalman_matrix_##suffix##_update(&matrix, &measurement, &observation, &noise, workspace); \
    return result; \
}

PROBE_FILTERS(f32, float, float, float, 1.0f, 0, 0.0f, 1.0f, 0.5f)
PROBE_FILTERS(f64, double, double, double, 1.0, 0, 0.0, 1.0, 0.5)
PROBE_FILTERS(q15, int16_t, linears_fixed_t, linears_q15_t, 16384, 1,
              ((linears_q15_t){0, 0}), ((linears_q15_t){16384, 1}), ((linears_q15_t){16384, 0}))
PROBE_FILTERS(q31, int32_t, linears_fixed_t, linears_q31_t, 1073741824, 1,
              ((linears_q31_t){0, 0}), ((linears_q31_t){1073741824, 1}), ((linears_q31_t){1073741824, 0}))
#undef PROBE_FILTERS

int fieldctrl_probe_filters(void) {
    int result = probe_f32();
    result |= probe_f64();
    result |= probe_q15();
    result |= probe_q31();
    return result;
}
