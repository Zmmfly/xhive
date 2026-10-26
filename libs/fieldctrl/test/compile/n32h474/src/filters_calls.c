/* Compile/link fixture only; all state and finite scratch belong to this caller. */
#include "filters.h"
#include "filters_kalman.h"

/* Every signal is a representable binary fraction. Q scalar gains and matrix
 * intermediates use the documented wide F31 type, not raw signal storage. */
#define PROBE_FILTERS(suffix, raw_type, work_type, scalar_type, coefficient_type, quarter, eighth, half, noise, gain_half, storage_count, basic_bytes) \
static int probe_##suffix(void) { \
    const scalar_type zero = 0; \
    const coefficient_type gain = gain_half; \
    filters_ema_##suffix##_t ema; \
    filters_kalman_scalar_##suffix##_t scalar; \
    scalar_type filtered = zero; \
    raw_type estimate[1] = {0}, covariance[1] = {0}; \
    raw_type initial_estimate[1] = {eighth}, initial_covariance[1] = {quarter}; \
    raw_type transition_data[1] = {half}, process_data[1] = {noise}; \
    raw_type measurement_data[1] = {eighth}, observation_data[1] = {half}; \
    raw_type noise_data[1] = {eighth}; \
    linears_matrix_t x0 = {initial_estimate, 1, 1}; \
    linears_matrix_t p0 = {initial_covariance, 1, 1}; \
    linears_matrix_t transition = {transition_data, 1, 1}; \
    linears_matrix_t process = {process_data, 1, 1}; \
    linears_matrix_t measurement = {measurement_data, 1, 1}; \
    linears_matrix_t observation = {observation_data, 1, 1}; \
    linears_matrix_t measurement_noise = {noise_data, 1, 1}; \
    filters_kalman_matrix_t matrix = {{estimate, 1, 1}, {covariance, 1, 1}, 1, 1}; \
    work_type intermediate[storage_count]; \
    linears_workspace_unit_t basic[LINEARS_WORKSPACE_UNITS(basic_bytes)]; \
    work_type decomp[FILTERS_KALMAN_UPDATE_DECOMP_STORAGE(1, 1)]; \
    filters_kalman_matrix_##suffix##_workspace_t workspace = {intermediate, storage_count, basic, sizeof(basic), {decomp, FILTERS_KALMAN_UPDATE_DECOMP_STORAGE(1, 1)}}; \
    int result = filters_ema_##suffix##_init(&ema, half, zero); \
    result |= filters_ema_##suffix##_step(&ema, quarter, &filtered); \
    result |= filters_kalman_scalar_##suffix##_reset(&scalar, eighth, quarter); \
    result |= filters_kalman_scalar_##suffix##_predict(&scalar, gain, noise); \
    result |= filters_kalman_scalar_##suffix##_update(&scalar, eighth, gain, eighth); \
    result |= filters_kalman_matrix_##suffix##_reset(&matrix, &x0, &p0); \
    result |= filters_kalman_matrix_##suffix##_predict(&matrix, &transition, &process, workspace); \
    result |= filters_kalman_matrix_##suffix##_update(&matrix, &measurement, &observation, &measurement_noise, workspace); \
    return result; \
}

PROBE_FILTERS(f32, float, float, float, float, 0.25f, 0.125f, 0.5f, 0.03125f, 0.5f,
              FILTERS_KALMAN_UPDATE_STORAGE(1, 1), FILTERS_KALMAN_UPDATE_BASIC_BYTES(1, 1))
PROBE_FILTERS(f64, double, double, double, double, 0.25, 0.125, 0.5, 0.03125, 0.5,
              FILTERS_KALMAN_UPDATE_STORAGE(1, 1), FILTERS_KALMAN_UPDATE_BASIC_BYTES(1, 1))
PROBE_FILTERS(q15, int16_t, linears_fixed_t, linears_q15_t, linears_fixed_t,
              8192, 4096, 16384, 1024, INT64_C(1073741824),
              FILTERS_KALMAN_Q_UPDATE_STORAGE(1, 1), FILTERS_KALMAN_Q_UPDATE_BASIC_BYTES(1, 1))
PROBE_FILTERS(q31, int32_t, linears_fixed_t, linears_q31_t, linears_fixed_t,
              536870912, 268435456, 1073741824, 67108864, INT64_C(1073741824),
              FILTERS_KALMAN_Q_UPDATE_STORAGE(1, 1), FILTERS_KALMAN_Q_UPDATE_BASIC_BYTES(1, 1))
#undef PROBE_FILTERS

int fieldctrl_probe_filters(void) {
    int result = probe_f32();
    result |= probe_f64();
    result |= probe_q15();
    result |= probe_q31();
    return result;
}
