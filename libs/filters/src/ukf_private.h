/* Library-internal UKF kernel cores shared by ukf.c (definitions) and pf.c
 * (the UPF proposal loop). Not installed and not part of the public API. */
#ifndef FILTERS_UKF_PRIVATE_H
#define FILTERS_UKF_PRIVATE_H

#include "filters_ukf.h"

#include <math.h>

/**
 * @brief Validity test shared by the kernels and the UPF for sigma weights.
 *
 * @param[in] weights Candidate weights; may be NULL.
 * @return linears_status_t ARGUMENT for NULL, RANGE for a nonfinite member,
 *         DOMAIN for a nonpositive scale, otherwise OK.
 */
static inline linears_status_t ukf_weights_valid_f32(const filters_ukf_f32_weights_t *weights)
{
    if (weights == NULL) return LINEARS_ERROR_ARGUMENT;
    if (!isfinite(weights->scale) || !isfinite(weights->mean0) ||
        !isfinite(weights->covariance0) || !isfinite(weights->other))
        return LINEARS_ERROR_RANGE;
    return weights->scale > 0 ? LINEARS_OK : LINEARS_ERROR_DOMAIN;
}

/**
 * @copydoc ukf_weights_valid_f32
 */
static inline linears_status_t ukf_weights_valid_f64(const filters_ukf_f64_weights_t *weights)
{
    if (weights == NULL) return LINEARS_ERROR_ARGUMENT;
    if (!isfinite(weights->scale) || !isfinite(weights->mean0) ||
        !isfinite(weights->covariance0) || !isfinite(weights->other))
        return LINEARS_ERROR_RANGE;
    return weights->scale > 0 ? LINEARS_OK : LINEARS_ERROR_DOMAIN;
}

/**
 * @brief Prediction core of filters_ukf_f32_predict_work on validated inputs.
 *
 * Same parameters, callback order, results and failure codes as the public
 * kernel, which equals this core preceded by its argument validation. The
 * core performs no descriptor, shape, alias, workspace or input-finite
 * checks; it keeps every per-element RANGE check of the computation, the
 * finite checks of callback outputs and the NOT_POSITIVE_DEFINITE check of
 * the scaled covariance factor.
 *
 * @pre mean is n-by-1, covariance and process_noise n-by-n, out_mean n-by-1
 *      and out_covariance n-by-n, n > 0, all naturally aligned.
 * @pre Every input element is finite; covariance and process_noise are
 *      exactly symmetric with nonnegative diagonals.
 * @pre weights passed ukf_weights_valid_f32 and transition is not NULL.
 * @pre workspace.data holds at least FILTERS_UKF_PREDICT_WORK_STORAGE(n)
 *      elements and workspace.basic at least FILTERS_UKF_PREDICT_BASIC_BYTES(n)
 *      bytes; both are disjoint from each other, from every input and
 *      output array and from every descriptor. workspace.decomposition is
 *      not used.
 * @post On failure the outputs and scratch hold unspecified partial values;
 *       no input is modified.
 */
linears_status_t ukf_work_f32_predict_core(const linears_matrix_t *mean, const linears_matrix_t *covariance,
                                           const linears_matrix_t *process_noise,
                                           const filters_ukf_f32_weights_t *weights,
                                           filters_ukf_f32_fn transition, void *transition_context,
                                           filters_ukf_f32_residual_fn residual, void *residual_context,
                                           linears_matrix_t *out_mean, linears_matrix_t *out_covariance,
                                           filters_ukf_f32_workspace_t workspace);

/**
 * @brief Update core of filters_ukf_f32_update_work on validated inputs.
 *
 * Same parameters, callback order, results and failure codes as the public
 * kernel, except that the core does not factor measurement_noise: the public
 * kernel does so before calling it, and the UPF does so once per step for
 * the R shared by all particles. S = Pzz + R is still Cholesky-checked here.
 *
 * @pre mean is n-by-1, covariance n-by-n, measurement_noise m-by-m,
 *      measurement m-by-1, out_mean n-by-1 and out_covariance n-by-n,
 *      n,m > 0, all naturally aligned.
 * @pre Every input element is finite; covariance is exactly symmetric with a
 *      nonnegative diagonal and measurement_noise exactly symmetric.
 * @pre weights passed ukf_weights_valid_f32 and observation is not NULL.
 * @pre workspace.data holds at least FILTERS_UKF_UPDATE_WORK_STORAGE(n,m)
 *      elements, workspace.basic at least FILTERS_UKF_UPDATE_BASIC_BYTES(n,m)
 *      bytes and workspace.decomposition.capacity is at least
 *      LINEARS_SOLVE_WORKSPACE(m,n) with that prefix disjoint like the other
 *      two arrays; all three are disjoint from each other, from every input
 *      and output array and from every descriptor.
 * @post On failure the outputs and scratch hold unspecified partial values;
 *       no input is modified.
 */
linears_status_t ukf_work_f32_update_core(const linears_matrix_t *mean, const linears_matrix_t *covariance,
                                          const linears_matrix_t *measurement_noise,
                                          const linears_matrix_t *measurement,
                                          const filters_ukf_f32_weights_t *weights,
                                          filters_ukf_f32_fn observation, void *observation_context,
                                          filters_ukf_f32_residual_fn residual, void *residual_context,
                                          linears_matrix_t *out_mean, linears_matrix_t *out_covariance,
                                          filters_ukf_f32_workspace_t workspace);

/**
 * @copydoc ukf_work_f32_predict_core
 */
linears_status_t ukf_work_f64_predict_core(const linears_matrix_t *mean, const linears_matrix_t *covariance,
                                           const linears_matrix_t *process_noise,
                                           const filters_ukf_f64_weights_t *weights,
                                           filters_ukf_f64_fn transition, void *transition_context,
                                           filters_ukf_f64_residual_fn residual, void *residual_context,
                                           linears_matrix_t *out_mean, linears_matrix_t *out_covariance,
                                           filters_ukf_f64_workspace_t workspace);

/**
 * @copydoc ukf_work_f32_update_core
 */
linears_status_t ukf_work_f64_update_core(const linears_matrix_t *mean, const linears_matrix_t *covariance,
                                          const linears_matrix_t *measurement_noise,
                                          const linears_matrix_t *measurement,
                                          const filters_ukf_f64_weights_t *weights,
                                          filters_ukf_f64_fn observation, void *observation_context,
                                          filters_ukf_f64_residual_fn residual, void *residual_context,
                                          linears_matrix_t *out_mean, linears_matrix_t *out_covariance,
                                          filters_ukf_f64_workspace_t workspace);

/**
 * @brief Prediction core of filters_ukf_q15_predict_work on validated inputs.
 *
 * Contract of ukf_work_f32_predict_core with the mixed-Q kernel's storage:
 * the float views are the widened signals, workspace.data holds at least
 * FILTERS_UKF_Q_PREDICT_WORK_STORAGE(n) floats and workspace.signal at least
 * FILTERS_UKF_Q_PREDICT_SIGNAL_STORAGE(n) raw elements, disjoint as above.
 * workspace.basic and workspace.decomposition are not used.
 */
linears_status_t ukf_work_q15_predict_core(const linears_matrix_t *mean, const linears_matrix_t *covariance,
                                           const linears_matrix_t *process_noise,
                                           const filters_ukf_f32_weights_t *weights,
                                           filters_ukf_f32_fn transition, void *transition_context,
                                           filters_ukf_f32_residual_fn residual, void *residual_context,
                                           linears_matrix_t *out_mean, linears_matrix_t *out_covariance,
                                           filters_ukf_q15_workspace_t workspace);

/**
 * @brief Update core of filters_ukf_q15_update_work on validated inputs.
 *
 * Contract of ukf_work_f32_update_core with the mixed-Q kernel's storage:
 * workspace.data holds at least FILTERS_UKF_Q_UPDATE_WORK_STORAGE(n,m) floats
 * and workspace.signal at least FILTERS_UKF_Q_UPDATE_SIGNAL_STORAGE(n,m) raw
 * elements, disjoint as above. measurement_noise is not factored here;
 * workspace.basic and workspace.decomposition are not used.
 */
linears_status_t ukf_work_q15_update_core(const linears_matrix_t *mean, const linears_matrix_t *covariance,
                                          const linears_matrix_t *measurement_noise,
                                          const linears_matrix_t *measurement,
                                          const filters_ukf_f32_weights_t *weights,
                                          filters_ukf_f32_fn observation, void *observation_context,
                                          filters_ukf_f32_residual_fn residual, void *residual_context,
                                          linears_matrix_t *out_mean, linears_matrix_t *out_covariance,
                                          filters_ukf_q15_workspace_t workspace);

/**
 * @copydoc ukf_work_q15_predict_core
 */
linears_status_t ukf_work_q31_predict_core(const linears_matrix_t *mean, const linears_matrix_t *covariance,
                                           const linears_matrix_t *process_noise,
                                           const filters_ukf_f64_weights_t *weights,
                                           filters_ukf_f64_fn transition, void *transition_context,
                                           filters_ukf_f64_residual_fn residual, void *residual_context,
                                           linears_matrix_t *out_mean, linears_matrix_t *out_covariance,
                                           filters_ukf_q31_workspace_t workspace);

/**
 * @copydoc ukf_work_q15_update_core
 */
linears_status_t ukf_work_q31_update_core(const linears_matrix_t *mean, const linears_matrix_t *covariance,
                                          const linears_matrix_t *measurement_noise,
                                          const linears_matrix_t *measurement,
                                          const filters_ukf_f64_weights_t *weights,
                                          filters_ukf_f64_fn observation, void *observation_context,
                                          filters_ukf_f64_residual_fn residual, void *residual_context,
                                          linears_matrix_t *out_mean, linears_matrix_t *out_covariance,
                                          filters_ukf_q31_workspace_t workspace);

#endif
