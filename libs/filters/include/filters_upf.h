/**
 * @file filters_upf.h
 * @brief Caller-owned unscented particle filters (UPF).
 *
 * Every particle carries its own mean and covariance. One step runs the
 * matching UKF predict and update kernels of filters_ukf.h per particle
 * with the current measurement, draws the new particle from that
 * Gaussian proposal N(xbar_i, Pbar_i), and updates its importance weight by
 * likelihood * prior / proposal. Resampling gathers particles and
 * covariances together. The particle set and weights are a filters_pf_*_t,
 * so filters_pf_*_estimate and filters_pf_*_effective_sample_size apply to
 * the embedded base unchanged.
 *
 * Storage, callback, residual, sampler, workspace, failure and
 * serialization rules are those of filters_pf.h and filters_ukf.h. q15/q31
 * use their dedicated integer statistical kernels and raw particle,
 * covariance and weight staging. Floating storage holds one particle's
 * extended-range intermediates, the callback/decomposition boundary and
 * one log-density exponent per particle. The transition must be
 * deterministic: process noise enters only through Q. See PF.md for the
 * weight derivation, storage layout and validation.
 */
#ifndef FILTERS_UPF_H
#define FILTERS_UPF_H

#include "filters_pf.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Single-precision UPF model: callbacks and their contexts.
 *
 * Residuals may be NULL for plain subtraction; contexts may be NULL.
 */
typedef struct {
    filters_ukf_f32_fn transition; /**< Deterministic f, n-by-1 to n-by-1; not NULL. */
    void *transition_context; /**< Passed to transition unchanged. */
    filters_ukf_f32_fn observation; /**< h, n-by-1 to m-by-1; not NULL. */
    void *observation_context; /**< Passed to observation unchanged. */
    filters_ukf_f32_residual_fn state_residual; /**< State-space residual or NULL. */
    void *state_residual_context; /**< Passed to state_residual unchanged. */
    filters_ukf_f32_residual_fn measurement_residual; /**< Measurement-space residual or NULL. */
    void *measurement_residual_context; /**< Passed to measurement_residual unchanged. */
} filters_upf_f32_model_t;

/**
 * @copydoc filters_upf_f32_model_t
 */
typedef struct {
    filters_ukf_f64_fn transition; /**< Deterministic f, n-by-1 to n-by-1; not NULL. */
    void *transition_context; /**< Passed to transition unchanged. */
    filters_ukf_f64_fn observation; /**< h, n-by-1 to m-by-1; not NULL. */
    void *observation_context; /**< Passed to observation unchanged. */
    filters_ukf_f64_residual_fn state_residual; /**< State-space residual or NULL. */
    void *state_residual_context; /**< Passed to state_residual unchanged. */
    filters_ukf_f64_residual_fn measurement_residual; /**< Measurement-space residual or NULL. */
    void *measurement_residual_context; /**< Passed to measurement_residual unchanged. */
} filters_upf_f64_model_t;

/**
 * @brief Caller-owned single-precision unscented particle filter.
 *
 * Set up base as for filters_pf_f32_t, point covariances at an N-by-(n*n)
 * float array (row i holds P_i row-major) with its capacity, and leave
 * sigma_weights to init. Rules of filters_pf_f32_t apply to every field.
 */
typedef struct {
    filters_pf_f32_t base; /**< Particles, weights, Q and R. */
    linears_matrix_t covariances; /**< N-by-(n*n) working per-particle covariances. */
    size_t covariances_capacity; /**< Allocated covariance elements; at least N*n*n. */
    filters_ukf_f32_weights_t sigma_weights; /**< Written by init from alpha/beta/kappa. */
} filters_upf_f32_t;

/**
 * @copydoc filters_upf_f32_t
 */
typedef struct {
    filters_pf_f64_t base; /**< Particles, weights, Q and R. */
    linears_matrix_t covariances; /**< N-by-(n*n) working per-particle covariances. */
    size_t covariances_capacity; /**< Allocated covariance elements; at least N*n*n. */
    filters_ukf_f64_weights_t sigma_weights; /**< Written by init from alpha/beta/kappa. */
} filters_upf_f64_t;

/**
 * @brief Caller-owned q15 unscented particle filter; every array is raw q15.
 *
 * Rules of filters_upf_f32_t with int16_t elements, float callbacks and
 * sigma weights, and filters_ukf_q15_workspace_t scratch. Committed
 * particles and posterior covariances must stay inside [-1,+1], or the
 * step fails with RANGE. Sigma points, residuals, normal draws and the
 * predicted prior are not restricted to that interval.
 */
typedef struct {
    filters_pf_q15_t base; /**< Raw q15 particles, weights, Q and R. */
    linears_matrix_t covariances; /**< N-by-(n*n) raw q15 per-particle covariances. */
    size_t covariances_capacity; /**< Allocated covariance elements; at least N*n*n. */
    filters_ukf_f32_weights_t sigma_weights; /**< Written by init from float alpha/beta/kappa. */
} filters_upf_q15_t;

/**
 * @brief Caller-owned q31 unscented particle filter; every array is raw q31.
 *
 * Rules of filters_upf_q15_t with int32_t elements, the f64 model and
 * sigma weights, and filters_ukf_q31_workspace_t scratch.
 */
typedef struct {
    filters_pf_q31_t base; /**< Raw q31 particles, weights, Q and R. */
    linears_matrix_t covariances; /**< N-by-(n*n) raw q31 per-particle covariances. */
    size_t covariances_capacity; /**< Allocated covariance elements; at least N*n*n. */
    filters_ukf_f64_weights_t sigma_weights; /**< Written by init from double alpha/beta/kappa. */
} filters_upf_q31_t;

/**
 * @brief Floating working elements for one UPF step; SIZE_MAX on overflow.
 *
 * N*(n*n + n + 2) + 3*n*n + 6*n + m*m + 2*m + 1 plus
 * FILTERS_UKF_UPDATE_WORK_STORAGE(n,m) for N particles, n states and m
 * measurements, all positive and side-effect-free: staged covariances,
 * particles, exponents and weights, one particle's vectors and factors,
 * the Cholesky factors of Q and R, and the per-particle kernel scratch.
 */
#define FILTERS_UPF_STEP_STORAGE(N, n, m) \
    LINEARS_SIZE_ADD( \
        LINEARS_SIZE_ADD( \
            LINEARS_SIZE_MUL((N), LINEARS_SIZE_ADD(LINEARS_SIZE_MUL((n), (n)), LINEARS_SIZE_ADD((n), 2))), \
            FILTERS_UKF_UPDATE_WORK_STORAGE((n), (m))), \
        LINEARS_SIZE_ADD(LINEARS_SIZE_ADD(LINEARS_SIZE_MUL(3, LINEARS_SIZE_MUL((n), (n))), LINEARS_SIZE_MUL((m), (m))), \
                         LINEARS_SIZE_ADD(LINEARS_SIZE_MUL(6, (n)), LINEARS_SIZE_ADD(LINEARS_SIZE_MUL(2, (m)), 1))))

/**
 * @brief Floating working elements for a Q step; SIZE_MAX on overflow.
 *
 * N + 4*n*n + 2*m*m + 4*n + 3*m plus the larger mixed-Q UKF work amount.
 * Only the N log-density exponents need per-particle floating storage:
 * particles, covariances and weights are staged raw. The other elements
 * hold one particle's extended-range prior/posterior and the Q/R factors.
 * Allocate float for q15 and double for q31. N, n and m are positive and
 * side-effect-free.
 */
#define FILTERS_UPF_Q_STEP_STORAGE(N, n, m) \
    LINEARS_SIZE_ADD( \
        LINEARS_SIZE_ADD((N), \
            FILTERS_KALMAN_MAX_SIZE(FILTERS_UKF_Q_PREDICT_WORK_STORAGE(n), \
                                    FILTERS_UKF_Q_UPDATE_WORK_STORAGE((n), (m)))), \
        LINEARS_SIZE_ADD( \
            LINEARS_SIZE_ADD(LINEARS_SIZE_MUL(4, LINEARS_SIZE_MUL((n), (n))), \
                             LINEARS_SIZE_MUL(2, LINEARS_SIZE_MUL((m), (m)))), \
            LINEARS_SIZE_ADD(LINEARS_SIZE_MUL(4, (n)), LINEARS_SIZE_MUL(3, (m)))))

/**
 * @brief Raw Q elements for a UPF step; SIZE_MAX on overflow.
 *
 * N*(n*n+n+1) elements stage posterior covariances, particles and weights;
 * the remaining elements are reused by the mixed-Q UKF kernels. Allocate
 * int16_t for q15 and int32_t for q31.
 */
#define FILTERS_UPF_Q_STEP_SIGNAL_STORAGE(N, n, m) \
    LINEARS_SIZE_ADD( \
        LINEARS_SIZE_MUL((N), LINEARS_SIZE_ADD(LINEARS_SIZE_MUL((n), (n)), LINEARS_SIZE_ADD((n), 1))), \
        FILTERS_KALMAN_MAX_SIZE(FILTERS_UKF_Q_PREDICT_SIGNAL_STORAGE(n), \
                                FILTERS_UKF_Q_UPDATE_SIGNAL_STORAGE((n), (m))))

/**
 * @brief Basic scratch bytes for a floating UPF step.
 */
#define FILTERS_UPF_STEP_BASIC_BYTES(n, m) FILTERS_UKF_UPDATE_BASIC_BYTES((n), (m))

/**
 * @brief Decomposition elements for a UPF step: max(n*n + n, m*m + m*n).
 *
 * Covers the floating gain solve and the prior/likelihood exponent solves.
 * Private Cholesky factors use data; the required capacity is unchanged.
 */
#define FILTERS_UPF_STEP_DECOMP_STORAGE(n, m) \
    FILTERS_KALMAN_MAX_SIZE(LINEARS_SIZE_ADD(LINEARS_SIZE_MUL((n), (n)), (n)), LINEARS_SOLVE_WORKSPACE((m), (n)))

/**
 * @brief Basic scratch bytes for a Q UPF step: none.
 */
#define FILTERS_UPF_Q_STEP_BASIC_BYTES(n, m) FILTERS_UKF_Q_UPDATE_BASIC_BYTES((n), (m))

/**
 * @brief Q step decomposition elements: max(n*n,m*m); SIZE_MAX on overflow.
 *
 * Allocate float for q15 and double for q31. The step currently factors
 * into data and does not write this array, but still validates its capacity
 * and disjointness. The Gaussian density uses direct triangular solves.
 */
#define FILTERS_UPF_Q_STEP_DECOMP_STORAGE(n, m) FILTERS_UKF_Q_UPDATE_DECOMP_STORAGE((n), (m))

/**
 * @brief Initialize a single-precision UPF from a Gaussian prior.
 *
 * Particles are drawn as in filters_pf_f32_init; every per-particle
 * covariance starts as the prior covariance.
 *
 * @param[in,out] state Non-NULL object whose base, covariances and capacities are set.
 * @param[in] alpha Sigma point spread, strictly positive.
 * @param[in] beta Distribution prior; 2 for Gaussian states.
 * @param[in] kappa Secondary scaling; n+kappa must be positive.
 * @param[in] mean Non-NULL finite n-by-1 prior mean.
 * @param[in] covariance Non-NULL finite symmetric positive-definite n-by-n prior covariance.
 * @param[in] process_noise Non-NULL finite symmetric SPD n-by-n Q; exact alias rules of the PF.
 * @param[in] measurement_noise Non-NULL finite symmetric SPD m-by-m R; same alias rules.
 * @param[in] sample Non-NULL standard-normal sampler.
 * @param[in] context Passed to sample unchanged; may be NULL.
 * @param[in] workspace Arrays meeting FILTERS_PF_PREDICT_STORAGE(N,n),
 *                      FILTERS_PF_PREDICT_BASIC_BYTES(n) and
 *                      FILTERS_PF_PREDICT_DECOMP_STORAGE(n).
 * @return linears_status_t As filters_pf_f32_init, plus DOMAIN for invalid
 *         sigma hyperparameters.
 */
linears_status_t filters_upf_f32_init(filters_upf_f32_t *state, float alpha, float beta, float kappa,
                                      const linears_matrix_t *mean, const linears_matrix_t *covariance,
                                      const linears_matrix_t *process_noise,
                                      const linears_matrix_t *measurement_noise,
                                      filters_pf_f32_sample_fn sample, void *context,
                                      filters_ukf_f32_workspace_t workspace);

/**
 * @brief One single-precision UPF time step with measurement z.
 *
 * Per particle: UKF predict with f and Q, UKF update with h, R and z, draw
 * x_i = xbar_i + L_i*xi with L_i*L_i^T = Pbar_i, and set the weight to
 * w_i * prod(diag L_i) * exp(-e_i) with e_i the likelihood plus prior minus
 * proposal Gaussian exponents; the weights are then shift-normalized.
 *
 * @param[in,out] state Non-NULL initialized filter.
 * @param[in] measurement Non-NULL finite m-by-1 z.
 * @param[in] model Non-NULL callbacks; transition and observation not NULL.
 * @param[in] sample Non-NULL standard-normal sampler, called once per particle.
 * @param[in] sample_context Passed to sample unchanged; may be NULL.
 * @param[in] workspace Arrays meeting FILTERS_UPF_STEP_STORAGE(N,n,m),
 *                      FILTERS_UPF_STEP_BASIC_BYTES(n,m) and
 *                      FILTERS_UPF_STEP_DECOMP_STORAGE(n,m).
 * @return linears_status_t OK, ARGUMENT, DIMENSION, SIZE, WORKSPACE, ALIAS,
 *         RANGE, DOMAIN, NOT_POSITIVE_DEFINITE when Q, R (both checked before
 *         any callback), a particle covariance, an innovation covariance or a
 *         proposal covariance has no Cholesky factor, SINGULAR, or a callback error.
 * @note The transition is evaluated at every particle and at its sigma
 *       points; the observation at every sigma point and every new particle.
 *       Failure preserves particles, covariances and weights.
 */
linears_status_t filters_upf_f32_step(filters_upf_f32_t *state, const linears_matrix_t *measurement,
                                      const filters_upf_f32_model_t *model, filters_pf_f32_sample_fn sample,
                                      void *sample_context, filters_ukf_f32_workspace_t workspace);

/**
 * @brief Systematically resample particles and covariances; weights become 1/N.
 *
 * Particles and covariances are gathered in place, as in filters_pf_f32_resample.
 *
 * @param[in,out] state Non-NULL initialized filter.
 * @param[in] uniform Draw in [0,1).
 * @param[out] indices Array of at least N size_t entries receiving the source rows.
 * @param[in] capacity Available indices entries.
 * @param[in] workspace Ignored by the f32 and f64 filters; pass any value.
 * @return linears_status_t As filters_pf_f32_resample.
 */
linears_status_t filters_upf_f32_resample(filters_upf_f32_t *state, float uniform, size_t *indices,
                                          size_t capacity, filters_ukf_f32_workspace_t workspace);

/**
 * @copydoc filters_upf_f32_init
 */
linears_status_t filters_upf_f64_init(filters_upf_f64_t *state, double alpha, double beta, double kappa,
                                      const linears_matrix_t *mean, const linears_matrix_t *covariance,
                                      const linears_matrix_t *process_noise,
                                      const linears_matrix_t *measurement_noise,
                                      filters_pf_f64_sample_fn sample, void *context,
                                      filters_ukf_f64_workspace_t workspace);

/**
 * @copydoc filters_upf_f32_step
 */
linears_status_t filters_upf_f64_step(filters_upf_f64_t *state, const linears_matrix_t *measurement,
                                      const filters_upf_f64_model_t *model, filters_pf_f64_sample_fn sample,
                                      void *sample_context, filters_ukf_f64_workspace_t workspace);

/**
 * @copydoc filters_upf_f32_resample
 */
linears_status_t filters_upf_f64_resample(filters_upf_f64_t *state, double uniform, size_t *indices,
                                          size_t capacity, filters_ukf_f64_workspace_t workspace);

/**
 * @brief Initialize a q15 UPF; contract as filters_upf_f32_init with raw q15 inputs.
 *
 * The particle set is drawn by filters_pf_q15_init; every per-particle
 * covariance row is a raw copy of the prior covariance.
 *
 * @param[in,out] state Non-NULL object whose base, covariances and capacities are set.
 * @param[in] alpha Sigma point spread, strictly positive.
 * @param[in] beta Distribution prior; 2 for Gaussian states.
 * @param[in] kappa Secondary scaling; n+kappa must be positive.
 * @param[in] mean Non-NULL n-by-1 raw q15 prior mean.
 * @param[in] covariance Non-NULL symmetric positive-definite n-by-n raw q15 prior covariance.
 * @param[in] process_noise Non-NULL symmetric SPD n-by-n raw q15 Q.
 * @param[in] measurement_noise Non-NULL symmetric SPD m-by-m raw q15 R.
 * @param[in] sample Non-NULL float standard-normal sampler.
 * @param[in] context Passed to sample unchanged; may be NULL.
 * @param[in] workspace Float data meeting FILTERS_PF_Q_INIT_STORAGE(N,n),
 *                      raw signal staging meeting FILTERS_PF_Q_INIT_SIGNAL_STORAGE(N,n),
 *                      basic bytes meeting FILTERS_PF_Q_PREDICT_BASIC_BYTES(n),
 *                      and FILTERS_PF_PREDICT_DECOMP_STORAGE(n) float elements.
 * @return linears_status_t As filters_pf_q15_init, plus DOMAIN for invalid
 *         sigma hyperparameters.
 */
linears_status_t filters_upf_q15_init(filters_upf_q15_t *state, float alpha, float beta, float kappa,
                                      const linears_matrix_t *mean, const linears_matrix_t *covariance,
                                      const linears_matrix_t *process_noise,
                                      const linears_matrix_t *measurement_noise,
                                      filters_pf_f32_sample_fn sample, void *context,
                                      filters_ukf_q15_workspace_t workspace);

/**
 * @brief One q15 UPF step using the dedicated mixed-Q UKF kernels.
 *
 * Contract as filters_upf_f32_step. The predicted prior stays in local
 * float storage until the measurement update; it need not fit q15 when
 * the posterior does. New particles, posterior covariances and weights
 * are staged raw and committed together after the whole set succeeds.
 *
 * @param[in,out] state Non-NULL initialized q15 filter.
 * @param[in] measurement Non-NULL m-by-1 raw q15 z.
 * @param[in] model Non-NULL float callbacks; transition and observation not NULL.
 * @param[in] sample Non-NULL float standard-normal sampler.
 * @param[in] sample_context Passed to sample unchanged; may be NULL.
 * @param[in] workspace Float data meeting FILTERS_UPF_Q_STEP_STORAGE(N,n,m),
 *                      raw signal staging meeting FILTERS_UPF_Q_STEP_SIGNAL_STORAGE(N,n,m),
 *                      basic bytes meeting FILTERS_UPF_Q_STEP_BASIC_BYTES(n,m),
 *                      and FILTERS_UPF_Q_STEP_DECOMP_STORAGE(n,m) float elements.
 * @return linears_status_t As filters_upf_f32_step. RANGE also covers a new
 *         particle or covariance element outside [-1,+1]; nothing is
 *         committed then.
 * @note Rows run predict, update, sample and density callbacks in order,
 *       then undergo their final range check and raw staging. A failure
 *       returns immediately without processing later rows; persistent
 *       particles, covariances and weights remain unchanged.
 */
linears_status_t filters_upf_q15_step(filters_upf_q15_t *state, const linears_matrix_t *measurement,
                                      const filters_upf_f32_model_t *model, filters_pf_f32_sample_fn sample,
                                      void *sample_context, filters_ukf_q15_workspace_t workspace);

/**
 * @brief Systematic q15 UPF resampling; contract as filters_pf_q15_resample.
 *
 * @param[in,out] state Non-NULL initialized q15 filter.
 * @param[in] uniform Draw in [0,1).
 * @param[out] indices Array of at least N size_t entries.
 * @param[in] capacity Available indices entries.
 * @param[in] workspace Ignored; raw weights are read directly without scratch.
 * @return linears_status_t As filters_pf_f32_resample.
 */
linears_status_t filters_upf_q15_resample(filters_upf_q15_t *state, float uniform, size_t *indices,
                                          size_t capacity, filters_ukf_q15_workspace_t workspace);

/**
 * @brief Initialize a q31 UPF; contract as filters_upf_q15_init with int32_t/2^31 inputs.
 *
 * @param[in,out] state Non-NULL object whose base, covariances and capacities are set.
 * @param[in] alpha Sigma point spread, strictly positive.
 * @param[in] beta Distribution prior; 2 for Gaussian states.
 * @param[in] kappa Secondary scaling; n+kappa must be positive.
 * @param[in] mean Non-NULL n-by-1 raw q31 prior mean.
 * @param[in] covariance Non-NULL symmetric positive-definite n-by-n raw q31 prior covariance.
 * @param[in] process_noise Non-NULL symmetric SPD n-by-n raw q31 Q.
 * @param[in] measurement_noise Non-NULL symmetric SPD m-by-m raw q31 R.
 * @param[in] sample Non-NULL double standard-normal sampler.
 * @param[in] context Passed to sample unchanged; may be NULL.
 * @param[in] workspace Double data meeting FILTERS_PF_Q_INIT_STORAGE(N,n),
 *                      raw signal staging meeting FILTERS_PF_Q_INIT_SIGNAL_STORAGE(N,n),
 *                      basic bytes meeting FILTERS_PF_Q_PREDICT_BASIC_BYTES(n),
 *                      and FILTERS_PF_PREDICT_DECOMP_STORAGE(n) double elements.
 * @return linears_status_t As filters_upf_q15_init.
 */
linears_status_t filters_upf_q31_init(filters_upf_q31_t *state, double alpha, double beta, double kappa,
                                      const linears_matrix_t *mean, const linears_matrix_t *covariance,
                                      const linears_matrix_t *process_noise,
                                      const linears_matrix_t *measurement_noise,
                                      filters_pf_f64_sample_fn sample, void *context,
                                      filters_ukf_q31_workspace_t workspace);

/**
 * @brief One q31 UPF step using the dedicated mixed-Q UKF kernels.
 *
 * Contract as filters_upf_q15_step, with double local intermediates and
 * int32_t signal staging.
 *
 * @param[in,out] state Non-NULL initialized q31 filter.
 * @param[in] measurement Non-NULL m-by-1 raw q31 z.
 * @param[in] model Non-NULL double callbacks; transition and observation not NULL.
 * @param[in] sample Non-NULL double standard-normal sampler.
 * @param[in] sample_context Passed to sample unchanged; may be NULL.
 * @param[in] workspace Double data meeting FILTERS_UPF_Q_STEP_STORAGE(N,n,m),
 *                      raw signal staging meeting FILTERS_UPF_Q_STEP_SIGNAL_STORAGE(N,n,m),
 *                      basic bytes meeting FILTERS_UPF_Q_STEP_BASIC_BYTES(n,m),
 *                      and FILTERS_UPF_Q_STEP_DECOMP_STORAGE(n,m) double elements.
 * @return linears_status_t As filters_upf_q15_step.
 */
linears_status_t filters_upf_q31_step(filters_upf_q31_t *state, const linears_matrix_t *measurement,
                                      const filters_upf_f64_model_t *model, filters_pf_f64_sample_fn sample,
                                      void *sample_context, filters_ukf_q31_workspace_t workspace);

/**
 * @brief Systematic q31 UPF resampling; contract as filters_pf_q31_resample.
 *
 * @param[in,out] state Non-NULL initialized q31 filter.
 * @param[in] uniform Draw in [0,1).
 * @param[out] indices Array of at least N size_t entries.
 * @param[in] capacity Available indices entries.
 * @param[in] workspace Ignored; raw weights are read directly without scratch.
 * @return linears_status_t As filters_pf_f32_resample.
 */
linears_status_t filters_upf_q31_resample(filters_upf_q31_t *state, double uniform, size_t *indices,
                                          size_t capacity, filters_ukf_q31_workspace_t workspace);

#ifdef __cplusplus
}
#endif
#endif
