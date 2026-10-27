/**
 * @file filters_pf.h
 * @brief Caller-owned bootstrap particle filters (PF) and their kernels.
 *
 * A filter keeps N particles (one n-vector per row), their normalized
 * weights, and the noise matrices Q and R in caller-provided storage.
 * predict propagates every particle through the caller's transition f and
 * adds Gaussian process noise L_Q*xi with L_Q*L_Q^T = Q and caller-supplied
 * standard-normal draws xi. update multiplies the weights by the Gaussian
 * likelihood of the measurement given h(particle) and R, then normalizes.
 * resample performs systematic resampling from a caller-supplied uniform
 * draw and resets the weights. estimate exports the weighted mean and
 * covariance. No function allocates, performs I/O, uses variable-size stack
 * storage or mutable global state, and none contains a random generator:
 * callers own their generator and hand over draws through callbacks or
 * arguments. Serialize calls sharing a state or workspace.
 *
 * f32/f64 filters store particles, weights, Q and R as float/double and
 * their callbacks and kernels see those values directly. q15/q31 filters
 * store every persistent array in the raw signal type (int16_t/2^15,
 * int32_t/2^31). Their dedicated kernels use integer weight arithmetic,
 * exact integer resampling and integer statistical MACs. Callback rows,
 * Cholesky factors, exponentials and genuinely extended-range products use
 * float for q15 and double for q31. Typed Q workspaces separate raw staging
 * from this local floating work. Final particles and estimates must lie
 * in [-1,+1]; RANGE preserves the entire transaction. See the q15 state
 * type for weight scaling. Q calls process one particle at a time and stop
 * at the first failure; callback side effects are not rolled back.
 *
 * The transition, observation, residual and workspace types come from
 * filters_ukf.h. Floating estimate calls share its UKF kernels; Q estimate
 * uses dedicated integer statistics. The floating PF kernels (perturb,
 * gaussian_exponent, reweight,
 * effective_sample_size, resample_indices, gather) are public so unscented
 * and other particle filters can reuse them. Failure preserves particles,
 * weights and descriptors; scratch may change. See PF.md.
 */
#ifndef FILTERS_PF_H
#define FILTERS_PF_H

#include "filters_ukf.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Single-precision standard-normal sampler.
 *
 * @param[out] out Non-NULL k-by-1 view; every element must receive an
 *                 independent finite N(0,1) draw.
 * @param[in] context Caller pointer passed through unchanged; may be NULL.
 * @return linears_status_t LINEARS_OK or an error returned verbatim.
 */
typedef linears_status_t (*filters_pf_f32_sample_fn)(linears_matrix_t *out, void *context);

/**
 * @copydoc filters_pf_f32_sample_fn
 */
typedef linears_status_t (*filters_pf_f64_sample_fn)(linears_matrix_t *out, void *context);

/**
 * @brief Caller-owned single-precision particle filter.
 *
 * Before init, point particles at an N-by-n and weights at an N-by-1 float
 * array, process_noise at n-by-n and measurement_noise at m-by-m float
 * arrays (N,n,m>0), all naturally aligned and mutually disjoint, and set the
 * capacities in elements. Init fills all four; it never changes shapes,
 * addresses or capacities. Q and R contents are re-read and re-validated on
 * every call and may be rewritten between calls. Weights are a public field
 * so the kernels can be applied to them directly.
 */
typedef struct {
    linears_matrix_t particles; /**< N-by-n particles, one per row. */
    linears_matrix_t weights; /**< N-by-1 normalized weights; sum to one. */
    linears_matrix_t process_noise; /**< n-by-n symmetric SPD Q. */
    linears_matrix_t measurement_noise; /**< m-by-m symmetric SPD R. */
    size_t particles_capacity; /**< Allocated particle elements; at least N*n. */
    size_t weights_capacity; /**< Allocated weight elements; at least N. */
    size_t process_noise_capacity; /**< Allocated Q elements; at least n*n. */
    size_t measurement_noise_capacity; /**< Allocated R elements; at least m*m. */
} filters_pf_f32_t;

/**
 * @copydoc filters_pf_f32_t
 */
typedef struct {
    linears_matrix_t particles; /**< N-by-n particles, one per row. */
    linears_matrix_t weights; /**< N-by-1 normalized weights; sum to one. */
    linears_matrix_t process_noise; /**< n-by-n symmetric SPD Q. */
    linears_matrix_t measurement_noise; /**< m-by-m symmetric SPD R. */
    size_t particles_capacity; /**< Allocated particle elements; at least N*n. */
    size_t weights_capacity; /**< Allocated weight elements; at least N. */
    size_t process_noise_capacity; /**< Allocated Q elements; at least n*n. */
    size_t measurement_noise_capacity; /**< Allocated R elements; at least m*m. */
} filters_pf_f64_t;

/**
 * @brief Caller-owned q15 particle filter; every array holds raw int16_t/2^15 values.
 *
 * Storage rules follow filters_pf_f32_t with int16_t elements. Callbacks
 * and the standard-normal sampler retain float rows, including values
 * outside [-1,+1]. The typed q15 workspace stages particles and weights in
 * int16_t arrays. Integer MACs handle bounded products; float is local to
 * decomposition, callback and extended-range work. Particles must stay
 * inside [-1,+1]; init and predict commit nothing on an out-of-range draw.
 *
 * Raw weights are scaled so that the largest equals INT16_MAX instead of
 * summing to one: a sum-normalized vector would leave most of the 15-bit
 * range unused for large N. They define the distribution up to that common
 * factor, which every consumer (update, resample, estimate and the q15
 * effective sample size) divides out. A weight below 2^-16 of the largest
 * rounds to zero, so a particle more than about 11 nats behind the best one
 * is lost at the commit.
 */
typedef struct {
    linears_matrix_t particles; /**< N-by-n raw q15 particles, one per row. */
    linears_matrix_t weights; /**< N-by-1 raw q15 weights; largest is INT16_MAX. */
    linears_matrix_t process_noise; /**< n-by-n symmetric SPD raw q15 Q. */
    linears_matrix_t measurement_noise; /**< m-by-m symmetric SPD raw q15 R. */
    size_t particles_capacity; /**< Allocated particle elements; at least N*n. */
    size_t weights_capacity; /**< Allocated weight elements; at least N. */
    size_t process_noise_capacity; /**< Allocated Q elements; at least n*n. */
    size_t measurement_noise_capacity; /**< Allocated R elements; at least m*m. */
} filters_pf_q15_t;

/**
 * @brief Caller-owned q31 particle filter; every array holds raw int32_t/2^31 values.
 *
 * Rules of filters_pf_q15_t with int32_t elements, integer MACs, double
 * local working arithmetic, f64 callbacks and sampler, and a typed q31
 * workspace. The largest raw
 * weight is INT32_MAX; a weight below 2^-32 of it (about 22 nats behind)
 * rounds to zero.
 */
typedef struct {
    linears_matrix_t particles; /**< N-by-n raw q31 particles, one per row. */
    linears_matrix_t weights; /**< N-by-1 raw q31 weights; largest is INT32_MAX. */
    linears_matrix_t process_noise; /**< n-by-n symmetric SPD raw q31 Q. */
    linears_matrix_t measurement_noise; /**< m-by-m symmetric SPD raw q31 R. */
    size_t particles_capacity; /**< Allocated particle elements; at least N*n. */
    size_t weights_capacity; /**< Allocated weight elements; at least N. */
    size_t process_noise_capacity; /**< Allocated Q elements; at least n*n. */
    size_t measurement_noise_capacity; /**< Allocated R elements; at least m*m. */
} filters_pf_q31_t;

/**
 * @brief Working elements for perturb: 2*k for k-vectors; SIZE_MAX on overflow.
 */
#define FILTERS_PF_PERTURB_STORAGE(k) LINEARS_SIZE_MUL(2, (k))

/**
 * @brief Basic scratch bytes for perturb of k-vectors; SIZE_MAX on overflow.
 */
#define FILTERS_PF_PERTURB_BASIC_BYTES(k) LINEARS_BASIC_WORKSPACE_BYTES((k), 1)

/**
 * @brief Working elements for gaussian_exponent of k-vectors: k.
 */
#define FILTERS_PF_GAUSSIAN_EXPONENT_STORAGE(k) ((size_t)(k))

/**
 * @brief Basic scratch bytes for gaussian_exponent: one unit.
 */
#define FILTERS_PF_GAUSSIAN_EXPONENT_BASIC_BYTES LINEARS_BASIC_WORKSPACE_BYTES(1, 1)

/**
 * @brief Decomposition elements for gaussian_exponent of k-vectors: k*k + k.
 */
#define FILTERS_PF_GAUSSIAN_EXPONENT_DECOMP_STORAGE(k) LINEARS_SOLVE_WORKSPACE((k), 1)

/**
 * @brief Floating working elements for init and predict: N*n + n*n + 2*n.
 *
 * N, n are positive and side-effect-free; SIZE_MAX signals overflow.
 */
#define FILTERS_PF_PREDICT_STORAGE(N, n) \
    LINEARS_SIZE_ADD(LINEARS_SIZE_MUL((N), (n)), \
                     LINEARS_SIZE_ADD(LINEARS_SIZE_MUL((n), (n)), LINEARS_SIZE_MUL(2, (n))))

/**
 * @brief Local float/double elements for Q init: 2*n*n + 3*n, independent of N.
 */
#define FILTERS_PF_Q_INIT_STORAGE(N, n) \
    LINEARS_SIZE_ADD(LINEARS_SIZE_MUL(2, LINEARS_SIZE_MUL((n), (n))), LINEARS_SIZE_MUL(3, (n)))

/**
 * @brief Raw signal elements for Q init: N*n staged particles.
 */
#define FILTERS_PF_Q_INIT_SIGNAL_STORAGE(N, n) LINEARS_SIZE_MUL((N), (n))

/**
 * @brief Local float/double elements for Q predict: 2*n*n + 3*n.
 */
#define FILTERS_PF_Q_PREDICT_STORAGE(N, n) FILTERS_PF_Q_INIT_STORAGE((N), (n))

/**
 * @brief Raw signal elements for Q predict: N*n staged particles.
 */
#define FILTERS_PF_Q_PREDICT_SIGNAL_STORAGE(N, n) LINEARS_SIZE_MUL((N), (n))

/**
 * @brief Q init and predict need no basic scratch; decomposition is separate.
 */
#define FILTERS_PF_Q_PREDICT_BASIC_BYTES(n) ((size_t)0)

/**
 * @brief Basic scratch bytes for floating init and predict.
 */
#define FILTERS_PF_PREDICT_BASIC_BYTES(n) LINEARS_BASIC_WORKSPACE_BYTES((n), 1)

/**
 * @brief Decomposition elements for init and predict: the Cholesky of P0 or Q.
 */
#define FILTERS_PF_PREDICT_DECOMP_STORAGE(n) LINEARS_CHOLESKY_WORKSPACE(n)

/**
 * @brief Floating working elements for update: 2*N*m + 2*N + m*m + m.
 */
#define FILTERS_PF_UPDATE_STORAGE(N, m) \
    LINEARS_SIZE_ADD(LINEARS_SIZE_MUL((N), LINEARS_SIZE_ADD(LINEARS_SIZE_MUL(2, (m)), 2)), \
                     LINEARS_SIZE_ADD(LINEARS_SIZE_MUL((m), (m)), (m)))

/**
 * @brief Local float/double elements for Q update: N + 2*m*m + n + 3*m.
 *
 * Only the scalar Gaussian exponents grow with N, not callback row storage.
 */
#define FILTERS_PF_Q_UPDATE_STORAGE(N, n, m) \
    LINEARS_SIZE_ADD((N), LINEARS_SIZE_ADD(LINEARS_SIZE_MUL(2, LINEARS_SIZE_MUL((m), (m))), \
                                          LINEARS_SIZE_ADD((n), LINEARS_SIZE_MUL(3, (m)))))

/**
 * @brief Raw signal elements for Q update: N staged weights.
 */
#define FILTERS_PF_Q_UPDATE_SIGNAL_STORAGE(N, n, m) ((size_t)(N))

/**
 * @brief Q update uses no basic scratch.
 */
#define FILTERS_PF_Q_UPDATE_BASIC_BYTES(m) ((size_t)0)

/**
 * @brief Q update decomposition elements: m*m for the Cholesky factorization.
 */
#define FILTERS_PF_Q_UPDATE_DECOMP_STORAGE(m) LINEARS_CHOLESKY_WORKSPACE(m)

/**
 * @brief Basic scratch bytes for floating update.
 */
#define FILTERS_PF_UPDATE_BASIC_BYTES(m) LINEARS_BASIC_WORKSPACE_BYTES((m), 1)

/**
 * @brief Decomposition elements for update: Cholesky check of R and the solves.
 */
#define FILTERS_PF_UPDATE_DECOMP_STORAGE(m) LINEARS_SOLVE_WORKSPACE((m), 1)

/**
 * @brief Q resample needs no floating scratch.
 */
#define FILTERS_PF_Q_RESAMPLE_STORAGE(N) ((size_t)0)

/**
 * @brief Q resample needs no raw scratch; rows are gathered in place.
 */
#define FILTERS_PF_Q_RESAMPLE_SIGNAL_STORAGE(N) ((size_t)0)

/**
 * @brief Working elements for estimate: 3*N*n + 2*n*n + n; SIZE_MAX on overflow.
 */
#define FILTERS_PF_ESTIMATE_STORAGE(N, n) \
    LINEARS_SIZE_ADD(LINEARS_SIZE_MUL(3, LINEARS_SIZE_MUL((N), (n))), \
                     LINEARS_SIZE_ADD(LINEARS_SIZE_MUL(2, LINEARS_SIZE_MUL((n), (n))), (n)))

/**
 * @brief Local float/double elements for Q estimate: n*n + 3*n, independent of N.
 */
#define FILTERS_PF_Q_ESTIMATE_STORAGE(N, n) \
    LINEARS_SIZE_ADD(LINEARS_SIZE_MUL((n), (n)), LINEARS_SIZE_MUL(3, (n)))

/**
 * @brief Raw signal elements for Q estimate: n*n + 2*n, independent of N.
 */
#define FILTERS_PF_Q_ESTIMATE_SIGNAL_STORAGE(N, n) \
    LINEARS_SIZE_ADD(LINEARS_SIZE_MUL((n), (n)), LINEARS_SIZE_MUL(2, (n)))

/**
 * @brief Q estimate basic bytes: two integer accumulator words per covariance element.
 *
 * These are exact scalar integer sums, not wide fixed signal arrays.
 */
#define FILTERS_PF_Q_ESTIMATE_BASIC_BYTES(N, n) \
    LINEARS_SIZE_MUL(LINEARS_SIZE_MUL(2, LINEARS_SIZE_MUL((n), (n))), sizeof(linears_workspace_unit_t))

/**
 * @brief Floating estimate basic-scratch bytes; SIZE_MAX on overflow.
 */
#define FILTERS_PF_ESTIMATE_BASIC_BYTES(N, n) \
    LINEARS_BASIC_WORKSPACE_BYTES((n), FILTERS_KALMAN_MAX_SIZE((N), (n)))

/**
 * @brief Add Gaussian noise L*xi to every row of points in single precision.
 *
 * @param[in] points Non-NULL finite N-by-k matrix, one point per row.
 * @param[in] factor Non-NULL finite k-by-k matrix L with L*L^T the noise covariance.
 * @param[in] sample Non-NULL standard-normal sampler, called once per row with a k-by-1 view.
 * @param[in] context Passed to sample unchanged; may be NULL.
 * @param[in,out] out Non-NULL N-by-k destination; either exactly the points
 *                    storage (in place) or disjoint from it.
 * @param[in] workspace Arrays meeting FILTERS_PF_PERTURB_STORAGE(k) and
 *                      FILTERS_PF_PERTURB_BASIC_BYTES(k); decomposition is ignored.
 * @return linears_status_t OK, ARGUMENT, DIMENSION, SIZE, WORKSPACE, ALIAS,
 *         RANGE for a nonfinite draw or result, or the sampler's error.
 * @note Rows are processed in order; out is partially written after a failure.
 */
linears_status_t filters_pf_f32_perturb(const linears_matrix_t *points, const linears_matrix_t *factor,
                                        filters_pf_f32_sample_fn sample, void *context,
                                        linears_matrix_t *out, filters_ukf_f32_workspace_t workspace);

/**
 * @brief Row-wise Gaussian exponents (1/2)*d^T*(L*L^T)^-1*d in single precision.
 *
 * @param[in] deviations Non-NULL finite N-by-k matrix of residual rows d.
 * @param[in] factor Non-NULL finite nonsingular k-by-k L, typically the Cholesky
 *                   factor of the covariance.
 * @param[in,out] out Non-NULL N-by-1 destination of nonnegative exponents.
 * @param[in] workspace Arrays meeting FILTERS_PF_GAUSSIAN_EXPONENT_STORAGE(k),
 *                      _BASIC_BYTES and _DECOMP_STORAGE(k).
 * @return linears_status_t OK, ARGUMENT, DIMENSION, SIZE, WORKSPACE, ALIAS,
 *         RANGE, or SINGULAR when L has a zero pivot at working precision.
 * @note Each row solves L*y = d by the shared pivoted LU and returns
 *       (1/2)*y^T*y; the determinant term is constant across rows and omitted.
 */
linears_status_t filters_pf_f32_gaussian_exponent(const linears_matrix_t *deviations,
                                                  const linears_matrix_t *factor, linears_matrix_t *out,
                                                  filters_ukf_f32_workspace_t workspace);

/**
 * @brief Multiply weights by exp(-exponent), shift-stabilized, and normalize.
 *
 * w[i] <- w[i]*exp(-(e[i]-min(e))) / sum, so the result sums to one.
 *
 * @param[in,out] weights Non-NULL finite nonnegative N-by-1 weights; replaced
 *                        by normalized weights. Partially modified on failure.
 * @param[in] exponents Non-NULL finite N-by-1 exponents of any sign.
 * @return linears_status_t OK, ARGUMENT, DIMENSION, SIZE, ALIAS, RANGE, or
 *         DOMAIN for a negative weight or a zero weight sum.
 */
linears_status_t filters_pf_f32_reweight(linears_matrix_t *weights, const linears_matrix_t *exponents);

/**
 * @brief Effective sample size (sum w)^2 / sum w^2 of a weight vector.
 *
 * @param[in] weights Non-NULL finite nonnegative N-by-1 weights.
 * @param[out] out Non-NULL destination in [1,N]; unchanged on failure.
 * @return linears_status_t OK, ARGUMENT, DIMENSION, SIZE, RANGE, or DOMAIN
 *         for a negative weight or an all-zero vector.
 */
linears_status_t filters_pf_f32_effective_sample_size(const linears_matrix_t *weights, float *out);

/**
 * @brief Systematic resampling indices from one uniform draw.
 *
 * Target i is (uniform + i)/N scaled by the weight total; indices[i] is the
 * first particle whose cumulative weight reaches it. Indices are nondecreasing.
 *
 * @param[in] weights Non-NULL finite nonnegative N-by-1 weights with positive sum.
 * @param[in] uniform Draw in [0,1).
 * @param[out] indices Array of at least N size_t entries; written only on success.
 * @param[in] capacity Available indices entries.
 * @return linears_status_t OK, ARGUMENT, DIMENSION, SIZE, WORKSPACE for a
 *         short capacity, RANGE, or DOMAIN for uniform outside [0,1), a
 *         negative weight or a zero total.
 */
linears_status_t filters_pf_f32_resample_indices(const linears_matrix_t *weights, float uniform,
                                                 size_t *indices, size_t capacity);

/**
 * @brief Copy selected rows: out[i] = points[indices[i]].
 *
 * @param[in] points Non-NULL N-by-k matrix.
 * @param[in] indices Non-NULL array of count row indices below N.
 * @param[in] count Number of output rows; equals out->rows.
 * @param[in,out] out Non-NULL count-by-k destination disjoint from points.
 * @return linears_status_t OK, ARGUMENT (including an index of N or more),
 *         DIMENSION, SIZE or ALIAS.
 */
linears_status_t filters_pf_f32_gather(const linears_matrix_t *points, const size_t *indices,
                                       size_t count, linears_matrix_t *out);

/**
 * @brief Initialize a single-precision particle filter from a Gaussian prior.
 *
 * Particles are mean + L*xi with L*L^T = covariance and sampler draws xi;
 * weights become 1/N; Q and R are copied.
 *
 * @param[in,out] state Non-NULL object whose descriptors and capacities are set.
 * @param[in] mean Non-NULL finite n-by-1 prior mean.
 * @param[in] covariance Non-NULL finite symmetric positive-definite n-by-n prior covariance.
 * @param[in] process_noise Non-NULL finite symmetric SPD n-by-n Q; may exactly
 *                          alias its destination or be the state's own descriptor.
 * @param[in] measurement_noise Non-NULL finite symmetric SPD m-by-m R; same alias rule.
 * @param[in] sample Non-NULL standard-normal sampler.
 * @param[in] context Passed to sample unchanged; may be NULL.
 * @param[in] workspace Arrays meeting FILTERS_PF_PREDICT_STORAGE(N,n),
 *                      FILTERS_PF_PREDICT_BASIC_BYTES(n) and
 *                      FILTERS_PF_PREDICT_DECOMP_STORAGE(n).
 * @return linears_status_t OK, ARGUMENT, DIMENSION, SIZE, WORKSPACE, ALIAS,
 *         RANGE, DOMAIN (asymmetry or negative diagonal),
 *         NOT_POSITIVE_DEFINITE for the prior covariance, or the sampler's error.
 * @note Particles are staged in scratch, so a failing sampler leaves the
 *       state unchanged.
 */
linears_status_t filters_pf_f32_init(filters_pf_f32_t *state, const linears_matrix_t *mean,
                                     const linears_matrix_t *covariance, const linears_matrix_t *process_noise,
                                     const linears_matrix_t *measurement_noise, filters_pf_f32_sample_fn sample,
                                     void *context, filters_ukf_f32_workspace_t workspace);

/**
 * @brief Propagate every particle through f and add process noise.
 *
 * @param[in,out] state Non-NULL initialized filter.
 * @param[in] transition Non-NULL deterministic map f (n-by-1 to n-by-1).
 * @param[in] transition_context Passed to transition unchanged; may be NULL.
 * @param[in] sample Non-NULL standard-normal sampler, called once per particle.
 * @param[in] sample_context Passed to sample unchanged; may be NULL.
 * @param[in] workspace Arrays meeting FILTERS_PF_PREDICT_STORAGE(N,n),
 *                      FILTERS_PF_PREDICT_BASIC_BYTES(n) and
 *                      FILTERS_PF_PREDICT_DECOMP_STORAGE(n).
 * @return linears_status_t OK, ARGUMENT, DIMENSION, SIZE, WORKSPACE, ALIAS,
 *         RANGE, DOMAIN, NOT_POSITIVE_DEFINITE when Q has no Cholesky factor
 *         (checked before any callback), or a callback error.
 * @note Weights are unchanged. Failure preserves particles.
 */
linears_status_t filters_pf_f32_predict(filters_pf_f32_t *state, filters_ukf_f32_fn transition,
                                        void *transition_context, filters_pf_f32_sample_fn sample,
                                        void *sample_context, filters_ukf_f32_workspace_t workspace);

/**
 * @brief Reweight particles by the Gaussian likelihood of a measurement.
 *
 * @param[in,out] state Non-NULL initialized filter with m-by-m R.
 * @param[in] measurement Non-NULL finite m-by-1 z.
 * @param[in] observation Non-NULL map h (n-by-1 to m-by-1).
 * @param[in] observation_context Passed to observation unchanged; may be NULL.
 * @param[in] residual Measurement-space residual, or NULL for plain subtraction.
 * @param[in] residual_context Passed to residual unchanged; may be NULL.
 * @param[in] workspace Arrays meeting FILTERS_PF_UPDATE_STORAGE(N,m),
 *                      FILTERS_PF_UPDATE_BASIC_BYTES(m) and
 *                      FILTERS_PF_UPDATE_DECOMP_STORAGE(m).
 * @return linears_status_t OK, ARGUMENT, DIMENSION, SIZE, WORKSPACE, ALIAS,
 *         RANGE, DOMAIN (asymmetric R or zero weight sum),
 *         NOT_POSITIVE_DEFINITE for R (checked before any callback), SINGULAR,
 *         or a callback error.
 * @note Particles are unchanged. Failure preserves weights.
 */
linears_status_t filters_pf_f32_update(filters_pf_f32_t *state, const linears_matrix_t *measurement,
                                       filters_ukf_f32_fn observation, void *observation_context,
                                       filters_ukf_f32_residual_fn residual, void *residual_context,
                                       filters_ukf_f32_workspace_t workspace);

/**
 * @brief Systematically resample the particles and reset the weights to 1/N.
 *
 * The particle rows are gathered in place, which the nondecreasing
 * systematic indices allow without staging.
 *
 * @param[in,out] state Non-NULL initialized filter.
 * @param[in] uniform Draw in [0,1).
 * @param[out] indices Array of at least N size_t entries receiving the source
 *                     index of every new particle, for resampling caller-owned
 *                     per-particle data alongside.
 * @param[in] capacity Available indices entries.
 * @param[in] workspace Ignored by the f32 and f64 filters, which need no
 *                      scratch; the parameter exists so the four formats share
 *                      one signature. Pass any value.
 * @return linears_status_t OK, ARGUMENT, DIMENSION, SIZE, WORKSPACE for a
 *         short capacity, ALIAS, RANGE or DOMAIN (uniform outside [0,1) or
 *         invalid weights).
 * @note Call when filters_pf_*_effective_sample_size of the weights falls
 *       below the caller's threshold. Failure preserves particles and weights.
 */
linears_status_t filters_pf_f32_resample(filters_pf_f32_t *state, float uniform, size_t *indices,
                                         size_t capacity, filters_ukf_f32_workspace_t workspace);

/**
 * @brief Export the weighted mean and covariance of the particles.
 *
 * @param[in] state Non-NULL initialized filter.
 * @param[in] residual State-space residual, or NULL for plain subtraction.
 * @param[in] residual_context Passed to residual unchanged; may be NULL.
 * @param[in,out] mean Non-NULL n-by-1 destination.
 * @param[in,out] covariance Non-NULL n-by-n destination; symmetrized.
 * @param[in] workspace Arrays meeting FILTERS_PF_ESTIMATE_STORAGE(N,n) and
 *                      FILTERS_PF_ESTIMATE_BASIC_BYTES(N,n); decomposition is ignored.
 * @return linears_status_t OK, ARGUMENT, DIMENSION, SIZE, WORKSPACE, ALIAS,
 *         RANGE, or a residual callback error.
 * @note Both outputs are written only after every value succeeded.
 */
linears_status_t filters_pf_f32_estimate(const filters_pf_f32_t *state, filters_ukf_f32_residual_fn residual,
                                         void *residual_context, linears_matrix_t *mean,
                                         linears_matrix_t *covariance, filters_ukf_f32_workspace_t workspace);

/**
 * @copydoc filters_pf_f32_perturb
 */
linears_status_t filters_pf_f64_perturb(const linears_matrix_t *points, const linears_matrix_t *factor,
                                        filters_pf_f64_sample_fn sample, void *context,
                                        linears_matrix_t *out, filters_ukf_f64_workspace_t workspace);

/**
 * @copydoc filters_pf_f32_gaussian_exponent
 */
linears_status_t filters_pf_f64_gaussian_exponent(const linears_matrix_t *deviations,
                                                  const linears_matrix_t *factor, linears_matrix_t *out,
                                                  filters_ukf_f64_workspace_t workspace);

/**
 * @copydoc filters_pf_f32_reweight
 */
linears_status_t filters_pf_f64_reweight(linears_matrix_t *weights, const linears_matrix_t *exponents);

/**
 * @copydoc filters_pf_f32_effective_sample_size
 */
linears_status_t filters_pf_f64_effective_sample_size(const linears_matrix_t *weights, double *out);

/**
 * @copydoc filters_pf_f32_resample_indices
 */
linears_status_t filters_pf_f64_resample_indices(const linears_matrix_t *weights, double uniform,
                                                 size_t *indices, size_t capacity);

/**
 * @copydoc filters_pf_f32_gather
 */
linears_status_t filters_pf_f64_gather(const linears_matrix_t *points, const size_t *indices,
                                       size_t count, linears_matrix_t *out);

/**
 * @copydoc filters_pf_f32_init
 */
linears_status_t filters_pf_f64_init(filters_pf_f64_t *state, const linears_matrix_t *mean,
                                     const linears_matrix_t *covariance, const linears_matrix_t *process_noise,
                                     const linears_matrix_t *measurement_noise, filters_pf_f64_sample_fn sample,
                                     void *context, filters_ukf_f64_workspace_t workspace);

/**
 * @copydoc filters_pf_f32_predict
 */
linears_status_t filters_pf_f64_predict(filters_pf_f64_t *state, filters_ukf_f64_fn transition,
                                        void *transition_context, filters_pf_f64_sample_fn sample,
                                        void *sample_context, filters_ukf_f64_workspace_t workspace);

/**
 * @copydoc filters_pf_f32_update
 */
linears_status_t filters_pf_f64_update(filters_pf_f64_t *state, const linears_matrix_t *measurement,
                                       filters_ukf_f64_fn observation, void *observation_context,
                                       filters_ukf_f64_residual_fn residual, void *residual_context,
                                       filters_ukf_f64_workspace_t workspace);

/**
 * @copydoc filters_pf_f32_resample
 */
linears_status_t filters_pf_f64_resample(filters_pf_f64_t *state, double uniform, size_t *indices,
                                         size_t capacity, filters_ukf_f64_workspace_t workspace);

/**
 * @copydoc filters_pf_f32_estimate
 */
linears_status_t filters_pf_f64_estimate(const filters_pf_f64_t *state, filters_ukf_f64_residual_fn residual,
                                         void *residual_context, linears_matrix_t *mean,
                                         linears_matrix_t *covariance, filters_ukf_f64_workspace_t workspace);

/**
 * @brief Effective sample size of raw q15 weights, with exact integer sums.
 *
 * The result is independent of the common weight scale, so it applies to
 * the filter's INT16_MAX-scaled weights unchanged.
 *
 * @param[in] weights Non-NULL nonnegative N-by-1 raw q15 weights.
 * @param[out] out Non-NULL destination in [1,N]; unchanged on failure.
 * @return linears_status_t OK, ARGUMENT, DIMENSION, SIZE, or DOMAIN for a
 *         negative weight or an all-zero vector.
 */
linears_status_t filters_pf_q15_effective_sample_size(const linears_matrix_t *weights, float *out);

/**
 * @brief Initialize a q15 particle filter from a raw q15 Gaussian prior.
 *
 * Contract as filters_pf_f32_init with int16_t/2^15 mean, covariance, Q and
 * R and the f32 sampler. Cholesky and one draw use local float work;
 * bounded products use integer MACs. Raw particles are staged until the
 * complete transaction succeeds; weights all become INT16_MAX.
 *
 * @param[in,out] state Non-NULL object whose descriptors and capacities are set.
 * @param[in] mean Non-NULL n-by-1 raw q15 prior mean.
 * @param[in] covariance Non-NULL symmetric positive-definite n-by-n raw q15 prior covariance.
 * @param[in] process_noise Non-NULL symmetric SPD n-by-n raw q15 Q; alias rules of the f32 filter.
 * @param[in] measurement_noise Non-NULL symmetric SPD m-by-m raw q15 R; same alias rules.
 * @param[in] sample Non-NULL float standard-normal sampler.
 * @param[in] context Passed to sample unchanged; may be NULL.
 * @param[in] workspace Typed q15 workspace meeting FILTERS_PF_Q_INIT_STORAGE(N,n),
 *                      FILTERS_PF_Q_INIT_SIGNAL_STORAGE(N,n),
 *                      FILTERS_PF_Q_PREDICT_BASIC_BYTES(n) and
 *                      FILTERS_PF_PREDICT_DECOMP_STORAGE(n).
 * @return linears_status_t As filters_pf_f32_init. RANGE also covers a drawn
 *         particle element outside [-1,+1]; nothing is committed then.
 * @note Process and range-check particles in order, stopping at the first
 *       error without committing any state. Exact +1 and its rounding cell
 *       narrow to INT16_MAX. Callback side effects are not rolled back.
 */
linears_status_t filters_pf_q15_init(filters_pf_q15_t *state, const linears_matrix_t *mean,
                                     const linears_matrix_t *covariance, const linears_matrix_t *process_noise,
                                     const linears_matrix_t *measurement_noise, filters_pf_f32_sample_fn sample,
                                     void *context, filters_ukf_q15_workspace_t workspace);

/**
 * @brief Predict q15 particles with integer MACs and local extended-range work.
 *
 * Process each particle in order: transition, sample, add factored noise,
 * range-check the final particle, then stage its raw q15 row. Stop at the
 * first failure. A transition output outside [-1,+1] is allowed when noise
 * brings the final particle back; it is never clipped or quantized early.
 * This ordering differs from f32/f64, which propagate all rows first.
 *
 * @param[in,out] state Non-NULL initialized q15 filter.
 * @param[in] transition Non-NULL deterministic float map f (n-by-1 to n-by-1).
 * @param[in] transition_context Passed to transition unchanged; may be NULL.
 * @param[in] sample Non-NULL float standard-normal sampler.
 * @param[in] sample_context Passed to sample unchanged; may be NULL.
 * @param[in] workspace Typed q15 workspace meeting FILTERS_PF_Q_PREDICT_STORAGE(N,n),
 *                      FILTERS_PF_Q_PREDICT_SIGNAL_STORAGE(N,n),
 *                      FILTERS_PF_Q_PREDICT_BASIC_BYTES(n) and
 *                      FILTERS_PF_PREDICT_DECOMP_STORAGE(n).
 * @return linears_status_t As filters_pf_f32_predict. RANGE also covers a
 *         predicted particle element outside [-1,+1]; nothing is committed
 *         then. NOT_POSITIVE_DEFINITE includes positive pivots lost to q15
 *         quantization of Q.
 */
linears_status_t filters_pf_q15_predict(filters_pf_q15_t *state, filters_ukf_f32_fn transition,
                                        void *transition_context, filters_pf_f32_sample_fn sample,
                                        void *sample_context, filters_ukf_q15_workspace_t workspace);

/**
 * @brief Reweight q15 particles using integer products and normalization.
 *
 * Process each particle in order: observation, optional residual, Gaussian
 * exponent. Stop at the first failure. Callback rows can exceed [-1,+1].
 * Exponentials use float; high-resolution integer likelihoods multiply the
 * raw weights, which are rescaled once so the largest equals INT16_MAX.
 * Only N scalar exponents and N raw staged weights grow with particle count.
 *
 * @param[in,out] state Non-NULL initialized q15 filter with m-by-m R.
 * @param[in] measurement Non-NULL m-by-1 raw q15 z.
 * @param[in] observation Non-NULL float map h (n-by-1 to m-by-1).
 * @param[in] observation_context Passed to observation unchanged; may be NULL.
 * @param[in] residual Float measurement-space residual, or NULL for plain subtraction.
 * @param[in] residual_context Passed to residual unchanged; may be NULL.
 * @param[in] workspace Typed q15 workspace meeting FILTERS_PF_Q_UPDATE_STORAGE(N,n,m),
 *                      FILTERS_PF_Q_UPDATE_SIGNAL_STORAGE(N,n,m),
 *                      FILTERS_PF_Q_UPDATE_BASIC_BYTES(m) and
 *                      FILTERS_PF_Q_UPDATE_DECOMP_STORAGE(m).
 * @return linears_status_t As filters_pf_f32_update.
 */
linears_status_t filters_pf_q15_update(filters_pf_q15_t *state, const linears_matrix_t *measurement,
                                       filters_ukf_f32_fn observation, void *observation_context,
                                       filters_ukf_f32_residual_fn residual, void *residual_context,
                                       filters_ukf_q15_workspace_t workspace);

/**
 * @brief Systematic q15 resampling; contract as filters_pf_f32_resample.
 *
 * Integer cumulative weights and quotient/remainder targets preserve the
 * exact binary uniform, including ties. Raw rows are gathered in place and
 * weights reset to INT16_MAX; no scratch or widened weight array is needed.
 *
 * @param[in,out] state Non-NULL initialized q15 filter.
 * @param[in] uniform Draw in [0,1).
 * @param[out] indices Array of at least N size_t entries.
 * @param[in] capacity Available indices entries.
 * @param[in] workspace Ignored; pass any value. No scratch is needed.
 * @return linears_status_t As filters_pf_f32_resample.
 */
linears_status_t filters_pf_q15_resample(filters_pf_q15_t *state, float uniform, size_t *indices,
                                         size_t capacity, filters_ukf_q15_workspace_t workspace);

/**
 * @brief Export the q15 weighted mean and covariance; contract as filters_pf_f32_estimate.
 *
 * Raw particles and weights feed integer weighted sums and centered
 * covariance MACs, with scalar division only after accumulation. Custom
 * residuals receive one float row at a time; products requiring extended
 * range stay local to that row. Both output arrays commit together.
 *
 * @param[in] state Non-NULL initialized q15 filter.
 * @param[in] residual Float state-space residual, or NULL for plain subtraction.
 * @param[in] residual_context Passed to residual unchanged; may be NULL.
 * @param[in,out] mean Non-NULL n-by-1 raw q15 destination.
 * @param[in,out] covariance Non-NULL n-by-n raw q15 destination.
 * @param[in] workspace Typed q15 workspace meeting FILTERS_PF_Q_ESTIMATE_STORAGE(N,n),
 *                      FILTERS_PF_Q_ESTIMATE_SIGNAL_STORAGE(N,n) and
 *                      FILTERS_PF_Q_ESTIMATE_BASIC_BYTES(N,n).
 * @return linears_status_t As filters_pf_f32_estimate. RANGE also covers a
 *         computed mean or covariance element outside [-1,+1]; DOMAIN also
 *         covers an all-zero weight vector. Nothing is written on failure.
 */
linears_status_t filters_pf_q15_estimate(const filters_pf_q15_t *state, filters_ukf_f32_residual_fn residual,
                                         void *residual_context, linears_matrix_t *mean,
                                         linears_matrix_t *covariance, filters_ukf_q15_workspace_t workspace);

/**
 * @brief Effective sample size of raw q31 weights, with exact integer sums.
 *
 * @param[in] weights Non-NULL nonnegative N-by-1 raw q31 weights.
 * @param[out] out Non-NULL destination in [1,N]; unchanged on failure.
 * @return linears_status_t As filters_pf_q15_effective_sample_size.
 */
linears_status_t filters_pf_q31_effective_sample_size(const linears_matrix_t *weights, double *out);

/**
 * @brief Initialize a q31 particle filter; contract as filters_pf_q15_init with int32_t/2^31 inputs.
 *
 * @param[in,out] state Non-NULL object whose descriptors and capacities are set.
 * @param[in] mean Non-NULL n-by-1 raw q31 prior mean.
 * @param[in] covariance Non-NULL symmetric positive-definite n-by-n raw q31 prior covariance.
 * @param[in] process_noise Non-NULL symmetric SPD n-by-n raw q31 Q.
 * @param[in] measurement_noise Non-NULL symmetric SPD m-by-m raw q31 R.
 * @param[in] sample Non-NULL double standard-normal sampler.
 * @param[in] context Passed to sample unchanged; may be NULL.
 * @param[in] workspace Typed q31 workspace meeting FILTERS_PF_Q_INIT_STORAGE(N,n),
 *                      FILTERS_PF_Q_INIT_SIGNAL_STORAGE(N,n),
 *                      FILTERS_PF_Q_PREDICT_BASIC_BYTES(n) and
 *                      FILTERS_PF_PREDICT_DECOMP_STORAGE(n).
 * @return linears_status_t As filters_pf_q15_init; exact +1 narrows to INT32_MAX.
 */
linears_status_t filters_pf_q31_init(filters_pf_q31_t *state, const linears_matrix_t *mean,
                                     const linears_matrix_t *covariance, const linears_matrix_t *process_noise,
                                     const linears_matrix_t *measurement_noise, filters_pf_f64_sample_fn sample,
                                     void *context, filters_ukf_q31_workspace_t workspace);

/**
 * @brief q31 particle prediction; contract and ordering as filters_pf_q15_predict.
 *
 * @param[in,out] state Non-NULL initialized q31 filter.
 * @param[in] transition Non-NULL deterministic double map f (n-by-1 to n-by-1).
 * @param[in] transition_context Passed to transition unchanged; may be NULL.
 * @param[in] sample Non-NULL double standard-normal sampler.
 * @param[in] sample_context Passed to sample unchanged; may be NULL.
 * @param[in] workspace Typed q31 workspace meeting FILTERS_PF_Q_PREDICT_STORAGE(N,n),
 *                      FILTERS_PF_Q_PREDICT_SIGNAL_STORAGE(N,n),
 *                      FILTERS_PF_Q_PREDICT_BASIC_BYTES(n) and
 *                      FILTERS_PF_PREDICT_DECOMP_STORAGE(n).
 * @return linears_status_t As filters_pf_q15_predict.
 */
linears_status_t filters_pf_q31_predict(filters_pf_q31_t *state, filters_ukf_f64_fn transition,
                                        void *transition_context, filters_pf_f64_sample_fn sample,
                                        void *sample_context, filters_ukf_q31_workspace_t workspace);

/**
 * @brief q31 integer reweighting; contract and ordering as filters_pf_q15_update.
 *
 * @param[in,out] state Non-NULL initialized q31 filter with m-by-m R.
 * @param[in] measurement Non-NULL m-by-1 raw q31 z.
 * @param[in] observation Non-NULL double map h (n-by-1 to m-by-1).
 * @param[in] observation_context Passed to observation unchanged; may be NULL.
 * @param[in] residual Double measurement-space residual, or NULL for plain subtraction.
 * @param[in] residual_context Passed to residual unchanged; may be NULL.
 * @param[in] workspace Typed q31 workspace meeting FILTERS_PF_Q_UPDATE_STORAGE(N,n,m),
 *                      FILTERS_PF_Q_UPDATE_SIGNAL_STORAGE(N,n,m),
 *                      FILTERS_PF_Q_UPDATE_BASIC_BYTES(m) and
 *                      FILTERS_PF_Q_UPDATE_DECOMP_STORAGE(m).
 * @return linears_status_t As filters_pf_q15_update.
 */
linears_status_t filters_pf_q31_update(filters_pf_q31_t *state, const linears_matrix_t *measurement,
                                       filters_ukf_f64_fn observation, void *observation_context,
                                       filters_ukf_f64_residual_fn residual, void *residual_context,
                                       filters_ukf_q31_workspace_t workspace);

/**
 * @brief Systematic q31 resampling; contract as filters_pf_q15_resample.
 *
 * @param[in,out] state Non-NULL initialized q31 filter.
 * @param[in] uniform Draw in [0,1).
 * @param[out] indices Array of at least N size_t entries.
 * @param[in] capacity Available indices entries.
 * @param[in] workspace Ignored; pass any value. No scratch is needed.
 * @return linears_status_t As filters_pf_q15_resample.
 */
linears_status_t filters_pf_q31_resample(filters_pf_q31_t *state, double uniform, size_t *indices,
                                         size_t capacity, filters_ukf_q31_workspace_t workspace);

/**
 * @brief Export the q31 weighted mean and covariance; contract as filters_pf_q15_estimate.
 *
 * @param[in] state Non-NULL initialized q31 filter.
 * @param[in] residual Double state-space residual, or NULL for plain subtraction.
 * @param[in] residual_context Passed to residual unchanged; may be NULL.
 * @param[in,out] mean Non-NULL n-by-1 raw q31 destination.
 * @param[in,out] covariance Non-NULL n-by-n raw q31 destination.
 * @param[in] workspace Typed q31 workspace meeting FILTERS_PF_Q_ESTIMATE_STORAGE(N,n),
 *                      FILTERS_PF_Q_ESTIMATE_SIGNAL_STORAGE(N,n) and
 *                      FILTERS_PF_Q_ESTIMATE_BASIC_BYTES(N,n).
 * @return linears_status_t As filters_pf_q15_estimate; exact +1 narrows to INT32_MAX.
 */
linears_status_t filters_pf_q31_estimate(const filters_pf_q31_t *state, filters_ukf_f64_residual_fn residual,
                                         void *residual_context, linears_matrix_t *mean,
                                         linears_matrix_t *covariance, filters_ukf_q31_workspace_t workspace);

#ifdef __cplusplus
}
#endif
#endif
