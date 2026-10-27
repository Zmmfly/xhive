/**
 * @file filters_ukf.h
 * @brief Caller-owned unscented Kalman filters (UKF) built on linears kernels.
 *
 * The filter keeps x (n-by-1), P (n-by-n), Q (n-by-n) and R (m-by-m) in
 * caller-provided storage. Predict and update are separate transactions: a
 * failure preserves x and P (their data and descriptors), but may change
 * scratch. Both regenerate 2n+1 sigma points from the current x and P through
 * a Cholesky factor of (n+lambda)*P, so P must be symmetric positive definite
 * at working precision; a merely positive-semidefinite P fails with
 * NOT_POSITIVE_DEFINITE. Nothing allocates, performs I/O, uses variable-size
 * stack storage or mutable global state. Serialize calls sharing a state or
 * workspace; disjoint instances are reentrant.
 *
 * Nonlinear models are caller callbacks: a transition f(x) for predict, an
 * observation h(x) for update, and optional residual functions a (-) b that
 * replace plain subtraction wherever two points of the same space are
 * differenced. Residuals keep angle-like components from wrapping incorrectly;
 * NULL selects plain subtraction. Every callback receives borrowed views into
 * library scratch and must write every element of its output view.
 *
 * The individual steps (sigma points, propagation, weighted mean, deviations,
 * weighted covariance) are public working-format kernels so particle and
 * unscented particle filters can reuse them with their own weights.
 *
 * f32/f64 callbacks and kernels see float/double matrices. q15/q31 filters
 * store raw signals (int16_t/2^15, int32_t/2^31) in every persistent array
 * and compute in the floating type whose significand covers the signal: q15
 * uses the f32 kernels, callbacks, weights and workspace, q31 the f64 ones.
 * Signals widen once on entry; sigma points and intermediates may exceed the
 * signal interval; the final x/P are range-checked against [-1,+1] and
 * narrowed once with the endpoint policy of linears_numeric.h (exact +1 and
 * the rounding cell reaching it map to the maximum), or the transaction
 * fails with RANGE and commits nothing. See UKF.md for the algorithm,
 * storage layout and validation record.
 */
#ifndef FILTERS_UKF_H
#define FILTERS_UKF_H

#include "filters_kalman.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Number of sigma points for n states: 2n+1, or SIZE_MAX on overflow.
 *
 * n is a positive, side-effect-free state count.
 */
#define FILTERS_UKF_SIGMA_COUNT(n) LINEARS_SIZE_ADD(LINEARS_SIZE_MUL(2, (n)), 1)

/**
 * @brief Single-precision point map used for the transition f and observation h.
 *
 * @param[in] in Non-NULL n-by-1 input point; read-only borrowed view.
 * @param[out] out Non-NULL n-by-1 (transition) or m-by-1 (observation) view;
 *                 every element must be written with a finite value.
 * @param[in] context Caller pointer passed through unchanged; may be NULL.
 * @return linears_status_t LINEARS_OK, or any error, which the caller receives
 *         verbatim and which leaves the filter state unchanged.
 * @note in and out never overlap. The views are only valid during the call.
 *       Sigma points are formed by plain addition and may carry angle-like
 *       components outside their normalized interval; the map must accept them.
 */
typedef linears_status_t (*filters_ukf_f32_fn)(const linears_matrix_t *in,
                                               linears_matrix_t *out, void *context);

/**
 * @brief Single-precision residual out = a (-) b for one space.
 *
 * @param[in] a Non-NULL k-by-1 minuend point.
 * @param[in] b Non-NULL k-by-1 reference point; may share data with a.
 * @param[out] out Non-NULL k-by-1 destination; every element must be finite.
 * @param[in] context Caller pointer passed through unchanged; may be NULL.
 * @return linears_status_t LINEARS_OK or an error returned verbatim.
 * @note Linear components return a-b; angle-like components return the
 *       wrapped difference. out never overlaps a or b.
 */
typedef linears_status_t (*filters_ukf_f32_residual_fn)(const linears_matrix_t *a,
                                                        const linears_matrix_t *b,
                                                        linears_matrix_t *out, void *context);

/**
 * @copydoc filters_ukf_f32_fn
 */
typedef linears_status_t (*filters_ukf_f64_fn)(const linears_matrix_t *in,
                                               linears_matrix_t *out, void *context);

/**
 * @copydoc filters_ukf_f32_residual_fn
 */
typedef linears_status_t (*filters_ukf_f64_residual_fn)(const linears_matrix_t *a,
                                                        const linears_matrix_t *b,
                                                        linears_matrix_t *out, void *context);

/**
 * @brief Single-precision scaled unscented transform weights.
 *
 * lambda = alpha^2*(n+kappa) - n. Only three distinct weights exist; the
 * kernels expand them into 2n+1 element vectors in scratch.
 */
typedef struct {
    float scale; /**< n+lambda > 0; multiplies P before the Cholesky factor. */
    float mean0; /**< W_m[0] = lambda/(n+lambda). */
    float covariance0; /**< W_c[0] = W_m[0] + 1 - alpha^2 + beta; may be negative. */
    float other; /**< W_m[i] = W_c[i] = 1/(2*(n+lambda)) for i >= 1. */
} filters_ukf_f32_weights_t;

/**
 * @copydoc filters_ukf_f32_weights_t
 */
typedef struct {
    double scale; /**< n+lambda > 0; multiplies P before the Cholesky factor. */
    double mean0; /**< W_m[0] = lambda/(n+lambda). */
    double covariance0; /**< W_c[0] = W_m[0] + 1 - alpha^2 + beta; may be negative. */
    double other; /**< W_m[i] = W_c[i] = 1/(2*(n+lambda)) for i >= 1. */
} filters_ukf_f64_weights_t;

/**
 * @brief Single-precision scratch; same arrays as the linear matrix filter.
 *
 * Supply real float, linears_workspace_unit_t and float arrays sized by the
 * FILTERS_UKF_* macros of the operation; mutually disjoint and disjoint from
 * state, inputs, outputs and descriptors. Only the required prefix is used.
 * The q15 filters use this scratch with the FILTERS_UKF_Q_* data amounts.
 */
typedef filters_kalman_matrix_f32_workspace_t filters_ukf_f32_workspace_t;

/**
 * @brief Double-precision scratch; same arrays as the linear matrix filter.
 *
 * The q31 filters use this scratch with the FILTERS_UKF_Q_* data amounts.
 */
typedef filters_kalman_matrix_f64_workspace_t filters_ukf_f64_workspace_t;

/**
 * @brief Caller-owned single-precision UKF state and explicitly bounded storage.
 *
 * Before init, point the four descriptors at naturally aligned, mutually
 * disjoint float arrays: x n-by-1, P n-by-n, Q n-by-n, R m-by-m with n,m>0,
 * and set the capacities in elements. Init fills values and weights; it never
 * changes shapes, addresses or capacities. Do not edit descriptors, capacities
 * or weights afterwards. Q and R contents are re-read and re-validated on every
 * predict/update, so the caller may rewrite them between calls for
 * time-varying noise. Failure preserves x and P.
 */
typedef struct {
    linears_matrix_t estimate; /**< n-by-1 state in physical units. */
    linears_matrix_t covariance; /**< n-by-n symmetric positive-definite P. */
    linears_matrix_t process_noise; /**< n-by-n symmetric PSD Q, state covariance units. */
    linears_matrix_t measurement_noise; /**< m-by-m symmetric SPD R. */
    size_t estimate_capacity; /**< Allocated estimate elements; at least n. */
    size_t covariance_capacity; /**< Allocated covariance elements; at least n*n. */
    size_t process_noise_capacity; /**< Allocated Q elements; at least n*n. */
    size_t measurement_noise_capacity; /**< Allocated R elements; at least m*m. */
    filters_ukf_f32_weights_t weights; /**< Written by init from alpha/beta/kappa. */
} filters_ukf_f32_t;

/**
 * @copydoc filters_ukf_f32_t
 */
typedef struct {
    linears_matrix_t estimate; /**< n-by-1 state in physical units. */
    linears_matrix_t covariance; /**< n-by-n symmetric positive-definite P. */
    linears_matrix_t process_noise; /**< n-by-n symmetric PSD Q, state covariance units. */
    linears_matrix_t measurement_noise; /**< m-by-m symmetric SPD R. */
    size_t estimate_capacity; /**< Allocated estimate elements; at least n. */
    size_t covariance_capacity; /**< Allocated covariance elements; at least n*n. */
    size_t process_noise_capacity; /**< Allocated Q elements; at least n*n. */
    size_t measurement_noise_capacity; /**< Allocated R elements; at least m*m. */
    filters_ukf_f64_weights_t weights; /**< Written by init from alpha/beta/kappa. */
} filters_ukf_f64_t;

/**
 * @brief Caller-owned q15 UKF state; arrays hold raw int16_t/2^15 signals.
 *
 * Storage rules follow filters_ukf_f32_t with int16_t elements. Weights,
 * callbacks and scratch are the f32 ones: the filter widens its signals to
 * float on every call and narrows the results back.
 */
typedef struct {
    linears_matrix_t estimate; /**< n-by-1 raw q15 state. */
    linears_matrix_t covariance; /**< n-by-n raw q15 symmetric positive-definite P. */
    linears_matrix_t process_noise; /**< n-by-n raw q15 symmetric PSD Q. */
    linears_matrix_t measurement_noise; /**< m-by-m raw q15 symmetric SPD R. */
    size_t estimate_capacity; /**< Allocated estimate elements; at least n. */
    size_t covariance_capacity; /**< Allocated covariance elements; at least n*n. */
    size_t process_noise_capacity; /**< Allocated Q elements; at least n*n. */
    size_t measurement_noise_capacity; /**< Allocated R elements; at least m*m. */
    filters_ukf_f32_weights_t weights; /**< Written by init from float alpha/beta/kappa. */
} filters_ukf_q15_t;

/**
 * @brief Caller-owned q31 UKF state; arrays hold raw int32_t/2^31 signals.
 *
 * Storage rules follow filters_ukf_f32_t with int32_t elements. Weights,
 * callbacks and scratch are the f64 ones: the filter widens its signals to
 * double on every call and narrows the results back.
 */
typedef struct {
    linears_matrix_t estimate; /**< n-by-1 raw q31 state. */
    linears_matrix_t covariance; /**< n-by-n raw q31 symmetric positive-definite P. */
    linears_matrix_t process_noise; /**< n-by-n raw q31 symmetric PSD Q. */
    linears_matrix_t measurement_noise; /**< m-by-m raw q31 symmetric SPD R. */
    size_t estimate_capacity; /**< Allocated estimate elements; at least n. */
    size_t covariance_capacity; /**< Allocated covariance elements; at least n*n. */
    size_t process_noise_capacity; /**< Allocated Q elements; at least n*n. */
    size_t measurement_noise_capacity; /**< Allocated R elements; at least m*m. */
    filters_ukf_f64_weights_t weights; /**< Written by init from double alpha/beta/kappa. */
} filters_ukf_q31_t;

/**
 * @brief Working elements for sigma point generation: 2*n*n; SIZE_MAX on overflow.
 *
 * n is a positive, side-effect-free state count. Allocate the kernel's working
 * element type (float or double).
 */
#define FILTERS_UKF_SIGMA_POINTS_STORAGE(n) LINEARS_SIZE_MUL(2, LINEARS_SIZE_MUL((n), (n)))

/**
 * @brief Basic scratch bytes for sigma point generation; SIZE_MAX on overflow.
 */
#define FILTERS_UKF_SIGMA_POINTS_BASIC_BYTES(n) LINEARS_BASIC_WORKSPACE_BYTES((n), (n))

/**
 * @brief Decomposition elements for sigma point generation; SIZE_MAX on overflow.
 */
#define FILTERS_UKF_SIGMA_POINTS_DECOMP_STORAGE(n) LINEARS_CHOLESKY_WORKSPACE(n)

/**
 * @brief Working elements for a weighted mean of points k-vectors: 2*points*k.
 *
 * points and k are positive and side-effect-free; SIZE_MAX signals overflow.
 * The full amount is required whether or not a residual is supplied.
 */
#define FILTERS_UKF_MEAN_STORAGE(points, k) LINEARS_SIZE_MUL(2, LINEARS_SIZE_MUL((points), (k)))

/**
 * @brief Basic scratch bytes for a weighted mean; SIZE_MAX on overflow.
 */
#define FILTERS_UKF_MEAN_BASIC_BYTES(points, k) LINEARS_BASIC_WORKSPACE_BYTES((points), (k))

/**
 * @brief Basic scratch bytes for deviations of k-vectors; no working elements needed.
 */
#define FILTERS_UKF_DEVIATIONS_BASIC_BYTES(k) LINEARS_BASIC_WORKSPACE_BYTES((k), 1)

/**
 * @brief Working elements for a weighted covariance of p- and q-vectors: points*(p+q).
 *
 * points, p and q are positive and side-effect-free; SIZE_MAX signals overflow.
 */
#define FILTERS_UKF_WEIGHTED_COVARIANCE_STORAGE(points, p, q) \
    LINEARS_SIZE_MUL((points), LINEARS_SIZE_ADD((p), (q)))

/**
 * @brief Basic scratch bytes for a weighted covariance; SIZE_MAX on overflow.
 */
#define FILTERS_UKF_WEIGHTED_COVARIANCE_BASIC_BYTES(points, p, q) \
    LINEARS_BASIC_WORKSPACE_BYTES((p), FILTERS_KALMAN_MAX_SIZE((points), (q)))

/**
 * @brief Working elements for the predict kernel: 7*n*n + 7*n + 2; SIZE_MAX on overflow.
 *
 * n is a positive, side-effect-free state count. The filter macros below add
 * their own output staging and Q imports on top of this amount.
 */
#define FILTERS_UKF_PREDICT_WORK_STORAGE(n) \
    LINEARS_SIZE_ADD(LINEARS_SIZE_MUL(7, LINEARS_SIZE_MUL((n), (n))), \
                     LINEARS_SIZE_ADD(LINEARS_SIZE_MUL(7, (n)), 2))

/**
 * @brief Working elements for the update kernel; SIZE_MAX on overflow.
 *
 * 6*n*n + 2*m*m + 10*n*m + 6*n + 5*m + 2 + (2*n+1)*max(n,m) elements for n
 * states and m measurements, both positive and side-effect-free.
 */
#define FILTERS_UKF_UPDATE_WORK_STORAGE(n, m) \
    LINEARS_SIZE_ADD( \
        LINEARS_SIZE_ADD( \
            LINEARS_SIZE_ADD(LINEARS_SIZE_MUL(LINEARS_SIZE_MUL(2, (n)), FILTERS_KALMAN_MAX_SIZE((n), (m))), \
                             FILTERS_KALMAN_MAX_SIZE((n), (m))), \
            LINEARS_SIZE_ADD(LINEARS_SIZE_MUL(6, LINEARS_SIZE_MUL((n), (n))), \
                             LINEARS_SIZE_MUL(2, LINEARS_SIZE_MUL((m), (m))))), \
        LINEARS_SIZE_ADD(LINEARS_SIZE_MUL(10, LINEARS_SIZE_MUL((n), (m))), \
                         LINEARS_SIZE_ADD(LINEARS_SIZE_MUL(6, (n)), \
                                          LINEARS_SIZE_ADD(LINEARS_SIZE_MUL(5, (m)), 2))))

/**
 * @brief Floating working elements for predict: 8*n*n + 8*n + 2; SIZE_MAX on overflow.
 *
 * n is a positive, side-effect-free state count. Allocate a real float or
 * double array. q15/q31 use FILTERS_UKF_Q_PREDICT_STORAGE(n) instead.
 */
#define FILTERS_UKF_PREDICT_STORAGE(n) \
    LINEARS_SIZE_ADD(LINEARS_SIZE_MUL(8, LINEARS_SIZE_MUL((n), (n))), \
                     LINEARS_SIZE_ADD(LINEARS_SIZE_MUL(8, (n)), 2))

/**
 * @brief Working elements for Q predict: 10*n*n + 9*n + 2.
 *
 * Includes the widened x/P/Q. n is positive and side-effect-free; SIZE_MAX
 * signals overflow. Allocate float for q15 and double for q31.
 */
#define FILTERS_UKF_Q_PREDICT_STORAGE(n) \
    LINEARS_SIZE_ADD(LINEARS_SIZE_MUL(10, LINEARS_SIZE_MUL((n), (n))), \
                     LINEARS_SIZE_ADD(LINEARS_SIZE_MUL(9, (n)), 2))

/**
 * @brief Predict basic-scratch bytes in every format; SIZE_MAX means overflow.
 *
 * Allocate linears_workspace_unit_t[LINEARS_WORKSPACE_UNITS(result)]. The
 * predict kernel has the same basic and decomposition requirements.
 */
#define FILTERS_UKF_PREDICT_BASIC_BYTES(n) \
    LINEARS_BASIC_WORKSPACE_BYTES((n), FILTERS_UKF_SIGMA_COUNT(n))

/**
 * @brief Decomposition elements for predict in every format: the Cholesky of P.
 *
 * Allocate float (f32, q15) or double (f64, q31) to match the filter.
 */
#define FILTERS_UKF_PREDICT_DECOMP_STORAGE(n) LINEARS_CHOLESKY_WORKSPACE(n)

/**
 * @brief Floating working elements for update; SIZE_MAX on overflow.
 *
 * 7*n*n + 2*m*m + 10*n*m + 7*n + 5*m + 2 + (2*n+1)*max(n,m) elements for n
 * states and m measurements, both positive and side-effect-free.
 */
#define FILTERS_UKF_UPDATE_STORAGE(n, m) \
    LINEARS_SIZE_ADD( \
        LINEARS_SIZE_ADD( \
            LINEARS_SIZE_ADD(LINEARS_SIZE_MUL(LINEARS_SIZE_MUL(2, (n)), FILTERS_KALMAN_MAX_SIZE((n), (m))), \
                             FILTERS_KALMAN_MAX_SIZE((n), (m))), \
            LINEARS_SIZE_ADD(LINEARS_SIZE_MUL(7, LINEARS_SIZE_MUL((n), (n))), \
                             LINEARS_SIZE_MUL(2, LINEARS_SIZE_MUL((m), (m))))), \
        LINEARS_SIZE_ADD(LINEARS_SIZE_MUL(10, LINEARS_SIZE_MUL((n), (m))), \
                         LINEARS_SIZE_ADD(LINEARS_SIZE_MUL(7, (n)), \
                                          LINEARS_SIZE_ADD(LINEARS_SIZE_MUL(5, (m)), 2))))

/**
 * @brief Working elements for Q update, including the widened x/P/R/z.
 *
 * Adds n*n + m*m + n + m elements to FILTERS_UKF_UPDATE_STORAGE(n,m).
 */
#define FILTERS_UKF_Q_UPDATE_STORAGE(n, m) \
    LINEARS_SIZE_ADD(FILTERS_UKF_UPDATE_STORAGE((n), (m)), \
        LINEARS_SIZE_ADD(LINEARS_SIZE_ADD(LINEARS_SIZE_MUL((n), (n)), LINEARS_SIZE_MUL((m), (m))), \
                         LINEARS_SIZE_ADD((n), (m))))

/**
 * @brief Update basic-scratch bytes in every format; SIZE_MAX means overflow.
 *
 * The update kernel has the same basic and decomposition requirements.
 */
#define FILTERS_UKF_UPDATE_BASIC_BYTES(n, m) \
    LINEARS_BASIC_WORKSPACE_BYTES(FILTERS_KALMAN_MAX_SIZE((n), (m)), FILTERS_UKF_SIGMA_COUNT(n))

/**
 * @brief Decomposition elements for update: max(n*n, m*m + m*n); SIZE_MAX on overflow.
 *
 * Covers the Cholesky of P, the Cholesky checks of R and S, and the gain solve.
 */
#define FILTERS_UKF_UPDATE_DECOMP_STORAGE(n, m) \
    FILTERS_KALMAN_MAX_SIZE(LINEARS_CHOLESKY_WORKSPACE(n), LINEARS_SOLVE_WORKSPACE((m), (n)))

/**
 * @brief Compute single-precision scaled unscented transform weights.
 *
 * @param[in] states Positive state count n.
 * @param[in] alpha Finite spread parameter, strictly positive; typically (0,1].
 * @param[in] beta Finite prior-knowledge parameter; 2 is optimal for Gaussians.
 * @param[in] kappa Finite secondary scaling; n+kappa must be positive.
 * @param[out] out Non-NULL destination, unchanged on failure.
 * @return linears_status_t Computation status.
 * @retval LINEARS_OK Weights were written; W_m sums to one.
 * @retval LINEARS_ERROR_ARGUMENT out is NULL.
 * @retval LINEARS_ERROR_DIMENSION states is zero.
 * @retval LINEARS_ERROR_RANGE A parameter or result is nonfinite.
 * @retval LINEARS_ERROR_DOMAIN alpha or alpha^2*(n+kappa) is not positive.
 */
linears_status_t filters_ukf_f32_weights(size_t states, float alpha, float beta, float kappa,
                                         filters_ukf_f32_weights_t *out);

/**
 * @brief Generate 2n+1 single-precision sigma points, one per row.
 *
 * Row 0 is mean; rows 1..n are mean + column i of L and rows n+1..2n are
 * mean - column i of L, where L*L^T = scale*covariance.
 *
 * @param[in] mean Non-NULL finite n-by-1 center point.
 * @param[in] covariance Non-NULL finite, exactly symmetric, positive-definite n-by-n matrix.
 * @param[in] scale Finite positive factor, normally weights.scale = n+lambda.
 * @param[in,out] points Non-NULL (2n+1)-by-n destination; data disjoint from inputs.
 * @param[in] workspace Arrays meeting FILTERS_UKF_SIGMA_POINTS_STORAGE(n),
 *                      _BASIC_BYTES(n) and _DECOMP_STORAGE(n); disjoint from
 *                      every matrix and descriptor.
 * @return linears_status_t OK, ARGUMENT, DIMENSION (shape mismatch or an
 *         asymmetric covariance, reported by the shared Cholesky), SIZE,
 *         WORKSPACE, ALIAS, RANGE, DOMAIN for nonpositive scale, or
 *         NOT_POSITIVE_DEFINITE when the Cholesky factor does not exist at
 *         working precision.
 * @note points contents are unspecified after a failure. Serialize shared scratch.
 */
linears_status_t filters_ukf_f32_sigma_points(const linears_matrix_t *mean,
                                              const linears_matrix_t *covariance, float scale,
                                              linears_matrix_t *points,
                                              filters_ukf_f32_workspace_t workspace);

/**
 * @brief Map every row of points through a single-precision callback.
 *
 * @param[in] points Non-NULL s-by-k matrix of s input points, s,k>0.
 * @param[in] fn Non-NULL map from k-by-1 to j-by-1 views.
 * @param[in] context Passed to fn unchanged; may be NULL.
 * @param[in,out] out Non-NULL s-by-j destination, j>0; disjoint from points.
 * @return linears_status_t OK, ARGUMENT, DIMENSION, SIZE, ALIAS, RANGE when a
 *         callback produced a nonfinite value, or the callback's own error.
 * @note Rows are processed in order; out is partially written after a failure.
 */
linears_status_t filters_ukf_f32_propagate(const linears_matrix_t *points, filters_ukf_f32_fn fn,
                                           void *context, linears_matrix_t *out);

/**
 * @brief Weighted mean of the rows of points in single precision.
 *
 * Without a residual, out = points^T * weights. With a residual,
 * out = p0 + sum_i weights[i] * residual(p_i, p0), where p0 is row 0; this
 * equals the plain weighted sum for linear components when the weights sum
 * to one, and averages angle-like components relative to p0.
 *
 * @param[in] points Non-NULL s-by-k matrix, s,k>0.
 * @param[in] weights Non-NULL finite s-by-1 weights; must sum to one when a
 *                    residual is supplied.
 * @param[in] residual Residual callback, or NULL for plain subtraction.
 * @param[in] context Passed to residual unchanged; may be NULL.
 * @param[in,out] out Non-NULL k-by-1 destination; disjoint from inputs.
 * @param[in] workspace Arrays meeting FILTERS_UKF_MEAN_STORAGE(s,k) and
 *                      FILTERS_UKF_MEAN_BASIC_BYTES(s,k); decomposition is ignored.
 * @return linears_status_t OK, ARGUMENT, DIMENSION, SIZE, WORKSPACE, ALIAS,
 *         RANGE, or a residual callback error.
 */
linears_status_t filters_ukf_f32_mean(const linears_matrix_t *points, const linears_matrix_t *weights,
                                      filters_ukf_f32_residual_fn residual, void *context,
                                      linears_matrix_t *out, filters_ukf_f32_workspace_t workspace);

/**
 * @brief Row-wise residuals of points about a mean in single precision.
 *
 * @param[in] points Non-NULL s-by-k matrix, s,k>0.
 * @param[in] mean Non-NULL k-by-1 reference; may share data with points.
 * @param[in] residual Residual callback, or NULL for plain subtraction.
 * @param[in] context Passed to residual unchanged; may be NULL.
 * @param[in,out] out Non-NULL s-by-k destination; disjoint from inputs.
 * @param[in] workspace basic must meet FILTERS_UKF_DEVIATIONS_BASIC_BYTES(k);
 *                      data and decomposition are ignored.
 * @return linears_status_t OK, ARGUMENT, DIMENSION, SIZE, WORKSPACE, ALIAS,
 *         RANGE, or a residual callback error.
 */
linears_status_t filters_ukf_f32_deviations(const linears_matrix_t *points, const linears_matrix_t *mean,
                                            filters_ukf_f32_residual_fn residual, void *context,
                                            linears_matrix_t *out, filters_ukf_f32_workspace_t workspace);

/**
 * @brief Weighted sum of outer products out = a^T * diag(weights) * b.
 *
 * With a == b and rows holding deviations this is a weighted covariance; with
 * distinct deviation sets it is the cross covariance. Add noise separately.
 *
 * @param[in] a Non-NULL s-by-p matrix, s,p>0.
 * @param[in] b Non-NULL s-by-q matrix, q>0; may be a itself.
 * @param[in] weights Non-NULL finite s-by-1 weights; negative values are allowed.
 * @param[in,out] out Non-NULL p-by-q destination; disjoint from inputs.
 * @param[in] workspace Arrays meeting FILTERS_UKF_WEIGHTED_COVARIANCE_STORAGE(s,p,q)
 *                      and _BASIC_BYTES(s,p,q); decomposition is ignored.
 * @return linears_status_t OK, ARGUMENT, DIMENSION, SIZE, WORKSPACE, ALIAS or RANGE.
 */
linears_status_t filters_ukf_f32_weighted_covariance(const linears_matrix_t *a, const linears_matrix_t *b,
                                                     const linears_matrix_t *weights, linears_matrix_t *out,
                                                     filters_ukf_f32_workspace_t workspace);

/**
 * @brief Unscented prediction on single-precision working matrices.
 *
 * This is the predict step without state storage: it maps the sigma points
 * of (mean, covariance) through transition and writes the predicted mean and
 * the symmetrized predicted covariance plus process_noise.
 *
 * @param[in] mean Non-NULL finite n-by-1 prior mean.
 * @param[in] covariance Non-NULL finite symmetric positive-definite n-by-n prior covariance.
 * @param[in] process_noise Non-NULL finite symmetric PSD n-by-n Q.
 * @param[in] weights Non-NULL weights from filters_ukf_f32_weights.
 * @param[in] transition Non-NULL map f from an n-by-1 point to its n-by-1 successor.
 * @param[in] transition_context Passed to transition unchanged; may be NULL.
 * @param[in] residual State-space residual, or NULL for plain subtraction.
 * @param[in] residual_context Passed to residual unchanged; may be NULL.
 * @param[in,out] out_mean Non-NULL n-by-1 destination; disjoint from every input.
 * @param[in,out] out_covariance Non-NULL n-by-n destination; disjoint from every input.
 * @param[in] workspace Arrays meeting FILTERS_UKF_PREDICT_WORK_STORAGE(n),
 *                      FILTERS_UKF_PREDICT_BASIC_BYTES(n) and
 *                      FILTERS_UKF_PREDICT_DECOMP_STORAGE(n).
 * @return linears_status_t OK, ARGUMENT, DIMENSION, SIZE, WORKSPACE, ALIAS,
 *         RANGE, DOMAIN (asymmetry, negative diagonal, invalid weights),
 *         NOT_POSITIVE_DEFINITE, or a callback error.
 * @note Output contents are unspecified after a failure.
 */
linears_status_t filters_ukf_f32_predict_work(const linears_matrix_t *mean, const linears_matrix_t *covariance,
                                              const linears_matrix_t *process_noise,
                                              const filters_ukf_f32_weights_t *weights,
                                              filters_ukf_f32_fn transition, void *transition_context,
                                              filters_ukf_f32_residual_fn residual, void *residual_context,
                                              linears_matrix_t *out_mean, linears_matrix_t *out_covariance,
                                              filters_ukf_f32_workspace_t workspace);

/**
 * @brief Unscented measurement update on single-precision working matrices.
 *
 * This is the update step without state storage; see filters_ukf_f32_update
 * for the equations. R is Cholesky-checked before any callback runs.
 *
 * @param[in] mean Non-NULL finite n-by-1 prior mean.
 * @param[in] covariance Non-NULL finite symmetric positive-definite n-by-n prior covariance.
 * @param[in] measurement_noise Non-NULL finite symmetric SPD m-by-m R.
 * @param[in] measurement Non-NULL finite m-by-1 z.
 * @param[in] weights Non-NULL weights from filters_ukf_f32_weights.
 * @param[in] observation Non-NULL map h from an n-by-1 point to an m-by-1 measurement.
 * @param[in] observation_context Passed to observation unchanged; may be NULL.
 * @param[in] residual Measurement-space residual, or NULL for plain subtraction.
 * @param[in] residual_context Passed to residual unchanged; may be NULL.
 * @param[in,out] out_mean Non-NULL n-by-1 destination; disjoint from every input.
 * @param[in,out] out_covariance Non-NULL n-by-n destination; disjoint from every input.
 * @param[in] workspace Arrays meeting FILTERS_UKF_UPDATE_WORK_STORAGE(n,m),
 *                      FILTERS_UKF_UPDATE_BASIC_BYTES(n,m) and
 *                      FILTERS_UKF_UPDATE_DECOMP_STORAGE(n,m).
 * @return linears_status_t OK, ARGUMENT, DIMENSION, SIZE, WORKSPACE, ALIAS,
 *         RANGE, DOMAIN, NOT_POSITIVE_DEFINITE (P, R or S), SINGULAR, or a
 *         callback error.
 * @note Output contents are unspecified after a failure.
 */
linears_status_t filters_ukf_f32_update_work(const linears_matrix_t *mean, const linears_matrix_t *covariance,
                                             const linears_matrix_t *measurement_noise,
                                             const linears_matrix_t *measurement,
                                             const filters_ukf_f32_weights_t *weights,
                                             filters_ukf_f32_fn observation, void *observation_context,
                                             filters_ukf_f32_residual_fn residual, void *residual_context,
                                             linears_matrix_t *out_mean, linears_matrix_t *out_covariance,
                                             filters_ukf_f32_workspace_t workspace);

/**
 * @brief Initialize a single-precision UKF: hyperparameters and x/P/Q/R.
 *
 * @param[in,out] state Non-NULL object whose descriptors and capacities are set.
 * @param[in] alpha Sigma point spread, strictly positive; typically (0,1].
 * @param[in] beta Distribution prior; 2 for Gaussian states.
 * @param[in] kappa Secondary scaling; n+kappa must be positive (0 or 3-n are common).
 * @param[in] estimate Non-NULL finite n-by-1 initial x.
 * @param[in] covariance Non-NULL finite symmetric n-by-n initial P with
 *                       nonnegative diagonal; must be positive definite for
 *                       later predict/update to succeed.
 * @param[in] process_noise Non-NULL finite symmetric PSD n-by-n Q.
 * @param[in] measurement_noise Non-NULL finite symmetric m-by-m R; SPD is
 *                              checked by Cholesky in update.
 * @return linears_status_t OK, ARGUMENT for NULL/misalignment, DIMENSION for
 *         shape mismatch or zero sizes, SIZE for size overflow, WORKSPACE for
 *         a short capacity, ALIAS for forbidden overlap, RANGE for nonfinite
 *         values, or DOMAIN for asymmetry, a negative P/Q diagonal or invalid
 *         hyperparameters.
 * @note Each source may exactly alias its own destination storage or be the
 *       state's own descriptor; partial or cross alias fails. No scratch is
 *       used. Failure leaves the state unchanged.
 */
linears_status_t filters_ukf_f32_init(filters_ukf_f32_t *state, float alpha, float beta, float kappa,
                                      const linears_matrix_t *estimate, const linears_matrix_t *covariance,
                                      const linears_matrix_t *process_noise,
                                      const linears_matrix_t *measurement_noise);

/**
 * @brief Unscented prediction x,P <- f(sigma points of x,P), + Q, in single precision.
 *
 * @param[in,out] state Non-NULL initialized filter.
 * @param[in] transition Non-NULL map f from an n-by-1 point to its n-by-1 successor.
 * @param[in] transition_context Passed to transition unchanged; may be NULL.
 * @param[in] residual State-space residual for the mean and covariance of the
 *                     propagated points, or NULL for plain subtraction.
 * @param[in] residual_context Passed to residual unchanged; may be NULL.
 * @param[in] workspace Arrays meeting FILTERS_UKF_PREDICT_STORAGE(n),
 *                      FILTERS_UKF_PREDICT_BASIC_BYTES(n) and
 *                      FILTERS_UKF_PREDICT_DECOMP_STORAGE(n), disjoint from the
 *                      state object and its arrays.
 * @return linears_status_t OK, ARGUMENT, DIMENSION, SIZE, WORKSPACE, ALIAS,
 *         RANGE (nonfinite input, callback output or result), DOMAIN
 *         (asymmetric P/Q, negative diagonal, invalid weights),
 *         NOT_POSITIVE_DEFINITE when P has no Cholesky factor, or a callback error.
 * @note Success commits the symmetrized covariance. Failure preserves x and P,
 *       not scratch. Callbacks run before any state write.
 */
linears_status_t filters_ukf_f32_predict(filters_ukf_f32_t *state, filters_ukf_f32_fn transition,
                                         void *transition_context, filters_ukf_f32_residual_fn residual,
                                         void *residual_context, filters_ukf_f32_workspace_t workspace);

/**
 * @brief Unscented measurement update in single precision.
 *
 * Regenerates sigma points from x and P, maps them through h, forms the
 * predicted measurement, S = Pzz + R and the cross covariance C, solves
 * S*K^T = C^T, then commits x + K*(z (-) zhat) and P - K*C^T symmetrized.
 *
 * @param[in,out] state Non-NULL initialized filter with m-by-m R.
 * @param[in] measurement Non-NULL finite m-by-1 z in measurement units.
 * @param[in] observation Non-NULL map h from an n-by-1 point to an m-by-1 measurement.
 * @param[in] observation_context Passed to observation unchanged; may be NULL.
 * @param[in] residual Measurement-space residual used for the predicted
 *                     measurement mean, its deviations and the innovation, or
 *                     NULL for plain subtraction.
 * @param[in] residual_context Passed to residual unchanged; may be NULL.
 * @param[in] workspace Arrays meeting FILTERS_UKF_UPDATE_STORAGE(n,m),
 *                      FILTERS_UKF_UPDATE_BASIC_BYTES(n,m) and
 *                      FILTERS_UKF_UPDATE_DECOMP_STORAGE(n,m), disjoint from the
 *                      state, its arrays and the measurement.
 * @return linears_status_t OK, ARGUMENT, DIMENSION, SIZE, WORKSPACE, ALIAS,
 *         RANGE, DOMAIN (asymmetric P/R, negative P diagonal, invalid weights),
 *         NOT_POSITIVE_DEFINITE when P, R or S cannot be factored, SINGULAR
 *         for a zero LU pivot, or a callback error.
 * @note R is Cholesky-checked before any callback runs. Zero innovation is
 *       valid. Failure preserves x and P, not scratch.
 */
linears_status_t filters_ukf_f32_update(filters_ukf_f32_t *state, const linears_matrix_t *measurement,
                                        filters_ukf_f32_fn observation, void *observation_context,
                                        filters_ukf_f32_residual_fn residual, void *residual_context,
                                        filters_ukf_f32_workspace_t workspace);

/**
 * @copydoc filters_ukf_f32_weights
 */
linears_status_t filters_ukf_f64_weights(size_t states, double alpha, double beta, double kappa,
                                         filters_ukf_f64_weights_t *out);

/**
 * @copydoc filters_ukf_f32_sigma_points
 */
linears_status_t filters_ukf_f64_sigma_points(const linears_matrix_t *mean,
                                              const linears_matrix_t *covariance, double scale,
                                              linears_matrix_t *points,
                                              filters_ukf_f64_workspace_t workspace);

/**
 * @copydoc filters_ukf_f32_propagate
 */
linears_status_t filters_ukf_f64_propagate(const linears_matrix_t *points, filters_ukf_f64_fn fn,
                                           void *context, linears_matrix_t *out);

/**
 * @copydoc filters_ukf_f32_mean
 */
linears_status_t filters_ukf_f64_mean(const linears_matrix_t *points, const linears_matrix_t *weights,
                                      filters_ukf_f64_residual_fn residual, void *context,
                                      linears_matrix_t *out, filters_ukf_f64_workspace_t workspace);

/**
 * @copydoc filters_ukf_f32_deviations
 */
linears_status_t filters_ukf_f64_deviations(const linears_matrix_t *points, const linears_matrix_t *mean,
                                            filters_ukf_f64_residual_fn residual, void *context,
                                            linears_matrix_t *out, filters_ukf_f64_workspace_t workspace);

/**
 * @copydoc filters_ukf_f32_weighted_covariance
 */
linears_status_t filters_ukf_f64_weighted_covariance(const linears_matrix_t *a, const linears_matrix_t *b,
                                                     const linears_matrix_t *weights, linears_matrix_t *out,
                                                     filters_ukf_f64_workspace_t workspace);

/**
 * @copydoc filters_ukf_f32_predict_work
 */
linears_status_t filters_ukf_f64_predict_work(const linears_matrix_t *mean, const linears_matrix_t *covariance,
                                              const linears_matrix_t *process_noise,
                                              const filters_ukf_f64_weights_t *weights,
                                              filters_ukf_f64_fn transition, void *transition_context,
                                              filters_ukf_f64_residual_fn residual, void *residual_context,
                                              linears_matrix_t *out_mean, linears_matrix_t *out_covariance,
                                              filters_ukf_f64_workspace_t workspace);

/**
 * @copydoc filters_ukf_f32_update_work
 */
linears_status_t filters_ukf_f64_update_work(const linears_matrix_t *mean, const linears_matrix_t *covariance,
                                             const linears_matrix_t *measurement_noise,
                                             const linears_matrix_t *measurement,
                                             const filters_ukf_f64_weights_t *weights,
                                             filters_ukf_f64_fn observation, void *observation_context,
                                             filters_ukf_f64_residual_fn residual, void *residual_context,
                                             linears_matrix_t *out_mean, linears_matrix_t *out_covariance,
                                             filters_ukf_f64_workspace_t workspace);

/**
 * @copydoc filters_ukf_f32_init
 */
linears_status_t filters_ukf_f64_init(filters_ukf_f64_t *state, double alpha, double beta, double kappa,
                                      const linears_matrix_t *estimate, const linears_matrix_t *covariance,
                                      const linears_matrix_t *process_noise,
                                      const linears_matrix_t *measurement_noise);

/**
 * @copydoc filters_ukf_f32_predict
 */
linears_status_t filters_ukf_f64_predict(filters_ukf_f64_t *state, filters_ukf_f64_fn transition,
                                         void *transition_context, filters_ukf_f64_residual_fn residual,
                                         void *residual_context, filters_ukf_f64_workspace_t workspace);

/**
 * @copydoc filters_ukf_f32_update
 */
linears_status_t filters_ukf_f64_update(filters_ukf_f64_t *state, const linears_matrix_t *measurement,
                                        filters_ukf_f64_fn observation, void *observation_context,
                                        filters_ukf_f64_residual_fn residual, void *residual_context,
                                        filters_ukf_f64_workspace_t workspace);

/**
 * @brief Initialize a q15 UKF from raw q15 matrices and float hyperparameters.
 *
 * Contract as filters_ukf_f32_init with int16_t/2^15 elements; raw values are
 * copied unchanged, so no value fails with RANGE here. Positive noise must
 * remain positive after quantization: a variance that rounds to raw zero is zero.
 *
 * @param[in,out] state Non-NULL object whose descriptors and capacities are set.
 * @param[in] alpha Sigma point spread, strictly positive; typically (0,1].
 * @param[in] beta Distribution prior; 2 for Gaussian states.
 * @param[in] kappa Secondary scaling; n+kappa must be positive.
 * @param[in] estimate Non-NULL n-by-1 raw q15 initial x.
 * @param[in] covariance Non-NULL symmetric n-by-n raw q15 initial P, nonnegative diagonal.
 * @param[in] process_noise Non-NULL symmetric PSD n-by-n raw q15 Q.
 * @param[in] measurement_noise Non-NULL symmetric m-by-m raw q15 R.
 * @return linears_status_t As filters_ukf_f32_init.
 */
linears_status_t filters_ukf_q15_init(filters_ukf_q15_t *state, float alpha, float beta, float kappa,
                                      const linears_matrix_t *estimate, const linears_matrix_t *covariance,
                                      const linears_matrix_t *process_noise,
                                      const linears_matrix_t *measurement_noise);

/**
 * @brief q15 unscented prediction computed in float; contract as filters_ukf_f32_predict.
 *
 * x, P and Q are widened to float, the f32 predict kernel runs with the
 * given float callbacks, and the results are narrowed back to q15.
 *
 * @param[in,out] state Non-NULL initialized q15 filter.
 * @param[in] transition Non-NULL float map f (n-by-1 to n-by-1).
 * @param[in] transition_context Passed to transition unchanged; may be NULL.
 * @param[in] residual Float state-space residual, or NULL for plain subtraction.
 * @param[in] residual_context Passed to residual unchanged; may be NULL.
 * @param[in] workspace Float arrays meeting FILTERS_UKF_Q_PREDICT_STORAGE(n),
 *                      FILTERS_UKF_PREDICT_BASIC_BYTES(n) and
 *                      FILTERS_UKF_PREDICT_DECOMP_STORAGE(n).
 * @return linears_status_t As filters_ukf_f32_predict. RANGE also covers a
 *         computed x or P element outside [-1,+1]; nothing is committed then.
 * @note Both outputs are range-checked before either is written; exact +1
 *       and the rounding cell reaching it narrow to INT16_MAX.
 */
linears_status_t filters_ukf_q15_predict(filters_ukf_q15_t *state, filters_ukf_f32_fn transition,
                                         void *transition_context, filters_ukf_f32_residual_fn residual,
                                         void *residual_context, filters_ukf_f32_workspace_t workspace);

/**
 * @brief q15 unscented measurement update computed in float; contract as filters_ukf_f32_update.
 *
 * @param[in,out] state Non-NULL initialized q15 filter with m-by-m R.
 * @param[in] measurement Non-NULL m-by-1 raw q15 z.
 * @param[in] observation Non-NULL float map h (n-by-1 to m-by-1).
 * @param[in] observation_context Passed to observation unchanged; may be NULL.
 * @param[in] residual Float measurement-space residual, or NULL for plain subtraction.
 * @param[in] residual_context Passed to residual unchanged; may be NULL.
 * @param[in] workspace Float arrays meeting FILTERS_UKF_Q_UPDATE_STORAGE(n,m),
 *                      FILTERS_UKF_UPDATE_BASIC_BYTES(n,m) and
 *                      FILTERS_UKF_UPDATE_DECOMP_STORAGE(n,m).
 * @return linears_status_t As filters_ukf_f32_update, plus RANGE for a
 *         computed x or P element outside [-1,+1]. NOT_POSITIVE_DEFINITE
 *         includes positive pivots lost to q15 quantization of P or R.
 */
linears_status_t filters_ukf_q15_update(filters_ukf_q15_t *state, const linears_matrix_t *measurement,
                                        filters_ukf_f32_fn observation, void *observation_context,
                                        filters_ukf_f32_residual_fn residual, void *residual_context,
                                        filters_ukf_f32_workspace_t workspace);

/**
 * @brief Initialize a q31 UKF from raw q31 matrices and double hyperparameters.
 *
 * Contract as filters_ukf_q15_init with int32_t/2^31 elements.
 *
 * @param[in,out] state Non-NULL object whose descriptors and capacities are set.
 * @param[in] alpha Sigma point spread, strictly positive; typically (0,1].
 * @param[in] beta Distribution prior; 2 for Gaussian states.
 * @param[in] kappa Secondary scaling; n+kappa must be positive.
 * @param[in] estimate Non-NULL n-by-1 raw q31 initial x.
 * @param[in] covariance Non-NULL symmetric n-by-n raw q31 initial P, nonnegative diagonal.
 * @param[in] process_noise Non-NULL symmetric PSD n-by-n raw q31 Q.
 * @param[in] measurement_noise Non-NULL symmetric m-by-m raw q31 R.
 * @return linears_status_t As filters_ukf_f32_init.
 */
linears_status_t filters_ukf_q31_init(filters_ukf_q31_t *state, double alpha, double beta, double kappa,
                                      const linears_matrix_t *estimate, const linears_matrix_t *covariance,
                                      const linears_matrix_t *process_noise,
                                      const linears_matrix_t *measurement_noise);

/**
 * @brief q31 unscented prediction computed in double; contract as filters_ukf_q15_predict.
 *
 * @param[in,out] state Non-NULL initialized q31 filter.
 * @param[in] transition Non-NULL double map f (n-by-1 to n-by-1).
 * @param[in] transition_context Passed to transition unchanged; may be NULL.
 * @param[in] residual Double state-space residual, or NULL for plain subtraction.
 * @param[in] residual_context Passed to residual unchanged; may be NULL.
 * @param[in] workspace Double arrays meeting FILTERS_UKF_Q_PREDICT_STORAGE(n),
 *                      FILTERS_UKF_PREDICT_BASIC_BYTES(n) and
 *                      FILTERS_UKF_PREDICT_DECOMP_STORAGE(n).
 * @return linears_status_t As filters_ukf_q15_predict; exact +1 narrows to INT32_MAX.
 */
linears_status_t filters_ukf_q31_predict(filters_ukf_q31_t *state, filters_ukf_f64_fn transition,
                                         void *transition_context, filters_ukf_f64_residual_fn residual,
                                         void *residual_context, filters_ukf_f64_workspace_t workspace);

/**
 * @brief q31 unscented measurement update computed in double; contract as filters_ukf_q15_update.
 *
 * @param[in,out] state Non-NULL initialized q31 filter with m-by-m R.
 * @param[in] measurement Non-NULL m-by-1 raw q31 z.
 * @param[in] observation Non-NULL double map h (n-by-1 to m-by-1).
 * @param[in] observation_context Passed to observation unchanged; may be NULL.
 * @param[in] residual Double measurement-space residual, or NULL for plain subtraction.
 * @param[in] residual_context Passed to residual unchanged; may be NULL.
 * @param[in] workspace Double arrays meeting FILTERS_UKF_Q_UPDATE_STORAGE(n,m),
 *                      FILTERS_UKF_UPDATE_BASIC_BYTES(n,m) and
 *                      FILTERS_UKF_UPDATE_DECOMP_STORAGE(n,m).
 * @return linears_status_t As filters_ukf_q15_update.
 */
linears_status_t filters_ukf_q31_update(filters_ukf_q31_t *state, const linears_matrix_t *measurement,
                                        filters_ukf_f64_fn observation, void *observation_context,
                                        filters_ukf_f64_residual_fn residual, void *residual_context,
                                        filters_ukf_f64_workspace_t workspace);

#ifdef __cplusplus
}
#endif
#endif
