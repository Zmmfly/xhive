/**
 * @file filters_kalman.h
 * @brief Caller-owned scalar and dense linear Kalman filters.
 *
 * Reset initializes an estimate and covariance. Predict and update are separate
 * transactions: failure preserves both state quantities and their descriptors,
 * but may change scratch. A successful predict is not rolled back by a later
 * failed update. No function allocates, performs I/O, uses variable-size stack
 * storage, or accesses mutable global state. Serialize calls sharing a context
 * or workspace, including task/interrupt calls; disjoint instances are reentrant.
 *
 * Scalar variance and process noise are nonnegative; measurement noise is
 * strictly positive. Matrix P and Q must be symmetric positive semidefinite;
 * this is a caller precondition (only symmetry and diagonal signs are checked).
 * R must be symmetric positive definite and is checked by Cholesky. Floating
 * values must be finite. q15/q31 are signed raw integers divided by 2^15/2^31.
 * Their binary points never change. Integer working arithmetic is wide F31;
 * signal output policy and nearest-away rounding follow linears_numeric.h.
 * Scalar final F62 expressions are range-checked before signal rounding.
 * Matrix filters range-check their computed/rounded F31 estimates/covariances,
 * not the ideal unrounded full expression. A conversion input outside [-1,+1]
 * fails; exact +1 maps to MAX, which remains 1 minus one signal LSB. Matrix
 * working-rounding error is separate from this endpoint error: an ideal value
 * slightly above +1 can round to working ONE and then map to MAX successfully.
 * Raw zero measurement noise fails.
 * Covariance uses the Joseph form, followed by matrix symmetrization, not a
 * positive-semidefinite projection. See KALMAN.md for limits and provenance.
 */
#ifndef FILTERS_KALMAN_H
#define FILTERS_KALMAN_H

#include "linears.h"
#include "linears_numeric.h"
#include "linears_decomp.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Caller-owned single-precision scalar estimate and variance.
 *
 * Initialize with reset; keep the object alive for all calls. Do not edit its
 * fields between calls. Independent objects are reentrant; shared use requires
 * caller serialization. Failed operations leave both fields unchanged.
 */
typedef struct {
    float estimate; /**< Estimated state, in caller-selected physical units. */
    float covariance; /**< Nonnegative variance, in squared state units. */
} filters_kalman_scalar_f32_t;

/**
 * @brief Caller-owned double-precision scalar estimate and variance.
 *
 * Initialize with reset; retain and serialize the object for its entire use.
 * Independent objects are reentrant. Failed operations preserve both fields.
 */
typedef struct {
    double estimate; /**< Estimated state, in caller-selected physical units. */
    double covariance; /**< Nonnegative variance, in squared state units. */
} filters_kalman_scalar_f64_t;

/**
 * @brief Caller-owned fixed-F15 scalar estimate and variance.
 *
 * Each int16_t field represents raw/32768, in [-1,1-2^-15]. Initialize with
 * reset; retain and serialize the object for all calls. Failures preserve both
 * fields. Working arithmetic is integer-only F31 with no scale metadata.
 */
typedef struct {
    linears_q15_t estimate; /**< State in caller-selected units, raw/32768. */
    linears_q15_t covariance; /**< Nonnegative variance in squared state units. */
} filters_kalman_scalar_q15_t;

/**
 * @brief Caller-owned fixed-F31 scalar estimate and variance.
 *
 * Each int32_t field represents raw/2147483648, in [-1,1-2^-31]. Initialize
 * with reset; retain and serialize the object for all calls. Failures preserve
 * both fields. Working arithmetic is integer-only F31 with no scale metadata.
 */
typedef struct {
    linears_q31_t estimate; /**< State in caller-selected units, raw/2147483648. */
    linears_q31_t covariance; /**< Nonnegative variance in squared state units. */
} filters_kalman_scalar_q31_t;

/**
 * @brief Initialize or reset a single-precision scalar filter.
 *
 * @param[out] state Non-NULL caller-owned object; prior initialization not needed.
 * @param[in] estimate Finite initial state in caller-selected physical units.
 * @param[in] covariance Finite nonnegative variance in squared state units.
 * @return linears_status_t Reset status.
 * @retval LINEARS_OK Both fields were committed.
 * @retval LINEARS_ERROR_ARGUMENT state is NULL.
 * @retval LINEARS_ERROR_RANGE A value is nonfinite.
 * @retval LINEARS_ERROR_DOMAIN covariance is negative.
 * @note No workspace is needed. Failure preserves state; serialize shared use.
 */
linears_status_t filters_kalman_scalar_f32_reset(filters_kalman_scalar_f32_t *state,
                                               float estimate, float covariance);

/**
 * @brief Predict x=a*x and p=a*p*a+q in single precision.
 *
 * @param[in,out] state Non-NULL successfully reset object, retained by caller.
 * @param[in] transition Finite dimensionless transition a for one time interval.
 * @param[in] process_noise Finite nonnegative q, in squared state units.
 * @return linears_status_t Prediction status.
 * @retval LINEARS_OK Estimate and variance were committed.
 * @retval LINEARS_ERROR_ARGUMENT state is NULL.
 * @retval LINEARS_ERROR_RANGE A value or intermediate is nonfinite.
 * @retval LINEARS_ERROR_DOMAIN State variance or process_noise is negative.
 * @note No workspace; failure preserves both fields. Serialize shared use.
 */
linears_status_t filters_kalman_scalar_f32_predict(filters_kalman_scalar_f32_t *state,
                                                 float transition, float process_noise);

/**
 * @brief Apply a single-precision measurement with Joseph variance update.
 *
 * @param[in,out] state Non-NULL successfully reset object, retained by caller.
 * @param[in] measurement Finite z in measurement units; zero residual is valid.
 * @param[in] observation Finite h, in measurement units per state unit.
 * @param[in] measurement_noise Finite strictly positive r, in squared measurement units.
 * @return linears_status_t Update status.
 * @retval LINEARS_OK Estimate and Joseph variance were committed.
 * @retval LINEARS_ERROR_ARGUMENT state is NULL.
 * @retval LINEARS_ERROR_RANGE A value or intermediate is nonfinite.
 * @retval LINEARS_ERROR_DOMAIN Negative state variance, nonpositive r, or
 *                             nonpositive innovation variance at working precision.
 * @note No workspace; failure preserves both fields. Serialize shared use.
 */
linears_status_t filters_kalman_scalar_f32_update(filters_kalman_scalar_f32_t *state,
                                                float measurement, float observation,
                                                float measurement_noise);

/**
 * @brief Initialize or reset a double-precision scalar filter.
 *
 * @param[out] state Non-NULL caller-owned object; prior initialization not needed.
 * @param[in] estimate Finite initial state in physical units.
 * @param[in] covariance Finite nonnegative variance in squared state units.
 * @return linears_status_t Reset status.
 * @retval LINEARS_OK Both fields were committed.
 * @retval LINEARS_ERROR_ARGUMENT state is NULL.
 * @retval LINEARS_ERROR_RANGE A value is nonfinite.
 * @retval LINEARS_ERROR_DOMAIN covariance is negative.
 * @note No workspace; failure preserves state. Serialize shared use.
 */
linears_status_t filters_kalman_scalar_f64_reset(filters_kalman_scalar_f64_t *state,
                                               double estimate, double covariance);

/**
 * @brief Predict x=a*x and p=a*p*a+q in double precision.
 *
 * @param[in,out] state Non-NULL successfully reset object, retained by caller.
 * @param[in] transition Finite dimensionless a for one time interval.
 * @param[in] process_noise Finite nonnegative q in squared state units.
 * @return linears_status_t Prediction status.
 * @retval LINEARS_OK Estimate and variance were committed.
 * @retval LINEARS_ERROR_ARGUMENT state is NULL.
 * @retval LINEARS_ERROR_RANGE A value or intermediate is nonfinite.
 * @retval LINEARS_ERROR_DOMAIN State variance or process_noise is negative.
 * @note No workspace; failure preserves both fields. Serialize shared use.
 */
linears_status_t filters_kalman_scalar_f64_predict(filters_kalman_scalar_f64_t *state,
                                                 double transition, double process_noise);

/**
 * @brief Apply a double-precision measurement with Joseph variance update.
 *
 * @param[in,out] state Non-NULL successfully reset object, retained by caller.
 * @param[in] measurement Finite z in measurement units; zero residual is valid.
 * @param[in] observation Finite h, in measurement units per state unit.
 * @param[in] measurement_noise Finite strictly positive r in squared measurement units.
 * @return linears_status_t Update status.
 * @retval LINEARS_OK Estimate and Joseph variance were committed.
 * @retval LINEARS_ERROR_ARGUMENT state is NULL.
 * @retval LINEARS_ERROR_RANGE A value or intermediate is nonfinite.
 * @retval LINEARS_ERROR_DOMAIN Negative state variance, nonpositive r, or
 *                             nonpositive innovation variance at working precision.
 * @note No workspace; failure preserves both fields. Serialize shared use.
 */
linears_status_t filters_kalman_scalar_f64_update(filters_kalman_scalar_f64_t *state,
                                                double measurement, double observation,
                                                double measurement_noise);

/**
 * @brief Initialize or reset a q15 scalar filter using integer arithmetic.
 *
 * @param[out] state Non-NULL caller-owned object; prior initialization not needed.
 * @param[in] estimate State in caller-selected units, fixed F15 raw/32768.
 * @param[in] covariance Nonnegative variance in squared state units, same encoding.
 * @return linears_status_t Reset status.
 * @retval LINEARS_OK Both raw fields were committed unchanged.
 * @retval LINEARS_ERROR_ARGUMENT state is NULL.
 * @retval LINEARS_ERROR_DOMAIN covariance is negative.
 * @note No workspace; failure preserves state. Serialize shared use.
 */
linears_status_t filters_kalman_scalar_q15_reset(filters_kalman_scalar_q15_t *state,
                                               linears_q15_t estimate, linears_q15_t covariance);

/**
 * @brief Predict x=a*x and p=a*p*a+q using the shared integer working kernel.
 *
 * @param[in,out] state Non-NULL successfully reset q15 object, retained by caller.
 * @param[in] transition Dimensionless wide F31 a (raw/2^31) for one interval;
 *                       LINEARS_FIXED_ONE is exact unity. Values above one are allowed.
 * @param[in] process_noise Nonnegative q in squared state units, q15 encoded.
 * @return linears_status_t Prediction status.
 * @retval LINEARS_OK Both fixed-fraction raw fields were committed.
 * @retval LINEARS_ERROR_ARGUMENT state is NULL.
 * @retval LINEARS_ERROR_RANGE Working overflow or a final output outside [-1,+1].
 * @retval LINEARS_ERROR_DOMAIN State variance or process_noise is negative.
 * @note No workspace; failure preserves both fields. Serialize shared use.
 */
linears_status_t filters_kalman_scalar_q15_predict(filters_kalman_scalar_q15_t *state,
                                                 linears_fixed_t transition,
                                                 linears_q15_t process_noise);

/**
 * @brief Apply a q15 measurement and integer-only Joseph variance update.
 *
 * @param[in,out] state Non-NULL successfully reset object, retained by caller.
 * @param[in] measurement q15 z in measurement units; zero residual is valid.
 * @param[in] observation Wide F31 h (raw/2^31), in measurement units per state
 *                        unit; exact unity and gains above one are representable.
 * @param[in] measurement_noise Strictly positive q15 r in squared measurement units.
 * @return linears_status_t Update status.
 * @retval LINEARS_OK Both fixed-fraction raw fields were committed.
 * @retval LINEARS_ERROR_ARGUMENT state is NULL.
 * @retval LINEARS_ERROR_RANGE Working overflow or a final output outside [-1,+1].
 * @retval LINEARS_ERROR_DOMAIN Negative state variance, nonpositive r, or a
 *                             nonpositive innovation variance at working precision.
 * @note No workspace; failure preserves both fields. Serialize shared use.
 */
linears_status_t filters_kalman_scalar_q15_update(filters_kalman_scalar_q15_t *state,
                                                linears_q15_t measurement,
                                                linears_fixed_t observation,
                                                linears_q15_t measurement_noise);

/**
 * @brief Initialize or reset a q31 scalar filter using integer arithmetic.
 *
 * @param[out] state Non-NULL caller-owned object; prior initialization not needed.
 * @param[in] estimate State in caller-selected units, fixed F31 raw/2147483648.
 * @param[in] covariance Nonnegative variance in squared state units, same encoding.
 * @return linears_status_t Reset status.
 * @retval LINEARS_OK Both raw fields were committed unchanged.
 * @retval LINEARS_ERROR_ARGUMENT state is NULL.
 * @retval LINEARS_ERROR_DOMAIN covariance is negative.
 * @note No workspace; failure preserves state. Serialize shared use.
 */
linears_status_t filters_kalman_scalar_q31_reset(filters_kalman_scalar_q31_t *state,
                                               linears_q31_t estimate, linears_q31_t covariance);

/**
 * @brief Predict x=a*x and p=a*p*a+q using the shared integer working kernel.
 *
 * @param[in,out] state Non-NULL successfully reset q31 object, retained by caller.
 * @param[in] transition Dimensionless wide F31 a (raw/2^31) for one interval;
 *                       LINEARS_FIXED_ONE is exact unity. Values above one are allowed.
 * @param[in] process_noise Nonnegative q in squared state units, q31 encoded.
 * @return linears_status_t Prediction status.
 * @retval LINEARS_OK Both fixed-fraction raw fields were committed.
 * @retval LINEARS_ERROR_ARGUMENT state is NULL.
 * @retval LINEARS_ERROR_RANGE Working overflow or a final output outside [-1,+1].
 * @retval LINEARS_ERROR_DOMAIN State variance or process_noise is negative.
 * @note No workspace; failure preserves both fields. Serialize shared use.
 */
linears_status_t filters_kalman_scalar_q31_predict(filters_kalman_scalar_q31_t *state,
                                                 linears_fixed_t transition,
                                                 linears_q31_t process_noise);

/**
 * @brief Apply a q31 measurement and integer-only Joseph variance update.
 *
 * @param[in,out] state Non-NULL successfully reset object, retained by caller.
 * @param[in] measurement q31 z in measurement units; zero residual is valid.
 * @param[in] observation Wide F31 h (raw/2^31), in measurement units per state
 *                        unit; exact unity and gains above one are representable.
 * @param[in] measurement_noise Strictly positive q31 r in squared measurement units.
 * @return linears_status_t Update status.
 * @retval LINEARS_OK Both fixed-fraction raw fields were committed.
 * @retval LINEARS_ERROR_ARGUMENT state is NULL.
 * @retval LINEARS_ERROR_RANGE Working overflow or a final output outside [-1,+1].
 * @retval LINEARS_ERROR_DOMAIN Negative state variance, nonpositive r, or a
 *                             nonpositive innovation variance at working precision.
 * @note No workspace; failure preserves both fields. Serialize shared use.
 */
linears_status_t filters_kalman_scalar_q31_update(filters_kalman_scalar_q31_t *state,
                                                linears_q31_t measurement,
                                                linears_fixed_t observation,
                                                linears_q31_t measurement_noise);

/**
 * @brief Caller-owned dense Kalman state and explicitly bounded output storage.
 *
 * Before reset, initialize estimate as n-by-1 and covariance as n-by-n, n>0,
 * with naturally aligned disjoint storage of the API's numeric element type.
 * Capacities are element counts, not bytes, and must cover n and n*n. Data and
 * this object must stay alive and disjoint. Do not change fields after reset.
 * Calls sharing this object or workspace require serialization; independent
 * instances are reentrant. Failure preserves active data and descriptors;
 * success changes active values only, never shapes, addresses or capacities.
 */
typedef struct {
    linears_matrix_t estimate; /**< n-by-1 state vector in physical state units. */
    linears_matrix_t covariance; /**< n-by-n covariance in products of state units. */
    size_t estimate_capacity; /**< Allocated estimate elements; at least n. */
    size_t covariance_capacity; /**< Allocated covariance elements; at least n*n. */
} filters_kalman_matrix_t;

/**
 * @brief Floating intermediate elements needed by prediction; SIZE_MAX on overflow.
 *
 * n is a positive, side-effect-free state count. Allocate a real float/double
 * array, not cast workspace units. Only the required prefix is accessed.
 * Q APIs instead require FILTERS_KALMAN_Q_PREDICT_STORAGE(n) F31 elements.
 */
#define FILTERS_KALMAN_PREDICT_STORAGE(n) \
    LINEARS_SIZE_ADD((n), LINEARS_SIZE_MUL(4, LINEARS_SIZE_MUL((n), (n))))

/**
 * @brief Floating intermediate elements needed by update; SIZE_MAX on overflow.
 *
 * n and m are positive, side-effect-free state and measurement counts.
 * Allocate float/double; Q APIs use FILTERS_KALMAN_Q_UPDATE_STORAGE(n,m).
 */
#define FILTERS_KALMAN_UPDATE_STORAGE(n, m) \
    LINEARS_SIZE_ADD( \
        LINEARS_SIZE_ADD(LINEARS_SIZE_MUL(4, LINEARS_SIZE_MUL((n), (n))), \
                        LINEARS_SIZE_MUL(4, LINEARS_SIZE_MUL((n), (m)))), \
        LINEARS_SIZE_ADD(LINEARS_SIZE_MUL(2, LINEARS_SIZE_MUL((m), (m))), \
                        LINEARS_SIZE_MUL(2, LINEARS_SIZE_ADD((n), (m)))))

/**
 * @brief Larger size; arguments must be nonnegative and side-effect-free.
 */
#define FILTERS_KALMAN_MAX_SIZE(a, b) ((size_t)(a) > (size_t)(b) ? (size_t)(a) : (size_t)(b))

/**
 * @brief Floating prediction basic-scratch bytes; SIZE_MAX means overflow.
 *
 * Allocate linears_workspace_unit_t[LINEARS_WORKSPACE_UNITS(result)]. n must
 * be positive and side-effect-free. This is separate from raw numeric storage.
 */
#define FILTERS_KALMAN_PREDICT_BASIC_BYTES(n) LINEARS_BASIC_WORKSPACE_BYTES((n), (n))

/**
 * @brief Floating update basic-scratch bytes; SIZE_MAX means overflow.
 *
 * n and m are positive, side-effect-free dimensions. Allocate a real array of
 * linears_workspace_unit_t, separate from raw and decomposition storage.
 */
#define FILTERS_KALMAN_UPDATE_BASIC_BYTES(n, m) \
    FILTERS_KALMAN_MAX_SIZE(LINEARS_BASIC_WORKSPACE_BYTES((n), (n)), \
        FILTERS_KALMAN_MAX_SIZE(LINEARS_BASIC_WORKSPACE_BYTES((n), (m)), \
                               LINEARS_BASIC_WORKSPACE_BYTES((m), (m))))

/**
 * @brief Decomposition working elements needed by update, or SIZE_MAX on overflow.
 *
 * Allocate float for f32, double for f64, or linears_fixed_t for q15/q31.
 * n and m are positive side-effect-free dimensions. Predict ignores this array.
 */
#define FILTERS_KALMAN_UPDATE_DECOMP_STORAGE(n, m) LINEARS_SOLVE_WORKSPACE((m), (n))

/**
 * @brief Fixed-F31 working elements for Q prediction, or SIZE_MAX on overflow.
 *
 * Includes imported x/P/F/Q and all intermediates: 2*n + 7*n*n int64_t elements.
 * n is positive and side-effect-free. Q signals are not intermediate storage.
 */
#define FILTERS_KALMAN_Q_PREDICT_STORAGE(n) \
    LINEARS_SIZE_ADD(LINEARS_SIZE_MUL(2, (n)), LINEARS_SIZE_MUL(7, LINEARS_SIZE_MUL((n), (n))))

/**
 * @brief Fixed-F31 working elements for Q update, or SIZE_MAX on overflow.
 *
 * Includes imported x/P/z/H/R: 5*n*n + 5*n*m + 3*m*m + 3*n + 3*m elements.
 * n and m are positive, side-effect-free state and measurement counts.
 */
#define FILTERS_KALMAN_Q_UPDATE_STORAGE(n, m) \
    LINEARS_SIZE_ADD( \
        LINEARS_SIZE_ADD(LINEARS_SIZE_MUL(5, LINEARS_SIZE_MUL((n), (n))), \
                        LINEARS_SIZE_MUL(5, LINEARS_SIZE_MUL((n), (m)))), \
        LINEARS_SIZE_ADD(LINEARS_SIZE_MUL(3, LINEARS_SIZE_MUL((m), (m))), \
                        LINEARS_SIZE_MUL(3, LINEARS_SIZE_ADD((n), (m)))))

/**
 * @brief Basic Q-prediction scratch bytes, including atomic output staging.
 *
 * n is positive and side-effect-free; SIZE_MAX signals overflow. The n+n*n
 * units also hold final quantized x/P through their q15/q31 union members.
 */
#define FILTERS_KALMAN_Q_PREDICT_BASIC_BYTES(n) \
    LINEARS_BASIC_WORKSPACE_BYTES(LINEARS_SIZE_ADD((n), LINEARS_SIZE_MUL((n), (n))), 1)

/**
 * @brief Basic Q-update scratch bytes, including atomic output staging.
 *
 * n and m are positive and side-effect-free; SIZE_MAX signals overflow.
 * Allocate a real linears_workspace_unit_t array covering this byte count.
 */
#define FILTERS_KALMAN_Q_UPDATE_BASIC_BYTES(n, m) \
    FILTERS_KALMAN_MAX_SIZE(FILTERS_KALMAN_UPDATE_BASIC_BYTES((n), (m)), \
                           FILTERS_KALMAN_Q_PREDICT_BASIC_BYTES(n))

/**
 * @brief Caller-owned single-precision scratch arrays, passed as a borrowed view.
 *
 * Arrays must be real arrays of their declared element types, naturally aligned,
 * mutually disjoint, and disjoint from all active state/input data and descriptors.
 * Capacities must meet the operation's macros. Keep arrays alive for the call;
 * serialize shared use. Scratch is unspecified on failure. Predict ignores
 * decomposition. No view is retained; unused capacity is untouched.
 */
typedef struct {
    float *data; /**< Raw intermediate float array. */
    size_t capacity; /**< Raw capacity in float elements. */
    linears_workspace_unit_t *basic; /**< Separate basic-kernel union array. */
    size_t basic_bytes; /**< Available bytes in basic, not element count. */
    linears_f32_workspace_t decomposition; /**< Separate real float working array. */
} filters_kalman_matrix_f32_workspace_t;

/**
 * @brief Caller-owned double-precision scratch arrays, borrowed for one call.
 *
 * Supply naturally aligned real arrays of the declared types, disjoint from
 * each other, state/input data and descriptors. Sizes follow the operation's
 * macros. Retain arrays throughout the call and serialize shared use. No view
 * is retained. Failure may change scratch; unused capacity is untouched.
 * Prediction ignores decomposition, which may be {NULL,0}.
 */
typedef struct {
    double *data; /**< Raw intermediate double array. */
    size_t capacity; /**< Raw capacity in double elements. */
    linears_workspace_unit_t *basic; /**< Separate basic-kernel union array. */
    size_t basic_bytes; /**< Available bytes in basic. */
    linears_f64_workspace_t decomposition; /**< Separate real double working array. */
} filters_kalman_matrix_f64_workspace_t;

/**
 * @brief Caller-owned q15 scratch arrays with integer-only working storage.
 *
 * Supply naturally aligned real linears_fixed_t, workspace-unit and
 * linears_fixed_t arrays, mutually disjoint and disjoint from signal data and
 * descriptors. Raw signal elements remain int16_t; working matrices are int64_t
 * F31. Use the Q-specific STORAGE/BASIC_BYTES macros and the common update
 * decomposition macro. Retain arrays and serialize shared use. Failure may
 * change scratch; unused capacity is untouched. Prediction ignores decomposition.
 */
typedef struct {
    linears_fixed_t *data; /**< Wide F31 imported matrices and intermediates. */
    size_t capacity; /**< Working capacity in linears_fixed_t elements. */
    linears_workspace_unit_t *basic; /**< Separate basic-kernel union array. */
    size_t basic_bytes; /**< Available bytes in basic. */
    linears_q15_workspace_t decomposition; /**< Real linears_fixed_t working array. */
} filters_kalman_matrix_q15_workspace_t;

/**
 * @brief Caller-owned q31 scratch arrays with integer-only working storage.
 *
 * Supply naturally aligned real linears_fixed_t, workspace-unit and
 * linears_fixed_t arrays, mutually disjoint and disjoint from signal data and
 * descriptors. Raw signal elements remain int32_t; working matrices are int64_t
 * F31. Use the Q-specific STORAGE/BASIC_BYTES macros and the common update
 * decomposition macro. Retain arrays and serialize shared use. Failure may
 * change scratch; unused capacity is untouched. Prediction ignores decomposition.
 */
typedef struct {
    linears_fixed_t *data; /**< Wide F31 imported matrices and intermediates. */
    size_t capacity; /**< Working capacity in linears_fixed_t elements. */
    linears_workspace_unit_t *basic; /**< Separate basic-kernel union array. */
    size_t basic_bytes; /**< Available bytes in basic. */
    linears_q31_workspace_t decomposition; /**< Real linears_fixed_t working array. */
} filters_kalman_matrix_q31_workspace_t;

/**
 * @brief Reset a single-precision dense filter in caller-provided storage.
 *
 * @param[in,out] state Non-NULL initialized storage descriptors and element capacities.
 * @param[in] estimate Non-NULL n-by-1 finite initial state; allocation covers n floats.
 * @param[in] covariance Non-NULL symmetric PSD n-by-n initial covariance, n*n floats.
 * @return linears_status_t OK, ARGUMENT for NULL/misalignment, DIMENSION for
 *         shape mismatch, SIZE for size overflow, WORKSPACE for low capacity,
 *         ALIAS for forbidden overlap, RANGE for nonfinite values, or DOMAIN
 *         for asymmetry/negative covariance diagonal. Full PSD is a precondition.
 * @note No scratch. Exact corresponding self-alias is allowed, partial/cross
 *       alias is not. Failure preserves state; serialize shared use.
 */
linears_status_t filters_kalman_matrix_f32_reset(filters_kalman_matrix_t *state,
                                               const linears_matrix_t *estimate,
                                               const linears_matrix_t *covariance);

/**
 * @brief Predict x=F*x and P=F*P*F^T+Q in single precision.
 *
 * @param[in,out] state Non-NULL reset n-state filter; float buffers stay alive.
 * @param[in] transition Non-NULL finite n-by-n F for one caller-selected interval.
 * @param[in] process_noise Non-NULL symmetric PSD n-by-n Q in covariance units.
 * @param[in] workspace Borrowed aligned raw/basic arrays meeting prediction
 *                      size macros, disjoint from state/data/descriptors.
 *                      decomposition is ignored; array contents may change.
 * @return linears_status_t OK, ARGUMENT, DIMENSION, SIZE, WORKSPACE, ALIAS,
 *         RANGE (nonfinite input/result), or DOMAIN (asymmetric/negative diagonal).
 * @note Inputs may alias each other, never state. All allocations cover their
 *       shapes. Failure preserves both state arrays/descriptors, not scratch.
 *       Successful covariance is symmetrized. Serialize shared state/scratch.
 */
linears_status_t filters_kalman_matrix_f32_predict(filters_kalman_matrix_t *state,
                                                 const linears_matrix_t *transition,
                                                 const linears_matrix_t *process_noise,
                                                 filters_kalman_matrix_f32_workspace_t workspace);

/**
 * @brief Apply a dense single-precision Joseph measurement update.
 *
 * @param[in,out] state Non-NULL reset n-state filter; float buffers stay alive.
 * @param[in] measurement Non-NULL finite m-by-1 z in measurement units, m>0.
 * @param[in] observation Non-NULL finite m-by-n H mapping state to measurement units.
 * @param[in] measurement_noise Non-NULL symmetric SPD m-by-m R in measurement covariance units.
 * @param[in] workspace Borrowed aligned raw/basic/decomposition arrays meeting
 *                      update size macros for n,m; mutually disjoint and disjoint
 *                      from state/data/descriptors. Array contents may change.
 * @return linears_status_t OK, ARGUMENT, DIMENSION, SIZE, WORKSPACE, ALIAS,
 *         RANGE, DOMAIN (asymmetry/negative P diagonal), or NOT_POSITIVE_DEFINITE
 *         when R or the innovation covariance cannot be factored. A zero LU
 *         working pivot returns SINGULAR.
 * @note Cholesky-checks R/S, then uses pivoted LU solve, never inverse. Zero
 *       residual is valid. Inputs may alias
 *       each other, never state; allocations cover shapes. Failure preserves
 *       state, not scratch. Serialize calls sharing state or workspace.
 */
linears_status_t filters_kalman_matrix_f32_update(filters_kalman_matrix_t *state,
                                                const linears_matrix_t *measurement,
                                                const linears_matrix_t *observation,
                                                const linears_matrix_t *measurement_noise,
                                                filters_kalman_matrix_f32_workspace_t workspace);

/**
 * @brief Reset a double-precision dense filter in caller-provided storage.
 *
 * @param[in,out] state Non-NULL initialized storage descriptors and element capacities.
 * @param[in] estimate Non-NULL n-by-1 finite initial state; allocation covers n doubles.
 * @param[in] covariance Non-NULL symmetric PSD n-by-n initial covariance, n*n doubles.
 * @return linears_status_t OK, ARGUMENT for NULL/misalignment, DIMENSION for
 *         shape mismatch, SIZE for overflow, WORKSPACE for low capacity,
 *         ALIAS for forbidden overlap, RANGE for nonfinite values, or DOMAIN
 *         for asymmetry/negative diagonal. Full PSD remains a precondition.
 * @note No scratch. Exact corresponding self-alias is allowed, partial/cross
 *       alias is not. Failure preserves state; serialize shared use.
 */
linears_status_t filters_kalman_matrix_f64_reset(filters_kalman_matrix_t *state,
                                               const linears_matrix_t *estimate,
                                               const linears_matrix_t *covariance);

/**
 * @brief Predict x=F*x and P=F*P*F^T+Q in double precision.
 *
 * @param[in,out] state Non-NULL reset n-state filter; double buffers stay alive.
 * @param[in] transition Non-NULL finite n-by-n F for one caller-selected interval.
 * @param[in] process_noise Non-NULL symmetric PSD n-by-n Q in covariance units.
 * @param[in] workspace Borrowed aligned raw/basic arrays meeting prediction
 *                      size macros, disjoint from state/data/descriptors.
 *                      decomposition is ignored; array contents may change.
 * @return linears_status_t OK, ARGUMENT, DIMENSION, SIZE, WORKSPACE, ALIAS,
 *         RANGE (nonfinite input/result), or DOMAIN (asymmetric/negative diagonal).
 * @note Inputs may alias each other, never state; allocations cover shapes.
 *       Failure preserves both state arrays/descriptors, not scratch. Successful
 *       covariance is symmetrized. Serialize shared state/workspace.
 */
linears_status_t filters_kalman_matrix_f64_predict(filters_kalman_matrix_t *state,
                                                 const linears_matrix_t *transition,
                                                 const linears_matrix_t *process_noise,
                                                 filters_kalman_matrix_f64_workspace_t workspace);

/**
 * @brief Apply a dense double-precision Joseph measurement update.
 *
 * @param[in,out] state Non-NULL reset n-state filter; double buffers stay alive.
 * @param[in] measurement Non-NULL finite m-by-1 z in measurement units, m>0.
 * @param[in] observation Non-NULL finite m-by-n H mapping state to measurement units.
 * @param[in] measurement_noise Non-NULL symmetric SPD m-by-m R in measurement covariance units.
 * @param[in] workspace Borrowed aligned raw/basic/decomposition arrays meeting
 *                      update size macros for n,m; mutually disjoint and disjoint
 *                      from state/data/descriptors. Array contents may change.
 * @return linears_status_t OK, ARGUMENT, DIMENSION, SIZE, WORKSPACE, ALIAS,
 *         RANGE, DOMAIN (asymmetry/negative P diagonal), or NOT_POSITIVE_DEFINITE
 *         when R or the innovation covariance cannot be factored. A zero LU
 *         working pivot returns SINGULAR.
 * @note Cholesky-checks R/S, then uses pivoted LU solve, never inverse. Zero
 *       residual is valid. Inputs may alias
 *       each other, never state; allocations cover shapes. Failure preserves
 *       state, not scratch. Serialize shared state/workspace.
 */
linears_status_t filters_kalman_matrix_f64_update(filters_kalman_matrix_t *state,
                                                const linears_matrix_t *measurement,
                                                const linears_matrix_t *observation,
                                                const linears_matrix_t *measurement_noise,
                                                filters_kalman_matrix_f64_workspace_t workspace);

/**
 * @brief Reset a q15 dense filter without floating-point conversion.
 *
 * @param[in,out] state Non-NULL initialized storage descriptors and element capacities.
 * @param[in] estimate Non-NULL n-by-1 q15 state in physical units; n int16_t elements.
 * @param[in] covariance Non-NULL symmetric PSD n-by-n q15 covariance; n*n elements.
 * @return linears_status_t OK, ARGUMENT for NULL/misalignment, DIMENSION for
 *         shape mismatch, SIZE for overflow, WORKSPACE for low capacity,
 *         ALIAS for forbidden overlap, or DOMAIN
 *         for asymmetry/negative diagonal. Full PSD remains a precondition.
 * @note Each element is fixed F15 raw/32768. No scratch. Exact corresponding
 *       self-alias is allowed, partial/cross alias is not. Failure preserves
 *       state; serialize shared use. Reset copies raw values unchanged.
 */
linears_status_t filters_kalman_matrix_q15_reset(filters_kalman_matrix_t *state,
                                               const linears_matrix_t *estimate,
                                               const linears_matrix_t *covariance);

/**
 * @brief Predict a dense q15 filter with shared integer-only matrix kernels.
 *
 * @param[in,out] state Non-NULL reset n-state filter; int16_t signal buffers stay alive.
 * @param[in] transition Non-NULL raw F15 n-by-n F for one caller-selected interval.
 * @param[in] process_noise Non-NULL symmetric PSD F15 n-by-n Q in covariance units.
 * @param[in] workspace Borrowed fixed-F31/basic arrays meeting Q_PREDICT_STORAGE
 *                      and Q_PREDICT_BASIC_BYTES, disjoint from state/data/descriptors.
 *                      decomposition is ignored; array contents may change.
 * @return linears_status_t OK, ARGUMENT, DIMENSION, SIZE, WORKSPACE, ALIAS,
 *         RANGE for working overflow or a computed/rounded F31 estimate or
 *         covariance outside [-1,+1], or DOMAIN for asymmetry/negative diagonal.
 * @note Inputs may alias each other, never state; allocations cover shapes.
 *       Success commits raw outputs and symmetrizes covariance. The range test
 *       is not against an ideal unrounded expression: F31 working rounding can
 *       precede the separate positive-endpoint one-signal-LSB mapping error.
 *       Failure preserves state, not scratch. Serialize shared state/workspace.
 */
linears_status_t filters_kalman_matrix_q15_predict(filters_kalman_matrix_t *state,
                                                 const linears_matrix_t *transition,
                                                 const linears_matrix_t *process_noise,
                                                 filters_kalman_matrix_q15_workspace_t workspace);

/**
 * @brief Apply an integer-only dense q15 Joseph measurement update.
 *
 * @param[in,out] state Non-NULL reset n-state filter; int16_t buffers stay alive.
 * @param[in] measurement Non-NULL q15 m-by-1 z in measurement units, m>0.
 * @param[in] observation Non-NULL q15 m-by-n H mapping state to measurement units.
 * @param[in] measurement_noise Non-NULL symmetric SPD F15 m-by-m R in measurement covariance units.
 * @param[in] workspace Borrowed fixed-F31/basic/decomposition arrays meeting the
 *                      Q_UPDATE_STORAGE, Q_UPDATE_BASIC_BYTES and common
 *                      UPDATE_DECOMP_STORAGE macros; all active storage is disjoint.
 * @return linears_status_t OK, ARGUMENT, DIMENSION, SIZE, WORKSPACE, ALIAS,
 *         RANGE, DOMAIN (asymmetry/negative P diagonal), or NOT_POSITIVE_DEFINITE
 *         for R/innovation pivots, including positive pivots lost to quantization.
 *         A zero LU working pivot returns SINGULAR. RANGE includes working
 *         overflow or a computed/rounded F31 estimate or covariance outside
 *         [-1,+1], not a check of the ideal unrounded full expression.
 * @note Integer Cholesky checks R/S before pivoted LU solve; no inverse. Zero
 *       residual is valid. Inputs may
 *       alias each other, never state; allocations cover shapes. Success commits
 *       raw outputs. F31 working-rounding error is additional to the separate
 *       positive-endpoint one-signal-LSB mapping error. Failure preserves state,
 *       not scratch. Serialize shared use.
 */
linears_status_t filters_kalman_matrix_q15_update(filters_kalman_matrix_t *state,
                                                const linears_matrix_t *measurement,
                                                const linears_matrix_t *observation,
                                                const linears_matrix_t *measurement_noise,
                                                filters_kalman_matrix_q15_workspace_t workspace);

/**
 * @brief Reset a q31 dense filter without floating-point conversion.
 *
 * @param[in,out] state Non-NULL initialized storage descriptors and element capacities.
 * @param[in] estimate Non-NULL n-by-1 q31 state in physical units; n int32_t elements.
 * @param[in] covariance Non-NULL symmetric PSD n-by-n q31 covariance; n*n elements.
 * @return linears_status_t OK, ARGUMENT for NULL/misalignment, DIMENSION for
 *         shape mismatch, SIZE for overflow, WORKSPACE for low capacity,
 *         ALIAS for forbidden overlap, or DOMAIN
 *         for asymmetry/negative diagonal. Full PSD remains a precondition.
 * @note Each element is fixed F31 raw/2147483648. No scratch. Exact corresponding
 *       self-alias is allowed, partial/cross alias is not. Failure preserves
 *       state; serialize shared use. Reset copies raw values unchanged.
 */
linears_status_t filters_kalman_matrix_q31_reset(filters_kalman_matrix_t *state,
                                               const linears_matrix_t *estimate,
                                               const linears_matrix_t *covariance);

/**
 * @brief Predict a dense q31 filter with shared integer-only matrix kernels.
 *
 * @param[in,out] state Non-NULL reset n-state filter; int32_t signal buffers stay alive.
 * @param[in] transition Non-NULL raw F31 n-by-n F for one caller-selected interval.
 * @param[in] process_noise Non-NULL symmetric PSD F31 n-by-n Q in covariance units.
 * @param[in] workspace Borrowed fixed-F31/basic arrays meeting Q_PREDICT_STORAGE
 *                      and Q_PREDICT_BASIC_BYTES, disjoint from state/data/descriptors.
 *                      decomposition is ignored; array contents may change.
 * @return linears_status_t OK, ARGUMENT, DIMENSION, SIZE, WORKSPACE, ALIAS,
 *         RANGE for working overflow or a computed/rounded F31 estimate or
 *         covariance outside [-1,+1], or DOMAIN for asymmetry/negative diagonal.
 * @note Inputs may alias each other, never state; allocations cover shapes.
 *       Success commits raw outputs and symmetrizes covariance. The range test
 *       is not against an ideal unrounded expression: F31 working rounding can
 *       precede the separate positive-endpoint one-signal-LSB mapping error.
 *       Failure preserves state, not scratch. Serialize shared state/workspace.
 */
linears_status_t filters_kalman_matrix_q31_predict(filters_kalman_matrix_t *state,
                                                 const linears_matrix_t *transition,
                                                 const linears_matrix_t *process_noise,
                                                 filters_kalman_matrix_q31_workspace_t workspace);

/**
 * @brief Apply an integer-only dense q31 Joseph measurement update.
 *
 * @param[in,out] state Non-NULL reset n-state filter; int32_t buffers stay alive.
 * @param[in] measurement Non-NULL q31 m-by-1 z in measurement units, m>0.
 * @param[in] observation Non-NULL q31 m-by-n H mapping state to measurement units.
 * @param[in] measurement_noise Non-NULL symmetric SPD F31 m-by-m R in measurement covariance units.
 * @param[in] workspace Borrowed fixed-F31/basic/decomposition arrays meeting the
 *                      Q_UPDATE_STORAGE, Q_UPDATE_BASIC_BYTES and common
 *                      UPDATE_DECOMP_STORAGE macros; all active storage is disjoint.
 * @return linears_status_t OK, ARGUMENT, DIMENSION, SIZE, WORKSPACE, ALIAS,
 *         RANGE, DOMAIN (asymmetry/negative P diagonal), or NOT_POSITIVE_DEFINITE
 *         for R/innovation pivots, including positive pivots lost to quantization.
 *         A zero LU working pivot returns SINGULAR. RANGE includes working
 *         overflow or a computed/rounded F31 estimate or covariance outside
 *         [-1,+1], not a check of the ideal unrounded full expression.
 * @note Integer Cholesky checks R/S before pivoted LU solve; no inverse. Zero
 *       residual is valid. Inputs may
 *       alias each other, never state; allocations cover shapes. Success commits
 *       raw outputs. F31 working-rounding error is additional to the separate
 *       positive-endpoint one-signal-LSB mapping error. Failure preserves state,
 *       not scratch. Serialize shared use.
 */
linears_status_t filters_kalman_matrix_q31_update(filters_kalman_matrix_t *state,
                                                const linears_matrix_t *measurement,
                                                const linears_matrix_t *observation,
                                                const linears_matrix_t *measurement_noise,
                                                filters_kalman_matrix_q31_workspace_t workspace);

#ifdef __cplusplus
}
#endif
#endif
