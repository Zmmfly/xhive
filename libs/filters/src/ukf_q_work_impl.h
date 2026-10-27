/* Dedicated mixed Q UKF. Only callback rows, extended samples, factors and
 * gain rows use U_WORK; ordinary statistics accumulate raw integer products.
 * U_BITS is a compile-time signal format, never dynamic scaling metadata. */
#include "q_workspace_impl.h"

#define Q_JOIN_INNER(a, b) a##b
#define Q_JOIN(a, b) Q_JOIN_INNER(a, b)
#define Q_API(name) Q_JOIN(Q_JOIN(filters_ukf_, U_TAG), Q_JOIN(_, name))
#define Q_LOCAL(name) Q_JOIN(Q_JOIN(ukf_q_, U_TAG), Q_JOIN(_, name))
#define Q_SCRATCH(name) Q_JOIN(Q_JOIN(ukf_work_, U_TAG), Q_JOIN(_, name))
#define Q_LINEAR(name) Q_JOIN(Q_JOIN(linears_, name), Q_JOIN(_, U_FLOAT_TAG))
#define Q_TRY(expression) do { status = (expression); if (status != LINEARS_OK) return status; } while (0)

#if U_BITS == 15
#define Q_SIGNAL_MAX INT16_MAX
typedef int64_t Q_LOCAL(accum_t);
#define Q_ACCUM_ZERO INT64_C(0)
static inline void Q_LOCAL(mac)(Q_LOCAL(accum_t) *sum, int64_t product)
{
    /* The validated n*n floating workspace bounds 2*n+1 products, each
     * <=2^31 even for the difference dot below, inside signed int64_t. */
    *sum += product;
}
static inline U_WORK Q_LOCAL(accum_value)(Q_LOCAL(accum_t) sum)
{
    return (U_WORK)sum * (U_WORK)0x1p-30;
}
#else
#define Q_SIGNAL_MAX INT32_MAX
typedef linears_accum_t Q_LOCAL(accum_t);
#define Q_ACCUM_ZERO {UINT64_C(0), UINT64_C(0)}
static inline void Q_LOCAL(mac)(Q_LOCAL(accum_t) *sum, int64_t product)
{
    uint64_t old = sum->low;
    sum->low += (uint64_t)product;
    sum->high += (product < 0 ? UINT64_MAX : UINT64_C(0)) + (sum->low < old);
}
static inline U_WORK Q_LOCAL(accum_value)(Q_LOCAL(accum_t) sum)
{
    int negative = (int)(sum.high >> 63);
    U_WORK value;
    if (negative) {
        sum.low = ~sum.low + UINT64_C(1);
        sum.high = ~sum.high + (sum.low == 0);
    }
    if (sum.high == 0) value = (U_WORK)sum.low * (U_WORK)0x1p-62;
    else value = ((U_WORK)sum.high * (U_WORK)0x1p64 + (U_WORK)sum.low) * (U_WORK)0x1p-62;
    return negative ? -value : value;
}
#endif

static U_WORK Q_LOCAL(dot)(const U_SIGNAL *a, size_t astride, const U_SIGNAL *b,
                            size_t bstride, size_t count)
{
    Q_LOCAL(accum_t) sum = Q_ACCUM_ZERO;
    size_t i;
    for (i = 0; i < count; ++i)
        Q_LOCAL(mac)(&sum, (int64_t)a[i * astride] * b[i * bstride]);
    return Q_LOCAL(accum_value)(sum);
}

/* For a regenerated sigma pair, dx+ = L and dx- = -L. Raw q31 times a
 * difference of two q31 values still fits int64_t; only the sum needs limbs. */
static U_WORK Q_LOCAL(dot_difference)(const U_SIGNAL *a, const U_SIGNAL *positive,
                                       const U_SIGNAL *negative, size_t stride, size_t count)
{
    Q_LOCAL(accum_t) sum = Q_ACCUM_ZERO;
    size_t i;
    for (i = 0; i < count; ++i) {
        int64_t difference = (int64_t)positive[i * stride] - negative[i * stride];
        Q_LOCAL(mac)(&sum, (int64_t)a[i] * difference);
    }
    return Q_LOCAL(accum_value)(sum);
}

/* Three k-element metadata vectors: maximum magnitude, weighted conversion
 * error, and all-in-range flag. An out-of-range sample is not clipped or
 * rejected; its column keeps the original floating calculation. */
static void Q_LOCAL(columns)(const linears_matrix_t *points, U_WORK first_weight,
                              U_WORK other_weight, U_SIGNAL *raw, U_WORK *metadata)
{
    const U_WORK *values = (const U_WORK *)points->data;
    size_t i, j, count = points->rows, k = points->cols;
    U_WORK *maximum = metadata, *error = metadata + k, *valid = metadata + 2 * k;
    for (j = 0; j < k; ++j) {
        maximum[j] = error[j] = (U_WORK)0;
        valid[j] = (U_WORK)1;
        for (i = 0; i < count; ++i) {
            U_WORK value = values[i * k + j], magnitude = U_ABS(value);
            if (magnitude > maximum[j]) maximum[j] = magnitude;
            if (!U_IN_UNIT(value)) {
                valid[j] = (U_WORK)0;
                raw[i * k + j] = 0;
            } else {
                U_WORK difference;
                Q_JOIN(ukf_store_, U_TAG)(&value, 1, raw + i * k + j);
                difference = U_ABS(value - Q_JOIN(ukf_widen_value_, U_TAG)(raw[i * k + j]));
                /* A nonzero sub-LSB deviation must not collapse a positive
                 * working covariance needed by the next UPF solve. */
                if (value != (U_WORK)0 && raw[i * k + j] == 0) valid[j] = (U_WORK)0;
                error[j] += U_ABS(i == 0 ? first_weight : other_weight) * difference;
            }
        }
    }
}

/* A covariance entry is allowed at most one signal LSB of conversion error.
 * That absolute budget cannot resolve a working covariance whose nonzero
 * product scale is itself below one signal LSB: retain floating products
 * there, even when each deviation rounds to a nonzero Q value. Both checks
 * are outside the MAC loop and select a local dot, never an entire filter. */
static int Q_LOCAL(product_fits)(const U_WORK *a, size_t acount, size_t ai,
                                 const U_WORK *b, size_t bcount, size_t bi)
{
    U_WORK ae = a[acount + ai], be = b[bcount + bi];
    U_WORK bound;
    if (a[2 * acount + ai] == 0 || b[2 * bcount + bi] == 0) return 0;
    if (a[ai] != (U_WORK)0 && b[bi] != (U_WORK)0 && a[ai] * b[bi] < U_LSB) return 0;
    bound = a[ai] * be + b[bi] * ae + U_LSB * (ae < be ? ae : be);
    return bound <= U_LSB;
}

static linears_status_t Q_LOCAL(mean)(const linears_matrix_t *points, const U_WEIGHTS *weights,
                                       U_RESIDUAL_FN residual, void *context, linears_matrix_t *out,
                                       U_SIGNAL *raw, U_WORK *temporary)
{
    const U_WORK *values = (const U_WORK *)points->data;
    U_WORK *result = (U_WORK *)out->data;
    size_t i, j, count = points->rows, k = points->cols;
    linears_status_t status;
    if (residual != NULL) {
        linears_matrix_t reference = Q_SCRATCH(row)(points, 0), delta = {temporary, k, 1};
        for (j = 0; j < k; ++j) result[j] = (U_WORK)0;
        for (i = 0; i < count; ++i) {
            linears_matrix_t point = Q_SCRATCH(row)(points, i);
            U_WORK weight = i == 0 ? weights->mean0 : weights->other;
            Q_TRY(residual(&point, &reference, &delta, context));
            Q_TRY(U_FINITE(&delta));
            for (j = 0; j < k; ++j) result[j] += weight * temporary[j];
        }
        for (j = 0; j < k; ++j) result[j] += values[j];
    } else {
        U_WORK bound = (U_WORK)0;
        U_WORK term_bound = U_ABS(weights->other) * (U_LSB * (U_WORK)0.5);
        int bounded_error;
        /* A nonsaturating cell contributes at most half an LSB. Repeated
         * positive additions preserve the original error sum's rounding
         * order, so monotonicity makes this a sufficient bound. */
        for (i = 1; i < count; ++i) bound += term_bound;
        bounded_error = bound <= U_LSB * (U_WORK)0.5;
        for (j = 0; j < k; ++j) {
            U_WORK error = (U_WORK)0;
            int valid = 1, finite_differences = 1, has_maximum = 0;
            /* Quantize offsets, not the absolute center. A sub-LSB point
             * cloud then retains its center instead of inventing a variance
             * from a prematurely rounded working mean. */
            for (i = 1; i < count; ++i) {
                U_WORK delta = values[i * k + j] - values[j];
                if (!U_ISFINITE(delta)) finite_differences = 0;
                if (!U_IN_UNIT(delta)) valid = 0;
                else {
                    Q_JOIN(ukf_store_, U_TAG)(&delta, 1, raw + i * k + j);
                    if (delta != (U_WORK)0 && raw[i * k + j] == 0) valid = 0;
                    if (bounded_error) {
                        if (raw[i * k + j] == Q_SIGNAL_MAX) has_maximum = 1;
                    } else {
                        error += U_ABS(weights->other) *
                                 U_ABS(delta - Q_JOIN(ukf_widen_value_, U_TAG)(raw[i * k + j]));
                    }
                }
            }
            /* MAX can represent +1 with a full LSB of error. Such columns
             * need their actual error, in the original order, not a forced
             * floating fallback. Invalid columns cannot use the Q mean. */
            if (bounded_error && has_maximum && valid) {
                for (i = 1; i < count; ++i) {
                    U_WORK delta = values[i * k + j] - values[j];
                    error += U_ABS(weights->other) *
                             U_ABS(delta - Q_JOIN(ukf_widen_value_, U_TAG)(raw[i * k + j]));
                }
            }
            if (valid && error <= U_LSB * (U_WORK)0.5) {
                int64_t sum = 0;
                for (i = 1; i < count; ++i) sum += raw[i * k + j];
                result[j] = values[j] + weights->other * ((U_WORK)sum * U_LSB);
            } else if (finite_differences) {
                U_WORK sum = (U_WORK)0;
                for (i = 1; i < count; ++i) sum += weights->other * (values[i * k + j] - values[j]);
                result[j] = values[j] + sum;
            } else {
                U_WORK sum = weights->mean0 * values[j];
                for (i = 1; i < count; ++i) sum += weights->other * values[i * k + j];
                result[j] = sum;
            }
        }
    }
    return U_FINITE(out);
}

/* Replace one dead sample row at a time; the residual callback always gets
 * disjoint input/output and the original sample, not an early Q quantization. */
static linears_status_t Q_LOCAL(deviations)(linears_matrix_t *points, const linears_matrix_t *mean,
                                            U_RESIDUAL_FN residual, void *context, U_WORK *temporary)
{
    const U_WORK *center = (const U_WORK *)mean->data;
    linears_status_t status;
    size_t i, j, k = points->cols;
    linears_matrix_t delta = {temporary, k, 1};
    for (i = 0; i < points->rows; ++i) {
        linears_matrix_t point = Q_SCRATCH(row)(points, i);
        if (residual != NULL) Q_TRY(residual(&point, mean, &delta, context));
        else for (j = 0; j < k; ++j) temporary[j] = ((const U_WORK *)point.data)[j] - center[j];
        Q_TRY(U_FINITE(&delta));
        memcpy(point.data, temporary, k * sizeof(U_WORK));
    }
    return LINEARS_OK;
}

static linears_status_t Q_LOCAL(covariance)(const linears_matrix_t *deviations, const U_SIGNAL *raw,
                                            const U_WORK *metadata, const U_WEIGHTS *weights,
                                            const linears_matrix_t *noise, linears_matrix_t *out)
{
    const U_WORK *values = (const U_WORK *)deviations->data, *q = (const U_WORK *)noise->data;
    U_WORK *result = (U_WORK *)out->data;
    size_t i, r, c, count = deviations->rows, k = deviations->cols;
    for (r = 0; r < k; ++r) {
        for (c = 0; c <= r; ++c) {
            U_WORK value;
            if (Q_LOCAL(product_fits)(metadata, k, r, metadata, k, c)) {
                U_WORK first = Q_LOCAL(dot)(raw + r, k, raw + c, k, 1);
                U_WORK others = Q_LOCAL(dot)(raw + k + r, k, raw + k + c, k, count - 1);
                value = weights->covariance0 * first + weights->other * others;
            } else {
                value = (U_WORK)0;
                for (i = 0; i < count; ++i) {
                    U_WORK weight = i == 0 ? weights->covariance0 : weights->other;
                    value += values[i * k + r] * (weight * values[i * k + c]);
                }
            }
            value += q[r * k + c];
            if (!U_ISFINITE(value)) return LINEARS_ERROR_RANGE;
            result[r * k + c] = result[c * k + r] = value;
        }
    }
    return LINEARS_OK;
}

static linears_status_t Q_LOCAL(weights)(const U_WEIGHTS *weights)
{
    if (weights == NULL) return LINEARS_ERROR_ARGUMENT;
    if (!U_ISFINITE(weights->scale) || !U_ISFINITE(weights->mean0) ||
        !U_ISFINITE(weights->covariance0) || !U_ISFINITE(weights->other)) return LINEARS_ERROR_RANGE;
    return weights->scale > (U_WORK)0 ? LINEARS_OK : LINEARS_ERROR_DOMAIN;
}

static linears_status_t Q_LOCAL(factor)(const linears_matrix_t *covariance, U_WORK scale,
                                        linears_matrix_t *scaled, linears_matrix_t *factor,
                                        U_WORKSPACE workspace)
{
    size_t i, count = covariance->rows * covariance->cols;
    const U_WORK *values = (const U_WORK *)covariance->data;
    U_WORK *target = (U_WORK *)scaled->data;
    for (i = 0; i < count; ++i) {
        target[i] = scale * values[i];
        if (!U_ISFINITE(target[i])) return LINEARS_ERROR_RANGE;
    }
    return Q_LINEAR(cholesky)(scaled, factor, workspace.decomposition);
}

static linears_status_t Q_LOCAL(propagate)(const linears_matrix_t *mean, const linears_matrix_t *factor,
                                           U_FN model, void *context, linears_matrix_t *samples,
                                           U_WORK *row_data)
{
    const U_WORK *center = (const U_WORK *)mean->data, *l = (const U_WORK *)factor->data;
    size_t i, j, n = mean->rows;
    linears_status_t status;
    linears_matrix_t row = {row_data, n, 1};
    for (i = 0; i < samples->rows; ++i) {
        linears_matrix_t target = Q_SCRATCH(row)(samples, i);
        for (j = 0; j < n; ++j) {
            U_WORK offset = i == 0 ? (U_WORK)0 : l[j * n + (i <= n ? i - 1 : i - n - 1)];
            row_data[j] = i <= n ? center[j] + offset : center[j] - offset;
        }
        Q_TRY(U_FINITE(&row));
        Q_TRY(model(&row, &target, context));
        Q_TRY(U_FINITE(&target));
    }
    return LINEARS_OK;
}

linears_status_t Q_API(predict_work)(const linears_matrix_t *mean, const linears_matrix_t *covariance,
                                     const linears_matrix_t *process_noise, const U_WEIGHTS *weights,
                                     U_FN transition, void *transition_context,
                                     U_RESIDUAL_FN residual, void *residual_context,
                                     linears_matrix_t *out_mean, linears_matrix_t *out_covariance,
                                     U_WORKSPACE workspace)
{
    linears_status_t status;
    ukf_spans_t spans;
    linears_matrix_t scaled, factor, samples;
    U_WORK *cursor, *row, *temporary, *metadata;
    size_t n, s;
    if (transition == NULL) return LINEARS_ERROR_ARGUMENT;
    Q_TRY(Q_LOCAL(weights)(weights));
    spans.count = 0;
    Q_TRY(ukf_matrix(&spans, mean, 0, 1, sizeof(U_WORK), U_ALIGNOF(U_WORK), UKF_INPUT));
    n = mean->rows;
    Q_TRY(ukf_matrix(&spans, covariance, n, n, sizeof(U_WORK), U_ALIGNOF(U_WORK), UKF_INPUT));
    Q_TRY(ukf_matrix(&spans, process_noise, n, n, sizeof(U_WORK), U_ALIGNOF(U_WORK), UKF_INPUT));
    Q_TRY(ukf_matrix(&spans, out_mean, n, 1, sizeof(U_WORK), U_ALIGNOF(U_WORK), UKF_WRITABLE));
    Q_TRY(ukf_matrix(&spans, out_covariance, n, n, sizeof(U_WORK), U_ALIGNOF(U_WORK), UKF_WRITABLE));
    Q_TRY(Q_SCRATCH(workspace)(&spans, workspace, FILTERS_UKF_Q_PREDICT_WORK_STORAGE(n), 0,
                               FILTERS_UKF_Q_PREDICT_DECOMP_STORAGE(n), FILTERS_UKF_Q_PREDICT_SIGNAL_STORAGE(n)));
    Q_TRY(U_FINITE(mean)); Q_TRY(U_FINITE(covariance)); Q_TRY(U_FINITE(process_noise));
    Q_TRY(Q_JOIN(ukf_symmetric_, U_FLOAT_TAG)(covariance, 1));
    Q_TRY(Q_JOIN(ukf_symmetric_, U_FLOAT_TAG)(process_noise, 1));
    workspace.decomposition.capacity = FILTERS_UKF_Q_PREDICT_DECOMP_STORAGE(n);
    s = FILTERS_UKF_SIGMA_COUNT(n);
    cursor = workspace.data;
    scaled = Q_SCRATCH(take)(&cursor, n, n);
    factor = Q_SCRATCH(take)(&cursor, n, n);
    samples = Q_SCRATCH(take)(&cursor, s, n);
    row = cursor; cursor += n;
    temporary = cursor; cursor += n;
    metadata = cursor;
    Q_TRY(Q_LOCAL(factor)(covariance, weights->scale, &scaled, &factor, workspace));
    Q_TRY(Q_LOCAL(propagate)(mean, &factor, transition, transition_context, &samples, row));
    Q_TRY(Q_LOCAL(mean)(&samples, weights, residual, residual_context, out_mean,
                         workspace.signal, temporary));
    Q_TRY(Q_LOCAL(deviations)(&samples, out_mean, residual, residual_context, temporary));
    Q_LOCAL(columns)(&samples, weights->covariance0, weights->other, workspace.signal, metadata);
    return Q_LOCAL(covariance)(&samples, workspace.signal, metadata, weights, process_noise, out_covariance);
}

/* Cache the untransposed factor in Q. Each row represents the paired state
 * deviations +L[:,j], -L[:,j], so its weighted error counts each entry twice. */
static void Q_LOCAL(factor_signals)(const linears_matrix_t *factor, U_WORK other,
                                    U_SIGNAL *raw, U_WORK *metadata)
{
    const U_WORK *values = (const U_WORK *)factor->data;
    size_t r, j, n = factor->rows;
    for (r = 0; r < n; ++r) {
        U_WORK maximum = 0, error = 0, valid = 1;
        for (j = 0; j <= r; ++j) {
            U_WORK value = values[r * n + j], magnitude = U_ABS(value);
            if (magnitude > maximum) maximum = magnitude;
            if (!U_IN_UNIT(value)) {
                valid = 0;
                raw[r * n + j] = 0;
            } else {
                Q_JOIN(ukf_store_, U_TAG)(&value, 1, raw + r * n + j);
                if (value != (U_WORK)0 && raw[r * n + j] == 0) valid = 0;
                error += U_ABS(value - Q_JOIN(ukf_widen_value_, U_TAG)(raw[r * n + j]));
            }
        }
        /* Cholesky writes exact zeros above the diagonal. */
        for (j = r + 1; j < n; ++j) raw[r * n + j] = 0;
        metadata[r] = maximum;
        metadata[n + r] = ((U_WORK)2 * U_ABS(other)) * error;
        metadata[2 * n + r] = valid;
    }
}

static linears_status_t Q_LOCAL(cross)(const linears_matrix_t *factor, const U_SIGNAL *raw_factor,
                                       const U_WORK *factor_metadata, const linears_matrix_t *deviations,
                                       const U_SIGNAL *raw_deviations, const U_WORK *metadata,
                                       U_WORK weight, linears_matrix_t *out)
{
    const U_WORK *l = (const U_WORK *)factor->data, *d = (const U_WORK *)deviations->data;
    U_WORK *result = (U_WORK *)out->data;
    size_t r, c, j, n = factor->rows, m = deviations->cols;
    for (r = 0; r < n; ++r) {
        for (c = 0; c < m; ++c) {
            U_WORK value;
            if (Q_LOCAL(product_fits)(factor_metadata, n, r, metadata, m, c)) {
                value = weight * Q_LOCAL(dot_difference)(raw_factor + r * n,
                    raw_deviations + m + c, raw_deviations + (n + 1) * m + c, m, r + 1);
            } else {
                value = (U_WORK)0;
                for (j = 0; j < n; ++j) {
                    value += l[r * n + j] * (weight * d[(j + 1) * m + c]);
                    value -= l[r * n + j] * (weight * d[(j + n + 1) * m + c]);
                }
            }
            if (!U_ISFINITE(value)) return LINEARS_ERROR_RANGE;
            result[r * m + c] = value;
        }
    }
    return LINEARS_OK;
}

/* S = L*L^T was already checked SPD. Solve one C row without a second LU
 * factorization or a whole gain/transpose matrix. No Q range applies to K. */
static linears_status_t Q_LOCAL(gain_row)(const linears_matrix_t *factor, const U_WORK *rhs, U_WORK *gain)
{
    const U_WORK *l = (const U_WORK *)factor->data;
    size_t i, j, m = factor->rows;
    for (i = 0; i < m; ++i) {
        U_WORK value = rhs[i];
        for (j = 0; j < i; ++j) value -= l[i * m + j] * gain[j];
        gain[i] = value / l[i * m + i];
        if (!U_ISFINITE(gain[i])) return LINEARS_ERROR_RANGE;
    }
    for (i = m; i-- > 0;) {
        U_WORK value = gain[i];
        for (j = i + 1; j < m; ++j) value -= l[j * m + i] * gain[j];
        gain[i] = value / l[i * m + i];
        if (!U_ISFINITE(gain[i])) return LINEARS_ERROR_RANGE;
    }
    return LINEARS_OK;
}

linears_status_t Q_API(update_work)(const linears_matrix_t *mean, const linears_matrix_t *covariance,
                                    const linears_matrix_t *measurement_noise,
                                    const linears_matrix_t *measurement, const U_WEIGHTS *weights,
                                    U_FN observation, void *observation_context,
                                    U_RESIDUAL_FN residual, void *residual_context,
                                    linears_matrix_t *out_mean, linears_matrix_t *out_covariance,
                                    U_WORKSPACE workspace)
{
    linears_status_t status;
    ukf_spans_t spans;
    linears_matrix_t scaled, factor, samples, cross, zbar, innovation, s_matrix;
    U_WORK *cursor, *row, *temporary, *gain, *factor_metadata, *metadata;
    U_SIGNAL *raw_factor, *raw_deviations;
    const U_WORK *x, *p;
    U_WORK *xout, *pout;
    size_t n, m, s, k, i, j, r;
    if (observation == NULL) return LINEARS_ERROR_ARGUMENT;
    Q_TRY(Q_LOCAL(weights)(weights));
    spans.count = 0;
    Q_TRY(ukf_matrix(&spans, mean, 0, 1, sizeof(U_WORK), U_ALIGNOF(U_WORK), UKF_INPUT));
    n = mean->rows;
    Q_TRY(ukf_matrix(&spans, covariance, n, n, sizeof(U_WORK), U_ALIGNOF(U_WORK), UKF_INPUT));
    Q_TRY(ukf_matrix(&spans, measurement_noise, 0, 0, sizeof(U_WORK), U_ALIGNOF(U_WORK), UKF_INPUT));
    m = measurement_noise->rows;
    if (measurement_noise->cols != m) return LINEARS_ERROR_DIMENSION;
    Q_TRY(ukf_matrix(&spans, measurement, m, 1, sizeof(U_WORK), U_ALIGNOF(U_WORK), UKF_INPUT));
    Q_TRY(ukf_matrix(&spans, out_mean, n, 1, sizeof(U_WORK), U_ALIGNOF(U_WORK), UKF_WRITABLE));
    Q_TRY(ukf_matrix(&spans, out_covariance, n, n, sizeof(U_WORK), U_ALIGNOF(U_WORK), UKF_WRITABLE));
    Q_TRY(Q_SCRATCH(workspace)(&spans, workspace, FILTERS_UKF_Q_UPDATE_WORK_STORAGE(n, m), 0,
                               FILTERS_UKF_Q_UPDATE_DECOMP_STORAGE(n, m), FILTERS_UKF_Q_UPDATE_SIGNAL_STORAGE(n, m)));
    Q_TRY(U_FINITE(mean)); Q_TRY(U_FINITE(covariance));
    Q_TRY(U_FINITE(measurement_noise)); Q_TRY(U_FINITE(measurement));
    Q_TRY(Q_JOIN(ukf_symmetric_, U_FLOAT_TAG)(covariance, 1));
    Q_TRY(Q_JOIN(ukf_symmetric_, U_FLOAT_TAG)(measurement_noise, 0));
    workspace.decomposition.capacity = FILTERS_UKF_Q_UPDATE_DECOMP_STORAGE(n, m);
    s = FILTERS_UKF_SIGMA_COUNT(n);
    k = FILTERS_KALMAN_MAX_SIZE(n, m);
    cursor = workspace.data;
    scaled = Q_SCRATCH(take)(&cursor, k, k);
    factor = Q_SCRATCH(take)(&cursor, k, k);
    samples = Q_SCRATCH(take)(&cursor, s, m);
    cross = Q_SCRATCH(take)(&cursor, n, m);
    row = cursor; cursor += n;
    temporary = cursor; cursor += m;
    zbar = Q_SCRATCH(take)(&cursor, m, 1);
    innovation = Q_SCRATCH(take)(&cursor, m, 1);
    gain = cursor; cursor += m;
    factor_metadata = cursor; cursor += 3 * n;
    metadata = cursor;
    raw_factor = workspace.signal;
    raw_deviations = workspace.signal + n * n;
    factor.rows = factor.cols = m;
    Q_TRY(Q_LINEAR(cholesky)(measurement_noise, &factor, workspace.decomposition));
    scaled.rows = scaled.cols = factor.rows = factor.cols = n;
    Q_TRY(Q_LOCAL(factor)(covariance, weights->scale, &scaled, &factor, workspace));
    Q_TRY(Q_LOCAL(propagate)(mean, &factor, observation, observation_context, &samples, row));
    Q_TRY(Q_LOCAL(mean)(&samples, weights, residual, residual_context, &zbar, raw_deviations, temporary));
    Q_TRY(Q_LOCAL(deviations)(&samples, &zbar, residual, residual_context, temporary));
    Q_LOCAL(columns)(&samples, weights->covariance0, weights->other, raw_deviations, metadata);
    Q_LOCAL(factor_signals)(&factor, weights->other, raw_factor, factor_metadata);
    Q_TRY(Q_LOCAL(cross)(&factor, raw_factor, factor_metadata, &samples, raw_deviations, metadata,
                          weights->other, &cross));
    s_matrix = scaled;
    s_matrix.rows = s_matrix.cols = m;
    Q_TRY(Q_LOCAL(covariance)(&samples, raw_deviations, metadata, weights, measurement_noise, &s_matrix));
    factor.rows = factor.cols = m;
    Q_TRY(Q_LINEAR(cholesky)(&s_matrix, &factor, workspace.decomposition));
    if (residual != NULL) Q_TRY(residual(measurement, &zbar, &innovation, residual_context));
    else for (j = 0; j < m; ++j)
        ((U_WORK *)innovation.data)[j] = ((const U_WORK *)measurement->data)[j] - ((const U_WORK *)zbar.data)[j];
    Q_TRY(U_FINITE(&innovation));
    x = (const U_WORK *)mean->data;
    p = (const U_WORK *)covariance->data;
    xout = (U_WORK *)out_mean->data;
    pout = (U_WORK *)out_covariance->data;
    for (r = 0; r < n; ++r) {
        U_WORK correction = (U_WORK)0;
        Q_TRY(Q_LOCAL(gain_row)(&factor, (const U_WORK *)cross.data + r * m, gain));
        for (j = 0; j < m; ++j) correction += gain[j] * ((const U_WORK *)innovation.data)[j];
        xout[r] = x[r] + correction;
        if (!U_ISFINITE(xout[r])) return LINEARS_ERROR_RANGE;
        for (i = 0; i < n; ++i) {
            U_WORK reduction = (U_WORK)0;
            for (j = 0; j < m; ++j) reduction += gain[j] * ((const U_WORK *)cross.data)[i * m + j];
            pout[r * n + i] = p[r * n + i] - reduction;
            if (!U_ISFINITE(pout[r * n + i])) return LINEARS_ERROR_RANGE;
        }
    }
    for (r = 0; r < n; ++r)
        for (i = 0; i < r; ++i) {
            U_WORK value = (U_WORK)0.5 * pout[r * n + i] + (U_WORK)0.5 * pout[i * n + r];
            pout[r * n + i] = pout[i * n + r] = value;
        }
    return LINEARS_OK;
}

#undef Q_ACCUM_ZERO
#undef Q_SIGNAL_MAX
#undef Q_TRY
#undef Q_LINEAR
#undef Q_SCRATCH
#undef Q_LOCAL
#undef Q_API
#undef Q_JOIN
#undef Q_JOIN_INNER
#undef U_TAG
#undef U_FLOAT_TAG
#undef U_SIGNAL
#undef U_BITS
#undef U_WORK
#undef U_LSB
#undef U_ABS
#undef U_WEIGHTS
#undef U_WORKSPACE
#undef U_FN
#undef U_RESIDUAL_FN
#undef U_FINITE
#undef U_ISFINITE
#undef U_IN_UNIT
