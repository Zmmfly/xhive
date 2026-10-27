/* Working-format UKF kernels; included by ukf.c once per kernel tag. */
#include "ukf_scratch_impl.h"

#define U_JOIN_INNER(a, b) a##b
#define U_JOIN(a, b) U_JOIN_INNER(a, b)
#define U_API(name) U_JOIN(U_JOIN(filters_ukf_, U_TAG), U_JOIN(_, name))
#define U_LOCAL(name) U_JOIN(U_JOIN(ukf_work_, U_TAG), U_JOIN(_, name))
#define U_LINEAR(name) U_JOIN(U_JOIN(linears_, name), U_JOIN(_, U_TAG))
#define U_SYMMETRIC U_JOIN(ukf_symmetric_, U_TAG)
#define U_TRY(expression) do { \
    status = (expression); \
    if (status != LINEARS_OK) return status; \
} while (0)
#define U_BASIC workspace.basic, workspace.basic_bytes
#define U_BASIC_MEMBER_f32 f32
#define U_BASIC_MEMBER_f64 floating
#define U_BASIC_MEMBER U_JOIN(U_BASIC_MEMBER_, U_TAG)
#define U_ISFINITE U_JOIN(ukf_isfinite_, U_TAG)
#define U_CHECK_FINITE(matrix) U_TRY(U_FINITE(matrix))

linears_status_t U_API(weights)(size_t states, U_WORK alpha, U_WORK beta, U_WORK kappa,
                                U_WEIGHTS *out)
{
    U_WEIGHTS result;
    U_WORK n = (U_WORK)states, alpha_squared;
    if (out == NULL) return LINEARS_ERROR_ARGUMENT;
    if (states == 0) return LINEARS_ERROR_DIMENSION;
    if (!isfinite(alpha) || !isfinite(beta) || !isfinite(kappa)) return LINEARS_ERROR_RANGE;
    if (!(alpha > 0)) return LINEARS_ERROR_DOMAIN;
    alpha_squared = alpha * alpha;
    result.scale = alpha_squared * (n + kappa);
    if (!isfinite(result.scale)) return LINEARS_ERROR_RANGE;
    if (!(result.scale > 0)) return LINEARS_ERROR_DOMAIN;
    result.mean0 = (result.scale - n) / result.scale;
    result.covariance0 = result.mean0 + (U_WORK)1 - alpha_squared + beta;
    result.other = (U_WORK)0.5 / result.scale;
    if (!isfinite(result.mean0) || !isfinite(result.covariance0) || !isfinite(result.other))
        return LINEARS_ERROR_RANGE;
    *out = result;
    return LINEARS_OK;
}

linears_status_t U_API(sigma_points)(const linears_matrix_t *mean, const linears_matrix_t *covariance,
                                     U_WORK scale, linears_matrix_t *points, U_WORKSPACE workspace)
{
    linears_status_t status;
    ukf_spans_t spans;
    linears_matrix_t scaled, factor, center, row, column;
    U_WORK *cursor;
    size_t n, count, i;
    spans.count = 0;
    U_TRY(ukf_matrix(&spans, mean, 0, 1, sizeof(U_WORK), U_ALIGNOF(U_WORK), UKF_INPUT));
    n = mean->rows;
    U_TRY(ukf_matrix(&spans, covariance, n, n, sizeof(U_WORK), U_ALIGNOF(U_WORK), UKF_INPUT));
    count = FILTERS_UKF_SIGMA_COUNT(n);
    if (count == SIZE_MAX) return LINEARS_ERROR_SIZE;
    U_TRY(ukf_matrix(&spans, points, count, n, sizeof(U_WORK), U_ALIGNOF(U_WORK), UKF_WRITABLE));
    U_TRY(U_LOCAL(workspace)(&spans, workspace, FILTERS_UKF_SIGMA_POINTS_STORAGE(n),
                             FILTERS_UKF_SIGMA_POINTS_BASIC_BYTES(n),
                             FILTERS_UKF_SIGMA_POINTS_DECOMP_STORAGE(n)));
    U_CHECK_FINITE(mean);
    U_CHECK_FINITE(covariance);
    if (!isfinite(scale)) return LINEARS_ERROR_RANGE;
    if (!(scale > 0)) return LINEARS_ERROR_DOMAIN;
    workspace.basic_bytes = FILTERS_UKF_SIGMA_POINTS_BASIC_BYTES(n);
    workspace.decomposition.capacity = FILTERS_UKF_SIGMA_POINTS_DECOMP_STORAGE(n);
    cursor = workspace.data;
    scaled = U_LOCAL(take)(&cursor, n, n);
    factor = U_LOCAL(take)(&cursor, n, n);
    U_TRY(U_LINEAR(axpby)(scale, covariance, (U_WORK)0, covariance, &scaled, U_BASIC));
    U_TRY(U_LINEAR(cholesky)(&scaled, &factor, workspace.decomposition));
    /* Rows of the transposed factor are the columns of L. */
    U_TRY(U_LINEAR(transpose)(&factor, &scaled, U_BASIC));
    center = U_LOCAL(row)(points, 0);
    memcpy(center.data, mean->data, n * sizeof(U_WORK));
    for (i = 0; i < n; ++i) {
        column = U_LOCAL(row)(&scaled, i);
        row = U_LOCAL(row)(points, 1 + i);
        U_TRY(U_LINEAR(add)(mean, &column, &row, U_BASIC));
        row = U_LOCAL(row)(points, 1 + n + i);
        U_TRY(U_LINEAR(sub)(mean, &column, &row, U_BASIC));
    }
    return LINEARS_OK;
}

linears_status_t U_API(propagate)(const linears_matrix_t *points, U_FN fn, void *context,
                                  linears_matrix_t *out)
{
    linears_status_t status;
    ukf_spans_t spans;
    linears_matrix_t in, result;
    size_t i;
    if (fn == NULL) return LINEARS_ERROR_ARGUMENT;
    spans.count = 0;
    U_TRY(ukf_matrix(&spans, points, 0, 0, sizeof(U_WORK), U_ALIGNOF(U_WORK), UKF_INPUT));
    U_TRY(ukf_matrix(&spans, out, points->rows, 0, sizeof(U_WORK), U_ALIGNOF(U_WORK), UKF_WRITABLE));
    U_TRY(ukf_disjoint(&spans));
    U_CHECK_FINITE(points);
    for (i = 0; i < points->rows; ++i) {
        in = U_LOCAL(row)(points, i);
        result = U_LOCAL(row)(out, i);
        U_TRY(fn(&in, &result, context));
        U_CHECK_FINITE(&result);
    }
    return LINEARS_OK;
}

linears_status_t U_API(deviations)(const linears_matrix_t *points, const linears_matrix_t *mean,
                                   U_RESIDUAL_FN residual, void *context, linears_matrix_t *out,
                                   U_WORKSPACE workspace)
{
    linears_status_t status;
    ukf_spans_t spans;
    linears_matrix_t in, result;
    size_t i, k;
    spans.count = 0;
    U_TRY(ukf_matrix(&spans, points, 0, 0, sizeof(U_WORK), U_ALIGNOF(U_WORK), UKF_INPUT));
    k = points->cols;
    U_TRY(ukf_matrix(&spans, mean, k, 1, sizeof(U_WORK), U_ALIGNOF(U_WORK), UKF_INPUT));
    U_TRY(ukf_matrix(&spans, out, points->rows, k, sizeof(U_WORK), U_ALIGNOF(U_WORK), UKF_WRITABLE));
    U_TRY(U_LOCAL(workspace)(&spans, workspace, 0, FILTERS_UKF_DEVIATIONS_BASIC_BYTES(k), 0));
    U_CHECK_FINITE(points);
    U_CHECK_FINITE(mean);
    workspace.basic_bytes = FILTERS_UKF_DEVIATIONS_BASIC_BYTES(k);
    for (i = 0; i < points->rows; ++i) {
        in = U_LOCAL(row)(points, i);
        result = U_LOCAL(row)(out, i);
        if (residual == NULL) {
            U_TRY(U_LINEAR(sub)(&in, mean, &result, U_BASIC));
        } else {
            U_TRY(residual(&in, mean, &result, context));
            U_CHECK_FINITE(&result);
        }
    }
    return LINEARS_OK;
}

linears_status_t U_API(mean)(const linears_matrix_t *points, const linears_matrix_t *weights,
                             U_RESIDUAL_FN residual, void *context, linears_matrix_t *out,
                             U_WORKSPACE workspace)
{
    linears_status_t status;
    ukf_spans_t spans;
    linears_matrix_t deviations, reference;
    const linears_matrix_t *source = points;
    const U_WORK *values, *w;
    U_WORK *cursor;
    size_t s, k, i, j;
    spans.count = 0;
    U_TRY(ukf_matrix(&spans, points, 0, 0, sizeof(U_WORK), U_ALIGNOF(U_WORK), UKF_INPUT));
    s = points->rows;
    k = points->cols;
    U_TRY(ukf_matrix(&spans, weights, s, 1, sizeof(U_WORK), U_ALIGNOF(U_WORK), UKF_INPUT));
    U_TRY(ukf_matrix(&spans, out, k, 1, sizeof(U_WORK), U_ALIGNOF(U_WORK), UKF_WRITABLE));
    U_TRY(U_LOCAL(workspace)(&spans, workspace, FILTERS_UKF_MEAN_STORAGE(s, k),
                             FILTERS_UKF_MEAN_BASIC_BYTES(s, k), 0));
    U_CHECK_FINITE(points);
    U_CHECK_FINITE(weights);
    workspace.basic_bytes = FILTERS_UKF_MEAN_BASIC_BYTES(s, k);
    cursor = workspace.data;
    if (residual != NULL) {
        /* Average residuals about row 0, then move back; exact for linear parts. */
        deviations = U_LOCAL(take)(&cursor, s, k);
        reference = U_LOCAL(row)(points, 0);
        U_TRY(U_API(deviations)(points, &reference, residual, context, &deviations, workspace));
        source = &deviations;
    }
    values = (const U_WORK *)source->data;
    w = (const U_WORK *)weights->data;
    /* Read columns directly, retaining the transposed product's reduction order. */
    for (j = 0; j < k; ++j) {
        U_WORK sum = (U_WORK)0;
        for (i = 0; i < s; ++i) {
            U_WORK product = values[i * k + j] * w[i];
            sum += product;
            if (!U_ISFINITE(sum)) return LINEARS_ERROR_RANGE;
        }
        workspace.basic[j].U_BASIC_MEMBER = sum;
    }
    for (j = 0; j < k; ++j)
        ((U_WORK *)out->data)[j] = workspace.basic[j].U_BASIC_MEMBER;
    if (residual != NULL) return U_LINEAR(add)(&reference, out, out, U_BASIC);
    return LINEARS_OK;
}

linears_status_t U_API(weighted_covariance)(const linears_matrix_t *a, const linears_matrix_t *b,
                                            const linears_matrix_t *weights, linears_matrix_t *out,
                                            U_WORKSPACE workspace)
{
    linears_status_t status;
    ukf_spans_t spans;
    const U_WORK *av, *bv, *w;
    U_WORK *scaled;
    size_t s, p, q, i, j, k;
    spans.count = 0;
    U_TRY(ukf_matrix(&spans, a, 0, 0, sizeof(U_WORK), U_ALIGNOF(U_WORK), UKF_INPUT));
    s = a->rows;
    p = a->cols;
    U_TRY(ukf_matrix(&spans, b, s, 0, sizeof(U_WORK), U_ALIGNOF(U_WORK), UKF_INPUT));
    q = b->cols;
    U_TRY(ukf_matrix(&spans, weights, s, 1, sizeof(U_WORK), U_ALIGNOF(U_WORK), UKF_INPUT));
    U_TRY(ukf_matrix(&spans, out, p, q, sizeof(U_WORK), U_ALIGNOF(U_WORK), UKF_WRITABLE));
    U_TRY(U_LOCAL(workspace)(&spans, workspace, FILTERS_UKF_WEIGHTED_COVARIANCE_STORAGE(s, p, q),
                             FILTERS_UKF_WEIGHTED_COVARIANCE_BASIC_BYTES(s, p, q), 0));
    U_CHECK_FINITE(a);
    U_CHECK_FINITE(b);
    U_CHECK_FINITE(weights);
    scaled = workspace.data;
    av = (const U_WORK *)a->data;
    bv = (const U_WORK *)b->data;
    w = (const U_WORK *)weights->data;
    for (i = 0; i < s; ++i) {
        for (j = 0; j < q; ++j) {
            U_WORK left = w[i] * bv[i * q + j];
            U_WORK right = (U_WORK)0 * bv[i * q + j];
            U_WORK value = left + right;
            if (!U_ISFINITE(value)) return LINEARS_ERROR_RANGE;
            scaled[i * q + j] = value;
        }
    }
    /* Keep a*(w*b + 0*b), including signed zeros, and the original dot order. */
    for (i = 0; i < p; ++i) {
        for (j = 0; j < q; ++j) {
            U_WORK sum = (U_WORK)0;
            for (k = 0; k < s; ++k) {
                U_WORK product = av[k * p + i] * scaled[k * q + j];
                sum += product;
                if (!U_ISFINITE(sum)) return LINEARS_ERROR_RANGE;
            }
            workspace.basic[i * q + j].U_BASIC_MEMBER = sum;
        }
    }
    for (i = 0; i < p * q; ++i)
        ((U_WORK *)out->data)[i] = workspace.basic[i].U_BASIC_MEMBER;
    return LINEARS_OK;
}

static linears_status_t U_LOCAL(weights_valid)(const U_WEIGHTS *weights)
{
    if (weights == NULL) return LINEARS_ERROR_ARGUMENT;
    if (!isfinite(weights->scale) || !isfinite(weights->mean0) ||
        !isfinite(weights->covariance0) || !isfinite(weights->other))
        return LINEARS_ERROR_RANGE;
    return weights->scale > 0 ? LINEARS_OK : LINEARS_ERROR_DOMAIN;
}

static void U_LOCAL(expand_weights)(const U_WEIGHTS *weights, linears_matrix_t *mean,
                                    linears_matrix_t *covariance)
{
    U_WORK *wm = (U_WORK *)mean->data, *wc = (U_WORK *)covariance->data;
    size_t i;
    wm[0] = weights->mean0;
    wc[0] = weights->covariance0;
    for (i = 1; i < mean->rows; ++i) {
        wm[i] = weights->other;
        wc[i] = weights->other;
    }
}

static linears_status_t U_LOCAL(symmetrize)(linears_matrix_t *matrix, linears_matrix_t *scratch,
                                            U_WORKSPACE workspace)
{
    linears_status_t status;
    U_TRY(U_LINEAR(transpose)(matrix, scratch, U_BASIC));
    return U_LINEAR(axpby)((U_WORK)0.5, matrix, (U_WORK)0.5, scratch, matrix, U_BASIC);
}

linears_status_t U_API(predict_work)(const linears_matrix_t *mean, const linears_matrix_t *covariance,
                                     const linears_matrix_t *process_noise, const U_WEIGHTS *weights,
                                     U_FN transition, void *transition_context,
                                     U_RESIDUAL_FN residual, void *residual_context,
                                     linears_matrix_t *out_mean, linears_matrix_t *out_covariance,
                                     U_WORKSPACE workspace)
{
    linears_status_t status;
    ukf_spans_t spans;
    linears_matrix_t tmp, wm, wc, points, scratch, propagated;
    U_WORKSPACE sub;
    U_WORK *cursor;
    size_t n, s, basic_bytes, decomposition_count;
    if (transition == NULL) return LINEARS_ERROR_ARGUMENT;
    U_TRY(U_LOCAL(weights_valid)(weights));
    spans.count = 0;
    U_TRY(ukf_matrix(&spans, mean, 0, 1, sizeof(U_WORK), U_ALIGNOF(U_WORK), UKF_INPUT));
    n = mean->rows;
    U_TRY(ukf_matrix(&spans, covariance, n, n, sizeof(U_WORK), U_ALIGNOF(U_WORK), UKF_INPUT));
    U_TRY(ukf_matrix(&spans, process_noise, n, n, sizeof(U_WORK), U_ALIGNOF(U_WORK), UKF_INPUT));
    U_TRY(ukf_matrix(&spans, out_mean, n, 1, sizeof(U_WORK), U_ALIGNOF(U_WORK), UKF_WRITABLE));
    U_TRY(ukf_matrix(&spans, out_covariance, n, n, sizeof(U_WORK), U_ALIGNOF(U_WORK), UKF_WRITABLE));
    s = FILTERS_UKF_SIGMA_COUNT(n);
    basic_bytes = FILTERS_UKF_PREDICT_BASIC_BYTES(n);
    decomposition_count = FILTERS_UKF_PREDICT_DECOMP_STORAGE(n);
    U_TRY(U_LOCAL(workspace)(&spans, workspace, FILTERS_UKF_PREDICT_WORK_STORAGE(n), basic_bytes,
                             decomposition_count));
    U_CHECK_FINITE(mean);
    U_CHECK_FINITE(covariance);
    U_CHECK_FINITE(process_noise);
    U_TRY(U_SYMMETRIC(covariance, 1));
    U_TRY(U_SYMMETRIC(process_noise, 1));
    /* Pass only the used prefixes; unused supplied capacity stays untouched. */
    workspace.basic_bytes = basic_bytes;
    workspace.decomposition.capacity = decomposition_count;
    cursor = workspace.data;
    tmp = U_LOCAL(take)(&cursor, n, n);
    wm = U_LOCAL(take)(&cursor, s, 1);
    wc = U_LOCAL(take)(&cursor, s, 1);
    points = U_LOCAL(take)(&cursor, s, n);
    scratch = U_LOCAL(take)(&cursor, s, n);
    propagated = U_LOCAL(take)(&cursor, s, n);
    U_LOCAL(expand_weights)(weights, &wm, &wc);
    /* Each step borrows the two dead s-by-n blocks adjacent to its inputs. */
    sub = workspace;
    sub.capacity = 2 * s * n;
    sub.data = (U_WORK *)scratch.data;
    U_TRY(U_API(sigma_points)(mean, covariance, weights->scale, &points, sub));
    U_TRY(U_API(propagate)(&points, transition, transition_context, &propagated));
    sub.data = (U_WORK *)points.data;
    U_TRY(U_API(mean)(&propagated, &wm, residual, residual_context, out_mean, sub));
    U_TRY(U_API(deviations)(&propagated, out_mean, residual, residual_context, &points, workspace));
    sub.data = (U_WORK *)scratch.data;
    U_TRY(U_API(weighted_covariance)(&points, &points, &wc, out_covariance, sub));
    U_TRY(U_LINEAR(add)(out_covariance, process_noise, out_covariance, U_BASIC));
    return U_LOCAL(symmetrize)(out_covariance, &tmp, workspace);
}

linears_status_t U_API(update_work)(const linears_matrix_t *mean, const linears_matrix_t *covariance,
                                    const linears_matrix_t *measurement_noise,
                                    const linears_matrix_t *measurement, const U_WEIGHTS *weights,
                                    U_FN observation, void *observation_context,
                                    U_RESIDUAL_FN residual, void *residual_context,
                                    linears_matrix_t *out_mean, linears_matrix_t *out_covariance,
                                    U_WORKSPACE workspace)
{
    linears_status_t status;
    ukf_spans_t spans;
    linears_matrix_t tmp, wm, wc, innovation_covariance, factor, cross, cross_t, gain_t, gain;
    linears_matrix_t reduction, innovation, zbar, points, measured, dx, dz, scratch;
    linears_matrix_t z_row, innovation_row;
    U_WORKSPACE sub;
    U_WORK *cursor;
    size_t n, m, s, k, basic_bytes, decomposition_count;
    if (observation == NULL) return LINEARS_ERROR_ARGUMENT;
    U_TRY(U_LOCAL(weights_valid)(weights));
    spans.count = 0;
    U_TRY(ukf_matrix(&spans, mean, 0, 1, sizeof(U_WORK), U_ALIGNOF(U_WORK), UKF_INPUT));
    n = mean->rows;
    U_TRY(ukf_matrix(&spans, covariance, n, n, sizeof(U_WORK), U_ALIGNOF(U_WORK), UKF_INPUT));
    U_TRY(ukf_matrix(&spans, measurement_noise, 0, 0, sizeof(U_WORK), U_ALIGNOF(U_WORK), UKF_INPUT));
    m = measurement_noise->rows;
    if (measurement_noise->cols != m) return LINEARS_ERROR_DIMENSION;
    U_TRY(ukf_matrix(&spans, measurement, m, 1, sizeof(U_WORK), U_ALIGNOF(U_WORK), UKF_INPUT));
    U_TRY(ukf_matrix(&spans, out_mean, n, 1, sizeof(U_WORK), U_ALIGNOF(U_WORK), UKF_WRITABLE));
    U_TRY(ukf_matrix(&spans, out_covariance, n, n, sizeof(U_WORK), U_ALIGNOF(U_WORK), UKF_WRITABLE));
    s = FILTERS_UKF_SIGMA_COUNT(n);
    k = FILTERS_KALMAN_MAX_SIZE(n, m);
    basic_bytes = FILTERS_UKF_UPDATE_BASIC_BYTES(n, m);
    decomposition_count = FILTERS_UKF_UPDATE_DECOMP_STORAGE(n, m);
    U_TRY(U_LOCAL(workspace)(&spans, workspace, FILTERS_UKF_UPDATE_WORK_STORAGE(n, m), basic_bytes,
                             decomposition_count));
    U_CHECK_FINITE(mean);
    U_CHECK_FINITE(covariance);
    U_CHECK_FINITE(measurement_noise);
    U_CHECK_FINITE(measurement);
    U_TRY(U_SYMMETRIC(covariance, 1));
    U_TRY(U_SYMMETRIC(measurement_noise, 0));
    workspace.basic_bytes = basic_bytes;
    workspace.decomposition.capacity = decomposition_count;
    cursor = workspace.data;
    tmp = U_LOCAL(take)(&cursor, n, n);
    wm = U_LOCAL(take)(&cursor, s, 1);
    wc = U_LOCAL(take)(&cursor, s, 1);
    innovation_covariance = U_LOCAL(take)(&cursor, m, m);
    factor = U_LOCAL(take)(&cursor, m, m);
    cross = U_LOCAL(take)(&cursor, n, m);
    cross_t = U_LOCAL(take)(&cursor, m, n);
    gain_t = U_LOCAL(take)(&cursor, m, n);
    gain = U_LOCAL(take)(&cursor, n, m);
    reduction = U_LOCAL(take)(&cursor, n, n);
    innovation = U_LOCAL(take)(&cursor, m, 1);
    zbar = U_LOCAL(take)(&cursor, m, 1);
    points = U_LOCAL(take)(&cursor, s, n);
    measured = U_LOCAL(take)(&cursor, s, m);
    dx = U_LOCAL(take)(&cursor, s, n);
    dz = U_LOCAL(take)(&cursor, s, m);
    scratch = U_LOCAL(take)(&cursor, s, m + k);
    U_LOCAL(expand_weights)(weights, &wm, &wc);
    /* Validate R itself before any callback: S can be SPD when R is not. */
    U_TRY(U_LINEAR(cholesky)(measurement_noise, &factor, workspace.decomposition));
    sub = workspace;
    sub.data = (U_WORK *)measured.data;
    sub.capacity = (size_t)(((U_WORK *)scratch.data + s * (m + k)) - (U_WORK *)measured.data);
    U_TRY(U_API(sigma_points)(mean, covariance, weights->scale, &points, sub));
    U_TRY(U_API(propagate)(&points, observation, observation_context, &measured));
    sub.data = (U_WORK *)scratch.data;
    sub.capacity = s * (m + k);
    U_TRY(U_API(mean)(&measured, &wm, residual, residual_context, &zbar, sub));
    U_TRY(U_API(deviations)(&measured, &zbar, residual, residual_context, &dz, workspace));
    /* Regenerated sigma points differ from the mean by exactly +-L columns. */
    U_TRY(U_API(deviations)(&points, mean, NULL, NULL, &dx, workspace));
    U_TRY(U_API(weighted_covariance)(&dz, &dz, &wc, &innovation_covariance, sub));
    U_TRY(U_LINEAR(add)(&innovation_covariance, measurement_noise, &innovation_covariance, U_BASIC));
    U_TRY(U_LOCAL(symmetrize)(&innovation_covariance, &factor, workspace));
    U_TRY(U_LINEAR(cholesky)(&innovation_covariance, &factor, workspace.decomposition));
    U_TRY(U_API(weighted_covariance)(&dx, &dz, &wc, &cross, sub));
    U_TRY(U_LINEAR(transpose)(&cross, &cross_t, U_BASIC));
    /* S*K^T = C^T through the shared pivoted solve; no explicit inverse. */
    U_TRY(U_LINEAR(solve)(&innovation_covariance, &cross_t, &gain_t, workspace.decomposition));
    U_TRY(U_LINEAR(transpose)(&gain_t, &gain, U_BASIC));
    z_row = *measurement;
    z_row.rows = 1;
    z_row.cols = m;
    innovation_row = innovation;
    innovation_row.rows = 1;
    innovation_row.cols = m;
    U_TRY(U_API(deviations)(&z_row, &zbar, residual, residual_context, &innovation_row, workspace));
    U_TRY(U_LINEAR(mul)(&gain, &innovation, out_mean, U_BASIC));
    U_TRY(U_LINEAR(add)(mean, out_mean, out_mean, U_BASIC));
    /* P - K*S*K^T equals P - K*C^T because S*K^T = C^T. */
    U_TRY(U_LINEAR(mul)(&gain, &cross_t, &reduction, U_BASIC));
    U_TRY(U_LINEAR(sub)(covariance, &reduction, out_covariance, U_BASIC));
    return U_LOCAL(symmetrize)(out_covariance, &tmp, workspace);
}

#undef U_CHECK_FINITE
#undef U_ISFINITE
#undef U_BASIC_MEMBER
#undef U_BASIC_MEMBER_f64
#undef U_BASIC_MEMBER_f32
#undef U_BASIC
#undef U_TRY
#undef U_SYMMETRIC
#undef U_LINEAR
#undef U_LOCAL
#undef U_API
#undef U_JOIN
#undef U_JOIN_INNER
#undef U_TAG
#undef U_WORK
#undef U_WEIGHTS
#undef U_WORKSPACE
#undef U_FN
#undef U_RESIDUAL_FN
#undef U_FINITE
