/* Working-format UKF kernels; included by ukf.c once per kernel tag. */
#include "ukf_scratch_impl.h"

#define U_JOIN_INNER(a, b) a##b
#define U_JOIN(a, b) U_JOIN_INNER(a, b)
#define U_API(name) U_JOIN(U_JOIN(filters_ukf_, U_TAG), U_JOIN(_, name))
#define U_LOCAL(name) U_JOIN(U_JOIN(ukf_work_, U_TAG), U_JOIN(_, name))
#define U_LINEAR(name) U_JOIN(U_JOIN(linears_, name), U_JOIN(_, U_TAG))
#define U_SYMMETRIC U_JOIN(ukf_symmetric_, U_TAG)
#define U_CHOLESKY U_JOIN(ukf_cholesky_, U_TAG)
#define U_WEIGHTS_VALID U_JOIN(ukf_weights_valid_, U_TAG)
#define U_TRY(expression) do { \
    status = (expression); \
    if (status != LINEARS_OK) return status; \
} while (0)
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

/* The cores below compute on views their caller has validated: exact shapes,
 * finite values, exactly symmetric covariances, valid weights and disjoint
 * scratch. They keep the per-element RANGE checks of the former basic matrix
 * calls and stage each result row in the basic scratch before committing it,
 * but register no spans and rescan no input. Each public kernel is its
 * validation followed by one core; the UPF calls the cores directly. */

/* Stage k finite values in basic[0..k) and copy them to target: a row is
 * committed only after every element has passed, as the basic kernels did. */
static void U_LOCAL(commit)(const linears_workspace_unit_t *basic, size_t k, U_WORK *target)
{
    size_t j;
    for (j = 0; j < k; ++j) target[j] = basic[j].U_BASIC_MEMBER;
}

/* scaled = scale*covariance. The former axpby computed scale*p + 0*p; for
 * finite p and a finite positive scale the second term is a zero carrying the
 * first term's sign, so the sum, its sign and its overflow are those of
 * scale*p alone. */
static linears_status_t U_LOCAL(scale_core)(const linears_matrix_t *covariance, U_WORK scale, U_WORK *scaled)
{
    const U_WORK *p = (const U_WORK *)covariance->data;
    size_t i, count = covariance->rows * covariance->cols;
    for (i = 0; i < count; ++i) {
        U_WORK value = scale * p[i];
        if (!U_ISFINITE(value)) return LINEARS_ERROR_RANGE;
        scaled[i] = value;
    }
    return LINEARS_OK;
}

/* Sigma points of mean and the scaled covariance: row 0 the mean, rows 1..n
 * mean + L column, rows n+1..2n mean - L column, L*L^T = scaled. factor is
 * an n*n scratch block. */
static linears_status_t U_LOCAL(sigma_core)(const linears_matrix_t *mean, const U_WORK *scaled,
                                            linears_matrix_t *points, U_WORK *factor,
                                            linears_workspace_unit_t *basic)
{
    linears_status_t status;
    const U_WORK *center = (const U_WORK *)mean->data;
    U_WORK *rows = (U_WORK *)points->data;
    size_t n = mean->rows, i, j;
    U_TRY(U_CHOLESKY(scaled, n, factor));
    memcpy(rows, center, n * sizeof(U_WORK));
    for (i = 0; i < n; ++i) {
        for (j = 0; j < n; ++j) {
            U_WORK value = center[j] + factor[j * n + i];
            if (!U_ISFINITE(value)) return LINEARS_ERROR_RANGE;
            basic[j].U_BASIC_MEMBER = value;
        }
        U_LOCAL(commit)(basic, n, rows + (1 + i) * n);
        for (j = 0; j < n; ++j) {
            U_WORK value = center[j] - factor[j * n + i];
            if (!U_ISFINITE(value)) return LINEARS_ERROR_RANGE;
            basic[j].U_BASIC_MEMBER = value;
        }
        U_LOCAL(commit)(basic, n, rows + (1 + n + i) * n);
    }
    return LINEARS_OK;
}

static linears_status_t U_LOCAL(propagate_core)(const linears_matrix_t *points, U_FN fn, void *context,
                                                linears_matrix_t *out)
{
    linears_status_t status;
    linears_matrix_t in, result;
    size_t i;
    for (i = 0; i < points->rows; ++i) {
        in = U_LOCAL(row)(points, i);
        result = U_LOCAL(row)(out, i);
        U_TRY(fn(&in, &result, context));
        U_CHECK_FINITE(&result);
    }
    return LINEARS_OK;
}

static linears_status_t U_LOCAL(deviations_core)(const linears_matrix_t *points, const linears_matrix_t *mean,
                                                 U_RESIDUAL_FN residual, void *context, linears_matrix_t *out,
                                                 linears_workspace_unit_t *basic)
{
    linears_status_t status;
    linears_matrix_t in, result;
    const U_WORK *center = (const U_WORK *)mean->data;
    size_t i, j, k = points->cols;
    for (i = 0; i < points->rows; ++i) {
        in = U_LOCAL(row)(points, i);
        result = U_LOCAL(row)(out, i);
        if (residual == NULL) {
            const U_WORK *point = (const U_WORK *)in.data;
            for (j = 0; j < k; ++j) {
                U_WORK value = point[j] - center[j];
                if (!U_ISFINITE(value)) return LINEARS_ERROR_RANGE;
                basic[j].U_BASIC_MEMBER = value;
            }
            U_LOCAL(commit)(basic, k, (U_WORK *)result.data);
        } else {
            U_TRY(residual(&in, mean, &result, context));
            U_CHECK_FINITE(&result);
        }
    }
    return LINEARS_OK;
}

/* Weighted mean; with a residual, averages residuals about row 0 in the
 * s-by-k deviation scratch and moves back, which is exact for linear parts. */
static linears_status_t U_LOCAL(mean_core)(const linears_matrix_t *points, const U_WORK *w,
                                           U_RESIDUAL_FN residual, void *context, linears_matrix_t *out,
                                           U_WORK *deviation_scratch, linears_workspace_unit_t *basic)
{
    linears_status_t status;
    linears_matrix_t deviations, reference;
    const linears_matrix_t *source = points;
    const U_WORK *values;
    U_WORK *result = (U_WORK *)out->data;
    size_t s = points->rows, k = points->cols, i, j;
    if (residual != NULL) {
        deviations.data = deviation_scratch;
        deviations.rows = s;
        deviations.cols = k;
        reference = U_LOCAL(row)(points, 0);
        U_TRY(U_LOCAL(deviations_core)(points, &reference, residual, context, &deviations, basic));
        source = &deviations;
    }
    values = (const U_WORK *)source->data;
    /* Read columns directly, retaining the transposed product's reduction order. */
    for (j = 0; j < k; ++j) {
        U_WORK sum = (U_WORK)0;
        for (i = 0; i < s; ++i) {
            U_WORK product = values[i * k + j] * w[i];
            sum += product;
            if (!U_ISFINITE(sum)) return LINEARS_ERROR_RANGE;
        }
        basic[j].U_BASIC_MEMBER = sum;
    }
    U_LOCAL(commit)(basic, k, result);
    if (residual != NULL) {
        const U_WORK *center = (const U_WORK *)reference.data;
        for (j = 0; j < k; ++j) {
            U_WORK value = center[j] + result[j];
            if (!U_ISFINITE(value)) return LINEARS_ERROR_RANGE;
            basic[j].U_BASIC_MEMBER = value;
        }
        U_LOCAL(commit)(basic, k, result);
    }
    return LINEARS_OK;
}

/* out = a^T * diag(w) * b through the s-by-q scaled scratch. With a staging
 * array the result is committed only after every sum is finite, as the
 * public kernel and the predicted covariance always were; without one each
 * sum is written as it completes, which the update uses for its S and C
 * scratch. The staged form needs p*q units, which the update storage macro,
 * max(n,m)*(2n+1) units, does not cover for S when m exceeds 2n+1. */
static linears_status_t U_LOCAL(weighted_covariance_core)(const linears_matrix_t *a, const linears_matrix_t *b,
                                                          const U_WORK *w, linears_matrix_t *out, U_WORK *scaled,
                                                          linears_workspace_unit_t *basic)
{
    const U_WORK *av = (const U_WORK *)a->data, *bv = (const U_WORK *)b->data;
    U_WORK *result = (U_WORK *)out->data;
    size_t s = a->rows, p = a->cols, q = b->cols, i, j, k;
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
            if (basic != NULL) basic[i * q + j].U_BASIC_MEMBER = sum;
            else result[i * q + j] = sum;
        }
    }
    if (basic != NULL) U_LOCAL(commit)(basic, p * q, result);
    return LINEARS_OK;
}

/* out = out + addend elementwise; a public output is committed through the
 * basic staging after every element is finite, scratch is written directly. */
static linears_status_t U_LOCAL(add_core)(linears_matrix_t *out, const linears_matrix_t *addend,
                                          linears_workspace_unit_t *basic)
{
    U_WORK *target = (U_WORK *)out->data;
    const U_WORK *source = (const U_WORK *)addend->data;
    size_t i, count = out->rows * out->cols;
    for (i = 0; i < count; ++i) {
        U_WORK value = target[i] + source[i];
        if (!U_ISFINITE(value)) return LINEARS_ERROR_RANGE;
        if (basic != NULL) basic[i].U_BASIC_MEMBER = value;
        else target[i] = value;
    }
    if (basic != NULL) U_LOCAL(commit)(basic, count, target);
    return LINEARS_OK;
}

/* 0.5*M + 0.5*M^T in place. Both mirrored sums are the same two rounded
 * products added in commuted order, so they are bitwise equal and finite. */
static void U_LOCAL(symmetrize)(linears_matrix_t *matrix)
{
    U_WORK *m = (U_WORK *)matrix->data;
    size_t n = matrix->rows, i, j;
    for (i = 0; i < n; ++i) {
        for (j = 0; j < i; ++j) {
            U_WORK left = (U_WORK)0.5 * m[i * n + j], right = (U_WORK)0.5 * m[j * n + i];
            m[i * n + j] = left + right;
            m[j * n + i] = right + left;
        }
        m[i * n + i] = (U_WORK)0.5 * m[i * n + i] + (U_WORK)0.5 * m[i * n + i];
    }
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

linears_status_t U_API(sigma_points)(const linears_matrix_t *mean, const linears_matrix_t *covariance,
                                     U_WORK scale, linears_matrix_t *points, U_WORKSPACE workspace)
{
    linears_status_t status;
    ukf_spans_t spans;
    linears_matrix_t scaled, factor;
    U_WORK *cursor;
    size_t n, count;
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
    cursor = workspace.data;
    scaled = U_LOCAL(take)(&cursor, n, n);
    factor = U_LOCAL(take)(&cursor, n, n);
    U_TRY(U_LOCAL(scale_core)(covariance, scale, (U_WORK *)scaled.data));
    /* The shared decomposition reported an asymmetric input as DIMENSION
     * after loading it, so the check applies to the scaled copy it loaded:
     * mirrored elements that differ only below the scaled rounding threshold
     * still pass. The private factor assumes symmetry. */
    if (U_SYMMETRIC(&scaled, 0) != LINEARS_OK) return LINEARS_ERROR_DIMENSION;
    return U_LOCAL(sigma_core)(mean, (const U_WORK *)scaled.data, points, (U_WORK *)factor.data,
                               workspace.basic);
}

linears_status_t U_API(propagate)(const linears_matrix_t *points, U_FN fn, void *context,
                                  linears_matrix_t *out)
{
    linears_status_t status;
    ukf_spans_t spans;
    if (fn == NULL) return LINEARS_ERROR_ARGUMENT;
    spans.count = 0;
    U_TRY(ukf_matrix(&spans, points, 0, 0, sizeof(U_WORK), U_ALIGNOF(U_WORK), UKF_INPUT));
    U_TRY(ukf_matrix(&spans, out, points->rows, 0, sizeof(U_WORK), U_ALIGNOF(U_WORK), UKF_WRITABLE));
    U_TRY(ukf_disjoint(&spans));
    U_CHECK_FINITE(points);
    return U_LOCAL(propagate_core)(points, fn, context, out);
}

linears_status_t U_API(deviations)(const linears_matrix_t *points, const linears_matrix_t *mean,
                                   U_RESIDUAL_FN residual, void *context, linears_matrix_t *out,
                                   U_WORKSPACE workspace)
{
    linears_status_t status;
    ukf_spans_t spans;
    size_t k;
    spans.count = 0;
    U_TRY(ukf_matrix(&spans, points, 0, 0, sizeof(U_WORK), U_ALIGNOF(U_WORK), UKF_INPUT));
    k = points->cols;
    U_TRY(ukf_matrix(&spans, mean, k, 1, sizeof(U_WORK), U_ALIGNOF(U_WORK), UKF_INPUT));
    U_TRY(ukf_matrix(&spans, out, points->rows, k, sizeof(U_WORK), U_ALIGNOF(U_WORK), UKF_WRITABLE));
    U_TRY(U_LOCAL(workspace)(&spans, workspace, 0, FILTERS_UKF_DEVIATIONS_BASIC_BYTES(k), 0));
    U_CHECK_FINITE(points);
    U_CHECK_FINITE(mean);
    return U_LOCAL(deviations_core)(points, mean, residual, context, out, workspace.basic);
}

linears_status_t U_API(mean)(const linears_matrix_t *points, const linears_matrix_t *weights,
                             U_RESIDUAL_FN residual, void *context, linears_matrix_t *out,
                             U_WORKSPACE workspace)
{
    linears_status_t status;
    ukf_spans_t spans;
    size_t s, k;
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
    return U_LOCAL(mean_core)(points, (const U_WORK *)weights->data, residual, context, out,
                              workspace.data, workspace.basic);
}

linears_status_t U_API(weighted_covariance)(const linears_matrix_t *a, const linears_matrix_t *b,
                                            const linears_matrix_t *weights, linears_matrix_t *out,
                                            U_WORKSPACE workspace)
{
    linears_status_t status;
    ukf_spans_t spans;
    size_t s, p, q;
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
    return U_LOCAL(weighted_covariance_core)(a, b, (const U_WORK *)weights->data, out, workspace.data,
                                             workspace.basic);
}

linears_status_t U_LOCAL(predict_core)(const linears_matrix_t *mean, const linears_matrix_t *covariance,
                                       const linears_matrix_t *process_noise, const U_WEIGHTS *weights,
                                       U_FN transition, void *transition_context,
                                       U_RESIDUAL_FN residual, void *residual_context,
                                       linears_matrix_t *out_mean, linears_matrix_t *out_covariance,
                                       U_WORKSPACE workspace)
{
    linears_status_t status;
    linears_matrix_t tmp, wm, wc, points, scratch, propagated;
    U_WORK *cursor;
    size_t n = mean->rows, s = FILTERS_UKF_SIGMA_COUNT(n);
    cursor = workspace.data;
    tmp = U_LOCAL(take)(&cursor, n, n);
    wm = U_LOCAL(take)(&cursor, s, 1);
    wc = U_LOCAL(take)(&cursor, s, 1);
    points = U_LOCAL(take)(&cursor, s, n);
    scratch = U_LOCAL(take)(&cursor, s, n);
    propagated = U_LOCAL(take)(&cursor, s, n);
    (void)tmp;
    U_LOCAL(expand_weights)(weights, &wm, &wc);
    /* Each step borrows the dead s-by-n blocks adjacent to its inputs. */
    U_TRY(U_LOCAL(scale_core)(covariance, weights->scale, (U_WORK *)scratch.data));
    U_TRY(U_LOCAL(sigma_core)(mean, (const U_WORK *)scratch.data, &points, (U_WORK *)scratch.data + n * n,
                              workspace.basic));
    U_TRY(U_LOCAL(propagate_core)(&points, transition, transition_context, &propagated));
    U_TRY(U_LOCAL(mean_core)(&propagated, (const U_WORK *)wm.data, residual, residual_context, out_mean,
                             (U_WORK *)points.data, workspace.basic));
    U_TRY(U_LOCAL(deviations_core)(&propagated, out_mean, residual, residual_context, &points,
                                   workspace.basic));
    U_TRY(U_LOCAL(weighted_covariance_core)(&points, &points, (const U_WORK *)wc.data, out_covariance,
                                            (U_WORK *)scratch.data, workspace.basic));
    U_TRY(U_LOCAL(add_core)(out_covariance, process_noise, workspace.basic));
    U_LOCAL(symmetrize)(out_covariance);
    return LINEARS_OK;
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
    size_t n, basic_bytes, decomposition_count;
    if (transition == NULL) return LINEARS_ERROR_ARGUMENT;
    U_TRY(U_WEIGHTS_VALID(weights));
    spans.count = 0;
    U_TRY(ukf_matrix(&spans, mean, 0, 1, sizeof(U_WORK), U_ALIGNOF(U_WORK), UKF_INPUT));
    n = mean->rows;
    U_TRY(ukf_matrix(&spans, covariance, n, n, sizeof(U_WORK), U_ALIGNOF(U_WORK), UKF_INPUT));
    U_TRY(ukf_matrix(&spans, process_noise, n, n, sizeof(U_WORK), U_ALIGNOF(U_WORK), UKF_INPUT));
    U_TRY(ukf_matrix(&spans, out_mean, n, 1, sizeof(U_WORK), U_ALIGNOF(U_WORK), UKF_WRITABLE));
    U_TRY(ukf_matrix(&spans, out_covariance, n, n, sizeof(U_WORK), U_ALIGNOF(U_WORK), UKF_WRITABLE));
    basic_bytes = FILTERS_UKF_PREDICT_BASIC_BYTES(n);
    decomposition_count = FILTERS_UKF_PREDICT_DECOMP_STORAGE(n);
    U_TRY(U_LOCAL(workspace)(&spans, workspace, FILTERS_UKF_PREDICT_WORK_STORAGE(n), basic_bytes,
                             decomposition_count));
    U_CHECK_FINITE(mean);
    U_CHECK_FINITE(covariance);
    U_CHECK_FINITE(process_noise);
    U_TRY(U_SYMMETRIC(covariance, 1));
    U_TRY(U_SYMMETRIC(process_noise, 1));
    return U_LOCAL(predict_core)(mean, covariance, process_noise, weights, transition, transition_context,
                                 residual, residual_context, out_mean, out_covariance, workspace);
}

linears_status_t U_LOCAL(update_core)(const linears_matrix_t *mean, const linears_matrix_t *covariance,
                                      const linears_matrix_t *measurement_noise,
                                      const linears_matrix_t *measurement, const U_WEIGHTS *weights,
                                      U_FN observation, void *observation_context,
                                      U_RESIDUAL_FN residual, void *residual_context,
                                      linears_matrix_t *out_mean, linears_matrix_t *out_covariance,
                                      U_WORKSPACE workspace)
{
    linears_status_t status;
    linears_matrix_t tmp, wm, wc, innovation_covariance, factor, cross, cross_t, gain_t, gain;
    linears_matrix_t reduction, innovation, zbar, points, measured, dx, dz, scratch;
    linears_matrix_t z_row, innovation_row;
    const U_WORK *x, *p, *kt, *ct, *dy;
    U_WORK *cursor, *xout, *pout, *red, *c;
    size_t n = mean->rows, m = measurement_noise->rows, s = FILTERS_UKF_SIGMA_COUNT(n);
    size_t k = FILTERS_KALMAN_MAX_SIZE(n, m), i, j, r;
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
    (void)tmp;
    (void)gain;
    U_LOCAL(expand_weights)(weights, &wm, &wc);
    /* The sigma factor borrows the dead sample blocks that follow points. */
    U_TRY(U_LOCAL(scale_core)(covariance, weights->scale, (U_WORK *)measured.data));
    U_TRY(U_LOCAL(sigma_core)(mean, (const U_WORK *)measured.data, &points, (U_WORK *)measured.data + n * n,
                              workspace.basic));
    U_TRY(U_LOCAL(propagate_core)(&points, observation, observation_context, &measured));
    U_TRY(U_LOCAL(mean_core)(&measured, (const U_WORK *)wm.data, residual, residual_context, &zbar,
                             (U_WORK *)scratch.data, workspace.basic));
    U_TRY(U_LOCAL(deviations_core)(&measured, &zbar, residual, residual_context, &dz, workspace.basic));
    /* Regenerated sigma points differ from the mean by exactly +-L columns. */
    U_TRY(U_LOCAL(deviations_core)(&points, mean, NULL, NULL, &dx, workspace.basic));
    U_TRY(U_LOCAL(weighted_covariance_core)(&dz, &dz, (const U_WORK *)wc.data, &innovation_covariance,
                                            (U_WORK *)scratch.data, NULL));
    U_TRY(U_LOCAL(add_core)(&innovation_covariance, measurement_noise, NULL));
    U_LOCAL(symmetrize)(&innovation_covariance);
    U_TRY(U_CHOLESKY((const U_WORK *)innovation_covariance.data, m, (U_WORK *)factor.data));
    U_TRY(U_LOCAL(weighted_covariance_core)(&dx, &dz, (const U_WORK *)wc.data, &cross,
                                            (U_WORK *)scratch.data, NULL));
    c = (U_WORK *)cross.data;
    ct = (const U_WORK *)cross_t.data;
    for (i = 0; i < n; ++i)
        for (j = 0; j < m; ++j) ((U_WORK *)cross_t.data)[j * n + i] = c[i * m + j];
    /* S*K^T = C^T through the shared pivoted solve; no explicit inverse. */
    U_TRY(U_LINEAR(solve)(&innovation_covariance, &cross_t, &gain_t, workspace.decomposition));
    z_row = *measurement;
    z_row.rows = 1;
    z_row.cols = m;
    innovation_row = innovation;
    innovation_row.rows = 1;
    innovation_row.cols = m;
    U_TRY(U_LOCAL(deviations_core)(&z_row, &zbar, residual, residual_context, &innovation_row,
                                   workspace.basic));
    /* K is read as the transpose of the solved m-by-n block. */
    kt = (const U_WORK *)gain_t.data;
    dy = (const U_WORK *)innovation.data;
    x = (const U_WORK *)mean->data;
    p = (const U_WORK *)covariance->data;
    xout = (U_WORK *)out_mean->data;
    pout = (U_WORK *)out_covariance->data;
    red = (U_WORK *)reduction.data;
    for (i = 0; i < n; ++i) {
        U_WORK sum = (U_WORK)0;
        for (j = 0; j < m; ++j) {
            U_WORK product = kt[j * n + i] * dy[j];
            sum += product;
            if (!U_ISFINITE(sum)) return LINEARS_ERROR_RANGE;
        }
        workspace.basic[i].U_BASIC_MEMBER = sum;
    }
    for (i = 0; i < n; ++i) {
        U_WORK value = x[i] + workspace.basic[i].U_BASIC_MEMBER;
        if (!U_ISFINITE(value)) return LINEARS_ERROR_RANGE;
        workspace.basic[i].U_BASIC_MEMBER = value;
    }
    U_LOCAL(commit)(workspace.basic, n, xout);
    /* P - K*S*K^T equals P - K*C^T because S*K^T = C^T. */
    for (i = 0; i < n; ++i) {
        for (r = 0; r < n; ++r) {
            U_WORK sum = (U_WORK)0;
            for (j = 0; j < m; ++j) {
                U_WORK product = kt[j * n + i] * ct[j * n + r];
                sum += product;
                if (!U_ISFINITE(sum)) return LINEARS_ERROR_RANGE;
            }
            red[i * n + r] = sum;
        }
    }
    for (i = 0; i < n * n; ++i) {
        U_WORK value = p[i] - red[i];
        if (!U_ISFINITE(value)) return LINEARS_ERROR_RANGE;
        workspace.basic[i].U_BASIC_MEMBER = value;
    }
    U_LOCAL(commit)(workspace.basic, n * n, pout);
    U_LOCAL(symmetrize)(out_covariance);
    return LINEARS_OK;
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
    U_WORK *factor;
    size_t n, m, s, basic_bytes, decomposition_count;
    if (observation == NULL) return LINEARS_ERROR_ARGUMENT;
    U_TRY(U_WEIGHTS_VALID(weights));
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
    /* Pass only the used solve prefix; unused supplied capacity stays untouched. */
    workspace.decomposition.capacity = decomposition_count;
    /* Validate R itself before any callback: S can be SPD when R is not. The
     * factor lands in the core's S-factor slot (after tmp, wm, wc and S) and
     * is overwritten there. */
    factor = workspace.data + n * n + 2 * s + m * m;
    U_TRY(U_CHOLESKY((const U_WORK *)measurement_noise->data, m, factor));
    return U_LOCAL(update_core)(mean, covariance, measurement_noise, measurement, weights, observation,
                                observation_context, residual, residual_context, out_mean, out_covariance,
                                workspace);
}

#undef U_CHECK_FINITE
#undef U_ISFINITE
#undef U_BASIC_MEMBER
#undef U_BASIC_MEMBER_f64
#undef U_BASIC_MEMBER_f32
#undef U_TRY
#undef U_WEIGHTS_VALID
#undef U_CHOLESKY
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
