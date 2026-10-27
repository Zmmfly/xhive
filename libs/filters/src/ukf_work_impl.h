/* Working-format UKF kernels; included by ukf.c once per kernel tag. */
#define U_JOIN_INNER(a, b) a##b
#define U_JOIN(a, b) U_JOIN_INNER(a, b)
#define U_API(name) U_JOIN(U_JOIN(filters_ukf_, U_TAG), U_JOIN(_, name))
#define U_LOCAL(name) U_JOIN(U_JOIN(ukf_work_, U_TAG), U_JOIN(_, name))
#define U_LINEAR(name) U_JOIN(U_JOIN(linears_, name), U_JOIN(_, U_TAG))
#define U_TRY(expression) do { \
    status = (expression); \
    if (status != LINEARS_OK) return status; \
} while (0)
#define U_BASIC workspace.basic, workspace.basic_bytes
#if U_FIXED
#define U_CHECK_FINITE(matrix) ((void)0)
#else
#define U_CHECK_FINITE(matrix) U_TRY(U_FINITE(matrix))
#endif

static linears_matrix_t U_LOCAL(take)(U_WORK **cursor, size_t rows, size_t cols)
{
    linears_matrix_t result = {*cursor, rows, cols};
    *cursor += rows * cols;
    return result;
}

/* Row i of a row-major s-by-k matrix viewed as a contiguous k-by-1 vector. */
static linears_matrix_t U_LOCAL(row)(const linears_matrix_t *matrix, size_t index)
{
    linears_matrix_t result;
    result.data = (U_WORK *)matrix->data + index * matrix->cols;
    result.rows = matrix->cols;
    result.cols = 1;
    return result;
}

static linears_status_t U_LOCAL(workspace)(ukf_spans_t *spans, U_WORKSPACE workspace,
                                          size_t data_count, size_t basic_bytes,
                                          size_t decomposition_count)
{
    linears_status_t status;
    size_t data_bytes = LINEARS_SIZE_MUL(data_count, sizeof(U_WORK));
    size_t decomposition_bytes = LINEARS_SIZE_MUL(decomposition_count, sizeof(U_WORK));
    if (data_count == SIZE_MAX || data_bytes == SIZE_MAX || basic_bytes == SIZE_MAX ||
        decomposition_count == SIZE_MAX || decomposition_bytes == SIZE_MAX)
        return LINEARS_ERROR_SIZE;
    if (workspace.capacity < data_count || workspace.basic_bytes < basic_bytes ||
        workspace.decomposition.capacity < decomposition_count)
        return LINEARS_ERROR_WORKSPACE;
    U_TRY(ukf_scratch(spans, workspace.data, data_bytes, U_ALIGNOF(U_WORK)));
    U_TRY(ukf_scratch(spans, workspace.basic, basic_bytes, U_ALIGNOF(linears_workspace_unit_t)));
    U_TRY(ukf_scratch(spans, workspace.decomposition.data, decomposition_bytes, U_ALIGNOF(U_WORK)));
    return ukf_disjoint(spans);
}

linears_status_t U_API(weights)(size_t states, U_WORK alpha, U_WORK beta, U_WORK kappa,
                                U_WEIGHTS *out)
{
    U_WEIGHTS result;
#if U_FIXED
    linears_status_t status;
    linears_fixed_t n, alpha_squared, lambda;
#else
    U_WORK n = (U_WORK)states, alpha_squared;
#endif
    if (out == NULL) return LINEARS_ERROR_ARGUMENT;
    if (states == 0) return LINEARS_ERROR_DIMENSION;
#if U_FIXED
    if (alpha <= 0) return LINEARS_ERROR_DOMAIN;
    U_TRY(linears_fixed_from_int((int64_t)states, &n));
    U_TRY(linears_fixed_mul(alpha, alpha, &alpha_squared));
    U_TRY(linears_fixed_add(n, kappa, &result.scale));
    U_TRY(linears_fixed_mul(alpha_squared, result.scale, &result.scale));
    if (result.scale <= 0) return LINEARS_ERROR_DOMAIN;
    U_TRY(linears_fixed_sub(result.scale, n, &lambda));
    U_TRY(linears_fixed_div(lambda, result.scale, &result.mean0));
    U_TRY(linears_fixed_add(result.mean0, LINEARS_FIXED_ONE, &result.covariance0));
    U_TRY(linears_fixed_sub(result.covariance0, alpha_squared, &result.covariance0));
    U_TRY(linears_fixed_add(result.covariance0, beta, &result.covariance0));
    U_TRY(linears_fixed_div(LINEARS_FIXED_ONE / 2, result.scale, &result.other));
#else
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
#endif
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
#if !U_FIXED
    if (!isfinite(scale)) return LINEARS_ERROR_RANGE;
#endif
    if (!(scale > 0)) return LINEARS_ERROR_DOMAIN;
    workspace.basic_bytes = FILTERS_UKF_SIGMA_POINTS_BASIC_BYTES(n);
    workspace.decomposition.capacity = FILTERS_UKF_SIGMA_POINTS_DECOMP_STORAGE(n);
    cursor = workspace.data;
    scaled = U_LOCAL(take)(&cursor, n, n);
    factor = U_LOCAL(take)(&cursor, n, n);
    U_TRY(U_LINEAR(axpby)(scale, covariance, U_ZERO, covariance, &scaled, U_BASIC));
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
    linears_matrix_t deviations, transposed, reference;
    U_WORK *cursor;
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
    workspace.basic_bytes = FILTERS_UKF_MEAN_BASIC_BYTES(s, k);
    cursor = workspace.data;
    if (residual == NULL) {
        transposed = U_LOCAL(take)(&cursor, k, s);
        U_TRY(U_LINEAR(transpose)(points, &transposed, U_BASIC));
        return U_LINEAR(mul)(&transposed, weights, out, U_BASIC);
    }
    /* Average residuals about row 0, then move back; exact for linear parts. */
    deviations = U_LOCAL(take)(&cursor, s, k);
    transposed = U_LOCAL(take)(&cursor, k, s);
    reference = U_LOCAL(row)(points, 0);
    U_TRY(U_API(deviations)(points, &reference, residual, context, &deviations, workspace));
    U_TRY(U_LINEAR(transpose)(&deviations, &transposed, U_BASIC));
    U_TRY(U_LINEAR(mul)(&transposed, weights, out, U_BASIC));
    return U_LINEAR(add)(&reference, out, out, U_BASIC);
}

linears_status_t U_API(weighted_covariance)(const linears_matrix_t *a, const linears_matrix_t *b,
                                            const linears_matrix_t *weights, linears_matrix_t *out,
                                            U_WORKSPACE workspace)
{
    linears_status_t status;
    ukf_spans_t spans;
    linears_matrix_t scaled, transposed, source, target;
    const U_WORK *w;
    U_WORK *cursor;
    size_t s, p, q, i;
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
    workspace.basic_bytes = FILTERS_UKF_WEIGHTED_COVARIANCE_BASIC_BYTES(s, p, q);
    cursor = workspace.data;
    scaled = U_LOCAL(take)(&cursor, s, q);
    transposed = U_LOCAL(take)(&cursor, p, s);
    w = (const U_WORK *)weights->data;
    for (i = 0; i < s; ++i) {
        source = U_LOCAL(row)(b, i);
        target = U_LOCAL(row)(&scaled, i);
        U_TRY(U_LINEAR(axpby)(w[i], &source, U_ZERO, &source, &target, U_BASIC));
    }
    U_TRY(U_LINEAR(transpose)(a, &transposed, U_BASIC));
    return U_LINEAR(mul)(&transposed, &scaled, out, U_BASIC);
}

#undef U_CHECK_FINITE
#undef U_BASIC
#undef U_TRY
#undef U_LINEAR
#undef U_LOCAL
#undef U_API
#undef U_JOIN
#undef U_JOIN_INNER
#undef U_TAG
#undef U_WORK
#undef U_FIXED
#undef U_WEIGHTS
#undef U_WORKSPACE
#undef U_FN
#undef U_RESIDUAL_FN
#undef U_ZERO
#undef U_FINITE
