/* Signal-format UKF filters; included by ukf.c once per filter tag. */
#define U_JOIN_INNER(a, b) a##b
#define U_JOIN(a, b) U_JOIN_INNER(a, b)
#define U_API(name) U_JOIN(U_JOIN(filters_ukf_, U_TAG), U_JOIN(_, name))
#define U_KERNEL_API(name) U_JOIN(U_JOIN(filters_ukf_, U_KERNEL), U_JOIN(_, name))
#define U_LOCAL(name) U_JOIN(U_JOIN(ukf_filter_, U_TAG), U_JOIN(_, name))
#define U_WORK_LOCAL(name) U_JOIN(U_JOIN(ukf_work_, U_KERNEL), U_JOIN(_, name))
#define U_LINEAR(name) U_JOIN(U_JOIN(linears_, name), U_JOIN(_, U_KERNEL))
#define U_TRY(expression) do { \
    status = (expression); \
    if (status != LINEARS_OK) return status; \
} while (0)
#define U_BASIC workspace.basic, workspace.basic_bytes
#define U_TAKE(rows, cols) U_WORK_LOCAL(take)(&cursor, (rows), (cols))
#if U_FIXED
#define U_CHECK_FINITE(matrix) ((void)0)
#define U_IMPORT_VIEW(matrix) U_LOCAL(import)((matrix), &cursor)
#else
#define U_CHECK_FINITE(matrix) U_TRY(U_FINITE(matrix))
#define U_IMPORT_VIEW(matrix) (*(matrix))
#endif

static linears_status_t U_LOCAL(symmetric)(const linears_matrix_t *matrix, int check_diagonal)
{
    size_t i, j, n = matrix->rows;
    const U_SIGNAL *values = (const U_SIGNAL *)matrix->data;
    for (i = 0; i < n; ++i) {
        if (check_diagonal && values[i * n + i] < 0) return LINEARS_ERROR_DOMAIN;
        for (j = 0; j < i; ++j)
            if (values[i * n + j] != values[j * n + i]) return LINEARS_ERROR_DOMAIN;
    }
    return LINEARS_OK;
}

static linears_status_t U_LOCAL(state)(ukf_spans_t *spans, const U_STATE *state,
                                       size_t *states, size_t *measurements)
{
    linears_status_t status;
    size_t n, m;
    U_TRY(ukf_span(spans, state, sizeof(*state), U_ALIGNOF(U_STATE), UKF_WRITABLE));
    n = state->estimate.rows;
    m = state->measurement_noise.rows;
    U_TRY(ukf_owned(spans, &state->estimate, n, 1, state->estimate_capacity,
                    sizeof(U_SIGNAL), U_ALIGNOF(U_SIGNAL)));
    U_TRY(ukf_owned(spans, &state->covariance, n, n, state->covariance_capacity,
                    sizeof(U_SIGNAL), U_ALIGNOF(U_SIGNAL)));
    U_TRY(ukf_owned(spans, &state->process_noise, n, n, state->process_noise_capacity,
                    sizeof(U_SIGNAL), U_ALIGNOF(U_SIGNAL)));
    U_TRY(ukf_owned(spans, &state->measurement_noise, m, m, state->measurement_noise_capacity,
                    sizeof(U_SIGNAL), U_ALIGNOF(U_SIGNAL)));
    *states = n;
    *measurements = m;
    return LINEARS_OK;
}

static linears_status_t U_LOCAL(weights)(const U_WEIGHTS *weights)
{
#if !U_FIXED
    if (!isfinite(weights->scale) || !isfinite(weights->mean0) ||
        !isfinite(weights->covariance0) || !isfinite(weights->other))
        return LINEARS_ERROR_RANGE;
#endif
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

/* Register an init source: its own descriptor/storage is not a second range. */
static linears_status_t U_LOCAL(source)(ukf_spans_t *spans, const linears_matrix_t *source,
                                        const linears_matrix_t *target)
{
    linears_status_t status;
    size_t bytes;
    if (source != target)
        U_TRY(ukf_span(spans, source, sizeof(*source), U_ALIGNOF(linears_matrix_t), UKF_DESCRIPTOR));
    U_TRY(ukf_shape(source, target->rows, target->cols, sizeof(U_SIGNAL), &bytes));
    if (source->data == target->data) return LINEARS_OK;
    return ukf_span(spans, source->data, bytes, U_ALIGNOF(U_SIGNAL), UKF_INPUT);
}

#if U_FIXED
static linears_matrix_t U_LOCAL(import)(const linears_matrix_t *source, U_WORK **cursor)
{
    size_t i, count = source->rows * source->cols;
    linears_matrix_t result = U_WORK_LOCAL(take)(cursor, source->rows, source->cols);
    const U_SIGNAL *input = (const U_SIGNAL *)source->data;
    U_WORK *output = (U_WORK *)result.data;
    for (i = 0; i < count; ++i) output[i] = U_IMPORT(input[i]);
    return result;
}
#endif

static linears_status_t U_LOCAL(symmetrize)(linears_matrix_t *matrix, linears_matrix_t *scratch,
                                            U_WORKSPACE workspace)
{
    linears_status_t status;
    U_TRY(U_LINEAR(transpose)(matrix, scratch, U_BASIC));
    return U_LINEAR(axpby)(U_HALF, matrix, U_HALF, scratch, matrix, U_BASIC);
}

static linears_status_t U_LOCAL(commit)(U_STATE *state, const linears_matrix_t *estimate,
                                        const linears_matrix_t *covariance,
                                        linears_workspace_unit_t *staging)
{
    size_t n = state->estimate.rows;
#if U_FIXED
    linears_status_t status;
    size_t i;
    const U_WORK *x = (const U_WORK *)estimate->data;
    const U_WORK *p = (const U_WORK *)covariance->data;
    U_SIGNAL *out_x = (U_SIGNAL *)state->estimate.data;
    U_SIGNAL *out_p = (U_SIGNAL *)state->covariance.data;
    /* Stage through declared union members so a late RANGE leaves state intact. */
    for (i = 0; i < n; ++i) U_TRY(U_EXPORT(x[i], &staging[i].U_PACK_FIELD));
    for (i = 0; i < n * n; ++i) U_TRY(U_EXPORT(p[i], &staging[n + i].U_PACK_FIELD));
    for (i = 0; i < n; ++i) out_x[i] = staging[i].U_PACK_FIELD;
    for (i = 0; i < n * n; ++i) out_p[i] = staging[n + i].U_PACK_FIELD;
#else
    (void)staging;
    memcpy(state->estimate.data, estimate->data, n * sizeof(U_SIGNAL));
    memcpy(state->covariance.data, covariance->data, n * n * sizeof(U_SIGNAL));
#endif
    return LINEARS_OK;
}

linears_status_t U_API(init)(U_STATE *state, U_WORK alpha, U_WORK beta, U_WORK kappa,
                             const linears_matrix_t *estimate, const linears_matrix_t *covariance,
                             const linears_matrix_t *process_noise,
                             const linears_matrix_t *measurement_noise)
{
    linears_status_t status;
    ukf_spans_t spans;
    U_WEIGHTS weights;
    const linears_matrix_t *sources[4];
    const linears_matrix_t *targets[4];
    size_t n, m, i;
    spans.count = 0;
    U_TRY(U_LOCAL(state)(&spans, state, &n, &m));
    sources[0] = estimate;
    sources[1] = covariance;
    sources[2] = process_noise;
    sources[3] = measurement_noise;
    targets[0] = &state->estimate;
    targets[1] = &state->covariance;
    targets[2] = &state->process_noise;
    targets[3] = &state->measurement_noise;
    for (i = 0; i < 4; ++i) U_TRY(U_LOCAL(source)(&spans, sources[i], targets[i]));
    U_TRY(ukf_disjoint(&spans));
    U_TRY(U_KERNEL_API(weights)(n, alpha, beta, kappa, &weights));
    for (i = 0; i < 4; ++i) U_CHECK_FINITE(sources[i]);
    U_TRY(U_LOCAL(symmetric)(covariance, 1));
    U_TRY(U_LOCAL(symmetric)(process_noise, 1));
    U_TRY(U_LOCAL(symmetric)(measurement_noise, 0));
    for (i = 0; i < 4; ++i) {
        if (sources[i]->data != targets[i]->data)
            memcpy(targets[i]->data, sources[i]->data,
                   sources[i]->rows * sources[i]->cols * sizeof(U_SIGNAL));
    }
    state->weights = weights;
    return LINEARS_OK;
}

linears_status_t U_API(predict)(U_STATE *state, U_FN transition, void *transition_context,
                                U_RESIDUAL_FN residual, void *residual_context,
                                U_WORKSPACE workspace)
{
    linears_status_t status;
    ukf_spans_t spans;
    linears_matrix_t xbar, pbar, tmp, wm, wc, points, scratch, propagated, x_in, p_in, q_in;
    U_WORKSPACE sub;
    U_WORK *cursor;
    size_t n, m, s, data_count, basic_bytes, decomposition_count;
    if (transition == NULL) return LINEARS_ERROR_ARGUMENT;
    spans.count = 0;
    U_TRY(U_LOCAL(state)(&spans, state, &n, &m));
    s = FILTERS_UKF_SIGMA_COUNT(n);
    data_count = U_PREDICT_STORAGE(n);
    basic_bytes = U_PREDICT_BASIC(n);
    decomposition_count = FILTERS_UKF_PREDICT_DECOMP_STORAGE(n);
    U_TRY(U_WORK_LOCAL(workspace)(&spans, workspace, data_count, basic_bytes, decomposition_count));
    U_TRY(U_LOCAL(weights)(&state->weights));
    U_CHECK_FINITE(&state->estimate);
    U_CHECK_FINITE(&state->covariance);
    U_CHECK_FINITE(&state->process_noise);
    U_TRY(U_LOCAL(symmetric)(&state->covariance, 1));
    U_TRY(U_LOCAL(symmetric)(&state->process_noise, 1));
    /* Pass only the used prefixes; unused supplied capacity stays untouched. */
    workspace.basic_bytes = basic_bytes;
    workspace.decomposition.capacity = decomposition_count;
    cursor = workspace.data;
    xbar = U_TAKE(n, 1);
    pbar = U_TAKE(n, n);
    tmp = U_TAKE(n, n);
    wm = U_TAKE(s, 1);
    wc = U_TAKE(s, 1);
    points = U_TAKE(s, n);
    scratch = U_TAKE(s, n);
    propagated = U_TAKE(s, n);
    x_in = U_IMPORT_VIEW(&state->estimate);
    p_in = U_IMPORT_VIEW(&state->covariance);
    q_in = U_IMPORT_VIEW(&state->process_noise);
    U_LOCAL(expand_weights)(&state->weights, &wm, &wc);
    /* Each step borrows the two dead s-by-n blocks adjacent to its inputs. */
    sub = workspace;
    sub.capacity = 2 * s * n;
    sub.data = (U_WORK *)scratch.data;
    U_TRY(U_KERNEL_API(sigma_points)(&x_in, &p_in, state->weights.scale, &points, sub));
    U_TRY(U_KERNEL_API(propagate)(&points, transition, transition_context, &propagated));
    sub.data = (U_WORK *)points.data;
    U_TRY(U_KERNEL_API(mean)(&propagated, &wm, residual, residual_context, &xbar, sub));
    U_TRY(U_KERNEL_API(deviations)(&propagated, &xbar, residual, residual_context, &points, workspace));
    sub.data = (U_WORK *)scratch.data;
    U_TRY(U_KERNEL_API(weighted_covariance)(&points, &points, &wc, &pbar, sub));
    U_TRY(U_LINEAR(add)(&pbar, &q_in, &pbar, U_BASIC));
    U_TRY(U_LOCAL(symmetrize)(&pbar, &tmp, workspace));
    return U_LOCAL(commit)(state, &xbar, &pbar, workspace.basic);
}

linears_status_t U_API(update)(U_STATE *state, const linears_matrix_t *measurement,
                               U_FN observation, void *observation_context,
                               U_RESIDUAL_FN residual, void *residual_context,
                               U_WORKSPACE workspace)
{
    linears_status_t status;
    ukf_spans_t spans;
    linears_matrix_t xbar, pbar, tmp, wm, wc, innovation_covariance, factor, cross, cross_t;
    linears_matrix_t gain_t, gain, reduction, innovation, zbar, points, measured, dx, dz, scratch;
    linears_matrix_t x_in, p_in, r_in, z_in, z_row, innovation_row;
    U_WORKSPACE sub;
    U_WORK *cursor;
    size_t n, m, s, k, data_count, basic_bytes, decomposition_count;
    if (observation == NULL) return LINEARS_ERROR_ARGUMENT;
    spans.count = 0;
    U_TRY(U_LOCAL(state)(&spans, state, &n, &m));
    U_TRY(ukf_matrix(&spans, measurement, m, 1, sizeof(U_SIGNAL), U_ALIGNOF(U_SIGNAL), UKF_INPUT));
    s = FILTERS_UKF_SIGMA_COUNT(n);
    k = FILTERS_KALMAN_MAX_SIZE(n, m);
    data_count = U_UPDATE_STORAGE(n, m);
    basic_bytes = U_UPDATE_BASIC(n, m);
    decomposition_count = FILTERS_UKF_UPDATE_DECOMP_STORAGE(n, m);
    U_TRY(U_WORK_LOCAL(workspace)(&spans, workspace, data_count, basic_bytes, decomposition_count));
    U_TRY(U_LOCAL(weights)(&state->weights));
    U_CHECK_FINITE(&state->estimate);
    U_CHECK_FINITE(&state->covariance);
    U_CHECK_FINITE(&state->measurement_noise);
    U_CHECK_FINITE(measurement);
    U_TRY(U_LOCAL(symmetric)(&state->covariance, 1));
    U_TRY(U_LOCAL(symmetric)(&state->measurement_noise, 0));
    workspace.basic_bytes = basic_bytes;
    workspace.decomposition.capacity = decomposition_count;
    cursor = workspace.data;
    xbar = U_TAKE(n, 1);
    pbar = U_TAKE(n, n);
    tmp = U_TAKE(n, n);
    wm = U_TAKE(s, 1);
    wc = U_TAKE(s, 1);
    innovation_covariance = U_TAKE(m, m);
    factor = U_TAKE(m, m);
    cross = U_TAKE(n, m);
    cross_t = U_TAKE(m, n);
    gain_t = U_TAKE(m, n);
    gain = U_TAKE(n, m);
    reduction = U_TAKE(n, n);
    innovation = U_TAKE(m, 1);
    zbar = U_TAKE(m, 1);
    points = U_TAKE(s, n);
    measured = U_TAKE(s, m);
    dx = U_TAKE(s, n);
    dz = U_TAKE(s, m);
    scratch = U_TAKE(s, m + k);
    x_in = U_IMPORT_VIEW(&state->estimate);
    p_in = U_IMPORT_VIEW(&state->covariance);
    r_in = U_IMPORT_VIEW(&state->measurement_noise);
    z_in = U_IMPORT_VIEW(measurement);
    U_LOCAL(expand_weights)(&state->weights, &wm, &wc);
    /* Validate R itself before any callback: S can be SPD when R is not. */
    U_TRY(U_LINEAR(cholesky)(&r_in, &factor, workspace.decomposition));
    sub = workspace;
    sub.data = (U_WORK *)measured.data;
    sub.capacity = (size_t)(((U_WORK *)scratch.data + s * (m + k)) - (U_WORK *)measured.data);
    U_TRY(U_KERNEL_API(sigma_points)(&x_in, &p_in, state->weights.scale, &points, sub));
    U_TRY(U_KERNEL_API(propagate)(&points, observation, observation_context, &measured));
    sub.data = (U_WORK *)scratch.data;
    sub.capacity = s * (m + k);
    U_TRY(U_KERNEL_API(mean)(&measured, &wm, residual, residual_context, &zbar, sub));
    U_TRY(U_KERNEL_API(deviations)(&measured, &zbar, residual, residual_context, &dz, workspace));
    /* Regenerated sigma points differ from x by exactly +-L columns. */
    U_TRY(U_KERNEL_API(deviations)(&points, &x_in, NULL, NULL, &dx, workspace));
    U_TRY(U_KERNEL_API(weighted_covariance)(&dz, &dz, &wc, &innovation_covariance, sub));
    U_TRY(U_LINEAR(add)(&innovation_covariance, &r_in, &innovation_covariance, U_BASIC));
    U_TRY(U_LOCAL(symmetrize)(&innovation_covariance, &factor, workspace));
    U_TRY(U_LINEAR(cholesky)(&innovation_covariance, &factor, workspace.decomposition));
    U_TRY(U_KERNEL_API(weighted_covariance)(&dx, &dz, &wc, &cross, sub));
    U_TRY(U_LINEAR(transpose)(&cross, &cross_t, U_BASIC));
    /* S*K^T = C^T through the shared pivoted solve; no explicit inverse. */
    U_TRY(U_LINEAR(solve)(&innovation_covariance, &cross_t, &gain_t, workspace.decomposition));
    U_TRY(U_LINEAR(transpose)(&gain_t, &gain, U_BASIC));
    z_row = z_in;
    z_row.rows = 1;
    z_row.cols = m;
    innovation_row = innovation;
    innovation_row.rows = 1;
    innovation_row.cols = m;
    U_TRY(U_KERNEL_API(deviations)(&z_row, &zbar, residual, residual_context, &innovation_row, workspace));
    U_TRY(U_LINEAR(mul)(&gain, &innovation, &xbar, U_BASIC));
    U_TRY(U_LINEAR(add)(&x_in, &xbar, &xbar, U_BASIC));
    /* P - K*S*K^T equals P - K*C^T because S*K^T = C^T. */
    U_TRY(U_LINEAR(mul)(&gain, &cross_t, &reduction, U_BASIC));
    U_TRY(U_LINEAR(sub)(&p_in, &reduction, &pbar, U_BASIC));
    U_TRY(U_LOCAL(symmetrize)(&pbar, &tmp, workspace));
    return U_LOCAL(commit)(state, &xbar, &pbar, workspace.basic);
}

#undef U_IMPORT_VIEW
#undef U_CHECK_FINITE
#undef U_TAKE
#undef U_BASIC
#undef U_TRY
#undef U_LINEAR
#undef U_WORK_LOCAL
#undef U_LOCAL
#undef U_KERNEL_API
#undef U_API
#undef U_JOIN
#undef U_JOIN_INNER
#undef U_TAG
#undef U_KERNEL
#undef U_SIGNAL
#undef U_WORK
#undef U_FIXED
#undef U_STATE
#undef U_WEIGHTS
#undef U_WORKSPACE
#undef U_FN
#undef U_RESIDUAL_FN
#undef U_HALF
#undef U_FINITE
#undef U_IMPORT
#undef U_EXPORT
#undef U_PACK_FIELD
#undef U_PREDICT_STORAGE
#undef U_PREDICT_BASIC
#undef U_UPDATE_STORAGE
#undef U_UPDATE_BASIC
