/* Signal-format UKF filters; included by ukf.c once per filter tag. */
#define U_JOIN_INNER(a, b) a##b
#define U_JOIN(a, b) U_JOIN_INNER(a, b)
#define U_API(name) U_JOIN(U_JOIN(filters_ukf_, U_TAG), U_JOIN(_, name))
#define U_KERNEL_API(name) U_JOIN(U_JOIN(filters_ukf_, U_KERNEL), U_JOIN(_, name))
#define U_LOCAL(name) U_JOIN(U_JOIN(ukf_filter_, U_TAG), U_JOIN(_, name))
#define U_WORK_LOCAL(name) U_JOIN(U_JOIN(ukf_work_, U_KERNEL), U_JOIN(_, name))
#define U_TRY(expression) do { \
    status = (expression); \
    if (status != LINEARS_OK) return status; \
} while (0)
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

/* The kernel borrows everything after the filter's own staging and imports. */
static U_WORKSPACE U_LOCAL(remaining)(U_WORKSPACE workspace, const U_WORK *cursor)
{
    workspace.capacity -= (size_t)(cursor - (const U_WORK *)workspace.data);
    workspace.data = (U_WORK *)cursor;
    return workspace;
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
    linears_matrix_t xbar, pbar, x_in, p_in, q_in;
    U_WORK *cursor;
    size_t n, m;
    spans.count = 0;
    U_TRY(U_LOCAL(state)(&spans, state, &n, &m));
    U_TRY(U_WORK_LOCAL(workspace)(&spans, workspace, U_PREDICT_STORAGE(n), U_PREDICT_BASIC(n),
                                  FILTERS_UKF_PREDICT_DECOMP_STORAGE(n)));
    cursor = workspace.data;
    xbar = U_TAKE(n, 1);
    pbar = U_TAKE(n, n);
    x_in = U_IMPORT_VIEW(&state->estimate);
    p_in = U_IMPORT_VIEW(&state->covariance);
    q_in = U_IMPORT_VIEW(&state->process_noise);
    U_TRY(U_KERNEL_API(predict_work)(&x_in, &p_in, &q_in, &state->weights, transition,
                                     transition_context, residual, residual_context, &xbar, &pbar,
                                     U_LOCAL(remaining)(workspace, cursor)));
    return U_LOCAL(commit)(state, &xbar, &pbar, workspace.basic);
}

linears_status_t U_API(update)(U_STATE *state, const linears_matrix_t *measurement,
                               U_FN observation, void *observation_context,
                               U_RESIDUAL_FN residual, void *residual_context,
                               U_WORKSPACE workspace)
{
    linears_status_t status;
    ukf_spans_t spans;
    linears_matrix_t xbar, pbar, x_in, p_in, r_in, z_in;
    U_WORK *cursor;
    size_t n, m;
    spans.count = 0;
    U_TRY(U_LOCAL(state)(&spans, state, &n, &m));
    U_TRY(ukf_matrix(&spans, measurement, m, 1, sizeof(U_SIGNAL), U_ALIGNOF(U_SIGNAL), UKF_INPUT));
    U_TRY(U_WORK_LOCAL(workspace)(&spans, workspace, U_UPDATE_STORAGE(n, m), U_UPDATE_BASIC(n, m),
                                  FILTERS_UKF_UPDATE_DECOMP_STORAGE(n, m)));
    cursor = workspace.data;
    xbar = U_TAKE(n, 1);
    pbar = U_TAKE(n, n);
    x_in = U_IMPORT_VIEW(&state->estimate);
    p_in = U_IMPORT_VIEW(&state->covariance);
    r_in = U_IMPORT_VIEW(&state->measurement_noise);
    z_in = U_IMPORT_VIEW(measurement);
    U_TRY(U_KERNEL_API(update_work)(&x_in, &p_in, &r_in, &z_in, &state->weights, observation,
                                    observation_context, residual, residual_context, &xbar, &pbar,
                                    U_LOCAL(remaining)(workspace, cursor)));
    return U_LOCAL(commit)(state, &xbar, &pbar, workspace.basic);
}

#undef U_IMPORT_VIEW
#undef U_CHECK_FINITE
#undef U_TAKE
#undef U_TRY
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
#undef U_FINITE
#undef U_IMPORT
#undef U_EXPORT
#undef U_PACK_FIELD
#undef U_PREDICT_STORAGE
#undef U_PREDICT_BASIC
#undef U_UPDATE_STORAGE
#undef U_UPDATE_BASIC
