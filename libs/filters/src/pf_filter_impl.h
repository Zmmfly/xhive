/* Floating particle filters; the Q filters have dedicated integer kernels. */
#define U_JOIN_INNER(a, b) a##b
#define U_JOIN(a, b) U_JOIN_INNER(a, b)
#define U_API(name) U_JOIN(U_JOIN(filters_pf_, U_TAG), U_JOIN(_, name))
#define U_KERNEL_API(name) U_JOIN(U_JOIN(filters_pf_, U_KERNEL), U_JOIN(_, name))
#define U_UKF_API(name) U_JOIN(U_JOIN(filters_ukf_, U_KERNEL), U_JOIN(_, name))
#define U_LOCAL(name) U_JOIN(U_JOIN(pf_filter_, U_TAG), U_JOIN(_, name))
#define U_WORK_LOCAL(name) U_JOIN(U_JOIN(ukf_work_, U_KERNEL), U_JOIN(_, name))
#define U_PF_WORK_LOCAL(name) U_JOIN(U_JOIN(pf_work_, U_KERNEL), U_JOIN(_, name))
#define U_LINEAR(name) U_JOIN(U_JOIN(linears_, name), U_JOIN(_, U_KERNEL))
#define U_SYMMETRIC U_JOIN(ukf_symmetric_, U_TAG)
#define U_TRY(expression) do { \
    status = (expression); \
    if (status != LINEARS_OK) return status; \
} while (0)
#define U_TAKE(rows, cols) U_WORK_LOCAL(take)(&cursor, (rows), (cols))
#define U_BASIC workspace.basic, workspace.basic_bytes
#include "ukf_signal_impl.h"
#define U_INIT_STORAGE FILTERS_PF_PREDICT_STORAGE
#define U_PREDICT_STORAGE FILTERS_PF_PREDICT_STORAGE
#define U_UPDATE_STORAGE(N, n, m) FILTERS_PF_UPDATE_STORAGE((N), (m))
#define U_RESAMPLE_STORAGE(N) ((size_t)0)
#define U_ESTIMATE_STORAGE FILTERS_PF_ESTIMATE_STORAGE
#define U_NORMALIZE(weights) ((void)0)

/* The four arrays of an already registered state object. */
static linears_status_t U_LOCAL(arrays)(ukf_spans_t *spans, const U_STATE *state,
                                        size_t *particles, size_t *states, size_t *measurements)
{
    linears_status_t status;
    size_t count = state->particles.rows, n = state->particles.cols, m = state->measurement_noise.rows;
    U_TRY(ukf_owned(spans, &state->particles, count, n, state->particles_capacity,
                    sizeof(U_SIGNAL), U_ALIGNOF(U_SIGNAL)));
    U_TRY(ukf_owned(spans, &state->weights, count, 1, state->weights_capacity,
                    sizeof(U_SIGNAL), U_ALIGNOF(U_SIGNAL)));
    U_TRY(ukf_owned(spans, &state->process_noise, n, n, state->process_noise_capacity,
                    sizeof(U_SIGNAL), U_ALIGNOF(U_SIGNAL)));
    U_TRY(ukf_owned(spans, &state->measurement_noise, m, m, state->measurement_noise_capacity,
                    sizeof(U_SIGNAL), U_ALIGNOF(U_SIGNAL)));
    *particles = count;
    *states = n;
    *measurements = m;
    return LINEARS_OK;
}

static linears_status_t U_LOCAL(state)(ukf_spans_t *spans, const U_STATE *state,
                                       size_t *particles, size_t *states, size_t *measurements)
{
    linears_status_t status;
    U_TRY(ukf_span(spans, state, sizeof(*state), U_ALIGNOF(U_STATE), UKF_WRITABLE));
    return U_LOCAL(arrays)(spans, state, particles, states, measurements);
}

/* Floating weights sum to one; the native Q implementation is separate. */
static void U_LOCAL(reset_weights)(linears_matrix_t *weights)
{
    size_t i;
    U_SIGNAL *w = (U_SIGNAL *)weights->data;
    const U_SIGNAL value = (U_SIGNAL)1 / (U_SIGNAL)weights->rows;
    for (i = 0; i < weights->rows; ++i) w[i] = value;
}

static linears_matrix_t U_LOCAL(load_weights)(const linears_matrix_t *weights, U_WORK **cursor)
{
    (void)cursor;
    return *weights;
}

static linears_matrix_t U_LOCAL(stage_weights)(const linears_matrix_t *weights, U_WORK **cursor)
{
    linears_matrix_t staged = U_WORK_LOCAL(take)(cursor, weights->rows, 1);
    memcpy(staged.data, weights->data, weights->rows * sizeof(U_WORK));
    return staged;
}

static void U_LOCAL(store_weights)(linears_matrix_t *staged, linears_matrix_t *weights)
{
    U_STORE(staged, weights);
}

static linears_status_t U_LOCAL(propagate)(const linears_matrix_t *particles, U_FN fn, void *context,
                                           linears_matrix_t *out, U_WORK **cursor)
{
    (void)cursor;
    return U_UKF_API(propagate)(particles, fn, context, out);
}

linears_status_t U_API(init)(U_STATE *state, const linears_matrix_t *mean, const linears_matrix_t *covariance,
                             const linears_matrix_t *process_noise, const linears_matrix_t *measurement_noise,
                             U_SAMPLE_FN sample, void *context, U_WORKSPACE workspace)
{
    linears_status_t status;
    ukf_spans_t spans;
    linears_matrix_t staged, factor, mean_in, covariance_in, row;
    U_WORK *cursor;
    size_t count, n, m, i;
    if (sample == NULL) return LINEARS_ERROR_ARGUMENT;
    spans.count = 0;
    U_TRY(U_LOCAL(state)(&spans, state, &count, &n, &m));
    U_TRY(ukf_matrix(&spans, mean, n, 1, sizeof(U_SIGNAL), U_ALIGNOF(U_SIGNAL), UKF_INPUT));
    U_TRY(ukf_matrix(&spans, covariance, n, n, sizeof(U_SIGNAL), U_ALIGNOF(U_SIGNAL), UKF_INPUT));
    U_TRY(ukf_source(&spans, process_noise, &state->process_noise, sizeof(U_SIGNAL), U_ALIGNOF(U_SIGNAL)));
    U_TRY(ukf_source(&spans, measurement_noise, &state->measurement_noise, sizeof(U_SIGNAL), U_ALIGNOF(U_SIGNAL)));
    U_TRY(U_WORK_LOCAL(workspace)(&spans, workspace, U_INIT_STORAGE(count, n),
                                  FILTERS_PF_PREDICT_BASIC_BYTES(n), FILTERS_PF_PREDICT_DECOMP_STORAGE(n)));
    U_CHECK_FINITE(mean);
    U_CHECK_FINITE(covariance);
    U_CHECK_FINITE(process_noise);
    U_CHECK_FINITE(measurement_noise);
    U_TRY(U_SYMMETRIC(covariance, 1));
    U_TRY(U_SYMMETRIC(process_noise, 1));
    U_TRY(U_SYMMETRIC(measurement_noise, 0));
    workspace.decomposition.capacity = FILTERS_PF_PREDICT_DECOMP_STORAGE(n);
    cursor = workspace.data;
    staged = U_TAKE(count, n);
    factor = U_TAKE(n, n);
    mean_in = U_IMPORT_VIEW(mean);
    covariance_in = U_IMPORT_VIEW(covariance);
    U_TRY(U_LINEAR(cholesky)(&covariance_in, &factor, workspace.decomposition));
    /* Every row starts at the mean; perturb adds L*xi in place. */
    for (i = 0; i < count; ++i) {
        row = U_WORK_LOCAL(row)(&staged, i);
        memcpy(row.data, mean_in.data, n * sizeof(U_WORK));
    }
    U_TRY(U_KERNEL_API(perturb)(&staged, &factor, sample, context, &staged,
                                U_WORK_LOCAL(remaining)(workspace, cursor)));
    U_FITS(&staged);
    U_STORE(&staged, &state->particles);
    U_LOCAL(reset_weights)(&state->weights);
    if (process_noise->data != state->process_noise.data)
        memcpy(state->process_noise.data, process_noise->data, n * n * sizeof(U_SIGNAL));
    if (measurement_noise->data != state->measurement_noise.data)
        memcpy(state->measurement_noise.data, measurement_noise->data, m * m * sizeof(U_SIGNAL));
    return LINEARS_OK;
}

linears_status_t U_API(predict)(U_STATE *state, U_FN transition, void *transition_context,
                                U_SAMPLE_FN sample, void *sample_context, U_WORKSPACE workspace)
{
    linears_status_t status;
    ukf_spans_t spans;
    linears_matrix_t staged, factor, q_in;
    U_WORK *cursor;
    size_t count, n, m;
    if (transition == NULL || sample == NULL) return LINEARS_ERROR_ARGUMENT;
    spans.count = 0;
    U_TRY(U_LOCAL(state)(&spans, state, &count, &n, &m));
    U_TRY(U_WORK_LOCAL(workspace)(&spans, workspace, U_PREDICT_STORAGE(count, n),
                                  FILTERS_PF_PREDICT_BASIC_BYTES(n), FILTERS_PF_PREDICT_DECOMP_STORAGE(n)));
    U_CHECK_FINITE(&state->process_noise);
    U_TRY(U_SYMMETRIC(&state->process_noise, 1));
    workspace.decomposition.capacity = FILTERS_PF_PREDICT_DECOMP_STORAGE(n);
    cursor = workspace.data;
    staged = U_TAKE(count, n);
    factor = U_TAKE(n, n);
    q_in = U_IMPORT_VIEW(&state->process_noise);
    /* Q must have a Cholesky factor before any callback runs. */
    U_TRY(U_LINEAR(cholesky)(&q_in, &factor, workspace.decomposition));
    U_TRY(U_LOCAL(propagate)(&state->particles, transition, transition_context, &staged, &cursor));
    U_TRY(U_KERNEL_API(perturb)(&staged, &factor, sample, sample_context, &staged,
                                U_WORK_LOCAL(remaining)(workspace, cursor)));
    U_FITS(&staged);
    U_STORE(&staged, &state->particles);
    return LINEARS_OK;
}

linears_status_t U_API(update)(U_STATE *state, const linears_matrix_t *measurement, U_FN observation,
                               void *observation_context, U_RESIDUAL_FN residual, void *residual_context,
                               U_WORKSPACE workspace)
{
    linears_status_t status;
    ukf_spans_t spans;
    linears_matrix_t measured, deviations, exponents, staged, factor, r_in, z_in;
    U_WORK *cursor;
    size_t count, n, m, i;
    if (observation == NULL) return LINEARS_ERROR_ARGUMENT;
    spans.count = 0;
    U_TRY(U_LOCAL(state)(&spans, state, &count, &n, &m));
    U_TRY(ukf_matrix(&spans, measurement, m, 1, sizeof(U_SIGNAL), U_ALIGNOF(U_SIGNAL), UKF_INPUT));
    U_TRY(U_WORK_LOCAL(workspace)(&spans, workspace, U_UPDATE_STORAGE(count, n, m),
                                  FILTERS_PF_UPDATE_BASIC_BYTES(m), FILTERS_PF_UPDATE_DECOMP_STORAGE(m)));
    U_CHECK_FINITE(&state->measurement_noise);
    U_CHECK_FINITE(measurement);
    U_TRY(U_SYMMETRIC(&state->measurement_noise, 0));
    workspace.decomposition.capacity = FILTERS_PF_UPDATE_DECOMP_STORAGE(m);
    cursor = workspace.data;
    measured = U_TAKE(count, m);
    deviations = U_TAKE(count, m);
    exponents = U_TAKE(count, 1);
    factor = U_TAKE(m, m);
    r_in = U_IMPORT_VIEW(&state->measurement_noise);
    z_in = U_IMPORT_VIEW(measurement);
    staged = U_LOCAL(stage_weights)(&state->weights, &cursor);
    U_TRY(U_LINEAR(cholesky)(&r_in, &factor, workspace.decomposition));
    U_TRY(U_LOCAL(propagate)(&state->particles, observation, observation_context, &measured, &cursor));
    U_TRY(U_UKF_API(deviations)(&measured, &z_in, residual, residual_context, &deviations, workspace));
    if (U_PF_WORK_LOCAL(diagonal_factor)((const U_WORK *)factor.data, m)) {
        for (i = 0; i < count; ++i)
            U_TRY(U_PF_WORK_LOCAL(gaussian_diagonal)((const U_WORK *)deviations.data + i * m,
                     (const U_WORK *)factor.data, m, cursor, (U_WORK *)exponents.data + i));
    } else {
        U_TRY(U_KERNEL_API(gaussian_exponent)(&deviations, &factor, &exponents,
                                               U_WORK_LOCAL(remaining)(workspace, cursor)));
    }
    U_TRY(U_KERNEL_API(reweight)(&staged, &exponents));
    U_LOCAL(store_weights)(&staged, &state->weights);
    return LINEARS_OK;
}

linears_status_t U_API(resample)(U_STATE *state, U_WORK uniform, size_t *indices, size_t capacity,
                                 U_WORKSPACE workspace)
{
    linears_status_t status;
    ukf_spans_t spans;
    linears_matrix_t weights;
    U_WORK *cursor;
    size_t count, n, m;
    spans.count = 0;
    U_TRY(U_LOCAL(state)(&spans, state, &count, &n, &m));
    if (indices == NULL) return LINEARS_ERROR_ARGUMENT;
    if (capacity < count) return LINEARS_ERROR_WORKSPACE;
    U_TRY(ukf_span(&spans, indices, LINEARS_SIZE_MUL(count, sizeof(size_t)), U_ALIGNOF(size_t), UKF_WRITABLE));
    U_TRY(U_WORK_LOCAL(workspace)(&spans, workspace, U_RESAMPLE_STORAGE(count), 0, 0));
    cursor = workspace.data;
    weights = U_LOCAL(load_weights)(&state->weights, &cursor);
    U_TRY(U_KERNEL_API(resample_indices)(&weights, uniform, indices, capacity));
    /* Systematic indices are nondecreasing, so the rows move in place. */
    ukf_gather_rows(state->particles.data, n * sizeof(U_SIGNAL), indices, count);
    U_LOCAL(reset_weights)(&state->weights);
    return LINEARS_OK;
}

linears_status_t U_API(estimate)(const U_STATE *state, U_RESIDUAL_FN residual, void *residual_context,
                                 linears_matrix_t *mean, linears_matrix_t *covariance, U_WORKSPACE workspace)
{
    linears_status_t status;
    ukf_spans_t spans;
    linears_matrix_t mean_out, covariance_out, transposed, deviations, particles_in, weights_in;
    U_WORKSPACE sub;
    U_WORK *cursor;
    size_t count, n, m;
    spans.count = 0;
    U_TRY(U_LOCAL(state)(&spans, state, &count, &n, &m));
    U_TRY(ukf_matrix(&spans, mean, n, 1, sizeof(U_SIGNAL), U_ALIGNOF(U_SIGNAL), UKF_WRITABLE));
    U_TRY(ukf_matrix(&spans, covariance, n, n, sizeof(U_SIGNAL), U_ALIGNOF(U_SIGNAL), UKF_WRITABLE));
    U_TRY(U_WORK_LOCAL(workspace)(&spans, workspace, U_ESTIMATE_STORAGE(count, n),
                                  FILTERS_PF_ESTIMATE_BASIC_BYTES(count, n), 0));
    cursor = workspace.data;
    mean_out = U_TAKE(n, 1);
    covariance_out = U_TAKE(n, n);
    transposed = U_TAKE(n, n);
    deviations = U_TAKE(count, n);
    particles_in = U_IMPORT_VIEW(&state->particles);
    weights_in = U_IMPORT_VIEW(&state->weights);
    U_NORMALIZE(&weights_in);
    sub = U_WORK_LOCAL(remaining)(workspace, cursor);
    U_TRY(U_UKF_API(mean)(&particles_in, &weights_in, residual, residual_context, &mean_out, sub));
    U_TRY(U_UKF_API(deviations)(&particles_in, &mean_out, residual, residual_context, &deviations, workspace));
    U_TRY(U_UKF_API(weighted_covariance)(&deviations, &deviations, &weights_in, &covariance_out, sub));
    U_TRY(U_LINEAR(transpose)(&covariance_out, &transposed, U_BASIC));
    U_TRY(U_LINEAR(axpby)((U_WORK)0.5, &covariance_out, (U_WORK)0.5, &transposed, &covariance_out, U_BASIC));
    U_FITS(&mean_out);
    U_FITS(&covariance_out);
    U_STORE(&mean_out, mean);
    U_STORE(&covariance_out, covariance);
    return LINEARS_OK;
}

#undef U_NORMALIZE
#undef U_ESTIMATE_STORAGE
#undef U_RESAMPLE_STORAGE
#undef U_UPDATE_STORAGE
#undef U_PREDICT_STORAGE
#undef U_INIT_STORAGE
#undef U_STORE
#undef U_FITS
#undef U_IMPORT_VIEW
#undef U_CHECK_FINITE
#undef U_BASIC
#undef U_TAKE
#undef U_TRY
#undef U_SYMMETRIC
#undef U_LINEAR
#undef U_PF_WORK_LOCAL
#undef U_WORK_LOCAL
#undef U_LOCAL
#undef U_UKF_API
#undef U_KERNEL_API
#undef U_API
#undef U_JOIN
#undef U_JOIN_INNER
#undef U_TAG
#undef U_KERNEL
#undef U_SIGNAL
#undef U_SIGNAL_MAX
#undef U_WORK
#undef U_FIXED
#undef U_STATE
#undef U_WORKSPACE
#undef U_FN
#undef U_RESIDUAL_FN
#undef U_SAMPLE_FN
#undef U_FINITE
