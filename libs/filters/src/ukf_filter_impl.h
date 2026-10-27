/* Signal-format UKF filters; included by ukf.c once per filter tag. */
#define U_JOIN_INNER(a, b) a##b
#define U_JOIN(a, b) U_JOIN_INNER(a, b)
#define U_API(name) U_JOIN(U_JOIN(filters_ukf_, U_TAG), U_JOIN(_, name))
#define U_WEIGHTS_API U_JOIN(U_JOIN(filters_ukf_, U_KERNEL), _weights)
#define U_LOCAL(name) U_JOIN(U_JOIN(ukf_filter_, U_TAG), U_JOIN(_, name))
#if U_FIXED
#define U_KERNEL_API(name) U_API(name)
#define U_WORK_LOCAL(name) U_JOIN(U_JOIN(ukf_work_, U_TAG), U_JOIN(_, name))
#define U_REMAINING(workspace, cursor) U_WORK_LOCAL(remaining)((workspace), (cursor), (workspace).signal)
#else
#define U_KERNEL_API(name) U_JOIN(U_JOIN(filters_ukf_, U_KERNEL), U_JOIN(_, name))
#define U_WORK_LOCAL(name) U_JOIN(U_JOIN(ukf_work_, U_KERNEL), U_JOIN(_, name))
#define U_REMAINING(workspace, cursor) U_WORK_LOCAL(remaining)((workspace), (cursor))
#endif
#define U_SYMMETRIC U_JOIN(ukf_symmetric_, U_TAG)
#define U_TRY(expression) do { \
    status = (expression); \
    if (status != LINEARS_OK) return status; \
} while (0)
#define U_TAKE(rows, cols) U_WORK_LOCAL(take)(&cursor, (rows), (cols))
#include "ukf_signal_impl.h"

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
    for (i = 0; i < 4; ++i)
        U_TRY(ukf_source(&spans, sources[i], targets[i], sizeof(U_SIGNAL), U_ALIGNOF(U_SIGNAL)));
    U_TRY(ukf_disjoint(&spans));
    U_TRY(U_WEIGHTS_API(n, alpha, beta, kappa, &weights));
    for (i = 0; i < 4; ++i) U_CHECK_FINITE(sources[i]);
    U_TRY(U_SYMMETRIC(covariance, 1));
    U_TRY(U_SYMMETRIC(process_noise, 1));
    U_TRY(U_SYMMETRIC(measurement_noise, 0));
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
#if U_FIXED
    U_TRY(U_WORK_LOCAL(workspace)(&spans, workspace, U_PREDICT_STORAGE(n),
                                  FILTERS_UKF_Q_PREDICT_BASIC_BYTES(n),
                                  FILTERS_UKF_Q_PREDICT_DECOMP_STORAGE(n),
                                  FILTERS_UKF_Q_PREDICT_SIGNAL_STORAGE(n)));
#else
    U_TRY(U_WORK_LOCAL(workspace)(&spans, workspace, U_PREDICT_STORAGE(n),
                                  FILTERS_UKF_PREDICT_BASIC_BYTES(n),
                                  FILTERS_UKF_PREDICT_DECOMP_STORAGE(n)));
#endif
    cursor = workspace.data;
    xbar = U_TAKE(n, 1);
    pbar = U_TAKE(n, n);
    x_in = U_IMPORT_VIEW(&state->estimate);
    p_in = U_IMPORT_VIEW(&state->covariance);
    q_in = U_IMPORT_VIEW(&state->process_noise);
    U_TRY(U_KERNEL_API(predict_work)(&x_in, &p_in, &q_in, &state->weights, transition,
                                     transition_context, residual, residual_context, &xbar, &pbar,
                                     U_REMAINING(workspace, cursor)));
    U_FITS(&xbar);
    U_FITS(&pbar);
    U_STORE(&xbar, &state->estimate);
    U_STORE(&pbar, &state->covariance);
    return LINEARS_OK;
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
#if U_FIXED
    U_TRY(U_WORK_LOCAL(workspace)(&spans, workspace, U_UPDATE_STORAGE(n, m),
                                  FILTERS_UKF_Q_UPDATE_BASIC_BYTES(n, m),
                                  FILTERS_UKF_Q_UPDATE_DECOMP_STORAGE(n, m),
                                  FILTERS_UKF_Q_UPDATE_SIGNAL_STORAGE(n, m)));
#else
    U_TRY(U_WORK_LOCAL(workspace)(&spans, workspace, U_UPDATE_STORAGE(n, m),
                                  FILTERS_UKF_UPDATE_BASIC_BYTES(n, m),
                                  FILTERS_UKF_UPDATE_DECOMP_STORAGE(n, m)));
#endif
    cursor = workspace.data;
    xbar = U_TAKE(n, 1);
    pbar = U_TAKE(n, n);
    x_in = U_IMPORT_VIEW(&state->estimate);
    p_in = U_IMPORT_VIEW(&state->covariance);
    r_in = U_IMPORT_VIEW(&state->measurement_noise);
    z_in = U_IMPORT_VIEW(measurement);
    U_TRY(U_KERNEL_API(update_work)(&x_in, &p_in, &r_in, &z_in, &state->weights, observation,
                                    observation_context, residual, residual_context, &xbar, &pbar,
                                    U_REMAINING(workspace, cursor)));
    U_FITS(&xbar);
    U_FITS(&pbar);
    U_STORE(&xbar, &state->estimate);
    U_STORE(&pbar, &state->covariance);
    return LINEARS_OK;
}

#undef U_STORE
#undef U_FITS
#undef U_IMPORT_VIEW
#undef U_CHECK_FINITE
#undef U_TAKE
#undef U_TRY
#undef U_SYMMETRIC
#undef U_REMAINING
#undef U_WORK_LOCAL
#undef U_LOCAL
#undef U_WEIGHTS_API
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
#undef U_PREDICT_STORAGE
#undef U_UPDATE_STORAGE
