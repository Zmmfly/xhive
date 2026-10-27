/* Signal-format unscented particle filters; included by pf.c once per filter
 * tag after pf_filter_impl.h so the PF state helpers of the same tag exist. */
#define U_JOIN_INNER(a, b) a##b
#define U_JOIN(a, b) U_JOIN_INNER(a, b)
#define U_API(name) U_JOIN(U_JOIN(filters_upf_, U_TAG), U_JOIN(_, name))
#define U_PF_API(name) U_JOIN(U_JOIN(filters_pf_, U_TAG), U_JOIN(_, name))
#define U_KERNEL_API(name) U_JOIN(U_JOIN(filters_pf_, U_KERNEL), U_JOIN(_, name))
#define U_UKF_API(name) U_JOIN(U_JOIN(filters_ukf_, U_KERNEL), U_JOIN(_, name))
#define U_LOCAL(name) U_JOIN(U_JOIN(upf_filter_, U_TAG), U_JOIN(_, name))
#define U_PF_LOCAL(name) U_JOIN(U_JOIN(pf_filter_, U_TAG), U_JOIN(_, name))
#define U_WORK_LOCAL(name) U_JOIN(U_JOIN(ukf_work_, U_KERNEL), U_JOIN(_, name))
#define U_LINEAR(name) U_JOIN(U_JOIN(linears_, name), U_JOIN(_, U_KERNEL))
#define U_SYMMETRIC U_JOIN(ukf_symmetric_, U_TAG)
#define U_TRY(expression) do { \
    status = (expression); \
    if (status != LINEARS_OK) return status; \
} while (0)
#define U_TAKE(rows, cols) U_WORK_LOCAL(take)(&cursor, (rows), (cols))
#define U_BASIC workspace.basic, workspace.basic_bytes
#if U_FIXED
#define U_CHECK_FINITE(matrix) ((void)0)
#define U_IMPORT_VIEW(matrix) U_JOIN(ukf_import_, U_TAG)((matrix), &cursor)
#define U_DIVIDE(a, b, out) linears_fixed_div((a), (b), (out))
#else
#define U_CHECK_FINITE(matrix) U_TRY(U_FINITE(matrix))
/* Floating signals are already working values; the cursor stays where it is. */
#define U_IMPORT_VIEW(matrix) ((void)cursor, *(matrix))
#define U_DIVIDE(a, b, out) U_LOCAL(divide)((a), (b), (out))
#endif

#if !U_FIXED
static linears_status_t U_LOCAL(divide)(U_WORK a, U_WORK b, U_WORK *out)
{
    *out = a / b;
    return isfinite(*out) ? LINEARS_OK : LINEARS_ERROR_RANGE;
}
#endif

/* The UPF object, the four PF arrays and the per-particle covariances. */
static linears_status_t U_LOCAL(state)(ukf_spans_t *spans, const U_STATE *state,
                                       size_t *particles, size_t *states, size_t *measurements)
{
    linears_status_t status;
    size_t square;
    U_TRY(ukf_span(spans, state, sizeof(*state), U_ALIGNOF(U_STATE), UKF_WRITABLE));
    U_TRY(U_PF_LOCAL(arrays)(spans, &state->base, particles, states, measurements));
    square = LINEARS_SIZE_MUL(*states, *states);
    if (square == SIZE_MAX) return LINEARS_ERROR_SIZE;
    return ukf_owned(spans, &state->covariances, *particles, square, state->covariances_capacity,
                     sizeof(U_WORK), U_ALIGNOF(U_WORK));
}

linears_status_t U_API(init)(U_STATE *state, U_WORK alpha, U_WORK beta, U_WORK kappa,
                             const linears_matrix_t *mean, const linears_matrix_t *covariance,
                             const linears_matrix_t *process_noise, const linears_matrix_t *measurement_noise,
                             U_SAMPLE_FN sample, void *context, U_WORKSPACE workspace)
{
    linears_status_t status;
    ukf_spans_t spans;
    U_WEIGHTS weights;
    linears_matrix_t prior;
    U_WORK *cursor;
    size_t count, n, m, i;
    spans.count = 0;
    U_TRY(U_LOCAL(state)(&spans, state, &count, &n, &m));
    U_TRY(ukf_matrix(&spans, mean, n, 1, sizeof(U_SIGNAL), U_ALIGNOF(U_SIGNAL), UKF_INPUT));
    U_TRY(ukf_matrix(&spans, covariance, n, n, sizeof(U_SIGNAL), U_ALIGNOF(U_SIGNAL), UKF_INPUT));
    U_TRY(ukf_source(&spans, process_noise, &state->base.process_noise, sizeof(U_SIGNAL), U_ALIGNOF(U_SIGNAL)));
    U_TRY(ukf_source(&spans, measurement_noise, &state->base.measurement_noise, sizeof(U_SIGNAL), U_ALIGNOF(U_SIGNAL)));
    U_TRY(U_WORK_LOCAL(workspace)(&spans, workspace, U_INIT_STORAGE(count, n),
                                  FILTERS_PF_PREDICT_BASIC_BYTES(n), FILTERS_PF_PREDICT_DECOMP_STORAGE(n)));
    U_TRY(U_UKF_API(weights)(n, alpha, beta, kappa, &weights));
    /* The particle set is validated again and drawn by the PF; it commits atomically. */
    U_TRY(U_PF_API(init)(&state->base, mean, covariance, process_noise, measurement_noise, sample, context,
                         workspace));
    cursor = workspace.data;
    prior = U_IMPORT_VIEW(covariance);
    for (i = 0; i < count; ++i)
        memcpy((U_WORK *)state->covariances.data + i * n * n, prior.data, n * n * sizeof(U_WORK));
    state->sigma_weights = weights;
    return LINEARS_OK;
}

linears_status_t U_API(step)(U_STATE *state, const linears_matrix_t *measurement, const U_MODEL *model,
                             U_SAMPLE_FN sample, void *sample_context, U_WORKSPACE workspace)
{
    linears_status_t status;
    ukf_spans_t spans;
    linears_matrix_t staged_covariances, staged_particles, exponents, staged_weights;
    linears_matrix_t draw, draw_row, factor, prior_mean, prior_covariance, posterior_mean;
    linears_matrix_t predicted, predicted_row, delta, measured, innovation, term;
    linears_matrix_t q_factor, r_factor, reference, q_in, r_in, z_in;
    linears_matrix_t particle, particle_row, covariance, new_covariance, new_particle, new_row;
    linears_matrix_t exponent, weight;
    U_WORKSPACE kernel;
    U_WORK *cursor, ratio;
    const U_WORK *diagonal;
    size_t count, n, m, i, j;
    if (model == NULL || model->transition == NULL || model->observation == NULL || sample == NULL)
        return LINEARS_ERROR_ARGUMENT;
    spans.count = 0;
    U_TRY(U_LOCAL(state)(&spans, state, &count, &n, &m));
    U_TRY(ukf_matrix(&spans, measurement, m, 1, sizeof(U_SIGNAL), U_ALIGNOF(U_SIGNAL), UKF_INPUT));
    U_TRY(U_WORK_LOCAL(workspace)(&spans, workspace, U_STEP_STORAGE(count, n, m),
                                  FILTERS_UPF_STEP_BASIC_BYTES(n, m), FILTERS_UPF_STEP_DECOMP_STORAGE(n, m)));
    U_CHECK_FINITE(&state->base.process_noise);
    U_CHECK_FINITE(&state->base.measurement_noise);
    U_CHECK_FINITE(measurement);
    U_TRY(U_SYMMETRIC(&state->base.process_noise, 1));
    U_TRY(U_SYMMETRIC(&state->base.measurement_noise, 0));
    workspace.decomposition.capacity = FILTERS_UPF_STEP_DECOMP_STORAGE(n, m);
    cursor = workspace.data;
    staged_covariances = U_TAKE(count, n * n);
    staged_particles = U_TAKE(count, n);
    exponents = U_TAKE(count, 1);
    staged_weights = U_TAKE(count, 1);
    draw = U_TAKE(n, 1);
    factor = U_TAKE(n, n);
    prior_mean = U_TAKE(n, 1);
    prior_covariance = U_TAKE(n, n);
    posterior_mean = U_TAKE(n, 1);
    predicted = U_TAKE(n, 1);
    delta = U_TAKE(1, n);
    measured = U_TAKE(1, m);
    innovation = U_TAKE(1, m);
    term = U_TAKE(1, 1);
    q_factor = U_TAKE(n, n);
    r_factor = U_TAKE(m, m);
    reference = U_TAKE(n, 1);
    q_in = U_IMPORT_VIEW(&state->base.process_noise);
    r_in = U_IMPORT_VIEW(&state->base.measurement_noise);
    z_in = U_IMPORT_VIEW(measurement);
    kernel = U_WORK_LOCAL(remaining)(workspace, cursor);
    /* Both noise factors are needed for the weights; check them before any callback. */
    U_TRY(U_LINEAR(cholesky)(&q_in, &q_factor, workspace.decomposition));
    U_TRY(U_LINEAR(cholesky)(&r_in, &r_factor, workspace.decomposition));
    memcpy(staged_weights.data, state->base.weights.data, count * sizeof(U_WORK));
    draw_row = draw;
    draw_row.rows = 1;
    draw_row.cols = n;
    predicted_row = predicted;
    predicted_row.rows = 1;
    predicted_row.cols = n;
    covariance.rows = covariance.cols = new_covariance.rows = new_covariance.cols = n;
    exponent.rows = exponent.cols = weight.rows = weight.cols = 1;
    diagonal = (const U_WORK *)factor.data;
    for (i = 0; i < count; ++i) {
        particle = U_WORK_LOCAL(row)(&state->base.particles, i);
        particle_row = particle;
        particle_row.rows = 1;
        particle_row.cols = n;
        covariance.data = (U_WORK *)state->covariances.data + i * n * n;
        new_covariance.data = (U_WORK *)staged_covariances.data + i * n * n;
        new_particle = U_WORK_LOCAL(row)(&staged_particles, i);
        new_row = new_particle;
        new_row.rows = 1;
        new_row.cols = n;
        exponent.data = (U_WORK *)exponents.data + i;
        weight.data = (U_WORK *)staged_weights.data + i;
        /* Gaussian proposal N(M_i, C_i) from one UKF predict/update pair. */
        U_TRY(U_UKF_API(predict_work)(&particle, &covariance, &q_in, &state->sigma_weights, model->transition,
                                      model->transition_context, model->state_residual,
                                      model->state_residual_context, &prior_mean, &prior_covariance, kernel));
        U_TRY(U_UKF_API(update_work)(&prior_mean, &prior_covariance, &r_in, &z_in, &state->sigma_weights,
                                     model->observation, model->observation_context,
                                     model->measurement_residual, model->measurement_residual_context,
                                     &posterior_mean, &new_covariance, kernel));
        U_TRY(U_LINEAR(cholesky)(&new_covariance, &factor, workspace.decomposition));
        U_TRY(sample(&draw, sample_context));
        U_CHECK_FINITE(&draw);
        U_TRY(U_LINEAR(mul)(&factor, &draw, &new_particle, U_BASIC));
        U_TRY(U_LINEAR(add)(&new_particle, &posterior_mean, &new_particle, U_BASIC));
        /* Proposal density exponent -(1/2)*xi^T*xi; its sqrt(det C_i) enters the
         * weight as a ratio to particle 0 so F31 weights keep their precision. */
        U_TRY(U_LINEAR(mul)(&draw_row, &draw, &exponent, U_BASIC));
        U_TRY(U_LINEAR(axpby)(U_NEG_HALF, &exponent, U_ZERO, &exponent, &exponent, U_BASIC));
        for (j = 0; j < n; ++j) {
            if (i == 0) {
                ((U_WORK *)reference.data)[j] = diagonal[j * n + j];
                continue;
            }
            U_TRY(U_DIVIDE(diagonal[j * n + j], ((const U_WORK *)reference.data)[j], &ratio));
            U_TRY(U_LINEAR(axpby)(ratio, &weight, U_ZERO, &weight, &weight, U_BASIC));
        }
        /* Likelihood exponent of z given the new particle. */
        U_TRY(U_UKF_API(propagate)(&new_row, model->observation, model->observation_context, &measured));
        U_TRY(U_UKF_API(deviations)(&measured, &z_in, model->measurement_residual,
                                    model->measurement_residual_context, &innovation, workspace));
        U_TRY(U_KERNEL_API(gaussian_exponent)(&innovation, &r_factor, &term, kernel));
        U_TRY(U_LINEAR(add)(&exponent, &term, &exponent, U_BASIC));
        /* Transition prior exponent of the new particle given the old one. */
        U_TRY(U_UKF_API(propagate)(&particle_row, model->transition, model->transition_context, &predicted_row));
        U_TRY(U_UKF_API(deviations)(&new_row, &predicted, model->state_residual, model->state_residual_context,
                                    &delta, workspace));
        U_TRY(U_KERNEL_API(gaussian_exponent)(&delta, &q_factor, &term, kernel));
        U_TRY(U_LINEAR(add)(&exponent, &term, &exponent, U_BASIC));
    }
    U_TRY(U_KERNEL_API(reweight)(&staged_weights, &exponents));
    memcpy(state->base.particles.data, staged_particles.data, count * n * sizeof(U_WORK));
    memcpy(state->covariances.data, staged_covariances.data, count * n * n * sizeof(U_WORK));
    memcpy(state->base.weights.data, staged_weights.data, count * sizeof(U_WORK));
    return LINEARS_OK;
}

linears_status_t U_API(resample)(U_STATE *state, U_WORK uniform, size_t *indices, size_t capacity,
                                 U_WORKSPACE workspace)
{
    linears_status_t status;
    ukf_spans_t spans;
    linears_matrix_t staged_particles, staged_covariances;
    U_WORK *cursor, weight;
    size_t count, n, m;
    spans.count = 0;
    U_TRY(U_LOCAL(state)(&spans, state, &count, &n, &m));
    if (indices == NULL) return LINEARS_ERROR_ARGUMENT;
    if (capacity < count) return LINEARS_ERROR_WORKSPACE;
    U_TRY(ukf_span(&spans, indices, LINEARS_SIZE_MUL(count, sizeof(size_t)), U_ALIGNOF(size_t), UKF_WRITABLE));
    U_TRY(U_WORK_LOCAL(workspace)(&spans, workspace, FILTERS_UPF_RESAMPLE_STORAGE(count, n), 0, 0));
    U_TRY(U_PF_LOCAL(uniform)(count, &weight));
    U_TRY(U_KERNEL_API(resample_indices)(&state->base.weights, uniform, indices, capacity));
    cursor = workspace.data;
    staged_particles = U_TAKE(count, n);
    staged_covariances = U_TAKE(count, n * n);
    U_TRY(U_KERNEL_API(gather)(&state->base.particles, indices, count, &staged_particles));
    U_TRY(U_KERNEL_API(gather)(&state->covariances, indices, count, &staged_covariances));
    memcpy(state->base.particles.data, staged_particles.data, count * n * sizeof(U_WORK));
    memcpy(state->covariances.data, staged_covariances.data, count * n * n * sizeof(U_WORK));
    U_PF_LOCAL(fill)(&state->base.weights, weight);
    return LINEARS_OK;
}

#undef U_DIVIDE
#undef U_IMPORT_VIEW
#undef U_CHECK_FINITE
#undef U_BASIC
#undef U_TAKE
#undef U_TRY
#undef U_SYMMETRIC
#undef U_LINEAR
#undef U_WORK_LOCAL
#undef U_PF_LOCAL
#undef U_LOCAL
#undef U_UKF_API
#undef U_KERNEL_API
#undef U_PF_API
#undef U_API
#undef U_JOIN
#undef U_JOIN_INNER
#undef U_TAG
#undef U_KERNEL
#undef U_SIGNAL
#undef U_WORK
#undef U_FIXED
#undef U_STATE
#undef U_MODEL
#undef U_WEIGHTS
#undef U_WORKSPACE
#undef U_SAMPLE_FN
#undef U_FINITE
#undef U_ZERO
#undef U_NEG_HALF
#undef U_INIT_STORAGE
#undef U_STEP_STORAGE
