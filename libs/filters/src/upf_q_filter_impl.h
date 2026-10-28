/* Dedicated Q UPF filters. Include after the matching pf_q_filter_impl.h:
 * its raw-state, Gaussian, weighting and resampling helpers are shared. */
#include "ukf_private.h"
#define U_JOIN_INNER(a, b) a##b
#define U_JOIN(a, b) U_JOIN_INNER(a, b)
#define U_API(name) U_JOIN(U_JOIN(filters_upf_, U_TAG), U_JOIN(_, name))
#define U_CORE(name) U_JOIN(U_JOIN(ukf_work_, U_TAG), U_JOIN(_, name))
#define U_FLOAT_UKF(name) U_JOIN(U_JOIN(filters_ukf_, U_KERNEL), U_JOIN(_, name))
#define U_LOCAL(name) U_JOIN(U_JOIN(upf_q_, U_TAG), U_JOIN(_, name))
#define U_PF_LOCAL(name) U_JOIN(U_JOIN(pf_q_, U_TAG), U_JOIN(_, name))
#define U_WORK_LOCAL(name) U_JOIN(U_JOIN(ukf_work_, U_TAG), U_JOIN(_, name))
#define U_CHOLESKY U_JOIN(ukf_cholesky_, U_KERNEL)
#define U_WEIGHTS_VALID U_JOIN(ukf_weights_valid_, U_KERNEL)
#define U_SYMMETRIC U_JOIN(ukf_symmetric_, U_TAG)
#define U_FLOAT_SYMMETRIC U_JOIN(ukf_symmetric_, U_KERNEL)
#define U_FINITE U_JOIN(ukf_finite_, U_KERNEL)
#define U_RESIDUAL_FN U_JOIN(U_JOIN(filters_ukf_, U_KERNEL), _residual_fn)
#define U_TRY(expression) do { \
    status = (expression); \
    if (status != LINEARS_OK) return status; \
} while (0)
#define U_TAKE(rows, cols) U_WORK_LOCAL(take)(&cursor, (rows), (cols))
#define U_TAKE_SIGNAL(rows, cols) U_WORK_LOCAL(take_signal)(&signal_cursor, (rows), (cols))

/* Register the outer object once; the embedded PF descriptors are inside it. */
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
                     sizeof(U_SIGNAL), U_ALIGNOF(U_SIGNAL));
}

/* A local callback boundary; residuals may exceed the raw signal interval. */
static linears_status_t U_LOCAL(residual)(const linears_matrix_t *a, const linears_matrix_t *b,
                                          U_RESIDUAL_FN fn, void *context, linears_matrix_t *out)
{
    linears_status_t status;
    size_t j;
    if (fn != NULL) {
        U_TRY(fn(a, b, out, context));
    } else {
        for (j = 0; j < out->rows; ++j)
            ((U_WORK *)out->data)[j] = ((const U_WORK *)a->data)[j] - ((const U_WORK *)b->data)[j];
    }
    return U_FINITE(out);
}

linears_status_t U_API(init)(U_STATE *state, U_WORK alpha, U_WORK beta, U_WORK kappa,
                             const linears_matrix_t *mean, const linears_matrix_t *covariance,
                             const linears_matrix_t *process_noise, const linears_matrix_t *measurement_noise,
                             U_SAMPLE_FN sample, void *context, U_WORKSPACE workspace)
{
    linears_status_t status;
    ukf_spans_t spans;
    U_WEIGHTS weights;
    size_t count, n, m, i;
    if (sample == NULL) return LINEARS_ERROR_ARGUMENT;
    spans.count = 0;
    U_TRY(U_LOCAL(state)(&spans, state, &count, &n, &m));
    U_TRY(ukf_matrix(&spans, mean, n, 1, sizeof(U_SIGNAL), U_ALIGNOF(U_SIGNAL), UKF_INPUT));
    U_TRY(ukf_matrix(&spans, covariance, n, n, sizeof(U_SIGNAL), U_ALIGNOF(U_SIGNAL), UKF_INPUT));
    U_TRY(ukf_source(&spans, process_noise, &state->base.process_noise, sizeof(U_SIGNAL), U_ALIGNOF(U_SIGNAL)));
    U_TRY(ukf_source(&spans, measurement_noise, &state->base.measurement_noise, sizeof(U_SIGNAL), U_ALIGNOF(U_SIGNAL)));
    U_TRY(U_WORK_LOCAL(workspace)(&spans, workspace, FILTERS_PF_Q_INIT_STORAGE(count, n),
                                  FILTERS_PF_Q_PREDICT_BASIC_BYTES(n), FILTERS_PF_PREDICT_DECOMP_STORAGE(n),
                                  FILTERS_PF_Q_INIT_SIGNAL_STORAGE(count, n)));
    U_TRY(U_FLOAT_UKF(weights)(n, alpha, beta, kappa, &weights));
    U_TRY(U_SYMMETRIC(covariance, 1));
    U_TRY(U_SYMMETRIC(process_noise, 1));
    U_TRY(U_SYMMETRIC(measurement_noise, 0));
    U_TRY(U_PF_LOCAL(init_validated)(&state->base, mean, covariance, process_noise, measurement_noise,
                                     sample, context, workspace));
    for (i = 0; i < count; ++i)
        memcpy((U_SIGNAL *)state->covariances.data + i * n * n, covariance->data, n * n * sizeof(U_SIGNAL));
    state->sigma_weights = weights;
    return LINEARS_OK;
}

linears_status_t U_API(step)(U_STATE *state, const linears_matrix_t *measurement, const U_MODEL *model,
                             U_SAMPLE_FN sample, void *sample_context, U_WORKSPACE workspace)
{
    linears_status_t status;
    ukf_spans_t spans;
    linears_matrix_t staged_covariances, staged_particles, staged_weights;
    linears_matrix_t particle, covariance, prior_mean, prior_covariance, posterior_mean, draw;
    linears_matrix_t q_in, r_in, z_in, q_factor, r_factor, measured, innovation, raw_covariance;
    U_WORKSPACE kernel;
    U_WORK *cursor, *exponents, exponent, term, log_determinant;
    U_PF_LOCAL(scale_t) factor_scale, draw_scale;
    U_SIGNAL *signal_cursor;
    int factor_valid, draw_valid;
    size_t count, n, m, i, j;
    if (model == NULL || model->transition == NULL || model->observation == NULL || sample == NULL)
        return LINEARS_ERROR_ARGUMENT;
    spans.count = 0;
    U_TRY(U_LOCAL(state)(&spans, state, &count, &n, &m));
    U_TRY(ukf_matrix(&spans, measurement, m, 1, sizeof(U_SIGNAL), U_ALIGNOF(U_SIGNAL), UKF_INPUT));
    U_TRY(U_WORK_LOCAL(workspace)(&spans, workspace, FILTERS_UPF_Q_STEP_STORAGE(count, n, m),
                                  FILTERS_UPF_Q_STEP_BASIC_BYTES(n, m), FILTERS_UPF_Q_STEP_DECOMP_STORAGE(n, m),
                                  FILTERS_UPF_Q_STEP_SIGNAL_STORAGE(count, n, m)));
    U_TRY(U_SYMMETRIC(&state->base.process_noise, 1));
    U_TRY(U_SYMMETRIC(&state->base.measurement_noise, 0));
    cursor = workspace.data;
    signal_cursor = workspace.signal;
    staged_covariances = U_TAKE_SIGNAL(count, n * n);
    staged_particles = U_TAKE_SIGNAL(count, n);
    staged_weights = U_TAKE_SIGNAL(count, 1);
    exponents = cursor;
    cursor += count;
    particle = U_TAKE(n, 1);
    covariance = U_TAKE(n, n);
    prior_mean = U_TAKE(n, 1);
    prior_covariance = U_TAKE(n, n);
    posterior_mean = U_TAKE(n, 1);
    draw = U_TAKE(n, 1);
    q_in = U_JOIN(ukf_import_, U_TAG)(&state->base.process_noise, &cursor);
    r_in = U_JOIN(ukf_import_, U_TAG)(&state->base.measurement_noise, &cursor);
    z_in = U_JOIN(ukf_import_, U_TAG)(measurement, &cursor);
    q_factor = U_TAKE(n, n);
    r_factor = U_TAKE(m, m);
    measured = U_TAKE(m, 1);
    innovation = U_TAKE(m, 1);
    kernel = U_WORK_LOCAL(remaining)(workspace, cursor, signal_cursor);
    /* Both noise factors are required by the importance density; check them
     * before any callback. The sigma weights follow, where the first kernel
     * call used to validate them. */
    U_TRY(U_CHOLESKY((const U_WORK *)q_in.data, n, (U_WORK *)q_factor.data));
    U_TRY(U_CHOLESKY((const U_WORK *)r_in.data, m, (U_WORK *)r_factor.data));
    U_TRY(U_WEIGHTS_VALID(&state->sigma_weights));
    raw_covariance.rows = raw_covariance.cols = n;
    for (i = 0; i < count; ++i) {
        raw_covariance.data = (U_SIGNAL *)state->covariances.data + i * n * n;
        /* The persistent row is caller-owned: the public kernel rejected an
         * asymmetric or negative-diagonal covariance before any callback. */
        U_TRY(U_SYMMETRIC(&raw_covariance, 1));
        U_JOIN(ukf_widen_, U_TAG)((const U_SIGNAL *)state->base.particles.data + i * n,
                                  n, (U_WORK *)particle.data);
        U_JOIN(ukf_widen_, U_TAG)((const U_SIGNAL *)raw_covariance.data, n * n, (U_WORK *)covariance.data);
        /* Keep the prior in its extended range. Only the posterior will be
         * narrowed: a prior outside [-1,+1] can have a valid Q posterior. */
        U_TRY(U_CORE(predict_core)(&particle, &covariance, &q_in, &state->sigma_weights,
                                   model->transition, model->transition_context,
                                   model->state_residual, model->state_residual_context,
                                   &prior_mean, &prior_covariance, kernel));
        /* The prior is exactly symmetric by construction; a negative diagonal
         * under a negative center weight was DOMAIN in the public update. */
        U_TRY(U_FLOAT_SYMMETRIC(&prior_covariance, 1));
        U_TRY(U_CORE(update_core)(&prior_mean, &prior_covariance, &r_in, &z_in, &state->sigma_weights,
                                  model->observation, model->observation_context,
                                  model->measurement_residual, model->measurement_residual_context,
                                  &posterior_mean, &covariance, kernel));
        /* The old covariance is now the posterior; the prior covariance
         * slot becomes its factor and the prior mean slot the new particle. */
        U_TRY(U_CHOLESKY((const U_WORK *)covariance.data, n, (U_WORK *)prior_covariance.data));
        U_TRY(sample(&draw, sample_context));
        U_TRY(U_FINITE(&draw));
        /* Preserve the posterior base before its dead slot becomes draw bytes.
         * The decomposition is rebuilt after every UKF call, never cached
         * across one. kernel.data remains dead until the next particle. */
        memcpy(prior_mean.data, posterior_mean.data, n * sizeof(U_WORK));
        factor_valid = U_PF_LOCAL(gaussian_prepare)((const U_WORK *)prior_covariance.data, n,
                                                     (U_WORK *)kernel.decomposition.data, &factor_scale);
        draw_valid = U_PF_LOCAL(vector_prepare)((const U_WORK *)draw.data, n,
                                                 (U_WORK *)posterior_mean.data, &draw_scale);
        if (factor_valid && draw_valid &&
            U_PF_LOCAL(perturb_try)((const U_WORK *)kernel.decomposition.data, factor_scale,
                                    (const U_WORK *)posterior_mean.data, draw_scale, n, kernel.data)) {
            for (j = 0; j < n; ++j) ((U_WORK *)prior_mean.data)[j] += kernel.data[j];
        } else {
            for (j = 0; j < n; ++j)
                ((U_WORK *)prior_mean.data)[j] +=
                    U_PF_LOCAL(dot)((const U_WORK *)prior_covariance.data + j * n, 1,
                                    (const U_WORK *)draw.data, 1, j + 1);
        }
        U_TRY(U_FINITE(&prior_mean));
        if (draw_valid &&
            U_PF_LOCAL(dot_try)((const U_WORK *)posterior_mean.data, draw_scale,
                                (const U_WORK *)posterior_mean.data, draw_scale, n, &term)) {
            exponent = (U_WORK)-0.5 * term;
        } else {
            exponent = (U_WORK)-0.5 * U_PF_LOCAL(dot)((const U_WORK *)draw.data, 1,
                                                     (const U_WORK *)draw.data, 1, n);
        }
        if (!U_PF_LOCAL(log_determinant_try)((const U_WORK *)prior_covariance.data, n, &log_determinant)) {
            log_determinant = (U_WORK)0;
            for (j = 0; j < n; ++j)
                log_determinant += U_LOG(((const U_WORK *)prior_covariance.data)[j * n + j]);
        }
        exponent -= log_determinant;
        /* Likelihood of z given the new particle. Callback and residual
         * views remain float/double and retain their full finite range. */
        U_TRY(model->observation(&prior_mean, &measured, model->observation_context));
        U_TRY(U_FINITE(&measured));
        U_TRY(U_LOCAL(residual)(&measured, &z_in, model->measurement_residual,
                                model->measurement_residual_context, &innovation));
        factor_valid = U_PF_LOCAL(gaussian_prepare)((const U_WORK *)r_factor.data, m,
                                                     (U_WORK *)kernel.decomposition.data, &factor_scale);
        /* The remaining UKF data arena has at least 8*m elements, so Z and
         * its Q32 error vector need no n-versus-m dimension assumption. */
        if (factor_valid) {
            U_TRY(U_PF_LOCAL(gaussian_retry)((const U_WORK *)innovation.data, (const U_WORK *)r_factor.data,
                                              m, kernel.data, &term,
                                              (const U_WORK *)kernel.decomposition.data, factor_scale,
                                              (U_WORK *)measured.data, kernel.data + m));
        } else {
            U_TRY(U_PF_LOCAL(gaussian_mixed)((const U_WORK *)innovation.data, (const U_WORK *)r_factor.data,
                                              m, kernel.data, &term));
        }
        exponent += term;
        /* posterior_mean and draw are no longer live: reuse them for f(x)
         * and the transition residual, rather than retaining N work rows. */
        U_TRY(model->transition(&particle, &posterior_mean, model->transition_context));
        U_TRY(U_FINITE(&posterior_mean));
        U_TRY(U_LOCAL(residual)(&prior_mean, &posterior_mean, model->state_residual,
                                model->state_residual_context, &draw));
        factor_valid = U_PF_LOCAL(gaussian_prepare)((const U_WORK *)q_factor.data, n,
                                                     (U_WORK *)kernel.decomposition.data, &factor_scale);
        if (factor_valid) {
            U_TRY(U_PF_LOCAL(gaussian_retry)((const U_WORK *)draw.data, (const U_WORK *)q_factor.data,
                                              n, kernel.data, &term,
                                              (const U_WORK *)kernel.decomposition.data, factor_scale,
                                              (U_WORK *)posterior_mean.data, kernel.data + n));
        } else {
            U_TRY(U_PF_LOCAL(gaussian_mixed)((const U_WORK *)draw.data, (const U_WORK *)q_factor.data,
                                              n, kernel.data, &term));
        }
        exponent += term;
        if (!U_JOIN(ukf_isfinite_, U_KERNEL)(exponent)) return LINEARS_ERROR_RANGE;
        exponents[i] = exponent;
        U_TRY(U_JOIN(ukf_fits_, U_TAG)((const U_WORK *)prior_mean.data, n));
        U_TRY(U_JOIN(ukf_fits_, U_TAG)((const U_WORK *)covariance.data, n * n));
        U_JOIN(ukf_store_, U_TAG)((const U_WORK *)prior_mean.data, n,
                                  (U_SIGNAL *)staged_particles.data + i * n);
        U_JOIN(ukf_store_, U_TAG)((const U_WORK *)covariance.data, n * n,
                                  (U_SIGNAL *)staged_covariances.data + i * n * n);
    }
    U_TRY(U_PF_LOCAL(reweight)(&state->base.weights, exponents, (U_SIGNAL *)staged_weights.data, count));
    memcpy(state->base.particles.data, staged_particles.data, count * n * sizeof(U_SIGNAL));
    memcpy(state->covariances.data, staged_covariances.data, count * n * n * sizeof(U_SIGNAL));
    memcpy(state->base.weights.data, staged_weights.data, count * sizeof(U_SIGNAL));
    return LINEARS_OK;
}

linears_status_t U_API(resample)(U_STATE *state, U_WORK uniform, size_t *indices, size_t capacity,
                                 U_WORKSPACE workspace)
{
    linears_status_t status;
    ukf_spans_t spans;
    size_t count, n, m;
    (void)workspace;
    spans.count = 0;
    U_TRY(U_LOCAL(state)(&spans, state, &count, &n, &m));
    if (indices == NULL) return LINEARS_ERROR_ARGUMENT;
    if (capacity < count) return LINEARS_ERROR_WORKSPACE;
    U_TRY(ukf_span(&spans, indices, LINEARS_SIZE_MUL(count, sizeof(size_t)), U_ALIGNOF(size_t), UKF_WRITABLE));
    U_TRY(ukf_disjoint(&spans));
    U_TRY(U_PF_LOCAL(resample_indices)(&state->base.weights, uniform, indices, capacity));
    ukf_gather_rows(state->base.particles.data, n * sizeof(U_SIGNAL), indices, count);
    ukf_gather_rows(state->covariances.data, n * n * sizeof(U_SIGNAL), indices, count);
    U_PF_LOCAL(reset_weights)(&state->base.weights);
    return LINEARS_OK;
}

#undef U_TAKE_SIGNAL
#undef U_TAKE
#undef U_TRY
#undef U_RESIDUAL_FN
#undef U_FINITE
#undef U_FLOAT_SYMMETRIC
#undef U_SYMMETRIC
#undef U_WEIGHTS_VALID
#undef U_CHOLESKY
#undef U_WORK_LOCAL
#undef U_PF_LOCAL
#undef U_LOCAL
#undef U_FLOAT_UKF
#undef U_CORE
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
#undef U_MODEL
#undef U_WEIGHTS
#undef U_WORKSPACE
#undef U_SAMPLE_FN
#undef U_LOG
