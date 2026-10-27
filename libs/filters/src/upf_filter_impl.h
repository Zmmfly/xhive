/* Floating unscented particle filters; included by pf.c once per filter
 * tag after pf_filter_impl.h so the PF state helpers of the same tag exist. */
#include "ukf_private.h"
#define U_JOIN_INNER(a, b) a##b
#define U_JOIN(a, b) U_JOIN_INNER(a, b)
#define U_API(name) U_JOIN(U_JOIN(filters_upf_, U_TAG), U_JOIN(_, name))
#define U_PF_API(name) U_JOIN(U_JOIN(filters_pf_, U_TAG), U_JOIN(_, name))
#define U_KERNEL_API(name) U_JOIN(U_JOIN(filters_pf_, U_KERNEL), U_JOIN(_, name))
#define U_UKF_API(name) U_JOIN(U_JOIN(filters_ukf_, U_KERNEL), U_JOIN(_, name))
#define U_CORE(name) U_JOIN(U_JOIN(ukf_work_, U_KERNEL), U_JOIN(_, name))
#define U_LOCAL(name) U_JOIN(U_JOIN(upf_filter_, U_TAG), U_JOIN(_, name))
#define U_PF_LOCAL(name) U_JOIN(U_JOIN(pf_filter_, U_TAG), U_JOIN(_, name))
#define U_WORK_LOCAL(name) U_JOIN(U_JOIN(ukf_work_, U_KERNEL), U_JOIN(_, name))
#define U_PF_WORK_LOCAL(name) U_JOIN(U_JOIN(pf_work_, U_KERNEL), U_JOIN(_, name))
#define U_CHOLESKY U_JOIN(ukf_cholesky_, U_KERNEL)
#define U_WEIGHTS_VALID U_JOIN(ukf_weights_valid_, U_KERNEL)
#define U_SYMMETRIC U_JOIN(ukf_symmetric_, U_TAG)
#define U_FN U_JOIN(U_JOIN(filters_ukf_, U_KERNEL), _fn)
#define U_RESIDUAL_FN U_JOIN(U_JOIN(filters_ukf_, U_KERNEL), _residual_fn)
#define U_TRY(expression) do { \
    status = (expression); \
    if (status != LINEARS_OK) return status; \
} while (0)
#define U_TAKE(rows, cols) U_WORK_LOCAL(take)(&cursor, (rows), (cols))
#include "ukf_signal_impl.h"
#define U_INIT_STORAGE FILTERS_PF_PREDICT_STORAGE
#define U_STEP_STORAGE FILTERS_UPF_STEP_STORAGE
#define U_RESAMPLE_STORAGE(N) ((size_t)0)

static linears_status_t U_LOCAL(divide)(U_WORK a, U_WORK b, U_WORK *out)
{
    *out = a / b;
    return isfinite(*out) ? LINEARS_OK : LINEARS_ERROR_RANGE;
}

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
                     sizeof(U_SIGNAL), U_ALIGNOF(U_SIGNAL));
}

/* Shape the views through which the kernels read one particle and its covariance. */
static void U_LOCAL(particle_slots)(size_t n, U_WORK **cursor, linears_matrix_t *particle,
                                    linears_matrix_t *covariance)
{
    (void)cursor;
    particle->data = NULL;
    particle->rows = n;
    particle->cols = 1;
    covariance->data = NULL;
    covariance->rows = n;
    covariance->cols = n;
}

/* Point the slots at particle i in the state arrays. */
static void U_LOCAL(load_particle)(const U_STATE *state, size_t i, linears_matrix_t *particle,
                                   linears_matrix_t *covariance)
{
    size_t n = state->base.particles.cols;
    particle->data = (U_WORK *)state->base.particles.data + i * n;
    covariance->data = (U_WORK *)state->covariances.data + i * n * n;
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
    /* Every particle starts with the prior covariance, copied in the signal format. */
    for (i = 0; i < count; ++i)
        memcpy((U_SIGNAL *)state->covariances.data + i * n * n, covariance->data, n * n * sizeof(U_SIGNAL));
    state->sigma_weights = weights;
    return LINEARS_OK;
}

/* One callback row view over a contiguous k-vector. */
static linears_matrix_t U_LOCAL(vector)(U_WORK *data, size_t k)
{
    linears_matrix_t result;
    result.data = data;
    result.rows = k;
    result.cols = 1;
    return result;
}

/* out = h(point) or f(point) for one particle; the callback sees disjoint
 * n-by-1 and k-by-1 views and its result must be finite. */
static linears_status_t U_LOCAL(map)(const linears_matrix_t *point, U_FN fn, void *context, linears_matrix_t *out)
{
    linears_status_t status;
    U_TRY(fn(point, out, context));
    return U_FINITE(out);
}

/* out = a (-) b for one row: the residual callback or plain subtraction with
 * the finite check of the former deviations kernel. */
static linears_status_t U_LOCAL(residual)(const linears_matrix_t *a, const linears_matrix_t *b,
                                          U_RESIDUAL_FN fn, void *context, linears_matrix_t *out)
{
    linears_status_t status;
    size_t j;
    if (fn != NULL) {
        U_TRY(fn(a, b, out, context));
        return U_FINITE(out);
    }
    for (j = 0; j < out->rows; ++j) {
        U_WORK value = ((const U_WORK *)a->data)[j] - ((const U_WORK *)b->data)[j];
        if (!U_JOIN(ukf_isfinite_, U_KERNEL)(value)) return LINEARS_ERROR_RANGE;
        ((U_WORK *)out->data)[j] = value;
    }
    return LINEARS_OK;
}

linears_status_t U_API(step)(U_STATE *state, const linears_matrix_t *measurement, const U_MODEL *model,
                             U_SAMPLE_FN sample, void *sample_context, U_WORKSPACE workspace)
{
    linears_status_t status;
    ukf_spans_t spans;
    linears_matrix_t staged_covariances, staged_particles, exponents, staged_weights;
    linears_matrix_t draw, factor, prior_mean, prior_covariance, posterior_mean;
    linears_matrix_t predicted, delta, measured, innovation, term;
    linears_matrix_t q_factor, r_factor, reference, q_in, r_in, z_in;
    linears_matrix_t particle, covariance, new_covariance, new_particle;
    U_WORKSPACE kernel;
    U_WORK *cursor, *w, *e;
    const U_WORK *diagonal, *xi;
    size_t count, n, m, i, j, k;
    int diagonal_q, diagonal_r;
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
    staged_weights = U_PF_LOCAL(stage_weights)(&state->base.weights, &cursor);
    U_LOCAL(particle_slots)(n, &cursor, &particle, &covariance);
    kernel = U_WORK_LOCAL(remaining)(workspace, cursor);
    /* Both noise factors are needed for the weights; check them before any
     * callback. The sigma weights follow, where the first kernel call used
     * to validate them. */
    U_TRY(U_CHOLESKY((const U_WORK *)q_in.data, n, (U_WORK *)q_factor.data));
    U_TRY(U_CHOLESKY((const U_WORK *)r_in.data, m, (U_WORK *)r_factor.data));
    U_TRY(U_WEIGHTS_VALID(&state->sigma_weights));
    diagonal_q = U_PF_WORK_LOCAL(diagonal_factor)((const U_WORK *)q_factor.data, n);
    diagonal_r = U_PF_WORK_LOCAL(diagonal_factor)((const U_WORK *)r_factor.data, m);
    new_covariance.rows = new_covariance.cols = n;
    diagonal = (const U_WORK *)factor.data;
    xi = (const U_WORK *)draw.data;
    w = (U_WORK *)staged_weights.data;
    e = (U_WORK *)exponents.data;
    for (i = 0; i < count; ++i) {
        linears_matrix_t measured_vector = U_LOCAL(vector)((U_WORK *)measured.data, m);
        linears_matrix_t innovation_vector = U_LOCAL(vector)((U_WORK *)innovation.data, m);
        linears_matrix_t delta_vector = U_LOCAL(vector)((U_WORK *)delta.data, n);
        U_WORK *x, exponent, sum;
        U_LOCAL(load_particle)(state, i, &particle, &covariance);
        new_covariance.data = (U_WORK *)staged_covariances.data + i * n * n;
        new_particle = U_WORK_LOCAL(row)(&staged_particles, i);
        x = (U_WORK *)new_particle.data;
        /* The persistent row is caller-owned: it keeps the checks the public
         * kernel applied to every particle before the shared ones moved to
         * the step boundary. */
        U_CHECK_FINITE(&particle);
        U_CHECK_FINITE(&covariance);
        U_TRY(U_SYMMETRIC(&covariance, 1));
        /* Gaussian proposal N(M_i, C_i) from one UKF predict/update pair. */
        U_TRY(U_CORE(predict_core)(&particle, &covariance, &q_in, &state->sigma_weights, model->transition,
                                   model->transition_context, model->state_residual,
                                   model->state_residual_context, &prior_mean, &prior_covariance, kernel));
        /* The prior is exactly symmetric by construction; its diagonal can
         * still be negative under a negative center weight, which the public
         * update rejected as DOMAIN before any callback. */
        U_TRY(U_SYMMETRIC(&prior_covariance, 1));
        U_TRY(U_CORE(update_core)(&prior_mean, &prior_covariance, &r_in, &z_in, &state->sigma_weights,
                                  model->observation, model->observation_context,
                                  model->measurement_residual, model->measurement_residual_context,
                                  &posterior_mean, &new_covariance, kernel));
        U_TRY(U_CHOLESKY((const U_WORK *)new_covariance.data, n, (U_WORK *)factor.data));
        U_TRY(sample(&draw, sample_context));
        U_TRY(U_FINITE(&draw));
        /* x = M_i + L_i*xi in the former matrix order: every column of L,
         * including its exact upper zeros, then the mean. */
        for (j = 0; j < n; ++j) {
            sum = (U_WORK)0;
            for (k = 0; k < n; ++k) {
                U_WORK product = diagonal[j * n + k] * xi[k];
                sum += product;
                if (!U_JOIN(ukf_isfinite_, U_KERNEL)(sum)) return LINEARS_ERROR_RANGE;
            }
            x[j] = sum;
        }
        for (j = 0; j < n; ++j) {
            U_WORK value = x[j] + ((const U_WORK *)posterior_mean.data)[j];
            if (!U_JOIN(ukf_isfinite_, U_KERNEL)(value)) return LINEARS_ERROR_RANGE;
            x[j] = value;
        }
        /* Proposal density exponent -(1/2)*xi^T*xi, written as the former
         * axpby so a zero keeps its sign; its sqrt(det C_i) enters the weight
         * as a ratio to particle 0 so the products stay near one. */
        sum = (U_WORK)0;
        for (k = 0; k < n; ++k) {
            U_WORK product = xi[k] * xi[k];
            sum += product;
            if (!U_JOIN(ukf_isfinite_, U_KERNEL)(sum)) return LINEARS_ERROR_RANGE;
        }
        exponent = (U_WORK)-0.5 * sum + (U_WORK)0 * sum;
        for (j = 0; j < n; ++j) {
            U_WORK ratio;
            if (i == 0) {
                ((U_WORK *)reference.data)[j] = diagonal[j * n + j];
                continue;
            }
            U_TRY(U_LOCAL(divide)(diagonal[j * n + j], ((const U_WORK *)reference.data)[j], &ratio));
            /* The former axpby formed ratio*w + 0*w. ratio is a quotient of
             * two positive diagonals, so both terms carry the sign of w and
             * adding the signed zero leaves ratio*w unchanged, zero or not;
             * a nonfinite w or product fails the same RANGE check. */
            w[i] = ratio * w[i];
            if (!U_JOIN(ukf_isfinite_, U_KERNEL)(w[i])) return LINEARS_ERROR_RANGE;
        }
        /* Likelihood exponent of z given the new particle. */
        U_TRY(U_LOCAL(map)(&new_particle, model->observation, model->observation_context, &measured_vector));
        U_TRY(U_LOCAL(residual)(&measured_vector, &z_in, model->measurement_residual,
                                model->measurement_residual_context, &innovation_vector));
        if (diagonal_r)
            U_TRY(U_PF_WORK_LOCAL(gaussian_diagonal)((const U_WORK *)innovation.data,
                     (const U_WORK *)r_factor.data, m, kernel.data, (U_WORK *)term.data));
        else U_TRY(U_KERNEL_API(gaussian_exponent)(&innovation, &r_factor, &term, kernel));
        exponent = exponent + ((const U_WORK *)term.data)[0];
        if (!U_JOIN(ukf_isfinite_, U_KERNEL)(exponent)) return LINEARS_ERROR_RANGE;
        /* Transition prior exponent of the new particle given the old one. */
        U_TRY(U_LOCAL(map)(&particle, model->transition, model->transition_context, &predicted));
        U_TRY(U_LOCAL(residual)(&new_particle, &predicted, model->state_residual,
                                model->state_residual_context, &delta_vector));
        if (diagonal_q)
            U_TRY(U_PF_WORK_LOCAL(gaussian_diagonal)((const U_WORK *)delta.data,
                     (const U_WORK *)q_factor.data, n, kernel.data, (U_WORK *)term.data));
        else U_TRY(U_KERNEL_API(gaussian_exponent)(&delta, &q_factor, &term, kernel));
        exponent = exponent + ((const U_WORK *)term.data)[0];
        if (!U_JOIN(ukf_isfinite_, U_KERNEL)(exponent)) return LINEARS_ERROR_RANGE;
        e[i] = exponent;
    }
    U_TRY(U_KERNEL_API(reweight)(&staged_weights, &exponents));
    U_FITS(&staged_particles);
    U_FITS(&staged_covariances);
    U_STORE(&staged_particles, &state->base.particles);
    U_STORE(&staged_covariances, &state->covariances);
    U_PF_LOCAL(store_weights)(&staged_weights, &state->base.weights);
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
    weights = U_PF_LOCAL(load_weights)(&state->base.weights, &cursor);
    U_TRY(U_KERNEL_API(resample_indices)(&weights, uniform, indices, capacity));
    ukf_gather_rows(state->base.particles.data, n * sizeof(U_SIGNAL), indices, count);
    ukf_gather_rows(state->covariances.data, n * n * sizeof(U_SIGNAL), indices, count);
    U_PF_LOCAL(reset_weights)(&state->base.weights);
    return LINEARS_OK;
}

#undef U_RESAMPLE_STORAGE
#undef U_STEP_STORAGE
#undef U_INIT_STORAGE
#undef U_STORE
#undef U_FITS
#undef U_IMPORT_VIEW
#undef U_CHECK_FINITE
#undef U_TAKE
#undef U_TRY
#undef U_SYMMETRIC
#undef U_RESIDUAL_FN
#undef U_FN
#undef U_WEIGHTS_VALID
#undef U_CHOLESKY
#undef U_PF_WORK_LOCAL
#undef U_WORK_LOCAL
#undef U_PF_LOCAL
#undef U_LOCAL
#undef U_CORE
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
