/* Native-Q PF implementation. Callback rows and unbounded Gaussian work
 * remain floating, while particles, weights and statistical MACs do not. */
#include "q_workspace_impl.h"
#include "pf_q_internal.h"

#define P_JOIN_INNER(a, b) a##b
#define P_JOIN(a, b) P_JOIN_INNER(a, b)
#define P_API(name) P_JOIN(P_JOIN(filters_pf_, U_TAG), P_JOIN(_, name))
#define P_LOCAL(name) P_JOIN(P_JOIN(pf_q_, U_TAG), P_JOIN(_, name))
#define P_SCRATCH(name) P_JOIN(P_JOIN(ukf_work_, U_TAG), P_JOIN(_, name))
#define P_LINEAR(name) P_JOIN(P_JOIN(linears_, name), P_JOIN(_, U_KERNEL))
#define P_WIDEN P_JOIN(ukf_widen_, U_TAG)
#define P_WIDEN_VALUE P_JOIN(ukf_widen_value_, U_TAG)
#define P_FITS P_JOIN(ukf_fits_, U_TAG)
#define P_STORE P_JOIN(ukf_store_, U_TAG)
#define P_FINITE P_JOIN(ukf_finite_, U_KERNEL)
#define P_IN_UNIT P_JOIN(ukf_in_unit_, U_KERNEL)
#define P_SYMMETRIC P_JOIN(ukf_symmetric_, U_TAG)
#define P_TRY(expression) do { status = (expression); if (status != LINEARS_OK) return status; } while (0)
#define P_TAKE(rows, cols) P_SCRATCH(take)(&cursor, (rows), (cols))

linears_status_t P_API(effective_sample_size)(const linears_matrix_t *weights, U_WORK *out)
{
    ukf_spans_t spans;
    linears_status_t status;
    pf_q_u128_t total, squares = {0, 0};
    const U_SIGNAL *w;
    U_WORK sum;
    size_t i;
    if (out == NULL) return LINEARS_ERROR_ARGUMENT;
    spans.count = 0;
    P_TRY(ukf_matrix(&spans, weights, 0, 1, sizeof(U_SIGNAL), U_ALIGNOF(U_SIGNAL), UKF_INPUT));
    w = (const U_SIGNAL *)weights->data;
    P_TRY(P_LOCAL(weight_total)(w, weights->rows, &total));
    for (i = 0; i < weights->rows; ++i) {
        pf_q_u128_t square = {(uint64_t)w[i] * (uint64_t)w[i], 0};
        squares = pf_q_uadd(squares, square);
    }
    sum = P_LOCAL(unsigned_value)(total);
    /* Divide first: sum*sum need not fit float even though ESS <= N does. */
    *out = (sum / P_LOCAL(unsigned_value)(squares)) * sum;
    return LINEARS_OK;
}

static linears_status_t P_LOCAL(init_validated)(U_STATE *state, const linears_matrix_t *mean,
                                                const linears_matrix_t *covariance,
                                                const linears_matrix_t *process_noise,
                                                const linears_matrix_t *measurement_noise,
                                                U_SAMPLE_FN sample, void *context, U_WORKSPACE workspace)
{
    linears_status_t status;
    U_WORK *cursor = workspace.data;
    linears_matrix_t covariance_in = P_TAKE(mean->rows, mean->rows);
    linears_matrix_t factor = P_TAKE(mean->rows, mean->rows);
    linears_matrix_t mean_in = P_TAKE(mean->rows, 1);
    linears_matrix_t draw = P_TAKE(mean->rows, 1);
    linears_matrix_t result = P_TAKE(mean->rows, 1);
    size_t i, j, n = mean->rows, count = state->particles.rows, m = state->measurement_noise.rows;
    P_WIDEN((const U_SIGNAL *)mean->data, n, (U_WORK *)mean_in.data);
    P_WIDEN((const U_SIGNAL *)covariance->data, n * n, (U_WORK *)covariance_in.data);
    workspace.decomposition.capacity = FILTERS_PF_PREDICT_DECOMP_STORAGE(n);
    P_TRY(P_LINEAR(cholesky)(&covariance_in, &factor, workspace.decomposition));
    for (i = 0; i < count; ++i) {
        P_TRY(sample(&draw, context));
        P_TRY(P_FINITE(&draw));
        for (j = 0; j < n; ++j)
            ((U_WORK *)result.data)[j] = ((const U_WORK *)mean_in.data)[j] +
                P_LOCAL(dot)((const U_WORK *)factor.data + j * n, 1, (const U_WORK *)draw.data, 1, j + 1);
        P_TRY(P_FITS((const U_WORK *)result.data, n));
        P_STORE((const U_WORK *)result.data, n, workspace.signal + i * n);
    }
    memcpy(state->particles.data, workspace.signal, count * n * sizeof(U_SIGNAL));
    P_LOCAL(reset_weights)(&state->weights);
    if (process_noise->data != state->process_noise.data)
        memcpy(state->process_noise.data, process_noise->data, n * n * sizeof(U_SIGNAL));
    if (measurement_noise->data != state->measurement_noise.data)
        memcpy(state->measurement_noise.data, measurement_noise->data, m * m * sizeof(U_SIGNAL));
    return LINEARS_OK;
}

linears_status_t P_API(init)(U_STATE *state, const linears_matrix_t *mean, const linears_matrix_t *covariance,
                             const linears_matrix_t *process_noise, const linears_matrix_t *measurement_noise,
                             U_SAMPLE_FN sample, void *context, U_WORKSPACE workspace)
{
    linears_status_t status;
    ukf_spans_t spans;
    size_t count, n, m;
    if (sample == NULL) return LINEARS_ERROR_ARGUMENT;
    spans.count = 0;
    P_TRY(P_LOCAL(state)(&spans, state, &count, &n, &m));
    P_TRY(ukf_matrix(&spans, mean, n, 1, sizeof(U_SIGNAL), U_ALIGNOF(U_SIGNAL), UKF_INPUT));
    P_TRY(ukf_matrix(&spans, covariance, n, n, sizeof(U_SIGNAL), U_ALIGNOF(U_SIGNAL), UKF_INPUT));
    P_TRY(ukf_source(&spans, process_noise, &state->process_noise, sizeof(U_SIGNAL), U_ALIGNOF(U_SIGNAL)));
    P_TRY(ukf_source(&spans, measurement_noise, &state->measurement_noise, sizeof(U_SIGNAL), U_ALIGNOF(U_SIGNAL)));
    P_TRY(P_SCRATCH(workspace)(&spans, workspace, FILTERS_PF_Q_INIT_STORAGE(count, n),
                               FILTERS_PF_Q_PREDICT_BASIC_BYTES(n), FILTERS_PF_PREDICT_DECOMP_STORAGE(n),
                               FILTERS_PF_Q_INIT_SIGNAL_STORAGE(count, n)));
    P_TRY(P_SYMMETRIC(covariance, 1));
    P_TRY(P_SYMMETRIC(process_noise, 1));
    P_TRY(P_SYMMETRIC(measurement_noise, 0));
    return P_LOCAL(init_validated)(state, mean, covariance, process_noise, measurement_noise,
                                   sample, context, workspace);
}

linears_status_t P_API(predict)(U_STATE *state, U_FN transition, void *transition_context,
                                U_SAMPLE_FN sample, void *sample_context, U_WORKSPACE workspace)
{
    linears_status_t status;
    ukf_spans_t spans;
    linears_matrix_t q, factor, input, predicted, draw;
    U_WORK *cursor;
    size_t count, n, m, i, j;
    if (transition == NULL || sample == NULL) return LINEARS_ERROR_ARGUMENT;
    spans.count = 0;
    P_TRY(P_LOCAL(state)(&spans, state, &count, &n, &m));
    P_TRY(P_SCRATCH(workspace)(&spans, workspace, FILTERS_PF_Q_PREDICT_STORAGE(count, n),
                               FILTERS_PF_Q_PREDICT_BASIC_BYTES(n), FILTERS_PF_PREDICT_DECOMP_STORAGE(n),
                               FILTERS_PF_Q_PREDICT_SIGNAL_STORAGE(count, n)));
    P_TRY(P_SYMMETRIC(&state->process_noise, 1));
    cursor = workspace.data;
    q = P_TAKE(n, n); factor = P_TAKE(n, n);
    input = P_TAKE(n, 1); predicted = P_TAKE(n, 1); draw = P_TAKE(n, 1);
    P_WIDEN((const U_SIGNAL *)state->process_noise.data, n * n, (U_WORK *)q.data);
    workspace.decomposition.capacity = FILTERS_PF_PREDICT_DECOMP_STORAGE(n);
    P_TRY(P_LINEAR(cholesky)(&q, &factor, workspace.decomposition));
    /* Unlike the floating PF, Q processes a complete particle before moving
     * on. An unbounded transition result is not quantized before adding noise. */
    for (i = 0; i < count; ++i) {
        P_WIDEN((const U_SIGNAL *)state->particles.data + i * n, n, (U_WORK *)input.data);
        P_TRY(transition(&input, &predicted, transition_context));
        P_TRY(P_FINITE(&predicted));
        P_TRY(sample(&draw, sample_context));
        P_TRY(P_FINITE(&draw));
        for (j = 0; j < n; ++j)
            ((U_WORK *)predicted.data)[j] +=
                P_LOCAL(dot)((const U_WORK *)factor.data + j * n, 1, (const U_WORK *)draw.data, 1, j + 1);
        P_TRY(P_FITS((const U_WORK *)predicted.data, n));
        P_STORE((const U_WORK *)predicted.data, n, workspace.signal + i * n);
    }
    memcpy(state->particles.data, workspace.signal, count * n * sizeof(U_SIGNAL));
    return LINEARS_OK;
}

linears_status_t P_API(update)(U_STATE *state, const linears_matrix_t *measurement, U_FN observation,
                               void *observation_context, U_RESIDUAL_FN residual, void *residual_context,
                               U_WORKSPACE workspace)
{
    linears_status_t status;
    ukf_spans_t spans;
    linears_matrix_t r, factor, particle, measured, z, deviation;
    pf_q_u128_t total;
    U_WORK *cursor, *exponents, *solve;
    size_t count, n, m, i, j;
    if (observation == NULL) return LINEARS_ERROR_ARGUMENT;
    spans.count = 0;
    P_TRY(P_LOCAL(state)(&spans, state, &count, &n, &m));
    P_TRY(ukf_matrix(&spans, measurement, m, 1, sizeof(U_SIGNAL), U_ALIGNOF(U_SIGNAL), UKF_INPUT));
    P_TRY(P_SCRATCH(workspace)(&spans, workspace, FILTERS_PF_Q_UPDATE_STORAGE(count, n, m),
                               FILTERS_PF_Q_UPDATE_BASIC_BYTES(m), FILTERS_PF_Q_UPDATE_DECOMP_STORAGE(m),
                               FILTERS_PF_Q_UPDATE_SIGNAL_STORAGE(count, n, m)));
    P_TRY(P_SYMMETRIC(&state->measurement_noise, 0));
    P_TRY(P_LOCAL(weight_total)((const U_SIGNAL *)state->weights.data, count, &total));
    cursor = workspace.data;
    exponents = cursor; cursor += count;
    r = P_TAKE(m, m); factor = P_TAKE(m, m);
    particle = P_TAKE(n, 1); measured = P_TAKE(m, 1); z = P_TAKE(m, 1); deviation = P_TAKE(m, 1);
    /* Once the observation/residual finished, its input row is dead and can
     * serve as the solve vector. n may be smaller than m, so use measured. */
    solve = (U_WORK *)measured.data;
    P_WIDEN((const U_SIGNAL *)state->measurement_noise.data, m * m, (U_WORK *)r.data);
    P_WIDEN((const U_SIGNAL *)measurement->data, m, (U_WORK *)z.data);
    workspace.decomposition.capacity = FILTERS_PF_Q_UPDATE_DECOMP_STORAGE(m);
    P_TRY(P_LINEAR(cholesky)(&r, &factor, workspace.decomposition));
    for (i = 0; i < count; ++i) {
        P_WIDEN((const U_SIGNAL *)state->particles.data + i * n, n, (U_WORK *)particle.data);
        P_TRY(observation(&particle, &measured, observation_context));
        P_TRY(P_FINITE(&measured));
        if (residual != NULL) {
            P_TRY(residual(&measured, &z, &deviation, residual_context));
            P_TRY(P_FINITE(&deviation));
        } else {
            for (j = 0; j < m; ++j)
                ((U_WORK *)deviation.data)[j] = ((const U_WORK *)measured.data)[j] - ((const U_WORK *)z.data)[j];
        }
        P_TRY(P_LOCAL(gaussian)((const U_WORK *)deviation.data, (const U_WORK *)factor.data,
                                 m, solve, exponents + i));
    }
    P_TRY(P_LOCAL(reweight)(&state->weights, exponents, workspace.signal, count));
    memcpy(state->weights.data, workspace.signal, count * sizeof(U_SIGNAL));
    return LINEARS_OK;
}

linears_status_t P_API(resample)(U_STATE *state, U_WORK uniform, size_t *indices, size_t capacity,
                                 U_WORKSPACE workspace)
{
    linears_status_t status;
    ukf_spans_t spans;
    size_t count, n, m;
    (void)workspace;
    spans.count = 0;
    P_TRY(P_LOCAL(state)(&spans, state, &count, &n, &m));
    if (indices == NULL) return LINEARS_ERROR_ARGUMENT;
    if (capacity < count) return LINEARS_ERROR_WORKSPACE;
    P_TRY(ukf_span(&spans, indices, LINEARS_SIZE_MUL(count, sizeof(size_t)), U_ALIGNOF(size_t), UKF_WRITABLE));
    P_TRY(ukf_disjoint(&spans));
    P_TRY(P_LOCAL(resample_indices)(&state->weights, uniform, indices, capacity));
    ukf_gather_rows(state->particles.data, n * sizeof(U_SIGNAL), indices, count);
    P_LOCAL(reset_weights)(&state->weights);
    return LINEARS_OK;
}

static void P_LOCAL(clear_accumulators)(linears_workspace_unit_t *storage, U_WORK *extended, size_t count)
{
    size_t i;
    pf_q_i128_t zero = {0, 0};
    for (i = 0; i < count; ++i) { pf_q_acc_write(storage + 2 * i, zero); extended[i] = 0; }
}

/* A raw residual can span (-2,2). Its product is unsigned 64-bit magnitude;
 * q31 rounds the product once to 31 fractional bits BEFORE weighting. The
 * weighted average error is <= half an output LSB, independent of N.
 * q15 retains the full 30-bit product and therefore loses no product bits. */
static uint64_t P_LOCAL(covariance_product)(int64_t a, int64_t b, U_SIGNAL weight)
{
    uint64_t product = pf_q_magnitude(a) * pf_q_magnitude(b);
#if U_BITS == 31
    product = (product + UINT64_C(1073741824)) >> 31;
#endif
    return product * (uint64_t)weight;
}

linears_status_t P_API(estimate)(const U_STATE *state, U_RESIDUAL_FN residual, void *residual_context,
                                 linears_matrix_t *mean, linears_matrix_t *covariance, U_WORKSPACE workspace)
{
    linears_status_t status;
    ukf_spans_t spans;
    linears_matrix_t input, reference, difference;
    pf_q_u128_t total;
    U_WORK *center, *row, *delta, *extended, denominator, covariance_scale;
    U_SIGNAL *mean_stage, *covariance_stage, *raw_row;
    const U_SIGNAL *particles, *weights;
    size_t count, n, m, i, j, k;
    spans.count = 0;
    P_TRY(P_LOCAL(state)(&spans, state, &count, &n, &m));
    P_TRY(ukf_matrix(&spans, mean, n, 1, sizeof(U_SIGNAL), U_ALIGNOF(U_SIGNAL), UKF_WRITABLE));
    P_TRY(ukf_matrix(&spans, covariance, n, n, sizeof(U_SIGNAL), U_ALIGNOF(U_SIGNAL), UKF_WRITABLE));
    P_TRY(P_SCRATCH(workspace)(&spans, workspace, FILTERS_PF_Q_ESTIMATE_STORAGE(count, n),
                               FILTERS_PF_Q_ESTIMATE_BASIC_BYTES(count, n), 0,
                               FILTERS_PF_Q_ESTIMATE_SIGNAL_STORAGE(count, n)));
    particles = (const U_SIGNAL *)state->particles.data;
    weights = (const U_SIGNAL *)state->weights.data;
    P_TRY(P_LOCAL(weight_total)(weights, count, &total));
    denominator = P_LOCAL(unsigned_value)(total);
    center = workspace.data; row = center + n; delta = row + n; extended = delta + n;
    mean_stage = workspace.signal; covariance_stage = mean_stage + n; raw_row = covariance_stage + n * n;
    input.data = row; input.rows = n; input.cols = 1;
    reference.data = center; reference.rows = n; reference.cols = 1;
    difference.data = delta; difference.rows = n; difference.cols = 1;
    P_LOCAL(clear_accumulators)(workspace.basic, extended, n);
    if (residual == NULL) {
        /* One exact wide sum stays local across its particle loop. Each
         * element retains the original particle order, without loading and
         * storing both workspace limbs for every contribution. */
        for (j = 0; j < n; ++j) {
            pf_q_i128_t sum = {0, 0};
            for (i = 0; i < count; ++i) {
                int64_t value = particles[i * n + j];
                pf_q_iadd(&sum, pf_q_magnitude(value) * (uint64_t)weights[i], value < 0);
            }
            pf_q_acc_write(workspace.basic + 2 * j, sum);
        }
    } else {
        P_WIDEN(particles, n, center);
        for (i = 0; i < count; ++i) {
            P_WIDEN(particles + i * n, n, row);
            P_TRY(residual(&input, &reference, &difference, residual_context));
            P_TRY(P_FINITE(&difference));
            for (j = 0; j < n; ++j) {
                if (P_IN_UNIT(delta[j])) {
                    int64_t value = P_LOCAL(raw)(delta[j]);
                    pf_q_acc_add(workspace.basic + 2 * j, pf_q_magnitude(value) * (uint64_t)weights[i], value < 0);
                } else {
                    extended[j] += ((U_WORK)weights[i] / denominator) * delta[j];
                }
            }
        }
    }
    for (j = 0; j < n; ++j) {
        U_WORK value = P_LOCAL(signed_value)(pf_q_acc_read(workspace.basic + 2 * j)) / denominator / (U_WORK)U_SCALE;
        center[j] = value + extended[j] + (residual != NULL ? center[j] : (U_WORK)0);
    }
    P_TRY(P_FITS(center, n));
    P_STORE(center, n, mean_stage);
    P_LOCAL(clear_accumulators)(workspace.basic, extended, n * n);
    if (residual == NULL) {
        for (j = 0; j < n; ++j) {
            for (k = 0; k <= j; ++k) {
                pf_q_i128_t sum = {0, 0};
                for (i = 0; i < count; ++i) {
                    int64_t a = (int64_t)particles[i * n + j] - mean_stage[j];
                    int64_t b = (int64_t)particles[i * n + k] - mean_stage[k];
                    pf_q_iadd(&sum, P_LOCAL(covariance_product)(a, b, weights[i]), (a < 0) != (b < 0));
                }
                pf_q_acc_write(workspace.basic + 2 * (j * n + k), sum);
            }
        }
    } else {
        /* Callback order is observable: residuals retain their row-major
         * path and extended-range accumulation exactly as before. */
        for (i = 0; i < count; ++i) {
            P_WIDEN(particles + i * n, n, row);
            P_TRY(residual(&input, &reference, &difference, residual_context));
            P_TRY(P_FINITE(&difference));
            for (j = 0; j < n; ++j)
                if (P_IN_UNIT(delta[j])) {
                    int64_t value = P_LOCAL(raw)(delta[j]);
                    raw_row[j] = (U_SIGNAL)(value > U_SIGNAL_MAX ? U_SIGNAL_MAX : value);
                }
            for (j = 0; j < n; ++j) {
                for (k = 0; k <= j; ++k) {
                    size_t element = j * n + k;
                    if (P_IN_UNIT(delta[j]) && P_IN_UNIT(delta[k])) {
                        U_WORK positive_cell = ((U_WORK)U_SIGNAL_MAX + (U_WORK)0.5) / (U_WORK)U_SCALE;
                        int64_t a = delta[j] >= positive_cell ? (int64_t)U_SCALE : raw_row[j];
                        int64_t b = delta[k] >= positive_cell ? (int64_t)U_SCALE : raw_row[k];
                        pf_q_acc_add(workspace.basic + 2 * element,
                                     P_LOCAL(covariance_product)(a, b, weights[i]), (a < 0) != (b < 0));
                    } else {
                        extended[element] += ((U_WORK)weights[i] / denominator) * delta[j] * delta[k];
                    }
                }
            }
        }
    }
#if U_BITS == 31
    covariance_scale = (U_WORK)U_SCALE;
#else
    covariance_scale = (U_WORK)U_SCALE * (U_WORK)U_SCALE;
#endif
    for (j = 0; j < n; ++j) {
        for (k = 0; k <= j; ++k) {
            size_t element = j * n + k;
            U_WORK value = P_LOCAL(signed_value)(pf_q_acc_read(workspace.basic + 2 * element)) /
                           denominator / covariance_scale + extended[element];
            if (residual == NULL) {
                U_WORK a = center[j] - P_WIDEN_VALUE(mean_stage[j]);
                U_WORK b = center[k] - P_WIDEN_VALUE(mean_stage[k]);
                value -= a * b;
            }
            P_TRY(P_FITS(&value, 1));
            P_STORE(&value, 1, covariance_stage + element);
            covariance_stage[k * n + j] = covariance_stage[element];
        }
    }
    memcpy(mean->data, mean_stage, n * sizeof(U_SIGNAL));
    memcpy(covariance->data, covariance_stage, n * n * sizeof(U_SIGNAL));
    return LINEARS_OK;
}

#undef P_TAKE
#undef P_TRY
#undef P_SYMMETRIC
#undef P_IN_UNIT
#undef P_FINITE
#undef P_STORE
#undef P_FITS
#undef P_WIDEN_VALUE
#undef P_WIDEN
#undef P_LINEAR
#undef P_SCRATCH
#undef P_LOCAL
#undef P_API
#undef P_JOIN
#undef P_JOIN_INNER
#undef U_TAG
#undef U_KERNEL
#undef U_SIGNAL
#undef U_SIGNAL_MAX
#undef U_WORK
#undef U_STATE
#undef U_WORKSPACE
#undef U_FN
#undef U_RESIDUAL_FN
#undef U_SAMPLE_FN
#undef U_BITS
#undef U_SCALE
#undef U_LIKELIHOOD_SCALE
#undef U_EXP_NEG
#undef U_FREXP
#undef U_LDEXP
#undef U_MANT_DIG
