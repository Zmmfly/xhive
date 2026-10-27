/* Working-format particle filter kernels; included by pf.c once per kernel tag. */
#include "ukf_scratch_impl.h"

#define U_JOIN_INNER(a, b) a##b
#define U_JOIN(a, b) U_JOIN_INNER(a, b)
#define U_API(name) U_JOIN(U_JOIN(filters_pf_, U_TAG), U_JOIN(_, name))
#define U_LOCAL(name) U_JOIN(U_JOIN(pf_work_, U_TAG), U_JOIN(_, name))
#define U_SCRATCH(name) U_JOIN(U_JOIN(ukf_work_, U_TAG), U_JOIN(_, name))
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

/* out may be exactly the points storage (in place) or disjoint from it. */
static linears_status_t U_LOCAL(in_out)(ukf_spans_t *spans, const linears_matrix_t *points,
                                        const linears_matrix_t *out)
{
    linears_status_t status;
    size_t bytes;
    U_TRY(ukf_span(spans, points, sizeof(*points), U_ALIGNOF(linears_matrix_t), UKF_DESCRIPTOR));
    U_TRY(ukf_span(spans, out, sizeof(*out), U_ALIGNOF(linears_matrix_t), UKF_DESCRIPTOR));
    U_TRY(ukf_shape(points, 0, 0, sizeof(U_WORK), &bytes));
    if (out->data != points->data)
        U_TRY(ukf_span(spans, points->data, bytes, U_ALIGNOF(U_WORK), UKF_INPUT));
    U_TRY(ukf_shape(out, points->rows, points->cols, sizeof(U_WORK), &bytes));
    return ukf_span(spans, out->data, bytes, U_ALIGNOF(U_WORK), UKF_WRITABLE);
}

static linears_status_t U_LOCAL(nonnegative)(const linears_matrix_t *weights)
{
    size_t i;
    const U_WORK *w = (const U_WORK *)weights->data;
    for (i = 0; i < weights->rows; ++i)
        if (w[i] < 0) return LINEARS_ERROR_DOMAIN;
    return LINEARS_OK;
}

linears_status_t U_API(perturb)(const linears_matrix_t *points, const linears_matrix_t *factor,
                                U_SAMPLE_FN sample, void *context, linears_matrix_t *out,
                                U_WORKSPACE workspace)
{
    linears_status_t status;
    ukf_spans_t spans;
    linears_matrix_t draw, noise, source, target;
    U_WORK *cursor;
    size_t k, i;
    if (sample == NULL) return LINEARS_ERROR_ARGUMENT;
    spans.count = 0;
    U_TRY(U_LOCAL(in_out)(&spans, points, out));
    k = points->cols;
    U_TRY(ukf_matrix(&spans, factor, k, k, sizeof(U_WORK), U_ALIGNOF(U_WORK), UKF_INPUT));
    U_TRY(U_SCRATCH(workspace)(&spans, workspace, FILTERS_PF_PERTURB_STORAGE(k),
                               FILTERS_PF_PERTURB_BASIC_BYTES(k), 0));
    U_CHECK_FINITE(points);
    U_CHECK_FINITE(factor);
    workspace.basic_bytes = FILTERS_PF_PERTURB_BASIC_BYTES(k);
    cursor = workspace.data;
    draw = U_SCRATCH(take)(&cursor, k, 1);
    noise = U_SCRATCH(take)(&cursor, k, 1);
    for (i = 0; i < points->rows; ++i) {
        U_TRY(sample(&draw, context));
        U_CHECK_FINITE(&draw);
        U_TRY(U_LINEAR(mul)(factor, &draw, &noise, U_BASIC));
        source = U_SCRATCH(row)(points, i);
        target = U_SCRATCH(row)(out, i);
        U_TRY(U_LINEAR(add)(&source, &noise, &target, U_BASIC));
    }
    return LINEARS_OK;
}

linears_status_t U_API(gaussian_exponent)(const linears_matrix_t *deviations, const linears_matrix_t *factor,
                                          linears_matrix_t *out, U_WORKSPACE workspace)
{
    linears_status_t status;
    ukf_spans_t spans;
    linears_matrix_t solved, transposed, deviation, result;
    U_WORK *cursor;
    size_t k, i;
    spans.count = 0;
    U_TRY(ukf_matrix(&spans, deviations, 0, 0, sizeof(U_WORK), U_ALIGNOF(U_WORK), UKF_INPUT));
    k = deviations->cols;
    U_TRY(ukf_matrix(&spans, factor, k, k, sizeof(U_WORK), U_ALIGNOF(U_WORK), UKF_INPUT));
    U_TRY(ukf_matrix(&spans, out, deviations->rows, 1, sizeof(U_WORK), U_ALIGNOF(U_WORK), UKF_WRITABLE));
    U_TRY(U_SCRATCH(workspace)(&spans, workspace, FILTERS_PF_GAUSSIAN_EXPONENT_STORAGE(k),
                               FILTERS_PF_GAUSSIAN_EXPONENT_BASIC_BYTES,
                               FILTERS_PF_GAUSSIAN_EXPONENT_DECOMP_STORAGE(k)));
    U_CHECK_FINITE(deviations);
    U_CHECK_FINITE(factor);
    workspace.basic_bytes = FILTERS_PF_GAUSSIAN_EXPONENT_BASIC_BYTES;
    workspace.decomposition.capacity = FILTERS_PF_GAUSSIAN_EXPONENT_DECOMP_STORAGE(k);
    cursor = workspace.data;
    solved = U_SCRATCH(take)(&cursor, k, 1);
    transposed = solved;
    transposed.rows = 1;
    transposed.cols = k;
    result.rows = 1;
    result.cols = 1;
    /* L*y = d, then d^T*(L*L^T)^-1*d = y^T*y, halved in place in the output row. */
    for (i = 0; i < deviations->rows; ++i) {
        deviation = U_SCRATCH(row)(deviations, i);
        result.data = (U_WORK *)out->data + i;
        U_TRY(U_LINEAR(solve)(factor, &deviation, &solved, workspace.decomposition));
        U_TRY(U_LINEAR(mul)(&transposed, &solved, &result, U_BASIC));
        U_TRY(U_LINEAR(axpby)(U_HALF, &result, U_ZERO, &result, &result, U_BASIC));
    }
    return LINEARS_OK;
}

linears_status_t U_API(reweight)(linears_matrix_t *weights, const linears_matrix_t *exponents)
{
    linears_status_t status;
    ukf_spans_t spans;
    U_WORK *w;
    const U_WORK *e;
    U_WORK lowest, sum = U_ZERO;
    size_t count, i;
    spans.count = 0;
    U_TRY(ukf_matrix(&spans, weights, 0, 1, sizeof(U_WORK), U_ALIGNOF(U_WORK), UKF_WRITABLE));
    count = weights->rows;
    U_TRY(ukf_matrix(&spans, exponents, count, 1, sizeof(U_WORK), U_ALIGNOF(U_WORK), UKF_INPUT));
    U_TRY(ukf_disjoint(&spans));
    U_CHECK_FINITE(weights);
    U_CHECK_FINITE(exponents);
    U_TRY(U_LOCAL(nonnegative)(weights));
    w = (U_WORK *)weights->data;
    e = (const U_WORK *)exponents->data;
    lowest = e[0];
    for (i = 1; i < count; ++i)
        if (e[i] < lowest) lowest = e[i];
    /* Shifting by the smallest exponent keeps the best particle at exp(0). */
    for (i = 0; i < count; ++i) {
#if U_FIXED
        U_WORK shifted;
        U_TRY(linears_fixed_sub(e[i], lowest, &shifted));
        U_TRY(linears_fixed_mul(w[i], pf_fixed_exp_neg(shifted), &w[i]));
        U_TRY(linears_fixed_add(sum, w[i], &sum));
#else
        w[i] *= U_EXP_NEG(e[i] - lowest);
        sum += w[i];
#endif
    }
#if !U_FIXED
    if (!isfinite(sum)) return LINEARS_ERROR_RANGE;
#endif
    if (sum == 0) return LINEARS_ERROR_DOMAIN;
    for (i = 0; i < count; ++i) {
#if U_FIXED
        U_TRY(linears_fixed_div(w[i], sum, &w[i]));
#else
        w[i] /= sum;
#endif
    }
    return LINEARS_OK;
}

linears_status_t U_API(effective_sample_size)(const linears_matrix_t *weights, U_WORK *out)
{
    linears_status_t status;
    ukf_spans_t spans;
    const U_WORK *w;
    U_WORK sum = U_ZERO, squares = U_ZERO, result;
    size_t count, i;
#if U_FIXED
    linears_accum_t accumulator = {0, 0};
#endif
    if (out == NULL) return LINEARS_ERROR_ARGUMENT;
    spans.count = 0;
    U_TRY(ukf_matrix(&spans, weights, 0, 1, sizeof(U_WORK), U_ALIGNOF(U_WORK), UKF_INPUT));
    U_CHECK_FINITE(weights);
    U_TRY(U_LOCAL(nonnegative)(weights));
    w = (const U_WORK *)weights->data;
    count = weights->rows;
    for (i = 0; i < count; ++i) {
#if U_FIXED
        U_TRY(linears_fixed_add(sum, w[i], &sum));
        U_TRY(linears_fixed_mac(&accumulator, w[i], w[i]));
#else
        sum += w[i];
        squares += w[i] * w[i];
        if (!isfinite(sum) || !isfinite(squares)) return LINEARS_ERROR_RANGE;
#endif
    }
    if (sum == 0) return LINEARS_ERROR_DOMAIN;
#if U_FIXED
    /* Exact F62 sum of squares rounded once; sum^2 / squares then rounds twice. */
    U_TRY(linears_accum_to_fixed(accumulator, &squares));
    U_TRY(linears_fixed_mul(sum, sum, &result));
    U_TRY(linears_fixed_div(result, squares, &result));
#else
    result = sum * sum / squares;
    if (!isfinite(result)) return LINEARS_ERROR_RANGE;
#endif
    *out = result;
    return LINEARS_OK;
}

linears_status_t U_API(resample_indices)(const linears_matrix_t *weights, U_WORK uniform,
                                         size_t *indices, size_t capacity)
{
    linears_status_t status;
    ukf_spans_t spans;
    const U_WORK *w;
    U_WORK total = U_ZERO, cumulative, target;
    size_t count, i, j;
#if U_FIXED
    linears_fixed_t divisor, position;
#endif
    spans.count = 0;
    U_TRY(ukf_matrix(&spans, weights, 0, 1, sizeof(U_WORK), U_ALIGNOF(U_WORK), UKF_INPUT));
    count = weights->rows;
    if (indices == NULL) return LINEARS_ERROR_ARGUMENT;
    if (capacity < count) return LINEARS_ERROR_WORKSPACE;
    U_TRY(ukf_span(&spans, indices, LINEARS_SIZE_MUL(count, sizeof(size_t)), U_ALIGNOF(size_t), UKF_WRITABLE));
    U_TRY(ukf_disjoint(&spans));
    U_CHECK_FINITE(weights);
#if !U_FIXED
    if (!isfinite(uniform)) return LINEARS_ERROR_RANGE;
    if (!(uniform >= 0 && uniform < 1)) return LINEARS_ERROR_DOMAIN;
#else
    if (uniform < 0 || uniform >= LINEARS_FIXED_ONE) return LINEARS_ERROR_DOMAIN;
#endif
    U_TRY(U_LOCAL(nonnegative)(weights));
    w = (const U_WORK *)weights->data;
    for (i = 0; i < count; ++i) {
#if U_FIXED
        U_TRY(linears_fixed_add(total, w[i], &total));
#else
        total += w[i];
        if (!isfinite(total)) return LINEARS_ERROR_RANGE;
#endif
    }
    if (total == 0) return LINEARS_ERROR_DOMAIN;
#if U_FIXED
    /* Targets grow with i, so the last one fitting int64 proves the others do. */
    U_TRY(linears_fixed_from_int((int64_t)count, &divisor));
    position = uniform + (linears_fixed_t)(count - 1) * LINEARS_FIXED_ONE;
    U_TRY(linears_fixed_muldiv(position, total, divisor, &target));
#endif
    /* Systematic resampling: one draw, N equally spaced targets, running sum. */
    j = 0;
    cumulative = w[0];
    for (i = 0; i < count; ++i) {
#if U_FIXED
        position = uniform + (linears_fixed_t)i * LINEARS_FIXED_ONE;
        U_TRY(linears_fixed_muldiv(position, total, divisor, &target));
#else
        target = ((U_WORK)i + uniform) / (U_WORK)count * total;
#endif
        while (cumulative < target && j + 1 < count) {
            ++j;
            cumulative += w[j]; /* Partial sums never exceed the checked total. */
        }
        indices[i] = j;
    }
    return LINEARS_OK;
}

linears_status_t U_API(gather)(const linears_matrix_t *points, const size_t *indices, size_t count,
                               linears_matrix_t *out)
{
    linears_status_t status;
    ukf_spans_t spans;
    linears_matrix_t source, target;
    size_t k, i;
    spans.count = 0;
    U_TRY(ukf_matrix(&spans, points, 0, 0, sizeof(U_WORK), U_ALIGNOF(U_WORK), UKF_INPUT));
    k = points->cols;
    if (indices == NULL) return LINEARS_ERROR_ARGUMENT;
    if (count == 0) return LINEARS_ERROR_DIMENSION;
    U_TRY(ukf_span(&spans, indices, LINEARS_SIZE_MUL(count, sizeof(size_t)), U_ALIGNOF(size_t), UKF_INPUT));
    U_TRY(ukf_matrix(&spans, out, count, k, sizeof(U_WORK), U_ALIGNOF(U_WORK), UKF_WRITABLE));
    U_TRY(ukf_disjoint(&spans));
    for (i = 0; i < count; ++i)
        if (indices[i] >= points->rows) return LINEARS_ERROR_ARGUMENT;
    for (i = 0; i < count; ++i) {
        source = U_SCRATCH(row)(points, indices[i]);
        target = U_SCRATCH(row)(out, i);
        memcpy(target.data, source.data, k * sizeof(U_WORK));
    }
    return LINEARS_OK;
}

#undef U_CHECK_FINITE
#undef U_BASIC
#undef U_TRY
#undef U_LINEAR
#undef U_SCRATCH
#undef U_LOCAL
#undef U_API
#undef U_JOIN
#undef U_JOIN_INNER
#undef U_TAG
#undef U_WORK
#undef U_FIXED
#undef U_WORKSPACE
#undef U_SAMPLE_FN
#undef U_ZERO
#undef U_HALF
#undef U_FINITE
#undef U_EXP_NEG
