/* Dedicated mixed Q UKF. Only callback rows, extended samples, factors and
 * gain rows use U_WORK; ordinary statistics accumulate raw integer products.
 * U_BITS is a compile-time signal format, never dynamic scaling metadata. */
#include "q_workspace_impl.h"

#define Q_JOIN_INNER(a, b) a##b
#define Q_JOIN(a, b) Q_JOIN_INNER(a, b)
#define Q_API(name) Q_JOIN(Q_JOIN(filters_ukf_, U_TAG), Q_JOIN(_, name))
#define Q_CORE(name) Q_JOIN(Q_JOIN(ukf_work_, U_TAG), Q_JOIN(_, name))
#define Q_LOCAL(name) Q_JOIN(Q_JOIN(ukf_q_, U_TAG), Q_JOIN(_, name))
#define Q_SCRATCH(name) Q_JOIN(Q_JOIN(ukf_work_, U_TAG), Q_JOIN(_, name))
#define Q_CHOLESKY Q_JOIN(ukf_cholesky_, U_FLOAT_TAG)
#define Q_WEIGHTS_VALID Q_JOIN(ukf_weights_valid_, U_FLOAT_TAG)
#define Q_TRY(expression) do { status = (expression); if (status != LINEARS_OK) return status; } while (0)

/* Column summary tags (see Q_LOCAL(quantize)) share the byte width of
 * U_WORK, so a tag occupies one slot of the third metadata vector. */
#if U_BITS == 15
#define Q_SIGNAL_MAX INT16_MAX
#define Q_EPSILON FLT_EPSILON
typedef uint32_t Q_LOCAL(tag_t);
#define Q_TAG_SATURATED (UINT32_C(1) << 20)
typedef int64_t Q_LOCAL(accum_t);
#define Q_ACCUM_ZERO INT64_C(0)
static inline void Q_LOCAL(mac)(Q_LOCAL(accum_t) *sum, int64_t product)
{
    /* The validated n*n floating workspace bounds 2*n+1 products, each
     * <=2^31 even for the difference dot below, inside signed int64_t. */
    *sum += product;
}
static inline U_WORK Q_LOCAL(accum_value)(Q_LOCAL(accum_t) sum)
{
    return (U_WORK)sum * (U_WORK)0x1p-30;
}
#else
#define Q_SIGNAL_MAX INT32_MAX
#define Q_EPSILON DBL_EPSILON
typedef uint64_t Q_LOCAL(tag_t);
#define Q_TAG_SATURATED (UINT64_C(1) << 40)
typedef linears_accum_t Q_LOCAL(accum_t);
#define Q_ACCUM_ZERO {UINT64_C(0), UINT64_C(0)}
static inline void Q_LOCAL(mac)(Q_LOCAL(accum_t) *sum, int64_t product)
{
    uint64_t old = sum->low;
    sum->low += (uint64_t)product;
    sum->high += (product < 0 ? UINT64_MAX : UINT64_C(0)) + (sum->low < old);
}
static inline U_WORK Q_LOCAL(accum_value)(Q_LOCAL(accum_t) sum)
{
    int negative = (int)(sum.high >> 63);
    U_WORK value;
    if (negative) {
        sum.low = ~sum.low + UINT64_C(1);
        sum.high = ~sum.high + (sum.low == 0);
    }
    if (sum.high == 0) value = (U_WORK)sum.low * (U_WORK)0x1p-62;
    else value = ((U_WORK)sum.high * (U_WORK)0x1p64 + (U_WORK)sum.low) * (U_WORK)0x1p-62;
    return negative ? -value : value;
}
#endif

static U_WORK Q_LOCAL(dot)(const U_SIGNAL *a, size_t astride, const U_SIGNAL *b,
                            size_t bstride, size_t count)
{
    Q_LOCAL(accum_t) sum = Q_ACCUM_ZERO;
    size_t i;
    for (i = 0; i < count; ++i)
        Q_LOCAL(mac)(&sum, (int64_t)a[i * astride] * b[i * bstride]);
    return Q_LOCAL(accum_value)(sum);
}

/* For a regenerated sigma pair, dx+ = L and dx- = -L. Raw q31 times a
 * difference of two q31 values still fits int64_t; only the sum needs limbs. */
static U_WORK Q_LOCAL(dot_difference)(const U_SIGNAL *a, const U_SIGNAL *positive,
                                       const U_SIGNAL *negative, size_t stride, size_t count)
{
    Q_LOCAL(accum_t) sum = Q_ACCUM_ZERO;
    size_t i;
    for (i = 0; i < count; ++i) {
        int64_t difference = (int64_t)positive[i * stride] - negative[i * stride];
        Q_LOCAL(mac)(&sum, (int64_t)a[i] * difference);
    }
    return Q_LOCAL(accum_value)(sum);
}

/* Three k-element metadata vectors summarize each sample column (or factor
 * row): maximum magnitude, weighted conversion error, and an integer tag
 * stored in the bytes of the third vector's slot. The error is the sum the
 * exact bound below needs; it is negative until a pair beyond the integer
 * screen first requests it, because ordinary pairs never read it. An
 * out-of-range sample is not clipped or rejected; its column keeps the
 * original floating calculation. */
typedef char Q_LOCAL(tag_fits_slot)[sizeof(Q_LOCAL(tag_t)) == sizeof(U_WORK) ? 1 : -1];
#define Q_TAG_ONE ((Q_LOCAL(tag_t))1 << U_BITS)

static inline Q_LOCAL(tag_t) Q_LOCAL(tag)(const U_WORK *slot)
{
    Q_LOCAL(tag_t) tag;
    memcpy(&tag, slot, sizeof(tag));
    return tag;
}

/* Quantize one sample and fold it into a running summary. Tag 0 marks a
 * summary that must keep floating products: a sample outside [-1,+1], or a
 * nonzero sub-LSB sample that would collapse a positive working covariance
 * needed by the next UPF solve. Otherwise the tag is max|raw| + 1, with
 * Q_TAG_SATURATED once a sample rounded to the positive maximum, whose cell
 * may carry a whole LSB of conversion error. */
static void Q_LOCAL(quantize)(U_WORK value, U_SIGNAL *raw, U_WORK *maximum, Q_LOCAL(tag_t) *tag)
{
    Q_LOCAL(tag_t) magnitude;
    if (Q_JOIN(ukf_abs_greater_, U_FLOAT_TAG)(value, *maximum)) *maximum = U_ABS(value);
    if (!U_IN_UNIT(value)) {
        *raw = 0;
        *tag = 0;
        return;
    }
    Q_JOIN(ukf_store_, U_TAG)(&value, 1, raw);
    if (*raw == 0 && value != (U_WORK)0) *tag = 0;
    if (*tag == 0) return;
    magnitude = (Q_LOCAL(tag_t))(*raw < 0 ? -(int64_t)*raw : (int64_t)*raw) + 1;
    if (*raw == Q_SIGNAL_MAX) *tag |= Q_TAG_SATURATED;
    if (magnitude > (*tag & ~Q_TAG_SATURATED)) *tag = (*tag & Q_TAG_SATURATED) | magnitude;
}

static void Q_LOCAL(summarize)(U_WORK *metadata, size_t count, size_t index, U_WORK maximum,
                               Q_LOCAL(tag_t) tag)
{
    metadata[index] = maximum;
    metadata[count + index] = (U_WORK)-1;
    memcpy(metadata + 2 * count + index, &tag, sizeof(tag));
}

static void Q_LOCAL(columns)(const linears_matrix_t *points, U_SIGNAL *raw, U_WORK *metadata)
{
    const U_WORK *values = (const U_WORK *)points->data;
    size_t i, j, count = points->rows, k = points->cols;
    for (j = 0; j < k; ++j) {
        U_WORK maximum = (U_WORK)0;
        Q_LOCAL(tag_t) tag = 1;
        for (i = 0; i < count; ++i)
            Q_LOCAL(quantize)(values[i * k + j], raw + i * k + j, &maximum, &tag);
        Q_LOCAL(summarize)(metadata, k, j, maximum, tag);
    }
}

/* The weighted conversion error of a valid column, in the original sum order. */
static void Q_LOCAL(column_error)(const linears_matrix_t *points, const U_SIGNAL *raw,
                                  const U_WEIGHTS *weights, U_WORK *metadata, size_t j)
{
    const U_WORK *values = (const U_WORK *)points->data;
    size_t i, k = points->cols;
    U_WORK error = (U_WORK)0;
    if (metadata[k + j] >= (U_WORK)0) return;
    for (i = 0; i < points->rows; ++i)
        error += U_ABS(i == 0 ? weights->covariance0 : weights->other) *
                 Q_JOIN(ukf_quantization_error_, U_TAG)(values[i * k + j], raw[i * k + j]);
    metadata[k + j] = error;
}

/* Error units of a deviation column: every nonsaturated sample rounds to
 * within half an LSB, so its weighted error is at most units*LSB. */
static U_WORK Q_LOCAL(column_units)(const U_WEIGHTS *weights, size_t count)
{
    return (U_ABS(weights->covariance0) + (U_WORK)(count - 1) * U_ABS(weights->other)) * (U_WORK)0.5;
}

/* Integer screen budget. With |values| <= (max|raw|+1)*LSB and errors at
 * most units*LSB, the exact bound below is at most
 * units*LSB^2*(tag_a + tag_b + 1). The limit keeps that within one LSB after
 * a relative margin for the floating evaluation of up to terms additions
 * in the exact error sums and bound, including directed rounding. */
static Q_LOCAL(tag_t) Q_LOCAL(limit)(U_WORK units, size_t terms)
{
    U_WORK scaled = units * ((U_WORK)1 + (U_WORK)(terms + 8) * (U_WORK)Q_EPSILON);
    /* Every nonsaturated pair sum is below 4*2^bits; saturated tags exceed it. */
    if (!(scaled > (U_WORK)0.25)) return Q_TAG_ONE * 4;
    return (Q_LOCAL(tag_t))((U_WORK)Q_TAG_ONE / scaled);
}

/* Select raw products for a pair of summaries from integer tags: 1 accepts
 * the raw-Q dot, 0 keeps floating products, and -1 defers to the exact
 * bound of product_fits. An accepted pair also passes the exact bound, so
 * the screen changes only how often the floating bound is evaluated.
 * |a| lies in [(max|raw|-1/2)*LSB, (max|raw|+1)*LSB], so integer brackets
 * settle the sub-LSB product scale test except within a narrow band. An
 * all-zero column has max|raw| == 0 and is never below that scale. */
static int Q_LOCAL(screen)(const U_WORK *a, size_t acount, size_t ai,
                           const U_WORK *b, size_t bcount, size_t bi, Q_LOCAL(tag_t) limit)
{
    Q_LOCAL(tag_t) at = Q_LOCAL(tag)(a + 2 * acount + ai), bt = Q_LOCAL(tag)(b + 2 * bcount + bi);
    Q_LOCAL(tag_t) ra, rb;
    if (at == 0 || bt == 0) return 0;
    ra = (at & ~Q_TAG_SATURATED) - 1;
    rb = (bt & ~Q_TAG_SATURATED) - 1;
    if (ra != 0 && rb != 0 && (ra - 1) * (rb - 1) < Q_TAG_ONE &&
        ((ra + 1) * (rb + 1) <= Q_TAG_ONE || a[ai] * b[bi] < U_LSB))
        return 0;
    return at + bt + 1 <= limit ? 1 : -1;
}

/* A covariance entry is allowed at most one signal LSB of conversion error.
 * That absolute budget cannot resolve a working covariance whose nonzero
 * product scale is itself below one signal LSB: retain floating products
 * there, even when each deviation rounds to a nonzero Q value. Both checks
 * are outside the MAC loop and select a local dot, never an entire filter.
 * screen has already applied the validity and scale checks; this is the
 * exact error bound for a pair beyond its integer budget, and both actual
 * errors must be present. */
static int Q_LOCAL(product_fits)(const U_WORK *a, size_t acount, size_t ai,
                                 const U_WORK *b, size_t bcount, size_t bi)
{
    U_WORK ae = a[acount + ai], be = b[bcount + bi];
    U_WORK bound = a[ai] * be + b[bi] * ae + U_LSB * (ae < be ? ae : be);
    return bound <= U_LSB;
}

static linears_status_t Q_LOCAL(mean)(const linears_matrix_t *points, const U_WEIGHTS *weights,
                                       U_RESIDUAL_FN residual, void *context, linears_matrix_t *out,
                                       U_SIGNAL *raw, U_WORK *temporary)
{
    const U_WORK *values = (const U_WORK *)points->data;
    U_WORK *result = (U_WORK *)out->data;
    size_t i, j, count = points->rows, k = points->cols;
    linears_status_t status;
    if (residual != NULL) {
        linears_matrix_t reference = Q_SCRATCH(row)(points, 0), delta = {temporary, k, 1};
        for (j = 0; j < k; ++j) result[j] = (U_WORK)0;
        for (i = 0; i < count; ++i) {
            linears_matrix_t point = Q_SCRATCH(row)(points, i);
            U_WORK weight = i == 0 ? weights->mean0 : weights->other;
            Q_TRY(residual(&point, &reference, &delta, context));
            Q_TRY(U_FINITE(&delta));
            for (j = 0; j < k; ++j) result[j] += weight * temporary[j];
        }
        for (j = 0; j < k; ++j) result[j] += values[j];
    } else {
        U_WORK bound = (U_WORK)0;
        U_WORK term_bound = U_ABS(weights->other) * (U_LSB * (U_WORK)0.5);
        int bounded_error;
        /* A nonsaturating cell contributes at most half an LSB. Repeated
         * positive additions preserve the original error sum's rounding
         * order, so monotonicity makes this a sufficient bound. */
        for (i = 1; i < count; ++i) bound += term_bound;
        bounded_error = bound <= U_LSB * (U_WORK)0.5;
        for (j = 0; j < k; ++j) {
            U_WORK error = (U_WORK)0;
            int valid = 1, finite_differences = 1, has_maximum = 0;
            /* Quantize offsets, not the absolute center. A sub-LSB point
             * cloud then retains its center instead of inventing a variance
             * from a prematurely rounded working mean. */
            for (i = 1; i < count; ++i) {
                U_WORK delta = values[i * k + j] - values[j];
                if (!U_ISFINITE(delta)) finite_differences = 0;
                if (!U_IN_UNIT(delta)) valid = 0;
                else {
                    Q_JOIN(ukf_store_, U_TAG)(&delta, 1, raw + i * k + j);
                    if (raw[i * k + j] == 0 && delta != (U_WORK)0) valid = 0;
                    if (bounded_error) {
                        if (raw[i * k + j] == Q_SIGNAL_MAX) has_maximum = 1;
                    } else {
                        error += U_ABS(weights->other) *
                                 Q_JOIN(ukf_quantization_error_, U_TAG)(delta, raw[i * k + j]);
                    }
                }
            }
            /* MAX can represent +1 with a full LSB of error. Such columns
             * need their actual error, in the original order, not a forced
             * floating fallback. Invalid columns cannot use the Q mean. */
            if (bounded_error && has_maximum && valid) {
                for (i = 1; i < count; ++i) {
                    U_WORK delta = values[i * k + j] - values[j];
                    error += U_ABS(weights->other) *
                             Q_JOIN(ukf_quantization_error_, U_TAG)(delta, raw[i * k + j]);
                }
            }
            if (valid && error <= U_LSB * (U_WORK)0.5) {
                int64_t sum = 0;
                for (i = 1; i < count; ++i) sum += raw[i * k + j];
                result[j] = values[j] + weights->other * ((U_WORK)sum * U_LSB);
            } else if (finite_differences) {
                U_WORK sum = (U_WORK)0;
                for (i = 1; i < count; ++i) sum += weights->other * (values[i * k + j] - values[j]);
                result[j] = values[j] + sum;
            } else {
                U_WORK sum = weights->mean0 * values[j];
                for (i = 1; i < count; ++i) sum += weights->other * values[i * k + j];
                result[j] = sum;
            }
        }
    }
    return U_FINITE(out);
}

/* Replace one dead sample row at a time; the residual callback always gets
 * disjoint input/output and the original sample, not an early Q quantization. */
static linears_status_t Q_LOCAL(deviations)(linears_matrix_t *points, const linears_matrix_t *mean,
                                            U_RESIDUAL_FN residual, void *context, U_WORK *temporary)
{
    const U_WORK *center = (const U_WORK *)mean->data;
    linears_status_t status;
    size_t i, j, k = points->cols;
    linears_matrix_t delta = {temporary, k, 1};
    for (i = 0; i < points->rows; ++i) {
        linears_matrix_t point = Q_SCRATCH(row)(points, i);
        if (residual != NULL) Q_TRY(residual(&point, mean, &delta, context));
        else for (j = 0; j < k; ++j) temporary[j] = ((const U_WORK *)point.data)[j] - center[j];
        Q_TRY(U_FINITE(&delta));
        memcpy(point.data, temporary, k * sizeof(U_WORK));
    }
    return LINEARS_OK;
}

static linears_status_t Q_LOCAL(covariance)(const linears_matrix_t *deviations, const U_SIGNAL *raw,
                                            U_WORK *metadata, const U_WEIGHTS *weights,
                                            Q_LOCAL(tag_t) limit, const linears_matrix_t *noise,
                                            linears_matrix_t *out)
{
    const U_WORK *values = (const U_WORK *)deviations->data, *q = (const U_WORK *)noise->data;
    U_WORK *result = (U_WORK *)out->data;
    size_t i, r, c, count = deviations->rows, k = deviations->cols;
    for (r = 0; r < k; ++r) {
        for (c = 0; c <= r; ++c) {
            U_WORK value;
            int fits = Q_LOCAL(screen)(metadata, k, r, metadata, k, c, limit);
            if (fits < 0) {
                Q_LOCAL(column_error)(deviations, raw, weights, metadata, r);
                Q_LOCAL(column_error)(deviations, raw, weights, metadata, c);
                fits = Q_LOCAL(product_fits)(metadata, k, r, metadata, k, c);
            }
            if (fits) {
                U_WORK first = Q_LOCAL(dot)(raw + r, k, raw + c, k, 1);
                U_WORK others = Q_LOCAL(dot)(raw + k + r, k, raw + k + c, k, count - 1);
                value = weights->covariance0 * first + weights->other * others;
            } else {
                value = (U_WORK)0;
                for (i = 0; i < count; ++i) {
                    U_WORK weight = i == 0 ? weights->covariance0 : weights->other;
                    value += values[i * k + r] * (weight * values[i * k + c]);
                }
            }
            value += q[r * k + c];
            if (!U_ISFINITE(value)) return LINEARS_ERROR_RANGE;
            result[r * k + c] = result[c * k + r] = value;
        }
    }
    return LINEARS_OK;
}

static linears_status_t Q_LOCAL(factor)(const linears_matrix_t *covariance, U_WORK scale,
                                        linears_matrix_t *scaled, linears_matrix_t *factor)
{
    size_t i, count = covariance->rows * covariance->cols;
    const U_WORK *values = (const U_WORK *)covariance->data;
    U_WORK *target = (U_WORK *)scaled->data;
    for (i = 0; i < count; ++i) {
        target[i] = scale * values[i];
        if (!U_ISFINITE(target[i])) return LINEARS_ERROR_RANGE;
    }
    /* Exact symmetry of the scaled input follows from the validated input. */
    return Q_CHOLESKY(target, covariance->rows, (U_WORK *)factor->data);
}

static linears_status_t Q_LOCAL(propagate)(const linears_matrix_t *mean, const linears_matrix_t *factor,
                                           U_FN model, void *context, linears_matrix_t *samples,
                                           U_WORK *row_data)
{
    const U_WORK *center = (const U_WORK *)mean->data, *l = (const U_WORK *)factor->data;
    size_t i, j, n = mean->rows;
    linears_status_t status;
    linears_matrix_t row = {row_data, n, 1};
    for (i = 0; i < samples->rows; ++i) {
        linears_matrix_t target = Q_SCRATCH(row)(samples, i);
        for (j = 0; j < n; ++j) {
            U_WORK offset = i == 0 ? (U_WORK)0 : l[j * n + (i <= n ? i - 1 : i - n - 1)];
            row_data[j] = i <= n ? center[j] + offset : center[j] - offset;
        }
        Q_TRY(U_FINITE(&row));
        Q_TRY(model(&row, &target, context));
        Q_TRY(U_FINITE(&target));
    }
    return LINEARS_OK;
}

linears_status_t Q_CORE(predict_core)(const linears_matrix_t *mean, const linears_matrix_t *covariance,
                                      const linears_matrix_t *process_noise, const U_WEIGHTS *weights,
                                      U_FN transition, void *transition_context,
                                      U_RESIDUAL_FN residual, void *residual_context,
                                      linears_matrix_t *out_mean, linears_matrix_t *out_covariance,
                                      U_WORKSPACE workspace)
{
    linears_status_t status;
    linears_matrix_t scaled, factor, samples;
    U_WORK *cursor, *row, *temporary, *metadata;
    size_t n = mean->rows, s = FILTERS_UKF_SIGMA_COUNT(n);
    cursor = workspace.data;
    scaled = Q_SCRATCH(take)(&cursor, n, n);
    factor = Q_SCRATCH(take)(&cursor, n, n);
    samples = Q_SCRATCH(take)(&cursor, s, n);
    row = cursor; cursor += n;
    temporary = cursor; cursor += n;
    metadata = cursor;
    Q_TRY(Q_LOCAL(factor)(covariance, weights->scale, &scaled, &factor));
    Q_TRY(Q_LOCAL(propagate)(mean, &factor, transition, transition_context, &samples, row));
    Q_TRY(Q_LOCAL(mean)(&samples, weights, residual, residual_context, out_mean,
                         workspace.signal, temporary));
    Q_TRY(Q_LOCAL(deviations)(&samples, out_mean, residual, residual_context, temporary));
    Q_LOCAL(columns)(&samples, workspace.signal, metadata);
    return Q_LOCAL(covariance)(&samples, workspace.signal, metadata, weights,
                               Q_LOCAL(limit)(Q_LOCAL(column_units)(weights, s), s),
                               process_noise, out_covariance);
}

linears_status_t Q_API(predict_work)(const linears_matrix_t *mean, const linears_matrix_t *covariance,
                                     const linears_matrix_t *process_noise, const U_WEIGHTS *weights,
                                     U_FN transition, void *transition_context,
                                     U_RESIDUAL_FN residual, void *residual_context,
                                     linears_matrix_t *out_mean, linears_matrix_t *out_covariance,
                                     U_WORKSPACE workspace)
{
    linears_status_t status;
    ukf_spans_t spans;
    size_t n;
    if (transition == NULL) return LINEARS_ERROR_ARGUMENT;
    Q_TRY(Q_WEIGHTS_VALID(weights));
    spans.count = 0;
    Q_TRY(ukf_matrix(&spans, mean, 0, 1, sizeof(U_WORK), U_ALIGNOF(U_WORK), UKF_INPUT));
    n = mean->rows;
    Q_TRY(ukf_matrix(&spans, covariance, n, n, sizeof(U_WORK), U_ALIGNOF(U_WORK), UKF_INPUT));
    Q_TRY(ukf_matrix(&spans, process_noise, n, n, sizeof(U_WORK), U_ALIGNOF(U_WORK), UKF_INPUT));
    Q_TRY(ukf_matrix(&spans, out_mean, n, 1, sizeof(U_WORK), U_ALIGNOF(U_WORK), UKF_WRITABLE));
    Q_TRY(ukf_matrix(&spans, out_covariance, n, n, sizeof(U_WORK), U_ALIGNOF(U_WORK), UKF_WRITABLE));
    Q_TRY(Q_SCRATCH(workspace)(&spans, workspace, FILTERS_UKF_Q_PREDICT_WORK_STORAGE(n), 0,
                               FILTERS_UKF_Q_PREDICT_DECOMP_STORAGE(n), FILTERS_UKF_Q_PREDICT_SIGNAL_STORAGE(n)));
    Q_TRY(U_FINITE(mean)); Q_TRY(U_FINITE(covariance)); Q_TRY(U_FINITE(process_noise));
    Q_TRY(Q_JOIN(ukf_symmetric_, U_FLOAT_TAG)(covariance, 1));
    Q_TRY(Q_JOIN(ukf_symmetric_, U_FLOAT_TAG)(process_noise, 1));
    return Q_CORE(predict_core)(mean, covariance, process_noise, weights, transition, transition_context,
                                residual, residual_context, out_mean, out_covariance, workspace);
}

/* Cache the untransposed factor in Q, summarized per row like columns. */
static void Q_LOCAL(factor_signals)(const linears_matrix_t *factor, U_SIGNAL *raw, U_WORK *metadata)
{
    const U_WORK *values = (const U_WORK *)factor->data;
    size_t r, j, n = factor->rows;
    for (r = 0; r < n; ++r) {
        U_WORK maximum = (U_WORK)0;
        Q_LOCAL(tag_t) tag = 1;
        for (j = 0; j <= r; ++j) Q_LOCAL(quantize)(values[r * n + j], raw + r * n + j, &maximum, &tag);
        /* Cholesky writes exact zeros above the diagonal. */
        for (j = r + 1; j < n; ++j) raw[r * n + j] = 0;
        Q_LOCAL(summarize)(metadata, n, r, maximum, tag);
    }
}

/* Each factor row represents the paired state deviations +L[:,j], -L[:,j],
 * so its weighted error counts each entry twice. */
static void Q_LOCAL(row_error)(const linears_matrix_t *factor, const U_SIGNAL *raw, U_WORK other,
                               U_WORK *metadata, size_t r)
{
    const U_WORK *values = (const U_WORK *)factor->data;
    size_t j, n = factor->rows;
    U_WORK error = (U_WORK)0;
    if (metadata[n + r] >= (U_WORK)0) return;
    for (j = 0; j <= r; ++j)
        error += Q_JOIN(ukf_quantization_error_, U_TAG)(values[r * n + j], raw[r * n + j]);
    metadata[n + r] = ((U_WORK)2 * U_ABS(other)) * error;
}

static linears_status_t Q_LOCAL(cross)(const linears_matrix_t *factor, const U_SIGNAL *raw_factor,
                                       U_WORK *factor_metadata, const linears_matrix_t *deviations,
                                       const U_SIGNAL *raw_deviations, U_WORK *metadata,
                                       const U_WEIGHTS *weights, Q_LOCAL(tag_t) limit,
                                       linears_matrix_t *out)
{
    const U_WORK *l = (const U_WORK *)factor->data, *d = (const U_WORK *)deviations->data;
    U_WORK *result = (U_WORK *)out->data, weight = weights->other;
    size_t r, c, j, n = factor->rows, m = deviations->cols;
    for (r = 0; r < n; ++r) {
        for (c = 0; c < m; ++c) {
            U_WORK value;
            int fits = Q_LOCAL(screen)(factor_metadata, n, r, metadata, m, c, limit);
            if (fits < 0) {
                Q_LOCAL(row_error)(factor, raw_factor, weight, factor_metadata, r);
                Q_LOCAL(column_error)(deviations, raw_deviations, weights, metadata, c);
                fits = Q_LOCAL(product_fits)(factor_metadata, n, r, metadata, m, c);
            }
            if (fits) {
                value = weight * Q_LOCAL(dot_difference)(raw_factor + r * n,
                    raw_deviations + m + c, raw_deviations + (n + 1) * m + c, m, r + 1);
            } else {
                value = (U_WORK)0;
                for (j = 0; j < n; ++j) {
                    value += l[r * n + j] * (weight * d[(j + 1) * m + c]);
                    value -= l[r * n + j] * (weight * d[(j + n + 1) * m + c]);
                }
            }
            if (!U_ISFINITE(value)) return LINEARS_ERROR_RANGE;
            result[r * m + c] = value;
        }
    }
    return LINEARS_OK;
}

/* S = L*L^T was already checked SPD. Solve one C row without a second LU
 * factorization or a whole gain/transpose matrix. No Q range applies to K.
 * inverse holds the m reciprocal pivots. Each pivot is the square root of a
 * positive finite value, so its reciprocal is finite and normal; the shared
 * pivot division keeps each quotient's overflow status. */
static linears_status_t Q_LOCAL(gain_row)(const linears_matrix_t *factor, const U_WORK *inverse,
                                          const U_WORK *rhs, U_WORK *gain)
{
    const U_WORK *l = (const U_WORK *)factor->data;
    size_t i, j, m = factor->rows;
    for (i = 0; i < m; ++i) {
        U_WORK value = rhs[i];
        for (j = 0; j < i; ++j) value -= l[i * m + j] * gain[j];
        gain[i] = Q_JOIN(ukf_pivot_divide_, U_FLOAT_TAG)(value, inverse[i], l[i * m + i]);
        if (!U_ISFINITE(gain[i])) return LINEARS_ERROR_RANGE;
    }
    for (i = m; i-- > 0;) {
        U_WORK value = gain[i];
        for (j = i + 1; j < m; ++j) value -= l[j * m + i] * gain[j];
        gain[i] = Q_JOIN(ukf_pivot_divide_, U_FLOAT_TAG)(value, inverse[i], l[i * m + i]);
        if (!U_ISFINITE(gain[i])) return LINEARS_ERROR_RANGE;
    }
    return LINEARS_OK;
}

linears_status_t Q_CORE(update_core)(const linears_matrix_t *mean, const linears_matrix_t *covariance,
                                     const linears_matrix_t *measurement_noise,
                                     const linears_matrix_t *measurement, const U_WEIGHTS *weights,
                                     U_FN observation, void *observation_context,
                                     U_RESIDUAL_FN residual, void *residual_context,
                                     linears_matrix_t *out_mean, linears_matrix_t *out_covariance,
                                     U_WORKSPACE workspace)
{
    linears_status_t status;
    linears_matrix_t scaled, factor, samples, cross, zbar, innovation, s_matrix;
    U_WORK *cursor, *row, *temporary, *gain, *factor_metadata, *metadata, *inverse;
    U_SIGNAL *raw_factor, *raw_deviations;
    const U_WORK *x, *p;
    U_WORK *xout, *pout;
    Q_LOCAL(tag_t) limit;
    size_t n = mean->rows, m = measurement_noise->rows, s = FILTERS_UKF_SIGMA_COUNT(n);
    size_t k = FILTERS_KALMAN_MAX_SIZE(n, m), i, j, r;
    cursor = workspace.data;
    scaled = Q_SCRATCH(take)(&cursor, k, k);
    factor = Q_SCRATCH(take)(&cursor, k, k);
    samples = Q_SCRATCH(take)(&cursor, s, m);
    cross = Q_SCRATCH(take)(&cursor, n, m);
    row = cursor; cursor += n;
    temporary = cursor; cursor += m;
    zbar = Q_SCRATCH(take)(&cursor, m, 1);
    innovation = Q_SCRATCH(take)(&cursor, m, 1);
    gain = cursor; cursor += m;
    factor_metadata = cursor; cursor += 3 * n;
    metadata = cursor;
    raw_factor = workspace.signal;
    raw_deviations = workspace.signal + n * n;
    scaled.rows = scaled.cols = factor.rows = factor.cols = n;
    Q_TRY(Q_LOCAL(factor)(covariance, weights->scale, &scaled, &factor));
    Q_TRY(Q_LOCAL(propagate)(mean, &factor, observation, observation_context, &samples, row));
    Q_TRY(Q_LOCAL(mean)(&samples, weights, residual, residual_context, &zbar, raw_deviations, temporary));
    Q_TRY(Q_LOCAL(deviations)(&samples, &zbar, residual, residual_context, temporary));
    Q_LOCAL(columns)(&samples, raw_deviations, metadata);
    Q_LOCAL(factor_signals)(&factor, raw_factor, factor_metadata);
    /* A factor row's error is at most |other|*n LSB (n entries, each within
     * half an LSB and counted twice), never above the deviation columns'
     * |covariance0|/2 + n*|other|, so cross pairs share the column limit. */
    limit = Q_LOCAL(limit)(Q_LOCAL(column_units)(weights, s), s);
    Q_TRY(Q_LOCAL(cross)(&factor, raw_factor, factor_metadata, &samples, raw_deviations, metadata,
                          weights, limit, &cross));
    s_matrix = scaled;
    s_matrix.rows = s_matrix.cols = m;
    Q_TRY(Q_LOCAL(covariance)(&samples, raw_deviations, metadata, weights, limit, measurement_noise,
                              &s_matrix));
    factor.rows = factor.cols = m;
    Q_TRY(Q_CHOLESKY((const U_WORK *)s_matrix.data, m, (U_WORK *)factor.data));
    /* S is dead once factored; its first m slots hold the reciprocal pivots,
     * disjoint from the live factor, C rows and gain. */
    inverse = (U_WORK *)scaled.data;
    for (j = 0; j < m; ++j) inverse[j] = (U_WORK)1 / ((const U_WORK *)factor.data)[j * m + j];
    if (residual != NULL) Q_TRY(residual(measurement, &zbar, &innovation, residual_context));
    else for (j = 0; j < m; ++j)
        ((U_WORK *)innovation.data)[j] = ((const U_WORK *)measurement->data)[j] - ((const U_WORK *)zbar.data)[j];
    Q_TRY(U_FINITE(&innovation));
    x = (const U_WORK *)mean->data;
    p = (const U_WORK *)covariance->data;
    xout = (U_WORK *)out_mean->data;
    pout = (U_WORK *)out_covariance->data;
    for (r = 0; r < n; ++r) {
        U_WORK correction = (U_WORK)0;
        Q_TRY(Q_LOCAL(gain_row)(&factor, inverse, (const U_WORK *)cross.data + r * m, gain));
        for (j = 0; j < m; ++j) correction += gain[j] * ((const U_WORK *)innovation.data)[j];
        xout[r] = x[r] + correction;
        if (!U_ISFINITE(xout[r])) return LINEARS_ERROR_RANGE;
        /* K*C^T = C*S^-1*C^T is symmetric: form the lower triangle of row r
         * and mirror it, which also keeps the stored output exactly symmetric. */
        for (i = 0; i <= r; ++i) {
            U_WORK reduction = (U_WORK)0;
            for (j = 0; j < m; ++j) reduction += gain[j] * ((const U_WORK *)cross.data)[i * m + j];
            pout[r * n + i] = p[r * n + i] - reduction;
            if (!U_ISFINITE(pout[r * n + i])) return LINEARS_ERROR_RANGE;
            pout[i * n + r] = pout[r * n + i];
        }
    }
    return LINEARS_OK;
}

linears_status_t Q_API(update_work)(const linears_matrix_t *mean, const linears_matrix_t *covariance,
                                    const linears_matrix_t *measurement_noise,
                                    const linears_matrix_t *measurement, const U_WEIGHTS *weights,
                                    U_FN observation, void *observation_context,
                                    U_RESIDUAL_FN residual, void *residual_context,
                                    linears_matrix_t *out_mean, linears_matrix_t *out_covariance,
                                    U_WORKSPACE workspace)
{
    linears_status_t status;
    ukf_spans_t spans;
    size_t n, m, k;
    if (observation == NULL) return LINEARS_ERROR_ARGUMENT;
    Q_TRY(Q_WEIGHTS_VALID(weights));
    spans.count = 0;
    Q_TRY(ukf_matrix(&spans, mean, 0, 1, sizeof(U_WORK), U_ALIGNOF(U_WORK), UKF_INPUT));
    n = mean->rows;
    Q_TRY(ukf_matrix(&spans, covariance, n, n, sizeof(U_WORK), U_ALIGNOF(U_WORK), UKF_INPUT));
    Q_TRY(ukf_matrix(&spans, measurement_noise, 0, 0, sizeof(U_WORK), U_ALIGNOF(U_WORK), UKF_INPUT));
    m = measurement_noise->rows;
    if (measurement_noise->cols != m) return LINEARS_ERROR_DIMENSION;
    Q_TRY(ukf_matrix(&spans, measurement, m, 1, sizeof(U_WORK), U_ALIGNOF(U_WORK), UKF_INPUT));
    Q_TRY(ukf_matrix(&spans, out_mean, n, 1, sizeof(U_WORK), U_ALIGNOF(U_WORK), UKF_WRITABLE));
    Q_TRY(ukf_matrix(&spans, out_covariance, n, n, sizeof(U_WORK), U_ALIGNOF(U_WORK), UKF_WRITABLE));
    Q_TRY(Q_SCRATCH(workspace)(&spans, workspace, FILTERS_UKF_Q_UPDATE_WORK_STORAGE(n, m), 0,
                               FILTERS_UKF_Q_UPDATE_DECOMP_STORAGE(n, m), FILTERS_UKF_Q_UPDATE_SIGNAL_STORAGE(n, m)));
    Q_TRY(U_FINITE(mean)); Q_TRY(U_FINITE(covariance));
    Q_TRY(U_FINITE(measurement_noise)); Q_TRY(U_FINITE(measurement));
    Q_TRY(Q_JOIN(ukf_symmetric_, U_FLOAT_TAG)(covariance, 1));
    Q_TRY(Q_JOIN(ukf_symmetric_, U_FLOAT_TAG)(measurement_noise, 0));
    /* Validate R itself before any callback: S can be SPD when R is not. The
     * factor lands in the core's second k-by-k block and is overwritten. */
    k = FILTERS_KALMAN_MAX_SIZE(n, m);
    Q_TRY(Q_CHOLESKY((const U_WORK *)measurement_noise->data, m, workspace.data + k * k));
    return Q_CORE(update_core)(mean, covariance, measurement_noise, measurement, weights, observation,
                               observation_context, residual, residual_context, out_mean, out_covariance,
                               workspace);
}

#undef Q_TAG_ONE
#undef Q_TAG_SATURATED
#undef Q_EPSILON
#undef Q_ACCUM_ZERO
#undef Q_SIGNAL_MAX
#undef Q_TRY
#undef Q_WEIGHTS_VALID
#undef Q_CHOLESKY
#undef Q_SCRATCH
#undef Q_LOCAL
#undef Q_CORE
#undef Q_API
#undef Q_JOIN
#undef Q_JOIN_INNER
#undef U_TAG
#undef U_FLOAT_TAG
#undef U_SIGNAL
#undef U_BITS
#undef U_WORK
#undef U_LSB
#undef U_ABS
#undef U_WEIGHTS
#undef U_WORKSPACE
#undef U_FN
#undef U_RESIDUAL_FN
#undef U_FINITE
#undef U_ISFINITE
#undef U_IN_UNIT
