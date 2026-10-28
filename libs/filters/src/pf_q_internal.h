/* Q particle-filter arithmetic. The two-word values below are scalar integer
 * sums/products, not a wide fixed signal format or an expanded particle set. */
#ifndef FILTERS_PF_Q_INTEGER_HELPERS
#define FILTERS_PF_Q_INTEGER_HELPERS

#include "q_integer_wide.h"

typedef filters_q_u128_t pf_q_u128_t;
typedef struct { uint64_t lo; int64_t hi; } pf_q_i128_t;

static pf_q_u128_t pf_q_uadd(pf_q_u128_t a, pf_q_u128_t b)
{
    pf_q_u128_t r;
    r.lo = a.lo + b.lo;
    r.hi = a.hi + b.hi + (r.lo < a.lo);
    return r;
}

static int pf_q_ucmp(pf_q_u128_t a, pf_q_u128_t b)
{
    if (a.hi != b.hi) return a.hi < b.hi ? -1 : 1;
    return a.lo == b.lo ? 0 : (a.lo < b.lo ? -1 : 1);
}

/* Keep the PF-local spelling while sharing the exact scalar product. */
static pf_q_u128_t pf_q_umul(uint64_t a, uint64_t b)
{
    return filters_q_multiply_u64(a, b);
}

/* The callers prove a*b fits 128 bits. */
static pf_q_u128_t pf_q_umul32(pf_q_u128_t a, uint32_t b)
{
    pf_q_u128_t r = pf_q_umul(a.lo, b);
    r.hi += a.hi * b;
    return r;
}

/**
 * @brief Extend the shared magnitude bit count to a two-limb integer.
 *
 * @param[in] a Any unsigned 128-bit value.
 * @return unsigned Zero for zero, otherwise the significant bit count (1 to 128).
 */
static unsigned pf_q_ubits(pf_q_u128_t a)
{
    return filters_q_u128_bit_length(a);
}

/* PF callers prove that the rounded product fits uint64_t. */
static uint64_t pf_q_uround(pf_q_u128_t a, unsigned shift)
{
    return filters_q_round_u128(a, shift).value;
}

/* a/d is at most a raw weight. Only two such divisions are needed by a
 * systematic resampling call; the N-particle loop contains no division. */
static uint64_t pf_q_udiv(pf_q_u128_t a, uint64_t d, uint64_t *remainder)
{
    unsigned bit = pf_q_ubits(a);
    uint64_t r = 0, q = 0;
    while (bit-- != 0) {
        uint64_t digit = bit >= 64 ? ((a.hi >> (bit - 64)) & 1) : ((a.lo >> bit) & 1);
        /* d <= SIZE_MAX / sizeof(signal), hence 2*r+digit cannot overflow. */
        r = (r << 1) | digit;
        if (r >= d) { r -= d; if (bit < 64) q |= UINT64_C(1) << bit; }
    }
    *remainder = r;
    return q;
}

/* floor(a*mantissa/2^shift), retaining whether any fraction was discarded.
 * A binary float/double uniform is represented exactly by mantissa/2^shift. */
static pf_q_u128_t pf_q_uniform_product(pf_q_u128_t a, uint64_t mantissa,
                                        unsigned shift, int *fraction)
{
    pf_q_u128_t p = pf_q_umul(a.lo, mantissa), q = pf_q_umul(a.hi, mantissa), result = {0, 0};
    uint64_t words[3], output[2] = {0, 0};
    unsigned whole = shift / 64, tail = shift % 64, i;
    words[0] = p.lo;
    words[1] = p.hi + q.lo;
    words[2] = q.hi + (words[1] < p.hi);
    *fraction = 0;
    for (i = 0; i < 3 && i < whole; ++i) if (words[i] != 0) *fraction = 1;
    if (whole < 3 && tail != 0 && (words[whole] & ((UINT64_C(1) << tail) - 1)) != 0)
        *fraction = 1;
    for (i = 0; i < 2; ++i) {
        unsigned source = whole + i;
        if (source < 3) {
            output[i] = words[source] >> tail;
            if (tail != 0 && source + 1 < 3) output[i] |= words[source + 1] << (64 - tail);
        }
    }
    result.lo = output[0]; result.hi = output[1];
    return result;
}

static void pf_q_iadd(pf_q_i128_t *sum, uint64_t magnitude, int negative)
{
    uint64_t previous = sum->lo;
    if (negative) {
        sum->lo -= magnitude;
        sum->hi -= previous < magnitude;
    } else {
        sum->lo += magnitude;
        sum->hi += sum->lo < previous;
    }
}

static uint64_t pf_q_magnitude(int64_t value)
{
    return value < 0 ? (uint64_t)(-(value + 1)) + 1 : (uint64_t)value;
}

static pf_q_i128_t pf_q_acc_read(const linears_workspace_unit_t *storage)
{
    pf_q_i128_t result;
    memcpy(&result.lo, &storage[0].integer, sizeof(result.lo));
    result.hi = storage[1].integer;
    return result;
}

static void pf_q_acc_write(linears_workspace_unit_t *storage, pf_q_i128_t value)
{
    memcpy(&storage[0].integer, &value.lo, sizeof(value.lo));
    storage[1].integer = value.hi;
}

static void pf_q_acc_add(linears_workspace_unit_t *storage, uint64_t magnitude, int negative)
{
    pf_q_i128_t sum = pf_q_acc_read(storage);
    pf_q_iadd(&sum, magnitude, negative);
    pf_q_acc_write(storage, sum);
}
#endif

#define PQ_JOIN_INNER(a, b) a##b
#define PQ_JOIN(a, b) PQ_JOIN_INNER(a, b)
#define PQ(name) PQ_JOIN(PQ_JOIN(pf_q_, U_TAG), PQ_JOIN(_, name))
#define PQ_ISFINITE PQ_JOIN(ukf_isfinite_, U_KERNEL)
#define PQ_IN_UNIT PQ_JOIN(ukf_in_unit_, U_KERNEL)
#define PQ_SCALE ((U_WORK)U_SCALE)

static U_WORK PQ(unsigned_value)(pf_q_u128_t value)
{
    if (value.hi == 0) return (U_WORK)value.lo;
    return (U_WORK)value.hi * (U_WORK)18446744073709551616.0 + (U_WORK)value.lo;
}

static U_WORK PQ(signed_value)(pf_q_i128_t value)
{
    pf_q_u128_t magnitude;
    if (value.hi < 0) {
        magnitude.lo = ~value.lo + 1;
        magnitude.hi = ~(uint64_t)value.hi + (magnitude.lo == 0);
        return -PQ(unsigned_value)(magnitude);
    }
    magnitude.lo = value.lo; magnitude.hi = (uint64_t)value.hi;
    return PQ(unsigned_value)(magnitude);
}

/* A bounded operand is rounded once to its native Q precision. +1 is kept
 * in the scalar accumulator, rather than clipped to the stored maximum. */
static int64_t PQ(raw)(U_WORK value)
{
#if defined(__GNUC__) && defined(__arm__) && defined(__ARM_EABI__) && defined(__SOFTFP__) && \
    !defined(__ARM_FP) && !defined(__clang__) && \
    ((U_BITS == 15 && UKF_IEEE_BINARY32) || (U_BITS == 31 && UKF_IEEE_BINARY64))
    /* The verified GNU Arm libgcc soft add rounds to nearest-even without
     * observing or updating fenv. Hardware FP and other targets keep the
     * expression below, including its rounding-mode and exception effects.
     * For bounded inputs, scaling is exact. Only the predecessor of 0.5 LSB
     * double-rounds across an integer: its gap below 0.5 is half the gap
     * below 1, so adding 0.5 ties to the even 1. All other cells have equal
     * spacing below their half-integer threshold and target integer. */
#if U_BITS == 15
    uint32_t bits, magnitude, rounded;
    memcpy(&bits, &value, sizeof(bits));
    magnitude = bits & UINT32_C(0x7fffffff);
    if (magnitude < UINT32_C(0x37800000)) {
        rounded = magnitude == UINT32_C(0x377fffff);
    } else {
        unsigned shift = 135u - (magnitude >> 23);
        uint32_t mantissa = (magnitude & UINT32_C(0x007fffff)) | UINT32_C(0x00800000);
        rounded = (mantissa + (UINT32_C(1) << (shift - 1))) >> shift;
    }
    return bits >> 31 ? -(int32_t)rounded : (int32_t)rounded;
#else
    uint64_t bits, magnitude, rounded;
    memcpy(&bits, &value, sizeof(bits));
    magnitude = bits & UINT64_C(0x7fffffffffffffff);
    if (magnitude < UINT64_C(0x3df0000000000000)) {
        rounded = magnitude == UINT64_C(0x3defffffffffffff);
    } else {
        unsigned shift = 1044u - (unsigned)(magnitude >> 52);
        uint64_t mantissa = (magnitude & UINT64_C(0x000fffffffffffff)) | UINT64_C(0x0010000000000000);
        rounded = (mantissa + (UINT64_C(1) << (shift - 1))) >> shift;
    }
    return bits >> 63 ? -(int64_t)rounded : (int64_t)rounded;
#endif
#else
    U_WORK scaled = value * PQ_SCALE;
#if U_BITS == 15
    /* The rounded bounded operand is in [-32768,+32768]. Keep the same
     * floating addition before truncation, including its double rounding. */
    return (int32_t)(scaled >= 0 ? scaled + (U_WORK)0.5 : scaled - (U_WORK)0.5);
#else
    return (int64_t)(scaled >= 0 ? scaled + (U_WORK)0.5 : scaled - (U_WORK)0.5);
#endif
#endif
}

/* Mixed dot products keep unit-range products in native Q integer MACs.
 * Standard-normal draws, solved residuals and arbitrary callback results
 * may exceed that range; only those products use the floating work type. */
static U_WORK PQ(dot)(const U_WORK *a, size_t astride, const U_WORK *b, size_t bstride, size_t count)
{
#if U_BITS == 15
    /* count is a state/measurement dimension, not the particle count. Its
     * validated square work array bounds count below 2^31 on 32/64-bit
     * size_t targets; each bounded product is at most 2^30 in magnitude. */
    int64_t sum = 0;
#else
    pf_q_i128_t sum = {0, 0};
#endif
    U_WORK extended = 0, value;
    size_t i;
    for (i = 0; i < count; ++i) {
        U_WORK x = a[i * astride], y = b[i * bstride];
        if (PQ_IN_UNIT(x) && PQ_IN_UNIT(y)) {
            int64_t p = PQ(raw)(x) * PQ(raw)(y);
#if U_BITS == 15
            sum += p;
#else
            pf_q_iadd(&sum, pf_q_magnitude(p), p < 0);
#endif
        } else {
            extended += x * y;
        }
    }
#if U_BITS == 15
    value = (U_WORK)pf_q_magnitude(sum);
    if (sum < 0) value = -value;
#else
    value = PQ(signed_value)(sum);
#endif
    /* The converted integer and both old power-of-two divisions stay
     * normal (or zero), so one exact power-of-two scale gives the same bits. */
    return extended + value * ((U_WORK)1 / (PQ_SCALE * PQ_SCALE));
}

static linears_status_t PQ(arrays)(ukf_spans_t *spans, const U_STATE *state,
                                    size_t *count, size_t *n, size_t *m)
{
    linears_status_t status;
    *count = state->particles.rows; *n = state->particles.cols; *m = state->measurement_noise.rows;
#define PQ_OWN(field, rows, cols) do { \
    status = ukf_owned(spans, &state->field, (rows), (cols), state->field##_capacity, \
                       sizeof(U_SIGNAL), U_ALIGNOF(U_SIGNAL)); \
    if (status != LINEARS_OK) return status; \
} while (0)
    PQ_OWN(particles, *count, *n);
    PQ_OWN(weights, *count, 1);
    PQ_OWN(process_noise, *n, *n);
    PQ_OWN(measurement_noise, *m, *m);
#undef PQ_OWN
    return LINEARS_OK;
}

static linears_status_t PQ(state)(ukf_spans_t *spans, const U_STATE *state,
                                   size_t *count, size_t *n, size_t *m)
{
    linears_status_t status = ukf_span(spans, state, sizeof(*state), U_ALIGNOF(U_STATE), UKF_WRITABLE);
    return status == LINEARS_OK ? PQ(arrays)(spans, state, count, n, m) : status;
}

static void PQ(reset_weights)(linears_matrix_t *weights)
{
    U_SIGNAL *values = (U_SIGNAL *)weights->data;
    size_t i;
    for (i = 0; i < weights->rows; ++i) values[i] = U_SIGNAL_MAX;
}

static linears_status_t PQ(weight_total)(const U_SIGNAL *weights, size_t count, pf_q_u128_t *total)
{
    size_t i;
    pf_q_u128_t sum = {0, 0};
    for (i = 0; i < count; ++i) {
        pf_q_u128_t add;
        if (weights[i] < 0) return LINEARS_ERROR_DOMAIN;
        add.lo = (uint64_t)weights[i]; add.hi = 0;
        sum = pf_q_uadd(sum, add);
    }
    if (sum.lo == 0 && sum.hi == 0) return LINEARS_ERROR_DOMAIN;
    *total = sum;
    return LINEARS_OK;
}

/* Inputs have already been structurally validated by the caller. All value
 * checks finish before indices are touched. Integer quotient/remainder
 * recurrence implements the exact binary uniform, including boundary ties. */
static linears_status_t PQ(resample_indices)(const linears_matrix_t *weights, U_WORK uniform,
                                             size_t *indices, size_t capacity)
{
    const U_SIGNAL *w = (const U_SIGNAL *)weights->data;
    pf_q_u128_t total, initial, cumulative = {0, 0}, floor_target = {0, 0};
    uint64_t remainder, step_remainder, step, start, mantissa = 0;
    int exponent = 0, fraction = 0;
    unsigned shift = 0;
    size_t i, j = 0, count = weights->rows;
    linears_status_t status;
    if (indices == NULL) return LINEARS_ERROR_ARGUMENT;
    if (capacity < count) return LINEARS_ERROR_WORKSPACE;
    if (!PQ_ISFINITE(uniform)) return LINEARS_ERROR_RANGE;
    if (!(uniform >= 0 && uniform < 1)) return LINEARS_ERROR_DOMAIN;
    status = PQ(weight_total)(w, count, &total);
    if (status != LINEARS_OK) return status;
    if (uniform != 0) {
        U_WORK fraction_value = U_FREXP(uniform, &exponent);
        mantissa = (uint64_t)U_LDEXP(fraction_value, U_MANT_DIG);
        shift = (unsigned)(U_MANT_DIG - exponent);
    }
    initial = pf_q_uniform_product(total, mantissa, shift, &fraction);
    start = pf_q_udiv(initial, (uint64_t)count, &remainder);
    step = pf_q_udiv(total, (uint64_t)count, &step_remainder);
    floor_target.lo = start;
    cumulative.lo = (uint64_t)w[0];
    for (i = 0; i < count; ++i) {
        pf_q_u128_t target = floor_target, increment = {step, 0};
        if (remainder != 0 || fraction) {
            pf_q_u128_t one = {1, 0};
            target = pf_q_uadd(target, one);
        }
        while (pf_q_ucmp(cumulative, target) < 0 && j + 1 < count) {
            pf_q_u128_t add = {(uint64_t)w[++j], 0};
            cumulative = pf_q_uadd(cumulative, add);
        }
        indices[i] = j;
        remainder += step_remainder;
        if (remainder >= count) { remainder -= count; ++increment.lo; }
        floor_target = pf_q_uadd(floor_target, increment);
    }
    return LINEARS_OK;
}

#include "pf_q_gaussian_impl.h"

/* Original mixed triangular path, retained for local numerical retries. The
 * factor is a checked Cholesky lower triangle; no callback is repeated. */
static linears_status_t PQ(gaussian_mixed)(const U_WORK *deviation, const U_WORK *lower, size_t k,
                                           U_WORK *solve, U_WORK *out)
{
    size_t i;
    U_WORK value;
    for (i = 0; i < k; ++i) {
        value = deviation[i] - PQ(dot)(lower + i * k, 1, solve, 1, i);
        solve[i] = value / lower[i * k + i];
        if (!PQ_ISFINITE(solve[i])) return LINEARS_ERROR_RANGE;
    }
    value = (U_WORK)0.5 * PQ(dot)(solve, 1, solve, 1, k);
    if (!PQ_ISFINITE(value)) return LINEARS_ERROR_RANGE;
    *out = value;
    return LINEARS_OK;
}

static linears_status_t PQ(gaussian_retry)(const U_WORK *deviation, const U_WORK *lower, size_t k,
                                           U_WORK *solve, U_WORK *out, const U_WORK *raw_a,
                                           PQ(scale_t) sa, U_WORK *raw_b, U_WORK *raw_error)
{
    PQ(scale_t) sb;
    if (PQ(vector_prepare)(deviation, k, raw_b, &sb) &&
        PQ(gaussian_try)(raw_a, sa, raw_b, sb, k, solve, raw_error, out)) return LINEARS_OK;
    return PQ(gaussian_mixed)(deviation, lower, k, solve, out);
}

#include "pf_q_exp_impl.h"

/* Likelihood precision covers both the input and output weight dynamic
 * ranges: F31 for q15, F63 for q31. Fixed evaluation runs only inside its
 * proof envelope; every other case uses the original exponential locally. */
static uint64_t PQ(likelihood)(U_WORK delta)
{
#if U_BITS == 15
    uint32_t value;
    if (pf_q15_exp_neg_try(delta, &value)) return value;
#else
    uint64_t value;
    if (pf_q31_exp_neg_try(delta, &value)) return value;
#endif
    {
        U_WORK probability = U_EXP_NEG(delta);
        return (uint64_t)(probability * (U_WORK)U_LIKELIHOOD_SCALE + (U_WORK)0.5);
    }
}

#include "pf_q_likelihood_impl.h"
#include "pf_q_log_impl.h"

/* Once the exponent difference is consumed, its scratch slot holds raw
 * likelihood bytes. No pointer cast or floating read occurs in that lifetime. */
static void PQ(cache_likelihood)(U_WORK *slot, uint64_t likelihood)
{
#if U_BITS == 15
    uint32_t raw = (uint32_t)likelihood;
    memcpy(slot, &raw, sizeof(raw));
#else
    memcpy(slot, &likelihood, sizeof(likelihood));
#endif
}

static uint64_t PQ(cached_likelihood)(const U_WORK *slot)
{
#if U_BITS == 15
    uint32_t raw;
    memcpy(&raw, slot, sizeof(raw));
    return raw;
#else
    uint64_t raw;
    memcpy(&raw, slot, sizeof(raw));
    return raw;
#endif
}

static linears_status_t PQ(reweight)(const linears_matrix_t *weights, U_WORK *exponents,
                                     U_SIGNAL *staged, size_t count)
{
    const U_SIGNAL *w = (const U_SIGNAL *)weights->data;
    pf_q_u128_t maximum = {0, 0}, numerator;
    U_WORK lowest = 0;
    uint32_t multiplier;
    unsigned shift;
    size_t i;
    int found = 0;
    for (i = 0; i < count; ++i) {
        if (w[i] < 0) return LINEARS_ERROR_DOMAIN;
        if (!PQ_ISFINITE(exponents[i])) return LINEARS_ERROR_RANGE;
        if (w[i] != 0 && (!found || PQ(exponent_less)(exponents[i], lowest))) { lowest = exponents[i]; found = 1; }
    }
    if (!found) return LINEARS_ERROR_DOMAIN;
    for (i = 0; i < count; ++i) {
        pf_q_u128_t product;
        uint64_t likelihood;
        if (w[i] == 0) {
            likelihood = 0;
        } else {
            /* Keep both original operands for a zero/one-cell retry; the
             * ordinary difference and exponential stay on integer grids. */
            likelihood = PQ(likelihood_pair)(exponents[i], lowest);
        }
        PQ(cache_likelihood)(exponents + i, likelihood);
        product = pf_q_umul(likelihood, (uint64_t)w[i]);
        if (pf_q_ucmp(product, maximum) > 0) maximum = product;
    }
    shift = pf_q_ubits(maximum);
    /* max<2^46/2^94 and W<2^15/2^31 prove 64/128-bit numerator
     * and product bounds. The rounded shared quotient fits uint32_t. */
    numerator = pf_q_exp_shl((pf_q_u128_t){(uint64_t)U_SIGNAL_MAX, 0}, shift);
    multiplier = (uint32_t)pf_q_exp_round_div(numerator, maximum);
    for (i = 0; i < count; ++i) {
        uint64_t likelihood = PQ(cached_likelihood)(exponents + i);
        pf_q_u128_t product = pf_q_umul(likelihood, (uint64_t)w[i]);
        uint64_t value = pf_q_ucmp(product, maximum) == 0 ? (uint64_t)U_SIGNAL_MAX :
            pf_q_uround(pf_q_umul32(product, multiplier), shift);
        staged[i] = (U_SIGNAL)(value > (uint64_t)U_SIGNAL_MAX ? (uint64_t)U_SIGNAL_MAX : value);
    }
    return LINEARS_OK;
}

#undef PQ_SCALE
#undef PQ_IN_UNIT
#undef PQ_ISFINITE
#undef PQ
#undef PQ_JOIN
#undef PQ_JOIN_INNER
