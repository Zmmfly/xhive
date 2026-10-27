/* Q particle-filter arithmetic. The two-word values below are scalar integer
 * sums/products, not a wide fixed signal format or an expanded particle set. */
#ifndef FILTERS_PF_Q_INTEGER_HELPERS
#define FILTERS_PF_Q_INTEGER_HELPERS

typedef struct { uint64_t lo, hi; } pf_q_u128_t;
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

/* Full 64-by-64 multiplication using the native 32-by-32 products. */
static pf_q_u128_t pf_q_umul(uint64_t a, uint64_t b)
{
    const uint64_t mask = UINT64_C(0xffffffff);
    uint64_t a0 = a & mask, a1 = a >> 32, b0 = b & mask, b1 = b >> 32;
    uint64_t low = a0 * b0, middle = a1 * b0 + (low >> 32), carry = middle >> 32;
    pf_q_u128_t r;
    middle = (middle & mask) + a0 * b1;
    r.lo = (middle << 32) | (low & mask);
    r.hi = a1 * b1 + carry + (middle >> 32);
    return r;
}

/* The callers prove a*b fits 128 bits. */
static pf_q_u128_t pf_q_umul32(pf_q_u128_t a, uint32_t b)
{
    pf_q_u128_t r = pf_q_umul(a.lo, b);
    r.hi += a.hi * b;
    return r;
}

static unsigned pf_q_ubits(pf_q_u128_t a)
{
    uint64_t top = a.hi != 0 ? a.hi : a.lo;
    unsigned bits = a.hi != 0 ? 64 : 0;
    while (top != 0) { ++bits; top >>= 1; }
    return bits;
}

/* Round a positive product divided by 2^shift; the result fits 32 bits. */
static uint64_t pf_q_uround(pf_q_u128_t a, unsigned shift)
{
    uint64_t result, rounding;
    if (shift >= 64) {
        unsigned tail = shift - 64;
        result = a.hi >> tail;
        rounding = tail != 0 ? ((a.hi >> (tail - 1)) & 1) : (a.lo >> 63);
    } else {
        result = (a.lo >> shift) | (a.hi << (64 - shift));
        rounding = (a.lo >> (shift - 1)) & 1;
    }
    return result + rounding;
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
#define PQ_FINITE PQ_JOIN(ukf_finite_, U_KERNEL)
#define PQ_SCALE ((U_WORK)U_SCALE)

static U_WORK PQ(unsigned_value)(pf_q_u128_t value)
{
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
    U_WORK scaled = value * PQ_SCALE;
    return (int64_t)(scaled >= 0 ? scaled + (U_WORK)0.5 : scaled - (U_WORK)0.5);
}

/* Mixed dot products keep unit-range products in native Q integer MACs.
 * Standard-normal draws, solved residuals and arbitrary callback results
 * may exceed that range; only those products use the floating work type. */
static U_WORK PQ(dot)(const U_WORK *a, size_t astride, const U_WORK *b, size_t bstride, size_t count)
{
    pf_q_i128_t sum = {0, 0};
    U_WORK extended = 0;
    size_t i;
    for (i = 0; i < count; ++i) {
        U_WORK x = a[i * astride], y = b[i * bstride];
        if (x >= -1 && x <= 1 && y >= -1 && y <= 1) {
            int64_t p = PQ(raw)(x) * PQ(raw)(y);
            pf_q_iadd(&sum, pf_q_magnitude(p), p < 0);
        } else {
            extended += x * y;
        }
    }
    return extended + (PQ(signed_value)(sum) / PQ_SCALE) / PQ_SCALE;
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
    if (!isfinite(uniform)) return LINEARS_ERROR_RANGE;
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

/* The factor is a checked Cholesky lower triangle. Reusing it avoids the
 * old per-particle pivoted LU and its matrix-sized scratch. */
static linears_status_t PQ(gaussian)(const U_WORK *deviation, const U_WORK *lower, size_t k,
                                     U_WORK *solve, U_WORK *out)
{
    size_t i;
    U_WORK value;
    for (i = 0; i < k; ++i) {
        value = deviation[i] - PQ(dot)(lower + i * k, 1, solve, 1, i);
        solve[i] = value / lower[i * k + i];
        if (!isfinite(solve[i])) return LINEARS_ERROR_RANGE;
    }
    value = (U_WORK)0.5 * PQ(dot)(solve, 1, solve, 1, k);
    if (!isfinite(value)) return LINEARS_ERROR_RANGE;
    *out = value;
    return LINEARS_OK;
}

/* Likelihood precision covers both the input and output weight dynamic
 * ranges: F31 for q15, F63 for q31. These are scalar unsigned probabilities,
 * never persistent signals. Only exp and one common reciprocal are floating;
 * all weight products, maximum comparisons and normalization are integer. */
static linears_status_t PQ(reweight)(const linears_matrix_t *weights, U_WORK *exponents,
                                     U_SIGNAL *staged, size_t count)
{
    const U_SIGNAL *w = (const U_SIGNAL *)weights->data;
    pf_q_u128_t maximum = {0, 0};
    U_WORK lowest = 0, reciprocal;
    uint32_t multiplier;
    unsigned shift;
    size_t i;
    int found = 0;
    for (i = 0; i < count; ++i) {
        if (w[i] < 0) return LINEARS_ERROR_DOMAIN;
        if (!isfinite(exponents[i])) return LINEARS_ERROR_RANGE;
        if (w[i] != 0 && (!found || exponents[i] < lowest)) { lowest = exponents[i]; found = 1; }
    }
    if (!found) return LINEARS_ERROR_DOMAIN;
    for (i = 0; i < count; ++i) {
        pf_q_u128_t product;
        uint64_t likelihood;
        /* A zero-weight row must not overflow exp when its exponent is lower. */
        exponents[i] = w[i] == 0 ? 0 : U_EXP_NEG(exponents[i] - lowest);
        likelihood = (uint64_t)(exponents[i] * (U_WORK)U_LIKELIHOOD_SCALE + (U_WORK)0.5);
        product = pf_q_umul(likelihood, (uint64_t)w[i]);
        if (pf_q_ucmp(product, maximum) > 0) maximum = product;
    }
    shift = pf_q_ubits(maximum);
    reciprocal = U_LDEXP((U_WORK)U_SIGNAL_MAX / PQ(unsigned_value)(maximum), (int)shift);
    multiplier = (uint32_t)(reciprocal + (U_WORK)0.5);
    for (i = 0; i < count; ++i) {
        uint64_t likelihood = (uint64_t)(exponents[i] * (U_WORK)U_LIKELIHOOD_SCALE + (U_WORK)0.5);
        pf_q_u128_t product = pf_q_umul(likelihood, (uint64_t)w[i]);
        uint64_t value = pf_q_ucmp(product, maximum) == 0 ? (uint64_t)U_SIGNAL_MAX :
            pf_q_uround(pf_q_umul32(product, multiplier), shift);
        staged[i] = (U_SIGNAL)(value > (uint64_t)U_SIGNAL_MAX ? (uint64_t)U_SIGNAL_MAX : value);
    }
    return LINEARS_OK;
}

#undef PQ_SCALE
#undef PQ_FINITE
#undef PQ
#undef PQ_JOIN
#undef PQ_JOIN_INNER
