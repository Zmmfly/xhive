/* Exact integer moments for the default (subtraction) Q residual.
 * Include after covariance_product in pf_q_filter_impl.h. No floating value
 * is read or computed here; basic scratch retains n exact mean remainders,
 * while each covariance accumulator stays local until its raw result is staged.
 */
#include <limits.h>

#define PM(name) P_LOCAL(moment_##name)
#if U_BITS == 15
#define PM_WORDS 6u
#else
#define PM_WORDS 8u
#endif

/* Only the ambiguous final-rounding cell needs this width. For 64-bit size_t,
 * validated arrays bound the q15 numerator below 2^189 and the q31 numerator
 * below 2^251, including the centering correction. These are local rational
 * products, not widened signal arrays. */
typedef struct { uint32_t word[PM_WORDS]; } PM(wide_t);
typedef struct { uint64_t quotient; pf_q_u128_t remainder; } PM(division_t);

static unsigned PM(bits64)(uint64_t value)
{
#if (defined(__GNUC__) || defined(__clang__)) && UINT_MAX == UINT32_MAX
    uint32_t high = (uint32_t)(value >> 32);
    if (high != 0u) return 64u - (unsigned)__builtin_clz(high);
    return value == 0u ? 0u : 32u - (unsigned)__builtin_clz((uint32_t)value);
#else
    unsigned bits = 0u;
    while (value != 0u) { ++bits; value >>= 1; }
    return bits;
#endif
}

static unsigned PM(bits128)(pf_q_u128_t value)
{
    return value.hi != 0u ? 64u + PM(bits64)(value.hi) : PM(bits64)(value.lo);
}

/* All left shifts at this boundary are 0..32; callers prove no lost top bit. */
static pf_q_u128_t PM(left128)(pf_q_u128_t value, unsigned shift)
{
    pf_q_u128_t result;
    if (shift == 0u) return value;
    result.lo = value.lo << shift;
    result.hi = (value.hi << shift) | (value.lo >> (64u - shift));
    return result;
}

static pf_q_u128_t PM(sub128)(pf_q_u128_t a, pf_q_u128_t b)
{
    pf_q_u128_t result;
    result.lo = a.lo - b.lo;
    result.hi = a.hi - b.hi - (a.lo < b.lo);
    return result;
}

static pf_q_u128_t PM(magnitude)(pf_q_i128_t value)
{
    pf_q_u128_t result = {value.lo, (uint64_t)value.hi};
    if (value.hi < 0) {
        result.lo = ~result.lo + UINT64_C(1);
        result.hi = ~result.hi + (result.lo == 0u);
    }
    return result;
}

/* The quotient is at most 2^31. Division visits only its significant bits,
 * never all 128 dividend bits. The denominator is a nonzero weight sum. */
static PM(division_t) PM(divide128)(pf_q_u128_t numerator, pf_q_u128_t denominator)
{
    PM(division_t) result;
    result.quotient = 0u;
    result.remainder = numerator;
    if (pf_q_ucmp(numerator, denominator) >= 0) {
        unsigned shift = PM(bits128)(numerator) - PM(bits128)(denominator);
        pf_q_u128_t divisor = PM(left128)(denominator, shift);
        uint64_t bit = UINT64_C(1) << shift;
        for (;;) {
            if (pf_q_ucmp(result.remainder, divisor) >= 0) {
                result.remainder = PM(sub128)(result.remainder, divisor);
                result.quotient |= bit;
            }
            if (bit == 1u) break;
            divisor.lo = (divisor.lo >> 1) | (divisor.hi << 63);
            divisor.hi >>= 1;
            bit >>= 1;
        }
    }
    return result;
}

static U_SIGNAL PM(signal)(uint64_t magnitude, int negative)
{
    if (negative) return (U_SIGNAL)-(int64_t)magnitude;
    return (U_SIGNAL)(magnitude == (uint64_t)U_SCALE ? (uint64_t)U_SIGNAL_MAX : magnitude);
}

/* S - mu*T, where mu is the actual staged raw mean. Its magnitude is <=T/2
 * because a weighted average of raw signals cannot exceed the raw endpoints.
 * The product fits 128 bits even on a 64-bit size_t host. */
static pf_q_i128_t PM(mean_remainder)(pf_q_i128_t sum, pf_q_u128_t total, U_SIGNAL mean)
{
    pf_q_u128_t magnitude = PM(magnitude)(sum);
    uint64_t raw = pf_q_magnitude((int64_t)mean);
    pf_q_u128_t product = pf_q_umul(total.lo, raw), remainder;
    pf_q_i128_t result;
    int smaller, negative;
    product.hi += total.hi * raw;
    smaller = pf_q_ucmp(magnitude, product) < 0;
    remainder = smaller ? PM(sub128)(product, magnitude) : PM(sub128)(magnitude, product);
    negative = (sum.hi < 0) != smaller;
    result.lo = negative ? UINT64_C(0) - remainder.lo : remainder.lo;
    result.hi = negative ? -(int64_t)remainder.hi - (remainder.lo != 0u) : (int64_t)remainder.hi;
    return result;
}

static int PM(compare_wide)(const PM(wide_t) *a, const PM(wide_t) *b)
{
    unsigned i;
    for (i = PM_WORDS; i-- > 0u;) {
        if (a->word[i] != b->word[i]) return a->word[i] < b->word[i] ? -1 : 1;
    }
    return 0;
}

static PM(wide_t) PM(multiply_wide)(pf_q_u128_t a, pf_q_u128_t b)
{
    const uint32_t x[4] = {(uint32_t)a.lo, (uint32_t)(a.lo >> 32), (uint32_t)a.hi, (uint32_t)(a.hi >> 32)};
    const uint32_t y[4] = {(uint32_t)b.lo, (uint32_t)(b.lo >> 32), (uint32_t)b.hi, (uint32_t)(b.hi >> 32)};
    PM(wide_t) result = {{0}};
    unsigned i, j;
    for (i = 0u; i < 4u; ++i) {
        uint64_t carry = 0u;
        for (j = 0u; j < 4u && i + j < PM_WORDS; ++j) {
            uint64_t value = (uint64_t)x[i] * y[j] + result.word[i + j] + carry;
            result.word[i + j] = (uint32_t)value;
            carry = value >> 32;
        }
        for (j = i + j; carry != 0u && j < PM_WORDS; ++j) {
            uint64_t value = (uint64_t)result.word[j] + carry;
            result.word[j] = (uint32_t)value;
            carry = value >> 32;
        }
    }
    return result;
}

static PM(wide_t) PM(left_wide)(PM(wide_t) value, unsigned shift)
{
    PM(wide_t) result;
    uint32_t carry = 0u;
    unsigned i;
    for (i = 0u; i < PM_WORDS; ++i) {
        uint64_t word = ((uint64_t)value.word[i] << shift) | carry;
        result.word[i] = (uint32_t)word;
        carry = (uint32_t)(word >> 32);
    }
    return result;
}

static void PM(right_wide)(PM(wide_t) *value)
{
    uint32_t carry = 0u;
    unsigned i;
    for (i = PM_WORDS; i-- > 0u;) {
        uint32_t word = value->word[i];
        value->word[i] = (word >> 1) | (carry << 31);
        carry = word & 1u;
    }
}

static PM(wide_t) PM(add_wide)(PM(wide_t) a, PM(wide_t) b)
{
    PM(wide_t) result;
    uint64_t carry = 0u;
    unsigned i;
    for (i = 0u; i < PM_WORDS; ++i) {
        uint64_t word = (uint64_t)a.word[i] + b.word[i] + carry;
        result.word[i] = (uint32_t)word;
        carry = word >> 32;
    }
    return result;
}

static PM(wide_t) PM(sub_wide)(PM(wide_t) a, PM(wide_t) b)
{
    PM(wide_t) result;
    uint64_t borrow = 0u;
    unsigned i;
    for (i = 0u; i < PM_WORDS; ++i) {
        uint64_t subtrahend = (uint64_t)b.word[i] + borrow;
        result.word[i] = (uint32_t)((uint64_t)a.word[i] - subtrahend);
        borrow = (uint64_t)a.word[i] < subtrahend;
    }
    return result;
}

static unsigned PM(bits_wide)(const PM(wide_t) *value)
{
    unsigned i;
    for (i = PM_WORDS; i-- > 0u;)
        if (value->word[i] != 0u) return i * 32u + PM(bits64)(value->word[i]);
    return 0u;
}

/* Rare exact rounding near a half-cell. Preserve the q31 per-product rounding
 * already performed by covariance_product; only its final rational conversion
 * changes from floating arithmetic to integer division. */
static linears_status_t PM(covariance_wide)(pf_q_i128_t sum, pf_q_u128_t total,
                                           pf_q_i128_t first, pf_q_i128_t second, U_SIGNAL *out)
{
    PM(wide_t) numerator = PM(multiply_wide)(PM(magnitude)(sum), total);
    PM(wide_t) correction = PM(multiply_wide)(PM(magnitude)(first), PM(magnitude)(second));
    PM(wide_t) denominator = PM(left_wide)(PM(multiply_wide)(total, total), U_BITS);
    PM(wide_t) limit, remainder;
    uint64_t quotient = 0u;
    int negative = sum.hi < 0;
    int correction_negative = (first.hi < 0) != (second.hi < 0);
#if U_BITS == 31
    numerator = PM(left_wide)(numerator, 31u);
#endif
    if (negative != correction_negative) numerator = PM(add_wide)(numerator, correction);
    else if (PM(compare_wide)(&numerator, &correction) >= 0)
        numerator = PM(sub_wide)(numerator, correction);
    else {
        numerator = PM(sub_wide)(correction, numerator);
        negative = !negative;
    }
    limit = PM(left_wide)(denominator, U_BITS);
    if (PM(compare_wide)(&numerator, &limit) > 0) return LINEARS_ERROR_RANGE;
    remainder = numerator;
    if (PM(compare_wide)(&numerator, &denominator) >= 0) {
        unsigned shift = PM(bits_wide)(&numerator) - PM(bits_wide)(&denominator);
        PM(wide_t) divisor = PM(left_wide)(denominator, shift);
        uint64_t bit = UINT64_C(1) << shift;
        for (;;) {
            if (PM(compare_wide)(&remainder, &divisor) >= 0) {
                remainder = PM(sub_wide)(remainder, divisor);
                quotient |= bit;
            }
            if (bit == 1u) break;
            PM(right_wide)(&divisor);
            bit >>= 1;
        }
    }
    remainder = PM(left_wide)(remainder, 1u);
    if (PM(compare_wide)(&remainder, &denominator) >= 0) ++quotient;
    *out = PM(signal)(quotient, negative);
    return LINEARS_OK;
}

static linears_status_t PM(covariance)(pf_q_i128_t sum, pf_q_u128_t total,
                                      pf_q_i128_t first, pf_q_i128_t second, U_SIGNAL *out)
{
    pf_q_u128_t magnitude = PM(magnitude)(sum);
    pf_q_u128_t denominator = total;
    PM(division_t) division;
    pf_q_u128_t twice, distance, guard;
    uint64_t rounded;
#if U_BITS == 15
    denominator = PM(left128)(denominator, 15u);
#endif
    if (pf_q_ucmp(magnitude, PM(left128)(denominator, U_BITS)) > 0)
        return PM(covariance_wide)(sum, total, first, second, out);
    division = PM(divide128)(magnitude, denominator);
    twice = PM(left128)(division.remainder, 1u);
    rounded = division.quotient + (pf_q_ucmp(twice, denominator) >= 0);
    if ((first.lo == 0u && first.hi == 0) || (second.lo == 0u && second.hi == 0)) {
        *out = PM(signal)(rounded, sum.hi < 0);
        return LINEARS_OK;
    }
    /* |(S_j/T-mu_j)*(S_k/T-mu_k)/2^B| <= 1/(4*2^B) raw
     * cells. Away from a half-cell this cannot change nearest rounding.
     * The guard fits 128 bits: q15 <2^109, q31 <2^125. */
    distance = pf_q_ucmp(twice, denominator) >= 0 ? PM(sub128)(twice, denominator)
                                                 : PM(sub128)(denominator, twice);
    guard = PM(left128)(distance, U_BITS + 1u);
    if (rounded < (uint64_t)U_SCALE && pf_q_ucmp(guard, denominator) > 0) {
        *out = PM(signal)(rounded, sum.hi < 0);
        return LINEARS_OK;
    }
    return PM(covariance_wide)(sum, total, first, second, out);
}

/**
 * @brief Estimate ordinary Q moments using integer sums and rational rounding.
 *
 * The caller has validated all views, capacities, aliases and nonnegative
 * weights; total is nonzero. basic needs only 2*n units for the remainders,
 * which fits its existing 2*n*n allocation for every validated n>=1.
 * Output arrays are committed only after all staged values pass their bounds.
 */
static linears_status_t P_LOCAL(estimate_integer)(const U_STATE *state, linears_matrix_t *mean,
                                                 linears_matrix_t *covariance, U_WORKSPACE workspace,
                                                 pf_q_u128_t total)
{
    const U_SIGNAL *particles = (const U_SIGNAL *)state->particles.data;
    const U_SIGNAL *weights = (const U_SIGNAL *)state->weights.data;
    size_t count = state->particles.rows, n = state->particles.cols, i, j, k;
    U_SIGNAL *mean_stage = workspace.signal, *covariance_stage = workspace.signal + n;
    for (j = 0u; j < n; ++j) {
        pf_q_i128_t sum = {0u, 0};
        pf_q_u128_t magnitude;
        PM(division_t) division;
        uint64_t rounded;
#if U_BITS == 15
        /* Each absolute term is <=32768*32767. This exact count bound also
         * covers all 32-bit size_t targets without a runtime-wide sum. */
        if ((uint64_t)count <= (uint64_t)INT64_MAX / (UINT64_C(32768) * UINT64_C(32767))) {
            int64_t narrow = 0;
            for (i = 0u; i < count; ++i)
                narrow += (int64_t)particles[i * n + j] * weights[i];
            sum.lo = (uint64_t)narrow;
            sum.hi = narrow < 0 ? -1 : 0;
        } else
#endif
        {
            for (i = 0u; i < count; ++i) {
                int64_t value = particles[i * n + j];
                pf_q_iadd(&sum, pf_q_magnitude(value) * (uint64_t)weights[i], value < 0);
            }
        }
        magnitude = PM(magnitude)(sum);
        if (pf_q_ucmp(magnitude, PM(left128)(total, U_BITS)) > 0) return LINEARS_ERROR_RANGE;
        division = PM(divide128)(magnitude, total);
        rounded = division.quotient + (pf_q_ucmp(PM(left128)(division.remainder, 1u), total) >= 0);
        mean_stage[j] = PM(signal)(rounded, sum.hi < 0);
        pf_q_acc_write(workspace.basic + 2u * j, PM(mean_remainder)(sum, total, mean_stage[j]));
    }
    for (j = 0u; j < n; ++j) {
        pf_q_i128_t first = pf_q_acc_read(workspace.basic + 2u * j);
        for (k = 0u; k <= j; ++k) {
            pf_q_i128_t sum = {0u, 0};
            linears_status_t status;
#if U_BITS == 15
            /* Raw residuals have magnitude <=65535; their full 30-bit
             * product is retained. N<=65540 is the exact signed-64 bound. */
            if ((uint64_t)count <= (uint64_t)INT64_MAX /
                                      (UINT64_C(65535) * UINT64_C(65535) * UINT64_C(32767))) {
                int64_t narrow = 0;
                for (i = 0u; i < count; ++i) {
                    int32_t a = (int32_t)particles[i * n + j] - mean_stage[j];
                    int32_t b = (int32_t)particles[i * n + k] - mean_stage[k];
                    narrow += (int64_t)a * b * weights[i];
                }
                sum.lo = (uint64_t)narrow;
                sum.hi = narrow < 0 ? -1 : 0;
            } else
#endif
            {
                for (i = 0u; i < count; ++i) {
                    int64_t a = (int64_t)particles[i * n + j] - mean_stage[j];
                    int64_t b = (int64_t)particles[i * n + k] - mean_stage[k];
                    pf_q_iadd(&sum, P_LOCAL(covariance_product)(a, b, weights[i]), (a < 0) != (b < 0));
                }
            }
            status = PM(covariance)(sum, total, first, pf_q_acc_read(workspace.basic + 2u * k),
                                     covariance_stage + j * n + k);
            if (status != LINEARS_OK) return status;
            covariance_stage[k * n + j] = covariance_stage[j * n + k];
        }
    }
    memcpy(mean->data, mean_stage, n * sizeof(U_SIGNAL));
    memcpy(covariance->data, covariance_stage, n * n * sizeof(U_SIGNAL));
    return LINEARS_OK;
}

#undef PM_WORDS
#undef PM
