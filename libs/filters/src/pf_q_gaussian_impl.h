/* Certified integer perturbation and Gaussian triangular arithmetic. Raw
 * integers live in dead U_WORK slots only through memcpy; the original
 * floating operands remain available for a local mixed-arithmetic retry. */
#include <float.h>
#include <math.h>

#include "pf_q_exp_impl.h"
#include "q_integer_bits.h"

#if U_BITS == 15
#define PQ_GAUSSIAN_F 30
#define PQ_GAUSSIAN_K 12
#define PQ_GAUSSIAN_ERROR_MAX UINT32_MAX
#else
#define PQ_GAUSSIAN_F 62
#define PQ_GAUSSIAN_K 30
#define PQ_GAUSSIAN_ERROR_MAX UINT64_MAX
#endif

typedef struct {
    int exponent; /* Original values are raw*2^(exponent-F). */
    int exact;    /* Every converted element round-trips exactly. */
    int diagonal; /* All off-diagonal source elements are exactly zero. */
#if U_BITS == 15
    uint32_t maximum; /* Q15-only native-width bounds; raw magnitudes fit 32 bits. */
#endif
} PQ(scale_t);

typedef struct {
    pf_q_u128_t magnitude;
    int negative;
} PQ(wide_t);

static void PQ(raw_store)(U_WORK *slot, int64_t value)
{
#if U_BITS == 15
    int32_t raw = (int32_t)value;
    memcpy(slot, &raw, sizeof(raw));
#else
    memcpy(slot, &value, sizeof(value));
#endif
}

static int64_t PQ(raw_load)(const U_WORK *slot)
{
#if U_BITS == 15
    int32_t raw;
    memcpy(&raw, slot, sizeof(raw));
    return raw;
#else
    int64_t raw;
    memcpy(&raw, slot, sizeof(raw));
    return raw;
#endif
}

static void PQ(error_store)(U_WORK *slot, uint64_t value)
{
#if U_BITS == 15
    uint32_t raw = (uint32_t)value;
    memcpy(slot, &raw, sizeof(raw));
#else
    memcpy(slot, &value, sizeof(value));
#endif
}

static uint64_t PQ(error_load)(const U_WORK *slot)
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

static pf_q_u128_t PQ(from_u64)(uint64_t value)
{
    pf_q_u128_t result = {value, 0};
    return result;
}

/* Q15 raw operands and Q32 errors are 32-bit, so their products need one
 * native 32-by-32 multiplication rather than the generic 64-by-64 helper. */
static pf_q_u128_t PQ(raw_product)(uint64_t a, uint64_t b)
{
#if U_BITS == 15
    return PQ(from_u64)(a * b);
#else
    return pf_q_umul(a, b);
#endif
}

static int PQ(add_unsigned)(pf_q_u128_t *sum, pf_q_u128_t value)
{
    pf_q_u128_t result = pf_q_uadd(*sum, value);
    if (result.hi < sum->hi || result.hi < value.hi) return 0;
    *sum = result;
    return 1;
}

static pf_q_u128_t PQ(sub_unsigned)(pf_q_u128_t a, pf_q_u128_t b)
{
    pf_q_u128_t result;
    result.lo = a.lo - b.lo;
    result.hi = a.hi - b.hi - (a.lo < b.lo);
    return result;
}

static int PQ(shift_left)(pf_q_u128_t value, unsigned shift, pf_q_u128_t *result)
{
    if (value.lo == 0 && value.hi == 0) {
        *result = value;
        return 1;
    }
    if (shift >= 128 || pf_q_ubits(value) + shift > 128) return 0;
    *result = pf_q_exp_shl(value, shift);
    return 1;
}

static pf_q_u128_t PQ(shift_right)(pf_q_u128_t value, unsigned shift)
{
    pf_q_u128_t result = {0, 0};
    if (shift == 0) return value;
    if (shift < 64) {
        result.lo = (value.lo >> shift) | (value.hi << (64 - shift));
        result.hi = value.hi >> shift;
    } else if (shift < 128) {
        result.lo = value.hi >> (shift - 64);
    }
    return result;
}

/* Add a signed magnitude without allowing a wrapped 128-bit accumulator. */
static int PQ(add_wide)(PQ(wide_t) *sum, pf_q_u128_t magnitude, int negative)
{
    if (magnitude.lo == 0 && magnitude.hi == 0) return 1;
    if (sum->magnitude.lo == 0 && sum->magnitude.hi == 0) {
        sum->magnitude = magnitude;
        sum->negative = negative;
        return 1;
    }
    if (sum->negative == negative) return PQ(add_unsigned)(&sum->magnitude, magnitude);
    {
        int comparison = pf_q_ucmp(sum->magnitude, magnitude);
        if (comparison == 0) {
            sum->magnitude.lo = sum->magnitude.hi = 0;
            sum->negative = 0;
        } else if (comparison > 0) {
            sum->magnitude = PQ(sub_unsigned)(sum->magnitude, magnitude);
        } else {
            sum->magnitude = PQ(sub_unsigned)(magnitude, sum->magnitude);
            sum->negative = negative;
        }
    }
    return 1;
}

static U_WORK PQ(wide_value)(PQ(wide_t) value)
{
    U_WORK result = PQ(unsigned_value)(value.magnitude);
    return value.negative ? -result : result;
}

/* Compare value*2^shift with one without rounding or overflowing. */
static int PQ(scaled_le_one)(pf_q_u128_t value, int shift)
{
    pf_q_u128_t bound;
    if (value.lo == 0 && value.hi == 0) return 1;
    if (shift > 0) return 0;
    if (shift == 0) return value.hi == 0 && value.lo <= 1;
    if (shift <= -128) return 1;
    bound = pf_q_exp_shl(PQ(from_u64)(1), (unsigned)-shift);
    return pf_q_ucmp(value, bound) <= 0;
}

#if (U_BITS == 15 && UKF_IEEE_BINARY32) || (U_BITS == 31 && UKF_IEEE_BINARY64)
typedef struct {
    uint64_t absolute, mantissa;
    int exponent, negative;
} PQ(ieee_t);

/* Share finite classification and exact mantissa/exponent extraction with
 * vector scaling. Positive IEEE bit patterns order finite magnitudes exactly. */
static int PQ(ieee_read)(U_WORK value, PQ(ieee_t) *parts)
{
#if U_BITS == 15
    uint32_t bits;
    unsigned encoded;
    memcpy(&bits, &value, sizeof(bits));
    parts->absolute = bits & UINT32_C(0x7fffffff);
    parts->negative = (int)(bits >> 31);
    encoded = (bits >> 23) & UINT32_C(0xff);
    if (encoded == 0xff) return 0;
    parts->mantissa = bits & UINT32_C(0x007fffff);
    parts->exponent = -126;
    if (encoded != 0) {
        parts->mantissa |= UINT32_C(0x00800000);
        parts->exponent = (int)encoded - 127;
    }
#else
    uint64_t bits;
    unsigned encoded;
    memcpy(&bits, &value, sizeof(bits));
    parts->absolute = bits & UINT64_C(0x7fffffffffffffff);
    parts->negative = (int)(bits >> 63);
    encoded = (unsigned)((bits >> 52) & UINT64_C(0x7ff));
    if (encoded == 0x7ff) return 0;
    parts->mantissa = bits & UINT64_C(0x000fffffffffffff);
    parts->exponent = -1022;
    if (encoded != 0) {
        parts->mantissa |= UINT64_C(0x0010000000000000);
        parts->exponent = (int)encoded - 1023;
    }
#endif
    return 1;
}

/* Called for a strictly positive finite magnitude selected by a prepare pass. */
static int PQ(ieee_floor)(uint64_t absolute)
{
#if U_BITS == 15
    unsigned encoded = (unsigned)(absolute >> 23);
    return encoded != 0 ? (int)encoded - 127 : -150 + (int)filters_q_bit_length(absolute);
#else
    unsigned encoded = (unsigned)(absolute >> 52);
    return encoded != 0 ? (int)encoded - 1023 : -1075 + (int)filters_q_bit_length(absolute);
#endif
}

/* IEEE input bits provide the exact integer mantissa and exponent. Output
 * limits are checked directly; no 128-bit shift or bit-count loop is needed. */
static int PQ(convert_ieee)(U_WORK value, int exponent, int *exact, int64_t *raw)
{
    PQ(ieee_t) parts;
    uint64_t magnitude, limit = UINT64_C(1) << (PQ_GAUSSIAN_F + 1);
    int shift;
    if (!PQ(ieee_read)(value, &parts)) return 0;
    if (parts.mantissa == 0) { *raw = 0; *exact = 1; return 1; }
    shift = PQ_GAUSSIAN_F - exponent + parts.exponent - (U_MANT_DIG - 1);
    *exact = 1;
    if (shift >= 0) {
        if (shift >= 64 || parts.mantissa > (limit >> shift)) return 0;
        magnitude = parts.mantissa << shift;
    } else {
        unsigned right = (unsigned)-shift;
        if (right >= 64) {
            magnitude = 0;
            *exact = 0;
        } else {
            magnitude = parts.mantissa >> right;
            if (right != 0) {
                if ((parts.mantissa & ((UINT64_C(1) << right) - 1)) != 0) *exact = 0;
                magnitude += (parts.mantissa >> (right - 1)) & 1;
            }
        }
    }
#if U_BITS == 15
    if ((!parts.negative && magnitude > (uint64_t)INT32_MAX) || magnitude > UINT64_C(2147483648)) return 0;
    if (parts.negative) *raw = magnitude == UINT64_C(2147483648) ? INT32_MIN : -(int32_t)magnitude;
    else *raw = (int32_t)magnitude;
#else
    if ((!parts.negative && magnitude > (uint64_t)INT64_MAX) || magnitude > (UINT64_C(1) << 63)) return 0;
    if (parts.negative) *raw = magnitude == (UINT64_C(1) << 63) ? INT64_MIN : -(int64_t)magnitude;
    else *raw = (int64_t)magnitude;
#endif
    return 1;
}
#endif

/* Convert one value at a vector-wide binary scale. Portable hosts retain the
 * modf boundary; IEEE hosts use exact input bits and integer shifts instead. */
static int PQ(convert_scaled)(U_WORK value, int exponent, int *exact, int64_t *raw)
{
#if (U_BITS == 15 && UKF_IEEE_BINARY32) || (U_BITS == 31 && UKF_IEEE_BINARY64)
    return PQ(convert_ieee)(value, exponent, exact, raw);
#elif FLT_RADIX == 2 && U_MANT_DIG <= 64
    U_WORK scaled = U_LDEXP(value, PQ_GAUSSIAN_F - exponent);
    U_WORK magnitude, rounded, limit, fraction;
    int negative;
    if (!PQ_ISFINITE(scaled)) return 0;
    negative = scaled < 0;
    magnitude = negative ? -scaled : scaled;
#if U_BITS == 15
    fraction = modff(magnitude, &rounded);
#else
    fraction = modf(magnitude, &rounded);
#endif
    /* modf does not depend on the current rounding mode. Adding one here is
     * exact: a nonzero fraction implies magnitude is below the significand's
     * integer-spacing threshold. */
    if (fraction >= (U_WORK)0.5) rounded += 1;
    limit = U_LDEXP((U_WORK)1, PQ_GAUSSIAN_F + 1);
    if (!PQ_ISFINITE(rounded) || rounded > limit || (rounded == limit && !negative)) return 0;
    if (rounded == limit) {
#if U_BITS == 15
        *raw = INT32_MIN;
#else
        *raw = INT64_MIN;
#endif
    } else {
        uint64_t integer = (uint64_t)rounded;
        *raw = negative ? -(int64_t)integer : (int64_t)integer;
    }
    *exact = U_LDEXP((U_WORK)*raw, exponent - PQ_GAUSSIAN_F) == value;
    return 1;
#else
    (void)value; (void)exponent; (void)exact; (void)raw;
    return 0;
#endif
}

static int PQ(vector_prepare)(const U_WORK *values, size_t count, U_WORK *raw, PQ(scale_t) *scale)
{
#if (U_BITS == 15 && UKF_IEEE_BINARY32) || (U_BITS == 31 && UKF_IEEE_BINARY64)
    uint64_t maximum = 0;
#else
    U_WORK maximum = 0;
#endif
    size_t i;
    scale->exponent = 0;
    scale->exact = 1;
    scale->diagonal = 0;
#if U_BITS == 15
    scale->maximum = 0;
#endif
    for (i = 0; i < count; ++i) {
#if (U_BITS == 15 && UKF_IEEE_BINARY32) || (U_BITS == 31 && UKF_IEEE_BINARY64)
        PQ(ieee_t) parts;
        if (!PQ(ieee_read)(values[i], &parts)) return 0;
        if (parts.absolute > maximum) maximum = parts.absolute;
#else
        U_WORK value = values[i], magnitude;
        if (!PQ_ISFINITE(value)) return 0;
        magnitude = value < 0 ? -value : value;
        if (magnitude > maximum) maximum = magnitude;
#endif
    }
    if (maximum != 0) {
#if (U_BITS == 15 && UKF_IEEE_BINARY32) || (U_BITS == 31 && UKF_IEEE_BINARY64)
        scale->exponent = PQ(ieee_floor)(maximum);
#else
        int exponent;
        (void)U_FREXP(maximum, &exponent);
        scale->exponent = exponent - 1;
#endif
    }
    for (i = 0; i < count; ++i) {
        int exact;
        int64_t value;
        if (!PQ(convert_scaled)(values[i], scale->exponent, &exact, &value)) return 0;
        if (!exact) scale->exact = 0;
#if U_BITS == 15
        {
            uint32_t magnitude = (uint32_t)pf_q_magnitude(value);
            if (magnitude > scale->maximum) scale->maximum = magnitude;
        }
#endif
        PQ(raw_store)(raw + i, value);
    }
    return 1;
}

/* Cache the lower triangle once. The error recurrence below accounts for
 * off-diagonal conversion error; requiring an exact positive pivot is what
 * keeps a diagonal conversion error out of that recurrence. */
static int PQ(gaussian_prepare)(const U_WORK *lower, size_t k, U_WORK *raw, PQ(scale_t) *scale)
{
#if (U_BITS == 15 && UKF_IEEE_BINARY32) || (U_BITS == 31 && UKF_IEEE_BINARY64)
    uint64_t maximum = 0;
#else
    U_WORK maximum = 0;
#endif
    size_t i, j;
    scale->exponent = 0;
    scale->exact = 1;
    scale->diagonal = 1;
#if U_BITS == 15
    scale->maximum = 0;
#endif
    for (i = 0; i < k; ++i) {
        for (j = 0; j <= i; ++j) {
#if (U_BITS == 15 && UKF_IEEE_BINARY32) || (U_BITS == 31 && UKF_IEEE_BINARY64)
            PQ(ieee_t) parts;
            if (!PQ(ieee_read)(lower[i * k + j], &parts)) return 0;
            if (i == j && (parts.negative || parts.absolute == 0)) return 0;
            if (i != j && parts.absolute != 0) scale->diagonal = 0;
            if (parts.absolute > maximum) maximum = parts.absolute;
#else
            U_WORK value = lower[i * k + j], magnitude;
            if (!PQ_ISFINITE(value) || (i == j && value <= 0)) return 0;
            if (i != j && value != 0) scale->diagonal = 0;
            magnitude = value < 0 ? -value : value;
            if (magnitude > maximum) maximum = magnitude;
#endif
        }
    }
    if (maximum == 0) return 0;
#if (U_BITS == 15 && UKF_IEEE_BINARY32) || (U_BITS == 31 && UKF_IEEE_BINARY64)
    scale->exponent = PQ(ieee_floor)(maximum);
#else
    {
        int exponent;
        (void)U_FREXP(maximum, &exponent);
        scale->exponent = exponent - 1;
    }
#endif
    for (i = 0; i < k; ++i) {
        for (j = 0; j < k; ++j) {
            int exact = 1;
            int64_t value = 0;
            if (j <= i) {
                if (!PQ(convert_scaled)(lower[i * k + j], scale->exponent, &exact, &value)) return 0;
                if (i == j && (!exact || value <= 0)) return 0;
                if (i != j && !exact) scale->exact = 0;
            }
#if U_BITS == 15
            {
                uint32_t magnitude = (uint32_t)pf_q_magnitude(value);
                if (magnitude > scale->maximum) scale->maximum = magnitude;
            }
#endif
            PQ(raw_store)(raw + i * k + j, value);
        }
    }
    return 1;
}

/* Exact integer dot with a conservative input-product certificate. */
static int PQ(dot_wide)(const U_WORK *a, PQ(scale_t) sa, const U_WORK *b, PQ(scale_t) sb,
                        size_t count, U_WORK *out)
{
    PQ(wide_t) sum;
    pf_q_u128_t error = {0, 0};
    U_WORK value;
    size_t i;
    sum.magnitude.lo = sum.magnitude.hi = 0;
    sum.negative = 0;
    for (i = 0; i < count; ++i) {
        int64_t av = PQ(raw_load)(a + i), bv = PQ(raw_load)(b + i);
        uint64_t am = pf_q_magnitude(av), bm = pf_q_magnitude(bv);
        pf_q_u128_t product = PQ(raw_product)(am, bm);
        if (!PQ(add_wide)(&sum, product, (av < 0) != (bv < 0))) return 0;
        if (!sa.exact || !sb.exact) {
            if (!sb.exact && !PQ(add_unsigned)(&error, PQ(from_u64)(am))) return 0;
            if (!sa.exact && !PQ(add_unsigned)(&error, PQ(from_u64)(bm))) return 0;
            if (!sa.exact && !sb.exact && !PQ(add_unsigned)(&error, PQ(from_u64)(1))) return 0;
        }
    }
    if (!PQ(scaled_le_one)(error, sa.exponent + sb.exponent - 2 * PQ_GAUSSIAN_F + PQ_GAUSSIAN_K))
        return 0;
    value = U_LDEXP(PQ(wide_value)(sum), sa.exponent + sb.exponent - 2 * PQ_GAUSSIAN_F);
    if (!PQ_ISFINITE(value)) return 0;
    *out = value;
    return 1;
}

static int PQ(dot_try)(const U_WORK *a, PQ(scale_t) sa, const U_WORK *b, PQ(scale_t) sb,
                       size_t count, U_WORK *out)
{
#if U_BITS == 15
    uint64_t maximum_product = (uint64_t)sa.maximum * sb.maximum;
    uint64_t error_term = 0, error;
    pf_q_u128_t bound;
    int64_t sum = 0;
    U_WORK value;
    size_t i;
    /* The exact widened products test the same capacity bounds without
     * dividing merely to prove that a native-width accumulation fits. */
    bound = pf_q_umul(maximum_product, (uint64_t)count);
    if (bound.hi != 0 || bound.lo > (uint64_t)INT64_MAX)
        return PQ(dot_wide)(a, sa, b, sb, count, out);
    if (!sb.exact) error_term += sa.maximum;
    if (!sa.exact) error_term += sb.maximum;
    if (!sa.exact && !sb.exact) ++error_term;
    bound = pf_q_umul(error_term, (uint64_t)count);
    if (bound.hi != 0) return PQ(dot_wide)(a, sa, b, sb, count, out);
    error = bound.lo;
    for (i = 0; i < count; ++i) {
        int32_t av = (int32_t)PQ(raw_load)(a + i), bv = (int32_t)PQ(raw_load)(b + i);
        sum += (int64_t)av * bv;
    }
    /* A maximum-based certificate can be looser than the original sum of
     * individual errors. Keep that accepted integer path at the same scale
     * rather than falling through to mixed floating arithmetic. */
    if (!PQ(scaled_le_one)(PQ(from_u64)(error),
            sa.exponent + sb.exponent - 2 * PQ_GAUSSIAN_F + PQ_GAUSSIAN_K))
        return PQ(dot_wide)(a, sa, b, sb, count, out);
    value = (U_WORK)pf_q_magnitude(sum);
    value = U_LDEXP(sum < 0 ? -value : value, sa.exponent + sb.exponent - 2 * PQ_GAUSSIAN_F);
    if (!PQ_ISFINITE(value)) return 0;
    *out = value;
    return 1;
#else
    return PQ(dot_wide)(a, sa, b, sb, count, out);
#endif
}

static int PQ(perturb_try)(const U_WORK *a, PQ(scale_t) sa, const U_WORK *b, PQ(scale_t) sb,
                           size_t k, U_WORK *out)
{
    size_t i;
    for (i = 0; i < k; ++i) {
        const U_WORK *row = sa.diagonal ? a + i * k + i : a + i * k;
        const U_WORK *vector = sa.diagonal ? b + i : b;
        size_t count = sa.diagonal ? 1 : i + 1;
        if (!PQ(dot_try)(row, sa, vector, sb, count, out + i)) return 0;
    }
    return 1;
}

static void PQ(perturb_mixed)(const U_WORK *lower, const U_WORK *draw, size_t k, U_WORK *out)
{
    size_t i;
    for (i = 0; i < k; ++i) out[i] = PQ(dot)(lower + i * k, 1, draw, 1, i + 1);
}

static void PQ(perturb_retry)(const U_WORK *lower, const U_WORK *draw, size_t k,
                              const U_WORK *raw_a, PQ(scale_t) sa, U_WORK *raw_b, U_WORK *out)
{
    PQ(scale_t) sb;
    if (PQ(vector_prepare)(draw, k, raw_b, &sb) && PQ(perturb_try)(raw_a, sa, raw_b, sb, k, out)) return;
    PQ(perturb_mixed)(lower, draw, k, out);
}

/* The checked divider supplies a positive value, so the shared exact bit
 * count gives a shift in [0,63] without a second leading-bit search. */
static uint64_t PQ(normalize_divisor)(uint64_t value, unsigned *shift)
{
    *shift = 64u - filters_q_bit_length(value);
    return value << *shift;
}

/* One base-2^32 quotient digit. The divisor is normalized, so the leading
 * estimate needs at most two corrections. An exact wide multiply/subtract
 * performs the final add-back check without signed-limb arithmetic. */
static void PQ(divide_digit)(pf_q_u128_t *value, uint64_t divisor, unsigned position, uint32_t *digit)
{
    const uint64_t base = UINT64_C(1) << 32;
    uint32_t limbs[4], a, b, c;
    uint64_t v1 = divisor >> 32, v0 = (uint32_t)divisor;
    uint64_t numerator, q, r;
    pf_q_u128_t product, step;
    limbs[0] = (uint32_t)value->lo;
    limbs[1] = (uint32_t)(value->lo >> 32);
    limbs[2] = (uint32_t)value->hi;
    limbs[3] = (uint32_t)(value->hi >> 32);
    a = position != 0 ? limbs[3] : limbs[2];
    b = position != 0 ? limbs[2] : limbs[1];
    c = position != 0 ? limbs[1] : limbs[0];
    numerator = ((uint64_t)a << 32) | b;
    q = numerator / v1;
    r = numerator % v1;
    while (q >= base || q * v0 > (r << 32) + c) {
        --q;
        r += v1;
        if (r >= base) break;
    }
    product = pf_q_umul(q, divisor);
    step = position != 0 ? pf_q_exp_shl(PQ(from_u64)(divisor), 32) : PQ(from_u64)(divisor);
    if (position != 0) product = pf_q_exp_shl(product, 32);
    if (pf_q_ucmp(product, *value) > 0) {
        --q;
        product = pf_q_exp_usub(product, step);
    }
    *value = pf_q_exp_usub(*value, product);
    *digit = (uint32_t)q;
}

/* Two native 64/32 estimates replace up to 64 bit-at-a-time iterations.
 * high<divisor proves a 64-bit quotient. Normalized Knuth correction gives
 * q_true<=q<=q_true+1, so exactly one add-back suffices (see the opt2 proof). */
static void PQ(divide_wide)(pf_q_u128_t numerator, uint64_t denominator,
                            uint64_t *quotient, pf_q_u128_t *remainder)
{
    unsigned shift;
    uint64_t normalized = PQ(normalize_divisor)(denominator, &shift);
    uint32_t high, low;
    numerator = pf_q_exp_shl(numerator, shift);
    PQ(divide_digit)(&numerator, normalized, 1, &high);
    PQ(divide_digit)(&numerator, normalized, 0, &low);
    *quotient = ((uint64_t)high << 32) | low;
    /* The normalized remainder is below 2^64 and exactly divisible by 2^shift. */
    numerator.lo >>= shift;
    numerator.hi = 0;
    *remainder = numerator;
}

/* Divide a 128-bit numerator by a positive 64-bit denominator, rejecting a
 * quotient outside 64 bits before calling the bounded long division. */
static int PQ(divide)(pf_q_u128_t numerator, uint64_t denominator, int round, uint64_t *quotient)
{
    uint64_t rest;
    /* numerator < denominator*2^64 iff its high limb is below the
     * denominator. This also rejects a zero denominator. */
    if (numerator.hi >= denominator) return 0;
    if ((denominator & (denominator - 1)) == 0) {
        unsigned shift = filters_q_bit_length(denominator) - 1;
        if (shift == 0) {
            rest = 0;
            *quotient = numerator.lo;
        } else {
            /* hi<2^shift proves the quotient and the left shift fit uint64. */
            rest = numerator.lo & (denominator - 1);
            *quotient = (numerator.lo >> shift) | (numerator.hi << (64 - shift));
        }
    } else if (numerator.hi == 0) {
        rest = numerator.lo % denominator;
        *quotient = numerator.lo / denominator;
    } else {
        pf_q_u128_t remainder;
        /* The bounded division returns remainder < denominator, so the
         * remainder's high limb is zero on both arithmetic paths. */
        PQ(divide_wide)(numerator, denominator, quotient, &remainder);
        rest = remainder.lo;
    }
    /* Comparing rest with denominator-rest avoids widening 2*rest. */
    if ((round ? rest >= denominator - rest : rest != 0)) {
        if (*quotient == UINT64_MAX) return 0;
        ++*quotient;
    }
    return 1;
}

/* Optional per-pivot reciprocal cache. The upper triangle is dead raw-factor
 * storage. A zero slot selects the ordinary divider, including for unprepared
 * factors and power-of-two pivots. Tests may count preparation and hits without
 * adding production work. */
#ifndef PQ_GAUSSIAN_DIVISION_HOOK
#define PQ_GAUSSIAN_DIVISION_HOOK(prepare, hit) ((void)0)
#endif

static size_t PQ(division_slot)(size_t k, size_t i)
{
    return i + 1 < k ? i * k + i + 1 : 2;
}

/* Prepare only after gaussian_prepare succeeded. k==2 stores its first
 * pivot's reciprocal in the sole upper slot; an equal second pivot can reuse
 * it. k<2 has no upper slot. Non-power setup is exact but costs one divide. */
static void PQ(gaussian_prepare_division)(U_WORK *raw, size_t k)
{
    size_t i, count = k == 2 ? 1 : k;
    PQ_GAUSSIAN_DIVISION_HOOK(1, 0);
    if (k < 2) return;
    for (i = 0; i < count; ++i) {
        size_t slot = k == 2 ? 1 : PQ(division_slot)(k, i);
#if U_BITS == 15
        uint32_t pivot = (uint32_t)PQ(raw_load)(raw + i * k + i);
        unsigned shift = 32u - filters_q_bit_length(pivot);
        uint32_t normalized = pivot << shift;
        uint64_t reciprocal;
        if (normalized == (UINT32_C(1) << 31)) {
            PQ(error_store)(raw + slot, 0);
            continue;
        }
        reciprocal = (((UINT64_C(1) << 32) - normalized) << 32) / normalized;
#else
        uint64_t pivot = (uint64_t)PQ(raw_load)(raw + i * k + i);
        unsigned shift = 64u - filters_q_bit_length(pivot);
        uint64_t normalized = pivot << shift;
        uint64_t reciprocal;
        if (normalized == (UINT64_C(1) << 63)) {
            PQ(error_store)(raw + slot, 0);
            continue;
        }
        {
            pf_q_u128_t numerator = {0, UINT64_MAX - normalized + 1}, remainder;
            PQ(divide_wide)(numerator, normalized, &reciprocal, &remainder);
        }
#endif
        PQ(error_store)(raw + slot, reciprocal);
    }
}

/* Exact normalized reciprocal division. The estimate is never high and is at
 * most one low; one add-back returns floor and the exact remainder. The
 * reciprocal is nonzero; solve dispatch sends zero slots directly to divide. */
static int PQ(divide_solve)(pf_q_u128_t numerator, uint64_t denominator, int round,
                            uint64_t reciprocal, uint64_t *quotient)
{
    uint64_t rest;
#if U_BITS == 15
    if (numerator.hi != 0 || numerator.lo >= (denominator << 32)) {
        PQ_GAUSSIAN_DIVISION_HOOK(0, 0);
        return PQ(divide)(numerator, denominator, round, quotient);
    }
#else
    if (numerator.hi >= denominator) {
        PQ_GAUSSIAN_DIVISION_HOOK(0, 0);
        return PQ(divide)(numerator, denominator, round, quotient);
    }
#endif
    PQ_GAUSSIAN_DIVISION_HOOK(0, 1);
#if U_BITS == 15
    {
        unsigned shift = 32u - filters_q_bit_length((uint32_t)denominator);
        uint32_t normalized = (uint32_t)denominator << shift;
        uint32_t r = (uint32_t)reciprocal, a, b, q;
        uint64_t n = numerator.lo << shift, t, remainder;
        a = (uint32_t)(n >> 32);
        b = (uint32_t)n;
        t = (uint64_t)a * r + b + (((uint64_t)b * r) >> 32);
        /* q0<=floor(n/v)<2^32; the one correction cannot wrap either. */
        q = (uint32_t)(a + (t >> 32));
        remainder = n - (uint64_t)q * normalized;
        if (remainder >= normalized) {
            ++q;
            remainder -= normalized;
        }
        rest = remainder >> shift;
        *quotient = q;
    }
#else
    {
        unsigned shift = 64u - filters_q_bit_length(denominator);
        uint64_t normalized = denominator << shift;
        pf_q_u128_t n = pf_q_exp_shl(numerator, shift);
        pf_q_u128_t ar = pf_q_umul(n.hi, reciprocal), br = pf_q_umul(n.lo, reciprocal);
        pf_q_u128_t t = pf_q_uadd(ar, PQ(from_u64)(n.lo));
        pf_q_u128_t product, remainder, d = {normalized, 0};
        uint64_t q;
        t = pf_q_uadd(t, PQ(from_u64)(br.hi));
        q = n.hi + t.hi;
        product = pf_q_umul(q, normalized);
        remainder = pf_q_exp_usub(n, product);
        if (pf_q_ucmp(remainder, d) >= 0) {
            ++q;
            remainder = pf_q_exp_usub(remainder, d);
        }
        rest = remainder.lo >> shift;
        *quotient = q;
    }
#endif
    if ((round ? rest >= denominator - rest : rest != 0)) {
        if (*quotient == UINT64_MAX) return 0;
        ++*quotient;
    }
    return 1;
}

/* Z is a signed raw integer and E is an unsigned Q32 forward-error bound.
 * A*Z=B*2^fractional, so deltaB contributes 2^(fractional+32) before the
 * pivot divide; off-diagonal |Z| is shifted to Q32 separately. Z starts at F
 * fractional bits; a capacity failure reduces only Z's local dyadic scale and
 * restarts the arithmetic. The norm certificate must still accept it. */
static int PQ(gaussian_try)(const U_WORK *a, PQ(scale_t) sa, const U_WORK *b, PQ(scale_t) sb,
                            size_t k, U_WORK *z, U_WORK *error, U_WORK *out)
{
    size_t i, j;
    unsigned fractional = PQ_GAUSSIAN_F;
    pf_q_u128_t norm, variance;
    U_WORK value;
    const int exact_diagonal = sa.diagonal && sb.exact;
#if U_BITS == 15
    uint64_t zmax, emax;
#endif
restart:
    norm.lo = norm.hi = variance.lo = variance.hi = 0;
#if U_BITS == 15
    zmax = emax = 0;
#endif
    for (i = 0; i < k; ++i) {
        PQ(wide_t) numerator;
        pf_q_u128_t propagated, product;
        int64_t pivot = PQ(raw_load)(a + i * k + i), bv = PQ(raw_load)(b + i);
        uint64_t reciprocal = 0, quotient;
        if (k >= 3) {
            reciprocal = PQ(error_load)(a + PQ(division_slot)(k, i));
        } else if (k == 2 && (i == 0 || pivot == PQ(raw_load)(a))) {
            reciprocal = PQ(error_load)(a + 1);
        }
        if (pivot <= 0) return 0;
#if U_BITS == 15
        {
            uint64_t initial = (uint64_t)sb.maximum << fractional;
            pf_q_u128_t bound = {0, 0};
            /* An exact diagonal row subtracts no products; its initial
             * magnitude is at most 2^31*2^30 and already fits int64. */
            if (!sa.diagonal) bound = pf_q_umul((uint64_t)sa.maximum * zmax, (uint64_t)i);
            if (bound.hi == 0 && bound.lo <= (uint64_t)INT64_MAX - initial) {
                int64_t signed_numerator = (int64_t)(int32_t)bv * (INT64_C(1) << fractional);
                if (!sa.diagonal) for (j = 0; j < i; ++j) {
                    int32_t av = (int32_t)PQ(raw_load)(a + i * k + j);
                    int32_t zv = (int32_t)PQ(raw_load)(z + j);
                    signed_numerator -= (int64_t)av * zv;
                }
                numerator.magnitude = PQ(from_u64)(pf_q_magnitude(signed_numerator));
                numerator.negative = signed_numerator < 0;
            } else
#endif
            {
                numerator.magnitude.lo = numerator.magnitude.hi = 0;
                numerator.negative = 0;
                if (!PQ(shift_left)(PQ(from_u64)(pf_q_magnitude(bv)), fractional, &product) ||
                    !PQ(add_wide)(&numerator, product, bv < 0)) goto reduce_precision;
                if (!sa.diagonal) for (j = 0; j < i; ++j) {
                    int64_t av = PQ(raw_load)(a + i * k + j), zv = PQ(raw_load)(z + j);
                    int product_negative;
                    product = PQ(raw_product)(pf_q_magnitude(av), pf_q_magnitude(zv));
                    product_negative = av != 0 && zv != 0 && !((av < 0) != (zv < 0));
                    if (!PQ(add_wide)(&numerator, product, product_negative)) goto reduce_precision;
                }
            }
#if U_BITS == 15
        }
#endif
        if (reciprocal != 0) {
            if (!PQ(divide_solve)(numerator.magnitude, (uint64_t)pivot, 1, reciprocal, &quotient)) goto reduce_precision;
        } else {
            PQ_GAUSSIAN_DIVISION_HOOK(0, 0);
            if (!PQ(divide)(numerator.magnitude, (uint64_t)pivot, 1, &quotient)) goto reduce_precision;
        }
        if (!numerator.negative) {
#if U_BITS == 15
            if (quotient > (uint64_t)INT32_MAX) goto reduce_precision;
#else
            if (quotient > (uint64_t)INT64_MAX) goto reduce_precision;
#endif
            PQ(raw_store)(z + i, (int64_t)quotient);
        } else {
#if U_BITS == 15
            if (quotient > UINT64_C(2147483648)) goto reduce_precision;
            PQ(raw_store)(z + i, quotient == UINT64_C(2147483648) ? INT32_MIN : -(int32_t)quotient);
#else
            if (quotient > (UINT64_C(1) << 63)) goto reduce_precision;
            PQ(raw_store)(z + i, quotient == (UINT64_C(1) << 63) ? INT64_MIN : -(int64_t)quotient);
#endif
        }
#if U_BITS == 15
        {
            uint64_t magnitude = pf_q_magnitude(PQ(raw_load)(z + i));
            if (magnitude > zmax) zmax = magnitude;
        }
#endif

        if (exact_diagonal) {
            /* The old zero divide always returns zero and stores E=2^31. */
            quotient = 0;
        } else {
#if U_BITS == 15
        {
            uint64_t delta = sb.exact ? 0 : UINT64_C(1) << (fractional + 32);
            uint64_t term = 0;
            int fast = 1;
            if (!sa.diagonal) {
                if (!sa.exact) term = (zmax << 32) + emax;
                {
                    uint64_t product_bound = sa.maximum * emax;
                    if (term > UINT64_MAX - product_bound) fast = 0;
                    else term += product_bound;
                }
            }
            if (fast && term != 0) {
                pf_q_u128_t bound = pf_q_umul(term, (uint64_t)i);
                if (bound.hi != 0 || bound.lo > UINT64_MAX - delta) fast = 0;
            }
            if (fast) {
                uint64_t sum = delta;
                if (!sa.diagonal) for (j = 0; j < i; ++j) {
                    int32_t av = (int32_t)PQ(raw_load)(a + i * k + j);
                    int32_t zv = (int32_t)PQ(raw_load)(z + j);
                    uint32_t ev = (uint32_t)PQ(error_load)(error + j);
                    if (!sa.exact) sum += ((uint64_t)(uint32_t)pf_q_magnitude(zv) << 32) + ev;
                    sum += (uint64_t)(uint32_t)pf_q_magnitude(av) * ev;
                }
                propagated = PQ(from_u64)(sum);
            } else
#endif
            {
                propagated = sb.exact ? PQ(from_u64)(0) :
                    pf_q_exp_shl(PQ(from_u64)(1), fractional + 32);
                if (!sa.diagonal) for (j = 0; j < i; ++j) {
                    int64_t av = PQ(raw_load)(a + i * k + j), zv = PQ(raw_load)(z + j);
                    uint64_t ev = PQ(error_load)(error + j);
                    pf_q_u128_t zbound;
                    if (!sa.exact) {
                        if (!PQ(shift_left)(PQ(from_u64)(pf_q_magnitude(zv)), 32, &zbound) ||
                            !PQ(add_unsigned)(&zbound, PQ(from_u64)(ev)) ||
                            !PQ(add_unsigned)(&propagated, zbound)) goto reduce_precision;
                    }
                    product = PQ(raw_product)(pf_q_magnitude(av), ev);
                    if (!PQ(add_unsigned)(&propagated, product)) goto reduce_precision;
                }
            }
#if U_BITS == 15
        }
#endif
        if (reciprocal != 0) {
            if (!PQ(divide_solve)(propagated, (uint64_t)pivot, 0, reciprocal, &quotient) ||
                quotient > (uint64_t)PQ_GAUSSIAN_ERROR_MAX - (UINT64_C(1) << 31)) goto reduce_precision;
        } else {
            PQ_GAUSSIAN_DIVISION_HOOK(0, 0);
            if (!PQ(divide)(propagated, (uint64_t)pivot, 0, &quotient) ||
                quotient > (uint64_t)PQ_GAUSSIAN_ERROR_MAX - (UINT64_C(1) << 31)) goto reduce_precision;
        }
        }
        quotient += UINT64_C(1) << 31;
        PQ(error_store)(error + i, quotient);
#if U_BITS == 15
        if (quotient > emax) emax = quotient;
#endif
    }

    if (exact_diagonal) {
        pf_q_u128_t magnitudes = PQ(from_u64)(0);
#if U_BITS == 15
        /* Keep native norm accumulation whenever its own bound fits; the
         * independently computed variance does not force it wider. */
        {
            uint64_t square_bound = zmax * zmax;
            pf_q_u128_t norm_bound = pf_q_umul(square_bound, (uint64_t)k);
            if (norm_bound.hi == 0) {
                uint64_t norm64 = 0, magnitude64 = 0;
                /* Integer zm <= zm*zm, so the same norm bound proves that
                 * the magnitude sum and every positive partial sum fit. */
                for (i = 0; i < k; ++i) {
                    uint64_t zm = (uint32_t)pf_q_magnitude(PQ(raw_load)(z + i));
                    norm64 += zm * zm;
                    magnitude64 += zm;
                }
                norm = PQ(from_u64)(norm64);
                magnitudes = PQ(from_u64)(magnitude64);
            } else {
                for (i = 0; i < k; ++i) {
                    uint64_t zm = pf_q_magnitude(PQ(raw_load)(z + i));
                    if (!PQ(add_unsigned)(&norm, PQ(raw_product)(zm, zm)) ||
                        !PQ(add_unsigned)(&magnitudes, PQ(from_u64)(zm))) goto reduce_precision;
                }
            }
        }
#else
        for (i = 0; i < k; ++i) {
            uint64_t zm = pf_q_magnitude(PQ(raw_load)(z + i));
            if (!PQ(add_unsigned)(&norm, PQ(raw_product)(zm, zm)) ||
                !PQ(add_unsigned)(&magnitudes, PQ(from_u64)(zm))) goto reduce_precision;
        }
#endif
        /* E=2^31 makes variance=(sum|Z|)<<32+k*2^30 exactly. Existing
         * k<2^31 keeps this below 2^94(q15)/2^126(q31), with checks retained. */
        if (!PQ(shift_left)(magnitudes, 32, &variance) ||
            !PQ(add_unsigned)(&variance, pf_q_exp_shl(PQ(from_u64)((uint64_t)k), 30))) goto reduce_precision;
    } else {
#if U_BITS == 15
    {
        uint64_t square_bound = zmax * zmax;
        uint64_t term_bound = 2 * zmax * emax + ((emax * emax) >> 32) + 1;
        pf_q_u128_t norm_bound = pf_q_umul(square_bound, (uint64_t)k);
        pf_q_u128_t variance_bound = pf_q_umul(term_bound, (uint64_t)k);
        if (norm_bound.hi == 0 && variance_bound.hi == 0) {
            uint64_t norm64 = 0, variance64 = 0;
            for (i = 0; i < k; ++i) {
                int32_t zv = (int32_t)PQ(raw_load)(z + i);
                uint32_t ev = (uint32_t)PQ(error_load)(error + i);
                uint64_t zm = (uint32_t)pf_q_magnitude(zv);
                uint64_t error_square = (uint64_t)ev * ev;
                norm64 += zm * zm;
                variance64 += 2 * zm * ev + (error_square >> 32) +
                    ((error_square & UINT64_C(0xffffffff)) != 0);
            }
            norm = PQ(from_u64)(norm64);
            variance = PQ(from_u64)(variance64);
        } else
#endif
        {
            for (i = 0; i < k; ++i) {
                uint64_t zm = pf_q_magnitude(PQ(raw_load)(z + i)), ev = PQ(error_load)(error + i);
                pf_q_u128_t square = PQ(raw_product)(zm, zm);
                pf_q_u128_t error_square = PQ(raw_product)(ev, ev), cross, term;
                if (!PQ(add_unsigned)(&norm, square)) goto reduce_precision;
                cross = PQ(raw_product)(zm, ev);
                if (!PQ(shift_left)(cross, 1, &term) || !PQ(add_unsigned)(&variance, term)) goto reduce_precision;
                term = PQ(shift_right)(error_square, 32);
                if ((error_square.lo & UINT64_C(0xffffffff)) != 0 &&
                    !PQ(add_unsigned)(&term, PQ(from_u64)(1))) goto reduce_precision;
                if (!PQ(add_unsigned)(&variance, term)) goto reduce_precision;
            }
        }
#if U_BITS == 15
    }
#endif
    }
    /* A*z=B gives z=y*2^(sL-sD), hence y=z*2^(sD-sL). */
    if (!PQ(scaled_le_one)(variance,
            2 * (sb.exponent - sa.exponent - (int)fractional) - 33 + PQ_GAUSSIAN_K)) return 0;
    value = U_LDEXP(PQ(unsigned_value)(norm),
                    2 * (sb.exponent - sa.exponent - (int)fractional) - 1);
    if (!PQ_ISFINITE(value)) return 0;
    *out = value;
    return 1;
reduce_precision:
    if (fractional == 0) return 0;
    --fractional;
    goto restart;
}

#undef PQ_GAUSSIAN_ERROR_MAX
#undef PQ_GAUSSIAN_K
#undef PQ_GAUSSIAN_F
