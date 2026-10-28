/* Bounded integer exponential and wide quotient helpers for Q reweighting.
 * Include after pf_q_internal.h has defined its 128-bit integer helpers. */
#ifndef FILTERS_PF_Q_EXP_IMPL_H
#define FILTERS_PF_Q_EXP_IMPL_H

#include <math.h>

#ifndef PF_Q_F40_ROUTE
#define PF_Q_F40_ROUTE() ((void)0)
#endif

/* The fixed kernels use one scale per quantity, not exponent/mantissa signal
 * records. q15 uses F40 and degree 4; q31 uses F64 and degree 7. Conservative
 * relative exponential error bounds are 2^-27 and 2^-52 respectively.
 * Range-reduction error is at most 1023*2^-41 (q15) or 2047*2^-65 (q31).
 * Input and table quantization each stay below 2^-41/2^-65, at most 16
 * coefficient/Horner roundings stay below 2^-37/2^-61, and the reduced
 * Taylor remainders stay below 2^-34.51/2^-59.49. Their sums stay below the
 * declared bounds, including one extra input-quantization unit when two
 * fixed exponents are subtracted. Likelihood output adds at most one final
 * F31/F63 rounding error. Any capacity or boundary case outside
 * these proofs returns zero to the caller's local libm retry. */
typedef struct {
    unsigned fractional;
    unsigned likelihood_fraction;
    unsigned degree;
    pf_q_u128_t one;
    uint64_t likelihood_one;
    uint64_t step;
    const pf_q_u128_t *table;
    const uint64_t *coefficients;
} pf_q_exp_format_t;

static const pf_q_u128_t pf_q_exp_q15_table[32] = {
    {UINT64_C(0x10000000000), 0}, {UINT64_C(0xfa83b2db72), 0},
    {UINT64_C(0xf5257d1525), 0}, {UINT64_C(0xefe4b99bdd), 0},
    {UINT64_C(0xeac0c6e7dd), 0}, {UINT64_C(0xe5b906e77d), 0},
    {UINT64_C(0xe0ccdeec2b), 0}, {UINT64_C(0xdbfbb797db), 0},
    {UINT64_C(0xd744fccad7), 0}, {UINT64_C(0xd2a81d91f1), 0},
    {UINT64_C(0xce248c1520), 0}, {UINT64_C(0xc9b9bd866e), 0},
    {UINT64_C(0xc5672a1155), 0}, {UINT64_C(0xc12c4cca66), 0},
    {UINT64_C(0xbd08a39f58), 0}, {UINT64_C(0xb8fbaf4763), 0},
    {UINT64_C(0xb504f333fa), 0}, {UINT64_C(0xb123f581d3), 0},
    {UINT64_C(0xad583eea43), 0}, {UINT64_C(0xa9a15ab4ea), 0},
    {UINT64_C(0xa5fed6a9b1), 0}, {UINT64_C(0xa27043030c), 0},
    {UINT64_C(0x9ef5326092), 0}, {UINT64_C(0x9b8d39b9d5), 0},
    {UINT64_C(0x9837f0518e), 0}, {UINT64_C(0x94f4efa8ff), 0},
    {UINT64_C(0x91c3d373ab), 0}, {UINT64_C(0x8ea4398b46), 0},
    {UINT64_C(0x8b95c1e3eb), 0}, {UINT64_C(0x88980e8093), 0},
    {UINT64_C(0x85aac367cc), 0}, {UINT64_C(0x82cd8698ac), 0}
};
static const uint64_t pf_q_exp_q15_coefficients[5] = {
    UINT64_C(0x10000000000), UINT64_C(0x10000000000), UINT64_C(0x8000000000),
    UINT64_C(0x2aaaaaaaab), UINT64_C(0xaaaaaaaab)
};
static const pf_q_u128_t pf_q_exp_q31_table[32] = {
    {UINT64_C(0), 1}, {UINT64_C(0xfa83b2db722a033a), 0},
    {UINT64_C(0xf5257d152486cc2c), 0}, {UINT64_C(0xefe4b99bdcdaf5cb), 0},
    {UINT64_C(0xeac0c6e7dd24392f), 0}, {UINT64_C(0xe5b906e77c8348a8), 0},
    {UINT64_C(0xe0ccdeec2a94e111), 0}, {UINT64_C(0xdbfbb797daf23755), 0},
    {UINT64_C(0xd744fccad69d6af4), 0}, {UINT64_C(0xd2a81d91f12ae45a), 0},
    {UINT64_C(0xce248c151f8480e4), 0}, {UINT64_C(0xc9b9bd866e2f27a3), 0},
    {UINT64_C(0xc5672a115506dadd), 0}, {UINT64_C(0xc12c4cca66709456), 0},
    {UINT64_C(0xbd08a39f580c36bf), 0}, {UINT64_C(0xb8fbaf4762fb9ee9), 0},
    {UINT64_C(0xb504f333f9de6484), 0}, {UINT64_C(0xb123f581d2ac2590), 0},
    {UINT64_C(0xad583eea42a14ac6), 0}, {UINT64_C(0xa9a15ab4ea7c0ef8), 0},
    {UINT64_C(0xa5fed6a9b15138ea), 0}, {UINT64_C(0xa27043030c496819), 0},
    {UINT64_C(0x9ef5326091a111ae), 0}, {UINT64_C(0x9b8d39b9d54e5539), 0},
    {UINT64_C(0x9837f0518db8a96f), 0}, {UINT64_C(0x94f4efa8fef70961), 0},
    {UINT64_C(0x91c3d373ab11c336), 0}, {UINT64_C(0x8ea4398b45cd53c0), 0},
    {UINT64_C(0x8b95c1e3ea8bd6e7), 0}, {UINT64_C(0x88980e8092da8527), 0},
    {UINT64_C(0x85aac367cc487b15), 0}, {UINT64_C(0x82cd8698ac2ba1d7), 0}
};
static const uint64_t pf_q_exp_q31_coefficients[8] = {
    UINT64_C(0), UINT64_C(0), UINT64_C(0x8000000000000000),
    UINT64_C(0x2aaaaaaaaaaaaaab), UINT64_C(0xaaaaaaaaaaaaaab),
    UINT64_C(0x222222222222222), UINT64_C(0x5b05b05b05b05b),
    UINT64_C(0xd00d00d00d00d)
};

static const pf_q_exp_format_t pf_q_exp_q15 = {
    40, 31, 4, {UINT64_C(0x10000000000), 0}, UINT64_C(2147483648),
    UINT64_C(23816355775), pf_q_exp_q15_table, pf_q_exp_q15_coefficients
};
static const pf_q_exp_format_t pf_q_exp_q31 = {
    64, 63, 7, {UINT64_C(0), 1}, UINT64_C(9223372036854775808),
    UINT64_C(399572145162582989), pf_q_exp_q31_table, pf_q_exp_q31_coefficients
};

static pf_q_u128_t pf_q_exp_shl(pf_q_u128_t a, unsigned shift)
{
    pf_q_u128_t r = {0, 0};
    if (shift == 0) return a;
    if (shift < 64) {
        r.lo = a.lo << shift;
        r.hi = (a.hi << shift) | (a.lo >> (64 - shift));
    } else {
        /* All callers prove that discarded high bits are zero and shift<128. */
        r.hi = a.lo << (shift - 64);
        if (shift > 64) r.hi |= a.hi >> (128 - shift);
    }
    return r;
}

static pf_q_u128_t pf_q_exp_usub(pf_q_u128_t a, pf_q_u128_t b)
{
    pf_q_u128_t r;
    r.lo = a.lo - b.lo;
    r.hi = a.hi - b.hi - (a.lo < b.lo);
    return r;
}

/* Quotient is known to fit 64 bits. Only quotient-significant bits are
 * visited: at most 11 for range reduction and 32 for normalization. */
static uint64_t pf_q_exp_div(pf_q_u128_t numerator, pf_q_u128_t denominator,
                             pf_q_u128_t *remainder)
{
    uint64_t quotient = 0;
    unsigned shift;
    if (pf_q_ucmp(numerator, denominator) < 0) {
        *remainder = numerator;
        return 0;
    }
    shift = pf_q_ubits(numerator) - pf_q_ubits(denominator);
    denominator = pf_q_exp_shl(denominator, shift);
    for (;;) {
        if (pf_q_ucmp(numerator, denominator) >= 0) {
            numerator = pf_q_exp_usub(numerator, denominator);
            quotient |= UINT64_C(1) << shift;
        }
        if (shift == 0) break;
        denominator.lo = (denominator.lo >> 1) | (denominator.hi << 63);
        denominator.hi >>= 1;
        --shift;
    }
    *remainder = numerator;
    return quotient;
}

/* q15's accepted delta is below 2^45, so range reduction needs only a
 * 64-bit quotient-significant loop with at most 11 iterations. */
static uint64_t pf_q_exp_div64(uint64_t numerator, uint64_t denominator, uint64_t *remainder)
{
    uint64_t quotient = 0, shifted;
    unsigned shift;
    if (numerator < denominator) {
        *remainder = numerator;
        return 0;
    }
    shift = filters_q_bit_length(numerator) - filters_q_bit_length(denominator);
    shifted = denominator << shift;
    for (;;) {
        if (numerator >= shifted) {
            numerator -= shifted;
            quotient |= UINT64_C(1) << shift;
        }
        if (shift == 0) break;
        shifted >>= 1;
        --shift;
    }
    *remainder = numerator;
    return quotient;
}

static uint64_t pf_q_exp_round_div(pf_q_u128_t numerator, pf_q_u128_t denominator)
{
    pf_q_u128_t remainder, twice;
    uint64_t quotient = pf_q_exp_div(numerator, denominator, &remainder);
    twice = pf_q_exp_shl(remainder, 1);
    return quotient + (pf_q_ucmp(twice, denominator) >= 0);
}

/* Return mantissa*2^shift, rounded to an integer for a negative shift.
 * The mantissa has at most 53 bits; left-shift callers prove a 128-bit fit. */
static pf_q_u128_t pf_q_exp_mantissa(uint64_t mantissa, int shift)
{
    pf_q_u128_t result = {0, 0};
    unsigned right;
    if (mantissa == 0) return result;
    if (shift >= 0) {
        if (shift < 64) {
            result.lo = mantissa << shift;
            result.hi = shift != 0 ? mantissa >> (64 - shift) : 0;
        } else {
            result.hi = mantissa << (shift - 64);
        }
        return result;
    }
    right = (unsigned)-shift;
    if (right >= 64) return result; /* |value| < 2^-11 here. */
    result.lo = mantissa >> right;
    if (right != 0 && ((mantissa >> (right - 1)) & 1) != 0) ++result.lo;
    return result;
}

static pf_q_i128_t pf_q_exp_coefficient(const pf_q_exp_format_t *format, unsigned i)
{
    pf_q_i128_t result;
    if (i <= 1) { /* Both 0! and 1! have coefficient one. */
        result.lo = format->one.lo;
        result.hi = (int64_t)format->one.hi;
    } else {
        result.lo = format->coefficients[i];
        result.hi = 0;
    }
    if ((i & 1) != 0) {
        result.lo = ~result.lo + 1;
        result.hi = ~result.hi + (result.lo == 0);
    }
    return result;
}

/* Multiply a signed QF value by an unsigned QF value and round back to QF.
 * The signed input is in [-2^64,2^64], and the unsigned input fits uint64_t. */
static uint64_t pf_q_exp_signed_product(pf_q_i128_t a, uint64_t b, unsigned fractional,
                                        int *negative)
{
    pf_q_u128_t magnitude, product;
    uint64_t rounded;
    if (a.hi < 0) {
        magnitude.lo = ~a.lo + 1;
        magnitude.hi = ~(uint64_t)a.hi + (magnitude.lo == 0);
    } else {
        magnitude.lo = a.lo;
        magnitude.hi = (uint64_t)a.hi;
    }
    /* A magnitude with hi!=0 is exactly 2^64 by the Horner invariants. */
    product = magnitude.hi != 0 ? (pf_q_u128_t){0, b} : pf_q_umul(magnitude.lo, b);
    rounded = pf_q_uround(product, fractional);
    *negative = a.hi < 0 && rounded != 0;
    return rounded;
}

/* Multiply two values in [0,2^64]. The overflow flag identifies exactly
 * 2^128, which the caller rescales directly as a power of two. */
static pf_q_u128_t pf_q_exp_positive_product(pf_q_u128_t a, pf_q_u128_t b, int *overflow)
{
    *overflow = a.hi != 0 && b.hi != 0;
    if (*overflow) return (pf_q_u128_t){0, 0};
    if (a.hi != 0) return (pf_q_u128_t){0, b.lo};
    if (b.hi != 0) return (pf_q_u128_t){0, a.lo};
    return pf_q_umul(a.lo, b.lo);
}

static int pf_q_exp_fixed(const pf_q_exp_format_t *format, pf_q_u128_t delta,
                          uint64_t *likelihood)
{
    pf_q_u128_t remainder, numerator;
    pf_q_i128_t polynomial;
    uint64_t k, value;
    unsigned i, shift;
    int negative, overflow;
    k = pf_q_exp_div(delta, (pf_q_u128_t){format->step, 0}, &remainder);
    if (k > 32 * (format->likelihood_fraction + 1)) return 0;
    polynomial = pf_q_exp_coefficient(format, format->degree);
    for (i = format->degree; i-- > 0;) {
        uint64_t product = pf_q_exp_signed_product(polynomial, remainder.lo,
                                                   format->fractional, &negative);
        polynomial = pf_q_exp_coefficient(format, i);
        pf_q_iadd(&polynomial, product, negative);
    }
    if (polynomial.hi < 0) return 0;
    numerator = pf_q_exp_positive_product(format->table[k & 31],
        (pf_q_u128_t){polynomial.lo, (uint64_t)polynomial.hi}, &overflow);
    if (overflow) {
        /* Only q31's exact one*one product reaches 2^128. Its F63 result
         * is a power of two; no transcendental retry is needed. Preserve
         * the existing zero/one-cell guard before shifting by 64. */
        shift = (unsigned)(k >> 5);
        if (shift >= format->likelihood_fraction) return 0;
        value = format->likelihood_one >> shift;
    } else {
        shift = 2 * format->fractional - format->likelihood_fraction + (unsigned)(k >> 5);
        if (shift >= 128) return 0;
        value = pf_q_uround(numerator, shift);
    }
    /* Cells zero and one always use libm: this is the certified tiny-weight
     * and cutoff guard, not a heuristic approximation threshold. */
    if (value <= 1 || value > format->likelihood_one) return 0;
    *likelihood = value;
    return 1;
}

/* Exact q15 F40 path. With P_i=(-1)^i*m_i, Horner's recurrence is
 * m_i=c_i-round(m_(i+1)*r/2^40)>0. Here r<step<2^35 and the current
 * coefficients satisfy ceil(c_(i+1)/32)<c_i, so induction proves every
 * subtraction positive and m_i<=c_i. The generic kernel remains available
 * for q31 and differential checks of this specialization. */
static int pf_q_exp_fixed_f40(pf_q_u128_t delta, uint64_t *likelihood)
{
    const pf_q_exp_format_t *format = &pf_q_exp_q15;
    pf_q_u128_t numerator;
    uint64_t k, remainder, polynomial, value;
    unsigned i, shift;
    PF_Q_F40_ROUTE();
    if (delta.hi != 0 ||
        delta.lo >= format->step * (32 * (format->likelihood_fraction + 1) + 1)) return 0;
    k = pf_q_exp_div64(delta.lo, format->step, &remainder);
    polynomial = format->coefficients[format->degree];
    for (i = format->degree; i-- > 0;) {
        const uint64_t coefficient = format->coefficients[i];
        const uint64_t product = pf_q_uround(pf_q_umul(polynomial, remainder), format->fractional);
        polynomial = coefficient - product;
    }
    numerator = pf_q_umul(format->table[k & 31].lo, polynomial);
    shift = 2 * format->fractional - format->likelihood_fraction + (unsigned)(k >> 5);
    value = pf_q_uround(numerator, shift);
    if (value <= 1 || value > format->likelihood_one) return 0;
    *likelihood = value;
    return 1;
}

/** Try the bounded q15 integer exponential; zero requests the local libm path. */
static int pf_q15_exp_neg_try(float delta, uint32_t *likelihood)
{
    if (likelihood == NULL) return 0;
    if (delta == 0) { *likelihood = UINT32_C(2147483648); return 1; }
    if (isnan(delta) || delta < 0) return 0;
    if (isinf(delta)) {
        if (delta > 0) { *likelihood = 0; return 1; }
        return 0;
    }
    if (delta > 0x1.62e432p+4f) { *likelihood = 0; return 1; }
#if UKF_IEEE_BINARY32
    {
        uint32_t bits, encoded;
        uint64_t mantissa, value;
        int shift;
        pf_q_u128_t fixed;
        memcpy(&bits, &delta, sizeof(bits));
        encoded = bits >> 23;
        if (encoded == 0) {
            mantissa = bits & UINT32_C(0x007fffff);
            shift = -109; /* 40-149 */
        } else {
            mantissa = (bits & UINT32_C(0x007fffff)) | UINT32_C(0x00800000);
            shift = (int)encoded - 110; /* exponent-150+40 */
        }
        fixed = pf_q_exp_mantissa(mantissa, shift);
        if (!pf_q_exp_fixed_f40(fixed, &value)) return 0;
        *likelihood = (uint32_t)value;
        return 1;
    }
#elif FLT_RADIX == 2 && FLT_MANT_DIG <= 53
    {
        /* Some compilers advertise binary precision but not floating word
         * order. Convert only this input boundary without inspecting bits. */
        int exponent;
        float fraction = frexpf(delta, &exponent);
        uint64_t mantissa = (uint64_t)ldexpf(fraction, FLT_MANT_DIG), value;
        pf_q_u128_t fixed = pf_q_exp_mantissa(mantissa, exponent - FLT_MANT_DIG + 40);
        if (!pf_q_exp_fixed_f40(fixed, &value)) return 0;
        *likelihood = (uint32_t)value;
        return 1;
    }
#else
    return 0; /* Unsupported working representation: preserve the libm path. */
#endif
}

/** Try the bounded q31 integer exponential; zero requests the local libm path. */
static int pf_q31_exp_neg_try(double delta, uint64_t *likelihood)
{
    if (likelihood == NULL) return 0;
    if (delta == 0) { *likelihood = UINT64_C(9223372036854775808); return 1; }
    if (isnan(delta) || delta < 0) return 0;
    if (isinf(delta)) {
        if (delta > 0) { *likelihood = 0; return 1; }
        return 0;
    }
    if (delta > 0x1.62e42fefa39f0p+5) { *likelihood = 0; return 1; }
#if UKF_IEEE_BINARY64
    {
        uint64_t bits, mantissa, value;
        unsigned encoded;
        int shift;
        pf_q_u128_t fixed;
        memcpy(&bits, &delta, sizeof(bits));
        encoded = (unsigned)((bits >> 52) & UINT64_C(0x7ff));
        if (encoded == 0) {
            mantissa = bits & UINT64_C(0x000fffffffffffff);
            shift = -1010; /* 64-1074 */
        } else {
            mantissa = (bits & UINT64_C(0x000fffffffffffff)) | UINT64_C(0x0010000000000000);
            shift = (int)encoded - 1011; /* exponent-1075+64 */
        }
        fixed = pf_q_exp_mantissa(mantissa, shift);
        if (!pf_q_exp_fixed(&pf_q_exp_q31, fixed, &value)) return 0;
        *likelihood = value;
        return 1;
    }
#elif FLT_RADIX == 2 && DBL_MANT_DIG <= 53
    {
        int exponent;
        double fraction = frexp(delta, &exponent);
        uint64_t mantissa = (uint64_t)ldexp(fraction, DBL_MANT_DIG), value;
        pf_q_u128_t fixed = pf_q_exp_mantissa(mantissa, exponent - DBL_MANT_DIG + 64);
        if (!pf_q_exp_fixed(&pf_q_exp_q31, fixed, &value)) return 0;
        *likelihood = value;
        return 1;
    }
#else
    return 0; /* Unsupported working representation: preserve the libm path. */
#endif
}

#endif
