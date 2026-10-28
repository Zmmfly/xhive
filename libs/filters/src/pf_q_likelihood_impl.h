/* Reweight input boundary: compare finite exponents and subtract them on the
 * exponential's F40/F64 grid. No per-particle exponent/mantissa work array is
 * introduced. Only a zero/one likelihood cell keeps the original local retry. */

static int PQ(exponent_less)(U_WORK a, U_WORK b)
{
#if (U_BITS == 15 && UKF_IEEE_BINARY32) || (U_BITS == 31 && UKF_IEEE_BINARY64)
#if U_BITS == 15
    uint32_t av, bv;
    const uint32_t sign = UINT32_C(0x80000000);
#else
    uint64_t av, bv;
    const uint64_t sign = UINT64_C(0x8000000000000000);
#endif
    memcpy(&av, &a, sizeof(av));
    memcpy(&bv, &b, sizeof(bv));
    if (((av ^ bv) & sign) != 0)
        return (av & sign) != 0 && ((av | bv) & ~sign) != 0;
    return (av & sign) != 0 ? av > bv : av < bv;
#else
    /* A compiler that does not establish the IEEE byte layout keeps the
     * portable comparison at this input boundary. */
    return a < b;
#endif
}

/* high and low are finite, with high>=low. For a p-bit working significand,
 * distinct values with max magnitude >=2^(p+6) differ by at least 64, even
 * across the preceding binade. Both F31/F63 likelihoods then round to zero.
 * Otherwise each F40/F64 input uses at most 71/124 unsigned bits; their sum
 * fits 128 bits. Rounding the two inputs contributes at most one fixed unit
 * of difference error, inside the exponential's documented error budget. */
static int PQ(likelihood_delta_try)(U_WORK high, U_WORK low, uint64_t *out)
{
#if ((U_BITS == 15 && UKF_IEEE_BINARY32) || (U_BITS == 31 && UKF_IEEE_BINARY64)) || \
    (FLT_RADIX == 2 && U_MANT_DIG <= 53)
#if U_BITS == 15
    const pf_q_exp_format_t *format = &pf_q_exp_q15;
#else
    const pf_q_exp_format_t *format = &pf_q_exp_q31;
#endif
    pf_q_u128_t a, b, delta, cutoff;
    int negative_a, negative_b;
#if (U_BITS == 15 && UKF_IEEE_BINARY32) || (U_BITS == 31 && UKF_IEEE_BINARY64)
#if U_BITS == 15
    uint32_t av, bv, am, bm;
    unsigned ae, be;
    memcpy(&av, &high, sizeof(av));
    memcpy(&bv, &low, sizeof(bv));
    if (av == bv) { *out = format->likelihood_one; return 1; }
    negative_a = (int)(av >> 31); negative_b = (int)(bv >> 31);
    am = av & UINT32_C(0x7fffffff); bm = bv & UINT32_C(0x7fffffff);
    if (am >= UINT32_C(0x4e800000) || bm >= UINT32_C(0x4e800000)) { *out = 0; return 1; }
    ae = am >> 23; be = bm >> 23;
    a = pf_q_exp_mantissa((am & UINT32_C(0x007fffff)) | (ae ? UINT32_C(0x00800000) : 0),
                          ae ? (int)ae - 110 : -109);
    b = pf_q_exp_mantissa((bm & UINT32_C(0x007fffff)) | (be ? UINT32_C(0x00800000) : 0),
                          be ? (int)be - 110 : -109);
#else
    uint64_t av, bv, am, bm;
    unsigned ae, be;
    memcpy(&av, &high, sizeof(av));
    memcpy(&bv, &low, sizeof(bv));
    if (av == bv) { *out = format->likelihood_one; return 1; }
    negative_a = (int)(av >> 63); negative_b = (int)(bv >> 63);
    am = av & UINT64_C(0x7fffffffffffffff); bm = bv & UINT64_C(0x7fffffffffffffff);
    if (am >= UINT64_C(0x43a0000000000000) || bm >= UINT64_C(0x43a0000000000000)) { *out = 0; return 1; }
    ae = (unsigned)(am >> 52); be = (unsigned)(bm >> 52);
    a = pf_q_exp_mantissa((am & UINT64_C(0x000fffffffffffff)) | (ae ? UINT64_C(0x0010000000000000) : 0),
                          ae ? (int)ae - 1011 : -1010);
    b = pf_q_exp_mantissa((bm & UINT64_C(0x000fffffffffffff)) | (be ? UINT64_C(0x0010000000000000) : 0),
                          be ? (int)be - 1011 : -1010);
#endif
#elif FLT_RADIX == 2 && U_MANT_DIG <= 53
    {
        int ae, be;
        U_WORK am, bm, af, bf;
        uint64_t av, bv;
        if (high == low) { *out = format->likelihood_one; return 1; }
        negative_a = high < 0; negative_b = low < 0;
        am = negative_a ? -high : high; bm = negative_b ? -low : low;
        if (am >= U_LDEXP((U_WORK)1, U_MANT_DIG + 6) ||
            bm >= U_LDEXP((U_WORK)1, U_MANT_DIG + 6)) { *out = 0; return 1; }
        af = U_FREXP(am, &ae); bf = U_FREXP(bm, &be);
        av = (uint64_t)U_LDEXP(af, U_MANT_DIG); bv = (uint64_t)U_LDEXP(bf, U_MANT_DIG);
        a = pf_q_exp_mantissa(av, ae - U_MANT_DIG + (int)format->fractional);
        b = pf_q_exp_mantissa(bv, be - U_MANT_DIG + (int)format->fractional);
    }
#endif
    if (negative_a != negative_b) delta = pf_q_uadd(a, b);
    else delta = negative_a ? pf_q_exp_usub(b, a) : pf_q_exp_usub(a, b);
    if (delta.lo == 0 && delta.hi == 0) { *out = format->likelihood_one; return 1; }
    /* One ln(2)/32 interval beyond the half-cell cutoff is certainly zero,
     * including input/constant error. This also bounds the quotient before
     * entering the exponential's 64-bit range-reduction division. */
    cutoff = pf_q_umul(format->step, 32 * (format->likelihood_fraction + 1) + 1);
    if (pf_q_ucmp(delta, cutoff) >= 0) { *out = 0; return 1; }
#if U_BITS == 15
    return pf_q_exp_fixed_f40(delta, out);
#else
    return pf_q_exp_fixed(format, delta, out);
#endif
#else
    (void)high; (void)low; (void)out;
    return 0;
#endif
}

static uint64_t PQ(likelihood_pair)(U_WORK high, U_WORK low)
{
    uint64_t value;
    if (PQ(likelihood_delta_try)(high, low, &value)) return value;
    return PQ(likelihood)(high - low);
}
