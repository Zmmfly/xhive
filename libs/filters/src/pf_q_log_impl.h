/* Log of a checked Cholesky determinant without an absolute product. Each
 * positive diagonal is range-reduced to m*2^e, 1<=m<2, then evaluated on one
 * fixed grid. Only the final scalar sum returns to the working type. */
#if U_BITS == 15
#define PQ_LOG_F 30
#define PQ_LOG_TERMS 8
#define PQ_LOG_LN2 UINT64_C(744261118)
#define PQ_LOG_SUPPORTED (FLT_RADIX == 2 && FLT_MANT_DIG == 24 && FLT_MIN_EXP == -125 && FLT_MAX_EXP == 128)
#else
#define PQ_LOG_F 54
#define PQ_LOG_TERMS 17
#define PQ_LOG_LN2 UINT64_C(12486629536330718)
#define PQ_LOG_SUPPORTED (FLT_RADIX == 2 && DBL_MANT_DIG == 53 && DBL_MIN_EXP == -1021 && DBL_MAX_EXP == 1024)
#endif

#if PQ_LOG_SUPPORTED
#if U_BITS == 15
static const uint32_t PQ(log_coefficients)[PQ_LOG_TERMS] = {
    UINT32_C(1073741824), UINT32_C(357913941), UINT32_C(214748365), UINT32_C(153391689),
    UINT32_C(119304647), UINT32_C(97612893), UINT32_C(82595525), UINT32_C(71582788)
};
#else
static const uint64_t PQ(log_coefficients)[PQ_LOG_TERMS] = {
    UINT64_C(18014398509481984), UINT64_C(6004799503160661), UINT64_C(3602879701896397),
    UINT64_C(2573485501354569), UINT64_C(2001599834386887), UINT64_C(1637672591771089),
    UINT64_C(1385722962267845), UINT64_C(1200959900632132), UINT64_C(1059670500557764),
    UINT64_C(948126237341157), UINT64_C(857828500451523), UINT64_C(783234717803565),
    UINT64_C(720575940379279), UINT64_C(667199944795629), UINT64_C(621186155499379),
    UINT64_C(581109629338129), UINT64_C(545890863923696)
};
#endif

/* value is strictly positive and finite: the caller's Cholesky check already
 * established this. Normalization is exact, including subnormal diagonals. */
static uint64_t PQ(log_input)(U_WORK value, int *exponent)
{
#if U_BITS == 15 && UKF_IEEE_BINARY32
    uint32_t bits, mantissa;
    unsigned encoded;
    memcpy(&bits, &value, sizeof(bits));
    encoded = bits >> 23;
    mantissa = bits & UINT32_C(0x007fffff);
    if (encoded == 0) {
        *exponent = -126;
        while (mantissa < UINT32_C(0x00800000)) { mantissa <<= 1; --*exponent; }
    } else {
        *exponent = (int)encoded - 127;
        mantissa |= UINT32_C(0x00800000);
    }
    return (uint64_t)mantissa << (PQ_LOG_F - 23);
#elif U_BITS == 31 && UKF_IEEE_BINARY64
    uint64_t bits, mantissa;
    unsigned encoded;
    memcpy(&bits, &value, sizeof(bits));
    encoded = (unsigned)(bits >> 52);
    mantissa = bits & UINT64_C(0x000fffffffffffff);
    if (encoded == 0) {
        *exponent = -1022;
        while (mantissa < UINT64_C(0x0010000000000000)) { mantissa <<= 1; --*exponent; }
    } else {
        *exponent = (int)encoded - 1023;
        mantissa |= UINT64_C(0x0010000000000000);
    }
    return mantissa << (PQ_LOG_F - 52);
#else
    U_WORK fraction = U_FREXP(value, exponent);
    --*exponent;
    return (uint64_t)U_LDEXP(fraction, PQ_LOG_F + 1);
#endif
}

/* Positive fixed multiplication; operands are at most 1.125 and 1/3 in
 * polynomial products, or both at most 1/3 when forming t^2. */
static uint64_t PQ(log_product)(uint64_t a, uint64_t b)
{
#if U_BITS == 15
    return (a * b + (UINT64_C(1) << (PQ_LOG_F - 1))) >> PQ_LOG_F;
#else
    return pf_q_uround(pf_q_umul(a, b), PQ_LOG_F);
#endif
}

/* log(m)=2*atanh(t), t=(m-1)/(m+1), 0<=t<1/3. Rounded coefficient/Horner,
 * t and t^2 errors contribute less than 8 fixed units. Truncation tails are
 * below 1.025e-9 (8 terms) and 1.285e-18 (17 terms). ln2 quantization adds
 * at most |sum e|/2 fixed units to the complete determinant sum. */
static uint64_t PQ(log_mantissa)(uint64_t mantissa)
{
    const uint64_t one = UINT64_C(1) << PQ_LOG_F;
    pf_q_u128_t numerator;
    uint64_t t, squared, polynomial;
    unsigned i = PQ_LOG_TERMS - 1;
    if (mantissa == one) return 0;
#if U_BITS == 15
    numerator.lo = (mantissa - one) << PQ_LOG_F;
    numerator.hi = 0;
#else
    numerator = pf_q_exp_shl(PQ(from_u64)(mantissa - one), PQ_LOG_F);
#endif
    /* The positive denominator is at least 2*one and t<one/3, so the
     * shared exact divider cannot overflow its 64-bit quotient. */
    (void)PQ(divide)(numerator, mantissa + one, 1, &t);
    squared = PQ(log_product)(t, t);
    polynomial = PQ(log_coefficients)[i];
    while (i-- != 0)
        polynomial = PQ(log_coefficients)[i] + PQ(log_product)(polynomial, squared);
    return 2 * PQ(log_product)(polynomial, t);
}
#endif

/* lower is a checked finite positive-diagonal factor with a validated k*k
 * workspace extent. Exponents lie in [-149,127]/[-1074,1023]; each e*ln2
 * magnitude fits uint64_t. The dimension bound keeps the exact signed sum
 * below 2^96, also finite before its single final working-type conversion. */
static int PQ(log_determinant_try)(const U_WORK *lower, size_t k, U_WORK *out)
{
#if PQ_LOG_SUPPORTED
    pf_q_i128_t sum = {0, 0};
    size_t i;
    for (i = 0; i < k; ++i) {
        int exponent;
        uint64_t mantissa = PQ(log_input)(lower[i * k + i], &exponent);
        pf_q_iadd(&sum, PQ(log_mantissa)(mantissa), 0);
        pf_q_iadd(&sum, (uint64_t)(exponent < 0 ? -exponent : exponent) * PQ_LOG_LN2, exponent < 0);
    }
    *out = U_LDEXP(PQ(signed_value)(sum), -PQ_LOG_F);
    return 1;
#else
    (void)lower; (void)k; (void)out;
    return 0;
#endif
}

#undef PQ_LOG_SUPPORTED
#undef PQ_LOG_LN2
#undef PQ_LOG_TERMS
#undef PQ_LOG_F
