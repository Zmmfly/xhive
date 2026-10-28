#include "filters_pf.h"
#include "../src/ukf_internal.h"

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cfenv>
#include <cmath>
#include <cstdint>
#include <limits>
#include <random>
#include <utility>

U_ALIGNMENT_TYPE(filters_pf_q15_t);
U_ALIGNMENT_TYPE(filters_pf_q31_t);
#define U_TAG q15
#define U_KERNEL f32
#define U_SIGNAL int16_t
#define U_SIGNAL_MAX INT16_MAX
#define U_WORK float
#define U_STATE filters_pf_q15_t
#define U_BITS 15
#define U_SCALE INT64_C(32768)
#define U_LIKELIHOOD_SCALE UINT64_C(2147483648)
#define U_EXP_NEG(x) expf(-(x))
#define U_FREXP frexpf
#define U_LDEXP ldexpf
#define U_MANT_DIG 24
#include "../src/pf_q_internal.h"
#undef U_TAG
#undef U_KERNEL
#undef U_SIGNAL
#undef U_SIGNAL_MAX
#undef U_WORK
#undef U_STATE
#undef U_BITS
#undef U_SCALE
#undef U_LIKELIHOOD_SCALE
#undef U_EXP_NEG
#undef U_FREXP
#undef U_LDEXP
#undef U_MANT_DIG

#define U_TAG q31
#define U_KERNEL f64
#define U_SIGNAL int32_t
#define U_SIGNAL_MAX INT32_MAX
#define U_WORK double
#define U_STATE filters_pf_q31_t
#define U_BITS 31
#define U_SCALE INT64_C(2147483648)
#define U_LIKELIHOOD_SCALE UINT64_C(9223372036854775808)
#define U_EXP_NEG(x) exp(-(x))
#define U_FREXP frexp
#define U_LDEXP ldexp
#define U_MANT_DIG 53
#include "../src/pf_q_internal.h"
#undef U_TAG
#undef U_KERNEL
#undef U_SIGNAL
#undef U_SIGNAL_MAX
#undef U_WORK
#undef U_STATE
#undef U_BITS
#undef U_SCALE
#undef U_LIKELIHOOD_SCALE
#undef U_EXP_NEG
#undef U_FREXP
#undef U_LDEXP
#undef U_MANT_DIG

namespace {

template<class W> struct Delta;
template<> struct Delta<float> {
    static constexpr auto less = pf_q_q15_exponent_less;
    static constexpr auto try_pair = pf_q_q15_likelihood_delta_try;
    static constexpr auto pair = pf_q_q15_likelihood_pair;
    static constexpr int fraction = 31;
    static constexpr long double bound = 0x1p-27L;
};
template<> struct Delta<double> {
    static constexpr auto less = pf_q_q31_exponent_less;
    static constexpr auto try_pair = pf_q_q31_likelihood_delta_try;
    static constexpr auto pair = pf_q_q31_likelihood_pair;
    static constexpr int fraction = 63;
    static constexpr long double bound = 0x1p-52L;
};

template<class W> class ReweightDelta : public ::testing::Test {};
using Formats = ::testing::Types<float, double>;
TYPED_TEST_SUITE(ReweightDelta, Formats);

TYPED_TEST(ReweightDelta, FiniteOrderingIncludesSignedZeroAndSubnormals) {
    using W = TypeParam;
    const W tiny = std::numeric_limits<W>::denorm_min();
    const W huge = std::numeric_limits<W>::max();
    const std::array<W, 11> values{-huge, W(-1000), W(-1), -tiny, W(-0.0), W(0), tiny, W(0.1), W(1), W(1000), huge};
    for (W a : values) for (W b : values)
        EXPECT_EQ(Delta<W>::less(a,b) != 0, a < b) << a << ' ' << b;
}

TYPED_TEST(ReweightDelta, ExactFixedDifferencesHaveCertifiedLikelihoods) {
    using W = TypeParam;
    using D = Delta<W>;
    const uint64_t one = UINT64_C(1) << D::fraction;
    const int p = std::numeric_limits<W>::digits;
    const W threshold = std::ldexp(W(1), p+6);
    const W huge = std::numeric_limits<W>::max();
    const W tiny = std::numeric_limits<W>::denorm_min();
    std::mt19937_64 random(0x44454c5441464958ULL + sizeof(W));
    unsigned accepted=0, retried=0;
    const auto check = [&](W high, W low) {
        ASSERT_GE(high, low);
        uint64_t value=19;
        const bool integer = D::try_pair(high,low,&value) != 0;
        if (integer) ++accepted;
        else { ++retried; value=D::pair(high,low); }
        const long double difference=static_cast<long double>(high)-static_cast<long double>(low);
        const long double expected=std::exp(-difference)*static_cast<long double>(one);
        EXPECT_LE(std::abs(static_cast<long double>(value)-expected), expected*D::bound+1.0L)
            << "high=" << high << " low=" << low << " integer=" << integer;
    };
    for (const auto& pair : {std::make_pair(W(0),-tiny), std::make_pair(tiny,-tiny),
                             std::make_pair(huge,huge), std::make_pair(-huge,-huge),
                             std::make_pair(huge,-huge), std::make_pair(threshold,std::nextafter(threshold,W(0))),
                             std::make_pair(-std::nextafter(threshold,W(0)),-threshold),
                             std::make_pair(W(1),std::nextafter(W(1),W(0))),
                             std::make_pair(W(-1),std::nextafter(W(-1),-huge))})
        check(pair.first,pair.second);
    for (unsigned i=0;i<20000;++i) {
        W a=static_cast<W>(std::uniform_real_distribution<long double>(-64,64)(random));
        W b=static_cast<W>(std::uniform_real_distribution<long double>(-64,64)(random));
        if (i%4==0) {
            const int shift=int(random()%200)-100;
            a=std::ldexp(a,shift);b=std::ldexp(b,shift);
        }
        check(std::max(a,b),std::min(a,b));
    }
    EXPECT_GT(accepted,19000u);
    ::testing::Test::RecordProperty("integer_differences",accepted);
    ::testing::Test::RecordProperty("boundary_retries",retried);
}

TYPED_TEST(ReweightDelta, NativeDifferenceIsIndependentOfHostRoundingMode) {
    using W = TypeParam;
    using D = Delta<W>;
    std::fenv_t saved;
    ASSERT_EQ(std::fegetenv(&saved),0);
    struct Restore { std::fenv_t *env; ~Restore() { std::fesetenv(env); } } restore{&saved};
    const std::array<std::pair<W,W>,5> inputs{{{W(0.125),W(-0.5)}, {W(-0.125),W(-0.25)},
        {std::numeric_limits<W>::max(),std::numeric_limits<W>::max()},
        {std::numeric_limits<W>::denorm_min(),W(0)}, {W(65),W(-65)}}};
    std::array<uint64_t,inputs.size()> baseline{},actual{};
    ASSERT_EQ(std::fesetround(FE_TONEAREST),0);
    for (size_t i=0;i<inputs.size();++i)
        ASSERT_TRUE(D::try_pair(inputs[i].first,inputs[i].second,&baseline[i]));
    for (int mode : {FE_UPWARD,FE_DOWNWARD,FE_TOWARDZERO}) {
        ASSERT_EQ(std::fesetround(mode),0);
        for (size_t i=0;i<inputs.size();++i) {
            ASSERT_TRUE(D::try_pair(inputs[i].first,inputs[i].second,&actual[i]));
            EXPECT_EQ(actual[i],baseline[i]) << mode << ' ' << i;
        }
    }
}

TEST(ReweightDelta, Q15UnsignedF40MatchesGenericInEveryRoundingMode) {
    std::fenv_t saved;
    ASSERT_EQ(std::fegetenv(&saved),0);
    struct Restore { std::fenv_t *env; ~Restore() { std::fesetenv(env); } } restore{&saved};
    const uint64_t step = pf_q_exp_q15.step;
    const std::array<uint64_t, 9> cells{0, 1, 2, 31, 32, 33, 511, 1023, 1024};
    const std::array<uint64_t, 5> remainders{0, 1, step / 2, step - 2, step - 1};
    for (int mode : {FE_TONEAREST, FE_UPWARD, FE_DOWNWARD, FE_TOWARDZERO}) {
        ASSERT_EQ(std::fesetround(mode),0);
        for (uint64_t k : cells) for (uint64_t remainder : remainders) {
            const pf_q_u128_t delta{k * step + remainder, 0};
            uint64_t expected = 0, actual = 0;
            const int old_status = pf_q_exp_fixed(&pf_q_exp_q15, delta, &expected);
            const int new_status = pf_q_exp_fixed_f40(delta, &actual);
            ASSERT_EQ(new_status, old_status) << mode << ' ' << k << ' ' << remainder;
            if (old_status) {
                ASSERT_EQ(actual, expected) << mode << ' ' << k << ' ' << remainder;
            }
        }
    }
}

TEST(ReweightDelta, Q31ExactOneProductStaysInteger) {
    uint64_t likelihood=0;
    ASSERT_TRUE(pf_q31_exp_neg_try(0x1p-66,&likelihood));
    EXPECT_EQ(likelihood,UINT64_C(1)<<63);
    for (unsigned power=0;power<62;++power) {
        const pf_q_u128_t delta=pf_q_umul(pf_q_exp_q31.step,32*power);
        ASSERT_TRUE(pf_q_exp_fixed(&pf_q_exp_q31,delta,&likelihood)) << power;
        EXPECT_EQ(likelihood,UINT64_C(1)<<(63-power)) << power;
    }
}

} // namespace
