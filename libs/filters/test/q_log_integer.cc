#include "filters_pf.h"
#include "../src/ukf_internal.h"

#include <gtest/gtest.h>

#include <array>
#include <cfenv>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <random>
#include <vector>

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

template<class W> struct Log;
template<> struct Log<float> {
    static constexpr auto determinant = pf_q_q15_log_determinant_try;
    static constexpr auto input = pf_q_q15_log_input;
    static constexpr auto mantissa = pf_q_q15_log_mantissa;
    static constexpr int fraction = 30;
    static constexpr long double tail = 1.025e-9L;
};
template<> struct Log<double> {
    static constexpr auto determinant = pf_q_q31_log_determinant_try;
    static constexpr auto input = pf_q_q31_log_input;
    static constexpr auto mantissa = pf_q_q31_log_mantissa;
    static constexpr int fraction = 54;
    static constexpr long double tail = 1.285e-18L;
};

template<class W> W check(const std::vector<W>& diagonal) {
    const size_t k=diagonal.size();
    std::vector<W> lower(k*k,W(0));
    long double mantissa_logs=0;
    int exponent_sum=0;
    for (size_t i=0;i<k;++i) {
        lower[i*k+i]=diagonal[i];
        int exponent;
        const long double fraction=std::frexp(static_cast<long double>(diagonal[i]),&exponent);
        mantissa_logs+=std::log(2*fraction);
        exponent_sum+=exponent-1;
    }
    const long double reference=mantissa_logs+exponent_sum*std::log(2.0L);
    W actual=W(99);
    EXPECT_TRUE(Log<W>::determinant(lower.data(),k,&actual));
    const long double allowance=std::ldexp(8.0L*k+0.5L*std::abs(exponent_sum),-Log<W>::fraction)
        +k*Log<W>::tail+4*std::numeric_limits<W>::epsilon()*std::abs(reference);
    EXPECT_LE(std::abs(static_cast<long double>(actual)-reference),allowance)
        << "k=" << k << " actual=" << actual << " expected=" << reference;
    return actual;
}

template<class W> class QLogInteger : public ::testing::Test {};
using Formats=::testing::Types<float,double>;
TYPED_TEST_SUITE(QLogInteger,Formats);

TYPED_TEST(QLogInteger, PowersOfTwoCoverTheWholeFiniteExponentRange) {
    using W=TypeParam;
    for (int e=std::numeric_limits<W>::min_exponent-std::numeric_limits<W>::digits;
         e<std::numeric_limits<W>::max_exponent;++e) {
        const W value=std::ldexp(W(1),e);
        int exponent=0;
        EXPECT_EQ(Log<W>::input(value,&exponent),UINT64_C(1)<<Log<W>::fraction);
        EXPECT_EQ(exponent,e);
        check<W>({value});
    }
    const int large=sizeof(W)==4 ? 120 : 1000;
    EXPECT_EQ(check<W>({std::ldexp(W(1),large),std::ldexp(W(1),-large)}),W(0));
    EXPECT_EQ(check<W>({W(1),W(1),W(1)}),W(0));
}

TYPED_TEST(QLogInteger, NearOneAndAbsoluteProductUnderflowStayInteger) {
    using W=TypeParam;
    check<W>({std::nextafter(W(1),W(0))});
    check<W>({std::nextafter(W(1),W(2))});
    check<W>({std::numeric_limits<W>::denorm_min(),std::numeric_limits<W>::max()});
    const int large=sizeof(W)==4 ? 120 : 1000;
    const W tiny=std::ldexp(W(0.75),-large);
    EXPECT_EQ(W(tiny*tiny),W(0));
    EXPECT_TRUE(std::isfinite(check<W>({tiny,tiny,tiny,tiny})));
    check<W>({std::ldexp(W(0.75),large),std::ldexp(W(4.0/3.0),-large)});
    check<W>({W(0.25),W(0.3),W(0.7),W(1.75)});
}

TYPED_TEST(QLogInteger, BroadPositiveFactorsMatchTheErrorBudget) {
    using W=TypeParam;
    std::mt19937_64 random(0x4c4f47494e544547ULL+sizeof(W));
    constexpr unsigned trials=20000;
    for (unsigned trial=0;trial<trials;++trial) {
        std::vector<W> diagonal(1+random()%8);
        for (W& value:diagonal) {
            for (;;) {
                if constexpr (sizeof(W)==4) {
                    const uint32_t bits=static_cast<uint32_t>(random())&UINT32_C(0x7fffffff);
                    if (!bits || (bits&UINT32_C(0x7f800000))==UINT32_C(0x7f800000)) continue;
                    std::memcpy(&value,&bits,sizeof(value));
                } else {
                    const uint64_t bits=random()&UINT64_C(0x7fffffffffffffff);
                    if (!bits || (bits&UINT64_C(0x7ff0000000000000))==UINT64_C(0x7ff0000000000000)) continue;
                    std::memcpy(&value,&bits,sizeof(value));
                }
                break;
            }
        }
        check(diagonal);
    }
    ::testing::Test::RecordProperty("integer_factor_sets",trials);
}

TYPED_TEST(QLogInteger, NormalizedPolynomialIsIndependentOfRoundingMode) {
    using W=TypeParam;
    std::fenv_t saved;
    ASSERT_EQ(std::fegetenv(&saved),0);
    struct Restore { std::fenv_t *env; ~Restore() { std::fesetenv(env); } } restore{&saved};
    std::array<W,5> values{W(1),W(0.3),W(0.75),std::numeric_limits<W>::denorm_min(),std::numeric_limits<W>::max()};
    std::array<uint64_t,5> original{};
    ASSERT_EQ(std::fesetround(FE_TONEAREST),0);
    for (size_t i=0;i<values.size();++i) {
        int exponent;
        original[i]=Log<W>::mantissa(Log<W>::input(values[i],&exponent));
    }
    for (int mode:{FE_UPWARD,FE_DOWNWARD,FE_TOWARDZERO}) {
        ASSERT_EQ(std::fesetround(mode),0);
        for (size_t i=0;i<values.size();++i) {
            int exponent;
            EXPECT_EQ(Log<W>::mantissa(Log<W>::input(values[i],&exponent)),original[i]);
        }
    }
}

} // namespace
