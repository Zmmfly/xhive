#include "support.h"
#include <random>

namespace {
template<class T> class Transforms : public ::testing::Test {};
TYPED_TEST_SUITE(Transforms, fc_test::Types,);
constexpr double pi = 3.1415926535897932384626433832795;

TYPED_TEST(Transforms, CardinalAnglesWrapAndNegativeZero) {
    using A = fc_test::Api<TypeParam>;
    for (int n = A::fixed ? -4 : -16; n <= (A::fixed ? 3 : 16); ++n) {
        auto a = A::make(n / 4.0), sn = A::make(0.75), cs = A::make(0.75), wrapped = A::make(0.75);
        ASSERT_EQ(A::wrap(a, &wrapped), LINEARS_OK);
        EXPECT_DOUBLE_EQ(fc_test::value(wrapped), fc_test::wrap(n / 4.0));
        ASSERT_EQ(A::sincos(a, &sn, &cs), LINEARS_OK);
        const int k = (n % 4 + 4) % 4;
        EXPECT_DOUBLE_EQ(fc_test::value(sn), fc_test::value(A::make(k == 1 ? 1 : k == 3 ? -1 : 0)));
        EXPECT_DOUBLE_EQ(fc_test::value(cs), fc_test::value(A::make(k == 0 ? 1 : k == 2 ? -1 : 0)));
    }
    auto z = A::make(-0.0), out = A::make(0.75);
    ASSERT_EQ(A::wrap(z, &out), LINEARS_OK);
    EXPECT_FALSE(std::signbit(fc_test::value(out)));
    if constexpr (A::fixed) {
        EXPECT_EQ(z, 0);
        EXPECT_EQ(out, 0);
    }
}

TYPED_TEST(Transforms, FixedSeedTrigonometryAndVectoringAgainstIndependentLibm) {
    using A = fc_test::Api<TypeParam>;
    std::mt19937 rng(0x43524f54);
    std::uniform_real_distribution<double> turns(A::fixed ? -1 : -200, A::fixed ? 1 : 200);
    std::uniform_real_distribution<double> components(A::fixed ? -0.9 : -30, A::fixed ? 0.9 : 30);
    // Includes angle reduction and a native sin/cos or 32-step CORDIC error.
    const double trig_bound = A::fixed ? (std::is_same_v<TypeParam, fc_test::Q15> ? A::unit + 9.4e-10 : 2*A::unit)
                                      : (std::is_same_v<TypeParam, fc_test::F32> ? 8e-7 : 2e-15);
    for (int n = 0; n < 800; ++n) {
        const auto a = A::make(turns(rng));
        auto s = A::make(0), c = A::make(0), angle = A::make(0);
        ASSERT_EQ(A::sincos(a, &s, &c), LINEARS_OK);
        const double r = 2*pi*fc_test::wrap(fc_test::value(a));
        EXPECT_NEAR(fc_test::value(s), std::sin(r), trig_bound);
        EXPECT_NEAR(fc_test::value(c), std::cos(r), trig_bound);
        const auto x = A::make(components(rng)), y = A::make(components(rng));
        ASSERT_EQ(A::atan2(y, x, &angle), LINEARS_OK);
        const double reference = fc_test::wrap(std::atan2(fc_test::value(y), fc_test::value(x))/(2*pi));
        EXPECT_NEAR(fc_test::turn_error(fc_test::value(angle), reference), 0, 2*A::unit);
        EXPECT_GE(fc_test::value(angle), 0); EXPECT_LT(fc_test::value(angle), 1);
    }
}

TYPED_TEST(Transforms, ClarkeParkMatchIndependentEquationsAndRemoveZeroSequence) {
    using A = fc_test::Api<TypeParam>;
    std::mt19937 rng(0xc1a4ce);
    std::uniform_real_distribution<double> samples(A::fixed ? -0.375 : -12, A::fixed ? 0.375 : 12), turns(-1, 1);
    for (int n = 0; n < 400; ++n) {
        typename A::abc phase{A::make(samples(rng)), A::make(samples(rng)), A::make(samples(rng))};
        typename A::ab stationary{}, restored{};
        typename A::dq rotating{};
        typename A::abc balanced{};
        const auto angle = A::make(turns(rng));
        ASSERT_EQ(A::clarke(phase, &stationary), LINEARS_OK);
        const double a = fc_test::value(phase.a), b = fc_test::value(phase.b), c = fc_test::value(phase.c);
        EXPECT_NEAR(fc_test::value(stationary.alpha), (2*a-b-c)/3, 20*A::unit);
        EXPECT_NEAR(fc_test::value(stationary.beta), (b-c)/std::sqrt(3.0), 20*A::unit);
        ASSERT_EQ(A::inverse_clarke(stationary, &balanced), LINEARS_OK);
        EXPECT_NEAR(fc_test::value(balanced.a), a-(a+b+c)/3, 50*A::unit);
        EXPECT_NEAR(fc_test::value(balanced.b), b-(a+b+c)/3, 50*A::unit);
        EXPECT_NEAR(fc_test::value(balanced.c), c-(a+b+c)/3, 50*A::unit);
        ASSERT_EQ(A::park(stationary, angle, &rotating), LINEARS_OK);
        const double r = 2*pi*fc_test::wrap(fc_test::value(angle));
        const double alpha = fc_test::value(stationary.alpha), beta = fc_test::value(stationary.beta);
        EXPECT_NEAR(fc_test::value(rotating.d), alpha*std::cos(r)+beta*std::sin(r), 50*A::unit);
        EXPECT_NEAR(fc_test::value(rotating.q), -alpha*std::sin(r)+beta*std::cos(r), 50*A::unit);
        ASSERT_EQ(A::inverse_park(rotating, angle, &restored), LINEARS_OK);
        EXPECT_NEAR(fc_test::value(restored.alpha), alpha, 70*A::unit);
        EXPECT_NEAR(fc_test::value(restored.beta), beta, 70*A::unit);
    }
}

TYPED_TEST(Transforms, AtanAxesAndRawDynamicRange) {
    using A = fc_test::Api<TypeParam>;
    for (const auto &xy : {std::pair<double,double>{1,0}, {0,1}, {-1,0}, {0,-1}}) {
        auto a = A::make(0.75);
        ASSERT_EQ(A::atan2(A::make(xy.second), A::make(xy.first), &a), LINEARS_OK);
        EXPECT_DOUBLE_EQ(fc_test::value(a), fc_test::wrap(std::atan2(xy.second,xy.first)/(2*pi)));
    }
    if constexpr (A::fixed) {
        using S = typename A::scalar;
        const S large = std::numeric_limits<S>::max(), tiny = 1;
        auto out = A::make(0.75);
        ASSERT_EQ(A::atan2(tiny, large, &out), LINEARS_OK);
        EXPECT_NEAR(fc_test::value(out), 0, 2*A::unit);
        ASSERT_EQ(A::atan2(large, tiny, &out), LINEARS_OK);
        EXPECT_NEAR(fc_test::value(out), 0.25, 2*A::unit);
        ASSERT_EQ(A::atan2(S(-1), large, &out), LINEARS_OK);
        EXPECT_EQ(out, 0); // Nearest circular angle, not the ordinary +1 endpoint.
        ASSERT_EQ(A::wrap(S(-1), &out), LINEARS_OK);
        EXPECT_EQ(out, large); // An exact negative signal LSB maps to 1 - LSB.
    }
}

TYPED_TEST(Transforms, RejectInvalidWithoutTouchingOutputs) {
    using A = fc_test::Api<TypeParam>;
    auto out = A::make(0.3), saved = out, z = A::make(0);
    EXPECT_EQ(A::atan2(z, z, &out), LINEARS_ERROR_DOMAIN);
    EXPECT_TRUE(fc_test::same_bytes(saved, out));
    EXPECT_EQ(A::wrap(z, nullptr), LINEARS_ERROR_ARGUMENT);
    EXPECT_EQ(A::sincos(z, &out, &out), LINEARS_ERROR_ALIAS);
    EXPECT_TRUE(fc_test::same_bytes(saved, out));
    typename A::ab ab{A::make(A::fixed ? 0.25 : 1),A::make(A::fixed ? 0.5 : 2)};
    typename A::dq dq{A::make(A::fixed ? 0.125 : 3),A::make(A::fixed ? 0.25 : 4)}, saved_dq = dq;
    EXPECT_EQ(A::clarke({z,z,z}, nullptr), LINEARS_ERROR_ARGUMENT);
    if constexpr (A::fixed) {
        const auto hi = std::numeric_limits<typename A::scalar>::max();
        const auto lo = std::numeric_limits<typename A::scalar>::min();
        const auto saved_ab = ab;
        EXPECT_EQ(A::clarke({hi,lo,lo}, &ab), LINEARS_ERROR_RANGE);
        EXPECT_TRUE(fc_test::same_bytes(ab, saved_ab));
        EXPECT_EQ(A::park({hi,hi}, A::make(0.125), &dq), LINEARS_ERROR_RANGE);
        EXPECT_TRUE(fc_test::same_bytes(dq, saved_dq));
        typename A::abc phases{z,z,z}, saved_phases = phases;
        EXPECT_EQ(A::inverse_clarke({hi,hi}, &phases), LINEARS_ERROR_RANGE);
        EXPECT_TRUE(fc_test::same_bytes(phases, saved_phases));
    } else {
        auto invalid = std::numeric_limits<typename A::scalar>::quiet_NaN();
        EXPECT_EQ(A::park(ab, invalid, &dq), LINEARS_ERROR_RANGE);
        EXPECT_TRUE(fc_test::same_bytes(dq, saved_dq));
        EXPECT_EQ(A::wrap(invalid, &out), LINEARS_ERROR_RANGE);
        EXPECT_TRUE(fc_test::same_bytes(saved, out));
        for (double sign : {-1.0, 1.0}) {
            invalid = static_cast<typename A::scalar>(sign*std::numeric_limits<double>::infinity());
            EXPECT_EQ(A::atan2(invalid, z, &out), LINEARS_ERROR_RANGE);
            EXPECT_EQ(A::sincos(invalid, &out, &z), LINEARS_ERROR_RANGE);
            EXPECT_TRUE(fc_test::same_bytes(saved, out));
        }
    }
}
} // namespace
