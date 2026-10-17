#include "support.h"
#include <random>

namespace {
template<class T> class Transforms : public ::testing::Test {};
TYPED_TEST_SUITE(Transforms, fc_test::Types);
constexpr double pi = 3.1415926535897932384626433832795;

TYPED_TEST(Transforms, CardinalAnglesWrapAndNegativeZero) {
    using A = fc_test::Api<TypeParam>;
    for (int n = -16; n <= 16; ++n) {
        auto a = A::make(n / 4.0), sn = A::make(9), cs = A::make(9), wrapped = A::make(9);
        ASSERT_EQ(A::wrap(a, &wrapped), LINEARS_OK);
        EXPECT_DOUBLE_EQ(fc_test::value(wrapped), fc_test::wrap(n / 4.0));
        ASSERT_EQ(A::sincos(a, &sn, &cs), LINEARS_OK);
        const int k = (n % 4 + 4) % 4;
        EXPECT_DOUBLE_EQ(fc_test::value(sn), k == 1 ? 1 : k == 3 ? -1 : 0);
        EXPECT_DOUBLE_EQ(fc_test::value(cs), k == 0 ? 1 : k == 2 ? -1 : 0);
    }
    auto z = A::make(-0.0), out = A::make(2);
    ASSERT_EQ(A::wrap(z, &out), LINEARS_OK);
    EXPECT_FALSE(std::signbit(fc_test::value(out)));
    if constexpr (A::fixed) {
        z.exponent = 100;
        ASSERT_EQ(A::wrap(z, &out), LINEARS_OK);
        EXPECT_EQ(out.mantissa, 0);
        EXPECT_EQ(out.exponent, 0);
    }
}

TYPED_TEST(Transforms, FixedSeedTrigonometryAndVectoringAgainstIndependentLibm) {
    using A = fc_test::Api<TypeParam>;
    std::mt19937 rng(0x43524f54);
    std::uniform_real_distribution<double> turns(-200, 200), components(-30, 30);
    // Includes angle reduction and a native sin/cos or 32-step CORDIC error.
    const double trig_bound = A::fixed ? (std::is_same_v<TypeParam, fc_test::Q15> ? 3.2e-5 : 1.8e-9)
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
    std::uniform_real_distribution<double> samples(-12, 12), turns(-1, 1);
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

TYPED_TEST(Transforms, AtanAxesAndExponentSeparation) {
    using A = fc_test::Api<TypeParam>;
    for (const auto &xy : {std::pair<double,double>{1,0}, {0,1}, {-1,0}, {0,-1}}) {
        auto a = A::make(9);
        ASSERT_EQ(A::atan2(A::make(xy.second), A::make(xy.first), &a), LINEARS_OK);
        EXPECT_DOUBLE_EQ(fc_test::value(a), fc_test::wrap(std::atan2(xy.second,xy.first)/(2*pi)));
    }
    if constexpr (A::fixed) {
        auto huge = A::make(1), tiny = A::make(1), out = A::make(9);
        huge.exponent = 1000; tiny.exponent = -1000;
        ASSERT_EQ(A::atan2(tiny, huge, &out), LINEARS_OK);
        EXPECT_NEAR(fc_test::value(out), 0, 2*A::unit);
        ASSERT_EQ(A::atan2(huge, tiny, &out), LINEARS_OK);
        EXPECT_NEAR(fc_test::value(out), 0.25, 2*A::unit);
        auto near_one = A::make(-0.000000001);
        ASSERT_EQ(A::wrap(near_one, &out), LINEARS_OK);
        EXPECT_GE(fc_test::value(out), 0); EXPECT_LT(fc_test::value(out), 1);
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
    typename A::ab ab{A::make(1),A::make(2)};
    typename A::dq dq{A::make(3),A::make(4)}, saved_dq = dq;
    EXPECT_EQ(A::clarke({z,z,z}, nullptr), LINEARS_ERROR_ARGUMENT);
    auto invalid = z;
    if constexpr (A::fixed) invalid.exponent = LINEARS_EXPONENT_MAX + 1;
    else invalid = std::numeric_limits<typename A::scalar>::quiet_NaN();
    EXPECT_EQ(A::park(ab, invalid, &dq), LINEARS_ERROR_RANGE);
    EXPECT_TRUE(fc_test::same_bytes(dq, saved_dq));
    EXPECT_EQ(A::wrap(invalid, &out), LINEARS_ERROR_RANGE);
    EXPECT_TRUE(fc_test::same_bytes(saved, out));
    if constexpr (!A::fixed) {
        for (double sign : {-1.0, 1.0}) {
            invalid = static_cast<typename A::scalar>(sign*std::numeric_limits<double>::infinity());
            EXPECT_EQ(A::atan2(invalid, z, &out), LINEARS_ERROR_RANGE);
            EXPECT_EQ(A::sincos(invalid, &out, &z), LINEARS_ERROR_RANGE);
            EXPECT_TRUE(fc_test::same_bytes(saved, out));
        }
    }
}
} // namespace
