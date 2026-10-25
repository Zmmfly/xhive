#include "support.h"
#include <array>

namespace {
using FloatingTypes = ::testing::Types<fc_test::F32, fc_test::F64>;
template<class Tag> class AuditPwm : public ::testing::Test {};
TYPED_TEST_SUITE(AuditPwm, FloatingTypes);

/* The reference is computed in normalized DC-bus units, not by invoking the
 * production voltage limiter, inverse Clarke, or modulation functions.
 * Regression snapshot: base_impl.h da0b803cfab4c40e55782381449034e25c522acafee5eb3fd55ed1bced965ea6. */
TYPED_TEST(AuditPwm, DutyRatiosDoNotDependOnCommonVoltageScale)
{
    using A = fc_test::Api<TypeParam>;
    using S = typename A::scalar;
    const auto scaled = [](int component, int exponent) -> S {
        return std::ldexp(static_cast<S>(component), exponent);
    };
    const int small = std::numeric_limits<S>::min_exponent - std::numeric_limits<S>::digits;
    const int large = std::numeric_limits<S>::max_exponent - 1;
    for (const int exponent : {small, 0, large}) {
        for (const int direction : {-1, 1}) {
            SCOPED_TRACE(::testing::Message() << "exponent=" << exponent
                << " direction=" << direction);
            typename A::abc duty{};
            ASSERT_EQ(A::svpwm({scaled(direction, exponent), A::make(0)},
                scaled(1, exponent), &duty), LINEARS_OK);
            const double excursion = direction * std::sqrt(3.0) / 4;
            EXPECT_NEAR(fc_test::value(duty.a), 0.5 + excursion, 4 * TypeParam::unit);
            EXPECT_NEAR(fc_test::value(duty.b), 0.5 - excursion, 4 * TypeParam::unit);
            EXPECT_NEAR(fc_test::value(duty.c), 0.5 - excursion, 4 * TypeParam::unit);
        }
    }
}

TYPED_TEST(AuditPwm, RadialLimiterPreservesRepresentableProductsAcrossScaleSeparation)
{
    using A = fc_test::Api<TypeParam>;
    const double tiny = std::is_same_v<TypeParam, fc_test::F64> ? 0x1p-600 : 0x1p-80;
    const double huge = 1 / tiny;
    for (const bool tiny_component : {false, true}) {
        SCOPED_TRACE(tiny_component);
        const double radius = tiny_component ? huge / 2 : tiny;
        const typename A::ab request{A::make(tiny_component ? tiny : huge),
            A::make(tiny_component ? huge : 0)};
        typename A::ab limited{};
        ASSERT_EQ(A::limit_voltage(request, A::make(radius), &limited), LINEARS_OK);
        /* For the unequal-component ray, hypot(tiny,huge) rounds to huge;
         * its exact relative correction is below every format's precision. */
        EXPECT_NEAR(fc_test::value(limited.alpha) / (tiny_component ? tiny / 2 : radius),
            1, 4 * TypeParam::unit);
        if (tiny_component)
            EXPECT_NEAR(fc_test::value(limited.beta) / radius, 1, 4 * TypeParam::unit);
        else
            EXPECT_EQ(fc_test::value(limited.beta), 0);
    }
}

template<class Tag> class AuditFixedPwm : public ::testing::Test {};
using FixedTypes = ::testing::Types<fc_test::Q15, fc_test::Q31>;
TYPED_TEST_SUITE(AuditFixedPwm, FixedTypes);

/* No clipping must preserve a one-LSB component beside a near-full-scale
 * component; a rounded vector norm must not erase the smaller input. */
TYPED_TEST(AuditFixedPwm, NoClippingRetainsRawLsbBesideNearFullScale)
{
    using A = fc_test::Api<TypeParam>;
    using S = typename A::scalar;
    constexpr S maximum = std::numeric_limits<S>::max();
    const std::array<typename A::ab, 8> voltages{{
        {S(1), S(0)}, {S(0), S(-1)}, {S(1), S(-1)}, {S(-1), S(1)},
        {S(maximum - 1), S(1)}, {S(-maximum + 1), S(-1)},
        {S(maximum), S(0)}, {S(0), S(-maximum)}
    }};
    for (const auto voltage : voltages) {
        typename A::ab limited{S(123), S(234)};
        ASSERT_EQ(A::limit_voltage(voltage, maximum, &limited), LINEARS_OK);
        EXPECT_EQ(limited.alpha, voltage.alpha);
        EXPECT_EQ(limited.beta, voltage.beta);
    }
}

TYPED_TEST(AuditFixedPwm, OneLsbBusKeepsDimensionlessDutyAndZeroCommand)
{
    using A = fc_test::Api<TypeParam>;
    using S = typename A::scalar;
    constexpr int64_t one = INT64_C(1) << std::numeric_limits<S>::digits;
    for (const int64_t bus : {INT64_C(1), INT64_C(2), one / 2, one - 1}) {
        for (const int direction : {-1, 1}) {
            SCOPED_TRACE(::testing::Message() << "bus raw=" << bus << " sign=" << direction);
            typename A::abc duty{};
            ASSERT_EQ(A::svpwm({S(direction * bus), S(0)}, S(bus), &duty), LINEARS_OK);
            const long double excursion = direction * std::sqrt(3.0L) / 4;
            /* With beta=0: sqrt/div and the two halved common-mode stages
             * contribute at most three F31 units, then one signal rounding. */
            const long double budget = 3.0L / LINEARS_FIXED_ONE + 0.5L / one;
            EXPECT_LE(std::abs(static_cast<long double>(duty.a) / one - (0.5L + excursion)), budget);
            EXPECT_LE(std::abs(static_cast<long double>(duty.b) / one - (0.5L - excursion)), budget);
            EXPECT_LE(std::abs(static_cast<long double>(duty.c) / one - (0.5L - excursion)), budget);
            ASSERT_EQ(A::svpwm({S(0), S(0)}, S(bus), &duty), LINEARS_OK);
            EXPECT_EQ(int64_t(duty.a), one / 2);
            EXPECT_EQ(int64_t(duty.b), one / 2);
            EXPECT_EQ(int64_t(duty.c), one / 2);
        }
    }
}

TYPED_TEST(AuditFixedPwm, NegativeFullScaleLimitsWithoutLosingSignAndErrorsAreAtomic)
{
    using A = fc_test::Api<TypeParam>;
    using S = typename A::scalar;
    constexpr int64_t one = INT64_C(1) << std::numeric_limits<S>::digits;
    const typename A::ab voltage{std::numeric_limits<S>::min(), S(0)};
    typename A::ab limited{};
    for (const int64_t radius : {INT64_C(0), INT64_C(1), one / 2, one - 1}) {
        ASSERT_EQ(A::limit_voltage(voltage, S(radius), &limited), LINEARS_OK);
        EXPECT_EQ(int64_t(limited.alpha), -radius);
        EXPECT_EQ(limited.beta, 0);
    }
    const auto saved = limited;
    EXPECT_EQ(A::limit_voltage(voltage, S(-1), &limited), LINEARS_ERROR_DOMAIN);
    EXPECT_TRUE(fc_test::same_bytes(limited, saved));
    typename A::abc duty{S(123), S(234), S(345)};
    const auto saved_duty = duty;
    EXPECT_EQ(A::svpwm(voltage, S(0), &duty), LINEARS_ERROR_DOMAIN);
    EXPECT_TRUE(fc_test::same_bytes(duty, saved_duty));
}
} // namespace
