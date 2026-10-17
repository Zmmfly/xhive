#include "support.h"

namespace {
template<class Tag> class AuditPwm : public ::testing::Test {};
TYPED_TEST_SUITE(AuditPwm, fc_test::Types);

/* The reference is computed in normalized DC-bus units, not by invoking the
 * production voltage limiter, inverse Clarke, or modulation functions.
 * Regression snapshot: base_impl.h da0b803cfab4c40e55782381449034e25c522acafee5eb3fd55ed1bced965ea6. */
TYPED_TEST(AuditPwm, DutyRatiosDoNotDependOnCommonVoltageScale)
{
    using A = fc_test::Api<TypeParam>;
    using S = typename A::scalar;
    const auto scaled = [](int component, int exponent) -> S {
        if constexpr (TypeParam::fixed) {
            S result = A::make(component);
            if (result.mantissa != 0) result.exponent += exponent;
            return result;
        } else {
            return std::ldexp(static_cast<S>(component), exponent);
        }
    };
    const int small = [] {
        if constexpr (TypeParam::fixed) return LINEARS_EXPONENT_MIN + 4;
        else return std::numeric_limits<S>::min_exponent - std::numeric_limits<S>::digits;
    }();
    const int large = [] {
        if constexpr (TypeParam::fixed) return LINEARS_EXPONENT_MAX - 4;
        else return std::numeric_limits<S>::max_exponent - 1;
    }();
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

/* These vector components have independent exponents, not one matrix block.
 * lo/hi=2^-32768 cannot be represented by a working scalar, but its squared
 * contribution is far below half a Q31 unit: rounded hypot is exactly hi.
 * Regression snapshot: fieldctrl_arith.h 4961b24a6e3cb3de7bbbaffad1844da9bcf810cc7765e1260057e976de2f1b80. */
TYPED_TEST(AuditFixedPwm, NoClippingRetainsIndependentExtremeExponentComponents)
{
    using A = fc_test::Api<TypeParam>;
    typename A::ab voltage{A::make(0.5), A::make(0.5)};
    voltage.alpha.exponent = LINEARS_EXPONENT_MAX;
    voltage.beta.exponent = LINEARS_EXPONENT_MIN;
    typename A::ab limited{A::make(1), A::make(2)};
    ASSERT_EQ(A::limit_voltage(voltage, voltage.alpha, &limited), LINEARS_OK);
    EXPECT_EQ(limited.alpha.mantissa, voltage.alpha.mantissa);
    EXPECT_EQ(limited.alpha.exponent, voltage.alpha.exponent);
    EXPECT_EQ(limited.beta.mantissa, voltage.beta.mantissa);
    EXPECT_EQ(limited.beta.exponent, voltage.beta.exponent);
}
} // namespace
