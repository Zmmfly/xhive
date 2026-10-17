#include "support.h"
#include <array>

namespace {
template<class Tag> class AuditTurns : public ::testing::Test {};
TYPED_TEST_SUITE(AuditTurns, fc_test::Types);

/* Exhaust the signed Q15 angular lattice; every input is exactly representable
 * in all four APIs. This visits both sides of each quadrant and the negative
 * wrap seam without generating expectations from a production transform. */
TYPED_TEST(AuditTurns, SignedAngularLatticePreservesQuadrantsAndTurnUnits)
{
    using A = fc_test::Api<TypeParam>;
    const long double tau = 2 * std::acos(-1.0L);
    double largest_error = 0;
    for (int raw = -32768; raw <= 32767; ++raw) {
        const double angle = std::ldexp(static_cast<double>(raw), -15);
        typename A::scalar sine{}, cosine{}, wrapped{};
        ASSERT_EQ(A::sincos(A::make(angle), &sine, &cosine), LINEARS_OK);
        ASSERT_EQ(A::wrap(A::make(angle), &wrapped), LINEARS_OK);
        const double expected_wrap = angle < 0 ? angle + 1 : angle;
        ASSERT_EQ(fc_test::value(wrapped), expected_wrap) << "raw=" << raw;
        const long double s = std::sin(tau * angle), c = std::cos(tau * angle);
        largest_error = std::max(largest_error, static_cast<double>(std::max(
            std::abs(static_cast<long double>(fc_test::value(sine)) - s),
            std::abs(static_cast<long double>(fc_test::value(cosine)) - c))));
        /* Away from exact cardinal zeros, even the smallest lattice component
         * exceeds its numeric error budget by four orders of magnitude. */
        if (raw % 8192 != 0) {
            ASSERT_EQ(fc_test::value(sine) > 0, s > 0) << "raw=" << raw;
            ASSERT_EQ(fc_test::value(cosine) > 0, c > 0) << "raw=" << raw;
        }
    }
    /* Q15 final rounding <=2^-16 on [-1,1]; Q31 working rounding and the
     * 32-step CORDIC residual together are <1.5e-9. Native float uses libm. */
    const double bound = TypeParam::fixed
        ? (std::is_same_v<TypeParam, fc_test::Q15> ? 0x1p-16 : 0x1p-31) + 1.5e-9
        : 2 * TypeParam::unit;
    EXPECT_LE(largest_error, bound);
}

TYPED_TEST(AuditTurns, VectoringKeepsDirectionAcrossTheFullExponentRange)
{
    using A = fc_test::Api<TypeParam>;
    using S = typename A::scalar;
    const int small = [] {
        if constexpr (TypeParam::fixed) return LINEARS_EXPONENT_MIN;
        else return std::numeric_limits<S>::min_exponent - std::numeric_limits<S>::digits;
    }();
    const int large = [] {
        if constexpr (TypeParam::fixed) return LINEARS_EXPONENT_MAX - 3;
        else return std::numeric_limits<S>::max_exponent - 3;
    }();
    const auto scaled = [](int component, int exponent) -> S {
        if constexpr (TypeParam::fixed) {
            S result = A::make(component);
            if (result.mantissa != 0) result.exponent += exponent;
            return result;
        } else {
            return std::ldexp(static_cast<S>(component), exponent);
        }
    };
    const std::array<std::array<int, 2>, 8> rays{{
        {{2, 1}}, {{1, 2}}, {{-1, 2}}, {{-2, 1}},
        {{-2, -1}}, {{-1, -2}}, {{1, -2}}, {{2, -1}}
    }};
    for (const int exponent : {small, 0, large}) {
        for (const auto& ray : rays) {
            SCOPED_TRACE(::testing::Message() << "exponent=" << exponent
                << " x=" << ray[0] << " y=" << ray[1]);
            S actual{};
            ASSERT_EQ(A::atan2(scaled(ray[1], exponent), scaled(ray[0], exponent),
                &actual), LINEARS_OK);
            const long double expected = std::atan2(static_cast<long double>(ray[1]),
                static_cast<long double>(ray[0])) / (2 * std::acos(-1.0L));
            const double discrepancy = std::abs(std::remainder(
                fc_test::value(actual) - static_cast<double>(expected), 1.0));
            EXPECT_LE(discrepancy, TypeParam::unit + (TypeParam::fixed ? 1.5e-9 : 0));
        }
    }
}
} // namespace
