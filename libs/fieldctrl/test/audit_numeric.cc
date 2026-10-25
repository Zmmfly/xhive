#include "support.h"
#include <array>

namespace {
using FloatingTypes = ::testing::Types<fc_test::F32, fc_test::F64>;
template<class Tag> class AuditTurns : public ::testing::Test {};
TYPED_TEST_SUITE(AuditTurns, FloatingTypes);

/* Retain the native-float sweep of the signed Q15 angular lattice. It visits
 * both sides of every quadrant and the negative wrap seam without generating
 * expectations from a production transform. */
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
    EXPECT_LE(largest_error, 2 * TypeParam::unit);
}

TYPED_TEST(AuditTurns, VectoringKeepsDirectionAcrossTheFullExponentRange)
{
    using A = fc_test::Api<TypeParam>;
    using S = typename A::scalar;
    const int small = std::numeric_limits<S>::min_exponent - std::numeric_limits<S>::digits;
    const int large = std::numeric_limits<S>::max_exponent - 3;
    const auto scaled = [](int component, int exponent) -> S {
        return std::ldexp(static_cast<S>(component), exponent);
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
            EXPECT_LE(discrepancy, TypeParam::unit);
        }
    }
}

using FixedTypes = ::testing::Types<fc_test::Q15, fc_test::Q31>;
template<class Tag> class AuditFixedTurns : public ::testing::Test {};
TYPED_TEST_SUITE(AuditFixedTurns, FixedTypes);

TYPED_TEST(AuditFixedTurns, RawSignedAngularLatticePreservesQuadrantsAndEndpointPolicy)
{
    using A = fc_test::Api<TypeParam>;
    using S = typename A::scalar;
    constexpr int bits = std::numeric_limits<S>::digits;
    constexpr int64_t one = INT64_C(1) << bits;
    constexpr int64_t stride = INT64_C(1) << (bits - 15);
    const long double tau = 2 * std::acos(-1.0L);
    const long double quantum = 1.0L / one;
    long double largest_excess = -1;
    int worst_raw = 0;
    for (int raw = -32768; raw <= 32767; ++raw) {
        const S angle = static_cast<S>(int64_t(raw) * stride);
        S sine = 0, cosine = 0, wrapped = 0;
        ASSERT_EQ(A::sincos(angle, &sine, &cosine), LINEARS_OK);
        ASSERT_EQ(A::wrap(angle, &wrapped), LINEARS_OK);
        ASSERT_EQ(int64_t(wrapped), int64_t(raw < 0 ? raw + 32768 : raw) * stride)
            << "raw=" << raw;
        const long double turns = static_cast<long double>(raw) / 32768;
        const std::array<long double, 2> expected{{std::sin(tau * turns), std::cos(tau * turns)}};
        const std::array<S, 2> actual{{sine, cosine}};
        for (size_t i = 0; i < actual.size(); ++i) {
            /* Q's positive endpoint cell has a full-LSB error budget; all
             * other cells have half an LSB. The unchanged 32-round Q60/Q62
             * CORDIC plus F31 output rounding contributes <1.5e-9. */
            const long double rounding = expected[i] > 1 - quantum / 2
                ? quantum : quantum / 2;
            const long double error = std::abs(static_cast<long double>(actual[i]) / one - expected[i]);
            const long double excess = error - rounding - 1.5e-9L;
            if (excess > largest_excess) { largest_excess = excess; worst_raw = raw; }
        }
        if (raw % 8192 != 0) {
            ASSERT_EQ(sine > 0, expected[0] > 0) << "raw=" << raw;
            ASSERT_EQ(cosine > 0, expected[1] > 0) << "raw=" << raw;
        }
    }
    EXPECT_LE(largest_excess, 0) << "worst input lattice point=" << worst_raw;
}

TYPED_TEST(AuditFixedTurns, RawMinimumAdjacentTurnsAndCardinalsAreDefined)
{
    using A = fc_test::Api<TypeParam>;
    using S = typename A::scalar;
    constexpr int64_t one = INT64_C(1) << std::numeric_limits<S>::digits;
    constexpr int64_t quarter = one / 4;
    const std::array<int64_t, 17> angles{{-one, -one + 1, -2 * quarter - 1,
        -2 * quarter, -2 * quarter + 1, -quarter - 1, -quarter, -quarter + 1,
        -1, 0, 1, quarter - 1, quarter, quarter + 1, 2 * quarter, one - 2, one - 1}};
    for (const int64_t raw : angles) {
        S wrapped = 123, sine = 123, cosine = 123;
        ASSERT_EQ(A::wrap(static_cast<S>(raw), &wrapped), LINEARS_OK);
        EXPECT_EQ(int64_t(wrapped), raw < 0 ? raw + one : raw);
        ASSERT_EQ(A::sincos(static_cast<S>(raw), &sine, &cosine), LINEARS_OK);
        const long double phase = 2 * std::acos(-1.0L) * raw / one;
        const std::array<long double, 2> expected{{std::sin(phase), std::cos(phase)}};
        const std::array<S, 2> actual{{sine, cosine}};
        for (size_t i = 0; i < actual.size(); ++i) {
            const long double rounding = expected[i] > 1 - 0.5L / one ? 1.0L / one : 0.5L / one;
            EXPECT_LE(std::abs(static_cast<long double>(actual[i]) / one - expected[i]),
                rounding + 1.5e-9L) << "angle raw=" << raw;
        }
    }
    const std::array<S, 4> expected_sine{{0, std::numeric_limits<S>::max(),
        0, std::numeric_limits<S>::min()}};
    const std::array<S, 4> expected_cosine{{std::numeric_limits<S>::max(), 0,
        std::numeric_limits<S>::min(), 0}};
    for (int n = 0; n < 4; ++n) {
        S sine = 123, cosine = 123;
        ASSERT_EQ(A::sincos(static_cast<S>(n * quarter), &sine, &cosine), LINEARS_OK);
        EXPECT_EQ(sine, expected_sine[n]);
        EXPECT_EQ(cosine, expected_cosine[n]);
    }
    S angle = 123;
    ASSERT_EQ(A::atan2(std::numeric_limits<S>::min(), S(0), &angle), LINEARS_OK);
    EXPECT_EQ(int64_t(angle), 3 * quarter);
    ASSERT_EQ(A::atan2(S(0), std::numeric_limits<S>::min(), &angle), LINEARS_OK);
    EXPECT_EQ(int64_t(angle), 2 * quarter);
    /* An angle rounds across the turn seam, unlike a unit signal's +1
     * endpoint. -atan(1/MAX) is less than half an angular LSB; -atan(8/MAX)
     * is about -1.27 LSB, safely on the other side of that rounding cell. */
    ASSERT_EQ(A::atan2(S(-1), std::numeric_limits<S>::max(), &angle), LINEARS_OK);
    EXPECT_EQ(angle, 0);
    ASSERT_EQ(A::atan2(S(-8), std::numeric_limits<S>::max(), &angle), LINEARS_OK);
    EXPECT_EQ(angle, std::numeric_limits<S>::max());
    const S saved = angle;
    EXPECT_EQ(A::atan2(S(0), S(0), &angle), LINEARS_ERROR_DOMAIN);
    EXPECT_EQ(angle, saved);
    EXPECT_EQ(A::sincos(std::numeric_limits<S>::min(), &angle, &angle), LINEARS_ERROR_ALIAS);
    EXPECT_EQ(angle, saved);
}

TYPED_TEST(AuditFixedTurns, VectoringRawLsbAndNearFullScalePreservesDirection)
{
    using A = fc_test::Api<TypeParam>;
    using S = typename A::scalar;
    constexpr int64_t one = INT64_C(1) << std::numeric_limits<S>::digits;
    const std::array<std::array<int, 2>, 8> rays{{
        {{2, 1}}, {{1, 2}}, {{-1, 2}}, {{-2, 1}},
        {{-2, -1}}, {{-1, -2}}, {{1, -2}}, {{2, -1}}
    }};
    for (const int64_t scale : {INT64_C(1), INT64_C(2), (one - 1) / 2}) {
        for (const auto& ray : rays) {
            SCOPED_TRACE(::testing::Message() << "scale=" << scale
                << " x=" << ray[0] << " y=" << ray[1]);
            S actual = 0;
            ASSERT_EQ(A::atan2(static_cast<S>(ray[1] * scale),
                static_cast<S>(ray[0] * scale), &actual), LINEARS_OK);
            const long double expected = std::atan2(static_cast<long double>(ray[1]),
                static_cast<long double>(ray[0])) / (2 * std::acos(-1.0L));
            const long double error = std::abs(std::remainder(
                static_cast<long double>(actual) / one - expected, 1.0L));
            EXPECT_LE(error, 0.5L / one + 1.5e-9L);
        }
    }
}
} // namespace
