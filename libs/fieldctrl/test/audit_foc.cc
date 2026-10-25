#include "foc_smo_support.h"

namespace {
using FloatingTypes = ::testing::Types<fc_test::F32, fc_test::F64>;
template<class Tag> class AuditFoc : public ::testing::Test {};
TYPED_TEST_SUITE(AuditFoc, FloatingTypes);

/* Scaling every current/voltage/bus quantity must not disable anti-windup.
 * All nonzero float inputs and retained states below are NORMAL, but the
 * dimensional delta-I dot voltage product underflows in the old predicate.
 * Regression snapshot: foc_impl.h 639cdae7b9210d72d343694bb780c8307bd4813ed10225927aac54e0594f6432. */
TYPED_TEST(AuditFoc, SmallUnitsDoNotTurnOffRadialAntiWindup)
{
    using A = fc_loop_test::Api<TypeParam>;
    const double radius = std::is_same_v<TypeParam, fc_test::F64> ? 0x1p-600 : 0x1p-80;
    for (const double scale : {1.0, radius}) {
        SCOPED_TRACE(::testing::Message() << "scale=" << scale);
        const typename A::foc_config config{A::make(0), A::make(1),
            A::make(0), A::make(1), A::make(scale)};
        typename A::foc ctx{};
        typename A::foc_output output{};
        ASSERT_EQ(A::foc_init(&ctx, &config), LINEARS_OK);
        const typename A::ab current{A::make(0), A::make(0)};
        const typename A::dq reference{A::make(scale), A::make(0)};
        for (int n = 0; n < 8; ++n) {
            ASSERT_EQ(A::foc_step(&ctx, current, reference, A::make(0),
                A::make(4 * scale), A::make(1), &output), LINEARS_OK);
            /* I[1]=radius is admissible. Every following proposed 2*radius
             * points strictly outward and must leave I at radius. */
            EXPECT_EQ(fc_test::value(ctx.integral.d) / scale, 1);
            EXPECT_EQ(fc_test::value(ctx.integral.q), 0);
            EXPECT_NEAR(fc_test::value(output.voltage.d) / scale, 1,
                4 * TypeParam::unit);
        }
    }
}

TYPED_TEST(AuditFoc, SmallSpeedErrorAndVoltageResidualStillCancelOuterIntegral)
{
    using A = fc_loop_test::Api<TypeParam>;
    const double tiny = std::is_same_v<TypeParam, fc_test::F64> ? 0x1p-600 : 0x1p-80;
    for (const double scale : {1.0, tiny}) {
        SCOPED_TRACE(::testing::Message() << "scale=" << scale);
        const typename A::speed_config config{
            {A::make(1), A::make(0), A::make(1), A::make(0), A::make(scale)},
            A::make(2), A::make(1), A::make(4 * scale), 1};
        typename A::speed ctx{};
        typename A::speed_output output{};
        ASSERT_EQ(A::speed_init(&ctx, &config), LINEARS_OK);
        for (int n = 0; n < 8; ++n) {
            ASSERT_EQ(A::speed_step(&ctx, {A::make(0), A::make(0)},
                A::make(scale), A::make(0), A::make(0), A::make(4 * scale),
                A::make(1), &output), LINEARS_OK);
            /* The unsaturated speed PI asks for 3*r amperes, but the inner
             * loop can produce only r volts. Matching positive signs must
             * cancel the outer increment without multiplying tiny values. */
            ASSERT_TRUE(output.current.saturated);
            EXPECT_EQ(fc_test::value(ctx.speed.integral), 0);
            EXPECT_NEAR(fc_test::value(output.reference.q) / scale, 3,
                4 * TypeParam::unit);
        }
    }
}

/* Neither radius/norm nor component/norm is necessarily representable even
 * when component*radius/norm is. Two opposite scale separations prevent an
 * order-of-operations fix from merely moving the underflow to another input. */
TYPED_TEST(AuditFoc, RadialScalingPreservesBothTinyRadiusAndTinyComponent)
{
    using A = fc_loop_test::Api<TypeParam>;
    const double tiny = std::is_same_v<TypeParam, fc_test::F64> ? 0x1p-600 : 0x1p-80;
    const double huge = 1 / tiny;
    for (const bool tiny_component : {false, true}) {
        SCOPED_TRACE(tiny_component);
        const double radius = tiny_component ? huge / 2 : tiny;
        const typename A::foc_config config{A::make(1), A::make(0),
            A::make(1), A::make(0), A::make(radius)};
        typename A::foc ctx{};
        typename A::foc_output output{};
        ASSERT_EQ(A::foc_init(&ctx, &config), LINEARS_OK);
        const typename A::dq request{A::make(tiny_component ? tiny : huge),
            A::make(tiny_component ? huge : 0)};
        ASSERT_EQ(A::foc_step(&ctx, {A::make(0), A::make(0)}, request,
            A::make(0), A::make(4 * radius), A::make(1), &output), LINEARS_OK);
        ASSERT_TRUE(output.saturated);
        const double expected_d = tiny_component ? tiny / 2 : radius;
        EXPECT_NEAR(fc_test::value(output.voltage.d) / expected_d, 1,
            4 * TypeParam::unit);
        if (tiny_component)
            EXPECT_NEAR(fc_test::value(output.voltage.q) / radius, 1,
                4 * TypeParam::unit);
        else
            EXPECT_EQ(fc_test::value(output.voltage.q), 0);
    }
}

using FixedTypes = ::testing::Types<fc_test::Q15, fc_test::Q31>;
template<class Tag> class AuditFixedFoc : public ::testing::Test {};
TYPED_TEST_SUITE(AuditFixedFoc, FixedTypes);

TYPED_TEST(AuditFixedFoc, ResolvableMicroincrementsAccumulateThenRejectPositiveF62Dot)
{
    using A = fc_loop_test::Api<TypeParam>;
    using S = typename A::scalar;
    constexpr bool q15 = std::is_same_v<S, int16_t>;
    constexpr int64_t one = INT64_C(1) << std::numeric_limits<S>::digits;
    constexpr int64_t radius_work = q15 ? 65536 : 1;
    constexpr int64_t increment_work = q15 ? 2 : 1;
    const S dt = q15 ? S(1) : S(2);
    const S error = q15 ? S(1) : S(one / 2);
    const typename A::foc_config config{0, LINEARS_FIXED_ONE, 0, 0, S(1)};
    for (const int sign : {-1, 1}) {
        SCOPED_TRACE(sign);
        typename A::foc ctx{};
        typename A::foc_output output{};
        ASSERT_EQ(A::foc_init(&ctx, &config), LINEARS_OK);
        const int64_t steps = radius_work / increment_work + 8;
        for (int64_t n = 1; n <= steps; ++n) {
            ASSERT_EQ(A::foc_step(&ctx, {S(0), S(0)}, {S(sign * error), S(0)},
                S(0), S(4), dt, &output), LINEARS_OK);
            /* Q15: 2^-15 * 2^-15 is exactly two F31 units. Q31:
             * .5 * (2 raw units) is exactly one F31 unit. Once saturated,
             * the positive F62 dot is below half an F31 unit in both APIs. */
            ASSERT_EQ(ctx.integral.d, sign * std::min(n * increment_work, radius_work));
            ASSERT_EQ(ctx.integral.q, 0);
        }
        EXPECT_EQ(output.voltage.d, S(sign));
        ASSERT_EQ(A::foc_step(&ctx, {S(0), S(0)}, {S(-sign * error), S(0)},
            S(0), S(4), dt, &output), LINEARS_OK);
        EXPECT_EQ(ctx.integral.d, sign * (radius_work - increment_work));
    }
}

TYPED_TEST(AuditFixedFoc, OuterSpeedAntiWindupKeepsSubWorkLsbResidualSign)
{
    using A = fc_loop_test::Api<TypeParam>;
    using S = typename A::scalar;
    constexpr bool q15 = std::is_same_v<S, int16_t>;
    constexpr int64_t one = INT64_C(1) << std::numeric_limits<S>::digits;
    /* For Q15 the inner request is exactly 65537 F31 units, just one
     * above the one-signal-LSB radius. Its product with the speed error is
     * positive but rounds to zero if narrowed from F62 before sign testing. */
    const linears_fixed_t inner_kp = q15 ? LINEARS_FIXED_ONE / 2 + 16384 : LINEARS_FIXED_ONE;
    const typename A::speed_config config{{0, 0, inner_kp, 0, S(1)},
        2 * LINEARS_FIXED_ONE, LINEARS_FIXED_ONE, S(4), 1};
    const S dt = q15 ? S(1) : S(one / 2);
    for (const int sign : {-1, 1}) {
        typename A::speed ctx{};
        typename A::speed_output output{};
        ASSERT_EQ(A::speed_init(&ctx, &config), LINEARS_OK);
        for (int n = 0; n < 8; ++n) {
            ASSERT_EQ(A::speed_step(&ctx, {S(0), S(0)}, S(sign), S(0), S(0),
                S(4), dt, &output), LINEARS_OK);
            ASSERT_TRUE(output.current.saturated);
            EXPECT_EQ(ctx.speed.integral, 0);
            EXPECT_EQ(output.reference.q, S(sign * (q15 ? 2 : 3)));
            EXPECT_EQ(output.current.requested_voltage.q, sign * (q15 ? INT64_C(65537) : INT64_C(3)));
            EXPECT_EQ(output.current.voltage.q, S(sign));
        }
    }
}

TYPED_TEST(AuditFixedFoc, WideGainsAndRequestDiagnosticDoNotNarrowBeforeLimiting)
{
    using A = fc_loop_test::Api<TypeParam>;
    using S = typename A::scalar;
    constexpr int64_t one = INT64_C(1) << std::numeric_limits<S>::digits;
    const typename A::foc_config config{4 * LINEARS_FIXED_ONE, 0, 0, 0, S(one / 4)};
    typename A::foc ctx{};
    typename A::foc_output output{};
    ASSERT_EQ(A::foc_init(&ctx, &config), LINEARS_OK);
    ASSERT_EQ(A::foc_step(&ctx, {S(0), S(0)}, {S(3 * one / 4), S(0)},
        S(0), S(one / 2), S(one / 2), &output), LINEARS_OK);
    EXPECT_EQ(output.requested_voltage.d, 3 * LINEARS_FIXED_ONE);
    EXPECT_EQ(output.voltage.d, S(one / 4));
    EXPECT_TRUE(output.saturated);
    const auto saved_ctx = ctx;
    const auto saved_output = output;
    /* Both inputs are legal raw signals, but rotating (-1,-1) by 1/8 turn
     * produces d=-sqrt(2), outside the documented public current range. */
    EXPECT_EQ(A::foc_step(&ctx, {std::numeric_limits<S>::min(), std::numeric_limits<S>::min()},
        {S(0), S(0)}, S(one / 8), S(one / 2), S(one / 2), &output), LINEARS_ERROR_RANGE);
    EXPECT_TRUE(fc_test::same_bytes(ctx, saved_ctx));
    EXPECT_TRUE(fc_test::same_bytes(output, saved_output));
    EXPECT_EQ(A::foc_step(&ctx, {S(0), S(0)}, {S(0), S(0)}, S(0),
        S(one / 2), S(-1), &output), LINEARS_ERROR_DOMAIN);
    EXPECT_TRUE(fc_test::same_bytes(ctx, saved_ctx));
    EXPECT_TRUE(fc_test::same_bytes(output, saved_output));
    EXPECT_EQ(A::foc_step(&ctx, {S(0), S(0)}, {S(0), S(0)}, S(0),
        S(one / 2), S(one / 2), reinterpret_cast<typename A::foc_output*>(&ctx)), LINEARS_ERROR_ALIAS);
    EXPECT_TRUE(fc_test::same_bytes(ctx, saved_ctx));
}
} // namespace
