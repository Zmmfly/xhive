#include "foc_smo_support.h"

namespace {
template<class Tag> class AuditFoc : public ::testing::Test {};
TYPED_TEST_SUITE(AuditFoc, fc_test::Types);

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
} // namespace
