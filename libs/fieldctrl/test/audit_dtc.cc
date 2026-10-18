#include "dtc_support.h"

namespace {
using FloatingTypes = ::testing::Types<fc_test::F32, fc_test::F64>;
template<class Tag> class AuditDtc : public ::testing::Test {};
TYPED_TEST_SUITE(AuditDtc, FloatingTypes);

/* A sector is a direction, independent of the magnitude of nonzero flux.
 * Integer rays keep the oracle away from every irrational 30-degree boundary.
 * Regression snapshot: dtc_impl.h f5fccca95062912165ce4852b156cc67baf58fe0f0ec9b77aac236fd2dea009b. */
TYPED_TEST(AuditDtc, SectorIsScaleInvariantAtUnderflowAndLargeMagnitude)
{
    using A = dtc_test::Api<TypeParam>;
    using S = typename A::scalar;
    const typename A::config cfg{A::make(0), A::make(0.125), A::make(0.125), 1};
    const typename A::ab zero{A::make(0), A::make(0)};
    const std::array<std::array<int, 2>, 12> rays{{
        {{2, 1}}, {{1, 2}}, {{-1, 2}}, {{-2, 1}}, {{-2, -1}}, {{-1, -2}},
        {{1, -2}}, {{2, -1}}, {{1, 0}}, {{0, 1}}, {{-1, 0}}, {{0, -1}}
    }};
    const auto scaled = [](int component, int level) -> S {
        return std::ldexp(static_cast<S>(component), level);
    };
    const int smallest = std::numeric_limits<S>::min_exponent - std::numeric_limits<S>::digits;
    const int largest = std::numeric_limits<S>::max_exponent - 3;
    for (const int scale : {smallest, -8, 0, largest}) {
        for (const auto& ray : rays) {
            SCOPED_TRACE(::testing::Message() << "scale=" << scale << " ray=("
                << ray[0] << ',' << ray[1] << ')');
            typename A::context ctx{};
            typename A::result result{};
            const typename A::ab flux{scaled(ray[0], scale), scaled(ray[1], scale)};
            ASSERT_EQ(A::init(&ctx, cfg, flux), LINEARS_OK);
            ASSERT_EQ(A::step(&ctx, zero, zero, A::make(1), A::make(1),
                A::make(1), &result), LINEARS_OK);
            const long double angle = std::atan2(static_cast<long double>(ray[1]),
                static_cast<long double>(ray[0]));
            const long double turn = angle / (2 * std::acos(-1.0L));
            const int expected = (static_cast<int>(std::floor(6 * (turn + 1.0L / 12))) + 6) % 6 + 1;
            EXPECT_EQ(result.sector, expected);
        }
    }
}

using FixedTypes = ::testing::Types<fc_test::Q15, fc_test::Q31>;
template<class Tag> class AuditFixedDtc : public ::testing::Test {};
TYPED_TEST_SUITE(AuditFixedDtc, FixedTypes);

TYPED_TEST(AuditFixedDtc, RawPrimitiveRaysKeepSectorAndBothComponents)
{
    using A = dtc_test::Api<TypeParam>;
    using S = typename A::scalar;
    constexpr int64_t one = INT64_C(1) << std::numeric_limits<S>::digits;
    const typename A::config config{S(0), S(one / 8), S(one / 8), 1};
    const typename A::ab zero{S(0), S(0)};
    const std::array<std::array<int, 2>, 12> rays{{
        {{2, 1}}, {{1, 2}}, {{-1, 2}}, {{-2, 1}}, {{-2, -1}}, {{-1, -2}},
        {{1, -2}}, {{2, -1}}, {{1, 0}}, {{0, 1}}, {{-1, 0}}, {{0, -1}}
    }};
    for (const int64_t scale : {INT64_C(1), INT64_C(2), one / 4}) {
        for (const auto& ray : rays) {
            SCOPED_TRACE(::testing::Message() << "raw scale=" << scale
                << " ray=" << ray[0] << ',' << ray[1]);
            typename A::context ctx{};
            typename A::result result{};
            const typename A::ab flux{S(ray[0] * scale), S(ray[1] * scale)};
            ASSERT_EQ(A::init(&ctx, config, flux), LINEARS_OK);
            ASSERT_EQ(A::step(&ctx, zero, zero, S(one / 2), S(0), S(one / 8), &result), LINEARS_OK);
            const long double turn = std::atan2(static_cast<long double>(ray[1]),
                static_cast<long double>(ray[0])) / (2 * std::acos(-1.0L));
            const int expected = (static_cast<int>(std::floor(6 * (turn + 1.0L / 12))) + 6) % 6 + 1;
            EXPECT_EQ(result.sector, expected);
            EXPECT_EQ(result.stator_flux.alpha, flux.alpha);
            EXPECT_EQ(result.stator_flux.beta, flux.beta);
        }
    }
    const std::array<typename A::ab, 4> axes{{
        {std::numeric_limits<S>::min(), S(0)}, {S(0), std::numeric_limits<S>::min()},
        {std::numeric_limits<S>::max(), S(0)}, {S(0), std::numeric_limits<S>::max()}
    }};
    const std::array<int, 4> sectors{{4, 6, 1, 3}};
    for (size_t i = 0; i < axes.size(); ++i) {
        typename A::context ctx{};
        typename A::result result{};
        ASSERT_EQ(A::init(&ctx, config, axes[i]), LINEARS_OK);
        ASSERT_EQ(A::step(&ctx, zero, zero, S(one / 2), S(0), S(one / 8), &result), LINEARS_OK);
        EXPECT_EQ(result.sector, sectors[i]);
        EXPECT_EQ(result.stator_flux.alpha, axes[i].alpha);
        EXPECT_EQ(result.stator_flux.beta, axes[i].beta);
    }
}

TYPED_TEST(AuditFixedDtc, SmallTimeVoltageProductsAccumulateAtFixedF31Resolution)
{
    using A = dtc_test::Api<TypeParam>;
    using S = typename A::scalar;
    constexpr bool q15 = std::is_same_v<S, int16_t>;
    constexpr int64_t one = INT64_C(1) << std::numeric_limits<S>::digits;
    constexpr int64_t increment = q15 ? 2 : 1;
    const int steps = q15 ? 32768 : 1024;
    const S dt = q15 ? S(1) : S(2);
    const typename A::ab voltage{q15 ? S(1) : S(one / 2), S(0)};
    const typename A::config config{S(0), S(one / 16), S(one / 16), 1};
    typename A::context ctx{};
    typename A::result result{};
    ASSERT_EQ(A::init(&ctx, config, {S(one / 4), S(0)}), LINEARS_OK);
    for (int n = 1; n <= steps; ++n) {
        ASSERT_EQ(A::step(&ctx, {S(0), S(0)}, voltage, S(one / 2), S(0), dt, &result), LINEARS_OK);
        ASSERT_EQ(ctx.flux_alpha, LINEARS_FIXED_ONE / 4 + n * increment);
        ASSERT_EQ(ctx.flux_beta, 0);
    }
    EXPECT_EQ(result.stator_flux.alpha, S(one / 4 + (q15 ? 1 : steps)));
    EXPECT_EQ(result.stator_flux.beta, 0);
}

TYPED_TEST(AuditFixedDtc, ExplicitPhysicalBasesMatchFluxTorqueAndActualVoltage)
{
    using A = dtc_test::Api<TypeParam>;
    using S = typename A::scalar;
    constexpr int64_t one = INT64_C(1) << std::numeric_limits<S>::digits;
    /* Independent physical model: Vbase=32 V, Ibase=8 A, Tbase=1/8 s,
     * flux base=4 Wb and torque base=32 N*m. R=1 ohm, poles=2,
     * dt=1/64 s, initial flux=(1,0) Wb, i=(0,+/-2) A, v=(4,+/-8) V.
     * Euler gives flux=(17/16,+/-3/32) Wb and torque=+/-51/8 N*m.
     * All inputs/products below are dyadic and exact at both signal widths. */
    const typename A::config config{S(one / 4), S(one / 32), S(one / 32), 2};
    typename A::context ctx{};
    typename A::result result{};
    for (const int sign : {-1, 1}) {
        ASSERT_EQ(A::init(&ctx, config, {S(one / 4), S(0)}), LINEARS_OK);
        const typename A::ab current{S(0), S(sign * one / 4)};
        ASSERT_EQ(A::step(&ctx, current, {S(one / 8), S(sign * one / 4)},
            S(one / 2), S(sign * one / 4), S(one / 8), &result), LINEARS_OK);
        EXPECT_EQ(ctx.flux_alpha, 17 * LINEARS_FIXED_ONE / 64);
        EXPECT_EQ(ctx.flux_beta, sign * 3 * LINEARS_FIXED_ONE / 128);
        EXPECT_EQ(result.stator_flux.alpha, S(17 * one / 64));
        EXPECT_EQ(result.stator_flux.beta, S(sign * 3 * one / 128));
        EXPECT_EQ(result.torque, S(sign * 51 * one / 256));
        ASSERT_NE(result.switch_state, 0);
        ASSERT_NE(result.switch_state, 7);
        /* Actual voltage equal to R*i holds flux even though the controller
         * just requested an active inverter vector. No command is fed back. */
        ASSERT_EQ(A::step(&ctx, current, {S(0), S(sign * one / 16)},
            S(one / 2), S(sign * one / 4), S(one / 8), &result), LINEARS_OK);
        EXPECT_EQ(ctx.flux_alpha, 17 * LINEARS_FIXED_ONE / 64);
        EXPECT_EQ(ctx.flux_beta, sign * 3 * LINEARS_FIXED_ONE / 128);
        ASSERT_EQ(A::step(&ctx, current, {S(0), S(0)}, S(one / 2),
            S(sign * one / 4), S(one / 8), &result), LINEARS_OK);
        EXPECT_EQ(ctx.flux_beta, sign * 2 * LINEARS_FIXED_ONE / 128);
    }
    ASSERT_EQ(A::init(&ctx, config, {S(3 * one / 4), S(3 * one / 4)}), LINEARS_OK);
    const auto saved_ctx = ctx;
    const auto saved_result = result;
    /* Each flux component is in range, but their magnitude exceeds +1. */
    EXPECT_EQ(A::step(&ctx, {S(0), S(0)}, {S(0), S(0)}, S(one / 2), S(0),
        S(one / 8), &result), LINEARS_ERROR_RANGE);
    EXPECT_TRUE(fc_test::same_bytes(ctx, saved_ctx));
    EXPECT_TRUE(fc_test::same_bytes(result, saved_result));
}
} // namespace
