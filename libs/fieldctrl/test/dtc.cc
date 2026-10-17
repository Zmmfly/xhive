#include "dtc_support.h"
#include <random>

using dtc_test::value;

template<class Tag> class Dtc : public ::testing::Test {};
TYPED_TEST_SUITE(Dtc, fc_test::Types, dtc_test::VariantNames);

TYPED_TEST(Dtc, QuantizedInputsMatchIndependentVoltageAndTorqueEquations) {
    using A = dtc_test::Api<TypeParam>;
    auto q = A::make;
    std::mt19937 random(0x4d544321u);
    std::uniform_real_distribution<double> flux(-0.3, 0.3), current(-6, 6), voltage(-24, 24);
    std::uniform_real_distribution<double> resistance(0, 2), dt(0.00001, 0.0003);
    for (int n = 0; n < 600; ++n) {
        typename A::context ctx{};
        typename A::config cfg{q(resistance(random)), q(0.002), q(0.025),
                               static_cast<uint16_t>(1 + n % 13)};
        typename A::ab seed{q(flux(random)), q(flux(random))};
        typename A::ab i{q(current(random)), q(current(random))};
        typename A::ab v{q(voltage(random)), q(voltage(random))};
        const auto interval = q(dt(random));
        ASSERT_EQ(A::init(&ctx, cfg, seed), LINEARS_OK);
        const auto expected = dtc_test::reference<TypeParam>(ctx, i, v, interval);
        typename A::result out{};
        ASSERT_EQ(A::step(&ctx, i, v, q(0.25), q(0.1), interval, &out), LINEARS_OK);
        dtc_test::near(expected.alpha, expected.alpha_error, out.stator_flux.alpha);
        dtc_test::near(expected.beta, expected.beta_error, out.stator_flux.beta);
        dtc_test::near(expected.magnitude, expected.magnitude_error, out.flux_magnitude);
        dtc_test::near(expected.torque, expected.torque_error, out.torque);
        dtc_test::near(expected.alpha, expected.alpha_error, ctx.flux_alpha);
        dtc_test::near(expected.beta, expected.beta_error, ctx.flux_beta);
        EXPECT_GE(out.sector, 1);
        EXPECT_LE(out.sector, 6);
        EXPECT_LE(out.switch_state, 7);
    }
}

TYPED_TEST(Dtc, AllSixSectorsAndEveryFluxTorqueTableEntry) {
    using A = dtc_test::Api<TypeParam>;
    auto q = A::make;
    // Explicit expected bit patterns, independently transcribed from BST V indices.
    const uint8_t increase_flux_increase_torque[6] = {6, 2, 3, 1, 5, 4};
    const uint8_t increase_flux_decrease_torque[6] = {5, 4, 6, 2, 3, 1};
    const uint8_t decrease_flux_increase_torque[6] = {2, 3, 1, 5, 4, 6};
    const uint8_t decrease_flux_decrease_torque[6] = {1, 5, 4, 6, 2, 3};
    const typename A::config cfg{q(0.5), q(0.125), q(0.125), 2};
    const typename A::ab zero{q(0), q(0)};
    for (int sector = 1; sector <= 6; ++sector) {
        const double angle = (sector - 1) * std::acos(-1.0) / 3;
        const typename A::ab seed{q(std::cos(angle)), q(std::sin(angle))};
        for (int f : {-1, 1}) for (int t : {-1, 0, 1}) {
            typename A::context ctx{};
            typename A::result out{};
            ASSERT_EQ(A::init(&ctx, cfg, seed), LINEARS_OK);
            ASSERT_EQ(A::step(&ctx, zero, zero, q(f > 0 ? 2 : 0.5), q(t), q(0.01), &out), LINEARS_OK);
            EXPECT_EQ(out.sector, sector);
            EXPECT_EQ(out.flux_state, f);
            EXPECT_EQ(out.torque_state, t);
            uint8_t expected = 0;
            if (f > 0 && t > 0) expected = increase_flux_increase_torque[sector - 1];
            if (f > 0 && t < 0) expected = increase_flux_decrease_torque[sector - 1];
            if (f < 0 && t > 0) expected = decrease_flux_increase_torque[sector - 1];
            if (f < 0 && t < 0) expected = decrease_flux_decrease_torque[sector - 1];
            EXPECT_EQ(out.switch_state, expected);
            EXPECT_EQ(ctx.last_switch, expected);
        }
    }
}

TYPED_TEST(Dtc, ZeroVectorsAreRealNearestStatesAndDoNotRepresentNegativeTorque) {
    using A = dtc_test::Api<TypeParam>;
    auto q = A::make;
    const typename A::ab zero{q(0), q(0)};
    for (int sector = 1; sector <= 6; ++sector) {
        typename A::context ctx{};
        typename A::result active{}, neutral{}, retained{};
        const double angle = (sector - 1) * std::acos(-1.0) / 3;
        ASSERT_EQ(A::init(&ctx, {q(0.5), q(0.125), q(0.125), 2},
                          {q(std::cos(angle)), q(std::sin(angle))}), LINEARS_OK);
        ASSERT_EQ(A::step(&ctx, zero, zero, q(2), q(-1), q(0.01), &active), LINEARS_OK);
        EXPECT_NE(active.switch_state, 0);
        EXPECT_NE(active.switch_state, 7);
        ASSERT_EQ(A::step(&ctx, zero, zero, q(2), q(0), q(0.01), &neutral), LINEARS_OK);
        const unsigned bits = (active.switch_state & 1u) + ((active.switch_state >> 1u) & 1u) +
                              ((active.switch_state >> 2u) & 1u);
        EXPECT_EQ(neutral.switch_state, bits >= 2 ? 7 : 0);
        EXPECT_EQ(neutral.torque_state, 0);
        const auto actual = dtc_test::inverter_voltage(neutral.switch_state, 48);
        EXPECT_EQ(actual[0], 0);
        EXPECT_EQ(actual[1], 0);
        ASSERT_EQ(A::step(&ctx, zero, zero, q(2), q(0), q(0.01), &retained), LINEARS_OK);
        EXPECT_EQ(retained.switch_state, neutral.switch_state);
    }
}

TYPED_TEST(Dtc, HysteresisEqualityMemoryAndZeroCrossings) {
    using A = dtc_test::Api<TypeParam>;
    auto q = A::make;
    typename A::context ctx{};
    typename A::result out{};
    const typename A::ab zero{q(0), q(0)};
    ASSERT_EQ(A::init(&ctx, {q(0), q(0.125), q(0.125), 1}, {q(1), q(0)}), LINEARS_OK);
    // All thresholds below are exactly binary representable in all variants.
    for (const auto item : std::array<std::array<double, 3>, 11>{{
        {1.125, 0.125, 1}, {1, 0.0625, 1}, {0.875, 0, 0},
        {1, -0.0625, 0}, {1, -0.125, -1}, {1, -0.0625, -1},
        {1, 0, 0}, {1, 0.0625, 0}, {1.125, 0.125, 1},
        {1, -0.03125, 0}, {1, -0.25, -1}}}) {
        const int old_flux_state = ctx.flux_state;
        ASSERT_EQ(A::step(&ctx, zero, zero, q(item[0]), q(item[1]), q(0.125), &out), LINEARS_OK);
        EXPECT_EQ(out.torque_state, item[2]);
        if (item[0] == 0.875) { EXPECT_EQ(out.flux_state, -1); }
        if (item[0] == 1.125) { EXPECT_EQ(out.flux_state, 1); }
        if (item[0] == 1) { EXPECT_EQ(out.flux_state, old_flux_state); }
    }
    // Inside the flux band retain the negative decision even at zero error.
    ASSERT_EQ(A::step(&ctx, zero, zero, q(0.875), q(0), q(0.125), &out), LINEARS_OK);
    ASSERT_EQ(A::step(&ctx, zero, zero, q(1.0625), q(0), q(0.125), &out), LINEARS_OK);
    EXPECT_EQ(out.flux_state, -1);
}

TYPED_TEST(Dtc, CenteredSectorEdgesAndAxes) {
    using A = dtc_test::Api<TypeParam>;
    auto q = A::make;
    const typename A::ab zero{q(0), q(0)};
    // +/-0.001 turn is far above input quantization, while testing both sides
    // of every centered-sector boundary rather than only vector centers.
    for (int boundary = 0; boundary < 6; ++boundary) for (int side : {-1, 1}) {
        const double turns = (2 * boundary + 1) / 12.0 + side * 0.001;
        typename A::context ctx{};
        typename A::result out{};
        const double a = 2 * std::acos(-1.0) * turns;
        ASSERT_EQ(A::init(&ctx, {q(0), q(0.125), q(0.125), 1},
                          {q(std::cos(a)), q(std::sin(a))}), LINEARS_OK);
        ASSERT_EQ(A::step(&ctx, zero, zero, q(1), q(0), q(0.125), &out), LINEARS_OK);
        const int expected = side < 0 ? boundary + 1 : (boundary + 1) % 6 + 1;
        EXPECT_EQ(out.sector, expected);
    }
    const double axes[4][3] = {{1, 0, 1}, {0, 1, 3}, {-1, 0, 4}, {0, -1, 6}};
    for (const auto& axis : axes) {
        typename A::context ctx{};
        typename A::result out{};
        ASSERT_EQ(A::init(&ctx, {q(0), q(0.125), q(0.125), 1}, {q(axis[0]), q(axis[1])}), LINEARS_OK);
        ASSERT_EQ(A::step(&ctx, zero, zero, q(1), q(0), q(0.125), &out), LINEARS_OK);
        EXPECT_EQ(out.sector, axis[2]);
    }
}

TYPED_TEST(Dtc, SectorIsInvariantUnderExtremeFluxScaling) {
    using A = dtc_test::Api<TypeParam>;
    using S = typename A::scalar;
    auto q = A::make;
    const typename A::ab zero{q(0), q(0)};
    // Slope 1/2 lies strictly below tan(30 degrees), even when the vector
    // coordinates are only one or two floating-point subnormal units.
    const int vectors[8][3] = {
        {2, 1, 1}, {-1, 2, 3}, {-2, -1, 4}, {1, -2, 6},
        {1, 2, 2}, {-2, 1, 4}, {-1, -2, 5}, {2, -1, 1}
    };
    for (int scale = 0; scale < 4; ++scale) for (const auto& v : vectors) {
        typename A::ab seed;
        if constexpr (A::fixed) {
            const int shifts[4] = {LINEARS_EXPONENT_MIN, -1, 0, LINEARS_EXPONENT_MAX - 3};
            seed = {q(v[0]), q(v[1])};
            seed.alpha.exponent += shifts[scale];
            seed.beta.exponent += shifts[scale];
        } else {
            const S scales[4] = {std::numeric_limits<S>::denorm_min(),
                                 std::numeric_limits<S>::min(), S(1),
                                 std::numeric_limits<S>::max() / S(4)};
            seed = {S(v[0]) * scales[scale], S(v[1]) * scales[scale]};
        }
        typename A::context ctx{};
        typename A::result out{};
        ASSERT_EQ(A::init(&ctx, {q(0), q(0.125), q(0.125), 1}, seed), LINEARS_OK);
        ASSERT_EQ(A::step(&ctx, zero, zero, q(1), q(1), q(1), &out), LINEARS_OK);
        EXPECT_EQ(out.sector, v[2]) << "scale=" << scale << " alpha=" << v[0] << " beta=" << v[1];
    }
}

TYPED_TEST(Dtc, LongIntegrationRetainsSubQ15OutputIncrements) {
    using A = dtc_test::Api<TypeParam>;
    auto q = A::make;
    typename A::context ctx{};
    typename A::result out{};
    ASSERT_EQ(A::init(&ctx, {q(0), q(0.001), q(0.01), 3}, {q(0.125), q(-0.25)}), LINEARS_OK);
    constexpr int count = 32768;
    const auto dt = q(0x1p-15);
    const typename A::ab voltage{q(0x1p-5), q(-0x1p-6)}, zero{q(0), q(0)};
    for (int n = 1; n <= count; ++n) {
        ASSERT_EQ(A::step(&ctx, zero, voltage, q(0.3), q(0), dt, &out), LINEARS_OK);
        // Every work-state sum is exactly binary representable; no tolerance.
        EXPECT_EQ(value(ctx.flux_alpha), 0.125 + n * 0x1p-20);
        EXPECT_EQ(value(ctx.flux_beta), -0.25 - n * 0x1p-21);
        dtc_test::near(0.125 + n * 0x1p-20, 0, out.stator_flux.alpha);
        dtc_test::near(-0.25 - n * 0x1p-21, 0, out.stator_flux.beta);
    }
}

TYPED_TEST(Dtc, InitialBiasAndSensorOffsetsProduceUncorrectedVoltageModelDrift) {
    using A = dtc_test::Api<TypeParam>;
    auto q = A::make;
    typename A::context clean{}, biased{};
    typename A::result clean_out{}, biased_out{};
    const typename A::config config{q(0.5), q(0.001), q(0.01), 2};
    const typename A::ab zero{q(0), q(0)};
    ASSERT_EQ(A::init(&clean, config, {q(0.25), q(0.125)}), LINEARS_OK);
    ASSERT_EQ(A::init(&biased, config, {q(0.3125), q(0.125)}), LINEARS_OK);
    // Physical plant is static at psi=(0.25,0.125), i=v=0. The second observer
    // has +1/16 Wb initial bias, +1/32 V voltage offset and +1/64 A current offset.
    for (int n = 1; n <= 4096; ++n) {
        ASSERT_EQ(A::step(&clean, zero, zero, q(0.3), q(0), q(0x1p-12), &clean_out), LINEARS_OK);
        ASSERT_EQ(A::step(&biased, {q(0x1p-6), q(0)}, {q(0x1p-5), q(0)},
                          q(0.3), q(0), q(0x1p-12), &biased_out), LINEARS_OK);
        EXPECT_EQ(value(biased.flux_alpha) - value(clean.flux_alpha),
                  0.0625 + n * 0x1.8p-18);
        EXPECT_EQ(value(clean.flux_alpha), 0.25);
        EXPECT_EQ(value(biased.flux_beta), 0.125);
    }
    EXPECT_GT(value(biased_out.flux_magnitude), value(clean_out.flux_magnitude));
    EXPECT_LT(value(biased_out.torque), 0); // A current offset also creates false torque.
}

TYPED_TEST(Dtc, AppliedVoltageIsFeedbackNotAnAssumedSwitchingCommand) {
    using A = dtc_test::Api<TypeParam>;
    auto q = A::make;
    typename A::context ctx{};
    typename A::result out{};
    const typename A::ab zero{q(0), q(0)};
    ASSERT_EQ(A::init(&ctx, {q(0.5), q(0.125), q(0.125), 1}, {q(1), q(0)}), LINEARS_OK);
    for (int n = 0; n < 30; ++n) {
        ASSERT_EQ(A::step(&ctx, zero, zero, q(2), q(1), q(0.125), &out), LINEARS_OK);
        ASSERT_NE(out.switch_state, 0);
        ASSERT_NE(out.switch_state, 7);
        EXPECT_EQ(value(ctx.flux_alpha), 1);
        EXPECT_EQ(value(ctx.flux_beta), 0);
    }
    // The caller reports a voltage different from the command; that is integrated.
    ASSERT_EQ(A::step(&ctx, zero, {q(-0.25), q(0.5)}, q(2), q(1), q(0.125), &out), LINEARS_OK);
    EXPECT_EQ(value(ctx.flux_alpha), 0.96875);
    EXPECT_EQ(value(ctx.flux_beta), 0.0625);
}

TYPED_TEST(Dtc, AllFailuresPreserveContextAndOutputBytes) {
    using A = dtc_test::Api<TypeParam>;
    auto q = A::make;
    typename A::context ctx{};
    typename A::result out;
    std::memset(&out, 0x6b, sizeof(out));
    const typename A::config cfg{q(0.5), q(0.125), q(0.125), 2};
    const typename A::ab zero{q(0), q(0)}, seed{q(1), q(0)};
    ASSERT_EQ(A::init(&ctx, cfg, seed), LINEARS_OK);
    const auto before_ctx = dtc_test::bytes(ctx), before_out = dtc_test::bytes(out);
    EXPECT_EQ(A::init(nullptr, cfg, seed), LINEARS_ERROR_ARGUMENT);
    EXPECT_EQ(A::init(&ctx, cfg, zero), LINEARS_ERROR_DOMAIN);
    for (int field = 0; field < 4; ++field) {
        auto bad = cfg;
        if (field == 0) bad.resistance = q(-0.5);
        if (field == 1) bad.flux_band = q(0);
        if (field == 2) bad.torque_band = q(-0.125);
        if (field == 3) bad.pole_pairs = 0;
        EXPECT_EQ(A::init(&ctx, bad, seed), LINEARS_ERROR_ARGUMENT);
        EXPECT_EQ(dtc_test::bytes(ctx), before_ctx);
    }
    EXPECT_EQ(A::step(nullptr, zero, zero, q(1), q(0), q(0.125), &out), LINEARS_ERROR_ARGUMENT);
    EXPECT_EQ(A::step(&ctx, zero, zero, q(1), q(0), q(0.125), nullptr), LINEARS_ERROR_ARGUMENT);
    for (double dt : {0.0, -0.125})
        EXPECT_EQ(A::step(&ctx, zero, zero, q(1), q(0), q(dt), &out), LINEARS_ERROR_ARGUMENT);
    for (double ref : {-1.0, 0.0, 0.125})
        EXPECT_EQ(A::step(&ctx, zero, zero, q(ref), q(0), q(0.125), &out), LINEARS_ERROR_ARGUMENT);
    EXPECT_EQ(A::step(&ctx, zero, {q(-8), q(0)}, q(1), q(0), q(0.125), &out), LINEARS_ERROR_DOMAIN);
    EXPECT_EQ(A::step(&ctx, zero, zero, q(1), q(0), q(0.125),
                      reinterpret_cast<typename A::result*>(&ctx)), LINEARS_ERROR_ALIAS);
    EXPECT_EQ(dtc_test::bytes(ctx), before_ctx);
    EXPECT_EQ(dtc_test::bytes(out), before_out);
}

TYPED_TEST(Dtc, NonfiniteAndInvalidExponentArgumentsFailAtomically) {
    using A = dtc_test::Api<TypeParam>;
    auto q = A::make;
    using S = typename A::scalar;
    typename A::context ctx{};
    typename A::result out;
    std::memset(&out, 0x39, sizeof(out));
    const typename A::config cfg{q(0.5), q(0.125), q(0.125), 2};
    const typename A::ab zero{q(0), q(0)}, seed{q(1), q(0)};
    ASSERT_EQ(A::init(&ctx, cfg, seed), LINEARS_OK);
    const auto before_ctx = dtc_test::bytes(ctx), before_out = dtc_test::bytes(out);
    for (int kind = 0; kind < 3; ++kind) {
        S bad;
        if constexpr (A::fixed) {
            bad = q(1);
            bad.exponent = kind == 0 ? LINEARS_EXPONENT_MAX + 1 : LINEARS_EXPONENT_MIN - 1;
            if (kind == 2) bad.mantissa = 0; // Invalid zero exponents are not silently ignored.
        } else {
            bad = kind == 0 ? std::numeric_limits<S>::quiet_NaN() :
                  kind == 1 ? std::numeric_limits<S>::infinity() : -std::numeric_limits<S>::infinity();
        }
        for (int field = 0; field < 5; ++field) {
            auto c = cfg;
            auto f = seed;
            if (field == 0) c.resistance = bad;
            if (field == 1) c.flux_band = bad;
            if (field == 2) c.torque_band = bad;
            if (field == 3) f.alpha = bad;
            if (field == 4) f.beta = bad;
            EXPECT_EQ(A::init(&ctx, c, f), LINEARS_ERROR_RANGE);
            EXPECT_EQ(dtc_test::bytes(ctx), before_ctx);
        }
        for (int field = 0; field < 7; ++field) {
            auto i = zero, v = zero;
            auto flux_ref = q(1), torque_ref = q(0), dt = q(0.125);
            if (field == 0) i.alpha = bad;
            if (field == 1) i.beta = bad;
            if (field == 2) v.alpha = bad;
            if (field == 3) v.beta = bad;
            if (field == 4) flux_ref = bad;
            if (field == 5) torque_ref = bad;
            if (field == 6) dt = bad;
            EXPECT_EQ(A::step(&ctx, i, v, flux_ref, torque_ref, dt, &out), LINEARS_ERROR_RANGE);
            EXPECT_EQ(dtc_test::bytes(ctx), before_ctx);
            EXPECT_EQ(dtc_test::bytes(out), before_out);
        }
    }
}

TYPED_TEST(Dtc, IntermediateRangeFailureAndScalarRangeEndpoints) {
    using A = dtc_test::Api<TypeParam>;
    using S = typename A::scalar;
    auto q = A::make;
    typename A::context ctx{};
    typename A::result out;
    std::memset(&out, 0x55, sizeof(out));
    const typename A::ab zero{q(0), q(0)};
    ASSERT_EQ(A::init(&ctx, {q(0.5), q(0.125), q(0.125), 2}, {q(1), q(0)}), LINEARS_OK);
    const auto before_ctx = dtc_test::bytes(ctx), before_out = dtc_test::bytes(out);
    S huge;
    if constexpr (A::fixed) {
        huge = q(0.75);
        huge.exponent = LINEARS_EXPONENT_MAX;
    } else huge = std::numeric_limits<S>::max();
    EXPECT_EQ(A::step(&ctx, zero, {huge, q(0)}, q(1), q(0), q(4), &out), LINEARS_ERROR_RANGE);
    EXPECT_EQ(dtc_test::bytes(ctx), before_ctx);
    EXPECT_EQ(dtc_test::bytes(out), before_out);
    if constexpr (A::fixed) {
        for (int exponent : {LINEARS_EXPONENT_MIN, LINEARS_EXPONENT_MAX}) {
            S end = q(0.5);
            end.exponent = exponent;
            ASSERT_EQ(A::init(&ctx, {q(0), q(0.125), q(0.125), 1}, {end, q(0)}), LINEARS_OK);
            ASSERT_EQ(A::step(&ctx, zero, zero, q(1), q(0), q(1), &out), LINEARS_OK);
            EXPECT_EQ(out.stator_flux.alpha.mantissa, end.mantissa);
            EXPECT_EQ(out.stator_flux.alpha.exponent, end.exponent);
        }
        auto too_small = q(0.5);
        too_small.mantissa = 1;
        too_small.exponent = LINEARS_EXPONENT_MIN;
        const auto saved = dtc_test::bytes(ctx);
        EXPECT_EQ(A::init(&ctx, {q(0), q(0.125), q(0.125), 1}, {too_small, q(0)}), LINEARS_ERROR_RANGE);
        EXPECT_EQ(dtc_test::bytes(ctx), saved);
        if constexpr (std::is_same_v<TypeParam, fc_test::Q15>) {
            // The q31 work result fits, but final q15 rounding needs exponent
            // MAX+1. Failure must occur before any candidate state is committed.
            const linears_q15_t largest{32767, LINEARS_EXPONENT_MAX};
            const linears_q15_t half{16384, LINEARS_EXPONENT_MAX};
            ASSERT_EQ(A::init(&ctx, {q(0), q(0.125), q(0.125), 1}, {largest, q(0)}), LINEARS_OK);
            const auto saved_ctx = dtc_test::bytes(ctx), saved_out = dtc_test::bytes(out);
            EXPECT_EQ(A::step(&ctx, zero, {half, q(0)}, q(1), q(0), q(0x1p-15), &out), LINEARS_ERROR_RANGE);
            EXPECT_EQ(dtc_test::bytes(ctx), saved_ctx);
            EXPECT_EQ(dtc_test::bytes(out), saved_out);
        }
    }
}

TYPED_TEST(Dtc, MaximumPolePairCountAndBothPhysicalTorqueSigns) {
    using A = dtc_test::Api<TypeParam>;
    auto q = A::make;
    const typename A::ab zero{q(0), q(0)};
    for (int direction : {-1, 1}) {
        typename A::context ctx{};
        typename A::result out{};
        ASSERT_EQ(A::init(&ctx, {q(0), q(0.125), q(0.125), UINT16_MAX}, {q(0.5), q(0)}), LINEARS_OK);
        ASSERT_EQ(A::step(&ctx, {q(0), q(direction)}, zero, q(1), q(0), q(0.125), &out), LINEARS_OK);
        dtc_test::near(direction * 49151.25L, 0, out.torque);
        EXPECT_EQ(out.torque_state, -direction);
    }
}

TYPED_TEST(Dtc, IndependentInstancesRemainIdenticalWhenInterleaved) {
    using A = dtc_test::Api<TypeParam>;
    auto q = A::make;
    typename A::context interleaved{}, isolated{}, unrelated{};
    typename A::result a{}, b{}, c{};
    const typename A::config cfg{q(0.375), q(0.002), q(0.02), 4};
    ASSERT_EQ(A::init(&interleaved, cfg, {q(0.1), q(0.2)}), LINEARS_OK);
    ASSERT_EQ(A::init(&isolated, cfg, {q(0.1), q(0.2)}), LINEARS_OK);
    ASSERT_EQ(A::init(&unrelated, cfg, {q(-0.3), q(0.05)}), LINEARS_OK);
    for (int n = 0; n < 1000; ++n) {
        const typename A::ab i{q(std::sin(n * 0.01)), q(std::cos(n * 0.01))};
        const typename A::ab v{q(3 * std::cos(n * 0.013)), q(-2 * std::sin(n * 0.013))};
        ASSERT_EQ(A::step(&interleaved, i, v, q(0.25), q(0.4), q(0.0001), &a), LINEARS_OK);
        ASSERT_EQ(A::step(&unrelated, v, i, q(0.5), q(-0.6), q(0.0002), &c), LINEARS_OK);
        ASSERT_EQ(A::step(&isolated, i, v, q(0.25), q(0.4), q(0.0001), &b), LINEARS_OK);
        EXPECT_EQ(value(interleaved.flux_alpha), value(isolated.flux_alpha));
        EXPECT_EQ(value(interleaved.flux_beta), value(isolated.flux_beta));
        EXPECT_EQ(value(a.torque), value(b.torque));
        EXPECT_EQ(a.switch_state, b.switch_state);
        EXPECT_EQ(a.flux_state, b.flux_state);
        EXPECT_EQ(a.torque_state, b.torque_state);
    }
}

TYPED_TEST(Dtc, SurfacePmsmMechanicalClosedLoopTracksBothSignsAndReverses) {
    using A = dtc_test::Api<TypeParam>;
    using Plant = dtc_test::Plant;
    auto q = A::make;
    const auto sample = q(0.00001);
    const double dt = value(sample); // Plant and observer share the quantized clock.
    const double bus = 48, command = 0.4, flux_band = 0.001, torque_band = 0.015;
    const double iq_ref = command / (1.5 * Plant::pole_pairs * Plant::magnet_flux);
    const auto flux_ref = q(std::hypot(Plant::magnet_flux, Plant::inductance * iq_ref));
    for (int direction : {-1, 1}) {
        Plant plant;
        plant.theta = 0.37;
        plant.omega = direction * 20;
        typename A::context ctx{};
        typename A::result result{};
        const auto seed = plant.flux();
        ASSERT_EQ(A::init(&ctx, {q(Plant::resistance), q(flux_band), q(torque_band), Plant::pole_pairs},
                          {q(seed[0]), q(seed[1])}), LINEARS_OK);
        uint8_t executed = 0;
        double peak_flux_error = 0, peak_estimation_error = 0, peak_current = 0;
        double torque_squared_error = 0, signed_torque_error = 0;
        double forward_speed = 0;
        unsigned steady_count = 0, zero_count = 0;
        for (int n = 0; n < 20000; ++n) {
            // Only the plant sees true theta. The DTC receives current and actual
            // terminal voltage, including the previous step's one-interval delay.
            const auto v = dtc_test::inverter_voltage(executed, bus);
            plant.advance(v[0], v[1], dt);
            const double torque_ref = direction * (n < 5000 ? command : -command);
            ASSERT_EQ(A::step(&ctx, {q(plant.ia), q(plant.ib)}, {q(v[0]), q(v[1])},
                              flux_ref, q(torque_ref), sample, &result), LINEARS_OK);
            executed = result.switch_state;
            zero_count += executed == 0 || executed == 7;
            peak_current = std::max(peak_current, std::hypot(plant.ia, plant.ib));
            const auto actual_flux = plant.flux();
            peak_estimation_error = std::max(peak_estimation_error,
                std::hypot(value(ctx.flux_alpha) - actual_flux[0], value(ctx.flux_beta) - actual_flux[1]));
            peak_flux_error = std::max(peak_flux_error,
                std::abs(std::hypot(actual_flux[0], actual_flux[1]) - value(flux_ref)));
            if (n == 4999) forward_speed = plant.omega;
            // Exclude the startup and commanded-torque transition, not any speed
            // interval: the statistics include crossing zero mechanical speed.
            if (n > 1000 && (n < 5000 || n > 6000)) {
                const double error = plant.torque() - torque_ref;
                torque_squared_error += error * error;
                signed_torque_error += error;
                ++steady_count;
            }
        }
        const double rms = std::sqrt(torque_squared_error / steady_count);
        const std::string prefix = direction > 0 ? "positive_start_" : "negative_start_";
        this->RecordProperty(prefix + "rms_torque_error", ::testing::PrintToString(rms));
        this->RecordProperty(prefix + "peak_flux_estimation_error", ::testing::PrintToString(peak_estimation_error));
        this->RecordProperty(prefix + "final_speed", ::testing::PrintToString(plant.omega));
        this->RecordProperty(prefix + "peak_current", ::testing::PrintToString(peak_current));
        this->RecordProperty(prefix + "peak_flux_error", ::testing::PrintToString(peak_flux_error));
        // The maximum active vector is 2/3 Vdc. One sample's torque increment
        // is bounded here by (3 p psi_pm / 2L) * |v| dt, plus hysteresis and
        // a 10% reserve for R*i and back EMF at this specified operating point.
        const double one_sample_torque = 1.5 * Plant::pole_pairs * Plant::magnet_flux /
                                         Plant::inductance * (2 * bus / 3) * dt;
        EXPECT_LT(rms, torque_band + 1.1 * one_sample_torque);
        EXPECT_LT(std::abs(signed_torque_error / steady_count), torque_band);
        // Flux ripple includes hysteresis, one voltage step and the independently
        // checked observer discretization/quantization error, not a fitted bound.
        const double observer_budget = 0.01 * flux_band; // <=1% of the flux half-band.
        EXPECT_LT(peak_estimation_error, observer_budget);
        EXPECT_LT(peak_flux_error, flux_band + (2 * bus / 3) * dt + observer_budget);
        EXPECT_LT(peak_current, 3.0);
        EXPECT_GT(direction * forward_speed, 24);
        EXPECT_LT(direction * plant.omega, -20);
        EXPECT_GT(zero_count, 0u);
    }
}
