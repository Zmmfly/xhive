#include "dtc_support.h"
#include <random>

using dtc_test::value;
constexpr double dtc_sample_time = 0.00001;

template<class Tag> class Dtc : public ::testing::Test {};
TYPED_TEST_SUITE(Dtc, fc_test::Types, dtc_test::VariantNames);

TYPED_TEST(Dtc, QuantizedInputsMatchIndependentVoltageAndTorqueEquations) {
    using A = dtc_test::Api<TypeParam>;
    auto q = A::make;
    std::mt19937 random(0x4d544321u);
    const double flux_limit = A::fixed ? 0.25 : 0.3;
    const double current_limit = A::fixed ? 0.25 : 6;
    const double voltage_limit = A::fixed ? 0.5 : 24;
    std::uniform_real_distribution<double> flux(-flux_limit, flux_limit);
    std::uniform_real_distribution<double> current(-current_limit, current_limit);
    std::uniform_real_distribution<double> voltage(-voltage_limit, voltage_limit);
    std::uniform_real_distribution<double> resistance(0, A::fixed ? 0.5 : 2);
    std::uniform_real_distribution<double> dt(A::fixed ? 0x1p-12 : 0.00001,
                                              A::fixed ? 0x1p-7 : 0.0003);
    for (int n = 0; n < 600; ++n) {
        typename A::context ctx{};
        typename A::config cfg{q(resistance(random)), q(0.002), q(0.025),
                               static_cast<uint16_t>(1 + n % (A::fixed ? 4 : 13))};
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
    const double signal = A::fixed ? 0.25 : 1;
    const typename A::config cfg{q(0.5), q(0.125 * signal), q(0.125 * signal), 2};
    const typename A::ab zero{q(0), q(0)};
    for (int sector = 1; sector <= 6; ++sector) {
        const double angle = (sector - 1) * std::acos(-1.0) / 3;
        const typename A::ab seed{q(signal * std::cos(angle)), q(signal * std::sin(angle))};
        for (int f : {-1, 1}) for (int t : {-1, 0, 1}) {
            typename A::context ctx{};
            typename A::result out{};
            ASSERT_EQ(A::init(&ctx, cfg, seed), LINEARS_OK);
            ASSERT_EQ(A::step(&ctx, zero, zero, q(signal * (f > 0 ? 2 : 0.5)),
                              q(signal * t), q(0.01), &out), LINEARS_OK);
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
    const double signal = A::fixed ? 0.25 : 1;
    for (int sector = 1; sector <= 6; ++sector) {
        typename A::context ctx{};
        typename A::result active{}, neutral{}, retained{};
        const double angle = (sector - 1) * std::acos(-1.0) / 3;
        ASSERT_EQ(A::init(&ctx, {q(0.5), q(0.125 * signal), q(0.125 * signal), 2},
                          {q(signal * std::cos(angle)), q(signal * std::sin(angle))}), LINEARS_OK);
        ASSERT_EQ(A::step(&ctx, zero, zero, q(2 * signal), q(-signal), q(0.01), &active), LINEARS_OK);
        EXPECT_NE(active.switch_state, 0);
        EXPECT_NE(active.switch_state, 7);
        ASSERT_EQ(A::step(&ctx, zero, zero, q(2 * signal), q(0), q(0.01), &neutral), LINEARS_OK);
        const unsigned bits = (active.switch_state & 1u) + ((active.switch_state >> 1u) & 1u) +
                              ((active.switch_state >> 2u) & 1u);
        EXPECT_EQ(neutral.switch_state, bits >= 2 ? 7 : 0);
        EXPECT_EQ(neutral.torque_state, 0);
        const auto actual = dtc_test::inverter_voltage(neutral.switch_state, 48);
        EXPECT_EQ(actual[0], 0);
        EXPECT_EQ(actual[1], 0);
        ASSERT_EQ(A::step(&ctx, zero, zero, q(2 * signal), q(0), q(0.01), &retained), LINEARS_OK);
        EXPECT_EQ(retained.switch_state, neutral.switch_state);
    }
}

TYPED_TEST(Dtc, HysteresisEqualityMemoryAndZeroCrossings) {
    using A = dtc_test::Api<TypeParam>;
    auto q = A::make;
    typename A::context ctx{};
    typename A::result out{};
    const typename A::ab zero{q(0), q(0)};
    const double signal = A::fixed ? 0.5 : 1;
    ASSERT_EQ(A::init(&ctx, {q(0), q(0.125 * signal), q(0.125), 1}, {q(signal), q(0)}), LINEARS_OK);
    // All thresholds below are exactly binary representable in all variants.
    for (const auto item : std::array<std::array<double, 3>, 11>{{
        {1.125, 0.125, 1}, {1, 0.0625, 1}, {0.875, 0, 0},
        {1, -0.0625, 0}, {1, -0.125, -1}, {1, -0.0625, -1},
        {1, 0, 0}, {1, 0.0625, 0}, {1.125, 0.125, 1},
        {1, -0.03125, 0}, {1, -0.25, -1}}}) {
        const int old_flux_state = ctx.flux_state;
        ASSERT_EQ(A::step(&ctx, zero, zero, q(signal * item[0]), q(item[1]), q(0.125), &out), LINEARS_OK);
        EXPECT_EQ(out.torque_state, item[2]);
        if (item[0] == 0.875) { EXPECT_EQ(out.flux_state, -1); }
        if (item[0] == 1.125) { EXPECT_EQ(out.flux_state, 1); }
        if (item[0] == 1) { EXPECT_EQ(out.flux_state, old_flux_state); }
    }
    // Inside the flux band retain the negative decision even at zero error.
    ASSERT_EQ(A::step(&ctx, zero, zero, q(signal * 0.875), q(0), q(0.125), &out), LINEARS_OK);
    ASSERT_EQ(A::step(&ctx, zero, zero, q(signal * 1.0625), q(0), q(0.125), &out), LINEARS_OK);
    EXPECT_EQ(out.flux_state, -1);
}

TYPED_TEST(Dtc, CenteredSectorEdgesAndAxes) {
    using A = dtc_test::Api<TypeParam>;
    auto q = A::make;
    const typename A::ab zero{q(0), q(0)};
    const double signal = A::fixed ? 0.25 : 1;
    // +/-0.001 turn is far above input quantization, while testing both sides
    // of every centered-sector boundary rather than only vector centers.
    for (int boundary = 0; boundary < 6; ++boundary) for (int side : {-1, 1}) {
        const double turns = (2 * boundary + 1) / 12.0 + side * 0.001;
        typename A::context ctx{};
        typename A::result out{};
        const double a = 2 * std::acos(-1.0) * turns;
        ASSERT_EQ(A::init(&ctx, {q(0), q(0.125), q(0.125), 1},
                          {q(signal * std::cos(a)), q(signal * std::sin(a))}), LINEARS_OK);
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
            const int64_t raw_units[4] = {1, 2, std::numeric_limits<S>::max() / 4,
                                          std::numeric_limits<S>::max() / 3};
            seed = {static_cast<S>(v[0] * raw_units[scale]),
                    static_cast<S>(v[1] * raw_units[scale])};
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

TYPED_TEST(Dtc, WorkingLsbIncrementsRemainInCallerOwnedFluxState) {
    using A = dtc_test::Api<TypeParam>;
    auto q = A::make;
    typename A::context ctx{};
    typename A::result out{};
    ASSERT_EQ(A::init(&ctx, {q(0x1p-15), q(0.001), q(0.01), 1},
                      {q(0.25), q(-0.125)}), LINEARS_OK);
    for (int n = 1; n <= 65536; ++n) {
        ASSERT_EQ(A::step(&ctx, {q(0.5), q(0)}, {q(0), q(0)},
                          q(0.3), q(0), q(0x1p-15), &out), LINEARS_OK);
        if constexpr (A::fixed) {
            EXPECT_EQ(ctx.flux_alpha, LINEARS_FIXED_ONE / 4 - n);
            if constexpr (std::is_same_v<TypeParam, fc_test::Q15>) {
                EXPECT_EQ(out.stator_flux.alpha, (ctx.flux_alpha + 32768) / 65536);
            } else { EXPECT_EQ(out.stator_flux.alpha, ctx.flux_alpha); }
            dtc_test::near(0.25L - n * 0x1p-31L, 0, out.stator_flux.alpha);
        } else if constexpr (std::is_same_v<TypeParam, fc_test::F32>) {
            // A 2^-31 decrement is below the float working ULP at 1/4.
            EXPECT_EQ(ctx.flux_alpha, 0.25f);
        } else EXPECT_EQ(ctx.flux_alpha, 0.25 - n * 0x1p-31);
        EXPECT_EQ(value(ctx.flux_beta), -0.125);
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
    const double signal = A::fixed ? 0.25 : 1;
    ASSERT_EQ(A::init(&ctx, {q(0.5), q(0.125), q(0.125), 1}, {q(signal), q(0)}), LINEARS_OK);
    for (int n = 0; n < 30; ++n) {
        ASSERT_EQ(A::step(&ctx, zero, zero, q(2 * signal), q(signal), q(0.125), &out), LINEARS_OK);
        ASSERT_NE(out.switch_state, 0);
        ASSERT_NE(out.switch_state, 7);
        EXPECT_EQ(value(ctx.flux_alpha), signal);
        EXPECT_EQ(value(ctx.flux_beta), 0);
    }
    // The caller reports a voltage different from the command; that is integrated.
    ASSERT_EQ(A::step(&ctx, zero, {q(-0.25), q(0.5)}, q(2 * signal), q(signal), q(0.125), &out), LINEARS_OK);
    EXPECT_EQ(value(ctx.flux_alpha), signal - 0.03125);
    EXPECT_EQ(value(ctx.flux_beta), 0.0625);
}

TYPED_TEST(Dtc, AllFailuresPreserveContextAndOutputBytes) {
    using A = dtc_test::Api<TypeParam>;
    auto q = A::make;
    typename A::context ctx{};
    typename A::result out;
    std::memset(&out, 0x6b, sizeof(out));
    const typename A::config cfg{q(0.5), q(0.125), q(0.125), 2};
    const typename A::ab zero{q(0), q(0)}, seed{q(A::fixed ? 0.25 : 1), q(0)};
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
    EXPECT_EQ(A::step(&ctx, zero, {q(A::fixed ? -0.5 : -8), q(0)}, q(1), q(0),
                      q(A::fixed ? 0.5 : 0.125), &out), LINEARS_ERROR_DOMAIN);
    EXPECT_EQ(A::step(&ctx, zero, zero, q(1), q(0), q(0.125),
                      reinterpret_cast<typename A::result*>(&ctx)), LINEARS_ERROR_ALIAS);
    EXPECT_EQ(dtc_test::bytes(ctx), before_ctx);
    EXPECT_EQ(dtc_test::bytes(out), before_out);
}

TYPED_TEST(Dtc, RawInputEndpointsAndNonfiniteArguments) {
    using A = dtc_test::Api<TypeParam>;
    auto q = A::make;
    using S = typename A::scalar;
    typename A::context ctx{};
    typename A::result out;
    std::memset(&out, 0x39, sizeof(out));
    const typename A::config cfg{q(0.5), q(0.125), q(0.125), 2};
    const typename A::ab zero{q(0), q(0)}, seed{q(A::fixed ? 0.25 : 1), q(0)};
    ASSERT_EQ(A::init(&ctx, cfg, seed), LINEARS_OK);
    const auto before_ctx = dtc_test::bytes(ctx), before_out = dtc_test::bytes(out);
    if constexpr (A::fixed) {
        static_assert(std::is_integral_v<S>);
        for (S raw : {std::numeric_limits<S>::min(), std::numeric_limits<S>::max(), S(-1), S(1)}) {
            for (int field = 0; field < 5; ++field) {
                ASSERT_EQ(A::init(&ctx, cfg, seed), LINEARS_OK);
                const auto saved = dtc_test::bytes(ctx);
                auto c = cfg;
                auto f = seed;
                if (field == 0) c.resistance = raw;
                if (field == 1) c.flux_band = raw;
                if (field == 2) c.torque_band = raw;
                if (field == 3) f.alpha = raw;
                if (field == 4) f.beta = raw;
                const auto expected = field < 3 && raw < 0 ? LINEARS_ERROR_ARGUMENT : LINEARS_OK;
                EXPECT_EQ(A::init(&ctx, c, f), expected);
                if (expected != LINEARS_OK) { EXPECT_EQ(dtc_test::bytes(ctx), saved); }
            }
            for (int field = 0; field < 7; ++field) {
                ASSERT_EQ(A::init(&ctx, cfg, seed), LINEARS_OK);
                const auto saved_ctx = dtc_test::bytes(ctx), saved_out = dtc_test::bytes(out);
                auto i = zero, v = zero;
                auto flux_ref = q(0.5), torque_ref = q(0), dt = q(0.125);
                if (field == 0) i.alpha = raw;
                if (field == 1) i.beta = raw;
                if (field == 2) v.alpha = raw;
                if (field == 3) v.beta = raw;
                if (field == 4) flux_ref = raw;
                if (field == 5) torque_ref = raw;
                if (field == 6) dt = raw;
                const auto expected = (field == 4 && raw <= cfg.flux_band) || (field == 6 && raw < 0)
                    ? LINEARS_ERROR_ARGUMENT : LINEARS_OK;
                EXPECT_EQ(A::step(&ctx, i, v, flux_ref, torque_ref, dt, &out), expected);
                if (expected != LINEARS_OK) {
                    EXPECT_EQ(dtc_test::bytes(ctx), saved_ctx);
                    EXPECT_EQ(dtc_test::bytes(out), saved_out);
                }
            }
        }
    } else for (int kind = 0; kind < 3; ++kind) {
        const S bad = kind == 0 ? std::numeric_limits<S>::quiet_NaN() :
            kind == 1 ? std::numeric_limits<S>::infinity() : -std::numeric_limits<S>::infinity();
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
    ASSERT_EQ(A::init(&ctx, {q(0.5), q(0.125), q(0.125), 2},
                      {q(A::fixed ? 0.75 : 1), q(0)}), LINEARS_OK);
    const auto before_ctx = dtc_test::bytes(ctx), before_out = dtc_test::bytes(out);
    S huge;
    if constexpr (A::fixed) huge = q(0.5);
    else huge = std::numeric_limits<S>::max();
    EXPECT_EQ(A::step(&ctx, zero, {huge, q(0)}, q(1), q(0), q(A::fixed ? 0.75 : 4), &out), LINEARS_ERROR_RANGE);
    EXPECT_EQ(dtc_test::bytes(ctx), before_ctx);
    EXPECT_EQ(dtc_test::bytes(out), before_out);
    if constexpr (A::fixed) {
        for (S end : {std::numeric_limits<S>::min(), std::numeric_limits<S>::max(), S(-1), S(1)}) {
            ASSERT_EQ(A::init(&ctx, {q(0), q(0.125), q(0.125), 1}, {end, q(0)}), LINEARS_OK);
            ASSERT_EQ(A::step(&ctx, zero, zero, q(0.5), q(0), q(0.5), &out), LINEARS_OK);
            EXPECT_EQ(out.stator_flux.alpha, end);
            const auto mag = end == std::numeric_limits<S>::min() ? std::numeric_limits<S>::max()
                             : static_cast<S>(end < 0 ? -end : end);
            EXPECT_EQ(out.flux_magnitude, mag);
        }
        for (int sign : {-1, 1}) {
            // Exact +/-1 is accepted. One additional F31 working LSB is not.
            ASSERT_EQ(A::init(&ctx, {q(0x1p-15), q(0.125), q(0.125), 1},
                              {q(sign * 0.75), q(0)}), LINEARS_OK);
            ASSERT_EQ(A::step(&ctx, zero, {q(sign * 0.5), q(0)}, q(0.5), q(0), q(0.5), &out), LINEARS_OK);
            EXPECT_EQ(ctx.flux_alpha, sign * LINEARS_FIXED_ONE);
            EXPECT_EQ(out.stator_flux.alpha, sign > 0 ? std::numeric_limits<S>::max() : std::numeric_limits<S>::min());
            const auto saved_ctx = dtc_test::bytes(ctx), saved_out = dtc_test::bytes(out);
            EXPECT_EQ(A::step(&ctx, {q(-sign * 0.5), q(0)}, zero,
                              q(0.5), q(0), q(0x1p-15), &out), LINEARS_ERROR_RANGE);
            EXPECT_EQ(dtc_test::bytes(ctx), saved_ctx);
            EXPECT_EQ(dtc_test::bytes(out), saved_out);
        }
        const S largest = std::numeric_limits<S>::max();
        ASSERT_EQ(A::init(&ctx, {q(0), q(0.125), q(0.125), 1}, {largest, q(0)}), LINEARS_OK);
        ASSERT_EQ(A::step(&ctx, zero, {S(1), q(0)}, q(0.5), q(0), q(0.5), &out), LINEARS_OK);
        EXPECT_EQ(out.stator_flux.alpha, largest); // Upper rounding cell maps to MAX.
        for (S tiny : {S(-1), S(1)}) {
            ASSERT_EQ(A::init(&ctx, {q(0), q(0.125), q(0.125), 1}, {q(0.5), tiny}), LINEARS_OK);
            ASSERT_EQ(A::step(&ctx, zero, zero, q(0.5), q(0), q(0.5), &out), LINEARS_OK);
            EXPECT_EQ(out.sector, 1);
            ASSERT_EQ(A::init(&ctx, {q(0), q(0.125), q(0.125), 1}, {tiny, q(0.5)}), LINEARS_OK);
            ASSERT_EQ(A::step(&ctx, zero, zero, q(0.5), q(0), q(0.5), &out), LINEARS_OK);
            EXPECT_EQ(out.sector, tiny > 0 ? 2 : 3);
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
        if constexpr (A::fixed) {
            const auto saved_ctx = dtc_test::bytes(ctx), saved_out = dtc_test::bytes(out);
            EXPECT_EQ(A::step(&ctx, {q(0), q(direction)}, zero, q(1), q(0), q(0.125), &out), LINEARS_ERROR_RANGE);
            EXPECT_EQ(dtc_test::bytes(ctx), saved_ctx);
            EXPECT_EQ(dtc_test::bytes(out), saved_out);
            ASSERT_EQ(A::init(&ctx, {q(0), q(0.125), q(0.125), UINT16_MAX}, {q(0x1p-9), q(0)}), LINEARS_OK);
            ASSERT_EQ(A::step(&ctx, {q(0), q(direction * 0x1p-9)}, zero, q(0.5), q(0), q(0.125), &out), LINEARS_OK);
            dtc_test::near(direction * 1.5L * UINT16_MAX * 0x1p-18L, 0, out.torque);
        } else {
            ASSERT_EQ(A::step(&ctx, {q(0), q(direction)}, zero, q(1), q(0), q(0.125), &out), LINEARS_OK);
            dtc_test::near(direction * 49151.25L, 0, out.torque);
        }
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
    const double signal = A::fixed ? 0.125 : 1;
    for (int n = 0; n < 1000; ++n) {
        const typename A::ab i{q(signal * std::sin(n * 0.01)), q(signal * std::cos(n * 0.01))};
        const typename A::ab v{q(3 * signal * std::cos(n * 0.013)), q(-2 * signal * std::sin(n * 0.013))};
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

template<class TypeParam>
static void run_dtc_plant(double voltage_base, double current_base, double time_base,
                          const std::string& unit_case, bool require_total_error) {
    using A = dtc_test::Api<TypeParam>;
    using Plant = dtc_test::Plant;
    auto q = A::make;
    const double flux_base = voltage_base * time_base;
    const double torque_base = voltage_base * current_base * time_base;
    const auto sample = q(dtc_sample_time / time_base);
    const double dt = value(sample) * time_base; // Actual quantized sample interval.
    const auto bus_input = q(48 / voltage_base);
    const double bus = value(bus_input) * voltage_base; // +1 -> MAX really changes the plant bus.
    const double command = 0.4, flux_band = 0.001, torque_band = 0.015;
    const double iq_ref = command / (1.5 * Plant::pole_pairs * Plant::magnet_flux);
    const auto flux_ref = q(std::hypot(Plant::magnet_flux, Plant::inductance * iq_ref) / flux_base);
    if constexpr (A::fixed) {
        if (voltage_base == 48) { EXPECT_LT(bus, 48); }
        else {
            EXPECT_EQ(bus, 48);
            EXPECT_EQ(dt, dtc_sample_time);
        }
    }
    for (int direction : {-1, 1}) {
        Plant plant;
        plant.theta = 0.37;
        plant.omega = direction * 20;
        typename A::context ctx{};
        typename A::result result{};
        const auto seed = plant.flux();
        ASSERT_EQ(A::init(&ctx, {q(Plant::resistance * current_base / voltage_base),
                                q(flux_band / flux_base), q(torque_band / torque_base), Plant::pole_pairs},
                          {q(seed[0] / flux_base), q(seed[1] / flux_base)}), LINEARS_OK);
        uint8_t executed = 0;
        double peak_flux_error = 0, peak_estimation_error = 0, peak_current = 0;
        double peak_input_bias = 0, peak_model_error = 0;
        double peak_display_quantization = 0, peak_displayed_error = 0;
        long double input_bias_a = flux_base * value(ctx.flux_alpha) - seed[0];
        long double input_bias_b = flux_base * value(ctx.flux_beta) - seed[1];
        const long double decoded_r = value(ctx.config.resistance) * voltage_base / current_base;
        double torque_squared_error = 0, signed_torque_error = 0;
        double forward_speed = 0;
        unsigned steady_count = 0, zero_count = 0;
        for (int n = 0; n < 20000; ++n) {
            // Only the plant sees true theta. The DTC receives current and actual
            // terminal voltage, including the previous step's one-interval delay.
            const auto v = dtc_test::inverter_voltage(executed, bus);
            plant.advance(v[0], v[1], dt);
            const double torque_ref = direction * (n < 5000 ? command : -command);
            const typename A::ab current{q(plant.ia / current_base), q(plant.ib / current_base)};
            const typename A::ab voltage{q(v[0] / voltage_base), q(v[1] / voltage_base)};
            const auto expected = dtc_test::reference<TypeParam>(ctx, current, voltage, sample);
            ASSERT_EQ(A::step(&ctx, current, voltage, flux_ref, q(torque_ref / torque_base),
                              sample, &result), LINEARS_OK);
            dtc_test::near(expected.alpha, expected.alpha_error, ctx.flux_alpha);
            dtc_test::near(expected.beta, expected.beta_error, ctx.flux_beta);
            executed = result.switch_state;
            zero_count += executed == 0 || executed == 7;
            peak_current = std::max(peak_current, std::hypot(plant.ia, plant.ib));
            // This bias prediction is diagnostic only: it never changes ctx.
            // It is derived from independently quantized voltage/current/R,
            // not fitted to the measured estimator error or the true angle.
            input_bias_a += dt * ((voltage_base * value(voltage.alpha) - v[0]) -
                (decoded_r * current_base * value(current.alpha) - Plant::resistance * plant.ia));
            input_bias_b += dt * ((voltage_base * value(voltage.beta) - v[1]) -
                (decoded_r * current_base * value(current.beta) - Plant::resistance * plant.ib));
            const auto actual_flux = plant.flux();
            const double error_a = flux_base * value(ctx.flux_alpha) - actual_flux[0];
            const double error_b = flux_base * value(ctx.flux_beta) - actual_flux[1];
            peak_estimation_error = std::max(peak_estimation_error, std::hypot(error_a, error_b));
            peak_input_bias = std::max(peak_input_bias, static_cast<double>(std::hypot(input_bias_a, input_bias_b)));
            peak_model_error = std::max(peak_model_error,
                static_cast<double>(std::hypot(error_a - input_bias_a, error_b - input_bias_b)));
            const double display_delta_a = flux_base * (value(result.stator_flux.alpha) - value(ctx.flux_alpha));
            const double display_delta_b = flux_base * (value(result.stator_flux.beta) - value(ctx.flux_beta));
            peak_display_quantization = std::max(peak_display_quantization,
                std::hypot(display_delta_a, display_delta_b));
            peak_displayed_error = std::max(peak_displayed_error,
                std::hypot(error_a + display_delta_a, error_b + display_delta_b));
            peak_flux_error = std::max(peak_flux_error,
                std::abs(std::hypot(actual_flux[0], actual_flux[1]) - flux_base * value(flux_ref)));
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
        const std::string prefix = unit_case + (direction > 0 ? "positive_start_" : "negative_start_");
        ::testing::Test::RecordProperty(prefix + "rms_torque_error", ::testing::PrintToString(rms));
        ::testing::Test::RecordProperty(prefix + "peak_flux_estimation_error", ::testing::PrintToString(peak_estimation_error));
        ::testing::Test::RecordProperty(prefix + "peak_input_quantization_bias", ::testing::PrintToString(peak_input_bias));
        ::testing::Test::RecordProperty(prefix + "peak_flux_model_residual", ::testing::PrintToString(peak_model_error));
        ::testing::Test::RecordProperty(prefix + "peak_display_quantization", ::testing::PrintToString(peak_display_quantization));
        ::testing::Test::RecordProperty(prefix + "peak_displayed_flux_error", ::testing::PrintToString(peak_displayed_error));
        ::testing::Test::RecordProperty(prefix + "actual_sample_time", ::testing::PrintToString(dt));
        ::testing::Test::RecordProperty(prefix + "final_speed", ::testing::PrintToString(plant.omega));
        ::testing::Test::RecordProperty(prefix + "peak_current", ::testing::PrintToString(peak_current));
        ::testing::Test::RecordProperty(prefix + "peak_flux_error", ::testing::PrintToString(peak_flux_error));
        // The maximum active vector is 2/3 Vdc. One sample's torque increment
        // is bounded here by (3 p psi_pm / 2L) * |v| dt, plus hysteresis and
        // a 10% reserve for R*i and back EMF at this specified operating point.
        const double one_sample_torque = 1.5 * Plant::pole_pairs * Plant::magnet_flux /
                                         Plant::inductance * (2 * bus / 3) * dt;
        EXPECT_LT(rms, torque_band + 1.1 * one_sample_torque);
        EXPECT_LT(std::abs(signed_torque_error / steady_count), torque_band);
        // Flux ripple includes hysteresis, one voltage step and the independently
        // checked observer discretization/quantization error, not a fitted bound.
        const double observer_budget = 0.01 * flux_band;
        if constexpr (A::fixed) {
            // Input/R quantization produces real voltage-model drift; do not
            // disguise it as a larger numerical tolerance or feed it back.
            EXPECT_LT(peak_model_error, observer_budget);
        }
        const double total_budget = require_total_error ? observer_budget : peak_input_bias + observer_budget;
        EXPECT_LT(peak_estimation_error, total_budget);
        // Plant flux is away from the upper export cell; q15 has half-LSB
        // rounding per component. Other outputs preserve the work fraction.
        const double display_budget = std::is_same_v<TypeParam, fc_test::Q15>
            ? flux_base * std::sqrt(2.0) / 65536 : 0;
        EXPECT_LE(peak_display_quantization, std::nextafter(display_budget, INFINITY));
        EXPECT_LT(peak_displayed_error, total_budget + display_budget);
        EXPECT_LT(peak_flux_error, flux_band + (2 * bus / 3) * dt + observer_budget);
        EXPECT_LT(peak_current, 3.0);
        EXPECT_GT(direction * forward_speed, 24);
        EXPECT_LT(direction * plant.omega, -20);
        EXPECT_GT(zero_count, 0u);
    }
}

TYPED_TEST(Dtc, SurfacePmsmMechanicalClosedLoopTracksBothSignsAndReverses) {
    if constexpr (TypeParam::fixed) {
        run_dtc_plant<TypeParam>(48, 8, 0.005, "quantized_r_", false);
        // Rpu=.0625, bus=.75, active alpha=.5 and dtpu=1/256 are exact.
        // Tbase=Ts*256 preserves the actual 10 us plant interval; Ts*64
        // would require PM flux >1 pu and cannot represent this motor.
        run_dtc_plant<TypeParam>(64, 10, dtc_sample_time * 256, "exact_r_", true);
    } else run_dtc_plant<TypeParam>(1, 1, 1, "", true);
}
