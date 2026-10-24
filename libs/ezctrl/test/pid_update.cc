/*
 * Update-step tests of the typed PID C API: term arithmetic as delivered by
 * fieldctrl, output clamping, anti-windup, callback delivery and rejected
 * steps.
 *
 * The suite runs once per scalar variant, so the documented semantics must hold
 * identically for f32 and f64.
 */

#include "pid_test_types.h"

#include <limits>

TYPED_TEST(PidTest, ProportionalTermFollowsTheError) {
    using api = ezc_test::pid_api<TypeParam>;
    using T = TypeParam;
    const T eps = ezc_test::pid_epsilon<T>();

    typename api::context_type ctx{};
    typename api::config_type cfg = api::make_config();
    cfg.kp = T(2);
    ASSERT_EQ(api::init(&ctx, &cfg), EZC_OK);

    EXPECT_NEAR(api::step(&ctx, T(1), T(0)), T(2), eps);
    EXPECT_NEAR(api::step(&ctx, T(1), T(0.5)), T(1), eps);
    EXPECT_NEAR(api::step(&ctx, T(4), T(4)), T(0), eps);
    EXPECT_EQ(ctx.pid.integral, T(0));
}

TYPED_TEST(PidTest, IntegralTermAccumulatesPerSample) {
    using api = ezc_test::pid_api<TypeParam>;
    using T = TypeParam;
    const T eps = ezc_test::pid_epsilon<T>();

    typename api::context_type ctx{};
    typename api::config_type cfg = api::make_config();
    cfg.ki = T(0.5);
    ASSERT_EQ(api::init(&ctx, &cfg), EZC_OK);

    EXPECT_NEAR(api::step(&ctx, T(1), T(0)), T(0.5), eps);
    EXPECT_NEAR(api::step(&ctx, T(1), T(0)), T(1), eps);
    EXPECT_NEAR(api::step(&ctx, T(1), T(0)), T(1.5), eps);
    EXPECT_NEAR(ctx.pid.integral, T(1.5), eps);
}

TYPED_TEST(PidTest, DerivativeTermFollowsErrorChanges) {
    using api = ezc_test::pid_api<TypeParam>;
    using T = TypeParam;
    const T eps = ezc_test::pid_epsilon<T>();

    typename api::context_type ctx{};
    typename api::config_type cfg = api::make_config();
    cfg.kd = T(1);
    ASSERT_EQ(api::init(&ctx, &cfg), EZC_OK);

    /* The first step only records the error: a nonzero start causes no kick. */
    EXPECT_NEAR(api::step(&ctx, T(1), T(0)), T(0), eps);
    /* Same error: the derivative is zero even while the setpoint is nonzero. */
    EXPECT_NEAR(api::step(&ctx, T(1), T(0)), T(0), eps);
    /* Error increases from 1 to 3: the derivative is 2. */
    EXPECT_NEAR(api::step(&ctx, T(3), T(0)), T(2), eps);
    EXPECT_EQ(ctx.pid.previous_error, T(3));
}

TYPED_TEST(PidTest, FullPidCombinesTheTerms) {
    using api = ezc_test::pid_api<TypeParam>;
    using T = TypeParam;
    const T eps = ezc_test::pid_epsilon<T>();

    typename api::context_type ctx{};
    typename api::config_type cfg = api::make_config();
    cfg.kp = T(1);
    cfg.ki = T(0.5);
    cfg.kd = T(0.25);
    ASSERT_EQ(api::init(&ctx, &cfg), EZC_OK);

    /* P = 1, I = 0.5, D = 0 on the first step */
    EXPECT_NEAR(api::step(&ctx, T(1), T(0)), T(1.5), eps);
    /* P = 1, I = 1.0, D = 0 */
    EXPECT_NEAR(api::step(&ctx, T(1), T(0)), T(2), eps);
    /* P = 3, I = 2.5, D = 0.25 * (3 - 1) */
    EXPECT_NEAR(api::step(&ctx, T(3), T(0)), T(6), eps);
    EXPECT_NEAR(ctx.pid.integral, T(2.5), eps);
    EXPECT_EQ(ctx.pid.previous_error, T(3));
}

TYPED_TEST(PidTest, LimitClampsBothDirections) {
    using api = ezc_test::pid_api<TypeParam>;
    using T = TypeParam;
    const T eps = ezc_test::pid_epsilon<T>();

    typename api::context_type ctx{};
    typename api::config_type cfg = api::make_config();
    cfg.kp = T(10);
    cfg.limit_enabled = true;
    cfg.out_min = T(-1);
    cfg.out_max = T(1);
    ASSERT_EQ(api::init(&ctx, &cfg), EZC_OK);

    EXPECT_NEAR(api::step(&ctx, T(1), T(0)), T(1), eps);
    EXPECT_NEAR(api::step(&ctx, T(-1), T(0)), T(-1), eps);
    /* Inside the range the output is not modified. */
    EXPECT_NEAR(api::step(&ctx, T(0.05), T(0)), T(0.5), eps);
}

TYPED_TEST(PidTest, CallbackReceivesContextAndResult) {
    using api = ezc_test::pid_api<TypeParam>;
    using T = TypeParam;
    const T eps = ezc_test::pid_epsilon<T>();

    typename api::context_type ctx{};
    typename api::config_type cfg = api::make_config();
    typename api::callback_sink sink;
    cfg.kp = T(2);
    cfg.ki = T(1);
    cfg.kd = T(0.5);
    cfg.callback = &api::callback;
    cfg.callback_ctx = &sink;
    ASSERT_EQ(api::init(&ctx, &cfg), EZC_OK);

    const T first = api::step(&ctx, T(1), T(0));

    EXPECT_EQ(sink.calls, 1);
    EXPECT_EQ(sink.last_user, static_cast<void *>(&sink));
    EXPECT_EQ(sink.output, first);
    EXPECT_NEAR(first, T(3), eps);
    EXPECT_NEAR(sink.setpoint, T(1), eps);
    EXPECT_NEAR(sink.measure, T(0), eps);
    EXPECT_NEAR(sink.error, T(1), eps);
    EXPECT_NEAR(sink.p_term, T(2), eps);
    EXPECT_NEAR(sink.i_term, T(1), eps);
    EXPECT_NEAR(sink.d_term, T(0), eps);
    EXPECT_FALSE(sink.limited);

    /* e = 1.5: P = 3, I = 1 + 1.5, D = 0.5 * (1.5 - 1) */
    const T second = api::step(&ctx, T(2), T(0.5));

    EXPECT_EQ(sink.calls, 2);
    EXPECT_EQ(sink.output, second);
    EXPECT_NEAR(second, T(5.75), eps);
    EXPECT_NEAR(sink.error, T(1.5), eps);
    EXPECT_NEAR(sink.p_term, T(3), eps);
    EXPECT_NEAR(sink.i_term, T(2.5), eps);
    EXPECT_NEAR(sink.d_term, T(0.25), eps);
    EXPECT_EQ(ctx.output, second);
}

TYPED_TEST(PidTest, CallbackReportsClippedOutput) {
    using api = ezc_test::pid_api<TypeParam>;
    using T = TypeParam;

    typename api::context_type ctx{};
    typename api::config_type cfg = api::make_config();
    typename api::callback_sink sink;
    cfg.kp = T(10);
    cfg.limit_enabled = true;
    cfg.out_min = T(-1);
    cfg.out_max = T(1);
    cfg.callback = &api::callback;
    cfg.callback_ctx = &sink;
    ASSERT_EQ(api::init(&ctx, &cfg), EZC_OK);

    EXPECT_EQ(api::step(&ctx, T(1), T(0)), T(1));
    EXPECT_EQ(sink.calls, 1);
    EXPECT_TRUE(sink.limited);
    EXPECT_EQ(sink.output, T(1));

    EXPECT_NEAR(api::step(&ctx, T(0.05), T(0)), T(0.5), ezc_test::pid_epsilon<T>());
    EXPECT_EQ(sink.calls, 2);
    EXPECT_FALSE(sink.limited);
    EXPECT_NEAR(sink.output, T(0.5), ezc_test::pid_epsilon<T>());
}

TYPED_TEST(PidTest, CallbackCanBeReplacedAndRemoved) {
    using api = ezc_test::pid_api<TypeParam>;
    using T = TypeParam;

    typename api::context_type ctx{};
    typename api::config_type cfg = api::make_config();
    typename api::callback_sink first;
    typename api::callback_sink second;
    cfg.kp = T(2);
    ASSERT_EQ(api::init(&ctx, &cfg), EZC_OK);

    ASSERT_EQ(api::set_callback(&ctx, &api::callback, &first), EZC_OK);
    EXPECT_EQ(api::step(&ctx, T(1), T(0)), T(2));
    EXPECT_EQ(first.calls, 1);
    EXPECT_EQ(second.calls, 0);

    ASSERT_EQ(api::set_callback(&ctx, &api::callback, &second), EZC_OK);
    EXPECT_EQ(api::step(&ctx, T(1), T(0)), T(2));
    EXPECT_EQ(first.calls, 1);
    EXPECT_EQ(second.calls, 1);

    ASSERT_EQ(api::set_callback(&ctx, nullptr, nullptr), EZC_OK);
    EXPECT_EQ(ctx.cfg.callback, nullptr);
    EXPECT_EQ(ctx.cfg.callback_ctx, nullptr);
    /* The output is still computed and returned without a callback. */
    EXPECT_EQ(api::step(&ctx, T(1), T(0)), T(2));
    EXPECT_EQ(first.calls, 1);
    EXPECT_EQ(second.calls, 1);
}

TYPED_TEST(PidTest, AntiWindupAdvancesTheIntegralToTheSaturationBoundary) {
    using api = ezc_test::pid_api<TypeParam>;
    using T = TypeParam;
    const T eps = ezc_test::pid_epsilon<T>();

    typename api::context_type ctx{};
    typename api::config_type cfg = api::make_config();
    typename api::callback_sink sink;
    cfg.kp = T(1);
    cfg.ki = T(1);
    cfg.limit_enabled = true;
    cfg.out_min = T(-10);
    cfg.out_max = T(10);
    cfg.callback = &api::callback;
    cfg.callback_ctx = &sink;
    ASSERT_EQ(api::init(&ctx, &cfg), EZC_OK);

    /* P = 8, I would be 8: the integral stops where P + I is exactly 10. */
    EXPECT_NEAR(api::step(&ctx, T(8), T(0)), T(10), eps);
    EXPECT_NEAR(ctx.pid.integral, T(2), eps);
    EXPECT_TRUE(sink.limited);

    /* A reversed error leaves saturation at once, on the lower boundary. */
    EXPECT_NEAR(api::step(&ctx, T(-8), T(0)), T(-10), eps);
    EXPECT_NEAR(ctx.pid.integral, T(-2), eps);

    /* Back inside the range the limiter does not touch the state. */
    EXPECT_NEAR(api::step(&ctx, T(1), T(0)), T(0), eps);
    EXPECT_NEAR(ctx.pid.integral, T(-1), eps);
    EXPECT_FALSE(sink.limited);
}

TYPED_TEST(PidTest, IntegralIsBoundedByTheOutputRange) {
    using api = ezc_test::pid_api<TypeParam>;
    using T = TypeParam;
    const T eps = ezc_test::pid_epsilon<T>();

    typename api::context_type ctx{};
    typename api::config_type cfg = api::make_config();
    cfg.ki = T(1);
    cfg.limit_enabled = true;
    cfg.out_min = T(-10);
    cfg.out_max = T(10);
    ASSERT_EQ(api::init(&ctx, &cfg), EZC_OK);

    /* The integral cannot wind up beyond the output range. */
    EXPECT_NEAR(api::step(&ctx, T(1000), T(0)), T(10), eps);
    EXPECT_NEAR(ctx.pid.integral, T(10), eps);

    /* So a single reversed step already reaches the other limit. */
    EXPECT_NEAR(api::step(&ctx, T(-20), T(0)), T(-10), eps);
    EXPECT_NEAR(ctx.pid.integral, T(-10), eps);

    EXPECT_NEAR(api::step(&ctx, T(4), T(0)), T(-6), eps);
    EXPECT_NEAR(ctx.pid.integral, T(-6), eps);
}

TYPED_TEST(PidTest, AntiWindupDoesNotManufactureIntegralState) {
    using api = ezc_test::pid_api<TypeParam>;
    using T = TypeParam;

    typename api::context_type ctx{};
    typename api::config_type cfg = api::make_config();
    cfg.kp = T(100);
    cfg.ki = T(1);
    cfg.limit_enabled = true;
    cfg.out_min = T(-10);
    cfg.out_max = T(10);
    ASSERT_EQ(api::init(&ctx, &cfg), EZC_OK);

    /* The proportional term alone saturates, so the integral must not move. */
    EXPECT_EQ(api::step(&ctx, T(1), T(0)), T(10));
    EXPECT_EQ(ctx.pid.integral, T(0));

    EXPECT_EQ(api::step(&ctx, T(1), T(0)), T(10));
    EXPECT_EQ(ctx.pid.integral, T(0));
}

TYPED_TEST(PidTest, WithoutALimitTheIntegralIsUnbounded) {
    using api = ezc_test::pid_api<TypeParam>;
    using T = TypeParam;
    const T eps = ezc_test::pid_epsilon<T>();

    typename api::context_type ctx{};
    typename api::config_type cfg = api::make_config();
    cfg.ki = T(1);
    ASSERT_EQ(api::init(&ctx, &cfg), EZC_OK);

    EXPECT_NEAR(api::step(&ctx, T(1000), T(0)), T(1000), eps);
    EXPECT_NEAR(ctx.pid.integral, T(1000), eps);
}

TYPED_TEST(PidTest, RejectedStepsChangeNothingAndReturnTheLastOutput) {
    using api = ezc_test::pid_api<TypeParam>;
    using T = TypeParam;
    const T eps = ezc_test::pid_epsilon<T>();
    const T nan = std::numeric_limits<T>::quiet_NaN();
    const T inf = std::numeric_limits<T>::infinity();
    const T max = std::numeric_limits<T>::max();
    /* setpoint, measure, dt; the last case overflows the error. */
    const T rejected[][3] = {
        { nan, T(0), T(1) }, { T(1), nan, T(1) }, { inf, T(0), T(1) }, { T(1), -inf, T(1) },
        { T(2), T(0), T(0) }, { T(2), T(0), T(-0.5) }, { T(2), T(0), nan }, { T(2), T(0), inf },
        { max, -max, T(1) },
    };

    typename api::context_type ctx{};
    typename api::config_type cfg = api::make_config();
    typename api::callback_sink sink;
    cfg.kp = T(1);
    cfg.ki = T(1);
    cfg.kd = T(1);
    cfg.callback = &api::callback;
    cfg.callback_ctx = &sink;
    ASSERT_EQ(api::init(&ctx, &cfg), EZC_OK);

    /* One second of settled state: I = 1, e(n-1) = 1, output = P + I = 2. */
    const T held = api::update(&ctx, T(1), T(0), T(1));
    ASSERT_NEAR(held, T(2), eps);

    for (const auto &input : rejected) {
        EXPECT_EQ(api::update(&ctx, input[0], input[1], input[2]), held);
        EXPECT_EQ(ctx.pid.integral, T(1));
        EXPECT_EQ(ctx.pid.previous_error, T(1));
        EXPECT_EQ(ctx.output, held);
        EXPECT_EQ(sink.calls, 1);
    }

    /* The next valid step continues from the untouched state. */
    /* P = 2, I = 1 + 1 * 0.5 * 2, D = 1 * (2 - 1) / 0.5 */
    EXPECT_NEAR(api::update(&ctx, T(2), T(0), T(0.5)), T(6), eps);
    EXPECT_EQ(sink.calls, 2);
}

TYPED_TEST(PidTest, RejectedFirstStepReturnsZero) {
    using api = ezc_test::pid_api<TypeParam>;
    using T = TypeParam;

    typename api::context_type ctx{};
    typename api::config_type cfg = api::make_config();
    cfg.kp = T(1);
    ASSERT_EQ(api::init(&ctx, &cfg), EZC_OK);

    EXPECT_EQ(api::update(&ctx, T(1), T(0), T(0)), T(0));
    EXPECT_EQ(ctx.pid.history, 0);
    EXPECT_EQ(api::update(&ctx, T(1), T(0), T(1)), T(1));
}

TYPED_TEST(PidTest, ReverseActingGainsDriveACoolingActuator) {
    using api = ezc_test::pid_api<TypeParam>;
    using T = TypeParam;
    const T eps = ezc_test::pid_epsilon<T>();

    typename api::context_type ctx{};
    typename api::config_type cfg = api::make_config();
    typename api::callback_sink sink;
    /* More cooling lowers the temperature: all gains are negative. */
    cfg.kp = T(-0.5);
    cfg.ki = T(-0.25);
    cfg.limit_enabled = true;
    cfg.out_min = T(0);
    cfg.out_max = T(1);
    cfg.callback = &api::callback;
    cfg.callback_ctx = &sink;
    ASSERT_EQ(api::init(&ctx, &cfg), EZC_OK);

    /* 2 K too warm: P = 1 alone saturates, so anti-windup holds I at 0. */
    EXPECT_NEAR(api::step(&ctx, T(20), T(22)), T(1), eps);
    EXPECT_TRUE(sink.limited);
    EXPECT_EQ(ctx.pid.integral, T(0));

    /* 0.5 K too warm: P = 0.25, I = 0.125. The callback reports real inputs. */
    EXPECT_NEAR(api::step(&ctx, T(20), T(20.5)), T(0.375), eps);
    EXPECT_FALSE(sink.limited);
    EXPECT_EQ(sink.setpoint, T(20));
    EXPECT_EQ(sink.measure, T(20.5));
    EXPECT_EQ(sink.error, T(-0.5));
    EXPECT_NEAR(sink.p_term, T(0.25), eps);
    EXPECT_NEAR(sink.i_term, T(0.125), eps);

    /* Too cold: the cooler switches off at the lower limit. */
    EXPECT_EQ(api::step(&ctx, T(20), T(18)), T(0));
}

TYPED_TEST(PidTest, SamplePeriodScalesIntegralAndDerivative) {
    using api = ezc_test::pid_api<TypeParam>;
    using T = TypeParam;
    const T eps = ezc_test::pid_epsilon<T>();

    typename api::context_type ctx{};
    typename api::config_type cfg = api::make_config();
    cfg.ki = T(2);
    cfg.kd = T(1);
    ASSERT_EQ(api::init(&ctx, &cfg), EZC_OK);

    /* dt = 0.5 s: I = 2 * 0.5 * 1 = 1, no derivative on the first step */
    EXPECT_NEAR(api::update(&ctx, T(1), T(0), T(0.5)), T(1), eps);
    EXPECT_NEAR(ctx.pid.integral, T(1), eps);
    /* dt = 0.5 s, e = 2: I = 1 + 2 * 0.5 * 2 = 3, D = 1 * (2 - 1) / 0.5 = 2 */
    EXPECT_NEAR(api::update(&ctx, T(2), T(0), T(0.5)), T(5), eps);
    EXPECT_NEAR(ctx.pid.integral, T(3), eps);
    /* dt = 2 s with the same error: I = 3 + 2 * 2 * 2 = 11, D = 0 */
    EXPECT_NEAR(api::update(&ctx, T(2), T(0), T(2)), T(11), eps);
    EXPECT_NEAR(ctx.pid.integral, T(11), eps);
}

TYPED_TEST(PidTest, VaryingSampleRatesUseTheSameGains) {
    using api = ezc_test::pid_api<TypeParam>;
    using T = TypeParam;
    const T eps = ezc_test::pid_epsilon<T>();

    typename api::context_type ctx{};
    typename api::config_type cfg = api::make_config();
    cfg.ki = T(1);
    ASSERT_EQ(api::init(&ctx, &cfg), EZC_OK);

    /* A jittering period accumulates exactly ki * dt * e per step. */
    EXPECT_NEAR(api::update(&ctx, T(1), T(0), T(0.1)), T(0.1), eps);
    EXPECT_NEAR(api::update(&ctx, T(1), T(0), T(0.2)), T(0.3), eps);
    EXPECT_NEAR(api::update(&ctx, T(1), T(0), T(0.05)), T(0.35), eps);
    EXPECT_NEAR(ctx.pid.integral, T(0.35), eps);
}

TYPED_TEST(PidTest, AntiWindupHoldsAtAnySamplePeriod) {
    using api = ezc_test::pid_api<TypeParam>;
    using T = TypeParam;
    const T eps = ezc_test::pid_epsilon<T>();

    typename api::context_type ctx{};
    typename api::config_type cfg = api::make_config();
    cfg.ki = T(1);
    cfg.limit_enabled = true;
    cfg.out_min = T(-10);
    cfg.out_max = T(10);
    ASSERT_EQ(api::init(&ctx, &cfg), EZC_OK);

    /* dt = 2 s: the integral stops on the boundary just like at 1 s. */
    EXPECT_NEAR(api::update(&ctx, T(1000), T(0), T(2)), T(10), eps);
    EXPECT_NEAR(ctx.pid.integral, T(10), eps);
    EXPECT_NEAR(api::update(&ctx, T(-20), T(0), T(2)), T(-10), eps);
    EXPECT_NEAR(ctx.pid.integral, T(-10), eps);
}

TYPED_TEST(PidTest, UpdateStateIsIndependentPerContext) {
    using api = ezc_test::pid_api<TypeParam>;
    using T = TypeParam;

    typename api::context_type first{};
    typename api::context_type second{};
    typename api::config_type cfg = api::make_config();
    cfg.ki = T(1);
    ASSERT_EQ(api::init(&first, &cfg), EZC_OK);
    ASSERT_EQ(api::init(&second, &cfg), EZC_OK);

    EXPECT_EQ(api::step(&first, T(1), T(0)), T(1));
    EXPECT_EQ(api::step(&first, T(1), T(0)), T(2));
    EXPECT_EQ(api::step(&second, T(1), T(0)), T(1));
    EXPECT_EQ(first.pid.integral, T(2));
    EXPECT_EQ(second.pid.integral, T(1));
}
