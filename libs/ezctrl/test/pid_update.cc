/*
 * Update-step tests of the typed PID C API: term arithmetic, output clamping,
 * anti-windup behavior and callback delivery.
 *
 * The suite runs once per scalar variant, so the documented semantics must hold
 * identically for f32 and f64.
 */

#include "pid_test_types.h"

#include <cmath>
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
    EXPECT_EQ(ctx.integral, T(0));
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
    EXPECT_NEAR(ctx.integral, T(1.5), eps);
}

TYPED_TEST(PidTest, DerivativeTermFollowsErrorChanges) {
    using api = ezc_test::pid_api<TypeParam>;
    using T = TypeParam;
    const T eps = ezc_test::pid_epsilon<T>();

    typename api::context_type ctx{};
    typename api::config_type cfg = api::make_config();
    cfg.kd = T(1);
    ASSERT_EQ(api::init(&ctx, &cfg), EZC_OK);

    /* First step: the missing previous error counts as zero. */
    EXPECT_NEAR(api::step(&ctx, T(1), T(0)), T(1), eps);
    /* Same error: the derivative is zero even while the setpoint is nonzero. */
    EXPECT_NEAR(api::step(&ctx, T(1), T(0)), T(0), eps);
    /* Error increases from 1 to 3: the derivative is 2. */
    EXPECT_NEAR(api::step(&ctx, T(3), T(0)), T(2), eps);
    EXPECT_EQ(ctx.prev_error, T(3));
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

    /* P = 1, I = 0.5, D = 0.25 * (1 - 0) */
    EXPECT_NEAR(api::step(&ctx, T(1), T(0)), T(1.75), eps);
    /* P = 1, I = 1.0, D = 0 */
    EXPECT_NEAR(api::step(&ctx, T(1), T(0)), T(2), eps);
    EXPECT_NEAR(ctx.integral, T(1), eps);
    EXPECT_EQ(ctx.prev_error, T(1));
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

    const T output = api::step(&ctx, T(1), T(0));

    EXPECT_EQ(sink.calls, 1);
    EXPECT_EQ(sink.last_user, static_cast<void *>(&sink));
    EXPECT_EQ(sink.output, output);
    EXPECT_NEAR(output, T(3.5), eps);
    EXPECT_NEAR(sink.setpoint, T(1), eps);
    EXPECT_NEAR(sink.measure, T(0), eps);
    EXPECT_NEAR(sink.error, T(1), eps);
    EXPECT_NEAR(sink.p_term, T(2), eps);
    EXPECT_NEAR(sink.i_term, T(1), eps);
    EXPECT_NEAR(sink.d_term, T(0.5), eps);
    EXPECT_FALSE(sink.limited);
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
    EXPECT_EQ(ctx.callback, nullptr);
    EXPECT_EQ(ctx.callback_ctx, nullptr);
    /* The output is still computed and returned without a callback. */
    EXPECT_EQ(api::step(&ctx, T(1), T(0)), T(2));
    EXPECT_EQ(first.calls, 1);
    EXPECT_EQ(second.calls, 1);
}

TYPED_TEST(PidTest, AntiWindupHoldsTheIntegralAtTheSaturationBoundary) {
    using api = ezc_test::pid_api<TypeParam>;
    using T = TypeParam;
    const T eps = ezc_test::pid_epsilon<T>();

    typename api::context_type ctx{};
    typename api::config_type cfg = api::make_config();
    cfg.ki = T(1);
    cfg.limit_enabled = true;
    cfg.out_min = T(-10);
    cfg.out_max = T(10);
    cfg.anti_windup_enabled = true;
    ASSERT_EQ(api::init(&ctx, &cfg), EZC_OK);

    /* The integral stops exactly where the output reaches the upper limit. */
    EXPECT_NEAR(api::step(&ctx, T(1000), T(0)), T(10), eps);
    EXPECT_NEAR(ctx.integral, T(10), eps);

    /* A reversed error leaves saturation immediately instead of unwinding. */
    EXPECT_NEAR(api::step(&ctx, T(-1000), T(0)), T(-10), eps);
    EXPECT_NEAR(ctx.integral, T(-10), eps);

    /* Back inside the range the limiter does not touch the state at all. */
    EXPECT_NEAR(api::step(&ctx, T(4), T(0)), T(-6), eps);
    EXPECT_NEAR(ctx.integral, T(-6), eps);
}

TYPED_TEST(PidTest, WithoutAntiWindupTheIntegralWindsUp) {
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

    /* The output is clamped, but the integral keeps the whole error. */
    EXPECT_NEAR(api::step(&ctx, T(1000), T(0)), T(10), eps);
    EXPECT_NEAR(ctx.integral, T(1000), eps);

    /* The wound-up integral keeps the output pinned after the error reverses. */
    EXPECT_NEAR(api::step(&ctx, T(-20), T(0)), T(10), eps);
    EXPECT_NEAR(ctx.integral, T(980), eps);
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
    cfg.anti_windup_enabled = true;
    ASSERT_EQ(api::init(&ctx, &cfg), EZC_OK);

    /* The proportional term alone saturates, so the integral must not move. */
    EXPECT_EQ(api::step(&ctx, T(1), T(0)), T(10));
    EXPECT_EQ(ctx.integral, T(0));

    EXPECT_EQ(api::step(&ctx, T(1), T(0)), T(10));
    EXPECT_EQ(ctx.integral, T(0));
}

TYPED_TEST(PidTest, AntiWindupIsInertWhileTheLimitIsDisabled) {
    using api = ezc_test::pid_api<TypeParam>;
    using T = TypeParam;
    const T eps = ezc_test::pid_epsilon<T>();

    typename api::context_type ctx{};
    typename api::config_type cfg = api::make_config();
    cfg.ki = T(1);
    cfg.anti_windup_enabled = true;
    ASSERT_EQ(api::init(&ctx, &cfg), EZC_OK);

    EXPECT_NEAR(api::step(&ctx, T(1000), T(0)), T(1000), eps);
    EXPECT_NEAR(ctx.integral, T(1000), eps);
}

TYPED_TEST(PidTest, NonFiniteInputsAreNotRejected) {
    using api = ezc_test::pid_api<TypeParam>;
    using T = TypeParam;

    typename api::context_type ctx{};
    typename api::config_type cfg = api::make_config();
    cfg.kp = T(1);
    cfg.ki = T(1);
    cfg.limit_enabled = true;
    cfg.out_min = T(-1);
    cfg.out_max = T(1);
    ASSERT_EQ(api::init(&ctx, &cfg), EZC_OK);

    /*
     * NaN reaches the output and the state instead of being rejected. The
     * limiter cannot absorb it either, because every comparison against NaN is
     * false, so the caller must validate sensor data.
     */
    const T nan = std::numeric_limits<T>::quiet_NaN();
    EXPECT_TRUE(std::isnan(api::step(&ctx, nan, T(0))));
    EXPECT_TRUE(std::isnan(ctx.integral));
    EXPECT_TRUE(std::isnan(ctx.prev_error));

    /* Reset is the documented way out of a poisoned state. */
    api::reset(&ctx);
    EXPECT_EQ(ctx.integral, T(0));
    EXPECT_EQ(ctx.prev_error, T(0));
    EXPECT_EQ(api::step(&ctx, T(1), T(0)), T(1));
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

    /* dt = 0.5 s: I = 2 * 1 * 0.5 = 1, D = 1 * (1 - 0) / 0.5 = 2 */
    EXPECT_NEAR(api::update(&ctx, T(1), T(0), T(0.5)), T(3), eps);
    EXPECT_NEAR(ctx.integral, T(1), eps);
    /* dt = 0.5 s again with the same error: I = 2, D = 0 */
    EXPECT_NEAR(api::update(&ctx, T(1), T(0), T(0.5)), T(2), eps);
    EXPECT_NEAR(ctx.integral, T(2), eps);
    /* dt = 2 s: I = 2 + 2 * 1 * 2 = 6, D = 0 */
    EXPECT_NEAR(api::update(&ctx, T(1), T(0), T(2)), T(6), eps);
    EXPECT_NEAR(ctx.integral, T(6), eps);
}

TYPED_TEST(PidTest, VaryingSampleRatesUseTheSameGains) {
    using api = ezc_test::pid_api<TypeParam>;
    using T = TypeParam;
    const T eps = ezc_test::pid_epsilon<T>();

    typename api::context_type ctx{};
    typename api::config_type cfg = api::make_config();
    cfg.ki = T(1);
    ASSERT_EQ(api::init(&ctx, &cfg), EZC_OK);

    /* A jittering period accumulates exactly ki * e * dt per step. */
    EXPECT_NEAR(api::update(&ctx, T(1), T(0), T(0.1)), T(0.1), eps);
    EXPECT_NEAR(api::update(&ctx, T(1), T(0), T(0.2)), T(0.3), eps);
    EXPECT_NEAR(api::update(&ctx, T(1), T(0), T(0.05)), T(0.35), eps);
    EXPECT_NEAR(ctx.integral, T(0.35), eps);
}

TYPED_TEST(PidTest, InvalidSamplePeriodIsAZeroTimeStep) {
    using api = ezc_test::pid_api<TypeParam>;
    using T = TypeParam;
    const T eps = ezc_test::pid_epsilon<T>();
    const T nan = std::numeric_limits<T>::quiet_NaN();
    const T inf = std::numeric_limits<T>::infinity();
    const T invalid[] = { T(0), T(-0.5), nan, inf };

    for (const T dt : invalid) {
        typename api::context_type ctx{};
        typename api::config_type cfg = api::make_config();
        cfg.kp = T(1);
        cfg.ki = T(1);
        cfg.kd = T(1);
        ASSERT_EQ(api::init(&ctx, &cfg), EZC_OK);

        /* One second of settled state: I = 1, e(n-1) = 1, output = 3. */
        ASSERT_NEAR(api::update(&ctx, T(1), T(0), T(1)), T(3), eps);
        ASSERT_NEAR(ctx.integral, T(1), eps);

        /* The invalid period holds the integral and drops the derivative. */
        EXPECT_NEAR(api::update(&ctx, T(2), T(0), dt), T(3), eps);
        EXPECT_EQ(ctx.integral, T(1));
        EXPECT_EQ(ctx.prev_error, T(2));

        /* The next valid step sees a single interval of error change. */
        EXPECT_NEAR(api::update(&ctx, T(2), T(0), T(0.5)), T(4), eps);
    }
}

TYPED_TEST(PidTest, SamplePeriodReciprocalIsCached) {
    using api = ezc_test::pid_api<TypeParam>;
    using T = TypeParam;

    typename api::context_type ctx{};
    typename api::config_type cfg = api::make_config();
    cfg.kd = T(1);
    ASSERT_EQ(api::init(&ctx, &cfg), EZC_OK);

    /* The first step caches dt and its reciprocal. */
    EXPECT_EQ(api::update(&ctx, T(1), T(0), T(0.25)), T(4));
    EXPECT_EQ(ctx.dt_cached, T(0.25));
    EXPECT_EQ(ctx.dt_inv_cached, T(4));

    /*
     * White-box check that the cached reciprocal is really reused instead of
     * being recomputed every step: while dt is unchanged, a stale cached value
     * is what the derivative term uses, and a new dt refreshes the cache.
     */
    ctx.dt_inv_cached = T(8);
    EXPECT_EQ(api::update(&ctx, T(2), T(0), T(0.25)), T(8));
    EXPECT_EQ(api::update(&ctx, T(3), T(0), T(0.5)), T(2));
    EXPECT_EQ(ctx.dt_cached, T(0.5));
    EXPECT_EQ(ctx.dt_inv_cached, T(2));
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
    cfg.anti_windup_enabled = true;
    ASSERT_EQ(api::init(&ctx, &cfg), EZC_OK);

    /* dt = 2 s: the integral stops on the boundary just like at 1 s. */
    EXPECT_NEAR(api::update(&ctx, T(1000), T(0), T(2)), T(10), eps);
    EXPECT_NEAR(ctx.integral, T(10), eps);
    EXPECT_NEAR(api::update(&ctx, T(-20), T(0), T(2)), T(-10), eps);
    EXPECT_NEAR(ctx.integral, T(-10), eps);
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
    EXPECT_EQ(first.integral, T(2));
    EXPECT_EQ(second.integral, T(1));
}
