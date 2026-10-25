/*
 * Configuration, validation and state tests of the typed PID C API.
 *
 * The suite runs once per scalar variant, so f32 and f64 must both satisfy the
 * same contract for initialization, runtime setters, rejection of invalid
 * arguments, the mapping onto the fieldctrl controller and state reset.
 */

#include "pid_test_types.h"

#include <limits>

TYPED_TEST(PidTest, ZeroInitializedContextIsADisabledController) {
    using api = ezc_test::pid_api<TypeParam>;
    using T = TypeParam;

    typename api::context_type ctx{};

    EXPECT_EQ(ctx.cfg.kp, T(0));
    EXPECT_EQ(ctx.cfg.ki, T(0));
    EXPECT_EQ(ctx.cfg.kd, T(0));
    EXPECT_FALSE(ctx.cfg.limit_enabled);
    EXPECT_EQ(ctx.cfg.callback, nullptr);
    EXPECT_EQ(ctx.cfg.callback_ctx, nullptr);
    EXPECT_EQ(api::step(&ctx, T(5), T(0)), T(0));

    /* The first step built the disabled controller over the whole finite range. */
    EXPECT_EQ(ctx.pid.config.lower, -std::numeric_limits<T>::max());
    EXPECT_EQ(ctx.pid.config.upper, std::numeric_limits<T>::max());
}

TYPED_TEST(PidTest, ZeroInitializedContextAcceptsACallbackBeforeTheFirstStep) {
    using api = ezc_test::pid_api<TypeParam>;
    using T = TypeParam;

    typename api::context_type ctx{};
    typename api::callback_sink sink;

    ASSERT_EQ(api::set_callback(&ctx, &api::callback, &sink), EZC_OK);
    EXPECT_EQ(api::step(&ctx, T(2), T(0.5)), T(0));
    EXPECT_EQ(sink.calls, 1);
    EXPECT_EQ(sink.error, T(1.5));
    EXPECT_EQ(sink.output, T(0));
}

TYPED_TEST(PidTest, InitRejectsNullContext) {
    using api = ezc_test::pid_api<TypeParam>;

    typename api::config_type cfg = api::make_config();
    cfg.kp = TypeParam(1);

    EXPECT_EQ(api::init(nullptr, &cfg), EZC_ERR_INVALID_ARG);
}

TYPED_TEST(PidTest, InitRejectsNullConfigAndClearsTheContext) {
    using api = ezc_test::pid_api<TypeParam>;
    using T = TypeParam;

    typename api::context_type ctx{};
    typename api::config_type cfg = api::make_config();
    cfg.kp = T(4);
    cfg.ki = T(2);
    cfg.kd = T(1);
    cfg.limit_enabled = true;
    cfg.out_min = T(-1);
    cfg.out_max = T(1);
    ASSERT_EQ(api::init(&ctx, &cfg), EZC_OK);
    ASSERT_EQ(api::step(&ctx, T(1), T(0)), T(1));

    EXPECT_EQ(api::init(&ctx, nullptr), EZC_ERR_INVALID_ARG);

    EXPECT_EQ(ctx.cfg.kp, T(0));
    EXPECT_EQ(ctx.cfg.ki, T(0));
    EXPECT_EQ(ctx.cfg.kd, T(0));
    EXPECT_FALSE(ctx.cfg.limit_enabled);
    EXPECT_EQ(ctx.cfg.callback, nullptr);
    EXPECT_EQ(ctx.cfg.callback_ctx, nullptr);
    EXPECT_EQ(ctx.pid.integral, T(0));
    EXPECT_EQ(ctx.pid.history, 0);
    EXPECT_EQ(ctx.output, T(0));
    EXPECT_EQ(api::step(&ctx, T(1), T(0)), T(0));
}

TYPED_TEST(PidTest, InitRejectsInvertedLimitAndClearsTheContext) {
    using api = ezc_test::pid_api<TypeParam>;
    using T = TypeParam;

    typename api::context_type ctx{};
    typename api::config_type cfg = api::make_config();
    cfg.kp = T(3);
    cfg.limit_enabled = true;
    cfg.out_min = T(1);
    cfg.out_max = T(-1);

    EXPECT_EQ(api::init(&ctx, &cfg), EZC_ERR_INVALID_ARG);
    EXPECT_EQ(ctx.cfg.kp, T(0));
    EXPECT_FALSE(ctx.cfg.limit_enabled);
    EXPECT_EQ(api::step(&ctx, T(100), T(0)), T(0));
}

TYPED_TEST(PidTest, InitRejectsAnEmptyOrNonFiniteLimit) {
    using api = ezc_test::pid_api<TypeParam>;
    using T = TypeParam;
    const T nan = std::numeric_limits<T>::quiet_NaN();
    const T inf = std::numeric_limits<T>::infinity();
    const T ranges[][2] = { { T(-2), T(-2) }, { nan, T(1) }, { T(-1), nan }, { T(-1), inf } };

    for (const auto &range : ranges) {
        typename api::context_type ctx{};
        typename api::config_type cfg = api::make_config();
        cfg.kp = T(1);
        cfg.limit_enabled = true;
        cfg.out_min = range[0];
        cfg.out_max = range[1];

        EXPECT_EQ(api::init(&ctx, &cfg), EZC_ERR_INVALID_ARG);
        EXPECT_FALSE(ctx.cfg.limit_enabled);
        EXPECT_EQ(ctx.cfg.kp, T(0));
    }
}

TYPED_TEST(PidTest, InitRejectsMixedGainSigns) {
    using api = ezc_test::pid_api<TypeParam>;
    using T = TypeParam;

    for (int gain = 0; gain < 3; ++gain) {
        typename api::context_type ctx{};
        typename api::config_type cfg = api::make_config();
        cfg.kp = T(1);
        cfg.ki = T(1);
        cfg.kd = T(1);
        (gain == 0 ? cfg.kp : (gain == 1 ? cfg.ki : cfg.kd)) = T(-1);

        EXPECT_EQ(api::init(&ctx, &cfg), EZC_ERR_INVALID_ARG) << "gain " << gain;
        EXPECT_EQ(ctx.cfg.kp, T(0));
        EXPECT_EQ(api::step(&ctx, T(1), T(0)), T(0));
    }
}

TYPED_TEST(PidTest, InitAcceptsUniformlyNegativeGainsForReverseAction) {
    using api = ezc_test::pid_api<TypeParam>;
    using T = TypeParam;
    const T eps = ezc_test::pid_epsilon<T>();

    typename api::context_type ctx{};
    typename api::config_type cfg = api::make_config();
    cfg.kp = T(-2);
    cfg.ki = T(-1);
    ASSERT_EQ(api::init(&ctx, &cfg), EZC_OK);
    EXPECT_EQ(ctx.pid.config.kp, T(-2));
    EXPECT_EQ(ctx.pid.config.ki, T(-1));

    /* e = 1: P = -2, I = -1. */
    EXPECT_NEAR(api::step(&ctx, T(1), T(0)), T(-3), eps);
    EXPECT_EQ(ctx.pid.integral, T(-1));
}

TYPED_TEST(PidTest, InitStoresEveryConfigurationField) {
    using api = ezc_test::pid_api<TypeParam>;
    using T = TypeParam;

    typename api::context_type ctx{};
    typename api::config_type cfg = api::make_config();
    typename api::callback_sink sink;

    cfg.kp = T(1.5);
    cfg.ki = T(0.25);
    cfg.kd = T(0.125);
    cfg.limit_enabled = true;
    cfg.out_min = T(-8);
    cfg.out_max = T(8);
    cfg.callback = &api::callback;
    cfg.callback_ctx = &sink;

    ASSERT_EQ(api::init(&ctx, &cfg), EZC_OK);

    EXPECT_EQ(ctx.cfg.kp, cfg.kp);
    EXPECT_EQ(ctx.cfg.ki, cfg.ki);
    EXPECT_EQ(ctx.cfg.kd, cfg.kd);
    EXPECT_EQ(ctx.cfg.out_min, cfg.out_min);
    EXPECT_EQ(ctx.cfg.out_max, cfg.out_max);
    EXPECT_TRUE(ctx.cfg.limit_enabled);
    EXPECT_EQ(ctx.cfg.callback, cfg.callback);
    EXPECT_EQ(ctx.cfg.callback_ctx, cfg.callback_ctx);

    /* The fieldctrl controller carries the mapped tuning and a fresh state. */
    EXPECT_EQ(ctx.pid.config.kp, cfg.kp);
    EXPECT_EQ(ctx.pid.config.ki, cfg.ki);
    EXPECT_EQ(ctx.pid.config.kd, cfg.kd);
    EXPECT_EQ(ctx.pid.config.lower, cfg.out_min);
    EXPECT_EQ(ctx.pid.config.upper, cfg.out_max);
    EXPECT_EQ(ctx.pid.integral, T(0));
    EXPECT_EQ(ctx.pid.history, 0);
    EXPECT_EQ(ctx.output, T(0));
}

TYPED_TEST(PidTest, InitClampsTheInitialIntegralIntoTheOutputRange) {
    using api = ezc_test::pid_api<TypeParam>;
    using T = TypeParam;

    typename api::context_type ctx{};
    typename api::config_type cfg = api::make_config();
    cfg.limit_enabled = true;
    cfg.out_min = T(0.25);
    cfg.out_max = T(1);

    /* fieldctrl requires the integral inside its bounds: zero becomes 0.25. */
    ASSERT_EQ(api::init(&ctx, &cfg), EZC_OK);
    EXPECT_EQ(ctx.pid.integral, T(0.25));
    EXPECT_EQ(api::step(&ctx, T(0), T(0)), T(0.25));
}

TYPED_TEST(PidTest, SettersRejectNullContext) {
    using api = ezc_test::pid_api<TypeParam>;
    using T = TypeParam;

    EXPECT_EQ(api::set_gains(nullptr, T(1), T(0), T(0)), EZC_ERR_INVALID_ARG);
    EXPECT_EQ(api::set_limit(nullptr, T(-1), T(1)), EZC_ERR_INVALID_ARG);
    EXPECT_EQ(api::clear_limit(nullptr), EZC_ERR_INVALID_ARG);
    EXPECT_EQ(api::set_callback(nullptr, nullptr, nullptr), EZC_ERR_INVALID_ARG);

    api::reset(nullptr);
    EXPECT_EQ(api::step(nullptr, T(1), T(0)), T(0));
}

TYPED_TEST(PidTest, SetGainsKeepsTheIntegral) {
    using api = ezc_test::pid_api<TypeParam>;
    using T = TypeParam;
    const T eps = ezc_test::pid_epsilon<T>();

    typename api::context_type ctx{};
    typename api::config_type cfg = api::make_config();
    cfg.ki = T(1);
    ASSERT_EQ(api::init(&ctx, &cfg), EZC_OK);
    ASSERT_NEAR(api::step(&ctx, T(1), T(0)), T(1), eps);
    ASSERT_EQ(ctx.pid.integral, T(1));

    ASSERT_EQ(api::set_gains(&ctx, T(0), T(2), T(0)), EZC_OK);

    EXPECT_EQ(ctx.cfg.kp, T(0));
    EXPECT_EQ(ctx.cfg.ki, T(2));
    EXPECT_EQ(ctx.cfg.kd, T(0));
    EXPECT_EQ(ctx.pid.config.ki, T(2));
    EXPECT_EQ(ctx.pid.integral, T(1));
    EXPECT_NEAR(api::step(&ctx, T(1), T(0)), T(3), eps);
    EXPECT_EQ(ctx.pid.integral, T(3));
}

TYPED_TEST(PidTest, SetGainsRejectsMixedSignsOrNonFiniteGainsAndKeepsTheContext) {
    using api = ezc_test::pid_api<TypeParam>;
    using T = TypeParam;
    const T nan = std::numeric_limits<T>::quiet_NaN();

    typename api::context_type ctx{};
    typename api::config_type cfg = api::make_config();
    cfg.kp = T(1);
    cfg.ki = T(1);
    ASSERT_EQ(api::init(&ctx, &cfg), EZC_OK);
    ASSERT_EQ(api::step(&ctx, T(1), T(0)), T(2));

    EXPECT_EQ(api::set_gains(&ctx, T(-1), T(1), T(0)), EZC_ERR_INVALID_ARG);
    EXPECT_EQ(api::set_gains(&ctx, T(1), T(-1), T(0)), EZC_ERR_INVALID_ARG);
    EXPECT_EQ(api::set_gains(&ctx, T(0), T(1), T(-1)), EZC_ERR_INVALID_ARG);
    EXPECT_EQ(api::set_gains(&ctx, nan, T(0), T(0)), EZC_ERR_INVALID_ARG);

    /* Nothing was retuned: tuning, integral and history are untouched. */
    EXPECT_EQ(ctx.cfg.kp, T(1));
    EXPECT_EQ(ctx.cfg.ki, T(1));
    EXPECT_EQ(ctx.pid.config.kp, T(1));
    EXPECT_EQ(ctx.pid.integral, T(1));
    EXPECT_NE(ctx.pid.history, 0);
    EXPECT_EQ(api::step(&ctx, T(1), T(0)), T(3));
}

TYPED_TEST(PidTest, SetLimitRejectsInvalidRangeAndKeepsThePreviousLimit) {
    using api = ezc_test::pid_api<TypeParam>;
    using T = TypeParam;
    const T nan = std::numeric_limits<T>::quiet_NaN();
    const T inf = std::numeric_limits<T>::infinity();

    typename api::context_type ctx{};
    typename api::config_type cfg = api::make_config();
    cfg.kp = T(10);
    ASSERT_EQ(api::init(&ctx, &cfg), EZC_OK);
    ASSERT_EQ(api::set_limit(&ctx, T(-1), T(1)), EZC_OK);
    ASSERT_EQ(api::step(&ctx, T(1), T(0)), T(1));

    EXPECT_EQ(api::set_limit(&ctx, T(5), T(1)), EZC_ERR_INVALID_ARG);
    EXPECT_EQ(api::set_limit(&ctx, T(2), T(2)), EZC_ERR_INVALID_ARG);
    EXPECT_EQ(api::set_limit(&ctx, nan, T(1)), EZC_ERR_INVALID_ARG);
    EXPECT_EQ(api::set_limit(&ctx, T(-1), inf), EZC_ERR_INVALID_ARG);

    EXPECT_TRUE(ctx.cfg.limit_enabled);
    EXPECT_EQ(ctx.cfg.out_min, T(-1));
    EXPECT_EQ(ctx.cfg.out_max, T(1));
    EXPECT_EQ(ctx.pid.config.lower, T(-1));
    EXPECT_EQ(ctx.pid.config.upper, T(1));
    EXPECT_EQ(api::step(&ctx, T(1), T(0)), T(1));
}

TYPED_TEST(PidTest, SetLimitClampsTheIntegralIntoTheNewRange) {
    using api = ezc_test::pid_api<TypeParam>;
    using T = TypeParam;

    typename api::context_type ctx{};
    typename api::config_type cfg = api::make_config();
    cfg.ki = T(1);
    ASSERT_EQ(api::init(&ctx, &cfg), EZC_OK);
    ASSERT_EQ(api::step(&ctx, T(1), T(0)), T(1));
    ASSERT_EQ(api::step(&ctx, T(1), T(0)), T(2));
    ASSERT_EQ(api::step(&ctx, T(1), T(0)), T(3));

    ASSERT_EQ(api::set_limit(&ctx, T(-2), T(2)), EZC_OK);
    EXPECT_EQ(ctx.pid.integral, T(2));
    EXPECT_EQ(api::step(&ctx, T(0), T(0)), T(2));
}

TYPED_TEST(PidTest, ClearLimitDisablesClamping) {
    using api = ezc_test::pid_api<TypeParam>;
    using T = TypeParam;

    typename api::context_type ctx{};
    typename api::config_type cfg = api::make_config();
    cfg.kp = T(5);
    ASSERT_EQ(api::init(&ctx, &cfg), EZC_OK);
    ASSERT_EQ(api::set_limit(&ctx, T(-1), T(1)), EZC_OK);
    ASSERT_EQ(api::step(&ctx, T(1), T(0)), T(1));

    ASSERT_EQ(api::clear_limit(&ctx), EZC_OK);
    EXPECT_FALSE(ctx.cfg.limit_enabled);
    EXPECT_EQ(ctx.cfg.out_min, T(-1));
    EXPECT_EQ(ctx.cfg.out_max, T(1));
    EXPECT_EQ(ctx.pid.config.lower, -std::numeric_limits<T>::max());
    EXPECT_EQ(ctx.pid.config.upper, std::numeric_limits<T>::max());
    EXPECT_EQ(api::step(&ctx, T(1), T(0)), T(5));
}

TYPED_TEST(PidTest, SettersKeepTheDerivativeHistory) {
    using api = ezc_test::pid_api<TypeParam>;
    using T = TypeParam;

    typename api::context_type ctx{};
    typename api::config_type cfg = api::make_config();
    cfg.kd = T(1);
    ASSERT_EQ(api::init(&ctx, &cfg), EZC_OK);
    ASSERT_EQ(api::step(&ctx, T(1), T(0)), T(0));
    ASSERT_EQ(api::step(&ctx, T(3), T(0)), T(2));

    /* Retuning keeps the history and, with unchanged ki/kd, the dt cache. */
    ASSERT_EQ(api::set_gains(&ctx, T(0), T(0), T(1)), EZC_OK);
    EXPECT_NE(ctx.pid.history, 0);
    EXPECT_EQ(ctx.pid.cached_dt, T(1));

    /* The error change 3 -> 5 is differentiated across the setter. */
    EXPECT_EQ(api::step(&ctx, T(5), T(0)), T(2));
    EXPECT_EQ(api::step(&ctx, T(6), T(0)), T(1));
}

TYPED_TEST(PidTest, SetLimitEveryPeriodDoesNotDisturbTheController) {
    using api = ezc_test::pid_api<TypeParam>;
    using T = TypeParam;

    typename api::context_type dynamic{};
    typename api::context_type fixed{};
    typename api::config_type cfg = api::make_config();
    cfg.kp = T(1);
    cfg.ki = T(0.5);
    cfg.kd = T(0.25);
    cfg.deriv_filter_t = T(0.5);
    cfg.limit_enabled = true;
    cfg.out_min = T(-4);
    cfg.out_max = T(4);
    ASSERT_EQ(api::init(&dynamic, &cfg), EZC_OK);
    ASSERT_EQ(api::init(&fixed, &cfg), EZC_OK);

    /* A limit that is reapplied every period, as from a supply measurement. */
    for (int n = 0; n < 40; ++n) {
        const T setpoint = (n % 10 < 5) ? T(3) : T(-2);
        const T measure = T(0.125) * T(n % 4);
        ASSERT_EQ(api::set_limit(&dynamic, T(-4), T(4)), EZC_OK);
        EXPECT_EQ(api::update(&dynamic, setpoint, measure, T(0.25)),
                  api::update(&fixed, setpoint, measure, T(0.25)))
            << "step " << n;
    }
    EXPECT_EQ(dynamic.pid.integral, fixed.pid.integral);
    EXPECT_EQ(dynamic.pid.derivative, fixed.pid.derivative);
    EXPECT_NE(fixed.pid.derivative, T(0));
}

TYPED_TEST(PidTest, SetCallbackDoesNotRetuneTheController) {
    using api = ezc_test::pid_api<TypeParam>;
    using T = TypeParam;

    typename api::context_type ctx{};
    typename api::config_type cfg = api::make_config();
    typename api::callback_sink sink;
    cfg.kd = T(1);
    ASSERT_EQ(api::init(&ctx, &cfg), EZC_OK);
    ASSERT_EQ(api::step(&ctx, T(1), T(0)), T(0));

    ASSERT_EQ(api::set_callback(&ctx, &api::callback, &sink), EZC_OK);
    EXPECT_NE(ctx.pid.history, 0);
    EXPECT_EQ(api::step(&ctx, T(3), T(0)), T(2));
    EXPECT_EQ(sink.calls, 1);
    EXPECT_EQ(sink.d_term, T(2));
}

TYPED_TEST(PidTest, ResetClearsStateAndKeepsConfiguration) {
    using api = ezc_test::pid_api<TypeParam>;
    using T = TypeParam;
    const T eps = ezc_test::pid_epsilon<T>();

    typename api::context_type ctx{};
    typename api::config_type cfg = api::make_config();
    typename api::callback_sink sink;
    cfg.ki = T(1);
    cfg.limit_enabled = true;
    cfg.out_min = T(-10);
    cfg.out_max = T(10);
    cfg.callback = &api::callback;
    cfg.callback_ctx = &sink;
    ASSERT_EQ(api::init(&ctx, &cfg), EZC_OK);

    ASSERT_NEAR(api::step(&ctx, T(1), T(0)), T(1), eps);
    ASSERT_EQ(api::step(&ctx, T(1), T(0)), T(2));
    ASSERT_EQ(ctx.pid.integral, T(2));

    api::reset(&ctx);

    EXPECT_EQ(ctx.pid.integral, T(0));
    EXPECT_EQ(ctx.pid.derivative, T(0));
    EXPECT_EQ(ctx.pid.history, 0);
    EXPECT_EQ(ctx.cfg.ki, T(1));
    EXPECT_TRUE(ctx.cfg.limit_enabled);
    EXPECT_EQ(ctx.cfg.out_min, T(-10));
    EXPECT_EQ(ctx.cfg.out_max, T(10));
    EXPECT_EQ(ctx.cfg.callback, &api::callback);
    EXPECT_EQ(ctx.cfg.callback_ctx, &sink);
    /* The last output stays the value a rejected step would hold. */
    EXPECT_EQ(ctx.output, T(2));

    EXPECT_NEAR(api::step(&ctx, T(1), T(0)), T(1), eps);
    EXPECT_EQ(sink.calls, 3);
}

TYPED_TEST(PidTest, ResetClampsZeroIntoTheIntegralBounds) {
    using api = ezc_test::pid_api<TypeParam>;
    using T = TypeParam;

    typename api::context_type ctx{};
    typename api::config_type cfg = api::make_config();
    cfg.ki = T(1);
    cfg.limit_enabled = true;
    cfg.out_min = T(0.25);
    cfg.out_max = T(4);
    ASSERT_EQ(api::init(&ctx, &cfg), EZC_OK);
    ASSERT_EQ(api::step(&ctx, T(1), T(0)), T(1.25));

    api::reset(&ctx);
    EXPECT_EQ(ctx.pid.integral, T(0.25));
}

TYPED_TEST(PidTest, ResetIgnoresAnUnbuiltContext) {
    using api = ezc_test::pid_api<TypeParam>;
    using T = TypeParam;

    typename api::context_type ctx{};
    api::reset(&ctx);
    EXPECT_EQ(ctx.pid.config.upper, T(0));
    EXPECT_EQ(api::step(&ctx, T(1), T(0)), T(0));
}

TYPED_TEST(PidTest, ResetToStartsFromTheGivenIntegral) {
    using api = ezc_test::pid_api<TypeParam>;
    using T = TypeParam;
    const T nan = std::numeric_limits<T>::quiet_NaN();

    typename api::context_type ctx{};
    typename api::config_type cfg = api::make_config();
    cfg.kp = T(1);
    cfg.ki = T(1);
    cfg.kd = T(1);
    cfg.limit_enabled = true;
    cfg.out_min = T(0);
    cfg.out_max = T(10);
    ASSERT_EQ(api::init(&ctx, &cfg), EZC_OK);
    ASSERT_EQ(api::step(&ctx, T(1), T(0)), T(2));

    /* Bumpless switch from a manual command of 6 at zero error. */
    ASSERT_EQ(api::reset_to(&ctx, T(6)), EZC_OK);
    EXPECT_EQ(ctx.pid.integral, T(6));
    EXPECT_EQ(ctx.pid.history, 0);
    EXPECT_EQ(ctx.output, T(2));
    EXPECT_EQ(api::step(&ctx, T(0), T(0)), T(6));

    /* Outside the integral bounds or not finite: rejected, nothing changes. */
    EXPECT_EQ(api::reset_to(&ctx, T(11)), EZC_ERR_INVALID_ARG);
    EXPECT_EQ(api::reset_to(&ctx, T(-1)), EZC_ERR_INVALID_ARG);
    EXPECT_EQ(api::reset_to(&ctx, nan), EZC_ERR_INVALID_ARG);
    EXPECT_EQ(api::reset_to(nullptr, T(0)), EZC_ERR_INVALID_ARG);
    EXPECT_EQ(ctx.pid.integral, T(6));
    EXPECT_NE(ctx.pid.history, 0);

    /* A zero-initialized context is built with the given integral. */
    typename api::context_type fresh{};
    ASSERT_EQ(api::reset_to(&fresh, T(3)), EZC_OK);
    EXPECT_EQ(fresh.pid.integral, T(3));
    EXPECT_EQ(api::step(&fresh, T(5), T(0)), T(3));
}
