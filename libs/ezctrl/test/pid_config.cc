/*
 * Configuration, validation and state tests of the typed PID C API.
 *
 * The suite runs once per scalar variant, so f32 and f64 must both satisfy the
 * same contract for initialization, runtime setters, rejection of invalid
 * arguments and state reset.
 */

#include "pid_test_types.h"

#include <limits>

TYPED_TEST(PidTest, ZeroInitializedContextIsADisabledController) {
    using api = ezc_test::pid_api<TypeParam>;
    using T = TypeParam;

    typename api::context_type ctx{};

    EXPECT_EQ(ctx.kp, T(0));
    EXPECT_EQ(ctx.ki, T(0));
    EXPECT_EQ(ctx.kd, T(0));
    EXPECT_FALSE(ctx.limit_enabled);
    EXPECT_FALSE(ctx.anti_windup_enabled);
    EXPECT_EQ(ctx.callback, nullptr);
    EXPECT_EQ(ctx.callback_ctx, nullptr);
    EXPECT_EQ(api::step(&ctx, T(5), T(0)), T(0));
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
    cfg.anti_windup_enabled = true;
    ASSERT_EQ(api::init(&ctx, &cfg), EZC_OK);
    ASSERT_EQ(api::step(&ctx, T(1), T(0)), T(1));

    EXPECT_EQ(api::init(&ctx, nullptr), EZC_ERR_INVALID_ARG);

    EXPECT_EQ(ctx.kp, T(0));
    EXPECT_EQ(ctx.ki, T(0));
    EXPECT_EQ(ctx.kd, T(0));
    EXPECT_EQ(ctx.integral, T(0));
    EXPECT_EQ(ctx.prev_error, T(0));
    EXPECT_FALSE(ctx.limit_enabled);
    EXPECT_FALSE(ctx.anti_windup_enabled);
    EXPECT_EQ(ctx.callback, nullptr);
    EXPECT_EQ(ctx.callback_ctx, nullptr);
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
    EXPECT_EQ(ctx.kp, T(0));
    EXPECT_FALSE(ctx.limit_enabled);
    EXPECT_EQ(api::step(&ctx, T(100), T(0)), T(0));
}

TYPED_TEST(PidTest, InitRejectsNonComparableLimit) {
    using api = ezc_test::pid_api<TypeParam>;
    using T = TypeParam;

    typename api::context_type ctx{};
    typename api::config_type cfg = api::make_config();
    cfg.limit_enabled = true;
    cfg.out_min = std::numeric_limits<T>::quiet_NaN();
    cfg.out_max = T(1);

    EXPECT_EQ(api::init(&ctx, &cfg), EZC_ERR_INVALID_ARG);
    EXPECT_FALSE(ctx.limit_enabled);

    cfg.out_min = T(-1);
    cfg.out_max = std::numeric_limits<T>::quiet_NaN();
    EXPECT_EQ(api::init(&ctx, &cfg), EZC_ERR_INVALID_ARG);
    EXPECT_FALSE(ctx.limit_enabled);
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
    cfg.anti_windup_enabled = true;
    cfg.callback = &api::callback;
    cfg.callback_ctx = &sink;

    ASSERT_EQ(api::init(&ctx, &cfg), EZC_OK);

    EXPECT_EQ(ctx.kp, cfg.kp);
    EXPECT_EQ(ctx.ki, cfg.ki);
    EXPECT_EQ(ctx.kd, cfg.kd);
    EXPECT_EQ(ctx.out_min, cfg.out_min);
    EXPECT_EQ(ctx.out_max, cfg.out_max);
    EXPECT_TRUE(ctx.limit_enabled);
    EXPECT_TRUE(ctx.anti_windup_enabled);
    EXPECT_EQ(ctx.callback, cfg.callback);
    EXPECT_EQ(ctx.callback_ctx, cfg.callback_ctx);
    EXPECT_EQ(ctx.integral, T(0));
    EXPECT_EQ(ctx.prev_error, T(0));
}

TYPED_TEST(PidTest, InitAcceptsAnEqualLimitAndPinsTheOutput) {
    using api = ezc_test::pid_api<TypeParam>;
    using T = TypeParam;

    typename api::context_type ctx{};
    typename api::config_type cfg = api::make_config();
    cfg.kp = T(1);
    cfg.limit_enabled = true;
    cfg.out_min = T(-2);
    cfg.out_max = T(-2);

    ASSERT_EQ(api::init(&ctx, &cfg), EZC_OK);
    EXPECT_EQ(api::step(&ctx, T(4), T(0)), T(-2));
    EXPECT_EQ(api::step(&ctx, T(-4), T(0)), T(-2));
}

TYPED_TEST(PidTest, SettersRejectNullContext) {
    using api = ezc_test::pid_api<TypeParam>;
    using T = TypeParam;

    EXPECT_EQ(api::set_gains(nullptr, T(1), T(0), T(0)), EZC_ERR_INVALID_ARG);
    EXPECT_EQ(api::set_limit(nullptr, T(-1), T(1)), EZC_ERR_INVALID_ARG);
    EXPECT_EQ(api::clear_limit(nullptr), EZC_ERR_INVALID_ARG);
    EXPECT_EQ(api::set_anti_windup(nullptr, true), EZC_ERR_INVALID_ARG);
    EXPECT_EQ(api::set_callback(nullptr, nullptr, nullptr), EZC_ERR_INVALID_ARG);

    api::reset(nullptr);
    EXPECT_EQ(api::step(nullptr, T(1), T(0)), T(0));
}

TYPED_TEST(PidTest, SetGainsKeepsTheControllerState) {
    using api = ezc_test::pid_api<TypeParam>;
    using T = TypeParam;
    const T eps = ezc_test::pid_epsilon<T>();

    typename api::context_type ctx{};
    typename api::config_type cfg = api::make_config();
    cfg.ki = T(1);
    ASSERT_EQ(api::init(&ctx, &cfg), EZC_OK);
    ASSERT_NEAR(api::step(&ctx, T(1), T(0)), T(1), eps);
    ASSERT_EQ(ctx.integral, T(1));

    ASSERT_EQ(api::set_gains(&ctx, T(0), T(2), T(0)), EZC_OK);

    EXPECT_EQ(ctx.kp, T(0));
    EXPECT_EQ(ctx.ki, T(2));
    EXPECT_EQ(ctx.kd, T(0));
    EXPECT_EQ(ctx.integral, T(1));
    EXPECT_NEAR(api::step(&ctx, T(1), T(0)), T(3), eps);
    EXPECT_EQ(ctx.integral, T(3));
}

TYPED_TEST(PidTest, SetLimitRejectsInvalidRangeAndKeepsThePreviousLimit) {
    using api = ezc_test::pid_api<TypeParam>;
    using T = TypeParam;

    typename api::context_type ctx{};
    typename api::config_type cfg = api::make_config();
    cfg.kp = T(10);
    ASSERT_EQ(api::init(&ctx, &cfg), EZC_OK);
    ASSERT_EQ(api::set_limit(&ctx, T(-1), T(1)), EZC_OK);
    ASSERT_EQ(api::step(&ctx, T(1), T(0)), T(1));

    EXPECT_EQ(api::set_limit(&ctx, T(5), T(1)), EZC_ERR_INVALID_ARG);
    EXPECT_TRUE(ctx.limit_enabled);
    EXPECT_EQ(ctx.out_min, T(-1));
    EXPECT_EQ(ctx.out_max, T(1));
    EXPECT_EQ(api::step(&ctx, T(1), T(0)), T(1));

    EXPECT_EQ(api::set_limit(&ctx, std::numeric_limits<T>::quiet_NaN(), T(1)),
              EZC_ERR_INVALID_ARG);
    EXPECT_TRUE(ctx.limit_enabled);
    EXPECT_EQ(ctx.out_max, T(1));
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
    EXPECT_FALSE(ctx.limit_enabled);
    EXPECT_EQ(ctx.out_min, T(-1));
    EXPECT_EQ(ctx.out_max, T(1));
    EXPECT_EQ(api::step(&ctx, T(1), T(0)), T(5));
}

TYPED_TEST(PidTest, SetAntiWindupIsIndependentOfTheLimit) {
    using api = ezc_test::pid_api<TypeParam>;
    using T = TypeParam;

    typename api::context_type ctx{};
    typename api::config_type cfg = api::make_config();
    cfg.kp = T(1);
    ASSERT_EQ(api::init(&ctx, &cfg), EZC_OK);
    ASSERT_FALSE(ctx.anti_windup_enabled);

    ASSERT_EQ(api::set_anti_windup(&ctx, true), EZC_OK);
    EXPECT_TRUE(ctx.anti_windup_enabled);
    EXPECT_FALSE(ctx.limit_enabled);

    ASSERT_EQ(api::set_anti_windup(&ctx, false), EZC_OK);
    EXPECT_FALSE(ctx.anti_windup_enabled);
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
    ASSERT_EQ(ctx.integral, T(2));

    api::reset(&ctx);

    EXPECT_EQ(ctx.integral, T(0));
    EXPECT_EQ(ctx.prev_error, T(0));
    EXPECT_EQ(ctx.ki, T(1));
    EXPECT_TRUE(ctx.limit_enabled);
    EXPECT_EQ(ctx.out_min, T(-10));
    EXPECT_EQ(ctx.out_max, T(10));
    EXPECT_EQ(ctx.callback, &api::callback);
    EXPECT_EQ(ctx.callback_ctx, &sink);

    EXPECT_NEAR(api::step(&ctx, T(1), T(0)), T(1), eps);
    EXPECT_EQ(sink.calls, 3);
}
