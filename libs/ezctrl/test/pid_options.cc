/*
 * Optional PID feature tests of the typed C API: setpoint weight, derivative
 * source, derivative filter, integral limit and integral separation.
 *
 * The suite runs once per scalar variant and starts from the documented
 * invariant that a zero-initialized configuration has every option switched
 * off, so enabling a feature is always an explicit change.
 */

#include "pid_test_types.h"

#include <limits>

TYPED_TEST(PidTest, OptionsAreOffByDefault) {
    using api = ezc_test::pid_api<TypeParam>;
    using T = TypeParam;

    typename api::context_type ctx{};
    typename api::config_type cfg = api::make_config();
    cfg.kp = T(1);
    cfg.ki = T(1);
    cfg.kd = T(1);
    ASSERT_EQ(api::init(&ctx, &cfg), EZC_OK);

    EXPECT_EQ(ctx.cfg.options, 0u);
    EXPECT_EQ(ctx.cfg.deriv_source, EZC_PID_DERIV_ON_ERROR);
    EXPECT_EQ(ctx.cfg.deriv_filter_t, T(0));
    EXPECT_EQ(ctx.cfg.integral_min, T(0));
    EXPECT_EQ(ctx.cfg.integral_max, T(0));
    EXPECT_EQ(ctx.cfg.integral_separation, T(0));
    EXPECT_EQ(ctx.pid.config.options, 0u);

    /* Default behavior: P + I of the error at dt = 1 s, no first-step D. */
    EXPECT_NEAR(api::step(&ctx, T(1), T(0)), T(2), ezc_test::pid_epsilon<T>());
    EXPECT_EQ(ctx.pid.integral, T(1));
}

TYPED_TEST(PidTest, DerivativeOnMeasurementIgnoresSetpointSteps) {
    using api = ezc_test::pid_api<TypeParam>;
    using T = TypeParam;

    typename api::context_type ctx{};
    typename api::config_type cfg = api::make_config();
    cfg.kd = T(1);
    ASSERT_EQ(api::init(&ctx, &cfg), EZC_OK);
    ASSERT_EQ(api::set_derivative_source(&ctx, EZC_PID_DERIV_ON_MEASUREMENT), EZC_OK);

    /* A setpoint step with a steady measurement produces no derivative. */
    EXPECT_EQ(api::step(&ctx, T(1), T(0)), T(0));
    EXPECT_EQ(api::step(&ctx, T(100), T(0)), T(0));

    /* A measurement change is still differentiated. */
    EXPECT_EQ(api::step(&ctx, T(100), T(2)), T(-2));

    /* The default source adds the setpoint kick for comparison. */
    typename api::context_type kick{};
    ASSERT_EQ(api::init(&kick, &cfg), EZC_OK);
    EXPECT_EQ(api::step(&kick, T(1), T(0)), T(0));
    EXPECT_EQ(api::step(&kick, T(100), T(0)), T(99));
}

TYPED_TEST(PidTest, DerivativeSourceSwitchKeepsTheHistory) {
    using api = ezc_test::pid_api<TypeParam>;
    using T = TypeParam;

    typename api::context_type ctx{};
    typename api::config_type cfg = api::make_config();
    cfg.kd = T(1);
    ASSERT_EQ(api::init(&ctx, &cfg), EZC_OK);

    EXPECT_EQ(api::step(&ctx, T(1), T(0)), T(0));
    /* Error from 1 to -2. */
    EXPECT_EQ(api::step(&ctx, T(1), T(3)), T(-3));

    /* Both signals are recorded every step, so the switch needs no restart. */
    ASSERT_EQ(api::set_derivative_source(&ctx, EZC_PID_DERIV_ON_MEASUREMENT), EZC_OK);
    EXPECT_NE(ctx.pid.history, 0);
    /* Measurement from 3 to 4. */
    EXPECT_EQ(api::step(&ctx, T(1), T(4)), T(-1));
    /* Measurement from 4 to 6; the setpoint step is ignored. */
    EXPECT_EQ(api::step(&ctx, T(2), T(6)), T(-2));

    ASSERT_EQ(api::set_derivative_source(&ctx, EZC_PID_DERIV_ON_ERROR), EZC_OK);
    /* Error from -4 to -3. */
    EXPECT_EQ(api::step(&ctx, T(3), T(6)), T(1));
}

TYPED_TEST(PidTest, DerivativeFilterFollowsTheDiscreteLowPass) {
    using api = ezc_test::pid_api<TypeParam>;
    using T = TypeParam;

    typename api::context_type ctx{};
    typename api::config_type cfg = api::make_config();
    cfg.kd = T(1);
    ASSERT_EQ(api::init(&ctx, &cfg), EZC_OK);
    ASSERT_EQ(api::set_derivative_filter(&ctx, T(1)), EZC_OK);

    /* The first step only records the error. */
    EXPECT_EQ(api::step(&ctx, T(0), T(0)), T(0));
    /* a = dt / (Tf + dt) = 0.5 at dt = Tf = 1 s; the raw derivative is 1. */
    EXPECT_EQ(api::step(&ctx, T(1), T(0)), T(0.5));
    EXPECT_EQ(ctx.pid.derivative, T(0.5));
    EXPECT_EQ(api::step(&ctx, T(1), T(0)), T(0.25));
    EXPECT_EQ(api::step(&ctx, T(1), T(0)), T(0.125));
}

TYPED_TEST(PidTest, DisabledDerivativeFilterPassesTheRawDerivative) {
    using api = ezc_test::pid_api<TypeParam>;
    using T = TypeParam;

    typename api::context_type ctx{};
    typename api::config_type cfg = api::make_config();
    cfg.kd = T(1);
    ASSERT_EQ(api::init(&ctx, &cfg), EZC_OK);
    ASSERT_EQ(api::set_derivative_filter(&ctx, T(1)), EZC_OK);
    ASSERT_EQ(api::step(&ctx, T(0), T(0)), T(0));
    ASSERT_EQ(api::step(&ctx, T(1), T(0)), T(0.5));

    /* Disabling the filter keeps the stored term; the next step is raw. */
    ASSERT_EQ(api::set_derivative_filter(&ctx, T(0)), EZC_OK);
    EXPECT_EQ(ctx.pid.derivative, T(0.5));
    EXPECT_EQ(api::step(&ctx, T(1), T(0)), T(0));
    EXPECT_EQ(api::step(&ctx, T(3), T(0)), T(2));

    /* Enabling it again filters from the current derivative term. */
    ASSERT_EQ(api::set_derivative_filter(&ctx, T(1)), EZC_OK);
    EXPECT_EQ(api::step(&ctx, T(3), T(0)), T(1));
}

TYPED_TEST(PidTest, IntegralLimitHoldsTheIntegralState) {
    using api = ezc_test::pid_api<TypeParam>;
    using T = TypeParam;

    typename api::context_type ctx{};
    typename api::config_type cfg = api::make_config();
    cfg.ki = T(1);
    ASSERT_EQ(api::init(&ctx, &cfg), EZC_OK);
    ASSERT_EQ(api::set_integral_limit(&ctx, T(-2), T(2)), EZC_OK);

    EXPECT_EQ(api::step(&ctx, T(1), T(0)), T(1));
    EXPECT_EQ(api::step(&ctx, T(1), T(0)), T(2));
    EXPECT_EQ(api::step(&ctx, T(1), T(0)), T(2));
    EXPECT_EQ(ctx.pid.integral, T(2));

    /* The clamped state also recovers through the lower limit. */
    EXPECT_EQ(api::step(&ctx, T(-5), T(0)), T(-2));
    EXPECT_EQ(ctx.pid.integral, T(-2));

    ASSERT_EQ(api::clear_integral_limit(&ctx), EZC_OK);
    EXPECT_EQ(api::step(&ctx, T(-5), T(0)), T(-7));
}

TYPED_TEST(PidTest, IntegralLimitClampsAnExistingState) {
    using api = ezc_test::pid_api<TypeParam>;
    using T = TypeParam;

    typename api::context_type ctx{};
    typename api::config_type cfg = api::make_config();
    cfg.ki = T(1);
    ASSERT_EQ(api::init(&ctx, &cfg), EZC_OK);
    ASSERT_EQ(api::step(&ctx, T(1), T(0)), T(1));
    ASSERT_EQ(api::step(&ctx, T(1), T(0)), T(2));
    ASSERT_EQ(ctx.pid.integral, T(2));

    /* Enabling the limit at run time pulls a state outside the range in. */
    ASSERT_EQ(api::set_integral_limit(&ctx, T(-1), T(1)), EZC_OK);
    EXPECT_EQ(ctx.pid.integral, T(1));
    EXPECT_EQ(api::step(&ctx, T(1), T(0)), T(1));
}

TYPED_TEST(PidTest, IntegralLimitReplacesTheOutputRangeAsIntegralBounds) {
    using api = ezc_test::pid_api<TypeParam>;
    using T = TypeParam;

    typename api::context_type ctx{};
    typename api::config_type cfg = api::make_config();
    cfg.ki = T(1);
    cfg.limit_enabled = true;
    cfg.out_min = T(-10);
    cfg.out_max = T(10);
    ASSERT_EQ(api::init(&ctx, &cfg), EZC_OK);
    ASSERT_EQ(api::set_integral_limit(&ctx, T(-1), T(1)), EZC_OK);

    EXPECT_EQ(api::step(&ctx, T(5), T(0)), T(1));
    EXPECT_EQ(ctx.pid.integral, T(1));

    /* Without it the output range bounds the integral again. */
    ASSERT_EQ(api::clear_integral_limit(&ctx), EZC_OK);
    EXPECT_EQ(api::step(&ctx, T(5), T(0)), T(6));
    EXPECT_EQ(api::step(&ctx, T(5), T(0)), T(10));
    EXPECT_EQ(ctx.pid.integral, T(10));
}

TYPED_TEST(PidTest, IntegralSeparationSuspendsIntegrationAboveThreshold) {
    using api = ezc_test::pid_api<TypeParam>;
    using T = TypeParam;

    typename api::context_type ctx{};
    typename api::config_type cfg = api::make_config();
    cfg.ki = T(1);
    ASSERT_EQ(api::init(&ctx, &cfg), EZC_OK);
    ASSERT_EQ(api::set_integral_separation(&ctx, T(5)), EZC_OK);

    /* A large error leaves the integral untouched. */
    EXPECT_EQ(api::step(&ctx, T(10), T(0)), T(0));
    EXPECT_EQ(api::step(&ctx, T(10), T(0)), T(0));
    EXPECT_EQ(ctx.pid.integral, T(0));

    /* Inside the threshold the integral accumulates again. */
    EXPECT_EQ(api::step(&ctx, T(4), T(0)), T(4));
    EXPECT_EQ(api::step(&ctx, T(1), T(0)), T(5));

    /* And it is suspended again when the error grows. */
    EXPECT_EQ(api::step(&ctx, T(10), T(0)), T(5));
    EXPECT_EQ(ctx.pid.integral, T(5));

    ASSERT_EQ(api::clear_integral_separation(&ctx), EZC_OK);
    EXPECT_EQ(api::step(&ctx, T(10), T(0)), T(15));
}

TYPED_TEST(PidTest, SetpointWeightShapesTheProportionalTerm) {
    using api = ezc_test::pid_api<TypeParam>;
    using T = TypeParam;

    typename api::context_type ctx{};
    typename api::config_type cfg = api::make_config();
    cfg.kp = T(10);
    ASSERT_EQ(api::init(&ctx, &cfg), EZC_OK);
    ASSERT_EQ(api::set_setpoint_weight(&ctx, T(0)), EZC_OK);

    /* Proportional on the measurement only: setpoint steps cannot kick it. */
    EXPECT_EQ(api::step(&ctx, T(1), T(0)), T(0));
    EXPECT_EQ(api::step(&ctx, T(100), T(0)), T(0));
    EXPECT_EQ(api::step(&ctx, T(100), T(1)), T(-10));

    /* A partial weight scales the setpoint contribution. */
    ASSERT_EQ(api::set_setpoint_weight(&ctx, T(0.5)), EZC_OK);
    EXPECT_EQ(api::step(&ctx, T(100), T(1)), T(490));

    /* Weight 1 restores the default term. */
    ASSERT_EQ(api::set_setpoint_weight(&ctx, T(1)), EZC_OK);
    EXPECT_EQ(api::step(&ctx, T(100), T(1)), T(990));
}

TYPED_TEST(PidTest, OptionsFromConfigurationAreAppliedByInit) {
    using api = ezc_test::pid_api<TypeParam>;
    using T = TypeParam;

    typename api::context_type ctx{};
    typename api::config_type cfg = api::make_config();
    cfg.kp = T(2);
    cfg.ki = T(1);
    cfg.options = EZC_PID_OPT_SETPOINT_WEIGHT | EZC_PID_OPT_INTEGRAL_SEPARATION |
                  EZC_PID_OPT_INTEGRAL_LIMIT;
    cfg.weight_p = T(0.5);
    cfg.integral_separation = T(5);
    cfg.integral_min = T(-1);
    cfg.integral_max = T(1);
    cfg.deriv_source = EZC_PID_DERIV_ON_MEASUREMENT;
    cfg.deriv_filter_t = T(0.25);

    ASSERT_EQ(api::init(&ctx, &cfg), EZC_OK);
    EXPECT_EQ(ctx.cfg.options, cfg.options);
    EXPECT_EQ(ctx.cfg.weight_p, T(0.5));
    EXPECT_EQ(ctx.cfg.integral_separation, T(5));
    EXPECT_EQ(ctx.cfg.integral_min, T(-1));
    EXPECT_EQ(ctx.cfg.integral_max, T(1));
    EXPECT_EQ(ctx.cfg.deriv_source, EZC_PID_DERIV_ON_MEASUREMENT);
    EXPECT_EQ(ctx.cfg.deriv_filter_t, T(0.25));

    /* The options reach fieldctrl unchanged. */
    EXPECT_EQ(ctx.pid.config.options, cfg.options);
    EXPECT_EQ(static_cast<int>(ctx.pid.config.derivative),
              static_cast<int>(FIELDCTRL_PID_DERIVATIVE_ON_MEASUREMENT));
    EXPECT_EQ(ctx.pid.config.setpoint_weight, T(0.5));
    EXPECT_EQ(ctx.pid.config.separation, T(5));
    EXPECT_EQ(ctx.pid.config.integral_lower, T(-1));
    EXPECT_EQ(ctx.pid.config.integral_upper, T(1));
    EXPECT_EQ(ctx.pid.config.derivative_filter, T(0.25));

    /* P = 2 * (0.5 * 10 - 0) = 10, I suspended by the separation threshold. */
    EXPECT_EQ(api::step(&ctx, T(10), T(0)), T(10));
    EXPECT_EQ(ctx.pid.integral, T(0));
}

TYPED_TEST(PidTest, InvalidOptionParametersAreRejected) {
    using api = ezc_test::pid_api<TypeParam>;
    using T = TypeParam;
    const T nan = std::numeric_limits<T>::quiet_NaN();

    typename api::context_type ctx{};
    typename api::config_type cfg = api::make_config();
    cfg.kp = T(1);
    ASSERT_EQ(api::init(&ctx, &cfg), EZC_OK);

    EXPECT_EQ(api::set_derivative_source(&ctx, static_cast<ezc_pid_deriv_source_t>(7)),
              EZC_ERR_INVALID_ARG);
    EXPECT_EQ(api::set_derivative_filter(&ctx, T(-1)), EZC_ERR_INVALID_ARG);
    EXPECT_EQ(api::set_derivative_filter(&ctx, nan), EZC_ERR_INVALID_ARG);
    EXPECT_EQ(api::set_integral_limit(&ctx, T(1), T(-1)), EZC_ERR_INVALID_ARG);
    EXPECT_EQ(api::set_integral_limit(&ctx, nan, T(1)), EZC_ERR_INVALID_ARG);
    EXPECT_EQ(api::set_integral_separation(&ctx, T(0)), EZC_ERR_INVALID_ARG);
    EXPECT_EQ(api::set_integral_separation(&ctx, T(-1)), EZC_ERR_INVALID_ARG);
    EXPECT_EQ(api::set_setpoint_weight(&ctx, T(-0.1)), EZC_ERR_INVALID_ARG);
    EXPECT_EQ(api::set_setpoint_weight(&ctx, T(1.5)), EZC_ERR_INVALID_ARG);
    EXPECT_EQ(api::set_setpoint_weight(&ctx, nan), EZC_ERR_INVALID_ARG);

    /* Rejected setters leave the configuration and the controller untouched. */
    EXPECT_EQ(ctx.cfg.options, 0u);
    EXPECT_EQ(ctx.cfg.deriv_source, EZC_PID_DERIV_ON_ERROR);
    EXPECT_EQ(ctx.cfg.deriv_filter_t, T(0));
    EXPECT_EQ(ctx.cfg.integral_separation, T(0));
    EXPECT_EQ(ctx.cfg.weight_p, T(0));
    EXPECT_EQ(ctx.pid.config.options, 0u);

    /* A rejected initialization zeroes the whole context. */
    cfg.options = EZC_PID_OPT_INTEGRAL_LIMIT;
    cfg.integral_min = T(1);
    cfg.integral_max = T(-1);
    EXPECT_EQ(api::init(&ctx, &cfg), EZC_ERR_INVALID_ARG);
    EXPECT_EQ(ctx.cfg.options, 0u);
    EXPECT_EQ(ctx.cfg.kp, T(0));

    cfg.integral_min = T(-1);
    cfg.integral_max = T(1);
    cfg.options = EZC_PID_OPT_INTEGRAL_SEPARATION;
    cfg.integral_separation = T(0);
    EXPECT_EQ(api::init(&ctx, &cfg), EZC_ERR_INVALID_ARG);

    cfg.options = EZC_PID_OPT_SETPOINT_WEIGHT;
    cfg.weight_p = T(2);
    EXPECT_EQ(api::init(&ctx, &cfg), EZC_ERR_INVALID_ARG);

    cfg.options = 0u;
    cfg.deriv_filter_t = T(-0.5);
    EXPECT_EQ(api::init(&ctx, &cfg), EZC_ERR_INVALID_ARG);

    cfg.deriv_filter_t = T(0);
    cfg.deriv_source = static_cast<ezc_pid_deriv_source_t>(7);
    EXPECT_EQ(api::init(&ctx, &cfg), EZC_ERR_INVALID_ARG);

    /* Bits outside EZC_PID_OPT_* are rejected as well. */
    cfg.deriv_source = EZC_PID_DERIV_ON_ERROR;
    cfg.options = 1u << 5;
    EXPECT_EQ(api::init(&ctx, &cfg), EZC_ERR_INVALID_ARG);

    /* Disabled options must still hold finite parameters. */
    cfg.options = 0u;
    cfg.integral_separation = nan;
    EXPECT_EQ(api::init(&ctx, &cfg), EZC_ERR_INVALID_ARG);
}

TYPED_TEST(PidTest, OptionSettersRejectNullContext) {
    using api = ezc_test::pid_api<TypeParam>;
    using T = TypeParam;

    EXPECT_EQ(api::set_derivative_source(nullptr, EZC_PID_DERIV_ON_MEASUREMENT),
              EZC_ERR_INVALID_ARG);
    EXPECT_EQ(api::set_derivative_filter(nullptr, T(1)), EZC_ERR_INVALID_ARG);
    EXPECT_EQ(api::set_integral_limit(nullptr, T(-1), T(1)), EZC_ERR_INVALID_ARG);
    EXPECT_EQ(api::clear_integral_limit(nullptr), EZC_ERR_INVALID_ARG);
    EXPECT_EQ(api::set_integral_separation(nullptr, T(1)), EZC_ERR_INVALID_ARG);
    EXPECT_EQ(api::clear_integral_separation(nullptr), EZC_ERR_INVALID_ARG);
    EXPECT_EQ(api::set_setpoint_weight(nullptr, T(1)), EZC_ERR_INVALID_ARG);
}

TYPED_TEST(PidTest, ResetClearsTheOptionStateButKeepsConfiguration) {
    using api = ezc_test::pid_api<TypeParam>;
    using T = TypeParam;

    typename api::context_type ctx{};
    typename api::config_type cfg = api::make_config();
    cfg.kd = T(1);
    cfg.options = EZC_PID_OPT_INTEGRAL_LIMIT;
    cfg.integral_min = T(-1);
    cfg.integral_max = T(1);
    ASSERT_EQ(api::init(&ctx, &cfg), EZC_OK);
    ASSERT_EQ(api::set_derivative_filter(&ctx, T(1)), EZC_OK);

    ASSERT_EQ(api::step(&ctx, T(0), T(0)), T(0));
    ASSERT_EQ(api::step(&ctx, T(1), T(0)), T(0.5));
    ASSERT_EQ(ctx.pid.derivative, T(0.5));

    api::reset(&ctx);

    EXPECT_EQ(ctx.pid.integral, T(0));
    EXPECT_EQ(ctx.pid.previous_error, T(0));
    EXPECT_EQ(ctx.pid.previous_measurement, T(0));
    EXPECT_EQ(ctx.pid.derivative, T(0));
    EXPECT_EQ(ctx.pid.history, 0);
    EXPECT_EQ(ctx.cfg.deriv_filter_t, T(1));
    EXPECT_EQ(ctx.cfg.options, cfg.options);
    EXPECT_EQ(ctx.cfg.integral_max, T(1));
    EXPECT_EQ(ctx.pid.config.derivative_filter, T(1));
}
