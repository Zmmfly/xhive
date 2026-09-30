/*
 * Tests of the header-only C++ wrapper layer (ezctrl.hpp -> ezctrl/pid.hpp).
 *
 * The wrapper must forward every entry point to the typed C API without adding
 * state of its own. Both scalar specializations run the same checks through a
 * shared helper.
 */

#include "ezctrl.hpp"

#include "pid_test_types.h"

namespace {

template <typename T>
const char *scalar_name();

template <>
const char *scalar_name<float>() {
    return "float";
}

template <>
const char *scalar_name<double>() {
    return "double";
}

/* Exercises the read-only context accessor. */
template <typename T>
const typename ezc::pid<T>::context_type &as_const(const ezc::pid<T> &controller) {
    return controller.context();
}

/* Records what the wrapper callback receives. */
template <typename T>
struct record {
    int calls = 0;
    T output = T(0);
    bool limited = false;
    void *user = nullptr;
};

/* One wrapper update step with a one-second sample period. */
template <typename T>
T step(ezc::pid<T> &pid, T setpoint, T measure) {
    return pid.update(setpoint, measure, T(1));
}

template <typename T>
void check_configured_flow() {
    using controller = ezc::pid<T>;
    using config = typename controller::config_type;
    const T eps = ezc_test::pid_epsilon<T>();
    SCOPED_TRACE(scalar_name<T>());

    controller pid;

    /* A default-constructed wrapper is a disabled controller. */
    EXPECT_EQ(step(pid, T(1), T(0)), T(0));
    EXPECT_EQ(pid.context().kp, T(0));
    EXPECT_EQ(as_const(pid).ki, T(0));

    config cfg{};
    cfg.kp = T(1);
    cfg.ki = T(0.5);
    cfg.kd = T(0.25);
    ASSERT_EQ(pid.init(cfg), EZC_OK);

    /* P = 1, I = 0.5, D = 0.25 */
    EXPECT_NEAR(step(pid, T(1), T(0)), T(1.75), eps);
    /* P = 1, I = 1.0, D = 0 */
    EXPECT_NEAR(step(pid, T(1), T(0)), T(2), eps);
    EXPECT_NEAR(pid.context().integral, T(1), eps);

    /* Runtime gain changes and reset keep their C semantics. */
    ASSERT_EQ(pid.set_params(T(0), T(1), T(0)), EZC_OK);
    EXPECT_EQ(step(pid, T(1), T(0)), T(2));
    pid.reset();
    EXPECT_EQ(pid.context().integral, T(0));
    EXPECT_EQ(pid.context().prev_error, T(0));
    EXPECT_EQ(step(pid, T(1), T(0)), T(1));

    /* A rejected configuration leaves a zeroed, still usable context. */
    config bad{};
    bad.kp = T(5);
    bad.limit_enabled = true;
    bad.out_min = T(1);
    bad.out_max = T(-1);
    EXPECT_EQ(pid.init(bad), EZC_ERR_INVALID_ARG);
    EXPECT_EQ(step(pid, T(1), T(0)), T(0));

    /* The setters report rejected ranges without changing the context. */
    ASSERT_EQ(pid.limit(T(-1), T(1)), EZC_OK);
    EXPECT_EQ(pid.limit(T(1), T(-1)), EZC_ERR_INVALID_ARG);
    EXPECT_TRUE(as_const(pid).limit_enabled);
    EXPECT_EQ(as_const(pid).out_max, T(1));
}

template <typename T>
void check_limit_and_windup_flow() {
    using controller = ezc::pid<T>;
    using config = typename controller::config_type;
    using result_type = typename controller::result_type;
    const T eps = ezc_test::pid_epsilon<T>();
    SCOPED_TRACE(scalar_name<T>());

    controller pid;
    record<T> sink;

    config cfg{};
    cfg.ki = T(1);
    cfg.limit_enabled = true;
    cfg.out_min = T(-10);
    cfg.out_max = T(10);
    cfg.anti_windup_enabled = true;
    ASSERT_EQ(pid.init(cfg), EZC_OK);

    /* A captureless lambda converts to the C callback type. */
    auto callback = [](void *user_ctx, const result_type *result) {
        auto *target = static_cast<record<T> *>(user_ctx);
        target->calls += 1;
        target->output = result->output;
        target->limited = result->limited;
        target->user = user_ctx;
    };
    ASSERT_EQ(pid.set_callback(callback, &sink), EZC_OK);

    const T saturated = step(pid, T(1000), T(0));
    EXPECT_NEAR(saturated, T(10), eps);
    EXPECT_NEAR(pid.context().integral, T(10), eps);
    EXPECT_EQ(sink.calls, 1);
    EXPECT_EQ(sink.user, static_cast<void *>(&sink));
    EXPECT_EQ(sink.output, saturated);
    EXPECT_TRUE(sink.limited);

    /* Without a limit the whole raw output is returned, still with callback. */
    ASSERT_EQ(pid.clear_limit(), EZC_OK);
    const T unclamped = step(pid, T(0), T(0));
    EXPECT_NEAR(unclamped, T(10), eps);
    EXPECT_EQ(sink.calls, 2);
    EXPECT_EQ(sink.output, unclamped);
    EXPECT_FALSE(sink.limited);

    /* Anti-windup can be disabled again without touching the state. */
    ASSERT_EQ(pid.anti_windup(false), EZC_OK);
    EXPECT_FALSE(as_const(pid).anti_windup_enabled);
    ASSERT_EQ(pid.limit(T(-10), T(10)), EZC_OK);
    EXPECT_NEAR(step(pid, T(100), T(0)), T(10), eps);
    EXPECT_NEAR(pid.context().integral, T(110), eps);

    /* Removing the callback stops delivery without stopping the computation. */
    ASSERT_EQ(pid.set_callback(nullptr), EZC_OK);
    ASSERT_EQ(pid.clear_limit(), EZC_OK);
    const int calls = sink.calls;
    EXPECT_NEAR(step(pid, T(0), T(0)), T(110), eps);
    EXPECT_EQ(sink.calls, calls);
}

template <typename T>
void check_sample_period_flow() {
    using controller = ezc::pid<T>;
    using config = typename controller::config_type;
    const T eps = ezc_test::pid_epsilon<T>();
    SCOPED_TRACE(scalar_name<T>());

    controller pid;
    config cfg{};
    cfg.kp = T(1);
    cfg.ki = T(1);
    ASSERT_EQ(pid.init(cfg), EZC_OK);

    /* The wrapper passes a varying sample period straight through. */
    EXPECT_NEAR(pid.update(T(1), T(0), T(0.5)), T(1.5), eps);
    EXPECT_NEAR(pid.update(T(1), T(0), T(1)), T(2.5), eps);
    EXPECT_NEAR(pid.context().integral, T(1.5), eps);

    /* An invalid period is the documented zero-time step. */
    EXPECT_NEAR(pid.update(T(2), T(0), T(0)), T(3.5), eps);
    EXPECT_NEAR(pid.context().integral, T(1.5), eps);
    EXPECT_EQ(pid.context().prev_error, T(2));
}

/* The default-constructed wrapper needs neither init() nor a C structure. */
template <typename T>
void check_setter_only_flow() {
    using controller = ezc::pid<T>;
    const T eps = ezc_test::pid_epsilon<T>();
    SCOPED_TRACE(scalar_name<T>());

    controller ctrl;
    ASSERT_EQ(ctrl.set_params(T(2), T(1), T(0)), EZC_OK);
    EXPECT_EQ(ctrl.context().kp, T(2));
    EXPECT_EQ(ctrl.context().ki, T(1));
    EXPECT_EQ(ctrl.context().kd, T(0));

    /* P = 2, I = 1 at dt = 1 s. */
    EXPECT_NEAR(ctrl.update(T(1), T(0), T(1)), T(3), eps);
}

/* Every setter enables what it configures and clears just its own bit. */
template <typename T>
void check_setters_enable_their_features() {
    using controller = ezc::pid<T>;
    const T eps = ezc_test::pid_epsilon<T>();
    SCOPED_TRACE(scalar_name<T>());

    controller ctrl;
    ASSERT_EQ(ctrl.set_params(T(0), T(1), T(0)), EZC_OK);
    EXPECT_EQ(ctrl.context().options, 0u);

    ASSERT_EQ(ctrl.limit(T(-2), T(2)), EZC_OK);
    EXPECT_TRUE(ctrl.context().limit_enabled);
    ASSERT_EQ(ctrl.anti_windup(true), EZC_OK);
    EXPECT_TRUE(ctrl.context().anti_windup_enabled);
    ASSERT_EQ(ctrl.integral_limit(T(-1), T(1)), EZC_OK);
    EXPECT_NE(ctrl.context().options & EZC_PID_OPT_INTEGRAL_LIMIT, 0u);
    ASSERT_EQ(ctrl.integral_separation(T(5)), EZC_OK);
    EXPECT_NE(ctrl.context().options & EZC_PID_OPT_INTEGRAL_SEPARATION, 0u);
    ASSERT_EQ(ctrl.setpoint_weight(T(0)), EZC_OK);
    EXPECT_NE(ctrl.context().options & EZC_PID_OPT_SETPOINT_WEIGHT, 0u);

    /* The features are active: P = 0, I clamps at 1 while |e| = 4 <= 5. */
    EXPECT_NEAR(ctrl.update(T(4), T(0), T(1)), T(1), eps);
    EXPECT_NEAR(ctrl.context().integral, T(1), eps);
    EXPECT_NEAR(ctrl.update(T(4), T(0), T(1)), T(1), eps);
    EXPECT_NEAR(ctrl.context().integral, T(1), eps);

    /* Clearing removes the option bit and keeps the values. */
    ASSERT_EQ(ctrl.clear_integral_limit(), EZC_OK);
    EXPECT_EQ(ctrl.context().options & EZC_PID_OPT_INTEGRAL_LIMIT, 0u);
    EXPECT_EQ(ctrl.context().integral_max, T(1));
}

} /* namespace */

TEST(PidCppTest, FloatWrapperMatchesTheCApi) {
    check_configured_flow<float>();
}

TEST(PidCppTest, DoubleWrapperMatchesTheCApi) {
    check_configured_flow<double>();
}

TEST(PidCppTest, FloatWrapperLimitAndWindupMatchTheCApi) {
    check_limit_and_windup_flow<float>();
}

TEST(PidCppTest, DoubleWrapperLimitAndWindupMatchTheCApi) {
    check_limit_and_windup_flow<double>();
}

TEST(PidCppTest, FloatWrapperFollowsTheSamplePeriodRules) {
    check_sample_period_flow<float>();
}

TEST(PidCppTest, DoubleWrapperFollowsTheSamplePeriodRules) {
    check_sample_period_flow<double>();
}

TEST(PidCppTest, FloatSetterOnlyConfigurationWorks) {
    check_setter_only_flow<float>();
}

TEST(PidCppTest, DoubleSetterOnlyConfigurationWorks) {
    check_setter_only_flow<double>();
}

TEST(PidCppTest, FloatSettersEnableTheirFeatures) {
    check_setters_enable_their_features<float>();
}

TEST(PidCppTest, DoubleSettersEnableTheirFeatures) {
    check_setters_enable_their_features<double>();
}
