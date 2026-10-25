#include "support.h"
#include <array>
#include <random>
#include <utility>

namespace {
template<class T> class PiPwm : public ::testing::Test {};
TYPED_TEST_SUITE(PiPwm, fc_test::Types,);

/* Tuning with every option off; tests then enable only what they exercise. */
template<class A> typename A::pid_config pid_tuning(double kp, double ki, double kd,
                                                     double lower, double upper) {
    typename A::pid_config config{};
    config.kp=A::gain(kp);config.ki=A::gain(ki);config.kd=A::gain(kd);
    config.lower=A::make(lower);config.upper=A::make(upper);
    return config;
}

/* The former error-input PI: setpoint=error, measurement=0. */
template<class A> linears_status_t pi_step(typename A::pid *state, typename A::scalar error,
                                           typename A::scalar dt, typename A::scalar *out) {
    typename A::pid_output result{};
    const linears_status_t status=A::pid_step(state,error,A::make(0),dt,&result);
    if(status==LINEARS_OK) *out=result.output;
    return status;
}

TYPED_TEST(PiPwm, BoundaryAntiWindupStopsWindupAndRecovers) {
    using A=fc_test::Api<TypeParam>;
    typename A::pid state{};
    const double bound=A::fixed?0.5:4,drive=A::fixed?0.75:10,recover=A::fixed?-0.125:-1;
    const auto config=pid_tuning<A>(2,3,0,-bound,bound);
    ASSERT_EQ(A::pid_init(&state,&config,A::make(0)),LINEARS_OK);
    auto out=A::make(0.25),dt=A::make(0.01);
    for(int n=0;n<1000;++n) ASSERT_EQ(pi_step<A>(&state,A::make(drive),dt,&out),LINEARS_OK);
    EXPECT_DOUBLE_EQ(fc_test::value(out),bound);
    EXPECT_DOUBLE_EQ(fc_test::value(state.integral),0);
    ASSERT_EQ(pi_step<A>(&state,A::make(recover),dt,&out),LINEARS_OK);
    const double increment=3*fc_test::value(dt)*fc_test::value(A::make(recover));
    EXPECT_NEAR(fc_test::value(out),2*recover+increment,4*A::unit);
    EXPECT_NEAR(fc_test::value(state.integral),increment,3*A::unit);
    for(int n=0;n<1000;++n) ASSERT_EQ(pi_step<A>(&state,A::make(-drive),dt,&out),LINEARS_OK);
    EXPECT_DOUBLE_EQ(fc_test::value(out),-bound);
    EXPECT_NEAR(fc_test::value(state.integral),increment,3*A::unit);
}

TYPED_TEST(PiPwm, RetainsSubQ15IntegralIncrements) {
    using A=fc_test::Api<TypeParam>;
    typename A::pid state{};
    if constexpr(A::fixed) {
        // The increment 2^-19 is 1/16 of a Q15 LSB but an exact F31 value.
        const auto config=pid_tuning<A>(0,1,0,-0.5,0.5);
        ASSERT_EQ(A::pid_init(&state,&config,A::make(0.25)),LINEARS_OK);
        auto out=A::make(0),dt=A::make(1.0/16384),error=A::make(1.0/32);
        for(int n=0;n<20000;++n) ASSERT_EQ(pi_step<A>(&state,error,dt,&out),LINEARS_OK);
        const double target=0.25+20000.0/524288;
        EXPECT_DOUBLE_EQ(fc_test::value(state.integral),target);
        EXPECT_EQ(out,A::make(target));
    } else {
        const auto config=pid_tuning<A>(0,1,0,-2,2);
        ASSERT_EQ(A::pid_init(&state,&config,A::make(1)),LINEARS_OK);
        auto out=A::make(0),dt=A::make(0.00001);
        for(int n=0;n<20000;++n) ASSERT_EQ(pi_step<A>(&state,A::make(1),dt,&out),LINEARS_OK);
        const double target=1+20000*fc_test::value(dt);
        const double accumulation=std::is_same_v<TypeParam,fc_test::F32>?20000*std::ldexp(1.0,-24):20000*std::ldexp(1.0,-53);
        EXPECT_NEAR(fc_test::value(state.integral),target,accumulation);
        EXPECT_NEAR(fc_test::value(out),target,accumulation+2*A::unit);
        EXPECT_GT(fc_test::value(state.integral),1.19);
    }
}

TYPED_TEST(PiPwm, SoftwareRLCurrentLoopTracksBothDirectionsWithBusSaturation) {
    using A=fc_test::Api<TypeParam>;
    typename A::pid state{};
    // Q retains the original physical RL plant. Bases are caller-side only.
    const double ibase=A::fixed?64:1,vbase=A::fixed?64:1,tbase=A::fixed?0.01:1;
    const auto config=pid_tuning<A>(3*ibase/vbase,700*ibase*tbase/vbase,0,-12/vbase,12/vbase);
    ASSERT_EQ(A::pid_init(&state,&config,A::make(0)),LINEARS_OK);
    const auto dt=A::make(0.0001/tbase);
    const double h=fc_test::value(dt)*tbase,resistance=0.7,inductance=0.003;
    double current=0;
    auto output=A::make(0);
    int saturated=0;
    for(int n=0;n<12000;++n) {
        const double ref=n<3000?5:n<6000?-5:n<9000?30:2;
        const auto error=A::make((ref-current)/ibase);
        ASSERT_EQ(pi_step<A>(&state,error,dt,&output),LINEARS_OK);
        const double applied=fc_test::value(output)*vbase;
        EXPECT_LE(applied,12);EXPECT_GE(applied,-12);
        if(std::abs(applied)==12)++saturated;
        current+=h*(applied-resistance*current)/inductance;
        if(n==2999||n==5999||n==11999) { EXPECT_NEAR(current,ref,0.002); }
        if(n==8999) { EXPECT_NEAR(current,12/resistance,0.002); }
    }
    EXPECT_GT(saturated,1000);
}

TYPED_TEST(PiPwm, VoltageLimitAndPwmMatchIndependentCommonModeFormula) {
    using A=fc_test::Api<TypeParam>;
    std::mt19937 rng(0x50574d);
    std::uniform_real_distribution<double> components(A::fixed?-0.8:-60,A::fixed?0.8:60);
    std::uniform_real_distribution<double> buses(A::fixed?0.125:2,A::fixed?0.95:48);
    for(int n=0;n<600;++n) {
        typename A::ab in{A::make(components(rng)),A::make(components(rng))},limited{};
        typename A::abc duty{};
        const auto bus=A::make(buses(rng)),limit=A::make(A::fixed?0.4:9);
        ASSERT_EQ(A::limit_voltage(in,limit,&limited),LINEARS_OK);
        double a=fc_test::value(in.alpha),b=fc_test::value(in.beta);
        const double m=std::hypot(a,b),radius=fc_test::value(limit),scale=m>radius?radius/m:1;
        EXPECT_NEAR(fc_test::value(limited.alpha),a*scale,20*A::unit);
        EXPECT_NEAR(fc_test::value(limited.beta),b*scale,20*A::unit);
        ASSERT_EQ(A::svpwm(in,bus,&duty),LINEARS_OK);
        const double vdc=fc_test::value(bus),bound=vdc/std::sqrt(3.0),s=m>bound?bound/m:1;
        a*=s;b*=s;
        const double phases[3]={a,(-a+std::sqrt(3.0)*b)/2,(-a-std::sqrt(3.0)*b)/2};
        const double offset=(*std::min_element(phases,phases+3)+*std::max_element(phases,phases+3))/2;
        const double actual[3]={fc_test::value(duty.a),fc_test::value(duty.b),fc_test::value(duty.c)};
        for(int k=0;k<3;++k) {
            EXPECT_NEAR(actual[k],0.5+(phases[k]-offset)/vdc,8*A::unit);
            EXPECT_GE(actual[k],0);EXPECT_LE(actual[k],1);
        }
        EXPECT_NEAR((actual[0]-actual[1])*vdc,phases[0]-phases[1],16*A::unit*vdc);
    }
}

TYPED_TEST(PiPwm, ZeroVoltageLargeInputsAndIndependentContexts) {
    using A=fc_test::Api<TypeParam>;
    typename A::abc duty{};
    typename A::ab zero{A::make(0),A::make(0)},large{A::make(A::fixed?1:10000),A::make(A::fixed?-1:-10000)},limited{};
    const auto bus=A::make(A::fixed?0.75:12);
    ASSERT_EQ(A::svpwm(zero,bus,&duty),LINEARS_OK);
    EXPECT_DOUBLE_EQ(fc_test::value(duty.a),0.5);EXPECT_DOUBLE_EQ(fc_test::value(duty.b),0.5);
    ASSERT_EQ(A::limit_voltage(large,A::make(0),&limited),LINEARS_OK);
    EXPECT_DOUBLE_EQ(fc_test::value(limited.alpha),0);EXPECT_DOUBLE_EQ(fc_test::value(limited.beta),0);
    ASSERT_EQ(A::svpwm(large,bus,&duty),LINEARS_OK);
    EXPECT_LE(fc_test::value(duty.a),1);EXPECT_GE(fc_test::value(duty.b),0);
    typename A::pid one{},two{};
    const double bound=A::fixed?0.75:5,error=A::fixed?0.125:1;
    const auto config=pid_tuning<A>(1,2,0,-bound,bound);
    ASSERT_EQ(A::pid_init(&one,&config,A::make(0)),LINEARS_OK);
    ASSERT_EQ(A::pid_init(&two,&config,A::make(0)),LINEARS_OK);
    auto out=A::make(0),dt=A::make(0.01);
    for(int n=0;n<30;++n) ASSERT_EQ(pi_step<A>(&one,A::make(error),dt,&out),LINEARS_OK);
    EXPECT_DOUBLE_EQ(fc_test::value(two.integral),0);
    ASSERT_EQ(pi_step<A>(&two,A::make(-error),dt,&out),LINEARS_OK);
    EXPECT_LT(fc_test::value(two.integral),0);EXPECT_GT(fc_test::value(one.integral),0);
}

TYPED_TEST(PiPwm, FailureIsAtomicForParametersAliasAndNumericRange) {
    using A=fc_test::Api<TypeParam>;
    typename A::pid state{};
    auto config=pid_tuning<A>(1,1,0,-1,1);
    ASSERT_EQ(A::pid_init(&state,&config,A::make(0)),LINEARS_OK);
    auto saved=state;auto out=A::make(0.3),old=out;
    EXPECT_EQ(pi_step<A>(&state,A::make(1),A::make(0),&out),LINEARS_ERROR_DOMAIN);
    EXPECT_TRUE(fc_test::same_bytes(state,saved));EXPECT_TRUE(fc_test::same_bytes(out,old));
    typename A::pid_output result{};
    EXPECT_EQ(A::pid_step(&state,A::make(1),A::make(0),A::make(0.01),
        reinterpret_cast<typename A::pid_output*>(&state.integral)),LINEARS_ERROR_ALIAS);
    EXPECT_TRUE(fc_test::same_bytes(state,saved));
    EXPECT_EQ(A::pid_step(nullptr,A::make(1),A::make(0),A::make(0.01),&result),LINEARS_ERROR_ARGUMENT);
    EXPECT_EQ(A::pid_step(&state,A::make(1),A::make(0),A::make(0.01),nullptr),LINEARS_ERROR_ARGUMENT);
    config.kp=A::gain(-1);
    EXPECT_EQ(A::pid_init(&state,&config,A::make(0)),LINEARS_ERROR_DOMAIN);
    EXPECT_TRUE(fc_test::same_bytes(state,saved));
    config=pid_tuning<A>(1,1,0,1,1);
    EXPECT_EQ(A::pid_init(&state,&config,A::make(0)),LINEARS_ERROR_DOMAIN);
    typename A::abc duty{old,old,old},duty_saved=duty;
    typename A::ab voltage{A::make(A::fixed?0.25:1),A::make(A::fixed?0.5:2)};
    EXPECT_EQ(A::svpwm(voltage,A::make(0),&duty),LINEARS_ERROR_DOMAIN);
    EXPECT_TRUE(fc_test::same_bytes(duty,duty_saved));
    if constexpr(!A::fixed) {
        auto bad=std::numeric_limits<typename A::scalar>::infinity();
        EXPECT_EQ(pi_step<A>(&state,bad,A::make(0.01),&out),LINEARS_ERROR_RANGE);
        EXPECT_TRUE(fc_test::same_bytes(state,saved));EXPECT_TRUE(fc_test::same_bytes(out,old));
        EXPECT_EQ(A::svpwm({bad,A::make(0)},A::make(12),&duty),LINEARS_ERROR_RANGE);
        EXPECT_TRUE(fc_test::same_bytes(duty,duty_saved));
    } else {
        // There are no invalid Q bit patterns; invalid signs remain a domain error.
        EXPECT_EQ(pi_step<A>(&state,A::make(0.5),A::make(-0.25),&out),LINEARS_ERROR_DOMAIN);
        EXPECT_TRUE(fc_test::same_bytes(state,saved));EXPECT_TRUE(fc_test::same_bytes(out,old));
        EXPECT_EQ(A::svpwm(voltage,A::make(-1),&duty),LINEARS_ERROR_DOMAIN);
        EXPECT_TRUE(fc_test::same_bytes(duty,duty_saved));
    }
    EXPECT_EQ(A::limit_voltage(voltage,A::make(-1),&voltage),LINEARS_ERROR_DOMAIN);
}

TYPED_TEST(PiPwm, ArithmeticOverflowDoesNotCommitEvenWithFiniteInputs) {
    using A=fc_test::Api<TypeParam>;
    typename A::pid state{};
    typename A::pid_config config{};
    if constexpr(A::fixed) {
        config=pid_tuning<A>(0,0,0,-1,0.5);
        config.kp=INT64_MAX;
        ASSERT_EQ(A::pid_init(&state,&config,A::make(-1)),LINEARS_OK);
    } else {
        config=pid_tuning<A>(0,1,0,-1,1);
        config.kp=std::numeric_limits<typename A::scalar>::max()/2;
        ASSERT_EQ(A::pid_init(&state,&config,A::make(0)),LINEARS_OK);
    }
    auto saved=state;
    auto output=A::make(0.25),saved_output=output;
    // Q: (-INT64_MAX) + (-ONE) overflows the fixed working range.
    EXPECT_EQ(pi_step<A>(&state,A::make(A::fixed?-1:4),A::make(0.01),&output),LINEARS_ERROR_RANGE);
    EXPECT_TRUE(fc_test::same_bytes(state,saved));EXPECT_TRUE(fc_test::same_bytes(output,saved_output));
}

TYPED_TEST(PiPwm, DutyIsInvariantUnderExtremeCommonVoltageScaling) {
    using A=fc_test::Api<TypeParam>;
    for(int scale_index=0;scale_index<4;++scale_index) {
        auto scaled=[scale_index](double factor) {
            if constexpr(A::fixed) {
                const double values[]={A::unit,1.0/1024,1.0/32,1.0/8};
                return A::make(factor*values[scale_index]);
            } else {
                using S=typename A::scalar;
                const S values[]={std::numeric_limits<S>::denorm_min(),std::numeric_limits<S>::min(),S(1),std::numeric_limits<S>::max()/8};
                return static_cast<S>(factor)*values[scale_index];
            }
        };
        for(const auto &command : {std::array<double,3>{1,0,1},{2,1,4},{4,-2,1}}) {
            typename A::abc duty{};
            ASSERT_EQ(A::svpwm({scaled(command[0]),scaled(command[1])},scaled(command[2]),&duty),LINEARS_OK);
            double a=command[0]/command[2],b=command[1]/command[2];
            const double norm=std::hypot(a,b),limit=1/std::sqrt(3.0);
            if(norm>limit){a*=limit/norm;b*=limit/norm;}
            const double phases[]={a,(-a+std::sqrt(3.0)*b)/2,(-a-std::sqrt(3.0)*b)/2};
            const double offset=(*std::min_element(phases,phases+3)+*std::max_element(phases,phases+3))/2;
            EXPECT_NEAR(fc_test::value(duty.a),0.5+phases[0]-offset,8*A::unit);
            EXPECT_NEAR(fc_test::value(duty.b),0.5+phases[1]-offset,8*A::unit);
            EXPECT_NEAR(fc_test::value(duty.c),0.5+phases[2]-offset,8*A::unit);
        }
    }
}

TYPED_TEST(PiPwm, RadialRescalingPreservesBothVerySmallRadiusAndSmallComponent) {
    using A=fc_test::Api<TypeParam>;
    typename A::ab output{};
    if constexpr(A::fixed) {
        using S=typename A::scalar;
        const S largest=std::numeric_limits<S>::max(),lsb=1;
        ASSERT_EQ(A::limit_voltage({largest,S(0)},lsb,&output),LINEARS_OK);
        EXPECT_EQ(output.alpha,lsb);EXPECT_EQ(output.beta,0);
        ASSERT_EQ(A::limit_voltage({S(2),A::make(0.5)},A::make(0.25),&output),LINEARS_OK);
        EXPECT_EQ(output.alpha,lsb);EXPECT_EQ(output.beta,A::make(0.25));
    } else {
        constexpr int power=std::is_same_v<TypeParam,fc_test::F32>?80:600;
        const double tiny=std::ldexp(1.0,-power),huge=std::ldexp(1.0,power);
        ASSERT_EQ(A::limit_voltage({A::make(huge),A::make(0)},A::make(tiny),&output),LINEARS_OK);
        EXPECT_DOUBLE_EQ(fc_test::value(output.alpha),tiny);EXPECT_DOUBLE_EQ(fc_test::value(output.beta),0);
        ASSERT_EQ(A::limit_voltage({A::make(tiny),A::make(huge)},A::make(huge/2),&output),LINEARS_OK);
        EXPECT_DOUBLE_EQ(fc_test::value(output.alpha),tiny/2);EXPECT_DOUBLE_EQ(fc_test::value(output.beta),huge/2);
    }
}

TYPED_TEST(PiPwm, RawEndpointsAndUnrepresentableFloatingNormContract) {
    using A=fc_test::Api<TypeParam>;
    typename A::ab output{A::make(A::fixed?0.25:2),A::make(A::fixed?0.5:3)};
    if constexpr(A::fixed) {
        using S=typename A::scalar;
        const S lo=std::numeric_limits<S>::min(),hi=std::numeric_limits<S>::max();
        ASSERT_EQ(A::limit_voltage({lo,S(0)},hi,&output),LINEARS_OK);
        EXPECT_EQ(output.alpha,-hi);EXPECT_EQ(output.beta,0);
        ASSERT_EQ(A::limit_voltage({lo,hi},S(0),&output),LINEARS_OK);
        EXPECT_EQ(output.alpha,0);EXPECT_EQ(output.beta,0);
        typename A::abc duty{};
        ASSERT_EQ(A::svpwm({lo,hi},S(1),&duty),LINEARS_OK);
        EXPECT_GE(duty.a,0);EXPECT_GE(duty.b,0);EXPECT_GE(duty.c,0);
    } else {
        const auto large=std::numeric_limits<typename A::scalar>::max();
        auto saved=output;
        EXPECT_EQ(A::limit_voltage({large,large},A::make(1),&output),LINEARS_ERROR_RANGE);
        EXPECT_TRUE(fc_test::same_bytes(output,saved));
    }
}

template<class T> void check_raw_dynamic_range_hypot() {
    using A=fc_test::Api<T>;using S=typename A::scalar;
    const int64_t largest=std::numeric_limits<S>::max();
    for(int sa : {-1,1})for(int sb : {-1,1})for(int swap : {0,1}) {
        const S a=static_cast<S>(sa*(swap?1:largest));
        const S b=static_cast<S>(sb*(swap?largest:1));
        typename A::ab output{};
        ASSERT_EQ(A::limit_voltage({a,b},static_cast<S>(largest),&output),LINEARS_OK);
        EXPECT_EQ(output.alpha,a);EXPECT_EQ(output.beta,b);
    }
}
TEST(PiPwmIntegerRange,Q15RawDynamicRangeRetainsBothComponents) {check_raw_dynamic_range_hypot<fc_test::Q15>();}
TEST(PiPwmIntegerRange,Q31RawDynamicRangeRetainsBothComponents) {check_raw_dynamic_range_hypot<fc_test::Q31>();}

TEST(PiPwmIntegerRange,Q31HalfwayTiesAndSubWorkingLsbHaveNoHiddenRemainder) {
    fieldctrl_q31_pid_t state{};
    fieldctrl_q31_pid_config_t config{};
    fieldctrl_q31_pid_output_t output{};
    const int32_t quarter=INT32_C(536870912),half=INT32_C(1073741824);
    config.ki=LINEARS_FIXED_ONE;config.lower=-half;config.upper=half;
    ASSERT_EQ(fieldctrl_q31_pid_init(&state,&config,quarter),LINEARS_OK);
    for(int n=0;n<4096;++n)
        ASSERT_EQ(fieldctrl_q31_pid_step(&state,1,0,quarter,&output),LINEARS_OK);
    EXPECT_EQ(state.integral,quarter); // Each exact increment is 1/4 of one F31 LSB.
    for(int n=0;n<1024;++n)
        ASSERT_EQ(fieldctrl_q31_pid_step(&state,1,0,half,&output),LINEARS_OK);
    EXPECT_EQ(state.integral,INT64_C(536870912)+1024); // Halfway rounds away from zero.
    for(int n=0;n<1024;++n)
        ASSERT_EQ(fieldctrl_q31_pid_step(&state,-1,0,half,&output),LINEARS_OK);
    EXPECT_EQ(state.integral,quarter);EXPECT_EQ(output.output,quarter);
}

TEST(PiPwmIntegerRange,Q15SmallestErrorAndTimeAccumulateBeforeSignalQuantization) {
    fieldctrl_q15_pid_t state{};
    fieldctrl_q15_pid_config_t config{};
    fieldctrl_q15_pid_output_t output{};
    config.ki=LINEARS_FIXED_ONE;config.lower=-16384;config.upper=16384;
    ASSERT_EQ(fieldctrl_q15_pid_init(&state,&config,8192),LINEARS_OK);
    for(int n=0;n<16384;++n)
        ASSERT_EQ(fieldctrl_q15_pid_step(&state,1,0,1,&output),LINEARS_OK);
    EXPECT_EQ(state.integral,INT64_C(536870912)+32768); // Each update is two F31 raw units.
    EXPECT_EQ(output.output,8193); // Exactly half a positive Q15 output LSB rounds upward.
}

/* PID feature cases. Unless noted, values are exact binary fractions, so every
 * format must reproduce them exactly. */
template<class T> class Pid : public ::testing::Test {};
TYPED_TEST_SUITE(Pid, fc_test::Types,);

template<class A> typename A::pid_output pid_run(typename A::pid *state, double setpoint,
                                                  double measurement, double dt) {
    typename A::pid_output result{};
    EXPECT_EQ(A::pid_step(state,A::make(setpoint),A::make(measurement),A::make(dt),&result),
              LINEARS_OK);
    return result;
}

TYPED_TEST(Pid, DerivativeStartsWithoutKickAndFollowsTheSelectedSource) {
    using A=fc_test::Api<TypeParam>;
    for(const auto source : {FIELDCTRL_PID_DERIVATIVE_ON_ERROR, FIELDCTRL_PID_DERIVATIVE_ON_MEASUREMENT}) {
        auto config=pid_tuning<A>(0,0,0.0625,-1,1); // kd/dt = 0.0625/0.125 = 0.5
        config.derivative=source;
        typename A::pid state{};
        ASSERT_EQ(A::pid_init(&state,&config,A::make(0)),LINEARS_OK);
        // Nonzero initial error and measurement: the first step only records them.
        auto r=pid_run<A>(&state,0.25,0.125,0.125);
        EXPECT_DOUBLE_EQ(fc_test::value(r.derivative),0);
        EXPECT_DOUBLE_EQ(fc_test::value(r.output),0);
        EXPECT_NE(state.history,0);
        // A measurement rise of 0.125 is differentiated by both sources.
        r=pid_run<A>(&state,0.25,0.25,0.125);
        EXPECT_DOUBLE_EQ(fc_test::value(r.derivative),-0.0625);
        // A setpoint step of 0.25 kicks only the error source.
        r=pid_run<A>(&state,0.5,0.25,0.125);
        const double kick=source==FIELDCTRL_PID_DERIVATIVE_ON_ERROR?0.125:0;
        EXPECT_DOUBLE_EQ(fc_test::value(r.derivative),kick);
        EXPECT_DOUBLE_EQ(fc_test::value(r.output),kick);
        EXPECT_FALSE(r.saturated);
    }
}

TYPED_TEST(Pid, DerivativeFilterFollowsTheDiscreteLowPass) {
    using A=fc_test::Api<TypeParam>;
    typename A::pid state{};
    auto config=pid_tuning<A>(0,0,0.0625,-1,1);
    config.derivative_filter=A::gain(0.125); // dt/(Tf+dt) = 1/2
    ASSERT_EQ(A::pid_init(&state,&config,A::make(0)),LINEARS_OK);
    EXPECT_DOUBLE_EQ(fc_test::value(pid_run<A>(&state,0,0,0.125).derivative),0);
    EXPECT_DOUBLE_EQ(fc_test::value(state.filter_weight),0.5);
    // Raw derivative 0.5*0.5 = 0.25, then zero: the filtered term halves each step.
    EXPECT_DOUBLE_EQ(fc_test::value(pid_run<A>(&state,0.5,0,0.125).derivative),0.125);
    EXPECT_DOUBLE_EQ(fc_test::value(pid_run<A>(&state,0.5,0,0.125).derivative),0.0625);
    const auto r=pid_run<A>(&state,0.5,0,0.125);
    EXPECT_DOUBLE_EQ(fc_test::value(r.derivative),0.03125);
    EXPECT_DOUBLE_EQ(fc_test::value(r.output),0.03125);
}

TYPED_TEST(Pid, CachedCoefficientsFollowSamplePeriodChanges) {
    using A=fc_test::Api<TypeParam>;
    typename A::pid state{};
    const auto config=pid_tuning<A>(0,0.5,0.0625,-1,1);
    ASSERT_EQ(A::pid_init(&state,&config,A::make(0)),LINEARS_OK);
    EXPECT_DOUBLE_EQ(fc_test::value(state.cached_dt),0);
    pid_run<A>(&state,0,0,0.125);
    EXPECT_DOUBLE_EQ(fc_test::value(state.cached_dt),0.125);
    EXPECT_DOUBLE_EQ(fc_test::value(state.integral_step),0.0625);
    EXPECT_DOUBLE_EQ(fc_test::value(state.derivative_gain),0.5);
    auto r=pid_run<A>(&state,0.25,0,0.125);
    EXPECT_DOUBLE_EQ(fc_test::value(r.integral),0.015625);
    EXPECT_DOUBLE_EQ(fc_test::value(r.derivative),0.125);
    // A different dt replaces both cached coefficients before they are used.
    r=pid_run<A>(&state,0.5,0,0.25);
    EXPECT_DOUBLE_EQ(fc_test::value(state.cached_dt),0.25);
    EXPECT_DOUBLE_EQ(fc_test::value(r.integral),0.015625+0.0625);
    EXPECT_DOUBLE_EQ(fc_test::value(r.derivative),0.0625);
    EXPECT_DOUBLE_EQ(fc_test::value(r.output),0.015625+0.0625+0.0625);
}

TYPED_TEST(Pid, BoundaryAntiWindupAdvancesToTheSaturationBoundary) {
    using A=fc_test::Api<TypeParam>;
    for(const double sign : {1.0,-1.0}) {
        typename A::pid state{};
        const auto config=pid_tuning<A>(1,1,0,-0.5,0.5); // ki*dt = 0.125
        ASSERT_EQ(A::pid_init(&state,&config,A::make(0)),LINEARS_OK);
        auto r=pid_run<A>(&state,sign*0.375,0,0.125);
        EXPECT_DOUBLE_EQ(fc_test::value(r.output),sign*0.421875);
        r=pid_run<A>(&state,sign*0.375,0,0.125);
        EXPECT_DOUBLE_EQ(fc_test::value(r.output),sign*0.46875);
        EXPECT_FALSE(r.saturated);
        EXPECT_DOUBLE_EQ(fc_test::value(r.integral),sign*0.09375);
        // Candidate I = 0.140625 would ask for 0.515625. I stops where P+I meets
        // the bound (0.125); rejecting the whole increment would keep 0.09375.
        r=pid_run<A>(&state,sign*0.375,0,0.125);
        EXPECT_TRUE(r.saturated);
        EXPECT_DOUBLE_EQ(fc_test::value(r.output),sign*0.5);
        EXPECT_DOUBLE_EQ(fc_test::value(r.integral),sign*0.125);
        r=pid_run<A>(&state,sign*0.375,0,0.125);
        EXPECT_TRUE(r.saturated);
        EXPECT_DOUBLE_EQ(fc_test::value(state.integral),sign*0.125);
        // Reversing the error leaves saturation at once: no wound-up integral.
        r=pid_run<A>(&state,-sign*0.125,0,0.125);
        EXPECT_FALSE(r.saturated);
        EXPECT_DOUBLE_EQ(fc_test::value(r.integral),sign*0.109375);
        EXPECT_DOUBLE_EQ(fc_test::value(r.output),-sign*0.015625);
    }
}

TYPED_TEST(Pid, SetpointWeightShapesTheProportionalTerm) {
    using A=fc_test::Api<TypeParam>;
    const std::pair<double,double> cases[]={{0.0,-0.125},{0.5,0.0},{1.0,0.125}};
    for(const auto &[weight,expected] : cases) {
        typename A::pid state{};
        auto config=pid_tuning<A>(0.5,0,0,-1,1);
        config.options=FIELDCTRL_PID_OPT_SETPOINT_WEIGHT;
        config.setpoint_weight=A::gain(weight);
        ASSERT_EQ(A::pid_init(&state,&config,A::make(0)),LINEARS_OK);
        const auto r=pid_run<A>(&state,0.5,0.25,0.125); // P = 0.5*(b*0.5-0.25)
        EXPECT_DOUBLE_EQ(fc_test::value(r.proportional),expected);
        EXPECT_DOUBLE_EQ(fc_test::value(r.output),expected);
    }
    // Without the option the weight is ignored, even when out of range.
    typename A::pid state{};
    auto config=pid_tuning<A>(0.5,0,0,-1,1);
    config.setpoint_weight=A::gain(2);
    ASSERT_EQ(A::pid_init(&state,&config,A::make(0)),LINEARS_OK);
    EXPECT_DOUBLE_EQ(fc_test::value(pid_run<A>(&state,0.5,0.25,0.125).proportional),0.125);
}

TYPED_TEST(Pid, IntegralSeparationHoldsTheIntegralAboveTheThreshold) {
    using A=fc_test::Api<TypeParam>;
    typename A::pid state{};
    auto config=pid_tuning<A>(0,1,0,-0.75,0.75);
    config.options=FIELDCTRL_PID_OPT_INTEGRAL_SEPARATION;
    config.separation=A::make(0.25);
    ASSERT_EQ(A::pid_init(&state,&config,A::make(0)),LINEARS_OK);
    EXPECT_DOUBLE_EQ(fc_test::value(pid_run<A>(&state,0.5,0,0.125).integral),0);
    EXPECT_DOUBLE_EQ(fc_test::value(pid_run<A>(&state,0.25,0,0.125).integral),0.03125);
    EXPECT_DOUBLE_EQ(fc_test::value(pid_run<A>(&state,-0.5,0,0.125).integral),0.03125);
    EXPECT_DOUBLE_EQ(fc_test::value(pid_run<A>(&state,-0.25,0,0.125).integral),0);
}

TYPED_TEST(Pid, IntegralLimitReplacesTheOutputBoundsForTheIntegral) {
    using A=fc_test::Api<TypeParam>;
    typename A::pid state{};
    auto config=pid_tuning<A>(0,1,0,-0.5,0.5);
    config.options=FIELDCTRL_PID_OPT_INTEGRAL_LIMIT;
    config.integral_lower=A::make(-0.25);config.integral_upper=A::make(0.25);
    ASSERT_EQ(A::pid_init(&state,&config,A::make(0)),LINEARS_OK);
    for(int n=1;n<=6;++n) {
        const auto r=pid_run<A>(&state,0.5,0,0.125);
        EXPECT_DOUBLE_EQ(fc_test::value(r.integral),std::min(0.0625*n,0.25));
        EXPECT_FALSE(r.saturated);
    }
    EXPECT_DOUBLE_EQ(fc_test::value(pid_run<A>(&state,-0.5,0,0.125).integral),0.1875);
    // The independent range may be wider than the output range.
    config.integral_lower=A::make(-0.75);config.integral_upper=A::make(0.75);
    typename A::pid wide{};
    ASSERT_EQ(A::pid_init(&wide,&config,A::make(0.625)),LINEARS_OK);
    const auto r=pid_run<A>(&wide,0,0,0.125);
    EXPECT_TRUE(r.saturated);
    EXPECT_DOUBLE_EQ(fc_test::value(r.output),0.5);
    EXPECT_DOUBLE_EQ(fc_test::value(r.integral),0.625);
    config.options=0;
    EXPECT_EQ(A::pid_init(&wide,&config,A::make(0.625)),LINEARS_ERROR_DOMAIN);
}

TYPED_TEST(Pid, ResetClearsHistoryAndKeepsTuningAndCache) {
    using A=fc_test::Api<TypeParam>;
    typename A::pid state{};
    auto config=pid_tuning<A>(0,1,0.0625,-0.5,0.5);
    config.derivative_filter=A::gain(0.125);
    ASSERT_EQ(A::pid_init(&state,&config,A::make(0)),LINEARS_OK);
    pid_run<A>(&state,0,0,0.125);
    EXPECT_DOUBLE_EQ(fc_test::value(pid_run<A>(&state,0.5,0,0.125).derivative),0.125);
    ASSERT_EQ(A::pid_reset(&state,A::make(0.125)),LINEARS_OK);
    EXPECT_DOUBLE_EQ(fc_test::value(state.integral),0.125);
    EXPECT_DOUBLE_EQ(fc_test::value(state.derivative),0);
    EXPECT_EQ(state.history,0);
    EXPECT_EQ(state.config.kd,config.kd);
    EXPECT_EQ(state.config.derivative_filter,config.derivative_filter);
    EXPECT_DOUBLE_EQ(fc_test::value(state.cached_dt),0.125);
    // Like the first step after init: a nonzero error causes no derivative kick.
    const auto r=pid_run<A>(&state,0.5,0,0.125);
    EXPECT_DOUBLE_EQ(fc_test::value(r.derivative),0);
    EXPECT_DOUBLE_EQ(fc_test::value(r.output),0.1875);
    const auto saved=state;
    EXPECT_EQ(A::pid_reset(&state,A::make(0.75)),LINEARS_ERROR_DOMAIN);
    EXPECT_TRUE(fc_test::same_bytes(state,saved));
    EXPECT_EQ(A::pid_reset(nullptr,A::make(0)),LINEARS_ERROR_ARGUMENT);
    if constexpr(!A::fixed) {
        EXPECT_EQ(A::pid_reset(&state,std::numeric_limits<typename A::scalar>::infinity()),
                  LINEARS_ERROR_RANGE);
        EXPECT_TRUE(fc_test::same_bytes(state,saved));
    }
}

TYPED_TEST(Pid, InitValidatesEnabledOptionsAtomically) {
    using A=fc_test::Api<TypeParam>;
    typename A::pid state{};
    const auto base=pid_tuning<A>(0.5,0.5,0.0625,-0.5,0.5);
    ASSERT_EQ(A::pid_init(&state,&base,A::make(0)),LINEARS_OK);
    const auto saved=state;
    auto rejected=[&](const typename A::pid_config &config, double integral) {
        EXPECT_EQ(A::pid_init(&state,&config,A::make(integral)),LINEARS_ERROR_DOMAIN);
        EXPECT_TRUE(fc_test::same_bytes(state,saved));
    };
    auto config=base;config.kd=A::gain(-0.0625);rejected(config,0);
    config=base;config.derivative_filter=A::gain(-0.125);rejected(config,0);
    config=base;config.options=1u<<3;rejected(config,0);
    config=base;
    const unsigned unknown_source=7;
    static_assert(sizeof(config.derivative)==sizeof(unknown_source));
    std::memcpy(&config.derivative,&unknown_source,sizeof(unknown_source));
    rejected(config,0);
    config=base;config.options=FIELDCTRL_PID_OPT_SETPOINT_WEIGHT;
    config.setpoint_weight=A::gain(1.5);rejected(config,0);
    config.setpoint_weight=A::gain(-0.25);rejected(config,0);
    config=base;config.options=FIELDCTRL_PID_OPT_INTEGRAL_SEPARATION;
    config.separation=A::make(0);rejected(config,0);
    config.separation=A::make(-0.25);rejected(config,0);
    config=base;config.options=FIELDCTRL_PID_OPT_INTEGRAL_LIMIT;
    config.integral_lower=A::make(0.25);config.integral_upper=A::make(-0.25);rejected(config,0);
    config.integral_lower=A::make(-0.25);config.integral_upper=A::make(0.25);rejected(config,0.375);
    rejected(base,0.75);
    EXPECT_EQ(A::pid_init(nullptr,&base,A::make(0)),LINEARS_ERROR_ARGUMENT);
    EXPECT_EQ(A::pid_init(&state,nullptr,A::make(0)),LINEARS_ERROR_ARGUMENT);
    EXPECT_TRUE(fc_test::same_bytes(state,saved));
    // Disabled option parameters are ignored; equal integral bounds are valid.
    config=base;config.setpoint_weight=A::gain(2);config.separation=A::make(-0.5);
    config.integral_lower=A::make(0.25);config.integral_upper=A::make(-0.25);
    EXPECT_EQ(A::pid_init(&state,&config,A::make(0)),LINEARS_OK);
    config=base;config.options=FIELDCTRL_PID_OPT_INTEGRAL_LIMIT;
    config.integral_lower=config.integral_upper=A::make(0.125);
    EXPECT_EQ(A::pid_init(&state,&config,A::make(0.125)),LINEARS_OK);
    if constexpr(!A::fixed) {
        const auto accepted=state;
        config=base;config.separation=std::numeric_limits<typename A::scalar>::quiet_NaN();
        EXPECT_EQ(A::pid_init(&state,&config,A::make(0)),LINEARS_ERROR_RANGE);
        EXPECT_TRUE(fc_test::same_bytes(state,accepted));
    }
}

/* Field-wise equality of two step results; padding bytes are not compared. */
template<class A> void expect_same_result(const typename A::pid_output &a,
                                          const typename A::pid_output &b) {
    EXPECT_EQ(a.output,b.output);
    EXPECT_EQ(a.proportional,b.proportional);
    EXPECT_EQ(a.integral,b.integral);
    EXPECT_EQ(a.derivative,b.derivative);
    EXPECT_EQ(a.saturated,b.saturated);
}

/* A setpoint square wave that saturates the tuning below, with a moving measurement. */
inline double square_setpoint(int n) { return n%16<8?0.375:-0.25; }
inline double ramp_measurement(int n) { return 0.0078125*(n%8); }

TYPED_TEST(Pid, ReverseActionMirrorsDirectActionOnNegatedInputs) {
    using A=fc_test::Api<TypeParam>;
    auto direct=pid_tuning<A>(0.5,2,0.0625,-0.5,0.5);
    auto reverse=pid_tuning<A>(-0.5,-2,-0.0625,-0.5,0.5);
    direct.derivative_filter=reverse.derivative_filter=A::gain(0.125);
    typename A::pid a{},b{};
    ASSERT_EQ(A::pid_init(&a,&direct,A::make(0)),LINEARS_OK);
    ASSERT_EQ(A::pid_init(&b,&reverse,A::make(0)),LINEARS_OK);
    int saturated=0;
    for(int n=0;n<64;++n) {
        const double setpoint=square_setpoint(n),measurement=ramp_measurement(n);
        const auto ra=pid_run<A>(&a,-setpoint,-measurement,0.125);
        const auto rb=pid_run<A>(&b,setpoint,measurement,0.125);
        expect_same_result<A>(ra,rb);
        saturated+=rb.saturated!=0;
    }
    EXPECT_GT(saturated,0);
    // Mixed signs would make the terms fight each other.
    auto mixed=direct;mixed.kd=A::gain(-0.0625);
    const auto saved=b;
    EXPECT_EQ(A::pid_init(&b,&mixed,A::make(0)),LINEARS_ERROR_DOMAIN);
    EXPECT_TRUE(fc_test::same_bytes(b,saved));
}

TYPED_TEST(Pid, RetuningEveryStepWithTheSameTuningChangesNothing) {
    using A=fc_test::Api<TypeParam>;
    auto config=pid_tuning<A>(0.5,2,0.0625,-0.5,0.5);
    config.derivative_filter=A::gain(0.125);
    typename A::pid untouched{},retuned{};
    ASSERT_EQ(A::pid_init(&untouched,&config,A::make(0)),LINEARS_OK);
    ASSERT_EQ(A::pid_init(&retuned,&config,A::make(0)),LINEARS_OK);
    for(int n=0;n<64;++n) {
        ASSERT_EQ(A::pid_retune(&retuned,&config),LINEARS_OK);
        const double setpoint=square_setpoint(n),measurement=ramp_measurement(n);
        expect_same_result<A>(pid_run<A>(&untouched,setpoint,measurement,0.125),
                              pid_run<A>(&retuned,setpoint,measurement,0.125));
    }
}

TYPED_TEST(Pid, RetuneKeepsTheHistoryAndClampsTheIntegral) {
    using A=fc_test::Api<TypeParam>;
    typename A::pid state{};
    auto config=pid_tuning<A>(0,1,0.0625,-0.5,0.5); // ki*dt = 0.125, kd/dt = 0.5
    ASSERT_EQ(A::pid_init(&state,&config,A::make(0)),LINEARS_OK);
    pid_run<A>(&state,0.5,0,0.125);
    pid_run<A>(&state,0.5,0,0.125);
    ASSERT_DOUBLE_EQ(fc_test::value(state.integral),0.125);
    // A narrower output range pulls the integral in; nothing else restarts.
    config.lower=A::make(-0.0625);config.upper=A::make(0.0625);
    ASSERT_EQ(A::pid_retune(&state,&config),LINEARS_OK);
    EXPECT_DOUBLE_EQ(fc_test::value(state.integral),0.0625);
    EXPECT_EQ(state.config.lower,config.lower);
    EXPECT_EQ(state.config.upper,config.upper);
    EXPECT_NE(state.history,0);
    EXPECT_DOUBLE_EQ(fc_test::value(state.previous_error),0.5);
    EXPECT_DOUBLE_EQ(fc_test::value(state.cached_dt),0.125);
    // The error change 0.5 -> 0.25 is differentiated across the retune.
    const auto r=pid_run<A>(&state,0.25,0,0.125);
    EXPECT_DOUBLE_EQ(fc_test::value(r.derivative),-0.125);
    EXPECT_DOUBLE_EQ(fc_test::value(r.integral),0.0625);
    EXPECT_DOUBLE_EQ(fc_test::value(r.output),-0.0625);
}

TYPED_TEST(Pid, RetuneRecomputesCachedCoefficientsOnlyWhenTheyChange) {
    using A=fc_test::Api<TypeParam>;
    typename A::pid state{};
    auto config=pid_tuning<A>(0,0.5,0,-1,1);
    ASSERT_EQ(A::pid_init(&state,&config,A::make(0)),LINEARS_OK);
    pid_run<A>(&state,0,0,0.125);
    ASSERT_DOUBLE_EQ(fc_test::value(state.cached_dt),0.125);
    ASSERT_DOUBLE_EQ(fc_test::value(state.derivative_gain),0);
    // Bounds do not enter the cache.
    config.lower=A::make(-0.75);config.upper=A::make(0.75);
    ASSERT_EQ(A::pid_retune(&state,&config),LINEARS_OK);
    EXPECT_DOUBLE_EQ(fc_test::value(state.cached_dt),0.125);
    // Enabling kd at the same dt must not reuse the never-computed kd/dt.
    config.kd=A::gain(0.0625);
    ASSERT_EQ(A::pid_retune(&state,&config),LINEARS_OK);
    EXPECT_DOUBLE_EQ(fc_test::value(state.cached_dt),0);
    const auto r=pid_run<A>(&state,0.25,0,0.125);
    EXPECT_DOUBLE_EQ(fc_test::value(state.derivative_gain),0.5);
    EXPECT_DOUBLE_EQ(fc_test::value(r.derivative),0.125);
    EXPECT_DOUBLE_EQ(fc_test::value(r.integral),0.015625);
}

TYPED_TEST(Pid, RetuneKeepsTheFilterStateUnlessKdIsZero) {
    using A=fc_test::Api<TypeParam>;
    typename A::pid state{};
    auto config=pid_tuning<A>(0,0,0.0625,-1,1);
    config.derivative_filter=A::gain(0.125); // dt/(Tf+dt) = 1/2
    ASSERT_EQ(A::pid_init(&state,&config,A::make(0)),LINEARS_OK);
    pid_run<A>(&state,0,0,0.125);
    ASSERT_DOUBLE_EQ(fc_test::value(pid_run<A>(&state,0.5,0,0.125).derivative),0.125);
    // Without the filter the next step applies the raw derivative 0.5*0.25.
    config.derivative_filter=A::gain(0);
    ASSERT_EQ(A::pid_retune(&state,&config),LINEARS_OK);
    EXPECT_DOUBLE_EQ(fc_test::value(state.derivative),0.125);
    EXPECT_DOUBLE_EQ(fc_test::value(pid_run<A>(&state,0.75,0,0.125).derivative),0.125);
    // A zero kd holds D(n-1) in the step, so retune must clear it.
    config.kd=A::gain(0);
    ASSERT_EQ(A::pid_retune(&state,&config),LINEARS_OK);
    EXPECT_DOUBLE_EQ(fc_test::value(state.derivative),0);
    const auto r=pid_run<A>(&state,0.25,0,0.125);
    EXPECT_DOUBLE_EQ(fc_test::value(r.derivative),0);
    EXPECT_DOUBLE_EQ(fc_test::value(r.output),0);
}

TYPED_TEST(Pid, RetuneValidatesLikeInitAndFailsAtomically) {
    using A=fc_test::Api<TypeParam>;
    typename A::pid state{};
    const auto base=pid_tuning<A>(0.5,0.5,0.0625,-0.5,0.5);
    ASSERT_EQ(A::pid_init(&state,&base,A::make(0)),LINEARS_OK);
    pid_run<A>(&state,0.25,0,0.125);
    const auto saved=state;
    auto rejected=[&](const typename A::pid_config &config) {
        EXPECT_EQ(A::pid_retune(&state,&config),LINEARS_ERROR_DOMAIN);
        EXPECT_TRUE(fc_test::same_bytes(state,saved));
    };
    auto config=base;config.ki=A::gain(-0.5);rejected(config);
    config=base;config.lower=A::make(0.5);rejected(config);
    config=base;config.derivative_filter=A::gain(-0.125);rejected(config);
    config=base;config.options=1u<<3;rejected(config);
    config=base;config.options=FIELDCTRL_PID_OPT_INTEGRAL_LIMIT;
    config.integral_lower=A::make(0.25);config.integral_upper=A::make(-0.25);rejected(config);
    EXPECT_EQ(A::pid_retune(nullptr,&base),LINEARS_ERROR_ARGUMENT);
    EXPECT_EQ(A::pid_retune(&state,nullptr),LINEARS_ERROR_ARGUMENT);
    EXPECT_TRUE(fc_test::same_bytes(state,saved));
    if constexpr(!A::fixed) {
        config=base;config.kp=std::numeric_limits<typename A::scalar>::quiet_NaN();
        EXPECT_EQ(A::pid_retune(&state,&config),LINEARS_ERROR_RANGE);
        EXPECT_TRUE(fc_test::same_bytes(state,saved));
    }
}

TYPED_TEST(Pid, CoefficientCacheOverflowDoesNotCommit) {
    using A=fc_test::Api<TypeParam>;
    typename A::pid state{};
    auto config=pid_tuning<A>(0,0,0,-0.5,0.5);
    typename A::scalar dt;
    if constexpr(A::fixed) {
        config.kd=INT64_MAX;dt=1; // kd/dt is about 2^32 divided by one raw LSB.
    } else {
        config.kd=std::numeric_limits<typename A::scalar>::max();dt=A::make(0.5);
    }
    ASSERT_EQ(A::pid_init(&state,&config,A::make(0)),LINEARS_OK);
    const auto saved=state;
    typename A::pid_output output{},saved_output=output;
    EXPECT_EQ(A::pid_step(&state,A::make(0),A::make(0),dt,&output),LINEARS_ERROR_RANGE);
    EXPECT_TRUE(fc_test::same_bytes(state,saved));
    EXPECT_TRUE(fc_test::same_bytes(output,saved_output));
}

TYPED_TEST(Pid, SoftwareMassPositionLoopSettlesWithMeasurementDerivative) {
    using A=fc_test::Api<TypeParam>;
    // Unit mass, force limited to +/-1 N. kp=4 N/m and kd=4 N*s/m make the
    // linear PD loop critically damped (2 rad/s); without D it would oscillate.
    // An independent float64 model of this discrete law settles without
    // overshoot after 244 saturated samples. Q uses position base 1 m, force
    // base 2 N and Tbase 1 s, so kp/kd in pu are halved and the limit is 0.5.
    const double force_base=A::fixed?2:1,dt=1.0/1024;
    auto config=pid_tuning<A>(4/force_base,0,4/force_base,-1/force_base,1/force_base);
    config.derivative=FIELDCTRL_PID_DERIVATIVE_ON_MEASUREMENT;
    config.derivative_filter=A::gain(1.0/64);
    typename A::pid state{};
    ASSERT_EQ(A::pid_init(&state,&config,A::make(0)),LINEARS_OK);
    double position=0,velocity=0,peak=0;
    int saturated=0;
    for(int n=0;n<16*1024;++n) {
        const auto r=pid_run<A>(&state,0.5,position,dt);
        const double force=fc_test::value(r.output)*force_base;
        EXPECT_LE(std::abs(force),1.0);
        saturated+=r.saturated!=0;
        velocity+=dt*force;
        position+=dt*velocity;
        peak=std::max(peak,position);
    }
    EXPECT_GT(saturated,100);
    EXPECT_LT(peak,0.5*1.01);
    EXPECT_NEAR(position,0.5,0.002);
    EXPECT_NEAR(velocity,0,0.002);
}
} // namespace
