#include "support.h"
#include <array>
#include <random>

namespace {
template<class T> class PiPwm : public ::testing::Test {};
TYPED_TEST_SUITE(PiPwm, fc_test::Types,);

TYPED_TEST(PiPwm, ConditionalIntegrationStopsWindupAndRecovers) {
    using A=fc_test::Api<TypeParam>;
    typename A::pi state{};
    const double bound=A::fixed?0.5:4,drive=A::fixed?0.75:10,recover=A::fixed?-0.125:-1;
    ASSERT_EQ(A::pi_init(&state,A::gain(2),A::gain(3),A::make(-bound),A::make(bound),A::make(0)),LINEARS_OK);
    auto out=A::make(0.25),dt=A::make(0.01);
    for(int n=0;n<1000;++n) ASSERT_EQ(A::pi_step(&state,A::make(drive),dt,&out),LINEARS_OK);
    EXPECT_DOUBLE_EQ(fc_test::value(out),bound);
    EXPECT_DOUBLE_EQ(fc_test::value(state.integral),0);
    ASSERT_EQ(A::pi_step(&state,A::make(recover),dt,&out),LINEARS_OK);
    const double increment=3*fc_test::value(dt)*fc_test::value(A::make(recover));
    EXPECT_NEAR(fc_test::value(out),2*recover+increment,4*A::unit);
    EXPECT_NEAR(fc_test::value(state.integral),increment,3*A::unit);
    for(int n=0;n<1000;++n) ASSERT_EQ(A::pi_step(&state,A::make(-drive),dt,&out),LINEARS_OK);
    EXPECT_DOUBLE_EQ(fc_test::value(out),-bound);
    EXPECT_NEAR(fc_test::value(state.integral),increment,3*A::unit);
}

TYPED_TEST(PiPwm, RetainsSubQ15IntegralIncrements) {
    using A=fc_test::Api<TypeParam>;
    typename A::pi state{};
    if constexpr(A::fixed) {
        // The increment 2^-19 is 1/16 of a Q15 LSB but an exact F31 value.
        ASSERT_EQ(A::pi_init(&state,A::gain(0),A::gain(1),A::make(-0.5),A::make(0.5),A::make(0.25)),LINEARS_OK);
        auto out=A::make(0),dt=A::make(1.0/16384),error=A::make(1.0/32);
        for(int n=0;n<20000;++n) ASSERT_EQ(A::pi_step(&state,error,dt,&out),LINEARS_OK);
        const double target=0.25+20000.0/524288;
        EXPECT_DOUBLE_EQ(fc_test::value(state.integral),target);
        EXPECT_EQ(out,A::make(target));
    } else {
        ASSERT_EQ(A::pi_init(&state,A::gain(0),A::gain(1),A::make(-2),A::make(2),A::make(1)),LINEARS_OK);
        auto out=A::make(0),dt=A::make(0.00001);
        for(int n=0;n<20000;++n) ASSERT_EQ(A::pi_step(&state,A::make(1),dt,&out),LINEARS_OK);
        const double target=1+20000*fc_test::value(dt);
        const double accumulation=std::is_same_v<TypeParam,fc_test::F32>?20000*std::ldexp(1.0,-24):20000*std::ldexp(1.0,-53);
        EXPECT_NEAR(fc_test::value(state.integral),target,accumulation);
        EXPECT_NEAR(fc_test::value(out),target,accumulation+2*A::unit);
        EXPECT_GT(fc_test::value(state.integral),1.19);
    }
}

TYPED_TEST(PiPwm, SoftwareRLCurrentLoopTracksBothDirectionsWithBusSaturation) {
    using A=fc_test::Api<TypeParam>;
    typename A::pi state{};
    // Q retains the original physical RL plant. Bases are caller-side only.
    const double ibase=A::fixed?64:1,vbase=A::fixed?64:1,tbase=A::fixed?0.01:1;
    ASSERT_EQ(A::pi_init(&state,A::gain(3*ibase/vbase),A::gain(700*ibase*tbase/vbase),
        A::make(-12/vbase),A::make(12/vbase),A::make(0)),LINEARS_OK);
    const auto dt=A::make(0.0001/tbase);
    const double h=fc_test::value(dt)*tbase,resistance=0.7,inductance=0.003;
    double current=0;
    auto output=A::make(0);
    int saturated=0;
    for(int n=0;n<12000;++n) {
        const double ref=n<3000?5:n<6000?-5:n<9000?30:2;
        const auto error=A::make((ref-current)/ibase);
        ASSERT_EQ(A::pi_step(&state,error,dt,&output),LINEARS_OK);
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
    typename A::pi one{},two{};
    const double bound=A::fixed?0.75:5,error=A::fixed?0.125:1;
    ASSERT_EQ(A::pi_init(&one,A::gain(1),A::gain(2),A::make(-bound),A::make(bound),A::make(0)),LINEARS_OK);
    ASSERT_EQ(A::pi_init(&two,A::gain(1),A::gain(2),A::make(-bound),A::make(bound),A::make(0)),LINEARS_OK);
    auto out=A::make(0),dt=A::make(0.01);
    for(int n=0;n<30;++n) ASSERT_EQ(A::pi_step(&one,A::make(error),dt,&out),LINEARS_OK);
    EXPECT_DOUBLE_EQ(fc_test::value(two.integral),0);
    ASSERT_EQ(A::pi_step(&two,A::make(-error),dt,&out),LINEARS_OK);
    EXPECT_LT(fc_test::value(two.integral),0);EXPECT_GT(fc_test::value(one.integral),0);
}

TYPED_TEST(PiPwm, FailureIsAtomicForParametersAliasAndNumericRange) {
    using A=fc_test::Api<TypeParam>;
    typename A::pi state{};
    ASSERT_EQ(A::pi_init(&state,A::gain(1),A::gain(1),A::make(-1),A::make(1),A::make(0)),LINEARS_OK);
    auto saved=state;auto out=A::make(0.3),old=out;
    EXPECT_EQ(A::pi_step(&state,A::make(1),A::make(0),&out),LINEARS_ERROR_DOMAIN);
    EXPECT_TRUE(fc_test::same_bytes(state,saved));EXPECT_TRUE(fc_test::same_bytes(out,old));
    EXPECT_EQ(A::pi_step(&state,A::make(1),A::make(0.01),reinterpret_cast<typename A::scalar*>(&state.kp)),LINEARS_ERROR_ALIAS);
    EXPECT_TRUE(fc_test::same_bytes(state,saved));
    EXPECT_EQ(A::pi_init(&state,A::gain(-1),A::gain(1),A::make(-1),A::make(1),A::make(0)),LINEARS_ERROR_DOMAIN);
    EXPECT_TRUE(fc_test::same_bytes(state,saved));
    EXPECT_EQ(A::pi_init(&state,A::gain(1),A::gain(1),A::make(1),A::make(1),A::make(0)),LINEARS_ERROR_DOMAIN);
    typename A::abc duty{old,old,old},duty_saved=duty;
    typename A::ab voltage{A::make(A::fixed?0.25:1),A::make(A::fixed?0.5:2)};
    EXPECT_EQ(A::svpwm(voltage,A::make(0),&duty),LINEARS_ERROR_DOMAIN);
    EXPECT_TRUE(fc_test::same_bytes(duty,duty_saved));
    if constexpr(!A::fixed) {
        auto bad=std::numeric_limits<typename A::scalar>::infinity();
        EXPECT_EQ(A::pi_step(&state,bad,A::make(0.01),&out),LINEARS_ERROR_RANGE);
        EXPECT_TRUE(fc_test::same_bytes(state,saved));EXPECT_TRUE(fc_test::same_bytes(out,old));
        EXPECT_EQ(A::svpwm({bad,A::make(0)},A::make(12),&duty),LINEARS_ERROR_RANGE);
        EXPECT_TRUE(fc_test::same_bytes(duty,duty_saved));
    } else {
        // There are no invalid Q bit patterns; invalid signs remain a domain error.
        EXPECT_EQ(A::pi_step(&state,A::make(0.5),A::make(-0.25),&out),LINEARS_ERROR_DOMAIN);
        EXPECT_TRUE(fc_test::same_bytes(state,saved));EXPECT_TRUE(fc_test::same_bytes(out,old));
        EXPECT_EQ(A::svpwm(voltage,A::make(-1),&duty),LINEARS_ERROR_DOMAIN);
        EXPECT_TRUE(fc_test::same_bytes(duty,duty_saved));
    }
    EXPECT_EQ(A::limit_voltage(voltage,A::make(-1),&voltage),LINEARS_ERROR_DOMAIN);
}

TYPED_TEST(PiPwm, ArithmeticOverflowDoesNotCommitEvenWithFiniteInputs) {
    using A=fc_test::Api<TypeParam>;
    typename A::pi state{};
    typename A::coefficient large;
    if constexpr(A::fixed) {
        large=INT64_MAX;
        ASSERT_EQ(A::pi_init(&state,large,A::gain(0),A::make(-1),A::make(0.5),A::make(-1)),LINEARS_OK);
    } else {
        large=std::numeric_limits<typename A::scalar>::max()/2;
        ASSERT_EQ(A::pi_init(&state,large,A::gain(1),A::make(-1),A::make(1),A::make(0)),LINEARS_OK);
    }
    auto saved=state;
    auto output=A::make(0.25),saved_output=output;
    // Q: (-INT64_MAX) + (-ONE) overflows the fixed working range.
    EXPECT_EQ(A::pi_step(&state,A::make(A::fixed?-1:4),A::make(0.01),&output),LINEARS_ERROR_RANGE);
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
    fieldctrl_q31_pi_t state{};
    const int32_t quarter=INT32_C(536870912),half=INT32_C(1073741824);
    ASSERT_EQ(fieldctrl_q31_pi_init(&state,0,LINEARS_FIXED_ONE,-half,half,quarter),LINEARS_OK);
    int32_t output=0;
    for(int n=0;n<4096;++n)
        ASSERT_EQ(fieldctrl_q31_pi_step(&state,1,quarter,&output),LINEARS_OK);
    EXPECT_EQ(state.integral,quarter); // Each exact increment is 1/4 of one F31 LSB.
    for(int n=0;n<1024;++n)
        ASSERT_EQ(fieldctrl_q31_pi_step(&state,1,half,&output),LINEARS_OK);
    EXPECT_EQ(state.integral,INT64_C(536870912)+1024); // Halfway rounds away from zero.
    for(int n=0;n<1024;++n)
        ASSERT_EQ(fieldctrl_q31_pi_step(&state,-1,half,&output),LINEARS_OK);
    EXPECT_EQ(state.integral,quarter);EXPECT_EQ(output,quarter);
}

TEST(PiPwmIntegerRange,Q15SmallestErrorAndTimeAccumulateBeforeSignalQuantization) {
    fieldctrl_q15_pi_t state{};
    ASSERT_EQ(fieldctrl_q15_pi_init(&state,0,LINEARS_FIXED_ONE,-16384,16384,8192),LINEARS_OK);
    int16_t output=0;
    for(int n=0;n<16384;++n)
        ASSERT_EQ(fieldctrl_q15_pi_step(&state,1,1,&output),LINEARS_OK);
    EXPECT_EQ(state.integral,INT64_C(536870912)+32768); // Each update is two F31 raw units.
    EXPECT_EQ(output,8193); // Exactly half a positive Q15 output LSB rounds upward.
}
} // namespace
