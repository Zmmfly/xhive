#include "support.h"
#include <array>
#include <random>

namespace {
template<class T> class PiPwm : public ::testing::Test {};
TYPED_TEST_SUITE(PiPwm, fc_test::Types);

TYPED_TEST(PiPwm, ConditionalIntegrationStopsWindupAndRecovers) {
    using A = fc_test::Api<TypeParam>;
    typename A::pi state{};
    ASSERT_EQ(A::pi_init(&state,A::make(2),A::make(3),A::make(-4),A::make(4),A::make(0)),LINEARS_OK);
    auto out = A::make(9);
    for (int n=0;n<1000;++n) ASSERT_EQ(A::pi_step(&state,A::make(10),A::make(0.01),&out),LINEARS_OK);
    EXPECT_DOUBLE_EQ(fc_test::value(out),4);
    EXPECT_DOUBLE_EQ(fc_test::value(state.integral),0);
    ASSERT_EQ(A::pi_step(&state,A::make(-1),A::make(0.01),&out),LINEARS_OK);
    const double dt=fc_test::value(A::make(0.01));
    EXPECT_NEAR(fc_test::value(out),-2-3*dt,4*A::unit);
    EXPECT_NEAR(fc_test::value(state.integral),-3*dt,3*A::unit);
    for (int n=0;n<1000;++n) ASSERT_EQ(A::pi_step(&state,A::make(-10),A::make(0.01),&out),LINEARS_OK);
    EXPECT_DOUBLE_EQ(fc_test::value(out),-4);
    EXPECT_NEAR(fc_test::value(state.integral),-3*dt,3*A::unit);
}

TYPED_TEST(PiPwm, RetainsSubQ15IntegralIncrements) {
    using A = fc_test::Api<TypeParam>;
    typename A::pi state{};
    ASSERT_EQ(A::pi_init(&state,A::make(0),A::make(1),A::make(-2),A::make(2),A::make(1)),LINEARS_OK);
    auto out=A::make(0),dt=A::make(0.00001);
    for(int n=0;n<20000;++n) ASSERT_EQ(A::pi_step(&state,A::make(1),dt,&out),LINEARS_OK);
    const double target=1+20000*fc_test::value(dt);
    // Each accumulator addition has <=0.5 ULP at magnitude [1,2).
    const double accumulation=std::is_same_v<TypeParam,fc_test::F32> ? 20000*std::ldexp(1.0,-24) :
                              std::is_same_v<TypeParam,fc_test::F64> ? 20000*std::ldexp(1.0,-53) :
                                                                    20000*std::ldexp(1.0,-31);
    EXPECT_NEAR(fc_test::value(state.integral),target,accumulation);
    EXPECT_NEAR(fc_test::value(out),target,accumulation+2*A::unit);
    EXPECT_GT(fc_test::value(state.integral),1.19);
}

TYPED_TEST(PiPwm, SoftwareRLCurrentLoopTracksBothDirectionsWithBusSaturation) {
    using A = fc_test::Api<TypeParam>;
    typename A::pi state{};
    ASSERT_EQ(A::pi_init(&state,A::make(3),A::make(700),A::make(-12),A::make(12),A::make(0)),LINEARS_OK);
    const auto dt=A::make(0.0001);
    const double h=fc_test::value(dt),resistance=0.7,inductance=0.003;
    double current=0;
    auto output=A::make(0);
    int saturated=0;
    for (int n=0;n<12000;++n) {
        const double ref=n<3000?5:n<6000?-5:n<9000?30:2;
        const auto error=A::make(ref-current);
        ASSERT_EQ(A::pi_step(&state,error,dt,&output),LINEARS_OK);
        const double applied=fc_test::value(output);
        EXPECT_LE(applied,12);EXPECT_GE(applied,-12);
        if(std::abs(applied)==12)++saturated;
        current+=h*(applied-resistance*current)/inductance;
        if(n==2999||n==5999||n==11999) { EXPECT_NEAR(current,ref,0.002); }
        if(n==8999) { EXPECT_NEAR(current,12/resistance,0.002); }
    }
    EXPECT_GT(saturated,1000);
}

TYPED_TEST(PiPwm, VoltageLimitAndPwmMatchIndependentCommonModeFormula) {
    using A = fc_test::Api<TypeParam>;
    std::mt19937 rng(0x50574d);
    std::uniform_real_distribution<double> components(-60,60),buses(2,48);
    for(int n=0;n<600;++n) {
        typename A::ab in{A::make(components(rng)),A::make(components(rng))},limited{};
        typename A::abc duty{};
        const auto bus=A::make(buses(rng)),limit=A::make(9);
        ASSERT_EQ(A::limit_voltage(in,limit,&limited),LINEARS_OK);
        double a=fc_test::value(in.alpha),b=fc_test::value(in.beta);
        const double m=std::hypot(a,b),scale=m>9?9/m:1;
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
        // Common mode must not change line-to-line voltages.
        EXPECT_NEAR((actual[0]-actual[1])*vdc,phases[0]-phases[1],16*A::unit*vdc);
    }
}

TYPED_TEST(PiPwm, ZeroVoltageLargeInputsAndIndependentContexts) {
    using A = fc_test::Api<TypeParam>;
    typename A::abc duty{};
    typename A::ab zero{A::make(0),A::make(0)},large{A::make(10000),A::make(-10000)},limited{};
    ASSERT_EQ(A::svpwm(zero,A::make(12),&duty),LINEARS_OK);
    EXPECT_DOUBLE_EQ(fc_test::value(duty.a),0.5);EXPECT_DOUBLE_EQ(fc_test::value(duty.b),0.5);
    ASSERT_EQ(A::limit_voltage(large,A::make(0),&limited),LINEARS_OK);
    EXPECT_DOUBLE_EQ(fc_test::value(limited.alpha),0);EXPECT_DOUBLE_EQ(fc_test::value(limited.beta),0);
    ASSERT_EQ(A::svpwm(large,A::make(12),&duty),LINEARS_OK);
    EXPECT_LE(fc_test::value(duty.a),1);EXPECT_GE(fc_test::value(duty.b),0);
    typename A::pi one{},two{};
    ASSERT_EQ(A::pi_init(&one,A::make(1),A::make(2),A::make(-5),A::make(5),A::make(0)),LINEARS_OK);
    ASSERT_EQ(A::pi_init(&two,A::make(1),A::make(2),A::make(-5),A::make(5),A::make(0)),LINEARS_OK);
    auto out=A::make(0),dt=A::make(0.01);
    for(int n=0;n<30;++n) ASSERT_EQ(A::pi_step(&one,A::make(1),dt,&out),LINEARS_OK);
    EXPECT_DOUBLE_EQ(fc_test::value(two.integral),0);
    ASSERT_EQ(A::pi_step(&two,A::make(-1),dt,&out),LINEARS_OK);
    EXPECT_LT(fc_test::value(two.integral),0);EXPECT_GT(fc_test::value(one.integral),0);
}

TYPED_TEST(PiPwm, FailureIsAtomicForParametersAliasAndNumericRange) {
    using A = fc_test::Api<TypeParam>;
    typename A::pi state{};
    ASSERT_EQ(A::pi_init(&state,A::make(1),A::make(1),A::make(-1),A::make(1),A::make(0)),LINEARS_OK);
    auto saved=state; auto out=A::make(0.3),old=out;
    EXPECT_EQ(A::pi_step(&state,A::make(1),A::make(0),&out),LINEARS_ERROR_DOMAIN);
    EXPECT_TRUE(fc_test::same_bytes(state,saved));EXPECT_TRUE(fc_test::same_bytes(out,old));
    EXPECT_EQ(A::pi_step(&state,A::make(1),A::make(0.01),&state.kp),LINEARS_ERROR_ALIAS);
    EXPECT_TRUE(fc_test::same_bytes(state,saved));
    EXPECT_EQ(A::pi_init(&state,A::make(-1),A::make(1),A::make(-1),A::make(1),A::make(0)),LINEARS_ERROR_DOMAIN);
    EXPECT_TRUE(fc_test::same_bytes(state,saved));
    EXPECT_EQ(A::pi_init(&state,A::make(1),A::make(1),A::make(1),A::make(1),A::make(0)),LINEARS_ERROR_DOMAIN);
    typename A::abc duty{old,old,old},duty_saved=duty;
    typename A::ab voltage{A::make(1),A::make(2)};
    EXPECT_EQ(A::svpwm(voltage,A::make(0),&duty),LINEARS_ERROR_DOMAIN);
    EXPECT_TRUE(fc_test::same_bytes(duty,duty_saved));
    auto bad=A::make(1);
    if constexpr(A::fixed) bad.exponent=LINEARS_EXPONENT_MAX+1;
    else bad=std::numeric_limits<typename A::scalar>::infinity();
    EXPECT_EQ(A::pi_step(&state,bad,A::make(0.01),&out),LINEARS_ERROR_RANGE);
    EXPECT_TRUE(fc_test::same_bytes(state,saved));EXPECT_TRUE(fc_test::same_bytes(out,old));
    EXPECT_EQ(A::svpwm({bad,A::make(0)},A::make(12),&duty),LINEARS_ERROR_RANGE);
    EXPECT_TRUE(fc_test::same_bytes(duty,duty_saved));
    EXPECT_EQ(A::limit_voltage(voltage,A::make(-1),&voltage),LINEARS_ERROR_DOMAIN);
}

TYPED_TEST(PiPwm, ArithmeticOverflowDoesNotCommitEvenWithFiniteInputs) {
    using A = fc_test::Api<TypeParam>;
    auto large = A::make(1);
    if constexpr (A::fixed) large.exponent = LINEARS_EXPONENT_MAX;
    else large = std::numeric_limits<typename A::scalar>::max() / 2;
    typename A::pi state{};
    ASSERT_EQ(A::pi_init(&state,large,A::make(1),A::make(-1),A::make(1),A::make(0)),LINEARS_OK);
    auto saved = state;
    auto output = A::make(0.25), saved_output = output;
    EXPECT_EQ(A::pi_step(&state,A::make(4),A::make(0.01),&output),LINEARS_ERROR_RANGE);
    EXPECT_TRUE(fc_test::same_bytes(state,saved));
    EXPECT_TRUE(fc_test::same_bytes(output,saved_output));
}
TYPED_TEST(PiPwm, DutyIsInvariantUnderExtremeCommonVoltageScaling) {
    using A = fc_test::Api<TypeParam>;
    for (int scale_index = 0; scale_index < 4; ++scale_index) {
        auto scaled = [scale_index](double factor) {
            auto v = A::make(factor);
            if constexpr (A::fixed) {
                const int powers[] = {-1000, -100, 0, 1000};
                v.exponent += powers[scale_index];
            } else {
                using S = typename A::scalar;
                const S scales[] = {std::numeric_limits<S>::denorm_min(),
                    std::numeric_limits<S>::min(), S(1), std::numeric_limits<S>::max()/8};
                v = static_cast<S>(factor)*scales[scale_index];
            }
            return v;
        };
        for (const auto &command : {std::array<double,3>{1,0,1}, {2,1,4}, {4,-2,1}}) {
            typename A::abc duty{};
            ASSERT_EQ(A::svpwm({scaled(command[0]),scaled(command[1])},scaled(command[2]),&duty),LINEARS_OK);
            double a=command[0]/command[2],b=command[1]/command[2];
            const double norm=std::hypot(a,b),limit=1/std::sqrt(3.0);
            if(norm>limit) { a*=limit/norm; b*=limit/norm; }
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
    constexpr int exponent=std::is_same_v<TypeParam,fc_test::F32>?80:600;
    const double tiny=std::ldexp(1.0,-exponent),huge=std::ldexp(1.0,exponent);
    typename A::ab output{};
    ASSERT_EQ(A::limit_voltage({A::make(huge),A::make(0)},A::make(tiny),&output),LINEARS_OK);
    EXPECT_DOUBLE_EQ(fc_test::value(output.alpha),tiny);
    EXPECT_DOUBLE_EQ(fc_test::value(output.beta),0);
    // Reversing division/multiplication fixes the first case but loses this
    // independently representable small component before multiplication.
    ASSERT_EQ(A::limit_voltage({A::make(tiny),A::make(huge)},A::make(huge/2),&output),LINEARS_OK);
    EXPECT_DOUBLE_EQ(fc_test::value(output.alpha),tiny/2);
    EXPECT_DOUBLE_EQ(fc_test::value(output.beta),huge/2);
}

TYPED_TEST(PiPwm, LimiterFinalExponentAndUnrepresentableNormContract) {
    using A=fc_test::Api<TypeParam>;
    typename A::ab output{A::make(2),A::make(3)};
    if constexpr(A::fixed) {
        for(int sign : {-1,1}) {
            auto input=A::make(sign*0.5),limit=A::make(0.5);
            input.exponent=LINEARS_EXPONENT_MAX;
            limit.exponent=LINEARS_EXPONENT_MIN;
            ASSERT_EQ(A::limit_voltage({input,A::make(0)},limit,&output),LINEARS_OK);
            EXPECT_EQ(output.alpha.mantissa,sign*limit.mantissa);
            EXPECT_EQ(output.alpha.exponent,limit.exponent);
            EXPECT_EQ(output.beta.mantissa,0);
            EXPECT_EQ(output.beta.exponent,0);
        }
    } else {
        // The API explicitly reports RANGE when its working vector norm
        // cannot be represented, even if a clipped result would fit.
        const auto large=std::numeric_limits<typename A::scalar>::max();
        auto saved=output;
        EXPECT_EQ(A::limit_voltage({large,large},A::make(1),&output),LINEARS_ERROR_RANGE);
        EXPECT_TRUE(fc_test::same_bytes(output,saved));
    }
}
template<class T> void check_independent_exponent_hypot() {
    using A=fc_test::Api<T>;
    for(int sign_a : {-1,1}) for(int sign_b : {-1,1}) for(int swap : {0,1}) {
        auto a=A::make(sign_a*0.5),b=A::make(sign_b*0.5),limit=A::make(0.5);
        a.exponent=swap?LINEARS_EXPONENT_MIN:LINEARS_EXPONENT_MAX;
        b.exponent=swap?LINEARS_EXPONENT_MAX:LINEARS_EXPONENT_MIN;
        limit.exponent=LINEARS_EXPONENT_MAX;
        typename A::ab output{};
        ASSERT_EQ(A::limit_voltage({a,b},limit,&output),LINEARS_OK);
        EXPECT_EQ(output.alpha.mantissa,a.mantissa);
        EXPECT_EQ(output.alpha.exponent,a.exponent);
        EXPECT_EQ(output.beta.mantissa,b.mantissa);
        EXPECT_EQ(output.beta.exponent,b.exponent);
    }
}

TEST(PiPwmIntegerRange, Q15IndependentScalarExponentGapPreservesBothComponents) {
    check_independent_exponent_hypot<fc_test::Q15>();
}
TEST(PiPwmIntegerRange, Q31IndependentScalarExponentGapPreservesBothComponents) {
    check_independent_exponent_hypot<fc_test::Q31>();
}
} // namespace
