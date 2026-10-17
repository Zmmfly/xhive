#include "support.h"

namespace {
template<class T> class Vvvf : public ::testing::Test {};
TYPED_TEST_SUITE(Vvvf,fc_test::Types);
constexpr long double two_pi=6.283185307179586476925286766559L;

TYPED_TEST(Vvvf, AdjustableFrequencyAmplitudeDirectionAndWrap) {
    using A=fc_test::Api<TypeParam>;
    typename A::vvvf state{};
    typename A::vvvf_output out{};
    ASSERT_EQ(A::vvvf_init(&state,A::make(0.9)),LINEARS_OK);
    double expected=fc_test::value(A::make(0.9));
    const auto dt=A::make(0.00025);
    const double h=fc_test::value(dt);
    for(int n=0;n<16000;++n) {
        // Ramp positive, decelerate, reverse, then hold zero frequency.
        const double command=n<4000?n*0.01:n<8000?(8000-n)*0.01:n<12000?-(n-8000)*0.01:0;
        const auto frequency=A::make(command),amplitude=A::make(n<12000?5+3*std::sin(n*0.001):0);
        ASSERT_EQ(A::vvvf_step(&state,frequency,amplitude,A::make(24),dt,&out),LINEARS_OK);
        expected=fc_test::wrap(expected+fc_test::value(frequency)*h);
        const double work_ulp=std::is_same_v<TypeParam,fc_test::F32>?std::ldexp(1.0,-23):
                              std::is_same_v<TypeParam,fc_test::F64>?std::ldexp(1.0,-52):std::ldexp(1.0,-30);
        // One phase addition and wrapping per step, with <=one work ULP bound.
        const double phase_bound=(n+1)*work_ulp+2*A::unit;
        EXPECT_NEAR(fc_test::turn_error(fc_test::value(out.angle),expected),0,phase_bound);
        EXPECT_GE(fc_test::value(out.angle),0);EXPECT_LT(fc_test::value(out.angle),1);
        const double theta=fc_test::value(state.angle),amp=fc_test::value(amplitude);
        EXPECT_NEAR(fc_test::value(out.voltage.alpha),amp*std::cos(two_pi*theta),20*A::unit);
        EXPECT_NEAR(fc_test::value(out.voltage.beta),amp*std::sin(two_pi*theta),20*A::unit);
        EXPECT_EQ(out.saturated,0);
        EXPECT_GE(fc_test::value(out.duty.a),0);EXPECT_LE(fc_test::value(out.duty.a),1);
        if(n==15999) {
            EXPECT_DOUBLE_EQ(fc_test::value(out.voltage.alpha),0);
            EXPECT_DOUBLE_EQ(fc_test::value(out.voltage.beta),0);
            EXPECT_DOUBLE_EQ(fc_test::value(out.duty.a),0.5);
            EXPECT_DOUBLE_EQ(fc_test::value(out.duty.b),0.5);
            EXPECT_DOUBLE_EQ(fc_test::value(out.duty.c),0.5);
        }
    }
}

TYPED_TEST(Vvvf, SaturationIsRadialAndAntialiasBoundIsInclusive) {
    using A=fc_test::Api<TypeParam>;
    typename A::vvvf state{};
    typename A::vvvf_output out{};
    ASSERT_EQ(A::vvvf_init(&state,A::make(-0.25)),LINEARS_OK);
    ASSERT_EQ(A::vvvf_step(&state,A::make(512),A::make(100),A::make(12),A::make(1.0/1024),&out),LINEARS_OK);
    EXPECT_DOUBLE_EQ(fc_test::value(out.angle),0.25);
    EXPECT_EQ(out.saturated,1);
    EXPECT_NEAR(fc_test::value(out.voltage.alpha),0,2*A::unit);
    EXPECT_NEAR(fc_test::value(out.voltage.beta),12/std::sqrt(3.0),12*A::unit);
    EXPECT_NEAR(fc_test::value(out.duty.a),0.5,2*A::unit);
    EXPECT_NEAR(fc_test::value(out.duty.b),1,2*A::unit);
    EXPECT_NEAR(fc_test::value(out.duty.c),0,2*A::unit);
    ASSERT_EQ(A::vvvf_step(&state,A::make(-512),A::make(2),A::make(12),A::make(1.0/1024),&out),LINEARS_OK);
    EXPECT_DOUBLE_EQ(fc_test::value(out.angle),0.75);
    EXPECT_EQ(out.saturated,0);
    EXPECT_NEAR(fc_test::value(out.voltage.beta),-2,2*A::unit);
}

TYPED_TEST(Vvvf, FractionalPhaseIsRetainedAndContextsAreIndependent) {
    using A=fc_test::Api<TypeParam>;
    typename A::vvvf a{},b{};
    typename A::vvvf_output out{};
    ASSERT_EQ(A::vvvf_init(&a,A::make(0.5)),LINEARS_OK);
    ASSERT_EQ(A::vvvf_init(&b,A::make(0.125)),LINEARS_OK);
    const auto dt=A::make(0.00001);
    for(int n=0;n<10000;++n) ASSERT_EQ(A::vvvf_step(&a,A::make(1),A::make(1),A::make(12),dt,&out),LINEARS_OK);
    EXPECT_GT(fc_test::value(a.angle),0.59);
    EXPECT_DOUBLE_EQ(fc_test::value(b.angle),0.125);
    ASSERT_EQ(A::vvvf_step(&b,A::make(-1),A::make(1),A::make(12),A::make(0.125),&out),LINEARS_OK);
    EXPECT_DOUBLE_EQ(fc_test::value(out.angle),0);
    EXPECT_GT(fc_test::value(a.angle),0.59);
    auto snapshot=a;
    ASSERT_EQ(A::vvvf_step(&a,A::make(0),A::make(2),A::make(12),dt,&out),LINEARS_OK);
    EXPECT_TRUE(fc_test::same_bytes(a,snapshot));
}

TYPED_TEST(Vvvf, InvalidSamplesPreserveStateAndResult) {
    using A=fc_test::Api<TypeParam>;
    typename A::vvvf state{};
    typename A::vvvf_output out{};
    ASSERT_EQ(A::vvvf_init(&state,A::make(0.2)),LINEARS_OK);
    ASSERT_EQ(A::vvvf_step(&state,A::make(1),A::make(2),A::make(12),A::make(0.001),&out),LINEARS_OK);
    auto saved=state;auto saved_out=out;
    for(int which=0;which<4;++which) {
        const auto freq=A::make(which==0?1024:1),amp=A::make(which==1?-1:2);
        const auto bus=A::make(which==2?0:12),dt=A::make(which==3?0:1.0/1024);
        EXPECT_EQ(A::vvvf_step(&state,freq,amp,bus,dt,&out),LINEARS_ERROR_DOMAIN);
        EXPECT_TRUE(fc_test::same_bytes(state,saved));EXPECT_TRUE(fc_test::same_bytes(out,saved_out));
    }
    auto bad=A::make(0);
    if constexpr(A::fixed) bad.exponent=LINEARS_EXPONENT_MIN-1;
    else bad=std::numeric_limits<typename A::scalar>::quiet_NaN();
    EXPECT_EQ(A::vvvf_step(&state,A::make(1),bad,A::make(12),A::make(0.001),&out),LINEARS_ERROR_RANGE);
    EXPECT_TRUE(fc_test::same_bytes(state,saved));EXPECT_TRUE(fc_test::same_bytes(out,saved_out));
    EXPECT_EQ(A::vvvf_init(&state,bad),LINEARS_ERROR_RANGE);
    EXPECT_TRUE(fc_test::same_bytes(state,saved));
    EXPECT_EQ(A::vvvf_step(nullptr,A::make(1),A::make(2),A::make(12),A::make(0.001),&out),LINEARS_ERROR_ARGUMENT);
    // The pointer is never dereferenced: overlap is rejected at the boundary.
    EXPECT_EQ(A::vvvf_step(&state,A::make(1),A::make(2),A::make(12),A::make(0.001),
                         reinterpret_cast<typename A::vvvf_output *>(&state)),LINEARS_ERROR_ALIAS);
    EXPECT_TRUE(fc_test::same_bytes(state,saved));
}
TYPED_TEST(Vvvf, ModulationAndClippingAreInvariantUnderCommonVoltageScaling) {
    using A=fc_test::Api<TypeParam>;
    for(int scale_index=0;scale_index<4;++scale_index) {
        auto scale=[scale_index](double factor) {
            auto v=A::make(factor);
            if constexpr(A::fixed) {
                const int powers[]={-1000,-100,0,1000};
                v.exponent+=powers[scale_index];
            } else {
                using S=typename A::scalar;
                const S scales[]={std::numeric_limits<S>::denorm_min(),
                    std::numeric_limits<S>::min(),S(1),std::numeric_limits<S>::max()/8};
                v=static_cast<S>(factor)*scales[scale_index];
            }
            return v;
        };
        for(int bus_factor : {1,4}) {
            typename A::vvvf state{};
            typename A::vvvf_output output{};
            ASSERT_EQ(A::vvvf_init(&state,A::make(1.0/16)),LINEARS_OK);
            ASSERT_EQ(A::vvvf_step(&state,A::make(0),scale(1),scale(bus_factor),A::make(1),&output),LINEARS_OK);
            EXPECT_EQ(output.saturated,bus_factor==1);
            const long double magnitude=std::min(1.L/bus_factor,1/std::sqrt(3.L));
            const long double a=magnitude*std::cos(two_pi/16),b=magnitude*std::sin(two_pi/16);
            const long double phases[]={a,(-a+std::sqrt(3.L)*b)/2,(-a-std::sqrt(3.L)*b)/2};
            const long double offset=(*std::min_element(phases,phases+3)+*std::max_element(phases,phases+3))/2;
            EXPECT_NEAR(fc_test::value(output.duty.a),0.5L+phases[0]-offset,8*A::unit);
            EXPECT_NEAR(fc_test::value(output.duty.b),0.5L+phases[1]-offset,8*A::unit);
            EXPECT_NEAR(fc_test::value(output.duty.c),0.5L+phases[2]-offset,8*A::unit);
        }
    }
}
} // namespace
