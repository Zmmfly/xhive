#include "support.h"

namespace {
template<class T> class Vvvf : public ::testing::Test {};
TYPED_TEST_SUITE(Vvvf,fc_test::Types,);
constexpr long double two_pi=6.283185307179586476925286766559L;

TYPED_TEST(Vvvf, AdjustableFrequencyAmplitudeDirectionAndWrap) {
    using A=fc_test::Api<TypeParam>;
    typename A::vvvf state{};
    typename A::vvvf_output out{};
    const double tbase=A::fixed?0.005:1,vbase=A::fixed?32:1;
    ASSERT_EQ(A::vvvf_init(&state,A::make(0.9)),LINEARS_OK);
    double expected=fc_test::value(A::make(0.9));
    const auto dt=A::make(0.00025/tbase);
    const double h=fc_test::value(dt);
    for(int n=0;n<16000;++n) {
        const double command_hz=n<4000?n*0.01:n<8000?(8000-n)*0.01:n<12000?-(n-8000)*0.01:0;
        const auto frequency=A::make(command_hz*tbase);
        const auto amplitude=A::make((n<12000?5+3*std::sin(n*0.001):0)/vbase);
        ASSERT_EQ(A::vvvf_step(&state,frequency,amplitude,A::make(24/vbase),dt,&out),LINEARS_OK);
        expected=fc_test::wrap(expected+fc_test::value(frequency)*h);
        const double round_bound=A::fixed?0.5/2147483648.0:
            (std::is_same_v<TypeParam,fc_test::F32>?std::ldexp(1.0,-23):std::ldexp(1.0,-52));
        // Q additions/modulo are exact; only the F31 frequency*dt product rounds.
        const double phase_bound=(n+1)*round_bound+(A::fixed?0.5:2)*A::unit+1e-14;
        EXPECT_NEAR(fc_test::turn_error(fc_test::value(out.angle),expected),0,phase_bound);
        EXPECT_GE(fc_test::value(out.angle),0);EXPECT_LT(fc_test::value(out.angle),1);
        const double theta=fc_test::value(state.angle),amp=fc_test::value(amplitude);
        EXPECT_NEAR(fc_test::value(out.voltage.alpha),amp*std::cos(two_pi*theta),20*A::unit);
        EXPECT_NEAR(fc_test::value(out.voltage.beta),amp*std::sin(two_pi*theta),20*A::unit);
        EXPECT_EQ(out.saturated,0);
        EXPECT_GE(fc_test::value(out.duty.a),0);EXPECT_LE(fc_test::value(out.duty.a),1);
        if(n==15999) {
            EXPECT_DOUBLE_EQ(fc_test::value(out.voltage.alpha),0);EXPECT_DOUBLE_EQ(fc_test::value(out.voltage.beta),0);
            EXPECT_DOUBLE_EQ(fc_test::value(out.duty.a),0.5);EXPECT_DOUBLE_EQ(fc_test::value(out.duty.b),0.5);
            EXPECT_DOUBLE_EQ(fc_test::value(out.duty.c),0.5);
        }
    }
}

TYPED_TEST(Vvvf, SaturationIsRadialAndAntialiasBoundIsInclusive) {
    using A=fc_test::Api<TypeParam>;
    typename A::vvvf state{};typename A::vvvf_output out{};
    const auto frequency=A::make(A::fixed?1:512),period=A::make(A::fixed?0.5:1.0/1024);
    const auto bus=A::make(A::fixed?0.25:12);
    ASSERT_EQ(A::vvvf_init(&state,A::make(-0.25)),LINEARS_OK);
    ASSERT_EQ(A::vvvf_step(&state,frequency,A::make(A::fixed?0.75:100),bus,period,&out),LINEARS_OK);
    EXPECT_DOUBLE_EQ(fc_test::value(out.angle),0.25);
    EXPECT_EQ(out.saturated,1);
    const double radius=fc_test::value(bus)/std::sqrt(3.0),theta=fc_test::value(state.angle);
    EXPECT_NEAR(fc_test::value(out.voltage.alpha),radius*std::cos(two_pi*theta),12*A::unit);
    EXPECT_NEAR(fc_test::value(out.voltage.beta),radius*std::sin(two_pi*theta),12*A::unit);
    // Q15 f=MAX is 1-LSB, so its working angle is slightly below a quarter turn.
    EXPECT_NEAR(fc_test::value(out.duty.a),0.5+1.5*radius*std::cos(two_pi*theta)/fc_test::value(bus),8*A::unit);
    EXPECT_NEAR(fc_test::value(out.duty.b),1,2*A::unit);
    EXPECT_NEAR(fc_test::value(out.duty.c),0,2*A::unit);
    ASSERT_EQ(A::vvvf_step(&state,A::make(A::fixed?-1:-512),A::make(A::fixed?0.125:2),bus,period,&out),LINEARS_OK);
    EXPECT_DOUBLE_EQ(fc_test::value(out.angle),0.75);EXPECT_EQ(out.saturated,0);
    EXPECT_NEAR(fc_test::value(out.voltage.beta),A::fixed?-0.125:-2,2*A::unit);
}

TYPED_TEST(Vvvf, FractionalPhaseIsRetainedAndContextsAreIndependent) {
    using A=fc_test::Api<TypeParam>;
    typename A::vvvf a{},b{};typename A::vvvf_output out{};
    ASSERT_EQ(A::vvvf_init(&a,A::make(0.5)),LINEARS_OK);
    ASSERT_EQ(A::vvvf_init(&b,A::make(0.125)),LINEARS_OK);
    const double tbase=A::fixed?0.005:1,vbase=A::fixed?32:1;
    const auto dt=A::make(0.00001/tbase),frequency=A::make(tbase);
    for(int n=0;n<10000;++n) ASSERT_EQ(A::vvvf_step(&a,frequency,A::make(1/vbase),A::make(12/vbase),dt,&out),LINEARS_OK);
    EXPECT_GT(fc_test::value(a.angle),0.59);EXPECT_DOUBLE_EQ(fc_test::value(b.angle),0.125);
    ASSERT_EQ(A::vvvf_step(&b,A::make(-1),A::make(1/vbase),A::make(12/vbase),A::make(0.125),&out),LINEARS_OK);
    EXPECT_DOUBLE_EQ(fc_test::value(out.angle),0);EXPECT_GT(fc_test::value(a.angle),0.59);
    auto snapshot=a;
    ASSERT_EQ(A::vvvf_step(&a,A::make(0),A::make(2/vbase),A::make(12/vbase),dt,&out),LINEARS_OK);
    EXPECT_TRUE(fc_test::same_bytes(a,snapshot));
}

TYPED_TEST(Vvvf, InvalidSamplesPreserveStateAndResult) {
    using A=fc_test::Api<TypeParam>;
    typename A::vvvf state{};typename A::vvvf_output out{};
    const auto normal_frequency=A::make(A::fixed?0.25:1),normal_amplitude=A::make(A::fixed?0.25:2);
    const auto normal_bus=A::make(A::fixed?0.75:12),normal_dt=A::make(A::fixed?0.125:0.001);
    ASSERT_EQ(A::vvvf_init(&state,A::make(0.2)),LINEARS_OK);
    ASSERT_EQ(A::vvvf_step(&state,normal_frequency,normal_amplitude,normal_bus,normal_dt,&out),LINEARS_OK);
    auto saved=state;auto saved_out=out;
    for(int which=0;which<4;++which) {
        const auto freq=which==0?A::make(A::fixed?1:1024):normal_frequency;
        const auto amp=which==1?A::make(-1):normal_amplitude;
        const auto bus=which==2?A::make(0):normal_bus;
        const auto dt=which==3?A::make(0):(which==0?A::make(A::fixed?0.75:1.0/1024):normal_dt);
        EXPECT_EQ(A::vvvf_step(&state,freq,amp,bus,dt,&out),LINEARS_ERROR_DOMAIN);
        EXPECT_TRUE(fc_test::same_bytes(state,saved));EXPECT_TRUE(fc_test::same_bytes(out,saved_out));
    }
    if constexpr(!A::fixed) {
        const auto bad=std::numeric_limits<typename A::scalar>::quiet_NaN();
        EXPECT_EQ(A::vvvf_step(&state,normal_frequency,bad,normal_bus,normal_dt,&out),LINEARS_ERROR_RANGE);
        EXPECT_TRUE(fc_test::same_bytes(state,saved));EXPECT_TRUE(fc_test::same_bytes(out,saved_out));
        EXPECT_EQ(A::vvvf_init(&state,bad),LINEARS_ERROR_RANGE);
        EXPECT_TRUE(fc_test::same_bytes(state,saved));
    } else {
        EXPECT_EQ(A::vvvf_step(&state,normal_frequency,normal_amplitude,normal_bus,A::make(-1),&out),LINEARS_ERROR_DOMAIN);
        EXPECT_TRUE(fc_test::same_bytes(state,saved));EXPECT_TRUE(fc_test::same_bytes(out,saved_out));
    }
    EXPECT_EQ(A::vvvf_step(nullptr,normal_frequency,normal_amplitude,normal_bus,normal_dt,&out),LINEARS_ERROR_ARGUMENT);
    EXPECT_EQ(A::vvvf_step(&state,normal_frequency,normal_amplitude,normal_bus,normal_dt,
                         reinterpret_cast<typename A::vvvf_output*>(&state)),LINEARS_ERROR_ALIAS);
    EXPECT_TRUE(fc_test::same_bytes(state,saved));
}

TYPED_TEST(Vvvf, ModulationAndClippingAreInvariantUnderCommonVoltageScaling) {
    using A=fc_test::Api<TypeParam>;
    for(int scale_index=0;scale_index<4;++scale_index) {
        auto scale=[scale_index](double factor) {
            if constexpr(A::fixed) {
                const double values[]={A::unit,1.0/1024,1.0/32,1.0/8};
                return A::make(factor*values[scale_index]);
            } else {
                using S=typename A::scalar;
                const S values[]={std::numeric_limits<S>::denorm_min(),std::numeric_limits<S>::min(),S(1),std::numeric_limits<S>::max()/8};
                return static_cast<S>(factor)*values[scale_index];
            }
        };
        for(int bus_factor : {1,4}) {
            typename A::vvvf state{};typename A::vvvf_output out{};
            ASSERT_EQ(A::vvvf_init(&state,A::make(1.0/16)),LINEARS_OK);
            ASSERT_EQ(A::vvvf_step(&state,A::make(0),scale(1),scale(bus_factor),A::make(A::fixed?0.5:1),&out),LINEARS_OK);
            EXPECT_EQ(out.saturated,bus_factor==1);
            const long double magnitude=std::min(1.L/bus_factor,1/std::sqrt(3.L));
            const long double a=magnitude*std::cos(two_pi/16),b=magnitude*std::sin(two_pi/16);
            const long double phases[]={a,(-a+std::sqrt(3.L)*b)/2,(-a-std::sqrt(3.L)*b)/2};
            const long double offset=(*std::min_element(phases,phases+3)+*std::max_element(phases,phases+3))/2;
            EXPECT_NEAR(fc_test::value(out.duty.a),0.5L+phases[0]-offset,8*A::unit);
            EXPECT_NEAR(fc_test::value(out.duty.b),0.5L+phases[1]-offset,8*A::unit);
            EXPECT_NEAR(fc_test::value(out.duty.c),0.5L+phases[2]-offset,8*A::unit);
        }
    }
}

TEST(VvvfPerUnit, ReadmeQ15PhysicalCommandUsesExplicitCallerBases) {
    // Vbase=32 V, Tbase=0.005 s: 50 Hz, 8 V amplitude, 24 V bus, 100 us sample.
    fieldctrl_q15_vvvf_t state{};
    fieldctrl_q15_vvvf_output_t out{};
    ASSERT_EQ(fieldctrl_q15_vvvf_init(&state,0),LINEARS_OK);
    ASSERT_EQ(fieldctrl_q15_vvvf_step(&state,8192,8192,24576,655,&out),LINEARS_OK);
    EXPECT_EQ(state.angle,INT64_C(10731520)); // 8192/32768 * 655/32768 in F31.
    EXPECT_EQ(out.angle,164);
    EXPECT_EQ(out.saturated,0);
    EXPECT_GE(out.duty.a,0);EXPECT_GE(out.duty.b,0);EXPECT_GE(out.duty.c,0);
}
} // namespace
