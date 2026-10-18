#include "foc_smo_support.h"
#include <random>

namespace {
using namespace fc_loop_test;
template<class T> class Foc : public ::testing::Test {};
TYPED_TEST_SUITE(Foc, fc_test::Types,);
template<class T> class FixedFoc : public ::testing::Test {};
using FixedTypes=::testing::Types<fc_test::Q15,fc_test::Q31>;
TYPED_TEST_SUITE(FixedFoc, FixedTypes,);
template<class T> class FloatingFoc : public ::testing::Test {};
using FloatingTypes=::testing::Types<fc_test::F32,fc_test::F64>;
TYPED_TEST_SUITE(FloatingFoc, FloatingTypes,);

TYPED_TEST(Foc, IndependentQuantizedFormulaAndDutyReconstruction)
{
    using T = TypeParam;
    using A = Api<T>;
    const auto u=current_units<T>();
    std::mt19937 random(0x50494d53);
    std::uniform_real_distribution<double> sample(-1, 1);
    for (unsigned n = 0; n < 320; ++n) {
        SCOPED_TRACE(n);
        auto config = current_config<T>(1.8, 70, 5,u);
        config.kp_d = T::gain((1.5 + sample(random))*u.i/u.v);
        config.kp_q = T::gain((1.5 + sample(random))*u.i/u.v);
        typename A::foc ctx{};
        ASSERT_EQ(A::foc_init(&ctx, &config), LINEARS_OK);
        const typename A::ab current{u.current(3*sample(random)), u.current(3*sample(random))};
        const typename A::dq reference{u.current(5*sample(random)), u.current(5*sample(random))};
        const auto angle = angle_input<T>(sample(random));
        const auto bus = u.voltage(8 + 4*sample(random));
        const auto dt = u.time(0.002);
        typename A::foc_output out{};
        ASSERT_EQ(A::foc_step(&ctx, current, reference, angle, bus, dt, &out), LINEARS_OK);
        auto measured = park(value(current.alpha), value(current.beta), value(angle));
        measured[0] = value(T::make(measured[0]));
        measured[1] = value(T::make(measured[1]));
        const double ed = value(reference.d)-measured[0], eq = value(reference.q)-measured[1];
        const double pd = value(config.kp_d)*ed, pq = value(config.kp_q)*eq;
        double id = value(config.ki_d)*value(dt)*ed, iq = value(config.ki_q)*value(dt)*eq;
        double vd = pd+id, vq = pq+iq;
        const double radius = std::min(value(config.voltage_limit), value(bus)/std::sqrt(3.0));
        if (std::hypot(vd,vq) > radius && vd*id+vq*iq > 0) {
            id = iq = 0;
            vd = pd; vq = pq;
        }
        const double magnitude = std::hypot(vd,vq);
        const double scale = magnitude > radius ? radius/magnitude : 1;
        EXPECT_EQ(out.saturated != 0, magnitude > radius);
        EXPECT_NEAR(value(ctx.integral.d), id, numeric_budget<T>(std::abs(id)+std::abs(pd)));
        EXPECT_NEAR(value(ctx.integral.q), iq, numeric_budget<T>(std::abs(iq)+std::abs(pq)));
        EXPECT_NEAR(value(out.current.d), measured[0], numeric_budget<T>(3));
        EXPECT_NEAR(value(out.current.q), measured[1], numeric_budget<T>(3));
        EXPECT_NEAR(value(out.requested_voltage.d), vd, numeric_budget<T>(std::abs(vd)+1));
        EXPECT_NEAR(value(out.requested_voltage.q), vq, numeric_budget<T>(std::abs(vq)+1));
        EXPECT_NEAR(value(out.voltage.d), vd*scale, numeric_budget<T>(radius));
        EXPECT_NEAR(value(out.voltage.q), vq*scale, numeric_budget<T>(radius));
        auto ab = rotate(value(T::make(vd*scale)), value(T::make(vq*scale)), value(angle));
        ab[0] = value(T::make(ab[0]));
        ab[1] = value(T::make(ab[1]));
        EXPECT_NEAR(value(out.stationary_voltage.alpha),ab[0],numeric_budget<T>(radius));
        EXPECT_NEAR(value(out.stationary_voltage.beta),ab[1],numeric_budget<T>(radius));
        if constexpr (T::fixed) {
            // Validate each public quantization boundary independently. PWM
            // sees these exact raw inputs; propagating a one-LSB voltage
            // difference through a small bus requires a 1/bus gain, not a
            // relative BFP tolerance reused as an absolute duty tolerance.
            ab={value(out.stationary_voltage.alpha),value(out.stationary_voltage.beta)};
        }
        const double final_radius = std::hypot(ab[0],ab[1]);
        const double final_scale = final_radius > value(bus)/std::sqrt(3.0)
            ? value(bus)/(std::sqrt(3.0)*final_radius) : 1;
        ab[0] *= final_scale; ab[1] *= final_scale;
        const std::array<double,3> phase{ab[0], -ab[0]/2+std::sqrt(3.0)*ab[1]/2,
            -ab[0]/2-std::sqrt(3.0)*ab[1]/2};
        const double common = -(*std::max_element(phase.begin(),phase.end())+
            *std::min_element(phase.begin(),phase.end()))/2;
        const std::array<double,3> duty{value(out.duty.a),value(out.duty.b),value(out.duty.c)};
        for (size_t j = 0; j < duty.size(); ++j) {
            EXPECT_GE(duty[j],0); EXPECT_LE(duty[j],1);
            EXPECT_NEAR(duty[j], 0.5+(phase[j]+common)/value(bus), numeric_budget<T>(1));
        }
        const auto applied = applied_voltage(out.duty,value(bus));
        EXPECT_NEAR(applied[0], ab[0], numeric_budget<T>(value(bus)));
        EXPECT_NEAR(applied[1], ab[1], numeric_budget<T>(value(bus)));
    }
}

TYPED_TEST(Foc, RadialConditionalIntegrationAndSaturationRecovery)
{
    using T = TypeParam;
    using A = Api<T>;
    const Units<T> u=T::fixed?Units<T>{4,4,0.02}:Units<T>{};
    auto config = current_config<T>(1,50,1.5,u);
    typename A::foc ctx{};
    typename A::foc_output out{};
    const typename A::ab zero{T::make(0),T::make(0)};
    ASSERT_EQ(A::foc_init(&ctx,&config),LINEARS_OK);
    // Both candidate axes are individually below 1.5; only the 2D circle clips.
    ASSERT_EQ(A::foc_step(&ctx,zero,{u.current(1),u.current(1)},T::make(0),
        u.voltage(T::fixed?3.5:24),u.time(0.01),&out),LINEARS_OK);
    EXPECT_EQ(value(ctx.integral.d),0);
    EXPECT_EQ(value(ctx.integral.q),0);
    EXPECT_NEAR(value(out.voltage.d),value(u.current(1)),numeric_budget<T>(1));
    EXPECT_NEAR(value(out.voltage.q),value(u.current(1)),numeric_budget<T>(1));
    for (unsigned i = 0; i < 1000; ++i) {
        ASSERT_EQ(A::foc_step(&ctx,zero,{u.current(3),u.current(4)},T::make(0),
            u.voltage(T::fixed?3.5:24),u.time(0.01),&out),LINEARS_OK);
        ASSERT_NE(out.saturated,0);
        ASSERT_EQ(value(ctx.integral.d),0);
        ASSERT_EQ(value(ctx.integral.q),0);
    }
    ASSERT_EQ(A::foc_step(&ctx,zero,{u.current(0.2),u.current(-0.1)},T::make(0),
        u.voltage(T::fixed?3.5:24),u.time(0.01),&out),LINEARS_OK);
    EXPECT_EQ(out.saturated,0);
    const double factor = value(config.ki_d)*value(u.time(0.01));
    EXPECT_NEAR(value(ctx.integral.d),factor*value(u.current(0.2)),numeric_budget<T>(1));
    EXPECT_NEAR(value(ctx.integral.q),factor*value(u.current(-0.1)),numeric_budget<T>(1));
}

TYPED_TEST(FloatingFoc, AntiwindupPredicatesAreIndependentOfPhysicalScale)
{
    using T=TypeParam;
    using A=Api<T>;
    const int exponent=std::is_same_v<T,fc_test::F64>?600:80;
    const auto zero=T::make(0),one=T::make(1);
    for (int power : {-exponent,exponent}) {
        SCOPED_TRACE(power);
        const double radius=std::ldexp(1.0,power);
        const auto limit=T::make(radius),bus=T::make(4*radius);
        auto config=current_config<T>(0,1,radius);
        typename A::foc ctx{};
        typename A::foc_output out{};
        ASSERT_EQ(A::foc_init(&ctx,&config),LINEARS_OK);
        for (unsigned n=0;n<8;++n) {
            ASSERT_EQ(A::foc_step(&ctx,{zero,zero},{limit,zero},zero,bus,one,&out),LINEARS_OK);
            // radius^2 underflows/overflows float/double, but every input and
            // expected integral here is normal. Only a normalized sign test works.
            EXPECT_EQ(value(ctx.integral.d)/radius,1);
            EXPECT_EQ(value(ctx.integral.q),0);
        }
        for (int direction : {-1,1}) {
            typename A::speed_config cascade{current_config<T>(2,0,radius),
                zero,one,T::make(2*radius),1};
            typename A::speed speed{};
            typename A::speed_output speed_out{};
            ASSERT_EQ(A::speed_init(&speed,&cascade),LINEARS_OK);
            ASSERT_EQ(A::speed_step(&speed,{zero,zero},T::make(direction*radius),
                zero,zero,bus,one,&speed_out),LINEARS_OK);
            ASSERT_NE(speed_out.current.saturated,0);
            EXPECT_EQ(value(speed.speed.integral),0);
        }
    }
}

TYPED_TEST(FloatingFoc, ExtremeRatioLimitingPreservesRepresentableVoltage)
{
    using T=TypeParam;
    using A=Api<T>;
    const int power=std::is_same_v<T,fc_test::F64>?600:80;
    const double radius=std::ldexp(1.0,-power);
    auto config=current_config<T>(1,0,radius);
    typename A::foc ctx{};
    typename A::foc_output out{};
    const auto zero=T::make(0);
    ASSERT_EQ(A::foc_init(&ctx,&config),LINEARS_OK);
    ASSERT_EQ(A::foc_step(&ctx,{zero,zero},{T::make(std::ldexp(1.0,power)),zero},
        zero,T::make(4*radius),T::make(1),&out),LINEARS_OK);
    ASSERT_NE(out.saturated,0);
    EXPECT_EQ(value(out.voltage.d)/radius,1);
    EXPECT_EQ(value(out.voltage.q),0);
    EXPECT_EQ(value(out.duty.a),0.6875);
    EXPECT_EQ(value(out.duty.b),0.3125);
    EXPECT_EQ(value(out.duty.c),0.3125);
    // The opposite ordering (component/norm)*limit also loses a small axis
    // when norm and limit are huge. A scaled product/ratio must retain it.
    const double large=std::ldexp(1.0,power);
    config=current_config<T>(1,0,large/2);
    ASSERT_EQ(A::foc_init(&ctx,&config),LINEARS_OK);
    ASSERT_EQ(A::foc_step(&ctx,{zero,zero},{T::make(radius),T::make(large)},
        zero,T::make(2*large),T::make(1),&out),LINEARS_OK);
    EXPECT_EQ(value(out.voltage.d)/radius,0.5);
    EXPECT_EQ(value(out.voltage.q)/large,0.5);
}

TYPED_TEST(FixedFoc, WideCoefficientsAndRequestedVoltageAreNotQ1Signals)
{
    using T=TypeParam;
    using A=Api<T>;
    typename A::foc_config config{T::gain(4),T::gain(0),T::gain(4),T::gain(0),T::make(0.25)};
    typename A::foc ctx{};
    typename A::foc_output out{};
    static_assert(std::is_same_v<decltype(config.kp_d),linears_fixed_t>);
    static_assert(std::is_same_v<decltype(ctx.integral),fieldctrl_fixed_dq_t>);
    static_assert(std::is_same_v<decltype(out.requested_voltage),fieldctrl_fixed_dq_t>);
    const auto zero=T::make(0),reference=T::make(0.8);
    ASSERT_EQ(A::foc_init(&ctx,&config),LINEARS_OK);
    ASSERT_EQ(A::foc_step(&ctx,{zero,zero},{reference,zero},zero,
        T::make(0.75),T::make(0.125),&out),LINEARS_OK);
    EXPECT_EQ(value(out.requested_voltage.d),4*value(reference));
    EXPECT_GT(value(out.requested_voltage.d),1);
    EXPECT_EQ(value(out.voltage.d),0.25);
    EXPECT_EQ(value(out.duty.a),0.75);
    // ONE=2147483648. Normalize alpha: round(ONE*0.25/0.75)=715827883.
    // beta=0, so phase b=c=round-away(-715827883/2)=-357913942.
    // Common mode is round-away((715827883-357913942)/2)=178956971.
    // Duty b=c=ONE/2-357913942-178956971=536870911; duty a=1610612736.
    // Q15 export rounds 536870911/65536 to 8192, exactly 0.25; F31 keeps it.
    const auto expected_b=T::make(0.25)-(std::is_same_v<T,fc_test::Q31>?1:0);
    EXPECT_EQ(out.duty.b,expected_b);
    EXPECT_EQ(out.duty.c,expected_b);
}

TYPED_TEST(FixedFoc, RawEndpointsAndOutOfRangeRotatedCurrent)
{
    using T=TypeParam;
    using A=Api<T>;
    typename A::foc_config config{0,0,0,0,T::make(0.25)};
    typename A::foc ctx{};
    typename A::foc_output out{};
    const auto zero=T::make(0);
    ASSERT_EQ(A::foc_init(&ctx,&config),LINEARS_OK);
    for (auto endpoint : {std::numeric_limits<typename T::scalar>::min(),
                          std::numeric_limits<typename T::scalar>::max()}) {
        ASSERT_EQ(A::foc_step(&ctx,{endpoint,zero},{zero,zero},zero,
            T::make(0.75),T::make(0.125),&out),LINEARS_OK);
        EXPECT_EQ(out.current.d,endpoint);
    }
    const auto old=ctx;
    const auto previous=out;
    EXPECT_EQ(A::foc_step(&ctx,{T::make(0.75),T::make(0.75)},{zero,zero},
        T::make(0.125),T::make(0.75),T::make(0.125),&out),LINEARS_ERROR_RANGE);
    EXPECT_TRUE(fc_test::same_bytes(ctx,old));
    EXPECT_TRUE(fc_test::same_bytes(out,previous));
}

TYPED_TEST(FixedFoc, WorkingOverflowPreservesAllStateAndOutput)
{
    using T=TypeParam;
    using A=Api<T>;
    typename A::foc_config config{INT64_MAX,INT64_MAX,0,0,T::make(0.25)};
    typename A::foc ctx{};
    typename A::foc_output out{};
    const auto zero=T::make(0);
    ASSERT_EQ(A::foc_init(&ctx,&config),LINEARS_OK);
    const auto old=ctx;
    const auto previous=out;
    EXPECT_EQ(A::foc_step(&ctx,{zero,zero},{T::make(0.75),zero},zero,
        T::make(0.75),T::make(0.75),&out),LINEARS_ERROR_RANGE);
    EXPECT_TRUE(fc_test::same_bytes(ctx,old));
    EXPECT_TRUE(fc_test::same_bytes(out,previous));
}

TYPED_TEST(FixedFoc, WideSpeedDifferenceHandlesOppositeValidQ1Inputs)
{
    using T=TypeParam;
    using A=Api<T>;
    const auto zero=T::make(0);
    typename A::speed_config config{{T::gain(0.25),0,T::gain(0.25),0,T::make(0.25)},
        T::gain(0.25),0,T::make(0.5),1};
    for (int direction : {-1,1}) {
        typename A::speed ctx{};
        typename A::speed_output out{};
        ASSERT_EQ(A::speed_init(&ctx,&config),LINEARS_OK);
        ASSERT_EQ(A::speed_step(&ctx,{zero,zero},T::make(direction*0.75),
            T::make(-direction*0.75),zero,T::make(0.75),T::make(0.125),&out),LINEARS_OK);
        EXPECT_EQ(value(out.reference.q),direction*0.375);
        EXPECT_EQ(value(out.mechanical_speed),-direction*0.75);
    }
}

TYPED_TEST(FixedFoc, ExactF62AntiwindupRetainsTinyPositiveProductSign)
{
    using T=TypeParam;
    using A=Api<T>;
    const auto zero=T::make(0),radius=T::make(1.0/32768);
    typename A::foc_config config{0,T::gain(1.0/32768),0,0,radius};
    for (int direction : {-1,1}) {
        typename A::foc ctx{};
        typename A::foc_output out{};
        ASSERT_EQ(A::foc_init(&ctx,&config),LINEARS_OK);
        for (unsigned n=0;n<2;++n)
            ASSERT_EQ(A::foc_step(&ctx,{zero,zero},{T::make(direction),zero},zero,
                T::make(1.0/4096),T::make(0.5),&out),LINEARS_OK);
        for (unsigned n=0;n<8;++n)
            ASSERT_EQ(A::foc_step(&ctx,{zero,zero},{T::make(direction/32768.0),zero},zero,
                T::make(1.0/4096),T::make(0.5),&out),LINEARS_OK);
        // DeltaI is one F31 raw unit; DeltaI*v is nonzero F62 but rounds to
        // zero in F31. The integral must stop at the exact configured radius.
        EXPECT_EQ(ctx.integral.d,direction*INT64_C(65536));
    }
}

TYPED_TEST(FixedFoc, SubQ15StepsAccumulateAndF31StepsUseTheirDeclaredRounding)
{
    using T=TypeParam;
    using A=Api<T>;
    const auto zero=T::make(0);
    typename A::foc_config config{0,T::gain(1),0,0,T::make(0.75)};
    typename A::foc ctx{};
    typename A::foc_output out{};
    ASSERT_EQ(A::foc_init(&ctx,&config),LINEARS_OK);
    ASSERT_EQ(A::foc_step(&ctx,{zero,zero},{T::make(0.5),zero},zero,
        T::make(0.875),T::make(0.5),&out),LINEARS_OK);
    const typename T::scalar period=1;
    for (unsigned n=0;n<8192;++n)
        ASSERT_EQ(A::foc_step(&ctx,{zero,zero},{T::make(0.125),zero},zero,
            T::make(0.875),period,&out),LINEARS_OK);
    const int64_t rounded_increment=std::is_same_v<T,fc_test::Q15>?8192:0;
    EXPECT_EQ(ctx.integral.d,INT64_C(536870912)+8192*rounded_increment);
    const double ideal=0.25+8192*value(period)*0.125;
    EXPECT_LE(std::abs(value(ctx.integral.d)-ideal),8192.0/4294967296.0);
    if constexpr (std::is_same_v<T,fc_test::Q15>) {
        EXPECT_GT(value(out.voltage.d),0.28);
    }
}

TYPED_TEST(FloatingFoc, WideIntegralRetainsSmallSampleIncrements)
{
    using T = TypeParam;
    using A = Api<T>;
    auto config = current_config<T>(0,1,8);
    typename A::foc ctx{};
    typename A::foc_output out{};
    ASSERT_EQ(A::foc_init(&ctx,&config),LINEARS_OK);
    const typename A::ab zero{T::make(0),T::make(0)};
    ASSERT_EQ(A::foc_step(&ctx,zero,{T::make(1),T::make(0)},T::make(0),
        T::make(24),T::make(1),&out),LINEARS_OK);
    const auto dt = T::make(0.000001);
    for (unsigned n=0;n<10000;++n)
        ASSERT_EQ(A::foc_step(&ctx,zero,{T::make(1),T::make(0)},T::make(0),
            T::make(24),dt,&out),LINEARS_OK);
    // q15 state rounded each call would stay exactly at 1 and fail by 0.01.
    const double working_unit = T::fixed ? fc_test::Q31::unit : T::unit;
    EXPECT_NEAR(value(ctx.integral.d),1+10000*value(dt),10000*working_unit);
    EXPECT_GT(value(ctx.integral.d),1.009);
}

TYPED_TEST(Foc, SensoredCurrentLoopMotorModelBothDirectionsAndBusRecovery)
{
    using T = TypeParam;
    using A = Api<T>;
    const auto u=current_units<T>();
    const auto period = u.time(0.0001);
    const double dt = u.seconds(period);
    for (int direction : {-1,1}) {
        SCOPED_TRACE(direction);
        auto config = current_config<T>();
        typename A::foc ctx{};
        typename A::foc_output out{};
        Motor motor;
        ASSERT_EQ(A::foc_init(&ctx,&config),LINEARS_OK);
        double squared_d = 0, squared_q = 0;
        unsigned saturated = 0;
        for (unsigned n=0;n<18000;++n) {
            const double frequency = direction*(n<4000 ? 15+15.0*n/4000 : 30);
            const bool overload = n>=7000 && n<10000;
            const auto bus = u.voltage(overload?5:24);
            const double target_q = direction*(overload?18:(n<10000?2:1.5));
            ASSERT_EQ(A::foc_step(&ctx,{u.current(motor.alpha),u.current(motor.beta)},
                {u.current(0.3),u.current(target_q)},angle_input<T>(motor.angle),bus,period,&out),LINEARS_OK);
            saturated += out.saturated != 0;
            const auto voltage = applied_voltage(out.duty,u.volts(bus));
            motor.electrical_step(voltage,frequency,dt);
            if (n>=17000) {
                const auto dq=park(motor.alpha,motor.beta,motor.angle);
                squared_d += (dq[0]-u.amps(u.current(0.3)))*(dq[0]-u.amps(u.current(0.3)));
                squared_q += (dq[1]-u.amps(u.current(target_q)))*(dq[1]-u.amps(u.current(target_q)));
            }
        }
        EXPECT_GT(saturated,1000u);
        // A physical tracking budget, separate from arithmetic-oracle tolerances.
        EXPECT_LT(std::sqrt(squared_d/1000),0.008);
        EXPECT_LT(std::sqrt(squared_q/1000),0.008);
    }
}

TYPED_TEST(Foc, MechanicalElectricalUnitsAndCascadedMotorReversal)
{
    using T = TypeParam;
    using A = Api<T>;
    const auto u=speed_units<T>();
    typename A::speed_config config{current_config<T>(3,400,13,u),
        T::gain(1.5/(u.i*u.t)),T::gain(6/u.i),u.current(6),3};
    typename A::speed ctx{};
    typename A::speed_output out{};
    ASSERT_EQ(A::speed_init(&ctx,&config),LINEARS_OK);
    const auto period=u.time(0.0002);
    const double dt=u.seconds(period);
    Motor motor;
    motor.resistance=0.4; motor.inductance=0.004; motor.flux=0.025;
    double final_error=0;
    unsigned voltage_saturation=0;
    for (unsigned n=0;n<24000;++n) {
        const double request=n<10000?6:-4;
        const auto bus=u.voltage(n>=6000 && n<8000?5:24);
        const auto electrical=u.frequency(config.pole_pairs*motor.mechanical_speed);
        ASSERT_EQ(A::speed_step(&ctx,{u.current(motor.alpha),u.current(motor.beta)},
            u.frequency(request),electrical,angle_input<T>(motor.angle),bus,period,&out),LINEARS_OK);
        EXPECT_NEAR(value(out.mechanical_speed),value(electrical)/config.pole_pairs,
            numeric_budget<T>(std::abs(motor.mechanical_speed)*u.t));
        EXPECT_LE(std::abs(value(out.reference.q)),value(config.current_limit));
        EXPECT_EQ(value(out.reference.d),0);
        voltage_saturation += out.current.saturated != 0;
        const auto dq=park(motor.alpha,motor.beta,motor.angle);
        const double torque=1.5*config.pole_pairs*motor.flux*dq[1];
        const double acceleration=(torque-0.001*tau*motor.mechanical_speed)/(tau*0.0015);
        motor.electrical_step(applied_voltage(out.current.duty,u.volts(bus)),
            config.pole_pairs*motor.mechanical_speed,dt);
        motor.mechanical_speed += dt*acceleration;
        if (n>=23000) final_error += std::abs(motor.mechanical_speed-request);
    }
    EXPECT_GT(voltage_saturation,100u);
    EXPECT_LT(final_error/1000,0.015);
    EXPECT_LT(motor.mechanical_speed,0);
}

TYPED_TEST(Foc, FailureIsAtomicIncludingNestedSpeedStep)
{
    using T = TypeParam;
    using A = Api<T>;
    const auto u=current_units<T>();
    const auto zero=T::make(0), one=T::make(1);
    auto config=current_config<T>();
    typename A::foc ctx{};
    typename A::foc_output out{};
    ASSERT_EQ(A::foc_init(&ctx,&config),LINEARS_OK);
    ASSERT_EQ(A::foc_step(&ctx,{zero,zero},{one,one},zero,u.voltage(24),u.time(0.001),&out),LINEARS_OK);
    const auto saved=ctx;
    const auto saved_output=out;
    for (unsigned failure=0;failure<4;++failure) {
        auto dt=u.time(0.001),bus=u.voltage(24),angle=zero;
        typename A::dq reference{one,one};
        typename A::ab measured{zero,zero};
        if (failure==0) dt=zero;
        if (failure==1) bus=zero;
        if (failure==2) angle=T::fixed?T::make(-0.125):one;
        if (failure==3) {
            if constexpr (T::fixed) { measured={T::make(0.75),T::make(0.75)}; angle=T::make(0.125); }
            else reference.q=invalid_scalar<T>();
        }
        EXPECT_NE(A::foc_step(&ctx,measured,reference,angle,bus,dt,&out),LINEARS_OK);
        EXPECT_TRUE(fc_test::same_bytes(ctx,saved));
        EXPECT_TRUE(fc_test::same_bytes(out,saved_output));
    }
    EXPECT_EQ(A::foc_step(&ctx,{zero,zero},{one,one},zero,u.voltage(24),one,
        reinterpret_cast<typename A::foc_output*>(&ctx)),LINEARS_ERROR_ALIAS);
    EXPECT_TRUE(fc_test::same_bytes(ctx,saved));
    config.ki_q=T::gain(-1);
    EXPECT_EQ(A::foc_init(&ctx,&config),LINEARS_ERROR_DOMAIN);
    EXPECT_TRUE(fc_test::same_bytes(ctx,saved));
    typename A::speed_config cascade{current_config<T>(),T::gain(1),T::gain(1),u.current(3),4};
    typename A::speed speed{};
    typename A::speed_output speed_out{};
    ASSERT_EQ(A::speed_init(&speed,&cascade),LINEARS_OK);
    const auto speed_saved=speed;
    const auto speed_out_saved=speed_out;
    EXPECT_EQ(A::speed_step(&speed,{zero,zero},u.frequency(10),zero,T::fixed?T::make(-0.125):one,u.voltage(24),
        u.time(0.001),&speed_out),LINEARS_ERROR_DOMAIN);
    EXPECT_TRUE(fc_test::same_bytes(speed,speed_saved));
    EXPECT_TRUE(fc_test::same_bytes(speed_out,speed_out_saved));
}

TYPED_TEST(Foc, InterleavedContextsMatchIndependentRuns)
{
    using T=TypeParam;
    using A=Api<T>;
    const auto u=current_units<T>();
    auto config=current_config<T>(1,5,7,u);
    typename A::foc first{},second{};
    ASSERT_EQ(A::foc_init(&first,&config),LINEARS_OK);
    ASSERT_EQ(A::foc_init(&second,&config),LINEARS_OK);
    auto first_alone=first, second_alone=second;
    typename A::foc_output a{},b{},aa{},bb{};
    for (unsigned n=0;n<500;++n) {
        const typename A::ab measured{u.current(std::sin(n*0.03)),u.current(std::cos(n*0.03))};
        const auto angle=angle_input<T>(n*0.017);
        ASSERT_EQ(A::foc_step(&first,measured,{u.current(0.1),u.current(0.3)},angle,
            u.voltage(24),u.time(0.001),&a),LINEARS_OK);
        ASSERT_EQ(A::foc_step(&second,measured,{u.current(-0.8),u.current(0.2)},angle,
            u.voltage(12),u.time(0.002),&b),LINEARS_OK);
    }
    for (unsigned n=0;n<500;++n) {
        const typename A::ab measured{u.current(std::sin(n*0.03)),u.current(std::cos(n*0.03))};
        ASSERT_EQ(A::foc_step(&first_alone,measured,{u.current(0.1),u.current(0.3)},
            angle_input<T>(n*0.017),u.voltage(24),u.time(0.001),&aa),LINEARS_OK);
    }
    for (unsigned n=0;n<500;++n) {
        const typename A::ab measured{u.current(std::sin(n*0.03)),u.current(std::cos(n*0.03))};
        ASSERT_EQ(A::foc_step(&second_alone,measured,{u.current(-0.8),u.current(0.2)},
            angle_input<T>(n*0.017),u.voltage(12),u.time(0.002),&bb),LINEARS_OK);
    }
    EXPECT_EQ(value(first.integral.d),value(first_alone.integral.d));
    EXPECT_EQ(value(first.integral.q),value(first_alone.integral.q));
    EXPECT_EQ(value(second.integral.d),value(second_alone.integral.d));
    EXPECT_EQ(value(second.integral.q),value(second_alone.integral.q));
    EXPECT_EQ(value(a.duty.a),value(aa.duty.a));
    EXPECT_EQ(value(b.duty.b),value(bb.duty.b));
}
} // namespace
