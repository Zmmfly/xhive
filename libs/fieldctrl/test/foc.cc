#include "foc_smo_support.h"
#include <random>

namespace {
using namespace fc_loop_test;
template<class T> class Foc : public ::testing::Test {};
TYPED_TEST_SUITE(Foc, fc_test::Types,);
template<class T> class FixedFoc : public ::testing::Test {};
using FixedTypes=::testing::Types<fc_test::Q15,fc_test::Q31>;
TYPED_TEST_SUITE(FixedFoc, FixedTypes,);

TYPED_TEST(Foc, IndependentQuantizedFormulaAndDutyReconstruction)
{
    using T = TypeParam;
    using A = Api<T>;
    std::mt19937 random(0x50494d53);
    std::uniform_real_distribution<double> sample(-1, 1);
    for (unsigned n = 0; n < 320; ++n) {
        SCOPED_TRACE(n);
        auto config = current_config<T>(1.8, 70, 5);
        config.kp_d = T::make(1.5 + sample(random));
        config.kp_q = T::make(1.5 + sample(random));
        typename A::foc ctx{};
        ASSERT_EQ(A::foc_init(&ctx, &config), LINEARS_OK);
        const typename A::ab current{T::make(3*sample(random)), T::make(3*sample(random))};
        const typename A::dq reference{T::make(5*sample(random)), T::make(5*sample(random))};
        const auto angle = angle_input<T>(sample(random));
        const auto bus = T::make(8 + 4*sample(random));
        const auto dt = T::make(0.002);
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
    auto config = current_config<T>(1,50,1.5);
    typename A::foc ctx{};
    typename A::foc_output out{};
    const typename A::ab zero{T::make(0),T::make(0)};
    ASSERT_EQ(A::foc_init(&ctx,&config),LINEARS_OK);
    // Both candidate axes are individually below 1.5; only the 2D circle clips.
    ASSERT_EQ(A::foc_step(&ctx,zero,{T::make(1),T::make(1)},T::make(0),
        T::make(24),T::make(0.01),&out),LINEARS_OK);
    EXPECT_EQ(value(ctx.integral.d),0);
    EXPECT_EQ(value(ctx.integral.q),0);
    EXPECT_NEAR(value(out.voltage.d),1,numeric_budget<T>(1));
    EXPECT_NEAR(value(out.voltage.q),1,numeric_budget<T>(1));
    for (unsigned i = 0; i < 1000; ++i) {
        ASSERT_EQ(A::foc_step(&ctx,zero,{T::make(3),T::make(4)},T::make(0),
            T::make(24),T::make(0.01),&out),LINEARS_OK);
        ASSERT_NE(out.saturated,0);
        ASSERT_EQ(value(ctx.integral.d),0);
        ASSERT_EQ(value(ctx.integral.q),0);
    }
    ASSERT_EQ(A::foc_step(&ctx,zero,{T::make(0.2),T::make(-0.1)},T::make(0),
        T::make(24),T::make(0.01),&out),LINEARS_OK);
    EXPECT_EQ(out.saturated,0);
    const double factor = value(config.ki_d)*value(T::make(0.01));
    EXPECT_NEAR(value(ctx.integral.d),factor*value(T::make(0.2)),numeric_budget<T>(1));
    EXPECT_NEAR(value(ctx.integral.q),factor*value(T::make(-0.1)),numeric_budget<T>(1));
}

TYPED_TEST(Foc, AntiwindupPredicatesAreIndependentOfPhysicalScale)
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

TYPED_TEST(Foc, ExtremeRatioLimitingPreservesRepresentableVoltage)
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

TYPED_TEST(FixedFoc, FixedLimiterSpansBothPublicExponentExtremes)
{
    using T=TypeParam;
    using A=Api<T>;
    typename T::scalar low{};
    if constexpr (std::is_same_v<T,fc_test::Q15>) low.mantissa=16384;
    else low.mantissa=1073741824;
    low.exponent=LINEARS_EXPONENT_MIN;
    auto high=low,bus=low;
    high.exponent=LINEARS_EXPONENT_MAX;
    bus.exponent=LINEARS_EXPONENT_MIN+2;
    auto config=current_config<T>(1,0,1);
    config.voltage_limit=low;
    typename A::foc ctx{};
    typename A::foc_output out{};
    const auto zero=T::make(0);
    ASSERT_EQ(A::foc_init(&ctx,&config),LINEARS_OK);
    ASSERT_EQ(A::foc_step(&ctx,{zero,zero},{high,zero},zero,bus,T::make(1),&out),LINEARS_OK);
    EXPECT_EQ(out.voltage.d.mantissa,low.mantissa);
    EXPECT_EQ(out.voltage.d.exponent,low.exponent);
    EXPECT_EQ(out.voltage.q.mantissa,0);
    EXPECT_EQ(value(out.duty.a),0.6875);
    EXPECT_EQ(value(out.duty.b),0.3125);
}

TYPED_TEST(Foc, WideIntegralRetainsSmallSampleIncrements)
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
    const auto period = T::make(0.0001);
    const double dt = value(period);
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
            const auto bus = T::make(overload?5:24);
            const double target_q = direction*(overload?18:(n<10000?2:1.5));
            ASSERT_EQ(A::foc_step(&ctx,{T::make(motor.alpha),T::make(motor.beta)},
                {T::make(0.3),T::make(target_q)},angle_input<T>(motor.angle),bus,period,&out),LINEARS_OK);
            saturated += out.saturated != 0;
            const auto voltage = applied_voltage(out.duty,value(bus));
            motor.electrical_step(voltage,frequency,dt);
            if (n>=17000) {
                const auto dq=park(motor.alpha,motor.beta,motor.angle);
                squared_d += (dq[0]-value(T::make(0.3)))*(dq[0]-value(T::make(0.3)));
                squared_q += (dq[1]-value(T::make(target_q)))*(dq[1]-value(T::make(target_q)));
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
    typename A::speed_config config{current_config<T>(3,400,13),
        T::make(1.5),T::make(6),T::make(6),3};
    typename A::speed ctx{};
    typename A::speed_output out{};
    ASSERT_EQ(A::speed_init(&ctx,&config),LINEARS_OK);
    const auto period=T::make(0.0002);
    const double dt=value(period);
    Motor motor;
    motor.resistance=0.4; motor.inductance=0.004; motor.flux=0.025;
    double final_error=0;
    unsigned voltage_saturation=0;
    for (unsigned n=0;n<24000;++n) {
        const double request=n<10000?6:-4;
        const auto bus=T::make(n>=6000 && n<8000?5:24);
        const auto electrical=T::make(config.pole_pairs*motor.mechanical_speed);
        ASSERT_EQ(A::speed_step(&ctx,{T::make(motor.alpha),T::make(motor.beta)},
            T::make(request),electrical,angle_input<T>(motor.angle),bus,period,&out),LINEARS_OK);
        EXPECT_NEAR(value(out.mechanical_speed),value(electrical)/config.pole_pairs,
            numeric_budget<T>(std::abs(motor.mechanical_speed)));
        EXPECT_LE(std::abs(value(out.reference.q)),value(config.current_limit));
        EXPECT_EQ(value(out.reference.d),0);
        voltage_saturation += out.current.saturated != 0;
        const auto dq=park(motor.alpha,motor.beta,motor.angle);
        const double torque=1.5*config.pole_pairs*motor.flux*dq[1];
        const double acceleration=(torque-0.001*tau*motor.mechanical_speed)/(tau*0.0015);
        motor.electrical_step(applied_voltage(out.current.duty,value(bus)),
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
    const auto zero=T::make(0), one=T::make(1);
    auto config=current_config<T>();
    typename A::foc ctx{};
    typename A::foc_output out{};
    ASSERT_EQ(A::foc_init(&ctx,&config),LINEARS_OK);
    ASSERT_EQ(A::foc_step(&ctx,{zero,zero},{one,one},zero,T::make(24),T::make(0.001),&out),LINEARS_OK);
    const auto saved=ctx;
    const auto saved_output=out;
    for (unsigned failure=0;failure<4;++failure) {
        auto dt=T::make(0.001),bus=T::make(24),angle=zero;
        typename A::dq reference{one,one};
        if (failure==0) dt=zero;
        if (failure==1) bus=zero;
        if (failure==2) angle=one;
        if (failure==3) reference.q=invalid_scalar<T>();
        EXPECT_NE(A::foc_step(&ctx,{zero,zero},reference,angle,bus,dt,&out),LINEARS_OK);
        EXPECT_TRUE(fc_test::same_bytes(ctx,saved));
        EXPECT_TRUE(fc_test::same_bytes(out,saved_output));
    }
    EXPECT_EQ(A::foc_step(&ctx,{zero,zero},{one,one},zero,T::make(24),one,
        reinterpret_cast<typename A::foc_output*>(&ctx)),LINEARS_ERROR_ALIAS);
    EXPECT_TRUE(fc_test::same_bytes(ctx,saved));
    config.ki_q=T::make(-1);
    EXPECT_EQ(A::foc_init(&ctx,&config),LINEARS_ERROR_DOMAIN);
    EXPECT_TRUE(fc_test::same_bytes(ctx,saved));
    typename A::speed_config cascade{current_config<T>(),one,one,T::make(3),4};
    typename A::speed speed{};
    typename A::speed_output speed_out{};
    ASSERT_EQ(A::speed_init(&speed,&cascade),LINEARS_OK);
    const auto speed_saved=speed;
    const auto speed_out_saved=speed_out;
    EXPECT_EQ(A::speed_step(&speed,{zero,zero},T::make(10),zero,one,T::make(24),
        T::make(0.001),&speed_out),LINEARS_ERROR_DOMAIN);
    EXPECT_TRUE(fc_test::same_bytes(speed,speed_saved));
    EXPECT_TRUE(fc_test::same_bytes(speed_out,speed_out_saved));
}

TYPED_TEST(Foc, InterleavedContextsMatchIndependentRuns)
{
    using T=TypeParam;
    using A=Api<T>;
    auto config=current_config<T>(1,5,7);
    typename A::foc first{},second{};
    ASSERT_EQ(A::foc_init(&first,&config),LINEARS_OK);
    ASSERT_EQ(A::foc_init(&second,&config),LINEARS_OK);
    auto first_alone=first, second_alone=second;
    typename A::foc_output a{},b{},aa{},bb{};
    for (unsigned n=0;n<500;++n) {
        const typename A::ab measured{T::make(std::sin(n*0.03)),T::make(std::cos(n*0.03))};
        const auto angle=angle_input<T>(n*0.017);
        ASSERT_EQ(A::foc_step(&first,measured,{T::make(0.1),T::make(0.3)},angle,
            T::make(24),T::make(0.001),&a),LINEARS_OK);
        ASSERT_EQ(A::foc_step(&second,measured,{T::make(-0.8),T::make(0.2)},angle,
            T::make(12),T::make(0.002),&b),LINEARS_OK);
    }
    for (unsigned n=0;n<500;++n) {
        const typename A::ab measured{T::make(std::sin(n*0.03)),T::make(std::cos(n*0.03))};
        ASSERT_EQ(A::foc_step(&first_alone,measured,{T::make(0.1),T::make(0.3)},
            angle_input<T>(n*0.017),T::make(24),T::make(0.001),&aa),LINEARS_OK);
    }
    for (unsigned n=0;n<500;++n) {
        const typename A::ab measured{T::make(std::sin(n*0.03)),T::make(std::cos(n*0.03))};
        ASSERT_EQ(A::foc_step(&second_alone,measured,{T::make(-0.8),T::make(0.2)},
            angle_input<T>(n*0.017),T::make(12),T::make(0.002),&bb),LINEARS_OK);
    }
    EXPECT_EQ(value(first.integral.d),value(first_alone.integral.d));
    EXPECT_EQ(value(first.integral.q),value(first_alone.integral.q));
    EXPECT_EQ(value(second.integral.d),value(second_alone.integral.d));
    EXPECT_EQ(value(second.integral.q),value(second_alone.integral.q));
    EXPECT_EQ(value(a.duty.a),value(aa.duty.a));
    EXPECT_EQ(value(b.duty.b),value(bb.duty.b));
}
} // namespace
