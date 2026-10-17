#include "foc_smo_support.h"
#include <random>

namespace {
using namespace fc_loop_test;
template<class T> class Smo : public ::testing::Test {};
TYPED_TEST_SUITE(Smo, fc_test::Types,);

TYPED_TEST(Smo, PredictorSaturationAndLowpassFollowIndependentRecurrence)
{
    using T=TypeParam;
    using A=Api<T>;
    auto config=observer_config<T>();
    config.min_emf=T::make(64); // Isolate predictor/filter arithmetic from angle extraction.
    typename A::smo ctx{};
    typename A::smo_output out{};
    const typename A::ab initial{T::make(0.4),T::make(-0.3)};
    ASSERT_EQ(A::smo_init(&ctx,&config,initial),LINEARS_OK);
    ObserverReference ref;
    ref.ia=value(initial.alpha); ref.ib=value(initial.beta);
    std::mt19937 random(0x534d4f31);
    std::uniform_real_distribution<double> sample(-1,1);
    double max_current_error=0,max_emf_error=0;
    for (unsigned n=0;n<4000;++n) {
        const typename A::ab current{T::make(sample(random)),T::make(sample(random))};
        const typename A::ab voltage{T::make(5*sample(random)),T::make(5*sample(random))};
        ref.step(value(current.alpha),value(current.beta),value(voltage.alpha),value(voltage.beta),
            value(config.resistance),value(config.inductance),value(config.dt),
            value(config.sliding_gain),value(config.boundary),value(config.emf_alpha));
        ASSERT_EQ(A::smo_step(&ctx,current,voltage,&out),LINEARS_OK);
        max_current_error=std::max({max_current_error,std::abs(value(out.current_hat.alpha)-ref.ia),
            std::abs(value(out.current_hat.beta)-ref.ib)});
        max_emf_error=std::max({max_emf_error,std::abs(value(out.emf.alpha)-ref.ea),
            std::abs(value(out.emf.beta)-ref.eb)});
        ASSERT_EQ(out.observable,0);
        ASSERT_EQ(out.locked,0);
    }
    EXPECT_LT(max_current_error,numeric_budget<T>(2));
    EXPECT_LT(max_emf_error,numeric_budget<T>(8));
}

TYPED_TEST(Smo, SignedSinusoidMatchesDiscreteTransferAndPhaseCompensation)
{
    using T=TypeParam;
    using A=Api<T>;
    const double working_unit=T::fixed?fc_test::Q31::unit:T::unit;
    for (int direction : {-1,1}) {
        SCOPED_TRACE(direction);
        auto config=observer_config<T>(direction);
        typename A::smo ctx{};
        typename A::smo_output out{};
        const typename A::ab zero{T::make(0),T::make(0)};
        ASSERT_EQ(A::smo_init(&ctx,&config,zero),LINEARS_OK);
        const double r=value(config.resistance), l=value(config.inductance), dt=value(config.dt);
        const double k=value(config.sliding_gain), boundary=value(config.boundary);
        const double alpha=value(config.emf_alpha), speed_alpha=value(config.speed_alpha);
        const double frequency=direction*30, amplitude=3;
        const double omega=tau*frequency*dt, c=k/boundary;
        const std::complex<double> z=std::polar(1.0,omega);
        // With |ihat|<boundary: Z/E = (dt*c/L)/(z-[1-dt*(R+c)/L]).
        const std::complex<double> observer=(dt*c/l)/(z-(1-dt*(r+c)/l));
        const std::complex<double> lowpass=alpha/(1.0-(1-alpha)/z);
        const auto transfer=observer*lowpass;
        const double observer_phase=std::arg(observer)/tau;
        ObserverReference ref;
        double angle=0,previous_ref_angle=0,reference_speed=0;
        bool has_ref_angle=false;
        double max_emf_error=0,max_angle_error=0,max_speed_numeric_error=0;
        double speed_average=0;
        for (unsigned n=0;n<6000;++n) {
            const auto emf=rotate(0,direction*amplitude,angle);
            const typename A::ab voltage{T::make(emf[0]),T::make(emf[1])};
            ref.step(0,0,value(voltage.alpha),value(voltage.beta),r,l,dt,k,boundary,alpha);
            if (std::hypot(ref.ea,ref.eb)>=value(config.min_emf)) {
                const double raw=fc_test::wrap(std::atan2(-direction*ref.ea,direction*ref.eb)/tau);
                if (has_ref_angle) {
                    const double raw_speed=fc_test::turn_error(raw,previous_ref_angle)/dt;
                    reference_speed=(1-speed_alpha)*reference_speed+speed_alpha*raw_speed;
                }
                previous_ref_angle=raw; has_ref_angle=true;
            } else { has_ref_angle=false; reference_speed=0; }
            ASSERT_EQ(A::smo_step(&ctx,zero,voltage,&out),LINEARS_OK);
            ASSERT_GE(value(out.electrical_angle),0);
            ASSERT_LT(value(out.electrical_angle),1);
            if (n>=2000) {
                ASSERT_NE(out.observable,0);
                ASSERT_NE(out.locked,0);
                const auto expected=std::complex<double>(emf[0],emf[1])*transfer;
                max_emf_error=std::max({max_emf_error,std::abs(value(out.emf.alpha)-expected.real()),
                    std::abs(value(out.emf.beta)-expected.imag())});
                max_angle_error=std::max(max_angle_error,std::abs(fc_test::turn_error(
                    value(out.electrical_angle),angle+observer_phase)));
                max_speed_numeric_error=std::max(max_speed_numeric_error,
                    std::abs(value(out.electrical_speed)-reference_speed));
                speed_average+=value(out.electrical_speed);
            }
            angle=fc_test::wrap(angle+frequency*dt);
        }
        // Input/output quantization plus eight work-rounding terms amplified by
        // the stable LP's l1 gain 1/alpha. This is not a fitted motor tolerance.
        const double emf_budget=3*amplitude*T::unit+8*working_unit*amplitude/alpha;
        const double angle_budget=3*T::unit+4*working_unit/alpha+(T::fixed?4e-9:0);
        const double speed_budget=4*working_unit/dt+2*T::unit*std::abs(frequency);
        EXPECT_LT(max_emf_error,emf_budget);
        EXPECT_LT(max_angle_error,angle_budget);
        EXPECT_LT(max_speed_numeric_error,speed_budget);
        EXPECT_NEAR(speed_average/4000,frequency,2*T::unit*std::abs(frequency)+2e-7);
        // The nonzero remaining bias is the observer's delay, not an LP sign mistake.
        EXPECT_GT(std::abs(observer_phase),0.004);
    }
}

TYPED_TEST(Smo, AcceleratingAndDeceleratingDrivenMotorModel)
{
    using T=TypeParam;
    using A=Api<T>;
    for (int direction : {-1,1}) {
        SCOPED_TRACE(direction);
        auto config=observer_config<T>(direction);
        typename A::smo ctx{};
        typename A::smo_output out{};
        Motor motor;
        motor.resistance=value(config.resistance);
        motor.inductance=value(config.inductance);
        motor.flux=0.01;
        const double dt=value(config.dt);
        ASSERT_EQ(A::smo_init(&ctx,&config,{T::make(0),T::make(0)}),LINEARS_OK);
        double max_angle_error=0,max_speed_error=0;
        unsigned observable_samples=0;
        for (unsigned n=0;n<18000;++n) {
            const double magnitude=n<6000?12+38.0*n/6000:
                (n<12000?50-42.0*(n-6000)/6000:8);
            const double frequency=direction*magnitude;
            const auto emf=rotate(0,tau*frequency*motor.flux,motor.angle);
            const auto drive=rotate(2,0,motor.angle);
            const typename A::ab voltage{T::make(emf[0]+drive[0]),T::make(emf[1]+drive[1])};
            ASSERT_EQ(A::smo_step(&ctx,{T::make(motor.alpha),T::make(motor.beta)},voltage,&out),LINEARS_OK);
            if (n>=2000) {
                max_angle_error=std::max(max_angle_error,std::abs(fc_test::turn_error(
                    value(out.electrical_angle),motor.angle)));
                max_speed_error=std::max(max_speed_error,std::abs(value(out.electrical_speed)-frequency));
                observable_samples+=out.observable!=0;
            }
            motor.electrical_step({value(voltage.alpha),value(voltage.beta)},frequency,dt);
        }
        // At 50 Hz, observer phase is <0.013 turn. Ramp acceleration is <=70
        // turns/s^2; LP/speed-filter group delay adds <0.002 turn and <0.2 Hz.
        EXPECT_LT(max_angle_error,0.015);
        EXPECT_LT(max_speed_error,0.25);
        EXPECT_GT(observable_samples,15000u);
        EXPECT_NE(out.locked,0);
        EXPECT_NEAR(value(out.electrical_speed),direction*8,0.03);
    }
}

TYPED_TEST(Smo, NoEmfStaticOffsetsAndWrongDirectionCannotLock)
{
    using T=TypeParam;
    using A=Api<T>;
    auto config=observer_config<T>();
    typename A::smo ctx{};
    typename A::smo_output out{};
    ASSERT_EQ(A::smo_init(&ctx,&config,{T::make(0),T::make(0)}),LINEARS_OK);
    Motor motor;
    motor.resistance=value(config.resistance); motor.inductance=value(config.inductance);
    motor.flux=0;
    const double dt=value(config.dt);
    for (unsigned n=0;n<6000;++n) {
        const typename A::ab voltage{T::make(1.5*std::sin(n*0.017)),T::make(std::cos(n*0.011))};
        ASSERT_EQ(A::smo_step(&ctx,{T::make(motor.alpha),T::make(motor.beta)},voltage,&out),LINEARS_OK);
        ASSERT_EQ(out.observable,0);
        ASSERT_EQ(out.locked,0);
        motor.electrical_step({value(voltage.alpha),value(voltage.beta)},0,dt);
    }
    ASSERT_EQ(A::smo_init(&ctx,&config,{T::make(0),T::make(0)}),LINEARS_OK);
    for (unsigned n=0;n<2000;++n) {
        ASSERT_EQ(A::smo_step(&ctx,{T::make(0),T::make(0)},{T::make(0),T::make(3)},&out),LINEARS_OK);
        ASSERT_EQ(out.observable,0); // Strong but stationary EMF/offset has zero speed.
        ASSERT_EQ(out.locked,0);
    }
    config.direction=FIELDCTRL_SMO_REVERSE;
    ASSERT_EQ(A::smo_init(&ctx,&config,{T::make(0),T::make(0)}),LINEARS_OK);
    for (unsigned n=0;n<4000;++n) {
        const auto emf=rotate(0,3,n*30*dt);
        ASSERT_EQ(A::smo_step(&ctx,{T::make(0),T::make(0)},
            {T::make(emf[0]),T::make(emf[1])},&out),LINEARS_OK);
        if (n>=1000) { ASSERT_EQ(out.observable,0); ASSERT_EQ(out.locked,0); }
    }
}

TYPED_TEST(Smo, EmfLossClearsLockAndReacquisitionNeedsConsecutiveSamples)
{
    using T=TypeParam;
    using A=Api<T>;
    auto config=observer_config<T>();
    typename A::smo ctx{};
    typename A::smo_output out{};
    const typename A::ab zero{T::make(0),T::make(0)};
    ASSERT_EQ(A::smo_init(&ctx,&config,zero),LINEARS_OK);
    double angle=0;
    for (unsigned n=0;n<3000;++n) {
        const auto emf=rotate(0,3,angle);
        ASSERT_EQ(A::smo_step(&ctx,zero,{T::make(emf[0]),T::make(emf[1])},&out),LINEARS_OK);
        angle=fc_test::wrap(angle+30*value(config.dt));
    }
    ASSERT_NE(out.locked,0);
    for (unsigned n=0;n<2000;++n)
        ASSERT_EQ(A::smo_step(&ctx,zero,zero,&out),LINEARS_OK);
    EXPECT_EQ(out.observable,0); EXPECT_EQ(out.locked,0);
    EXPECT_EQ(value(out.electrical_speed),0);
    EXPECT_EQ(ctx.has_angle,0);
    const double held_angle=value(out.electrical_angle);
    ASSERT_EQ(A::smo_step(&ctx,zero,zero,&out),LINEARS_OK);
    EXPECT_EQ(value(out.electrical_angle),held_angle);
    unsigned consecutive=0;
    for (unsigned n=0;n<3000;++n) {
        const auto emf=rotate(0,3,angle);
        ASSERT_EQ(A::smo_step(&ctx,zero,{T::make(emf[0]),T::make(emf[1])},&out),LINEARS_OK);
        consecutive=out.observable?std::min(consecutive+1,config.lock_samples):0;
        EXPECT_EQ(out.locked!=0,consecutive==config.lock_samples);
        angle=fc_test::wrap(angle+30*value(config.dt));
    }
    EXPECT_NE(out.locked,0);
}

TYPED_TEST(Smo, TinyPositiveLowpassWeightDoesNotCancelPhaseDenominator)
{
    using T=TypeParam;
    using A=Api<T>;
    auto config=observer_config<T>();
    config.emf_alpha=T::make(std::ldexp(1.0,-40));
    config.min_emf=T::make(std::ldexp(1.0,-45));
    typename A::smo ctx{};
    typename A::smo_output out{};
    ASSERT_EQ(A::smo_init(&ctx,&config,{T::make(0.1),T::make(0)}),LINEARS_OK);
    ASSERT_EQ(A::smo_step(&ctx,{T::make(0),T::make(0)},{T::make(0),T::make(0)},&out),LINEARS_OK);
    EXPECT_GT(value(out.emf.alpha),value(config.min_emf));
    EXPECT_EQ(value(out.electrical_speed),0);
    EXPECT_NEAR(value(out.electrical_angle),0.75,numeric_budget<T>(1));
    EXPECT_EQ(out.observable,0); EXPECT_EQ(out.locked,0);
}

TYPED_TEST(Smo, InvalidConfigurationAndSamplesPreserveStateAndOutput)
{
    using T=TypeParam;
    using A=Api<T>;
    auto config=observer_config<T>();
    typename A::smo ctx{};
    typename A::smo_output out{};
    const typename A::ab zero{T::make(0),T::make(0)};
    ASSERT_EQ(A::smo_init(&ctx,&config,zero),LINEARS_OK);
    ASSERT_EQ(A::smo_step(&ctx,zero,{T::make(1),T::make(2)},&out),LINEARS_OK);
    const auto saved=ctx;
    const auto saved_output=out;
    for (unsigned failure=0;failure<7;++failure) {
        SCOPED_TRACE(failure);
        auto bad=config;
        if (failure==0) bad.dt=T::make(0);
        if (failure==1) bad.dt=T::make(0.01); // Unstable Euler boundary layer.
        if (failure==2) { // Exactly representable quarter turn in every format.
            bad.dt=T::make(1.0/4096);
            bad.max_speed=T::make(1024);
        }
        if (failure==3) bad.emf_alpha=T::make(0);
        if (failure==4) bad.boundary=T::make(-1);
        if (failure==5) bad.lock_samples=0;
        if (failure==6) bad.direction=static_cast<fieldctrl_smo_direction_t>(0);
        ASSERT_EQ(A::smo_init(&ctx,&bad,zero),LINEARS_ERROR_DOMAIN);
        EXPECT_TRUE(fc_test::same_bytes(ctx,saved));
    }
    const auto bad=invalid_scalar<T>();
    EXPECT_EQ(A::smo_step(&ctx,{bad,T::make(0)},zero,&out),LINEARS_ERROR_RANGE);
    EXPECT_TRUE(fc_test::same_bytes(ctx,saved));
    EXPECT_TRUE(fc_test::same_bytes(out,saved_output));
    EXPECT_EQ(A::smo_step(&ctx,zero,{T::make(0),bad},&out),LINEARS_ERROR_RANGE);
    EXPECT_TRUE(fc_test::same_bytes(ctx,saved));
    EXPECT_TRUE(fc_test::same_bytes(out,saved_output));
    EXPECT_EQ(A::smo_step(&ctx,zero,zero,reinterpret_cast<typename A::smo_output*>(&ctx)),LINEARS_ERROR_ALIAS);
    EXPECT_TRUE(fc_test::same_bytes(ctx,saved));
}

TYPED_TEST(Smo, IndependentObserversRetainSeparatePredictorAndFilterState)
{
    using T=TypeParam;
    using A=Api<T>;
    auto forward=observer_config<T>(1),reverse=observer_config<T>(-1);
    typename A::smo a{},b{};
    const typename A::ab zero{T::make(0),T::make(0)};
    ASSERT_EQ(A::smo_init(&a,&forward,zero),LINEARS_OK);
    ASSERT_EQ(A::smo_init(&b,&reverse,zero),LINEARS_OK);
    auto aa=a,bb=b;
    typename A::smo_output ao{},bo{},aao{},bbo{};
    for (unsigned n=0;n<2500;++n) {
        const auto ea=rotate(0,3,n*25*value(forward.dt));
        const auto eb=rotate(0,-2,-int(n)*40*value(reverse.dt));
        ASSERT_EQ(A::smo_step(&a,zero,{T::make(ea[0]),T::make(ea[1])},&ao),LINEARS_OK);
        ASSERT_EQ(A::smo_step(&b,zero,{T::make(eb[0]),T::make(eb[1])},&bo),LINEARS_OK);
    }
    for (unsigned n=0;n<2500;++n) {
        const auto ea=rotate(0,3,n*25*value(forward.dt));
        ASSERT_EQ(A::smo_step(&aa,zero,{T::make(ea[0]),T::make(ea[1])},&aao),LINEARS_OK);
    }
    for (unsigned n=0;n<2500;++n) {
        const auto eb=rotate(0,-2,-int(n)*40*value(reverse.dt));
        ASSERT_EQ(A::smo_step(&bb,zero,{T::make(eb[0]),T::make(eb[1])},&bbo),LINEARS_OK);
    }
    EXPECT_EQ(value(ao.electrical_angle),value(aao.electrical_angle));
    EXPECT_EQ(value(ao.electrical_speed),value(aao.electrical_speed));
    EXPECT_EQ(value(bo.electrical_angle),value(bbo.electrical_angle));
    EXPECT_EQ(value(bo.electrical_speed),value(bbo.electrical_speed));
    EXPECT_NE(ao.locked,0); EXPECT_NE(bo.locked,0);
}

TYPED_TEST(Smo, RotatingHandoverClosesCurrentLoopOnObserverAngleBothDirections)
{
    using T=TypeParam;
    using A=Api<T>;
    for (int direction : {-1,1}) {
        SCOPED_TRACE(direction);
        auto observer_configured=observer_config<T>(direction);
        observer_configured.resistance=T::make(0.5);
        auto controller_configured=current_config<T>();
        typename A::smo observer{};
        typename A::smo_output estimate{};
        typename A::foc controller{};
        typename A::foc_output command{};
        Motor motor;
        const double dt=value(observer_configured.dt);
        ASSERT_EQ(A::smo_init(&observer,&observer_configured,{T::make(0),T::make(0)}),LINEARS_OK);
        ASSERT_EQ(A::foc_init(&controller,&controller_configured),LINEARS_OK);
        bool sensorless=false;
        unsigned sensorless_samples=0;
        double max_angle_error=0,current_q_squared_error=0,max_true_d=0;
        for (unsigned n=0;n<14000;++n) {
            if (!sensorless && n>=3000 && estimate.observable && estimate.locked)
                sensorless=true;
            // After handover, neither true plant angle nor true frequency is
            // fed back. The last timestamped estimate is advanced one sample
            // with the observer's own frequency, not the shaft-speed source.
            const double feedback_angle=sensorless
                ? fc_test::wrap(value(estimate.electrical_angle)+value(estimate.electrical_speed)*dt)
                : motor.angle;
            const double reference=direction*(n<6000?2:1.2);
            const typename A::ab measured{T::make(motor.alpha),T::make(motor.beta)};
            ASSERT_EQ(A::foc_step(&controller,measured,{T::make(0),T::make(reference)},
                angle_input<T>(feedback_angle),T::make(24),observer_configured.dt,&command),LINEARS_OK);
            const auto voltage=applied_voltage(command.duty,24);
            ASSERT_EQ(A::smo_step(&observer,measured,{T::make(voltage[0]),T::make(voltage[1])},&estimate),LINEARS_OK);
            if (sensorless) {
                ++sensorless_samples;
                ASSERT_NE(estimate.observable,0);
                ASSERT_NE(estimate.locked,0);
                max_angle_error=std::max(max_angle_error,std::abs(fc_test::turn_error(
                    value(estimate.electrical_angle),motor.angle)));
            }
            if (n>=13000) {
                const auto true_dq=park(motor.alpha,motor.beta,motor.angle);
                current_q_squared_error+=(true_dq[1]-reference)*(true_dq[1]-reference);
                max_true_d=std::max(max_true_d,std::abs(true_dq[0]));
            }
            // An ideal shaft drive supplies rotation; this test proves rotating
            // sensorless electrical-loop takeover, not standstill motor startup.
            const double frequency=direction*(n<6000?25:(n<9000?25+10.0*(n-6000)/3000:35));
            motor.electrical_step(voltage,frequency,dt);
        }
        EXPECT_GT(sensorless_samples,10000u);
        EXPECT_LT(max_angle_error,0.011);
        EXPECT_LT(std::sqrt(current_q_squared_error/1000),0.02);
        EXPECT_LT(max_true_d,0.09);
    }
}
} // namespace
