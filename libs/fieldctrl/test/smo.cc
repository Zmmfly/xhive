#include "foc_smo_support.h"
#include <random>
#include <string>
#include <iomanip>
#include <sstream>

namespace {
using namespace fc_loop_test;
template<class T> class Smo : public ::testing::Test {};
TYPED_TEST_SUITE(Smo, fc_test::Types,);
template<class T> class FloatingSmo : public ::testing::Test {};
using FloatingTypes=::testing::Types<fc_test::F32,fc_test::F64>;
TYPED_TEST_SUITE(FloatingSmo, FloatingTypes,);
template<class T> class FixedSmo : public ::testing::Test {};
using FixedTypes=::testing::Types<fc_test::Q15,fc_test::Q31>;
TYPED_TEST_SUITE(FixedSmo, FixedTypes,);

TYPED_TEST(Smo, PredictorSaturationAndLowpassFollowIndependentRecurrence)
{
    using T=TypeParam;
    using A=Api<T>;
    const auto u=observer_units<T>();
    auto config=observer_config<T>();
    config.min_emf=T::make(T::fixed?0.875:64); // Isolate predictor/filter arithmetic from angle extraction.
    typename A::smo ctx{};
    typename A::smo_output out{};
    const typename A::ab initial{u.current(0.4),u.current(-0.3)};
    ASSERT_EQ(A::smo_init(&ctx,&config,initial),LINEARS_OK);
    ObserverReference ref;
    ref.ia=value(initial.alpha); ref.ib=value(initial.beta);
    std::mt19937 random(0x534d4f31);
    std::uniform_real_distribution<double> sample(-1,1);
    double max_current_error=0,max_emf_error=0;
    for (unsigned n=0;n<4000;++n) {
        const typename A::ab current{u.current(sample(random)),u.current(sample(random))};
        const typename A::ab voltage{u.voltage(5*sample(random)),u.voltage(5*sample(random))};
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
    if constexpr (T::fixed) {
        // Outside the boundary layer the worst contraction is 1-dt*R/L.
        // At most two absolute F31 rounding units enter each predictor step.
        const double w=fc_test::Q31::unit;
        const double current_round=2*w/(value(config.dt)*value(config.resistance)/value(config.inductance));
        const double emf_round=value(config.sliding_gain)/value(config.boundary)*current_round+
            w+w/(2*value(config.emf_alpha));
        EXPECT_LT(max_current_error,T::unit/2+current_round);
        EXPECT_LT(max_emf_error,T::unit/2+emf_round);
    } else {
        EXPECT_LT(max_current_error,numeric_budget<T>(2));
        EXPECT_LT(max_emf_error,numeric_budget<T>(8));
    }
}

TYPED_TEST(Smo, SignedSinusoidMatchesDiscreteTransferAndPhaseCompensation)
{
    using T=TypeParam;
    using A=Api<T>;
    const auto u=observer_units<T>();
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
        const double frequency=direction*30*u.t, amplitude=3/u.v;
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
                double expected_angle=angle+observer_phase;
                if constexpr (T::fixed) {
                    const double omega_ref=tau*reference_speed*dt;
                    const double phase_ref=std::atan2((1-alpha)*std::sin(omega_ref),
                        alpha+(1-alpha)*(1-std::cos(omega_ref)))/tau;
                    expected_angle=previous_ref_angle+phase_ref;
                }
                max_angle_error=std::max(max_angle_error,std::abs(fc_test::turn_error(
                    value(out.electrical_angle),expected_angle)));
                max_speed_numeric_error=std::max(max_speed_numeric_error,
                    std::abs(value(out.electrical_speed)-reference_speed));
                speed_average+=value(out.electrical_speed);
            }
            angle=fc_test::wrap(angle+frequency*dt);
        }
        double emf_budget,angle_budget,speed_budget,mean_budget;
        if constexpr (T::fixed) {
            // Absolute F31 rounding propagates through the linear observer's
            // contraction and the LP's l1 gain; Q1 I/O adds absolute half-LSBs.
            const double contraction=std::abs(1-dt*(r+c)/l);
            const double current_round=2*working_unit/(1-contraction);
            const double emf_round=c*current_round+working_unit+working_unit/(2*alpha);
            const double input_gain=dt*c/l/(1-contraction);
            emf_budget=(input_gain+1)*T::unit/2+emf_round;
            const double emf_amplitude=amplitude*std::abs(transfer);
            const double phase_round=2*emf_round/(tau*(emf_amplitude-2*emf_budget))+4*working_unit;
            speed_budget=2*phase_round/dt+working_unit/(2*speed_alpha);
            angle_budget=T::unit/2+phase_round+
                dt*(1-alpha)/alpha*speed_budget+4*working_unit/alpha;
            const double signal_phase=2*emf_budget/(tau*emf_amplitude)+phase_round;
            mean_budget=(2+4*(1-speed_alpha)/speed_alpha)*signal_phase/(4000*dt)+
                working_unit/(2*speed_alpha);
        } else {
            emf_budget=3*amplitude*T::unit+8*working_unit*amplitude/alpha;
            angle_budget=3*T::unit+4*working_unit/alpha;
            speed_budget=4*working_unit/dt+2*T::unit*std::abs(frequency);
            mean_budget=2*T::unit*std::abs(frequency)+2e-7;
        }
        EXPECT_LT(max_emf_error,emf_budget);
        EXPECT_LT(max_angle_error,angle_budget);
        EXPECT_LT(max_speed_numeric_error,speed_budget);
        EXPECT_NEAR(speed_average/4000,frequency,mean_budget);
        // The nonzero remaining bias is the observer's delay, not an LP sign mistake.
        EXPECT_GT(std::abs(observer_phase),0.004);
    }
}

enum class FrequencyCriterion { BelowQuarterHertz, QuantizationLimited };

/* Same physical plant and gains for both explicit base choices. Separate
 * integer arithmetic error from sensor quantization instead of enlarging a
 * floating-input peak-speed tolerance to conceal Q15 differentiation noise. */
template<class T> void driven_motor_case(Units<T> u,FrequencyCriterion criterion)
{
    using A=Api<T>;
    for (int direction : {-1,1}) {
        SCOPED_TRACE(direction);
        auto config=observer_config<T>(direction,u);
        typename A::smo ctx{};
        typename A::smo_output out{};
        Motor motor;
        const double r=value(config.resistance),l=value(config.inductance);
        const double period=value(config.dt),k=value(config.sliding_gain),b=value(config.boundary);
        const double alpha=value(config.emf_alpha),sa=value(config.speed_alpha),threshold=value(config.min_emf);
        motor.resistance=r*u.v/u.i;
        motor.inductance=l*u.v*u.t/u.i;
        motor.flux=0.01;
        const double dt=u.seconds(config.dt);
        ASSERT_EQ(A::smo_init(&ctx,&config,{T::make(0),T::make(0)}),LINEARS_OK);
        MotionReference quantized,exact_sensor;
        double max_angle_error=0,max_speed_error=0,ideal_speed_error=0,sensor_ripple=0;
        double numerical_speed_error=0,minimum_emf=1,reference_current_error=0;
        double tail_speed_sum=0,peak_current=0;
        double peak_current_input=0,peak_voltage_input=0,peak_plant_frequency=0;
        unsigned observable_samples=0;
        for (unsigned n=0;n<18000;++n) {
            const double magnitude=n<6000?12+38.0*n/6000:
                (n<12000?50-42.0*(n-6000)/6000:8);
            const double frequency=direction*magnitude;
            const auto emf=rotate(0,tau*frequency*motor.flux,motor.angle);
            const auto drive=rotate(2,0,motor.angle);
            const typename A::ab voltage{u.voltage(emf[0]+drive[0]),u.voltage(emf[1]+drive[1])};
            const typename A::ab measured{u.current(motor.alpha),u.current(motor.beta)};
            peak_current=std::max(peak_current,std::hypot(motor.alpha,motor.beta));
            peak_current_input=std::max({peak_current_input,std::abs(value(measured.alpha)),
                std::abs(value(measured.beta))});
            peak_voltage_input=std::max({peak_voltage_input,std::abs(value(voltage.alpha)),
                std::abs(value(voltage.beta))});
            peak_plant_frequency=std::max(peak_plant_frequency,std::abs(frequency*u.t));
            reference_current_error=std::max({reference_current_error,
                std::abs(quantized.observer.ia-value(measured.alpha)),
                std::abs(quantized.observer.ib-value(measured.beta))});
            quantized.step(value(measured.alpha),value(measured.beta),value(voltage.alpha),value(voltage.beta),
                r,l,period,k,b,alpha,sa,threshold,direction);
            // Use the same quantized configuration and actually applied voltage;
            // remove only current-sensor quantization in this control sequence.
            exact_sensor.step(motor.alpha/u.i,motor.beta/u.i,value(voltage.alpha),value(voltage.beta),
                r,l,period,k,b,alpha,sa,threshold,direction);
            ASSERT_EQ(A::smo_step(&ctx,measured,voltage,&out),LINEARS_OK);
            if (n>=2000) {
                max_angle_error=std::max(max_angle_error,std::abs(fc_test::turn_error(
                    value(out.electrical_angle),motor.angle)));
                max_speed_error=std::max(max_speed_error,std::abs(u.hertz(out.electrical_speed)-frequency));
                ideal_speed_error=std::max(ideal_speed_error,std::abs(exact_sensor.speed/u.t-frequency));
                sensor_ripple=std::max(sensor_ripple,std::abs(quantized.speed-exact_sensor.speed)/u.t);
                numerical_speed_error=std::max(numerical_speed_error,
                    std::abs(value(out.electrical_speed)-quantized.speed));
                minimum_emf=std::min(minimum_emf,std::hypot(quantized.observer.ea,quantized.observer.eb));
                observable_samples+=out.observable!=0;
            }
            if (n>=17000) tail_speed_sum+=u.hertz(out.electrical_speed);
            motor.electrical_step({u.volts(voltage.alpha),u.volts(voltage.beta)},frequency,dt);
        }
        EXPECT_LT(max_angle_error,0.015);
        EXPECT_LT(ideal_speed_error,0.25);
        EXPECT_GT(observable_samples,15000u);
        EXPECT_NE(out.locked,0);
        if constexpr (T::fixed) {
            const double w=fc_test::Q31::unit,c=k/b;
            const double contraction=std::abs(1-period*(r+c)/l);
            const double current_round=2*w/(1-contraction);
            // These assertions establish the linear-region and angle-conditioning
            // premises for the following absolute F31 error propagation.
            EXPECT_LT(reference_current_error+current_round,b);
            const double emf_round=c*current_round+w+w/(2*alpha);
            ASSERT_GT(minimum_emf,2*emf_round);
            const double phase_round=2*emf_round/(tau*(minimum_emf-2*emf_round))+4*w;
            const double speed_round=2*phase_round/period+w/(2*sa);
            EXPECT_LT(numerical_speed_error,speed_round);
            EXPECT_LE(max_speed_error,ideal_speed_error+sensor_ripple+speed_round/u.t);
            // The 2 A candidate must prove at least 10% physical input margin;
            // the original 4 A fixture is also checked, not just assumed safe.
            EXPECT_LT(peak_current,0.9*u.i);
            EXPECT_LT(peak_current_input,1);
            EXPECT_LT(peak_voltage_input,1);
            EXPECT_GT(value(config.min_speed),0);
            EXPECT_LT(value(config.min_speed),value(config.max_speed));
            EXPECT_LT(value(config.max_speed),1);
            EXPECT_LT(peak_plant_frequency,value(config.max_speed));
        }
        if (criterion==FrequencyCriterion::BelowQuarterHertz) {
            EXPECT_LT(max_speed_error,0.25);
            EXPECT_NEAR(u.hertz(out.electrical_speed),direction*8,0.03);
        } else {
            // Explicitly retain the Q15 peak-noise limitation. A good mean is
            // additional convergence evidence, not a replacement peak claim.
            EXPECT_GT(max_speed_error,0.25);
            EXPECT_GT(std::abs(u.hertz(out.electrical_speed)-direction*8),0.03);
            EXPECT_NEAR(tail_speed_sum/1000,direction*8,0.03);
        }
        const std::string suffix=direction>0?"_positive":"_negative";
        const auto text=[](double v) {
            std::ostringstream stream;
            stream<<std::setprecision(17)<<v;
            return stream.str();
        };
        ::testing::Test::RecordProperty("peak_speed_error_hz"+suffix,text(max_speed_error));
        ::testing::Test::RecordProperty("ideal_sensor_reference_peak_hz"+suffix,text(ideal_speed_error));
        ::testing::Test::RecordProperty("last_sample_error_hz"+suffix,
            text(std::abs(u.hertz(out.electrical_speed)-direction*8)));
        ::testing::Test::RecordProperty("tail_mean_speed_hz"+suffix,text(tail_speed_sum/1000));
        ::testing::Test::RecordProperty("sensor_quantization_peak_hz"+suffix,text(sensor_ripple));
        ::testing::Test::RecordProperty("implementation_vs_reference_hz"+suffix,text(numerical_speed_error/u.t));
        ::testing::Test::RecordProperty("peak_current_amperes"+suffix,text(peak_current));
        ::testing::Test::RecordProperty("meets_legacy_peak"+suffix,max_speed_error<0.25?"true":"false");
        ::testing::Test::RecordProperty("meets_legacy_last_sample"+suffix,
            std::abs(u.hertz(out.electrical_speed)-direction*8)<0.03?"true":"false");
        if constexpr (T::fixed) {
            ::testing::Test::RecordProperty("peak_current_input_pu"+suffix,text(peak_current_input));
            ::testing::Test::RecordProperty("peak_current_vector_pu"+suffix,text(peak_current/u.i));
            ::testing::Test::RecordProperty("peak_applied_voltage_input_pu"+suffix,text(peak_voltage_input));
            ::testing::Test::RecordProperty("plant_electrical_frequency_peak_pu"+suffix,text(peak_plant_frequency));
            ::testing::Test::RecordProperty("config_min_speed_pu"+suffix,text(value(config.min_speed)));
            ::testing::Test::RecordProperty("config_max_speed_pu"+suffix,text(value(config.max_speed)));
            ::testing::Test::RecordProperty("config_resistance_pu"+suffix,text(r));
            ::testing::Test::RecordProperty("config_inductance_pu"+suffix,text(l));
            ::testing::Test::RecordProperty("config_sample_period_pu"+suffix,text(period));
            ::testing::Test::RecordProperty("config_injection_gain_pu"+suffix,text(k));
            ::testing::Test::RecordProperty("config_boundary_pu"+suffix,text(b));
            ::testing::Test::RecordProperty("config_emf_alpha"+suffix,text(alpha));
            ::testing::Test::RecordProperty("config_speed_alpha"+suffix,text(sa));
            ::testing::Test::RecordProperty("config_min_emf_pu"+suffix,text(threshold));
            ::testing::Test::RecordProperty("speed_reference"+suffix,"none: SMO-only observation");
        }
    }
}

TYPED_TEST(Smo, AcceleratingAndDeceleratingDrivenMotorModel)
{
    using T=TypeParam;
    driven_motor_case<T>(observer_units<T>(),std::is_same_v<T,fc_test::Q15>
        ?FrequencyCriterion::QuantizationLimited:FrequencyCriterion::BelowQuarterHertz);
}

TYPED_TEST(FixedSmo, EngineeringBaseCandidateDocumentsHeadroomAndPeakNoise)
{
    using T=TypeParam;
    // The only additional candidate: original 100 us Ts is exactly 1/64 in
    // these pu units; R=.125, L=.1953125 and K=.5 are exactly representable.
    driven_motor_case<T>({16,2,0.0064},std::is_same_v<T,fc_test::Q15>
        ?FrequencyCriterion::QuantizationLimited:FrequencyCriterion::BelowQuarterHertz);
}

TYPED_TEST(Smo, NoEmfStaticOffsetsAndWrongDirectionCannotLock)
{
    using T=TypeParam;
    using A=Api<T>;
    const auto u=observer_units<T>();
    auto config=observer_config<T>();
    typename A::smo ctx{};
    typename A::smo_output out{};
    ASSERT_EQ(A::smo_init(&ctx,&config,{T::make(0),T::make(0)}),LINEARS_OK);
    Motor motor;
    motor.resistance=value(config.resistance)*u.v/u.i;
    motor.inductance=value(config.inductance)*u.v*u.t/u.i;
    motor.flux=0;
    const double dt=u.seconds(config.dt);
    for (unsigned n=0;n<6000;++n) {
        const typename A::ab voltage{u.voltage(1.5*std::sin(n*0.017)),u.voltage(std::cos(n*0.011))};
        ASSERT_EQ(A::smo_step(&ctx,{u.current(motor.alpha),u.current(motor.beta)},voltage,&out),LINEARS_OK);
        ASSERT_EQ(out.observable,0);
        ASSERT_EQ(out.locked,0);
        motor.electrical_step({u.volts(voltage.alpha),u.volts(voltage.beta)},0,dt);
    }
    ASSERT_EQ(A::smo_init(&ctx,&config,{T::make(0),T::make(0)}),LINEARS_OK);
    for (unsigned n=0;n<2000;++n) {
        ASSERT_EQ(A::smo_step(&ctx,{T::make(0),T::make(0)},{T::make(0),u.voltage(3)},&out),LINEARS_OK);
        ASSERT_EQ(out.observable,0); // Strong but stationary EMF/offset has zero speed.
        ASSERT_EQ(out.locked,0);
    }
    config.direction=FIELDCTRL_SMO_REVERSE;
    ASSERT_EQ(A::smo_init(&ctx,&config,{T::make(0),T::make(0)}),LINEARS_OK);
    for (unsigned n=0;n<4000;++n) {
        const auto emf=rotate(0,3,n*30*dt);
        ASSERT_EQ(A::smo_step(&ctx,{T::make(0),T::make(0)},
            {u.voltage(emf[0]),u.voltage(emf[1])},&out),LINEARS_OK);
        if (n>=1000) { ASSERT_EQ(out.observable,0); ASSERT_EQ(out.locked,0); }
    }
}

TYPED_TEST(Smo, EmfLossClearsLockAndReacquisitionNeedsConsecutiveSamples)
{
    using T=TypeParam;
    using A=Api<T>;
    const auto u=observer_units<T>();
    auto config=observer_config<T>();
    typename A::smo ctx{};
    typename A::smo_output out{};
    const typename A::ab zero{T::make(0),T::make(0)};
    ASSERT_EQ(A::smo_init(&ctx,&config,zero),LINEARS_OK);
    double angle=0;
    for (unsigned n=0;n<3000;++n) {
        const auto emf=rotate(0,3,angle);
        ASSERT_EQ(A::smo_step(&ctx,zero,{u.voltage(emf[0]),u.voltage(emf[1])},&out),LINEARS_OK);
        angle=fc_test::wrap(angle+30*u.seconds(config.dt));
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
        ASSERT_EQ(A::smo_step(&ctx,zero,{u.voltage(emf[0]),u.voltage(emf[1])},&out),LINEARS_OK);
        consecutive=out.observable?std::min(consecutive+1,config.lock_samples):0;
        EXPECT_EQ(out.locked!=0,consecutive==config.lock_samples);
        angle=fc_test::wrap(angle+30*u.seconds(config.dt));
    }
    EXPECT_NE(out.locked,0);
}

TYPED_TEST(FloatingSmo, TinyPositiveLowpassWeightDoesNotCancelPhaseDenominator)
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
    const auto u=observer_units<T>();
    auto config=observer_config<T>();
    typename A::smo ctx{};
    typename A::smo_output out{};
    const typename A::ab zero{T::make(0),T::make(0)};
    ASSERT_EQ(A::smo_init(&ctx,&config,zero),LINEARS_OK);
    ASSERT_EQ(A::smo_step(&ctx,zero,{u.voltage(1),u.voltage(2)},&out),LINEARS_OK);
    const auto saved=ctx;
    const auto saved_output=out;
    for (unsigned failure=0;failure<7;++failure) {
        SCOPED_TRACE(failure);
        auto bad=config;
        if (failure==0) bad.dt=T::make(0);
        if (failure==1) bad.dt=T::make(T::fixed?0.875:0.01); // Unstable Euler boundary layer.
        if (failure==2) { // Exact quarter turn; the stability condition still passes.
            if constexpr (T::fixed) {
                bad.dt=T::make(0.5); bad.max_speed=T::make(0.5);
                bad.sliding_gain=T::make(0.03125); bad.boundary=T::make(0.25);
            } else {
                bad.dt=T::make(1.0/4096); bad.max_speed=T::make(1024);
            }
        }
        if (failure==3) bad.emf_alpha=T::make(0);
        if (failure==4) bad.boundary=T::make(-1);
        if (failure==5) bad.lock_samples=0;
        if (failure==6) bad.direction=static_cast<fieldctrl_smo_direction_t>(0);
        ASSERT_EQ(A::smo_init(&ctx,&bad,zero),LINEARS_ERROR_DOMAIN);
        EXPECT_TRUE(fc_test::same_bytes(ctx,saved));
    }
    if constexpr (!T::fixed) {
        const auto bad=invalid_scalar<T>();
        EXPECT_EQ(A::smo_step(&ctx,{bad,T::make(0)},zero,&out),LINEARS_ERROR_RANGE);
        EXPECT_TRUE(fc_test::same_bytes(ctx,saved));
        EXPECT_TRUE(fc_test::same_bytes(out,saved_output));
        EXPECT_EQ(A::smo_step(&ctx,zero,{T::make(0),bad},&out),LINEARS_ERROR_RANGE);
        EXPECT_TRUE(fc_test::same_bytes(ctx,saved));
        EXPECT_TRUE(fc_test::same_bytes(out,saved_output));
    }
    EXPECT_EQ(A::smo_step(&ctx,zero,zero,reinterpret_cast<typename A::smo_output*>(&ctx)),LINEARS_ERROR_ALIAS);
    EXPECT_TRUE(fc_test::same_bytes(ctx,saved));
    EXPECT_TRUE(fc_test::same_bytes(out,saved_output));
}

TYPED_TEST(Smo, IndependentObserversRetainSeparatePredictorAndFilterState)
{
    using T=TypeParam;
    using A=Api<T>;
    const auto u=observer_units<T>();
    auto forward=observer_config<T>(1),reverse=observer_config<T>(-1);
    typename A::smo a{},b{};
    const typename A::ab zero{T::make(0),T::make(0)};
    ASSERT_EQ(A::smo_init(&a,&forward,zero),LINEARS_OK);
    ASSERT_EQ(A::smo_init(&b,&reverse,zero),LINEARS_OK);
    auto aa=a,bb=b;
    typename A::smo_output ao{},bo{},aao{},bbo{};
    for (unsigned n=0;n<2500;++n) {
        const auto ea=rotate(0,3,n*25*u.seconds(forward.dt));
        const auto eb=rotate(0,-2,-int(n)*40*u.seconds(reverse.dt));
        ASSERT_EQ(A::smo_step(&a,zero,{u.voltage(ea[0]),u.voltage(ea[1])},&ao),LINEARS_OK);
        ASSERT_EQ(A::smo_step(&b,zero,{u.voltage(eb[0]),u.voltage(eb[1])},&bo),LINEARS_OK);
    }
    for (unsigned n=0;n<2500;++n) {
        const auto ea=rotate(0,3,n*25*u.seconds(forward.dt));
        ASSERT_EQ(A::smo_step(&aa,zero,{u.voltage(ea[0]),u.voltage(ea[1])},&aao),LINEARS_OK);
    }
    for (unsigned n=0;n<2500;++n) {
        const auto eb=rotate(0,-2,-int(n)*40*u.seconds(reverse.dt));
        ASSERT_EQ(A::smo_step(&bb,zero,{u.voltage(eb[0]),u.voltage(eb[1])},&bbo),LINEARS_OK);
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
    const auto u=observer_units<T>();
    for (int direction : {-1,1}) {
        SCOPED_TRACE(direction);
        auto observer_configured=observer_config<T>(direction);
        observer_configured.resistance=T::make(0.5*u.i/u.v);
        auto controller_configured=current_config<T>(2,400,12,u);
        typename A::smo observer{};
        typename A::smo_output estimate{};
        typename A::foc controller{};
        typename A::foc_output command{};
        Motor motor;
        const double dt=u.seconds(observer_configured.dt);
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
                ? fc_test::wrap(value(estimate.electrical_angle)+u.hertz(estimate.electrical_speed)*dt)
                : motor.angle;
            const double reference=direction*(n<6000?2:1.2);
            const typename A::ab measured{u.current(motor.alpha),u.current(motor.beta)};
            ASSERT_EQ(A::foc_step(&controller,measured,{T::make(0),u.current(reference)},
                angle_input<T>(feedback_angle),u.voltage(24),observer_configured.dt,&command),LINEARS_OK);
            const auto voltage=applied_voltage(command.duty,u.volts(u.voltage(24)));
            ASSERT_EQ(A::smo_step(&observer,measured,{u.voltage(voltage[0]),u.voltage(voltage[1])},&estimate),LINEARS_OK);
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
TYPED_TEST(FloatingSmo, ExtremeErrorDivisionRetainsItsRangeFailure)
{
    using T=TypeParam;
    using A=Api<T>;
    const auto zero=T::make(0),tiny=T::make(std::is_same_v<T,fc_test::F32>?1e-30:1e-200);
    typename A::smo_config config{zero,T::make(1),tiny,tiny,tiny,T::make(0.5),T::make(0.5),
        T::make(1),T::make(1),T::make(2),1,FIELDCTRL_SMO_FORWARD};
    const auto half_max=std::numeric_limits<typename T::scalar>::max()/2;
    typename A::smo ctx{};
    typename A::smo_output out{};
    ASSERT_EQ(A::smo_init(&ctx,&config,{half_max,zero}),LINEARS_OK);
    const auto old=ctx;
    const auto previous=out;
    EXPECT_EQ(A::smo_step(&ctx,{-half_max,zero},{zero,zero},&out),LINEARS_ERROR_RANGE);
    EXPECT_TRUE(fc_test::same_bytes(ctx,old));
    EXPECT_TRUE(fc_test::same_bytes(out,previous));
}

TYPED_TEST(FixedSmo, MinimumAndMaximumAlphaKeepTheirActualFixedValues)
{
    using T=TypeParam;
    using A=Api<T>;
    const typename T::scalar least=1;
    typename A::smo_config config{T::make(0),T::make(0.5),T::make(0.125),
        T::make(0.75),T::make(0.5),least,T::make(0.5),least,
        T::make(0.125),T::make(0.5),2,FIELDCTRL_SMO_FORWARD};
    const typename A::ab zero{T::make(0),T::make(0)};
    typename A::smo ctx{};
    typename A::smo_output out{};
    ASSERT_EQ(A::smo_init(&ctx,&config,{T::make(0.5),T::make(0)}),LINEARS_OK);
    for (unsigned n=0;n<2;++n)
        ASSERT_EQ(A::smo_step(&ctx,zero,{T::make(0.75),T::make(0)},&out),LINEARS_OK);
    EXPECT_GT(ctx.emf_alpha_filter.value,0);
    EXPECT_EQ(value(out.electrical_angle),0.75);
    EXPECT_EQ(out.electrical_speed,0);
    EXPECT_EQ(out.observable,0);
    config.emf_alpha=std::numeric_limits<typename T::scalar>::max();
    ASSERT_EQ(A::smo_init(&ctx,&config,{T::make(0.5),T::make(0)}),LINEARS_OK);
    ASSERT_EQ(A::smo_step(&ctx,zero,{T::make(0.75),T::make(0)},&out),LINEARS_OK);
    EXPECT_EQ(out.emf.alpha,T::make(0.75)-1);
    const int64_t lost=std::is_same_v<T,fc_test::Q15>?49152:1;
    EXPECT_EQ(ctx.emf_alpha_filter.value,INT64_C(1610612736)-lost);
}

TYPED_TEST(FixedSmo, UnlockedFrequencyDiagnosticDoesNotNarrowToQ1)
{
    using T=TypeParam;
    using A=Api<T>;
    const typename T::scalar least=1;
    const auto zero=T::make(0);
    const auto maximum=std::numeric_limits<typename T::scalar>::max();
    typename A::smo_config config{zero,T::make(0.5),least,T::make(0.75),T::make(0.5),
        maximum,T::make(0.5),T::make(0.01),T::make(0.125),T::make(0.5),2,FIELDCTRL_SMO_REVERSE};
    typename A::smo ctx{};
    typename A::smo_output out{};
    static_assert(std::is_same_v<decltype(out.electrical_speed),linears_fixed_t>);
    ASSERT_EQ(A::smo_init(&ctx,&config,{zero,zero}),LINEARS_OK);
    ASSERT_EQ(A::smo_step(&ctx,{zero,T::make(-0.5)},{zero,zero},&out),LINEARS_OK);
    ASSERT_EQ(A::smo_step(&ctx,{T::make(-0.5),zero},{zero,zero},&out),LINEARS_OK);
    EXPECT_GT(std::abs(value(out.electrical_speed)),1);
    EXPECT_EQ(out.observable,0);
    EXPECT_EQ(out.locked,0);
}

TYPED_TEST(FixedSmo, MathematicalEndpointsExportAndActualRangeFailureAreAtomic)
{
    using T=TypeParam;
    using A=Api<T>;
    const auto zero=T::make(0);
    typename A::smo_config config{zero,T::make(0.5),T::make(0.5),T::make(0.25),T::make(0.5),
        T::make(0.5),T::make(0.5),T::make(0.01),T::make(0.125),T::make(0.375),2,FIELDCTRL_SMO_FORWARD};
    for (int direction : {-1,1}) {
        typename A::smo ctx{};
        typename A::smo_output out{};
        const auto half=T::make(direction*0.5);
        ASSERT_EQ(A::smo_init(&ctx,&config,{half,zero}),LINEARS_OK);
        ASSERT_EQ(A::smo_step(&ctx,{half,zero},{half,zero},&out),LINEARS_OK);
        EXPECT_EQ(ctx.current_hat.alpha,direction*LINEARS_FIXED_ONE);
        EXPECT_EQ(out.current_hat.alpha,direction>0?std::numeric_limits<typename T::scalar>::max():
            std::numeric_limits<typename T::scalar>::min());
        const auto old=ctx;
        const auto previous=out;
        EXPECT_EQ(A::smo_step(&ctx,{half,zero},{half,zero},&out),LINEARS_ERROR_RANGE);
        EXPECT_TRUE(fc_test::same_bytes(ctx,old));
        EXPECT_TRUE(fc_test::same_bytes(out,previous));
    }
}

TYPED_TEST(FixedSmo, SaturationDoesNotComputeAnUnrepresentableErrorBoundaryRatio)
{
    using T=TypeParam;
    using A=Api<T>;
    const typename T::scalar least=1;
    const auto zero=T::make(0);
    const auto maximum=std::numeric_limits<typename T::scalar>::max();
    const auto minimum=std::numeric_limits<typename T::scalar>::min();
    typename A::smo_config config{zero,T::make(0.5),least,least,least,
        T::make(0.5),T::make(0.5),least,T::make(0.125),T::make(0.5),2,FIELDCTRL_SMO_FORWARD};
    typename A::smo ctx{};
    typename A::smo_output out{};
    ASSERT_EQ(A::smo_init(&ctx,&config,{maximum,zero}),LINEARS_OK);
    ASSERT_EQ(A::smo_step(&ctx,{maximum,zero},{T::make(0.5),zero},&out),LINEARS_OK);
    ASSERT_EQ(ctx.current_hat.alpha,LINEARS_FIXED_ONE);
    // The next error is exactly +2. For Q31, error/boundary would be +2^32,
    // outside int64/F31, but the required bounded injection is simply +K.
    EXPECT_EQ(A::smo_step(&ctx,{minimum,zero},{zero,zero},&out),LINEARS_OK);
}

} // namespace
