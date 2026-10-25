#ifndef FIELDCTRL_FOC_SMO_TEST_SUPPORT_H
#define FIELDCTRL_FOC_SMO_TEST_SUPPORT_H

#include "support.h"
#include "fieldctrl_foc.h"
#include "fieldctrl_smo.h"
#include <array>
#include <complex>
#include <type_traits>

namespace fc_loop_test {
using fc_test::value;
constexpr double tau = 6.283185307179586476925286766559;

template<class Tag> struct Api;
#define FC_LOOP_API(Tag, S) \
    template<> struct Api<fc_test::Tag> : fc_test::Api<fc_test::Tag> { \
        using foc = fieldctrl_##S##_foc_t; \
        using foc_config = fieldctrl_##S##_foc_config_t; \
        using foc_output = fieldctrl_##S##_foc_output_t; \
        using speed = fieldctrl_##S##_speed_foc_t; \
        using speed_config = fieldctrl_##S##_speed_foc_config_t; \
        using speed_output = fieldctrl_##S##_speed_foc_output_t; \
        using smo = fieldctrl_##S##_smo_t; \
        using smo_config = fieldctrl_##S##_smo_config_t; \
        using smo_output = fieldctrl_##S##_smo_output_t; \
        static constexpr auto foc_init = fieldctrl_##S##_foc_init; \
        static constexpr auto foc_step = fieldctrl_##S##_foc_step; \
        static constexpr auto speed_init = fieldctrl_##S##_speed_foc_init; \
        static constexpr auto speed_step = fieldctrl_##S##_speed_foc_step; \
        static constexpr auto smo_init = fieldctrl_##S##_smo_init; \
        static constexpr auto smo_step = fieldctrl_##S##_smo_step; \
    };
FC_LOOP_API(F32, f32)
FC_LOOP_API(F64, f64)
FC_LOOP_API(Q15, q15)
FC_LOOP_API(Q31, q31)
#undef FC_LOOP_API

template<class Tag> typename Tag::scalar invalid_scalar()
{
    static_assert(!Tag::fixed, "Every fixed signal bit pattern is valid");
    return std::numeric_limits<typename Tag::scalar>::infinity();
}

template<class Tag> typename Tag::scalar angle_input(double x)
{
    const double wrapped=fc_test::wrap(x);
    if constexpr (Tag::fixed) {
        const double units=std::is_same_v<Tag,fc_test::Q15>?32768.0:2147483648.0;
        if (std::round(wrapped*units)>=units) return Tag::make(0);
    }
    auto result=Tag::make(wrapped);
    return value(result)>=1?Tag::make(0):result;
}

/* A bounded scalar/coordinate pipeline has at most twelve final-precision
 * rounding contributions here. For two 32-round Q60/Q62 CORDIC rotations,
 * residual angle <= 2^-31 rad plus Q31 component rounding gives < 2e-9 per
 * rotation. Reserve 4e-9 separately from public-scalar rounding. */
template<class Tag> double numeric_budget(double scale)
{
    if constexpr (Tag::fixed)
        return 12*Tag::unit+4e-9*std::max(1.0,std::abs(scale));
    else
        return 12*Tag::unit*std::max(1.0,std::abs(scale));
}

/* Host-only unit conversion for the independent physical plant. None of
 * these bases, or a floating conversion, exists in a production Q context. */
template<class Tag> struct Units {
    double v=1, i=1, t=1;
    typename Tag::scalar voltage(double x) const { return Tag::make(x/v); }
    typename Tag::scalar current(double x) const { return Tag::make(x/i); }
    typename Tag::scalar time(double x) const { return Tag::make(x/t); }
    typename Tag::scalar frequency(double x) const { return Tag::make(x*t); }
    template<class N> double volts(N x) const { return value(x)*v; }
    template<class N> double amps(N x) const { return value(x)*i; }
    template<class N> double seconds(N x) const { return value(x)*t; }
    template<class N> double hertz(N x) const { return value(x)/t; }
};

template<class Tag> Units<Tag> current_units()
{
    if constexpr (Tag::fixed) return {128,32,0.005};
    else return {};
}

template<class Tag> Units<Tag> observer_units()
{
    if constexpr (Tag::fixed) return {32,4,0.005};
    else return {};
}

template<class Tag> Units<Tag> speed_units()
{
    if constexpr (Tag::fixed) return {64,8,0.005};
    else return {};
}

template<class Tag> typename Api<Tag>::foc_config current_config(
    double kp=2, double ki=400, double limit=12,
    Units<Tag> u=current_units<Tag>())
{
    return {Tag::gain(kp*u.i/u.v), Tag::gain(ki*u.i*u.t/u.v),
        Tag::gain(kp*u.i/u.v), Tag::gain(ki*u.i*u.t/u.v),u.voltage(limit)};
}

template<class Tag> typename Api<Tag>::smo_config observer_config(int direction=1,
    Units<Tag> u=observer_units<Tag>())
{
    return {Tag::make(u.i/u.v),Tag::make(0.01*u.i/(u.v*u.t)),u.time(0.0001),
        u.voltage(8),u.current(0.2),Tag::make(0.15),Tag::make(0.2),
        u.voltage(0.15),u.frequency(2),u.frequency(100),32,
        direction>0?FIELDCTRL_SMO_FORWARD:FIELDCTRL_SMO_REVERSE};
}

inline std::array<double, 2> rotate(double d, double q, double angle)
{
    const double s = std::sin(tau * angle), c = std::cos(tau * angle);
    return {c*d-s*q, s*d+c*q};
}

inline std::array<double, 2> park(double a, double b, double angle)
{
    const double s = std::sin(tau * angle), c = std::cos(tau * angle);
    return {c*a+s*b, -s*a+c*b};
}

/* Reconstruct line-neutral voltage from the actual three duty commands; the
 * plant never consumes an unlimited PI voltage or an unrounded internal state. */
template<class ABC> std::array<double, 2> applied_voltage(const ABC& duty, double bus)
{
    const double mean = (value(duty.a) + value(duty.b) + value(duty.c)) / 3;
    return {bus * (value(duty.a)-mean),
        bus * (value(duty.b)-value(duty.c)) / std::sqrt(3.0)};
}

struct Motor {
    double resistance = 0.5, inductance = 0.01, flux = 0.02;
    double alpha = 0, beta = 0, angle = 0, mechanical_speed = 0;
    unsigned pole_pairs = 3;

    void electrical_step(std::array<double, 2> voltage, double frequency, double dt)
    {
        const auto emf = rotate(0, tau * frequency * flux, angle);
        alpha += dt / inductance * (voltage[0]-resistance*alpha-emf[0]);
        beta += dt / inductance * (voltage[1]-resistance*beta-emf[1]);
        angle = fc_test::wrap(angle + frequency*dt);
    }
};

/* Independent real-valued recurrence; only externally supplied inputs and
 * configuration have API quantization. Persistent observer state is not q15. */
struct ObserverReference {
    double ia = 0, ib = 0, ea = 0, eb = 0;
    void step(double ca, double cb, double va, double vb, double r, double l,
        double dt, double k, double boundary, double alpha)
    {
        const double za = k*std::clamp((ia-ca)/boundary, -1.0, 1.0);
        const double zb = k*std::clamp((ib-cb)/boundary, -1.0, 1.0);
        ia += dt/l*(va-r*ia-za);
        ib += dt/l*(vb-r*ib-zb);
        ea = (1-alpha)*ea+alpha*za;
        eb = (1-alpha)*eb+alpha*zb;
    }
};

/* Independent real-valued angle/difference/LP reference. It receives either
 * quantized samples or an exact-sensor control sequence from the same plant. */
struct MotionReference {
    ObserverReference observer;
    double previous_angle=0, speed=0;
    bool has_angle=false;

    void step(double ia,double ib,double va,double vb,double r,double l,double dt,
        double k,double boundary,double alpha,double speed_alpha,double threshold,int direction)
    {
        observer.step(ia,ib,va,vb,r,l,dt,k,boundary,alpha);
        if (std::hypot(observer.ea,observer.eb)<threshold) {
            has_angle=false;
            speed=0;
            return;
        }
        const double angle=fc_test::wrap(std::atan2(-direction*observer.ea,
            direction*observer.eb)/tau);
        if (has_angle)
            speed=(1-speed_alpha)*speed+speed_alpha*fc_test::turn_error(angle,previous_angle)/dt;
        previous_angle=angle;
        has_angle=true;
    }
};
} // namespace fc_loop_test
#endif
