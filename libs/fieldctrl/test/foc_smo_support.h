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
    if constexpr (Tag::fixed) return {1, LINEARS_EXPONENT_MAX + 1};
    else return std::numeric_limits<typename Tag::scalar>::infinity();
}

template<class Tag> typename Tag::scalar angle_input(double x)
{
    auto result = Tag::make(fc_test::wrap(x));
    return value(result) >= 1 ? Tag::make(0) : result;
}

/* A bounded scalar/coordinate pipeline has at most twelve final-precision
 * rounding contributions here. For two 32-round Q60/Q62 CORDIC rotations,
 * residual angle <= 2^-31 rad plus Q31 component rounding gives < 2e-9 per
 * rotation. Reserve 4e-9 separately from public-scalar rounding. */
template<class Tag> double numeric_budget(double scale)
{
    return (12 * Tag::unit + (Tag::fixed ? 4e-9 : 0)) *
        std::max(1.0, std::abs(scale));
}

template<class Tag> typename Api<Tag>::foc_config current_config(
    double kp = 2, double ki = 400, double limit = 12)
{
    return {Tag::make(kp), Tag::make(ki), Tag::make(kp), Tag::make(ki), Tag::make(limit)};
}

template<class Tag> typename Api<Tag>::smo_config observer_config(int direction = 1)
{
    return {Tag::make(1), Tag::make(0.01), Tag::make(0.0001),
        Tag::make(8), Tag::make(0.2), Tag::make(0.15), Tag::make(0.2),
        Tag::make(0.15), Tag::make(2), Tag::make(100), 32,
        direction > 0 ? FIELDCTRL_SMO_FORWARD : FIELDCTRL_SMO_REVERSE};
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
} // namespace fc_loop_test
#endif
