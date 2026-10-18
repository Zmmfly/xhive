#ifndef FIELDCTRL_TEST_DTC_SUPPORT_H
#define FIELDCTRL_TEST_DTC_SUPPORT_H

#include "support.h"
#include "fieldctrl_dtc.h"
#include <array>
#include <type_traits>

namespace dtc_test {
using fc_test::value;

struct VariantNames {
    template<class Tag> static std::string GetName(int) {
        if constexpr (std::is_same_v<Tag, fc_test::F32>) return "F32";
        else if constexpr (std::is_same_v<Tag, fc_test::F64>) return "F64";
        else if constexpr (std::is_same_v<Tag, fc_test::Q15>) return "Q15";
        else return "Q31";
    }
};

template<class Tag> struct Api;
#define DTC_TEST_API(Tag, suffix) \
    template<> struct Api<fc_test::Tag> : fc_test::Api<fc_test::Tag> { \
        using config = fieldctrl_##suffix##_dtc_config_t; \
        using context = fieldctrl_##suffix##_dtc_t; \
        using result = fieldctrl_##suffix##_dtc_result_t; \
        static constexpr auto init = fieldctrl_##suffix##_dtc_init; \
        static constexpr auto step = fieldctrl_##suffix##_dtc_step; \
    };
DTC_TEST_API(F32, f32)
DTC_TEST_API(F64, f64)
DTC_TEST_API(Q15, q15)
DTC_TEST_API(Q31, q31)
#undef DTC_TEST_API

template<class T> std::array<unsigned char, sizeof(T)> bytes(const T& object) {
    std::array<unsigned char, sizeof(T)> result{};
    std::memcpy(result.data(), &object, sizeof(T));
    return result;
}

/* The upper Q cell has one-LSB error because +1 maps to MAX. Everywhere
 * else export has half-LSB error; the F31 context is not quantized on storage. */
inline long double output_roundoff(linears_q15_t x) {
    return x == INT16_MAX ? 0x1p-15L : 0x1p-16L;
}
inline long double output_roundoff(linears_q31_t x) {
    return x == INT32_MAX ? 0x1p-31L : 0x1p-32L;
}
inline long double output_roundoff(int64_t) { return 0; }
inline long double output_roundoff(float x) {
    return 0.5L * (std::nextafter(std::fabs(x), INFINITY) - std::fabs(x));
}
inline long double output_roundoff(double x) {
    return 0.5L * (std::nextafter(std::fabs(x), INFINITY) - std::fabs(x));
}

template<class Tag> constexpr long double work_roundoff() {
    if constexpr (std::is_same_v<Tag, fc_test::F32>) return 0x1p-24L;
    else return 0x1p-53L;
}

/* Independent SI/pu equations on already-quantized input data. Floating
 * budgets propagate relative roundoff; Q products/divisions use absolute
 * half-F31-LSB error, and Q addition/subtraction are exact. */
struct Estimate {
    long double alpha, beta, magnitude, torque;
    long double alpha_error, beta_error, magnitude_error, torque_error;
};

template<class Tag>
Estimate reference(const typename Api<Tag>::context& ctx,
                   typename Api<Tag>::ab current,
                   typename Api<Tag>::ab voltage,
                   typename Tag::scalar dt) {
    const long double r = value(ctx.config.resistance), t = value(dt);
    const long double ia = value(current.alpha), ib = value(current.beta);
    const long double va = value(voltage.alpha), vb = value(voltage.beta);
    const long double a = value(ctx.flux_alpha), b = value(ctx.flux_beta);
    Estimate e{};
    e.alpha = a + t * (va - r * ia);
    e.beta = b + t * (vb - r * ib);
    e.magnitude = std::hypot(e.alpha, e.beta);
    const long double cross = e.alpha * ib - e.beta * ia;
    const long double p = (1.5L * ctx.config.pole_pairs);
    e.torque = p * cross;
    if constexpr (Tag::fixed) {
        constexpr long double h = 0x1p-32L;
        e.alpha_error = t * h + h;
        e.beta_error = t * h + h;
        const long double hi = std::max(std::abs(e.alpha), std::abs(e.beta));
        // scaled hypot: ratio <=1, square, sqrt at argument >=1, final multiply.
        e.magnitude_error = std::hypot(e.alpha_error, e.beta_error) +
            h + (hi + e.alpha_error + e.beta_error) * (2.5L * h + h * h);
        e.torque_error = p * (std::abs(ib) * e.alpha_error +
                             std::abs(ia) * e.beta_error + 2 * h) + h;
    } else {
        const long double u = work_roundoff<Tag>();
        const long double g = u / (1 - u);
        auto component_error = [g, r, t](long double old, long double i, long double v) {
            const long double ri = r * i;
            const long double ri_err = g * std::abs(ri);
            const long double drop = v - ri;
            const long double drop_err = ri_err + g * (std::abs(drop) + ri_err);
            const long double increment = t * drop;
            const long double increment_err = t * drop_err +
                g * (std::abs(increment) + t * drop_err);
            return increment_err + g * (std::abs(old + increment) + increment_err);
        };
        e.alpha_error = component_error(a, ia, va);
        e.beta_error = component_error(b, ib, vb);
        e.magnitude_error = std::hypot(e.alpha_error, e.beta_error) + g * e.magnitude;
        const long double aerr = std::abs(ib) * e.alpha_error;
        const long double berr = std::abs(ia) * e.beta_error;
        const long double cross_err = aerr + berr +
            g * (std::abs(e.alpha * ib) + aerr + std::abs(e.beta * ia) + berr) +
            g * (std::abs(cross) + aerr + berr);
        e.torque_error = p * cross_err + g * (std::abs(e.torque) + p * cross_err);
    }
    return e;
}

template<class T> void near(long double expected, long double arithmetic_error, T actual) {
    const long double discrepancy = std::abs(static_cast<long double>(value(actual)) - expected);
    /* Long-double oracle evaluation has <= a few double-rounding units here. */
    const long double oracle_error = 16 * std::numeric_limits<long double>::epsilon() *
        (std::abs(expected) + arithmetic_error);
    EXPECT_LE(discrepancy, arithmetic_error + output_roundoff(actual) + oracle_error)
        << "expected=" << static_cast<double>(expected)
        << " actual=" << value(actual)
        << " arithmetic budget=" << static_cast<double>(arithmetic_error);
}

/* Independent surface-PMSM plant: alpha/beta differential current equations,
 * rotor motion and mechanical inertia. It never consumes the DTC flux output.
 * The true rotor angle is used only by these plant equations and diagnostics. */
struct Plant {
    static constexpr double resistance = 0.4;
    static constexpr double inductance = 0.003;
    static constexpr double magnet_flux = 0.08;
    static constexpr double inertia = 0.001;
    static constexpr double friction = 0.01;
    static constexpr unsigned pole_pairs = 3;
    double ia = 0, ib = 0, theta = 0, omega = 0;

    double torque() const {
        return 1.5 * pole_pairs * magnet_flux *
            (-std::sin(theta) * ia + std::cos(theta) * ib);
    }
    std::array<double, 2> flux() const {
        return {inductance * ia + magnet_flux * std::cos(theta),
                inductance * ib + magnet_flux * std::sin(theta)};
    }
    Plant derivative(double va, double vb) const {
        const double we = pole_pairs * omega;
        return {(va - resistance * ia + we * magnet_flux * std::sin(theta)) / inductance,
                (vb - resistance * ib - we * magnet_flux * std::cos(theta)) / inductance,
                we, (torque() - friction * omega) / inertia};
    }
    Plant plus(const Plant& d, double t) const {
        return {ia + t * d.ia, ib + t * d.ib, theta + t * d.theta, omega + t * d.omega};
    }
    void advance(double va, double vb, double dt) {
        const Plant k1 = derivative(va, vb);
        const Plant k2 = plus(k1, dt / 2).derivative(va, vb);
        const Plant k3 = plus(k2, dt / 2).derivative(va, vb);
        const Plant k4 = plus(k3, dt).derivative(va, vb);
        ia += dt / 6 * (k1.ia + 2 * k2.ia + 2 * k3.ia + k4.ia);
        ib += dt / 6 * (k1.ib + 2 * k2.ib + 2 * k3.ib + k4.ib);
        theta += dt / 6 * (k1.theta + 2 * k2.theta + 2 * k3.theta + k4.theta);
        omega += dt / 6 * (k1.omega + 2 * k2.omega + 2 * k3.omega + k4.omega);
    }
};

inline std::array<double, 2> inverter_voltage(uint8_t state, double bus) {
    const double a = (state >> 2) & 1u, b = (state >> 1) & 1u, c = state & 1u;
    return {bus * (2 * a - b - c) / 3, bus * (b - c) / std::sqrt(3.0)};
}
} // namespace dtc_test
#endif
