#ifndef FIELDCTRL_TEST_SUPPORT_H
#define FIELDCTRL_TEST_SUPPORT_H

#include <gtest/gtest.h>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <type_traits>
#include <stdexcept>
#include "fieldctrl_base.h"

namespace fc_test {

/* Test-only conversions reject accidental SI/gain values in a unit signal.
 * Their independent long-double rounding is not a production conversion call. */
inline linears_q15_t q15(double x) {
    if (!std::isfinite(x) || x < -1 || x > 1) throw std::domain_error("Q15 signal outside [-1,1]");
    const auto raw = static_cast<int64_t>(std::round(static_cast<long double>(x) * 32768));
    return raw == 32768 ? INT16_MAX : static_cast<int16_t>(raw);
}
inline linears_q31_t q31(double x) {
    if (!std::isfinite(x) || x < -1 || x > 1) throw std::domain_error("Q31 signal outside [-1,1]");
    const auto raw = static_cast<int64_t>(std::round(static_cast<long double>(x) * 2147483648.L));
    return raw == INT64_C(2147483648) ? INT32_MAX : static_cast<int32_t>(raw);
}
inline linears_fixed_t work(double x) {
    if (!std::isfinite(x)) throw std::domain_error("nonfinite F31 test coefficient");
    const long double raw = std::round(static_cast<long double>(x) * 2147483648.L);
    if (raw < -9223372036854775808.L || raw >= 9223372036854775808.L)
        throw std::domain_error("F31 test coefficient outside int64 range");
    return static_cast<int64_t>(raw);
}
inline double value(float x) { return x; }
inline double value(double x) { return x; }
inline double value(linears_q15_t x) { return double(x) / 32768.0; }
inline double value(linears_q31_t x) { return double(x) / 2147483648.0; }
inline double value(linears_fixed_t x) { return double(x) / 2147483648.0; }

struct F32 {
    using scalar = float; using coefficient = float; using work_type = float;
    static scalar make(double x) { return static_cast<float>(x); }
    static coefficient gain(double x) { return make(x); }
    static work_type work_make(double x) { return make(x); }
    static constexpr double unit = 1.2e-7; static constexpr bool fixed = false;
};
struct F64 {
    using scalar = double; using coefficient = double; using work_type = double;
    static scalar make(double x) { return x; }
    static coefficient gain(double x) { return x; }
    static work_type work_make(double x) { return x; }
    static constexpr double unit = 2.3e-16; static constexpr bool fixed = false;
};
struct Q15 {
    using scalar = linears_q15_t; using coefficient = linears_fixed_t; using work_type = linears_fixed_t;
    static scalar make(double x) { return q15(x); }
    static coefficient gain(double x) { return work(x); }
    static work_type work_make(double x) { return work(x); }
    static constexpr double unit = 1.0 / 32768.0; static constexpr bool fixed = true;
};
struct Q31 {
    using scalar = linears_q31_t; using coefficient = linears_fixed_t; using work_type = linears_fixed_t;
    static scalar make(double x) { return q31(x); }
    static coefficient gain(double x) { return work(x); }
    static work_type work_make(double x) { return work(x); }
    static constexpr double unit = 1.0 / 2147483648.0; static constexpr bool fixed = true;
};

using Types = ::testing::Types<F32, F64, Q15, Q31>;

template<class T> struct Api;
#define FC_TEST_API(tag, suffix) \
    template<> struct Api<tag> : tag { \
        using scalar = tag::scalar; \
        using ab = fieldctrl_##suffix##_ab_t; \
        using abc = fieldctrl_##suffix##_abc_t; \
        using dq = fieldctrl_##suffix##_dq_t; \
        using pi = fieldctrl_##suffix##_pi_t; \
        using vvvf = fieldctrl_##suffix##_vvvf_t; \
        using vvvf_output = fieldctrl_##suffix##_vvvf_output_t; \
        static constexpr auto wrap = fieldctrl_##suffix##_wrap; \
        static constexpr auto sincos = fieldctrl_##suffix##_sincos; \
        static constexpr auto atan2 = fieldctrl_##suffix##_atan2; \
        static constexpr auto clarke = fieldctrl_##suffix##_clarke; \
        static constexpr auto inverse_clarke = fieldctrl_##suffix##_inverse_clarke; \
        static constexpr auto park = fieldctrl_##suffix##_park; \
        static constexpr auto inverse_park = fieldctrl_##suffix##_inverse_park; \
        static constexpr auto limit_voltage = fieldctrl_##suffix##_limit_voltage; \
        static constexpr auto svpwm = fieldctrl_##suffix##_svpwm; \
        static constexpr auto pi_init = fieldctrl_##suffix##_pi_init; \
        static constexpr auto pi_step = fieldctrl_##suffix##_pi_step; \
        static constexpr auto vvvf_init = fieldctrl_##suffix##_vvvf_init; \
        static constexpr auto vvvf_step = fieldctrl_##suffix##_vvvf_step; \
    };
FC_TEST_API(F32, f32)
FC_TEST_API(F64, f64)
FC_TEST_API(Q15, q15)
FC_TEST_API(Q31, q31)
#undef FC_TEST_API

inline double wrap(double x) { x -= std::floor(x); return x == 1 ? 0 : x; }
inline double turn_error(double a, double b) { return std::remainder(a - b, 1.0); }

template<class T> bool same_bytes(const T& a, const T& b) {
    return std::memcmp(&a, &b, sizeof(T)) == 0;
}
} // namespace fc_test
#endif
