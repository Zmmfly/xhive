#ifndef FILTERS_TEST_SUPPORT_H
#define FILTERS_TEST_SUPPORT_H

#include "filters.h"
#include <gtest/gtest.h>
#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <limits>
#include <random>
#include <stdexcept>
#include <type_traits>
#include <vector>

namespace filters_test {

/* Test-only conversion: independent host arithmetic encodes the public Q
 * representation, not the production linears import/export implementation. */
template <typename T, int Bits> struct FixedNumeric {
    using Scalar = T;
    using Coefficient = linears_fixed_t;
    using State = linears_fixed_t;
    static constexpr bool fixed = true;
    static constexpr int bits = Bits;
    static constexpr double unit = Bits == 15 ? 0x1p-15 : 0x1p-31;
    static constexpr double work_unit = 0x1p-31;
    static Scalar make(double value) {
        if (!std::isfinite(value) || value < -1 || value > 1)
            throw std::out_of_range("Q signal fixture must be in [-1,+1]");
        const auto raw = static_cast<std::int64_t>(std::round(std::ldexp(value, Bits)));
        return raw > std::numeric_limits<Scalar>::max()
            ? std::numeric_limits<Scalar>::max() : static_cast<Scalar>(raw);
    }
    static double real(Scalar value) { return std::ldexp(static_cast<double>(value), -Bits); }
    static Coefficient coefficient(double value) {
        const long double raw = std::round(std::ldexp(static_cast<long double>(value), 31));
        if (!std::isfinite(value) || raw < static_cast<long double>(INT64_MIN) || raw > static_cast<long double>(INT64_MAX))
            throw std::out_of_range("F31 coefficient fixture does not fit int64_t");
        return static_cast<Coefficient>(raw);
    }
    static double real_coefficient(Coefficient value) { return std::ldexp(static_cast<double>(value), -31); }
    static State state_make(double value) { return coefficient(value); }
    static double state_real(State value) { return real_coefficient(value); }
    static Scalar huge() { return std::numeric_limits<Scalar>::max(); }
    static Scalar tiny() { return 1; }
};

template <typename T> struct FloatNumeric {
    using Scalar = T;
    using Coefficient = T;
    using State = T;
    static constexpr bool fixed = false;
    static constexpr double unit = std::numeric_limits<T>::epsilon();
    static constexpr double work_unit = unit;
    static Scalar make(double value) { return static_cast<T>(value); }
    static double real(Scalar value) { return static_cast<double>(value); }
    static Coefficient coefficient(double value) { return make(value); }
    static double real_coefficient(Coefficient value) { return real(value); }
    static State state_make(double value) { return make(value); }
    static double state_real(State value) { return real(value); }
    static Scalar invalid() { return std::numeric_limits<T>::quiet_NaN(); }
    static Scalar huge() { return std::numeric_limits<T>::max(); }
    static Scalar tiny() { return std::numeric_limits<T>::min(); }
};

#define FILTERS_TEST_BINDINGS(suffix) \
    using Lowpass = filters_lowpass_##suffix##_t; \
    using Highpass = filters_highpass_##suffix##_t; \
    using Ema = filters_ema_##suffix##_t; \
    using Fir = filters_fir_##suffix##_t; \
    using Biquad = filters_biquad_##suffix##_t; \
    using Coefficients = filters_biquad_##suffix##_coefficients_t; \
    using Average = filters_moving_average_##suffix##_t; \
    using Median = filters_median_##suffix##_t; \
    using Complementary = filters_complementary_##suffix##_t; \
    static constexpr auto lowpass_init = filters_lowpass_##suffix##_init; \
    static constexpr auto lowpass_reset = filters_lowpass_##suffix##_reset; \
    static constexpr auto lowpass_step = filters_lowpass_##suffix##_step; \
    static constexpr auto highpass_init = filters_highpass_##suffix##_init; \
    static constexpr auto highpass_reset = filters_highpass_##suffix##_reset; \
    static constexpr auto highpass_step = filters_highpass_##suffix##_step; \
    static constexpr auto ema_init = filters_ema_##suffix##_init; \
    static constexpr auto ema_reset = filters_ema_##suffix##_reset; \
    static constexpr auto ema_step = filters_ema_##suffix##_step; \
    static constexpr auto fir_init = filters_fir_##suffix##_init; \
    static constexpr auto fir_reset = filters_fir_##suffix##_reset; \
    static constexpr auto fir_step = filters_fir_##suffix##_step; \
    static constexpr auto biquad_init = filters_biquad_##suffix##_init; \
    static constexpr auto biquad_reset = filters_biquad_##suffix##_reset; \
    static constexpr auto biquad_step = filters_biquad_##suffix##_step; \
    static constexpr auto average_init = filters_moving_average_##suffix##_init; \
    static constexpr auto average_reset = filters_moving_average_##suffix##_reset; \
    static constexpr auto average_step = filters_moving_average_##suffix##_step; \
    static constexpr auto median_init = filters_median_##suffix##_init; \
    static constexpr auto median_reset = filters_median_##suffix##_reset; \
    static constexpr auto median_step = filters_median_##suffix##_step; \
    static constexpr auto complementary_init = filters_complementary_##suffix##_init; \
    static constexpr auto complementary_reset = filters_complementary_##suffix##_reset; \
    static constexpr auto complementary_step = filters_complementary_##suffix##_step

struct F32 : FloatNumeric<float> { FILTERS_TEST_BINDINGS(f32); };
struct F64 : FloatNumeric<double> { FILTERS_TEST_BINDINGS(f64); };
struct Q15 : FixedNumeric<linears_q15_t, 15> { FILTERS_TEST_BINDINGS(q15); };
struct Q31 : FixedNumeric<linears_q31_t, 31> { FILTERS_TEST_BINDINGS(q31); };
#undef FILTERS_TEST_BINDINGS
using Formats = ::testing::Types<F32, F64, Q15, Q31>;

template <typename A> double rounded(double value) { return A::real(A::make(value)); }
template <typename A> double state_rounded(double value) { return A::state_real(A::state_make(value)); }

template <typename A> void expect_close(typename A::Scalar result, double expected,
                                         double storage_ulps = 4, double operation_scale = 0) {
    /* Fixed signal LSBs (or floating relative units), not an ad hoc decimal tolerance.
     * The storage bound covers the final rounding; operation_scale bounds the
     * sum of working-precision error magnitudes in the independent recurrence. */
    const double tolerance = storage_ulps * A::unit * std::max(1.0, std::abs(expected))
                           + operation_scale * A::work_unit;
    EXPECT_NEAR(A::real(result), expected, tolerance);
}

template <typename T> void expect_unchanged(const T &current, const T &before) {
    EXPECT_EQ(std::memcmp(&current, &before, sizeof(T)), 0);
}

template <typename A, bool Work = false> struct Guards {
    using S = std::conditional_t<Work, typename A::State, typename A::Scalar>;
    static S sentinel() {
        if constexpr (Work) return A::state_make(0.875);
        else return A::make(0.875);
    }
    static double real(S value) {
        if constexpr (Work) return A::state_real(value);
        else return A::real(value);
    }
    std::vector<S> values;
    explicit Guards(std::size_t used) : values(used + 2, sentinel()) {}
    S *data() { return values.data() + 1; }
    void check() const {
        EXPECT_EQ(real(values.front()), 0.875);
        EXPECT_EQ(real(values.back()), 0.875);
    }
};
template <typename A> using StateGuards = Guards<A, true>;

} // namespace filters_test
#endif
