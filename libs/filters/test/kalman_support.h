#ifndef FILTERS_TEST_KALMAN_SUPPORT_H
#define FILTERS_TEST_KALMAN_SUPPORT_H

#include "filters_kalman.h"
#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <random>
#include <type_traits>
#include <vector>

namespace kalman_test {

/* Test-only conversion is independent of the production integer numerical core. */
template<class Raw, class Scalar, int Bits> struct Codec {
    static Scalar encode(long double value) {
        if constexpr (Bits == 0) {
            return static_cast<Scalar>(value);
        } else {
            int exponent = 0;
            if (value == 0) return Scalar{0, 0};
            const long double fraction = std::frexp(value, &exponent);
            long long mantissa = std::llround(std::ldexp(fraction, Bits));
            if (mantissa > std::numeric_limits<Raw>::max()) {
                ++exponent;
                mantissa = std::llround(std::ldexp(value, Bits - exponent));
            }
            return Scalar{static_cast<Raw>(mantissa), exponent};
        }
    }
    static long double decode(Scalar value) {
        if constexpr (Bits == 0) return value;
        else return std::ldexp(static_cast<long double>(value.mantissa), value.exponent - Bits);
    }
    static long double decode_raw(Raw value, int exponent) {
        if constexpr (Bits == 0) { (void)exponent; return value; }
        else return std::ldexp(static_cast<long double>(value), exponent - Bits);
    }
    static std::vector<Raw> pack(const std::vector<long double>& values, int& exponent) {
        std::vector<Raw> result(values.size());
        exponent = 0;
        if constexpr (Bits == 0) {
            std::transform(values.begin(), values.end(), result.begin(),
                           [](long double value) { return static_cast<Raw>(value); });
        } else {
            long double maximum = 0;
            for (long double value : values) maximum = std::max(maximum, std::abs(value));
            if (maximum != 0) std::frexp(maximum, &exponent);
            for (long double value : values) {
                if (std::llround(std::ldexp(value, Bits - exponent)) > std::numeric_limits<Raw>::max()) {
                    ++exponent;
                    break;
                }
            }
            for (std::size_t i = 0; i < values.size(); ++i)
                result[i] = static_cast<Raw>(std::llround(std::ldexp(values[i], Bits - exponent)));
        }
        return result;
    }
};

#define KALMAN_TEST_API(tag, raw_type, scalar_type, working_type, fraction, error) \
struct tag : Codec<raw_type, scalar_type, fraction> { \
    using Raw = raw_type; \
    using Scalar = scalar_type; \
    using Working = working_type; \
    using State = filters_kalman_scalar_##tag##_t; \
    using Workspace = filters_kalman_matrix_##tag##_workspace_t; \
    static constexpr int bits = fraction; \
    static constexpr long double tolerance = error; \
    static linears_status_t reset(State *s, Scalar x, Scalar p) \
    { return filters_kalman_scalar_##tag##_reset(s, x, p); } \
    static linears_status_t predict(State *s, Scalar a, Scalar q) \
    { return filters_kalman_scalar_##tag##_predict(s, a, q); } \
    static linears_status_t update(State *s, Scalar z, Scalar h, Scalar r) \
    { return filters_kalman_scalar_##tag##_update(s, z, h, r); } \
    static linears_status_t reset_matrix(filters_kalman_matrix_t *s, \
        const linears_matrix_t *x, const linears_matrix_t *p) \
    { return filters_kalman_matrix_##tag##_reset(s, x, p); } \
    static linears_status_t predict_matrix(filters_kalman_matrix_t *s, \
        const linears_matrix_t *f, const linears_matrix_t *q, Workspace w) \
    { return filters_kalman_matrix_##tag##_predict(s, f, q, w); } \
    static linears_status_t update_matrix(filters_kalman_matrix_t *s, const linears_matrix_t *z, \
        const linears_matrix_t *h, const linears_matrix_t *r, Workspace w) \
    { return filters_kalman_matrix_##tag##_update(s, z, h, r, w); } \
};
KALMAN_TEST_API(f32, float, float, float, 0, 2e-5L)
KALMAN_TEST_API(f64, double, double, double, 0, 2e-12L)
KALMAN_TEST_API(q15, int16_t, linears_q15_t, linears_fixed_t, 15, 4e-3L)
KALMAN_TEST_API(q31, int32_t, linears_q31_t, linears_fixed_t, 31, 2e-7L)
#undef KALMAN_TEST_API

using Formats = ::testing::Types<f32, f64, q15, q31>;

template<class T> auto bytes(const T& object) {
    std::array<unsigned char, sizeof(T)> result{};
    std::memcpy(result.data(), &object, sizeof(T));
    return result;
}

template<class Api> void expect_near(long double actual, long double expected,
                                    long double multiplier = 1) {
    EXPECT_LE(std::abs(actual - expected), Api::tolerance * multiplier * (1 + std::abs(expected)))
        << "actual=" << static_cast<double>(actual) << " expected=" << static_cast<double>(expected);
}

struct ScalarReference {
    long double x;
    long double p;
    void predict(long double a, long double q) { x *= a; p = a * p * a + q; }
    void update(long double z, long double h, long double r) {
        const long double k = p * h / (h * p * h + r);
        x += k * (z - h * x);
        /* Independent exact-arithmetic equivalent, not production Joseph code. */
        p = p * r / (h * p * h + r);
    }
};

struct Dense {
    std::size_t rows, cols;
    std::vector<long double> values;
    Dense(std::size_t r, std::size_t c) : rows(r), cols(c), values(r * c, 0) {}
    Dense(std::size_t r, std::size_t c, std::vector<long double> v)
        : rows(r), cols(c), values(std::move(v)) {}
    long double& operator()(std::size_t r, std::size_t c) { return values[r * cols + c]; }
    long double operator()(std::size_t r, std::size_t c) const { return values[r * cols + c]; }
    static Dense identity(std::size_t n, long double diagonal = 1) {
        Dense result(n, n);
        for (std::size_t i = 0; i < n; ++i) result(i, i) = diagonal;
        return result;
    }
    Dense transpose() const {
        Dense result(cols, rows);
        for (std::size_t r = 0; r < rows; ++r)
            for (std::size_t c = 0; c < cols; ++c) result(c, r) = (*this)(r, c);
        return result;
    }
};
inline Dense operator*(const Dense& a, const Dense& b) {
    Dense result(a.rows, b.cols);
    for (std::size_t i = 0; i < a.rows; ++i)
        for (std::size_t j = 0; j < b.cols; ++j)
            for (std::size_t k = 0; k < a.cols; ++k) result(i, j) += a(i, k) * b(k, j);
    return result;
}
inline Dense operator+(const Dense& a, const Dense& b) {
    Dense result = a;
    for (std::size_t i = 0; i < a.values.size(); ++i) result.values[i] += b.values[i];
    return result;
}
inline Dense operator-(const Dense& a, const Dense& b) {
    Dense result = a;
    for (std::size_t i = 0; i < a.values.size(); ++i) result.values[i] -= b.values[i];
    return result;
}
template<class Api> struct Matrix {
    std::size_t rows, cols;
    int exponent = 0;
    std::vector<typename Api::Raw> data;
    explicit Matrix(const Dense& d) : rows(d.rows), cols(d.cols), data(Api::pack(d.values, exponent)) {}
    linears_matrix_t view() { return {data.data(), rows, cols, exponent}; }
};
template<class Api> Dense decode(const linears_matrix_t& matrix) {
    Dense result(matrix.rows, matrix.cols);
    const auto *raw = static_cast<const typename Api::Raw*>(matrix.data);
    for (std::size_t i = 0; i < result.values.size(); ++i)
        result.values[i] = Api::decode_raw(raw[i], matrix.exponent);
    return result;
}
template<class Api> void expect_matrix(const linears_matrix_t& actual, const Dense& expected,
                                      long double multiplier = 1) {
    ASSERT_EQ(actual.rows, expected.rows);
    ASSERT_EQ(actual.cols, expected.cols);
    const Dense values = decode<Api>(actual);
    for (std::size_t i = 0; i < values.values.size(); ++i) {
        SCOPED_TRACE(i);
        expect_near<Api>(values.values[i], expected.values[i], multiplier);
    }
}

template<class Api> struct MatrixState {
    using Raw = typename Api::Raw;
    std::vector<Raw> x, p;
    filters_kalman_matrix_t state;
    static constexpr Raw sentinel = static_cast<Raw>(-12345);
    explicit MatrixState(std::size_t n) : x(n + 2, sentinel), p(n * n + 2, sentinel),
        state{{x.data() + 1, n, 1, 0}, {p.data() + 1, n, n, 0}, n, n * n} {}
    void guards() const {
        EXPECT_EQ(x.front(), sentinel); EXPECT_EQ(x.back(), sentinel);
        EXPECT_EQ(p.front(), sentinel); EXPECT_EQ(p.back(), sentinel);
    }
};

template<class Api> struct Workspace {
    using Raw = typename Api::Raw;
    using Working = typename Api::Working;
    std::size_t raw_count, basic_bytes, decomp_count;
    std::vector<Raw> raw;
    std::vector<linears_workspace_unit_t> basic;
    std::vector<Working> decomp;
    explicit Workspace(std::size_t n, std::size_t m = 0)
        : raw_count(m ? FILTERS_KALMAN_UPDATE_STORAGE(n, m) : FILTERS_KALMAN_PREDICT_STORAGE(n)),
          basic_bytes(m ? FILTERS_KALMAN_UPDATE_BASIC_BYTES(n, m) : FILTERS_KALMAN_PREDICT_BASIC_BYTES(n)),
          decomp_count(m ? FILTERS_KALMAN_UPDATE_DECOMP_STORAGE(n, m) : 0),
          raw(raw_count + 2, static_cast<Raw>(-23456)),
          basic(LINEARS_WORKSPACE_UNITS(basic_bytes) + 2), decomp(decomp_count + 2) {
        std::memset(basic.data(), 0x35, basic.size() * sizeof(basic[0]));
        std::memset(decomp.data(), 0x46, decomp.size() * sizeof(decomp[0]));
    }
    typename Api::Workspace view() {
        return {raw.data() + 1, raw_count, basic.data() + 1, basic_bytes,
                {decomp_count ? decomp.data() + 1 : nullptr, decomp_count}};
    }
    void guards() const {
        EXPECT_EQ(raw.front(), static_cast<Raw>(-23456)); EXPECT_EQ(raw.back(), static_cast<Raw>(-23456));
        const auto prefix = bytes(basic.front()), suffix = bytes(basic.back());
        for (auto c : prefix) EXPECT_EQ(c, 0x35);
        for (auto c : suffix) EXPECT_EQ(c, 0x35);
        const auto first = bytes(decomp.front()), last = bytes(decomp.back());
        for (auto c : first) EXPECT_EQ(c, 0x46);
        for (auto c : last) EXPECT_EQ(c, 0x46);
    }
};

inline Dense random_dense(std::mt19937& random, std::size_t rows, std::size_t cols,
                          long double magnitude = 1) {
    Dense result(rows, cols);
    std::uniform_real_distribution<double> distribution(-1, 1);
    for (auto& value : result.values) value = magnitude * distribution(random);
    return result;
}
inline Dense random_spd(std::mt19937& random, std::size_t n, long double floor = 1) {
    const Dense a = random_dense(random, n, n, 0.4L);
    return a * a.transpose() + Dense::identity(n, floor);
}

} // namespace kalman_test
#endif
