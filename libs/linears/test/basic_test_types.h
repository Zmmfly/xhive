#ifndef LINEARS_BASIC_TEST_TYPES_H
#define LINEARS_BASIC_TEST_TYPES_H

#include "linears.h"
#include <Eigen/Dense>
#include <gtest/gtest.h>
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <type_traits>
#include <vector>

namespace linears_test {
using Dense = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>;

#define BASIC_TRAIT(NAME, SUFFIX, RAW, SCALAR, BITS, TOLERANCE) \
struct NAME { \
    using Raw = RAW; using Scalar = SCALAR; \
    static constexpr int bits = BITS; \
    static constexpr double tolerance = TOLERANCE; \
    static constexpr auto add = linears_add_##SUFFIX; \
    static constexpr auto sub = linears_sub_##SUFFIX; \
    static constexpr auto hadamard = linears_hadamard_##SUFFIX; \
    static constexpr auto mul = linears_mul_##SUFFIX; \
    static constexpr auto matvec = linears_matvec_##SUFFIX; \
    static constexpr auto transpose = linears_transpose_##SUFFIX; \
    static constexpr auto axpby = linears_axpby_##SUFFIX; \
    static constexpr auto scale = linears_scale_##SUFFIX; \
    static constexpr auto copy = linears_copy_##SUFFIX; \
    static constexpr auto zero = linears_zero_##SUFFIX; \
    static constexpr auto identity = linears_identity_##SUFFIX; \
    static constexpr auto dot = linears_dot_##SUFFIX; \
    static constexpr auto norm = linears_norm_##SUFFIX; \
    static constexpr auto trace = linears_trace_##SUFFIX; \
};

BASIC_TRAIT(F32, f32, float, float, 0, 3e-6)
BASIC_TRAIT(F64, f64, double, double, 0, 2e-13)
BASIC_TRAIT(Q15, q15, int16_t, linears_q15_t, 15, 9e-5)
BASIC_TRAIT(Q31, q31, int32_t, linears_q31_t, 31, 9e-9)
#undef BASIC_TRAIT

using Types = ::testing::Types<F32, F64, Q15, Q31>;

template<class T> struct Matrix {
    std::vector<typename T::Raw> storage;
    linears_matrix_t view;
    Matrix(size_t rows, size_t cols, int exponent = 0)
        : storage(rows * cols), view{storage.data(), rows, cols, exponent} {}
    explicit Matrix(const Dense &reference, int exponent = 0)
        : Matrix(static_cast<size_t>(reference.rows()), static_cast<size_t>(reference.cols()), exponent) {
        for (size_t i = 0; i < storage.size(); ++i) {
            if constexpr (T::bits == 0) storage[i] = static_cast<typename T::Raw>(reference.data()[i]);
            else {
                double raw = std::round(std::ldexp(reference.data()[i], T::bits - exponent));
                EXPECT_GE(raw, static_cast<double>(std::numeric_limits<typename T::Raw>::min()));
                EXPECT_LE(raw, static_cast<double>(std::numeric_limits<typename T::Raw>::max()));
                storage[i] = static_cast<typename T::Raw>(raw);
            }
        }
    }
    Matrix(const Matrix &) = delete;
    Matrix &operator=(const Matrix &) = delete;
    Dense decoded() const {
        Dense result(view.rows, view.cols);
        const auto *data = static_cast<const typename T::Raw *>(view.data);
        for (size_t i = 0; i < view.rows * view.cols; ++i) {
            if constexpr (T::bits == 0) result.data()[i] = data[i];
            else result.data()[i] = std::ldexp(static_cast<double>(data[i]), view.exponent - T::bits);
        }
        return result;
    }
};

template<class T> typename T::Scalar scalar(double value) {
    if constexpr (T::bits == 0) return static_cast<typename T::Scalar>(value);
    else {
        if (value == 0) return {0, 0};
        int exponent;
        double mantissa = std::frexp(value, &exponent);
        int64_t raw = std::llround(std::ldexp(mantissa, T::bits));
        if (raw > std::numeric_limits<typename T::Raw>::max()) {
            ++exponent;
            raw = std::llround(std::ldexp(mantissa, T::bits - 1));
        }
        return {static_cast<typename T::Raw>(raw), exponent};
    }
}

template<class T> double decoded(typename T::Scalar value) {
    if constexpr (T::bits == 0) return static_cast<double>(value);
    else return std::ldexp(static_cast<double>(value.mantissa), value.exponent - T::bits);
}

template<class T> void expect_close(const Dense &actual, const Dense &reference) {
    ASSERT_EQ(actual.rows(), reference.rows());
    ASSERT_EQ(actual.cols(), reference.cols());
    for (Eigen::Index row = 0; row < reference.rows(); ++row)
        for (Eigen::Index col = 0; col < reference.cols(); ++col)
            EXPECT_NEAR(actual(row, col), reference(row, col),
                        T::tolerance * std::max(1.0, reference.cwiseAbs().maxCoeff()))
                << "row=" << row << " col=" << col;
}

inline Dense samples(size_t rows, size_t cols, unsigned seed = 1) {
    Dense result(rows, cols);
    uint32_t state = seed;
    for (size_t i = 0; i < rows * cols; ++i) {
        state = state * UINT32_C(1664525) + UINT32_C(1013904223);
        result.data()[i] = (static_cast<int>(state % 15001) - 7500) / 10000.0;
    }
    return result;
}
}
#endif
