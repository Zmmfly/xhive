#ifndef LINEARS_TEST_DECOMP_SUPPORT_H
#define LINEARS_TEST_DECOMP_SUPPORT_H

#include "linears_decomp.h"
#include "linears_numeric.h"
#include <Eigen/Dense>
#include <gtest/gtest.h>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <random>
#include <type_traits>
#include <vector>

namespace decomp_test {

/** @brief Per-format test adapter; Eigen and allocation are test-only. */
#define LINEARS_DECOMP_TEST_FORMAT(Name, Suffix, RawType, WorkType, Bits, StorageEps, WorkEps) \
struct Name { \
    using Raw = RawType; using Work = WorkType; using Workspace = linears_##Suffix##_workspace_t; \
    static constexpr int bits = Bits; \
    static constexpr double storage_epsilon = StorageEps; \
    static constexpr double working_epsilon = WorkEps; \
    static linears_status_t lu(const linears_matrix_t *a, linears_matrix_t *l, linears_matrix_t *u, \
        size_t *p, size_t pc, int *parity, Workspace w) { return linears_lu_##Suffix(a,l,u,p,pc,parity,w); } \
    static linears_status_t solve(const linears_matrix_t *a, const linears_matrix_t *b, \
        linears_matrix_t *x, Workspace w) { return linears_solve_##Suffix(a,b,x,w); } \
    static linears_status_t inverse(const linears_matrix_t *a, linears_matrix_t *x, Workspace w) \
        { return linears_inverse_##Suffix(a,x,w); } \
    static linears_status_t determinant(const linears_matrix_t *a, linears_matrix_t *x, Workspace w) \
        { return linears_determinant_##Suffix(a,x,w); } \
    static linears_status_t cholesky(const linears_matrix_t *a, linears_matrix_t *l, Workspace w) \
        { return linears_cholesky_##Suffix(a,l,w); } \
    static linears_status_t qr(const linears_matrix_t *a, linears_matrix_t *q, linears_matrix_t *r, Workspace w) \
        { return linears_qr_##Suffix(a,q,r,w); } \
    static linears_status_t ldlt(const linears_matrix_t *a, linears_matrix_t *l, linears_matrix_t *d, \
        size_t *p, size_t pc, uint8_t *b, size_t bc, Workspace w) \
        { return linears_ldlt_##Suffix(a,l,d,p,pc,b,bc,w); } \
    static linears_status_t svd(const linears_matrix_t *a, linears_matrix_t *u, linears_matrix_t *s, \
        linears_matrix_t *v, size_t sweeps, Workspace w) { return linears_svd_##Suffix(a,u,s,v,sweeps,w); } \
};
LINEARS_DECOMP_TEST_FORMAT(F32, f32, float, float, 0, 0x1p-23, 0x1p-23)
LINEARS_DECOMP_TEST_FORMAT(F64, f64, double, double, 0, 0x1p-52, 0x1p-52)
LINEARS_DECOMP_TEST_FORMAT(Q15, q15, int16_t, linears_fixed_t, 15, 0x1p-14, 0x1p-30)
LINEARS_DECOMP_TEST_FORMAT(Q31, q31, int32_t, linears_fixed_t, 31, 0x1p-30, 0x1p-30)
#undef LINEARS_DECOMP_TEST_FORMAT

using Formats = ::testing::Types<F32, F64, Q15, Q31>;

/**
 * @brief Owned row-major test matrix with explicit input quantization.
 *
 * Construct from Eigen values before forming the Eigen reference: decoded()
 * returns the exact represented input, not the original unquantized samples.
 * The optional scale multiplies values by 2^scale without changing q mantissas.
 */
template<class Format> class Matrix {
public:
    using Raw = typename Format::Raw;
    std::vector<Raw> data;
    linears_matrix_t matrix;

    Matrix(size_t rows, size_t cols)
        : data(rows * cols), matrix{data.empty() ? nullptr : data.data(), rows, cols, 0} {}

    explicit Matrix(const Eigen::MatrixXd &values, int scale = 0)
        : Matrix(static_cast<size_t>(values.rows()), static_cast<size_t>(values.cols()))
    {
        if constexpr (Format::bits == 0) {
            for (Eigen::Index i = 0; i < values.rows(); ++i)
                for (Eigen::Index j = 0; j < values.cols(); ++j)
                    data[static_cast<size_t>(i * values.cols() + j)] =
                        static_cast<Raw>(std::scalbn(values(i, j), scale));
        } else {
            const double maximum = values.size() == 0 ? 0.0 : values.cwiseAbs().maxCoeff();
            int exponent = 0;
            if (maximum != 0.0) {
                (void)std::frexp(maximum, &exponent);
                const double preview = std::round(std::scalbn(maximum, Format::bits - exponent));
                if (preview > static_cast<double>(std::numeric_limits<Raw>::max())) ++exponent;
            }
            for (Eigen::Index i = 0; i < values.rows(); ++i)
                for (Eigen::Index j = 0; j < values.cols(); ++j)
                    data[static_cast<size_t>(i * values.cols() + j)] = static_cast<Raw>(
                        std::round(std::scalbn(values(i, j), Format::bits - exponent)));
            matrix.exponent = maximum == 0.0 ? 0 : exponent + scale;
        }
    }

    Matrix(const Matrix &) = delete;
    Matrix &operator=(const Matrix &) = delete;

    Eigen::MatrixXd decoded() const
    {
        Eigen::MatrixXd result(static_cast<Eigen::Index>(matrix.rows),
                               static_cast<Eigen::Index>(matrix.cols));
        for (size_t i = 0; i < matrix.rows; ++i)
            for (size_t j = 0; j < matrix.cols; ++j) {
                double value = static_cast<double>(data[i * matrix.cols + j]);
                if constexpr (Format::bits != 0)
                    value = std::scalbn(value, matrix.exponent - Format::bits);
                result(static_cast<Eigen::Index>(i), static_cast<Eigen::Index>(j)) = value;
            }
        return result;
    }

    void sentinel()
    {
        std::fill(data.begin(), data.end(), static_cast<Raw>(37));
        matrix.exponent = 17;
    }
};

/** @brief Scratch storage with guards immediately outside the declared capacity. */
template<class Format> class Scratch {
public:
    using Work = typename Format::Work;
    std::vector<Work> data;
    typename Format::Workspace workspace;
    explicit Scratch(size_t count) : data(count + 2), workspace{data.data() + 1, count}
    {
        if constexpr (Format::bits == 0) {
            data.front() = static_cast<Work>(123.25);
            data.back() = static_cast<Work>(-456.5);
        } else {
            data.front() = {INT32_C(1431655765), 9};
            data.back() = {-INT32_C(1431655765), 11};
        }
    }
    void check() const
    {
        if constexpr (Format::bits == 0) {
            EXPECT_EQ(data.front(), static_cast<Work>(123.25));
            EXPECT_EQ(data.back(), static_cast<Work>(-456.5));
        } else {
            EXPECT_EQ(data.front().mantissa, INT32_C(1431655765));
            EXPECT_EQ(data.front().exponent, 9);
            EXPECT_EQ(data.back().mantissa, -INT32_C(1431655765));
            EXPECT_EQ(data.back().exponent, 11);
        }
    }
};

/** @brief Frobenius bound for final fixed-point block rounding; native formats return zero. */
template<class Format> double quantization_bound(const Matrix<Format> &matrix)
{
    if constexpr (Format::bits == 0) return 0.0;
    else return std::sqrt(static_cast<double>(matrix.data.size())) *
                std::scalbn(0.5, matrix.matrix.exponent - Format::bits);
}

/** @brief First-order operation-count bound plus the actual factors' block-rounding bounds. */
template<class Format> double product_bound(const Matrix<Format> &left,
                                            const Matrix<Format> &right,
                                            double input_norm, size_t inner_dimension)
{
    const double el = quantization_bound(left), er = quantization_bound(right);
    return el * right.decoded().norm() + left.decoded().norm() * er + el * er +
           8.0 * static_cast<double>(std::max<size_t>(inner_dimension, 1)) *
           Format::working_epsilon * input_norm;
}

/** @brief Deterministic finite input matrix with both signs. */
inline Eigen::MatrixXd random_matrix(size_t rows, size_t cols, uint32_t seed)
{
    std::mt19937 engine(seed);
    std::uniform_real_distribution<double> distribution(-1.0, 1.0);
    Eigen::MatrixXd result(static_cast<Eigen::Index>(rows), static_cast<Eigen::Index>(cols));
    for (size_t i = 0; i < rows; ++i)
        for (size_t j = 0; j < cols; ++j)
            result(static_cast<Eigen::Index>(i), static_cast<Eigen::Index>(j)) = distribution(engine);
    return result;
}

/** @brief Frobenius comparison relative to the represented reference matrix. */
inline void expect_relative(const Eigen::MatrixXd &actual, const Eigen::MatrixXd &expected,
                            double relative_tolerance, double absolute_tolerance = 0.0)
{
    ASSERT_EQ(actual.rows(), expected.rows());
    ASSERT_EQ(actual.cols(), expected.cols());
    const double scale = expected.norm();
    EXPECT_LE((actual - expected).norm(), absolute_tolerance + relative_tolerance * scale)
        << "relative tolerance=" << relative_tolerance << ", reference norm=" << scale;
}

/**
 * @brief Dimension-scaled first-order roundoff budget for well-conditioned tests.
 *
 * Constants are chosen per algorithm at call sites. Input quantization is
 * already included in decoded() references; storage epsilon models each final
 * factor's block quantization and working epsilon models arithmetic roundoff.
 */
template<class Format> constexpr double tolerance(size_t dimension, double operations = 16.0)
{
    return operations * static_cast<double>(std::max<size_t>(dimension, 1)) *
           (Format::storage_epsilon + Format::working_epsilon);
}

} // namespace decomp_test
#endif
