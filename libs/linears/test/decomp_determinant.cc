#include "decomp_support.h"

namespace {
using namespace decomp_test;
template<class F> class LinearDeterminant : public ::testing::Test {};
TYPED_TEST_SUITE(LinearDeterminant, Formats);

TYPED_TEST(LinearDeterminant, MatchesEigenForBothSignsAndSeveralDimensions)
{
    using F = TypeParam;
    for (size_t n = 1; n <= 7; ++n) {
        for (uint32_t seed = 0; seed < 8; ++seed) {
            Eigen::MatrixXd values = random_matrix(n, n, seed + 0x7710u);
            if constexpr (F::bits == 0) values.diagonal().array() += static_cast<double>(n);
            else {
                values /= 4.0 * static_cast<double>(n);
                values.diagonal().array() += 0.5;
            }
            if ((seed & 1u) != 0) values.row(0) *= -1.0;
            Matrix<F> a(values), determinant(1, 1);
            Scratch<F> scratch(LINEARS_DETERMINANT_WORKSPACE(n));
            ASSERT_EQ(F::determinant(&a.matrix, &determinant.matrix, scratch.workspace), LINEARS_OK);
            const Eigen::MatrixXd input = a.decoded();
            const double reference = input.determinant();
            // These diagonally dominant samples have bounded pivot growth.
            const double bound = 16.0 * n * F::working_epsilon *
                (F::bits == 0 ? std::abs(reference) : 1.0) + quantization_bound(determinant);
            EXPECT_NEAR(determinant.decoded()(0, 0), reference, bound);
            scratch.check();
        }
    }
}

TYPED_TEST(LinearDeterminant, EmptyIsOneAndSingularIsSuccessfulZero)
{
    using F = TypeParam;
    Matrix<F> empty(0, 0), result(1, 1);
    typename F::Workspace none{nullptr, 0};
    ASSERT_EQ(F::determinant(&empty.matrix, &result.matrix, none), LINEARS_OK);
    EXPECT_EQ(result.decoded()(0, 0), F::bits == 0 ? 1.0 : 1.0 - std::scalbn(1.0, -F::bits));
    Eigen::Matrix2d values; values << -1, 2, -2, 4;
    if constexpr (F::bits != 0) values /= 8.0;
    Matrix<F> singular(values);
    Scratch<F> scratch(LINEARS_DETERMINANT_WORKSPACE(2));
    ASSERT_EQ(F::determinant(&singular.matrix, &result.matrix, scratch.workspace), LINEARS_OK);
    EXPECT_EQ(result.decoded()(0, 0), 0.0);
    scratch.check();
}

TYPED_TEST(LinearDeterminant, NativeScaleSpansAndFixedDyadicProducts)
{
    using F = TypeParam;
    if constexpr (F::bits == 0) {
        for (int scale : {-20, 20}) {
            Eigen::Matrix3d values = Eigen::Matrix3d::Identity();
            values(0, 0) = -2.0;
            Matrix<F> a(values, scale), determinant(1, 1);
            Scratch<F> scratch(LINEARS_DETERMINANT_WORKSPACE(3));
            ASSERT_EQ(F::determinant(&a.matrix, &determinant.matrix, scratch.workspace), LINEARS_OK);
            EXPECT_EQ(determinant.decoded()(0, 0), -std::scalbn(2.0, 3 * scale));
            scratch.check();
        }
    } else {
        for (int scale : {0, -2, -6}) {
            Eigen::Matrix3d values = Eigen::Matrix3d::Zero();
            values.diagonal() << -0.5, 0.25, 0.125;
            Matrix<F> a(values, scale), determinant(1, 1);
            Scratch<F> scratch(LINEARS_DETERMINANT_WORKSPACE(3));
            ASSERT_EQ(F::determinant(&a.matrix, &determinant.matrix, scratch.workspace), LINEARS_OK);
            const double expected_raw = std::round(std::scalbn(a.decoded().determinant(), F::bits));
            EXPECT_EQ(determinant.data[0], static_cast<typename F::Raw>(expected_raw));
            scratch.check();
        }
    }
}

TYPED_TEST(LinearDeterminant, NativeBalancedProductsAndFixedProductRange)
{
    using F = TypeParam;
    if constexpr (F::bits == 0) {
        Eigen::Matrix4d values = Eigen::Matrix4d::Zero();
        values.diagonal() << std::scalbn(1.0, 100), std::scalbn(1.0, 100),
                             std::scalbn(1.0, -100), std::scalbn(1.0, -100);
        Matrix<F> a(values), determinant(1, 1);
        Scratch<F> scratch(LINEARS_DETERMINANT_WORKSPACE(4));
        ASSERT_EQ(F::determinant(&a.matrix, &determinant.matrix, scratch.workspace), LINEARS_OK);
        EXPECT_EQ(determinant.decoded()(0, 0), 1.0);
        scratch.check();
    } else {
        Matrix<F> tiny(4, 4), determinant(1, 1);
        for (size_t i = 0; i < 4; ++i) tiny.data[i * 4 + i] = 1;
        Scratch<F> scratch(LINEARS_DETERMINANT_WORKSPACE(4));
        ASSERT_EQ(F::determinant(&tiny.matrix, &determinant.matrix, scratch.workspace), LINEARS_OK);
        EXPECT_EQ(determinant.data[0], 0); // Product underflow is distinct from a zero LU pivot.
        Eigen::Matrix2d values; values << 0.75, 0.75, 0.75, -0.75;
        Matrix<F> large(values);
        determinant.sentinel(); const auto old = determinant.data;
        EXPECT_EQ(F::determinant(&large.matrix, &determinant.matrix, scratch.workspace), LINEARS_ERROR_RANGE);
        EXPECT_EQ(determinant.data, old);
        scratch.check();
    }
}

TEST(FixedDeterminant, BalancesFixedProductsWithoutExponentState)
{
    Eigen::MatrixXd hadamard(1, 1); hadamard(0, 0) = 1.0;
    while (hadamard.rows() < 8) {
        const Eigen::MatrixXd old = hadamard;
        const Eigen::Index n = old.rows();
        hadamard.resize(2 * n, 2 * n);
        hadamard.topLeftCorner(n, n) = old;
        hadamard.topRightCorner(n, n) = old;
        hadamard.bottomLeftCorner(n, n) = old;
        hadamard.bottomRightCorner(n, n) = -old;
    }
    Eigen::MatrixXd values = Eigen::MatrixXd::Zero(10, 10);
    values(0, 0) = 0x1p-17; values(1, 1) = 0x1p-17;
    values.bottomRightCorner(8, 8) = 0.75 * hadamard;
    Matrix<Q31> a(values), determinant(1, 1);
    Scratch<Q31> scratch(LINEARS_DETERMINANT_WORKSPACE(10));
    ASSERT_EQ(Q31::determinant(&a.matrix, &determinant.matrix, scratch.workspace), LINEARS_OK);
    const double reference = a.decoded().determinant();
    EXPECT_NE(determinant.data[0], 0);
    EXPECT_NEAR(determinant.decoded()(0, 0), reference, 8.0 * Q31::working_epsilon);
    scratch.check();
}
} // namespace
