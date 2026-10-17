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
            values.diagonal().array() += static_cast<double>(n);
            if ((seed & 1u) != 0) values.row(0) *= -1.0;
            Matrix<F> a(values), determinant(1, 1);
            Scratch<F> scratch(LINEARS_DETERMINANT_WORKSPACE(n));
            ASSERT_EQ(F::determinant(&a.matrix, &determinant.matrix, scratch.workspace), LINEARS_OK);
            const Eigen::MatrixXd input = a.decoded();
            const double reference = input.determinant();
            // These diagonally dominant samples have bounded pivot growth.
            const double bound = 16.0 * n * F::working_epsilon * std::abs(reference) +
                                 quantization_bound(determinant);
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
    EXPECT_EQ(result.decoded()(0, 0), 1.0);
    Eigen::Matrix2d values; values << -1, 2, -2, 4;
    Matrix<F> singular(values);
    Scratch<F> scratch(LINEARS_DETERMINANT_WORKSPACE(2));
    ASSERT_EQ(F::determinant(&singular.matrix, &result.matrix, scratch.workspace), LINEARS_OK);
    EXPECT_EQ(result.decoded()(0, 0), 0.0);
    scratch.check();
}

TYPED_TEST(LinearDeterminant, ScaledInputsKeepTheDeterminantExponent)
{
    using F = TypeParam;
    for (int scale : {-20, 20}) {
        Eigen::Matrix3d values = Eigen::Matrix3d::Identity();
        values(0, 0) = -2.0;
        Matrix<F> a(values, scale), determinant(1, 1);
        Scratch<F> scratch(LINEARS_DETERMINANT_WORKSPACE(3));
        ASSERT_EQ(F::determinant(&a.matrix, &determinant.matrix, scratch.workspace), LINEARS_OK);
        EXPECT_EQ(determinant.decoded()(0, 0), -std::scalbn(2.0, 3 * scale));
        scratch.check();
    }
}

TYPED_TEST(LinearDeterminant, BalancedNativeProductsAvoidIntermediateOverflow)
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
        Matrix<F> a(Eigen::MatrixXd::Identity(4, 4), 3000), determinant(1, 1);
        Scratch<F> scratch(LINEARS_DETERMINANT_WORKSPACE(4));
        ASSERT_EQ(F::determinant(&a.matrix, &determinant.matrix, scratch.workspace), LINEARS_OK);
        EXPECT_EQ(determinant.matrix.exponent, 12001);
        EXPECT_EQ(determinant.data[0], static_cast<typename F::Raw>(int64_t(1) << (F::bits - 1)));
        a.matrix.exponent += 2000;
        determinant.sentinel();
        const auto old = determinant.data;
        EXPECT_EQ(F::determinant(&a.matrix, &determinant.matrix, scratch.workspace), LINEARS_ERROR_RANGE);
        EXPECT_EQ(determinant.data, old);
        EXPECT_EQ(determinant.matrix.exponent, 17);
        scratch.check();
    }
}
} // namespace
