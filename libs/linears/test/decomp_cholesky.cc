#include "decomp_support.h"

namespace {
using namespace decomp_test;
template<class F> class LinearCholesky : public ::testing::Test {};
TYPED_TEST_SUITE(LinearCholesky, Formats);

TYPED_TEST(LinearCholesky, FixedSeedPositiveDefiniteInputsMatchEigen)
{
    using F = TypeParam;
    for (size_t n = 1; n <= 10; ++n) {
        Eigen::MatrixXd basis = random_matrix(n, n, 0x2718u + static_cast<uint32_t>(n));
        Eigen::MatrixXd values = basis * basis.transpose();
        values.diagonal().array() += static_cast<double>(n);
        // Eigen's vectorized product may round the two triangles differently;
        // this API explicitly requires exactly symmetric stored input.
        for (size_t i = 0; i < n; ++i)
            for (size_t j = i + 1; j < n; ++j) values(i, j) = values(j, i);
        Matrix<F> a(values), l(n, n);
        Scratch<F> scratch(LINEARS_CHOLESKY_WORKSPACE(n));
        ASSERT_EQ(F::cholesky(&a.matrix, &l.matrix, scratch.workspace), LINEARS_OK);
        Eigen::LLT<Eigen::MatrixXd> reference(a.decoded());
        ASSERT_EQ(reference.info(), Eigen::Success);
        const Eigen::MatrixXd expected = reference.matrixL();
        const Eigen::MatrixXd actual = l.decoded();
        const double rounding = quantization_bound(l);
        EXPECT_LE((actual - expected).norm(), rounding + 8.0 * n * F::working_epsilon * expected.norm());
        const double residual = 2.0 * actual.norm() * rounding + rounding * rounding +
                                8.0 * n * F::working_epsilon * a.decoded().norm();
        EXPECT_LE((actual * actual.transpose() - a.decoded()).norm(), residual);
        for (size_t i = 0; i < n; ++i) {
            EXPECT_GT(actual(i, i), 0.0);
            for (size_t j = i + 1; j < n; ++j) EXPECT_EQ(actual(i, j), 0.0);
        }
        scratch.check();
    }
}

TYPED_TEST(LinearCholesky, IndefiniteSemidefiniteAndNonsymmetricInputsFailAtomically)
{
    using F = TypeParam;
    for (int kind = 0; kind < 3; ++kind) {
        Eigen::Matrix2d values;
        if (kind == 0) values << 0, 1, 1, 0;
        else if (kind == 1) values << 1, -1, -1, 1;
        else values << 2, 1, 0, 2;
        Matrix<F> a(values), l(2, 2);
        l.sentinel(); const auto old = l.data;
        Scratch<F> scratch(LINEARS_CHOLESKY_WORKSPACE(2));
        EXPECT_EQ(F::cholesky(&a.matrix, &l.matrix, scratch.workspace),
            kind == 2 ? LINEARS_ERROR_DIMENSION : LINEARS_ERROR_NOT_POSITIVE_DEFINITE);
        EXPECT_EQ(l.data, old); EXPECT_EQ(l.matrix.exponent, 17);
        scratch.check();
    }
}

TYPED_TEST(LinearCholesky, EmptyScalarAndPowerOfTwoScales)
{
    using F = TypeParam;
    Matrix<F> empty(0, 0), result(0, 0);
    typename F::Workspace none{nullptr, 0};
    ASSERT_EQ(F::cholesky(&empty.matrix, &result.matrix, none), LINEARS_OK);
    for (int exponent : {-40, 0, 40}) {
        Matrix<F> scalar(4.0 * Eigen::MatrixXd::Identity(1, 1), exponent), root(1, 1);
        Scratch<F> scratch(1);
        ASSERT_EQ(F::cholesky(&scalar.matrix, &root.matrix, scratch.workspace), LINEARS_OK);
        EXPECT_EQ(root.decoded()(0, 0), std::scalbn(2.0, exponent / 2));
        scratch.check();
    }
}

TYPED_TEST(LinearCholesky, RepresentableSmallPivotIsNotRejectedAsSingular)
{
    using F = TypeParam;
    Eigen::Matrix3d values = Eigen::Matrix3d::Identity();
    values(2, 2) = 0x1p-12;
    Matrix<F> a(values), l(3, 3);
    Scratch<F> scratch(LINEARS_CHOLESKY_WORKSPACE(3));
    ASSERT_EQ(F::cholesky(&a.matrix, &l.matrix, scratch.workspace), LINEARS_OK);
    EXPECT_EQ(l.decoded()(2, 2), 0x1p-6);
    scratch.check();
}
} // namespace
