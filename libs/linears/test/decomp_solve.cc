#include "decomp_support.h"

namespace {
using namespace decomp_test;
template<class F> class LinearSolve : public ::testing::Test {};
TYPED_TEST_SUITE(LinearSolve, Formats);

TYPED_TEST(LinearSolve, MultipleRightHandSidesAndInverseMatchEigen)
{
    using F = TypeParam;
    for (size_t n = 1; n <= 8; ++n) {
        for (size_t rhs : {size_t(1), size_t(3)}) {
            Eigen::MatrixXd values = random_matrix(n, n, 127u + static_cast<uint32_t>(n));
            values.diagonal().array() += static_cast<double>(n);
            if (n > 1) values.row(0).swap(values.row(n - 1));
            Matrix<F> a(values), b(random_matrix(n, rhs, 517u + static_cast<uint32_t>(rhs)));
            Matrix<F> x(n, rhs), inverse(n, n);
            Scratch<F> ws(LINEARS_SOLVE_WORKSPACE(n, rhs));
            Scratch<F> wi(LINEARS_INVERSE_WORKSPACE(n));
            ASSERT_EQ(F::solve(&a.matrix, &b.matrix, &x.matrix, ws.workspace), LINEARS_OK);
            ASSERT_EQ(F::inverse(&a.matrix, &inverse.matrix, wi.workspace), LINEARS_OK);
            const Eigen::MatrixXd aq = a.decoded(), bq = b.decoded();
            const Eigen::MatrixXd reference_inverse = aq.partialPivLu().inverse();
            const Eigen::MatrixXd reference = aq.partialPivLu().solve(bq);
            const double arithmetic = 8.0 * n * F::working_epsilon;
            const double forward = arithmetic * reference_inverse.norm() *
                (aq.norm() * reference.norm() + bq.norm()) + quantization_bound(x);
            EXPECT_LE((x.decoded() - reference).norm(), forward);
            EXPECT_LE((aq * x.decoded() - bq).norm(),
                aq.norm() * quantization_bound(x) + arithmetic * (aq.norm() * reference.norm() + bq.norm()));
            EXPECT_LE((inverse.decoded() - reference_inverse).norm(),
                arithmetic * aq.norm() * reference_inverse.squaredNorm() + quantization_bound(inverse));
            ws.check(); wi.check();
        }
    }
}

TYPED_TEST(LinearSolve, IllConditionedRepresentedInputsUseConditionAwareBounds)
{
    using F = TypeParam;
    Eigen::MatrixXd hilbert(4, 4);
    for (Eigen::Index i = 0; i < 4; ++i)
        for (Eigen::Index j = 0; j < 4; ++j) hilbert(i, j) = 1.0 / static_cast<double>(i + j + 1);
    Matrix<F> a(hilbert), b(Eigen::MatrixXd::Ones(4, 1)), x(4, 1);
    Scratch<F> scratch(LINEARS_SOLVE_WORKSPACE(4, 1));
    ASSERT_EQ(F::solve(&a.matrix, &b.matrix, &x.matrix, scratch.workspace), LINEARS_OK);
    const auto aq = a.decoded();
    const Eigen::MatrixXd reference = aq.partialPivLu().solve(b.decoded());
    const double arithmetic = 32.0 * F::working_epsilon;
    const double residual_bound = arithmetic * (aq.norm() * reference.norm() + b.decoded().norm()) +
                                  aq.norm() * quantization_bound(x);
    EXPECT_LE((aq * x.decoded() - b.decoded()).norm(), residual_bound);
    EXPECT_LE((x.decoded() - reference).norm(),
        aq.partialPivLu().inverse().norm() * residual_bound);
    scratch.check();
}

TYPED_TEST(LinearSolve, IndependentInputExponentsArePreservedInTheSolution)
{
    using F = TypeParam;
    Eigen::Matrix2d av; av << 4, -1, 2, 3;
    Eigen::Vector2d bv; bv << -2, 5;
    for (int scale : {-30, 30}) {
        Matrix<F> a(av, scale), b(bv, -scale), x(2, 1);
        Scratch<F> scratch(LINEARS_SOLVE_WORKSPACE(2, 1));
        ASSERT_EQ(F::solve(&a.matrix, &b.matrix, &x.matrix, scratch.workspace), LINEARS_OK);
        Eigen::MatrixXd reference = a.decoded().partialPivLu().solve(b.decoded());
        const double error = 32.0 * F::working_epsilon * reference.norm() + quantization_bound(x);
        EXPECT_LE((x.decoded() - reference).norm(), error);
        scratch.check();
    }
}

TYPED_TEST(LinearSolve, SingularZeroRhsAndEmptyCasesHaveDefinedStatus)
{
    using F = TypeParam;
    Eigen::Matrix2d singular; singular << 1, -2, -2, 4;
    Matrix<F> a(singular), b(Eigen::MatrixXd::Ones(2, 1)), x(2, 1), inverse(2, 2);
    x.sentinel(); inverse.sentinel();
    const auto old_x = x.data, old_inverse = inverse.data;
    Scratch<F> ws(LINEARS_SOLVE_WORKSPACE(2, 1)), wi(LINEARS_INVERSE_WORKSPACE(2));
    EXPECT_EQ(F::solve(&a.matrix, &b.matrix, &x.matrix, ws.workspace), LINEARS_ERROR_SINGULAR);
    EXPECT_EQ(F::inverse(&a.matrix, &inverse.matrix, wi.workspace), LINEARS_ERROR_SINGULAR);
    EXPECT_EQ(x.data, old_x); EXPECT_EQ(inverse.data, old_inverse);
    EXPECT_EQ(x.matrix.exponent, 17); EXPECT_EQ(inverse.matrix.exponent, 17);
    Matrix<F> b0(2, 0), x0(2, 0);
    Scratch<F> wz(LINEARS_SOLVE_WORKSPACE(2, 0));
    EXPECT_EQ(F::solve(&a.matrix, &b0.matrix, &x0.matrix, wz.workspace), LINEARS_ERROR_SINGULAR);
    Matrix<F> empty(0, 0), empty_b(0, 3), empty_x(0, 3), empty_inverse(0, 0);
    typename F::Workspace none{nullptr, 0};
    EXPECT_EQ(F::solve(&empty.matrix, &empty_b.matrix, &empty_x.matrix, none), LINEARS_OK);
    EXPECT_EQ(F::inverse(&empty.matrix, &empty_inverse.matrix, none), LINEARS_OK);
    ws.check(); wi.check(); wz.check();
}
} // namespace
