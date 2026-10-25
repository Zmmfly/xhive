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
            Eigen::MatrixXd values;
            if constexpr (F::bits == 0) {
                values = random_matrix(n, n, 127u + static_cast<uint32_t>(n));
                values.diagonal().array() += static_cast<double>(n);
            } else {
                if (n == 1) values = -Eigen::MatrixXd::Identity(1, 1);
                else if (n == 2) {
                    values.resize(2, 2); values << 0.75, 0.75, 0.75, -0.75;
                } else {
                    // A reflection has a bounded-entry inverse; perturb it while
                    // retaining a margin below the fixed-Q output endpoints.
                    values = (2.0 / static_cast<double>(n)) * Eigen::MatrixXd::Ones(n, n) -
                             Eigen::MatrixXd::Identity(n, n);
                    values += (0.005 / static_cast<double>(n)) *
                              random_matrix(n, n, 127u + static_cast<uint32_t>(n));
                }
            }
            if (n > 1) values.row(0).swap(values.row(n - 1));
            Eigen::MatrixXd rhs_values = random_matrix(n, rhs, 517u + static_cast<uint32_t>(rhs));
            if constexpr (F::bits != 0) rhs_values *= 0.125;
            Matrix<F> a(values), b(rhs_values);
            Matrix<F> x(n, rhs), inverse(n, n);
            Scratch<F> ws(LINEARS_SOLVE_WORKSPACE(n, rhs));
            Scratch<F> wi(LINEARS_INVERSE_WORKSPACE(n));
            ASSERT_EQ(F::solve(&a.matrix, &b.matrix, &x.matrix, ws.workspace), LINEARS_OK);
            ASSERT_EQ(F::inverse(&a.matrix, &inverse.matrix, wi.workspace), LINEARS_OK);
            const Eigen::MatrixXd aq = a.decoded(), bq = b.decoded();
            const Eigen::MatrixXd reference_inverse = aq.partialPivLu().inverse();
            const Eigen::MatrixXd reference = aq.partialPivLu().solve(bq);
            const double arithmetic = 8.0 * n * F::working_epsilon;
            const double absolute_rounding = F::bits == 0 ? 0.0 : std::sqrt(static_cast<double>(n * rhs));
            const double forward = arithmetic * reference_inverse.norm() *
                (aq.norm() * reference.norm() + bq.norm() + absolute_rounding) + quantization_bound(x);
            EXPECT_LE((x.decoded() - reference).norm(), forward);
            EXPECT_LE((aq * x.decoded() - bq).norm(),
                aq.norm() * quantization_bound(x) + arithmetic *
                (aq.norm() * reference.norm() + bq.norm() + absolute_rounding));
            EXPECT_LE((inverse.decoded() - reference_inverse).norm(),
                arithmetic * reference_inverse.norm() * (aq.norm() * reference_inverse.norm() +
                    (F::bits == 0 ? 0.0 : static_cast<double>(n))) + quantization_bound(inverse));
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
    Matrix<F> a(hilbert), b((F::bits == 0 ? 1.0 : 0x1p-9) * Eigen::MatrixXd::Ones(4, 1)), x(4, 1);
    Scratch<F> scratch(LINEARS_SOLVE_WORKSPACE(4, 1));
    ASSERT_EQ(F::solve(&a.matrix, &b.matrix, &x.matrix, scratch.workspace), LINEARS_OK);
    const auto aq = a.decoded();
    const Eigen::MatrixXd reference = aq.partialPivLu().solve(b.decoded());
    const double arithmetic = 32.0 * F::working_epsilon;
    const double residual_bound = arithmetic * (aq.norm() * reference.norm() + b.decoded().norm() +
                                  (F::bits == 0 ? 0.0 : 2.0)) + aq.norm() * quantization_bound(x);
    EXPECT_LE((aq * x.decoded() - b.decoded()).norm(), residual_bound);
    EXPECT_LE((x.decoded() - reference).norm(),
        aq.partialPivLu().inverse().norm() * residual_bound);
    scratch.check();
}

TYPED_TEST(LinearSolve, NativeScaleSpansAndFixedMinimumRawValues)
{
    using F = TypeParam;
    if constexpr (F::bits == 0) {
        Eigen::Matrix2d av; av << 4, -1, 2, 3;
        Eigen::Vector2d bv; bv << -2, 5;
        for (int scale : {-30, 30}) {
            Matrix<F> a(av, scale), b(bv, -scale), x(2, 1);
            Scratch<F> scratch(LINEARS_SOLVE_WORKSPACE(2, 1));
            ASSERT_EQ(F::solve(&a.matrix, &b.matrix, &x.matrix, scratch.workspace), LINEARS_OK);
            Eigen::MatrixXd reference = a.decoded().partialPivLu().solve(b.decoded());
            EXPECT_LE((x.decoded() - reference).norm(), 32.0 * F::working_epsilon * reference.norm());
            scratch.check();
        }
    } else {
        Matrix<F> a(1, 1), b(1, 2), x(1, 2);
        a.data[0] = 1; b.data[0] = 1; b.data[1] = -1;
        Scratch<F> scratch(LINEARS_SOLVE_WORKSPACE(1, 2));
        ASSERT_EQ(F::solve(&a.matrix, &b.matrix, &x.matrix, scratch.workspace), LINEARS_OK);
        EXPECT_EQ(x.data[0], std::numeric_limits<typename F::Raw>::max());
        EXPECT_EQ(x.data[1], std::numeric_limits<typename F::Raw>::min());
        b.data[0] = 2;
        const auto old = x.data;
        EXPECT_EQ(F::solve(&a.matrix, &b.matrix, &x.matrix, scratch.workspace), LINEARS_ERROR_RANGE);
        EXPECT_EQ(x.data, old);
        scratch.check();
    }
}

TYPED_TEST(LinearSolve, SingularZeroRhsAndEmptyCasesHaveDefinedStatus)
{
    using F = TypeParam;
    Eigen::Matrix2d singular; singular << 1, -2, -2, 4;
    if constexpr (F::bits != 0) singular /= 8.0;
    Matrix<F> a(singular), b(Eigen::MatrixXd::Ones(2, 1)), x(2, 1), inverse(2, 2);
    x.sentinel(); inverse.sentinel();
    const auto old_x = x.data, old_inverse = inverse.data;
    Scratch<F> ws(LINEARS_SOLVE_WORKSPACE(2, 1)), wi(LINEARS_INVERSE_WORKSPACE(2));
    EXPECT_EQ(F::solve(&a.matrix, &b.matrix, &x.matrix, ws.workspace), LINEARS_ERROR_SINGULAR);
    EXPECT_EQ(F::inverse(&a.matrix, &inverse.matrix, wi.workspace), LINEARS_ERROR_SINGULAR);
    EXPECT_EQ(x.data, old_x); EXPECT_EQ(inverse.data, old_inverse);
    Matrix<F> b0(2, 0), x0(2, 0);
    Scratch<F> wz(LINEARS_SOLVE_WORKSPACE(2, 0));
    EXPECT_EQ(F::solve(&a.matrix, &b0.matrix, &x0.matrix, wz.workspace), LINEARS_ERROR_SINGULAR);
    Matrix<F> empty(0, 0), empty_b(0, 3), empty_x(0, 3), empty_inverse(0, 0);
    typename F::Workspace none{nullptr, 0};
    EXPECT_EQ(F::solve(&empty.matrix, &empty_b.matrix, &empty_x.matrix, none), LINEARS_OK);
    EXPECT_EQ(F::inverse(&empty.matrix, &empty_inverse.matrix, none), LINEARS_OK);
    ws.check(); wi.check(); wz.check();
}

using FixedSolveFormats = ::testing::Types<Q15, Q31>;
template<class F> class FixedInverseRange : public ::testing::Test {};
TYPED_TEST_SUITE(FixedInverseRange, FixedSolveFormats);

TYPED_TEST(FixedInverseRange, UnrepresentableInverseAndQuantizedIdentityAreNotClamped)
{
    using F = TypeParam;
    Matrix<F> a(1, 1), inverse(1, 1);
    Scratch<F> scratch(LINEARS_INVERSE_WORKSPACE(1));
    for (auto raw : {static_cast<typename F::Raw>(int64_t(1) << (F::bits - 1)),
                    std::numeric_limits<typename F::Raw>::max()}) {
        a.data[0] = raw; inverse.sentinel(); const auto old = inverse.data;
        EXPECT_EQ(F::inverse(&a.matrix, &inverse.matrix, scratch.workspace), LINEARS_ERROR_RANGE);
        EXPECT_EQ(inverse.data, old);
    }
    a.data[0] = std::numeric_limits<typename F::Raw>::min();
    ASSERT_EQ(F::inverse(&a.matrix, &inverse.matrix, scratch.workspace), LINEARS_OK);
    EXPECT_EQ(inverse.data[0], std::numeric_limits<typename F::Raw>::min());
    scratch.check();
}
} // namespace
