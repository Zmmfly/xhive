#include "decomp_support.h"
#include <numeric>

namespace {
using namespace decomp_test;
template<class F> class LinearLu : public ::testing::Test {};
TYPED_TEST_SUITE(LinearLu, Formats);

TYPED_TEST(LinearLu, FixedSeedPivotedFactorizationMatchesEigen)
{
    using F = TypeParam;
    for (size_t n = 1; n <= 9; ++n) {
        for (uint32_t sample = 0; sample < 8; ++sample) {
            Matrix<F> a(random_matrix(n, n, 0x51a0u + sample + static_cast<uint32_t>(n) * 17u));
            Matrix<F> l(n, n), u(n, n);
            Scratch<F> scratch(LINEARS_LU_WORKSPACE(n));
            std::vector<size_t> permutation(n);
            int parity = 0;
            ASSERT_EQ(F::lu(&a.matrix, &l.matrix, &u.matrix, permutation.data(), n,
                            &parity, scratch.workspace), LINEARS_OK);
            const Eigen::MatrixXd input = a.decoded();
            Eigen::MatrixXd permuted(n, n);
            std::vector<size_t> sorted = permutation;
            std::sort(sorted.begin(), sorted.end());
            size_t inversions = 0;
            for (size_t i = 0; i < n; ++i) {
                EXPECT_EQ(sorted[i], i);
                permuted.row(i) = input.row(permutation[i]);
                for (size_t j = i + 1; j < n; ++j) inversions += permutation[i] > permutation[j];
            }
            EXPECT_EQ(parity, inversions % 2 == 0 ? 1 : -1);
            const auto lower = l.decoded(), upper = u.decoded();
            EXPECT_LE((lower * upper - permuted).norm(), product_bound(l, u, input.norm(), n));
            for (size_t i = 0; i < n; ++i) {
                EXPECT_EQ(lower(i, i), 1.0);
                for (size_t j = i + 1; j < n; ++j) {
                    EXPECT_EQ(lower(i, j), 0.0);
                    EXPECT_EQ(upper(j, i), 0.0);
                }
            }
            Eigen::PartialPivLU<Eigen::MatrixXd> reference(input);
            EXPECT_LE((reference.permutationP() * input -
                (reference.matrixLU().template triangularView<Eigen::UnitLower>() *
                 reference.matrixLU().template triangularView<Eigen::Upper>().toDenseMatrix())).norm(),
                64.0 * std::numeric_limits<double>::epsilon() * input.norm());
            scratch.check();
        }
    }
}

TYPED_TEST(LinearLu, ZeroDiagonalRequiresRowPivotAndNegativeParity)
{
    using F = TypeParam;
    Eigen::Matrix2d values; values << 0, 2, -3, 4;
    Matrix<F> a(values), l(2, 2), u(2, 2);
    Scratch<F> scratch(LINEARS_LU_WORKSPACE(2));
    size_t permutation[2]; int parity = 0;
    ASSERT_EQ(F::lu(&a.matrix, &l.matrix, &u.matrix, permutation, 2, &parity, scratch.workspace), LINEARS_OK);
    EXPECT_EQ(parity, -1);
    EXPECT_EQ(permutation[0], 1u);
    EXPECT_EQ(permutation[1], 0u);
    scratch.check();
}

TYPED_TEST(LinearLu, SingularAndInsufficientMetadataLeaveMatricesUnchanged)
{
    using F = TypeParam;
    Eigen::Matrix2d values; values << 1, 2, 2, 4;
    Matrix<F> a(values), l(2, 2), u(2, 2);
    l.sentinel(); u.sentinel();
    const auto old_l = l.data, old_u = u.data;
    size_t permutation[2] = {99, 99}; int parity = 73;
    Scratch<F> scratch(LINEARS_LU_WORKSPACE(2));
    EXPECT_EQ(F::lu(&a.matrix, &l.matrix, &u.matrix, permutation, 1, &parity, scratch.workspace), LINEARS_ERROR_WORKSPACE);
    EXPECT_EQ(permutation[0], 99u);
    EXPECT_EQ(F::lu(&a.matrix, &l.matrix, &u.matrix, permutation, 2, &parity, scratch.workspace), LINEARS_ERROR_SINGULAR);
    EXPECT_EQ(l.data, old_l); EXPECT_EQ(u.data, old_u);
    EXPECT_EQ(l.matrix.exponent, 17); EXPECT_EQ(u.matrix.exponent, 17);
    EXPECT_EQ(parity, 73);
    scratch.check();
}

TYPED_TEST(LinearLu, EmptyMatrixAndIndependentWorkspaces)
{
    using F = TypeParam;
    Matrix<F> empty(0, 0), l0(0, 0), u0(0, 0);
    typename F::Workspace none{nullptr, 0}; int parity = 0;
    ASSERT_EQ(F::lu(&empty.matrix, &l0.matrix, &u0.matrix, nullptr, 0, &parity, none), LINEARS_OK);
    EXPECT_EQ(parity, 1);
    Matrix<F> a(Eigen::MatrixXd::Identity(3, 3)), l(3, 3), u(3, 3);
    Matrix<F> b(2.0 * Eigen::MatrixXd::Identity(3, 3)), l2(3, 3), u2(3, 3);
    Scratch<F> w1(LINEARS_LU_WORKSPACE(3)), w2(LINEARS_LU_WORKSPACE(3));
    size_t p[3], p2[3]; int p_sign, p2_sign;
    ASSERT_EQ(F::lu(&a.matrix, &l.matrix, &u.matrix, p, 3, &p_sign, w1.workspace), LINEARS_OK);
    const auto first_l = l.data, first_u = u.data;
    ASSERT_EQ(F::lu(&b.matrix, &l2.matrix, &u2.matrix, p2, 3, &p2_sign, w2.workspace), LINEARS_OK);
    EXPECT_EQ(l.data, first_l); EXPECT_EQ(u.data, first_u);
    w1.check(); w2.check();
}
} // namespace
