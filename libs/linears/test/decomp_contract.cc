#include "decomp_support.h"
#include <cstring>
#include <thread>

namespace {
using namespace decomp_test;
template<class F> class DecompositionContract : public ::testing::Test {};
TYPED_TEST_SUITE(DecompositionContract, Formats);

TYPED_TEST(DecompositionContract, AlignmentArgumentsAndAliasingAreCheckedBeforeAccess)
{
    using F = TypeParam;
    Matrix<F> a(Eigen::MatrixXd::Identity(2, 2)), b(Eigen::MatrixXd::Ones(2, 1)), x(2, 1);
    Scratch<F> scratch(LINEARS_SOLVE_WORKSPACE(2, 1));
    x.sentinel(); const auto old = x.data;
    EXPECT_EQ(F::solve(nullptr, &b.matrix, &x.matrix, scratch.workspace), LINEARS_ERROR_ARGUMENT);
    alignas(linears_matrix_t) unsigned char descriptor[sizeof(linears_matrix_t) + alignof(linears_matrix_t)]{};
    auto *misaligned = reinterpret_cast<const linears_matrix_t *>(descriptor + 1);
    EXPECT_EQ(F::solve(misaligned, &b.matrix, &x.matrix, scratch.workspace), LINEARS_ERROR_ARGUMENT);
    linears_matrix_t no_data = a.matrix; no_data.data = nullptr;
    EXPECT_EQ(F::solve(&no_data, &b.matrix, &x.matrix, scratch.workspace), LINEARS_ERROR_ARGUMENT);
    std::vector<unsigned char> raw(sizeof(typename F::Raw) * 4 + alignof(typename F::Raw));
    linears_matrix_t bad_data = a.matrix; bad_data.data = raw.data() + 1;
    EXPECT_EQ(F::solve(&bad_data, &b.matrix, &x.matrix, scratch.workspace), LINEARS_ERROR_ARGUMENT);
    typename F::Workspace no_workspace{nullptr, scratch.workspace.capacity};
    EXPECT_EQ(F::solve(&a.matrix, &b.matrix, &x.matrix, no_workspace), LINEARS_ERROR_WORKSPACE);
    std::vector<unsigned char> raw_workspace(sizeof(typename F::Work) * scratch.workspace.capacity + alignof(typename F::Work));
    typename F::Workspace bad_workspace{reinterpret_cast<typename F::Work *>(raw_workspace.data() + 1), scratch.workspace.capacity};
    EXPECT_EQ(F::solve(&a.matrix, &b.matrix, &x.matrix, bad_workspace), LINEARS_ERROR_WORKSPACE);
    linears_matrix_t alias = x.matrix; alias.data = b.matrix.data;
    EXPECT_EQ(F::solve(&a.matrix, &b.matrix, &alias, scratch.workspace), LINEARS_ERROR_ALIAS);
    EXPECT_EQ(x.data, old);
    scratch.check();
}

TYPED_TEST(DecompositionContract, RuntimeSizeOverflowAndWrongShapesDoNotReadData)
{
    using F = TypeParam;
    const size_t huge = std::numeric_limits<size_t>::max();
    linears_matrix_t a{nullptr, huge, huge}, x{nullptr, huge, huge};
    typename F::Workspace none{nullptr, 0};
    EXPECT_EQ(F::inverse(&a, &x, none), LINEARS_ERROR_SIZE);
    Matrix<F> valid(Eigen::MatrixXd::Identity(2, 2)), wrong(1, 2);
    Scratch<F> scratch(LINEARS_INVERSE_WORKSPACE(2));
    EXPECT_EQ(F::inverse(&valid.matrix, &wrong.matrix, scratch.workspace), LINEARS_ERROR_DIMENSION);
    // No dimension-sized array or arbitrary dimension ceiling: the empty solve
    // must not multiply or iterate over the huge rhs count.
    linears_matrix_t empty{nullptr, 0, 0}, empty_rhs{nullptr, 0, huge}, empty_x{nullptr, 0, huge};
    EXPECT_EQ(F::solve(&empty, &empty_rhs, &empty_x, none), LINEARS_OK);
    EXPECT_EQ(empty_x.rows, 0u); EXPECT_EQ(empty_x.cols, huge);
    scratch.check();
}

TYPED_TEST(DecompositionContract, NativeNonfiniteAndFixedOutputRangeFailuresAreAtomic)
{
    using F = TypeParam;
    if constexpr (F::bits == 0) {
        for (auto invalid : {std::numeric_limits<typename F::Raw>::quiet_NaN(),
                             std::numeric_limits<typename F::Raw>::infinity(),
                             -std::numeric_limits<typename F::Raw>::infinity()}) {
            Matrix<F> a(Eigen::MatrixXd::Identity(2, 2)), b(Eigen::MatrixXd::Ones(2, 2));
            Matrix<F> first(2, 2), second(2, 2), det(1, 1);
            first.sentinel(); second.sentinel(); det.sentinel();
            const auto old_first = first.data, old_second = second.data, old_det = det.data;
            a.data[1] = invalid;
            Scratch<F> scratch(LINEARS_QR_WORKSPACE(2, 2));
            size_t permutation[2] = {9, 9}; int parity = 9;
            EXPECT_EQ(F::lu(&a.matrix, &first.matrix, &second.matrix, permutation, 2, &parity, scratch.workspace), LINEARS_ERROR_RANGE);
            EXPECT_EQ(F::solve(&a.matrix, &b.matrix, &first.matrix, scratch.workspace), LINEARS_ERROR_RANGE);
            EXPECT_EQ(F::inverse(&a.matrix, &first.matrix, scratch.workspace), LINEARS_ERROR_RANGE);
            EXPECT_EQ(F::determinant(&a.matrix, &det.matrix, scratch.workspace), LINEARS_ERROR_RANGE);
            EXPECT_EQ(F::cholesky(&a.matrix, &first.matrix, scratch.workspace), LINEARS_ERROR_RANGE);
            EXPECT_EQ(F::qr(&a.matrix, &first.matrix, &second.matrix, scratch.workspace), LINEARS_ERROR_RANGE);
            EXPECT_EQ(first.data, old_first); EXPECT_EQ(second.data, old_second); EXPECT_EQ(det.data, old_det);
            EXPECT_EQ(permutation[0], 9u); EXPECT_EQ(parity, 9);
            scratch.check();
        }
    } else {
        // Every raw input code is valid. A range failure instead comes from
        // an actual computed inverse outside the fixed output domain.
        Matrix<F> a(0.5 * Eigen::MatrixXd::Identity(2, 2)), x(2, 2);
        x.sentinel(); const auto old = x.data;
        Scratch<F> scratch(LINEARS_INVERSE_WORKSPACE(2));
        EXPECT_EQ(F::inverse(&a.matrix, &x.matrix, scratch.workspace), LINEARS_ERROR_RANGE);
        EXPECT_EQ(x.data, old);
        scratch.check();
    }
}

TYPED_TEST(DecompositionContract, EveryScratchRequirementHasAnImmediateGuard)
{
    using F = TypeParam;
    Matrix<F> a(Eigen::MatrixXd::Identity(3, 3)), b(Eigen::MatrixXd::Ones(3, 3));
    Matrix<F> first(3, 3), second(3, 3), determinant(1, 1);
    first.sentinel(); second.sentinel(); determinant.sentinel();
    const auto old_first = first.data, old_second = second.data, old_det = determinant.data;
    Scratch<F> short_factor(LINEARS_LU_WORKSPACE(3) - 1);
    Scratch<F> short_solve(LINEARS_SOLVE_WORKSPACE(3, 3) - 1);
    Scratch<F> short_qr(LINEARS_QR_WORKSPACE(3, 3) - 1);
    size_t p[3] = {9, 9, 9}; int parity = 9;
    EXPECT_EQ(F::lu(&a.matrix, &first.matrix, &second.matrix, p, 3, &parity, short_factor.workspace), LINEARS_ERROR_WORKSPACE);
    EXPECT_EQ(F::cholesky(&a.matrix, &first.matrix, short_factor.workspace), LINEARS_ERROR_WORKSPACE);
    EXPECT_EQ(F::determinant(&a.matrix, &determinant.matrix, short_factor.workspace), LINEARS_ERROR_WORKSPACE);
    EXPECT_EQ(F::solve(&a.matrix, &b.matrix, &first.matrix, short_solve.workspace), LINEARS_ERROR_WORKSPACE);
    EXPECT_EQ(F::inverse(&a.matrix, &first.matrix, short_solve.workspace), LINEARS_ERROR_WORKSPACE);
    EXPECT_EQ(F::qr(&a.matrix, &first.matrix, &second.matrix, short_qr.workspace), LINEARS_ERROR_WORKSPACE);
    EXPECT_EQ(first.data, old_first); EXPECT_EQ(second.data, old_second); EXPECT_EQ(determinant.data, old_det);
    short_factor.check(); short_solve.check(); short_qr.check();
}

TYPED_TEST(DecompositionContract, SeparateCallsAreReentrant)
{
    using F = TypeParam;
    auto invoke = [](uint32_t seed) {
        Eigen::MatrixXd values = random_matrix(5, 5, seed);
        values.diagonal().array() += 5.0;
        Eigen::MatrixXd rhs = random_matrix(5, 2, seed + 1);
        if constexpr (F::bits != 0) { values /= 10.0; rhs *= 0.1; }
        Matrix<F> a(values), b(rhs), x(5, 2);
        Scratch<F> scratch(LINEARS_SOLVE_WORKSPACE(5, 2));
        for (int iteration = 0; iteration < 16; ++iteration)
            ASSERT_EQ(F::solve(&a.matrix, &b.matrix, &x.matrix, scratch.workspace), LINEARS_OK);
        const Eigen::MatrixXd reference = a.decoded().partialPivLu().solve(b.decoded());
        EXPECT_LE((x.decoded() - reference).norm(), quantization_bound(x) +
            80.0 * F::working_epsilon * (F::bits == 0 ? reference.norm() : std::sqrt(10.0)));
        scratch.check();
    };
    std::thread first(invoke, 11u), second(invoke, 73u);
    first.join(); second.join();
}
} // namespace
