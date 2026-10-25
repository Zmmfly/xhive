#include "linears_work.h"
#include "linears_numeric.h"
#include <gtest/gtest.h>
#include <array>
#include <cstdint>
#include <limits>

namespace {
constexpr linears_fixed_t one = LINEARS_FIXED_ONE;

TEST(WorkingDecomposition, SolveRetainsHeadroomAndExactUnitArithmetic)
{
    std::array<linears_fixed_t, 4> a_data{{4 * one, one, one, 3 * one}};
    std::array<linears_fixed_t, 2> b_data{{15 * one / 2, one / 2}}, x_data{{17, 19}};
    std::array<linears_fixed_t, LINEARS_SOLVE_WORKSPACE(2, 1) + 2> work{};
    work.front() = 37; work.back() = 41;
    linears_matrix_t a{a_data.data(), 2, 2}, b{b_data.data(), 2, 1}, x{x_data.data(), 2, 1};
    linears_fixed_workspace_t scratch{work.data() + 1, work.size() - 2};
    ASSERT_EQ(linears_solve_fixed(&a, &b, &x, scratch), LINEARS_OK);
    EXPECT_EQ(x_data[0], 2 * one);
    EXPECT_EQ(x_data[1], -one / 2);
    EXPECT_EQ(work.front(), 37); EXPECT_EQ(work.back(), 41);
}

TEST(WorkingDecomposition, CholeskyRetainsFactorsLargerThanOne)
{
    std::array<linears_fixed_t, 4> a_data{{4 * one, 2 * one, 2 * one, 5 * one}}, l_data{};
    std::array<linears_fixed_t, LINEARS_CHOLESKY_WORKSPACE(2) + 2> work{};
    work.front() = 37; work.back() = 41;
    linears_matrix_t a{a_data.data(), 2, 2}, l{l_data.data(), 2, 2};
    linears_fixed_workspace_t scratch{work.data() + 1, work.size() - 2};
    ASSERT_EQ(linears_cholesky_fixed(&a, &l, scratch), LINEARS_OK);
    EXPECT_EQ(l_data, (std::array<linears_fixed_t, 4>{{2 * one, 0, one, 2 * one}}));
    EXPECT_EQ(work.front(), 37); EXPECT_EQ(work.back(), 41);
}

TEST(WorkingDecomposition, CompleteInt64OutputRangeAndAtomicOverflow)
{
    linears_fixed_t a_value = one, b_value = std::numeric_limits<int64_t>::max(), x_value = 17;
    std::array<linears_fixed_t, LINEARS_SOLVE_WORKSPACE(1, 1)> work{};
    linears_matrix_t a{&a_value, 1, 1}, b{&b_value, 1, 1}, x{&x_value, 1, 1};
    linears_fixed_workspace_t scratch{work.data(), work.size()};
    ASSERT_EQ(linears_solve_fixed(&a, &b, &x, scratch), LINEARS_OK);
    EXPECT_EQ(x_value, std::numeric_limits<int64_t>::max());
    a_value = 1; b_value = -2 * one;
    ASSERT_EQ(linears_solve_fixed(&a, &b, &x, scratch), LINEARS_OK);
    EXPECT_EQ(x_value, std::numeric_limits<int64_t>::min());
    b_value = 2 * one; x_value = 17;
    EXPECT_EQ(linears_solve_fixed(&a, &b, &x, scratch), LINEARS_ERROR_RANGE);
    EXPECT_EQ(x_value, 17);
}

TEST(WorkingDecomposition, ShapeAliasCapacityAndNumericalFailuresAreAtomic)
{
    std::array<linears_fixed_t, 4> a_data{{one, one, one, one}}, l_data{{17, 19, 23, 29}};
    std::array<linears_fixed_t, 2> b_data{{one, 0}}, x_data{{31, 37}};
    std::array<linears_fixed_t, LINEARS_SOLVE_WORKSPACE(2, 1)> work{};
    linears_matrix_t a{a_data.data(), 2, 2}, b{b_data.data(), 2, 1}, x{x_data.data(), 2, 1};
    linears_matrix_t l{l_data.data(), 2, 2};
    linears_fixed_workspace_t scratch{work.data(), work.size()};
    const auto old_l = l_data;
    const auto old_x = x_data;
    EXPECT_EQ(linears_solve_fixed(&a, &b, &x, scratch), LINEARS_ERROR_SINGULAR);
    EXPECT_EQ(linears_cholesky_fixed(&a, &l, scratch), LINEARS_ERROR_NOT_POSITIVE_DEFINITE);
    EXPECT_EQ(l_data, old_l); EXPECT_EQ(x_data, old_x);
    scratch.capacity = 3;
    EXPECT_EQ(linears_cholesky_fixed(&a, &l, scratch), LINEARS_ERROR_WORKSPACE);
    scratch.capacity = work.size();
    linears_matrix_t alias{a_data.data(), 2, 2};
    EXPECT_EQ(linears_cholesky_fixed(&a, &alias, scratch), LINEARS_ERROR_ALIAS);
    linears_matrix_t wrong{l_data.data(), 1, 2};
    EXPECT_EQ(linears_cholesky_fixed(&a, &wrong, scratch), LINEARS_ERROR_DIMENSION);
    a_data[1] = 0;
    EXPECT_EQ(linears_cholesky_fixed(&a, &l, scratch), LINEARS_ERROR_DIMENSION);
    EXPECT_EQ(l_data, old_l);
    linears_matrix_t empty{nullptr, 0, 0}, rhs{nullptr, 0, 3}, result{nullptr, 0, 3};
    linears_fixed_workspace_t none{nullptr, 0};
    EXPECT_EQ(linears_solve_fixed(&empty, &rhs, &result, none), LINEARS_OK);
    EXPECT_EQ(linears_cholesky_fixed(&empty, &empty, none), LINEARS_ERROR_ALIAS);
    linears_matrix_t empty_l{nullptr, 0, 0};
    EXPECT_EQ(linears_cholesky_fixed(&empty, &empty_l, none), LINEARS_OK);
}
} // namespace
