#include "linears_work.h"
#include <gtest/gtest.h>
#include <array>
#include <cstdint>
#include <limits>

TEST(LinearsWorkingMatrices, IdentityIsExactAndDoesNotAttenuateRepeatedProducts) {
    int64_t identity[4]{};
    int64_t values[] = {2 * LINEARS_FIXED_ONE, -3 * LINEARS_FIXED_ONE, 1, INT64_C(123456789)};
    std::array<int64_t, 4> original{values[0], values[1], values[2], values[3]};
    linears_matrix_t a{identity, 2, 2}, b{values, 2, 2};
    linears_workspace_unit_t workspace[4];
    ASSERT_EQ(linears_identity_fixed(&a), LINEARS_OK);
    EXPECT_EQ(identity[0], LINEARS_FIXED_ONE);
    EXPECT_EQ(identity[3], LINEARS_FIXED_ONE);
    for (unsigned i = 0; i < 32; ++i) {
        ASSERT_EQ(linears_mul_fixed(&a, &b, &b, workspace, sizeof(workspace)), LINEARS_OK);
        for (unsigned j = 0; j < 4; ++j) EXPECT_EQ(values[j], original[j]);
    }
}

TEST(LinearsWorkingMatrices, AdditionSubtractionAndWeightedSumKeepWideRange) {
    int64_t first[] = {2 * LINEARS_FIXED_ONE, 3 * LINEARS_FIXED_ONE};
    int64_t second[] = {4 * LINEARS_FIXED_ONE, -LINEARS_FIXED_ONE};
    int64_t result[2]{};
    linears_matrix_t a{first, 1, 2}, b{second, 1, 2}, out{result, 1, 2};
    linears_workspace_unit_t workspace[2];
    ASSERT_EQ(linears_add_fixed(&a, &b, &out, workspace, sizeof(workspace)), LINEARS_OK);
    EXPECT_EQ(result[0], 6 * LINEARS_FIXED_ONE);
    EXPECT_EQ(result[1], 2 * LINEARS_FIXED_ONE);
    ASSERT_EQ(linears_sub_fixed(&a, &b, &out, workspace, sizeof(workspace)), LINEARS_OK);
    EXPECT_EQ(result[0], -2 * LINEARS_FIXED_ONE);
    EXPECT_EQ(result[1], 4 * LINEARS_FIXED_ONE);
    ASSERT_EQ(linears_axpby_fixed(2 * LINEARS_FIXED_ONE, &a, -LINEARS_FIXED_ONE,
                                 &b, &out, workspace, sizeof(workspace)), LINEARS_OK);
    EXPECT_EQ(result[0], 0);
    EXPECT_EQ(result[1], 7 * LINEARS_FIXED_ONE);
}

TEST(LinearsWorkingMatrices, RectangularTransposeUsesTheSameTypedScratch) {
    int64_t input[] = {1, 2, 3, 4, 5, 6}, output[6]{};
    const int64_t expected[] = {1, 4, 2, 5, 3, 6};
    linears_matrix_t a{input, 2, 3}, out{output, 3, 2};
    linears_workspace_unit_t workspace[6];
    ASSERT_EQ(linears_transpose_fixed(&a, &out, workspace, sizeof(workspace)), LINEARS_OK);
    for (unsigned i = 0; i < 6; ++i) EXPECT_EQ(output[i], expected[i]);
}

TEST(LinearsWorkingMatrices, WeightedProductsRoundAtTheOutputOnly) {
    int64_t value = 1, result = 99;
    linears_matrix_t a{&value, 1, 1}, out{&result, 1, 1};
    linears_workspace_unit_t workspace[1];
    ASSERT_EQ(linears_axpby_fixed(LINEARS_FIXED_ONE / 2, &a, LINEARS_FIXED_ONE / 2,
                                 &a, &out, workspace, sizeof(workspace)), LINEARS_OK);
    EXPECT_EQ(result, 1);
}

TEST(LinearsWorkingMatrices, WorkingAccumulatorOverflowIsAtomicEvenBeforeLaterCancellation) {
    int64_t first[] = {INT64_MIN, INT64_MIN, INT64_MIN, INT64_MIN};
    int64_t second[] = {INT64_MIN, INT64_MIN, INT64_MAX, INT64_MAX};
    int64_t result = 99;
    linears_matrix_t a{first, 1, 4}, b{second, 4, 1}, out{&result, 1, 1};
    linears_workspace_unit_t workspace[1];
    EXPECT_EQ(linears_mul_fixed(&a, &b, &out, workspace, sizeof(workspace)), LINEARS_ERROR_RANGE);
    EXPECT_EQ(result, 99);
    first[0] = INT64_MAX;
    second[0] = 1;
    a.cols = b.rows = b.cols = 1;
    EXPECT_EQ(linears_add_fixed(&a, &b, &out, workspace, sizeof(workspace)), LINEARS_ERROR_RANGE);
    EXPECT_EQ(result, 99);
}

TEST(LinearsWorkingMatrices, CapacityAliasingAndEmptyShapeUseBasicContracts) {
    int64_t input = LINEARS_FIXED_ONE, result = 99;
    linears_matrix_t a{&input, 1, 1}, out{&result, 1, 1};
    linears_workspace_unit_t workspace[1];
    EXPECT_EQ(linears_add_fixed(&a, &a, &out, workspace, sizeof(workspace) - 1), LINEARS_ERROR_WORKSPACE);
    EXPECT_EQ(result, 99);
    EXPECT_EQ(linears_add_fixed(&a, &a, &out,
                               reinterpret_cast<linears_workspace_unit_t *>(&input), sizeof(workspace)), LINEARS_ERROR_ALIAS);
    EXPECT_EQ(result, 99);
    linears_matrix_t empty{nullptr, 0, 0};
    EXPECT_EQ(linears_identity_fixed(&empty), LINEARS_OK);
    EXPECT_EQ(linears_mul_fixed(&empty, &empty, &empty, nullptr, 0), LINEARS_OK);
}
