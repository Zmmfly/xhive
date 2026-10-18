#include "basic_test_types.h"

using namespace linears_test;
template<class T> class LinearsProducts : public ::testing::Test {};
TYPED_TEST_SUITE(LinearsProducts, Types);

TYPED_TEST(LinearsProducts, RectangularProductsAgainstQuantizedEigen) {
    using T = TypeParam;
    for (size_t m : {size_t(1), size_t(2), size_t(7)})
        for (size_t k : {size_t(1), size_t(3), size_t(9)})
            for (size_t n : {size_t(1), size_t(5)}) {
                Matrix<T> a(samples_for<T>(m, k, 71)), b(samples_for<T>(k, n, 28)), out(m, n);
                std::array<linears_workspace_unit_t, 35> workspace;
                Dense expected = a.decoded() * b.decoded();
                ASSERT_EQ(T::mul(&a.view, &b.view, &out.view, workspace.data(), sizeof(workspace)), LINEARS_OK);
                expect_close<T>(out.decoded(), expected);
            }
}

TYPED_TEST(LinearsProducts, MatvecAndInPlaceSquareProduct) {
    using T = TypeParam;
    Matrix<T> a(samples_for<T>(6, 4, 1)), b(samples_for<T>(4, 1, 2)), out(6, 1);
    std::array<linears_workspace_unit_t, 16> workspace;
    ASSERT_EQ(T::matvec(&a.view, &b.view, &out.view, workspace.data(), sizeof(workspace)), LINEARS_OK);
    expect_close<T>(out.decoded(), a.decoded() * b.decoded());
    Matrix<T> square(samples_for<T>(4, 4, 3));
    Dense expected = square.decoded() * square.decoded();
    ASSERT_EQ(T::mul(&square.view, &square.view, &square.view, workspace.data(), sizeof(workspace)), LINEARS_OK);
    expect_close<T>(square.decoded(), expected);
}

TYPED_TEST(LinearsProducts, EmptyInnerDimensionFillsNonemptyOutputWithZero) {
    using T = TypeParam;
    linears_matrix_t a{nullptr, 3, 0}, b{nullptr, 0, 4};
    Matrix<T> out(samples_for<T>(3, 4, 99));
    std::array<linears_workspace_unit_t, 12> workspace;
    ASSERT_EQ(T::mul(&a, &b, &out.view, workspace.data(), sizeof(workspace)), LINEARS_OK);
    expect_close<T>(out.decoded(), Dense::Zero(3, 4));
}

TEST(LinearsRawProducts, MatrixProductRetainsSubLsbProductsUntilFinalRounding) {
    int16_t a15[] = {1, 1}, b15[] = {16384, 16384}, out15 = 99;
    int32_t a31[] = {1, 1}, b31[] = {INT32_C(1073741824), INT32_C(1073741824)}, out31 = 99;
    linears_matrix_t x15{a15, 1, 2}, y15{b15, 2, 1}, z15{&out15, 1, 1};
    linears_matrix_t x31{a31, 1, 2}, y31{b31, 2, 1}, z31{&out31, 1, 1};
    linears_workspace_unit_t workspace[1];
    ASSERT_EQ(linears_mul_q15(&x15, &y15, &z15, workspace, sizeof(workspace)), LINEARS_OK);
    ASSERT_EQ(linears_mul_q31(&x31, &y31, &z31, workspace, sizeof(workspace)), LINEARS_OK);
    EXPECT_EQ(out15, 1);
    EXPECT_EQ(out31, 1);
}
