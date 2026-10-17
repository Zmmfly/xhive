#include "basic_test_types.h"

using namespace linears_test;
template<class T> class LinearsProducts : public ::testing::Test {};
TYPED_TEST_SUITE(LinearsProducts, Types);

TYPED_TEST(LinearsProducts, RectangularProductsAgainstQuantizedEigen) {
    using T = TypeParam;
    for (size_t m : {size_t(1), size_t(2), size_t(7)})
        for (size_t k : {size_t(1), size_t(3), size_t(9)})
            for (size_t n : {size_t(1), size_t(5)}) {
                Matrix<T> a(samples(m, k, 71)), b(samples(k, n, 28)), out(m, n);
                std::array<linears_workspace_unit_t, 35> workspace;
                Dense expected = a.decoded() * b.decoded();
                ASSERT_EQ(T::mul(&a.view, &b.view, &out.view, workspace.data(), sizeof(workspace)), LINEARS_OK);
                expect_close<T>(out.decoded(), expected);
            }
}

TYPED_TEST(LinearsProducts, MatvecAndInPlaceSquareProduct) {
    using T = TypeParam;
    Matrix<T> a(samples(6, 4, 1)), b(samples(4, 1, 2)), out(6, 1);
    std::array<linears_workspace_unit_t, 16> workspace;
    ASSERT_EQ(T::matvec(&a.view, &b.view, &out.view, workspace.data(), sizeof(workspace)), LINEARS_OK);
    expect_close<T>(out.decoded(), a.decoded() * b.decoded());
    Matrix<T> square(samples(4, 4, 3));
    Dense expected = square.decoded() * square.decoded();
    ASSERT_EQ(T::mul(&square.view, &square.view, &square.view, workspace.data(), sizeof(workspace)), LINEARS_OK);
    expect_close<T>(square.decoded(), expected);
}

TYPED_TEST(LinearsProducts, EmptyInnerDimensionFillsNonemptyOutputWithZero) {
    using T = TypeParam;
    linears_matrix_t a{nullptr, 3, 0, 0}, b{nullptr, 0, 4, 0};
    Matrix<T> out(samples(3, 4, 99));
    std::array<linears_workspace_unit_t, 12> workspace;
    ASSERT_EQ(T::mul(&a, &b, &out.view, workspace.data(), sizeof(workspace)), LINEARS_OK);
    expect_close<T>(out.decoded(), Dense::Zero(3, 4));
    if constexpr (T::bits != 0) { EXPECT_EQ(out.view.exponent, 0); }
}
