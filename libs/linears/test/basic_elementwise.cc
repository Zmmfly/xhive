#include "basic_test_types.h"

using namespace linears_test;
template<class T> class LinearsElementwise : public ::testing::Test {};
TYPED_TEST_SUITE(LinearsElementwise, Types);

TYPED_TEST(LinearsElementwise, AddSubtractAndHadamardUseQuantizedEigenReference) {
    using T = TypeParam;
    Matrix<T> a(samples_for<T>(4, 7, 12)), b(samples_for<T>(4, 7, 91)), out(4, 7);
    std::array<linears_workspace_unit_t, 28> workspace;
    Dense x = a.decoded(), y = b.decoded();
    ASSERT_EQ(T::add(&a.view, &b.view, &out.view, workspace.data(), sizeof(workspace)), LINEARS_OK);
    expect_close<T>(out.decoded(), x + y);
    ASSERT_EQ(T::sub(&a.view, &b.view, &out.view, workspace.data(), sizeof(workspace)), LINEARS_OK);
    expect_close<T>(out.decoded(), x - y);
    ASSERT_EQ(T::hadamard(&a.view, &b.view, &out.view, workspace.data(), sizeof(workspace)), LINEARS_OK);
    expect_close<T>(out.decoded(), x.cwiseProduct(y));
}

TYPED_TEST(LinearsElementwise, ScalingAndWeightedSumAcceptInPlaceOutput) {
    using T = TypeParam;
    Matrix<T> a(samples_for<T>(3, 5, 8)), b(samples_for<T>(3, 5, 6));
    std::array<linears_workspace_unit_t, 15> workspace;
    Dense x = a.decoded(), y = b.decoded();
    auto alpha = coefficient<T>(-1.25), beta = coefficient<T>(0.375);
    ASSERT_EQ(T::axpby(alpha, &a.view, beta, &b.view, &a.view, workspace.data(), sizeof(workspace)), LINEARS_OK);
    expect_close<T>(a.decoded(), decoded_coefficient<T>(alpha) * x + decoded_coefficient<T>(beta) * y);
    x = a.decoded();
    ASSERT_EQ(T::scale(beta, &a.view, &a.view, workspace.data(), sizeof(workspace)), LINEARS_OK);
    expect_close<T>(a.decoded(), decoded_coefficient<T>(beta) * x);
}

TYPED_TEST(LinearsElementwise, RectangularAndInPlaceTranspose) {
    using T = TypeParam;
    Matrix<T> a(samples_for<T>(3, 5, 2)), out(5, 3), square(samples_for<T>(4, 4, 77));
    std::array<linears_workspace_unit_t, 16> workspace;
    ASSERT_EQ(T::transpose(&a.view, &out.view, workspace.data(), sizeof(workspace)), LINEARS_OK);
    expect_close<T>(out.decoded(), a.decoded().transpose());
    Dense expected = square.decoded().transpose();
    ASSERT_EQ(T::transpose(&square.view, &square.view, workspace.data(), sizeof(workspace)), LINEARS_OK);
    expect_close<T>(square.decoded(), expected);
    // Rectangular transpose can reuse storage through a distinct descriptor.
    linears_matrix_t reused{a.storage.data(), 5, 3};
    ASSERT_EQ(T::transpose(&a.view, &reused, workspace.data(), sizeof(workspace)), LINEARS_OK);
    a.view = reused;
    expect_close<T>(a.decoded(), out.decoded());
}

TYPED_TEST(LinearsElementwise, CopyPreservesRepresentationAndFillNeedsNoWorkspace) {
    using T = TypeParam;
    Matrix<T> a(samples_for<T>(3, 3, 4)), out(3, 3);
    ASSERT_EQ(T::copy(&a.view, &out.view), LINEARS_OK);
    EXPECT_EQ(a.storage, out.storage);
    ASSERT_EQ(T::zero(&out.view), LINEARS_OK);
    expect_close<T>(out.decoded(), Dense::Zero(3, 3));
    ASSERT_EQ(T::identity(&out.view), LINEARS_OK);
    expect_close<T>(out.decoded(), Dense::Identity(3, 3));
    if constexpr (T::bits != 0) { EXPECT_EQ(out.storage[0], std::numeric_limits<typename T::Raw>::max()); }
}

TYPED_TEST(LinearsElementwise, PartialInputOutputOverlapUsesStagedOutput) {
    using T = TypeParam;
    Matrix<T> storage(samples_for<T>(1, 9, 40)), b(samples_for<T>(1, 6, 66));
    linears_matrix_t a{storage.storage.data(), 1, 6};
    linears_matrix_t out{storage.storage.data() + 2, 1, 6};
    std::array<linears_workspace_unit_t, 6> workspace;
    Dense expected = storage.decoded().leftCols(6) + b.decoded();
    ASSERT_EQ(T::add(&a, &b.view, &out, workspace.data(), sizeof(workspace)), LINEARS_OK);
    storage.view = out;
    expect_close<T>(storage.decoded(), expected);
}

TYPED_TEST(LinearsElementwise, WideCoefficientPreservesGainAboveOne) {
    using T = TypeParam;
    Matrix<T> input(Dense::Constant(2, 2, 0.25)), output(2, 2);
    linears_workspace_unit_t workspace[4];
    ASSERT_EQ(T::scale(coefficient<T>(2), &input.view, &output.view, workspace, sizeof(workspace)), LINEARS_OK);
    expect_close<T>(output.decoded(), Dense::Constant(2, 2, 0.5));
}

TEST(LinearsRawWeightedSum, TwoHalfProductsAreRoundedOnlyOnce) {
    int16_t small = 1, small_out = 99;
    int32_t large = 1, large_out = 99;
    linears_matrix_t a{&small, 1, 1}, ao{&small_out, 1, 1};
    linears_matrix_t b{&large, 1, 1}, bo{&large_out, 1, 1};
    linears_workspace_unit_t workspace[1];
    ASSERT_EQ(linears_axpby_q15(LINEARS_FIXED_ONE / 2, &a, LINEARS_FIXED_ONE / 2, &a,
                              &ao, workspace, sizeof(workspace)), LINEARS_OK);
    ASSERT_EQ(linears_axpby_q31(LINEARS_FIXED_ONE / 2, &b, LINEARS_FIXED_ONE / 2, &b,
                              &bo, workspace, sizeof(workspace)), LINEARS_OK);
    EXPECT_EQ(small_out, 1);
    EXPECT_EQ(large_out, 1);
}
