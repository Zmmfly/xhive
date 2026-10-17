#include "basic_test_types.h"

using namespace linears_test;
template<class T> class LinearsElementwise : public ::testing::Test {};
TYPED_TEST_SUITE(LinearsElementwise, Types);

TYPED_TEST(LinearsElementwise, AddSubtractAndHadamardUseQuantizedEigenReference) {
    using T = TypeParam;
    Matrix<T> a(samples(4, 7, 12)), b(samples(4, 7, 91), 2), out(4, 7);
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
    Matrix<T> a(samples(3, 5, 8)), b(samples(3, 5, 6));
    std::array<linears_workspace_unit_t, 15> workspace;
    Dense x = a.decoded(), y = b.decoded();
    auto alpha = scalar<T>(-1.25), beta = scalar<T>(0.375);
    ASSERT_EQ(T::axpby(alpha, &a.view, beta, &b.view, &a.view, workspace.data(), sizeof(workspace)), LINEARS_OK);
    expect_close<T>(a.decoded(), decoded<T>(alpha) * x + decoded<T>(beta) * y);
    x = a.decoded();
    ASSERT_EQ(T::scale(beta, &a.view, &a.view, workspace.data(), sizeof(workspace)), LINEARS_OK);
    expect_close<T>(a.decoded(), decoded<T>(beta) * x);
}

TYPED_TEST(LinearsElementwise, RectangularAndInPlaceTranspose) {
    using T = TypeParam;
    Matrix<T> a(samples(3, 5, 2)), out(5, 3), square(samples(4, 4, 77));
    std::array<linears_workspace_unit_t, 16> workspace;
    ASSERT_EQ(T::transpose(&a.view, &out.view, workspace.data(), sizeof(workspace)), LINEARS_OK);
    expect_close<T>(out.decoded(), a.decoded().transpose());
    Dense expected = square.decoded().transpose();
    ASSERT_EQ(T::transpose(&square.view, &square.view, workspace.data(), sizeof(workspace)), LINEARS_OK);
    expect_close<T>(square.decoded(), expected);
    // Rectangular transpose can reuse storage through a distinct descriptor.
    linears_matrix_t reused{a.storage.data(), 5, 3, a.view.exponent};
    ASSERT_EQ(T::transpose(&a.view, &reused, workspace.data(), sizeof(workspace)), LINEARS_OK);
    a.view = reused;
    expect_close<T>(a.decoded(), out.decoded());
}

TYPED_TEST(LinearsElementwise, CopyPreservesRepresentationAndFillNeedsNoWorkspace) {
    using T = TypeParam;
    Matrix<T> a(samples(3, 3, 4), 3), out(3, 3, 99);
    ASSERT_EQ(T::copy(&a.view, &out.view), LINEARS_OK);
    EXPECT_EQ(a.storage, out.storage);
    EXPECT_EQ(out.view.exponent, T::bits == 0 ? 99 : 3);
    ASSERT_EQ(T::zero(&out.view), LINEARS_OK);
    expect_close<T>(out.decoded(), Dense::Zero(3, 3));
    ASSERT_EQ(T::identity(&out.view), LINEARS_OK);
    expect_close<T>(out.decoded(), Dense::Identity(3, 3));
    if constexpr (T::bits != 0) { EXPECT_EQ(out.view.exponent, 1); }
}

TYPED_TEST(LinearsElementwise, PartialInputOutputOverlapUsesStagedOutput) {
    using T = TypeParam;
    Matrix<T> storage(samples(1, 9, 40)), b(samples(1, 6, 66));
    linears_matrix_t a{storage.storage.data(), 1, 6, storage.view.exponent};
    linears_matrix_t out{storage.storage.data() + 2, 1, 6, 42};
    std::array<linears_workspace_unit_t, 6> workspace;
    Dense expected = storage.decoded().leftCols(6) + b.decoded();
    ASSERT_EQ(T::add(&a, &b.view, &out, workspace.data(), sizeof(workspace)), LINEARS_OK);
    storage.view = out;
    expect_close<T>(storage.decoded(), expected);
}
