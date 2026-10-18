#include "basic_test_types.h"
#include <climits>
#include <cstring>

using namespace linears_test;
template<class T> class LinearsBasicContract : public ::testing::Test {};
TYPED_TEST_SUITE(LinearsBasicContract, Types);

TYPED_TEST(LinearsBasicContract, ErrorsPreserveOutputAndExposeWorkspaceBound) {
    using T = TypeParam;
    Matrix<T> a(samples_for<T>(2, 2)), b(samples_for<T>(2, 2, 2)), out(samples_for<T>(2, 2, 3));
    std::array<linears_workspace_unit_t, 4> workspace;
    auto original = out.storage;
    linears_matrix_t original_descriptor = out.view;
    EXPECT_EQ(T::add(nullptr, &b.view, &out.view, workspace.data(), sizeof(workspace)), LINEARS_ERROR_ARGUMENT);
    EXPECT_EQ(T::add(&a.view, nullptr, &out.view, workspace.data(), sizeof(workspace)), LINEARS_ERROR_ARGUMENT);
    EXPECT_EQ(T::add(&a.view, &b.view, &out.view, nullptr, sizeof(workspace)), LINEARS_ERROR_WORKSPACE);
    EXPECT_EQ(T::add(&a.view, &b.view, &out.view, workspace.data(), sizeof(workspace) - 1), LINEARS_ERROR_WORKSPACE);
    b.view.rows = 1;
    EXPECT_EQ(T::add(&a.view, &b.view, &out.view, workspace.data(), sizeof(workspace)), LINEARS_ERROR_DIMENSION);
    EXPECT_EQ(T::matvec(&a.view, &a.view, &out.view, workspace.data(), sizeof(workspace)), LINEARS_ERROR_DIMENSION);
    EXPECT_EQ(out.storage, original);
    EXPECT_EQ(out.view.data, original_descriptor.data);
    EXPECT_EQ(out.view.rows, original_descriptor.rows);
    EXPECT_EQ(out.view.cols, original_descriptor.cols);
    EXPECT_EQ(LINEARS_BASIC_WORKSPACE_BYTES(2, 2), sizeof(workspace));
    EXPECT_EQ(LINEARS_SIZE_MUL(0, 0), size_t(0));
    EXPECT_EQ(LINEARS_SIZE_MUL(SIZE_MAX, 0), size_t(0));
    EXPECT_EQ(LINEARS_SIZE_MUL(0, SIZE_MAX), size_t(0));
    static_assert(LINEARS_SIZE_MUL(0, 0) == 0, "zero dimension remains constexpr");
}

TYPED_TEST(LinearsBasicContract, WorkspaceCanariesAndOverlapsAreChecked) {
    using T = TypeParam;
    Matrix<T> a(samples_for<T>(2, 2)), b(samples_for<T>(2, 2, 2)), out(2, 2);
    struct Guarded {
        uint64_t first;
        linears_workspace_unit_t slots[4];
        uint64_t last;
    } workspace;
    workspace.first = workspace.last = UINT64_C(0x123456789abcdef0);
    ASSERT_EQ(T::add(&a.view, &b.view, &out.view, workspace.slots, sizeof(workspace.slots)), LINEARS_OK);
    EXPECT_EQ(workspace.first, UINT64_C(0x123456789abcdef0));
    EXPECT_EQ(workspace.last, UINT64_C(0x123456789abcdef0));
    alignas(linears_workspace_unit_t) typename T::Raw aliased[4]{};
    a.view.data = aliased;
    EXPECT_EQ(T::add(&a.view, &b.view, &out.view,
                    reinterpret_cast<linears_workspace_unit_t *>(aliased), sizeof(workspace.slots)), LINEARS_ERROR_ALIAS);
    a.view.data = &a.view;
    EXPECT_EQ(T::add(&a.view, &b.view, &out.view, workspace.slots, sizeof(workspace.slots)), LINEARS_ERROR_ALIAS);
}

TYPED_TEST(LinearsBasicContract, SizeAlignmentAndEmptyDimensions) {
    using T = TypeParam;
    std::array<linears_workspace_unit_t, 4> workspace;
    Matrix<T> a(2, 2), out(2, 2);
    linears_matrix_t huge{nullptr, SIZE_MAX, 2};
    EXPECT_EQ(T::transpose(&huge, &out.view, workspace.data(), sizeof(workspace)), LINEARS_ERROR_SIZE);
    alignas(linears_workspace_unit_t) unsigned char misaligned[64]{};
    a.view.data = misaligned + 1;
    EXPECT_EQ(T::copy(&a.view, &out.view), LINEARS_ERROR_ARGUMENT);
    linears_matrix_t empty{nullptr, SIZE_MAX, 0}, transposed{nullptr, 0, SIZE_MAX};
    ASSERT_EQ(T::add(&empty, &empty, &empty, nullptr, 0), LINEARS_OK);
    ASSERT_EQ(T::transpose(&empty, &transposed, nullptr, 0), LINEARS_OK);
    ASSERT_EQ(T::zero(&empty), LINEARS_OK);
    EXPECT_EQ(T::identity(&empty), LINEARS_ERROR_DIMENSION);
    linears_matrix_t square{nullptr, 0, 0};
    EXPECT_EQ(T::identity(&square), LINEARS_OK);
}

TYPED_TEST(LinearsBasicContract, CopyUsesMemmoveForPartialOverlap) {
    using T = TypeParam;
    Matrix<T> source(samples_for<T>(1, 6, 67));
    auto original = source.storage;
    linears_matrix_t a{source.storage.data(), 1, 5};
    linears_matrix_t out{source.storage.data() + 1, 1, 5};
    ASSERT_EQ(T::copy(&a, &out), LINEARS_OK);
    for (size_t i = 0; i < 5; ++i) EXPECT_EQ(source.storage[i + 1], original[i]);
}

TEST(LinearsBasicSizes, RuntimeAndLiteralZeroMacrosStayChecked) {
    const size_t values[] = {0, 1, 2, 3, SIZE_MAX / 2, SIZE_MAX / 2 + 1, SIZE_MAX - 1, SIZE_MAX};
    for (size_t a : values) for (size_t b : values) {
        size_t product = b != 0 && a > SIZE_MAX / b ? SIZE_MAX : a * b;
        size_t sum = b > SIZE_MAX - a ? SIZE_MAX : a + b;
        EXPECT_EQ(LINEARS_SIZE_MUL(a, b), product);
        EXPECT_EQ(LINEARS_SIZE_ADD(a, b), sum);
        EXPECT_EQ(LINEARS_SIZE_MUL(0, b), size_t(0));
        EXPECT_EQ(LINEARS_SIZE_MUL(a, 0), size_t(0));
        EXPECT_EQ(LINEARS_SIZE_ADD(0, b), b);
        EXPECT_EQ(LINEARS_SIZE_ADD(a, 0), a);
    }
}

TEST(LinearsBasicRange, FloatNonfiniteAndOverflowLeaveOutputUnchanged) {
    float input[] = {std::numeric_limits<float>::max(), 1};
    float output[] = {7, 8};
    linears_matrix_t a{input, 1, 2}, out{output, 1, 2};
    linears_workspace_unit_t workspace[2];
    EXPECT_EQ(linears_scale_f32(2, &a, &out, workspace, sizeof(workspace)), LINEARS_ERROR_RANGE);
    EXPECT_EQ(output[0], 7);
    EXPECT_EQ(output[1], 8);
    input[1] = std::numeric_limits<float>::quiet_NaN();
    EXPECT_EQ(linears_add_f32(&a, &a, &out, workspace, sizeof(workspace)), LINEARS_ERROR_RANGE);
    EXPECT_EQ(output[0], 7);
    double dinput = std::numeric_limits<double>::infinity(), doutput = 123;
    linears_matrix_t d{&dinput, 1, 1};
    EXPECT_EQ(linears_norm_f64(&d, &doutput), LINEARS_ERROR_RANGE);
    EXPECT_EQ(doutput, 123);
}

TEST(LinearsBasicRange, FixedRangeFailureDoesNotPublishPartialResult) {
    int32_t input[] = {INT32_C(1073741824), INT32_MAX}, output[] = {123, 456};
    linears_matrix_t a{input, 1, 2}, out{output, 1, 2};
    linears_workspace_unit_t workspace[2];
    EXPECT_EQ(linears_add_q31(&a, &a, &out, workspace, sizeof(workspace)), LINEARS_ERROR_RANGE);
    EXPECT_EQ(output[0], 123);
    EXPECT_EQ(output[1], 456);
    EXPECT_EQ(linears_scale_q31(2 * LINEARS_FIXED_ONE, &a, &out, workspace, sizeof(workspace)), LINEARS_ERROR_RANGE);
    EXPECT_EQ(output[0], 123);
    EXPECT_EQ(output[1], 456);
}

TEST(LinearsBasicRange, ScalarOutputCannotOverwriteDescriptor) {
    double input[] = {0.25, 0.5};
    linears_matrix_t a{input, 1, 2};
    linears_matrix_t before = a;
    EXPECT_EQ(linears_norm_f64(&a, reinterpret_cast<double *>(&a)), LINEARS_ERROR_ALIAS);
    EXPECT_EQ(a.data, before.data);
    EXPECT_EQ(a.rows, before.rows);
    EXPECT_EQ(a.cols, before.cols);
}
