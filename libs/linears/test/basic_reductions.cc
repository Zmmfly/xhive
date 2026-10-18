#include "basic_test_types.h"

using namespace linears_test;
template<class T> class LinearsReductions : public ::testing::Test {};
TYPED_TEST_SUITE(LinearsReductions, Types);

TYPED_TEST(LinearsReductions, DotNormAndTraceAgainstQuantizedEigen) {
    using T = TypeParam;
    Matrix<T> a(samples_for<T>(7, 7, 13)), b(samples_for<T>(7, 7, 80));
    typename T::Scalar value{};
    Dense x = a.decoded(), y = b.decoded();
    ASSERT_EQ(T::dot(&a.view, &b.view, &value), LINEARS_OK);
    double expected = (x.array() * y.array()).sum();
    EXPECT_NEAR(decoded<T>(value), expected, T::tolerance * std::max(1.0, std::abs(expected)));
    ASSERT_EQ(T::norm(&a.view, &value), LINEARS_OK);
    EXPECT_NEAR(decoded<T>(value), x.norm(), T::bits == 0 ? T::tolerance * x.norm() : T::tolerance);
    ASSERT_EQ(T::trace(&a.view, &value), LINEARS_OK);
    EXPECT_NEAR(decoded<T>(value), x.trace(), T::tolerance * std::max(1.0, std::abs(x.trace())));
}

TYPED_TEST(LinearsReductions, EmptyReductionsAreZero) {
    using T = TypeParam;
    linears_matrix_t square{nullptr, 0, 0}, row{nullptr, 0, 123};
    typename T::Scalar value = scalar<T>(T::bits == 0 ? 42 : 0.5);
    ASSERT_EQ(T::dot(&row, &row, &value), LINEARS_OK);
    EXPECT_EQ(decoded<T>(value), 0);
    ASSERT_EQ(T::norm(&row, &value), LINEARS_OK);
    EXPECT_EQ(decoded<T>(value), 0);
    ASSERT_EQ(T::trace(&square, &value), LINEARS_OK);
    EXPECT_EQ(decoded<T>(value), 0);
}

TEST(LinearsReductionRange, ScaledNormAvoidsFloatingIntermediateOverflow) {
    float f32[] = {1e30f, -1e30f};
    double f64[] = {1e300, -1e300};
    linears_matrix_t a{f32, 2, 1}, b{f64, 2, 1};
    float small;
    double large;
    ASSERT_EQ(linears_norm_f32(&a, &small), LINEARS_OK);
    ASSERT_EQ(linears_norm_f64(&b, &large), LINEARS_OK);
    EXPECT_NEAR(small / 1e30f, std::sqrt(2.0), 2e-7);
    EXPECT_NEAR(large / 1e300, std::sqrt(2.0), 3e-16);
}

TEST(LinearsReductionRange, ExactQNormOverflowDoesNotDisappearDuringRounding) {
    int16_t small[] = {INT16_MIN, 1};
    int32_t large[] = {INT32_MIN, 1};
    linears_matrix_t a{small, 2, 1}, b{large, 2, 1};
    int16_t q15 = 123;
    int32_t q31 = 456;
    EXPECT_EQ(linears_norm_q15(&a, &q15), LINEARS_ERROR_RANGE);
    EXPECT_EQ(linears_norm_q31(&b, &q31), LINEARS_ERROR_RANGE);
    EXPECT_EQ(q15, 123);
    EXPECT_EQ(q31, 456);
}

TEST(LinearsReductionRange, QNormNegativeEndpointAndTinyRawValue) {
    int16_t small = INT16_MIN;
    int32_t large = INT32_MIN;
    linears_matrix_t a{&small, 1, 1}, b{&large, 1, 1};
    int16_t q15 = 0;
    int32_t q31 = 0;
    ASSERT_EQ(linears_norm_q15(&a, &q15), LINEARS_OK);
    ASSERT_EQ(linears_norm_q31(&b, &q31), LINEARS_OK);
    EXPECT_EQ(q15, INT16_MAX);
    EXPECT_EQ(q31, INT32_MAX);
    small = large = 1;
    ASSERT_EQ(linears_norm_q15(&a, &q15), LINEARS_OK);
    ASSERT_EQ(linears_norm_q31(&b, &q31), LINEARS_OK);
    EXPECT_EQ(q15, 1);
    EXPECT_EQ(q31, 1);
}

TEST(LinearsReductionRange, RawProductsCancelAfterCrossingSigned64Headroom) {
    int32_t a_data[] = {INT32_MIN, INT32_MIN, INT32_MIN, INT32_MIN};
    int32_t b_data[] = {INT32_MIN, INT32_MIN, INT32_MAX, INT32_MAX};
    linears_matrix_t a{a_data, 1, 4}, b{b_data, 1, 4};
    int32_t result = 123;
    ASSERT_EQ(linears_dot_q31(&a, &b, &result), LINEARS_OK);
    EXPECT_EQ(result, 2);
    a.cols = b.cols = 2;
    result = 123;
    EXPECT_EQ(linears_dot_q31(&a, &b, &result), LINEARS_ERROR_RANGE);
    EXPECT_EQ(result, 123);
}
