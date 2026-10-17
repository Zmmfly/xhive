#include "basic_test_types.h"

using namespace linears_test;
template<class T> class LinearsReductions : public ::testing::Test {};
TYPED_TEST_SUITE(LinearsReductions, Types);

TYPED_TEST(LinearsReductions, DotNormAndTraceAgainstQuantizedEigen) {
    using T = TypeParam;
    Matrix<T> a(samples(7, 7, 13)), b(samples(7, 7, 80));
    typename T::Scalar value{};
    Dense x = a.decoded(), y = b.decoded();
    ASSERT_EQ(T::dot(&a.view, &b.view, &value), LINEARS_OK);
    double expected = (x.array() * y.array()).sum();
    EXPECT_NEAR(decoded<T>(value), expected, T::tolerance * std::max(1.0, std::abs(expected)));
    ASSERT_EQ(T::norm(&a.view, &value), LINEARS_OK);
    EXPECT_NEAR(decoded<T>(value), x.norm(), T::tolerance * x.norm());
    ASSERT_EQ(T::trace(&a.view, &value), LINEARS_OK);
    EXPECT_NEAR(decoded<T>(value), x.trace(), T::tolerance * std::max(1.0, std::abs(x.trace())));
}

TYPED_TEST(LinearsReductions, EmptyReductionsAreZero) {
    using T = TypeParam;
    linears_matrix_t square{nullptr, 0, 0, 0}, row{nullptr, 0, 123, 0};
    typename T::Scalar value = scalar<T>(42);
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
    linears_matrix_t a{f32, 2, 1, 999}, b{f64, 2, 1, -999};
    float small;
    double large;
    ASSERT_EQ(linears_norm_f32(&a, &small), LINEARS_OK);
    ASSERT_EQ(linears_norm_f64(&b, &large), LINEARS_OK);
    EXPECT_NEAR(small / 1e30f, std::sqrt(2.0), 2e-7);
    EXPECT_NEAR(large / 1e300, std::sqrt(2.0), 3e-16);
}

TEST(LinearsReductionRange, ScaledFixedNormWorksAtMaximumWorkingExponent) {
    int32_t data[] = {INT32_C(1073741824), INT32_C(1073741824)};
    linears_matrix_t a{data, 2, 1, LINEARS_EXPONENT_MAX};
    linears_q31_t value{};
    ASSERT_EQ(linears_norm_q31(&a, &value), LINEARS_OK);
    EXPECT_EQ(value.exponent, LINEARS_EXPONENT_MAX);
    EXPECT_NEAR(static_cast<double>(value.mantissa) / INT32_C(1073741824), std::sqrt(2.0), 2e-9);
}

TEST(LinearsReductionRange, Q15NormAcceptsFullSharedExponentMagnitudeSpan) {
    int16_t data[] = {INT16_MIN, 1};
    for (int exponent : {-100, 0, 100}) {
        linears_matrix_t a{data, 2, 1, exponent};
        linears_q15_t value{};
        ASSERT_EQ(linears_norm_q15(&a, &value), LINEARS_OK);
        EXPECT_EQ(value.mantissa, INT16_C(16384));
        EXPECT_EQ(value.exponent, exponent + 1);
        double normalized = std::ldexp(static_cast<double>(value.mantissa), value.exponent - exponent - 15);
        EXPECT_NEAR(normalized, std::hypot(1.0, std::ldexp(1.0, -15)), std::ldexp(1.0, -15));
    }
}

TEST(LinearsReductionRange, Q31NormAcceptsFullSharedExponentMagnitudeSpan) {
    int32_t data[] = {INT32_MIN, 1};
    for (int exponent : {-100, 0, 100}) {
        linears_matrix_t a{data, 2, 1, exponent};
        linears_q31_t value{};
        ASSERT_EQ(linears_norm_q31(&a, &value), LINEARS_OK);
        EXPECT_EQ(value.mantissa, INT32_C(1073741824));
        EXPECT_EQ(value.exponent, exponent + 1);
        double normalized = std::ldexp(static_cast<double>(value.mantissa), value.exponent - exponent - 31);
        EXPECT_NEAR(normalized, std::hypot(1.0, std::ldexp(1.0, -31)), std::ldexp(1.0, -31));
    }
}
