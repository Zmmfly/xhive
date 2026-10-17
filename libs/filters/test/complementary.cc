#include "filter_test_support.h"

using namespace filters_test;
template <typename A> class ComplementaryTest : public ::testing::Test {};
TYPED_TEST_SUITE(ComplementaryTest, Formats);

TYPED_TEST(ComplementaryTest, LongIndependentRateMeasurementRecurrence) {
    using A = TypeParam;
    typename A::Complementary context;
    typename A::Scalar output{};
    ASSERT_EQ(A::complementary_init(&context, A::make(0.75), A::make(0.125), A::make(1)), LINEARS_OK);
    double reference = 1;
    std::mt19937 random(0xc17a);
    std::uniform_real_distribution<double> noise(-1, 1);
    for (int i = 0; i < 16000; ++i) {
        auto rate = A::make(2 * noise(random));
        auto measurement = A::make(i < 256 ? 5 : 5 * noise(random));
        reference = rounded<A>(0.75 * (reference + A::real(rate) * 0.125) + 0.25 * A::real(measurement));
        ASSERT_EQ(A::complementary_step(&context, rate, measurement, &output), LINEARS_OK);
        expect_close<A>(output, reference, 8, 32);
    }
    ASSERT_EQ(A::complementary_reset(&context, A::make(4)), LINEARS_OK);
    ASSERT_EQ(A::complementary_step(&context, A::make(8), A::make(1), &output), LINEARS_OK);
    EXPECT_EQ(A::real(output), 4);
}

TYPED_TEST(ComplementaryTest, ConstantRateTrackingAndComplementaryEndpoints) {
    using A = TypeParam;
    typename A::Complementary context;
    typename A::Scalar output{};
    ASSERT_EQ(A::complementary_init(&context, A::make(0.5), A::make(0.125), A::make(0)), LINEARS_OK);
    for (int i = 1; i <= 256; ++i) {
        ASSERT_EQ(A::complementary_step(&context, A::make(2), A::make(0.25 * i), &output), LINEARS_OK);
        EXPECT_EQ(A::real(output), 0.25 * i);
    }
    ASSERT_EQ(A::complementary_init(&context, A::make(0), A::make(4), A::make(0)), LINEARS_OK);
    ASSERT_EQ(A::complementary_step(&context, A::huge(), A::make(-7), &output), LINEARS_OK);
    EXPECT_EQ(A::real(output), -7); // ignored derivative must not overflow
    ASSERT_EQ(A::complementary_init(&context, A::make(1), A::make(0.25), A::make(3)), LINEARS_OK);
    ASSERT_EQ(A::complementary_step(&context, A::make(-8), A::make(100), &output), LINEARS_OK);
    EXPECT_EQ(A::real(output), 1);
}

TYPED_TEST(ComplementaryTest, InvalidParametersAliasAndOverflowAreAtomic) {
    using A = TypeParam;
    typename A::Complementary context;
    auto output = A::make(47);
    ASSERT_EQ(A::complementary_init(&context, A::make(0.5), A::make(4), A::make(2)), LINEARS_OK);
    auto before = context;
    EXPECT_EQ(A::complementary_init(&context, A::make(-0.25), A::make(1), A::make(0)), LINEARS_ERROR_DOMAIN);
    EXPECT_EQ(A::complementary_init(&context, A::make(1.25), A::make(1), A::make(0)), LINEARS_ERROR_DOMAIN);
    EXPECT_EQ(A::complementary_init(&context, A::make(0.5), A::make(0), A::make(0)), LINEARS_ERROR_DOMAIN);
    EXPECT_EQ(A::complementary_init(&context, A::make(0.5), A::make(-1), A::make(0)), LINEARS_ERROR_DOMAIN);
    EXPECT_EQ(A::complementary_step(&context, A::huge(), A::make(0), &output), LINEARS_ERROR_RANGE);
    EXPECT_EQ(A::complementary_step(&context, A::invalid(), A::make(0), &output), LINEARS_ERROR_RANGE);
    EXPECT_EQ(A::complementary_step(&context, A::make(0), A::invalid(), &output), LINEARS_ERROR_RANGE);
    EXPECT_EQ(A::complementary_step(&context, A::make(0), A::make(0), &context.value), LINEARS_ERROR_ALIAS);
    EXPECT_EQ(A::complementary_reset(&context, A::invalid()), LINEARS_ERROR_RANGE);
    expect_unchanged(context, before);
    EXPECT_EQ(A::real(output), 47);
}

TYPED_TEST(ComplementaryTest, SeparateContextsDoNotShareState) {
    using A = TypeParam;
    typename A::Complementary first, second;
    typename A::Scalar output{};
    ASSERT_EQ(A::complementary_init(&first, A::make(0.5), A::make(1), A::make(0)), LINEARS_OK);
    ASSERT_EQ(A::complementary_init(&second, A::make(1), A::make(0.5), A::make(10)), LINEARS_OK);
    ASSERT_EQ(A::complementary_step(&first, A::make(2), A::make(4), &output), LINEARS_OK);
    EXPECT_EQ(A::real(output), 3);
    ASSERT_EQ(A::complementary_step(&second, A::make(-2), A::make(0), &output), LINEARS_OK);
    EXPECT_EQ(A::real(output), 9);
    ASSERT_EQ(A::complementary_step(&first, A::make(0), A::make(5), &output), LINEARS_OK);
    EXPECT_EQ(A::real(output), 4);
}
