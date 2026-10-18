#include "filter_test_support.h"

using namespace filters_test;
template <typename A> class ComplementaryTest : public ::testing::Test {};
TYPED_TEST_SUITE(ComplementaryTest, Formats);

TYPED_TEST(ComplementaryTest, LongIndependentRateMeasurementRecurrence) {
    using A = TypeParam;
    typename A::Complementary context;
    typename A::Scalar output{};
    ASSERT_EQ(A::complementary_init(&context, A::make(0.75), A::coefficient(0.125), A::make(1)), LINEARS_OK);
    double reference = A::real(A::make(1));
    const double scale = A::fixed ? 0.0625 : 1.0;
    std::mt19937 random(0xc17a);
    std::uniform_real_distribution<double> noise(-1, 1);
    for (int i = 0; i < 16000; ++i) {
        auto rate = A::make(2 * scale * noise(random));
        auto measurement = A::make(scale * (i < 256 ? 5 : 5 * noise(random)));
        reference = state_rounded<A>(0.75 * (reference + A::real(rate) * 0.125) + 0.25 * A::real(measurement));
        ASSERT_EQ(A::complementary_step(&context, rate, measurement, &output), LINEARS_OK);
        expect_close<A>(output, reference, 8, 32);
    }
    ASSERT_EQ(A::complementary_reset(&context, A::make(4 * scale)), LINEARS_OK);
    ASSERT_EQ(A::complementary_step(&context, A::make(8 * scale), A::make(scale), &output), LINEARS_OK);
    EXPECT_EQ(A::real(output), 4 * scale);
}

TYPED_TEST(ComplementaryTest, ConstantRateTrackingAndComplementaryEndpoints) {
    using A = TypeParam;
    typename A::Complementary context;
    typename A::Scalar output{};
    const double scale = A::fixed ? 0.0078125 : 1.0;
    ASSERT_EQ(A::complementary_init(&context, A::make(0.5), A::coefficient(0.125), A::make(0)), LINEARS_OK);
    for (int i = 1; i <= 256; ++i) {
        ASSERT_EQ(A::complementary_step(&context, A::make(2 * scale), A::make(0.25 * i * scale), &output), LINEARS_OK);
        EXPECT_EQ(A::real(output), 0.25 * i * scale);
    }
    ASSERT_EQ(A::complementary_init(&context, A::make(0), A::coefficient(4), A::make(0)), LINEARS_OK);
    const auto measurement = A::make(A::fixed ? -0.4375 : -7);
    ASSERT_EQ(A::complementary_step(&context, A::huge(), measurement, &output), LINEARS_OK);
    EXPECT_EQ(output, measurement); // alpha zero must not evaluate an overflowing prediction
    const auto alpha = A::make(1), initial = A::make(A::fixed ? 0.1875 : 3);
    const auto rate = A::make(A::fixed ? -0.5 : -8), observation = A::make(A::fixed ? 0.75 : 100);
    ASSERT_EQ(A::complementary_init(&context, alpha, A::coefficient(0.25), initial), LINEARS_OK);
    ASSERT_EQ(A::complementary_step(&context, rate, observation, &output), LINEARS_OK);
    const double expected = A::real(alpha) * (A::real(initial) + 0.25 * A::real(rate))
                          + (1 - A::real(alpha)) * A::real(observation);
    expect_close<A>(output, expected, 1, 2);
}

TYPED_TEST(ComplementaryTest, InvalidParametersAliasAndOverflowAreAtomic) {
    using A = TypeParam;
    typename A::Complementary context;
    auto output = A::make(0.875);
    ASSERT_EQ(A::complementary_init(&context, A::make(0.5), A::coefficient(4), A::make(A::fixed ? 0.125 : 2)), LINEARS_OK);
    auto before = context;
    EXPECT_EQ(A::complementary_init(&context, A::make(-0.25), A::coefficient(1), A::make(0)), LINEARS_ERROR_DOMAIN);
    if constexpr (!A::fixed) {
        EXPECT_EQ(A::complementary_init(&context, A::make(1.25), A::coefficient(1), A::make(0)), LINEARS_ERROR_DOMAIN);
    }
    EXPECT_EQ(A::complementary_init(&context, A::make(0.5), A::coefficient(0), A::make(0)), LINEARS_ERROR_DOMAIN);
    EXPECT_EQ(A::complementary_init(&context, A::make(0.5), A::coefficient(-1), A::make(0)), LINEARS_ERROR_DOMAIN);
    EXPECT_EQ(A::complementary_step(&context, A::huge(), A::make(0), &output), LINEARS_ERROR_RANGE);
    if constexpr (!A::fixed) {
        EXPECT_EQ(A::complementary_step(&context, A::invalid(), A::make(0), &output), LINEARS_ERROR_RANGE);
        EXPECT_EQ(A::complementary_step(&context, A::make(0), A::invalid(), &output), LINEARS_ERROR_RANGE);
        EXPECT_EQ(A::complementary_step(&context, std::numeric_limits<typename A::Scalar>::infinity(), A::make(0), &output), LINEARS_ERROR_RANGE);
        EXPECT_EQ(A::complementary_reset(&context, A::invalid()), LINEARS_ERROR_RANGE);
    }
    EXPECT_EQ(A::complementary_step(&context, A::make(0), A::make(0), reinterpret_cast<typename A::Scalar *>(&context.value)), LINEARS_ERROR_ALIAS);
    expect_unchanged(context, before);
    EXPECT_EQ(A::real(output), 0.875);
}

TYPED_TEST(ComplementaryTest, SeparateContextsDoNotShareState) {
    using A = TypeParam;
    typename A::Complementary first, second;
    typename A::Scalar output{};
    const double scale = A::fixed ? 0.0625 : 1.0;
    ASSERT_EQ(A::complementary_init(&first, A::make(0.5), A::coefficient(1), A::make(0)), LINEARS_OK);
    ASSERT_EQ(A::complementary_init(&second, A::make(1), A::coefficient(0.5), A::make(10 * scale)), LINEARS_OK);
    ASSERT_EQ(A::complementary_step(&first, A::make(2 * scale), A::make(4 * scale), &output), LINEARS_OK);
    EXPECT_EQ(A::real(output), 3 * scale);
    ASSERT_EQ(A::complementary_step(&second, A::make(-2 * scale), A::make(0), &output), LINEARS_OK);
    expect_close<A>(output, 9 * scale * A::real(A::make(1)), 1, 2);
    ASSERT_EQ(A::complementary_step(&first, A::make(0), A::make(5 * scale), &output), LINEARS_OK);
    EXPECT_EQ(A::real(output), 4 * scale);
}
