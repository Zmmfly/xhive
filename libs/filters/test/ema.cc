#include "filter_test_support.h"

using namespace filters_test;
template <typename A> class EmaTest : public ::testing::Test {};
TYPED_TEST_SUITE(EmaTest, Formats);

TYPED_TEST(EmaTest, ExponentialRecurrenceAndExactLowpassEquivalence) {
    using A = TypeParam;
    typename A::Ema ema;
    typename A::Lowpass lowpass;
    typename A::Scalar result{}, other{};
    ASSERT_EQ(A::ema_init(&ema, A::make(0.25), A::make(-1)), LINEARS_OK);
    ASSERT_EQ(A::lowpass_init(&lowpass, A::make(0.25), A::make(-1)), LINEARS_OK);
    double reference = -1;
    std::mt19937 random(0xc9e0);
    std::uniform_real_distribution<double> samples(-5, 5);
    for (int i = 0; i < 12000; ++i) {
        auto input = A::make(i < 64 ? 2 : samples(random));
        reference = rounded<A>(0.75 * reference + 0.25 * A::real(input));
        ASSERT_EQ(A::ema_step(&ema, input, &result), LINEARS_OK);
        ASSERT_EQ(A::lowpass_step(&lowpass, input, &other), LINEARS_OK);
        EXPECT_EQ(A::real(result), A::real(other));
        expect_close<A>(result, reference, 6, 16);
    }
    ASSERT_EQ(A::ema_reset(&ema, A::make(8)), LINEARS_OK);
    ASSERT_EQ(A::ema_step(&ema, A::make(0), &result), LINEARS_OK);
    EXPECT_EQ(A::real(result), 6);
}

TYPED_TEST(EmaTest, AlphaEndpointsAndIndependentContexts) {
    using A = TypeParam;
    typename A::Ema hold, passthrough;
    typename A::Scalar result{};
    ASSERT_EQ(A::ema_init(&hold, A::make(0), A::make(3)), LINEARS_OK);
    ASSERT_EQ(A::ema_init(&passthrough, A::make(1), A::make(0)), LINEARS_OK);
    for (int i = 0; i < 30; ++i) {
        ASSERT_EQ(A::ema_step(&hold, A::make(i), &result), LINEARS_OK);
        EXPECT_EQ(A::real(result), 3);
        ASSERT_EQ(A::ema_step(&passthrough, A::make(-i), &result), LINEARS_OK);
        EXPECT_EQ(A::real(result), -i);
    }
}

TYPED_TEST(EmaTest, FailureAndAliasDoNotChangeTheEstimate) {
    using A = TypeParam;
    typename A::Ema context;
    auto output = A::make(47);
    ASSERT_EQ(A::ema_init(&context, A::make(0.5), A::make(2)), LINEARS_OK);
    const auto before = context;
    EXPECT_EQ(A::ema_init(&context, A::make(-0.5), A::make(0)), LINEARS_ERROR_DOMAIN);
    EXPECT_EQ(A::ema_reset(&context, A::invalid()), LINEARS_ERROR_RANGE);
    EXPECT_EQ(A::ema_step(&context, A::invalid(), &output), LINEARS_ERROR_RANGE);
    EXPECT_EQ(A::ema_step(&context, A::make(1), &context.value), LINEARS_ERROR_ALIAS);
    expect_unchanged(context, before);
    EXPECT_EQ(A::real(output), 47);
    EXPECT_EQ(A::ema_step(nullptr, A::make(0), &output), LINEARS_ERROR_ARGUMENT);
    EXPECT_EQ(A::ema_step(&context, A::make(0), nullptr), LINEARS_ERROR_ARGUMENT);
}
