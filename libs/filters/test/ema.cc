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
        auto input = A::make((A::fixed ? 0.125 : 1.0) * (i < 64 ? 2 : samples(random)));
        reference = state_rounded<A>(0.75 * reference + 0.25 * A::real(input));
        ASSERT_EQ(A::ema_step(&ema, input, &result), LINEARS_OK);
        ASSERT_EQ(A::lowpass_step(&lowpass, input, &other), LINEARS_OK);
        EXPECT_EQ(A::real(result), A::real(other));
        expect_close<A>(result, reference, 6, 16);
    }
    ASSERT_EQ(A::ema_reset(&ema, A::make(A::fixed ? 0.5 : 8)), LINEARS_OK);
    ASSERT_EQ(A::ema_step(&ema, A::make(0), &result), LINEARS_OK);
    EXPECT_EQ(A::real(result), A::fixed ? 0.375 : 6);
}

TYPED_TEST(EmaTest, AlphaEndpointsAndIndependentContexts) {
    using A = TypeParam;
    typename A::Ema hold, near_passthrough;
    typename A::Scalar result{};
    const double scale = A::fixed ? 0.015625 : 1.0;
    const auto weight = A::make(1);
    ASSERT_EQ(A::ema_init(&hold, A::make(0), A::make(3 * scale)), LINEARS_OK);
    ASSERT_EQ(A::ema_init(&near_passthrough, weight, A::make(0)), LINEARS_OK);
    double reference = 0;
    for (int i = 0; i < 30; ++i) {
        ASSERT_EQ(A::ema_step(&hold, A::make(i * scale), &result), LINEARS_OK);
        EXPECT_EQ(A::real(result), 3 * scale);
        const auto input = A::make(-i * scale);
        reference = state_rounded<A>((1 - A::real(weight)) * reference + A::real(weight) * A::real(input));
        ASSERT_EQ(A::ema_step(&near_passthrough, input, &result), LINEARS_OK);
        expect_close<A>(result, rounded<A>(reference), 1, 2);
    }
}

TYPED_TEST(EmaTest, FailureAndAliasDoNotChangeTheEstimate) {
    using A = TypeParam;
    typename A::Ema context;
    auto output = A::make(0.875);
    ASSERT_EQ(A::ema_init(&context, A::make(0.5), A::make(A::fixed ? 0.5 : 2)), LINEARS_OK);
    const auto before = context;
    EXPECT_EQ(A::ema_init(&context, A::make(-0.5), A::make(0)), LINEARS_ERROR_DOMAIN);
    if constexpr (!A::fixed) {
        EXPECT_EQ(A::ema_reset(&context, A::invalid()), LINEARS_ERROR_RANGE);
        EXPECT_EQ(A::ema_step(&context, A::invalid(), &output), LINEARS_ERROR_RANGE);
        EXPECT_EQ(A::ema_step(&context, std::numeric_limits<typename A::Scalar>::infinity(), &output), LINEARS_ERROR_RANGE);
    }
    EXPECT_EQ(A::ema_step(&context, A::make(1), reinterpret_cast<typename A::Scalar *>(&context.value)), LINEARS_ERROR_ALIAS);
    expect_unchanged(context, before);
    EXPECT_EQ(A::real(output), 0.875);
    EXPECT_EQ(A::ema_step(nullptr, A::make(0), &output), LINEARS_ERROR_ARGUMENT);
    EXPECT_EQ(A::ema_step(&context, A::make(0), nullptr), LINEARS_ERROR_ARGUMENT);
}
