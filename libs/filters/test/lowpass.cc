#include "filter_test_support.h"
#include <complex>

using namespace filters_test;
template <typename A> class LowpassTest : public ::testing::Test {};
TYPED_TEST_SUITE(LowpassTest, Formats);

TYPED_TEST(LowpassTest, ImpulseStepAndLongRandomRecurrence) {
    using A = TypeParam;
    using S = typename A::Scalar;
    typename A::Lowpass context;
    S output{};
    const S alpha = A::make(0.125);
    ASSERT_EQ(A::lowpass_init(&context, alpha, A::make(0)), LINEARS_OK);
    double reference = 0;
    for (int i = 0; i < 128; ++i) {
        double x = i == 0 ? 1 : 0;
        reference = rounded<A>(0.875 * reference + 0.125 * x);
        ASSERT_EQ(A::lowpass_step(&context, A::make(x), &output), LINEARS_OK);
        expect_close<A>(output, reference, 8);
    }
    ASSERT_EQ(A::lowpass_reset(&context, A::make(0)), LINEARS_OK);
    reference = 0;
    std::mt19937 random(0x19a45u);
    std::uniform_real_distribution<double> samples(-12, 12);
    for (int i = 0; i < 20000; ++i) {
        S input = A::make(i < 256 ? 3 : samples(random));
        reference = rounded<A>(0.875 * reference + 0.125 * A::real(input));
        ASSERT_EQ(A::lowpass_step(&context, input, &output), LINEARS_OK);
        // Recurrence gain 7/8 amplifies local rounding error by at most 8.
        expect_close<A>(output, reference, 12, 32);
    }
}

TYPED_TEST(LowpassTest, AnalyticFrequencyResponse) {
    using A = TypeParam;
    typename A::Lowpass context;
    typename A::Scalar output{};
    constexpr int period = 1024;
    const double pi = std::acos(-1.0);
    for (int bin : {8, 128, 384}) {
        const double omega = 2 * pi * bin / period;
        ASSERT_EQ(A::lowpass_init(&context, A::make(0.25), A::make(0)), LINEARS_OK);
        std::complex<double> response(0, 0), input_response(0, 0);
        for (int i = 0; i < 2 * period; ++i) {
            auto input = A::make(std::sin(omega * i));
            ASSERT_EQ(A::lowpass_step(&context, input, &output), LINEARS_OK);
            if (i >= period) {
                auto basis = std::polar(1.0, -omega * i);
                response += A::real(output) * basis;
                input_response += A::real(input) * basis;
            }
        }
        const auto expected = 0.25 / (1.0 - 0.75 * std::polar(1.0, -omega));
        EXPECT_NEAR(std::abs(response / input_response - expected), 0.0, 16 * A::unit);
    }
}

TYPED_TEST(LowpassTest, EndpointsLargeOppositeSignsAndConstantInput) {
    using A = TypeParam;
    typename A::Lowpass context;
    typename A::Scalar output{}, positive = A::huge(), negative = positive;
    if constexpr (A::fixed) negative.mantissa = -negative.mantissa;
    else negative = -negative;
    ASSERT_EQ(A::lowpass_init(&context, A::make(0), A::make(7)), LINEARS_OK);
    ASSERT_EQ(A::lowpass_step(&context, positive, &output), LINEARS_OK);
    EXPECT_EQ(A::real(output), 7);
    ASSERT_EQ(A::lowpass_init(&context, A::make(1), A::make(7)), LINEARS_OK);
    ASSERT_EQ(A::lowpass_step(&context, positive, &output), LINEARS_OK);
    if constexpr (A::fixed) {
        EXPECT_EQ(output.mantissa, positive.mantissa);
        EXPECT_EQ(output.exponent, positive.exponent);
    } else EXPECT_EQ(output, positive);
    ASSERT_EQ(A::lowpass_init(&context, A::make(0.5), positive), LINEARS_OK);
    ASSERT_EQ(A::lowpass_step(&context, negative, &output), LINEARS_OK);
    EXPECT_EQ(A::real(output), 0);
    for (auto constant : {positive, A::tiny()}) {
        ASSERT_EQ(A::lowpass_init(&context, A::make(0.25), constant), LINEARS_OK);
        for (int i = 0; i < 32; ++i) {
            ASSERT_EQ(A::lowpass_step(&context, constant, &output), LINEARS_OK);
            if constexpr (A::fixed) {
                EXPECT_EQ(output.mantissa, constant.mantissa);
                EXPECT_EQ(output.exponent, constant.exponent);
            } else EXPECT_EQ(output, constant);
        }
    }
}

TYPED_TEST(LowpassTest, InvalidArgumentsAliasAndFailureAtomicity) {
    using A = TypeParam;
    typename A::Lowpass context{};
    auto output = A::make(47);
    ASSERT_EQ(A::lowpass_init(&context, A::make(0.25), A::make(2)), LINEARS_OK);
    const auto before = context;
    EXPECT_EQ(A::lowpass_init(&context, A::make(-0.125), A::make(0)), LINEARS_ERROR_DOMAIN);
    expect_unchanged(context, before);
    EXPECT_EQ(A::lowpass_init(&context, A::make(1.125), A::make(0)), LINEARS_ERROR_DOMAIN);
    expect_unchanged(context, before);
    EXPECT_EQ(A::lowpass_reset(&context, A::invalid()), LINEARS_ERROR_RANGE);
    expect_unchanged(context, before);
    EXPECT_EQ(A::lowpass_step(&context, A::invalid(), &output), LINEARS_ERROR_RANGE);
    EXPECT_EQ(A::real(output), 47);
    expect_unchanged(context, before);
    EXPECT_EQ(A::lowpass_step(&context, A::make(1), &context.value), LINEARS_ERROR_ALIAS);
    expect_unchanged(context, before);
    EXPECT_EQ(A::lowpass_step(nullptr, A::make(1), &output), LINEARS_ERROR_ARGUMENT);
    EXPECT_EQ(A::lowpass_step(&context, A::make(1), nullptr), LINEARS_ERROR_ARGUMENT);
    if constexpr (!A::fixed) {
        EXPECT_EQ(A::lowpass_step(&context, std::numeric_limits<typename A::Scalar>::infinity(), &output), LINEARS_ERROR_RANGE);
        expect_unchanged(context, before);
    }
}

TYPED_TEST(LowpassTest, ResetAndIndependentInstances) {
    using A = TypeParam;
    typename A::Lowpass first, second;
    typename A::Scalar one{}, two{};
    ASSERT_EQ(A::lowpass_init(&first, A::make(0.5), A::make(0)), LINEARS_OK);
    ASSERT_EQ(A::lowpass_init(&second, A::make(0.25), A::make(0)), LINEARS_OK);
    for (int i = 0; i < 32; ++i) {
        ASSERT_EQ(A::lowpass_step(&first, A::make(4), &one), LINEARS_OK);
        ASSERT_EQ(A::lowpass_step(&second, A::make(-8), &two), LINEARS_OK);
    }
    expect_close<A>(one, 4 * (1 - std::pow(0.5, 32)), 4);
    expect_close<A>(two, -8 * (1 - std::pow(0.75, 32)), 4);
    ASSERT_EQ(A::lowpass_reset(&first, A::make(10)), LINEARS_OK);
    ASSERT_EQ(A::lowpass_step(&first, A::make(2), &one), LINEARS_OK);
    EXPECT_EQ(A::real(one), 6);
}
