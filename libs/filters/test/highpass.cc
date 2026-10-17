#include "filter_test_support.h"
#include <complex>

using namespace filters_test;
template <typename A> class HighpassTest : public ::testing::Test {};
TYPED_TEST_SUITE(HighpassTest, Formats);

TYPED_TEST(HighpassTest, ImpulseStepAndLongRandomRecurrence) {
    using A = TypeParam;
    typename A::Highpass context;
    typename A::Scalar output{};
    ASSERT_EQ(A::highpass_init(&context, A::make(0.75), A::make(0), A::make(0)), LINEARS_OK);
    double previous_input = 0, reference = 0;
    std::mt19937 random(0x91025);
    std::uniform_real_distribution<double> samples(-6, 6);
    for (int i = 0; i < 16000; ++i) {
        auto input = A::make(i < 128 ? (i == 0 ? 1 : 0) : (i < 512 ? 2 : samples(random)));
        reference = rounded<A>(0.75 * (reference + A::real(input) - previous_input));
        previous_input = A::real(input);
        ASSERT_EQ(A::highpass_step(&context, input, &output), LINEARS_OK);
        expect_close<A>(output, reference, 8, 48);
    }
    ASSERT_EQ(A::highpass_reset(&context, A::make(7), A::make(4)), LINEARS_OK);
    ASSERT_EQ(A::highpass_step(&context, A::make(7), &output), LINEARS_OK);
    EXPECT_EQ(A::real(output), 3);
}

TYPED_TEST(HighpassTest, AnalyticFrequencyResponse) {
    using A = TypeParam;
    typename A::Highpass context;
    typename A::Scalar output{};
    const double pi = std::acos(-1.0);
    for (int bin : {8, 128, 384}) {
        double omega = 2 * pi * bin / 1024;
        auto z = std::polar(1.0, -omega);
        auto expected = 0.75 * (1.0 - z) / (1.0 - 0.75 * z);
        ASSERT_EQ(A::highpass_init(&context, A::make(0.75), A::make(0), A::make(0)), LINEARS_OK);
        std::complex<double> response(0, 0), source(0, 0);
        for (int i = 0; i < 2048; ++i) {
            auto input = A::make(std::sin(omega * i));
            ASSERT_EQ(A::highpass_step(&context, input, &output), LINEARS_OK);
            if (i >= 1024) {
                auto phase = std::polar(1.0, -omega * i);
                response += A::real(output) * phase;
                source += A::real(input) * phase;
            }
        }
        EXPECT_NEAR(std::abs(response / source - expected), 0.0, 20 * A::unit);
    }
}

TYPED_TEST(HighpassTest, EndpointsInvalidInputsAndTransactionalOverflow) {
    using A = TypeParam;
    auto positive = A::huge(), negative = positive, output = A::make(47);
    if constexpr (A::fixed) negative.mantissa = -negative.mantissa;
    else negative = -negative;
    typename A::Highpass context;
    ASSERT_EQ(A::highpass_init(&context, A::make(0), negative, positive), LINEARS_OK);
    ASSERT_EQ(A::highpass_step(&context, positive, &output), LINEARS_OK);
    EXPECT_EQ(A::real(output), 0);
    ASSERT_EQ(A::highpass_init(&context, A::make(1), negative, A::make(0)), LINEARS_OK);
    auto before = context;
    output = A::make(47);
    EXPECT_EQ(A::highpass_step(&context, positive, &output), LINEARS_ERROR_RANGE);
    expect_unchanged(context, before);
    EXPECT_EQ(A::real(output), 47);
    EXPECT_EQ(A::highpass_step(&context, A::invalid(), &output), LINEARS_ERROR_RANGE);
    EXPECT_EQ(A::highpass_step(&context, A::make(1), &context.previous_input), LINEARS_ERROR_ALIAS);
    EXPECT_EQ(A::highpass_reset(&context, A::make(0), A::invalid()), LINEARS_ERROR_RANGE);
    EXPECT_EQ(A::highpass_init(&context, A::make(-1), A::make(0), A::make(0)), LINEARS_ERROR_DOMAIN);
    EXPECT_EQ(A::highpass_init(&context, A::make(2), A::make(0), A::make(0)), LINEARS_ERROR_DOMAIN);
    expect_unchanged(context, before);
}

TYPED_TEST(HighpassTest, IndependentContextsAndNulls) {
    using A = TypeParam;
    typename A::Highpass first, second;
    typename A::Scalar output{};
    ASSERT_EQ(A::highpass_init(&first, A::make(0.5), A::make(0), A::make(0)), LINEARS_OK);
    ASSERT_EQ(A::highpass_init(&second, A::make(1), A::make(2), A::make(0)), LINEARS_OK);
    ASSERT_EQ(A::highpass_step(&first, A::make(4), &output), LINEARS_OK);
    EXPECT_EQ(A::real(output), 2);
    ASSERT_EQ(A::highpass_step(&second, A::make(4), &output), LINEARS_OK);
    EXPECT_EQ(A::real(output), 2);
    ASSERT_EQ(A::highpass_step(&first, A::make(4), &output), LINEARS_OK);
    EXPECT_EQ(A::real(output), 1);
    EXPECT_EQ(A::highpass_step(nullptr, A::make(0), &output), LINEARS_ERROR_ARGUMENT);
    EXPECT_EQ(A::highpass_step(&first, A::make(0), nullptr), LINEARS_ERROR_ARGUMENT);
}
