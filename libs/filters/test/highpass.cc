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
        auto input = A::make((A::fixed ? 0.0625 : 1.0) * (i < 128 ? (i == 0 ? 1 : 0) : (i < 512 ? 2 : samples(random))));
        reference = state_rounded<A>(0.75 * (reference + A::real(input) - previous_input));
        previous_input = A::real(input);
        ASSERT_EQ(A::highpass_step(&context, input, &output), LINEARS_OK);
        expect_close<A>(output, reference, 8, 48);
    }
    const double scale = A::fixed ? 0.0625 : 1.0;
    ASSERT_EQ(A::highpass_reset(&context, A::make(7 * scale), A::make(4 * scale)), LINEARS_OK);
    ASSERT_EQ(A::highpass_step(&context, A::make(7 * scale), &output), LINEARS_OK);
    EXPECT_EQ(A::real(output), 3 * scale);
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
            auto input = A::make((A::fixed ? 0.5 : 1.0) * std::sin(omega * i));
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
    auto positive = A::huge(), negative = positive, output = A::make(0.875);
    negative = static_cast<typename A::Scalar>(-negative);
    typename A::Highpass context;
    ASSERT_EQ(A::highpass_init(&context, A::make(0), negative, positive), LINEARS_OK);
    ASSERT_EQ(A::highpass_step(&context, positive, &output), LINEARS_OK);
    EXPECT_EQ(A::real(output), 0);
    ASSERT_EQ(A::highpass_init(&context, A::make(1), negative, A::make(0)), LINEARS_OK);
    auto before = context;
    output = A::make(0.875);
    EXPECT_EQ(A::highpass_step(&context, positive, &output), LINEARS_ERROR_RANGE);
    expect_unchanged(context, before);
    EXPECT_EQ(A::real(output), 0.875);
    if constexpr (!A::fixed) {
        EXPECT_EQ(A::highpass_step(&context, A::invalid(), &output), LINEARS_ERROR_RANGE);
        EXPECT_EQ(A::highpass_step(&context, std::numeric_limits<typename A::Scalar>::infinity(), &output), LINEARS_ERROR_RANGE);
    }
    EXPECT_EQ(A::highpass_step(&context, A::make(1), &context.previous_input), LINEARS_ERROR_ALIAS);
    if constexpr (!A::fixed) {
        EXPECT_EQ(A::highpass_reset(&context, A::make(0), A::invalid()), LINEARS_ERROR_RANGE);
        EXPECT_EQ(A::highpass_init(&context, A::make(2), A::make(0), A::make(0)), LINEARS_ERROR_DOMAIN);
    }
    EXPECT_EQ(A::highpass_init(&context, A::make(-1), A::make(0), A::make(0)), LINEARS_ERROR_DOMAIN);
    expect_unchanged(context, before);
}

TYPED_TEST(HighpassTest, IndependentContextsAndNulls) {
    using A = TypeParam;
    typename A::Highpass first, second;
    typename A::Scalar output{};
    const double scale = A::fixed ? 0.0625 : 1.0;
    ASSERT_EQ(A::highpass_init(&first, A::make(0.5), A::make(0), A::make(0)), LINEARS_OK);
    ASSERT_EQ(A::highpass_init(&second, A::make(1), A::make(2 * scale), A::make(0)), LINEARS_OK);
    ASSERT_EQ(A::highpass_step(&first, A::make(4 * scale), &output), LINEARS_OK);
    expect_close<A>(output, 2 * scale, 1);
    ASSERT_EQ(A::highpass_step(&second, A::make(4 * scale), &output), LINEARS_OK);
    expect_close<A>(output, 2 * scale, 1);
    ASSERT_EQ(A::highpass_step(&first, A::make(4 * scale), &output), LINEARS_OK);
    EXPECT_EQ(A::real(output), scale);
    EXPECT_EQ(A::highpass_step(nullptr, A::make(0), &output), LINEARS_ERROR_ARGUMENT);
    EXPECT_EQ(A::highpass_step(&first, A::make(0), nullptr), LINEARS_ERROR_ARGUMENT);
}

TEST(HighpassFixed, RejectsExactProductAboveOneBeforeWorkingRound) {
    filters_highpass_q31_t context;
    linears_q31_t output = 123;
    ASSERT_EQ(filters_highpass_q31_init(&context, INT32_C(1610612736),
        -INT32_C(1073741824), INT32_C(715827883)), LINEARS_OK);
    const auto before = context;
    // 0.75 * (1/3 rounded upward + 0.5 - (-0.5)) = 1 + 2^-33.
    EXPECT_EQ(filters_highpass_q31_step(&context, INT32_C(1073741824), &output), LINEARS_ERROR_RANGE);
    expect_unchanged(context, before);
    EXPECT_EQ(output, 123);
}
