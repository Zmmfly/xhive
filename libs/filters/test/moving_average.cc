#include "filter_test_support.h"

using namespace filters_test;
template <typename A> class AverageTest : public ::testing::Test {};
TYPED_TEST_SUITE(AverageTest, Formats);

TYPED_TEST(AverageTest, ZeroPaddedStepImpulseAndRandomFiniteWindowOracle) {
    using A = TypeParam;
    using S = typename A::Scalar;
    const double scale = A::fixed ? 0.0625 : 1.0;
    for (std::size_t window : {1u, 2u, 7u, 16u}) {
        Guards<A> state(window);
        typename A::Average context;
        S output{};
        ASSERT_EQ(A::average_init(&context, window, state.data(), window), LINEARS_OK);
        for (std::size_t i = 0; i < 2 * window; ++i) {
            ASSERT_EQ(A::average_step(&context, A::make(A::fixed ? 0.25 : 4), &output), LINEARS_OK);
            expect_close<A>(output, 4 * scale * double(std::min(i + 1, window)) / window, 2, window);
        }
        ASSERT_EQ(A::average_reset(&context), LINEARS_OK);
        std::vector<double> history(window, 0);
        std::size_t head = 0;
        std::mt19937 random(0xe9215);
        std::uniform_real_distribution<double> samples(-8, 8);
        for (int i = 0; i < 10000; ++i) {
            S input = A::make(scale * (i < 32 ? (i == 0 ? 1 : 0) : samples(random)));
            history[head] = A::real(input);
            head = (head + 1) % window;
            double sum = 0;
            for (double v : history) sum += v;
            ASSERT_EQ(A::average_step(&context, input, &output), LINEARS_OK);
            expect_close<A>(output, rounded<A>(sum / window), 2, 4 * window);
        }
        state.check();
    }
}

TYPED_TEST(AverageTest, AlternatingNyquistSignalAndNoRunningSumDrift) {
    using A = TypeParam;
    std::array<typename A::Scalar, 8> state{};
    typename A::Average context;
    typename A::Scalar output{};
    ASSERT_EQ(A::average_init(&context, 8, state.data(), 8), LINEARS_OK);
    for (int i = 0; i < 20000; ++i) {
        ASSERT_EQ(A::average_step(&context, A::make((A::fixed ? 0.0625 : 1.0) * ((i & 1) ? -3 : 3)), &output), LINEARS_OK);
        if (i >= 7) { EXPECT_EQ(A::real(output), 0); }
    }
    ASSERT_EQ(A::average_reset(&context), LINEARS_OK);
    for (int i = 0; i < 40; ++i) {
        ASSERT_EQ(A::average_step(&context, A::make(A::fixed ? 0.3125 : 5), &output), LINEARS_OK);
        if (i >= 7) { EXPECT_EQ(A::real(output), A::fixed ? 0.3125 : 5); }
    }
}

TYPED_TEST(AverageTest, LargeConstantAvoidsSumOverflow) {
    using A = TypeParam;
    std::array<typename A::Scalar, 4> state{};
    typename A::Average context;
    typename A::Scalar output{};
    ASSERT_EQ(A::average_init(&context, 4, state.data(), 4), LINEARS_OK);
    for (int i = 0; i < 16; ++i) ASSERT_EQ(A::average_step(&context, A::huge(), &output), LINEARS_OK);
    EXPECT_EQ(output, A::huge());
}

TYPED_TEST(AverageTest, CapacityAliasResetAndFailurePreserveState) {
    using A = TypeParam;
    using S = typename A::Scalar;
    Guards<A> state(4);
    typename A::Average context{};
    auto output = A::make(0.875);
    EXPECT_EQ(A::average_init(&context, 0, state.data(), 4), LINEARS_ERROR_ARGUMENT);
    EXPECT_EQ(A::average_init(&context, SIZE_MAX, state.data(), SIZE_MAX), LINEARS_ERROR_SIZE);
    EXPECT_EQ(A::average_init(&context, 4, state.data(), 3), LINEARS_ERROR_WORKSPACE);
    EXPECT_EQ(A::real(state.data()[0]), 0.875);
    ASSERT_EQ(A::average_init(&context, 4, state.data(), 4), LINEARS_OK);
    ASSERT_EQ(A::average_step(&context, A::make(A::fixed ? 0.25 : 4), &output), LINEARS_OK);
    auto before = context;
    auto saved = state.values;
    output = A::make(0.875);
    if constexpr (!A::fixed) {
        EXPECT_EQ(A::average_step(&context, A::invalid(), &output), LINEARS_ERROR_RANGE);
        EXPECT_EQ(A::average_step(&context, std::numeric_limits<S>::infinity(), &output), LINEARS_ERROR_RANGE);
    }
    EXPECT_EQ(A::average_step(&context, A::make(1), state.data() + 3), LINEARS_ERROR_ALIAS);
    expect_unchanged(context, before);
    EXPECT_EQ(std::memcmp(saved.data(), state.values.data(), saved.size() * sizeof(S)), 0);
    EXPECT_EQ(A::real(output), 0.875);
    ASSERT_EQ(A::average_reset(&context), LINEARS_OK);
    ASSERT_EQ(A::average_step(&context, A::make(A::fixed ? 0.25 : 4), &output), LINEARS_OK);
    EXPECT_EQ(A::real(output), A::fixed ? 0.0625 : 1);
    state.check();
}

TYPED_TEST(AverageTest, SeparateContextsKeepSeparateHistory) {
    using A = TypeParam;
    std::array<typename A::Scalar, 4> states{};
    typename A::Average first, second;
    typename A::Scalar output{};
    const double scale = A::fixed ? 0.0625 : 1.0;
    ASSERT_EQ(A::average_init(&first, 2, states.data(), 2), LINEARS_OK);
    ASSERT_EQ(A::average_init(&second, 2, states.data() + 2, 2), LINEARS_OK);
    ASSERT_EQ(A::average_step(&first, A::make(2 * scale), &output), LINEARS_OK);
    EXPECT_EQ(A::real(output), 1 * scale);
    ASSERT_EQ(A::average_step(&second, A::make(-8 * scale), &output), LINEARS_OK);
    EXPECT_EQ(A::real(output), -4 * scale);
    ASSERT_EQ(A::average_step(&first, A::make(2 * scale), &output), LINEARS_OK);
    EXPECT_EQ(A::real(output), 2 * scale);
}

TYPED_TEST(AverageTest, TinySamplesRoundAfterTheCompleteSumAndReset) {
    using A = TypeParam;
    std::array<typename A::Scalar, 1024> state{};
    typename A::Average context;
    typename A::Scalar output{};
    ASSERT_EQ(A::average_init(&context, state.size(), state.data(), state.size()), LINEARS_OK);
    for (int i = 0; i < 4096; ++i) {
        ASSERT_EQ(A::average_step(&context, A::tiny(), &output), LINEARS_OK);
        if (i >= 1023) { EXPECT_EQ(output, A::tiny()); }
    }
    ASSERT_EQ(A::average_reset(&context), LINEARS_OK);
    for (int i = 0; i < 2048; ++i) {
        ASSERT_EQ(A::average_step(&context, A::make(0), &output), LINEARS_OK);
        EXPECT_EQ(A::real(output), 0);
    }
}
