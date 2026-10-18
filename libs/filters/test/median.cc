#include "filter_test_support.h"

using namespace filters_test;
template <typename A> class MedianTest : public ::testing::Test {};
TYPED_TEST_SUITE(MedianTest, Formats);

TYPED_TEST(MedianTest, OddEvenAndUnitWindowsAgainstSortedOracle) {
    using A = TypeParam;
    using S = typename A::Scalar;
    for (std::size_t window : {1u, 2u, 3u, 6u, 15u}) {
        Guards<A> state(window), workspace(window);
        typename A::Median context;
        S output{};
        ASSERT_EQ(A::median_init(&context, window, state.data(), window, workspace.data(), window), LINEARS_OK);
        std::vector<double> history(window, 0);
        std::mt19937 random(0xaf17);
        std::uniform_real_distribution<double> samples(-32, 32);
        for (int i = 0; i < 4000; ++i) {
            S input = A::make((A::fixed ? 0.015625 : 1.0) * (i < 40 ? (i % 4 == 0 ? -5 : 7) : samples(random)));
            history[static_cast<std::size_t>(i) % window] = A::real(input);
            auto sorted = history;
            std::sort(sorted.begin(), sorted.end());
            double expected = sorted[window / 2];
            if (window % 2 == 0) expected = 0.5 * sorted[window / 2 - 1] + 0.5 * expected;
            ASSERT_EQ(A::median_step(&context, input, &output), LINEARS_OK);
            if (window % 2 != 0) { EXPECT_EQ(A::real(output), expected); }
            else { expect_close<A>(output, rounded<A>(expected), 1, 4); }
        }
        state.check(); workspace.check();
    }
}

TYPED_TEST(MedianTest, RejectsIsolatedImpulseAndTracksStepAfterHalfWindow) {
    using A = TypeParam;
    std::array<typename A::Scalar, 5> state{}, workspace{};
    typename A::Median context;
    typename A::Scalar output{};
    ASSERT_EQ(A::median_init(&context, 5, state.data(), 5, workspace.data(), 5), LINEARS_OK);
    for (int i = 0; i < 12; ++i) {
        ASSERT_EQ(A::median_step(&context, A::make(i == 0 ? (A::fixed ? 0.875 : 100) : 0), &output), LINEARS_OK);
        EXPECT_EQ(A::real(output), 0);
    }
    ASSERT_EQ(A::median_reset(&context), LINEARS_OK);
    for (int i = 0; i < 10; ++i) {
        ASSERT_EQ(A::median_step(&context, A::make(A::fixed ? 0.1875 : 3), &output), LINEARS_OK);
        EXPECT_EQ(A::real(output), i < 2 ? 0 : (A::fixed ? 0.1875 : 3));
    }
}

TYPED_TEST(MedianTest, ExtremeEvenValuesDoNotOverflowTheirMean) {
    using A = TypeParam;
    std::array<typename A::Scalar, 2> state{}, workspace{};
    typename A::Median context;
    typename A::Scalar output{};
    ASSERT_EQ(A::median_init(&context, 2, state.data(), 2, workspace.data(), 2), LINEARS_OK);
    ASSERT_EQ(A::median_step(&context, A::huge(), &output), LINEARS_OK);
    ASSERT_EQ(A::median_step(&context, A::huge(), &output), LINEARS_OK);
    EXPECT_EQ(output, A::huge());
    auto negative = A::huge();
    negative = static_cast<typename A::Scalar>(-negative);
    ASSERT_EQ(A::median_step(&context, negative, &output), LINEARS_OK);
    EXPECT_EQ(A::real(output), 0);
}

TYPED_TEST(MedianTest, CapacityAliasAndInvalidInputAreAtomic) {
    using A = TypeParam;
    using S = typename A::Scalar;
    Guards<A> state(5), workspace(5);
    typename A::Median context{};
    auto output = A::make(0.875);
    EXPECT_EQ(A::median_init(&context, 0, state.data(), 5, workspace.data(), 5), LINEARS_ERROR_ARGUMENT);
    EXPECT_EQ(A::median_init(&context, SIZE_MAX, state.data(), SIZE_MAX, workspace.data(), SIZE_MAX), LINEARS_ERROR_SIZE);
    EXPECT_EQ(A::median_init(&context, 5, state.data(), 4, workspace.data(), 5), LINEARS_ERROR_WORKSPACE);
    EXPECT_EQ(A::median_init(&context, 5, state.data(), 5, workspace.data(), 4), LINEARS_ERROR_WORKSPACE);
    EXPECT_EQ(A::median_init(&context, 5, state.data(), 5, state.data() + 1, 5), LINEARS_ERROR_ALIAS);
    EXPECT_EQ(A::real(state.data()[0]), 0.875);
    ASSERT_EQ(A::median_init(&context, 5, state.data(), 5, workspace.data(), 5), LINEARS_OK);
    auto before = context;
    auto saved = state.values;
    if constexpr (!A::fixed) {
        EXPECT_EQ(A::median_step(&context, A::invalid(), &output), LINEARS_ERROR_RANGE);
        EXPECT_EQ(A::median_step(&context, std::numeric_limits<S>::infinity(), &output), LINEARS_ERROR_RANGE);
    }
    EXPECT_EQ(A::median_step(&context, A::make(1), state.data()), LINEARS_ERROR_ALIAS);
    EXPECT_EQ(A::median_step(&context, A::make(1), workspace.data()), LINEARS_ERROR_ALIAS);
    expect_unchanged(context, before);
    EXPECT_EQ(std::memcmp(saved.data(), state.values.data(), saved.size() * sizeof(S)), 0);
    EXPECT_EQ(A::real(output), 0.875);
    state.check(); workspace.check();
}

TYPED_TEST(MedianTest, IndependentInstances) {
    using A = TypeParam;
    std::array<typename A::Scalar, 6> state{}, workspace{};
    typename A::Median first, second;
    typename A::Scalar output{};
    ASSERT_EQ(A::median_init(&first, 3, state.data(), 3, workspace.data(), 3), LINEARS_OK);
    ASSERT_EQ(A::median_init(&second, 3, state.data() + 3, 3, workspace.data() + 3, 3), LINEARS_OK);
    for (int i = 0; i < 3; ++i) {
        ASSERT_EQ(A::median_step(&first, A::make((A::fixed ? 0.0625 : 1.0) * 7), &output), LINEARS_OK);
        ASSERT_EQ(A::median_step(&second, A::make((A::fixed ? 0.0625 : 1.0) * -5), &output), LINEARS_OK);
    }
    ASSERT_EQ(A::median_step(&first, A::make(0), &output), LINEARS_OK);
    EXPECT_EQ(A::real(output), (A::fixed ? 0.0625 : 1.0) * 7);
    ASSERT_EQ(A::median_step(&second, A::make(0), &output), LINEARS_OK);
    EXPECT_EQ(A::real(output), (A::fixed ? 0.0625 : 1.0) * -5);
}

TYPED_TEST(MedianTest, CentralPairRoundsOnlyOnce) {
    using A = TypeParam;
    std::array<typename A::Scalar, 2> state{}, workspace{};
    typename A::Median context;
    typename A::Scalar output{};
    ASSERT_EQ(A::median_init(&context, 2, state.data(), 2, workspace.data(), 2), LINEARS_OK);
    auto small = A::tiny();
    ASSERT_EQ(A::median_step(&context, small, &output), LINEARS_OK);
    ASSERT_EQ(A::median_step(&context, small, &output), LINEARS_OK);
    EXPECT_EQ(output, small);
    auto twice = static_cast<typename A::Scalar>(small + small);
    ASSERT_EQ(A::median_step(&context, twice, &output), LINEARS_OK);
    EXPECT_EQ(output, A::make(0.5 * A::real(small) + 0.5 * A::real(twice)));
    ASSERT_EQ(A::median_reset(&context), LINEARS_OK);
    ASSERT_EQ(A::median_step(&context, static_cast<typename A::Scalar>(-small), &output), LINEARS_OK);
    ASSERT_EQ(A::median_step(&context, static_cast<typename A::Scalar>(-twice), &output), LINEARS_OK);
    EXPECT_EQ(output, A::make(-0.5 * A::real(small) - 0.5 * A::real(twice)));
}
