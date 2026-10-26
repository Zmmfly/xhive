#include "filter_test_support.h"
#include <complex>

using namespace filters_test;
template <typename A> class EmaTest : public ::testing::Test {};
TYPED_TEST_SUITE(EmaTest, Formats);

TYPED_TEST(EmaTest, ImpulseStepAndLongRandomRecurrence) {
    using A = TypeParam;
    using S = typename A::Scalar;
    typename A::Ema context;
    S output{};
    const S alpha = A::make(0.125);
    ASSERT_EQ(A::ema_init(&context, alpha, A::make(0)), LINEARS_OK);
    double reference = 0;
    for (int i = 0; i < 128; ++i) {
        const auto input = A::make(i == 0 ? 1 : 0);
        reference = state_rounded<A>(0.875 * reference + 0.125 * A::real(input));
        ASSERT_EQ(A::ema_step(&context, input, &output), LINEARS_OK);
        expect_close<A>(output, reference, 8);
    }
    ASSERT_EQ(A::ema_reset(&context, A::make(0)), LINEARS_OK);
    reference = 0;
    std::mt19937 random(0x19a45u);
    std::uniform_real_distribution<double> samples(-12, 12);
    for (int i = 0; i < 20000; ++i) {
        S input = A::make((A::fixed ? 0.0625 : 1.0) * (i < 256 ? 3 : samples(random)));
        reference = state_rounded<A>(0.875 * reference + 0.125 * A::real(input));
        ASSERT_EQ(A::ema_step(&context, input, &output), LINEARS_OK);
        // Recurrence gain 7/8 amplifies local rounding error by at most 8.
        expect_close<A>(output, reference, 12, 32);
    }
}

TYPED_TEST(EmaTest, AnalyticFrequencyResponse) {
    using A = TypeParam;
    typename A::Ema context;
    typename A::Scalar output{};
    constexpr int period = 1024;
    const double pi = std::acos(-1.0);
    for (int bin : {8, 128, 384}) {
        const double omega = 2 * pi * bin / period;
        ASSERT_EQ(A::ema_init(&context, A::make(0.25), A::make(0)), LINEARS_OK);
        std::complex<double> response(0, 0), input_response(0, 0);
        for (int i = 0; i < 2 * period; ++i) {
            auto input = A::make(std::sin(omega * i));
            ASSERT_EQ(A::ema_step(&context, input, &output), LINEARS_OK);
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

TYPED_TEST(EmaTest, EndpointsLargeOppositeSignsAndConstantInput) {
    using A = TypeParam;
    typename A::Ema context;
    typename A::Scalar output{}, positive = A::huge();
    const auto negative = static_cast<typename A::Scalar>(-positive);
    const auto held = A::make(A::fixed ? 0.4375 : 7);
    ASSERT_EQ(A::ema_init(&context, A::make(0), held), LINEARS_OK);
    ASSERT_EQ(A::ema_step(&context, positive, &output), LINEARS_OK);
    EXPECT_EQ(output, held);
    if constexpr (A::fixed) {
        ASSERT_EQ(A::ema_init(&context, A::make(1), A::make(0)), LINEARS_OK);
        ASSERT_EQ(A::ema_step(&context, A::make(-1), &output), LINEARS_OK);
        EXPECT_EQ(output, std::numeric_limits<typename A::Scalar>::min() + 1);
        EXPECT_NE(output, A::make(-1)); // raw MAX alpha is not exact one
    } else {
        ASSERT_EQ(A::ema_init(&context, A::make(1), A::make(7)), LINEARS_OK);
        ASSERT_EQ(A::ema_step(&context, positive, &output), LINEARS_OK);
        EXPECT_EQ(output, positive);
    }
    ASSERT_EQ(A::ema_init(&context, A::make(0.5), positive), LINEARS_OK);
    ASSERT_EQ(A::ema_step(&context, negative, &output), LINEARS_OK);
    EXPECT_EQ(A::real(output), 0);
    for (auto constant : {positive, A::tiny()}) {
        ASSERT_EQ(A::ema_init(&context, A::make(0.25), constant), LINEARS_OK);
        for (int i = 0; i < 32; ++i) {
            ASSERT_EQ(A::ema_step(&context, constant, &output), LINEARS_OK);
            EXPECT_EQ(output, constant);
        }
    }
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

TYPED_TEST(EmaTest, InvalidArgumentsAliasAndFailureAtomicity) {
    using A = TypeParam;
    typename A::Ema context{};
    auto output = A::make(0.875);
    ASSERT_EQ(A::ema_init(&context, A::make(0.25), A::make(A::fixed ? 0.5 : 2)), LINEARS_OK);
    const auto before = context;
    EXPECT_EQ(A::ema_init(&context, A::make(-0.125), A::make(0)), LINEARS_ERROR_DOMAIN);
    expect_unchanged(context, before);
    if constexpr (!A::fixed) {
        EXPECT_EQ(A::ema_init(&context, A::make(1.125), A::make(0)), LINEARS_ERROR_DOMAIN);
        EXPECT_EQ(A::ema_reset(&context, A::invalid()), LINEARS_ERROR_RANGE);
        EXPECT_EQ(A::ema_step(&context, A::invalid(), &output), LINEARS_ERROR_RANGE);
        EXPECT_EQ(A::ema_step(&context, std::numeric_limits<typename A::Scalar>::infinity(), &output), LINEARS_ERROR_RANGE);
    }
    EXPECT_EQ(A::ema_step(&context, A::make(1), reinterpret_cast<typename A::Scalar *>(&context.value)), LINEARS_ERROR_ALIAS);
    expect_unchanged(context, before);
    EXPECT_EQ(A::real(output), 0.875);
    EXPECT_EQ(A::ema_step(nullptr, A::make(1), &output), LINEARS_ERROR_ARGUMENT);
    EXPECT_EQ(A::ema_step(&context, A::make(1), nullptr), LINEARS_ERROR_ARGUMENT);
}

TYPED_TEST(EmaTest, ResetAndIndependentInstances) {
    using A = TypeParam;
    typename A::Ema first, second;
    typename A::Scalar one{}, two{};
    const double scale = A::fixed ? 0.0625 : 1.0;
    ASSERT_EQ(A::ema_init(&first, A::make(0.5), A::make(0)), LINEARS_OK);
    ASSERT_EQ(A::ema_init(&second, A::make(0.25), A::make(0)), LINEARS_OK);
    for (int i = 0; i < 32; ++i) {
        ASSERT_EQ(A::ema_step(&first, A::make(4 * scale), &one), LINEARS_OK);
        ASSERT_EQ(A::ema_step(&second, A::make(-8 * scale), &two), LINEARS_OK);
    }
    expect_close<A>(one, 4 * scale * (1 - std::pow(0.5, 32)), 4);
    expect_close<A>(two, -8 * scale * (1 - std::pow(0.75, 32)), 4);
    ASSERT_EQ(A::ema_reset(&first, A::make(10 * scale)), LINEARS_OK);
    ASSERT_EQ(A::ema_step(&first, A::make(2 * scale), &one), LINEARS_OK);
    EXPECT_EQ(A::real(one), 6 * scale);
}

TYPED_TEST(EmaTest, SmallWeightRetainsSubSignalLsbHistory) {
    using A = TypeParam;
    typename A::Ema context;
    typename A::Scalar output{};
    const auto alpha = A::make(A::fixed ? 1.0 / 32768.0 : 1e-5);
    ASSERT_EQ(A::ema_init(&context, alpha, A::make(0)), LINEARS_OK);
    double reference = 0;
    for (int i = 0; i < 10000; ++i) {
        reference = state_rounded<A>((1 - A::real(alpha)) * reference + A::real(alpha) * 0.25);
        ASSERT_EQ(A::ema_step(&context, A::make(0.25), &output), LINEARS_OK);
    }
    if constexpr (!A::fixed) {
        // A closed form avoids drift from repeatedly rounding 1-alpha in the oracle.
        reference = static_cast<double>(-0.25L * std::expm1(10000.0L *
            std::log1p(-static_cast<long double>(A::real(alpha)))));
    }
    expect_close<A>(output, rounded<A>(reference), 2, 8);
    EXPECT_GT(A::real(output), 0.02);
    EXPECT_NEAR(A::state_real(context.value), reference, 8 * A::work_unit);
}

TYPED_TEST(EmaTest, WorkingHistoryNarrowsTransactionally) {
    using A = TypeParam;
    typename A::Ema context;
    auto output = A::make(0.875);
    ASSERT_EQ(A::ema_init(&context, A::make(0.5), A::make(0)), LINEARS_OK);
    if constexpr (A::fixed) {
        ASSERT_EQ(filters_ema_work_step(&context.value, INT32_C(1073741824),
                                       4 * LINEARS_FIXED_ONE), LINEARS_OK);
        EXPECT_EQ(context.value, 2 * LINEARS_FIXED_ONE);
        const auto before = context;
        EXPECT_EQ(A::ema_step(&context, A::make(0.25), &output), LINEARS_ERROR_RANGE);
        expect_unchanged(context, before);
        EXPECT_EQ(A::real(output), 0.875);
        ASSERT_EQ(filters_ema_work_step(&context.value, INT32_C(1073741824), 0), LINEARS_OK);
        ASSERT_EQ(A::ema_step(&context, A::make(0), &output), LINEARS_OK);
        EXPECT_EQ(A::real(output), 0.5);
    } else {
        ASSERT_EQ(A::ema_reset(&context, A::make(4)), LINEARS_OK);
        ASSERT_EQ(A::ema_step(&context, A::make(0.25), &output), LINEARS_OK);
        EXPECT_EQ(A::real(output), 2.125);
    }
}

TEST(EmaWorking, WideConvexStateAndActualRawAlpha) {
    linears_fixed_t state = 8 * LINEARS_FIXED_ONE;
    ASSERT_EQ(filters_ema_work_step(&state, INT32_C(1073741824), 2 * LINEARS_FIXED_ONE), LINEARS_OK);
    EXPECT_EQ(state, 5 * LINEARS_FIXED_ONE);
    for (int i = 0; i < 8; ++i)
        ASSERT_EQ(filters_ema_work_step(&state, INT32_C(1073741824), 0), LINEARS_OK);
    EXPECT_EQ(state, 5 * LINEARS_FIXED_ONE / 256);
    state = INT64_MIN;
    ASSERT_EQ(filters_ema_work_step(&state, 0, INT64_MAX), LINEARS_OK);
    EXPECT_EQ(state, INT64_MIN);
    ASSERT_EQ(filters_ema_work_step(&state, INT32_C(1073741824), INT64_MAX), LINEARS_OK);
    EXPECT_EQ(state, -1); // midpoint is exactly -0.5 F31 raw units: ties away
    state = INT64_MIN;
    ASSERT_EQ(filters_ema_work_step(&state, INT32_MAX, INT64_MAX), LINEARS_OK);
    EXPECT_EQ(state, INT64_MAX - INT64_C(8589934592));
    const auto before = state;
    EXPECT_EQ(filters_ema_work_step(&state, -1, 0), LINEARS_ERROR_DOMAIN);
    EXPECT_EQ(state, before);
    EXPECT_EQ(filters_ema_work_step(nullptr, 0, 0), LINEARS_ERROR_ARGUMENT);
}
