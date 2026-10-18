#include "filter_test_support.h"
#include <complex>
#include <thread>

using namespace filters_test;

namespace {

// Independent host-only integer convolution. The coefficient has 31 fraction
// bits; multiplying by an actual raw signal leaves exactly 31 bits to discard.
// Native __int128 is deliberately independent of the production two-limb MAC.
template <typename A, std::size_t N>
typename A::Scalar fixed_convolution(const std::array<typename A::Coefficient, N> &coefficients,
                                    const std::array<typename A::Scalar, N> &history) {
    using S = typename A::Scalar;
    __int128 sum = 0;
    for (std::size_t i = 0; i < N; ++i)
        sum += static_cast<__int128>(coefficients[i]) * history[i];
    const __int128 endpoint = static_cast<__int128>(1) << (31 + A::bits);
    if (sum < -endpoint || sum > endpoint)
        throw std::out_of_range("FIR oracle result is outside the signal domain");
    const __int128 magnitude = sum < 0 ? -sum : sum;
    const __int128 rounded_magnitude = (magnitude + (static_cast<__int128>(1) << 30)) >> 31;
    const __int128 raw = sum < 0 ? -rounded_magnitude : rounded_magnitude;
    return raw > std::numeric_limits<S>::max() ? std::numeric_limits<S>::max() : static_cast<S>(raw);
}

} // namespace

template <typename A> class FirTest : public ::testing::Test {};
TYPED_TEST_SUITE(FirTest, Formats);

TYPED_TEST(FirTest, ImpulseOrderSignedCoefficientsAndGainAboveOne) {
    using A = TypeParam;
    using S = typename A::Scalar;
    using C = typename A::Coefficient;
    std::array<C, 5> coefficients{A::coefficient(1.5), A::coefficient(-0.75), A::coefficient(0.25),
                                  A::coefficient(2), A::coefficient(-1)};
    Guards<A> state(5);
    typename A::Fir context;
    S output{};
    // The Q impulse is 1/4: even h[3] = 2 produces only 1/2.
    const auto amplitude = A::make(A::fixed ? 0.25 : 1);
    ASSERT_EQ(A::fir_init(&context, coefficients.data(), 5, state.data(), 5), LINEARS_OK);
    for (int repeat = 0; repeat < 2; ++repeat) {
        for (int i = 0; i < 15; ++i) {
            ASSERT_EQ(A::fir_step(&context, i == 0 ? amplitude : A::make(0), &output), LINEARS_OK);
            EXPECT_EQ(A::real(output), i < 5 ? A::real(amplitude) * A::real_coefficient(coefficients[i]) : 0);
        }
        ASSERT_EQ(A::fir_reset(&context), LINEARS_OK);
        EXPECT_EQ(context.head, 0u);
        for (std::size_t i = 0; i < 5; ++i) EXPECT_EQ(state.data()[i], A::make(0));
    }
    state.check();
}

TYPED_TEST(FirTest, SeededRandomConvolutionAndLongStep) {
    using A = TypeParam;
    using S = typename A::Scalar;
    constexpr std::size_t count = 9;
    std::array<typename A::Coefficient, count> coefficients{};
    std::array<S, count> state{}, history{};
    std::mt19937 random(0x7ef31);
    // For Q, sum |h| <= 18 and |x| <= 1/32, so |y| <= 9/16.
    const double amplitude = A::fixed ? 0.03125 : 4;
    std::uniform_real_distribution<double> gain(-2, 2), samples(-amplitude, amplitude);
    for (auto &v : coefficients) v = A::coefficient(gain(random));
    typename A::Fir context;
    S output{};
    ASSERT_EQ(A::fir_init(&context, coefficients.data(), count, state.data(), count), LINEARS_OK);
    for (int step = 0; step < 16000; ++step) {
        auto input = A::make(step < 512 ? (A::fixed ? amplitude : 1) : samples(random));
        for (std::size_t i = count - 1; i != 0; --i) history[i] = history[i - 1];
        history[0] = input;
        ASSERT_EQ(A::fir_step(&context, input, &output), LINEARS_OK) << "sample " << step;
        if constexpr (A::fixed) {
            EXPECT_EQ(output, fixed_convolution<A>(coefficients, history)) << "sample " << step;
        } else {
            double reference = 0, magnitude = 0;
            for (std::size_t i = 0; i < count; ++i) {
                const double term = A::real(history[i]) * A::real_coefficient(coefficients[i]);
                reference += term;
                magnitude += std::abs(term);
            }
            expect_close<A>(output, rounded<A>(reference), 2, 2 * count * magnitude);
        }
    }
}

TYPED_TEST(FirTest, AnalyticFrequencyResponse) {
    using A = TypeParam;
    using S = typename A::Scalar;
    std::array<typename A::Coefficient, 4> coefficients{A::coefficient(0.5), A::coefficient(0.25),
                                                       A::coefficient(-0.5), A::coefficient(0.125)};
    std::array<S, 4> state{};
    typename A::Fir context;
    S output{};
    const double pi = std::acos(-1.0);
    for (int bin : {4, 32, 96}) {
        ASSERT_EQ(A::fir_init(&context, coefficients.data(), 4, state.data(), 4), LINEARS_OK);
        const double omega = 2 * pi * bin / 256;
        std::complex<double> expected(0, 0), result(0, 0), source(0, 0);
        for (std::size_t i = 0; i < 4; ++i)
            expected += A::real_coefficient(coefficients[i]) * std::polar(1.0, -omega * i);
        for (int i = 0; i < 512; ++i) {
            // Q peak <= (1/2) * sum |h| = 11/16, independent of frequency.
            auto input = A::make((A::fixed ? 0.5 : 1) * std::sin(omega * i));
            ASSERT_EQ(A::fir_step(&context, input, &output), LINEARS_OK);
            if (i >= 256) {
                auto phase = std::polar(1.0, -omega * i);
                result += A::real(output) * phase;
                source += A::real(input) * phase;
            }
        }
        EXPECT_NEAR(std::abs(result / source - expected), 0.0, 8 * A::unit);
    }
}

TYPED_TEST(FirTest, CapacityAndPartialAliasesAreAtomic) {
    using A = TypeParam;
    using S = typename A::Scalar;
    using C = typename A::Coefficient;
    std::array<C, 3> coefficients{A::coefficient(0.5), A::coefficient(-1), A::coefficient(0.5)};
    const auto saved_coefficients = coefficients;
    Guards<A> state(3);
    typename A::Fir context{};
    const auto uninitialized = context;
    const auto untouched = state.values;
    EXPECT_EQ(A::fir_init(nullptr, coefficients.data(), 3, state.data(), 3), LINEARS_ERROR_ARGUMENT);
    EXPECT_EQ(A::fir_init(&context, nullptr, 3, state.data(), 3), LINEARS_ERROR_ARGUMENT);
    EXPECT_EQ(A::fir_init(&context, coefficients.data(), 3, nullptr, 3), LINEARS_ERROR_ARGUMENT);
    EXPECT_EQ(A::fir_init(&context, coefficients.data(), 3, state.data(), 2), LINEARS_ERROR_WORKSPACE);
    EXPECT_EQ(A::fir_init(&context, coefficients.data(), 0, state.data(), 3), LINEARS_ERROR_ARGUMENT);
    EXPECT_EQ(A::fir_init(&context, coefficients.data(), SIZE_MAX, state.data(), SIZE_MAX), LINEARS_ERROR_SIZE);
    // All fake typed pointers are naturally aligned and rejected before access.
    auto *coefficient_alias = reinterpret_cast<S *>(coefficients.data());
    auto *partial_alias = reinterpret_cast<S *>(reinterpret_cast<unsigned char *>(coefficients.data()) + sizeof(C));
    EXPECT_EQ(A::fir_init(&context, coefficients.data(), 3, coefficient_alias, 3), LINEARS_ERROR_ALIAS);
    EXPECT_EQ(A::fir_init(&context, coefficients.data(), 3, partial_alias, 3), LINEARS_ERROR_ALIAS);
    EXPECT_EQ(A::fir_init(&context, coefficients.data(), 3, reinterpret_cast<S *>(&context), 3), LINEARS_ERROR_ALIAS);
    expect_unchanged(context, uninitialized);
    EXPECT_EQ(state.values, untouched);
    EXPECT_EQ(coefficients, saved_coefficients);

    ASSERT_EQ(A::fir_init(&context, coefficients.data(), 3, state.data(), 3), LINEARS_OK);
    S output{};
    ASSERT_EQ(A::fir_step(&context, A::make(0.125), &output), LINEARS_OK);
    const auto before = context;
    const auto saved = state.values;
    output = A::make(0.875);
    EXPECT_EQ(A::fir_init(&context, coefficients.data(), 3, state.data(), 2), LINEARS_ERROR_WORKSPACE);
    EXPECT_EQ(A::fir_step(&context, A::make(0.25), state.data() + 1), LINEARS_ERROR_ALIAS);
    EXPECT_EQ(A::fir_step(&context, A::make(0.25), coefficient_alias), LINEARS_ERROR_ALIAS);
    EXPECT_EQ(A::fir_step(&context, A::make(0.25), partial_alias), LINEARS_ERROR_ALIAS);
    EXPECT_EQ(A::fir_step(&context, A::make(0.25), reinterpret_cast<S *>(&coefficients.back())), LINEARS_ERROR_ALIAS);
    EXPECT_EQ(A::fir_step(&context, A::make(0.25), reinterpret_cast<S *>(&context)), LINEARS_ERROR_ALIAS);
    EXPECT_EQ(A::fir_step(&context, A::make(0.25), nullptr), LINEARS_ERROR_ARGUMENT);
    EXPECT_EQ(A::fir_step(nullptr, A::make(0.25), &output), LINEARS_ERROR_ARGUMENT);
    EXPECT_EQ(A::fir_reset(nullptr), LINEARS_ERROR_ARGUMENT);
    expect_unchanged(context, before);
    EXPECT_EQ(state.values, saved);
    EXPECT_EQ(coefficients, saved_coefficients);
    EXPECT_EQ(output, A::make(0.875));
    state.check();
}

TYPED_TEST(FirTest, OutputOverflowDoesNotAdvanceHistory) {
    using A = TypeParam;
    using S = typename A::Scalar;
    std::array<typename A::Coefficient, 3> coefficients{A::coefficient(4), A::coefficient(-1), A::coefficient(0.5)};
    Guards<A> state(3), control_state(3);
    typename A::Fir context{}, control{};
    S output{}, expected{};
    ASSERT_EQ(A::fir_init(&context, coefficients.data(), 3, state.data(), 3), LINEARS_OK);
    ASSERT_EQ(A::fir_init(&control, coefficients.data(), 3, control_state.data(), 3), LINEARS_OK);
    ASSERT_EQ(A::fir_step(&context, A::make(0.125), &output), LINEARS_OK);
    ASSERT_EQ(A::fir_step(&control, A::make(0.125), &expected), LINEARS_OK);
    const auto before = context;
    const auto saved = state.values;
    output = A::make(0.875);
    for (int sign : {-1, 1}) {
        const auto excessive = A::fixed ? A::make(sign * 0.5) : static_cast<S>(sign * A::huge());
        EXPECT_EQ(A::fir_step(&context, excessive, &output), LINEARS_ERROR_RANGE);
        expect_unchanged(context, before);
        EXPECT_EQ(state.values, saved);
        EXPECT_EQ(output, A::make(0.875));
    }
    for (double input : {0.0625, -0.125, 0.0, 0.0, 0.0}) {
        ASSERT_EQ(A::fir_step(&context, A::make(input), &output), LINEARS_OK);
        ASSERT_EQ(A::fir_step(&control, A::make(input), &expected), LINEARS_OK);
        EXPECT_EQ(output, expected);
        EXPECT_EQ(state.values, control_state.values);
        EXPECT_EQ(context.head, control.head);
    }
    state.check(); control_state.check();
}

TYPED_TEST(FirTest, OneTapAndIndependentInstances) {
    using A = TypeParam;
    auto coefficient = A::coefficient(-2);
    auto s1 = A::make(0), s2 = A::make(0);
    typename A::Fir first, second;
    typename A::Scalar out{};
    ASSERT_EQ(A::fir_init(&first, &coefficient, 1, &s1, 1), LINEARS_OK);
    ASSERT_EQ(A::fir_init(&second, &coefficient, 1, &s2, 1), LINEARS_OK);
    const auto x1 = A::make(A::fixed ? 0.25 : 3), x2 = A::make(A::fixed ? -0.375 : -4);
    ASSERT_EQ(A::fir_step(&first, x1, &out), LINEARS_OK);
    EXPECT_EQ(A::real(out), -2 * A::real(x1));
    ASSERT_EQ(A::fir_step(&second, x2, &out), LINEARS_OK);
    EXPECT_EQ(A::real(out), -2 * A::real(x2));
    EXPECT_EQ(s1, x1);
    EXPECT_EQ(s2, x2);
    ASSERT_EQ(A::fir_reset(&first), LINEARS_OK);
    EXPECT_EQ(s1, A::make(0));
    EXPECT_EQ(s2, x2);
}

TYPED_TEST(FirTest, ParallelContextsAndReset) {
    using A = TypeParam;
    using S = typename A::Scalar;
    constexpr std::size_t count = 2048;
    std::array<typename A::Coefficient, 2> coefficients{A::coefficient(-2), A::coefficient(0.5)};
    std::array<std::array<S, 2>, 2> states{};
    std::array<typename A::Fir, 2> contexts{};
    std::array<std::array<S, count>, 2> inputs{}, outputs{};
    std::array<std::array<linears_status_t, count>, 2> statuses{};
    for (std::size_t j = 0; j < 2; ++j) {
        ASSERT_EQ(A::fir_init(&contexts[j], coefficients.data(), 2, states[j].data(), 2), LINEARS_OK);
        for (std::size_t i = 0; i < count; ++i)
            inputs[j][i] = A::make((A::fixed ? 0.25 : 4) * std::sin(i * (j == 0 ? 0.173 : -0.257)));
    }
    auto run = [&](std::size_t j) {
        for (std::size_t i = 0; i < count; ++i)
            statuses[j][i] = A::fir_step(&contexts[j], inputs[j][i], &outputs[j][i]);
    };
    std::thread first(run, 0), second(run, 1);
    first.join(); second.join();
    for (std::size_t j = 0; j < 2; ++j) {
        std::array<S, 2> history{};
        for (std::size_t i = 0; i < count; ++i) {
            ASSERT_EQ(statuses[j][i], LINEARS_OK);
            history[1] = history[0]; history[0] = inputs[j][i];
            if constexpr (A::fixed) {
                EXPECT_EQ(outputs[j][i], fixed_convolution<A>(coefficients, history));
            } else {
                expect_close<A>(outputs[j][i], -2 * A::real(history[0]) + 0.5 * A::real(history[1]), 2, 4);
            }
        }
    }
    const auto other = contexts[1];
    const auto other_state = states[1];
    ASSERT_EQ(A::fir_reset(&contexts[0]), LINEARS_OK);
    for (std::size_t i = 0; i < count; ++i) {
        S output{};
        ASSERT_EQ(A::fir_step(&contexts[0], inputs[0][i], &output), LINEARS_OK);
        EXPECT_EQ(output, outputs[0][i]);
    }
    expect_unchanged(contexts[1], other);
    EXPECT_EQ(states[1], other_state);
}

TYPED_TEST(FirTest, LargeSignedProductsCancelBeforeFinalQuantization) {
    using A = TypeParam;
    using S = typename A::Scalar;
    std::array<typename A::Coefficient, 2> coefficients{A::coefficient(4), A::coefficient(-4)};
    std::array<S, 2> state{};
    typename A::Fir context{};
    S output{};
    ASSERT_EQ(A::fir_init(&context, coefficients.data(), 2, state.data(), 2), LINEARS_OK);
    double previous = 0;
    for (int repeat = 0; repeat < 4; ++repeat) {
        for (int direction : {1, -1}) {
            for (int i = 1; i <= 6; ++i) {
                const double input = direction == 1 ? i * 0.125 : (6 - i) * 0.125;
                ASSERT_EQ(A::fir_step(&context, A::make(input), &output), LINEARS_OK);
                EXPECT_EQ(A::real(output), 4 * (input - previous));
                previous = input;
            }
        }
    }
    // At x=3/4 the individual products reach +/-3. Their F62 raw values
    // exceed int64_t, while the final difference is only +/-1/2.
}

using FirFixedFormats = ::testing::Types<Q15, Q31>;
template <typename A> class FirFixedTest : public ::testing::Test {};
TYPED_TEST_SUITE(FirFixedTest, FirFixedFormats);

TYPED_TEST(FirFixedTest, HalfwayAndSingleFinalRounding) {
    using A = TypeParam;
    using S = typename A::Scalar;
    std::array<typename A::Coefficient, 2> coefficients{A::coefficient(0.5), A::coefficient(0.5)};
    std::array<S, 2> state{};
    typename A::Fir context{};
    S output{};
    ASSERT_EQ(A::fir_init(&context, coefficients.data(), 2, state.data(), 2), LINEARS_OK);
    for (int sign : {-1, 1}) {
        ASSERT_EQ(A::fir_reset(&context), LINEARS_OK);
        ASSERT_EQ(A::fir_step(&context, static_cast<S>(sign), &output), LINEARS_OK);
        EXPECT_EQ(output, sign); // Half an output LSB rounds away from zero.
        ASSERT_EQ(A::fir_step(&context, static_cast<S>(sign), &output), LINEARS_OK);
        EXPECT_EQ(output, sign); // Two halves add BEFORE rounding, not to two LSBs.
        ASSERT_EQ(A::fir_reset(&context), LINEARS_OK);
        ASSERT_EQ(A::fir_step(&context, static_cast<S>(3 * sign), &output), LINEARS_OK);
        EXPECT_EQ(output, 2 * sign);
    }
}

TYPED_TEST(FirFixedTest, DirectExportRejectsTrueOverflowAndAvoidsDoubleRounding) {
    using A = TypeParam;
    using S = typename A::Scalar;
    using C = typename A::Coefficient;
    typename A::Fir context{};
    S state{}, output{};
    const C below_half = (INT64_C(1) << 30) - 1;
    ASSERT_EQ(A::fir_init(&context, &below_half, 1, &state, 1), LINEARS_OK);
    for (S input : {S(-1), S(1)}) {
        ASSERT_EQ(A::fir_step(&context, input, &output), LINEARS_OK);
        EXPECT_EQ(output, 0); // A preliminary F31 rounding would fail this for q15.
    }
    const C endpoint_gain = INT64_C(1) << (31 + A::bits);
    for (int sign : {-1, 1}) {
        const C coefficient = sign * (endpoint_gain + 1);
        ASSERT_EQ(A::fir_init(&context, &coefficient, 1, &state, 1), LINEARS_OK);
        const auto before = context;
        output = A::make(0.875);
        // The exact result is +/- (1 + 2^-(31+bits)); an F31 intermediate
        // would round it back onto the endpoint and hide the range violation.
        EXPECT_EQ(A::fir_step(&context, S(1), &output), LINEARS_ERROR_RANGE);
        expect_unchanged(context, before);
        EXPECT_EQ(state, 0);
        EXPECT_EQ(output, A::make(0.875));
    }
}

TYPED_TEST(FirFixedTest, EveryRawSignalIsValidAndMaximumIsNotOne) {
    using A = TypeParam;
    using S = typename A::Scalar;
    using C = typename A::Coefficient;
    static_assert(std::is_same_v<C, linears_fixed_t>);
    static_assert(std::is_same_v<S, std::conditional_t<A::bits == 15, int16_t, int32_t>>);
    const C identity = A::coefficient(1);
    typename A::Fir context{};
    S state{}, output{};
    ASSERT_EQ(A::fir_init(&context, &identity, 1, &state, 1), LINEARS_OK);
    // Exhaust all q15 patterns, and an evenly spaced signed q31 sample.
    for (std::int64_t i = 0; i < 65536; ++i) {
        const S input = static_cast<S>(static_cast<std::int64_t>(std::numeric_limits<S>::min())
                                      + i * (A::bits == 15 ? 1 : 65536));
        ASSERT_EQ(A::fir_step(&context, input, &output), LINEARS_OK);
        EXPECT_EQ(output, input);
        EXPECT_EQ(state, input);
    }
    const S maximum = std::numeric_limits<S>::max(), minimum = std::numeric_limits<S>::min();
    ASSERT_EQ(A::fir_step(&context, maximum, &output), LINEARS_OK);
    EXPECT_EQ(output, maximum);
    EXPECT_EQ(A::real(output), 1 - A::unit);
    const C negate = A::coefficient(-1);
    ASSERT_EQ(A::fir_init(&context, &negate, 1, &state, 1), LINEARS_OK);
    ASSERT_EQ(A::fir_step(&context, maximum, &output), LINEARS_OK);
    EXPECT_EQ(output, -maximum);
    ASSERT_EQ(A::fir_step(&context, minimum, &output), LINEARS_OK);
    EXPECT_EQ(output, maximum); // Exact +1, not an implicit unity interpretation of MAX.
    for (C coefficient : {std::numeric_limits<C>::min(), std::numeric_limits<C>::max()}) {
        ASSERT_EQ(A::fir_init(&context, &coefficient, 1, &state, 1), LINEARS_OK);
        ASSERT_EQ(A::fir_step(&context, S(0), &output), LINEARS_OK);
        EXPECT_EQ(output, 0); // Extreme wide coefficient bit patterns are valid.
        const auto before = context;
        output = A::make(0.875);
        EXPECT_EQ(A::fir_step(&context, S(1), &output), LINEARS_ERROR_RANGE);
        expect_unchanged(context, before);
        EXPECT_EQ(state, 0);
        EXPECT_EQ(output, A::make(0.875));
    }
}

using FirFloatFormats = ::testing::Types<F32, F64>;
template <typename A> class FirFloatTest : public ::testing::Test {};
TYPED_TEST_SUITE(FirFloatTest, FirFloatFormats);

TYPED_TEST(FirFloatTest, NonFiniteFailuresAndLargeFiniteSignals) {
    using A = TypeParam;
    using S = typename A::Scalar;
    std::array<S, 3> coefficients{A::coefficient(1), A::coefficient(0), A::coefficient(0)};
    Guards<A> state(3);
    typename A::Fir context{};
    S output = A::make(0.875);
    for (S invalid : {A::invalid(), std::numeric_limits<S>::infinity(), -std::numeric_limits<S>::infinity()}) {
        auto bad = coefficients;
        bad.back() = invalid;
        const auto before = context;
        const auto saved = state.values;
        EXPECT_EQ(A::fir_init(&context, bad.data(), 3, state.data(), 3), LINEARS_ERROR_RANGE);
        expect_unchanged(context, before);
        EXPECT_EQ(state.values, saved);
    }
    ASSERT_EQ(A::fir_init(&context, coefficients.data(), 3, state.data(), 3), LINEARS_OK);
    for (S input : {A::huge(), static_cast<S>(-A::huge()), A::tiny(), std::numeric_limits<S>::denorm_min()}) {
        ASSERT_EQ(A::fir_step(&context, input, &output), LINEARS_OK);
        EXPECT_EQ(output, input);
    }
    const auto before = context;
    const auto saved = state.values;
    output = A::make(0.875);
    for (S invalid : {A::invalid(), std::numeric_limits<S>::infinity(), -std::numeric_limits<S>::infinity()}) {
        EXPECT_EQ(A::fir_step(&context, invalid, &output), LINEARS_ERROR_RANGE);
        expect_unchanged(context, before);
        EXPECT_EQ(state.values, saved);
        EXPECT_EQ(output, A::make(0.875));
    }
    state.check();
}
