#include "filter_test_support.h"
#include <complex>

using namespace filters_test;
template <typename A> class FirTest : public ::testing::Test {};
TYPED_TEST_SUITE(FirTest, Formats);

TYPED_TEST(FirTest, ImpulseOrderSignedCoefficientsAndGainAboveOne) {
    using A = TypeParam;
    using S = typename A::Scalar;
    std::array<S, 5> coefficients{A::make(1.5), A::make(-0.75), A::make(0.25), A::make(2), A::make(-1)};
    Guards<A> state(5);
    typename A::Fir context;
    S output{};
    ASSERT_EQ(A::fir_init(&context, coefficients.data(), 5, state.data(), 5), LINEARS_OK);
    for (int repeat = 0; repeat < 2; ++repeat) {
        for (int i = 0; i < 15; ++i) {
            ASSERT_EQ(A::fir_step(&context, A::make(i == 0 ? 1 : 0), &output), LINEARS_OK);
            EXPECT_EQ(A::real(output), i < 5 ? A::real(coefficients[i]) : 0);
        }
        ASSERT_EQ(A::fir_reset(&context), LINEARS_OK);
    }
    state.check();
}

TYPED_TEST(FirTest, SeededRandomConvolutionAndLongStep) {
    using A = TypeParam;
    using S = typename A::Scalar;
    constexpr std::size_t count = 9;
    std::array<S, count> coefficients{}, state{};
    std::array<double, count> history{};
    std::mt19937 random(0x7ef31);
    std::uniform_real_distribution<double> gain(-2, 2), samples(-4, 4);
    for (auto &v : coefficients) v = A::make(gain(random));
    typename A::Fir context;
    S output{};
    ASSERT_EQ(A::fir_init(&context, coefficients.data(), count, state.data(), count), LINEARS_OK);
    for (int step = 0; step < 16000; ++step) {
        auto input = A::make(step < 512 ? 1 : samples(random));
        for (std::size_t i = count - 1; i != 0; --i) history[i] = history[i - 1];
        history[0] = A::real(input);
        double reference = 0, magnitude = 0;
        for (std::size_t i = 0; i < count; ++i) {
            const double term = history[i] * A::real(coefficients[i]);
            reference += term;
            magnitude += std::abs(term);
        }
        ASSERT_EQ(A::fir_step(&context, input, &output), LINEARS_OK);
        expect_close<A>(output, rounded<A>(reference), 2, 2 * count * magnitude);
    }
}

TYPED_TEST(FirTest, AnalyticFrequencyResponse) {
    using A = TypeParam;
    using S = typename A::Scalar;
    std::array<S, 4> coefficients{A::make(0.5), A::make(0.25), A::make(-0.5), A::make(0.125)}, state{};
    typename A::Fir context;
    S output{};
    const double pi = std::acos(-1.0);
    for (int bin : {4, 32, 96}) {
        ASSERT_EQ(A::fir_init(&context, coefficients.data(), 4, state.data(), 4), LINEARS_OK);
        const double omega = 2 * pi * bin / 256;
        std::complex<double> expected(0, 0), result(0, 0), source(0, 0);
        for (std::size_t i = 0; i < 4; ++i) expected += A::real(coefficients[i]) * std::polar(1.0, -omega * i);
        for (int i = 0; i < 512; ++i) {
            auto input = A::make(std::sin(omega * i));
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

TYPED_TEST(FirTest, CapacityAliasAndOverflowDoNotAdvanceHistory) {
    using A = TypeParam;
    using S = typename A::Scalar;
    std::array<S, 3> coefficients{A::make(4), A::make(-1), A::make(0.5)};
    Guards<A> state(3);
    typename A::Fir context{};
    auto output = A::make(47);
    const auto uninitialized = context;
    EXPECT_EQ(A::fir_init(&context, coefficients.data(), 3, state.data(), 2), LINEARS_ERROR_WORKSPACE);
    expect_unchanged(context, uninitialized);
    EXPECT_EQ(A::fir_init(&context, coefficients.data(), 0, state.data(), 3), LINEARS_ERROR_ARGUMENT);
    EXPECT_EQ(A::fir_init(&context, coefficients.data(), SIZE_MAX, state.data(), SIZE_MAX), LINEARS_ERROR_SIZE);
    EXPECT_EQ(A::fir_init(&context, coefficients.data(), 3, coefficients.data(), 3), LINEARS_ERROR_ALIAS);
    ASSERT_EQ(A::fir_init(&context, coefficients.data(), 3, state.data(), 3), LINEARS_OK);
    const auto before = context;
    const auto saved = state.values;
    EXPECT_EQ(A::fir_step(&context, A::huge(), &output), LINEARS_ERROR_RANGE);
    expect_unchanged(context, before);
    EXPECT_EQ(A::real(output), 47);
    EXPECT_EQ(std::memcmp(state.values.data(), saved.data(), saved.size() * sizeof(S)), 0);
    EXPECT_EQ(A::fir_step(&context, A::make(1), state.data()), LINEARS_ERROR_ALIAS);
    EXPECT_EQ(A::fir_step(&context, A::make(1), coefficients.data() + 1), LINEARS_ERROR_ALIAS);
    EXPECT_EQ(A::fir_step(&context, A::invalid(), &output), LINEARS_ERROR_RANGE);
    ASSERT_EQ(A::fir_step(&context, A::make(1), &output), LINEARS_OK);
    EXPECT_EQ(A::real(output), 4);
    state.check();
}

TYPED_TEST(FirTest, OneTapAndIndependentInstances) {
    using A = TypeParam;
    auto coefficient = A::make(-2), s1 = A::make(0), s2 = A::make(0);
    typename A::Fir first, second;
    typename A::Scalar out{};
    ASSERT_EQ(A::fir_init(&first, &coefficient, 1, &s1, 1), LINEARS_OK);
    ASSERT_EQ(A::fir_init(&second, &coefficient, 1, &s2, 1), LINEARS_OK);
    ASSERT_EQ(A::fir_step(&first, A::make(3), &out), LINEARS_OK);
    EXPECT_EQ(A::real(out), -6);
    ASSERT_EQ(A::fir_step(&second, A::make(-4), &out), LINEARS_OK);
    EXPECT_EQ(A::real(out), 8);
    EXPECT_EQ(A::real(s1), 3);
    EXPECT_EQ(A::real(s2), -4);
}
