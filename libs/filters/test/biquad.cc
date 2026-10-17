#include "filter_test_support.h"
#include <complex>

using namespace filters_test;
template <typename A> class BiquadTest : public ::testing::Test {};
TYPED_TEST_SUITE(BiquadTest, Formats);

TYPED_TEST(BiquadTest, ImpulseStepAndRandomDirectFormOneOracle) {
    using A = TypeParam;
    using S = typename A::Scalar;
    using C = typename A::Coefficients;
    std::array<C, 2> coefficients{{
        {A::make(0.5), A::make(0.25), A::make(0.125), A::make(-0.5), A::make(0.125)},
        {A::make(1.5), A::make(-0.25), A::make(0.125), A::make(-0.25), A::make(0.0625)}
    }};
    Guards<A> state(4), workspace(4);
    typename A::Biquad context;
    S output{};
    // Independent DF-I history, not the production transposed-DF-II states.
    std::array<std::array<double, 4>, 2> history{};
    ASSERT_EQ(A::biquad_init(&context, coefficients.data(), 2, state.data(), 4, workspace.data(), 4), LINEARS_OK);
    std::mt19937 random(0x51295);
    std::uniform_real_distribution<double> samples(-2, 2);
    for (int i = 0; i < 20000; ++i) {
        S input = A::make(i < 128 ? (i == 0 ? 1 : 0) : (i < 512 ? 1 : samples(random)));
        double reference = A::real(input);
        for (std::size_t j = 0; j < 2; ++j) {
            const auto &c = coefficients[j];
            auto &h = history[j];
            double next = A::real(c.b0) * reference + A::real(c.b1) * h[0] + A::real(c.b2) * h[1]
                        - A::real(c.a1) * h[2] - A::real(c.a2) * h[3];
            h[1] = h[0]; h[0] = reference;
            h[3] = h[2]; h[2] = rounded<A>(next);
            reference = h[2];
        }
        ASSERT_EQ(A::biquad_step(&context, input, &output), LINEARS_OK);
        // Both denominators have |a1|+|a2| <= 5/8, bounding feedback
        // amplification below 8/3; 16 units cover both section-state roundings.
        expect_close<A>(output, reference, 16, 64);
    }
    state.check(); workspace.check();
    ASSERT_EQ(A::biquad_reset(&context), LINEARS_OK);
    for (std::size_t i = 0; i < 4; ++i) EXPECT_EQ(A::real(state.data()[i]), 0);
    ASSERT_EQ(A::biquad_step(&context, A::make(1), &output), LINEARS_OK);
    EXPECT_EQ(A::real(output), 0.75);
}

TYPED_TEST(BiquadTest, AnalyticSosFrequencyResponse) {
    using A = TypeParam;
    typename A::Coefficients c{A::make(0.25), A::make(0.5), A::make(0.25), A::make(-0.5), A::make(0.25)};
    std::array<typename A::Scalar, 2> state{}, workspace{};
    typename A::Biquad context;
    typename A::Scalar output{};
    const double pi = std::acos(-1.0);
    for (int bin : {16, 128, 384}) {
        ASSERT_EQ(A::biquad_init(&context, &c, 1, state.data(), 2, workspace.data(), 2), LINEARS_OK);
        double omega = 2 * pi * bin / 1024;
        auto z = std::polar(1.0, -omega);
        auto expected = (A::real(c.b0) + A::real(c.b1) * z + A::real(c.b2) * z * z)
                      / (1.0 + A::real(c.a1) * z + A::real(c.a2) * z * z);
        std::complex<double> response(0, 0), source(0, 0);
        for (int i = 0; i < 2048; ++i) {
            auto input = A::make(std::sin(omega * i));
            ASSERT_EQ(A::biquad_step(&context, input, &output), LINEARS_OK);
            if (i >= 1024) {
                auto phase = std::polar(1.0, -omega * i);
                response += A::real(output) * phase;
                source += A::real(input) * phase;
            }
        }
        EXPECT_NEAR(std::abs(response / source - expected), 0.0, 24 * A::unit);
    }
}

TYPED_TEST(BiquadTest, CapacityAliasInvalidCoefficientAndLateStageOverflow) {
    using A = TypeParam;
    using S = typename A::Scalar;
    using C = typename A::Coefficients;
    std::array<C, 2> coefficients{{
        {A::make(1), A::make(0.25), A::make(0), A::make(0), A::make(0)},
        {A::make(4), A::make(0), A::make(0), A::make(0), A::make(0)}
    }};
    Guards<A> state(4), workspace(4);
    typename A::Biquad context{};
    auto output = A::make(47);
    EXPECT_EQ(A::biquad_init(&context, coefficients.data(), 2, state.data(), 3, workspace.data(), 4), LINEARS_ERROR_WORKSPACE);
    EXPECT_EQ(A::biquad_init(&context, coefficients.data(), 2, state.data(), 4, workspace.data(), 3), LINEARS_ERROR_WORKSPACE);
    EXPECT_EQ(A::biquad_init(&context, coefficients.data(), SIZE_MAX, state.data(), SIZE_MAX, workspace.data(), SIZE_MAX), LINEARS_ERROR_SIZE);
    EXPECT_EQ(A::biquad_init(&context, coefficients.data(), 2, state.data(), 4, state.data() + 1, 4), LINEARS_ERROR_ALIAS);
    auto invalid_coefficients = coefficients;
    invalid_coefficients[1].a2 = A::invalid();
    EXPECT_EQ(A::biquad_init(&context, invalid_coefficients.data(), 2, state.data(), 4, workspace.data(), 4), LINEARS_ERROR_RANGE);
    EXPECT_EQ(A::real(state.data()[0]), 47);
    ASSERT_EQ(A::biquad_init(&context, coefficients.data(), 2, state.data(), 4, workspace.data(), 4), LINEARS_OK);
    auto before = context;
    auto saved = state.values;
    EXPECT_EQ(A::biquad_step(&context, A::huge(), &output), LINEARS_ERROR_RANGE);
    expect_unchanged(context, before);
    EXPECT_EQ(std::memcmp(saved.data(), state.values.data(), saved.size() * sizeof(S)), 0);
    EXPECT_EQ(A::real(output), 47);
    EXPECT_EQ(A::biquad_step(&context, A::make(1), state.data()), LINEARS_ERROR_ALIAS);
    EXPECT_EQ(A::biquad_step(&context, A::make(1), workspace.data()), LINEARS_ERROR_ALIAS);
    EXPECT_EQ(A::biquad_step(&context, A::make(1), &coefficients[0].b0), LINEARS_ERROR_ALIAS);
    EXPECT_EQ(A::biquad_step(&context, A::invalid(), &output), LINEARS_ERROR_RANGE);
    ASSERT_EQ(A::biquad_step(&context, A::make(1), &output), LINEARS_OK);
    EXPECT_EQ(A::real(output), 4);
    state.check(); workspace.check();
}

TYPED_TEST(BiquadTest, CascadeEqualsSeparateInstances) {
    using A = TypeParam;
    using S = typename A::Scalar;
    using C = typename A::Coefficients;
    std::array<C, 2> coefficients{{
        {A::make(0.5), A::make(0.25), A::make(0), A::make(-0.25), A::make(0.125)},
        {A::make(1.25), A::make(0), A::make(-0.125), A::make(-0.5), A::make(0.125)}
    }};
    std::array<S, 4> cs{}, cw{}, ss{}, sw{};
    typename A::Biquad cascade, first, second;
    S whole{}, partial{}, split{};
    ASSERT_EQ(A::biquad_init(&cascade, coefficients.data(), 2, cs.data(), 4, cw.data(), 4), LINEARS_OK);
    ASSERT_EQ(A::biquad_init(&first, coefficients.data(), 1, ss.data(), 2, sw.data(), 2), LINEARS_OK);
    ASSERT_EQ(A::biquad_init(&second, coefficients.data() + 1, 1, ss.data() + 2, 2, sw.data() + 2, 2), LINEARS_OK);
    for (int i = 0; i < 1000; ++i) {
        auto input = A::make(std::sin(i * 0.173));
        ASSERT_EQ(A::biquad_step(&cascade, input, &whole), LINEARS_OK);
        ASSERT_EQ(A::biquad_step(&first, input, &partial), LINEARS_OK);
        ASSERT_EQ(A::biquad_step(&second, partial, &split), LINEARS_OK);
        EXPECT_EQ(A::real(whole), A::real(split));
        for (std::size_t j = 0; j < 4; ++j) EXPECT_EQ(A::real(cs[j]), A::real(ss[j]));
    }
}
