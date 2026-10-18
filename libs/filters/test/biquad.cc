#include "filter_test_support.h"
#include <complex>
#include <thread>

using namespace filters_test;

namespace {

template <typename A> long double coefficient_value(typename A::Coefficient value) {
    if constexpr (A::fixed) return std::ldexp(static_cast<long double>(value), -31);
    else return static_cast<long double>(value);
}

// Independent DF-I x/y histories, not production transposed-DF-II states.
// Feedback here is ideal long-double arithmetic. Tests account explicitly for
// production F31/native state roundings; only cascade interfaces are narrowed.
template <typename A>
long double direct_form_one(const typename A::Coefficients &c, std::array<long double, 4> &h,
                            long double input) {
    const long double next = coefficient_value<A>(c.b0) * input
                           + coefficient_value<A>(c.b1) * h[0]
                           + coefficient_value<A>(c.b2) * h[1]
                           - coefficient_value<A>(c.a1) * h[2]
                           - coefficient_value<A>(c.a2) * h[3];
    h[1] = h[0]; h[0] = input;
    h[3] = h[2]; h[2] = next;
    return next;
}

} // namespace

template <typename A> class BiquadTest : public ::testing::Test {};
TYPED_TEST_SUITE(BiquadTest, Formats);

TYPED_TEST(BiquadTest, ImpulseStepAndRandomDirectFormOneOracle) {
    using A = TypeParam;
    using S = typename A::Scalar;
    using C = typename A::Coefficients;
    std::array<C, 2> coefficients{{
        {A::coefficient(0.5), A::coefficient(0.25), A::coefficient(0.125), A::coefficient(-0.5), A::coefficient(0.125)},
        {A::coefficient(1.5), A::coefficient(-0.25), A::coefficient(0.125), A::coefficient(-0.25), A::coefficient(0.0625)}
    }};
    StateGuards<A> state(4), workspace(4);
    typename A::Biquad context;
    S output{};
    std::array<std::array<long double, 4>, 2> history{};
    ASSERT_EQ(A::biquad_init(&context, coefficients.data(), 2, state.data(), 4, workspace.data(), 4), LINEARS_OK);
    std::mt19937 random(0x51295);
    // Section l1 gains are bounded by 7/3 and 30/11. Q amplitude 1/32
    // therefore bounds both section outputs below 0.2, including rounding.
    const double amplitude = A::fixed ? 0.03125 : 2;
    std::uniform_real_distribution<double> samples(-amplitude, amplitude);
    const auto pulse = A::make(A::fixed ? amplitude : 1);
    for (int i = 0; i < 20000; ++i) {
        S input = i < 128 ? (i == 0 ? pulse : A::make(0)) : (i < 512 ? pulse : A::make(samples(random)));
        double reference = A::real(input);
        for (std::size_t j = 0; j < 2; ++j)
            reference = rounded<A>(static_cast<double>(direct_form_one<A>(coefficients[j], history[j], reference)));
        ASSERT_EQ(A::biquad_step(&context, input, &output), LINEARS_OK) << "sample " << i;
        if constexpr (A::fixed) {
            // The pre-export y recurrence has forcing bounded by
            // (2+|a1|+|a2|)/2 work LSBs, from both state roundings and the
            // rounded feedback y. Inverse-denominator l1 bounds 8/3 and 16/11
            // give local errors <= 4 and 2 work LSBs. With independently
            // quantized interfaces: (30/11)*(signal+4*work)+signal+2*work.
            expect_close<A>(output, reference, 4, 14);
        } else {
            expect_close<A>(output, reference, 16, 64);
        }
    }
    state.check(); workspace.check();
    ASSERT_EQ(A::biquad_reset(&context), LINEARS_OK);
    for (std::size_t i = 0; i < 4; ++i) EXPECT_EQ(A::state_real(state.data()[i]), 0);
    ASSERT_EQ(A::biquad_step(&context, pulse, &output), LINEARS_OK);
    EXPECT_EQ(A::real(output), 0.75 * A::real(pulse));
}

TYPED_TEST(BiquadTest, AnalyticSosFrequencyResponse) {
    using A = TypeParam;
    typename A::Coefficients c{A::coefficient(0.25), A::coefficient(0.5), A::coefficient(0.25),
                               A::coefficient(-0.5), A::coefficient(0.25)};
    std::array<typename A::State, 2> state{}, workspace{};
    typename A::Biquad context;
    typename A::Scalar output{};
    const double pi = std::acos(-1.0);
    for (int bin : {16, 128, 384}) {
        ASSERT_EQ(A::biquad_init(&context, &c, 1, state.data(), 2, workspace.data(), 2), LINEARS_OK);
        double omega = 2 * pi * bin / 1024;
        auto z = std::polar(1.0, -omega);
        auto expected = (A::real_coefficient(c.b0) + A::real_coefficient(c.b1) * z + A::real_coefficient(c.b2) * z * z)
                      / (1.0 + A::real_coefficient(c.a1) * z + A::real_coefficient(c.a2) * z * z);
        std::complex<double> response(0, 0), source(0, 0);
        for (int i = 0; i < 2048; ++i) {
            // Poles have radius 1/2 and angle pi/3, giving l1 gain < 2.31;
            // Q amplitude 1/4 leaves at least 0.42 output headroom.
            auto input = A::make((A::fixed ? 0.25 : 1) * std::sin(omega * i));
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

TYPED_TEST(BiquadTest, NegativeOnePointNineFeedbackImpulseStepAndLongSequence) {
    using A = TypeParam;
    using S = typename A::Scalar;
    const typename A::Coefficients c{A::coefficient(0.0125), A::coefficient(0.025), A::coefficient(0.0125),
                                     A::coefficient(-1.9), A::coefficient(0.95)};
    StateGuards<A> state(2), workspace(2);
    typename A::Biquad context{};
    S output{};
    ASSERT_EQ(A::biquad_init(&context, &c, 1, state.data(), 2, workspace.data(), 2), LINEARS_OK);
    const long double a1 = coefficient_value<A>(c.a1), a2 = coefficient_value<A>(c.a2);
    const long double radius = std::sqrt(a2), sine = std::sqrt(1 - a1 * a1 / (4 * a2));
    ASSERT_LT(a1, -1.8L);
    ASSERT_LT(radius, 1);
    ASSERT_GT(sine, 0);
    // For complex poles r*exp(+/-j theta), |1/D|'s impulse l1 norm is
    // <= 1 / ((1-r)*sin(theta)), about 176.7 for these ACTUAL stored poles.
    const long double inverse_norm = 1 / ((1 - radius) * sine);
    constexpr double amplitude = 0.03125;
    const long double feed_bound = amplitude * (std::abs(coefficient_value<A>(c.b0))
                                            + std::abs(coefficient_value<A>(c.b1))
                                            + std::abs(coefficient_value<A>(c.b2)));
    const long double signal_bound = feed_bound * inverse_norm;
    ASSERT_LT(signal_bound, 0.28L); // Includes arbitrary bounded inputs, not just a step.
    long double tolerance;
    if constexpr (A::fixed) {
        // TDF-II injects e_y[n]+e_s1[n-1]+e_s2[n-2], each <= 0.5 F31 LSB.
        // The direct signal export can differ from internal y by another
        // half work LSB plus half signal LSB. This bounds Q31 forward error
        // by the stored pole stability, rather than widening an observed miss.
        tolerance = (1.5L * inverse_norm + 0.5L) * A::work_unit + 0.5L * A::unit;
    } else {
        // Native multiply/add/sub roundings bounded by their operand sums;
        // factor 8 covers the y/s1/s2 operations and the ideal DF-I comparison.
        tolerance = 8 * A::unit * inverse_norm
                  * (feed_bound + (1 + std::abs(a1) + std::abs(a2)) * signal_bound);
    }
    std::array<long double, 4> history{};
    std::mt19937 random(0x1a195);
    std::uniform_real_distribution<double> samples(-amplitude, amplitude);
    for (int i = 0; i < 40000; ++i) {
        const S input = A::make(i < 1024 ? (i == 0 ? amplitude : 0)
                              : i < 4096 ? amplitude : i < 38000 ? samples(random) : 0);
        const long double reference = direct_form_one<A>(c, history, A::real(input));
        ASSERT_EQ(A::biquad_step(&context, input, &output), LINEARS_OK) << "sample " << i;
        EXPECT_NEAR(A::real(output), static_cast<double>(reference), static_cast<double>(tolerance)) << "sample " << i;
    }
    state.check(); workspace.check();
}

TYPED_TEST(BiquadTest, CapacityAndPartialAliasesAreAtomic) {
    using A = TypeParam;
    using S = typename A::Scalar;
    using W = typename A::State;
    using C = typename A::Coefficients;
    std::array<C, 2> coefficients{{
        {A::coefficient(1), A::coefficient(0.25), A::coefficient(0), A::coefficient(0), A::coefficient(0)},
        {A::coefficient(1), A::coefficient(0), A::coefficient(0), A::coefficient(0), A::coefficient(0)}
    }};
    const auto saved_coefficients = coefficients;
    StateGuards<A> state(4), workspace(4);
    typename A::Biquad context{};
    const auto uninitialized = context;
    const auto untouched = state.values;
    EXPECT_EQ(A::biquad_init(nullptr, coefficients.data(), 2, state.data(), 4, workspace.data(), 4), LINEARS_ERROR_ARGUMENT);
    EXPECT_EQ(A::biquad_init(&context, nullptr, 2, state.data(), 4, workspace.data(), 4), LINEARS_ERROR_ARGUMENT);
    EXPECT_EQ(A::biquad_init(&context, coefficients.data(), 2, nullptr, 4, workspace.data(), 4), LINEARS_ERROR_ARGUMENT);
    EXPECT_EQ(A::biquad_init(&context, coefficients.data(), 2, state.data(), 4, nullptr, 4), LINEARS_ERROR_ARGUMENT);
    EXPECT_EQ(A::biquad_init(&context, coefficients.data(), 0, state.data(), 4, workspace.data(), 4), LINEARS_ERROR_ARGUMENT);
    EXPECT_EQ(A::biquad_init(&context, coefficients.data(), 2, state.data(), 3, workspace.data(), 4), LINEARS_ERROR_WORKSPACE);
    EXPECT_EQ(A::biquad_init(&context, coefficients.data(), 2, state.data(), 4, workspace.data(), 3), LINEARS_ERROR_WORKSPACE);
    EXPECT_EQ(A::biquad_init(&context, coefficients.data(), SIZE_MAX, state.data(), SIZE_MAX, workspace.data(), SIZE_MAX), LINEARS_ERROR_SIZE);
    EXPECT_EQ(A::biquad_init(&context, coefficients.data(), 2, state.data(), 4, state.data(), 4), LINEARS_ERROR_ALIAS);
    EXPECT_EQ(A::biquad_init(&context, coefficients.data(), 2, state.data(), 4, state.data() + 1, 4), LINEARS_ERROR_ALIAS);
    EXPECT_EQ(A::biquad_init(&context, coefficients.data(), 2, reinterpret_cast<W *>(&coefficients[0].b1), 4, workspace.data(), 4), LINEARS_ERROR_ALIAS);
    EXPECT_EQ(A::biquad_init(&context, coefficients.data(), 2, state.data(), 4, reinterpret_cast<W *>(&coefficients[0].a1), 4), LINEARS_ERROR_ALIAS);
    EXPECT_EQ(A::biquad_init(&context, coefficients.data(), 2, reinterpret_cast<W *>(&context), 4, workspace.data(), 4), LINEARS_ERROR_ALIAS);
    expect_unchanged(context, uninitialized);
    EXPECT_EQ(state.values, untouched);
    EXPECT_EQ(std::memcmp(coefficients.data(), saved_coefficients.data(), sizeof(coefficients)), 0);

    ASSERT_EQ(A::biquad_init(&context, coefficients.data(), 2, state.data(), 4, workspace.data(), 4), LINEARS_OK);
    S output{};
    ASSERT_EQ(A::biquad_step(&context, A::make(0.125), &output), LINEARS_OK);
    const auto before = context;
    const auto saved = state.values;
    output = A::make(0.875);
    // Q output pointers only overlap part of a wide state/coefficient element.
    auto *partial_state = reinterpret_cast<S *>(reinterpret_cast<unsigned char *>(state.data() + 3) + sizeof(W) - sizeof(S));
    auto *partial_work = reinterpret_cast<S *>(reinterpret_cast<unsigned char *>(workspace.data() + 3) + sizeof(W) - sizeof(S));
    auto *partial_coefficient = reinterpret_cast<S *>(reinterpret_cast<unsigned char *>(&coefficients[1].a2)
                                                     + sizeof(typename A::Coefficient) - sizeof(S));
    EXPECT_EQ(A::biquad_init(&context, coefficients.data(), 2, state.data(), 3, workspace.data(), 4), LINEARS_ERROR_WORKSPACE);
    EXPECT_EQ(A::biquad_step(&context, A::make(0.25), partial_state), LINEARS_ERROR_ALIAS);
    EXPECT_EQ(A::biquad_step(&context, A::make(0.25), partial_work), LINEARS_ERROR_ALIAS);
    EXPECT_EQ(A::biquad_step(&context, A::make(0.25), partial_coefficient), LINEARS_ERROR_ALIAS);
    EXPECT_EQ(A::biquad_step(&context, A::make(0.25), reinterpret_cast<S *>(&context)), LINEARS_ERROR_ALIAS);
    EXPECT_EQ(A::biquad_step(&context, A::make(0.25), nullptr), LINEARS_ERROR_ARGUMENT);
    EXPECT_EQ(A::biquad_step(nullptr, A::make(0.25), &output), LINEARS_ERROR_ARGUMENT);
    EXPECT_EQ(A::biquad_reset(nullptr), LINEARS_ERROR_ARGUMENT);
    expect_unchanged(context, before);
    EXPECT_EQ(state.values, saved);
    EXPECT_EQ(output, A::make(0.875));
    EXPECT_EQ(std::memcmp(coefficients.data(), saved_coefficients.data(), sizeof(coefficients)), 0);
    state.check(); workspace.check();
}

TYPED_TEST(BiquadTest, LateStageOutputOverflowRollsBackAllStates) {
    using A = TypeParam;
    using S = typename A::Scalar;
    using C = typename A::Coefficients;
    const std::array<C, 2> coefficients{{
        {A::coefficient(1), A::coefficient(0.25), A::coefficient(0), A::coefficient(0), A::coefficient(0)},
        {A::coefficient(4), A::coefficient(0), A::coefficient(0), A::coefficient(0), A::coefficient(0)}
    }};
    StateGuards<A> state(4), workspace(4), control_state(4), control_work(4);
    typename A::Biquad context{}, control{};
    S output{}, expected{};
    ASSERT_EQ(A::biquad_init(&context, coefficients.data(), 2, state.data(), 4, workspace.data(), 4), LINEARS_OK);
    ASSERT_EQ(A::biquad_init(&control, coefficients.data(), 2, control_state.data(), 4, control_work.data(), 4), LINEARS_OK);
    ASSERT_EQ(A::biquad_step(&context, A::make(0.0625), &output), LINEARS_OK);
    ASSERT_EQ(A::biquad_step(&control, A::make(0.0625), &expected), LINEARS_OK);
    const auto before = context;
    const auto saved = state.values;
    for (int sign : {-1, 1}) {
        output = A::make(0.875);
        const S input = A::fixed ? A::make(sign * 0.5) : static_cast<S>(sign * (A::huge() / 2));
        EXPECT_EQ(A::biquad_step(&context, input, &output), LINEARS_ERROR_RANGE);
        expect_unchanged(context, before);
        EXPECT_EQ(state.values, saved);
        EXPECT_EQ(output, A::make(0.875));
        // Only persistent state/output are transactional; scratch may change.
        state.check(); workspace.check();
    }
    for (double input : {0.125, -0.125, 0.0, 0.0, 0.0}) {
        ASSERT_EQ(A::biquad_step(&context, A::make(input), &output), LINEARS_OK);
        ASSERT_EQ(A::biquad_step(&control, A::make(input), &expected), LINEARS_OK);
        EXPECT_EQ(output, expected);
        EXPECT_EQ(state.values, control_state.values);
    }
}

TYPED_TEST(BiquadTest, WideStateOverflowAtEitherUpdateIsAtomic) {
    using A = TypeParam;
    using S = typename A::Scalar;
    using C = typename A::Coefficients;
    using K = typename A::Coefficient;
    for (int update : {1, 2}) {
        K large;
        if constexpr (A::fixed) large = std::numeric_limits<K>::min();
        else large = A::huge();
        const std::array<C, 2> coefficients{{
            {A::coefficient(1), A::coefficient(0.25), A::coefficient(0), A::coefficient(0), A::coefficient(0)},
            {A::coefficient(0), update == 1 ? large : A::coefficient(0), update == 2 ? large : A::coefficient(0),
             A::coefficient(0), A::coefficient(0)}
        }};
        StateGuards<A> state(4), workspace(4);
        typename A::Biquad context{};
        ASSERT_EQ(A::biquad_init(&context, coefficients.data(), 2, state.data(), 4, workspace.data(), 4), LINEARS_OK);
        const auto before = context;
        const auto saved = state.values;
        S output = A::make(0.875);
        const S input = A::fixed ? A::make(-1) : static_cast<S>(A::huge() / 2);
        // y of the failing section is zero and representable. For Q the
        // candidate state is +2^32, one F31 raw unit above INT64_MAX.
        EXPECT_EQ(A::biquad_step(&context, input, &output), LINEARS_ERROR_RANGE);
        expect_unchanged(context, before);
        EXPECT_EQ(state.values, saved);
        EXPECT_EQ(output, A::make(0.875));
        for (int i = 0; i < 3; ++i) {
            ASSERT_EQ(A::biquad_step(&context, A::make(0), &output), LINEARS_OK);
            EXPECT_EQ(output, A::make(0));
            EXPECT_EQ(state.values, saved);
        }
        state.check(); workspace.check();
    }
}

TYPED_TEST(BiquadTest, CascadeEqualsSeparateInstances) {
    using A = TypeParam;
    using S = typename A::Scalar;
    using C = typename A::Coefficients;
    const std::array<C, 2> coefficients{{
        {A::coefficient(0.5), A::coefficient(0.25), A::coefficient(0), A::coefficient(-0.25), A::coefficient(0.125)},
        {A::coefficient(1.25), A::coefficient(0), A::coefficient(-0.125), A::coefficient(-0.5), A::coefficient(0.125)}
    }};
    std::array<typename A::State, 4> cs{}, cw{}, ss{}, sw{};
    typename A::Biquad cascade, first, second;
    S whole{}, partial{}, split{};
    ASSERT_EQ(A::biquad_init(&cascade, coefficients.data(), 2, cs.data(), 4, cw.data(), 4), LINEARS_OK);
    ASSERT_EQ(A::biquad_init(&first, coefficients.data(), 1, ss.data(), 2, sw.data(), 2), LINEARS_OK);
    ASSERT_EQ(A::biquad_init(&second, coefficients.data() + 1, 1, ss.data() + 2, 2, sw.data() + 2, 2), LINEARS_OK);
    // l1 gain bound is (0.75/0.625)*(1.375/0.375)=4.4; Q input 1/8
    // bounds the final output below 0.551 including interface rounding.
    for (int i = 0; i < 10000; ++i) {
        auto input = A::make((A::fixed ? 0.125 : 1) * std::sin(i * 0.173));
        ASSERT_EQ(A::biquad_step(&cascade, input, &whole), LINEARS_OK);
        ASSERT_EQ(A::biquad_step(&first, input, &partial), LINEARS_OK);
        ASSERT_EQ(A::biquad_step(&second, partial, &split), LINEARS_OK);
        EXPECT_EQ(whole, split);
        EXPECT_EQ(cs, ss); // Every raw state, not only the final output, agrees.
    }
    ASSERT_EQ(A::biquad_reset(&cascade), LINEARS_OK);
    ASSERT_EQ(A::biquad_reset(&first), LINEARS_OK);
    ASSERT_EQ(A::biquad_reset(&second), LINEARS_OK);
    EXPECT_EQ(cs, ss);
    for (auto value : cs) EXPECT_EQ(value, A::state_make(0));
}

TYPED_TEST(BiquadTest, WideStatesAndProductsCancelWhileOutputRemainsRepresentable) {
    using A = TypeParam;
    using S = typename A::Scalar;
    const typename A::Coefficients c{A::coefficient(4), A::coefficient(-8), A::coefficient(4),
                                     A::coefficient(0), A::coefficient(0)};
    StateGuards<A> state(2), workspace(2);
    typename A::Biquad context{};
    S output{};
    ASSERT_EQ(A::biquad_init(&context, &c, 1, state.data(), 2, workspace.data(), 2), LINEARS_OK);
    std::array<long double, 4> history{};
    for (int i = 1; i <= 64; ++i) {
        const S input = A::make(std::min(i, 6) * 0.125);
        const long double reference = direct_form_one<A>(c, history, A::real(input));
        ASSERT_EQ(A::biquad_step(&context, input, &output), LINEARS_OK);
        EXPECT_EQ(A::real(output), reference);
    }
    // The second difference is zero after the ramp: y=4*(3/4)-3.
    // Both states exceed the signal range and individual F62 products exceed
    // signed 64 bits; neither a signal-state buffer nor an int64 MAC suffices.
    EXPECT_EQ(A::state_real(state.data()[0]), -3);
    EXPECT_EQ(A::state_real(state.data()[1]), 3);
    EXPECT_EQ(output, A::make(0));
    state.check(); workspace.check();
}

TYPED_TEST(BiquadTest, ParallelContextsAndResetAreIndependent) {
    using A = TypeParam;
    using S = typename A::Scalar;
    constexpr std::size_t count = 2048;
    const typename A::Coefficients c{A::coefficient(0.5), A::coefficient(0.25), A::coefficient(0.125),
                                     A::coefficient(-0.5), A::coefficient(0.125)};
    std::array<std::array<typename A::State, 2>, 2> states{}, workspaces{};
    std::array<typename A::Biquad, 2> contexts{};
    std::array<std::array<S, count>, 2> inputs{}, outputs{};
    std::array<std::array<linears_status_t, count>, 2> statuses{};
    for (std::size_t j = 0; j < 2; ++j) {
        ASSERT_EQ(A::biquad_init(&contexts[j], &c, 1, states[j].data(), 2, workspaces[j].data(), 2), LINEARS_OK);
        for (std::size_t i = 0; i < count; ++i)
            inputs[j][i] = A::make((A::fixed ? 0.125 : 2) * std::sin(i * (j == 0 ? 0.173 : -0.257)));
    }
    auto run = [&](std::size_t j) {
        for (std::size_t i = 0; i < count; ++i)
            statuses[j][i] = A::biquad_step(&contexts[j], inputs[j][i], &outputs[j][i]);
    };
    std::thread first(run, 0), second(run, 1);
    first.join(); second.join();
    for (std::size_t j = 0; j < 2; ++j) {
        std::array<typename A::State, 2> state{}, workspace{};
        typename A::Biquad sequential{};
        ASSERT_EQ(A::biquad_init(&sequential, &c, 1, state.data(), 2, workspace.data(), 2), LINEARS_OK);
        for (std::size_t i = 0; i < count; ++i) {
            S expected{};
            ASSERT_EQ(statuses[j][i], LINEARS_OK);
            ASSERT_EQ(A::biquad_step(&sequential, inputs[j][i], &expected), LINEARS_OK);
            EXPECT_EQ(outputs[j][i], expected);
        }
        EXPECT_EQ(states[j], state);
    }
    const auto other = contexts[1];
    const auto other_state = states[1];
    const auto other_workspace = workspaces[1];
    ASSERT_EQ(A::biquad_reset(&contexts[0]), LINEARS_OK);
    for (std::size_t i = 0; i < count; ++i) {
        S output{};
        ASSERT_EQ(A::biquad_step(&contexts[0], inputs[0][i], &output), LINEARS_OK);
        EXPECT_EQ(output, outputs[0][i]);
    }
    expect_unchanged(contexts[1], other);
    EXPECT_EQ(states[1], other_state);
    EXPECT_EQ(workspaces[1], other_workspace);
}

using BiquadFixedFormats = ::testing::Types<Q15, Q31>;
template <typename A> class BiquadFixedTest : public ::testing::Test {};
TYPED_TEST_SUITE(BiquadFixedTest, BiquadFixedFormats);

TYPED_TEST(BiquadFixedTest, HalfwayDirectExportAndWideFeedback) {
    using A = TypeParam;
    using S = typename A::Scalar;
    using C = typename A::Coefficients;
    static_assert(std::is_same_v<typename A::Coefficient, linears_fixed_t>);
    static_assert(std::is_same_v<typename A::State, linears_fixed_t>);
    const C c{A::coefficient(0.5), 0, 0, A::coefficient(-0.5), 0};
    std::array<typename A::State, 2> state{}, workspace{};
    typename A::Biquad context{};
    S output{};
    ASSERT_EQ(A::biquad_init(&context, &c, 1, state.data(), 2, workspace.data(), 2), LINEARS_OK);
    for (int sign : {-1, 1}) {
        ASSERT_EQ(A::biquad_reset(&context), LINEARS_OK);
        ASSERT_EQ(A::biquad_step(&context, static_cast<S>(sign), &output), LINEARS_OK);
        EXPECT_EQ(output, sign);
        EXPECT_EQ(state[0], sign * (A::bits == 15 ? 16384 : 1));
        ASSERT_EQ(A::biquad_step(&context, S(0), &output), LINEARS_OK);
        EXPECT_EQ(output, A::bits == 15 ? 0 : sign);
        EXPECT_EQ(state[0], sign * (A::bits == 15 ? 8192 : 1));
    }
    const C below_half{(INT64_C(1) << 30) - 1, 0, 0, A::coefficient(-0.5), 0};
    ASSERT_EQ(A::biquad_init(&context, &below_half, 1, state.data(), 2, workspace.data(), 2), LINEARS_OK);
    ASSERT_EQ(A::biquad_step(&context, S(1), &output), LINEARS_OK);
    EXPECT_EQ(output, 0); // q15 would become one if y were first rounded to F31.
    EXPECT_EQ(state[0], A::bits == 15 ? 16384 : 0); // Feedback still uses F31 y.
}

TYPED_TEST(BiquadFixedTest, EndpointsPreserveWideFeedbackAndTrueOverflowIsNotClipped) {
    using A = TypeParam;
    using S = typename A::Scalar;
    using K = typename A::Coefficient;
    using C = typename A::Coefficients;
    std::array<typename A::State, 2> state{}, workspace{};
    typename A::Biquad context{};
    S output{};
    const C identity{A::coefficient(1), 0, 0, 0, 0};
    ASSERT_EQ(A::biquad_init(&context, &identity, 1, state.data(), 2, workspace.data(), 2), LINEARS_OK);
    const S maximum = std::numeric_limits<S>::max(), minimum = std::numeric_limits<S>::min();
    for (S input : {minimum, S(-1), S(0), S(1), maximum}) {
        ASSERT_EQ(A::biquad_step(&context, input, &output), LINEARS_OK);
        EXPECT_EQ(output, input);
    }
    EXPECT_EQ(A::real(output), 1 - A::unit);
    const K endpoint_gain = INT64_C(1) << (31 + A::bits);
    const C endpoint{endpoint_gain, 0, 0, A::coefficient(-1), 0};
    ASSERT_EQ(A::biquad_init(&context, &endpoint, 1, state.data(), 2, workspace.data(), 2), LINEARS_OK);
    ASSERT_EQ(A::biquad_step(&context, S(1), &output), LINEARS_OK);
    EXPECT_EQ(output, maximum); // Exact +1 uses the endpoint output policy.
    EXPECT_EQ(state[0], A::state_make(1)); // MAX is not substituted for internal y.
    ASSERT_EQ(A::biquad_step(&context, S(0), &output), LINEARS_OK);
    EXPECT_EQ(output, maximum);
    EXPECT_EQ(state[0], A::state_make(1));
    ASSERT_EQ(A::biquad_reset(&context), LINEARS_OK);
    ASSERT_EQ(A::biquad_step(&context, S(-1), &output), LINEARS_OK);
    EXPECT_EQ(output, minimum);
    EXPECT_EQ(state[0], A::state_make(-1));
    for (int sign : {-1, 1}) {
        const C excessive{sign * (endpoint_gain + 1), 0, 0, 0, 0};
        ASSERT_EQ(A::biquad_init(&context, &excessive, 1, state.data(), 2, workspace.data(), 2), LINEARS_OK);
        const auto before = context;
        const auto saved = state;
        output = A::make(0.875);
        // +/- (1 + 2^-(31+bits)): F31 rounding must not hide the violation.
        EXPECT_EQ(A::biquad_step(&context, S(1), &output), LINEARS_ERROR_RANGE);
        expect_unchanged(context, before);
        EXPECT_EQ(state, saved);
        EXPECT_EQ(output, A::make(0.875));
    }
}

TYPED_TEST(BiquadFixedTest, StateUpdatesRoundOnlyAfterAllProducts) {
    using A = TypeParam;
    using S = typename A::Scalar;
    // With input raw +/-1, each feedforward/feedback product is exactly
    // +/- half an F31 LSB, even for the coarser q15 signal representation.
    const auto half_work_product = INT64_C(1) << (A::bits - 1);
    const typename A::Coefficients c{A::coefficient(1), half_work_product, half_work_product,
                                     -half_work_product, -half_work_product};
    std::array<typename A::State, 2> state{}, workspace{};
    typename A::Biquad context{};
    S output{};
    ASSERT_EQ(A::biquad_init(&context, &c, 1, state.data(), 2, workspace.data(), 2), LINEARS_OK);
    for (int sign : {-1, 1}) {
        ASSERT_EQ(A::biquad_reset(&context), LINEARS_OK);
        ASSERT_EQ(A::biquad_step(&context, static_cast<S>(sign), &output), LINEARS_OK);
        EXPECT_EQ(output, sign);
        EXPECT_EQ(state[0], sign); // Rounding individual products would give +/-2.
        EXPECT_EQ(state[1], sign);
    }
}

TYPED_TEST(BiquadFixedTest, ExtremeFeedforwardFeedbackCancelBeforeStateRounding) {
    using A = TypeParam;
    using S = typename A::Scalar;
    using K = typename A::Coefficient;
    const K minimum = std::numeric_limits<K>::min(), maximum = std::numeric_limits<K>::max();
    // Exact pole-zero cancellation is an arithmetic identity, not a stability
    // fixture. For y=x, both state updates are zero even when individual
    // products cannot be represented as F31 (notably MIN times -1).
    const typename A::Coefficients c{A::coefficient(1), minimum, maximum, minimum, maximum};
    StateGuards<A> state(2), workspace(2);
    typename A::Biquad context{};
    S output{};
    ASSERT_EQ(A::biquad_init(&context, &c, 1, state.data(), 2, workspace.data(), 2), LINEARS_OK);
    for (S input : {std::numeric_limits<S>::min(), std::numeric_limits<S>::max(), S(-1), S(0), S(1)}) {
        ASSERT_EQ(A::biquad_step(&context, input, &output), LINEARS_OK);
        EXPECT_EQ(output, input);
        EXPECT_EQ(state.data()[0], 0);
        EXPECT_EQ(state.data()[1], 0);
    }
    std::mt19937 random(0x128f62);
    std::uniform_int_distribution<std::int64_t> samples(std::numeric_limits<S>::min(), std::numeric_limits<S>::max());
    for (int i = 0; i < 4096; ++i) {
        const S input = static_cast<S>(samples(random));
        ASSERT_EQ(A::biquad_step(&context, input, &output), LINEARS_OK);
        EXPECT_EQ(output, input);
        EXPECT_EQ(state.data()[0], 0);
        EXPECT_EQ(state.data()[1], 0);
    }
    state.check(); workspace.check();
}

using BiquadFloatFormats = ::testing::Types<F32, F64>;
template <typename A> class BiquadFloatTest : public ::testing::Test {};
TYPED_TEST_SUITE(BiquadFloatTest, BiquadFloatFormats);

TYPED_TEST(BiquadFloatTest, NonFiniteFailuresAndLargeFiniteSignals) {
    using A = TypeParam;
    using S = typename A::Scalar;
    using C = typename A::Coefficients;
    const std::array<C, 2> coefficients{{
        {A::coefficient(1), A::coefficient(0), A::coefficient(0), A::coefficient(0), A::coefficient(0)},
        {A::coefficient(1), A::coefficient(0), A::coefficient(0), A::coefficient(0), A::coefficient(0)}
    }};
    const std::array<S C::*, 5> members{&C::b0, &C::b1, &C::b2, &C::a1, &C::a2};
    StateGuards<A> state(4), workspace(4);
    typename A::Biquad context{};
    for (S invalid : {A::invalid(), std::numeric_limits<S>::infinity(), -std::numeric_limits<S>::infinity()}) {
        for (auto member : members) {
            auto bad = coefficients;
            bad[1].*member = invalid;
            const auto before = context;
            const auto saved = state.values;
            EXPECT_EQ(A::biquad_init(&context, bad.data(), 2, state.data(), 4, workspace.data(), 4), LINEARS_ERROR_RANGE);
            expect_unchanged(context, before);
            EXPECT_EQ(state.values, saved);
        }
    }
    ASSERT_EQ(A::biquad_init(&context, coefficients.data(), 2, state.data(), 4, workspace.data(), 4), LINEARS_OK);
    S output{};
    for (S input : {A::huge(), static_cast<S>(-A::huge()), A::tiny(), std::numeric_limits<S>::denorm_min()}) {
        ASSERT_EQ(A::biquad_step(&context, input, &output), LINEARS_OK);
        EXPECT_EQ(output, input);
    }
    const auto before = context;
    const auto saved = state.values;
    output = A::make(0.875);
    for (S invalid : {A::invalid(), std::numeric_limits<S>::infinity(), -std::numeric_limits<S>::infinity()}) {
        EXPECT_EQ(A::biquad_step(&context, invalid, &output), LINEARS_ERROR_RANGE);
        expect_unchanged(context, before);
        EXPECT_EQ(state.values, saved);
        EXPECT_EQ(output, A::make(0.875));
    }
    state.check(); workspace.check();
}
