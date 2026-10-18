#include "kalman_support.h"

namespace {
using namespace kalman_test;

template<class Api> class KalmanScalar : public ::testing::Test {};
TYPED_TEST_SUITE(KalmanScalar, Formats);

TYPED_TEST(KalmanScalar, ResetKnownExampleAndZeroInnovation) {
    using A = TypeParam;
    typename A::State state{};
    if constexpr (A::bits == 0) {
        ASSERT_EQ(A::reset(&state, A::encode(2), A::encode(4)), LINEARS_OK);
        ASSERT_EQ(A::predict(&state, A::coefficient(1), A::encode(1)), LINEARS_OK);
        expect_near<A>(A::decode(state.estimate), 2);
        expect_near<A>(A::decode(state.covariance), 5);
        ASSERT_EQ(A::update(&state, A::encode(2), A::coefficient(1), A::encode(1)), LINEARS_OK);
        expect_near<A>(A::decode(state.estimate), 2);
        expect_near<A>(A::decode(state.covariance), 5.0L / 6);
    } else {
        ASSERT_EQ(A::reset(&state, A::encode(0.25L), A::encode(0.25L)), LINEARS_OK);
        ASSERT_EQ(A::predict(&state, A::coefficient(0.5L), A::encode(0.0625L)), LINEARS_OK);
        expect_near<A>(A::decode(state.estimate), 0.125L);
        expect_near<A>(A::decode(state.covariance), 0.125L);
        ASSERT_EQ(A::update(&state, A::encode(0.0625L), A::coefficient(0.5L), A::encode(0.125L)), LINEARS_OK);
        expect_near<A>(A::decode(state.estimate), 0.125L);
        expect_near<A>(A::decode(state.covariance), 0.1L);
    }
    ASSERT_EQ(A::reset(&state, A::encode(-0.25L), A::encode(0)), LINEARS_OK);
    ASSERT_EQ(A::update(&state, A::encode(0.75L), A::coefficient(-0.5L), A::encode(0.25L)), LINEARS_OK);
    expect_near<A>(A::decode(state.estimate), -0.25L);
    EXPECT_EQ(A::decode(state.covariance), 0);
    ASSERT_EQ(A::predict(&state, A::coefficient(0), A::encode(0)), LINEARS_OK);
    EXPECT_EQ(A::decode(state.estimate), 0);
    EXPECT_EQ(A::decode(state.covariance), 0);
}

TYPED_TEST(KalmanScalar, JosephRetainsRepresentableNoise) {
    using A = TypeParam;
    typename A::State state{};
    typename A::Scalar noise;
    if constexpr (A::bits == 0) noise = A::encode(std::ldexp(1.0L, -80));
    else noise = 1; /* The minimum positive fixed-fraction raw variance. */
    ASSERT_EQ(A::reset(&state, A::encode(0), A::encode(0.5L)), LINEARS_OK);
    ASSERT_EQ(A::update(&state, A::encode(0), A::coefficient(1), noise), LINEARS_OK);
    const long double variance = A::decode(state.covariance);
    EXPECT_GT(variance, 0);
    EXPECT_LT(std::abs(variance / A::decode(noise) - 1), 1e-5L);
    if constexpr (A::bits != 0) {
        const auto before = bytes(state);
        const auto rounded_to_zero = A::encode(std::ldexp(1.0L, -80));
        ASSERT_EQ(rounded_to_zero, 0);
        EXPECT_EQ(A::update(&state, A::encode(0), A::coefficient(0.5L), rounded_to_zero), LINEARS_ERROR_DOMAIN);
        EXPECT_EQ(bytes(state), before);
    }
}

TYPED_TEST(KalmanScalar, WideGainInnovationAndExactUninformativeUpdate) {
    using A = TypeParam;
    const long double models[3][3] = {
        {0.5L, 0.25L, 0.015625L}, {0.75L, 0.75L, 0.75L}, {0.125L, 2, 0.125L}};
    typename A::State state{};
    for (int i = 0; i < 3; ++i) {
        const auto p = A::encode(models[i][0]), r = A::encode(models[i][2]);
        const auto h = A::coefficient(models[i][1]);
        const auto z = A::encode(0.125L);
        const long double s = A::decode_coefficient(h) * A::decode(p) * A::decode_coefficient(h) + A::decode(r);
        if (i == 0) ASSERT_GT(A::decode(p) * A::decode_coefficient(h) / s, 1);
        else if (i == 1) ASSERT_GT(s, 1);
        else ASSERT_EQ(A::decode_coefficient(h), 2);
        ScalarReference reference{0, A::decode(p)};
        reference.update(A::decode(z), A::decode_coefficient(h), A::decode(r));
        ASSERT_EQ(A::reset(&state, A::encode(0), p), LINEARS_OK);
        ASSERT_EQ(A::update(&state, z, h, r), LINEARS_OK);
        expect_near<A>(A::decode(state.estimate), reference.x);
        expect_near<A>(A::decode(state.covariance), reference.p);
    }
    ASSERT_EQ(A::reset(&state, A::encode(0.25L), A::encode(0.75L)), LINEARS_OK);
    const auto before = bytes(state);
    for (int i = 0; i < 400; ++i)
        ASSERT_EQ(A::update(&state, A::encode(0.5L), A::coefficient(0), A::encode(0.25L)), LINEARS_OK);
    EXPECT_EQ(bytes(state), before);
}

TYPED_TEST(KalmanScalar, RandomizedIndependentLongDoubleOracle) {
    using A = TypeParam;
    std::mt19937 random(0x4b414c31u);
    std::uniform_real_distribution<double> uniform(-1, 1);
    for (int i = 0; i < 1000; ++i) {
        SCOPED_TRACE(i);
        const auto x = A::encode((A::bits ? 0.4 : 4) * uniform(random));
        const auto p = A::encode((A::bits ? 0.25 : 1) * (1 + uniform(random)));
        const auto a = A::coefficient((A::bits ? 0.9 : 1.1) * uniform(random));
        const auto q = A::encode((A::bits ? 0.005 : 0.05) * (1 + uniform(random)));
        const auto z = A::encode((A::bits ? 0.3 : 5) * uniform(random));
        const auto h = A::coefficient((A::bits ? 0.8 : 2) * uniform(random));
        const auto r = A::encode(0.1 + (A::bits ? 0.1 : 0.5) * (1 + uniform(random)));
        typename A::State state{};
        ASSERT_EQ(A::reset(&state, x, p), LINEARS_OK);
        ScalarReference reference{A::decode(x), A::decode(p)};
        reference.predict(A::decode_coefficient(a), A::decode(q));
        ASSERT_EQ(A::predict(&state, a, q), LINEARS_OK);
        expect_near<A>(A::decode(state.estimate), reference.x);
        expect_near<A>(A::decode(state.covariance), reference.p);
        reference.update(A::decode(z), A::decode_coefficient(h), A::decode(r));
        ASSERT_EQ(A::update(&state, z, h, r), LINEARS_OK);
        expect_near<A>(A::decode(state.estimate), reference.x);
        expect_near<A>(A::decode(state.covariance), reference.p);
        EXPECT_GE(A::decode(state.covariance), 0);
    }
}

TYPED_TEST(KalmanScalar, LongSequenceResetAndIndependentContexts) {
    using A = TypeParam;
    typename A::State first{}, second{}, solo{};
    const auto a = A::coefficient(0.995L), h = A::coefficient(A::bits ? 0.75L : 1.25L);
    const auto q = A::encode(A::bits ? 0.002L : 0.01L), r = A::encode(A::bits ? 0.05L : 0.15L);
    const auto initial_variance = A::encode(A::bits ? 0.5L : 2);
    ASSERT_EQ(A::reset(&first, A::encode(0), initial_variance), LINEARS_OK);
    ASSERT_EQ(A::reset(&solo, A::encode(0), initial_variance), LINEARS_OK);
    ASSERT_EQ(A::reset(&second, A::encode(A::bits ? 0.75L : 100), A::encode(0.5L)), LINEARS_OK);
    ScalarReference reference{0, A::decode(initial_variance)};
    std::mt19937 random(0x4b414c32u);
    std::uniform_real_distribution<double> noise(A::bits ? -0.025 : -0.25, A::bits ? 0.025 : 0.25);
    for (int i = 0; i < 6000; ++i) {
        SCOPED_TRACE(i);
        const auto z = A::encode((A::bits ? 0.2L : 2) * std::sin(i * 0.013L) + noise(random));
        ASSERT_EQ(A::predict(&first, a, q), LINEARS_OK);
        ASSERT_EQ(A::predict(&second, A::coefficient(A::bits ? 0.25L : 1), q), LINEARS_OK);
        ASSERT_EQ(A::update(&first, z, h, r), LINEARS_OK);
        ASSERT_EQ(A::update(&second, A::encode(A::bits ? -0.25L : -2), A::coefficient(0), A::encode(0.25L)), LINEARS_OK);
        ASSERT_EQ(A::predict(&solo, a, q), LINEARS_OK);
        ASSERT_EQ(A::update(&solo, z, h, r), LINEARS_OK);
        reference.predict(A::decode_coefficient(a), A::decode(q));
        reference.update(A::decode(z), A::decode_coefficient(h), A::decode(r));
        expect_near<A>(A::decode(first.estimate), reference.x, 2);
        expect_near<A>(A::decode(first.covariance), reference.p, 2);
        EXPECT_EQ(A::decode(first.estimate), A::decode(solo.estimate));
        EXPECT_EQ(A::decode(first.covariance), A::decode(solo.covariance));
        EXPECT_GE(A::decode(first.covariance), 0);
    }
    ASSERT_EQ(A::reset(&first, A::encode(0), initial_variance), LINEARS_OK);
    expect_near<A>(A::decode(first.estimate), 0);
    expect_near<A>(A::decode(first.covariance), A::decode(initial_variance));
    expect_near<A>(A::decode(second.estimate), A::bits ? 0 : 100);
}

TYPED_TEST(KalmanScalar, InvalidNoiseNullAndFailureAtomicity) {
    using A = TypeParam;
    typename A::State state{};
    const auto zero = A::encode(0), one = A::encode(1), negative = A::encode(-1);
    ASSERT_EQ(A::reset(&state, one, one), LINEARS_OK);
    const auto original = bytes(state);
    EXPECT_EQ(A::reset(nullptr, zero, one), LINEARS_ERROR_ARGUMENT);
    EXPECT_EQ(A::predict(nullptr, A::coefficient_from_raw(one), zero), LINEARS_ERROR_ARGUMENT);
    EXPECT_EQ(A::update(nullptr, zero, A::coefficient_from_raw(one), one), LINEARS_ERROR_ARGUMENT);
    EXPECT_EQ(A::reset(&state, zero, negative), LINEARS_ERROR_DOMAIN);
    EXPECT_EQ(bytes(state), original);
    EXPECT_EQ(A::predict(&state, A::coefficient_from_raw(one), negative), LINEARS_ERROR_DOMAIN);
    EXPECT_EQ(bytes(state), original);
    EXPECT_EQ(A::update(&state, zero, A::coefficient_from_raw(one), zero), LINEARS_ERROR_DOMAIN);
    EXPECT_EQ(bytes(state), original);
    EXPECT_EQ(A::update(&state, zero, A::coefficient_from_raw(one), negative), LINEARS_ERROR_DOMAIN);
    EXPECT_EQ(bytes(state), original);
    state.covariance = negative;
    const auto invalid = bytes(state);
    EXPECT_EQ(A::predict(&state, A::coefficient_from_raw(one), zero), LINEARS_ERROR_DOMAIN);
    EXPECT_EQ(bytes(state), invalid);
    EXPECT_EQ(A::update(&state, zero, A::coefficient_from_raw(one), one), LINEARS_ERROR_DOMAIN);
    EXPECT_EQ(bytes(state), invalid);
}

TYPED_TEST(KalmanScalar, NumericRangeAndNonfiniteInputsAreAtomic) {
    using A = TypeParam;
    typename A::State state{};
    const auto zero = A::encode(0), one = A::encode(1);
    ASSERT_EQ(A::reset(&state, one, one), LINEARS_OK);
    if constexpr (A::bits == 0) {
        const auto original = bytes(state);
        for (auto invalid : {std::numeric_limits<typename A::Raw>::quiet_NaN(),
                             std::numeric_limits<typename A::Raw>::infinity(),
                             -std::numeric_limits<typename A::Raw>::infinity()}) {
            EXPECT_EQ(A::reset(&state, invalid, one), LINEARS_ERROR_RANGE);
            EXPECT_EQ(A::reset(&state, one, invalid), LINEARS_ERROR_RANGE);
            EXPECT_EQ(A::predict(&state, invalid, zero), LINEARS_ERROR_RANGE);
            EXPECT_EQ(A::predict(&state, A::coefficient_from_raw(one), invalid), LINEARS_ERROR_RANGE);
            EXPECT_EQ(A::update(&state, invalid, A::coefficient_from_raw(one), one), LINEARS_ERROR_RANGE);
            EXPECT_EQ(A::update(&state, one, invalid, one), LINEARS_ERROR_RANGE);
            EXPECT_EQ(A::update(&state, one, A::coefficient_from_raw(one), invalid), LINEARS_ERROR_RANGE);
            EXPECT_EQ(bytes(state), original);
        }
        const auto maximum = std::numeric_limits<typename A::Raw>::max();
        ASSERT_EQ(A::reset(&state, maximum, one), LINEARS_OK);
        const auto large = bytes(state);
        EXPECT_EQ(A::predict(&state, A::coefficient(2), zero), LINEARS_ERROR_RANGE);
        EXPECT_EQ(bytes(state), large);
        EXPECT_EQ(A::update(&state, -maximum, A::coefficient(2), one), LINEARS_ERROR_RANGE);
        EXPECT_EQ(bytes(state), large);
        ASSERT_EQ(A::reset(&state, zero, one), LINEARS_OK);
        const auto restored = bytes(state);
        EXPECT_EQ(A::predict(&state, A::coefficient_from_raw(maximum), zero), LINEARS_ERROR_RANGE);
        EXPECT_EQ(bytes(state), restored);
    } else {
        const auto minimum = std::numeric_limits<typename A::Raw>::min();
        const auto maximum = std::numeric_limits<typename A::Raw>::max();
        ASSERT_EQ(A::reset(&state, minimum, zero), LINEARS_OK);
        EXPECT_EQ(A::decode(state.estimate), -1);
        ASSERT_EQ(A::predict(&state, A::coefficient_from_raw(minimum), zero), LINEARS_OK); /* (-1)*(-1) is exactly +1. */
        EXPECT_EQ(state.estimate, maximum);
        EXPECT_EQ(1 - A::decode(state.estimate), std::ldexp(1.0L, -A::bits));
        ASSERT_EQ(A::reset(&state, maximum, zero), LINEARS_OK);
        ASSERT_EQ(A::predict(&state, A::coefficient(1), zero), LINEARS_OK);
        EXPECT_EQ(state.estimate, maximum);
        ASSERT_EQ(A::predict(&state, A::coefficient_from_raw(maximum), zero), LINEARS_OK);
        EXPECT_EQ(state.estimate, maximum - 1); /* MAX is not an identity coefficient. */
        ASSERT_EQ(A::reset(&state, A::encode(0.25L), zero), LINEARS_OK);
        const auto sub_working_lsb = bytes(state);
        const linears_fixed_t above_four = 4 * LINEARS_FIXED_ONE + 1;
        /* Exact products are +/- (1 + 2^-33): do not hide the excess by F31 rounding. */
        EXPECT_EQ(A::predict(&state, above_four, zero), LINEARS_ERROR_RANGE);
        EXPECT_EQ(bytes(state), sub_working_lsb);
        EXPECT_EQ(A::predict(&state, -above_four, zero), LINEARS_ERROR_RANGE);
        EXPECT_EQ(bytes(state), sub_working_lsb);
        ASSERT_EQ(A::reset(&state, A::encode(0.25L), A::encode(0.75L)), LINEARS_OK);
        const auto prediction_before = bytes(state);
        EXPECT_EQ(A::predict(&state, A::coefficient_from_raw(maximum), A::encode(0.75L)), LINEARS_ERROR_RANGE);
        EXPECT_EQ(bytes(state), prediction_before);
        ASSERT_EQ(A::reset(&state, A::encode(0.75L), A::encode(0.75L)), LINEARS_OK);
        const auto update_before = bytes(state);
        EXPECT_EQ(A::update(&state, A::encode(0.75L), A::coefficient(0.125L), A::encode(0.015625L)), LINEARS_ERROR_RANGE);
        EXPECT_EQ(bytes(state), update_before);
    }
}

} // namespace
