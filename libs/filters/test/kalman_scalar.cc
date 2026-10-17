#include "kalman_support.h"

namespace {
using namespace kalman_test;

template<class Api> class KalmanScalar : public ::testing::Test {};
TYPED_TEST_SUITE(KalmanScalar, Formats);

TYPED_TEST(KalmanScalar, ResetKnownExampleAndZeroInnovation) {
    using A = TypeParam;
    typename A::State state{};
    ASSERT_EQ(A::reset(&state, A::encode(2), A::encode(4)), LINEARS_OK);
    ASSERT_EQ(A::predict(&state, A::encode(1), A::encode(1)), LINEARS_OK);
    expect_near<A>(A::decode(state.estimate), 2);
    expect_near<A>(A::decode(state.covariance), 5);
    ASSERT_EQ(A::update(&state, A::encode(2), A::encode(1), A::encode(1)), LINEARS_OK);
    expect_near<A>(A::decode(state.estimate), 2);
    expect_near<A>(A::decode(state.covariance), 5.0L / 6);
    ASSERT_EQ(A::reset(&state, A::encode(-3), A::encode(0)), LINEARS_OK);
    ASSERT_EQ(A::update(&state, A::encode(100), A::encode(-2), A::encode(1)), LINEARS_OK);
    expect_near<A>(A::decode(state.estimate), -3);
    EXPECT_EQ(A::decode(state.covariance), 0);
    ASSERT_EQ(A::predict(&state, A::encode(0), A::encode(0)), LINEARS_OK);
    EXPECT_EQ(A::decode(state.estimate), 0);
    EXPECT_EQ(A::decode(state.covariance), 0);
}

TYPED_TEST(KalmanScalar, JosephRetainsNoiseBelowGainResolution) {
    using A = TypeParam;
    typename A::State state{};
    const auto tiny_noise = A::encode(std::ldexp(1.0L, -80));
    ASSERT_EQ(A::reset(&state, A::encode(0), A::encode(1)), LINEARS_OK);
    ASSERT_EQ(A::update(&state, A::encode(0), A::encode(1), tiny_noise), LINEARS_OK);
    const long double variance = A::decode(state.covariance);
    EXPECT_GT(variance, 0);
    EXPECT_LT(std::abs(variance / A::decode(tiny_noise) - 1), 1e-5L);
}

TYPED_TEST(KalmanScalar, RandomizedIndependentLongDoubleOracle) {
    using A = TypeParam;
    std::mt19937 random(0x4b414c31u);
    std::uniform_real_distribution<double> uniform(-1, 1);
    for (int i = 0; i < 1000; ++i) {
        SCOPED_TRACE(i);
        const auto x = A::encode(4 * uniform(random));
        const auto p = A::encode(1 + uniform(random));
        const auto a = A::encode(1.1 * uniform(random));
        const auto q = A::encode(0.05 * (1 + uniform(random)));
        const auto z = A::encode(5 * uniform(random));
        const auto h = A::encode(2 * uniform(random));
        const auto r = A::encode(0.1 + 0.5 * (1 + uniform(random)));
        typename A::State state{};
        ASSERT_EQ(A::reset(&state, x, p), LINEARS_OK);
        ScalarReference reference{A::decode(x), A::decode(p)};
        reference.predict(A::decode(a), A::decode(q));
        ASSERT_EQ(A::predict(&state, a, q), LINEARS_OK);
        expect_near<A>(A::decode(state.estimate), reference.x);
        expect_near<A>(A::decode(state.covariance), reference.p);
        reference.update(A::decode(z), A::decode(h), A::decode(r));
        ASSERT_EQ(A::update(&state, z, h, r), LINEARS_OK);
        expect_near<A>(A::decode(state.estimate), reference.x);
        expect_near<A>(A::decode(state.covariance), reference.p);
        EXPECT_GE(A::decode(state.covariance), 0);
    }
}

TYPED_TEST(KalmanScalar, LongSequenceResetAndIndependentContexts) {
    using A = TypeParam;
    typename A::State first{}, second{}, solo{};
    const auto a = A::encode(0.995L), q = A::encode(0.01L);
    const auto h = A::encode(1.25L), r = A::encode(0.15L);
    ASSERT_EQ(A::reset(&first, A::encode(0), A::encode(2)), LINEARS_OK);
    ASSERT_EQ(A::reset(&solo, A::encode(0), A::encode(2)), LINEARS_OK);
    ASSERT_EQ(A::reset(&second, A::encode(100), A::encode(0.5L)), LINEARS_OK);
    ScalarReference reference{0, 2};
    std::mt19937 random(0x4b414c32u);
    std::uniform_real_distribution<double> noise(-0.25, 0.25);
    for (int i = 0; i < 6000; ++i) {
        SCOPED_TRACE(i);
        const auto z = A::encode(2 * std::sin(i * 0.013L) + noise(random));
        ASSERT_EQ(A::predict(&first, a, q), LINEARS_OK);
        ASSERT_EQ(A::predict(&second, A::encode(1), q), LINEARS_OK);
        ASSERT_EQ(A::update(&first, z, h, r), LINEARS_OK);
        ASSERT_EQ(A::update(&second, A::encode(-2), A::encode(0), A::encode(1)), LINEARS_OK);
        ASSERT_EQ(A::predict(&solo, a, q), LINEARS_OK);
        ASSERT_EQ(A::update(&solo, z, h, r), LINEARS_OK);
        reference.predict(A::decode(a), A::decode(q));
        reference.update(A::decode(z), A::decode(h), A::decode(r));
        expect_near<A>(A::decode(first.estimate), reference.x, 2);
        expect_near<A>(A::decode(first.covariance), reference.p, 2);
        EXPECT_EQ(A::decode(first.estimate), A::decode(solo.estimate));
        EXPECT_EQ(A::decode(first.covariance), A::decode(solo.covariance));
        EXPECT_GE(A::decode(first.covariance), 0);
    }
    ASSERT_EQ(A::reset(&first, A::encode(0), A::encode(2)), LINEARS_OK);
    expect_near<A>(A::decode(first.estimate), 0);
    expect_near<A>(A::decode(first.covariance), 2);
    expect_near<A>(A::decode(second.estimate), 100);
}

TYPED_TEST(KalmanScalar, InvalidNoiseNullAndFailureAtomicity) {
    using A = TypeParam;
    typename A::State state{};
    const auto zero = A::encode(0), one = A::encode(1), negative = A::encode(-1);
    ASSERT_EQ(A::reset(&state, one, one), LINEARS_OK);
    const auto original = bytes(state);
    EXPECT_EQ(A::reset(nullptr, zero, one), LINEARS_ERROR_ARGUMENT);
    EXPECT_EQ(A::predict(nullptr, one, zero), LINEARS_ERROR_ARGUMENT);
    EXPECT_EQ(A::update(nullptr, zero, one, one), LINEARS_ERROR_ARGUMENT);
    EXPECT_EQ(A::reset(&state, zero, negative), LINEARS_ERROR_DOMAIN);
    EXPECT_EQ(bytes(state), original);
    EXPECT_EQ(A::predict(&state, one, negative), LINEARS_ERROR_DOMAIN);
    EXPECT_EQ(bytes(state), original);
    EXPECT_EQ(A::update(&state, zero, one, zero), LINEARS_ERROR_DOMAIN);
    EXPECT_EQ(bytes(state), original);
    EXPECT_EQ(A::update(&state, zero, one, negative), LINEARS_ERROR_DOMAIN);
    EXPECT_EQ(bytes(state), original);
    state.covariance = negative;
    const auto invalid = bytes(state);
    EXPECT_EQ(A::predict(&state, one, zero), LINEARS_ERROR_DOMAIN);
    EXPECT_EQ(bytes(state), invalid);
    EXPECT_EQ(A::update(&state, zero, one, one), LINEARS_ERROR_DOMAIN);
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
            EXPECT_EQ(A::predict(&state, one, invalid), LINEARS_ERROR_RANGE);
            EXPECT_EQ(A::update(&state, invalid, one, one), LINEARS_ERROR_RANGE);
            EXPECT_EQ(A::update(&state, one, invalid, one), LINEARS_ERROR_RANGE);
            EXPECT_EQ(A::update(&state, one, one, invalid), LINEARS_ERROR_RANGE);
            EXPECT_EQ(bytes(state), original);
        }
        const auto maximum = std::numeric_limits<typename A::Raw>::max();
        ASSERT_EQ(A::reset(&state, maximum, one), LINEARS_OK);
        const auto large = bytes(state);
        EXPECT_EQ(A::predict(&state, A::encode(2), zero), LINEARS_ERROR_RANGE);
        EXPECT_EQ(bytes(state), large);
        EXPECT_EQ(A::update(&state, -maximum, A::encode(2), one), LINEARS_ERROR_RANGE);
        EXPECT_EQ(bytes(state), large);
        ASSERT_EQ(A::reset(&state, zero, one), LINEARS_OK);
        const auto restored = bytes(state);
        EXPECT_EQ(A::predict(&state, maximum, zero), LINEARS_ERROR_RANGE);
        EXPECT_EQ(bytes(state), restored);
    } else {
        const auto original = bytes(state);
        auto invalid = zero;
        invalid.exponent = LINEARS_EXPONENT_MAX + 1;
        EXPECT_EQ(A::reset(&state, invalid, one), LINEARS_ERROR_RANGE);
        EXPECT_EQ(A::predict(&state, one, invalid), LINEARS_ERROR_RANGE);
        EXPECT_EQ(A::update(&state, one, one, invalid), LINEARS_ERROR_RANGE);
        EXPECT_EQ(bytes(state), original);
        invalid.exponent = LINEARS_EXPONENT_MIN - 1;
        EXPECT_EQ(A::reset(&state, one, invalid), LINEARS_ERROR_RANGE);
        EXPECT_EQ(A::predict(&state, invalid, zero), LINEARS_ERROR_RANGE);
        EXPECT_EQ(A::update(&state, invalid, one, one), LINEARS_ERROR_RANGE);
        EXPECT_EQ(A::update(&state, one, invalid, one), LINEARS_ERROR_RANGE);
        EXPECT_EQ(bytes(state), original);
        invalid.mantissa = 1;
        invalid.exponent = LINEARS_EXPONENT_MIN;
        EXPECT_EQ(A::reset(&state, invalid, one), LINEARS_ERROR_RANGE);
        EXPECT_EQ(bytes(state), original);
        auto largest = one;
        largest.exponent = LINEARS_EXPONENT_MAX;
        ASSERT_EQ(A::reset(&state, largest, zero), LINEARS_OK);
        const auto large = bytes(state);
        EXPECT_EQ(A::predict(&state, A::encode(2), zero), LINEARS_ERROR_RANGE);
        EXPECT_EQ(bytes(state), large);
        auto smallest = one;
        smallest.exponent = LINEARS_EXPONENT_MIN;
        ASSERT_EQ(A::reset(&state, zero, zero), LINEARS_OK);
        ASSERT_EQ(A::update(&state, zero, one, smallest), LINEARS_OK);
        EXPECT_EQ(A::decode(state.estimate), 0);
        EXPECT_EQ(A::decode(state.covariance), 0);
    }
}

} // namespace
