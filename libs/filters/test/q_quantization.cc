#include "../src/ukf_internal.h"
#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <cstring>
#include <initializer_list>
#include <limits>

namespace {
struct Q15 {
    using Raw = int16_t;
    using Work = float;
    using Bits = uint32_t;
    static constexpr int fractional_bits = 15;
    static constexpr unsigned mantissa_bits = 23, largest_exponent = 127;
    static constexpr auto store = ukf_store_q15;
    static constexpr auto error = ukf_quantization_error_q15;
    static constexpr auto abs_greater = ukf_abs_greater_f32;
};
struct Q31 {
    using Raw = int32_t;
    using Work = double;
    using Bits = uint64_t;
    static constexpr int fractional_bits = 31;
    static constexpr unsigned mantissa_bits = 52, largest_exponent = 1023;
    static constexpr auto store = ukf_store_q31;
    static constexpr auto error = ukf_quantization_error_q31;
    static constexpr auto abs_greater = ukf_abs_greater_f64;
};

template<class F> ::testing::AssertionResult exact_error(typename F::Work value) {
    using W = typename F::Work;
    using B = typename F::Bits;
    typename F::Raw raw;
    F::store(&value, 1, &raw);
    // A power-of-two decode is exact in the matching native working format.
    const W expected = std::fabs(value - std::ldexp(W(raw), -F::fractional_bits));
    const W actual = F::error(value, raw);
    B want, got;
    std::memcpy(&want, &expected, sizeof(want));
    std::memcpy(&got, &actual, sizeof(got));
    if (want == got) return ::testing::AssertionSuccess();
    return ::testing::AssertionFailure() << "value=" << value << " raw=" << raw
        << " expected bits=" << want << " actual bits=" << got;
}

uint64_t next_bits(uint64_t &state) {
    state ^= state << 13;
    state ^= state >> 7;
    state ^= state << 17;
    return state;
}

template<class F> class QQuantizationError : public ::testing::Test {};
using Formats = ::testing::Types<Q15, Q31>;
TYPED_TEST_SUITE(QQuantizationError, Formats);

TYPED_TEST(QQuantizationError, CellsTiesAndNeighborsRetainNativeErrorBits) {
    using F = TypeParam;
    using W = typename F::Work;
    uint64_t random = UINT64_C(0x1b932ad41cf24603);
    const unsigned count = F::fractional_bits == 15 ? 65537 : 100000;
    for (unsigned i = 0; i < count; ++i) {
        int64_t cell;
        if constexpr (F::fractional_bits == 15) cell = int64_t(i) - 32768;
        else cell = int64_t(next_bits(random) & UINT32_MAX) - INT64_C(2147483648);
        for (W offset : {W(-0.5), W(-0.25), W(0), W(0.25), W(0.5)}) {
            W value = std::ldexp(W(cell) + offset, -F::fractional_bits);
            if (value < W(-1) || value > W(1)) continue;
            ASSERT_TRUE(exact_error<F>(value));
            W down = std::nextafter(value, -std::numeric_limits<W>::infinity());
            W up = std::nextafter(value, std::numeric_limits<W>::infinity());
            if (down >= W(-1)) ASSERT_TRUE(exact_error<F>(down));
            if (up <= W(1)) ASSERT_TRUE(exact_error<F>(up));
        }
    }
    for (W value : {W(-1), W(1), W(-0.0), W(0.0), std::numeric_limits<W>::denorm_min(),
                    -std::numeric_limits<W>::denorm_min()})
        ASSERT_TRUE(exact_error<F>(value));
}

TYPED_TEST(QQuantizationError, EveryUnitRangeExponentRetainsNativeErrorBits) {
    using F = TypeParam;
    using W = typename F::Work;
    using B = typename F::Bits;
    if constexpr (!std::numeric_limits<W>::is_iec559 || sizeof(W) != sizeof(B)) {
        GTEST_SKIP() << "IEEE bit-pattern fixture requires matching native storage";
    } else {
        constexpr B fraction_mask = (B(1) << F::mantissa_bits) - 1;
        constexpr B sign_bit = B(1) << (sizeof(B)*8-1);
        for (unsigned sign = 0; sign < 2; ++sign) {
            for (unsigned exponent = 0; exponent <= F::largest_exponent; ++exponent) {
                for (B fraction : {B(0), B(1), fraction_mask / 2, B(1) << (F::mantissa_bits-1),
                                   fraction_mask-1, fraction_mask}) {
                    if (exponent == F::largest_exponent && fraction != 0) continue;
                    B bits = (sign ? sign_bit : 0) | (B(exponent) << F::mantissa_bits) | fraction;
                    W value;
                    std::memcpy(&value, &bits, sizeof(value));
                    ASSERT_TRUE(exact_error<F>(value));
                }
            }
        }
    }
}

TYPED_TEST(QQuantizationError, MagnitudeComparisonMatchesNativeForFiniteOperands) {
    using F = TypeParam;
    using W = typename F::Work;
    using B = typename F::Bits;
    if constexpr (!std::numeric_limits<W>::is_iec559 || sizeof(W) != sizeof(B)) {
        GTEST_SKIP() << "IEEE bit-pattern fixture requires matching native storage";
    } else {
        uint64_t random = UINT64_C(0x91e10da5c79e7b1d);
        for (unsigned i = 0; i < 100000; ++i) {
            B a_bits = B(next_bits(random)), b_bits = B(next_bits(random));
            W a, b;
            std::memcpy(&a, &a_bits, sizeof(a));
            std::memcpy(&b, &b_bits, sizeof(b));
            if (!std::isfinite(a) || !std::isfinite(b)) continue;
            ASSERT_EQ(F::abs_greater(a, b) != 0, std::fabs(a) > std::fabs(b));
        }
        const W zero = W(0), tiny = std::numeric_limits<W>::denorm_min();
        for (W a : {zero, -zero, tiny, -tiny, W(1), W(-1), std::numeric_limits<W>::max()})
            for (W b : {zero, -zero, tiny, -tiny, W(1), W(-1), std::numeric_limits<W>::max()})
                EXPECT_EQ(F::abs_greater(a, b) != 0, std::fabs(a) > std::fabs(b));
    }
}
} // namespace
