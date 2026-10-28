#include "../src/q_integer_wide.h"

#include <gtest/gtest.h>

#include <array>
#include <climits>
#include <cstdint>
#include <limits>
#include <random>

namespace {
using Native128 = unsigned __int128;

Native128 native(filters_q_u128_t value) {
    return (static_cast<Native128>(value.hi) << 64) | value.lo;
}

filters_q_u128_t limbs(Native128 value) {
    return {static_cast<uint64_t>(value), static_cast<uint64_t>(value >> 64)};
}

void expect_round(Native128 value, unsigned shift) {
    Native128 expected;
    bool inexact;
    if (shift == 0) {
        expected = value;
        inexact = false;
    } else if (shift >= 128) {
        expected = shift == 128 ? value >> 127 : 0;
        inexact = value != 0;
    } else {
        const Native128 remainder = value & ((Native128(1) << shift) - 1);
        expected = (value >> shift) + (remainder >= (Native128(1) << (shift - 1)));
        inexact = remainder != 0;
    }
    /* The shared primitive's callers prove that the rounded quotient fits.
     * Do not exercise inputs outside that explicit internal contract. */
    if (expected > std::numeric_limits<uint64_t>::max()) return;
    const auto actual = filters_q_round_u128(limbs(value), shift);
    EXPECT_EQ(actual.value, static_cast<uint64_t>(expected)) << shift;
    EXPECT_EQ(actual.inexact != 0, inexact) << shift;
}

TEST(QIntegerWide, MultiplyAndBitLengthMatchNative128) {
    const std::array<uint64_t, 8> edges{
        0, 1, 2, UINT32_MAX, UINT64_C(1) << 32,
        (UINT64_C(1) << 63) - 1, UINT64_C(1) << 63, UINT64_MAX};
    for (uint64_t a : edges)
        for (uint64_t b : edges)
            EXPECT_EQ(native(filters_q_multiply_u64(a, b)), static_cast<Native128>(a) * b);
    std::mt19937_64 random(0x128640031u);
    for (unsigned i = 0; i < 100000; ++i) {
        const uint64_t a = random(), b = random();
        const auto product = filters_q_multiply_u64(a, b);
        ASSERT_EQ(native(product), static_cast<Native128>(a) * b) << i;
        Native128 remaining = native(product);
        unsigned bits = 0;
        for (; remaining != 0; remaining >>= 1) ++bits;
        ASSERT_EQ(filters_q_u128_bit_length(product), bits) << i;
    }
    EXPECT_EQ(filters_q_u128_bit_length({0, 0}), 0u);
    for (unsigned bit = 0; bit < 128; ++bit) {
        const Native128 power = Native128(1) << bit;
        EXPECT_EQ(filters_q_u128_bit_length(limbs(power)), bit + 1);
        EXPECT_EQ(filters_q_u128_bit_length(limbs(power - 1)), bit);
        if (bit != 0) EXPECT_EQ(filters_q_u128_bit_length(limbs(power + 1)), bit + 1);
    }
}

TEST(QIntegerWide, RoundedShiftCoversAllLimbAndDiscardBoundaries) {
    const std::array<unsigned, 10> wide_shifts{0, 1, 63, 64, 65, 127, 128, 129, 4096, UINT_MAX};
    const Native128 maximum = ~Native128(0);
    for (unsigned shift : wide_shifts) {
        expect_round(0, shift);
        expect_round(1, shift);
        expect_round(maximum, shift);
    }
    for (unsigned bit = 0; bit < 128; ++bit) {
        const Native128 power = Native128(1) << bit;
        for (unsigned shift = 0; shift <= 130; ++shift) {
            expect_round(power - 1, shift);
            expect_round(power, shift);
            expect_round(power + 1, shift);
        }
    }
    for (unsigned shift = 1; shift < 128; ++shift) {
        const Native128 half = Native128(1) << (shift - 1);
        expect_round(half - 1, shift);
        expect_round(half, shift);
        expect_round(half + 1, shift);
    }
    std::mt19937_64 random(0x5348494654u);
    for (unsigned i = 0; i < 100000; ++i) {
        const Native128 value = (static_cast<Native128>(random()) << 64) | random();
        expect_round(value, static_cast<unsigned>(random() % 257));
    }
    /* A rounded zero can still have discarded information: certificates must
     * not infer exact input conversion merely from the rounded value. */
    const auto tiny = filters_q_round_u128({1, 0}, 128);
    EXPECT_EQ(tiny.value, 0u);
    EXPECT_EQ(tiny.inexact, 1u);
    const auto zero = filters_q_round_u128({0, 0}, UINT_MAX);
    EXPECT_EQ(zero.value, 0u);
    EXPECT_EQ(zero.inexact, 0u);
}
} // namespace
