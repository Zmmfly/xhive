#include "../src/q_integer_bits.h"

#include <gtest/gtest.h>

#include <cstdint>
#include <limits>
#include <random>

namespace {

unsigned reference_bit_length(uint64_t value) {
    unsigned result=0;
    while (value) {value>>=1;++result;}
    return result;
}

TEST(QIntegerBits, ZeroAndEveryWordBoundary) {
    EXPECT_EQ(filters_q_bit_length(0),0u);
    EXPECT_EQ(filters_q_bit_length(std::numeric_limits<uint64_t>::max()),64u);
    for (unsigned bit=0;bit<64;++bit) {
        const uint64_t one=UINT64_C(1)<<bit;
        EXPECT_EQ(filters_q_bit_length(one),bit+1);
        EXPECT_EQ(filters_q_bit_length(one-1),bit);
        EXPECT_EQ(filters_q_bit_length(one+1),reference_bit_length(one+1));
    }
}

TEST(QIntegerBits, DenseHalfWordsAndRandomMagnitudesMatchTheLoop) {
    for (uint64_t value=0;value<=UINT16_MAX;++value) {
        for (unsigned shift:{0u,32u,48u}) {
            const uint64_t input=value<<shift;
            ASSERT_EQ(filters_q_bit_length(input),reference_bit_length(input)) << input;
        }
    }
    std::mt19937_64 random(0x514249544c454eULL);
    for (unsigned i=0;i<100000;++i) {
        const uint64_t input=random();
        ASSERT_EQ(filters_q_bit_length(input),reference_bit_length(input)) << input;
    }
}

} // namespace
