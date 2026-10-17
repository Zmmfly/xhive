#include "linears_numeric.h"

#include <gtest/gtest.h>
#include <climits>
#include <cmath>
#include <cstdint>
#include <limits>

namespace {
long double decode(linears_fixed_t value) {
    return std::ldexp(static_cast<long double>(value.mantissa), value.exponent - 31);
}

void expect_value(linears_fixed_t actual, long double expected) {
    ASSERT_TRUE(linears_fixed_valid(actual));
    long double error = std::fabs(decode(actual) - expected);
    long double half_ulp = std::ldexp(0.5L, actual.exponent - 31);
    EXPECT_LE(error, half_ulp);
}
}

TEST(LinearsNumericContract, ExactImportAndCanonicalZero) {
    linears_fixed_t out{123, 456};
    ASSERT_EQ(linears_fixed_from_i64(1, 0, 0, &out), LINEARS_OK);
    EXPECT_EQ(out.mantissa, INT32_C(1073741824));
    EXPECT_EQ(out.exponent, 1);
    ASSERT_EQ(linears_fixed_from_q15({INT16_MIN, 3}, &out), LINEARS_OK);
    expect_value(out, -8);
    ASSERT_EQ(linears_fixed_from_q31({INT32_MIN, -2}, &out), LINEARS_OK);
    expect_value(out, -0.25L);
    ASSERT_EQ(linears_fixed_from_i64(INT64_MIN, 0, 0, &out), LINEARS_OK);
    expect_value(out, -std::ldexp(1.0L, 63));
    ASSERT_EQ(linears_fixed_from_i64(0, 63, LINEARS_EXPONENT_MAX, &out), LINEARS_OK);
    EXPECT_EQ(out.mantissa, 0);
    EXPECT_EQ(out.exponent, 0);
}

TEST(LinearsNumericContract, TiesRoundAwayAndNoSignedShift) {
    linears_fixed_t positive{}, negative{};
    // The exact magnitudes end halfway between adjacent 31-bit mantissas.
    ASSERT_EQ(linears_fixed_from_i64((INT64_C(1) << 31) + 1, 0, 0, &positive), LINEARS_OK);
    ASSERT_EQ(linears_fixed_from_i64(-((INT64_C(1) << 31) + 1), 0, 0, &negative), LINEARS_OK);
    EXPECT_EQ(positive.mantissa, INT32_C(1073741825));
    EXPECT_EQ(negative.mantissa, -INT32_C(1073741825));
    EXPECT_EQ(positive.exponent, 32);
    int16_t q15 = 123;
    ASSERT_EQ(linears_fixed_from_i64(1, 16, 0, &positive), LINEARS_OK);
    ASSERT_EQ(linears_fixed_neg(positive, &negative), LINEARS_OK);
    ASSERT_EQ(linears_fixed_to_q15(positive, 0, &q15), LINEARS_OK);
    EXPECT_EQ(q15, 1);
    ASSERT_EQ(linears_fixed_to_q15(negative, 0, &q15), LINEARS_OK);
    EXPECT_EQ(q15, -1);
    ASSERT_EQ(linears_fixed_to_q15(negative, LINEARS_EXPONENT_MAX, &q15), LINEARS_OK);
    EXPECT_EQ(q15, 0);
}

TEST(LinearsNumericContract, ExponentRangeAndDomainFailuresAreAtomic) {
    const linears_fixed_t sentinel{INT32_C(1234567890), 12};
    linears_fixed_t out = sentinel;
    EXPECT_EQ(linears_fixed_from_i64(1, -1, 0, &out), LINEARS_ERROR_ARGUMENT);
    EXPECT_EQ(linears_fixed_from_i64(1, 0, INT_MAX, &out), LINEARS_ERROR_RANGE);
    EXPECT_EQ(linears_fixed_from_q31({1, LINEARS_EXPONENT_MIN}, &out), LINEARS_ERROR_RANGE);
    const linears_fixed_t top{INT32_MAX, LINEARS_EXPONENT_MAX};
    EXPECT_EQ(linears_fixed_add(top, top, &out), LINEARS_ERROR_RANGE);
    EXPECT_EQ(linears_fixed_div(top, {0, 0}, &out), LINEARS_ERROR_DOMAIN);
    EXPECT_EQ(linears_fixed_sqrt({-INT32_C(1073741824), 0}, &out), LINEARS_ERROR_DOMAIN);
    EXPECT_EQ(linears_fixed_add({1, 0}, top, &out), LINEARS_ERROR_ARGUMENT);
    EXPECT_EQ(out.mantissa, sentinel.mantissa);
    EXPECT_EQ(out.exponent, sentinel.exponent);
    int32_t raw = 77;
    EXPECT_EQ(linears_fixed_to_q31(top, LINEARS_EXPONENT_MIN, &raw), LINEARS_ERROR_RANGE);
    EXPECT_EQ(raw, 77);
}

TEST(LinearsNumericContract, AlignmentAdditionAndOrdering) {
    for (int distance = 0; distance < 80; ++distance) {
        for (int sign_a : {-1, 1}) for (int sign_b : {-1, 1}) {
            linears_fixed_t a{sign_a * INT32_C(1610612735), 20};
            linears_fixed_t b{sign_b * INT32_C(2147483647), 20 - distance};
            linears_fixed_t result{};
            ASSERT_EQ(linears_fixed_add(a, b, &result), LINEARS_OK);
            expect_value(result, decode(a) + decode(b));
            ASSERT_EQ(linears_fixed_sub(a, b, &result), LINEARS_OK);
            expect_value(result, decode(a) - decode(b));
            int expected = (decode(a) > decode(b)) - (decode(a) < decode(b));
            EXPECT_EQ(linears_fixed_compare(a, b), expected);
        }
    }
    EXPECT_EQ(linears_fixed_compare({0, 0}, {-INT32_C(1073741824), -10}), 1);
    EXPECT_EQ(linears_fixed_compare({0, 0}, {INT32_C(1073741824), -10}), -1);
}

TEST(LinearsNumericContract, IntegerMultiplyDivideAndSquareRoot) {
    uint32_t state = 0x91827364u;
    for (unsigned i = 0; i < 4000; ++i) {
        state = state * UINT32_C(1664525) + UINT32_C(1013904223);
        linears_fixed_t a{static_cast<int32_t>((state >> 1) | UINT32_C(0x40000000)),
                          static_cast<int>(i % 61) - 30};
        state = state * UINT32_C(1664525) + UINT32_C(1013904223);
        linears_fixed_t b{static_cast<int32_t>((state >> 1) | UINT32_C(0x40000000)),
                          static_cast<int>((i * 17) % 61) - 30};
        if (state & 1) a.mantissa = -a.mantissa;
        linears_fixed_t result{};
        ASSERT_EQ(linears_fixed_mul(a, b, &result), LINEARS_OK);
        expect_value(result, decode(a) * decode(b));
        ASSERT_EQ(linears_fixed_div(a, b, &result), LINEARS_OK);
        expect_value(result, decode(a) / decode(b));
        ASSERT_EQ(linears_fixed_sqrt(b, &result), LINEARS_OK);
        expect_value(result, std::sqrt(decode(b)));
    }
}

TEST(LinearsNumericContract, BlockExportNormalizationAndAtomicity) {
    const linears_fixed_t values[] = {{INT32_MAX, 0}, {-INT32_C(1073741824), 0}, {0, 0}};
    int16_t q15[3] = {11, 22, 33};
    int exponent = 99;
    ASSERT_EQ(linears_fixed_export_q15(values, 3, q15, &exponent), LINEARS_OK);
    EXPECT_EQ(exponent, 1);
    EXPECT_EQ(q15[0], 16384);
    EXPECT_EQ(q15[1], -8192);
    EXPECT_EQ(q15[2], 0);
    const linears_fixed_t bad[] = {{INT32_C(1073741824), 2}, {1, 0}, {0, 0}};
    EXPECT_EQ(linears_fixed_export_q15(bad, 3, q15, &exponent), LINEARS_ERROR_ARGUMENT);
    EXPECT_EQ(exponent, 1);
    EXPECT_EQ(q15[0], 16384);
    const linears_fixed_t too_large[] = {{INT32_MAX, LINEARS_EXPONENT_MAX}};
    EXPECT_EQ(linears_fixed_export_q15(too_large, 1, q15, &exponent), LINEARS_ERROR_RANGE);
    EXPECT_EQ(q15[0], 16384);
    EXPECT_EQ(exponent, 1);
    ASSERT_EQ(linears_fixed_export_q31(nullptr, 0, nullptr, &exponent), LINEARS_OK);
    EXPECT_EQ(exponent, 0);
    EXPECT_EQ(linears_fixed_export_q15(values, SIZE_MAX, q15, &exponent), LINEARS_ERROR_SIZE);
}

TEST(LinearsNumericContract, WorkspaceBoundsAreConstantExpressions) {
    static_assert(LINEARS_BASIC_WORKSPACE_BYTES(2, 3) == 6 * sizeof(linears_workspace_unit_t));
    static_assert(LINEARS_BASIC_WORKSPACE_BYTES(0, SIZE_MAX) == 0);
    static_assert(LINEARS_BASIC_WORKSPACE_BYTES(SIZE_MAX, 2) == SIZE_MAX);
    static_assert(LINEARS_SIZE_ADD(SIZE_MAX, 1) == SIZE_MAX);
    static_assert(LINEARS_WORKSPACE_UNITS(SIZE_MAX) == SIZE_MAX);
    static_assert(LINEARS_WORKSPACE_UNITS(1) == 1);
}
