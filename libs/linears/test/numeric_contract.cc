#include "linears_numeric.h"

#include <gtest/gtest.h>
#include <climits>
#include <cstdint>
#include <limits>
#include <type_traits>
#include <utility>

namespace {
__extension__ typedef __int128 Wide;
__extension__ typedef unsigned __int128 UWide;

template<class T, class = void> struct HasExponent : std::false_type {};
template<class T> struct HasExponent<T, std::void_t<decltype(std::declval<T>().exponent)>> : std::true_type {};

Wide nearest(Wide numerator, Wide denominator) {
    if (denominator < 0) { numerator = -numerator; denominator = -denominator; }
    bool negative = numerator < 0;
    if (negative) numerator = -numerator;
    Wide quotient = numerator / denominator, remainder = numerator % denominator;
    quotient += remainder * 2 >= denominator;
    return negative ? -quotient : quotient;
}

void expect_work(linears_status_t status, int64_t actual, Wide expected, int64_t sentinel = 99) {
    if (expected < INT64_MIN || expected > INT64_MAX) {
        EXPECT_EQ(status, LINEARS_ERROR_RANGE);
        EXPECT_EQ(actual, sentinel);
    } else {
        EXPECT_EQ(status, LINEARS_OK);
        EXPECT_EQ(actual, static_cast<int64_t>(expected));
    }
}

uint64_t next(uint64_t &state) {
    state ^= state << 13;
    state ^= state >> 7;
    state ^= state << 17;
    return state;
}

linears_accum_t encoded(UWide raw) {
    return {static_cast<uint64_t>(raw), static_cast<uint64_t>(raw >> 64)};
}
}

TEST(LinearsNumericContract, RawLayoutsAndExactConversion) {
    static_assert(std::is_same_v<linears_q15_t, int16_t> && sizeof(linears_q15_t) == 2);
    static_assert(std::is_same_v<linears_q31_t, int32_t> && sizeof(linears_q31_t) == 4);
    static_assert(std::is_same_v<linears_fixed_t, int64_t> && sizeof(linears_fixed_t) == 8);
    static_assert(!HasExponent<linears_matrix_t>::value);
    static_assert(sizeof(linears_matrix_t) == sizeof(void *) + 2 * sizeof(size_t));
    static_assert(sizeof(linears_workspace_unit_t) == 8);
    for (int raw = INT16_MIN; raw <= INT16_MAX; ++raw)
        EXPECT_EQ(linears_fixed_from_q15(static_cast<int16_t>(raw)), static_cast<int64_t>(raw) * 65536);
    for (int32_t raw : {INT32_MIN, -1, 0, 1, INT32_MAX}) EXPECT_EQ(linears_fixed_from_q31(raw), raw);
    EXPECT_EQ(linears_fixed_compare(INT64_MIN, INT64_MAX), -1);
    EXPECT_EQ(linears_fixed_compare(INT64_MAX, INT64_MIN), 1);
}

TEST(LinearsNumericContract, IntegerImportAndOverflowAreAtomic) {
    linears_fixed_t out = 99;
    ASSERT_EQ(linears_fixed_from_int(-INT64_C(4294967296), &out), LINEARS_OK);
    EXPECT_EQ(out, INT64_MIN);
    ASSERT_EQ(linears_fixed_from_int(INT64_C(4294967295), &out), LINEARS_OK);
    EXPECT_EQ(out, INT64_MAX - LINEARS_FIXED_ONE + 1);
    out = 99;
    EXPECT_EQ(linears_fixed_from_int(INT64_C(4294967296), &out), LINEARS_ERROR_RANGE);
    EXPECT_EQ(linears_fixed_add(INT64_MAX, 1, &out), LINEARS_ERROR_RANGE);
    EXPECT_EQ(linears_fixed_sub(INT64_MIN, 1, &out), LINEARS_ERROR_RANGE);
    EXPECT_EQ(linears_fixed_abs(INT64_MIN, &out), LINEARS_ERROR_RANGE);
    EXPECT_EQ(linears_fixed_neg(INT64_MIN, &out), LINEARS_ERROR_RANGE);
    EXPECT_EQ(linears_fixed_div(1, 0, &out), LINEARS_ERROR_DOMAIN);
    EXPECT_EQ(linears_fixed_sqrt(-1, &out), LINEARS_ERROR_DOMAIN);
    EXPECT_EQ(out, 99);
}

TEST(LinearsNumericContract, TiesAwayAndPositiveEndpointAreExplicit) {
    linears_fixed_t work = 99;
    ASSERT_EQ(linears_fixed_mul(1, LINEARS_FIXED_ONE / 2, &work), LINEARS_OK);
    EXPECT_EQ(work, 1);
    ASSERT_EQ(linears_fixed_mul(-1, LINEARS_FIXED_ONE / 2, &work), LINEARS_OK);
    EXPECT_EQ(work, -1);
    ASSERT_EQ(linears_fixed_mul(1, LINEARS_FIXED_ONE / 2 - 1, &work), LINEARS_OK);
    EXPECT_EQ(work, 0);
    int16_t q15 = 55;
    ASSERT_EQ(linears_fixed_to_q15(32768, &q15), LINEARS_OK); EXPECT_EQ(q15, 1);
    ASSERT_EQ(linears_fixed_to_q15(-32768, &q15), LINEARS_OK); EXPECT_EQ(q15, -1);
    for (int64_t value : {LINEARS_FIXED_ONE - 32768, LINEARS_FIXED_ONE - 1, LINEARS_FIXED_ONE}) {
        ASSERT_EQ(linears_fixed_to_q15(value, &q15), LINEARS_OK); EXPECT_EQ(q15, INT16_MAX);
    }
    ASSERT_EQ(linears_fixed_to_q15(-LINEARS_FIXED_ONE, &q15), LINEARS_OK); EXPECT_EQ(q15, INT16_MIN);
    EXPECT_EQ(linears_fixed_to_q15(LINEARS_FIXED_ONE + 1, &q15), LINEARS_ERROR_RANGE);
    EXPECT_EQ(q15, INT16_MIN);
    int32_t q31 = 0;
    ASSERT_EQ(linears_fixed_to_q31(LINEARS_FIXED_ONE, &q31), LINEARS_OK); EXPECT_EQ(q31, INT32_MAX);
    EXPECT_EQ(linears_fixed_to_q31(LINEARS_FIXED_ONE + 1, &q31), LINEARS_ERROR_RANGE);
    EXPECT_EQ(q31, INT32_MAX);
}

TEST(LinearsNumericContract, FullWidthArithmeticMatchesExactIntegerOracle) {
    uint64_t state = UINT64_C(0xd7c634a582b917f3);
    for (unsigned i = 0; i < 5000; ++i) {
        int64_t a = static_cast<int64_t>(next(state) & INT64_MAX);
        int64_t b = static_cast<int64_t>(next(state) & INT64_MAX);
        if (i & 1) a = -a;
        if (i & 2) b = -b;
        if (i % 3 == 0) { a /= INT64_C(4294967296); b /= INT64_C(4294967296); }
        int64_t out = 99;
        auto status = linears_fixed_mul(a, b, &out);
        expect_work(status, out, nearest(Wide(a) * b, LINEARS_FIXED_ONE));
        if (b != 0) {
            out = 99;
            status = linears_fixed_div(a, b, &out);
            expect_work(status, out, nearest(Wide(a) * LINEARS_FIXED_ONE, b));
            out = 99;
            status = linears_fixed_muldiv(a, b, b, &out);
            expect_work(status, out, a);
        }
        out = 99;
        status = linears_fixed_add(a, b, &out);
        expect_work(status, out, Wide(a) + b);
        out = 99;
        status = linears_fixed_sub(a, b, &out);
        expect_work(status, out, Wide(a) - b);
    }
    int64_t out = 99;
    ASSERT_EQ(linears_fixed_muldiv(INT64_MIN, INT64_MIN, INT64_MIN, &out), LINEARS_OK);
    EXPECT_EQ(out, INT64_MIN);
}

TEST(LinearsNumericContract, IntegerSquareRootIncludesTinyAndWideValues) {
    uint64_t state = 0x814d2a5u;
    for (unsigned i = 0; i < 1000; ++i) {
        int64_t input = i == 0 ? 0 : i == 1 ? 1 : i == 2 ? INT64_MAX : static_cast<int64_t>(next(state) & INT64_MAX);
        UWide radicand = UWide(input) << 31;
        uint64_t low = 0, high = UINT64_C(1) << 48;
        while (low + 1 < high) {
            uint64_t middle = low + (high - low) / 2;
            if (UWide(middle) * middle <= radicand) low = middle;
            else high = middle;
        }
        uint64_t expected = low + (radicand - UWide(low) * low >= low + 1);
        linears_fixed_t out = -1;
        ASSERT_EQ(linears_fixed_sqrt(input, &out), LINEARS_OK);
        EXPECT_EQ(out, static_cast<int64_t>(expected));
    }
}

TEST(LinearsNumericContract, SignedWideCarryOverflowAndCancellation) {
    linears_accum_t acc{UINT64_MAX, 0};
    ASSERT_EQ(linears_fixed_mac(&acc, 1, 1), LINEARS_OK);
    EXPECT_EQ(acc.low, 0u); EXPECT_EQ(acc.high, 1u);
    ASSERT_EQ(linears_fixed_msu(&acc, 1, 1), LINEARS_OK);
    EXPECT_EQ(acc.low, UINT64_MAX); EXPECT_EQ(acc.high, 0u);
    acc = {0, 0};
    ASSERT_EQ(linears_fixed_mac(&acc, INT64_MIN, INT64_MIN), LINEARS_OK);
    ASSERT_EQ(linears_fixed_msu(&acc, INT64_MIN, INT64_MIN), LINEARS_OK);
    EXPECT_EQ(acc.low, 0u); EXPECT_EQ(acc.high, 0u);
    acc = {UINT64_MAX, static_cast<uint64_t>(INT64_MAX)};
    EXPECT_EQ(linears_fixed_mac(&acc, 1, 1), LINEARS_ERROR_RANGE);
    EXPECT_EQ(acc.low, UINT64_MAX); EXPECT_EQ(acc.high, static_cast<uint64_t>(INT64_MAX));
    acc = {0, UINT64_C(1) << 63};
    EXPECT_EQ(linears_fixed_msu(&acc, 1, 1), LINEARS_ERROR_RANGE);
    EXPECT_EQ(acc.low, 0u); EXPECT_EQ(acc.high, UINT64_C(1) << 63);
}

TEST(LinearsNumericContract, ExactAccumulatorRangePrecedesRounding) {
    int32_t out = 77;
    ASSERT_EQ(linears_accum_to_q31({UINT64_C(1) << 62, 0}, &out), LINEARS_OK);
    EXPECT_EQ(out, INT32_MAX);
    EXPECT_EQ(linears_accum_to_q31({(UINT64_C(1) << 62) + 1, 0}, &out), LINEARS_ERROR_RANGE);
    EXPECT_EQ(out, INT32_MAX);
    linears_accum_t two_halves{UINT64_C(1) << 31, 0};
    ASSERT_EQ(linears_accum_to_q31(two_halves, &out), LINEARS_OK);
    EXPECT_EQ(out, 1);
}

TEST(LinearsNumericContract, MeanDividesUnsignedCountWithoutIntermediateNarrowing) {
    for (uint64_t count : {UINT64_C(1), UINT64_C(3), UINT64_C(0x8000000000000000), UINT64_MAX}) {
        UWide bound = UWide(count) << 62;
        int16_t q15 = 99; int32_t q31 = 99;
        ASSERT_EQ(linears_accum_mean_q15(encoded(bound), count, &q15), LINEARS_OK);
        ASSERT_EQ(linears_accum_mean_q31(encoded(bound), count, &q31), LINEARS_OK);
        EXPECT_EQ(q15, INT16_MAX); EXPECT_EQ(q31, INT32_MAX);
        EXPECT_EQ(linears_accum_mean_q15(encoded(bound + 1), count, &q15), LINEARS_ERROR_RANGE);
        EXPECT_EQ(linears_accum_mean_q31(encoded(bound + 1), count, &q31), LINEARS_ERROR_RANGE);
        EXPECT_EQ(q15, INT16_MAX); EXPECT_EQ(q31, INT32_MAX);
        ASSERT_EQ(linears_accum_mean_q31(encoded(UWide(3) * count << 30), count, &q31), LINEARS_OK);
        EXPECT_EQ(q31, 2); // Exact 1.5 raw signal units, nearest away.
    }
    int32_t out = 99;
    EXPECT_EQ(linears_accum_mean_q31({0, 0}, 0, &out), LINEARS_ERROR_DOMAIN);
    EXPECT_EQ(out, 99);
}

TEST(LinearsNumericContract, ExportIsAtomicAlignedAndDisjoint) {
    const linears_fixed_t values[] = {LINEARS_FIXED_ONE / 2, -LINEARS_FIXED_ONE, LINEARS_FIXED_ONE};
    int16_t q15[] = {11, 22, 33};
    ASSERT_EQ(linears_fixed_export_q15(values, 3, q15), LINEARS_OK);
    EXPECT_EQ(q15[0], 16384); EXPECT_EQ(q15[1], INT16_MIN); EXPECT_EQ(q15[2], INT16_MAX);
    const linears_fixed_t bad[] = {0, 0, LINEARS_FIXED_ONE + 1};
    EXPECT_EQ(linears_fixed_export_q15(bad, 3, q15), LINEARS_ERROR_RANGE);
    EXPECT_EQ(q15[0], 16384); EXPECT_EQ(q15[2], INT16_MAX);
    EXPECT_EQ(linears_fixed_export_q15(values, SIZE_MAX, q15), LINEARS_ERROR_SIZE);
    EXPECT_EQ(linears_fixed_export_q31(nullptr, 0, nullptr), LINEARS_OK);
    linears_fixed_t same[2]{};
    EXPECT_EQ(linears_fixed_export_q31(same, 2, reinterpret_cast<int32_t *>(same)), LINEARS_ERROR_ALIAS);
}

TEST(LinearsNumericContract, WorkspaceBoundsRemainConstantExpressions) {
    static_assert(LINEARS_BASIC_WORKSPACE_BYTES(2, 3) == 48);
    static_assert(LINEARS_BASIC_WORKSPACE_BYTES(0, SIZE_MAX) == 0);
    static_assert(LINEARS_BASIC_WORKSPACE_BYTES(SIZE_MAX, 2) == SIZE_MAX);
    static_assert(LINEARS_SIZE_ADD(SIZE_MAX, 1) == SIZE_MAX);
    static_assert(LINEARS_WORKSPACE_UNITS(SIZE_MAX) == SIZE_MAX);
    static_assert(LINEARS_WORKSPACE_UNITS(1) == 1);
}
