#include "linears_numeric.h"
#include "linears_work.h"

#include <gtest/gtest.h>

#include <array>
#include <climits>
#include <cstdint>
#include <cstring>
#include <limits>
#include <random>
#include <type_traits>

namespace {

// Test-only native 128-bit arithmetic is independent of the production two-limb
// implementation. The Linux/GCC-or-Clang host oracle is not an ARM dependency.
#if !defined(__SIZEOF_INT128__)
#error "This independent integer oracle requires host compiler __int128 support"
#endif
__extension__ typedef __int128 Wide;
__extension__ typedef unsigned __int128 UWide;
constexpr int64_t kOne = INT64_C(2147483648);
constexpr int64_t kSentinel = INT64_C(0x123456789abcdef);
constexpr uint64_t kGuard = UINT64_C(0xa517cb392ea06d84);
constexpr Wide kWideMax = static_cast<Wide>((UWide{1} << 127) - 1);
constexpr Wide kWideMin = -kWideMax - 1;

UWide Magnitude(Wide value) {
    return value < 0 ? static_cast<UWide>(-(value + 1)) + 1 : static_cast<UWide>(value);
}

Wide SignedMagnitude(UWide magnitude, bool negative) {
    if (!negative || magnitude == 0) return static_cast<Wide>(magnitude);
    return -static_cast<Wide>(magnitude - 1) - 1;
}

Wide RoundRatio(Wide numerator, Wide denominator) {
    const bool negative = (numerator < 0) != (denominator < 0);
    const UWide n = Magnitude(numerator), d = Magnitude(denominator);
    UWide quotient = n / d;
    const UWide remainder = n % d;
    if (remainder >= d / 2 + d % 2) ++quotient;
    return SignedMagnitude(quotient, negative);
}

linears_accum_t EncodeAccum(Wide value) {
    const auto bits = static_cast<UWide>(value);
    return {static_cast<uint64_t>(bits), static_cast<uint64_t>(bits >> 64)};
}

Wide DecodeAccum(linears_accum_t value) {
    const UWide bits = (static_cast<UWide>(value.high) << 64) | value.low;
    if ((value.high >> 63) == 0) return static_cast<Wide>(bits);
    return -static_cast<Wide>(~bits) - 1;
}

bool SameAccum(linears_accum_t a, linears_accum_t b) {
    return a.low == b.low && a.high == b.high;
}

int64_t RandomRaw(std::mt19937_64 &engine) {
    const uint64_t bits = engine();
    return bits <= INT64_MAX ? static_cast<int64_t>(bits)
                             : -static_cast<int64_t>(~bits) - 1;
}

::testing::AssertionResult MatchesFixed(linears_status_t status, int64_t actual,
                                        Wide expected, int64_t before = kSentinel) {
    const bool fits = expected >= INT64_MIN && expected <= INT64_MAX;
    const linears_status_t wanted = fits ? LINEARS_OK : LINEARS_ERROR_RANGE;
    const int64_t raw = fits ? static_cast<int64_t>(expected) : before;
    if (status == wanted && actual == raw) return ::testing::AssertionSuccess();
    return ::testing::AssertionFailure() << "status=" << status << ", raw=" << actual
        << "; expected status=" << wanted << ", raw=" << raw;
}

struct SignalResult { linears_status_t status; int64_t raw; };
SignalResult SignalReference(Wide value, int fraction, int bits, int64_t before) {
    const Wide one = Wide{1} << fraction;
    if (value < -one || value > one) return {LINEARS_ERROR_RANGE, before};
    Wide rounded = RoundRatio(value, Wide{1} << (fraction - bits));
    const Wide maximum = (Wide{1} << bits) - 1;
    if (rounded > maximum) rounded = maximum;
    return {LINEARS_OK, static_cast<int64_t>(rounded)};
}

SignalResult MeanReference(Wide sum, uint64_t count, int bits, int64_t before) {
    if (count == 0) return {LINEARS_ERROR_DOMAIN, before};
    const Wide bound = static_cast<Wide>(count) * (Wide{1} << 62);
    if (sum < -bound || sum > bound) return {LINEARS_ERROR_RANGE, before};
    const Wide divisor = static_cast<Wide>(count) * (Wide{1} << (62 - bits));
    Wide raw = RoundRatio(sum, divisor);
    const Wide maximum = (Wide{1} << bits) - 1;
    if (raw > maximum) raw = maximum;
    return {LINEARS_OK, static_cast<int64_t>(raw)};
}

int64_t SqrtReference(int64_t raw) {
    const UWide radicand = static_cast<UWide>(raw) * static_cast<uint64_t>(kOne);
    uint64_t low = 0, high = UINT64_C(1) << 48;
    // Binary search, not the production restoring-square-root recurrence.
    while (high - low > 1) {
        const uint64_t middle = low + (high - low) / 2;
        if (static_cast<UWide>(middle) * middle <= radicand) low = middle;
        else high = middle;
    }
    const UWide midpoint_twice = static_cast<UWide>(low) * 2 + 1;
    return static_cast<int64_t>(low + (4 * radicand >= midpoint_twice * midpoint_twice));
}

template<class Operation>
void AtomicFailure(linears_status_t expected, Operation operation) {
    struct { uint64_t before; int64_t output; uint64_t after; } value{kGuard, kSentinel, kGuard};
    EXPECT_EQ(operation(&value.output), expected);
    EXPECT_EQ(value.output, kSentinel);
    EXPECT_EQ(value.before, kGuard);
    EXPECT_EQ(value.after, kGuard);
}

using Binary = linears_status_t (*)(int64_t, int64_t, int64_t *);
constexpr std::array<Binary, 4> kBinary{linears_fixed_add, linears_fixed_sub,
                                      linears_fixed_mul, linears_fixed_div};
constexpr std::array<int64_t, 21> kEdges{
    INT64_MIN, INT64_MIN + 1, -INT64_C(4294967297), -INT64_C(4294967296), -INT64_C(4294967295),
    -kOne - 1, -kOne, -kOne + 1, -3, -1, 0, 1, 3,
    kOne - 1, kOne, kOne + 1, INT64_C(4294967295), INT64_C(4294967296), INT64_C(4294967297), INT64_MAX - 1, INT64_MAX};

TEST(AuditFixedRepresentation, RawTypesAndInvariantBinaryPoint) {
    static_assert(std::is_same<linears_q15_t, int16_t>::value, "q15 is raw F15");
    static_assert(std::is_same<linears_q31_t, int32_t>::value, "q31 is raw F31");
    static_assert(std::is_same<linears_fixed_t, int64_t>::value, "working format is raw F31");
    static_assert(sizeof(linears_workspace_unit_t) == sizeof(int64_t), "basic scratch must remain eight bytes per output");
    EXPECT_EQ(LINEARS_FIXED_ONE, kOne);
    linears_matrix_t matrix{nullptr, 0, 0};
    EXPECT_EQ(matrix.data, nullptr);
    EXPECT_EQ(matrix.rows, size_t{0}); EXPECT_EQ(matrix.cols, size_t{0});
}

TEST(AuditFixedImport, EveryInt16RawCodeRoundTripsExactly) {
    for (int raw = INT16_MIN; raw <= INT16_MAX; ++raw) {
        const int64_t working = linears_fixed_from_q15(static_cast<int16_t>(raw));
        ASSERT_EQ(working, static_cast<int64_t>(raw) * 65536) << raw;
        int16_t restored = 123;
        ASSERT_EQ(linears_fixed_to_q15(working, &restored), LINEARS_OK);
        ASSERT_EQ(restored, raw);
    }
}

TEST(AuditFixedImport, Q31SignExtensionAndBoundariesRoundTrip) {
    constexpr std::array<int32_t, 13> samples{INT32_MIN, INT32_MIN + 1, -1073741825,
        -1073741824, -3, -1, 0, 1, 3, 1073741824, 1073741825, INT32_MAX - 1, INT32_MAX};
    for (const int32_t raw : samples) {
        EXPECT_EQ(linears_fixed_from_q31(raw), static_cast<int64_t>(raw));
        int32_t restored = 123;
        ASSERT_EQ(linears_fixed_to_q31(linears_fixed_from_q31(raw), &restored), LINEARS_OK);
        EXPECT_EQ(restored, raw);
    }
}

TEST(AuditFixedImport, IntegerRangeIsDistinctFromSignalRange) {
    constexpr std::array<int64_t, 12> integers{INT64_MIN, -INT64_C(4294967297),
        -INT64_C(4294967296), -INT64_C(4294967295), -2, -1, 0, 1, 2,
        INT64_C(4294967295), INT64_C(4294967296), INT64_MAX};
    for (const auto integer : integers) {
        int64_t result = kSentinel;
        const auto status = linears_fixed_from_int(integer, &result);
        ASSERT_TRUE(MatchesFixed(status, result, static_cast<Wide>(integer) * kOne));
    }
    EXPECT_EQ(linears_fixed_from_int(0, nullptr), LINEARS_ERROR_ARGUMENT);
}

TEST(AuditFixedAddSubtract, ExactFullWidthAndWorkingOverflow) {
    for (const auto a : kEdges) for (const auto b : kEdges) {
        int64_t result = kSentinel;
        auto status = linears_fixed_add(a, b, &result);
        ASSERT_TRUE(MatchesFixed(status, result, static_cast<Wide>(a) + b)) << a << "," << b;
        result = kSentinel;
        status = linears_fixed_sub(a, b, &result);
        ASSERT_TRUE(MatchesFixed(status, result, static_cast<Wide>(a) - b)) << a << "," << b;
    }
}

TEST(AuditFixedAddSubtract, LargeCancellationRetainsSmallRawResiduals) {
    for (const int64_t residual : {INT64_C(1), INT64_C(7), kOne - 1}) {
        int64_t result = kSentinel;
        ASSERT_EQ(linears_fixed_sub(INT64_MAX, INT64_MAX - residual, &result), LINEARS_OK);
        EXPECT_EQ(result, residual);
        ASSERT_EQ(linears_fixed_add(INT64_MIN + residual, INT64_MAX, &result), LINEARS_OK);
        EXPECT_EQ(result, residual - 1);
    }
    int64_t result = kSentinel;
    ASSERT_EQ(linears_fixed_sub(INT64_MIN, INT64_MIN, &result), LINEARS_OK);
    EXPECT_EQ(result, 0);
}

TEST(AuditFixedMultiply, EverySignedExtremePairMatchesExactProduct) {
    for (const auto a : kEdges) for (const auto b : kEdges) {
        int64_t result = kSentinel;
        const auto status = linears_fixed_mul(a, b, &result);
        ASSERT_TRUE(MatchesFixed(status, result, RoundRatio(static_cast<Wide>(a) * b, kOne))) << a << "," << b;
    }
}

TEST(AuditFixedMultiply, MidpointNeighborsAndCarryRoundOnceAwayFromZero) {
    for (const int sign : {-1, 1}) for (const int64_t raw : {kOne / 2 - 1, kOne / 2, kOne / 2 + 1}) {
        int64_t result = kSentinel;
        ASSERT_EQ(linears_fixed_mul(sign, raw, &result), LINEARS_OK);
        EXPECT_EQ(result, raw < kOne / 2 ? 0 : sign);
    }
    const int64_t a = (INT64_C(1) << 40) + 65537;
    const int64_t b = (INT64_C(1) << 36) - 65535;
    int64_t result = kSentinel;
    const auto status = linears_fixed_mul(a, b, &result);
    ASSERT_TRUE(MatchesFixed(status, result, RoundRatio(static_cast<Wide>(a) * b, kOne)));
}

TEST(AuditFixedDivide, SignedExtremesAndHalfwayQuotients) {
    for (const auto a : kEdges) for (const auto b : kEdges) if (b != 0) {
        int64_t result = kSentinel;
        const auto status = linears_fixed_div(a, b, &result);
        ASSERT_TRUE(MatchesFixed(status, result, RoundRatio(static_cast<Wide>(a) * kOne, b))) << a << "," << b;
    }
    for (const int sign : {-1, 1}) {
        int64_t result = kSentinel;
        ASSERT_EQ(linears_fixed_div(sign, 2 * kOne, &result), LINEARS_OK);
        EXPECT_EQ(result, sign);
        ASSERT_EQ(linears_fixed_div(sign, 2 * kOne + 1, &result), LINEARS_OK);
        EXPECT_EQ(result, 0);
        ASSERT_EQ(linears_fixed_div(sign, 2 * kOne - 1, &result), LINEARS_OK);
        EXPECT_EQ(result, sign);
    }
}

TEST(AuditFixedMultiplyDivide, IntermediateProductMayOverflowWorkingButFinalValueFits) {
    for (const int64_t value : {INT64_MIN, INT64_MIN + 1, INT64_MAX - 1, INT64_MAX}) {
        int64_t result = kSentinel;
        ASSERT_EQ(linears_fixed_muldiv(value, value, value, &result), LINEARS_OK);
        EXPECT_EQ(result, value);
    }
    int64_t result = kSentinel;
    ASSERT_EQ(linears_fixed_muldiv(1, kOne / 2, kOne, &result), LINEARS_OK);
    EXPECT_EQ(result, 1);
    ASSERT_EQ(linears_fixed_muldiv(-1, kOne / 2, kOne, &result), LINEARS_OK);
    EXPECT_EQ(result, -1);
    // Rounding the intermediate product first would erase this result.
    ASSERT_EQ(linears_fixed_muldiv(1, 1, 1, &result), LINEARS_OK);
    EXPECT_EQ(result, 1);
}

TEST(AuditFixedMultiplyDivide, DeterministicFullWidthRationalOracle) {
    std::mt19937_64 engine(0x583a17);
    for (int i = 0; i < 4000; ++i) {
        const int64_t a = RandomRaw(engine) / (INT64_C(1) << (engine() % 48));
        const int64_t b = RandomRaw(engine) / (INT64_C(1) << (engine() % 48));
        int64_t divisor = RandomRaw(engine) / (INT64_C(1) << (engine() % 48));
        if (divisor == 0) divisor = 1;
        int64_t result = kSentinel;
        auto status = linears_fixed_mul(a, b, &result);
        ASSERT_TRUE(MatchesFixed(status, result, RoundRatio(static_cast<Wide>(a) * b, kOne))) << i;
        result = kSentinel;
        status = linears_fixed_div(a, divisor, &result);
        ASSERT_TRUE(MatchesFixed(status, result, RoundRatio(static_cast<Wide>(a) * kOne, divisor))) << i;
        result = kSentinel;
        status = linears_fixed_muldiv(a, b, divisor, &result);
        ASSERT_TRUE(MatchesFixed(status, result, RoundRatio(static_cast<Wide>(a) * b, divisor))) << i;
    }
}

TEST(AuditFixedSquareRoot, TinyInputsExactSquaresAndInt64Maximum) {
    constexpr std::array<int64_t, 16> samples{0, 1, 2, 3, 4, kOne / 4,
        kOne - 1, kOne, kOne + 1, 4 * kOne, (INT64_C(1) << 33) - 1,
        INT64_C(1) << 33, (INT64_C(1) << 33) + 1, INT64_C(1) << 62, INT64_MAX - 1, INT64_MAX};
    for (const auto raw : samples) {
        int64_t result = kSentinel;
        ASSERT_EQ(linears_fixed_sqrt(raw, &result), LINEARS_OK);
        EXPECT_EQ(result, SqrtReference(raw)) << raw;
    }
    for (const int64_t integer : {INT64_C(1), INT64_C(7), INT64_C(65535)}) {
        int64_t result = kSentinel;
        const int64_t square = integer * integer * kOne;
        ASSERT_EQ(linears_fixed_sqrt(square, &result), LINEARS_OK);
        EXPECT_EQ(result, integer * kOne);
    }
}

TEST(AuditFixedSquareRoot, IndependentBinarySearchOnFullWorkingDomain) {
    std::mt19937_64 engine(0x2591ad);
    for (int i = 0; i < 3000; ++i) {
        const int64_t raw = static_cast<int64_t>(engine() & INT64_MAX);
        int64_t result = kSentinel;
        ASSERT_EQ(linears_fixed_sqrt(raw, &result), LINEARS_OK);
        ASSERT_EQ(result, SqrtReference(raw)) << raw;
    }
}

TEST(AuditFixedOrdering, EveryInt64ValueIsValidAndOrderingDoesNotRescale) {
    for (const auto a : kEdges) for (const auto b : kEdges)
        EXPECT_EQ(linears_fixed_compare(a, b), (a > b) - (a < b));
    for (const auto value : kEdges) {
        int64_t output = kSentinel;
        auto status = linears_fixed_neg(value, &output);
        EXPECT_TRUE(MatchesFixed(status, output, -static_cast<Wide>(value)));
        output = kSentinel;
        status = linears_fixed_abs(value, &output);
        const Wide expected = value < 0 ? -static_cast<Wide>(value) : value;
        EXPECT_TRUE(MatchesFixed(status, output, expected));
    }
}

TEST(AuditFixedQuantize, ClosedDomainAndPositiveEndpointGapAreMonotone) {
    constexpr std::array<int64_t, 18> inputs{INT64_MIN, -kOne - 1, -kOne, -kOne + 1,
        -32769, -32768, -32767, 0, 32767, 32768, 32769,
        kOne - 32769, kOne - 32768, kOne - 32767, kOne - 1, kOne, kOne + 1, INT64_MAX};
    for (const auto raw : inputs) {
        int16_t q15 = 123; int32_t q31 = 456;
        const auto ref15 = SignalReference(raw, 31, 15, q15);
        const auto ref31 = SignalReference(raw, 31, 31, q31);
        ASSERT_EQ(linears_fixed_to_q15(raw, &q15), ref15.status);
        ASSERT_EQ(linears_fixed_to_q31(raw, &q31), ref31.status);
        EXPECT_EQ(q15, ref15.raw) << raw;
        EXPECT_EQ(q31, ref31.raw) << raw;
    }
}

TEST(AuditFixedQuantize, EveryQ15RoundingMidpointAndItsNeighbors) {
    for (int64_t code = INT16_MIN; code < INT16_MAX; ++code) {
        const int64_t midpoint = code * 65536 + 32768;
        for (const int64_t delta : {-1, 0, 1}) {
            const int64_t raw = midpoint + delta;
            int16_t output = 123;
            const auto expected = SignalReference(raw, 31, 15, output);
            ASSERT_EQ(linears_fixed_to_q15(raw, &output), expected.status);
            ASSERT_EQ(output, expected.raw) << code << "," << delta;
        }
    }
}

TEST(AuditFixedAccumulator, CarryBorrowAndSignedBitPatternsAreExact) {
    constexpr std::array<linears_accum_t, 7> starts{{
        {0, 0}, {UINT64_MAX, 0}, {0, 1}, {0, UINT64_MAX},
        {UINT64_MAX, UINT64_MAX}, {UINT64_MAX - 1, UINT64_MAX >> 1}, {1, UINT64_C(1) << 63}}};
    for (const auto initial : starts) for (const int64_t term : {-1, 1}) {
        auto actual = initial;
        ASSERT_EQ(linears_fixed_mac(&actual, term, 1), LINEARS_OK);
        EXPECT_TRUE(SameAccum(actual, EncodeAccum(DecodeAccum(initial) + term)));
        actual = initial;
        ASSERT_EQ(linears_fixed_msu(&actual, term, 1), LINEARS_OK);
        EXPECT_TRUE(SameAccum(actual, EncodeAccum(DecodeAccum(initial) - term)));
    }
}

TEST(AuditFixedAccumulator, Signed128OverflowPreservesBothLimbs) {
    for (const bool subtract : {false, true}) {
        auto accumulator = EncodeAccum(subtract ? kWideMin : kWideMax);
        const auto before = accumulator;
        const auto status = subtract ? linears_fixed_msu(&accumulator, 1, 1)
                                     : linears_fixed_mac(&accumulator, 1, 1);
        EXPECT_EQ(status, LINEARS_ERROR_RANGE);
        EXPECT_TRUE(SameAccum(accumulator, before));
    }
    linears_accum_t accumulator{0, 0};
    ASSERT_EQ(linears_fixed_mac(&accumulator, INT64_MIN, INT64_MIN), LINEARS_OK);
    EXPECT_TRUE(SameAccum(accumulator, EncodeAccum(Wide{1} << 126)));
    const auto before = accumulator;
    EXPECT_EQ(linears_fixed_mac(&accumulator, INT64_MIN, INT64_MIN), LINEARS_ERROR_RANGE);
    EXPECT_TRUE(SameAccum(accumulator, before));
    accumulator = {0, 0};
    ASSERT_EQ(linears_fixed_msu(&accumulator, INT64_MIN, INT64_MIN), LINEARS_OK);
    ASSERT_EQ(linears_fixed_msu(&accumulator, INT64_MIN, INT64_MIN), LINEARS_OK);
    EXPECT_TRUE(SameAccum(accumulator, EncodeAccum(kWideMin)));
    EXPECT_EQ(linears_fixed_msu(&accumulator, 1, 1), LINEARS_ERROR_RANGE);
    EXPECT_TRUE(SameAccum(accumulator, EncodeAccum(kWideMin)));
}

TEST(AuditFixedAccumulator, MacAndMsuMatchIndependentSigned128Arithmetic) {
    std::mt19937_64 engine(0x785ae1);
    Wide exact = 0;
    linears_accum_t accumulator{0, 0};
    for (int i = 0; i < 4000; ++i) {
        const int64_t a = RandomRaw(engine), b = RandomRaw(engine);
        const bool subtract = (engine() & 1) != 0;
        Wide term = static_cast<Wide>(a) * b;
        if (subtract) term = -term;
        const bool overflow = (term > 0 && exact > kWideMax - term) ||
                              (term < 0 && exact < kWideMin - term);
        const auto before = accumulator;
        const auto status = subtract ? linears_fixed_msu(&accumulator, a, b)
                                     : linears_fixed_mac(&accumulator, a, b);
        ASSERT_EQ(status, overflow ? LINEARS_ERROR_RANGE : LINEARS_OK);
        if (overflow) {
            ASSERT_TRUE(SameAccum(accumulator, before));
        } else {
            exact += term;
            ASSERT_TRUE(SameAccum(accumulator, EncodeAccum(exact)));
        }
    }
}

TEST(AuditFixedAccumulator, HugeProductsCancelBeforeFinalRounding) {
    for (const int sign : {-1, 1}) {
        linears_accum_t accumulator{0, 0};
        ASSERT_EQ(linears_fixed_mac(&accumulator, INT64_MIN, INT64_MIN), LINEARS_OK);
        ASSERT_EQ(linears_fixed_mac(&accumulator, sign, kOne / 2), LINEARS_OK);
        ASSERT_EQ(linears_fixed_msu(&accumulator, INT64_MIN, INT64_MIN), LINEARS_OK);
        EXPECT_TRUE(SameAccum(accumulator, EncodeAccum(static_cast<Wide>(sign) * (kOne / 2))));
        int64_t working = kSentinel;
        ASSERT_EQ(linears_accum_to_fixed(accumulator, &working), LINEARS_OK);
        EXPECT_EQ(working, sign);
    }
    linears_accum_t sum{0, 0};
    ASSERT_EQ(linears_fixed_mac(&sum, 1, kOne / 2), LINEARS_OK);
    ASSERT_EQ(linears_fixed_mac(&sum, 1, kOne / 2), LINEARS_OK);
    int64_t output = kSentinel;
    ASSERT_EQ(linears_accum_to_fixed(sum, &output), LINEARS_OK);
    EXPECT_EQ(output, 1); // Separately rounded products would incorrectly add to 2.
}

TEST(AuditFixedAccumulator, WorkingExportChecksRoundedInt64Boundary) {
    const Wide upper = static_cast<Wide>(INT64_MAX) * kOne;
    const Wide lower = static_cast<Wide>(INT64_MIN) * kOne;
    const std::array<Wide, 12> inputs{kWideMin, kWideMax, lower - kOne / 2,
        lower - kOne / 2 + 1, lower, lower + kOne / 2, -Wide{kOne} / 2,
        Wide{kOne} / 2, upper, upper + kOne / 2 - 1, upper + kOne / 2, upper + kOne};
    for (const auto raw : inputs) {
        int64_t output = kSentinel;
        const auto status = linears_accum_to_fixed(EncodeAccum(raw), &output);
        ASSERT_TRUE(MatchesFixed(status, output, RoundRatio(raw, kOne)));
    }
}

TEST(AuditFixedAccumulator, ExactF62DomainCheckPrecedesAnyWorkingRounding) {
    const Wide one = Wide{1} << 62;
    const std::array<Wide, 8> values{-one - 1, -one, -one + 1, -1, 1, one - 1, one, one + 1};
    for (const auto raw : values) {
        int16_t q15 = 123; int32_t q31 = 456;
        const auto expected15 = SignalReference(raw, 62, 15, q15);
        const auto expected31 = SignalReference(raw, 62, 31, q31);
        ASSERT_EQ(linears_accum_to_q15(EncodeAccum(raw), &q15), expected15.status);
        ASSERT_EQ(linears_accum_to_q31(EncodeAccum(raw), &q31), expected31.status);
        EXPECT_EQ(q15, expected15.raw); EXPECT_EQ(q31, expected31.raw);
    }
    int64_t rounded = kSentinel;
    ASSERT_EQ(linears_accum_to_fixed(EncodeAccum(one + 1), &rounded), LINEARS_OK);
    EXPECT_EQ(rounded, kOne);
    int32_t output = 123;
    EXPECT_EQ(linears_accum_to_q31(EncodeAccum(one + 1), &output), LINEARS_ERROR_RANGE);
    EXPECT_EQ(output, 123);
}

TEST(AuditFixedAccumulator, DirectQExportRoundsOnlyOnceAtBothBinaryPoints) {
    for (const int bits : {15, 31}) {
        const Wide step = Wide{1} << (62 - bits);
        for (const int64_t code : {INT64_C(0), INT64_C(1), INT64_C(17), INT64_C(32766), INT64_C(32767)}) {
            for (const int delta : {-1, 0, 1}) for (const int sign : {-1, 1}) {
                const Wide raw = sign * (code * step + step / 2 + delta);
                const auto reference = SignalReference(raw, 62, bits, 123);
                if (bits == 15) {
                    int16_t output = 123;
                    ASSERT_EQ(linears_accum_to_q15(EncodeAccum(raw), &output), reference.status);
                    ASSERT_EQ(output, reference.raw);
                } else {
                    int32_t output = 123;
                    ASSERT_EQ(linears_accum_to_q31(EncodeAccum(raw), &output), reference.status);
                    ASSERT_EQ(output, reference.raw);
                }
            }
        }
    }
    // Just below a q15 midpoint, first rounding to F31 would create a false tie.
    const Wide raw = (Wide{1} << 46) - 1;
    int16_t output = 123;
    ASSERT_EQ(linears_accum_to_q15(EncodeAccum(raw), &output), LINEARS_OK);
    EXPECT_EQ(output, 0);
}

TEST(AuditFixedMean, FullUnsignedCountsCheckExactDomainBeforeDivision) {
    for (const uint64_t count : {UINT64_C(1), UINT64_C(3), UINT64_C(1) << 63, UINT64_MAX - 1, UINT64_MAX}) {
        const Wide edge = static_cast<Wide>(count) * (Wide{1} << 62);
        const std::array<Wide, 9> values{-edge - 1, -edge, -edge + 1, -1, 0, 1, edge - 1, edge, edge + 1};
        for (const auto value : values) {
            int16_t q15 = 123; int32_t q31 = 456;
            const auto ref15 = MeanReference(value, count, 15, q15);
            const auto ref31 = MeanReference(value, count, 31, q31);
            ASSERT_EQ(linears_accum_mean_q15(EncodeAccum(value), count, &q15), ref15.status);
            ASSERT_EQ(linears_accum_mean_q31(EncodeAccum(value), count, &q31), ref31.status);
            ASSERT_EQ(q15, ref15.raw) << count;
            ASSERT_EQ(q31, ref31.raw) << count;
        }
    }
}

TEST(AuditFixedMean, OddCountsAndMidpointNeighborsRoundOnlyTheFinalRational) {
    for (const int bits : {15, 31}) {
        const Wide step = Wide{1} << (62 - bits);
        for (const uint64_t count : {UINT64_C(1), UINT64_C(3), UINT64_C(17), UINT64_MAX}) {
            for (const int64_t code : {INT64_C(0), INT64_C(1), INT64_C(32766)}) {
                for (const int delta : {-1, 0, 1}) for (const int sign : {-1, 1}) {
                    const Wide value = sign * ((code * step + step / 2) * count + delta);
                    const auto expected = MeanReference(value, count, bits, 123);
                    if (bits == 15) {
                        int16_t raw = 123;
                        ASSERT_EQ(linears_accum_mean_q15(EncodeAccum(value), count, &raw), expected.status);
                        ASSERT_EQ(raw, expected.raw);
                    } else {
                        int32_t raw = 123;
                        ASSERT_EQ(linears_accum_mean_q31(EncodeAccum(value), count, &raw), expected.status);
                        ASSERT_EQ(raw, expected.raw);
                    }
                }
            }
        }
    }
}

TEST(AuditFixedMean, Random128SumsAndUint64DivisorsMatchOneExactDivision) {
    std::mt19937_64 engine(0x918427);
    for (int i = 0; i < 3000; ++i) {
        const linears_accum_t value{engine(), engine()};
        const uint64_t count = engine() | UINT64_C(1);
        const Wide exact = DecodeAccum(value);
        int16_t q15 = 123; int32_t q31 = 456;
        const auto ref15 = MeanReference(exact, count, 15, q15);
        const auto ref31 = MeanReference(exact, count, 31, q31);
        ASSERT_EQ(linears_accum_mean_q15(value, count, &q15), ref15.status);
        ASSERT_EQ(linears_accum_mean_q31(value, count, &q31), ref31.status);
        ASSERT_EQ(q15, ref15.raw); ASSERT_EQ(q31, ref31.raw);
    }
    int16_t q15 = 123; int32_t q31 = 456;
    EXPECT_EQ(linears_accum_mean_q15({0, 0}, 0, &q15), LINEARS_ERROR_DOMAIN);
    EXPECT_EQ(linears_accum_mean_q31({0, 0}, 0, &q31), LINEARS_ERROR_DOMAIN);
    EXPECT_EQ(q15, 123); EXPECT_EQ(q31, 456);
    EXPECT_EQ(linears_accum_mean_q15({0, 0}, 1, nullptr), LINEARS_ERROR_ARGUMENT);
    EXPECT_EQ(linears_accum_mean_q31({0, 0}, 1, nullptr), LINEARS_ERROR_ARGUMENT);
}

TEST(AuditFixedAtomicFailure, ArgumentsDomainsAndUnaryOverflowKeepSentinels) {
    for (const auto operation : kBinary) EXPECT_EQ(operation(kOne, kOne, nullptr), LINEARS_ERROR_ARGUMENT);
    EXPECT_EQ(linears_fixed_muldiv(1, 1, 1, nullptr), LINEARS_ERROR_ARGUMENT);
    EXPECT_EQ(linears_fixed_sqrt(0, nullptr), LINEARS_ERROR_ARGUMENT);
    EXPECT_EQ(linears_fixed_neg(0, nullptr), LINEARS_ERROR_ARGUMENT);
    EXPECT_EQ(linears_fixed_abs(0, nullptr), LINEARS_ERROR_ARGUMENT);
    EXPECT_EQ(linears_fixed_to_q15(0, nullptr), LINEARS_ERROR_ARGUMENT);
    EXPECT_EQ(linears_fixed_to_q31(0, nullptr), LINEARS_ERROR_ARGUMENT);
    EXPECT_EQ(linears_fixed_mac(nullptr, 0, 0), LINEARS_ERROR_ARGUMENT);
    EXPECT_EQ(linears_fixed_msu(nullptr, 0, 0), LINEARS_ERROR_ARGUMENT);
    EXPECT_EQ(linears_accum_to_fixed({0, 0}, nullptr), LINEARS_ERROR_ARGUMENT);
    EXPECT_EQ(linears_accum_to_q15({0, 0}, nullptr), LINEARS_ERROR_ARGUMENT);
    EXPECT_EQ(linears_accum_to_q31({0, 0}, nullptr), LINEARS_ERROR_ARGUMENT);
    AtomicFailure(LINEARS_ERROR_DOMAIN, [](auto *out) { return linears_fixed_div(0, 0, out); });
    AtomicFailure(LINEARS_ERROR_DOMAIN, [](auto *out) { return linears_fixed_muldiv(INT64_MIN, INT64_MIN, 0, out); });
    for (const int64_t negative : {INT64_MIN, -kOne, INT64_C(-1)})
        AtomicFailure(LINEARS_ERROR_DOMAIN, [=](auto *out) { return linears_fixed_sqrt(negative, out); });
    AtomicFailure(LINEARS_ERROR_RANGE, [](auto *out) { return linears_fixed_neg(INT64_MIN, out); });
    AtomicFailure(LINEARS_ERROR_RANGE, [](auto *out) { return linears_fixed_abs(INT64_MIN, out); });
    AtomicFailure(LINEARS_ERROR_RANGE, [](auto *out) { return linears_fixed_add(INT64_MAX, 1, out); });
    AtomicFailure(LINEARS_ERROR_RANGE, [](auto *out) { return linears_fixed_sub(INT64_MIN, 1, out); });
}

TEST(AuditFixedAliasing, ScalarOutputsMayReplaceInputsAndFailuresDoNotReplaceThem) {
    const int64_t a = kOne + 17, b = -kOne / 2 + 31;
    const std::array<Wide, 4> expected{Wide{a} + b, Wide{a} - b,
        RoundRatio(Wide{a} * b, kOne), RoundRatio(Wide{a} * kOne, b)};
    for (size_t i = 0; i < kBinary.size(); ++i) {
        int64_t left = a, right = b;
        auto status = kBinary[i](left, right, &left);
        EXPECT_TRUE(MatchesFixed(status, left, expected[i], a));
        left = a;
        status = kBinary[i](left, right, &right);
        EXPECT_TRUE(MatchesFixed(status, right, expected[i], b));
    }
    int64_t value = INT64_MIN;
    EXPECT_EQ(linears_fixed_neg(value, &value), LINEARS_ERROR_RANGE);
    EXPECT_EQ(value, INT64_MIN);
    value = INT64_MAX;
    ASSERT_EQ(linears_fixed_muldiv(value, value, value, &value), LINEARS_OK);
    EXPECT_EQ(value, INT64_MAX);
    value = kOne;
    ASSERT_EQ(linears_fixed_sqrt(value, &value), LINEARS_OK);
    EXPECT_EQ(value, kOne);
}

TEST(AuditFixedExport, MixedRawValuesUseIndependentEndpointQuantization) {
    constexpr std::array<int64_t, 11> values{-kOne, -kOne + 1, -65536, -32768, -1,
        0, 1, 32768, 65536, kOne - 1, kOne};
    std::array<int16_t, values.size() + 2> q15{}; q15.fill(123);
    std::array<int32_t, values.size() + 2> q31{}; q31.fill(456);
    ASSERT_EQ(linears_fixed_export_q15(values.data(), values.size(), q15.data() + 1), LINEARS_OK);
    ASSERT_EQ(linears_fixed_export_q31(values.data(), values.size(), q31.data() + 1), LINEARS_OK);
    for (size_t i = 0; i < values.size(); ++i) {
        EXPECT_EQ(q15[i + 1], SignalReference(values[i], 31, 15, 0).raw);
        EXPECT_EQ(q31[i + 1], SignalReference(values[i], 31, 31, 0).raw);
    }
    EXPECT_EQ(q15.front(), 123); EXPECT_EQ(q15.back(), 123);
    EXPECT_EQ(q31.front(), 456); EXPECT_EQ(q31.back(), 456);
    EXPECT_EQ(linears_fixed_export_q15(nullptr, 0, nullptr), LINEARS_OK);
    EXPECT_EQ(linears_fixed_export_q31(nullptr, 0, nullptr), LINEARS_OK);
}

TEST(AuditFixedExport, ALastOutOfDomainValueCannotPartiallyWriteTheArray) {
    for (const int64_t invalid : {-kOne - 1, kOne + 1, INT64_MIN, INT64_MAX}) {
        const std::array<int64_t, 4> values{kOne, -kOne, 0, invalid};
        std::array<int16_t, 6> q15{61, 62, 63, 64, 65, 66};
        std::array<int32_t, 6> q31{71, 72, 73, 74, 75, 76};
        const auto before15 = q15; const auto before31 = q31;
        EXPECT_EQ(linears_fixed_export_q15(values.data(), values.size(), q15.data() + 1), LINEARS_ERROR_RANGE);
        EXPECT_EQ(linears_fixed_export_q31(values.data(), values.size(), q31.data() + 1), LINEARS_ERROR_RANGE);
        EXPECT_EQ(q15, before15); EXPECT_EQ(q31, before31);
    }
}

TEST(AuditFixedStorage, CountOverflowNullAlignmentAndOverlapsAreAtomic) {
    std::array<int64_t, 4> values{kOne, -kOne, 0, 1};
    const auto original = values;
    int16_t q15 = 123; int32_t q31 = 456;
    for (const size_t count : {SIZE_MAX / sizeof(int64_t) + 1, SIZE_MAX}) {
        EXPECT_EQ(linears_fixed_export_q15(values.data(), count, &q15), LINEARS_ERROR_SIZE);
        EXPECT_EQ(linears_fixed_export_q31(values.data(), count, &q31), LINEARS_ERROR_SIZE);
    }
    for (const size_t offset : {size_t{0}, size_t{1}, size_t{3}}) {
        EXPECT_EQ(linears_fixed_export_q15(values.data(), 4, reinterpret_cast<int16_t *>(values.data() + offset)), LINEARS_ERROR_ALIAS);
        EXPECT_EQ(linears_fixed_export_q31(values.data(), 4, reinterpret_cast<int32_t *>(values.data() + offset)), LINEARS_ERROR_ALIAS);
        EXPECT_EQ(values, original);
    }
    EXPECT_EQ(linears_fixed_export_q15(nullptr, 1, &q15), LINEARS_ERROR_ARGUMENT);
    EXPECT_EQ(linears_fixed_export_q31(nullptr, 1, &q31), LINEARS_ERROR_ARGUMENT);
    EXPECT_EQ(linears_fixed_export_q15(values.data(), 1, nullptr), LINEARS_ERROR_ARGUMENT);
    EXPECT_EQ(linears_fixed_export_q31(values.data(), 1, nullptr), LINEARS_ERROR_ARGUMENT);
    alignas(int64_t) std::array<unsigned char, 64> bytes{}; bytes.fill(0x5a);
    const auto before = bytes;
    EXPECT_EQ(linears_fixed_export_q15(reinterpret_cast<const int64_t *>(bytes.data() + 1), 1, &q15), LINEARS_ERROR_ARGUMENT);
    EXPECT_EQ(linears_fixed_export_q31(reinterpret_cast<const int64_t *>(bytes.data() + 1), 1, &q31), LINEARS_ERROR_ARGUMENT);
    EXPECT_EQ(linears_fixed_export_q15(values.data(), 1, reinterpret_cast<int16_t *>(bytes.data() + 1)), LINEARS_ERROR_ARGUMENT);
    EXPECT_EQ(linears_fixed_export_q31(values.data(), 1, reinterpret_cast<int32_t *>(bytes.data() + 1)), LINEARS_ERROR_ARGUMENT);
    EXPECT_EQ(bytes, before); EXPECT_EQ(q15, 123); EXPECT_EQ(q31, 456);
}

TEST(AuditFixedStorage, StaticWorkspaceCapacityDoesNotGrowToTheAccumulatorSize) {
    constexpr size_t unit = sizeof(linears_workspace_unit_t);
    constexpr size_t bytes = LINEARS_BASIC_WORKSPACE_BYTES(3, 7);
    static_assert(unit == 8, "one bounded local accumulator, not one per output");
    static_assert(bytes == 21 * unit, "output workspace formula");
    static_assert(LINEARS_BASIC_WORKSPACE_BYTES(0, SIZE_MAX) == 0, "empty output");
    static_assert(LINEARS_BASIC_WORKSPACE_BYTES(SIZE_MAX, 0) == 0, "empty output");
    static_assert(LINEARS_SIZE_MUL(SIZE_MAX, 2) == SIZE_MAX, "checked multiply");
    static_assert(LINEARS_SIZE_ADD(SIZE_MAX, 1) == SIZE_MAX, "checked addition");
    static_assert(LINEARS_WORKSPACE_UNITS(SIZE_MAX) == SIZE_MAX, "invalid size propagates");
    std::array<linears_workspace_unit_t, LINEARS_WORKSPACE_UNITS(bytes)> workspace{};
    EXPECT_EQ(sizeof(workspace), bytes);
    EXPECT_GE(alignof(decltype(workspace)), alignof(int64_t));
    for (size_t required = 0; required < 3 * unit; ++required) {
        const size_t capacity = LINEARS_WORKSPACE_UNITS(required) * unit;
        EXPECT_GE(capacity, required); EXPECT_LT(capacity - required, unit);
    }
    EXPECT_EQ(LINEARS_BASIC_WORKSPACE_BYTES(SIZE_MAX / unit + 1, 1), SIZE_MAX);
}

#define AUDIT_RAW_FORMAT(Name, Suffix, RawType, Bits) \
    struct Name { \
        using Raw = RawType; static constexpr int bits = Bits; \
        static linears_status_t dot(const linears_matrix_t *a, const linears_matrix_t *b, Raw *out) { return linears_dot_##Suffix(a,b,out); } \
        static linears_status_t norm(const linears_matrix_t *a, Raw *out) { return linears_norm_##Suffix(a,out); } \
        static linears_status_t mul(const linears_matrix_t *a, const linears_matrix_t *b, linears_matrix_t *out, linears_workspace_unit_t *w, size_t bytes) { return linears_mul_##Suffix(a,b,out,w,bytes); } \
        static linears_status_t scale(int64_t gain, const linears_matrix_t *a, linears_matrix_t *out, linears_workspace_unit_t *w, size_t bytes) { return linears_scale_##Suffix(gain,a,out,w,bytes); } \
        static linears_status_t axpby(int64_t alpha, const linears_matrix_t *a, int64_t beta, const linears_matrix_t *b, linears_matrix_t *out, linears_workspace_unit_t *w, size_t bytes) { return linears_axpby_##Suffix(alpha,a,beta,b,out,w,bytes); } \
    };
AUDIT_RAW_FORMAT(RawQ15, q15, int16_t, 15)
AUDIT_RAW_FORMAT(RawQ31, q31, int32_t, 31)
#undef AUDIT_RAW_FORMAT
using RawFormats = ::testing::Types<RawQ15, RawQ31>;
template<class F> class AuditRawMac : public ::testing::Test {};
TYPED_TEST_SUITE(AuditRawMac, RawFormats);

TYPED_TEST(AuditRawMac, DotAllowsLargePartialSumsToCancelBeforeSignalExport) {
    using F = TypeParam; using Raw = typename F::Raw;
    constexpr Raw minimum = std::numeric_limits<Raw>::min(), maximum = std::numeric_limits<Raw>::max();
    std::array<Raw, 4> a{minimum, minimum, minimum, minimum};
    std::array<Raw, 4> b{minimum, minimum, maximum, maximum};
    linears_matrix_t am{a.data(), 1, 4}, bm{b.data(), 1, 4};
    Raw result = 123;
    ASSERT_EQ(F::dot(&am, &bm, &result), LINEARS_OK);
    EXPECT_EQ(result, 2);
    b = {minimum, maximum, minimum, maximum};
    ASSERT_EQ(F::dot(&am, &bm, &result), LINEARS_OK);
    EXPECT_EQ(result, 2);
}

TYPED_TEST(AuditRawMac, DotRoundsOnceAndRejectsAnExactResultOneProductBitOutsideRange) {
    using F = TypeParam; using Raw = typename F::Raw;
    constexpr Raw half = static_cast<Raw>(INT64_C(1) << (F::bits - 1));
    std::array<Raw, 2> a{1, 1}, b{half, half};
    linears_matrix_t am{a.data(), 1, 2}, bm{b.data(), 1, 2};
    Raw result = 123;
    ASSERT_EQ(F::dot(&am, &bm, &result), LINEARS_OK);
    EXPECT_EQ(result, 1);
    a = {std::numeric_limits<Raw>::min(), 1}; b = a;
    result = 123;
    EXPECT_EQ(F::dot(&am, &bm, &result), LINEARS_ERROR_RANGE);
    EXPECT_EQ(result, 123);
    // sqrt(1 + one product bit) is also outside the exact signal domain,
    // even though premature working rounding could make it appear to be one.
    EXPECT_EQ(F::norm(&am, &result), LINEARS_ERROR_RANGE);
    EXPECT_EQ(result, 123);
    a[1] = 0;
    ASSERT_EQ(F::norm(&am, &result), LINEARS_OK);
    EXPECT_EQ(result, std::numeric_limits<Raw>::max());
}

TYPED_TEST(AuditRawMac, MatrixLateRangeFailureKeepsAllOutputsAndWorkspaceGuards) {
    using F = TypeParam; using Raw = typename F::Raw;
    constexpr Raw minimum = std::numeric_limits<Raw>::min();
    constexpr Raw quarter = static_cast<Raw>(INT64_C(1) << (F::bits - 2));
    std::array<Raw, 4> a{quarter, 0, minimum, minimum};
    std::array<Raw, 2> b{minimum, minimum};
    std::array<Raw, 4> output{37, 101, 102, 37}; const auto before = output;
    linears_matrix_t am{a.data(), 2, 2}, bm{b.data(), 2, 1}, out{output.data() + 1, 2, 1};
    std::array<linears_workspace_unit_t, 4> work{};
    work.front().integer = 91; work.back().integer = 92;
    ASSERT_EQ(F::mul(&am, &bm, &out, work.data() + 1, LINEARS_BASIC_WORKSPACE_BYTES(2, 1)), LINEARS_ERROR_RANGE);
    EXPECT_EQ(output, before);
    EXPECT_EQ(work.front().integer, 91); EXPECT_EQ(work.back().integer, 92);
    EXPECT_EQ(out.data, output.data() + 1); EXPECT_EQ(out.rows, size_t{2}); EXPECT_EQ(out.cols, size_t{1});
}

TYPED_TEST(AuditRawMac, CoefficientsAboveOneUseWideF31NotSignalRawScaling) {
    using F = TypeParam; using Raw = typename F::Raw;
    constexpr Raw quarter = static_cast<Raw>(INT64_C(1) << (F::bits - 2));
    Raw input = quarter, output = 123;
    linears_matrix_t a{&input, 1, 1}, out{&output, 1, 1};
    std::array<linears_workspace_unit_t, 1> work{};
    ASSERT_EQ(F::scale(2 * kOne, &a, &out, work.data(), sizeof(work)), LINEARS_OK);
    EXPECT_EQ(output, static_cast<Raw>(quarter * 2));
    ASSERT_EQ(F::axpby(8 * kOne, &a, -7 * kOne, &a, &out, work.data(), sizeof(work)), LINEARS_OK);
    EXPECT_EQ(output, quarter);
    input = 1;
    ASSERT_EQ(F::axpby(kOne / 2, &a, kOne / 2, &a, &out, work.data(), sizeof(work)), LINEARS_OK);
    EXPECT_EQ(output, 1); // Two half-ULP products round once, not separately to two.
}

TEST(AuditWorkingMatrix, PrefixOverflowIsCheckedEvenWhenTheFinalRealSumWouldFit) {
    std::array<int64_t, 4> left{INT64_MIN, INT64_MIN, INT64_MIN, INT64_MIN};
    std::array<int64_t, 4> right{INT64_MIN, INT64_MIN, INT64_MAX, INT64_MAX};
    linears_matrix_t a{left.data(), 1, 4}, b{right.data(), 4, 1};
    int64_t result = kSentinel;
    linears_matrix_t out{&result, 1, 1};
    std::array<linears_workspace_unit_t, 3> work{};
    work.front().integer = 91; work.back().integer = 92;
    // The first two exact products total +2^127, outside signed 128 bits.
    EXPECT_EQ(linears_mul_fixed(&a, &b, &out, work.data() + 1, sizeof(work[1])), LINEARS_ERROR_RANGE);
    EXPECT_EQ(result, kSentinel);
    EXPECT_EQ(work.front().integer, 91); EXPECT_EQ(work.back().integer, 92);
    // The same mathematical terms in a safe prefix order sum to 2^64 F62,
    // hence 2^33 F31 (= real four), which is legal in working matrices.
    right = {INT64_MIN, INT64_MAX, INT64_MIN, INT64_MAX};
    ASSERT_EQ(linears_mul_fixed(&a, &b, &out, work.data() + 1, sizeof(work[1])), LINEARS_OK);
    EXPECT_EQ(result, 4 * kOne);
    EXPECT_EQ(work.front().integer, 91); EXPECT_EQ(work.back().integer, 92);
}

TEST(AuditWorkingMatrix, ExactUnitsAndLargeCoefficientCancellationDoNotNarrowToSignals) {
    std::array<int64_t, 4> identity{};
    linears_matrix_t id{identity.data(), 2, 2};
    ASSERT_EQ(linears_identity_fixed(&id), LINEARS_OK);
    EXPECT_EQ(identity, (std::array<int64_t, 4>{kOne, 0, 0, kOne}));
    int64_t lhs = 2 * kOne, rhs = kOne, result = kSentinel;
    linears_matrix_t a{&lhs, 1, 1}, b{&rhs, 1, 1}, out{&result, 1, 1};
    std::array<linears_workspace_unit_t, 1> work{};
    ASSERT_EQ(linears_mul_fixed(&a, &b, &out, work.data(), sizeof(work)), LINEARS_OK);
    EXPECT_EQ(result, 2 * kOne);
    lhs = rhs = INT64_MAX;
    ASSERT_EQ(linears_axpby_fixed(INT64_MAX, &a, -INT64_MAX, &b, &out, work.data(), sizeof(work)), LINEARS_OK);
    EXPECT_EQ(result, 0);
}

} // namespace
