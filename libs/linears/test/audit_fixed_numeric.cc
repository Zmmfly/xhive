#include "linears_numeric.h"

#include <gtest/gtest.h>

#include <array>
#include <climits>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <random>

namespace {

constexpr int32_t kHalf = INT32_C(1073741824);
constexpr int32_t kFull = INT32_MAX;
constexpr linears_fixed_t kZero{0, 0};
constexpr linears_fixed_t kOne{kHalf, 1};
constexpr linears_fixed_t kSentinel{INT32_C(0x13579bdf), -57};
constexpr uint32_t kCanary = UINT32_C(0xc5a39e71);

// A binary long double with >=64 significant bits represents every 31x31-bit
// product exactly. These tests deliberately do not use production normalization,
// division, square-root, or quantization helpers to construct expected values.
static_assert(std::numeric_limits<long double>::radix == 2,
              "The independent numeric oracle requires binary long double");
static_assert(std::numeric_limits<long double>::digits >= 64,
              "The independent numeric oracle requires >=64 significant bits");

long double AsReal(linears_fixed_t value) {
    return std::ldexp(static_cast<long double>(value.mantissa), value.exponent - 31);
}

linears_fixed_t RoundedReference(long double value) {
    if (value == 0.0L) return kZero;
    int exponent = 0;
    const long double fraction = std::frexp(value, &exponent);
    long double rounded = std::round(std::ldexp(fraction, 31));
    if (std::fabs(rounded) == 2147483648.0L) {
        rounded *= 0.5L;
        ++exponent;
    }
    return {static_cast<int32_t>(rounded), exponent};
}

::testing::AssertionResult SameFixed(linears_fixed_t actual, linears_fixed_t expected) {
    if (actual.mantissa == expected.mantissa && actual.exponent == expected.exponent)
        return ::testing::AssertionSuccess();
    return ::testing::AssertionFailure()
           << "actual {" << actual.mantissa << ", " << actual.exponent
           << "}, expected {" << expected.mantissa << ", " << expected.exponent << "}";
}

::testing::AssertionResult RoundedAs(linears_fixed_t actual, long double exact) {
    return SameFixed(actual, RoundedReference(exact));
}

linears_fixed_t RandomCanonical(std::minstd_rand &random, int exponent_span) {
    const int32_t magnitude = kHalf + static_cast<int32_t>(random() % kHalf);
    const int sign = random() % 2 == 0 ? 1 : -1;
    const int exponent = static_cast<int>(random() % (2 * exponent_span + 1)) - exponent_span;
    return {sign * magnitude, exponent};
}

struct GuardedScalar {
    uint32_t before = kCanary;
    linears_fixed_t value = kSentinel;
    uint32_t after = kCanary;
};

template <typename Operation>
void ExpectAtomicScalarFailure(linears_status_t expected, Operation operation) {
    GuardedScalar guarded;
    EXPECT_EQ(operation(&guarded.value), expected);
    EXPECT_TRUE(SameFixed(guarded.value, kSentinel));
    EXPECT_EQ(guarded.before, kCanary);
    EXPECT_EQ(guarded.after, kCanary);
}

template <typename Integer, size_t Count>
std::array<Integer, Count> ExportReference(const std::array<linears_fixed_t, Count> &input,
                                         int *exponent) {
    long double peak = 0.0L;
    for (const auto value : input) peak = std::fmax(peak, std::fabs(AsReal(value)));
    *exponent = 0;
    if (peak != 0.0L) (void)std::frexp(peak, exponent);
    constexpr int bits = std::numeric_limits<Integer>::digits;
    for (const auto value : input) {
        const long double rounded = std::round(std::ldexp(AsReal(value), bits - *exponent));
        if (rounded > static_cast<long double>(std::numeric_limits<Integer>::max())) {
            ++*exponent;
            break;
        }
    }
    std::array<Integer, Count> expected{};
    for (size_t index = 0; index < Count; ++index)
        expected[index] = static_cast<Integer>(
            std::round(std::ldexp(AsReal(input[index]), bits - *exponent)));
    return expected;
}

using BinaryOperation = linears_status_t (*)(linears_fixed_t, linears_fixed_t, linears_fixed_t *);
constexpr std::array<BinaryOperation, 4> kBinaryOperations{
    linears_fixed_add, linears_fixed_sub, linears_fixed_mul, linears_fixed_div};

TEST(AuditFixedImport, EveryInt16MantissaRoundTripsAtThreeScales) {
    for (const int exponent : {-32, 0, 31}) {
        for (int raw = INT16_MIN; raw <= INT16_MAX; ++raw) {
            const linears_q15_t input{static_cast<int16_t>(raw), exponent};
            linears_fixed_t value = kSentinel;
            ASSERT_EQ(linears_fixed_from_q15(input, &value), LINEARS_OK)
                << "raw=" << raw << ", exponent=" << exponent;
            const long double exact = std::ldexp(static_cast<long double>(raw), exponent - 15);
            ASSERT_TRUE(RoundedAs(value, exact)) << "raw=" << raw << ", exponent=" << exponent;
            ASSERT_EQ(AsReal(value), exact);
            int16_t output = 123;
            ASSERT_EQ(linears_fixed_to_q15(value, exponent, &output), LINEARS_OK);
            ASSERT_EQ(output, raw) << "raw=" << raw << ", exponent=" << exponent;
        }
    }
}

TEST(AuditFixedImport, Q31SignsNormalizationAndRawIntMinRoundTrip) {
    constexpr std::array<int32_t, 17> mantissas{
        INT32_MIN, -kFull, -kHalf - 1, -kHalf, -kHalf + 1, -536870912, -3, -1,
        0, 1, 3, 536870912, kHalf - 1, kHalf, kHalf + 1, kFull - 1, kFull};
    for (const int exponent : {-100, -1, 0, 1, 100}) {
        for (const int32_t raw : mantissas) {
            linears_fixed_t value = kSentinel;
            ASSERT_EQ(linears_fixed_from_q31({raw, exponent}, &value), LINEARS_OK);
            const long double exact = std::ldexp(static_cast<long double>(raw), exponent - 31);
            ASSERT_TRUE(RoundedAs(value, exact)) << "raw=" << raw << ", exponent=" << exponent;
            ASSERT_EQ(AsReal(value), exact);
            int32_t output = 123;
            ASSERT_EQ(linears_fixed_to_q31(value, exponent, &output), LINEARS_OK);
            ASSERT_EQ(output, raw);
        }
    }
    linears_fixed_t minimum{};
    ASSERT_EQ(linears_fixed_from_q31({INT32_MIN, 0}, &minimum), LINEARS_OK);
    EXPECT_TRUE(SameFixed(minimum, {-kHalf, 1}));
}

TEST(AuditFixedImport, EveryDenominatorAndInt64ExtremesUseIndependentMath) {
    constexpr std::array<int64_t, 17> integers{
        INT64_MIN, INT64_MIN + 1, -INT64_C(4294967295), -INT64_C(2147483649),
        -INT64_C(2147483648), -INT64_C(2147483647), -17, -1, 0, 1, 17,
        INT64_C(2147483647), INT64_C(2147483648), INT64_C(2147483649),
        INT64_C(4294967295), INT64_MAX - 1, INT64_MAX};
    for (int fractional_bits = 0; fractional_bits <= 63; ++fractional_bits) {
        for (const auto raw : integers) {
            for (const int exponent : {-20, 0, 20}) {
                linears_fixed_t output = kSentinel;
                ASSERT_EQ(linears_fixed_from_i64(raw, fractional_bits, exponent, &output), LINEARS_OK);
                ASSERT_TRUE(RoundedAs(output, std::ldexp(static_cast<long double>(raw),
                                                        exponent - fractional_bits)))
                    << "raw=" << raw << ", denominator=" << fractional_bits;
            }
        }
    }
}

TEST(AuditFixedImport, HalfwayRoundsAwayFromZeroAndCarryRenormalizes) {
    struct Case { int64_t input; linears_fixed_t expected; };
    constexpr std::array<Case, 6> cases{{
        {INT64_C(2147483649), {kHalf + 1, 32}},
        {INT64_C(4294967297), {kHalf, 33}},
        {INT64_C(4294967298), {kHalf + 1, 33}},
        {INT64_C(4294967299), {kHalf + 1, 33}},
        {INT64_C(4294967295), {kHalf, 33}},
        {INT64_MAX, {kHalf, 64}},
    }};
    for (const auto &item : cases) {
        for (const int sign : {-1, 1}) {
            linears_fixed_t output{};
            ASSERT_EQ(linears_fixed_from_i64(sign * item.input, 0, 0, &output), LINEARS_OK);
            EXPECT_TRUE(SameFixed(output, {sign * item.expected.mantissa, item.expected.exponent}));
        }
    }
    linears_fixed_t output{};
    ASSERT_EQ(linears_fixed_from_i64(INT64_MIN, 63, 0, &output), LINEARS_OK);
    EXPECT_TRUE(SameFixed(output, {-kHalf, 1}));
}

TEST(AuditFixedImport, ValidityRejectsNoncanonicalZeroAndIntMin) {
    for (const auto value : std::array<linears_fixed_t, 7>{{
             kZero, kOne, {-kHalf, 1}, {kFull, LINEARS_EXPONENT_MIN},
             {-kFull, LINEARS_EXPONENT_MIN}, {kHalf, LINEARS_EXPONENT_MAX},
             {-kHalf, LINEARS_EXPONENT_MAX}}})
        EXPECT_NE(linears_fixed_valid(value), 0);
    for (const auto value : std::array<linears_fixed_t, 8>{{
             {0, 1}, {0, LINEARS_EXPONENT_MIN}, {1, 0}, {-1, 0},
             {kHalf - 1, 0}, {INT32_MIN, 0},
             {kHalf, LINEARS_EXPONENT_MIN - 1}, {kHalf, LINEARS_EXPONENT_MAX + 1}}})
        EXPECT_EQ(linears_fixed_valid(value), 0) << value.mantissa << ", " << value.exponent;
}

TEST(AuditFixedAddSubtract, HalfwaySignsAndNormalizationBoundary) {
    for (const int sign : {-1, 1}) {
        linears_fixed_t result{};
        ASSERT_EQ(linears_fixed_add({sign * kHalf, 0}, {sign * kHalf, -31}, &result), LINEARS_OK);
        EXPECT_TRUE(SameFixed(result, {sign * (kHalf + 1), 0}));
        ASSERT_EQ(linears_fixed_add({sign * kFull, 0}, {sign * kHalf, -31}, &result), LINEARS_OK);
        EXPECT_TRUE(SameFixed(result, {sign * kHalf, 1}));
        // At a power of two, subtraction enters the finer neighboring binade.
        ASSERT_EQ(linears_fixed_sub({sign * kHalf, 0}, {sign * kHalf, -31}, &result), LINEARS_OK);
        EXPECT_TRUE(SameFixed(result, {sign * kFull, -1}));
    }
}

TEST(AuditFixedAddSubtract, CatastrophicCancellationRetainsTheExactResidual) {
    for (const int exponent : {-100, 0, 100}) {
        for (const int sign : {-1, 1}) {
            linears_fixed_t output{};
            const linears_fixed_t a{sign * (kHalf + 1), exponent};
            const linears_fixed_t b{sign * kHalf, exponent};
            ASSERT_EQ(linears_fixed_sub(a, b, &output), LINEARS_OK);
            EXPECT_TRUE(SameFixed(output, {sign * kHalf, exponent - 30}));
            ASSERT_EQ(linears_fixed_add(a, {-b.mantissa, b.exponent}, &output), LINEARS_OK);
            EXPECT_TRUE(RoundedAs(output, AsReal(a) - AsReal(b)));
            const linears_fixed_t lower{sign * kFull, exponent - 1};
            ASSERT_EQ(linears_fixed_sub(b, lower, &output), LINEARS_OK);
            EXPECT_TRUE(SameFixed(output, {sign * kHalf, exponent - 31}));
            ASSERT_EQ(linears_fixed_sub(a, a, &output), LINEARS_OK);
            EXPECT_TRUE(SameFixed(output, kZero));
        }
    }
}

TEST(AuditFixedAddSubtract, AlignmentDistancesAndOperandOrderMatchMath) {
    constexpr std::array<int, 12> distances{0, 1, 2, 15, 29, 30, 31, 32, 33, 34, 64, 80};
    for (const int distance : distances) {
        for (const int32_t ma : {kHalf, kHalf + 1, kFull}) {
            for (const int32_t mb : {kHalf, kHalf + 1, kFull}) {
                for (const int sign_a : {-1, 1}) {
                    for (const int sign_b : {-1, 1}) {
                        const linears_fixed_t a{sign_a * ma, 3};
                        const linears_fixed_t b{sign_b * mb, 3 - distance};
                        linears_fixed_t output{};
                        ASSERT_EQ(linears_fixed_add(a, b, &output), LINEARS_OK);
                        ASSERT_TRUE(RoundedAs(output, AsReal(a) + AsReal(b))) << "distance=" << distance;
                        ASSERT_EQ(linears_fixed_add(b, a, &output), LINEARS_OK);
                        ASSERT_TRUE(RoundedAs(output, AsReal(a) + AsReal(b)));
                        ASSERT_EQ(linears_fixed_sub(a, b, &output), LINEARS_OK);
                        ASSERT_TRUE(RoundedAs(output, AsReal(a) - AsReal(b)));
                    }
                }
            }
        }
    }
}

TEST(AuditFixedAddSubtract, WorkingPrecisionIsNotAnExactFractionalAccumulator) {
    const linears_fixed_t half_ulp{kHalf, -31};
    linears_fixed_t sum{};
    linears_fixed_t residual{};
    ASSERT_EQ(linears_fixed_add({kHalf, 0}, half_ulp, &sum), LINEARS_OK);
    ASSERT_EQ(linears_fixed_sub(sum, {kHalf, 0}, &residual), LINEARS_OK);
    EXPECT_TRUE(SameFixed(residual, {kHalf, -30}));
    EXPECT_NE(AsReal(residual), AsReal(half_ulp));
}

TEST(AuditFixedMultiplyDivide, ProductsAndQuotientsMatchIndependentMath) {
    std::minstd_rand random(92741);
    for (int iteration = 0; iteration < 4000; ++iteration) {
        const auto a = RandomCanonical(random, 100);
        const auto b = RandomCanonical(random, 100);
        linears_fixed_t output{};
        ASSERT_EQ(linears_fixed_mul(a, b, &output), LINEARS_OK);
        ASSERT_TRUE(RoundedAs(output, AsReal(a) * AsReal(b))) << "iteration=" << iteration;
        ASSERT_EQ(linears_fixed_div(a, b, &output), LINEARS_OK);
        ASSERT_TRUE(RoundedAs(output, AsReal(a) / AsReal(b))) << "iteration=" << iteration;
    }
}

TEST(AuditFixedMultiplyDivide, ProductMidpointsRoundAwayForBothSigns) {
    for (const int sign_a : {-1, 1}) {
        for (const int sign_b : {-1, 1}) {
            const linears_fixed_t a{sign_a * (kHalf + 1), 0};
            const linears_fixed_t b{sign_b * INT32_C(1610612736), 0};
            linears_fixed_t output{};
            ASSERT_EQ(linears_fixed_mul(a, b, &output), LINEARS_OK);
            EXPECT_TRUE(SameFixed(output, {sign_a * sign_b * INT32_C(1610612738), -1}));
            EXPECT_TRUE(RoundedAs(output, AsReal(a) * AsReal(b)));
        }
    }
}

TEST(AuditFixedMultiplyDivide, DivisionNearHalfwayAndExactIdentities) {
    for (const int sign : {-1, 1}) {
        linears_fixed_t output{};
        // 2^61/(2^31-1) = 2^30 + 1/2 + 1/(2*(2^31-1)).
        ASSERT_EQ(linears_fixed_div({sign * kHalf, 0}, {kFull, 0}, &output), LINEARS_OK);
        EXPECT_TRUE(SameFixed(output, {sign * (kHalf + 1), 0}));
        ASSERT_EQ(linears_fixed_div({sign * kHalf, 0}, {kFull - 2, 0}, &output), LINEARS_OK);
        EXPECT_TRUE(SameFixed(output, {sign * (kHalf + 2), 0}));
        for (const int exponent : {-100, 0, 100}) {
            const linears_fixed_t value{sign * kFull, exponent};
            ASSERT_EQ(linears_fixed_div(value, value, &output), LINEARS_OK);
            EXPECT_TRUE(SameFixed(output, kOne));
            ASSERT_EQ(linears_fixed_mul(value, kOne, &output), LINEARS_OK);
            EXPECT_TRUE(SameFixed(output, value));
            ASSERT_EQ(linears_fixed_div(value, kOne, &output), LINEARS_OK);
            EXPECT_TRUE(SameFixed(output, value));
        }
    }
}

TEST(AuditFixedSquareRoot, NegativeOddAndEvenExponentsMatchIndependentMath) {
    for (int exponent = -101; exponent <= 101; ++exponent) {
        for (const int32_t mantissa : {kHalf, kHalf + 1, INT32_C(1610612736), kFull - 1, kFull}) {
            const linears_fixed_t value{mantissa, exponent};
            linears_fixed_t result{};
            ASSERT_EQ(linears_fixed_sqrt(value, &result), LINEARS_OK);
            ASSERT_TRUE(RoundedAs(result, std::sqrt(AsReal(value))))
                << "mantissa=" << mantissa << ", exponent=" << exponent;
        }
    }
    std::minstd_rand random(75431);
    for (int iteration = 0; iteration < 4000; ++iteration) {
        auto value = RandomCanonical(random, 100);
        if (value.mantissa < 0) value.mantissa = -value.mantissa;
        linears_fixed_t result{};
        ASSERT_EQ(linears_fixed_sqrt(value, &result), LINEARS_OK);
        ASSERT_TRUE(RoundedAs(result, std::sqrt(AsReal(value)))) << "iteration=" << iteration;
    }
}

TEST(AuditFixedSquareRoot, ExactSquaresAndExponentExtremes) {
    linears_fixed_t result{};
    ASSERT_EQ(linears_fixed_sqrt(kZero, &result), LINEARS_OK);
    EXPECT_TRUE(SameFixed(result, kZero));
    for (const int exponent : {LINEARS_EXPONENT_MIN, -101, -1, 1, 101,
                               LINEARS_EXPONENT_MAX - 1, LINEARS_EXPONENT_MAX}) {
        const linears_fixed_t value{kHalf, exponent};
        ASSERT_EQ(linears_fixed_sqrt(value, &result), LINEARS_OK);
        if (exponent % 2 != 0) {
            EXPECT_TRUE(SameFixed(result, {kHalf, (exponent + 1) / 2}));
        } else {
            EXPECT_TRUE(SameFixed(result, {INT32_C(1518500250), exponent / 2}));
        }
    }
}

TEST(AuditFixedOrdering, SignZeroAndHugeExponentDifferences) {
    constexpr std::array<linears_fixed_t, 9> increasing{{
        {-kFull, LINEARS_EXPONENT_MAX}, {-kHalf, LINEARS_EXPONENT_MAX},
        {-kFull, LINEARS_EXPONENT_MIN}, {-kHalf, LINEARS_EXPONENT_MIN}, kZero,
        {kHalf, LINEARS_EXPONENT_MIN}, {kFull, LINEARS_EXPONENT_MIN},
        {kHalf, LINEARS_EXPONENT_MAX}, {kFull, LINEARS_EXPONENT_MAX}}};
    for (size_t a = 0; a < increasing.size(); ++a)
        for (size_t b = 0; b < increasing.size(); ++b)
            EXPECT_EQ(linears_fixed_compare(increasing[a], increasing[b]),
                      (a > b) - (a < b)) << "a=" << a << ", b=" << b;
    std::minstd_rand random(2347);
    for (int iteration = 0; iteration < 1000; ++iteration) {
        const auto a = RandomCanonical(random, 100);
        const auto b = RandomCanonical(random, 100);
        EXPECT_EQ(linears_fixed_compare(a, b), (AsReal(a) > AsReal(b)) - (AsReal(a) < AsReal(b)));
    }
}

TEST(AuditFixedOrdering, NegationAndAbsoluteValueDoNotChangeMagnitudeOrScale) {
    for (const int exponent : {LINEARS_EXPONENT_MIN, -1, 0, 1, LINEARS_EXPONENT_MAX}) {
        for (const int32_t mantissa : {-kFull, -kHalf, kHalf, kFull}) {
            const linears_fixed_t value{mantissa, exponent};
            linears_fixed_t output{};
            ASSERT_EQ(linears_fixed_neg(value, &output), LINEARS_OK);
            EXPECT_TRUE(SameFixed(output, {-mantissa, exponent}));
            ASSERT_EQ(linears_fixed_abs(value, &output), LINEARS_OK);
            EXPECT_TRUE(SameFixed(output, {mantissa < 0 ? -mantissa : mantissa, exponent}));
        }
    }
    linears_fixed_t output{};
    ASSERT_EQ(linears_fixed_neg(kZero, &output), LINEARS_OK);
    EXPECT_TRUE(SameFixed(output, kZero));
    ASSERT_EQ(linears_fixed_abs(kZero, &output), LINEARS_OK);
    EXPECT_TRUE(SameFixed(output, kZero));
}

TEST(AuditFixedQuantize, Q15HalfwayAndSignedStorageBoundaries) {
    struct Case { linears_fixed_t value; linears_status_t status; int16_t output; };
    constexpr std::array<Case, 13> cases{{
        {{kHalf, -15}, LINEARS_OK, 1},
        {{-kHalf, -15}, LINEARS_OK, -1},
        {{kHalf, -16}, LINEARS_OK, 0},
        {{-kHalf, -16}, LINEARS_OK, 0},
        {{INT32_C(0x7fff7fff), 0}, LINEARS_OK, INT16_MAX},
        {{INT32_C(0x7fff8000), 0}, LINEARS_ERROR_RANGE, 0},
        {{-INT32_C(0x7fff8000), 0}, LINEARS_OK, INT16_MIN},
        {{-kFull, 0}, LINEARS_OK, INT16_MIN},
        {{kHalf, 1}, LINEARS_ERROR_RANGE, 0},
        {{-kHalf, 1}, LINEARS_OK, INT16_MIN},
        {{-(kHalf + 16383), 1}, LINEARS_OK, INT16_MIN},
        {{-(kHalf + 16384), 1}, LINEARS_ERROR_RANGE, 0},
        {{kFull, 1}, LINEARS_ERROR_RANGE, 0},
    }};
    for (const auto &item : cases) {
        int16_t output = 117;
        ASSERT_EQ(linears_fixed_to_q15(item.value, 0, &output), item.status);
        EXPECT_EQ(output, item.status == LINEARS_OK ? item.output : 117);
    }
}

TEST(AuditFixedQuantize, Q31HalfwayAndSignedStorageBoundaries) {
    struct Case { linears_fixed_t value; int exponent; linears_status_t status; int32_t output; };
    constexpr std::array<Case, 10> cases{{
        {{kHalf + 1, 0}, 1, LINEARS_OK, 536870913},
        {{-kHalf - 1, 0}, 1, LINEARS_OK, -536870913},
        {{kHalf, -31}, 0, LINEARS_OK, 1},
        {{-kHalf, -31}, 0, LINEARS_OK, -1},
        {{kHalf, -32}, 0, LINEARS_OK, 0},
        {{-kHalf, -32}, 0, LINEARS_OK, 0},
        {{kFull, 0}, 0, LINEARS_OK, INT32_MAX},
        {{-kHalf, 1}, 0, LINEARS_OK, INT32_MIN},
        {{kHalf, 1}, 0, LINEARS_ERROR_RANGE, 0},
        {{-kHalf - 1, 1}, 0, LINEARS_ERROR_RANGE, 0},
    }};
    for (const auto &item : cases) {
        int32_t output = 119;
        ASSERT_EQ(linears_fixed_to_q31(item.value, item.exponent, &output), item.status);
        EXPECT_EQ(output, item.status == LINEARS_OK ? item.output : 119);
    }
}

TEST(AuditFixedQuantize, DifferentScalesMatchMathAndUnderflowToZero) {
    std::minstd_rand random(21431);
    for (int iteration = 0; iteration < 3000; ++iteration) {
        const auto value = RandomCanonical(random, 30);
        const int exponent = static_cast<int>(random() % 81) - 40;
        const long double exact15 = std::round(std::ldexp(AsReal(value), 15 - exponent));
        const long double exact31 = std::round(std::ldexp(AsReal(value), 31 - exponent));
        int16_t q15 = 123;
        int32_t q31 = 123;
        const bool fits15 = exact15 >= INT16_MIN && exact15 <= INT16_MAX;
        const bool fits31 = exact31 >= INT32_MIN && exact31 <= INT32_MAX;
        ASSERT_EQ(linears_fixed_to_q15(value, exponent, &q15),
                  fits15 ? LINEARS_OK : LINEARS_ERROR_RANGE);
        ASSERT_EQ(q15, fits15 ? static_cast<int16_t>(exact15) : 123);
        ASSERT_EQ(linears_fixed_to_q31(value, exponent, &q31),
                  fits31 ? LINEARS_OK : LINEARS_ERROR_RANGE);
        ASSERT_EQ(q31, fits31 ? static_cast<int32_t>(exact31) : 123);
    }
    for (const int sign : {-1, 1}) {
        const linears_fixed_t tiny{sign * kFull, LINEARS_EXPONENT_MIN};
        int16_t q15 = 1;
        int32_t q31 = 1;
        ASSERT_EQ(linears_fixed_to_q15(tiny, LINEARS_EXPONENT_MAX, &q15), LINEARS_OK);
        ASSERT_EQ(linears_fixed_to_q31(tiny, LINEARS_EXPONENT_MAX, &q31), LINEARS_OK);
        EXPECT_EQ(q15, 0);
        EXPECT_EQ(q31, 0);
    }
}

TEST(AuditFixedExport, PositiveQ15RoundingCarryRescalesEveryElement) {
    constexpr std::array<linears_fixed_t, 5> input{{
        {kHalf, 0}, {INT32_C(0x7fff8000), 0}, {-kHalf, 0}, {kFull, -40}, kZero}};
    std::array<int16_t, input.size()> output{};
    int exponent = 999;
    ASSERT_EQ(linears_fixed_export_q15(input.data(), input.size(), output.data(), &exponent), LINEARS_OK);
    EXPECT_EQ(exponent, 1);
    EXPECT_EQ(output, (std::array<int16_t, 5>{8192, 16384, -8192, 0, 0}));
    int reference_exponent = 999;
    EXPECT_EQ(output, ExportReference<int16_t>(input, &reference_exponent));
    EXPECT_EQ(exponent, reference_exponent);
}

TEST(AuditFixedExport, NegativeQ15LimitDoesNotForcePositiveCarry) {
    constexpr std::array<linears_fixed_t, 3> input{{{-kFull, 0}, {kHalf, 0}, kZero}};
    std::array<int16_t, input.size()> output{};
    int exponent = 999;
    ASSERT_EQ(linears_fixed_export_q15(input.data(), input.size(), output.data(), &exponent), LINEARS_OK);
    EXPECT_EQ(exponent, 0);
    EXPECT_EQ(output, (std::array<int16_t, 3>{INT16_MIN, 16384, 0}));
}

TEST(AuditFixedExport, SharedExponentBlocksMatchMathAndQuantizationError) {
    std::minstd_rand random(71341);
    for (int iteration = 0; iteration < 500; ++iteration) {
        std::array<linears_fixed_t, 17> input{};
        for (auto &value : input) value = RandomCanonical(random, 24);
        input[3] = kZero;
        std::array<int16_t, input.size()> q15{};
        std::array<int32_t, input.size()> q31{};
        int exponent15 = 999;
        int exponent31 = 999;
        int reference15 = 999;
        int reference31 = 999;
        const auto expected15 = ExportReference<int16_t>(input, &reference15);
        const auto expected31 = ExportReference<int32_t>(input, &reference31);
        ASSERT_EQ(linears_fixed_export_q15(input.data(), input.size(), q15.data(), &exponent15), LINEARS_OK);
        ASSERT_EQ(linears_fixed_export_q31(input.data(), input.size(), q31.data(), &exponent31), LINEARS_OK);
        ASSERT_EQ(exponent15, reference15);
        ASSERT_EQ(exponent31, reference31);
        ASSERT_EQ(q15, expected15);
        ASSERT_EQ(q31, expected31);
        for (size_t index = 0; index < input.size(); ++index) {
            const long double restored15 = std::ldexp(static_cast<long double>(q15[index]), exponent15 - 15);
            const long double restored31 = std::ldexp(static_cast<long double>(q31[index]), exponent31 - 31);
            ASSERT_LE(std::fabs(restored15 - AsReal(input[index])), std::ldexp(0.5L, exponent15 - 15));
            ASSERT_LE(std::fabs(restored31 - AsReal(input[index])), std::ldexp(0.5L, exponent31 - 31));
        }
    }
}

TEST(AuditFixedExport, Q31EqualScaleRoundTripsExactly) {
    for (const int exponent : {-100, 0, 100}) {
        const std::array<linears_fixed_t, 5> input{{
            {-kFull, exponent}, {-kHalf, exponent}, kZero, {kHalf, exponent}, {kFull, exponent}}};
        std::array<int32_t, input.size()> output{};
        int exported_exponent = 999;
        ASSERT_EQ(linears_fixed_export_q31(input.data(), input.size(), output.data(), &exported_exponent), LINEARS_OK);
        EXPECT_EQ(exported_exponent, exponent);
        for (size_t index = 0; index < input.size(); ++index) {
            linears_fixed_t restored{};
            ASSERT_EQ(linears_fixed_from_q31({output[index], exported_exponent}, &restored), LINEARS_OK);
            EXPECT_TRUE(SameFixed(restored, input[index]));
        }
    }
}

TEST(AuditFixedExport, EmptyAndAllZeroBlocksHaveExponentZero) {
    int exponent = 456;
    ASSERT_EQ(linears_fixed_export_q15(nullptr, 0, nullptr, &exponent), LINEARS_OK);
    EXPECT_EQ(exponent, 0);
    exponent = 456;
    ASSERT_EQ(linears_fixed_export_q31(nullptr, 0, nullptr, &exponent), LINEARS_OK);
    EXPECT_EQ(exponent, 0);
    const std::array<linears_fixed_t, 3> zeros{};
    std::array<int16_t, 3> q15{1, 2, 3};
    std::array<int32_t, 3> q31{1, 2, 3};
    ASSERT_EQ(linears_fixed_export_q15(zeros.data(), zeros.size(), q15.data(), &exponent), LINEARS_OK);
    EXPECT_EQ(q15, (std::array<int16_t, 3>{}));
    EXPECT_EQ(exponent, 0);
    exponent = 456;
    ASSERT_EQ(linears_fixed_export_q31(zeros.data(), zeros.size(), q31.data(), &exponent), LINEARS_OK);
    EXPECT_EQ(q31, (std::array<int32_t, 3>{}));
    EXPECT_EQ(exponent, 0);
}

TEST(AuditFixedAtomicFailure, ImportsPreserveOutputForArgumentAndRangeFailures) {
    for (const int bits : {-1, 64, INT_MIN, INT_MAX})
        ExpectAtomicScalarFailure(LINEARS_ERROR_ARGUMENT, [bits](auto *out) {
            return linears_fixed_from_i64(1, bits, 0, out);
        });
    for (const int exponent : {LINEARS_EXPONENT_MIN - 1, LINEARS_EXPONENT_MAX + 1, INT_MIN, INT_MAX}) {
        ExpectAtomicScalarFailure(LINEARS_ERROR_RANGE, [exponent](auto *out) {
            return linears_fixed_from_i64(0, 0, exponent, out);
        });
        ExpectAtomicScalarFailure(LINEARS_ERROR_RANGE, [exponent](auto *out) {
            return linears_fixed_from_q15({1, exponent}, out);
        });
        ExpectAtomicScalarFailure(LINEARS_ERROR_RANGE, [exponent](auto *out) {
            return linears_fixed_from_q31({1, exponent}, out);
        });
    }
    ExpectAtomicScalarFailure(LINEARS_ERROR_RANGE, [](auto *out) {
        return linears_fixed_from_i64(1, 63, LINEARS_EXPONENT_MIN, out);
    });
    ExpectAtomicScalarFailure(LINEARS_ERROR_RANGE, [](auto *out) {
        return linears_fixed_from_q15({1, LINEARS_EXPONENT_MIN}, out);
    });
    ExpectAtomicScalarFailure(LINEARS_ERROR_RANGE, [](auto *out) {
        return linears_fixed_from_q31({INT32_MIN, LINEARS_EXPONENT_MAX}, out);
    });
    EXPECT_EQ(linears_fixed_from_i64(0, 0, 0, nullptr), LINEARS_ERROR_ARGUMENT);
    EXPECT_EQ(linears_fixed_from_q15({0, 0}, nullptr), LINEARS_ERROR_ARGUMENT);
    EXPECT_EQ(linears_fixed_from_q31({0, 0}, nullptr), LINEARS_ERROR_ARGUMENT);
}

TEST(AuditFixedAtomicFailure, ArithmeticRejectsNoncanonicalOperandsWithoutWriting) {
    constexpr std::array<linears_fixed_t, 7> invalid{{
        {0, 1}, {1, 0}, {-1, 0}, {kHalf - 1, 0}, {INT32_MIN, 0},
        {kHalf, LINEARS_EXPONENT_MIN - 1}, {kHalf, LINEARS_EXPONENT_MAX + 1}}};
    for (const auto value : invalid) {
        for (const auto operation : kBinaryOperations) {
            ExpectAtomicScalarFailure(LINEARS_ERROR_ARGUMENT, [=](auto *out) { return operation(value, kOne, out); });
            ExpectAtomicScalarFailure(LINEARS_ERROR_ARGUMENT, [=](auto *out) { return operation(kOne, value, out); });
        }
        ExpectAtomicScalarFailure(LINEARS_ERROR_ARGUMENT, [=](auto *out) { return linears_fixed_sqrt(value, out); });
        ExpectAtomicScalarFailure(LINEARS_ERROR_ARGUMENT, [=](auto *out) { return linears_fixed_neg(value, out); });
        ExpectAtomicScalarFailure(LINEARS_ERROR_ARGUMENT, [=](auto *out) { return linears_fixed_abs(value, out); });
    }
    for (const auto operation : kBinaryOperations)
        EXPECT_EQ(operation(kOne, kOne, nullptr), LINEARS_ERROR_ARGUMENT);
    EXPECT_EQ(linears_fixed_sqrt(kOne, nullptr), LINEARS_ERROR_ARGUMENT);
    EXPECT_EQ(linears_fixed_neg(kOne, nullptr), LINEARS_ERROR_ARGUMENT);
    EXPECT_EQ(linears_fixed_abs(kOne, nullptr), LINEARS_ERROR_ARGUMENT);
}

TEST(AuditFixedAtomicFailure, RangeAndDomainFailuresPreserveScalarAndCanaries) {
    const linears_fixed_t largest{kFull, LINEARS_EXPONENT_MAX};
    const linears_fixed_t smallest{kHalf, LINEARS_EXPONENT_MIN};
    ExpectAtomicScalarFailure(LINEARS_ERROR_RANGE, [=](auto *out) { return linears_fixed_add(largest, largest, out); });
    ExpectAtomicScalarFailure(LINEARS_ERROR_RANGE, [=](auto *out) {
        return linears_fixed_sub({kHalf + 1, LINEARS_EXPONENT_MIN}, smallest, out);
    });
    ExpectAtomicScalarFailure(LINEARS_ERROR_RANGE, [=](auto *out) { return linears_fixed_mul(largest, largest, out); });
    ExpectAtomicScalarFailure(LINEARS_ERROR_RANGE, [=](auto *out) { return linears_fixed_mul(smallest, smallest, out); });
    ExpectAtomicScalarFailure(LINEARS_ERROR_RANGE, [=](auto *out) { return linears_fixed_div(largest, smallest, out); });
    ExpectAtomicScalarFailure(LINEARS_ERROR_RANGE, [=](auto *out) { return linears_fixed_div(smallest, largest, out); });
    ExpectAtomicScalarFailure(LINEARS_ERROR_DOMAIN, [](auto *out) { return linears_fixed_div(kOne, kZero, out); });
    ExpectAtomicScalarFailure(LINEARS_ERROR_DOMAIN, [](auto *out) { return linears_fixed_div(kZero, kZero, out); });
    ExpectAtomicScalarFailure(LINEARS_ERROR_DOMAIN, [](auto *out) { return linears_fixed_sqrt({-kHalf, 1}, out); });
}

TEST(AuditFixedAtomicFailure, QuantizationRejectsArgumentsWithoutWriting) {
    for (const auto value : std::array<linears_fixed_t, 3>{{{0, 1}, {1, 0}, {INT32_MIN, 0}}}) {
        int16_t q15 = 321;
        int32_t q31 = 654;
        EXPECT_EQ(linears_fixed_to_q15(value, 0, &q15), LINEARS_ERROR_ARGUMENT);
        EXPECT_EQ(linears_fixed_to_q31(value, 0, &q31), LINEARS_ERROR_ARGUMENT);
        EXPECT_EQ(q15, 321);
        EXPECT_EQ(q31, 654);
    }
    for (const int exponent : {LINEARS_EXPONENT_MIN - 1, LINEARS_EXPONENT_MAX + 1, INT_MIN, INT_MAX}) {
        int16_t q15 = 321;
        int32_t q31 = 654;
        EXPECT_EQ(linears_fixed_to_q15(kZero, exponent, &q15), LINEARS_ERROR_RANGE);
        EXPECT_EQ(linears_fixed_to_q31(kZero, exponent, &q31), LINEARS_ERROR_RANGE);
        EXPECT_EQ(q15, 321);
        EXPECT_EQ(q31, 654);
    }
    EXPECT_EQ(linears_fixed_to_q15(kOne, 0, nullptr), LINEARS_ERROR_ARGUMENT);
    EXPECT_EQ(linears_fixed_to_q31(kOne, 0, nullptr), LINEARS_ERROR_ARGUMENT);
}

TEST(AuditFixedAtomicFailure, ExportValidatesTheLastElementBeforeAnyWrite) {
    const std::array<linears_fixed_t, 4> input{{kOne, kZero, {kHalf, -10}, {1, 0}}};
    const std::array<int16_t, 6> before15{61, 62, 63, 64, 65, 66};
    const std::array<int32_t, 6> before31{71, 72, 73, 74, 75, 76};
    auto out15 = before15;
    auto out31 = before31;
    int exponent = 77;
    EXPECT_EQ(linears_fixed_export_q15(input.data(), input.size(), out15.data() + 1, &exponent), LINEARS_ERROR_ARGUMENT);
    EXPECT_EQ(out15, before15);
    EXPECT_EQ(exponent, 77);
    EXPECT_EQ(linears_fixed_export_q31(input.data(), input.size(), out31.data() + 1, &exponent), LINEARS_ERROR_ARGUMENT);
    EXPECT_EQ(out31, before31);
    EXPECT_EQ(exponent, 77);
    const std::array<linears_fixed_t, 3> carry_overflow{{
        {kHalf, LINEARS_EXPONENT_MAX}, kZero, {kFull, LINEARS_EXPONENT_MAX}}};
    EXPECT_EQ(linears_fixed_export_q15(carry_overflow.data(), carry_overflow.size(), out15.data() + 1, &exponent),
              LINEARS_ERROR_RANGE);
    EXPECT_EQ(out15, before15);
    EXPECT_EQ(exponent, 77);
}

TEST(AuditFixedAliasing, ScalarsAllowOutputToReplaceEitherByValueOperand) {
    const linears_fixed_t a{kHalf + 17, 2};
    const linears_fixed_t b{-kHalf - 31, -1};
    const std::array<long double, 4> expected{
        AsReal(a) + AsReal(b), AsReal(a) - AsReal(b), AsReal(a) * AsReal(b), AsReal(a) / AsReal(b)};
    for (size_t index = 0; index < kBinaryOperations.size(); ++index) {
        auto left = a;
        auto right = b;
        ASSERT_EQ(kBinaryOperations[index](left, right, &left), LINEARS_OK);
        EXPECT_TRUE(RoundedAs(left, expected[index]));
        left = a;
        ASSERT_EQ(kBinaryOperations[index](left, right, &right), LINEARS_OK);
        EXPECT_TRUE(RoundedAs(right, expected[index]));
    }
    auto value = a;
    ASSERT_EQ(linears_fixed_sqrt(value, &value), LINEARS_OK);
    EXPECT_TRUE(RoundedAs(value, std::sqrt(AsReal(a))));
    value = b;
    ASSERT_EQ(linears_fixed_abs(value, &value), LINEARS_OK);
    EXPECT_TRUE(SameFixed(value, {-b.mantissa, b.exponent}));
    ASSERT_EQ(linears_fixed_neg(value, &value), LINEARS_OK);
    EXPECT_TRUE(SameFixed(value, b));
    value = {kFull, LINEARS_EXPONENT_MAX};
    EXPECT_EQ(linears_fixed_add(value, value, &value), LINEARS_ERROR_RANGE);
    EXPECT_TRUE(SameFixed(value, {kFull, LINEARS_EXPONENT_MAX}));
}

TEST(AuditFixedAliasing, ExportRejectsInputOutputAndExponentOverlapsAtomically) {
    std::array<linears_fixed_t, 4> values{{kOne, {kHalf, 0}, {-kHalf, 0}, kZero}};
    const auto before = values;
    std::array<int16_t, 4> q15{41, 42, 43, 44};
    std::array<int32_t, 4> q31{51, 52, 53, 54};
    const auto before15 = q15;
    const auto before31 = q31;
    int exponent = 71;
    for (const size_t offset : {size_t{0}, size_t{1}, size_t{3}}) {
        EXPECT_EQ(linears_fixed_export_q15(values.data(), values.size(),
                  reinterpret_cast<int16_t *>(values.data() + offset), &exponent), LINEARS_ERROR_ALIAS);
        EXPECT_EQ(linears_fixed_export_q31(values.data(), values.size(),
                  reinterpret_cast<int32_t *>(values.data() + offset), &exponent), LINEARS_ERROR_ALIAS);
        EXPECT_EQ(std::memcmp(values.data(), before.data(), sizeof(values)), 0);
        EXPECT_EQ(exponent, 71);
    }
    EXPECT_EQ(linears_fixed_export_q15(values.data(), values.size(), q15.data(), &values.back().exponent), LINEARS_ERROR_ALIAS);
    EXPECT_EQ(linears_fixed_export_q31(values.data(), values.size(), q31.data(), &values.back().exponent), LINEARS_ERROR_ALIAS);
    EXPECT_EQ(std::memcmp(values.data(), before.data(), sizeof(values)), 0);
    EXPECT_EQ(q15, before15);
    EXPECT_EQ(q31, before31);
    alignas(linears_workspace_unit_t) std::array<int, 4> overlapped{81, 82, 83, 84};
    const auto original = overlapped;
    EXPECT_EQ(linears_fixed_export_q15(values.data(), values.size(),
              reinterpret_cast<int16_t *>(overlapped.data()), overlapped.data()), LINEARS_ERROR_ALIAS);
    EXPECT_EQ(overlapped, original);
    EXPECT_EQ(linears_fixed_export_q31(values.data(), values.size(),
              reinterpret_cast<int32_t *>(overlapped.data()), overlapped.data() + 3), LINEARS_ERROR_ALIAS);
    EXPECT_EQ(overlapped, original);
}

TEST(AuditFixedAliasing, AdjacentDisjointObjectsAndOutputCanariesAreAccepted) {
    struct Adjacent {
        std::array<linears_fixed_t, 3> input;
        std::array<int32_t, 3> output;
        int exponent;
        uint32_t canary;
    } data{{{{kHalf, 0}, {-kHalf, 0}, kZero}}, {{11, 12, 13}}, 37, kCanary};
    const auto before_input = data.input;
    ASSERT_EQ(linears_fixed_export_q31(data.input.data(), data.input.size(), data.output.data(), &data.exponent), LINEARS_OK);
    EXPECT_EQ(data.output, (std::array<int32_t, 3>{kHalf, -kHalf, 0}));
    EXPECT_EQ(data.exponent, 0);
    EXPECT_EQ(data.canary, kCanary);
    EXPECT_EQ(std::memcmp(data.input.data(), before_input.data(), sizeof(data.input)), 0);
    std::array<int16_t, 5> out15{11, 12, 13, 14, 15};
    ASSERT_EQ(linears_fixed_export_q15(data.input.data(), data.input.size(), out15.data() + 1, &data.exponent), LINEARS_OK);
    EXPECT_EQ(out15, (std::array<int16_t, 5>{11, 16384, -16384, 0, 15}));
}

TEST(AuditFixedStorage, ExportRejectsCountOverflowBeforeReadingBeyondOneElement) {
    const linears_fixed_t input = kOne;
    for (const size_t count : {SIZE_MAX / sizeof(linears_fixed_t) + 1, SIZE_MAX / sizeof(int32_t) + 1, SIZE_MAX}) {
        int16_t q15 = 123;
        int32_t q31 = 456;
        int exponent = 789;
        EXPECT_EQ(linears_fixed_export_q15(&input, count, &q15, &exponent), LINEARS_ERROR_SIZE);
        EXPECT_EQ(linears_fixed_export_q31(&input, count, &q31, &exponent), LINEARS_ERROR_SIZE);
        EXPECT_EQ(q15, 123);
        EXPECT_EQ(q31, 456);
        EXPECT_EQ(exponent, 789);
        EXPECT_TRUE(SameFixed(input, kOne));
    }
}

TEST(AuditFixedStorage, ExportRejectsNullAndMisalignedBuffersAtomically) {
    const linears_fixed_t input = kOne;
    int16_t q15 = 123;
    int32_t q31 = 456;
    int exponent = 789;
    EXPECT_EQ(linears_fixed_export_q15(nullptr, 1, &q15, &exponent), LINEARS_ERROR_ARGUMENT);
    EXPECT_EQ(linears_fixed_export_q31(nullptr, 1, &q31, &exponent), LINEARS_ERROR_ARGUMENT);
    EXPECT_EQ(linears_fixed_export_q15(&input, 1, nullptr, &exponent), LINEARS_ERROR_ARGUMENT);
    EXPECT_EQ(linears_fixed_export_q31(&input, 1, nullptr, &exponent), LINEARS_ERROR_ARGUMENT);
    EXPECT_EQ(linears_fixed_export_q15(&input, 1, &q15, nullptr), LINEARS_ERROR_ARGUMENT);
    EXPECT_EQ(linears_fixed_export_q31(&input, 1, &q31, nullptr), LINEARS_ERROR_ARGUMENT);
    EXPECT_EQ(linears_fixed_export_q15(nullptr, 0, nullptr, nullptr), LINEARS_ERROR_ARGUMENT);
    EXPECT_EQ(linears_fixed_export_q31(nullptr, 0, nullptr, nullptr), LINEARS_ERROR_ARGUMENT);
    alignas(linears_workspace_unit_t) std::array<unsigned char, 64> bytes{};
    bytes.fill(0xa7);
    const auto before = bytes;
    const auto *bad_input = reinterpret_cast<const linears_fixed_t *>(bytes.data() + 1);
    auto *bad_exponent = reinterpret_cast<int *>(bytes.data() + 1);
    EXPECT_EQ(linears_fixed_export_q15(bad_input, 1, &q15, &exponent), LINEARS_ERROR_ARGUMENT);
    EXPECT_EQ(linears_fixed_export_q31(bad_input, 1, &q31, &exponent), LINEARS_ERROR_ARGUMENT);
    EXPECT_EQ(linears_fixed_export_q15(&input, 1, reinterpret_cast<int16_t *>(bytes.data() + 1), &exponent), LINEARS_ERROR_ARGUMENT);
    EXPECT_EQ(linears_fixed_export_q31(&input, 1, reinterpret_cast<int32_t *>(bytes.data() + 1), &exponent), LINEARS_ERROR_ARGUMENT);
    EXPECT_EQ(linears_fixed_export_q15(&input, 1, &q15, bad_exponent), LINEARS_ERROR_ARGUMENT);
    EXPECT_EQ(linears_fixed_export_q31(&input, 1, &q31, bad_exponent), LINEARS_ERROR_ARGUMENT);
    EXPECT_EQ(bytes, before);
    EXPECT_EQ(q15, 123);
    EXPECT_EQ(q31, 456);
    EXPECT_EQ(exponent, 789);
}

TEST(AuditFixedStorage, WorkspaceBoundsAreConstantExpressionsAndDoNotWrap) {
    constexpr size_t unit = sizeof(linears_workspace_unit_t);
    constexpr size_t bytes = LINEARS_BASIC_WORKSPACE_BYTES(3, 7);
    constexpr size_t units = LINEARS_WORKSPACE_UNITS(bytes);
    static_assert(bytes == 21 * unit, "Basic workspace must cover every working element");
    static_assert(units == 21, "An exact byte multiple must not require an extra unit");
    static_assert(LINEARS_BASIC_WORKSPACE_BYTES(SIZE_MAX, 0) == 0, "Empty matrices need no workspace");
    static_assert(LINEARS_BASIC_WORKSPACE_BYTES(0, SIZE_MAX) == 0, "Empty matrices need no workspace");
    static_assert(LINEARS_SIZE_MUL(SIZE_MAX, 2) == SIZE_MAX, "Multiplication overflow must be explicit");
    static_assert(LINEARS_SIZE_ADD(SIZE_MAX, 1) == SIZE_MAX, "Addition overflow must be explicit");
    static_assert(LINEARS_WORKSPACE_UNITS(SIZE_MAX) == SIZE_MAX, "Invalid size must propagate");
    std::array<linears_workspace_unit_t, units> workspace{};
    EXPECT_EQ(sizeof(workspace), bytes);
    EXPECT_GE(alignof(decltype(workspace)), alignof(linears_fixed_t));
    EXPECT_GE(alignof(decltype(workspace)), alignof(int64_t));
    EXPECT_GE(alignof(decltype(workspace)), alignof(double));
    EXPECT_EQ(LINEARS_BASIC_WORKSPACE_BYTES(SIZE_MAX / unit + 1, 1), SIZE_MAX);
    EXPECT_EQ(LINEARS_BASIC_WORKSPACE_BYTES(SIZE_MAX, SIZE_MAX), SIZE_MAX);
    EXPECT_EQ(LINEARS_SIZE_ADD(SIZE_MAX - 7, 7), SIZE_MAX);
    EXPECT_EQ(LINEARS_SIZE_ADD(SIZE_MAX - 7, 6), SIZE_MAX - 1);
    for (size_t requested = 0; requested <= 3 * unit; ++requested) {
        const size_t capacity = LINEARS_WORKSPACE_UNITS(requested) * unit;
        EXPECT_GE(capacity, requested);
        EXPECT_LT(capacity - requested, unit);
    }
    EXPECT_EQ(LINEARS_WORKSPACE_UNITS(SIZE_MAX - 1),
              (SIZE_MAX - 1) / unit + ((SIZE_MAX - 1) % unit != 0));
}

TEST(AuditFixedExponentBoundary, ZeroAndCancellationDoNotRaiseSpuriousRangeErrors) {
    for (const int exponent : {LINEARS_EXPONENT_MIN, LINEARS_EXPONENT_MAX}) {
        const linears_fixed_t value{kFull, exponent};
        linears_fixed_t result = kSentinel;
        ASSERT_EQ(linears_fixed_add(value, kZero, &result), LINEARS_OK);
        EXPECT_TRUE(SameFixed(result, value));
        ASSERT_EQ(linears_fixed_sub(value, value, &result), LINEARS_OK);
        EXPECT_TRUE(SameFixed(result, kZero));
        ASSERT_EQ(linears_fixed_add(value, {-value.mantissa, value.exponent}, &result), LINEARS_OK);
        EXPECT_TRUE(SameFixed(result, kZero));
        ASSERT_EQ(linears_fixed_mul(value, kZero, &result), LINEARS_OK);
        EXPECT_TRUE(SameFixed(result, kZero));
        ASSERT_EQ(linears_fixed_div(kZero, value, &result), LINEARS_OK);
        EXPECT_TRUE(SameFixed(result, kZero));
        ASSERT_EQ(linears_fixed_div(value, value, &result), LINEARS_OK);
        EXPECT_TRUE(SameFixed(result, kOne));
        ASSERT_EQ(linears_fixed_from_i64(0, 63, exponent, &result), LINEARS_OK);
        EXPECT_TRUE(SameFixed(result, kZero));
    }
}

}  // namespace
