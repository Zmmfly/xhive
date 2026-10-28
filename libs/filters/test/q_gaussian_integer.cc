/* These tests change the rounding mode, including for the private arithmetic
 * definitions below. Without dynamic-fenv semantics an inlined constant
 * conversion may legally be folded in round-to-nearest while the wide path
 * still converts at runtime. Keep this declaration local to this test TU. */
#if defined(__clang__)
#pragma STDC FENV_ACCESS ON
#elif defined(__GNUC__)
#pragma GCC optimize ("rounding-math")
#endif

#include "pf_support.h"
#include "../src/ukf_internal.h"

#include <gtest/gtest.h>

#include <array>
#include <cfenv>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <random>
#include <type_traits>
#include <vector>

U_ALIGNMENT_TYPE(filters_pf_q15_t);
U_ALIGNMENT_TYPE(filters_pf_q31_t);
#define U_TAG q15
#define U_KERNEL f32
#define U_SIGNAL int16_t
#define U_SIGNAL_MAX INT16_MAX
#define U_WORK float
#define U_STATE filters_pf_q15_t
#define U_BITS 15
#define U_SCALE INT64_C(32768)
#define U_LIKELIHOOD_SCALE UINT64_C(2147483648)
#define U_EXP_NEG(x) expf(-(x))
#define U_FREXP frexpf
#define U_LDEXP ldexpf
#define U_MANT_DIG 24
#include "../src/pf_q_internal.h"
#undef U_TAG
#undef U_KERNEL
#undef U_SIGNAL
#undef U_SIGNAL_MAX
#undef U_WORK
#undef U_STATE
#undef U_BITS
#undef U_SCALE
#undef U_LIKELIHOOD_SCALE
#undef U_EXP_NEG
#undef U_FREXP
#undef U_LDEXP
#undef U_MANT_DIG

#define U_TAG q31
#define U_KERNEL f64
#define U_SIGNAL int32_t
#define U_SIGNAL_MAX INT32_MAX
#define U_WORK double
#define U_STATE filters_pf_q31_t
#define U_BITS 31
#define U_SCALE INT64_C(2147483648)
#define U_LIKELIHOOD_SCALE UINT64_C(9223372036854775808)
#define U_EXP_NEG(x) exp(-(x))
#define U_FREXP frexp
#define U_LDEXP ldexp
#define U_MANT_DIG 53
#include "../src/pf_q_internal.h"
#undef U_TAG
#undef U_KERNEL
#undef U_SIGNAL
#undef U_SIGNAL_MAX
#undef U_WORK
#undef U_STATE
#undef U_BITS
#undef U_SCALE
#undef U_LIKELIHOOD_SCALE
#undef U_EXP_NEG
#undef U_FREXP
#undef U_LDEXP
#undef U_MANT_DIG

namespace {

using namespace pf_test;

template<class W> struct GaussianOps;
template<> struct GaussianOps<float> {
    using Scale = pf_q_q15_scale_t;
    static constexpr int budget_shift = 12;
    static constexpr auto prepare = pf_q_q15_gaussian_prepare;
    static constexpr auto vector = pf_q_q15_vector_prepare;
    static constexpr auto perturb = pf_q_q15_perturb_try;
    static constexpr auto gaussian = pf_q_q15_gaussian_try;
    static constexpr auto mixed = pf_q_q15_gaussian_mixed;
    static constexpr auto retry = pf_q_q15_gaussian_retry;
    static constexpr auto raw_load = pf_q_q15_raw_load;
    static constexpr auto error_load = pf_q_q15_error_load;
    static constexpr auto raw_store = pf_q_q15_raw_store;
    static constexpr auto error_store = pf_q_q15_error_store;
    static constexpr auto divide = pf_q_q15_divide;
    static constexpr auto divide_solve = pf_q_q15_divide_solve;
    static constexpr auto prepare_division = pf_q_q15_gaussian_prepare_division;
    static constexpr auto convert = pf_q_q15_convert_scaled;
    static constexpr auto dot = pf_q_q15_dot_try;
    static constexpr auto dot_wide = pf_q_q15_dot_wide;
};
template<> struct GaussianOps<double> {
    using Scale = pf_q_q31_scale_t;
    static constexpr int budget_shift = 30;
    static constexpr auto prepare = pf_q_q31_gaussian_prepare;
    static constexpr auto vector = pf_q_q31_vector_prepare;
    static constexpr auto perturb = pf_q_q31_perturb_try;
    static constexpr auto gaussian = pf_q_q31_gaussian_try;
    static constexpr auto mixed = pf_q_q31_gaussian_mixed;
    static constexpr auto retry = pf_q_q31_gaussian_retry;
    static constexpr auto raw_load = pf_q_q31_raw_load;
    static constexpr auto error_load = pf_q_q31_error_load;
    static constexpr auto raw_store = pf_q_q31_raw_store;
    static constexpr auto error_store = pf_q_q31_error_store;
    static constexpr auto divide = pf_q_q31_divide;
    static constexpr auto divide_solve = pf_q_q31_divide_solve;
    static constexpr auto prepare_division = pf_q_q31_gaussian_prepare_division;
    static constexpr auto convert = pf_q_q31_convert_scaled;
    static constexpr auto dot = pf_q_q31_dot_try;
    static constexpr auto dot_wide = pf_q_q31_dot_wide;
};

template<class W>
uint64_t reciprocal_oracle(uint64_t denominator) {
    using Native128 = unsigned __int128;
    if constexpr (sizeof(W) == 4) {
        const uint32_t pivot = static_cast<uint32_t>(denominator);
        const unsigned shift = 32u - filters_q_bit_length(pivot);
        const uint32_t normalized = pivot << shift;
        if (normalized == (UINT32_C(1) << 31)) return 0;
        return (((UINT64_C(1) << 32) - normalized) << 32) / normalized;
    } else {
        const unsigned shift = 64u - filters_q_bit_length(denominator);
        const uint64_t normalized = denominator << shift;
        if (normalized == (UINT64_C(1) << 63)) return 0;
        const uint64_t high = UINT64_MAX - normalized + 1;
        return static_cast<uint64_t>((static_cast<Native128>(high) << 64) / normalized);
    }
}

template<class W>
long double gaussian_reference(const std::vector<W>& lower, const std::vector<W>& deviation, size_t k) {
    std::vector<long double> y(k);
    long double result = 0;
    for (size_t i = 0; i < k; ++i) {
        long double value = deviation[i];
        for (size_t j = 0; j < i; ++j) value -= static_cast<long double>(lower[i * k + j]) * y[j];
        y[i] = value / lower[i * k + i];
        result += y[i] * y[i];
    }
    return result / 2;
}

template<class W>
void expect_certified(W actual, long double reference, int budget_shift) {
    const long double allowance = std::ldexp(1.0L, -budget_shift) +
        4 * std::numeric_limits<W>::epsilon() * std::abs(reference);
    EXPECT_LE(std::abs(static_cast<long double>(actual) - reference), allowance)
        << "actual=" << actual << " reference=" << reference;
}

template<class W> class QGaussianInteger : public ::testing::Test {};
using WorkFormats = ::testing::Types<float, double>;
TYPED_TEST_SUITE(QGaussianInteger, WorkFormats);
template<class A> class QGaussianIntegration : public ::testing::Test {};
TYPED_TEST_SUITE(QGaussianIntegration, FixedFormats);

TYPED_TEST(QGaussianInteger, RawCachesUseIntegerBytes) {
    using W = TypeParam;
    using Ops = GaussianOps<W>;
    W slot = W(17.25);
    const int64_t minimum = sizeof(W) == 4 ? INT32_MIN : INT64_MIN;
    Ops::raw_store(&slot, minimum);
    EXPECT_EQ(Ops::raw_load(&slot), minimum);
    Ops::raw_store(&slot, -19);
    EXPECT_EQ(Ops::raw_load(&slot), -19);
    Ops::error_store(&slot, sizeof(W) == 4 ? UINT32_MAX : UINT64_MAX);
    EXPECT_EQ(Ops::error_load(&slot), sizeof(W) == 4 ? UINT32_MAX : UINT64_MAX);
}

TYPED_TEST(QGaussianInteger, WideDivisionMatchesNativeIntegerOracle) {
    using Ops = GaussianOps<TypeParam>;
    using Native128 = unsigned __int128;
    std::mt19937_64 random(0x12864a11u + sizeof(TypeParam));
    const auto check = [&](uint64_t high, uint64_t low, uint64_t denominator) {
        const Native128 numerator = (static_cast<Native128>(high) << 64) | low;
        const Native128 d = denominator;
        const Native128 quotient = numerator / d, remainder = numerator % d;
        for (int round = 0; round <= 1; ++round) {
            const Native128 expected = quotient + (round ? 2 * remainder >= d : remainder != 0);
            pf_q_u128_t input{low, high};
            uint64_t actual = 0;
            const bool fits = expected <= std::numeric_limits<uint64_t>::max();
            ASSERT_EQ(Ops::divide(input, denominator, round, &actual) != 0, fits)
                << high << ' ' << low << ' ' << denominator << ' ' << round;
            if (fits) EXPECT_EQ(actual, static_cast<uint64_t>(expected))
                << high << ' ' << low << ' ' << denominator << ' ' << round;
        }
    };
    for (const uint64_t denominator : {UINT64_C(1), UINT64_C(2), UINT64_C(3), UINT64_C(0xffffffff),
                                       UINT64_C(0x100000000), UINT64_C(0x8000000000000000), UINT64_MAX}) {
        check(0, 0, denominator);
        check(0, UINT64_MAX, denominator);
        check(denominator - 1, 0, denominator);
        check(denominator - 1, UINT64_MAX, denominator);
        check(denominator, 0, denominator);
    }
    for (unsigned bit = 1; bit < 64; ++bit) {
        const uint64_t power = UINT64_C(1) << bit;
        for (const uint64_t denominator : {power - 1, power, power + 1}) {
            check(0, UINT64_MAX, denominator);
            check(denominator - 1, UINT64_MAX, denominator);
            check(denominator, 0, denominator);
        }
    }
    for (int round = 0; round <= 1; ++round) {
        uint64_t actual = 0;
        EXPECT_FALSE(Ops::divide(pf_q_u128_t{0, 0}, 0, round, &actual));
    }
    for (size_t i = 0; i < 200000; ++i) {
        const uint64_t denominator = random() | 1;
        check(random() % denominator, random(), denominator);
    }
}

TYPED_TEST(QGaussianInteger, PowerOfTwoDividerMatchesNativeIntegerOracle) {
    using Ops = GaussianOps<TypeParam>;
    using Native128 = unsigned __int128;
    const auto check = [&](Native128 numerator, uint64_t denominator) {
        const Native128 d = denominator;
        for (int round = 0; round <= 1; ++round) {
            uint64_t actual = 0;
            if (denominator == 0) {
                EXPECT_FALSE(Ops::divide(pf_q_u128_t{static_cast<uint64_t>(numerator),
                                                   static_cast<uint64_t>(numerator >> 64)},
                                         denominator, round, &actual));
                continue;
            }
            const Native128 quotient = numerator / d, remainder = numerator % d;
            const Native128 expected = quotient + (round ? 2 * remainder >= d : remainder != 0);
            const bool fits = expected <= std::numeric_limits<uint64_t>::max();
            ASSERT_EQ(Ops::divide(pf_q_u128_t{static_cast<uint64_t>(numerator),
                                              static_cast<uint64_t>(numerator >> 64)},
                                  denominator, round, &actual) != 0, fits)
                << denominator << ' ' << round;
            if (fits) EXPECT_EQ(actual, static_cast<uint64_t>(expected)) << denominator << ' ' << round;
        }
    };
    for (unsigned bit = 0; bit < 64; ++bit) {
        const uint64_t denominator = UINT64_C(1) << bit;
        check(0, denominator);
        check(1, denominator);
        check(UINT64_MAX, denominator);
        check((Native128(1) << 64) - 1, denominator);
        check((Native128(1) << 64), denominator);
        check(static_cast<Native128>(UINT64_MAX) * denominator, denominator);
        check(static_cast<Native128>(UINT64_MAX) * denominator + denominator - 1, denominator);
        check(static_cast<Native128>(UINT64_MAX) * denominator + denominator / 2, denominator);
        check(~Native128(0), denominator);
    }
    for (const uint64_t denominator : {UINT64_C(0), UINT64_C(3), UINT64_MAX}) {
        check(0, denominator);
        check(UINT64_MAX, denominator);
        check(Native128(1) << 64, denominator);
        check(static_cast<Native128>(UINT64_MAX) * denominator + 1, denominator);
        check(~Native128(0), denominator);
    }
    std::mt19937_64 random(0x50325732u + sizeof(TypeParam));
    for (size_t i = 0; i < 20000; ++i) {
        const Native128 numerator = (static_cast<Native128>(random()) << 64) | random();
        check(numerator, random());
    }
}

TYPED_TEST(QGaussianInteger, ReciprocalDivisionMatchesNativeAndOrdinaryDivider) {
    using Ops = GaussianOps<TypeParam>;
    using Native128 = unsigned __int128;
    const uint64_t maximum_quotient = sizeof(TypeParam) == 4 ? UINT32_MAX : UINT64_MAX;
    const uint64_t maximum_pivot = sizeof(TypeParam) == 4 ? INT32_MAX : INT64_MAX;
    const auto check = [&](Native128 numerator, uint64_t denominator) {
        const Native128 d = denominator;
        const Native128 quotient = numerator / d, remainder = numerator % d;
        const uint64_t reciprocal = reciprocal_oracle<TypeParam>(denominator);
        pf_q_u128_t input{static_cast<uint64_t>(numerator), static_cast<uint64_t>(numerator >> 64)};
        for (int round = 0; round <= 1; ++round) {
            const Native128 expected = quotient + (round ? 2 * remainder >= d : remainder != 0);
            const bool fits = expected <= std::numeric_limits<uint64_t>::max();
            uint64_t cached = 0, ordinary = 0;
            const bool cached_ok = reciprocal != 0
                ? Ops::divide_solve(input, denominator, round, reciprocal, &cached) != 0
                : Ops::divide(input, denominator, round, &cached) != 0;
            ASSERT_EQ(cached_ok, fits)
                << input.hi << ' ' << input.lo << ' ' << denominator << ' ' << round;
            ASSERT_EQ(Ops::divide(input, denominator, round, &ordinary) != 0, fits)
                << input.hi << ' ' << input.lo << ' ' << denominator << ' ' << round;
            if (fits) {
                EXPECT_EQ(cached, static_cast<uint64_t>(expected)) << input.hi << ' ' << input.lo << ' ' << denominator << ' ' << round;
                EXPECT_EQ(ordinary, cached) << input.hi << ' ' << input.lo << ' ' << denominator << ' ' << round;
            }
        }
    };
    for (unsigned bit = 0; bit < (sizeof(TypeParam) == 4 ? 31u : 63u); ++bit) {
        const uint64_t power = UINT64_C(1) << bit;
        for (const uint64_t denominator : {power - 1, power, power + 1}) {
            if (denominator == 0 || denominator > maximum_pivot) continue;
            check(0, denominator);
            check(1, denominator);
            check(static_cast<Native128>(maximum_quotient) * denominator + denominator - 1, denominator);
            check(static_cast<Native128>(maximum_quotient) * denominator + denominator / 2, denominator);
        }
    }
    check((Native128(1) << 64), 3);
    check(Native128(3) << 32, 3);
    check(static_cast<Native128>(maximum_quotient) * 3 + 2, 3);
    std::mt19937_64 random(0x5245434950u + sizeof(TypeParam));
    for (size_t i = 0; i < 20000; ++i) {
        const uint64_t denominator = 1 + random() % maximum_pivot;
        const uint64_t quotient = random() & maximum_quotient;
        const uint64_t remainder = random() % denominator;
        check(static_cast<Native128>(quotient) * denominator + remainder, denominator);
    }
    /* The reciprocal==0 arm above uses the same direct generic-divider
     * dispatch as Gaussian solve, including unprepared non-power factors. */
}

TYPED_TEST(QGaussianInteger, DivisionCacheSlotsNoopsAndPerturbationStayIntact) {
    using Ops = GaussianOps<TypeParam>;
    using Scale = typename Ops::Scale;
    {
        std::array<TypeParam, 13> guarded;
        for (size_t i = 0; i < guarded.size(); ++i) Ops::raw_store(guarded.data() + i, i + 1);
        const auto before = guarded;
        Ops::prepare_division(guarded.data() + 2, 1);
        EXPECT_EQ(std::memcmp(guarded.data(), before.data(), sizeof(guarded)), 0);
    }
    std::array<TypeParam, 13> guarded{};
    TypeParam *raw = guarded.data() + 2;
    Ops::raw_store(raw, 3);
    Ops::raw_store(raw + 4, 4);
    Ops::raw_store(raw + 8, 5);
    const auto before = guarded;
    Ops::prepare_division(raw, 3);
    EXPECT_EQ(Ops::error_load(raw + 1), reciprocal_oracle<TypeParam>(3));
    EXPECT_EQ(Ops::error_load(raw + 5), 0);
    EXPECT_EQ(Ops::error_load(raw + 2), reciprocal_oracle<TypeParam>(5));
    for (size_t i = 0; i < 3; ++i)
        for (size_t j = 0; j <= i; ++j)
            EXPECT_EQ(Ops::raw_load(raw + i * 3 + j), Ops::raw_load(before.data() + 2 + i * 3 + j));
    EXPECT_EQ(std::memcmp(guarded.data(), before.data(), 2 * sizeof(TypeParam)), 0);
    EXPECT_EQ(std::memcmp(guarded.data() + 11, before.data() + 11, 2 * sizeof(TypeParam)), 0);

    const std::array<TypeParam, 9> lower{3, 0, 0, 0, 4, 0, 0, 0, 5};
    const std::array<TypeParam, 3> draw{TypeParam(0.25), TypeParam(-0.5), TypeParam(1)};
    std::array<TypeParam, 9> raw_factor{}, original_factor{};
    std::array<TypeParam, 3> raw_draw{}, first{}, second{};
    Scale sa, sb;
    ASSERT_TRUE(Ops::prepare(lower.data(), 3, raw_factor.data(), &sa));
    ASSERT_TRUE(Ops::vector(draw.data(), 3, raw_draw.data(), &sb));
    original_factor = raw_factor;
    for (size_t slot : {1u, 2u, 5u}) EXPECT_EQ(Ops::error_load(raw_factor.data() + slot), 0);
    ASSERT_TRUE(Ops::perturb(raw_factor.data(), sa, raw_draw.data(), sb, 3, first.data()));
    ASSERT_TRUE(Ops::perturb(raw_factor.data(), sa, raw_draw.data(), sb, 3, second.data()));
    EXPECT_EQ(first, second);
    EXPECT_EQ(std::memcmp(raw_factor.data(), original_factor.data(), sizeof(raw_factor)), 0);
    for (size_t slot : {1u, 2u, 5u}) EXPECT_EQ(Ops::error_load(raw_factor.data() + slot), 0);
}

TYPED_TEST(QGaussianInteger, TwoRowCacheReusesOnlyAnEqualPositivePivot) {
    using W = TypeParam;
    using Ops = GaussianOps<W>;
    using Scale = typename Ops::Scale;
    const std::array<std::array<W, 4>, 4> factors{{
        {{3, 0, 0, 3}}, {{3, 0, W(0.25), 3}},
        {{3, 0, W(0.25), 5}}, {{4, 0, W(0.25), 4}}
    }};
    for (const auto& lower : factors) {
        std::array<W, 8> guarded;
        guarded.fill(W(19));
        W *cached = guarded.data() + 2;
        std::array<W, 4> ordinary{};
        std::array<W, 2> draw{W(0.375), W(-0.625)}, b{};
        Scale sa, sb;
        ASSERT_TRUE(Ops::prepare(lower.data(), 2, cached, &sa));
        std::memcpy(ordinary.data(), cached, sizeof(ordinary));
        const auto before = guarded;
        Ops::prepare_division(cached, 2);
        EXPECT_EQ(Ops::error_load(cached + 1),
                  reciprocal_oracle<W>(static_cast<uint64_t>(Ops::raw_load(cached))));
        for (size_t slot : {0u, 2u, 3u})
            EXPECT_EQ(Ops::raw_load(cached + slot), Ops::raw_load(ordinary.data() + slot));
        EXPECT_EQ(std::memcmp(guarded.data(), before.data(), 2 * sizeof(W)), 0);
        EXPECT_EQ(std::memcmp(guarded.data() + 6, before.data() + 6, 2 * sizeof(W)), 0);
        ASSERT_TRUE(Ops::vector(draw.data(), 2, b.data(), &sb));
        for (bool force_inexact : {false, true}) {
            Scale tested = sb;
            if (force_inexact) tested.exact = 0;
            std::array<W, 2> z1{}, z2{}, e1{}, e2{};
            W first = -1, second = -1;
            const int cached_ok = Ops::gaussian(cached, sa, b.data(), tested, 2, z1.data(), e1.data(), &first);
            const int ordinary_ok = Ops::gaussian(ordinary.data(), sa, b.data(), tested, 2, z2.data(), e2.data(), &second);
            ASSERT_EQ(cached_ok, ordinary_ok);
            if (!force_inexact) ASSERT_TRUE(cached_ok);
            EXPECT_EQ(first, second);
            for (size_t i = 0; i < 2; ++i) {
                EXPECT_EQ(Ops::raw_load(z1.data() + i), Ops::raw_load(z2.data() + i));
                EXPECT_EQ(Ops::error_load(e1.data() + i), Ops::error_load(e2.data() + i));
            }
        }
    }
}

TYPED_TEST(QGaussianInteger, CachedAndOrdinaryGaussianSolveAgreeThroughRestarts) {
    using Ops = GaussianOps<TypeParam>;
    using Scale = typename Ops::Scale;
    constexpr size_t k = 3;
    std::mt19937_64 random(0x4741553433u + sizeof(TypeParam));
    std::uniform_real_distribution<long double> uniform(0.05, 0.75);
    for (size_t trial = 0; trial < 500; ++trial) {
        std::vector<TypeParam> lower(k * k, 0), deviation(k), cached(k * k), ordinary(k * k);
        std::vector<TypeParam> b(k), z1(k), z2(k), e1(k), e2(k);
        Scale sa, sb, ordinary_sa;
        for (size_t i = 0; i < k; ++i) {
            const long double diagonal = uniform(random);
            lower[i * k + i] = static_cast<TypeParam>(diagonal);
            for (size_t j = 0; j < i; ++j)
                lower[i * k + j] = static_cast<TypeParam>(diagonal * uniform(random) * ((i + j) & 1 ? 0.5L : -0.5L));
            deviation[i] = static_cast<TypeParam>(uniform(random) * (i & 1 ? -2 : 2));
        }
        ASSERT_TRUE(Ops::prepare(lower.data(), k, cached.data(), &sa)) << trial;
        ASSERT_TRUE(Ops::prepare(lower.data(), k, ordinary.data(), &ordinary_sa)) << trial;
        ASSERT_TRUE(Ops::vector(deviation.data(), k, b.data(), &sb)) << trial;
        Ops::prepare_division(cached.data(), k);
        const auto original_cached = cached;
        TypeParam cached_result = -1, ordinary_result = -1;
        const int cached_ok = Ops::gaussian(cached.data(), sa, b.data(), sb, k, z1.data(), e1.data(), &cached_result);
        const int ordinary_ok = Ops::gaussian(ordinary.data(), ordinary_sa, b.data(), sb, k, z2.data(), e2.data(), &ordinary_result);
        ASSERT_EQ(cached_ok, ordinary_ok) << trial;
        EXPECT_EQ(cached_result, ordinary_result) << trial;
        for (size_t i = 0; i < k; ++i) {
            EXPECT_EQ(Ops::raw_load(z1.data() + i), Ops::raw_load(z2.data() + i)) << trial << ' ' << i;
            EXPECT_EQ(Ops::error_load(e1.data() + i), Ops::error_load(e2.data() + i)) << trial << ' ' << i;
        }
        for (size_t i = 0; i < k; ++i)
            for (size_t j = 0; j <= i; ++j)
                EXPECT_EQ(Ops::raw_load(cached.data() + i * k + j),
                          Ops::raw_load(original_cached.data() + i * k + j)) << trial << ' ' << i << ' ' << j;
    }

    std::array<TypeParam, 9> cached{}, ordinary{};
    std::array<TypeParam, 3> b{}, z1{}, z2{}, e1{}, e2{};
    Scale sa, sb;
    sa.exponent = sb.exponent = 0;
    sa.exact = sb.exact = 1;
    sa.diagonal = 1;
    sb.diagonal = 0;
    const int64_t maximum = sizeof(TypeParam) == 4 ? INT32_MAX : INT64_MAX;
    if constexpr (sizeof(TypeParam) == 4) {
        sa.maximum = 3;
        sb.maximum = maximum;
    }
    for (size_t i = 0; i < 3; ++i) {
        Ops::raw_store(cached.data() + i * 3 + i, 3);
        Ops::raw_store(ordinary.data() + i * 3 + i, 3);
        Ops::raw_store(b.data() + i, maximum);
    }
    Ops::prepare_division(cached.data(), 3);
    TypeParam cached_result = -1, ordinary_result = -1;
    const int cached_ok = Ops::gaussian(cached.data(), sa, b.data(), sb, 3, z1.data(), e1.data(), &cached_result);
    const int ordinary_ok = Ops::gaussian(ordinary.data(), sa, b.data(), sb, 3, z2.data(), e2.data(), &ordinary_result);
    ASSERT_EQ(cached_ok, ordinary_ok);
    EXPECT_EQ(cached_result, ordinary_result);
    for (size_t i = 0; i < 3; ++i) {
        EXPECT_EQ(Ops::raw_load(z1.data() + i), Ops::raw_load(z2.data() + i)) << i;
        EXPECT_EQ(Ops::error_load(e1.data() + i), Ops::error_load(e2.data() + i)) << i;
    }
}

template<class W>
void expect_exact_diagonal_variance(const std::vector<W>& z, const std::vector<W>& error) {
    using Native128 = unsigned __int128;
    const Native128 e = Native128(1) << 31;
    Native128 previous = 0, magnitudes = 0;
    ASSERT_EQ(z.size(), error.size());
    for (size_t i = 0; i < z.size(); ++i) {
        uint64_t raw_error;
        int64_t raw_z;
        if constexpr (sizeof(W) == 4) {
            raw_error = pf_q_q15_error_load(error.data() + i);
            raw_z = pf_q_q15_raw_load(z.data() + i);
        } else {
            raw_error = pf_q_q31_error_load(error.data() + i);
            raw_z = pf_q_q31_raw_load(z.data() + i);
        }
        const Native128 magnitude = pf_q_magnitude(raw_z);
        ASSERT_EQ(raw_error, UINT64_C(1) << 31) << i;
        previous += 2 * magnitude * e + ((e * e) >> 32) + ((e * e & 0xffffffff) != 0);
        magnitudes += magnitude;
    }
    const Native128 direct = (magnitudes << 32) + z.size() * (Native128(1) << 30);
    EXPECT_EQ(previous, direct);
}

TYPED_TEST(QGaussianInteger, ExactDiagonalIdentityIncludesExtremesAndRestarts) {
    using Ops = GaussianOps<TypeParam>;
    using Scale = typename Ops::Scale;
    using Native128 = unsigned __int128;
    const int64_t minimum = sizeof(TypeParam) == 4 ? INT32_MIN : INT64_MIN;
    {
        std::array<TypeParam, 1> a{}, b{}, z{}, error{};
        Scale sa{0, 1, 1}, sb{0, 1, 0};
        sb.exponent = sizeof(TypeParam) == 4 ? -30 : -62;
        if constexpr (sizeof(TypeParam) == 4) sa.maximum = sb.maximum = 1;
        Ops::raw_store(a.data(), 1);
        Ops::raw_store(b.data(), minimum);
        TypeParam result = -1;
        ASSERT_TRUE(Ops::gaussian(a.data(), sa, b.data(), sb, 1, z.data(), error.data(), &result));
        EXPECT_EQ(result, TypeParam(2));
        EXPECT_EQ(Ops::raw_load(z.data()), minimum);
        expect_exact_diagonal_variance<TypeParam>(std::vector<TypeParam>(z.begin(), z.end()),
                                                  std::vector<TypeParam>(error.begin(), error.end()));
    }
    for (const size_t k : {size_t(1), size_t(2), size_t(3), size_t(8), size_t(17)}) {
        std::vector<TypeParam> a(k * k), b(k), z(k), error(k);
        Scale sa{0, 1, 1}, sb{0, 1, 0};
        if constexpr (sizeof(TypeParam) == 4) sa.maximum = sb.maximum = 1;
        Native128 norm = 0;
        for (size_t i = 0; i < k; ++i) {
            const int64_t value = static_cast<int64_t>(i % 7) - 3;
            Ops::raw_store(a.data() + i * k + i, 1);
            Ops::raw_store(b.data() + i, value);
            norm += static_cast<Native128>(value) * value;
        }
        TypeParam result = -1;
        ASSERT_TRUE(Ops::gaussian(a.data(), sa, b.data(), sb, k, z.data(), error.data(), &result)) << k;
        const TypeParam expected = static_cast<TypeParam>(norm) / 2;
        EXPECT_EQ(result, expected) << k;
        expect_exact_diagonal_variance<TypeParam>(z, error);
    }

    /* Precision restarts retain E=2^31 and the same variance identity. */
    std::array<TypeParam, 9> a{};
    std::array<TypeParam, 3> b{}, z{}, error{};
    Scale sa{0, 1, 1}, sb{0, 1, 0};
    if constexpr (sizeof(TypeParam) == 4) {
        sa.maximum = 3;
        sb.maximum = INT32_MAX;
    }
    for (size_t i = 0; i < 3; ++i) {
        Ops::raw_store(a.data() + i * 3 + i, 3);
        Ops::raw_store(b.data() + i, minimum == INT32_MIN ? INT32_MAX : INT64_MAX);
    }
    TypeParam result = -1;
    (void)Ops::gaussian(a.data(), sa, b.data(), sb, 3, z.data(), error.data(), &result);
    expect_exact_diagonal_variance<TypeParam>(std::vector<TypeParam>(z.begin(), z.end()),
                                              std::vector<TypeParam>(error.begin(), error.end()));

    /* An inexact vector must retain the original propagated-error path. */
    sb.exact = 0;
    result = -1;
    (void)Ops::gaussian(a.data(), sa, b.data(), sb, 3, z.data(), error.data(), &result);
    bool above_minimum = false;
    for (const TypeParam& slot : error) above_minimum |= Ops::error_load(&slot) > (UINT64_C(1) << 31);
    EXPECT_TRUE(above_minimum);
}

template<class W>
bool portable_convert(W value, int exponent, int *exact, int64_t *raw) {
    constexpr int F = sizeof(W) == 4 ? 30 : 62;
    W scaled = std::ldexp(value, F - exponent);
    if (!std::isfinite(scaled)) return false;
    const bool negative = scaled < 0;
    W magnitude = negative ? -scaled : scaled, rounded;
    const W fraction = std::modf(magnitude, &rounded);
    if (fraction >= W(0.5)) rounded += 1;
    const W limit = std::ldexp(W(1), F + 1);
    if (!std::isfinite(rounded) || rounded > limit || (rounded == limit && !negative)) return false;
    if (rounded == limit) *raw = sizeof(W) == 4 ? INT32_MIN : INT64_MIN;
    else {
        const uint64_t integer = static_cast<uint64_t>(rounded);
        *raw = negative ? -static_cast<int64_t>(integer) : static_cast<int64_t>(integer);
    }
    *exact = std::ldexp(static_cast<W>(*raw), exponent - F) == value;
    return true;
}

TYPED_TEST(QGaussianInteger, IeeeConversionMatchesPortableBoundary) {
    using W = TypeParam;
    using Ops = GaussianOps<W>;
    std::mt19937_64 random(0x1eee40u + sizeof(W));
    for (size_t i = 0; i < 100000; ++i) {
        W value;
        if constexpr (sizeof(W) == 4) {
            uint32_t bits = static_cast<uint32_t>(random());
            if ((bits & UINT32_C(0x7f800000)) == UINT32_C(0x7f800000)) continue;
            std::memcpy(&value, &bits, sizeof(value));
        } else {
            uint64_t bits = random();
            if ((bits & UINT64_C(0x7ff0000000000000)) == UINT64_C(0x7ff0000000000000)) continue;
            std::memcpy(&value, &bits, sizeof(value));
        }
        const int exponent = static_cast<int>(random() % 41) - 20;
        int expected_exact = -1, actual_exact = -1;
        int64_t expected_raw = 0, actual_raw = 0;
        const bool expected = portable_convert(value, exponent, &expected_exact, &expected_raw);
        const bool actual = Ops::convert(value, exponent, &actual_exact, &actual_raw) != 0;
        ASSERT_EQ(actual, expected) << value << ' ' << exponent;
        if (expected) {
            EXPECT_EQ(actual_raw, expected_raw) << value << ' ' << exponent;
            EXPECT_EQ(actual_exact, expected_exact) << value << ' ' << exponent;
        }
    }
}

TYPED_TEST(QGaussianInteger, OrdinaryAndDenseCorrelatedWorkUsesIntegerMacDivideAndNorm) {
    using W = TypeParam;
    using Ops = GaussianOps<W>;
    using Scale = typename Ops::Scale;
    constexpr size_t k = 4, trials = 16 * 128; /* N32H474-like n=4, N=128 batches. */
    std::mt19937_64 random(0x474155535349414eULL + sizeof(W));
    std::normal_distribution<long double> normal(0, 1);
    std::uniform_real_distribution<long double> uniform(0.05, 0.75);
    std::vector<W> lower(k * k), draw(k), raw_a(k * k), raw_b(k), z(k), error(k), perturb(k);
    size_t perturb_success = 0, gaussian_success = 0;
    for (size_t trial = 0; trial < trials; ++trial) {
        Scale sa, sb;
        if (trial % 128 == 0) {
            std::fill(lower.begin(), lower.end(), W(0));
            for (size_t i = 0; i < k; ++i) {
                const long double diagonal = uniform(random);
                lower[i * k + i] = W(diagonal);
                for (size_t j = 0; j < i; ++j)
                    lower[i * k + j] = W(diagonal * uniform(random) * ((i + j) & 1 ? 0.5L : -0.5L));
            }
        }
        for (W& value : draw) value = W(normal(random));
        ASSERT_TRUE(Ops::prepare(lower.data(), k, raw_a.data(), &sa)) << trial;
        ASSERT_TRUE(Ops::vector(draw.data(), k, raw_b.data(), &sb)) << trial;
        if (Ops::perturb(raw_a.data(), sa, raw_b.data(), sb, k, perturb.data())) {
            ++perturb_success;
            for (size_t i = 0; i < k; ++i) {
                long double reference = 0;
                for (size_t j = 0; j <= i; ++j)
                    reference += static_cast<long double>(lower[i * k + j]) * draw[j];
                expect_certified(perturb[i], reference, Ops::budget_shift);
            }
        }
        W result = W(-9);
        if (Ops::gaussian(raw_a.data(), sa, raw_b.data(), sb, k, z.data(), error.data(), &result)) {
            ++gaussian_success;
            expect_certified(result, gaussian_reference(lower, draw, k), Ops::budget_shift);
        }
    }
    EXPECT_GT(perturb_success, trials * 9 / 10);
    EXPECT_GT(gaussian_success, trials * 9 / 10);
    ::testing::Test::RecordProperty("perturb_integer_successes", perturb_success);
    ::testing::Test::RecordProperty("gaussian_integer_successes", gaussian_success);
    ::testing::Test::RecordProperty("integer_trials", trials);
}

TEST(QGaussianOpt2, Q15DotFastPathMatchesWideOnEveryBound) {
    using Ops = GaussianOps<float>;
    using Scale = Ops::Scale;
    std::mt19937_64 random(0x64015u);
    for (size_t trial = 0; trial < 10000; ++trial) {
        const size_t count = 1 + random() % 16;
        std::vector<float> a(count), b(count);
        Scale sa{0, 1, 0, 0}, sb{0, 1, 0, 0};
        for (size_t i = 0; i < count; ++i) {
            const int32_t av = static_cast<int32_t>(random());
            const int32_t bv = static_cast<int32_t>(random());
            Ops::raw_store(a.data() + i, av);
            Ops::raw_store(b.data() + i, bv);
            sa.maximum = std::max(sa.maximum, static_cast<uint32_t>(pf_q_magnitude(av)));
            sb.maximum = std::max(sb.maximum, static_cast<uint32_t>(pf_q_magnitude(bv)));
        }
        sa.exact = static_cast<int>(random() & 1);
        sb.exact = static_cast<int>(random() & 1);
        sa.exponent = static_cast<int>(random() % 7) - 3;
        sb.exponent = static_cast<int>(random() % 7) - 3;
        float fast = -1, wide = -2;
        const int fast_ok = Ops::dot(a.data(), sa, b.data(), sb, count, &fast);
        const int wide_ok = Ops::dot_wide(a.data(), sa, b.data(), sb, count, &wide);
        ASSERT_EQ(fast_ok, wide_ok) << trial;
        if (fast_ok) EXPECT_EQ(fast, wide) << trial;
        sa.maximum = sb.maximum = UINT64_C(2147483648);
        wide = -3;
        const int forced_ok = Ops::dot_wide(a.data(), sa, b.data(), sb, count, &wide);
        ASSERT_EQ(fast_ok, forced_ok) << trial;
        if (fast_ok) EXPECT_EQ(fast, wide) << trial;
    }
}

TEST(QGaussianOpt2, Q15DotFastPathMatchesWideWithDirectedRounding) {
    using Ops = GaussianOps<float>;
    using Scale = Ops::Scale;
    std::array<float, 1> a{}, b{};
    Ops::raw_store(b.data(), 1);
    Scale sa{0, 1, 0, 16777217}, sb{0, 1, 0, 1};
    std::fenv_t saved;
    ASSERT_EQ(std::fegetenv(&saved), 0);
    struct Restore { std::fenv_t *env; ~Restore() { std::fesetround(FE_TONEAREST); std::fesetenv(env); } } restore{&saved};
    for (const int32_t raw : {-16777217, 16777217}) {
        Ops::raw_store(a.data(), raw);
        for (const int mode : {FE_TONEAREST, FE_UPWARD, FE_DOWNWARD, FE_TOWARDZERO}) {
            ASSERT_EQ(std::fesetround(mode), 0);
            float fast = -1, wide = -2;
            ASSERT_TRUE(Ops::dot(a.data(), sa, b.data(), sb, 1, &fast));
            ASSERT_TRUE(Ops::dot_wide(a.data(), sa, b.data(), sb, 1, &wide));
            EXPECT_EQ(fast, wide) << raw << ' ' << mode;
            /* Both paths convert the positive magnitude before applying its
             * sign. 16777217 rounds to 16777218 only in FE_UPWARD; scaling
             * by 2^-60 is exact. Check bits so two wrongly folded paths
             * cannot satisfy the test merely by agreeing with each other. */
            const uint32_t expected = (raw < 0 ? UINT32_C(0xad800000) : UINT32_C(0x2d800000)) +
                                      (mode == FE_UPWARD ? 1u : 0u);
            uint32_t fast_bits, wide_bits;
            std::memcpy(&fast_bits, &fast, sizeof(fast_bits));
            std::memcpy(&wide_bits, &wide, sizeof(wide_bits));
            EXPECT_EQ(fast_bits, expected) << raw << ' ' << mode;
            EXPECT_EQ(wide_bits, expected) << raw << ' ' << mode;
        }
    }
}

TEST(QGaussianOpt2, ConservativeDotBoundRetainsTheAcceptedWideIntegerPath) {
    using Ops = GaussianOps<float>;
    using Scale = Ops::Scale;
    const std::array<float, 2> a{256, 0x1p-24f};
    const std::array<float, 2> b{0x1p-24f, 256};
    std::array<float, 2> raw_a{}, raw_b{};
    Scale sa, sb;
    ASSERT_TRUE(Ops::vector(a.data(), 2, raw_a.data(), &sa));
    ASSERT_TRUE(Ops::vector(b.data(), 2, raw_b.data(), &sb));
    EXPECT_FALSE(sa.exact);
    EXPECT_FALSE(sb.exact);
    /* count*maximum_error exceeds the certificate threshold, but the sum
     * of actual per-element errors is within it. Both dots must stay on
     * the original integer path, including the squared sampler norm. */
    for (bool square : {false, true}) {
        float expected = -1, actual = -2;
        const float *rhs = square ? raw_a.data() : raw_b.data();
        const Scale scale = square ? sa : sb;
        ASSERT_TRUE(Ops::dot_wide(raw_a.data(), sa, rhs, scale, 2, &expected));
        ASSERT_TRUE(Ops::dot(raw_a.data(), sa, rhs, scale, 2, &actual));
        EXPECT_EQ(actual, expected);
        EXPECT_EQ(actual, square ? 65536.0f : 0.0f);
    }
}

TEST(QGaussianOpt2, Q15GaussianFastPathsMatchForcedWideArithmetic) {
    using Ops = GaussianOps<float>;
    using Scale = Ops::Scale;
    constexpr size_t k = 8;
    std::mt19937_64 random(0x6415u);
    std::normal_distribution<long double> normal(0, 1);
    std::uniform_real_distribution<long double> uniform(0.05, 0.75);
    for (size_t trial = 0; trial < 2000; ++trial) {
        std::vector<float> lower(k * k, 0), deviation(k), a(k * k), b(k), z1(k), z2(k), e1(k), e2(k);
        Scale sa, sb;
        for (size_t i = 0; i < k; ++i) {
            const long double diagonal = uniform(random);
            lower[i * k + i] = static_cast<float>(diagonal);
            for (size_t j = 0; j < i; ++j)
                lower[i * k + j] = static_cast<float>(diagonal * uniform(random) * ((i + j) & 1 ? 0.5L : -0.5L));
            deviation[i] = static_cast<float>(normal(random));
        }
        ASSERT_TRUE(Ops::prepare(lower.data(), k, a.data(), &sa)) << trial;
        ASSERT_TRUE(Ops::vector(deviation.data(), k, b.data(), &sb)) << trial;
        Scale forced_a = sa, forced_b = sb;
        forced_a.maximum = forced_b.maximum = UINT64_C(2147483648);
        float fast = -1, wide = -2;
        const int fast_ok = Ops::gaussian(a.data(), sa, b.data(), sb, k, z1.data(), e1.data(), &fast);
        const int wide_ok = Ops::gaussian(a.data(), forced_a, b.data(), forced_b, k, z2.data(), e2.data(), &wide);
        ASSERT_EQ(fast_ok, wide_ok) << trial;
        if (fast_ok) EXPECT_EQ(fast, wide) << trial;
        for (size_t i = 0; i < k; ++i) {
            EXPECT_EQ(Ops::raw_load(z1.data() + i), Ops::raw_load(z2.data() + i)) << trial << ' ' << i;
            EXPECT_EQ(Ops::error_load(e1.data() + i), Ops::error_load(e2.data() + i)) << trial << ' ' << i;
        }
    }

    /* With Z=2^30 and E=2^31, k=3 uses the 64-bit norm accumulation while
     * k=4 exceeds its conservative variance bound and uses the wide loop. */
    for (size_t count = 3; count <= 4; ++count) {
        std::vector<float> a(count * count), b(count), z(count), e(count);
        Scale sa{0, 1, 1, UINT64_C(1) << 30};
        Scale sb{0, 1, 0, UINT64_C(1) << 30};
        for (size_t i = 0; i < count; ++i) {
            Ops::raw_store(a.data() + i * count + i, INT64_C(1) << 30);
            Ops::raw_store(b.data() + i, INT64_C(1) << 30);
        }
        float result = -1;
        ASSERT_TRUE(Ops::gaussian(a.data(), sa, b.data(), sb, count, z.data(), e.data(), &result));
        EXPECT_EQ(result, static_cast<float>(count) / 2);
        for (size_t i = 0; i < count; ++i) {
            EXPECT_EQ(Ops::raw_load(z.data() + i), INT64_C(1) << 30);
            EXPECT_EQ(Ops::error_load(e.data() + i), UINT64_C(1) << 31);
        }
    }
}

TYPED_TEST(QGaussianInteger, SubnormalScaleExponentAndSolveStayExact) {
    using W = TypeParam;
    using Ops = GaussianOps<W>;
    using Scale = typename Ops::Scale;
    const W tiny = std::numeric_limits<W>::denorm_min();
    std::array<W, 1> lower{tiny}, deviation{tiny}, a{}, b{}, z{}, error{};
    Scale sa, sb;
    ASSERT_TRUE(Ops::prepare(lower.data(), 1, a.data(), &sa));
    ASSERT_TRUE(Ops::vector(deviation.data(), 1, b.data(), &sb));
    EXPECT_EQ(sa.exponent, sizeof(W) == 4 ? -149 : -1074);
    EXPECT_EQ(sb.exponent, sa.exponent);
    EXPECT_TRUE(sa.exact);
    EXPECT_TRUE(sb.exact);
    W result = W(-1);
    ASSERT_TRUE(Ops::gaussian(a.data(), sa, b.data(), sb, 1, z.data(), error.data(), &result));
    EXPECT_EQ(result, W(0.5));
}

TYPED_TEST(QGaussianInteger, CorrectedDivisionBoundIncludesInputQuantization) {
    using W = TypeParam;
    using Ops = GaussianOps<W>;
    using Scale = typename Ops::Scale;
    std::array<W, 1> raw_a{}, raw_b{}, z{}, error{};
    Scale sa, sb;
    sa.exponent = sb.exponent = 0;
    sa.exact = sb.exact = 1;
    sa.diagonal = 1;
    sb.diagonal = 0;
    if constexpr (std::is_same_v<W, float>) {
        sa.maximum = 3;
        sb.maximum = 1;
    }
    Ops::raw_store(raw_a.data(), 3);
    Ops::raw_store(raw_b.data(), 1);
    W result = W(-7);
    /* Raw A=3,B=1 leaves a 1/3 raw-Z division error: above the old
     * 1/6 bound but below the corrected half-unit bound retained in E. */
    ASSERT_TRUE(Ops::gaussian(raw_a.data(), sa, raw_b.data(), sb, 1, z.data(), error.data(), &result));
    const unsigned fractional = sizeof(W) == 4 ? 30 : 62;
    EXPECT_EQ(Ops::raw_load(z.data()), (int64_t)((UINT64_C(1) << fractional) / 3));
    EXPECT_EQ(Ops::error_load(error.data()), UINT64_C(1) << 31);
    expect_certified(result, 1.0L / 18, Ops::budget_shift);

    std::array<W, 4> identity{W(1), W(0), W(0), W(1)}, wide_a{};
    const W tiny = std::ldexp(W(1), sizeof(W) == 4 ? -40 : -80);
    std::array<W, 2> input{W(1), tiny}, b{}, solved{}, bounds{};
    ASSERT_TRUE(Ops::prepare(identity.data(), 2, wide_a.data(), &sa));
    ASSERT_TRUE(Ops::vector(input.data(), 2, b.data(), &sb));
    EXPECT_FALSE(sb.exact);
    result = W(-7);
    if (Ops::gaussian(wide_a.data(), sa, b.data(), sb, 2, solved.data(), bounds.data(), &result))
        expect_certified(result, gaussian_reference(
            std::vector<W>(identity.begin(), identity.end()),
            std::vector<W>(input.begin(), input.end()), 2), Ops::budget_shift);
    EXPECT_GT(Ops::error_load(bounds.data()), UINT64_C(1) << 31);

    std::array<W, 4> off_diagonal{W(1), W(0), tiny, W(1)}, off_a{};
    std::array<W, 2> rhs{W(1), W(1)}, off_b{}, off_z{}, off_e{};
    Scale oa, ob;
    ASSERT_TRUE(Ops::prepare(off_diagonal.data(), 2, off_a.data(), &oa));
    ASSERT_TRUE(Ops::vector(rhs.data(), 2, off_b.data(), &ob));
    EXPECT_FALSE(oa.exact);
    EXPECT_TRUE(ob.exact);
    result = W(-7);
    if (Ops::gaussian(off_a.data(), oa, off_b.data(), ob, 2, off_z.data(), off_e.data(), &result))
        expect_certified(result, gaussian_reference(
            std::vector<W>(off_diagonal.begin(), off_diagonal.end()),
            std::vector<W>(rhs.begin(), rhs.end()), 2), Ops::budget_shift);
    EXPECT_GT(Ops::error_load(off_e.data() + 1), UINT64_C(1) << 31);
}

TYPED_TEST(QGaussianInteger, InputBoundTracksSolveScaleAndExactRawError) {
    using W = TypeParam;
    using Ops = GaussianOps<W>;
    using Scale = typename Ops::Scale;
    const int pivot_exponent = sizeof(W) == 4 ? -28 : -60;
    const int input_exponent = sizeof(W) == 4 ? -32 : -64;
    std::array<W, 4> lower{W(1), W(0), W(0), std::ldexp(W(1), pivot_exponent)}, raw_a{};
    std::array<W, 2> deviation{W(1), std::ldexp(W(1), input_exponent)}, raw_b{}, z{}, error{}, solve{};
    Scale sa, sb;
    ASSERT_TRUE(Ops::prepare(lower.data(), 2, raw_a.data(), &sa));
    ASSERT_TRUE(Ops::vector(deviation.data(), 2, raw_b.data(), &sb));
    EXPECT_FALSE(sb.exact);
    W result = W(-7);
    /* The second exact scaled input is 1/4 and its pivot is 4. At the final
     * reduced scale G, exact raw Z1 is 2^(G-4), while computed Z1 is zero. */
    EXPECT_FALSE(Ops::gaussian(raw_a.data(), sa, raw_b.data(), sb, 2, z.data(), error.data(), &result));
    EXPECT_EQ(result, W(-7));
    const uint64_t first = static_cast<uint64_t>(Ops::raw_load(z.data()));
    ASSERT_TRUE(first != 0 && (first & (first - 1)) == 0);
    unsigned fractional = 0;
    while ((UINT64_C(1) << fractional) != first) ++fractional;
    ASSERT_EQ(fractional, sizeof(W) == 4 ? 0u : 33u);
    EXPECT_EQ(Ops::raw_load(z.data() + 1), 0);
    const uint64_t expected_bound = (UINT64_C(1) << (fractional + 30)) + (UINT64_C(1) << 31);
    const uint64_t exact_error_q32 = UINT64_C(1) << (fractional + 28);
    EXPECT_EQ(Ops::error_load(error.data() + 1), expected_bound);
    EXPECT_GE(Ops::error_load(error.data() + 1), exact_error_q32);
    ASSERT_EQ(Ops::mixed(deviation.data(), lower.data(), 2, solve.data(), &result), LINEARS_OK);
    EXPECT_EQ(result, W(0.501953125));
}

TYPED_TEST(QGaussianInteger, ScaleConversionKeepsRawBytesAcrossRoundingModes) {
    using W = TypeParam;
    using Ops = GaussianOps<W>;
    using Scale = typename Ops::Scale;
    std::fenv_t saved;
    ASSERT_EQ(std::fegetenv(&saved), 0);
    struct Restore { std::fenv_t *env; ~Restore() { std::fesetround(FE_TONEAREST); std::fesetenv(env); } } restore{&saved};
    std::array<W, 6> values{W(0.1), W(-0.1), W(3.75), W(-2.25), std::ldexp(W(1), -20), W(1)};
    std::array<W, 6> baseline{}, actual{};
    Scale first, next;
    ASSERT_EQ(std::fesetround(FE_TONEAREST), 0);
    ASSERT_TRUE(Ops::vector(values.data(), values.size(), baseline.data(), &first));
    for (const int mode : {FE_UPWARD, FE_DOWNWARD, FE_TOWARDZERO}) {
        ASSERT_EQ(std::fesetround(mode), 0);
        ASSERT_TRUE(Ops::vector(values.data(), values.size(), actual.data(), &next));
        EXPECT_EQ(next.exponent, first.exponent);
        EXPECT_EQ(next.exact, first.exact);
        for (size_t i = 0; i < values.size(); ++i)
            EXPECT_EQ(Ops::raw_load(actual.data() + i), Ops::raw_load(baseline.data() + i)) << mode << ' ' << i;
    }
}

TYPED_TEST(QGaussianInteger, TinyFactorsScaleLocallyAndMixedRangePivotsRetry) {
    using W = TypeParam;
    using Ops = GaussianOps<W>;
    using Scale = typename Ops::Scale;
    const W tiny = W(sizeof(W) == 4 ? 1e-12L : 1e-24L);
    std::array<W, 1> lower{tiny}, deviation{tiny}, raw_a{}, raw_b{}, z{}, error{};
    Scale sa, sb;
    ASSERT_TRUE(Ops::prepare(lower.data(), 1, raw_a.data(), &sa));
    ASSERT_TRUE(Ops::vector(deviation.data(), 1, raw_b.data(), &sb));
    W result = W(-1);
    ASSERT_TRUE(Ops::gaussian(raw_a.data(), sa, raw_b.data(), sb, 1, z.data(), error.data(), &result));
    expect_certified(result, 0.5L, Ops::budget_shift);

    const W small_pivot = std::ldexp(W(1), -80);
    std::array<W, 4> mixed_lower{W(1), W(0), W(0), small_pivot};
    std::array<W, 2> mixed_deviation{W(0), small_pivot}, mixed_solve{};
    std::array<W, 4> wide_a{};
    EXPECT_FALSE(Ops::prepare(mixed_lower.data(), 2, wide_a.data(), &sa));
    ASSERT_EQ(Ops::mixed(mixed_deviation.data(), mixed_lower.data(), 2, mixed_solve.data(), &result), LINEARS_OK);
    EXPECT_EQ(result, W(0.5));
}

TYPED_TEST(QGaussianInteger, CancellationAndFinalRangeUseOriginalMixedPath) {
    using W = TypeParam;
    using Ops = GaussianOps<W>;
    using Scale = typename Ops::Scale;
    const W small = std::ldexp(W(1), -20);
    std::array<W, 4> lower{W(3), W(0), W(3), small}, raw_a{};
    std::array<W, 2> deviation{W(1), W(1)}, raw_b{}, z{}, error{}, expected_solve{}, actual_solve{};
    Scale sa, sb;
    W expected = W(-11), actual = W(-12);
    ASSERT_TRUE(Ops::prepare(lower.data(), 2, raw_a.data(), &sa));
    ASSERT_TRUE(Ops::vector(deviation.data(), 2, raw_b.data(), &sb));
    /* The first 2/3 rounding is amplified by the tiny second pivot. */
    const int accepted = Ops::gaussian(raw_a.data(), sa, raw_b.data(), sb, 2,
                                       z.data(), error.data(), &actual);
    ASSERT_EQ(Ops::mixed(deviation.data(), lower.data(), 2, expected_solve.data(), &expected), LINEARS_OK);
    if (accepted) {
        expect_certified(actual, gaussian_reference(
            std::vector<W>(lower.begin(), lower.end()),
            std::vector<W>(deviation.begin(), deviation.end()), 2), Ops::budget_shift);
    } else {
        ASSERT_EQ(Ops::retry(deviation.data(), lower.data(), 2, actual_solve.data(), &actual,
                             raw_a.data(), sa, raw_b.data(), error.data()), LINEARS_OK);
        EXPECT_EQ(actual, expected);
        EXPECT_EQ(actual_solve, expected_solve);
    }

    std::array<W, 1> unit{W(1)}, huge{std::numeric_limits<W>::max()}, cache{}, b{}, e{};
    ASSERT_TRUE(Ops::prepare(unit.data(), 1, cache.data(), &sa));
    actual = W(-13);
    EXPECT_EQ(Ops::retry(huge.data(), unit.data(), 1, actual_solve.data(), &actual,
                         cache.data(), sa, b.data(), e.data()), LINEARS_ERROR_RANGE);
    EXPECT_EQ(actual, W(-13));
}

TYPED_TEST(QGaussianIntegration, ExactMinimumDenseUpdateCallsEachCallbackOnce) {
    using A = TypeParam;
    using Raw = typename A::Raw;
    constexpr size_t count = 4;
    PfState<A> fixture(count, 1, 3);
    const Dense noise(3, 3, {
        0.0625L, 0.03125L, 0.025L,
        0.03125L, 0.055625L, 0.0225L,
        0.025L, 0.0225L, 0.035L});
    for (size_t i = 0; i < count; ++i) {
        fixture.weights[i + 1] = std::numeric_limits<Raw>::max();
        fixture.particles[i + 1] = A::encode((static_cast<int>(i) - 1) * 0.25L);
    }
    for (size_t i = 0; i < 9; ++i) fixture.r[i + 1] = A::encode(noise.values[i]);
    Matrix<A> measurement(Dense(3, 1, {0.02L, -0.01L, 0.03L}));
    auto z = measurement.view();
    Model observation{[](const Vec& x) { return Vec{0.1L * x[0], -0.2L * x[0], 0.05L * x[0]}; }}, residual;
    residual.residual = [](const Vec& a, const Vec& b) {
        Vec d(a.size());
        for (size_t i = 0; i < d.size(); ++i) d[i] = a[i] - b[i];
        return d;
    };
    auto workspace = pf_update_workspace<A>(count, 1, 3);
    ASSERT_EQ(Pf<A>::update(&fixture.state, &z, map_callback<A>, &observation,
                            residual_callback<A>, &residual, workspace.view()), LINEARS_OK);
    EXPECT_EQ(observation.calls, static_cast<int>(count));
    EXPECT_EQ(residual.calls, static_cast<int>(count));
    expect_raw_weight_scale<A>(fixture.weights);
    fixture.guards();
    workspace.guards();
}

TYPED_TEST(QGaussianIntegration, NonpowerDivisionCacheUsesExactMinimumWorkspace) {
    using A = TypeParam;
    using Raw = typename A::Raw;
    constexpr size_t count = 4;
    PfState<A> fixture(count, 1, 3);
    for (size_t i = 0; i < count; ++i) {
        fixture.weights[i + 1] = std::numeric_limits<Raw>::max();
        fixture.particles[i + 1] = 0;
    }
    const long double diagonal[] = {0.5625L, 0.390625L, 0.19140625L};
    for (size_t i = 0; i < 9; ++i) fixture.r[i + 1] = 0;
    for (size_t i = 0; i < 3; ++i) fixture.r[i * 3 + i + 1] = A::encode(diagonal[i]);
    Matrix<A> measurement(Dense(3, 1, {0, 0, 0}));
    auto z = measurement.view();
    Model observation{[](const Vec&) { return Vec{0, 0, 0}; }};
    auto workspace = pf_update_workspace<A>(count, 1, 3);
    ASSERT_EQ(Pf<A>::update(&fixture.state, &z, map_callback<A>, &observation,
                            nullptr, nullptr, workspace.view()), LINEARS_OK);
    EXPECT_EQ(observation.calls, static_cast<int>(count));
    expect_raw_weight_scale<A>(fixture.weights);
    fixture.guards();
    workspace.guards();
}

TYPED_TEST(QGaussianIntegration, RangeRetryIsAtomicAndDoesNotRepeatCallbacks) {
    using A = TypeParam;
    using Raw = typename A::Raw;
    constexpr size_t count = 3;
    PfState<A> fixture(count, 2, 1); /* m=1 also covers the local B/E slots. */
    for (size_t i = 0; i < count; ++i) {
        fixture.weights[i + 1] = std::numeric_limits<Raw>::max();
        fixture.particles[2 * i + 1] = fixture.particles[2 * i + 2] = 0;
    }
    fixture.r[1] = A::encode(1);
    Matrix<A> measurement(Dense(1, 1, {0}));
    auto z = measurement.view();
    Model observation{[](const Vec&) { return Vec{0}; }}, residual;
    residual.residual = [&residual](const Vec&, const Vec&) {
        return Vec{residual.calls == static_cast<int>(count) ?
                   static_cast<long double>(std::numeric_limits<Work<A>>::max()) : 0};
    };
    auto workspace = pf_update_workspace<A>(count, 2, 1);
    const auto before = fixture.snapshot();
    EXPECT_EQ(Pf<A>::update(&fixture.state, &z, map_callback<A>, &observation,
                            residual_callback<A>, &residual, workspace.view()), LINEARS_ERROR_RANGE);
    EXPECT_EQ(observation.calls, static_cast<int>(count));
    EXPECT_EQ(residual.calls, static_cast<int>(count));
    EXPECT_EQ(fixture.snapshot(), before);
    fixture.guards();
    workspace.guards();
}

} // namespace
