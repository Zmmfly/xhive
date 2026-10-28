#include "filters_pf.h"
#include "../src/ukf_internal.h"

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cfenv>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <random>
#include <type_traits>
#include <vector>

/* Exercise the private arithmetic directly: public signal rounding can hide
 * a changed half-LSB or likelihood result before the final transaction. */
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
template<class W> auto representation(W value) {
    std::conditional_t<sizeof(W) == 4, uint32_t, uint64_t> bits;
    std::memcpy(&bits, &value, sizeof(value));
    return bits;
}

template<class W, int B> int64_t old_raw(W value) {
    const W scale = static_cast<W>(UINT64_C(1) << B);
    const W scaled = value * scale;
    return static_cast<int64_t>(scaled >= 0 ? scaled + W(0.5) : scaled - W(0.5));
}

template<class W> W old_unsigned_value(pf_q_u128_t value) {
    return W(value.hi) * W(18446744073709551616.0) + W(value.lo);
}

template<class W> W old_signed_value(pf_q_i128_t value) {
    pf_q_u128_t magnitude;
    if (value.hi < 0) {
        magnitude.lo = ~value.lo + 1;
        magnitude.hi = ~static_cast<uint64_t>(value.hi) + (magnitude.lo == 0);
        return -old_unsigned_value<W>(magnitude);
    }
    magnitude.lo = value.lo;
    magnitude.hi = static_cast<uint64_t>(value.hi);
    return old_unsigned_value<W>(magnitude);
}

template<class W, int B>
W old_dot(const W *a, size_t astride, const W *b, size_t bstride, size_t count) {
    pf_q_i128_t sum = {0, 0};
    W extended = 0;
    const W scale = static_cast<W>(UINT64_C(1) << B);
    for (size_t i = 0; i < count; ++i) {
        const W x = a[i * astride], y = b[i * bstride];
        if (x >= -1 && x <= 1 && y >= -1 && y <= 1) {
            const int64_t product = old_raw<W, B>(x) * old_raw<W, B>(y);
            pf_q_iadd(&sum, pf_q_magnitude(product), product < 0);
        } else {
            extended += x * y;
        }
    }
    return extended + (old_signed_value<W>(sum) / scale) / scale;
}

/* Freeze the old F31/F63 likelihood and 128-bit normalization operations,
 * independently of any q15 product-width optimization. */
template<class W, class S, int B>
linears_status_t old_reweight(const linears_matrix_t *weights, W *exponents,
                             S *staged, size_t count) {
    const S *w = static_cast<const S *>(weights->data);
    const W likelihood_scale = static_cast<W>(UINT64_C(1) << (2 * B + 1));
    const S maximum_signal = std::numeric_limits<S>::max();
    pf_q_u128_t maximum = {0, 0};
    W lowest = 0;
    bool found = false;
    for (size_t i = 0; i < count; ++i) {
        if (w[i] < 0) return LINEARS_ERROR_DOMAIN;
        if (!std::isfinite(exponents[i])) return LINEARS_ERROR_RANGE;
        if (w[i] != 0 && (!found || exponents[i] < lowest)) {
            lowest = exponents[i];
            found = true;
        }
    }
    if (!found) return LINEARS_ERROR_DOMAIN;
    for (size_t i = 0; i < count; ++i) {
        exponents[i] = w[i] == 0 ? 0 : std::exp(-(exponents[i] - lowest));
        const uint64_t likelihood = static_cast<uint64_t>(exponents[i] * likelihood_scale + W(0.5));
        const auto product = pf_q_umul(likelihood, static_cast<uint64_t>(w[i]));
        if (pf_q_ucmp(product, maximum) > 0) maximum = product;
    }
    const unsigned shift = pf_q_ubits(maximum);
    const W reciprocal = std::ldexp(W(maximum_signal) / old_unsigned_value<W>(maximum), int(shift));
    const auto multiplier = static_cast<uint32_t>(reciprocal + W(0.5));
    for (size_t i = 0; i < count; ++i) {
        const uint64_t likelihood = static_cast<uint64_t>(exponents[i] * likelihood_scale + W(0.5));
        const auto product = pf_q_umul(likelihood, static_cast<uint64_t>(w[i]));
        const uint64_t value = pf_q_ucmp(product, maximum) == 0 ? uint64_t(maximum_signal) :
            pf_q_uround(pf_q_umul32(product, multiplier), shift);
        staged[i] = static_cast<S>(std::min(value, uint64_t(maximum_signal)));
    }
    return LINEARS_OK;
}

TEST(ParticleQArithmetic, Q15RawPreservesEveryHalfLsbNeighborhood) {
    for (int32_t raw = INT16_MIN; raw <= INT16_MAX; ++raw) {
        const float half = (float(raw) + 0.5f) * 0x1p-15f;
        for (const float value : {std::nextafter(half, -1.0f), half, std::nextafter(half, 1.0f)}) {
            ASSERT_EQ(pf_q_q15_raw(value), (old_raw<float, 15>(value))) << raw << ' ' << value;
        }
    }
    for (const float value : {-1.0f, -0.0f, 0.0f, 1.0f})
        EXPECT_EQ(pf_q_q15_raw(value), (old_raw<float, 15>(value)));
    /* This neighbor deliberately double-rounds up in the legacy expression. */
    const float below = std::nextafter(0x1p-16f, 0.0f);
    EXPECT_EQ(pf_q_q15_raw(below), 1);
    EXPECT_EQ(pf_q_q15_raw(-below), -1);
    EXPECT_EQ(pf_q_q15_raw(1.0f), INT64_C(32768));
}

TEST(ParticleQArithmetic, Q31RawPreservesHalfLsbNeighborsAndPositiveOne) {
    std::mt19937 random(0x71b49u);
    std::vector<int32_t> cells{INT32_MIN, INT32_MIN + 1, -1073741824, -65536, -1, 0, 1,
                               65536, 1073741824, INT32_MAX - 1, INT32_MAX};
    for (size_t i = 0; i < 1000000; ++i)
        cells.push_back(static_cast<int32_t>(int64_t(random()) + INT32_MIN));
    for (const int32_t raw : cells) {
        const double half = (double(raw) + 0.5) * 0x1p-31;
        for (const double value : {std::nextafter(half, -1.0), half, std::nextafter(half, 1.0)})
            ASSERT_EQ(pf_q_q31_raw(value), (old_raw<double, 31>(value))) << raw;
    }
    const double below = std::nextafter(0x1p-32, 0.0);
    EXPECT_EQ(pf_q_q31_raw(below), 1);
    EXPECT_EQ(pf_q_q31_raw(-below), -1);
    EXPECT_EQ(pf_q_q31_raw(1.0), INT64_C(2147483648));
    EXPECT_EQ(pf_q_q31_raw(-1.0), -INT64_C(2147483648));
}

template<class W, int B, class Dot>
void check_dot(Dot dot) {
    std::mt19937 random(0x519ba2u + B);
    const W half = std::ldexp(W(1), -B - 1);
    const std::array<W, 15> edges{W(0), -W(0), W(1), W(-1), std::nextafter(W(1), W(0)),
        std::nextafter(W(-1), W(0)), std::nextafter(W(1), W(2)), std::nextafter(W(-1), W(-2)),
        half, -half, std::nextafter(half, W(0)), std::nextafter(-half, W(0)),
        std::numeric_limits<W>::denorm_min(), W(3.75), W(-2.5)};
    std::vector<W> a(3 * 1024), b(2 * 1024);
    for (size_t trial = 0; trial < 3000; ++trial) {
        const size_t count = trial < 7 ? std::array<size_t, 7>{0, 1, 2, 4, 31, 128, 1024}[trial]
                                      : random() % 129;
        for (size_t i = 0; i < count; ++i) {
            a[3 * i] = trial < 100 ? edges[(trial + i) % edges.size()] :
                W(int64_t(random()) + INT32_MIN) * W(0x1p-30);
            b[2 * i] = trial < 100 ? edges[(3 * trial + 2 * i) % edges.size()] :
                W(int64_t(random()) + INT32_MIN) * W(0x1p-31);
        }
        ASSERT_EQ(representation(dot(a.data(), 3, b.data(), 2, count)),
                  representation(old_dot<W, B>(a.data(), 3, b.data(), 2, count))) << trial;
    }
    for (const W sign : {W(1), W(-1)}) {
        std::fill(a.begin(), a.end(), W(1));
        std::fill(b.begin(), b.end(), sign);
        EXPECT_EQ(representation(dot(a.data(), 3, b.data(), 2, 1024)),
                  representation(old_dot<W, B>(a.data(), 3, b.data(), 2, 1024)));
        for (size_t i = 0; i < 1024; ++i) a[3 * i] = i % 2 == 0 ? W(1) : W(-1);
        EXPECT_EQ(representation(dot(a.data(), 3, b.data(), 2, 1024)),
                  representation(old_dot<W, B>(a.data(), 3, b.data(), 2, 1024)));
    }
}

TEST(ParticleQArithmetic, MixedDotsAreBitwiseIdentical) {
    check_dot<float, 15>(pf_q_q15_dot);
    check_dot<double, 31>(pf_q_q31_dot);
}

TEST(ParticleQArithmetic, RandomBoundedIeeeRawMatchesTheOriginalExpression) {
    std::mt19937_64 random(0x17cba9u);
    for (size_t i = 0; i < 1000000; ++i) {
        uint32_t fbits = uint32_t(random()) % UINT32_C(0x3f800001);
        uint64_t dbits = random() % UINT64_C(0x3ff0000000000001);
        if (i % 2 != 0) { fbits |= UINT32_C(0x80000000); dbits |= UINT64_C(0x8000000000000000); }
        float f;
        double d;
        std::memcpy(&f, &fbits, sizeof(f));
        std::memcpy(&d, &dbits, sizeof(d));
        ASSERT_EQ(pf_q_q15_raw(f), (old_raw<float, 15>(f))) << fbits;
        ASSERT_EQ(pf_q_q31_raw(d), (old_raw<double, 31>(d))) << dbits;
    }
}

/* Host builds must retain the native expression, not just its RN result.
 * A test-only ARM predefine build exercises the integer branch separately;
 * its host arithmetic cannot emulate the real soft libgcc fenv contract. */
#if !defined(__arm__)
TEST(ParticleQArithmetic, HostFallbackPreservesRoundingModesAndExceptions) {
    std::fenv_t saved;
    ASSERT_EQ(std::fegetenv(&saved), 0);
    struct Restore { std::fenv_t *env; ~Restore() { std::fesetenv(env); } } restore{&saved};
    const uint32_t ordinary_bits = UINT32_C(0x34000001);
    float ordinary;
    std::memcpy(&ordinary, &ordinary_bits, sizeof(ordinary));
    for (const int mode : {FE_TONEAREST, FE_UPWARD, FE_DOWNWARD, FE_TOWARDZERO}) {
        ASSERT_EQ(std::fesetround(mode), 0);
        for (const float source : {ordinary, -ordinary, std::nextafter(0x1p-16f, 0.0f),
                                    -std::nextafter(0x1p-16f, 0.0f), 1.0f, -1.0f}) {
            volatile float value = source;
            std::feclearexcept(FE_ALL_EXCEPT);
            const auto actual = pf_q_q15_raw(value);
            const int actual_flags = std::fetestexcept(FE_ALL_EXCEPT);
            std::feclearexcept(FE_ALL_EXCEPT);
            const auto expected = old_raw<float, 15>(value);
            EXPECT_EQ(actual, expected);
            EXPECT_EQ(actual_flags, std::fetestexcept(FE_ALL_EXCEPT));
        }
        for (const double source : {double(ordinary), -double(ordinary), std::nextafter(0x1p-32, 0.0),
                                     -std::nextafter(0x1p-32, 0.0), 1.0, -1.0}) {
            volatile double value = source;
            std::feclearexcept(FE_ALL_EXCEPT);
            const auto actual = pf_q_q31_raw(value);
            const int actual_flags = std::fetestexcept(FE_ALL_EXCEPT);
            std::feclearexcept(FE_ALL_EXCEPT);
            const auto expected = old_raw<double, 31>(value);
            EXPECT_EQ(actual, expected);
            EXPECT_EQ(actual_flags, std::fetestexcept(FE_ALL_EXCEPT));
        }
    }
}
#endif

TEST(ParticleQArithmetic, WideAndSingleLimbConversionsAreBitwiseIdentical) {
    std::mt19937_64 random(0x512a17u);
    for (size_t i = 0; i < 10000; ++i) {
        const pf_q_u128_t value{random(), i % 2 == 0 ? 0 : random() >> 1};
        EXPECT_EQ(representation(pf_q_q15_unsigned_value(value)), representation(old_unsigned_value<float>(value)));
        EXPECT_EQ(representation(pf_q_q31_unsigned_value(value)), representation(old_unsigned_value<double>(value)));
    }
}

template<class W, class S, int B, class Reweight>
void check_reweight(Reweight reweight) {
    std::mt19937 random(0x23aa5u + B);
    const S maximum = std::numeric_limits<S>::max();
    for (size_t trial = 0; trial < 1000; ++trial) {
        const size_t count = 1 + random() % 128;
        std::vector<S> weights(count), expected(count, S(19)), actual(count, S(19));
        std::vector<W> exponents(count);
        for (size_t i = 0; i < count; ++i) {
            weights[i] = i % 7 == 0 ? 0 : i % 7 == 1 ? 1 :
                         i % 7 == 2 ? maximum : S(random() % uint32_t(maximum));
            exponents[i] = W(int64_t(random() % 32001) - 16000) * W(0.0625);
        }
        weights[trial % count] = maximum;
        linears_matrix_t view{weights.data(), count, 1};
        const auto input_exponents = exponents;
        auto reference_exponents = exponents;
        W lowest = 0;
        bool found = false;
        for (size_t i = 0; i < count; ++i) {
            if (weights[i] != 0 && (!found || input_exponents[i] < lowest)) {
                lowest = input_exponents[i];
                found = true;
            }
        }
        ASSERT_EQ((old_reweight<W, S, B>(&view, reference_exponents.data(), expected.data(), count)), LINEARS_OK);
        ASSERT_EQ(reweight(&view, exponents.data(), actual.data(), count), LINEARS_OK);
        ASSERT_EQ(actual, expected) << trial;
        for (size_t i = 0; i < count; ++i) {
            /* Scratch now contains raw likelihood bytes, not a float/double. */
            uint64_t raw;
            if constexpr (sizeof(W) == 4) {
                uint32_t value;
                std::memcpy(&value, exponents.data() + i, sizeof(value));
                raw = value;
            } else {
                std::memcpy(&raw, exponents.data() + i, sizeof(raw));
            }
            if (weights[i] == 0) {
                EXPECT_EQ(raw, 0) << trial << ' ' << i;
            } else if constexpr (B == 15) {
                EXPECT_EQ(raw, pf_q_q15_likelihood(input_exponents[i] - lowest)) << trial << ' ' << i;
            } else {
                EXPECT_EQ(raw, pf_q_q31_likelihood(input_exponents[i] - lowest)) << trial << ' ' << i;
            }
        }
    }
    for (int failure = 0; failure < 4; ++failure) {
        std::array<S, 3> weights{1, maximum, 1}, staged{7, 7, 7};
        std::array<W, 3> exponents{W(0), W(1), W(2)};
        if (failure == 0) weights[2] = -1;
        if (failure == 1) weights.fill(0);
        if (failure == 2) exponents[2] = std::numeric_limits<W>::infinity();
        if (failure == 3) exponents[2] = std::numeric_limits<W>::quiet_NaN();
        const auto before = exponents;
        linears_matrix_t view{weights.data(), weights.size(), 1};
        EXPECT_EQ(reweight(&view, exponents.data(), staged.data(), weights.size()),
                  failure < 2 ? LINEARS_ERROR_DOMAIN : LINEARS_ERROR_RANGE);
        EXPECT_EQ(staged, (std::array<S, 3>{7, 7, 7}));
        for (size_t i = 0; i < exponents.size(); ++i)
            EXPECT_EQ(representation(exponents[i]), representation(before[i]));
    }
}

TEST(ParticleQArithmetic, ReweightKeepsLegacyWeightsRawCacheAndAtomicFailures) {
    check_reweight<float, int16_t, 15>(pf_q_q15_reweight);
    check_reweight<double, int32_t, 31>(pf_q_q31_reweight);
}
} // namespace
