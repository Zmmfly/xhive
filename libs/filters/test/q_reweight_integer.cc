#include "filters_pf.h"
#include "../src/ukf_internal.h"

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <random>
#include <type_traits>
#include <vector>

static unsigned pf_q_f40_test_routes;
#define PF_Q_F40_ROUTE() (++pf_q_f40_test_routes)

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

template<class W> using Likelihood = std::conditional_t<sizeof(W) == 4, uint32_t, uint64_t>;

template<class W> bool fixed_exp(W delta, Likelihood<W> *out);
template<> bool fixed_exp<float>(float delta, uint32_t *out) { return pf_q15_exp_neg_try(delta, out) != 0; }
template<> bool fixed_exp<double>(double delta, uint64_t *out) { return pf_q31_exp_neg_try(delta, out) != 0; }

template<class W, int F> uint64_t old_likelihood(W delta) {
    const W scale = static_cast<W>(UINT64_C(1) << F);
    return static_cast<uint64_t>(std::exp(-delta) * scale + W(0.5));
}

template<class W> uint64_t cached(const W *slot);
template<> uint64_t cached<float>(const float *slot) {
    uint32_t raw;
    std::memcpy(&raw, slot, sizeof(raw));
    return raw;
}
template<> uint64_t cached<double>(const double *slot) {
    uint64_t raw;
    std::memcpy(&raw, slot, sizeof(raw));
    return raw;
}

template<class W, int F> uint64_t private_likelihood(W delta);
template<> uint64_t private_likelihood<float, 31>(float delta) { return pf_q_q15_likelihood(delta); }
template<> uint64_t private_likelihood<double, 63>(double delta) { return pf_q_q31_likelihood(delta); }

template<class W, class S, int F>
linears_status_t run_reweight(const linears_matrix_t *weights, W *exponents, S *staged, size_t count);
template<> linears_status_t run_reweight<float, int16_t, 31>(
    const linears_matrix_t *weights, float *exponents, int16_t *staged, size_t count) {
    return pf_q_q15_reweight(weights, exponents, staged, count);
}
template<> linears_status_t run_reweight<double, int32_t, 63>(
    const linears_matrix_t *weights, double *exponents, int32_t *staged, size_t count) {
    return pf_q_q31_reweight(weights, exponents, staged, count);
}

template<class W, int F> void check_fixed_exp() {
    constexpr long double tolerance = F == 31 ? 0x1p-27L : 0x1p-52L;
    const long double cutoff = (F + 1) * std::log(2.0L);
    std::mt19937_64 random(0x5046570a + F);
    unsigned succeeded = 0, retried = 0;
    const auto check = [&](W delta) {
        Likelihood<W> raw;
        if (!fixed_exp(delta, &raw)) {
            ++retried;
            return;
        }
        ++succeeded;
        const long double reference = std::exp(-static_cast<long double>(delta)) * std::ldexp(1.0L, F);
        const long double error = std::abs(static_cast<long double>(raw) - reference);
        ASSERT_LE(error, reference * tolerance + 0.75L) << delta;
    };
    for (int k = 0; k < 32 * (F + 1); ++k) {
        const long double delta = (k + 0.375L) * std::log(2.0L) / 32;
        check(static_cast<W>(delta));
    }
    for (unsigned i = 0; i < 10000; ++i) {
        const long double delta = cutoff * std::uniform_real_distribution<long double>(0, 1)(random);
        check(static_cast<W>(delta));
    }
    /* Ordinary inputs must exercise the integer path, not only compile it. */
    EXPECT_GT(succeeded, 9000u);
    EXPECT_LT(retried, 1100u);
    ::testing::Test::RecordProperty("integer_successes", succeeded);
    ::testing::Test::RecordProperty("certified_retries", retried);

    Likelihood<W> raw = 7;
    ASSERT_TRUE(fixed_exp(W(0), &raw));
    EXPECT_EQ(raw, UINT64_C(1) << F);
    ASSERT_TRUE(fixed_exp(std::numeric_limits<W>::infinity(), &raw));
    EXPECT_EQ(raw, 0);
    if constexpr (F == 31) {
        ASSERT_TRUE(fixed_exp(0x1.62e433p+4f, &raw));
    } else {
        ASSERT_TRUE(fixed_exp(0x1.62e42fefa39f1p+5, &raw));
    }
    EXPECT_EQ(raw, 0);
}

TEST(QReweightInteger, Q15FixedExpHasBoundedErrorAndOrdinaryCoverage) {
    check_fixed_exp<float, 31>();
}

TEST(QReweightInteger, Q31FixedExpHasBoundedErrorAndOrdinaryCoverage) {
    check_fixed_exp<double, 63>();
}

TEST(QReweightInteger, Q15UnsignedHornerBoundUsesTheCurrentConstants) {
    ASSERT_LT(pf_q_exp_q15.step, UINT64_C(1) << 35);
    ASSERT_EQ(pf_q_exp_q15.fractional, 40u);
    for (unsigned i = 0; i < pf_q_exp_q15.degree; ++i) {
        const uint64_t next = pf_q_exp_q15.coefficients[i + 1];
        const uint64_t upper_product = (next + 31) / 32;
        EXPECT_LT(upper_product, pf_q_exp_q15.coefficients[i]) << i;
        EXPECT_LE(pf_q_exp_q15.coefficients[i], UINT64_C(1) << 40);
    }
}

void check_unsigned_f40(pf_q_u128_t delta) {
    uint64_t expected = 17, actual = 19;
    const int old_status = pf_q_exp_fixed(&pf_q_exp_q15, delta, &expected);
    const int new_status = pf_q_exp_fixed_f40(delta, &actual);
    ASSERT_EQ(new_status, old_status) << "hi=" << delta.hi << " lo=" << delta.lo;
    if (old_status) {
        ASSERT_EQ(actual, expected) << "hi=" << delta.hi << " lo=" << delta.lo;
    }
}

TEST(QReweightInteger, Q15UnsignedF40ExactlyMatchesGenericAtCellsAndRandomInputs) {
    const uint64_t step = pf_q_exp_q15.step;
    const std::array<uint64_t, 7> remainders{0, 1, step / 2 - 1, step / 2, step / 2 + 1, step - 2, step - 1};
    std::mt19937_64 random(0x4634304e);
    for (uint64_t k = 0; k <= 1025; ++k) {
        for (uint64_t remainder : remainders) check_unsigned_f40({k * step + remainder, 0});
        check_unsigned_f40({k * step + random() % step, 0});
    }
    for (unsigned i = 0; i < 100000; ++i) {
        const uint64_t k = random() % 1025;
        check_unsigned_f40({k * step + random() % step, 0});
    }
    for (uint64_t delta : {step * 1025, step * 1026, UINT64_MAX}) {
        check_unsigned_f40({delta, 0});
        check_unsigned_f40({delta, 1});
    }
}

TEST(QReweightInteger, Q15PublicReweightRoutesUseUnsignedF40) {
    uint32_t direct = 0;
    uint64_t pair = 0, q31 = 0;
    pf_q_f40_test_routes = 0;
    ASSERT_TRUE(pf_q15_exp_neg_try(0.5f, &direct));
    EXPECT_EQ(pf_q_f40_test_routes, 1u);

    pf_q_f40_test_routes = 0;
    ASSERT_TRUE(pf_q_q15_likelihood_delta_try(0.5f, 0.0f, &pair));
    EXPECT_EQ(pf_q_f40_test_routes, 1u);

    pf_q_f40_test_routes = 0;
    (void)pf_q_q15_likelihood_pair(0.5f, 0.0f);
    EXPECT_EQ(pf_q_f40_test_routes, 1u);

    pf_q_f40_test_routes = 0;
    ASSERT_TRUE(pf_q31_exp_neg_try(0.5, &q31));
    EXPECT_EQ(pf_q_f40_test_routes, 0u);

    std::array<int16_t, 3> weights{INT16_MAX, 1, INT16_MAX}, staged{7, 7, 7};
    std::array<float, 3> exponents{0.0f, -1.0f, -2.0f};
    linears_matrix_t view{weights.data(), weights.size(), 1};
    pf_q_f40_test_routes = 0;
    ASSERT_EQ((run_reweight<float, int16_t, 31>(&view, exponents.data(), staged.data(), exponents.size())), LINEARS_OK);
    EXPECT_GT(pf_q_f40_test_routes, 0u);
}

TEST(QReweightInteger, CutoffNeighborsMatchTheLegacyBoundary) {
    const float q15 = 0x1.62e432p+4f;
    for (float d : {std::nextafter(q15, 0.0f), q15, std::nextafter(q15, INFINITY)}) {
        EXPECT_EQ((private_likelihood<float, 31>(d)), (old_likelihood<float, 31>(d))) << d;
    }
    const double q31 = 0x1.62e42fefa39f0p+5;
    for (double d : {std::nextafter(q31, 0.0), q31, std::nextafter(q31, INFINITY)}) {
        EXPECT_EQ((private_likelihood<double, 63>(d)), (old_likelihood<double, 63>(d))) << d;
    }
}

template<class W, class S, int F> void check_tiny_weights_and_cache() {
    constexpr S maximum = std::numeric_limits<S>::max();
    std::array<S, 3> weights{1, maximum, 0}, staged{7, 7, 7};
    std::array<W, 3> exponents{W(0), W(F == 31 ? 15 : 31), W(-1000)};
    const W tiny_delta = exponents[1];
    linears_matrix_t view{weights.data(), weights.size(), 1};
    ASSERT_EQ((run_reweight<W, S, F>(&view, exponents.data(), staged.data(), exponents.size())), LINEARS_OK);
    EXPECT_EQ(staged[0], maximum);
    EXPECT_GT(staged[1], 0) << "tiny nonzero posterior mass must survive";
    EXPECT_EQ(staged[2], 0);
    EXPECT_EQ(cached(exponents.data()), UINT64_C(1) << F);
    EXPECT_EQ(cached(exponents.data() + 1), (private_likelihood<W, F>(tiny_delta)));
    EXPECT_EQ(cached(exponents.data() + 2), 0);
}

TEST(QReweightInteger, Q15TinyWeightAndRawCache) {
    check_tiny_weights_and_cache<float, int16_t, 31>();
}

TEST(QReweightInteger, Q31TinyWeightAndRawCache) {
    check_tiny_weights_and_cache<double, int32_t, 63>();
}

template<class W, class S, int F> void check_normalized_distribution() {
    constexpr S maximum = std::numeric_limits<S>::max();
    constexpr long double tolerance = F == 31 ? 4e-3L : 2e-7L;
    std::mt19937 random(0x52570100 + F);
    long double maximum_error = 0;
    for (unsigned trial = 0; trial < 1000; ++trial) {
        const size_t count = 1 + random() % 128;
        std::vector<S> weights(count), staged(count, S(19));
        std::vector<W> exponents(count);
        std::vector<long double> expected(count);
        W lowest = 0;
        bool found = false;
        for (size_t i = 0; i < count; ++i) {
            weights[i] = i % 7 == 0 ? 0 : i % 7 == 1 ? 1 : i % 7 == 2 ? maximum :
                         static_cast<S>(random() % static_cast<uint32_t>(maximum));
            exponents[i] = W(static_cast<int64_t>(random() % 32001) - 16000) * W(0.0625);
        }
        weights[trial % count] = maximum;
        for (size_t i = 0; i < count; ++i) {
            if (weights[i] != 0 && (!found || exponents[i] < lowest)) {
                lowest = exponents[i];
                found = true;
            }
        }
        long double expected_sum = 0;
        for (size_t i = 0; i < count; ++i) {
            expected[i] = weights[i] == 0 ? 0 :
                static_cast<long double>(weights[i]) * std::exp(-static_cast<long double>(exponents[i] - lowest));
            expected_sum += expected[i];
        }
        linears_matrix_t view{weights.data(), count, 1};
        ASSERT_EQ((run_reweight<W, S, F>(&view, exponents.data(), staged.data(), count)), LINEARS_OK) << trial;
        long double actual_sum = 0;
        for (S value : staged) actual_sum += value;
        ASSERT_GT(actual_sum, 0) << trial;
        EXPECT_EQ(*std::max_element(staged.begin(), staged.end()), maximum) << trial;
        for (size_t i = 0; i < count; ++i) {
            const long double reference = expected[i] / expected_sum;
            const long double actual = staged[i] / actual_sum;
            const long double error = std::abs(actual - reference);
            maximum_error = std::max(maximum_error, error);
            ASSERT_LE(error, tolerance * (1 + reference)) << trial << ' ' << i;
            if (weights[i] == 0) EXPECT_EQ(staged[i], 0) << trial << ' ' << i;
        }
    }
    ::testing::Test::RecordProperty("maximum_absolute_normalized_error", maximum_error);
}

TEST(QReweightInteger, Q15NormalizedWeightsMatchLongDoubleOracle) {
    check_normalized_distribution<float, int16_t, 31>();
}

TEST(QReweightInteger, Q31NormalizedWeightsMatchLongDoubleOracle) {
    check_normalized_distribution<double, int32_t, 63>();
}

} // namespace
