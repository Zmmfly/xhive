/* Internal coverage of the reciprocal-pivot division used by the Q UKF gain
 * solve, in every rounding mode. A public update cannot easily place a gain
 * at the overflow threshold, so these tests call the private helper on
 * sqrt-derived pivots directly. Dynamic rounding must reach the helper's
 * arithmetic: keep this declaration local to this test TU. */
#if defined(__clang__)
#pragma STDC FENV_ACCESS ON
#elif defined(__GNUC__)
#pragma GCC optimize ("rounding-math")
#endif

#include "../src/ukf_internal.h"

#include <gtest/gtest.h>

#include <cfenv>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <random>
#include <type_traits>

namespace {

template<class W> struct Pivot;
template<> struct Pivot<float> {
    using Bits = uint32_t;
    static float divide(float value, float inverse, float pivot) { return ukf_pivot_divide_f32(value, inverse, pivot); }
    static constexpr float top_binade = 0x1p127f;
};
template<> struct Pivot<double> {
    using Bits = uint64_t;
    static double divide(double value, double inverse, double pivot) { return ukf_pivot_divide_f64(value, inverse, pivot); }
    static constexpr double top_binade = 0x1p1023;
};

constexpr int modes[] = {FE_TONEAREST, FE_UPWARD, FE_DOWNWARD, FE_TOWARDZERO};

struct Rounding {
    explicit Rounding(int mode) { EXPECT_EQ(std::fesetround(mode), 0); }
    ~Rounding() { std::fesetround(FE_TONEAREST); }
};

template<class W> W from_bits(typename Pivot<W>::Bits bits) {
    W value;
    std::memcpy(&value, &bits, sizeof(value));
    return value;
}

struct Counts {
    long cases = 0, product_nonfinite = 0, product_top_binade = 0, quotient_overflow = 0;
};

/* The helper must agree with the directly rounded quotient on finiteness;
 * a top-binade or nonfinite product returns that quotient exactly, and an
 * ordinary product stays within the two extra roundings. */
template<class W> void check(W value, W pivot, Counts &counts) {
    volatile W v = value, p = pivot;
    const W inverse = W(1) / p;
    const W quotient = v / p;
    const W product = v * inverse;
    const W result = Pivot<W>::divide(v, inverse, p);
    ++counts.cases;
    counts.product_nonfinite += !std::isfinite(product);
    counts.product_top_binade += std::isfinite(product) && std::fabs(product) >= Pivot<W>::top_binade;
    counts.quotient_overflow += !std::isfinite(quotient);
    ASSERT_EQ(std::isfinite(result), std::isfinite(quotient))
        << std::hexfloat << "value=" << value << " pivot=" << pivot << " result=" << result << " quotient=" << quotient;
    if (!std::isfinite(product) || std::fabs(product) >= Pivot<W>::top_binade) {
        EXPECT_EQ(result, quotient) << std::hexfloat << "value=" << value << " pivot=" << pivot;
    } else {
        EXPECT_EQ(result, product);
        EXPECT_LE(std::fabs(static_cast<long double>(result) - quotient),
                  4 * std::numeric_limits<W>::epsilon() * std::fabs(static_cast<long double>(quotient)));
    }
}

template<class W> class QUkfPivot : public ::testing::Test {};
using WorkTypes = ::testing::Types<float, double>;
TYPED_TEST_SUITE(QUkfPivot, WorkTypes);

/* Review counterexamples: the product rounds to the largest finite value
 * while the nearest quotient overflows. */
TYPED_TEST(QUkfPivot, TopBinadeProductsKeepTheQuotientOverflowStatus) {
    using W = TypeParam;
    const W pivot = std::is_same<W, float>::value ? W(0x1.5c410ap-2f) : W(0x1.51ada4482822cp-2);
    const W value = std::is_same<W, float>::value ? W(0x1.5c410ap126f) : W(0x1.51ada4482822cp1022);
    for (int mode : modes) {
        SCOPED_TRACE(mode);
        Rounding rounding(mode);
        for (W sign : {W(1), W(-1)}) {
            Counts counts;
            check<W>(sign * value, pivot, counts);
            if (mode == FE_TONEAREST) {
                volatile W v = sign * value, p = pivot;
                const W inverse = W(1) / p;
                EXPECT_EQ(std::fabs(v * inverse), std::numeric_limits<W>::max());
                EXPECT_EQ(Pivot<W>::divide(v, inverse, p), sign * std::numeric_limits<W>::infinity());
            }
        }
    }
}

/* Square-root pivots over a wide exponent range with values a few ulps
 * around max*pivot and around the start of the top binade, both signs. */
TYPED_TEST(QUkfPivot, SqrtPivotsAgreeWithDivisionNearTheOverflowThreshold) {
    using W = TypeParam;
    using Bits = typename Pivot<W>::Bits;
    constexpr int mantissa = std::numeric_limits<W>::digits - 1;
    constexpr int exponent_bias = std::numeric_limits<W>::max_exponent - 1;
    for (int mode : modes) {
        SCOPED_TRACE(mode);
        Rounding rounding(mode);
        std::mt19937_64 random(0x9e3779b97f4a7c15u + static_cast<unsigned>(mode));
        Counts counts;
        for (int trial = 0; trial < 20000; ++trial) {
            /* Variance exponents in [-60, 60) keep sqrt pivots normal in both types. */
            const Bits exponent = static_cast<Bits>(exponent_bias - 60 + static_cast<int>(random() % 120));
            const Bits fraction = static_cast<Bits>(random()) & ((Bits(1) << mantissa) - 1);
            const W pivot = std::sqrt(from_bits<W>((exponent << mantissa) | fraction));
            for (W sign : {W(1), W(-1)}) {
                for (W target : {std::numeric_limits<W>::max(), Pivot<W>::top_binade}) {
                    const W center = target * pivot;
                    if (!std::isfinite(center)) continue;
                    W value = center;
                    for (int step = 0; step < 3; ++step) value = std::nextafter(value, W(0));
                    for (int step = 0; step < 7; ++step) {
                        check<W>(sign * value, pivot, counts);
                        value = std::nextafter(value, std::numeric_limits<W>::infinity());
                        if (!std::isfinite(value)) break;
                    }
                }
            }
        }
        /* Every branch class of the helper was reached in this mode; only
         * rounding toward zero never overflows a finite product or quotient. */
        EXPECT_GT(counts.product_top_binade, 0);
        if (mode != FE_TOWARDZERO) {
            EXPECT_GT(counts.product_nonfinite, 0);
            EXPECT_GT(counts.quotient_overflow, 0);
        }
    }
}

TYPED_TEST(QUkfPivot, OrdinaryQuotientsStayWithinTwoRoundings) {
    using W = TypeParam;
    std::mt19937_64 random(0x51);
    std::uniform_real_distribution<double> unit(-1, 1), exponent(-40, 40);
    for (int mode : modes) {
        SCOPED_TRACE(mode);
        Rounding rounding(mode);
        Counts counts;
        for (int trial = 0; trial < 20000; ++trial) {
            const W pivot = std::sqrt(static_cast<W>(std::ldexp(1 + std::fabs(unit(random)), static_cast<int>(exponent(random)))));
            const W value = static_cast<W>(std::ldexp(unit(random), static_cast<int>(exponent(random))));
            check<W>(value, pivot, counts);
        }
        EXPECT_EQ(counts.product_nonfinite + counts.product_top_binade + counts.quotient_overflow, 0);
    }
}

}  // namespace
