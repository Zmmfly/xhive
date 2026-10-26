#include "filter_test_support.h"

using namespace filters_test;
template <typename A> class FirDesignTest : public ::testing::Test {};
TYPED_TEST_SUITE(FirDesignTest, Formats);

namespace {

constexpr int kRate = 1024;

/* Independent long-double windowed sinc normalized to unit DC gain. */
std::vector<long double> reference_lowpass(std::size_t taps, long double ratio) {
    const long double pi = std::acos(-1.0L);
    const long double half = static_cast<long double>(taps / 2);
    std::vector<long double> h(taps);
    long double sum = 0;
    for (std::size_t n = 0; n < taps; ++n) {
        const long double k = static_cast<long double>(n) - half;
        h[n] = k == 0 ? 2 * ratio
                      : (0.54L + 0.46L * std::cos(pi * k / half)) * std::sin(2 * pi * ratio * k) / (pi * k);
        sum += h[n];
    }
    for (auto &value : h) value /= sum;
    return h;
}

enum class Kind { low, high, band };

std::vector<long double> reference(Kind kind, std::size_t taps, double low, double high) {
    auto h = reference_lowpass(taps, (kind == Kind::band ? high : low) / kRate);
    if (kind == Kind::high) {
        for (auto &value : h) value = -value;
        h[taps / 2] += 1;
    } else if (kind == Kind::band) {
        const auto lower = reference_lowpass(taps, low / kRate);
        for (std::size_t n = 0; n < taps; ++n) h[n] -= lower[n];
    }
    return h;
}

template <typename A>
linears_status_t design(Kind kind, typename A::Coefficient *taps, std::size_t count, double low, double high) {
    using Hz = typename A::Hz;
    if (kind == Kind::low) return A::fir_design_lpf(taps, count, Hz(low), Hz(kRate));
    if (kind == Kind::high) return A::fir_design_hpf(taps, count, Hz(low), Hz(kRate));
    return A::fir_design_bpf(taps, count, Hz(low), Hz(high), Hz(kRate));
}

} // namespace

TYPED_TEST(FirDesignTest, TapsMatchReferenceWithExactDcAndSymmetry) {
    using A = TypeParam;
    for (std::size_t taps : {std::size_t(1), std::size_t(3), std::size_t(31), std::size_t(101)}) {
        for (Kind kind : {Kind::low, Kind::high, Kind::band}) {
            std::vector<typename A::Coefficient> h(taps);
            ASSERT_EQ(design<A>(kind, h.data(), taps, 100, 300), LINEARS_OK);
            const auto expected = reference(kind, taps, 100, 300);
            // Fixed: each tap is rounded once (1/2 F31 LSB) and the center tap
            // absorbs the others' rounding. Floating: per-tap libm/division
            // rounding plus the O(taps) normalization sum.
            const double tolerance = A::fixed ? (taps + 1) * 0x1p-31 : 4.0 * (taps + 4) * A::unit;
            double sum = 0;
            for (std::size_t n = 0; n < taps; ++n) {
                EXPECT_NEAR(A::real_coefficient(h[n]), static_cast<double>(expected[n]), tolerance)
                    << "taps " << taps << " n " << n;
                sum += A::real_coefficient(h[n]);
            }
            const double dc = kind == Kind::low ? 1.0 : 0.0;
            if constexpr (A::fixed) {
                linears_fixed_t raw_sum = 0;
                for (std::size_t n = 0; n < taps; ++n) {
                    raw_sum += h[n];
                    EXPECT_EQ(h[n], h[taps - 1 - n]);
                }
                EXPECT_EQ(raw_sum, kind == Kind::low ? LINEARS_FIXED_ONE : 0);
            } else {
                EXPECT_NEAR(sum, dc, (taps + 1) * A::unit);
            }
        }
    }
}

TYPED_TEST(FirDesignTest, DesignedLowpassPassesAndRejectsThroughFir) {
    using A = TypeParam;
    constexpr std::size_t taps = 63;
    std::vector<typename A::Coefficient> h(taps);
    std::vector<typename A::Scalar> history(taps);
    ASSERT_EQ(A::fir_design_lpf(h.data(), taps, 100, kRate), LINEARS_OK);
    for (int bin : {20, 300}) {
        typename A::Fir fir;
        ASSERT_EQ(A::fir_init(&fir, h.data(), taps, history.data(), taps), LINEARS_OK);
        const auto gain = measure_gain<A>([&](auto input, auto *output) {
            return A::fir_step(&fir, input, output);
        }, bin, kRate, 0.5, static_cast<int>(taps));
        // Hamming ripple is about 0.0022 around the ideal response (53 dB) past
        // the 3.3*rate/taps transition. Normalizing to exactly one at DC, itself
        // possibly a ripple extreme, allows twice that in the passband. Q output
        // rounding adds 2 LSB/amplitude.
        if (bin == 20) EXPECT_NEAR(std::abs(gain), 1.0, 0.0045 + 4 * A::unit);
        else EXPECT_LT(std::abs(gain), 0.0025 + 4 * A::unit);
    }
}

TYPED_TEST(FirDesignTest, InvalidArgumentsLeaveTapsUnchanged) {
    using A = TypeParam;
    using Hz = typename A::Hz;
    const auto sentinel = A::coefficient(0.875);
    std::vector<typename A::Coefficient> h(8, sentinel);
    const Hz nan = std::numeric_limits<Hz>::quiet_NaN();
    EXPECT_EQ(A::fir_design_lpf(nullptr, 7, 100, kRate), LINEARS_ERROR_ARGUMENT);
    EXPECT_EQ(A::fir_design_lpf(h.data(), 0, 100, kRate), LINEARS_ERROR_ARGUMENT);
    EXPECT_EQ(A::fir_design_lpf(h.data(), 8, 100, kRate), LINEARS_ERROR_DOMAIN);
    EXPECT_EQ(A::fir_design_hpf(h.data(), 7, 0, kRate), LINEARS_ERROR_DOMAIN);
    EXPECT_EQ(A::fir_design_hpf(h.data(), 7, Hz(kRate / 2), kRate), LINEARS_ERROR_DOMAIN);
    EXPECT_EQ(A::fir_design_hpf(h.data(), 7, 100, 0), LINEARS_ERROR_DOMAIN);
    EXPECT_EQ(A::fir_design_hpf(h.data(), 7, nan, kRate), LINEARS_ERROR_RANGE);
    EXPECT_EQ(A::fir_design_bpf(h.data(), 7, 300, 100, kRate), LINEARS_ERROR_DOMAIN);
    EXPECT_EQ(A::fir_design_bpf(h.data(), 7, 100, 100, kRate), LINEARS_ERROR_DOMAIN);
    EXPECT_EQ(A::fir_design_bpf(h.data(), 7, 100, nan, kRate), LINEARS_ERROR_RANGE);
    EXPECT_EQ(A::fir_design_bpf(h.data(), 6, 100, 300, kRate), LINEARS_ERROR_DOMAIN);
    for (const auto &value : h) EXPECT_EQ(value, sentinel);
}
