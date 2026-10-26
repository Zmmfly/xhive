#include "filter_test_support.h"

using namespace filters_test;
template <typename A> class BpfTest : public ::testing::Test {};
TYPED_TEST_SUITE(BpfTest, Formats);

namespace {

/* One second at this rate makes every whole-hertz test tone an exact DFT bin. */
constexpr int kRate = 1024;
constexpr filters_order_t kOrders[] = {FILTERS_ORDER_1, FILTERS_ORDER_2};

double prewarp(double frequency) { return std::tan(std::acos(-1.0) * frequency / kRate); }

/* Bilinear map: H(e^jw) equals the analog design at s = j*tan(w/2). Order two
 * is the low-pass-to-band-pass transform of the Butterworth prototype, written
 * directly rather than as the production section split. */
std::complex<double> bandpass_oracle(filters_order_t order, double center, double bandwidth,
                                     double frequency) {
    const std::complex<double> s(0, prewarp(frequency));
    if (order == FILTERS_ORDER_1) return bandwidth * s / (s * s + bandwidth * s + center * center);
    const auto p = (s * s + center * center) / (bandwidth * s);
    return 1.0 / (p * p + std::sqrt(2.0) * p + 1.0);
}

template <typename A, typename Section> void expect_same_coefficients(const Section &a, const Section &b) {
    EXPECT_EQ(a.b0, b.b0);
    EXPECT_EQ(a.b1, b.b1);
    EXPECT_EQ(a.b2, b.b2);
    EXPECT_EQ(a.a1, b.a1);
    EXPECT_EQ(a.a2, b.a2);
}

template <typename Section> void expect_same_history(const Section &a, const Section &b) {
    EXPECT_EQ(a.x1, b.x1);
    EXPECT_EQ(a.x2, b.x2);
    EXPECT_EQ(a.y1, b.y1);
    EXPECT_EQ(a.y2, b.y2);
}

} // namespace

TYPED_TEST(BpfTest, BandFormMatchesTransformedButterworthWithMinus3DbEdges) {
    using A = TypeParam;
    const double low = prewarp(80), high = prewarp(160);
    for (auto order : kOrders) {
        for (int bin : {20, 80, 113, 160, 400}) {
            typename A::Bpf context;
            ASSERT_EQ(A::bpf_init_band(&context, order, 80, 160, kRate, A::make(0)), LINEARS_OK);
            const auto gain = measure_gain<A>([&](auto input, auto *output) {
                return A::bpf_step(&context, input, output);
            }, bin, kRate, 0.5, 4 * kRate);
            // 64 storage units cover coefficient rounding, recurrence rounding in
            // these moderately damped sections, and Q output rounding.
            EXPECT_LT(std::abs(gain - bandpass_oracle(order, std::sqrt(low * high), high - low, bin)),
                      64 * A::unit) << "order " << order << " bin " << bin;
            if (bin == 80 || bin == 160) EXPECT_NEAR(std::abs(gain), std::sqrt(0.5), 64 * A::unit);
        }
    }
}

TYPED_TEST(BpfTest, CenterFormMatchesTransformedButterworthWithUnitPeak) {
    using A = TypeParam;
    const double center = prewarp(120);
    for (auto order : kOrders) {
        for (int bin : {30, 100, 120, 150, 450}) {
            typename A::Bpf context;
            ASSERT_EQ(A::bpf_init_center(&context, order, 120, 2, kRate, A::make(0)), LINEARS_OK);
            const auto gain = measure_gain<A>([&](auto input, auto *output) {
                return A::bpf_step(&context, input, output);
            }, bin, kRate, 0.5, 4 * kRate);
            EXPECT_LT(std::abs(gain - bandpass_oracle(order, center, center / 2, bin)), 64 * A::unit)
                << "order " << order << " bin " << bin;
            if (bin == 120) EXPECT_NEAR(std::abs(gain), 1.0, 64 * A::unit);
        }
    }
}

TYPED_TEST(BpfTest, HeldOffsetGivesExactZeroAndRetuneKeepsHistory) {
    using A = TypeParam;
    for (auto order : kOrders) {
        typename A::Bpf context, fresh;
        typename A::Scalar output = A::make(0.875);
        const auto held = A::make(A::fixed ? -0.75 : -6.0);
        ASSERT_EQ(A::bpf_init_band(&context, order, 40, 90, kRate, held), LINEARS_OK);
        if (order == FILTERS_ORDER_1) {
            const std::decay_t<decltype(context.sections[1])> unused{};
            expect_unchanged(context.sections[1], unused);
        }
        for (int i = 0; i < 32; ++i) {
            ASSERT_EQ(A::bpf_step(&context, held, &output), LINEARS_OK);
            EXPECT_EQ(A::real(output), 0);
        }
        for (int i = 0; i < 8; ++i) ASSERT_EQ(A::bpf_step(&context, A::make(0), &output), LINEARS_OK);
        EXPECT_NE(A::real(output), 0);
        const auto history = context;
        ASSERT_EQ(A::bpf_set_center(&context, 200, 3, kRate), LINEARS_OK);
        ASSERT_EQ(A::bpf_init_center(&fresh, order, 200, 3, kRate, held), LINEARS_OK);
        EXPECT_EQ(context.order, order);
        for (int k = 0; k < 2; ++k) {
            expect_same_coefficients<A>(context.sections[k], fresh.sections[k]);
            expect_same_history(context.sections[k], history.sections[k]);
        }
        ASSERT_EQ(A::bpf_set_band(&context, 40, 90, kRate), LINEARS_OK);
        for (int k = 0; k < 2; ++k) {
            expect_same_coefficients<A>(context.sections[k], history.sections[k]);
            expect_same_history(context.sections[k], history.sections[k]);
        }
        ASSERT_EQ(A::bpf_reset(&context, held), LINEARS_OK);
        ASSERT_EQ(A::bpf_step(&context, held, &output), LINEARS_OK);
        EXPECT_EQ(A::real(output), 0);
    }
}

TYPED_TEST(BpfTest, InvalidArgumentsAliasAndTransactionalFailure) {
    using A = TypeParam;
    using Hz = typename A::Hz;
    typename A::Bpf context;
    auto output = A::make(0.875);
    ASSERT_EQ(A::bpf_init_band(&context, FILTERS_ORDER_2, 80, 160, kRate, A::make(0.125)), LINEARS_OK);
    const auto before = context;
    const Hz nan = std::numeric_limits<Hz>::quiet_NaN();
    EXPECT_EQ(A::bpf_init_band(nullptr, FILTERS_ORDER_1, 80, 160, kRate, A::make(0)), LINEARS_ERROR_ARGUMENT);
    EXPECT_EQ(A::bpf_init_center(nullptr, FILTERS_ORDER_1, 80, 2, kRate, A::make(0)), LINEARS_ERROR_ARGUMENT);
    EXPECT_EQ(A::bpf_init_band(&context, static_cast<filters_order_t>(3), 80, 160, kRate, A::make(0)),
              LINEARS_ERROR_DOMAIN);
    EXPECT_EQ(A::bpf_init_center(&context, static_cast<filters_order_t>(3), 80, 2, kRate, A::make(0)),
              LINEARS_ERROR_DOMAIN);
    EXPECT_EQ(A::bpf_init_band(&context, FILTERS_ORDER_1, 160, 80, kRate, A::make(0)), LINEARS_ERROR_DOMAIN);
    EXPECT_EQ(A::bpf_init_band(&context, FILTERS_ORDER_1, 80, 80, kRate, A::make(0)), LINEARS_ERROR_DOMAIN);
    EXPECT_EQ(A::bpf_init_band(&context, FILTERS_ORDER_1, 0, 80, kRate, A::make(0)), LINEARS_ERROR_DOMAIN);
    EXPECT_EQ(A::bpf_init_band(&context, FILTERS_ORDER_1, 80, Hz(kRate / 2), kRate, A::make(0)),
              LINEARS_ERROR_DOMAIN);
    EXPECT_EQ(A::bpf_init_band(&context, FILTERS_ORDER_1, nan, 80, kRate, A::make(0)), LINEARS_ERROR_RANGE);
    EXPECT_EQ(A::bpf_init_center(&context, FILTERS_ORDER_1, 80, 0, kRate, A::make(0)), LINEARS_ERROR_DOMAIN);
    EXPECT_EQ(A::bpf_init_center(&context, FILTERS_ORDER_1, 80, -2, kRate, A::make(0)), LINEARS_ERROR_DOMAIN);
    EXPECT_EQ(A::bpf_init_center(&context, FILTERS_ORDER_1, 80, nan, kRate, A::make(0)), LINEARS_ERROR_RANGE);
    EXPECT_EQ(A::bpf_init_center(&context, FILTERS_ORDER_1, Hz(kRate), 2, kRate, A::make(0)), LINEARS_ERROR_DOMAIN);
    EXPECT_EQ(A::bpf_set_band(&context, 160, 80, kRate), LINEARS_ERROR_DOMAIN);
    EXPECT_EQ(A::bpf_set_center(&context, 80, 0, kRate), LINEARS_ERROR_DOMAIN);
    EXPECT_EQ(A::bpf_set_center(&context, 80, 2, nan), LINEARS_ERROR_RANGE);
    EXPECT_EQ(A::bpf_set_band(nullptr, 80, 160, kRate), LINEARS_ERROR_ARGUMENT);
    EXPECT_EQ(A::bpf_set_center(nullptr, 80, 2, kRate), LINEARS_ERROR_ARGUMENT);
    EXPECT_EQ(A::bpf_reset(nullptr, A::make(0)), LINEARS_ERROR_ARGUMENT);
    if constexpr (!A::fixed) {
        // B = w0/q overflows; the designed section coefficients are not finite.
        const Hz tiny = std::numeric_limits<Hz>::denorm_min();
        EXPECT_EQ(A::bpf_init_center(&context, FILTERS_ORDER_2, 80, tiny, kRate, A::make(0)), LINEARS_ERROR_RANGE);
        EXPECT_EQ(A::bpf_set_center(&context, 80, tiny, kRate), LINEARS_ERROR_RANGE);
        EXPECT_EQ(A::bpf_init_band(&context, FILTERS_ORDER_1, 80, 160, kRate, A::invalid()), LINEARS_ERROR_RANGE);
        EXPECT_EQ(A::bpf_step(&context, A::invalid(), &output), LINEARS_ERROR_RANGE);
    }
    EXPECT_EQ(A::bpf_step(&context, A::make(0), reinterpret_cast<typename A::Scalar *>(&context.sections[1].y2)),
              LINEARS_ERROR_ALIAS);
    EXPECT_EQ(A::bpf_step(&context, A::make(0), nullptr), LINEARS_ERROR_ARGUMENT);
    expect_unchanged(context, before);
    EXPECT_EQ(A::real(output), 0.875);
    if constexpr (A::fixed) {
        // A wide band has b0 = B/a0 above one half: a full-scale step overflows.
        ASSERT_EQ(A::bpf_init_band(&context, FILTERS_ORDER_1, 50, 450, kRate,
                                   std::numeric_limits<typename A::Scalar>::min()), LINEARS_OK);
        const auto settled = context;
        EXPECT_EQ(A::bpf_step(&context, std::numeric_limits<typename A::Scalar>::max(), &output),
                  LINEARS_ERROR_RANGE);
        expect_unchanged(context, settled);
        EXPECT_EQ(A::real(output), 0.875);
    }
}
