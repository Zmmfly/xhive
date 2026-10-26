#include "filter_test_support.h"

using namespace filters_test;
template <typename A> class HpfTest : public ::testing::Test {};
TYPED_TEST_SUITE(HpfTest, Formats);

namespace {

/* One second at this rate makes every whole-hertz test tone an exact DFT bin. */
constexpr int kRate = 1024;
constexpr filters_order_t kOrders[] = {FILTERS_ORDER_1, FILTERS_ORDER_2};

/* Bilinear map: H(e^jw) equals the analog prototype at s = j*tan(w/2). */
std::complex<double> highpass_oracle(filters_order_t order, double cutoff, double frequency) {
    const double pi = std::acos(-1.0);
    const double wc = std::tan(pi * cutoff / kRate);
    const std::complex<double> s(0, std::tan(pi * frequency / kRate));
    if (order == FILTERS_ORDER_1) return s / (s + wc);
    return s * s / (s * s + std::sqrt(2.0) * wc * s + wc * wc);
}

} // namespace

TYPED_TEST(HpfTest, ResponseMatchesBilinearButterworthAndCutoffIsMinus3Db) {
    using A = TypeParam;
    for (auto order : kOrders) {
        for (int bin : {10, 50, 100, 200, 400}) {
            typename A::Hpf context;
            ASSERT_EQ(A::hpf_init(&context, order, 100, kRate, A::make(0)), LINEARS_OK);
            const auto gain = measure_gain<A>([&](auto input, auto *output) {
                return A::hpf_step(&context, input, output);
            }, bin, kRate, 0.5, 2 * kRate);
            // Same bound as the low-pass response test: coefficient and recurrence
            // rounding in well-damped sections plus Q output rounding.
            EXPECT_LT(std::abs(gain - highpass_oracle(order, 100, bin)), 64 * A::unit)
                << "order " << order << " bin " << bin;
            if (bin == 100) EXPECT_NEAR(std::abs(gain), std::sqrt(0.5), 64 * A::unit);
        }
    }
}

TYPED_TEST(HpfTest, HeldOffsetGivesExactZeroAcrossInitRetuneAndReset) {
    using A = TypeParam;
    for (auto order : kOrders) {
        typename A::Hpf context, fresh;
        typename A::Scalar output = A::make(0.875);
        const auto held = A::make(A::fixed ? 0.625 : 5.0);
        // The numerator taps cancel exactly and the settled output history is zero.
        ASSERT_EQ(A::hpf_init(&context, order, 20, kRate, held), LINEARS_OK);
        for (int i = 0; i < 32; ++i) {
            ASSERT_EQ(A::hpf_step(&context, held, &output), LINEARS_OK);
            EXPECT_EQ(A::real(output), 0);
        }
        ASSERT_EQ(A::hpf_step(&context, A::make(0), &output), LINEARS_OK);
        EXPECT_LT(A::real(output), 0);
        const auto history = context.section;
        ASSERT_EQ(A::hpf_set(&context, 300, kRate), LINEARS_OK);
        ASSERT_EQ(A::hpf_init(&fresh, order, 300, kRate, held), LINEARS_OK);
        EXPECT_EQ(context.section.b0, fresh.section.b0);
        EXPECT_EQ(context.section.b1, fresh.section.b1);
        EXPECT_EQ(context.section.b2, fresh.section.b2);
        EXPECT_EQ(context.section.a1, fresh.section.a1);
        EXPECT_EQ(context.section.a2, fresh.section.a2);
        EXPECT_EQ(context.section.x1, history.x1);
        EXPECT_EQ(context.section.y1, history.y1);
        EXPECT_EQ(context.section.x2, history.x2);
        EXPECT_EQ(context.section.y2, history.y2);
        ASSERT_EQ(A::hpf_reset(&context, held), LINEARS_OK);
        ASSERT_EQ(A::hpf_step(&context, held, &output), LINEARS_OK);
        EXPECT_EQ(A::real(output), 0);
    }
}

TYPED_TEST(HpfTest, InvalidArgumentsAliasAndTransactionalFailure) {
    using A = TypeParam;
    using Hz = typename A::Hz;
    typename A::Hpf context;
    auto output = A::make(0.875);
    ASSERT_EQ(A::hpf_init(&context, FILTERS_ORDER_1, 100, kRate, A::make(0.125)), LINEARS_OK);
    const auto before = context;
    const Hz nan = std::numeric_limits<Hz>::quiet_NaN();
    EXPECT_EQ(A::hpf_init(nullptr, FILTERS_ORDER_1, 100, kRate, A::make(0)), LINEARS_ERROR_ARGUMENT);
    EXPECT_EQ(A::hpf_init(&context, static_cast<filters_order_t>(0), 100, kRate, A::make(0)), LINEARS_ERROR_DOMAIN);
    EXPECT_EQ(A::hpf_init(&context, FILTERS_ORDER_2, Hz(kRate / 2), kRate, A::make(0)), LINEARS_ERROR_DOMAIN);
    EXPECT_EQ(A::hpf_init(&context, FILTERS_ORDER_2, 100, Hz(0), A::make(0)), LINEARS_ERROR_DOMAIN);
    EXPECT_EQ(A::hpf_init(&context, FILTERS_ORDER_2, 100, nan, A::make(0)), LINEARS_ERROR_RANGE);
    EXPECT_EQ(A::hpf_set(&context, Hz(0), kRate), LINEARS_ERROR_DOMAIN);
    EXPECT_EQ(A::hpf_set(&context, 100, nan), LINEARS_ERROR_RANGE);
    EXPECT_EQ(A::hpf_set(nullptr, 100, kRate), LINEARS_ERROR_ARGUMENT);
    EXPECT_EQ(A::hpf_reset(nullptr, A::make(0)), LINEARS_ERROR_ARGUMENT);
    if constexpr (!A::fixed) {
        EXPECT_EQ(A::hpf_reset(&context, A::invalid()), LINEARS_ERROR_RANGE);
        EXPECT_EQ(A::hpf_step(&context, std::numeric_limits<typename A::Scalar>::infinity(), &output),
                  LINEARS_ERROR_RANGE);
    }
    EXPECT_EQ(A::hpf_step(&context, A::make(0), reinterpret_cast<typename A::Scalar *>(&context.order)),
              LINEARS_ERROR_ALIAS);
    EXPECT_EQ(A::hpf_step(nullptr, A::make(0), &output), LINEARS_ERROR_ARGUMENT);
    expect_unchanged(context, before);
    EXPECT_EQ(A::real(output), 0.875);
    if constexpr (A::fixed) {
        // A full-scale step makes y = 2*b0 > 1 on the first sample.
        ASSERT_EQ(A::hpf_init(&context, FILTERS_ORDER_1, 100, kRate,
                              std::numeric_limits<typename A::Scalar>::min()), LINEARS_OK);
        const auto settled = context;
        EXPECT_EQ(A::hpf_step(&context, std::numeric_limits<typename A::Scalar>::max(), &output),
                  LINEARS_ERROR_RANGE);
        expect_unchanged(context, settled);
        EXPECT_EQ(A::real(output), 0.875);
    }
}
