#include "filter_test_support.h"

using namespace filters_test;
template <typename A> class LpfTest : public ::testing::Test {};
TYPED_TEST_SUITE(LpfTest, Formats);

namespace {

/* One second at this rate makes every whole-hertz test tone an exact DFT bin. */
constexpr int kRate = 1024;
constexpr filters_order_t kOrders[] = {FILTERS_ORDER_1, FILTERS_ORDER_2};

/* The bilinear map gives H(e^jw) = H(s) at s = j*tan(w/2) exactly, so the
 * analog Butterworth prototype is an independent oracle for the response. */
std::complex<double> lowpass_oracle(filters_order_t order, double cutoff, double frequency) {
    const double pi = std::acos(-1.0);
    const double wc = std::tan(pi * cutoff / kRate);
    const std::complex<double> s(0, std::tan(pi * frequency / kRate));
    if (order == FILTERS_ORDER_1) return wc / (s + wc);
    return wc * wc / (s * s + std::sqrt(2.0) * wc * s + wc * wc);
}

} // namespace

TYPED_TEST(LpfTest, ResponseMatchesBilinearButterworthAndCutoffIsMinus3Db) {
    using A = TypeParam;
    for (auto order : kOrders) {
        for (int bin : {10, 50, 100, 200, 400}) {
            typename A::Lpf context;
            ASSERT_EQ(A::lpf_init(&context, order, 100, kRate, A::make(0)), LINEARS_OK);
            const auto gain = measure_gain<A>([&](auto input, auto *output) {
                return A::lpf_step(&context, input, output);
            }, bin, kRate, 0.5, 2 * kRate);
            // 64 storage units cover coefficient rounding, recurrence rounding in
            // these well-damped sections, and Q output rounding (2 LSB/amplitude).
            EXPECT_LT(std::abs(gain - lowpass_oracle(order, 100, bin)), 64 * A::unit)
                << "order " << order << " bin " << bin;
            if (bin == 100) EXPECT_NEAR(std::abs(gain), std::sqrt(0.5), 64 * A::unit);
        }
    }
}

TYPED_TEST(LpfTest, HeldInputSurvivesInitRetuneAndReset) {
    using A = TypeParam;
    for (auto order : kOrders) {
        typename A::Lpf context, fresh;
        typename A::Scalar output{};
        const auto held = A::make(A::fixed ? -0.375 : -3.0);
        ASSERT_EQ(A::lpf_init(&context, order, 37, kRate, held), LINEARS_OK);
        auto check_held = [&](typename A::Scalar level) {
            if constexpr (A::fixed) {
                // Integer numerator taps sum to 1 + a1 + a2: the DC gain is exact.
                EXPECT_EQ(output, level);
            } else {
                const auto &s = context.section;
                const double terms = std::abs(A::real_coefficient(s.b0)) + std::abs(A::real_coefficient(s.b1)) +
                    std::abs(A::real_coefficient(s.b2)) + std::abs(A::real_coefficient(s.a1)) +
                    std::abs(A::real_coefficient(s.a2));
                // Five roundings per step of terms bounded by |level|*sum|coefficients|,
                // accumulated through the all-pole noise gain.
                EXPECT_NEAR(A::real(output), A::real(level),
                    5 * A::unit * terms * std::abs(A::real(level)) * feedback_noise_gain<A>(s));
            }
        };
        for (int i = 0; i < 64; ++i) {
            ASSERT_EQ(A::lpf_step(&context, held, &output), LINEARS_OK);
            check_held(held);
        }
        const auto history = context.section;
        ASSERT_EQ(A::lpf_set(&context, 211, 2 * kRate), LINEARS_OK);
        ASSERT_EQ(A::lpf_init(&fresh, order, 211, 2 * kRate, held), LINEARS_OK);
        EXPECT_EQ(context.order, order);
        EXPECT_EQ(context.section.b0, fresh.section.b0);
        EXPECT_EQ(context.section.b1, fresh.section.b1);
        EXPECT_EQ(context.section.b2, fresh.section.b2);
        EXPECT_EQ(context.section.a1, fresh.section.a1);
        EXPECT_EQ(context.section.a2, fresh.section.a2);
        EXPECT_EQ(context.section.x1, history.x1);
        EXPECT_EQ(context.section.x2, history.x2);
        EXPECT_EQ(context.section.y1, history.y1);
        EXPECT_EQ(context.section.y2, history.y2);
        ASSERT_EQ(A::lpf_step(&context, held, &output), LINEARS_OK);
        check_held(held);
        const auto level = A::make(A::fixed ? 0.25 : 2.0);
        ASSERT_EQ(A::lpf_reset(&context, level), LINEARS_OK);
        for (int i = 0; i < 16; ++i) {
            ASSERT_EQ(A::lpf_step(&context, level, &output), LINEARS_OK);
            check_held(level);
        }
    }
}

TYPED_TEST(LpfTest, LowCutoffQHistoryKeepsSubLsbProgress) {
    using A = TypeParam;
    if constexpr (A::fixed) {
        for (auto order : kOrders) {
            typename A::Lpf context;
            typename A::Scalar output{};
            ASSERT_EQ(A::lpf_init(&context, order, 1, kRate, A::make(0)), LINEARS_OK);
            for (int i = 0; i < 40 * kRate; ++i) ASSERT_EQ(A::lpf_step(&context, A::make(0.25), &output), LINEARS_OK);
            // Rounding the history stalls once (1+a1+a2)*|x-y| is below half a
            // history LSB. With F31 history that gap is far below one Q15 LSB;
            // a Q15-precision history would stall thousands of Q15 LSBs short.
            const auto &s = context.section;
            const double dc_sum = A::real_coefficient(LINEARS_FIXED_ONE + s.a1 + s.a2);
            EXPECT_NEAR(A::real(output), 0.25, 0.5 * 0x1p-31 / dc_sum + 0.5 * A::unit) << "order " << order;
            if constexpr (A::bits == 15) EXPECT_EQ(output, A::make(0.25));
        }
    }
}

TYPED_TEST(LpfTest, InvalidArgumentsAliasAndTransactionalFailure) {
    using A = TypeParam;
    using Hz = typename A::Hz;
    typename A::Lpf context;
    auto output = A::make(0.875);
    ASSERT_EQ(A::lpf_init(&context, FILTERS_ORDER_2, 100, kRate, A::make(0.125)), LINEARS_OK);
    const auto before = context;
    const Hz nan = std::numeric_limits<Hz>::quiet_NaN(), inf = std::numeric_limits<Hz>::infinity();
    EXPECT_EQ(A::lpf_init(nullptr, FILTERS_ORDER_1, 100, kRate, A::make(0)), LINEARS_ERROR_ARGUMENT);
    EXPECT_EQ(A::lpf_init(&context, static_cast<filters_order_t>(3), 100, kRate, A::make(0)), LINEARS_ERROR_DOMAIN);
    for (Hz cutoff : {Hz(0), Hz(-1), Hz(kRate / 2), Hz(kRate)})
        EXPECT_EQ(A::lpf_init(&context, FILTERS_ORDER_1, cutoff, kRate, A::make(0)), LINEARS_ERROR_DOMAIN);
    for (Hz rate : {Hz(0), Hz(-kRate)})
        EXPECT_EQ(A::lpf_init(&context, FILTERS_ORDER_1, 100, rate, A::make(0)), LINEARS_ERROR_DOMAIN);
    EXPECT_EQ(A::lpf_init(&context, FILTERS_ORDER_1, nan, kRate, A::make(0)), LINEARS_ERROR_RANGE);
    EXPECT_EQ(A::lpf_init(&context, FILTERS_ORDER_1, 100, inf, A::make(0)), LINEARS_ERROR_RANGE);
    EXPECT_EQ(A::lpf_set(&context, Hz(kRate / 2), kRate), LINEARS_ERROR_DOMAIN);
    EXPECT_EQ(A::lpf_set(&context, nan, kRate), LINEARS_ERROR_RANGE);
    EXPECT_EQ(A::lpf_set(nullptr, 100, kRate), LINEARS_ERROR_ARGUMENT);
    EXPECT_EQ(A::lpf_reset(nullptr, A::make(0)), LINEARS_ERROR_ARGUMENT);
    if constexpr (!A::fixed) {
        EXPECT_EQ(A::lpf_init(&context, FILTERS_ORDER_1, 100, kRate, A::invalid()), LINEARS_ERROR_RANGE);
        EXPECT_EQ(A::lpf_reset(&context, A::invalid()), LINEARS_ERROR_RANGE);
        EXPECT_EQ(A::lpf_step(&context, A::invalid(), &output), LINEARS_ERROR_RANGE);
    }
    EXPECT_EQ(A::lpf_step(&context, A::make(0), reinterpret_cast<typename A::Scalar *>(&context.section.y1)),
              LINEARS_ERROR_ALIAS);
    EXPECT_EQ(A::lpf_step(nullptr, A::make(0), &output), LINEARS_ERROR_ARGUMENT);
    EXPECT_EQ(A::lpf_step(&context, A::make(0), nullptr), LINEARS_ERROR_ARGUMENT);
    expect_unchanged(context, before);
    EXPECT_EQ(A::real(output), 0.875);
}

TYPED_TEST(LpfTest, QOvershootReturnsRangeWithoutCommitting) {
    using A = TypeParam;
    if constexpr (A::fixed) {
        typename A::Lpf context;
        const auto low = std::numeric_limits<typename A::Scalar>::min();
        const auto high = std::numeric_limits<typename A::Scalar>::max();
        ASSERT_EQ(A::lpf_init(&context, FILTERS_ORDER_2, 100, kRate, low), LINEARS_OK);
        // A full-scale step overshoots +1 by about 2*4.3% in order two.
        linears_status_t status = LINEARS_OK;
        auto output = A::make(0.875);
        auto before = context;
        auto previous = output;
        for (int i = 0; i < 64 && status == LINEARS_OK; ++i) {
            before = context;
            previous = output;
            status = A::lpf_step(&context, high, &output);
        }
        EXPECT_EQ(status, LINEARS_ERROR_RANGE);
        expect_unchanged(context, before);
        EXPECT_EQ(output, previous);
    }
}
