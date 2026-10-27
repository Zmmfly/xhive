#include "filters_pf.h"
#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cstdint>
#include <limits>
#include <vector>

namespace {
struct Q15 {
    using Raw = int16_t;
    using Work = float;
    using State = filters_pf_q15_t;
    using Workspace = filters_ukf_q15_workspace_t;
    static constexpr int bits = 15;
    static constexpr auto init = filters_pf_q15_init;
    static constexpr auto estimate = filters_pf_q15_estimate;
};
struct Q31 {
    using Raw = int32_t;
    using Work = double;
    using State = filters_pf_q31_t;
    using Workspace = filters_ukf_q31_workspace_t;
    static constexpr int bits = 31;
    static constexpr auto init = filters_pf_q31_init;
    static constexpr auto estimate = filters_pf_q31_estimate;
};

template<class F> class QMoments : public ::testing::Test {};
using Formats = ::testing::Types<Q15, Q31>;
TYPED_TEST_SUITE(QMoments, Formats);

TYPED_TEST(QMoments, LargeCorrelatedCloudKeepsExactSignedCrossMoments) {
    using F = TypeParam;
    using R = typename F::Raw;
    using W = typename F::Work;
    constexpr std::size_t n = 4, count = 65537;
    constexpr R half = static_cast<R>(INT64_C(1) << (F::bits - 1));
    constexpr R quarter = static_cast<R>(INT64_C(1) << (F::bits - 2));
    std::vector<R> particles(count*n), weights(count), q(n*n), r(n*n);
    std::array<R, n> mean0{0, 0, 0, quarter}, mean{};
    std::array<R, n*n> prior{}, covariance{};
    for (std::size_t j = 0; j < n; ++j) prior[j*n+j] = quarter;
    const linears_matrix_t mean_in{mean0.data(), n, 1}, noise{prior.data(), n, n};
    typename F::State state{{particles.data(), count, n}, {weights.data(), count, 1},
                            {q.data(), n, n}, {r.data(), n, n}, count*n, count, n*n, n*n};
    const auto data_count = std::max(FILTERS_PF_Q_INIT_STORAGE(count, n), FILTERS_PF_Q_ESTIMATE_STORAGE(count, n));
    const auto signal_count = std::max(FILTERS_PF_Q_INIT_SIGNAL_STORAGE(count, n), FILTERS_PF_Q_ESTIMATE_SIGNAL_STORAGE(count, n));
    const auto basic_bytes = FILTERS_PF_Q_ESTIMATE_BASIC_BYTES(count, n);
    std::vector<W> data(data_count + 1), decomposition(FILTERS_PF_PREDICT_DECOMP_STORAGE(n));
    std::vector<R> signal(signal_count + 1);
    std::vector<linears_workspace_unit_t> basic(LINEARS_WORKSPACE_UNITS(basic_bytes) + 1);
    typename F::Workspace workspace{};
    workspace.data = data.data(); workspace.capacity = data_count;
    workspace.signal = signal.data(); workspace.signal_capacity = signal_count;
    workspace.basic = basic.data(); workspace.basic_bytes = basic_bytes;
    workspace.decomposition = {decomposition.data(), decomposition.size()};
    auto zero_draw = [](linears_matrix_t *out, void *) -> linears_status_t {
        std::fill_n(static_cast<W *>(out->data), out->rows * out->cols, W(0));
        return LINEARS_OK;
    };
    ASSERT_EQ(F::init(&state, &mean_in, &noise, &noise, &noise, zero_draw, nullptr, workspace), LINEARS_OK);
    for (std::size_t i = 0; i + 1 < count; ++i) {
        R value = i % 2 ? static_cast<R>(-half) : half;
        particles[i*n] = particles[i*n+1] = value;
        particles[i*n+2] = static_cast<R>(-value);
        particles[i*n+3] = quarter;
    }
    // The extra full-scale row has zero weight, but still exercises products.
    std::fill_n(particles.data() + (count-1)*n, n, std::numeric_limits<R>::min());
    weights.back() = 0;
    const auto original_particles = particles, original_weights = weights;
    workspace.capacity = FILTERS_PF_Q_ESTIMATE_STORAGE(count, n);
    workspace.signal_capacity = FILTERS_PF_Q_ESTIMATE_SIGNAL_STORAGE(count, n);
    workspace.decomposition.capacity = 0;
    data[workspace.capacity] = W(9.5);
    signal[workspace.signal_capacity] = R(1234);
    basic.back().integer = INT64_C(0x123456789);
    linears_matrix_t mean_out{mean.data(), n, 1}, covariance_out{covariance.data(), n, n};
    ASSERT_EQ(F::estimate(&state, nullptr, nullptr, &mean_out, &covariance_out, workspace), LINEARS_OK);
    EXPECT_EQ(mean, mean0);
    const std::array<R, n*n> expected{
        quarter, quarter, static_cast<R>(-quarter), 0,
        quarter, quarter, static_cast<R>(-quarter), 0,
        static_cast<R>(-quarter), static_cast<R>(-quarter), quarter, 0,
        0, 0, 0, 0};
    EXPECT_EQ(covariance, expected);
    EXPECT_EQ(particles, original_particles);
    EXPECT_EQ(weights, original_weights);
    EXPECT_EQ(data[workspace.capacity], W(9.5));
    EXPECT_EQ(signal[workspace.signal_capacity], R(1234));
    EXPECT_EQ(basic.back().integer, INT64_C(0x123456789));
}
} // namespace
