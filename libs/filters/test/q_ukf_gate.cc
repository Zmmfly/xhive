/* Dedicated Q UKF kernels against the floating kernels on the same working
 * inputs: raw-product gate boundaries (integer budget, exact bound,
 * saturated cells, sub-LSB product scale), adversarial sigma weights and the
 * reciprocal-pivot gain solve. Covariance entries may differ from the
 * floating products by one signal LSB and means by half an LSB (UKF.md),
 * plus the floating rounding of either kernel. */
#include "filters_ukf.h"

#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <functional>
#include <limits>
#include <vector>

namespace {

template<class W> struct Api;
template<> struct Api<float> {
    using Raw = int16_t;
    using Weights = filters_ukf_f32_weights_t;
    using FixedWorkspace = filters_ukf_q15_workspace_t;
    using FloatWorkspace = filters_ukf_f32_workspace_t;
    static constexpr int bits = 15;
    static constexpr auto weights = filters_ukf_f32_weights;
    static constexpr auto fixed_predict = filters_ukf_q15_predict_work;
    static constexpr auto fixed_update = filters_ukf_q15_update_work;
    static constexpr auto float_predict = filters_ukf_f32_predict_work;
    static constexpr auto float_update = filters_ukf_f32_update_work;
};
template<> struct Api<double> {
    using Raw = int32_t;
    using Weights = filters_ukf_f64_weights_t;
    using FixedWorkspace = filters_ukf_q31_workspace_t;
    using FloatWorkspace = filters_ukf_f64_workspace_t;
    static constexpr int bits = 31;
    static constexpr auto weights = filters_ukf_f64_weights;
    static constexpr auto fixed_predict = filters_ukf_q31_predict_work;
    static constexpr auto fixed_update = filters_ukf_q31_update_work;
    static constexpr auto float_predict = filters_ukf_f64_predict_work;
    static constexpr auto float_update = filters_ukf_f64_update_work;
};

template<class W> using Map = std::function<void(const W *x, std::size_t n, W *y, std::size_t m, std::size_t call)>;
template<class W> struct Callback {
    Map<W> map;
    std::size_t calls = 0;
};
template<class W> linears_status_t invoke(const linears_matrix_t *in, linears_matrix_t *out, void *context) {
    auto &callback = *static_cast<Callback<W> *>(context);
    callback.map(static_cast<const W *>(in->data), in->rows, static_cast<W *>(out->data), out->rows,
                 callback.calls++);
    return LINEARS_OK;
}

template<class W> struct Result {
    linears_status_t status = LINEARS_ERROR_ARGUMENT;
    std::vector<W> mean, covariance;
    std::size_t calls = 0;
};

template<class W> struct Matrix {
    std::vector<W> values;
    linears_matrix_t view;
    Matrix(std::vector<W> data, std::size_t rows, std::size_t cols)
        : values(std::move(data)), view{values.data(), rows, cols} {}
};

/* One kernel run with exactly sized scratch, fixed (Q) or floating. */
template<class W> Result<W> run(bool fixed, const std::vector<W> &x, const std::vector<W> &p,
                                const std::vector<W> &noise, const std::vector<W> *z,
                                const typename Api<W>::Weights &weights, const Map<W> &map) {
    using A = Api<W>;
    const std::size_t n = x.size(), m = z ? z->size() : n;
    Matrix<W> xv(x, n, 1), pv(p, n, n), nv(noise, m, m), zv(z ? *z : std::vector<W>(1), m, 1);
    Result<W> result;
    result.mean.assign(n, W(99));
    result.covariance.assign(n * n, W(99));
    linears_matrix_t mean{result.mean.data(), n, 1}, covariance{result.covariance.data(), n, n};
    Callback<W> callback{map};
    const std::size_t data = fixed ? (z ? FILTERS_UKF_Q_UPDATE_WORK_STORAGE(n, m) : FILTERS_UKF_Q_PREDICT_WORK_STORAGE(n))
                                   : (z ? FILTERS_UKF_UPDATE_WORK_STORAGE(n, m) : FILTERS_UKF_PREDICT_WORK_STORAGE(n));
    const std::size_t decomp = fixed ? (z ? FILTERS_UKF_Q_UPDATE_DECOMP_STORAGE(n, m) : FILTERS_UKF_Q_PREDICT_DECOMP_STORAGE(n))
                                     : (z ? FILTERS_UKF_UPDATE_DECOMP_STORAGE(n, m) : FILTERS_UKF_PREDICT_DECOMP_STORAGE(n));
    const std::size_t basic = fixed ? 0 : (z ? FILTERS_UKF_UPDATE_BASIC_BYTES(n, m) : FILTERS_UKF_PREDICT_BASIC_BYTES(n));
    std::vector<W> data_array(data), decomp_array(decomp);
    std::vector<linears_workspace_unit_t> basic_array(LINEARS_WORKSPACE_UNITS(basic));
    if (fixed) {
        const std::size_t signal = z ? FILTERS_UKF_Q_UPDATE_SIGNAL_STORAGE(n, m) : FILTERS_UKF_Q_PREDICT_SIGNAL_STORAGE(n);
        std::vector<typename A::Raw> signal_array(signal);
        typename A::FixedWorkspace workspace{};
        workspace.data = data_array.data();
        workspace.capacity = data;
        workspace.decomposition = {decomp_array.data(), decomp};
        workspace.signal = signal_array.data();
        workspace.signal_capacity = signal;
        result.status = z ? A::fixed_update(&xv.view, &pv.view, &nv.view, &zv.view, &weights, invoke<W>, &callback,
                                            nullptr, nullptr, &mean, &covariance, workspace)
                          : A::fixed_predict(&xv.view, &pv.view, &nv.view, &weights, invoke<W>, &callback,
                                             nullptr, nullptr, &mean, &covariance, workspace);
    } else {
        typename A::FloatWorkspace workspace{data_array.data(), data, basic ? basic_array.data() : nullptr, basic,
                                             {decomp_array.data(), decomp}};
        result.status = z ? A::float_update(&xv.view, &pv.view, &nv.view, &zv.view, &weights, invoke<W>, &callback,
                                            nullptr, nullptr, &mean, &covariance, workspace)
                          : A::float_predict(&xv.view, &pv.view, &nv.view, &weights, invoke<W>, &callback,
                                             nullptr, nullptr, &mean, &covariance, workspace);
    }
    result.calls = callback.calls;
    return result;
}

template<class W> long double lsb() { return std::ldexp(1.0L, -Api<W>::bits); }

/* |fixed - floating| <= lsbs signal LSBs plus 16 working ulps of the larger
 * term magnitude, the rounding allowance of the two floating evaluations. */
template<class W> void expect_within(const std::vector<W> &fixed, const std::vector<W> &floating,
                                     long double lsbs, long double terms) {
    ASSERT_EQ(fixed.size(), floating.size());
    const long double ulp = std::numeric_limits<W>::epsilon();
    for (std::size_t i = 0; i < fixed.size(); ++i) {
        SCOPED_TRACE(i);
        const long double reference = floating[i];
        const long double limit = lsbs * lsb<W>() + 16 * ulp * (terms + std::fabs(reference));
        EXPECT_LE(std::fabs(static_cast<long double>(fixed[i]) - reference), limit);
    }
}

template<class W> typename Api<W>::Weights make_weights(std::size_t n, W alpha, W beta, W kappa) {
    typename Api<W>::Weights weights{};
    EXPECT_EQ(Api<W>::weights(n, alpha, beta, kappa, &weights), LINEARS_OK);
    return weights;
}
template<class W> typename Api<W>::Weights custom(W scale, W mean0, W covariance0, W other) {
    return {scale, mean0, covariance0, other};
}

template<class W> class QUkfGate : public ::testing::Test {};
using WorkTypes = ::testing::Types<float, double>;
TYPED_TEST_SUITE(QUkfGate, WorkTypes);

/* Deviation magnitudes from 0.002 to about 1.3 cross the integer screen,
 * the exact one-LSB bound and the signal interval in both kernels. */
TYPED_TEST(QUkfGate, RawProductsStayWithinOneLsbAcrossTheScreenBudget) {
    using W = TypeParam;
    const Map<W> transition = [](const W *x, std::size_t n, W *y, std::size_t, std::size_t) {
        for (std::size_t j = 0; j < n; ++j)
            y[j] = W(0.875) * x[j] + W(0.125) * x[(j + 1) % n] + W(0.25) * x[j] * x[j];
    };
    const Map<W> observation = [](const W *x, std::size_t n, W *y, std::size_t m, std::size_t) {
        for (std::size_t j = 0; j < m; ++j)
            y[j] = W(0.75) * x[j % n] - W(0.25) * x[(j + 1) % n] + W(0.125) * x[j % n] * x[j % n];
    };
    for (std::size_t n = 1; n <= 4; ++n) {
        for (W alpha : {W(1), W(0.5)}) {
            const auto weights = make_weights<W>(n, alpha, W(2), W(0));
            for (W scale : {W(1e-6), W(1e-3), W(0.01), W(0.04), W(0.09), W(0.16), W(0.25), W(0.36)}) {
                SCOPED_TRACE(testing::Message() << "n=" << n << " alpha=" << alpha << " scale=" << scale);
                std::vector<W> x(n), p(n * n), q(n * n), r(n * n), z(n);
                for (std::size_t i = 0; i < n; ++i) {
                    x[i] = W(0.1) * W(i + 1) - W(0.2);
                    z[i] = W(0.05) * W(i);
                    q[i * n + i] = W(1e-3);
                    r[i * n + i] = W(0.02);
                    for (std::size_t j = 0; j < n; ++j) p[i * n + j] = scale * (i == j ? W(1) : W(0.25));
                }
                const auto fixed = run<W>(true, x, p, q, nullptr, weights, transition);
                const auto floating = run<W>(false, x, p, q, nullptr, weights, transition);
                ASSERT_EQ(fixed.status, floating.status);
                ASSERT_EQ(fixed.status, LINEARS_OK);
                EXPECT_EQ(fixed.calls, floating.calls);
                const long double terms = 4 * scale * (std::fabs(weights.covariance0) + 1) + 1;
                expect_within<W>(fixed.mean, floating.mean, 0.5L, terms);
                expect_within<W>(fixed.covariance, floating.covariance, 1, terms);
                /* The gain amplifies S and C entry errors; keep the existing
                 * per-format workload tolerance for the posterior. */
                const auto fixed_post = run<W>(true, x, p, r, &z, weights, observation);
                const auto floating_post = run<W>(false, x, p, r, &z, weights, observation);
                ASSERT_EQ(fixed_post.status, floating_post.status);
                ASSERT_EQ(fixed_post.status, LINEARS_OK);
                const long double tolerance = Api<W>::bits == 15 ? 4e-3L : 2e-7L;
                for (std::size_t i = 0; i < n; ++i)
                    EXPECT_LE(std::fabs(static_cast<long double>(fixed_post.mean[i]) - floating_post.mean[i]),
                              tolerance * (1 + std::fabs(static_cast<long double>(floating_post.mean[i]))));
                for (std::size_t i = 0; i < n * n; ++i)
                    EXPECT_LE(std::fabs(static_cast<long double>(fixed_post.covariance[i]) - floating_post.covariance[i]),
                              tolerance * (1 + std::fabs(static_cast<long double>(floating_post.covariance[i]))));
            }
        }
    }
}

/* Scripted rows: a deviation of +1 rounds to the positive maximum and its
 * cell carries a whole LSB; a second column sweeps the product of the column
 * maxima through one LSB, the scale below which entries stay floating. */
TYPED_TEST(QUkfGate, SaturatedCellsAndSubLsbProductScalesKeepFloatingPrecision) {
    using W = TypeParam;
    const W q_lsb = W(lsb<W>());
    {
        const auto weights = custom<W>(1, 0, 2, W(0.5));
        const Map<W> rows = [](const W *, std::size_t, W *y, std::size_t, std::size_t call) {
            y[0] = call == 0 ? W(0.25) : call == 1 ? W(1) : W(-1);
        };
        const std::vector<W> x{0}, p{W(0.25)}, q{W(0.125)};
        const auto fixed = run<W>(true, x, p, q, nullptr, weights, rows);
        const auto floating = run<W>(false, x, p, q, nullptr, weights, rows);
        ASSERT_EQ(fixed.status, LINEARS_OK);
        ASSERT_EQ(floating.status, LINEARS_OK);
        expect_within<W>(fixed.mean, floating.mean, 0.5L, 2);
        expect_within<W>(fixed.covariance, floating.covariance, 1, 2);
    }
    {
        /* Both deviations sit 63/128 LSB above a raw value near 0.9 and Wi = 1:
         * raw products would be 1.7 LSB off, so the exact bound must reject
         * this beyond-budget pair and keep floating products. */
        const auto weights = custom<W>(1, -1, 2, 1);
        const W v = (std::round(W(0.9) / q_lsb) + W(63) / W(128)) * q_lsb;
        const Map<W> rows = [v](const W *, std::size_t, W *y, std::size_t, std::size_t call) {
            y[0] = call == 0 ? W(0) : call == 1 ? v : -v;
        };
        const std::vector<W> x{0}, p{W(0.25)}, q{0};
        const auto fixed = run<W>(true, x, p, q, nullptr, weights, rows);
        const auto floating = run<W>(false, x, p, q, nullptr, weights, rows);
        ASSERT_EQ(fixed.status, LINEARS_OK);
        ASSERT_EQ(floating.status, LINEARS_OK);
        EXPECT_EQ(fixed.mean[0], W(0));
        expect_within<W>(fixed.covariance, floating.covariance, 1, 2);
    }
    const auto weights = custom<W>(2, 0, 2, W(0.25));
    for (int k = 1; k <= 40; ++k) {
        SCOPED_TRACE(k);
        const W a = W(0.5), b = W(k) * q_lsb / W(8);
        /* Rows (a,b), (-a,-b), (a,-b), (-a,b) about a zero center: P = diag(a^2, b^2). */
        const Map<W> rows = [a, b](const W *, std::size_t, W *y, std::size_t, std::size_t call) {
            const W signs[5][2] = {{0, 0}, {1, 1}, {-1, -1}, {1, -1}, {-1, 1}};
            y[0] = signs[call % 5][0] * a;
            y[1] = signs[call % 5][1] * b;
        };
        const std::vector<W> x{0, 0}, p{W(0.0625), 0, 0, W(0.0625)}, q{0, 0, 0, 0};
        const auto fixed = run<W>(true, x, p, q, nullptr, weights, rows);
        const auto floating = run<W>(false, x, p, q, nullptr, weights, rows);
        ASSERT_EQ(fixed.status, LINEARS_OK);
        ASSERT_EQ(floating.status, LINEARS_OK);
        expect_within<W>(fixed.covariance, floating.covariance, 1, 1);
        EXPECT_EQ(fixed.covariance[1], W(0));
        EXPECT_EQ(fixed.covariance[2], W(0));
        const long double small = static_cast<long double>(b) * b;
        EXPECT_LE(std::fabs(static_cast<long double>(fixed.covariance[3]) - small) / small, 1e-3L);
    }
}

TYPED_TEST(QUkfGate, AdversarialSigmaWeightsKeepSmallAndUnrelatedVariances) {
    using W = TypeParam;
    const Map<W> identity = [](const W *x, std::size_t n, W *y, std::size_t, std::size_t) {
        for (std::size_t j = 0; j < n; ++j) y[j] = x[j];
    };
    const Map<W> square = [](const W *x, std::size_t, W *y, std::size_t, std::size_t) { y[0] = x[0] * x[0]; };
    /* A tiny valid variance beside 0.01 keeps its relative precision. */
    for (W tiny : {W(1e-24), W(Api<W>::bits == 15 ? 1e-40 : 1e-80)}) {
        SCOPED_TRACE(static_cast<double>(tiny));
        const auto weights = custom<W>(2, 0, 2, W(0.25));
        const std::vector<W> x{0, 0}, p{W(0.01), 0, 0, tiny}, q{0, 0, 0, 0};
        const auto fixed = run<W>(true, x, p, q, nullptr, weights, identity);
        ASSERT_EQ(fixed.status, LINEARS_OK);
        EXPECT_EQ(fixed.calls, 5u);
        EXPECT_LE(std::fabs(static_cast<long double>(fixed.covariance[0]) - 0.01L), lsb<W>());
        EXPECT_GT(fixed.covariance[3], W(0));
        EXPECT_LE(std::fabs(static_cast<long double>(fixed.covariance[3]) - tiny) / tiny, 1e-3L);
        EXPECT_EQ(fixed.covariance[1], W(0));
        EXPECT_EQ(fixed.covariance[2], W(0));
    }
    {
        /* A huge finite Wi with zero rows: the mean is the row and P = Q. */
        const auto weights = custom<W>(1, 0, 0, std::numeric_limits<W>::max() * W(0.75));
        const Map<W> zero = [](const W *, std::size_t, W *y, std::size_t, std::size_t) { y[0] = 0; };
        const auto fixed = run<W>(true, {0}, {W(0.25)}, {W(0.125)}, nullptr, weights, zero);
        ASSERT_EQ(fixed.status, LINEARS_OK);
        EXPECT_EQ(fixed.mean[0], W(0));
        EXPECT_EQ(fixed.covariance[0], W(0.125));
    }
    {
        /* Rows 0, 1, 1 about mean 1: only the center deviation is nonzero. */
        const auto weights = custom<W>(1, 0, W(1e-24), W(0.5));
        const auto fixed = run<W>(true, {0}, {1}, {0}, nullptr, weights, square);
        ASSERT_EQ(fixed.status, LINEARS_OK);
        EXPECT_EQ(fixed.mean[0], W(1));
        EXPECT_LE(std::fabs(static_cast<long double>(fixed.covariance[0]) - 1e-24L) / 1e-24L, 1e-3L);
    }
    {
        /* Wc0 = 2^64 multiplies a nonzero center deviation in column 0 only;
         * column 1 has a zero center deviation and variance 4. */
        const auto weights = custom<W>(2, 0, std::ldexp(W(1), 64), W(0.25));
        const Map<W> rows = [](const W *, std::size_t, W *y, std::size_t, std::size_t call) {
            y[0] = call == 0 ? W(0) : W(1);
            y[1] = call == 0 ? W(0) : (call <= 2 ? W(2) : W(-2));
        };
        const auto fixed = run<W>(true, {0, 0}, {W(0.25), 0, 0, W(0.25)}, {0, 0, 0, 0}, nullptr, weights, rows);
        const auto floating = run<W>(false, {0, 0}, {W(0.25), 0, 0, W(0.25)}, {0, 0, 0, 0}, nullptr, weights, rows);
        ASSERT_EQ(fixed.status, LINEARS_OK);
        ASSERT_EQ(floating.status, LINEARS_OK);
        EXPECT_EQ(fixed.covariance[3], W(4));
        EXPECT_EQ(fixed.covariance[0], floating.covariance[0]);
    }
    {
        /* Signed cancellation: Wc0 = 8 against Wi = -1 (mean weights sum to 1). */
        const auto weights = custom<W>(1, 3, 8, -1);
        const auto fixed = run<W>(true, {W(0.5)}, {W(0.0625)}, {W(0.1)}, nullptr, weights, square);
        const auto floating = run<W>(false, {W(0.5)}, {W(0.0625)}, {W(0.1)}, nullptr, weights, square);
        ASSERT_EQ(fixed.status, LINEARS_OK);
        ASSERT_EQ(floating.status, LINEARS_OK);
        expect_within<W>(fixed.mean, floating.mean, 0.5L, 8);
        expect_within<W>(fixed.covariance, floating.covariance, 1, 8);
    }
    if constexpr (Api<W>::bits == 31) {
        /* alpha=1, beta=.5+2^-40, kappa=-.5: Wc0 = -.5+2^-40, Wi = 1, rows
         * 0, 1, 1 about mean 2 give exactly 2^-38. */
        const auto weights = make_weights<W>(1, 1, W(0.5) + std::ldexp(W(1), -40), W(-0.5));
        const auto fixed = run<W>(true, {0}, {2}, {0}, nullptr, weights, square);
        ASSERT_EQ(fixed.status, LINEARS_OK);
        EXPECT_EQ(fixed.mean[0], W(2));
        EXPECT_EQ(fixed.covariance[0], std::ldexp(W(1), -38));
    }
}

/* n = 512, m = 1: the 1024 non-center observation rows are equal, so C = 0
 * and the posterior equals the prior. */
TYPED_TEST(QUkfGate, LargeStateUpdateWithZeroCrossCovarianceKeepsThePrior) {
    using W = TypeParam;
    constexpr std::size_t n = 512;
    const W near = std::nextafter(W(1), W(0));
    const Map<W> observation = [near](const W *, std::size_t, W *y, std::size_t, std::size_t call) {
        y[0] = call == 0 ? -near : near;
    };
    std::vector<W> x(n, 0), p(n * n, 0);
    x[0] = W(0.25);
    for (std::size_t i = 0; i < n; ++i) p[i * n + i] = W(0.25);
    const std::vector<W> r{W(0.25)}, z{0};
    const auto weights = make_weights<W>(n, 1, 2, 0);
    const auto fixed = run<W>(true, x, p, r, &z, weights, observation);
    ASSERT_EQ(fixed.status, LINEARS_OK);
    EXPECT_EQ(fixed.calls, 2 * n + 1);
    EXPECT_EQ(fixed.mean, x);
    EXPECT_EQ(fixed.covariance, p);
}

/* The gain uses reciprocal pivots. Pivots are square roots of positive
 * finite values, so even the smallest and largest working S keep finite,
 * normal reciprocals; outputs agree with the floating solve. */
TYPED_TEST(QUkfGate, ReciprocalPivotGainMatchesTheFloatingSolveAtExtremeScales) {
    using W = TypeParam;
    const Map<W> observation = [](const W *x, std::size_t, W *y, std::size_t, std::size_t) {
        y[0] = W(0.5) * x[0] + W(0.25) * x[1];
        y[1] = W(0.25) * x[0] - W(0.5) * x[1];
    };
    const auto weights = make_weights<W>(2, 1, 2, 0);
    const int powers[] = {Api<W>::bits == 15 ? -120 : -1000, Api<W>::bits == 15 ? 120 : 1000};
    for (int power : powers) {
        SCOPED_TRACE(power);
        const W variance = std::ldexp(W(1), power), spread = std::ldexp(W(1), power / 2);
        const std::vector<W> x{W(0.5) * spread, W(-0.25) * spread};
        const std::vector<W> p{variance, W(0.25) * variance, W(0.25) * variance, variance};
        const std::vector<W> r{W(0.5) * variance, 0, 0, W(0.25) * variance};
        const std::vector<W> z{W(0.125) * spread, W(0.375) * spread};
        const auto fixed = run<W>(true, x, p, r, &z, weights, observation);
        const auto floating = run<W>(false, x, p, r, &z, weights, observation);
        ASSERT_EQ(fixed.status, LINEARS_OK);
        ASSERT_EQ(floating.status, LINEARS_OK);
        const long double relative = 64 * std::numeric_limits<W>::epsilon();
        for (std::size_t i = 0; i < 2; ++i)
            EXPECT_LE(std::fabs(static_cast<long double>(fixed.mean[i]) - floating.mean[i]), relative * spread);
        for (std::size_t i = 0; i < 4; ++i)
            EXPECT_LE(std::fabs(static_cast<long double>(fixed.covariance[i]) - floating.covariance[i]),
                      relative * variance);
        EXPECT_EQ(fixed.covariance[1], fixed.covariance[2]);
    }
}

}  // namespace
