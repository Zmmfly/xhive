#include "ukf_support.h"

#include <deque>

namespace {
using namespace ukf_test;

/* The predict/update kernels are compared bitwise with their former
 * composition of public linears operations, re-assembled here in the same
 * order. Loops the former kernels already evaluated inline are repeated
 * verbatim; everything else goes through the linears entry points the
 * kernels used to call, so the oracle shares no code with the kernels. */
template<class A> struct Linear;
#define PIPELINE_LINEAR_API(tag) \
template<> struct Linear<tag> { \
    using Decomposition = linears_##tag##_workspace_t; \
    static constexpr auto axpby = linears_axpby_##tag; \
    static constexpr auto add = linears_add_##tag; \
    static constexpr auto sub = linears_sub_##tag; \
    static constexpr auto mul = linears_mul_##tag; \
    static constexpr auto transpose = linears_transpose_##tag; \
    static constexpr auto cholesky = linears_cholesky_##tag; \
    static constexpr auto solve = linears_solve_##tag; \
};
PIPELINE_LINEAR_API(f32)
PIPELINE_LINEAR_API(f64)
#undef PIPELINE_LINEAR_API

#define LEGACY_TRY(expression) do { \
    const linears_status_t legacy_status = (expression); \
    if (legacy_status != LINEARS_OK) return legacy_status; \
} while (0)

template<class A> class Legacy {
public:
    using W = Work<A>;
    using L = Linear<A>;
    using Weights = typename Ukf<A>::Weights;
    using Fn = typename Ukf<A>::Fn;
    using Residual = typename Ukf<A>::Residual;

    linears_status_t sigma_points(const linears_matrix_t& mean, const linears_matrix_t& covariance,
                                  W scale, linears_matrix_t& points) {
        const std::size_t n = mean.rows;
        if (!std::isfinite(scale)) return LINEARS_ERROR_RANGE;
        if (!(scale > 0)) return LINEARS_ERROR_DOMAIN;
        linears_matrix_t scaled = make(n, n), factor = make(n, n), columns = make(n, n);
        LEGACY_TRY(L::axpby(scale, &covariance, W(0), &covariance, &scaled, basic(), basic_bytes()));
        LEGACY_TRY(L::cholesky(&scaled, &factor, decomposition()));
        LEGACY_TRY(L::transpose(&factor, &columns, basic(), basic_bytes()));
        std::memcpy(points.data, mean.data, n * sizeof(W));
        for (std::size_t i = 0; i < n; ++i) {
            linears_matrix_t column = row(columns, i), plus = row(points, 1 + i), minus = row(points, 1 + n + i);
            LEGACY_TRY(L::add(&mean, &column, &plus, basic(), basic_bytes()));
            LEGACY_TRY(L::sub(&mean, &column, &minus, basic(), basic_bytes()));
        }
        return LINEARS_OK;
    }

    linears_status_t predict(const linears_matrix_t& mean, const linears_matrix_t& covariance,
                             const linears_matrix_t& process_noise, const Weights& weights,
                             Fn transition, void* transition_context, Residual residual, void* residual_context,
                             linears_matrix_t& out_mean, linears_matrix_t& out_covariance) {
        const std::size_t n = mean.rows, s = 2 * n + 1;
        std::vector<W> wm(s, weights.other), wc(s, weights.other);
        wm[0] = weights.mean0;
        wc[0] = weights.covariance0;
        linears_matrix_t points = make(s, n), propagated = make(s, n), deviated = make(s, n);
        LEGACY_TRY(sigma_points(mean, covariance, weights.scale, points));
        LEGACY_TRY(propagate(points, transition, transition_context, propagated));
        LEGACY_TRY(average(propagated, wm.data(), residual, residual_context, out_mean));
        LEGACY_TRY(deviate(propagated, out_mean, residual, residual_context, deviated));
        LEGACY_TRY(weighted(deviated, deviated, wc.data(), out_covariance));
        LEGACY_TRY(L::add(&out_covariance, &process_noise, &out_covariance, basic(), basic_bytes()));
        return symmetrize(out_covariance);
    }

    linears_status_t update(const linears_matrix_t& mean, const linears_matrix_t& covariance,
                            const linears_matrix_t& measurement_noise, const linears_matrix_t& measurement,
                            const Weights& weights, Fn observation, void* observation_context,
                            Residual residual, void* residual_context,
                            linears_matrix_t& out_mean, linears_matrix_t& out_covariance) {
        const std::size_t n = mean.rows, m = measurement_noise.rows, s = 2 * n + 1;
        std::vector<W> wm(s, weights.other), wc(s, weights.other);
        wm[0] = weights.mean0;
        wc[0] = weights.covariance0;
        linears_matrix_t factor = make(m, m);
        /* R was factored before any callback. */
        LEGACY_TRY(L::cholesky(&measurement_noise, &factor, decomposition()));
        linears_matrix_t points = make(s, n), measured = make(s, m), zbar = make(m, 1);
        linears_matrix_t dz = make(s, m), dx = make(s, n), innovation_covariance = make(m, m);
        linears_matrix_t cross = make(n, m), cross_t = make(m, n), gain_t = make(m, n), gain = make(n, m);
        linears_matrix_t innovation = make(m, 1), reduction = make(n, n);
        LEGACY_TRY(sigma_points(mean, covariance, weights.scale, points));
        LEGACY_TRY(propagate(points, observation, observation_context, measured));
        LEGACY_TRY(average(measured, wm.data(), residual, residual_context, zbar));
        LEGACY_TRY(deviate(measured, zbar, residual, residual_context, dz));
        LEGACY_TRY(deviate(points, mean, nullptr, nullptr, dx));
        LEGACY_TRY(weighted(dz, dz, wc.data(), innovation_covariance));
        LEGACY_TRY(L::add(&innovation_covariance, &measurement_noise, &innovation_covariance, basic(), basic_bytes()));
        LEGACY_TRY(symmetrize(innovation_covariance));
        LEGACY_TRY(L::cholesky(&innovation_covariance, &factor, decomposition()));
        LEGACY_TRY(weighted(dx, dz, wc.data(), cross));
        LEGACY_TRY(L::transpose(&cross, &cross_t, basic(), basic_bytes()));
        LEGACY_TRY(L::solve(&innovation_covariance, &cross_t, &gain_t, decomposition()));
        LEGACY_TRY(L::transpose(&gain_t, &gain, basic(), basic_bytes()));
        linears_matrix_t z_row{measurement.data, 1, m}, innovation_row{innovation.data, 1, m};
        LEGACY_TRY(deviate(z_row, zbar, residual, residual_context, innovation_row));
        LEGACY_TRY(L::mul(&gain, &innovation, &out_mean, basic(), basic_bytes()));
        LEGACY_TRY(L::add(&mean, &out_mean, &out_mean, basic(), basic_bytes()));
        LEGACY_TRY(L::mul(&gain, &cross_t, &reduction, basic(), basic_bytes()));
        LEGACY_TRY(L::sub(&covariance, &reduction, &out_covariance, basic(), basic_bytes()));
        return symmetrize(out_covariance);
    }

private:
    std::vector<linears_workspace_unit_t> units = std::vector<linears_workspace_unit_t>(1024);
    std::vector<W> factor_scratch = std::vector<W>(4096);
    std::deque<std::vector<W>> arena;

    linears_matrix_t make(std::size_t rows, std::size_t cols) {
        arena.emplace_back(rows * cols, W(0));
        return {arena.back().data(), rows, cols};
    }
    linears_workspace_unit_t* basic() { return units.data(); }
    std::size_t basic_bytes() const { return units.size() * sizeof(linears_workspace_unit_t); }
    typename L::Decomposition decomposition() { return {factor_scratch.data(), factor_scratch.size()}; }
    static linears_matrix_t row(const linears_matrix_t& matrix, std::size_t index) {
        return {static_cast<W*>(matrix.data) + index * matrix.cols, matrix.cols, 1};
    }
    static bool finite(const linears_matrix_t& matrix) {
        const auto* values = static_cast<const W*>(matrix.data);
        for (std::size_t i = 0; i < matrix.rows * matrix.cols; ++i)
            if (!std::isfinite(values[i])) return false;
        return true;
    }
    linears_status_t propagate(const linears_matrix_t& points, Fn fn, void* context, linears_matrix_t& out) {
        for (std::size_t i = 0; i < points.rows; ++i) {
            linears_matrix_t in = row(points, i), result = row(out, i);
            LEGACY_TRY(fn(&in, &result, context));
            if (!finite(result)) return LINEARS_ERROR_RANGE;
        }
        return LINEARS_OK;
    }
    linears_status_t deviate(const linears_matrix_t& points, const linears_matrix_t& center,
                             Residual residual, void* context, linears_matrix_t& out) {
        for (std::size_t i = 0; i < points.rows; ++i) {
            linears_matrix_t in = row(points, i), result = row(out, i);
            if (residual == nullptr) {
                LEGACY_TRY(L::sub(&in, &center, &result, basic(), basic_bytes()));
            } else {
                LEGACY_TRY(residual(&in, &center, &result, context));
                if (!finite(result)) return LINEARS_ERROR_RANGE;
            }
        }
        return LINEARS_OK;
    }
    /* Weighted mean: the column sums the former kernel evaluated inline, with
     * the residual detour about row 0 and the final move back through add. */
    linears_status_t average(const linears_matrix_t& points, const W* w, Residual residual, void* context,
                             linears_matrix_t& out) {
        const std::size_t s = points.rows, k = points.cols;
        linears_matrix_t source = points, reference{};
        if (residual != nullptr) {
            linears_matrix_t deviated = make(s, k);
            reference = row(points, 0);
            LEGACY_TRY(deviate(points, reference, residual, context, deviated));
            source = deviated;
        }
        const auto* values = static_cast<const W*>(source.data);
        std::vector<W> sums(k);
        for (std::size_t j = 0; j < k; ++j) {
            W sum = W(0);
            for (std::size_t i = 0; i < s; ++i) {
                const W product = values[i * k + j] * w[i];
                sum += product;
                if (!std::isfinite(sum)) return LINEARS_ERROR_RANGE;
            }
            sums[j] = sum;
        }
        std::memcpy(out.data, sums.data(), k * sizeof(W));
        if (residual != nullptr) LEGACY_TRY(L::add(&reference, &out, &out, basic(), basic_bytes()));
        return LINEARS_OK;
    }
    /* a^T * diag(w) * b as the former kernel evaluated it: w*b + 0*b, then
     * the transposed product's reduction order, committed after every sum. */
    linears_status_t weighted(const linears_matrix_t& a, const linears_matrix_t& b, const W* w,
                              linears_matrix_t& out) {
        const std::size_t s = a.rows, p = a.cols, q = b.cols;
        const auto* av = static_cast<const W*>(a.data);
        const auto* bv = static_cast<const W*>(b.data);
        std::vector<W> scaled(s * q), sums(p * q);
        for (std::size_t i = 0; i < s; ++i) {
            for (std::size_t j = 0; j < q; ++j) {
                const W left = w[i] * bv[i * q + j], right = W(0) * bv[i * q + j];
                const W value = left + right;
                if (!std::isfinite(value)) return LINEARS_ERROR_RANGE;
                scaled[i * q + j] = value;
            }
        }
        for (std::size_t i = 0; i < p; ++i) {
            for (std::size_t j = 0; j < q; ++j) {
                W sum = W(0);
                for (std::size_t k = 0; k < s; ++k) {
                    const W product = av[k * p + i] * scaled[k * q + j];
                    sum += product;
                    if (!std::isfinite(sum)) return LINEARS_ERROR_RANGE;
                }
                sums[i * q + j] = sum;
            }
        }
        std::memcpy(out.data, sums.data(), p * q * sizeof(W));
        return LINEARS_OK;
    }
    linears_status_t symmetrize(linears_matrix_t& matrix) {
        linears_matrix_t transposed = make(matrix.rows, matrix.cols);
        LEGACY_TRY(L::transpose(&matrix, &transposed, basic(), basic_bytes()));
        return L::axpby(W(0.5), &matrix, W(0.5), &transposed, &matrix, basic(), basic_bytes());
    }
};
#undef LEGACY_TRY

template<class W> void expect_identical(const std::vector<W>& actual, const std::vector<W>& expected) {
    ASSERT_EQ(actual.size(), expected.size());
    for (std::size_t i = 0; i < actual.size(); ++i)
        EXPECT_EQ(std::memcmp(&actual[i], &expected[i], sizeof(W)), 0) << "element " << i;
}

/* Mildly nonlinear models in long double; the callback boundary rounds them
 * to the working type identically for the kernel and the oracle. */
Model transition_model(std::size_t n) {
    return Model{[n](const Vec& x) {
        Vec y(n);
        for (std::size_t j = 0; j < n; ++j) y[j] = 0.9L * x[j] + 0.1L * x[(j + 1) % n] + 0.05L * x[j] * x[j];
        return y;
    }};
}
Model observation_model(std::size_t n, std::size_t m) {
    return Model{[n, m](const Vec& x) {
        Vec y(m);
        for (std::size_t j = 0; j < m; ++j)
            y[j] = 0.75L * x[j % n] + 0.125L * x[(j + 1) % n] + 0.0625L * x[j % n] * x[(j + 2) % n];
        return y;
    }};
}
Model subtraction_model() {
    Model model;
    model.residual = [](const Vec& a, const Vec& b) {
        Vec d(a.size());
        for (std::size_t i = 0; i < d.size(); ++i) d[i] = a[i] - b[i];
        return d;
    };
    return model;
}

/* Fails, or emits a nonfinite value, at one chosen call of a model. */
template<class A> struct Faulty {
    Model model;
    int fail_at = -1;
    linears_status_t failure = LINEARS_OK;
    int nonfinite_at = -1;
    int calls = 0;
};
template<class A> linears_status_t faulty_map(const linears_matrix_t* in, linears_matrix_t* out, void* context) {
    auto* faulty = static_cast<Faulty<A>*>(context);
    const int call = faulty->calls++;
    if (call == faulty->fail_at) return faulty->failure;
    faulty->model.nonfinite = call == faulty->nonfinite_at;
    return map_callback<A>(in, out, &faulty->model);
}
template<class A> linears_status_t faulty_residual(const linears_matrix_t* a, const linears_matrix_t* b,
                                                   linears_matrix_t* out, void* context) {
    auto* faulty = static_cast<Faulty<A>*>(context);
    const int call = faulty->calls++;
    if (call == faulty->fail_at) return faulty->failure;
    faulty->model.nonfinite = call == faulty->nonfinite_at;
    return residual_callback<A>(a, b, out, &faulty->model);
}

/* Kernel inputs of one trial in the working type. */
template<class A> struct Trial {
    using W = Work<A>;
    std::size_t n, m;
    WorkMatrix<A> x, p, q, r, z;
    typename Ukf<A>::Weights weights{};
    Trial(std::mt19937& random, std::size_t n_, std::size_t m_)
        : n(n_), m(m_), x(random_dense(random, n_, 1, 0.5L)), p(random_spd(random, n_, 0.02L, 0.1L)),
          q(random_spd(random, n_, 0.001L, 0.02L)), r(random_spd(random, m_, 0.01L, 0.05L)),
          z(random_dense(random, m_, 1, 0.3L)) {
        EXPECT_EQ(UkfKernels<A>::weights(n, W(0.7), W(2), static_cast<W>(3.0L - static_cast<long double>(n)),
                                         &weights), LINEARS_OK);
    }
};

template<class A> class UnscentedPipelineKernels : public ::testing::Test {};
TYPED_TEST_SUITE(UnscentedPipelineKernels, KernelFormats);
template<class A> class UnscentedPipeline : public ::testing::Test {};
TYPED_TEST_SUITE(UnscentedPipeline, Formats);

TYPED_TEST(UnscentedPipelineKernels, PredictAndUpdateMatchTheLegacyCompositionBitwise) {
    using A = TypeParam;
    std::mt19937 random(0x50495045u);
    for (std::size_t n : {1u, 2u, 3u}) {
        for (std::size_t m : {1u, 2u, 4u, 6u}) {
            for (int residual_mode = 0; residual_mode < 2; ++residual_mode) {
                SCOPED_TRACE(::testing::Message() << "n=" << n << " m=" << m << " residual=" << residual_mode);
                Trial<A> trial(random, n, m);
                Model transition = transition_model(n), observation = observation_model(n, m);
                Model state_residual = subtraction_model(), measurement_residual = subtraction_model();
                Model legacy_transition = transition, legacy_observation = observation;
                Model legacy_state_residual = state_residual, legacy_measurement_residual = measurement_residual;
                const auto residual = residual_mode ? residual_callback<A> : nullptr;
                auto xv = trial.x.view(), pv = trial.p.view(), qv = trial.q.view(), rv = trial.r.view(), zv = trial.z.view();
                WorkMatrix<A> predicted_mean(n, 1), predicted_covariance(n, n), legacy_mean(n, 1), legacy_covariance(n, n);
                auto pm = predicted_mean.view(), pc = predicted_covariance.view(), lm = legacy_mean.view(), lc = legacy_covariance.view();
                UkfWorkspace<A> predict_ws(FILTERS_UKF_PREDICT_WORK_STORAGE(n), FILTERS_UKF_PREDICT_BASIC_BYTES(n),
                                           FILTERS_UKF_PREDICT_DECOMP_STORAGE(n));
                Legacy<A> legacy;
                ASSERT_EQ(Ukf<A>::predict_work(&xv, &pv, &qv, &trial.weights, map_callback<A>, &transition, residual,
                                               &state_residual, &pm, &pc, predict_ws.view()), LINEARS_OK);
                ASSERT_EQ(legacy.predict(xv, pv, qv, trial.weights, map_callback<A>, &legacy_transition, residual,
                                         &legacy_state_residual, lm, lc), LINEARS_OK);
                expect_identical(predicted_mean.data, legacy_mean.data);
                expect_identical(predicted_covariance.data, legacy_covariance.data);
                EXPECT_EQ(transition.calls, legacy_transition.calls);
                EXPECT_EQ(state_residual.calls, legacy_state_residual.calls);
                EXPECT_EQ(transition.calls, static_cast<int>(2 * n + 1));
                EXPECT_EQ(state_residual.calls, residual_mode ? static_cast<int>(2 * (2 * n + 1)) : 0);
                /* Both updates start from the kernel's prediction. */
                WorkMatrix<A> posterior_mean(n, 1), posterior_covariance(n, n), legacy_posterior_mean(n, 1), legacy_posterior_covariance(n, n);
                auto um = posterior_mean.view(), uc = posterior_covariance.view();
                auto lum = legacy_posterior_mean.view(), luc = legacy_posterior_covariance.view();
                UkfWorkspace<A> update_ws(FILTERS_UKF_UPDATE_WORK_STORAGE(n, m), FILTERS_UKF_UPDATE_BASIC_BYTES(n, m),
                                          FILTERS_UKF_UPDATE_DECOMP_STORAGE(n, m));
                ASSERT_EQ(Ukf<A>::update_work(&pm, &pc, &rv, &zv, &trial.weights, map_callback<A>, &observation, residual,
                                              &measurement_residual, &um, &uc, update_ws.view()), LINEARS_OK);
                ASSERT_EQ(legacy.update(pm, pc, rv, zv, trial.weights, map_callback<A>, &legacy_observation, residual,
                                        &legacy_measurement_residual, lum, luc), LINEARS_OK);
                expect_identical(posterior_mean.data, legacy_posterior_mean.data);
                expect_identical(posterior_covariance.data, legacy_posterior_covariance.data);
                EXPECT_EQ(observation.calls, legacy_observation.calls);
                EXPECT_EQ(measurement_residual.calls, legacy_measurement_residual.calls);
                EXPECT_EQ(observation.calls, static_cast<int>(2 * n + 1));
                EXPECT_EQ(measurement_residual.calls, residual_mode ? static_cast<int>(2 * (2 * n + 1) + 1) : 0);
                predict_ws.guards(); update_ws.guards();
            }
        }
    }
}

TYPED_TEST(UnscentedPipelineKernels, FailuresKeepTheLegacyStatusAndCallbackCounts) {
    using A = TypeParam;
    using W = Work<A>;
    std::mt19937 random(0x4641494cu);
    for (std::size_t n : {1u, 2u}) {
        const std::size_t m = n == 1 ? 4 : 3, s = 2 * n + 1;
        Trial<A> trial(random, n, m);
        auto xv = trial.x.view(), pv = trial.p.view(), qv = trial.q.view(), rv = trial.r.view(), zv = trial.z.view();
        UkfWorkspace<A> predict_ws(FILTERS_UKF_PREDICT_WORK_STORAGE(n), FILTERS_UKF_PREDICT_BASIC_BYTES(n),
                                   FILTERS_UKF_PREDICT_DECOMP_STORAGE(n));
        UkfWorkspace<A> update_ws(FILTERS_UKF_UPDATE_WORK_STORAGE(n, m), FILTERS_UKF_UPDATE_BASIC_BYTES(n, m),
                                  FILTERS_UKF_UPDATE_DECOMP_STORAGE(n, m));
        /* Transition failures and nonfinite outputs at every sigma point, and
         * state residual failures at every one of its 2s calls. A nonfinite
         * output is rejected at the call that produced it; a returned error
         * is reported at its call. */
        for (int mode = 0; mode < 4; ++mode) {
            const int limit = mode < 2 ? static_cast<int>(s) : static_cast<int>(2 * s);
            for (int at = 0; at < limit; ++at) {
                SCOPED_TRACE(::testing::Message() << "n=" << n << " predict mode=" << mode << " at=" << at);
                Faulty<A> transition{transition_model(n)}, residual{subtraction_model()};
                if (mode == 0) { transition.fail_at = at; transition.failure = LINEARS_ERROR_SINGULAR; }
                if (mode == 1) transition.nonfinite_at = at;
                if (mode == 2) { residual.fail_at = at; residual.failure = LINEARS_ERROR_NO_CONVERGENCE; }
                if (mode == 3) residual.nonfinite_at = at;
                Faulty<A> legacy_transition = transition, legacy_residual = residual;
                WorkMatrix<A> mean(n, 1), covariance(n, n), legacy_mean(n, 1), legacy_covariance(n, n);
                auto mv = mean.view(), cv = covariance.view(), lm = legacy_mean.view(), lc = legacy_covariance.view();
                Legacy<A> legacy;
                const linears_status_t expected = legacy.predict(xv, pv, qv, trial.weights, faulty_map<A>, &legacy_transition,
                                                                 faulty_residual<A>, &legacy_residual, lm, lc);
                EXPECT_EQ(Ukf<A>::predict_work(&xv, &pv, &qv, &trial.weights, faulty_map<A>, &transition, faulty_residual<A>,
                                               &residual, &mv, &cv, predict_ws.view()), expected);
                EXPECT_EQ(transition.calls, legacy_transition.calls);
                EXPECT_EQ(residual.calls, legacy_residual.calls);
                if (mode == 0) { EXPECT_EQ(expected, LINEARS_ERROR_SINGULAR); EXPECT_EQ(transition.calls, at + 1); }
                if (mode == 1) { EXPECT_EQ(expected, LINEARS_ERROR_RANGE); EXPECT_EQ(transition.calls, at + 1); }
                if (mode == 2) { EXPECT_EQ(expected, LINEARS_ERROR_NO_CONVERGENCE); EXPECT_EQ(residual.calls, at + 1); }
                if (mode == 3) { EXPECT_EQ(expected, LINEARS_ERROR_RANGE); EXPECT_EQ(residual.calls, at + 1); }
            }
        }
        /* Observation and measurement residual failures: the residual runs
         * 2s times for the statistics and once more for the innovation. */
        for (int mode = 0; mode < 4; ++mode) {
            const int limit = mode < 2 ? static_cast<int>(s) : static_cast<int>(2 * s + 1);
            for (int at = 0; at < limit; ++at) {
                SCOPED_TRACE(::testing::Message() << "n=" << n << " update mode=" << mode << " at=" << at);
                Faulty<A> observation{observation_model(n, m)}, residual{subtraction_model()};
                if (mode == 0) { observation.fail_at = at; observation.failure = LINEARS_ERROR_DOMAIN; }
                if (mode == 1) observation.nonfinite_at = at;
                if (mode == 2) { residual.fail_at = at; residual.failure = LINEARS_ERROR_SINGULAR; }
                if (mode == 3) residual.nonfinite_at = at;
                Faulty<A> legacy_observation = observation, legacy_residual = residual;
                WorkMatrix<A> mean(n, 1), covariance(n, n), legacy_mean(n, 1), legacy_covariance(n, n);
                auto mv = mean.view(), cv = covariance.view(), lm = legacy_mean.view(), lc = legacy_covariance.view();
                Legacy<A> legacy;
                const linears_status_t expected = legacy.update(xv, pv, rv, zv, trial.weights, faulty_map<A>, &legacy_observation,
                                                                faulty_residual<A>, &legacy_residual, lm, lc);
                EXPECT_EQ(Ukf<A>::update_work(&xv, &pv, &rv, &zv, &trial.weights, faulty_map<A>, &observation, faulty_residual<A>,
                                              &residual, &mv, &cv, update_ws.view()), expected);
                EXPECT_EQ(observation.calls, legacy_observation.calls);
                EXPECT_EQ(residual.calls, legacy_residual.calls);
                if (mode == 0) { EXPECT_EQ(expected, LINEARS_ERROR_DOMAIN); EXPECT_EQ(observation.calls, at + 1); }
                if (mode == 1) { EXPECT_EQ(expected, LINEARS_ERROR_RANGE); EXPECT_EQ(observation.calls, at + 1); }
                if (mode == 2) { EXPECT_EQ(expected, LINEARS_ERROR_SINGULAR); EXPECT_EQ(residual.calls, at + 1); }
                if (mode == 3) { EXPECT_EQ(expected, LINEARS_ERROR_RANGE); EXPECT_EQ(residual.calls, at + 1); }
            }
        }
        /* Factor failures precede every callback. A predicted covariance
         * large enough to overflow the sum with a maximal Q is RANGE after all
         * of them, with the former staged commit order. */
        Model transition = transition_model(n), observation = observation_model(n, m);
        WorkMatrix<A> mean(n, 1), covariance(n, n), zero(Dense::identity(n, 0)), huge(Dense::identity(n, 0));
        WorkMatrix<A> negative(decode_work<A>(rv));
        for (std::size_t i = 0; i < n; ++i) huge.data[i * n + i] = std::numeric_limits<W>::max();
        negative.data[0] = -negative.data[0];
        auto mv = mean.view(), cv = covariance.view(), zero_v = zero.view(), huge_v = huge.view(), negative_v = negative.view();
        EXPECT_EQ(Ukf<A>::predict_work(&xv, &zero_v, &qv, &trial.weights, map_callback<A>, &transition, nullptr, nullptr,
                                       &mv, &cv, predict_ws.view()), LINEARS_ERROR_NOT_POSITIVE_DEFINITE);
        EXPECT_EQ(Ukf<A>::update_work(&xv, &zero_v, &rv, &zv, &trial.weights, map_callback<A>, &observation, nullptr, nullptr,
                                      &mv, &cv, update_ws.view()), LINEARS_ERROR_NOT_POSITIVE_DEFINITE);
        EXPECT_EQ(Ukf<A>::update_work(&xv, &pv, &negative_v, &zv, &trial.weights, map_callback<A>, &observation, nullptr, nullptr,
                                      &mv, &cv, update_ws.view()), LINEARS_ERROR_NOT_POSITIVE_DEFINITE);
        EXPECT_EQ(transition.calls, 0);
        EXPECT_EQ(observation.calls, 0);
        /* Spread the points past half an ulp of the maximum: 2^60 for float,
         * 2^500 for double, both exact power-of-two scalings. */
        const int spread = std::numeric_limits<W>::max_exponent / 2 - 4;
        Model widening{[spread](const Vec& x) {
            Vec y(x.size());
            for (std::size_t j = 0; j < y.size(); ++j) y[j] = std::ldexp(x[j], spread);
            return y;
        }};
        Model legacy_widening = widening;
        Legacy<A> legacy;
        WorkMatrix<A> legacy_mean(n, 1), legacy_covariance(n, n);
        auto lm = legacy_mean.view(), lc = legacy_covariance.view();
        EXPECT_EQ(Ukf<A>::predict_work(&xv, &pv, &huge_v, &trial.weights, map_callback<A>, &widening, nullptr, nullptr,
                                       &mv, &cv, predict_ws.view()), LINEARS_ERROR_RANGE);
        EXPECT_EQ(legacy.predict(xv, pv, huge_v, trial.weights, map_callback<A>, &legacy_widening, nullptr, nullptr, lm, lc),
                  LINEARS_ERROR_RANGE);
        EXPECT_EQ(widening.calls, static_cast<int>(s));
        EXPECT_EQ(legacy_widening.calls, static_cast<int>(s));
        /* The same inputs with a modest Q succeed and agree bitwise. */
        ASSERT_EQ(Ukf<A>::predict_work(&xv, &pv, &qv, &trial.weights, map_callback<A>, &widening, nullptr, nullptr,
                                       &mv, &cv, predict_ws.view()), LINEARS_OK);
        ASSERT_EQ(legacy.predict(xv, pv, qv, trial.weights, map_callback<A>, &legacy_widening, nullptr, nullptr, lm, lc),
                  LINEARS_OK);
        expect_identical(mean.data, legacy_mean.data);
        expect_identical(covariance.data, legacy_covariance.data);
        predict_ws.guards(); update_ws.guards();
    }
}

TYPED_TEST(UnscentedPipelineKernels, SigmaPointsFactorMatchesAndChecksTheScaledCovariance) {
    using A = TypeParam;
    using W = Work<A>;
    std::mt19937 random(0x5349474du);
    for (std::size_t n : {1u, 2u, 3u, 4u, 6u}) {
        for (int trial = 0; trial < 4; ++trial) {
            SCOPED_TRACE(::testing::Message() << "n=" << n << " trial=" << trial);
            const std::size_t s = 2 * n + 1;
            WorkMatrix<A> mean(random_dense(random, n, 1, 2)), covariance(random_spd(random, n, 0.05L, 0.5L));
            WorkMatrix<A> points(s, n), legacy_points(s, n);
            auto mv = mean.view(), cv = covariance.view(), pv = points.view(), lv = legacy_points.view();
            const W scale = static_cast<W>(0.25L * (trial + 1));
            UkfWorkspace<A> ws(FILTERS_UKF_SIGMA_POINTS_STORAGE(n), FILTERS_UKF_SIGMA_POINTS_BASIC_BYTES(n),
                               FILTERS_UKF_SIGMA_POINTS_DECOMP_STORAGE(n));
            Legacy<A> legacy;
            ASSERT_EQ(UkfKernels<A>::sigma_points(&mv, &cv, scale, &pv, ws.view()), LINEARS_OK);
            ASSERT_EQ(legacy.sigma_points(mv, cv, scale, lv), LINEARS_OK);
            expect_identical(points.data, legacy_points.data);
            ws.guards();
        }
    }
    /* The former shared decomposition judged symmetry on the scaled copy it
     * loaded: mirrored elements that round to the same scaled value pass,
     * while a visibly asymmetric input is DIMENSION. The filter kernels keep
     * rejecting the raw asymmetry as DOMAIN. */
    const W tiny = std::numeric_limits<W>::denorm_min();
    WorkMatrix<A> mean(Dense(2, 1, {0.5L, -0.25L})), points(5, 2), legacy_points(5, 2), out_mean(2, 1), out_covariance(2, 2);
    WorkMatrix<A> rounding(Dense::identity(2)), visible(Dense(2, 2, {1, 0.5L, 0.25L, 1})), noise(Dense::identity(2, 0.01L));
    rounding.data[1] = tiny;
    rounding.data[2] = W(2) * tiny;
    auto mv = mean.view(), pv = points.view(), lv = legacy_points.view(), rv = rounding.view(), vv = visible.view();
    auto om = out_mean.view(), oc = out_covariance.view(), nv = noise.view();
    UkfWorkspace<A> ws(FILTERS_UKF_SIGMA_POINTS_STORAGE(2), FILTERS_UKF_SIGMA_POINTS_BASIC_BYTES(2),
                       FILTERS_UKF_SIGMA_POINTS_DECOMP_STORAGE(2));
    Legacy<A> legacy;
    ASSERT_EQ(W(0.25) * tiny, W(0));
    ASSERT_EQ(W(0.25) * (W(2) * tiny), W(0));
    EXPECT_EQ(UkfKernels<A>::sigma_points(&mv, &rv, W(0.25), &pv, ws.view()), LINEARS_OK);
    EXPECT_EQ(legacy.sigma_points(mv, rv, W(0.25), lv), LINEARS_OK);
    expect_identical(points.data, legacy_points.data);
    EXPECT_EQ(UkfKernels<A>::sigma_points(&mv, &rv, W(1), &pv, ws.view()), LINEARS_ERROR_DIMENSION);
    EXPECT_EQ(legacy.sigma_points(mv, rv, W(1), lv), LINEARS_ERROR_DIMENSION);
    EXPECT_EQ(UkfKernels<A>::sigma_points(&mv, &vv, W(0.25), &pv, ws.view()), LINEARS_ERROR_DIMENSION);
    EXPECT_EQ(legacy.sigma_points(mv, vv, W(0.25), lv), LINEARS_ERROR_DIMENSION);
    typename Ukf<A>::Weights weights{};
    ASSERT_EQ(UkfKernels<A>::weights(2, W(0.5), W(2), W(0), &weights), LINEARS_OK);
    ASSERT_EQ(weights.scale, W(0.5));
    Model transition = transition_model(2);
    UkfWorkspace<A> predict_ws(FILTERS_UKF_PREDICT_WORK_STORAGE(2), FILTERS_UKF_PREDICT_BASIC_BYTES(2),
                               FILTERS_UKF_PREDICT_DECOMP_STORAGE(2));
    EXPECT_EQ(Ukf<A>::predict_work(&mv, &rv, &nv, &weights, map_callback<A>, &transition, nullptr, nullptr,
                                   &om, &oc, predict_ws.view()), LINEARS_ERROR_DOMAIN);
    EXPECT_EQ(transition.calls, 0);
    ws.guards(); predict_ws.guards();
}

TYPED_TEST(UnscentedPipelineKernels, WideMeasurementUpdatesFitTheExactMinimumWorkspace) {
    using A = TypeParam;
    std::mt19937 random(0x57494445u);
    const std::size_t shapes[][2] = {{1, 4}, {1, 5}, {2, 6}, {3, 8}};
    for (const auto& shape : shapes) {
        const std::size_t n = shape[0], m = shape[1];
        SCOPED_TRACE(::testing::Message() << "n=" << n << " m=" << m);
        ASSERT_GT(m, 2 * n + 1);
        Trial<A> trial(random, n, m);
        Model observation = observation_model(n, m), legacy_observation = observation;
        auto xv = trial.x.view(), pv = trial.p.view(), rv = trial.r.view(), zv = trial.z.view();
        WorkMatrix<A> mean(n, 1), covariance(n, n), legacy_mean(n, 1), legacy_covariance(n, n);
        auto mv = mean.view(), cv = covariance.view(), lm = legacy_mean.view(), lc = legacy_covariance.view();
        UkfWorkspace<A> ws(FILTERS_UKF_UPDATE_WORK_STORAGE(n, m), FILTERS_UKF_UPDATE_BASIC_BYTES(n, m),
                           FILTERS_UKF_UPDATE_DECOMP_STORAGE(n, m));
        /* One element short in any array is still WORKSPACE, before callbacks. */
        for (int part = 0; part < 3; ++part) {
            auto w = ws.view();
            if (part == 0) --w.capacity;
            if (part == 1) --w.basic_bytes;
            if (part == 2) --w.decomposition.capacity;
            EXPECT_EQ(Ukf<A>::update_work(&xv, &pv, &rv, &zv, &trial.weights, map_callback<A>, &observation, nullptr, nullptr,
                                          &mv, &cv, w), LINEARS_ERROR_WORKSPACE);
        }
        EXPECT_EQ(observation.calls, 0);
        Legacy<A> legacy;
        ASSERT_EQ(Ukf<A>::update_work(&xv, &pv, &rv, &zv, &trial.weights, map_callback<A>, &observation, nullptr, nullptr,
                                      &mv, &cv, ws.view()), LINEARS_OK);
        ASSERT_EQ(legacy.update(xv, pv, rv, zv, trial.weights, map_callback<A>, &legacy_observation, nullptr, nullptr, lm, lc),
                  LINEARS_OK);
        expect_identical(mean.data, legacy_mean.data);
        expect_identical(covariance.data, legacy_covariance.data);
        EXPECT_EQ(observation.calls, static_cast<int>(2 * n + 1));
        ws.guards();
    }
}

TYPED_TEST(UnscentedPipeline, WideMeasurementFiltersMatchTheLinearKalmanFilter) {
    using A = TypeParam;
    std::mt19937 random(0x574b4c46u);
    const std::size_t shapes[][2] = {{1, 4}, {2, 6}, {1, 5}};
    for (const auto& shape : shapes) {
        const std::size_t n = shape[0], m = shape[1];
        SCOPED_TRACE(::testing::Message() << "n=" << n << " m=" << m);
        const Dense x0 = random_dense(random, n, 1, A::bits ? 0.1L : 1);
        const Dense p0 = random_spd(random, n, A::bits ? 0.125L : 1, A::bits ? 0.08L : 0.4L);
        const Dense q = Dense::identity(n, A::bits ? 0.002L : 0.01L);
        const Dense z = random_dense(random, m, 1, A::bits ? 0.1L : 1);
        const Dense h = random_dense(random, m, n, A::bits ? 0.2L : 0.7L);
        const Dense r = random_spd(random, m, A::bits ? 0.125L : 0.2L, A::bits ? 0.08L : 0.4L);
        Matrix<A> xm(x0), pm(p0), qm(q), zm(z), hm(h), rm(r);
        auto xv = xm.view(), pv = pm.view(), qv = qm.view(), zv = zm.view(), hv = hm.view(), rv = rm.view();
        Model observation{[H = decode<A>(hv)](const Vec& x) { return linear_map(H, x); }};
        UkfState<A> ukf(n, m);
        MatrixState<A> kf(n);
        auto update_ws = UkfWorkspace<A>::update(n, m);
        Workspace<A> kf_update(n, m);
        ASSERT_EQ(Ukf<A>::init(&ukf.state, work_encode<A>(1), work_encode<A>(2), work_encode<A>(3.0L - n),
                               &xv, &pv, &qv, &rv), LINEARS_OK);
        ASSERT_EQ(A::reset_matrix(&kf.state, &xv, &pv), LINEARS_OK);
        const auto snapshot = ukf.snapshot();
        for (int part = 0; part < 4; ++part) {
            auto w = update_ws.view();
            std::size_t *capacity = part == 0 ? &w.capacity : part == 1 ? &w.basic_bytes : part == 2 ? &w.decomposition.capacity : nullptr;
            if constexpr (A::bits != 0) {
                if (part == 3) capacity = &w.signal_capacity;
            }
            if (capacity == nullptr || *capacity == 0) continue;
            --*capacity;
            EXPECT_EQ(Ukf<A>::update(&ukf.state, &zv, map_callback<A>, &observation, nullptr, nullptr, w), LINEARS_ERROR_WORKSPACE);
        }
        EXPECT_EQ(observation.calls, 0);
        EXPECT_EQ(ukf.snapshot(), snapshot);
        ASSERT_EQ(Ukf<A>::update(&ukf.state, &zv, map_callback<A>, &observation, nullptr, nullptr, update_ws.view()), LINEARS_OK);
        ASSERT_EQ(A::update_matrix(&kf.state, &zv, &hv, &rv, kf_update.view()), LINEARS_OK);
        EXPECT_EQ(observation.calls, static_cast<int>(2 * n + 1));
        expect_matrix<A>(ukf.state.estimate, decode<A>(kf.state.estimate));
        expect_matrix<A>(ukf.state.covariance, decode<A>(kf.state.covariance));
        ukf.guards(); kf.guards(); update_ws.guards(); kf_update.guards();
    }
}

} // namespace
