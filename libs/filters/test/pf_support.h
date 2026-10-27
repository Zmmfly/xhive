#ifndef FILTERS_TEST_PF_SUPPORT_H
#define FILTERS_TEST_PF_SUPPORT_H

/* Shared by pf.cc and upf.cc: PF adapters, a logging standard-normal sampler,
 * guarded state fixtures and a long-double bootstrap particle filter. */
#include "ukf_support.h"
#include "filters_pf.h"

namespace pf_test {
using namespace ukf_test;

/* Floating public kernels are distinct from the raw-signal filter paths. */
template<class A> struct PfKernels;
#define PF_KERNEL_TEST_API(tag) \
template<> struct PfKernels<tag> { \
    static constexpr auto perturb = filters_pf_##tag##_perturb; \
    static constexpr auto gaussian_exponent = filters_pf_##tag##_gaussian_exponent; \
    static constexpr auto reweight = filters_pf_##tag##_reweight; \
    static constexpr auto resample_indices = filters_pf_##tag##_resample_indices; \
    static constexpr auto gather = filters_pf_##tag##_gather; \
};
PF_KERNEL_TEST_API(f32)
PF_KERNEL_TEST_API(f64)
#undef PF_KERNEL_TEST_API

template<class A> struct Pf;
#define PF_TEST_API(tag, callback) \
template<> struct Pf<tag> { \
    using State = filters_pf_##tag##_t; \
    using Workspace = filters_ukf_##tag##_workspace_t; \
    using Sample = filters_pf_##callback##_sample_fn; \
    static constexpr auto effective_sample_size = filters_pf_##tag##_effective_sample_size; \
    static constexpr auto init = filters_pf_##tag##_init; \
    static constexpr auto predict = filters_pf_##tag##_predict; \
    static constexpr auto update = filters_pf_##tag##_update; \
    static constexpr auto resample = filters_pf_##tag##_resample; \
    static constexpr auto estimate = filters_pf_##tag##_estimate; \
};
PF_TEST_API(f32, f32)
PF_TEST_API(f64, f64)
PF_TEST_API(q15, f32)
PF_TEST_API(q31, f64)
#undef PF_TEST_API

/* Standard-normal draws from a seeded generator or a fixed script; every draw
 * is logged as the kernel saw it (after working-format quantization) so a
 * long-double reference can replay exactly the same noise. */
struct Sampler {
    std::mt19937 random;
    std::normal_distribution<double> normal{0, 1};
    std::vector<Vec> script;
    std::size_t scripted = 0;
    std::vector<Vec> log;
    int calls = 0;
    int fail_at = -1;
    linears_status_t failure = LINEARS_ERROR_NO_CONVERGENCE;
    bool nonfinite = false;
    explicit Sampler(unsigned seed) : random(seed) {}
    Vec next(std::size_t k) {
        if (!script.empty()) {
            Vec draw = script[scripted++ % script.size()];
            draw.resize(k, 0);
            return draw;
        }
        Vec draw(k);
        for (auto& value : draw) value = normal(random);
        return draw;
    }
    std::vector<Vec> take_log() { std::vector<Vec> taken; taken.swap(log); return taken; }
    long double uniform() { return std::uniform_real_distribution<double>(0, 1)(random); }
};
template<class A> linears_status_t sample_callback(linears_matrix_t *out, void *context) {
    auto *sampler = static_cast<Sampler*>(context);
    if (sampler->calls++ == sampler->fail_at) return sampler->failure;
    EXPECT_EQ(out->cols, 1u);
    Vec draw = sampler->next(out->rows);
    encode_vector<A>(draw, out);
    for (auto& value : draw) value = work_decode<A>(work_encode<A>(value));
    sampler->log.push_back(draw);
    if (sampler->nonfinite) static_cast<Work<A>*>(out->data)[0] = std::numeric_limits<Work<A>>::quiet_NaN();
    return LINEARS_OK;
}

template<class A> std::vector<Vec> decode_rows(const linears_matrix_t& matrix) {
    const Dense all = decode<A>(matrix);
    std::vector<Vec> rows(all.rows, Vec(all.cols));
    for (std::size_t i = 0; i < all.rows; ++i)
        for (std::size_t j = 0; j < all.cols; ++j) rows[i][j] = all(i, j);
    return rows;
}
/* The distribution a weight vector defines: Q weights carry a common scale
 * (largest = raw maximum), so they are divided by their sum. */
template<class A> Vec distribution(const linears_matrix_t& weights) {
    Vec values = decode<A>(weights).values;
    if constexpr (A::bits != 0) {
        long double sum = 0;
        for (auto value : values) sum += value;
        if (sum != 0) for (auto& value : values) value /= sum;
    }
    return values;
}
/* Committed Q weights are scaled so the largest raw value is the maximum. */
template<class A> void expect_raw_weight_scale(const std::vector<typename A::Raw>& guarded) {
    if constexpr (A::bits != 0)
        EXPECT_EQ(*std::max_element(guarded.begin() + 1, guarded.end() - 1), std::numeric_limits<typename A::Raw>::max());
}

/* Raw particle and weight arrays with sentinels. */
template<class A> struct PfState {
    using Raw = typename A::Raw;
    static constexpr Raw sentinel = static_cast<Raw>(-12345);
    std::vector<Raw> particles, weights, q, r;
    typename Pf<A>::State state;
    PfState(std::size_t count, std::size_t n, std::size_t m)
        : particles(count * n + 2, sentinel), weights(count + 2, sentinel),
          q(n * n + 2, sentinel), r(m * m + 2, sentinel),
          state{{particles.data() + 1, count, n}, {weights.data() + 1, count, 1},
                {q.data() + 1, n, n}, {r.data() + 1, m, m}, count * n, count, n * n, m * m} {}
    void guards() const {
        for (const auto *v : {&particles, &weights, &q, &r}) { EXPECT_EQ(v->front(), sentinel); EXPECT_EQ(v->back(), sentinel); }
    }
    auto snapshot() const { return std::make_tuple(bytes(state), particles, weights, q, r); }
    std::vector<Vec> particle_rows() const { return decode_rows<A>(state.particles); }
    Vec weight_values() const { return distribution<A>(state.weights); }
};

template<class A> UkfWorkspace<A> pf_init_workspace(std::size_t count, std::size_t n) {
    return UkfWorkspace<A>(A::bits ? FILTERS_PF_Q_INIT_STORAGE(count, n) : FILTERS_PF_PREDICT_STORAGE(count, n),
                           A::bits ? FILTERS_PF_Q_PREDICT_BASIC_BYTES(n) : FILTERS_PF_PREDICT_BASIC_BYTES(n),
                           FILTERS_PF_PREDICT_DECOMP_STORAGE(n),
                           A::bits ? FILTERS_PF_Q_INIT_SIGNAL_STORAGE(count, n) : 0);
}
template<class A> UkfWorkspace<A> pf_predict_workspace(std::size_t count, std::size_t n) {
    return UkfWorkspace<A>(A::bits ? FILTERS_PF_Q_PREDICT_STORAGE(count, n) : FILTERS_PF_PREDICT_STORAGE(count, n),
                           A::bits ? FILTERS_PF_Q_PREDICT_BASIC_BYTES(n) : FILTERS_PF_PREDICT_BASIC_BYTES(n),
                           FILTERS_PF_PREDICT_DECOMP_STORAGE(n),
                           A::bits ? FILTERS_PF_Q_PREDICT_SIGNAL_STORAGE(count, n) : 0);
}
template<class A> UkfWorkspace<A> pf_update_workspace(std::size_t count, std::size_t n, std::size_t m) {
    return UkfWorkspace<A>(A::bits ? FILTERS_PF_Q_UPDATE_STORAGE(count, n, m) : FILTERS_PF_UPDATE_STORAGE(count, m),
                           A::bits ? FILTERS_PF_Q_UPDATE_BASIC_BYTES(m) : FILTERS_PF_UPDATE_BASIC_BYTES(m),
                           A::bits ? FILTERS_PF_Q_UPDATE_DECOMP_STORAGE(m) : FILTERS_PF_UPDATE_DECOMP_STORAGE(m),
                           A::bits ? FILTERS_PF_Q_UPDATE_SIGNAL_STORAGE(count, n, m) : 0);
}
template<class A> UkfWorkspace<A> pf_resample_workspace(std::size_t count) {
    return UkfWorkspace<A>(A::bits ? FILTERS_PF_Q_RESAMPLE_STORAGE(count) : 0, 0, 0,
                           A::bits ? FILTERS_PF_Q_RESAMPLE_SIGNAL_STORAGE(count) : 0);
}
template<class A> UkfWorkspace<A> pf_estimate_workspace(std::size_t count, std::size_t n) {
    return UkfWorkspace<A>(A::bits ? FILTERS_PF_Q_ESTIMATE_STORAGE(count, n) : FILTERS_PF_ESTIMATE_STORAGE(count, n),
                           A::bits ? FILTERS_PF_Q_ESTIMATE_BASIC_BYTES(count, n) : FILTERS_PF_ESTIMATE_BASIC_BYTES(count, n), 0,
                           A::bits ? FILTERS_PF_Q_ESTIMATE_SIGNAL_STORAGE(count, n) : 0);
}

/* Independent long-double bootstrap particle filter driven by logged draws. */
struct PfReference {
    std::vector<Vec> particles;
    Vec weights;
    static EigenMatrix cholesky(const Dense& covariance) {
        const Eigen::LLT<EigenMatrix> llt(eigen_matrix(covariance));
        EXPECT_EQ(llt.info(), Eigen::Success);
        return llt.matrixL();
    }
    static Vec shifted(const Vec& base, const EigenMatrix& L, const Vec& xi) {
        Vec result = base;
        for (std::size_t i = 0; i < result.size(); ++i)
            for (std::size_t j = 0; j < xi.size(); ++j) result[i] += L(static_cast<Eigen::Index>(i), static_cast<Eigen::Index>(j)) * xi[j];
        return result;
    }
    static Vec exponents(const std::vector<Vec>& deviations, const Dense& covariance) {
        const Eigen::LDLT<EigenMatrix> factor(eigen_matrix(covariance));
        Vec result(deviations.size());
        for (std::size_t i = 0; i < deviations.size(); ++i) {
            const EigenMatrix d = Eigen::Map<const EigenMatrix>(deviations[i].data(), static_cast<Eigen::Index>(deviations[i].size()), 1);
            result[i] = 0.5L * (d.transpose() * factor.solve(d))(0, 0);
        }
        return result;
    }
    static Vec reweighted(const Vec& w, const Vec& e) {
        const long double lowest = *std::min_element(e.begin(), e.end());
        Vec result(w.size());
        long double sum = 0;
        for (std::size_t i = 0; i < w.size(); ++i) sum += result[i] = w[i] * std::exp(-(e[i] - lowest));
        for (auto& value : result) value /= sum;
        return result;
    }
    static long double effective_sample_size(const Vec& w) {
        long double sum = 0, squares = 0;
        for (auto value : w) { sum += value; squares += value * value; }
        return sum * sum / squares;
    }
    static std::vector<std::size_t> systematic(const Vec& w, long double uniform) {
        const std::size_t count = w.size();
        long double total = 0;
        for (auto value : w) total += value;
        std::vector<std::size_t> indices(count);
        std::size_t j = 0;
        long double cumulative = w[0];
        for (std::size_t i = 0; i < count; ++i) {
            const long double target = (static_cast<long double>(i) + uniform) / static_cast<long double>(count) * total;
            while (cumulative < target && j + 1 < count) cumulative += w[++j];
            indices[i] = j;
        }
        return indices;
    }
    void init(const Dense& mean, const Dense& covariance, const std::vector<Vec>& draws) {
        const EigenMatrix L = cholesky(covariance);
        particles.resize(draws.size());
        for (std::size_t i = 0; i < draws.size(); ++i) particles[i] = shifted(mean.values, L, draws[i]);
        weights.assign(draws.size(), 1.0L / static_cast<long double>(draws.size()));
    }
    void predict(const Model& f, const Dense& Q, const std::vector<Vec>& draws) {
        const EigenMatrix L = cholesky(Q);
        for (std::size_t i = 0; i < particles.size(); ++i) particles[i] = shifted(f.map(particles[i]), L, draws[i]);
    }
    void update(const Dense& z, const Model& h, const Dense& R) {
        std::vector<Vec> deviations(particles.size());
        for (std::size_t i = 0; i < particles.size(); ++i)
            deviations[i] = UkfReference::residual_of(h, h.map(particles[i]), z.values);
        weights = reweighted(weights, exponents(deviations, R));
    }
    void resample(const std::vector<std::size_t>& indices) {
        std::vector<Vec> gathered(indices.size());
        for (std::size_t i = 0; i < indices.size(); ++i) gathered[i] = particles[indices[i]];
        particles = gathered;
        weights.assign(indices.size(), 1.0L / static_cast<long double>(indices.size()));
    }
    Vec mean(const Model& residual) const {
        Vec result(particles[0].size(), 0);
        for (std::size_t i = 0; i < particles.size(); ++i) {
            const Vec d = residual.residual ? residual.residual(particles[i], particles[0]) : particles[i];
            for (std::size_t j = 0; j < d.size(); ++j) result[j] += weights[i] * d[j];
        }
        if (residual.residual)
            for (std::size_t j = 0; j < result.size(); ++j) result[j] += particles[0][j];
        return result;
    }
    Dense covariance(const Vec& center, const Model& residual) const {
        Dense result(center.size(), center.size());
        for (std::size_t i = 0; i < particles.size(); ++i) {
            const Vec d = UkfReference::residual_of(residual, particles[i], center);
            for (std::size_t r = 0; r < d.size(); ++r)
                for (std::size_t c = 0; c < d.size(); ++c) result(r, c) += weights[i] * d[r] * d[c];
        }
        return result;
    }
};

} // namespace pf_test
#endif
