/* Reproducible nonlinear-filter transaction benchmark; no test-framework callbacks.
 * Build/run via run.py. Inputs are on the q15 grid for all four formats.
 * Each timed batch processes independent copies of the same initial state;
 * restoration, allocation, initialization and validation are outside timing.
 * The timed native callbacks only implement a small coupled linear map or copy
 * precomputed Gaussian draws. Their cost is included equally, not subtracted.
 * Define FILTERS_BENCH_NONLINEAR to add a small quadratic term to f and h.
 * This is a host latency measurement, not a proof of performance on every CPU.
 */
#include "filters_upf.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <limits>
#include <memory>
#include <random>
#include <string>
#include <vector>

namespace {
constexpr std::size_t lanes = 16;
constexpr std::size_t trials = 9;
constexpr std::size_t batches = 12;
volatile double sink = 0;

[[noreturn]] void fail(const char *operation, int status) {
    std::fprintf(stderr, "benchmark failure: %s status=%d\n", operation, status);
    std::exit(1);
}
void checked(linears_status_t status, const char *operation) {
    if (status != LINEARS_OK) fail(operation, static_cast<int>(status));
}

template<class R, class W, int B> struct Format {
    using Raw = R;
    using Work = W;
    static constexpr int bits = B;
    static Raw encode(double value) {
        if constexpr (bits == 0) return static_cast<Raw>(value);
        else return static_cast<Raw>(std::llround(std::ldexp(value, bits)));
    }
    static double decode(Raw value) {
        if constexpr (bits == 0) return value;
        else return std::ldexp(static_cast<double>(value), -bits);
    }
};
struct F32 : Format<float, float, 0> {};
struct F64 : Format<double, double, 0> {};
struct Q15 : Format<int16_t, float, 15> {};
struct Q31 : Format<int32_t, double, 31> {};

template<class F> struct Api;
#define BENCH_API(format, tag, work, scratch) \
template<> struct Api<format> { \
    static constexpr const char *name = #tag; \
    using Ukf = filters_ukf_##tag##_t; \
    using Pf = filters_pf_##tag##_t; \
    using Upf = filters_upf_##tag##_t; \
    using Model = filters_upf_##work##_model_t; \
    using Workspace = filters_ukf_##scratch##_workspace_t; \
    static constexpr auto ukf_init = filters_ukf_##tag##_init; \
    static constexpr auto ukf_predict = filters_ukf_##tag##_predict; \
    static constexpr auto ukf_update = filters_ukf_##tag##_update; \
    static constexpr auto pf_init = filters_pf_##tag##_init; \
    static constexpr auto pf_predict = filters_pf_##tag##_predict; \
    static constexpr auto pf_update = filters_pf_##tag##_update; \
    static constexpr auto pf_estimate = filters_pf_##tag##_estimate; \
    static constexpr auto upf_init = filters_upf_##tag##_init; \
    static constexpr auto upf_step = filters_upf_##tag##_step; \
};
BENCH_API(F32, f32, f32, f32)
BENCH_API(F64, f64, f64, f64)
#if defined(FILTERS_UKF_Q_PREDICT_SIGNAL_STORAGE)
BENCH_API(Q15, q15, f32, q15)
BENCH_API(Q31, q31, f64, q31)
#else
/* Keep the same driver usable against the recorded floating-wrapper revision. */
BENCH_API(Q15, q15, f32, f32)
BENCH_API(Q31, q31, f64, f64)
#endif
#undef BENCH_API

/* Exact powers of two keep callback arithmetic identical across paired types. */
template<class W> linears_status_t transition(const linears_matrix_t *in,
                                             linears_matrix_t *out, void *) {
    const auto *x = static_cast<const W *>(in->data);
    auto *y = static_cast<W *>(out->data);
    for (std::size_t j = 0; j < out->rows; ++j) {
        y[j] = W(0.875) * x[j] + W(0.0625) * x[(j + 1) % in->rows];
#if defined(FILTERS_BENCH_NONLINEAR)
        y[j] += W(0.03125) * x[j] * x[j];
#endif
    }
    return LINEARS_OK;
}
template<class W> linears_status_t observation(const linears_matrix_t *in,
                                              linears_matrix_t *out, void *) {
    const auto *x = static_cast<const W *>(in->data);
    auto *y = static_cast<W *>(out->data);
    for (std::size_t j = 0; j < out->rows; ++j) {
        y[j] = W(0.75) * x[j] + W(0.125) * x[(j + 1) % in->rows];
#if defined(FILTERS_BENCH_NONLINEAR)
        y[j] += W(0.0625) * x[j] * x[j];
#endif
    }
    return LINEARS_OK;
}

template<class F> struct Draws {
    using W = typename F::Work;
    std::vector<W> values;
    std::size_t position = 0;
    explicit Draws(std::size_t count) : values(count) {
        std::mt19937 random(0x51f15u);
        std::normal_distribution<double> normal(0.0, 1.0);
        for (auto& value : values)
            value = static_cast<W>(std::round(normal(random) * 32768.0) / 32768.0);
    }
    static linears_status_t sample(linears_matrix_t *out, void *context) {
        auto& self = *static_cast<Draws *>(context);
        std::memcpy(out->data, self.values.data() + self.position, out->rows * sizeof(W));
        self.position += out->rows;
        if (self.position == self.values.size()) self.position = 0;
        return LINEARS_OK;
    }
};

template<class F> struct Matrix {
    using R = typename F::Raw;
    std::vector<R> data;
    linears_matrix_t view;
    Matrix(std::size_t rows, std::size_t cols)
        : data(rows * cols), view{data.data(), rows, cols} {}
    Matrix(const Matrix&) = delete;
    Matrix& operator=(const Matrix&) = delete;
    void diagonal(double value) {
        for (std::size_t i = 0; i < view.rows; ++i) data[i * view.cols + i] = F::encode(value);
    }
    double sum() const {
        double result = 0;
        for (auto value : data) result += F::decode(value);
        return result;
    }
};

struct Storage {
    std::size_t data, basic, decomp;
    std::size_t signal = 0;
    void include(Storage other) {
        data = std::max(data, other.data);
        basic = std::max(basic, other.basic);
        decomp = std::max(decomp, other.decomp);
        signal = std::max(signal, other.signal);
    }
};
template<class F> Storage ukf_storage(std::size_t n) {
    Storage result{F::bits ? FILTERS_UKF_Q_PREDICT_STORAGE(n) : FILTERS_UKF_PREDICT_STORAGE(n),
                   FILTERS_UKF_PREDICT_BASIC_BYTES(n), FILTERS_UKF_PREDICT_DECOMP_STORAGE(n)};
    result.include({F::bits ? FILTERS_UKF_Q_UPDATE_STORAGE(n, n) : FILTERS_UKF_UPDATE_STORAGE(n, n),
                    FILTERS_UKF_UPDATE_BASIC_BYTES(n, n), FILTERS_UKF_UPDATE_DECOMP_STORAGE(n, n)});
#if defined(FILTERS_UKF_Q_PREDICT_SIGNAL_STORAGE)
    if constexpr (F::bits != 0) {
        result.signal = std::max(FILTERS_UKF_Q_PREDICT_SIGNAL_STORAGE(n), FILTERS_UKF_Q_UPDATE_SIGNAL_STORAGE(n, n));
        result.basic = std::max(FILTERS_UKF_Q_PREDICT_BASIC_BYTES(n), FILTERS_UKF_Q_UPDATE_BASIC_BYTES(n, n));
        result.decomp = std::max(FILTERS_UKF_Q_PREDICT_DECOMP_STORAGE(n), FILTERS_UKF_Q_UPDATE_DECOMP_STORAGE(n, n));
    }
#endif
    return result;
}
template<class F> Storage pf_storage(std::size_t count, std::size_t n) {
    Storage result{F::bits ? FILTERS_PF_Q_INIT_STORAGE(count, n) : FILTERS_PF_PREDICT_STORAGE(count, n),
                   FILTERS_PF_PREDICT_BASIC_BYTES(n), FILTERS_PF_PREDICT_DECOMP_STORAGE(n)};
    result.include({F::bits ? FILTERS_PF_Q_PREDICT_STORAGE(count, n) : FILTERS_PF_PREDICT_STORAGE(count, n),
                    FILTERS_PF_PREDICT_BASIC_BYTES(n), FILTERS_PF_PREDICT_DECOMP_STORAGE(n)});
    result.include({F::bits ? FILTERS_PF_Q_UPDATE_STORAGE(count, n, n) : FILTERS_PF_UPDATE_STORAGE(count, n),
                    FILTERS_PF_UPDATE_BASIC_BYTES(n), FILTERS_PF_UPDATE_DECOMP_STORAGE(n)});
    result.include({F::bits ? FILTERS_PF_Q_ESTIMATE_STORAGE(count, n) : FILTERS_PF_ESTIMATE_STORAGE(count, n),
                    FILTERS_PF_ESTIMATE_BASIC_BYTES(count, n), 0});
#if defined(FILTERS_PF_Q_PREDICT_SIGNAL_STORAGE)
    if constexpr (F::bits != 0) {
        result.signal = std::max({FILTERS_PF_Q_INIT_SIGNAL_STORAGE(count, n),
                                  FILTERS_PF_Q_PREDICT_SIGNAL_STORAGE(count, n),
                                  FILTERS_PF_Q_UPDATE_SIGNAL_STORAGE(count, n, n),
                                  FILTERS_PF_Q_ESTIMATE_SIGNAL_STORAGE(count, n)});
        result.basic = std::max({FILTERS_PF_Q_PREDICT_BASIC_BYTES(n), FILTERS_PF_Q_UPDATE_BASIC_BYTES(n),
                                 FILTERS_PF_Q_ESTIMATE_BASIC_BYTES(count, n)});
        result.decomp = std::max(FILTERS_PF_PREDICT_DECOMP_STORAGE(n), FILTERS_PF_Q_UPDATE_DECOMP_STORAGE(n));
    }
#endif
    return result;
}
template<class F> Storage upf_storage(std::size_t count, std::size_t n) {
    Storage result{F::bits ? FILTERS_PF_Q_INIT_STORAGE(count, n) : FILTERS_PF_PREDICT_STORAGE(count, n),
                   FILTERS_PF_PREDICT_BASIC_BYTES(n), FILTERS_PF_PREDICT_DECOMP_STORAGE(n)};
    result.include({F::bits ? FILTERS_UPF_Q_STEP_STORAGE(count, n, n) : FILTERS_UPF_STEP_STORAGE(count, n, n),
                    FILTERS_UPF_STEP_BASIC_BYTES(n, n), FILTERS_UPF_STEP_DECOMP_STORAGE(n, n)});
#if defined(FILTERS_UPF_Q_STEP_SIGNAL_STORAGE)
    if constexpr (F::bits != 0) {
        result.signal = std::max(FILTERS_PF_Q_INIT_SIGNAL_STORAGE(count, n), FILTERS_UPF_Q_STEP_SIGNAL_STORAGE(count, n, n));
        result.basic = FILTERS_UPF_Q_STEP_BASIC_BYTES(n, n);
        result.decomp = FILTERS_UPF_Q_STEP_DECOMP_STORAGE(n, n);
    }
#endif
    return result;
}

template<class F> struct Workspace {
    using W = typename F::Work;
    std::vector<W> data, decomp;
    std::vector<linears_workspace_unit_t> basic;
    std::vector<typename F::Raw> signal;
    typename Api<F>::Workspace view;
    explicit Workspace(Storage size)
        : data(size.data), decomp(size.decomp), basic(LINEARS_WORKSPACE_UNITS(size.basic)), signal(size.signal), view{} {
        view.data = data.data();
        view.capacity = data.size();
        view.basic = basic.data();
        view.basic_bytes = size.basic;
        view.decomposition = {decomp.data(), decomp.size()};
#if defined(FILTERS_UKF_Q_PREDICT_SIGNAL_STORAGE)
        if constexpr (F::bits != 0) {
            view.signal = signal.data();
            view.signal_capacity = signal.size();
        }
#endif
    }
    std::size_t bytes() const {
        return (data.size() + decomp.size()) * sizeof(W) + basic.size() * sizeof(linears_workspace_unit_t)
             + signal.size() * sizeof(typename F::Raw);
    }
};

template<class F> struct Inputs {
    Matrix<F> mean, covariance, q, r, z;
    explicit Inputs(std::size_t n) : mean(n, 1), covariance(n, n), q(n, n), r(n, n), z(n, 1) {
        for (std::size_t j = 0; j < n; ++j) {
            mean.data[j] = F::encode(0.0625 + 0.015625 * j);
            z.data[j] = F::encode(0.09375 + 0.015625 * j);
        }
        covariance.diagonal(0.015625);
        q.diagonal(0.00390625);
        r.diagonal(0.03125);
    }
};

template<class F> struct UkfLane {
    using A = Api<F>;
    Matrix<F> x, p, q, r;
    typename A::Ukf state;
    explicit UkfLane(std::size_t n, const Inputs<F>& input)
        : x(n, 1), p(n, n), q(n, n), r(n, n),
          state{x.view, p.view, q.view, r.view, n, n * n, n * n, n * n, {}} {
        checked(A::ukf_init(&state, 1, 2, 0, &input.mean.view, &input.covariance.view,
                            &input.q.view, &input.r.view), "ukf_init");
    }
    void reset(const Inputs<F>& input) {
        std::copy(input.mean.data.begin(), input.mean.data.end(), x.data.begin());
        std::copy(input.covariance.data.begin(), input.covariance.data.end(), p.data.begin());
    }
    linears_status_t run(const Inputs<F>& input, Workspace<F>& workspace) {
        auto status = A::ukf_predict(&state, transition<typename F::Work>, nullptr, nullptr, nullptr, workspace.view);
        if (status != LINEARS_OK) return status;
        return A::ukf_update(&state, &input.z.view, observation<typename F::Work>, nullptr,
                             nullptr, nullptr, workspace.view);
    }
    double checksum() const { return x.sum() + p.sum(); }
    std::size_t persistent_bytes() const { return (x.data.size() + p.data.size() + q.data.size() + r.data.size()) * sizeof(typename F::Raw); }
    std::size_t output_bytes() const { return 0; }
    std::size_t descriptor_bytes() const { return sizeof(state); }
};

template<class F, bool Unscented> struct ParticleLane {
    using A = Api<F>;
    using State = std::conditional_t<Unscented, typename A::Upf, typename A::Pf>;
    using R = typename F::Raw;
    Matrix<F> particles, weights, covariances, q, r, mean, covariance;
    std::vector<R> initial_particles, initial_weights, initial_covariances;
    State state{};
    Draws<F> draws;
    typename A::Model model{transition<typename F::Work>, nullptr,
                             observation<typename F::Work>, nullptr, nullptr, nullptr, nullptr, nullptr};
    ParticleLane(std::size_t count, std::size_t n, const Inputs<F>& input, Workspace<F>& workspace)
        : particles(count, n), weights(count, 1), covariances(Unscented ? count : 0, n * n),
          q(n, n), r(n, n), mean(n, 1), covariance(n, n), draws(count * n) {
        typename A::Pf base{particles.view, weights.view, q.view, r.view,
                             count * n, count, n * n, n * n};
        if constexpr (Unscented) {
            state = {base, covariances.view, count * n * n, {}};
            checked(A::upf_init(&state, 1, 2, 0, &input.mean.view, &input.covariance.view,
                                &input.q.view, &input.r.view, Draws<F>::sample, &draws, workspace.view), "upf_init");
        } else {
            state = base;
            checked(A::pf_init(&state, &input.mean.view, &input.covariance.view, &input.q.view,
                               &input.r.view, Draws<F>::sample, &draws, workspace.view), "pf_init");
        }
        /* Reconstruct the same q15-grid Gaussian prior independently of init
         * arithmetic; sqrt(0.015625) is exactly 0.125 in every format. */
        for (std::size_t i = 0; i < count; ++i) {
            for (std::size_t j = 0; j < n; ++j) {
                const auto k = i * n + j;
                const double value = F::decode(input.mean.data[j]) + 0.125 * static_cast<double>(draws.values[k]);
                particles.data[k] = F::encode(std::round(value * 32768.0) / 32768.0);
            }
        }
        initial_particles = particles.data;
        initial_weights = weights.data;
        initial_covariances = covariances.data;
    }
    void reset(const Inputs<F>&) {
        std::copy(initial_particles.begin(), initial_particles.end(), particles.data.begin());
        std::copy(initial_weights.begin(), initial_weights.end(), weights.data.begin());
        std::copy(initial_covariances.begin(), initial_covariances.end(), covariances.data.begin());
        draws.position = 0;
    }
    linears_status_t run(const Inputs<F>& input, Workspace<F>& workspace) {
        if constexpr (Unscented) return A::upf_step(&state, &input.z.view, &model, Draws<F>::sample, &draws, workspace.view);
        else {
            auto status = A::pf_predict(&state, transition<typename F::Work>, nullptr, Draws<F>::sample, &draws, workspace.view);
            if (status != LINEARS_OK) return status;
            status = A::pf_update(&state, &input.z.view, observation<typename F::Work>, nullptr, nullptr, nullptr, workspace.view);
            if (status != LINEARS_OK) return status;
            return A::pf_estimate(&state, nullptr, nullptr, &mean.view, &covariance.view, workspace.view);
        }
    }
    double checksum() const {
        if constexpr (Unscented) return particles.sum() + covariances.sum();
        else return mean.sum() + covariance.sum();
    }
    std::size_t persistent_bytes() const {
        return (particles.data.size() + weights.data.size() + covariances.data.size() + q.data.size() + r.data.size()) * sizeof(R);
    }
    std::size_t output_bytes() const { return Unscented ? 0 : (mean.data.size() + covariance.data.size()) * sizeof(R); }
    std::size_t descriptor_bytes() const { return sizeof(state); }
};

template<class F, class Lane> void measure(const char *family, std::size_t count, std::size_t n,
                                         const Inputs<F>& input, Workspace<F>& workspace,
                                         std::vector<std::unique_ptr<Lane>>& states) {
    using Clock = std::chrono::steady_clock;
    std::vector<double> durations;
    for (std::size_t trial = 0; trial < trials + 2; ++trial) {
        double elapsed = 0;
        for (std::size_t batch = 0; batch < batches; ++batch) {
            for (auto& state : states) state->reset(input);
            int error = 0;
            const auto before = Clock::now();
            for (auto& state : states) error |= static_cast<int>(state->run(input, workspace));
            const auto after = Clock::now();
            if (error) {
                std::fprintf(stderr, "case: %s %s N=%zu n=m=%zu\n", family, Api<F>::name, count, n);
                fail(family, error);
            }
            elapsed += std::chrono::duration<double, std::nano>(after - before).count();
        }
        if (trial >= 2) durations.push_back(elapsed / (batches * lanes));
    }
    std::sort(durations.begin(), durations.end());
    double checksum = states.front()->checksum();
    if (!std::isfinite(checksum)) fail("nonfinite output", -1);
    for (auto& state : states) if (state->checksum() != checksum) fail("non-reproducible output", -1);
    sink = sink + checksum;
    const auto& state = *states.front();
    const auto total = state.persistent_bytes() + state.output_bytes() + state.descriptor_bytes() + workspace.bytes();
    std::printf("%s,%s,%zu,%zu,%zu,%zu,%zu,%zu,%zu,%zu,%.2f,%.2f,%.2f,%.12g\n",
                family, Api<F>::name, count, n, n, state.persistent_bytes(), state.descriptor_bytes(),
                workspace.bytes(), state.output_bytes(), total, durations[durations.size() / 2],
                durations.front(), durations.back(), checksum);
    std::fflush(stdout);
}

template<class F> void ukf(std::size_t n) {
    Inputs<F> input(n);
    Workspace<F> workspace(ukf_storage<F>(n));
    std::vector<std::unique_ptr<UkfLane<F>>> states;
    for (std::size_t i = 0; i < lanes; ++i) states.emplace_back(std::make_unique<UkfLane<F>>(n, input));
    measure<F>("ukf_predict_update", 1, n, input, workspace, states);
}
template<class F, bool Unscented> void particles(std::size_t count, std::size_t n) {
    Inputs<F> input(n);
    Workspace<F> workspace(Unscented ? upf_storage<F>(count, n) : pf_storage<F>(count, n));
    std::vector<std::unique_ptr<ParticleLane<F, Unscented>>> states;
    for (std::size_t i = 0; i < lanes; ++i)
        states.emplace_back(std::make_unique<ParticleLane<F, Unscented>>(count, n, input, workspace));
    measure<F>(Unscented ? "upf_step" : "pf_predict_update_estimate", count, n, input, workspace, states);
}
} // namespace

int main() {
    std::puts("family,format,N,n,m,persistent_bytes,state_descriptor_bytes,workspace_bytes,output_bytes,total_bytes,median_ns,min_ns,max_ns,checksum");
    for (std::size_t n : {2u, 4u}) {
        ukf<F32>(n); ukf<Q15>(n); ukf<F64>(n); ukf<Q31>(n);
        for (std::size_t count : {32u, 128u}) {
            particles<F32, false>(count, n); particles<Q15, false>(count, n);
            particles<F64, false>(count, n); particles<Q31, false>(count, n);
            particles<F32, true>(count, n); particles<Q15, true>(count, n);
            particles<F64, true>(count, n); particles<Q31, true>(count, n);
        }
    }
    return 0;
}
