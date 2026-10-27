/* Allocation-free N32H474 transaction workload. See README-workload.md.
 * The only platform dependencies are bench_cycles(), bench_marker() and the
 * result mailbox. Define HOST_BENCH to execute this exact workload off-board.
 */
#include "bench_platform.h"
#include "bench_result.h"
#include "draws.h"
#include "filters_upf.h"

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <limits>
#ifdef HOST_BENCH
#include <chrono>
#include <cstdio>
#endif

#ifndef BENCH_FORMAT
#error "Select one image format: BENCH_FORMAT=0 (f32), 1 (f64), 2 (q15), 3 (q31)"
#endif

namespace {
constexpr std::size_t max_n = 4;
constexpr std::size_t max_particles = 128;
constexpr std::size_t warmups = 2;
static_assert(BENCH_TRIALS == 9 && BENCH_MAX_CASES == 20, "Frozen result ABI changed");
static_assert(sizeof(double) == 8 && std::numeric_limits<double>::is_iec559,
              "Checksum requires IEEE binary64");

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
#define BENCH_API(format, tag, work) \
template<> struct Api<format> { \
    using Ukf = filters_ukf_##tag##_t; \
    using Pf = filters_pf_##tag##_t; \
    using Upf = filters_upf_##tag##_t; \
    using Model = filters_upf_##work##_model_t; \
    using Workspace = filters_ukf_##tag##_workspace_t; \
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
BENCH_API(F32, f32, f32)
BENCH_API(F64, f64, f64)
BENCH_API(Q15, q15, f32)
BENCH_API(Q31, q31, f64)
#undef BENCH_API

#if BENCH_FORMAT == 0
using F = F32;
#elif BENCH_FORMAT == 1
using F = F64;
#elif BENCH_FORMAT == 2
using F = Q15;
#elif BENCH_FORMAT == 3
using F = Q31;
#else
#error "BENCH_FORMAT must be 0, 1, 2 or 3"
#endif
using R = F::Raw;
using W = F::Work;
using A = Api<F>;

constexpr std::size_t maximum(std::size_t a, std::size_t b) { return a > b ? a : b; }
struct Storage {
    std::size_t data, basic, decomp, signal;
    constexpr void include(Storage other) {
        data = maximum(data, other.data);
        basic = maximum(basic, other.basic);
        decomp = maximum(decomp, other.decomp);
        signal = maximum(signal, other.signal);
    }
    constexpr std::size_t bytes() const {
        return (data + decomp) * sizeof(W)
             + LINEARS_WORKSPACE_UNITS(basic) * sizeof(linears_workspace_unit_t)
             + signal * sizeof(R);
    }
};

/* The same public operation bounds as bench/nonlinear.cc, including init.
 * Capacities passed to the library are the actual case bounds, not max-case
 * bounds. Records therefore describe one logical filter, not this harness.
 */
constexpr Storage storage_for(uint32_t family, std::size_t count, std::size_t n) {
    Storage s{};
    if (family == 0) {
        if constexpr (F::bits == 0) {
            s = {FILTERS_UKF_PREDICT_STORAGE(n), FILTERS_UKF_PREDICT_BASIC_BYTES(n),
                 FILTERS_UKF_PREDICT_DECOMP_STORAGE(n), 0};
            s.include({FILTERS_UKF_UPDATE_STORAGE(n, n), FILTERS_UKF_UPDATE_BASIC_BYTES(n, n),
                       FILTERS_UKF_UPDATE_DECOMP_STORAGE(n, n), 0});
        } else {
            s = {FILTERS_UKF_Q_PREDICT_STORAGE(n), FILTERS_UKF_Q_PREDICT_BASIC_BYTES(n),
                 FILTERS_UKF_Q_PREDICT_DECOMP_STORAGE(n), FILTERS_UKF_Q_PREDICT_SIGNAL_STORAGE(n)};
            s.include({FILTERS_UKF_Q_UPDATE_STORAGE(n, n), FILTERS_UKF_Q_UPDATE_BASIC_BYTES(n, n),
                       FILTERS_UKF_Q_UPDATE_DECOMP_STORAGE(n, n), FILTERS_UKF_Q_UPDATE_SIGNAL_STORAGE(n, n)});
        }
    } else if (family == 1) {
        if constexpr (F::bits == 0) {
            s = {FILTERS_PF_PREDICT_STORAGE(count, n), FILTERS_PF_PREDICT_BASIC_BYTES(n),
                 FILTERS_PF_PREDICT_DECOMP_STORAGE(n), 0};
            s.include({FILTERS_PF_UPDATE_STORAGE(count, n), FILTERS_PF_UPDATE_BASIC_BYTES(n),
                       FILTERS_PF_UPDATE_DECOMP_STORAGE(n), 0});
            s.include({FILTERS_PF_ESTIMATE_STORAGE(count, n), FILTERS_PF_ESTIMATE_BASIC_BYTES(count, n), 0, 0});
        } else {
            s = {FILTERS_PF_Q_INIT_STORAGE(count, n), FILTERS_PF_Q_PREDICT_BASIC_BYTES(n),
                 FILTERS_PF_PREDICT_DECOMP_STORAGE(n), FILTERS_PF_Q_INIT_SIGNAL_STORAGE(count, n)};
            s.include({FILTERS_PF_Q_PREDICT_STORAGE(count, n), FILTERS_PF_Q_PREDICT_BASIC_BYTES(n),
                       FILTERS_PF_PREDICT_DECOMP_STORAGE(n), FILTERS_PF_Q_PREDICT_SIGNAL_STORAGE(count, n)});
            s.include({FILTERS_PF_Q_UPDATE_STORAGE(count, n, n), FILTERS_PF_Q_UPDATE_BASIC_BYTES(n),
                       FILTERS_PF_Q_UPDATE_DECOMP_STORAGE(n), FILTERS_PF_Q_UPDATE_SIGNAL_STORAGE(count, n, n)});
            s.include({FILTERS_PF_Q_ESTIMATE_STORAGE(count, n), FILTERS_PF_Q_ESTIMATE_BASIC_BYTES(count, n),
                       0, FILTERS_PF_Q_ESTIMATE_SIGNAL_STORAGE(count, n)});
        }
    } else {
        if constexpr (F::bits == 0) {
            s = {FILTERS_PF_PREDICT_STORAGE(count, n), FILTERS_PF_PREDICT_BASIC_BYTES(n),
                 FILTERS_PF_PREDICT_DECOMP_STORAGE(n), 0};
            s.include({FILTERS_UPF_STEP_STORAGE(count, n, n), FILTERS_UPF_STEP_BASIC_BYTES(n, n),
                       FILTERS_UPF_STEP_DECOMP_STORAGE(n, n), 0});
        } else {
            s = {FILTERS_PF_Q_INIT_STORAGE(count, n), FILTERS_PF_Q_PREDICT_BASIC_BYTES(n),
                 FILTERS_PF_PREDICT_DECOMP_STORAGE(n), FILTERS_PF_Q_INIT_SIGNAL_STORAGE(count, n)};
            s.include({FILTERS_UPF_Q_STEP_STORAGE(count, n, n), FILTERS_UPF_Q_STEP_BASIC_BYTES(n, n),
                       FILTERS_UPF_Q_STEP_DECOMP_STORAGE(n, n), FILTERS_UPF_Q_STEP_SIGNAL_STORAGE(count, n, n)});
        }
    }
    return s;
}
constexpr Storage maximum_storage() {
    Storage s = storage_for(0, 1, max_n);
    s.include(storage_for(1, max_particles, max_n));
    s.include(storage_for(2, max_particles, max_n));
    return s;
}
constexpr Storage max_storage = maximum_storage();

/* One fixed region, reused for every case. UKF borrows the prefixes of
 * particles/covariances as x/P. No signal array is widened for convenience.
 * The one-element floating-format signal placeholder is not passed to an API.
 */
struct Buffers {
    R particles[max_particles * max_n];
    R weights[max_particles];
    R covariances[max_particles * max_n * max_n];
    R q[max_n * max_n], r[max_n * max_n];
    R initial_particles[max_particles * max_n];
    R initial_weights[max_particles];
    R initial_covariances[max_particles * max_n * max_n];
    R mean[max_n], covariance[max_n * max_n];
    W data[max_storage.data], decomp[max_storage.decomp];
    linears_workspace_unit_t basic[LINEARS_WORKSPACE_UNITS(max_storage.basic)];
    R signal[max_storage.signal ? max_storage.signal : 1];
};
union FilterState {
    A::Ukf ukf;
    A::Pf pf;
    A::Upf upf;
};
struct Inputs {
    R mean[max_n], covariance[max_n * max_n];
    R q[max_n * max_n], r[max_n * max_n], z[max_n];
};
Buffers buffers;
FilterState filter;
Inputs inputs;

struct Draws {
    std::size_t position, length;
    static linears_status_t sample(linears_matrix_t *out, void *context) {
        auto &self = *static_cast<Draws *>(context);
        std::memcpy(out->data, filters_bench::normal_draws<W> + self.position, out->rows * sizeof(W));
        self.position += out->rows;
        if (self.position == self.length) self.position = 0;
        return LINEARS_OK;
    }
};
Draws draws;
constexpr std::size_t harness_static_bytes = sizeof(buffers) + sizeof(filter)
    + sizeof(inputs) + sizeof(draws) + sizeof(bench_results_t);
/* Leave at least 16 KiB of the real 96 KiB RAM for platform data and stack.
 * The linker and stack/canary analysis still own the complete image budget.
 */
static_assert(harness_static_bytes <= 80 * 1024, "Workload leaves less than 16 KiB for platform/stack");
static_assert(filters_bench::normal_draw_count == max_particles * max_n, "Draw table size mismatch");

template<bool Nonlinear> linears_status_t transition(const linears_matrix_t *in,
                                                     linears_matrix_t *out, void *) {
    const auto *x = static_cast<const W *>(in->data);
    auto *y = static_cast<W *>(out->data);
    for (std::size_t j = 0; j < out->rows; ++j) {
        y[j] = W(0.875) * x[j] + W(0.0625) * x[(j + 1) % in->rows];
        if constexpr (Nonlinear) y[j] += W(0.03125) * x[j] * x[j];
    }
    return LINEARS_OK;
}
template<bool Nonlinear> linears_status_t observation(const linears_matrix_t *in,
                                                      linears_matrix_t *out, void *) {
    const auto *x = static_cast<const W *>(in->data);
    auto *y = static_cast<W *>(out->data);
    for (std::size_t j = 0; j < out->rows; ++j) {
        y[j] = W(0.75) * x[j] + W(0.125) * x[(j + 1) % in->rows];
        if constexpr (Nonlinear) y[j] += W(0.0625) * x[j] * x[j];
    }
    return LINEARS_OK;
}

linears_matrix_t matrix(R *data, std::size_t rows, std::size_t cols = 1) {
    return {data, rows, cols};
}
void prepare_inputs(std::size_t n) {
    std::memset(&inputs, 0, sizeof(inputs));
    for (std::size_t j = 0; j < n; ++j) {
        inputs.mean[j] = F::encode(0.0625 + 0.015625 * j);
        inputs.z[j] = F::encode(0.09375 + 0.015625 * j);
        inputs.covariance[j * n + j] = F::encode(0.015625);
        inputs.q[j * n + j] = F::encode(0.00390625);
        inputs.r[j * n + j] = F::encode(0.03125);
    }
}
/* Dependent type keeps the Q-only members outside floating instantiations. */
template<class Format> typename Api<Format>::Workspace workspace_view(Storage s) {
    typename Api<Format>::Workspace ws{};
    ws.data = buffers.data;
    ws.capacity = s.data;
    ws.basic = s.basic ? buffers.basic : nullptr;
    ws.basic_bytes = s.basic;
    ws.decomposition = {buffers.decomp, s.decomp};
    if constexpr (Format::bits != 0) {
        ws.signal = buffers.signal;
        ws.signal_capacity = s.signal;
    }
    return ws;
}

linears_status_t initialize(uint32_t family, std::size_t count, std::size_t n, A::Workspace ws) {
    const auto mean = matrix(inputs.mean, n);
    const auto covariance = matrix(inputs.covariance, n, n);
    const auto q = matrix(inputs.q, n, n);
    const auto r = matrix(inputs.r, n, n);
    if (family == 0) {
        filter.ukf = {matrix(buffers.particles, n), matrix(buffers.covariances, n, n),
                      matrix(buffers.q, n, n), matrix(buffers.r, n, n), n, n*n, n*n, n*n, {}};
        return A::ukf_init(&filter.ukf, 1, 2, 0, &mean, &covariance, &q, &r);
    }
    const A::Pf base{matrix(buffers.particles, count, n), matrix(buffers.weights, count),
                     matrix(buffers.q, n, n), matrix(buffers.r, n, n), count*n, count, n*n, n*n};
    draws = {0, count * n};
    linears_status_t status;
    if (family == 1) {
        filter.pf = base;
        status = A::pf_init(&filter.pf, &mean, &covariance, &q, &r, Draws::sample, &draws, ws);
    } else {
        filter.upf = {base, matrix(buffers.covariances, count, n*n), count*n*n, {}};
        status = A::upf_init(&filter.upf, 1, 2, 0, &mean, &covariance, &q, &r, Draws::sample, &draws, ws);
    }
    if (status != LINEARS_OK) return status;
    /* Match nonlinear.cc: identical q15-grid prior, independently of each
     * format's init rounding. sqrt(P0) = 0.125 is exact in every format.
     */
    for (std::size_t i = 0; i < count; ++i) {
        for (std::size_t j = 0; j < n; ++j) {
            const auto k = i * n + j;
            const double value = F::decode(inputs.mean[j])
                               + 0.125 * static_cast<double>(filters_bench::normal_draws<W>[k]);
            buffers.particles[k] = F::encode(std::round(value * 32768.0) / 32768.0);
        }
    }
    std::memcpy(buffers.initial_particles, buffers.particles, count * n * sizeof(R));
    std::memcpy(buffers.initial_weights, buffers.weights, count * sizeof(R));
    if (family == 2)
        std::memcpy(buffers.initial_covariances, buffers.covariances, count * n * n * sizeof(R));
    return LINEARS_OK;
}
void restore(uint32_t family, std::size_t count, std::size_t n) {
    if (family == 0) {
        std::memcpy(buffers.particles, inputs.mean, n * sizeof(R));
        std::memcpy(buffers.covariances, inputs.covariance, n * n * sizeof(R));
    } else {
        std::memcpy(buffers.particles, buffers.initial_particles, count * n * sizeof(R));
        std::memcpy(buffers.weights, buffers.initial_weights, count * sizeof(R));
        if (family == 2)
            std::memcpy(buffers.covariances, buffers.initial_covariances, count * n * n * sizeof(R));
        else {
            std::memset(buffers.mean, 0, n * sizeof(R));
            std::memset(buffers.covariance, 0, n * n * sizeof(R));
        }
        draws.position = 0;
    }
}

template<bool Nonlinear> linears_status_t transaction(uint32_t family,
                                                      const linears_matrix_t &z,
                                                      linears_matrix_t &mean,
                                                      linears_matrix_t &covariance,
                                                      const A::Model &model,
                                                      A::Workspace ws) {
    if (family == 0) {
        const auto status = A::ukf_predict(&filter.ukf, transition<Nonlinear>, nullptr, nullptr, nullptr, ws);
        if (status != LINEARS_OK) return status;
        return A::ukf_update(&filter.ukf, &z, observation<Nonlinear>, nullptr, nullptr, nullptr, ws);
    }
    if (family == 1) {
        auto status = A::pf_predict(&filter.pf, transition<Nonlinear>, nullptr, Draws::sample, &draws, ws);
        if (status != LINEARS_OK) return status;
        status = A::pf_update(&filter.pf, &z, observation<Nonlinear>, nullptr, nullptr, nullptr, ws);
        if (status != LINEARS_OK) return status;
        return A::pf_estimate(&filter.pf, nullptr, nullptr, &mean, &covariance, ws);
    }
    return A::upf_step(&filter.upf, &z, &model, Draws::sample, &draws, ws);
}

/* FNV-1a includes every changed state/output element, not descriptor padding.
 * N32H474 and supported host oracle machines are little-endian. The numerical
 * checksum separately preserves the host benchmark's sum order and definition.
 */
bool inspect(const R *data, std::size_t count, double &sum, uint32_t &hash) {
    sum = 0;
    for (std::size_t i = 0; i < count; ++i) {
        const double value = F::decode(data[i]);
        if (!std::isfinite(value)) return false;
        sum += value;
    }
    const auto *bytes = reinterpret_cast<const unsigned char *>(data);
    for (std::size_t i = 0; i < count * sizeof(R); ++i) {
        hash ^= bytes[i];
        hash *= UINT32_C(16777619);
    }
    return std::isfinite(sum);
}
linears_status_t validate(uint32_t family, std::size_t count, std::size_t n,
                          double &checksum, uint32_t &hash) {
    hash = UINT32_C(2166136261);
    double first = 0, second = 0, ignored = 0;
    if (family == 0) {
        if (!inspect(buffers.particles, n, first, hash)
            || !inspect(buffers.covariances, n*n, second, hash)) return LINEARS_ERROR_RANGE;
    } else {
        if (!inspect(buffers.particles, count*n, first, hash)
            || !inspect(buffers.weights, count, ignored, hash)) return LINEARS_ERROR_RANGE;
        if (family == 1) {
            if (!inspect(buffers.mean, n, first, hash)
                || !inspect(buffers.covariance, n*n, second, hash)) return LINEARS_ERROR_RANGE;
        } else if (!inspect(buffers.covariances, count*n*n, second, hash)) return LINEARS_ERROR_RANGE;
    }
    checksum = first + second;
    return std::isfinite(checksum) ? LINEARS_OK : LINEARS_ERROR_RANGE;
}

#ifdef HOST_BENCH
void host_output(const bench_case_t &record);
#endif

void publish(volatile bench_case_t &out, const bench_case_t &value) {
    out.family = value.family;
    out.nonlinear = value.nonlinear;
    out.n = value.n;
    out.particles = value.particles;
    out.status = value.status;
    for (std::size_t j = 0; j < BENCH_TRIALS; ++j) out.cycles[j] = value.cycles[j];
    out.persistent_bytes = value.persistent_bytes;
    out.descriptor_bytes = value.descriptor_bytes;
    out.workspace_bytes = value.workspace_bytes;
    out.output_bytes = value.output_bytes;
    out.checksum_lo = value.checksum_lo;
    out.checksum_hi = value.checksum_hi;
    out.output_hash = value.output_hash;
}

template<bool Nonlinear> bool measure(uint32_t family, std::size_t count, std::size_t n) {
    const uint32_t index = g_bench_results.case_count;
    g_bench_results.active_case = index;
    const Storage size = storage_for(family, count, n);
    const auto ws = workspace_view<F>(size);
    prepare_inputs(n);
    auto z = matrix(inputs.z, n);
    auto mean = matrix(buffers.mean, n);
    auto covariance = matrix(buffers.covariance, n, n);
    const A::Model model{transition<Nonlinear>, nullptr, observation<Nonlinear>, nullptr,
                         nullptr, nullptr, nullptr, nullptr};
    bench_case_t record{};
    record.family = family;
    record.nonlinear = Nonlinear;
    record.n = static_cast<uint32_t>(n);
    record.particles = static_cast<uint32_t>(count);
    record.persistent_bytes = static_cast<uint32_t>((family == 0 ? n + 3*n*n
        : count*n + count + (family == 2 ? count*n*n : 0) + 2*n*n) * sizeof(R));
    record.descriptor_bytes = static_cast<uint32_t>(family == 0 ? sizeof(A::Ukf)
                                                   : family == 1 ? sizeof(A::Pf) : sizeof(A::Upf));
    record.workspace_bytes = static_cast<uint32_t>(size.bytes());
    record.output_bytes = static_cast<uint32_t>(family == 1 ? (n + n*n) * sizeof(R) : 0);
    publish(g_bench_results.cases[index], record);
    auto status = initialize(family, count, n, ws);
    uint32_t reference_hash = 0;
    uint64_t reference_checksum = 0;
    for (std::size_t trial = 0; status == LINEARS_OK && trial < warmups + BENCH_TRIALS; ++trial) {
        restore(family, count, n);
        const uint32_t channel = family * 4 + BENCH_FORMAT;
        bench_marker(channel, 1);
        const uint32_t before = bench_cycles();
        status = transaction<Nonlinear>(family, z, mean, covariance, model, ws);
        const uint32_t after = bench_cycles();
        bench_marker(channel, 0);
        if (status != LINEARS_OK) break;
        double checksum = 0;
        uint32_t hash = 0;
        status = validate(family, count, n, checksum, hash);
        if (status != LINEARS_OK) break;
        uint64_t checksum_bits;
        std::memcpy(&checksum_bits, &checksum, sizeof(checksum_bits));
        if (trial == 0) {
            reference_hash = hash;
            reference_checksum = checksum_bits;
        } else if (hash != reference_hash || checksum_bits != reference_checksum) {
            status = LINEARS_ERROR_NO_CONVERGENCE;
            break;
        }
        if (trial >= warmups) record.cycles[trial - warmups] = after - before;
    }
    record.status = static_cast<int32_t>(status);
    if (status == LINEARS_OK) {
        record.checksum_lo = static_cast<uint32_t>(reference_checksum);
        record.checksum_hi = static_cast<uint32_t>(reference_checksum >> 32);
        record.output_hash = reference_hash;
#ifdef HOST_BENCH
        host_output(record);
#endif
    } else {
        /* An invalid case must not be consumed as a timing sample, including
         * any earlier successful trials of that same case.
         */
        for (auto &cycles : record.cycles) cycles = 0;
    }
    publish(g_bench_results.cases[index], record);
    g_bench_results.case_count = index + 1;
    if (status != LINEARS_OK) {
        g_bench_results.phase = 4;
        return false;
    }
    return true;
}
template<bool Nonlinear> bool model_cases() {
    for (std::size_t n = 2; n <= 4; n += 2) {
        if (!measure<Nonlinear>(0, 1, n)) return false;
        for (std::size_t count = 32; count <= 128; count *= 4) {
            if (!measure<Nonlinear>(1, count, n) || !measure<Nonlinear>(2, count, n)) return false;
        }
    }
    return true;
}
} // namespace

extern "C" void bench_run(void) {
    g_bench_results.phase = 2;
    g_bench_results.magic = BENCH_MAGIC;
    g_bench_results.version = 1;
    g_bench_results.format = BENCH_FORMAT;
    g_bench_results.case_count = 0;
    g_bench_results.active_case = 0;
    const bench_case_t empty{};
    for (auto &record : g_bench_results.cases) publish(record, empty);
    const uint32_t before = bench_cycles();
    const uint32_t after = bench_cycles();
    g_bench_results.cycle_read_overhead = after - before;
    /* Clock, CPACR and fault registers belong to the platform and are retained. */
    if (!model_cases<false>() || !model_cases<true>()) return;
    g_bench_results.phase = 3;
}

#ifdef HOST_BENCH
namespace {
uint32_t host_active_marker = UINT32_MAX;
uint32_t host_marker_pairs = 0;
bool host_marker_error = false;
std::FILE *host_dump = nullptr;
void host_array(const bench_case_t &r, const char *name, const R *values, std::size_t count) {
    for (std::size_t i = 0; i < count; ++i) {
        uint64_t bits = 0;
        std::memcpy(&bits, values + i, sizeof(R));
        std::fprintf(host_dump, "%u,%u,%u,%u,%u,%s,%zu,%016llx,%.17g\n",
                     BENCH_FORMAT, r.nonlinear, r.family, r.n, r.particles, name, i,
                     static_cast<unsigned long long>(bits), F::decode(values[i]));
    }
}
void host_output(const bench_case_t &r) {
    if (!host_dump) return;
    if (r.family == 0) {
        host_array(r, "estimate", buffers.particles, r.n);
        host_array(r, "covariance", buffers.covariances, r.n * r.n);
    } else {
        host_array(r, "particles", buffers.particles, r.particles * r.n);
        host_array(r, "weights", buffers.weights, r.particles);
        if (r.family == 1) {
            host_array(r, "estimate", buffers.mean, r.n);
            host_array(r, "covariance", buffers.covariance, r.n * r.n);
        } else host_array(r, "covariances", buffers.covariances, r.particles * r.n * r.n);
    }
}
}
extern "C" {
volatile bench_results_t g_bench_results{};
uint32_t bench_cycles(void) {
    return static_cast<uint32_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count());
}
void bench_marker(uint32_t channel, uint32_t high) {
    if (channel >= 12 || channel % 4 != BENCH_FORMAT) host_marker_error = true;
    if (high) {
        if (host_active_marker != UINT32_MAX) host_marker_error = true;
        host_active_marker = channel;
    } else {
        if (host_active_marker != channel) host_marker_error = true;
        host_active_marker = UINT32_MAX;
        ++host_marker_pairs;
    }
}
}
int main(int argc, char **argv) {
    /* HOST_BENCH times are host nanoseconds, never board DWT observations. */
    const uint16_t endian = 1;
    if (*reinterpret_cast<const unsigned char *>(&endian) != 1) {
        std::fputs("HOST_BENCH raw-output oracle requires a little-endian host\n", stderr);
        return 1;
    }
    if (argc == 2) {
        host_dump = std::fopen(argv[1], "w");
        if (!host_dump) { std::perror("HOST_BENCH output file"); return 1; }
        std::fputs("format,nonlinear,family,n,particles,array,index,raw_hex,decoded\n", host_dump);
    } else if (argc != 1) {
        std::fputs("usage: host_bench [full-output.csv]\n", stderr);
        return 1;
    }
    g_bench_results.core_hz = 1000000000;
    uint32_t previous_hash[BENCH_MAX_CASES]{};
    uint64_t previous_checksum[BENCH_MAX_CASES]{};
    std::printf("# harness_static_bytes=%zu; HOST_BENCH, not MCU timing\n", harness_static_bytes);
    std::puts("format,nonlinear,family,n,particles,persistent_bytes,descriptor_bytes,workspace_bytes,output_bytes,checksum,checksum_hex,output_hash");
    for (unsigned repetition = 0; repetition < 2; ++repetition) {
        host_marker_pairs = 0;
        bench_run();
        if (host_dump) {
            const bool failed = std::ferror(host_dump) != 0;
            const int closed = std::fclose(host_dump);
            host_dump = nullptr;
            if (failed || closed != 0) { std::fputs("HOST_BENCH output write failed\n", stderr); return 1; }
        }
        if (g_bench_results.phase != 3 || g_bench_results.case_count != BENCH_MAX_CASES
            || host_marker_error || host_active_marker != UINT32_MAX
            || host_marker_pairs != BENCH_MAX_CASES * (warmups + BENCH_TRIALS)) {
            std::fprintf(stderr, "HOST_BENCH failed: phase=%u active=%u status=%d pairs=%u\n",
                         g_bench_results.phase, g_bench_results.active_case,
                         g_bench_results.cases[g_bench_results.active_case].status, host_marker_pairs);
            return 1;
        }
        for (std::size_t i = 0; i < BENCH_MAX_CASES; ++i) {
            const auto &r = g_bench_results.cases[i];
            const uint64_t bits = (uint64_t(r.checksum_hi) << 32) | r.checksum_lo;
            if (repetition) {
                if (r.output_hash != previous_hash[i] || bits != previous_checksum[i]) {
                    std::fprintf(stderr, "HOST_BENCH rerun differs at case %zu\n", i);
                    return 1;
                }
            } else {
                previous_hash[i] = r.output_hash;
                previous_checksum[i] = bits;
                double checksum;
                std::memcpy(&checksum, &bits, sizeof(checksum));
                std::printf("%u,%u,%u,%u,%u,%u,%u,%u,%u,%.17g,%08x%08x,%08x\n",
                            g_bench_results.format, r.nonlinear, r.family, r.n, r.particles,
                            r.persistent_bytes, r.descriptor_bytes, r.workspace_bytes, r.output_bytes,
                            checksum, r.checksum_hi, r.checksum_lo, r.output_hash);
            }
        }
    }
    return 0;
}
#endif
