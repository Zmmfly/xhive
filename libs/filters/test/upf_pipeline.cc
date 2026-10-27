#include "pf_support.h"
#include "filters_upf.h"

#include <string>

namespace {
using namespace pf_test;

/* The UPF adapters mirror upf.cc; this file locks the proposal loop's
 * callback order, its validation order and the wide-measurement shapes. */
template<class A> struct Upf;
#define UPF_PIPELINE_API(tag, kernel) \
template<> struct Upf<tag> { \
    using State = filters_upf_##tag##_t; \
    using Model = filters_upf_##kernel##_model_t; \
    static constexpr auto init = filters_upf_##tag##_init; \
    static constexpr auto step = filters_upf_##tag##_step; \
};
UPF_PIPELINE_API(f32, f32)
UPF_PIPELINE_API(f64, f64)
UPF_PIPELINE_API(q15, f32)
UPF_PIPELINE_API(q31, f64)
#undef UPF_PIPELINE_API

template<class A> struct UpfState {
    using Raw = typename A::Raw;
    static constexpr Raw sentinel = static_cast<Raw>(-12345);
    std::vector<Raw> particles, weights, covariances, q, r;
    typename Upf<A>::State state;
    UpfState(std::size_t count, std::size_t n, std::size_t m)
        : particles(count * n + 2, sentinel), weights(count + 2, sentinel),
          covariances(count * n * n + 2, sentinel), q(n * n + 2, sentinel), r(m * m + 2, sentinel),
          state{{{particles.data() + 1, count, n}, {weights.data() + 1, count, 1},
                 {q.data() + 1, n, n}, {r.data() + 1, m, m}, count * n, count, n * n, m * m},
                {covariances.data() + 1, count, n * n}, count * n * n, {}} {}
    void guards() const {
        for (const auto *v : {&particles, &weights, &covariances, &q, &r}) {
            EXPECT_EQ(v->front(), sentinel); EXPECT_EQ(v->back(), sentinel);
        }
    }
    auto snapshot() const { return std::make_tuple(bytes(state), particles, weights, covariances, q, r); }
    std::vector<Vec> particle_rows() const { return decode_rows<A>(state.base.particles); }
    Vec weight_values() const { return distribution<A>(state.base.weights); }
};

template<class A> UkfWorkspace<A> upf_step_workspace(std::size_t count, std::size_t n, std::size_t m) {
    return UkfWorkspace<A>(A::bits ? FILTERS_UPF_Q_STEP_STORAGE(count, n, m) : FILTERS_UPF_STEP_STORAGE(count, n, m),
                           A::bits ? FILTERS_UPF_Q_STEP_BASIC_BYTES(n, m) : FILTERS_UPF_STEP_BASIC_BYTES(n, m),
                           A::bits ? FILTERS_UPF_Q_STEP_DECOMP_STORAGE(n, m) : FILTERS_UPF_STEP_DECOMP_STORAGE(n, m),
                           A::bits ? FILTERS_UPF_Q_STEP_SIGNAL_STORAGE(count, n, m) : 0);
}

/* Every callback appends one letter to a shared log: f/h for the maps, r/m
 * for the state and measurement residuals, S for the sampler. */
struct Log {
    std::string events;
};
template<class A> struct Logged {
    Model model;
    Log* log;
    char tag;
};
template<class A> linears_status_t logged_map(const linears_matrix_t* in, linears_matrix_t* out, void* context) {
    auto* logged = static_cast<Logged<A>*>(context);
    logged->log->events.push_back(logged->tag);
    return map_callback<A>(in, out, &logged->model);
}
template<class A> linears_status_t logged_residual(const linears_matrix_t* a, const linears_matrix_t* b,
                                                   linears_matrix_t* out, void* context) {
    auto* logged = static_cast<Logged<A>*>(context);
    logged->log->events.push_back(logged->tag);
    return residual_callback<A>(a, b, out, &logged->model);
}
struct LoggedSampler {
    Sampler sampler;
    Log* log;
};
template<class A> linears_status_t logged_sample(linears_matrix_t* out, void* context) {
    auto* logged = static_cast<LoggedSampler*>(context);
    logged->log->events.push_back('S');
    return sample_callback<A>(out, &logged->sampler);
}

Model pendulum_transition() {
    return Model{[](const Vec& s) { return Vec{s[0] + 0.1L * s[1], s[1] - 0.05L * std::sin(s[0])}; }};
}
Model pendulum_observation(std::size_t m) {
    return Model{[m](const Vec& s) {
        Vec y(m);
        for (std::size_t j = 0; j < m; ++j) y[j] = j % 2 == 0 ? std::sin(s[0]) * (1 + 0.25L * j) : s[1] * (1 - 0.1L * j);
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

template<class A> class UnscentedParticlePipeline : public ::testing::Test {};
TYPED_TEST_SUITE(UnscentedParticlePipeline, Formats);

TYPED_TEST(UnscentedParticlePipeline, StepCallsOneProposalThenOneWeightPerParticle) {
    using A = TypeParam;
    const std::size_t count = 3, n = 2;
    for (std::size_t m : {2u, 5u}) {
        for (int residuals = 0; residuals < 4; ++residuals) {
            SCOPED_TRACE(::testing::Message() << "m=" << m << " residuals=" << residuals);
            const bool state_residual = residuals & 1, measurement_residual = residuals & 2;
            const std::size_t s = 2 * n + 1;
            Log log;
            Logged<A> transition{pendulum_transition(), &log, 'f'}, observation{pendulum_observation(m), &log, 'h'};
            Logged<A> state{subtraction_model(), &log, 'r'}, measurement{subtraction_model(), &log, 'm'};
            LoggedSampler sampler{Sampler(0x4c4f4731u), &log};
            Matrix<A> x0(Dense(2, 1, {0.3L, 0})), p0(Dense::identity(2, 0.01L)), q(Dense::identity(2, 1e-4L));
            Matrix<A> r(Dense::identity(m, 0.01L)), z(Dense(m, 1, Vec(m, 0.25L)));
            auto xv = x0.view(), pv = p0.view(), qv = q.view(), rv = r.view(), zv = z.view();
            UpfState<A> fixture(count, n, m);
            auto init_ws = pf_init_workspace<A>(count, n);
            auto step_ws = upf_step_workspace<A>(count, n, m);
            typename Upf<A>::Model model{};
            model.transition = logged_map<A>;
            model.transition_context = &transition;
            model.observation = logged_map<A>;
            model.observation_context = &observation;
            model.state_residual = state_residual ? logged_residual<A> : nullptr;
            model.state_residual_context = &state;
            model.measurement_residual = measurement_residual ? logged_residual<A> : nullptr;
            model.measurement_residual_context = &measurement;
            ASSERT_EQ(Upf<A>::init(&fixture.state, work_encode<A>(1), work_encode<A>(2), work_encode<A>(1),
                                   &xv, &pv, &qv, &rv, logged_sample<A>, &sampler, init_ws.view()), LINEARS_OK);
            EXPECT_EQ(log.events, std::string(count, 'S'));
            log.events.clear();
            ASSERT_EQ(Upf<A>::step(&fixture.state, &zv, &model, logged_sample<A>, &sampler, step_ws.view()), LINEARS_OK);
            /* Per particle: the UKF prediction (f on every sigma point, the
             * state residual for the mean and the deviations), the UKF update
             * (h on every sigma point, the measurement residual for the mean,
             * the deviations and the innovation), the proposal draw, then the
             * likelihood of the new particle and its transition prior. */
            std::string particle = std::string(s, 'f');
            if (state_residual) particle += std::string(2 * s, 'r');
            particle += std::string(s, 'h');
            if (measurement_residual) particle += std::string(2 * s + 1, 'm');
            particle += 'S';
            particle += 'h';
            if (measurement_residual) particle += 'm';
            particle += 'f';
            if (state_residual) particle += 'r';
            std::string expected;
            for (std::size_t i = 0; i < count; ++i) expected += particle;
            EXPECT_EQ(log.events, expected);
            EXPECT_EQ(transition.model.calls, static_cast<int>(count * (s + 1)));
            EXPECT_EQ(observation.model.calls, static_cast<int>(count * (s + 1)));
            EXPECT_EQ(state.model.calls, state_residual ? static_cast<int>(count * (2 * s + 1)) : 0);
            EXPECT_EQ(measurement.model.calls, measurement_residual ? static_cast<int>(count * (2 * s + 2)) : 0);
            EXPECT_EQ(sampler.sampler.calls, static_cast<int>(2 * count));
            fixture.guards(); init_ws.guards(); step_ws.guards();
        }
    }
}

TYPED_TEST(UnscentedParticlePipeline, StepBoundaryChecksPrecedeEveryCallback) {
    using A = TypeParam;
    using W = Work<A>;
    const std::size_t count = 3, n = 2, m = 1;
    Model transition = pendulum_transition(), observation = pendulum_observation(m);
    Matrix<A> x0(Dense(2, 1, {0.3L, 0})), p0(Dense::identity(2, 0.01L)), q(Dense::identity(2, 1e-4L));
    Matrix<A> r(Dense::identity(m, 0.01L)), z(Dense(m, 1, {0.25L})), zero_q(Dense::identity(2, 0)), zero_r(Dense(1, 1, {0}));
    auto xv = x0.view(), pv = p0.view(), qv = q.view(), rv = r.view(), zv = z.view();
    UpfState<A> fixture(count, n, m);
    auto init_ws = pf_init_workspace<A>(count, n);
    auto step_ws = upf_step_workspace<A>(count, n, m);
    Sampler sampler(0x424e4452u);
    typename Upf<A>::Model model{};
    model.transition = map_callback<A>;
    model.transition_context = &transition;
    model.observation = map_callback<A>;
    model.observation_context = &observation;
    ASSERT_EQ(Upf<A>::init(&fixture.state, work_encode<A>(1), work_encode<A>(2), work_encode<A>(1),
                           &xv, &pv, &qv, &rv, sample_callback<A>, &sampler, init_ws.view()), LINEARS_OK);
    sampler.calls = 0;
    const auto snapshot = fixture.snapshot();
    const auto weights = fixture.state.sigma_weights;
    const std::size_t raw = sizeof(typename A::Raw);
    /* Q is factored first, then R, then the sigma weights are checked: an
     * invalid weight block never masks a noise failure. */
    std::memcpy(fixture.state.base.process_noise.data, zero_q.data.data(), zero_q.data.size() * raw);
    fixture.state.sigma_weights.scale = W(0);
    EXPECT_EQ(Upf<A>::step(&fixture.state, &zv, &model, sample_callback<A>, &sampler, step_ws.view()), LINEARS_ERROR_NOT_POSITIVE_DEFINITE);
    std::memcpy(fixture.state.base.process_noise.data, q.data.data(), q.data.size() * raw);
    std::memcpy(fixture.state.base.measurement_noise.data, zero_r.data.data(), zero_r.data.size() * raw);
    EXPECT_EQ(Upf<A>::step(&fixture.state, &zv, &model, sample_callback<A>, &sampler, step_ws.view()), LINEARS_ERROR_NOT_POSITIVE_DEFINITE);
    std::memcpy(fixture.state.base.measurement_noise.data, r.data.data(), r.data.size() * raw);
    EXPECT_EQ(Upf<A>::step(&fixture.state, &zv, &model, sample_callback<A>, &sampler, step_ws.view()), LINEARS_ERROR_DOMAIN);
    fixture.state.sigma_weights.scale = W(-1);
    EXPECT_EQ(Upf<A>::step(&fixture.state, &zv, &model, sample_callback<A>, &sampler, step_ws.view()), LINEARS_ERROR_DOMAIN);
    fixture.state.sigma_weights = weights;
    fixture.state.sigma_weights.other = std::numeric_limits<W>::quiet_NaN();
    EXPECT_EQ(Upf<A>::step(&fixture.state, &zv, &model, sample_callback<A>, &sampler, step_ws.view()), LINEARS_ERROR_RANGE);
    fixture.state.sigma_weights = weights;
    fixture.state.sigma_weights.covariance0 = std::numeric_limits<W>::infinity();
    EXPECT_EQ(Upf<A>::step(&fixture.state, &zv, &model, sample_callback<A>, &sampler, step_ws.view()), LINEARS_ERROR_RANGE);
    fixture.state.sigma_weights = weights;
    EXPECT_EQ(transition.calls, 0); EXPECT_EQ(observation.calls, 0); EXPECT_EQ(sampler.calls, 0);
    EXPECT_EQ(fixture.snapshot(), snapshot);
    ASSERT_EQ(Upf<A>::step(&fixture.state, &zv, &model, sample_callback<A>, &sampler, step_ws.view()), LINEARS_OK);
    fixture.guards(); init_ws.guards(); step_ws.guards();
}

TYPED_TEST(UnscentedParticlePipeline, PerParticleCovarianceChecksFailAtTheOffendingParticle) {
    using A = TypeParam;
    using Raw = typename A::Raw;
    const std::size_t count = 4, n = 2, m = 1, s = 2 * n + 1, offender = 2;
    Model transition = pendulum_transition(), observation = pendulum_observation(m), subtract = subtraction_model();
    Matrix<A> x0(Dense(2, 1, {0.3L, 0})), p0(Dense::identity(2, 0.01L)), q(Dense::identity(2, 1e-4L));
    Matrix<A> r(Dense::identity(m, 0.01L)), z(Dense(m, 1, {0.25L}));
    auto xv = x0.view(), pv = p0.view(), qv = q.view(), rv = r.view(), zv = z.view();
    UpfState<A> fixture(count, n, m);
    auto init_ws = pf_init_workspace<A>(count, n);
    auto step_ws = upf_step_workspace<A>(count, n, m);
    Sampler sampler(0x4f464644u);
    typename Upf<A>::Model model{};
    model.transition = map_callback<A>;
    model.transition_context = &transition;
    model.observation = map_callback<A>;
    model.observation_context = &observation;
    model.state_residual = residual_callback<A>;
    model.state_residual_context = &subtract;
    model.measurement_residual = residual_callback<A>;
    model.measurement_residual_context = &subtract;
    ASSERT_EQ(Upf<A>::init(&fixture.state, work_encode<A>(1), work_encode<A>(2), work_encode<A>(1),
                           &xv, &pv, &qv, &rv, sample_callback<A>, &sampler, init_ws.view()), LINEARS_OK);
    const auto snapshot = fixture.snapshot();
    Raw* covariance = fixture.covariances.data() + 1 + offender * n * n;
    Raw* particle = fixture.particles.data() + 1 + offender * n;
    const std::vector<Raw> saved_covariance(covariance, covariance + n * n), saved_particle(particle, particle + n);
    auto expect_failure = [&](linears_status_t expected) {
        transition.calls = observation.calls = subtract.calls = sampler.calls = 0;
        EXPECT_EQ(Upf<A>::step(&fixture.state, &zv, &model, sample_callback<A>, &sampler, step_ws.view()), expected);
        /* The earlier particles ran their full proposal and weight; the
         * offender is rejected before its first callback. */
        EXPECT_EQ(transition.calls, static_cast<int>(offender * (s + 1)));
        EXPECT_EQ(observation.calls, static_cast<int>(offender * (s + 1)));
        EXPECT_EQ(subtract.calls, static_cast<int>(offender * (4 * s + 3)));
        EXPECT_EQ(sampler.calls, static_cast<int>(offender));
        std::copy(saved_covariance.begin(), saved_covariance.end(), covariance);
        std::copy(saved_particle.begin(), saved_particle.end(), particle);
        EXPECT_EQ(fixture.snapshot(), snapshot);
    };
    covariance[1] = A::encode(0.005L);
    expect_failure(LINEARS_ERROR_DOMAIN);
    covariance[0] = A::encode(-0.01L);
    expect_failure(LINEARS_ERROR_DOMAIN);
    std::fill(covariance, covariance + n * n, A::encode(0));
    expect_failure(LINEARS_ERROR_NOT_POSITIVE_DEFINITE);
    if constexpr (A::bits == 0) {
        covariance[3] = std::numeric_limits<Raw>::quiet_NaN();
        expect_failure(LINEARS_ERROR_RANGE);
        particle[1] = std::numeric_limits<Raw>::infinity();
        expect_failure(LINEARS_ERROR_RANGE);
    }
    transition.calls = observation.calls = subtract.calls = sampler.calls = 0;
    ASSERT_EQ(Upf<A>::step(&fixture.state, &zv, &model, sample_callback<A>, &sampler, step_ws.view()), LINEARS_OK);
    EXPECT_EQ(transition.calls, static_cast<int>(count * (s + 1)));
    EXPECT_EQ(subtract.calls, static_cast<int>(count * (4 * s + 3)));
    fixture.guards(); init_ws.guards(); step_ws.guards();
}

TYPED_TEST(UnscentedParticlePipeline, NegativePriorDiagonalIsRejectedBeforeTheObservation) {
    using A = TypeParam;
    /* alpha = 0.5, beta = -1, kappa = 0 with n = 1 gives the center covariance
     * weight -3.25. Squaring maps the symmetric sigma pair onto one value, so
     * the weighted spread about their mean is -P*P, below -Q. */
    const std::size_t count = 2;
    Model square{[](const Vec& x) { return Vec{x[0] * x[0]}; }};
    Model observation{[](const Vec& x) { return x; }};
    Matrix<A> x0(Dense(1, 1, {0})), p0(Dense(1, 1, {A::bits ? 0.25L : 1})), q(Dense(1, 1, {0.01L}));
    Matrix<A> r(Dense(1, 1, {0.25L})), z(Dense(1, 1, {0}));
    auto xv = x0.view(), pv = p0.view(), qv = q.view(), rv = r.view(), zv = z.view();
    UpfState<A> fixture(count, 1, 1);
    auto init_ws = pf_init_workspace<A>(count, 1);
    auto step_ws = upf_step_workspace<A>(count, 1, 1);
    Sampler sampler(0x4e454744u);
    sampler.script = {{0}};
    typename Upf<A>::Model model{};
    model.transition = map_callback<A>;
    model.transition_context = &square;
    model.observation = map_callback<A>;
    model.observation_context = &observation;
    ASSERT_EQ(Upf<A>::init(&fixture.state, work_encode<A>(0.5L), work_encode<A>(-1), work_encode<A>(0),
                           &xv, &pv, &qv, &rv, sample_callback<A>, &sampler, init_ws.view()), LINEARS_OK);
    EXPECT_EQ(work_decode<A>(fixture.state.sigma_weights.covariance0), -3.25L);
    sampler.calls = 0;
    const auto snapshot = fixture.snapshot();
    EXPECT_EQ(Upf<A>::step(&fixture.state, &zv, &model, sample_callback<A>, &sampler, step_ws.view()), LINEARS_ERROR_DOMAIN);
    EXPECT_EQ(square.calls, 3);
    EXPECT_EQ(observation.calls, 0);
    EXPECT_EQ(sampler.calls, 0);
    EXPECT_EQ(fixture.snapshot(), snapshot);
    fixture.guards(); init_ws.guards(); step_ws.guards();
}

TYPED_TEST(UnscentedParticlePipeline, WideMeasurementStepsFitTheExactMinimumWorkspace) {
    using A = TypeParam;
    const std::size_t shapes[][3] = {{3, 1, 4}, {3, 2, 6}, {2, 1, 5}};
    for (const auto& shape : shapes) {
        const std::size_t count = shape[0], n = shape[1], m = shape[2], s = 2 * n + 1;
        SCOPED_TRACE(::testing::Message() << "N=" << count << " n=" << n << " m=" << m);
        ASSERT_GT(m, s);
        Matrix<A> x(Dense(n, 1, Vec(n, 0.0625L))), p(Dense::identity(n, 0.0625L));
        Matrix<A> q(Dense::identity(n, 0.015625L)), r(Dense::identity(m, 0.125L)), z(Dense(m, 1, Vec(m, 0.03125L)));
        auto xv = x.view(), pv = p.view(), qv = q.view(), rv = r.view(), zv = z.view();
        UpfState<A> fixture(count, n, m);
        auto init_ws = pf_init_workspace<A>(count, n);
        auto step_ws = upf_step_workspace<A>(count, n, m);
        Sampler sampler(0x57494445u);
        sampler.script = {Vec(n, 0.5L), Vec(n, -0.25L)};
        Model transition{[](const Vec& value) { return value; }};
        Model observation{[=](const Vec& value) {
            Vec y(m);
            for (std::size_t j = 0; j < m; ++j) y[j] = 0.5L * value[j % n] + 0.0625L * static_cast<long double>(j);
            return y;
        }};
        typename Upf<A>::Model model{};
        model.transition = map_callback<A>; model.transition_context = &transition;
        model.observation = map_callback<A>; model.observation_context = &observation;
        ASSERT_EQ(Upf<A>::init(&fixture.state, work_encode<A>(1), work_encode<A>(2), work_encode<A>(0),
                               &xv, &pv, &qv, &rv, sample_callback<A>, &sampler, init_ws.view()), LINEARS_OK);
        const auto before = fixture.snapshot();
        sampler.calls = 0;
        for (int part = 0; part < 4; ++part) {
            auto workspace = step_ws.view();
            std::size_t *capacity = part == 0 ? &workspace.capacity
                                  : part == 1 ? &workspace.basic_bytes
                                  : part == 2 ? &workspace.decomposition.capacity : nullptr;
            if constexpr (A::bits != 0) {
                if (part == 3) capacity = &workspace.signal_capacity;
            }
            if (capacity == nullptr || *capacity == 0) continue;
            --*capacity;
            EXPECT_EQ(Upf<A>::step(&fixture.state, &zv, &model, sample_callback<A>, &sampler, workspace), LINEARS_ERROR_WORKSPACE);
        }
        EXPECT_EQ(fixture.snapshot(), before);
        EXPECT_EQ(transition.calls, 0); EXPECT_EQ(observation.calls, 0); EXPECT_EQ(sampler.calls, 0);
        ASSERT_EQ(Upf<A>::step(&fixture.state, &zv, &model, sample_callback<A>, &sampler, step_ws.view()), LINEARS_OK);
        EXPECT_EQ(transition.calls, static_cast<int>(count * (s + 1)));
        EXPECT_EQ(observation.calls, static_cast<int>(count * (s + 1)));
        EXPECT_EQ(sampler.calls, static_cast<int>(count));
        /* The step moved every particle and tightened every covariance. */
        const std::vector<Vec> rows = fixture.particle_rows();
        const Dense covariances = decode<A>(fixture.state.covariances);
        for (std::size_t i = 0; i < count; ++i) {
            for (std::size_t j = 0; j < n; ++j) EXPECT_NE(rows[i][j], decode<A>(xv).values[j]);
            for (std::size_t j = 0; j < n; ++j) {
                EXPECT_GT(covariances(i, j * n + j), 0);
                EXPECT_LT(covariances(i, j * n + j), decode<A>(pv)(j, j) + decode<A>(qv)(j, j));
            }
        }
        expect_raw_weight_scale<A>(fixture.weights);
        fixture.guards(); init_ws.guards(); step_ws.guards();
    }
}

TYPED_TEST(UnscentedParticlePipeline, WideLinearGaussianModelAgreesWithKalmanFilter) {
    using A = TypeParam;
    const std::size_t count = 1000;
    const std::size_t shapes[][3] = {{1, 4, 0}, {2, 6, 0}, {2, 6, 1}};
    for (const auto& shape : shapes) {
        const std::size_t n = shape[0], m = shape[1];
        SCOPED_TRACE(::testing::Message() << "n=" << n << " m=" << m << " correlated_Q=" << shape[2]);
        std::mt19937 random(0x574c4b46u + static_cast<unsigned>(m));
        const Dense x0 = random_dense(random, n, 1, 0.1L), p0 = random_spd(random, n, 0.01L, 0.05L);
        Dense q = Dense::identity(n, 0.001L);
        // Cover the unchanged general density solve for both Q and R.
        if (shape[2] != 0) q(0, 1) = q(1, 0) = 0.00025L;
        const Dense f = Dense::identity(n, 0.9L) + random_dense(random, n, n, 0.05L);
        const Dense h = random_dense(random, m, n, 0.5L), r = random_spd(random, m, 0.01L, 0.05L);
        Dense z = h * x0;
        for (auto& value : z.values) value += 0.03L;
        Matrix<A> xm(x0), pm(p0), qm(q), fm(f), hm(h), rm(r), zm(z), mean_out(Dense(n, 1)), covariance_out(Dense(n, n));
        auto xv = xm.view(), pv = pm.view(), qv = qm.view(), fv = fm.view(), hv = hm.view(), rv = rm.view(), zv = zm.view();
        auto mv = mean_out.view(), cv = covariance_out.view();
        Model transition{[F = decode<A>(fv)](const Vec& x) { return linear_map(F, x); }};
        Model observation{[H = decode<A>(hv)](const Vec& x) { return linear_map(H, x); }};
        MatrixState<A> kf(n);
        Workspace<A> kf_predict(n), kf_update(n, m);
        ASSERT_EQ(A::reset_matrix(&kf.state, &xv, &pv), LINEARS_OK);
        ASSERT_EQ(A::predict_matrix(&kf.state, &fv, &qv, kf_predict.view()), LINEARS_OK);
        ASSERT_EQ(A::update_matrix(&kf.state, &zv, &hv, &rv, kf_update.view()), LINEARS_OK);
        UpfState<A> fixture(count, n, m);
        auto init_ws = pf_init_workspace<A>(count, n);
        auto step_ws = upf_step_workspace<A>(count, n, m);
        auto estimate_ws = pf_estimate_workspace<A>(count, n);
        Sampler sampler(0x4b414c4du + static_cast<unsigned>(n));
        typename Upf<A>::Model model{};
        model.transition = map_callback<A>;
        model.transition_context = &transition;
        model.observation = map_callback<A>;
        model.observation_context = &observation;
        ASSERT_EQ(Upf<A>::init(&fixture.state, work_encode<A>(1), work_encode<A>(2), work_encode<A>(3.0L - n),
                               &xv, &pv, &qv, &rv, sample_callback<A>, &sampler, init_ws.view()), LINEARS_OK);
        ASSERT_EQ(Upf<A>::step(&fixture.state, &zv, &model, sample_callback<A>, &sampler, step_ws.view()), LINEARS_OK);
        EXPECT_EQ(transition.calls, static_cast<int>(count * (2 * n + 2)));
        EXPECT_EQ(observation.calls, static_cast<int>(count * (2 * n + 2)));
        Work<A> ess = work_encode<A>(0);
        ASSERT_EQ(Pf<A>::effective_sample_size(&fixture.state.base.weights, &ess), LINEARS_OK);
        EXPECT_GT(work_decode<A>(ess), static_cast<long double>(count) / 10);
        ASSERT_EQ(Pf<A>::estimate(&fixture.state.base, nullptr, nullptr, &mv, &cv, estimate_ws.view()), LINEARS_OK);
        const Dense kf_mean = decode<A>(kf.state.estimate), kf_cov = decode<A>(kf.state.covariance);
        const Dense pf_mean = decode<A>(mv), pf_cov = decode<A>(cv);
        for (std::size_t i = 0; i < n; ++i) {
            const long double sigma = std::sqrt(kf_cov(i, i));
            EXPECT_LT(std::abs(pf_mean.values[i] - kf_mean.values[i]), 0.2L * sigma) << i;
            for (std::size_t j = 0; j < n; ++j)
                EXPECT_LT(std::abs(pf_cov(i, j) - kf_cov(i, j)), 0.3L * std::sqrt(kf_cov(i, i) * kf_cov(j, j))) << i << j;
        }
        fixture.guards(); kf.guards(); init_ws.guards(); step_ws.guards(); estimate_ws.guards();
    }
}

} // namespace
