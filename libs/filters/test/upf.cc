#include "pf_support.h"
#include "filters_upf.h"

namespace {
using namespace pf_test;

template<class A> struct Upf;
#define UPF_TEST_API(tag, kernel) \
template<> struct Upf<tag> { \
    using State = filters_upf_##tag##_t; \
    using Model = filters_upf_##kernel##_model_t; \
    static constexpr auto init = filters_upf_##tag##_init; \
    static constexpr auto step = filters_upf_##tag##_step; \
    static constexpr auto resample = filters_upf_##tag##_resample; \
};
UPF_TEST_API(f32, f32)
UPF_TEST_API(f64, f64)
UPF_TEST_API(q15, fixed)
UPF_TEST_API(q31, fixed)
#undef UPF_TEST_API

template<class A> struct UpfState {
    using Raw = typename A::Raw;
    using W = Work<A>;
    static constexpr Raw sentinel = static_cast<Raw>(-12345);
    static constexpr W work_sentinel = static_cast<W>(-23456);
    std::vector<W> particles, weights, covariances;
    std::vector<Raw> q, r;
    typename Upf<A>::State state;
    UpfState(std::size_t count, std::size_t n, std::size_t m)
        : particles(count * n + 2, work_sentinel), weights(count + 2, work_sentinel),
          covariances(count * n * n + 2, work_sentinel), q(n * n + 2, sentinel), r(m * m + 2, sentinel),
          state{{{particles.data() + 1, count, n}, {weights.data() + 1, count, 1},
                 {q.data() + 1, n, n}, {r.data() + 1, m, m}, count * n, count, n * n, m * m},
                {covariances.data() + 1, count, n * n}, count * n * n, {}} {}
    void guards() const {
        for (const auto *v : {&particles, &weights, &covariances}) {
            EXPECT_EQ(v->front(), work_sentinel); EXPECT_EQ(v->back(), work_sentinel);
        }
        EXPECT_EQ(q.front(), sentinel); EXPECT_EQ(q.back(), sentinel);
        EXPECT_EQ(r.front(), sentinel); EXPECT_EQ(r.back(), sentinel);
    }
    auto snapshot() const { return std::make_tuple(bytes(state), particles, weights, covariances, q, r); }
    std::vector<Vec> particle_rows() const {
        const Dense all = decode_work<A>(state.base.particles);
        std::vector<Vec> rows(all.rows, Vec(all.cols));
        for (std::size_t i = 0; i < all.rows; ++i)
            for (std::size_t j = 0; j < all.cols; ++j) rows[i][j] = all(i, j);
        return rows;
    }
    Vec weight_values() const { return decode_work<A>(state.base.weights).values; }
    std::vector<Dense> covariance_rows() const {
        const Dense all = decode_work<A>(state.covariances);
        const std::size_t n = state.base.particles.cols;
        std::vector<Dense> rows(all.rows, Dense(n, n));
        for (std::size_t i = 0; i < all.rows; ++i)
            for (std::size_t k = 0; k < n * n; ++k) rows[i].values[k] = all(i, k);
        return rows;
    }
};

template<class A> UkfWorkspace<A> upf_step_workspace(std::size_t count, std::size_t n, std::size_t m) {
    return UkfWorkspace<A>(A::bits ? FILTERS_UPF_Q_STEP_STORAGE(count, n, m) : FILTERS_UPF_STEP_STORAGE(count, n, m),
                           FILTERS_UPF_STEP_BASIC_BYTES(n, m), FILTERS_UPF_STEP_DECOMP_STORAGE(n, m));
}
template<class A> UkfWorkspace<A> upf_resample_workspace(std::size_t count, std::size_t n) {
    return UkfWorkspace<A>(FILTERS_UPF_RESAMPLE_STORAGE(count, n), 0, 0);
}

/* Independent long-double UPF: one Eigen-based UKF per particle, the
 * Gaussian proposal draw, and the likelihood * prior / proposal weight. */
struct UpfReference {
    long double alpha, beta, kappa;
    std::vector<Vec> particles;
    std::vector<Dense> covariances;
    Vec weights;
    UpfReference(long double a, long double b, long double k) : alpha(a), beta(b), kappa(k) {}
    void init(const Dense& mean, const Dense& covariance, const std::vector<Vec>& draws) {
        PfReference base;
        base.init(mean, covariance, draws);
        particles = base.particles;
        weights = base.weights;
        covariances.assign(draws.size(), covariance);
    }
    void step(const Dense& z, const Model& f, const Model& h, const Model& state_residual,
              const Model& measurement_residual, const Dense& Q, const Dense& R, const std::vector<Vec>& draws) {
        const std::size_t count = particles.size(), n = particles[0].size();
        Model fm = f, hm = h;
        fm.residual = state_residual.residual;
        hm.residual = measurement_residual.residual;
        std::vector<Vec> moved(count);
        std::vector<Dense> posteriors(count, Dense(n, n));
        Vec exponents(count), log_scale(count);
        for (std::size_t i = 0; i < count; ++i) {
            UkfReference ukf(alpha, beta, kappa, Dense(n, 1, particles[i]), covariances[i]);
            ukf.predict(fm, Q);
            ukf.update(z, hm, R);
            const Dense proposal_mean = ukf.estimate();
            posteriors[i] = ukf.covariance_matrix();
            const EigenMatrix L = PfReference::cholesky(posteriors[i]);
            moved[i] = PfReference::shifted(proposal_mean.values, L, draws[i]);
            long double squared = 0;
            for (auto value : draws[i]) squared += value * value;
            exponents[i] = -0.5L * squared;
            log_scale[i] = 0;
            for (std::size_t j = 0; j < n; ++j) log_scale[i] += std::log(L(static_cast<Eigen::Index>(j), static_cast<Eigen::Index>(j)));
            const Vec dz = UkfReference::residual_of(hm, h.map(moved[i]), z.values);
            exponents[i] += PfReference::exponents({dz}, R)[0];
            const Vec dx = UkfReference::residual_of(fm, moved[i], f.map(particles[i]));
            exponents[i] += PfReference::exponents({dx}, Q)[0];
        }
        const long double lowest = *std::min_element(exponents.begin(), exponents.end());
        long double sum = 0;
        for (std::size_t i = 0; i < count; ++i)
            sum += weights[i] = weights[i] * std::exp(log_scale[i] - (exponents[i] - lowest));
        for (auto& w : weights) w /= sum;
        particles = moved;
        covariances = posteriors;
    }
    void resample(const std::vector<std::size_t>& indices) {
        std::vector<Vec> moved(indices.size());
        std::vector<Dense> gathered(indices.size(), covariances[0]);
        for (std::size_t i = 0; i < indices.size(); ++i) {
            moved[i] = particles[indices[i]];
            gathered[i] = covariances[indices[i]];
        }
        particles = moved;
        covariances = gathered;
        weights.assign(indices.size(), 1.0L / static_cast<long double>(indices.size()));
    }
};

template<class A> class UnscentedParticleFilter : public ::testing::Test {};
TYPED_TEST_SUITE(UnscentedParticleFilter, Formats);

TYPED_TEST(UnscentedParticleFilter, PendulumStepsMatchLongDoubleReference) {
    using A = TypeParam;
    const std::size_t count = 24;
    const long double dt = 0.1L, gravity = 0.5L;
    const long double alpha = A::bits ? 1 : 0.5L, beta = 2, kappa = 1;
    Model transition{[=](const Vec& s) { return Vec{s[0] + dt * s[1], s[1] - dt * gravity * std::sin(s[0])}; }};
    Model observation{[](const Vec& s) { return Vec{std::sin(s[0]), s[1]}; }};
    Model plain, subtract;
    subtract.residual = [](const Vec& a, const Vec& b) { Vec d(a.size()); for (std::size_t i = 0; i < d.size(); ++i) d[i] = a[i] - b[i]; return d; };
    Matrix<A> x0(Dense(2, 1, {0.3L, 0})), p0(Dense::identity(2, 0.01L)), q(Dense::identity(2, 1e-4L));
    Matrix<A> r(Dense::identity(2, 0.01L)), mean_out(Dense(2, 1)), covariance_out(Dense(2, 2));
    auto xv = x0.view(), pv = p0.view(), qv = q.view(), rv = r.view(), mv = mean_out.view(), cv = covariance_out.view();
    UpfState<A> fixture(count, 2, 2);
    auto init_ws = pf_init_workspace<A>(count, 2);
    auto step_ws = upf_step_workspace<A>(count, 2, 2);
    auto resample_ws = upf_resample_workspace<A>(count, 2);
    auto estimate_ws = pf_estimate_workspace<A>(count, 2);
    Sampler sampler(0x55504644u);
    std::vector<std::size_t> indices(count);
    UpfReference reference(work_decode<A>(work_encode<A>(alpha)), work_decode<A>(work_encode<A>(beta)),
                           work_decode<A>(work_encode<A>(kappa)));
    ASSERT_EQ(Upf<A>::init(&fixture.state, work_encode<A>(alpha), work_encode<A>(beta), work_encode<A>(kappa),
                           &xv, &pv, &qv, &rv, sample_callback<A>, &sampler, init_ws.view()), LINEARS_OK);
    reference.init(decode<A>(xv), decode<A>(pv), sampler.take_log());
    for (std::size_t i = 0; i < count; ++i) {
        for (std::size_t j = 0; j < 2; ++j) expect_near<A>(fixture.particle_rows()[i][j], reference.particles[i][j]);
        EXPECT_EQ(fixture.covariance_rows()[i].values, decode<A>(pv).values);
    }
    expect_near<A>(work_decode<A>(fixture.state.sigma_weights.scale), reference.alpha * reference.alpha * (2 + reference.kappa));
    typename Upf<A>::Model model{};
    model.transition = map_callback<A>;
    model.transition_context = &transition;
    model.observation = map_callback<A>;
    model.observation_context = &observation;
    std::mt19937 random(0x50454e44u);
    std::normal_distribution<double> noise(0, 0.1);
    Vec truth{0.3L, 0};
    int resamples = 0;
    for (int step = 0; step < 20; ++step) {
        SCOPED_TRACE(step);
        truth = transition.map(truth);
        const Vec observed = observation.map(truth);
        Matrix<A> z(Dense(2, 1, {observed[0] + noise(random), observed[1] + noise(random)}));
        auto zv = z.view();
        /* Odd steps route both residuals through explicit subtraction callbacks. */
        const bool explicit_residuals = step % 2 == 1;
        model.state_residual = explicit_residuals ? residual_callback<A> : nullptr;
        model.state_residual_context = &subtract;
        model.measurement_residual = explicit_residuals ? residual_callback<A> : nullptr;
        model.measurement_residual_context = &subtract;
        reference.particles = fixture.particle_rows();
        reference.covariances = fixture.covariance_rows();
        reference.weights = fixture.weight_values();
        subtract.calls = 0;
        ASSERT_EQ(Upf<A>::step(&fixture.state, &zv, &model, sample_callback<A>, &sampler, step_ws.view()), LINEARS_OK);
        EXPECT_EQ(subtract.calls > 0, explicit_residuals);
        reference.step(decode<A>(zv), transition, observation, explicit_residuals ? subtract : plain,
                       explicit_residuals ? subtract : plain, decode<A>(qv), decode<A>(rv), sampler.take_log());
        for (std::size_t i = 0; i < count; ++i) {
            SCOPED_TRACE(i);
            for (std::size_t j = 0; j < 2; ++j) expect_near<A>(fixture.particle_rows()[i][j], reference.particles[i][j]);
            for (std::size_t k = 0; k < 4; ++k) expect_near<A>(fixture.covariance_rows()[i].values[k], reference.covariances[i].values[k]);
            /* The prior exponent (1/2)*d^T*Q^-1*d amplifies working rounding of the
             * new particle by |d|/Q, about 300 here, so a weight carries a few
             * hundred working ulps of relative error instead of a few. */
            expect_near<A>(fixture.weight_values()[i], reference.weights[i], 4);
        }
        Work<A> ess = work_encode<A>(0);
        ASSERT_EQ(Pf<A>::effective_sample_size(&fixture.state.base.weights, &ess), LINEARS_OK);
        if (work_decode<A>(ess) < static_cast<long double>(count) / 2) {
            const long double uniform = work_decode<A>(work_encode<A>(sampler.uniform()));
            reference.weights = fixture.weight_values();
            reference.particles = fixture.particle_rows();
            reference.covariances = fixture.covariance_rows();
            const std::vector<std::size_t> expected = PfReference::systematic(reference.weights, uniform);
            ASSERT_EQ(Upf<A>::resample(&fixture.state, work_encode<A>(uniform), indices.data(), count, resample_ws.view()), LINEARS_OK);
            EXPECT_EQ(indices, expected);
            reference.resample(expected);
            EXPECT_EQ(fixture.particle_rows(), reference.particles);
            for (std::size_t i = 0; i < count; ++i) EXPECT_EQ(fixture.covariance_rows()[i].values, reference.covariances[i].values);
            for (std::size_t i = 0; i < count; ++i) expect_near<A>(fixture.weight_values()[i], reference.weights[i]);
            ++resamples;
        }
        PfReference estimator;
        estimator.particles = fixture.particle_rows();
        estimator.weights = fixture.weight_values();
        ASSERT_EQ(Pf<A>::estimate(&fixture.state.base, nullptr, nullptr, &mv, &cv, estimate_ws.view()), LINEARS_OK);
        const Vec mean = estimator.mean(plain);
        expect_matrix<A>(mv, Dense(2, 1, mean));
        expect_matrix<A>(cv, estimator.covariance(mean, plain));
    }
    EXPECT_GT(resamples, 0);
    fixture.guards(); init_ws.guards(); step_ws.guards(); resample_ws.guards(); estimate_ws.guards();
}

TYPED_TEST(UnscentedParticleFilter, LinearGaussianModelAgreesWithKalmanFilter) {
    using A = TypeParam;
    const std::size_t count = 600, n = 2, m = 1;
    std::mt19937 random(0x55504b46u);
    const Dense x0(2, 1, {0.1L, -0.05L}), p0 = random_spd(random, n, 0.01L, 0.05L);
    const Dense q = Dense::identity(n, 0.001L), f(2, 2, {1, 0.1L, 0, 0.9L});
    const Dense h(1, 2, {0.5L, 0.3L}), r(1, 1, {0.01L}), z(1, 1, {0.5L * 0.1L - 0.3L * 0.05L + 0.05L});
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
    Sampler sampler(0x4b414c4du);
    typename Upf<A>::Model model{};
    model.transition = map_callback<A>;
    model.transition_context = &transition;
    model.observation = map_callback<A>;
    model.observation_context = &observation;
    ASSERT_EQ(Upf<A>::init(&fixture.state, work_encode<A>(1), work_encode<A>(2), work_encode<A>(1),
                           &xv, &pv, &qv, &rv, sample_callback<A>, &sampler, init_ws.view()), LINEARS_OK);
    ASSERT_EQ(Upf<A>::step(&fixture.state, &zv, &model, sample_callback<A>, &sampler, step_ws.view()), LINEARS_OK);
    /* f runs on 2n+1 sigma points plus the particle itself; h on 2n+1 points plus the new particle. */
    EXPECT_EQ(transition.calls, static_cast<int>(count * (2 * n + 2)));
    EXPECT_EQ(observation.calls, static_cast<int>(count * (2 * n + 2)));
    EXPECT_EQ(sampler.calls, static_cast<int>(2 * count));
    Work<A> ess = work_encode<A>(0);
    ASSERT_EQ(Pf<A>::effective_sample_size(&fixture.state.base.weights, &ess), LINEARS_OK);
    /* Each particle proposes from its own UKF posterior, whose prior spread
     * includes the carried P_i, so the weights are far from uniform even here. */
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

TYPED_TEST(UnscentedParticleFilter, PendulumTrackingStaysWithinItsOwnSpread) {
    using A = TypeParam;
    const std::size_t count = 60;
    const long double dt = 0.1L, gravity = 0.5L;
    Model transition{[=](const Vec& s) { return Vec{s[0] + dt * s[1], s[1] - dt * gravity * std::sin(s[0])}; }};
    Model observation{[](const Vec& s) { return Vec{std::sin(s[0]), s[1]}; }};
    Matrix<A> x0(Dense(2, 1, {0.3L, 0})), p0(Dense::identity(2, 0.01L)), q(Dense::identity(2, 1e-4L));
    Matrix<A> r(Dense::identity(2, 0.01L)), mean_out(Dense(2, 1)), covariance_out(Dense(2, 2));
    auto xv = x0.view(), pv = p0.view(), qv = q.view(), rv = r.view(), mv = mean_out.view(), cv = covariance_out.view();
    UpfState<A> fixture(count, 2, 2);
    auto init_ws = pf_init_workspace<A>(count, 2);
    auto step_ws = upf_step_workspace<A>(count, 2, 2);
    auto resample_ws = upf_resample_workspace<A>(count, 2);
    auto estimate_ws = pf_estimate_workspace<A>(count, 2);
    Sampler sampler(0x55505452u);
    std::vector<std::size_t> indices(count);
    typename Upf<A>::Model model{};
    model.transition = map_callback<A>;
    model.transition_context = &transition;
    model.observation = map_callback<A>;
    model.observation_context = &observation;
    ASSERT_EQ(Upf<A>::init(&fixture.state, work_encode<A>(1), work_encode<A>(2), work_encode<A>(1),
                           &xv, &pv, &qv, &rv, sample_callback<A>, &sampler, init_ws.view()), LINEARS_OK);
    std::mt19937 random(0x54525554u);
    std::normal_distribution<double> noise(0, 0.1);
    Vec truth{0.3L, 0};
    for (int step = 0; step < 60; ++step) {
        SCOPED_TRACE(step);
        truth = transition.map(truth);
        const Vec observed = observation.map(truth);
        Matrix<A> z(Dense(2, 1, {observed[0] + noise(random), observed[1] + noise(random)}));
        auto zv = z.view();
        ASSERT_EQ(Upf<A>::step(&fixture.state, &zv, &model, sample_callback<A>, &sampler, step_ws.view()), LINEARS_OK);
        Work<A> ess = work_encode<A>(0);
        ASSERT_EQ(Pf<A>::effective_sample_size(&fixture.state.base.weights, &ess), LINEARS_OK);
        if (work_decode<A>(ess) < static_cast<long double>(count) / 2)
            ASSERT_EQ(Upf<A>::resample(&fixture.state, work_encode<A>(sampler.uniform()), indices.data(), count, resample_ws.view()), LINEARS_OK);
    }
    ASSERT_EQ(Pf<A>::estimate(&fixture.state.base, nullptr, nullptr, &mv, &cv, estimate_ws.view()), LINEARS_OK);
    const Dense estimate = decode<A>(mv), covariance = decode<A>(cv);
    for (std::size_t i = 0; i < 2; ++i) {
        EXPECT_GT(covariance(i, i), 0);
        EXPECT_LT(std::abs(estimate.values[i] - truth[i]), 4 * std::sqrt(covariance(i, i)) + 0.03L);
    }
    fixture.guards(); init_ws.guards(); step_ws.guards(); resample_ws.guards(); estimate_ws.guards();
}

TYPED_TEST(UnscentedParticleFilter, ValidationAliasRulesAndFailureAtomicity) {
    using A = TypeParam;
    Model transition{[](const Vec& x) { return Vec{0.5L * x[0] + 0.1L * x[1], 0.5L * x[1]}; }};
    Model observation{[](const Vec& x) { return Vec{0.5L * x[0]}; }};
    Matrix<A> x0(Dense(2, 1, {0.25L, -0.125L})), p0(Dense::identity(2, 0.25L)), q(Dense::identity(2, 0.01L));
    Matrix<A> r(Dense::identity(1, 0.25L)), z(Dense(1, 1, {0.125L}));
    Matrix<A> asymmetric(Dense(2, 2, {0.01L, 0.005L, 0, 0.01L})), zero(Dense::identity(2, 0)), zero_r(Dense(1, 1, {0}));
    auto xv = x0.view(), pv = p0.view(), qv = q.view(), rv = r.view(), zv = z.view(), zero_pv = zero.view();
    const std::size_t count = 5;
    UpfState<A> fixture(count, 2, 1);
    auto init_ws = pf_init_workspace<A>(count, 2);
    auto step_ws = upf_step_workspace<A>(count, 2, 1);
    auto resample_ws = upf_resample_workspace<A>(count, 2);
    Sampler sampler(7);
    const auto sample = sample_callback<A>;
    const auto one = work_encode<A>(1), two = work_encode<A>(2), zero_w = work_encode<A>(0);
    std::vector<std::size_t> indices(count, SIZE_MAX);
    typename Upf<A>::Model model{};
    model.transition = map_callback<A>;
    model.transition_context = &transition;
    model.observation = map_callback<A>;
    model.observation_context = &observation;
    /* Init validation happens before any draw; the state stays untouched. */
    EXPECT_EQ(Upf<A>::init(nullptr, one, two, zero_w, &xv, &pv, &qv, &rv, sample, &sampler, init_ws.view()), LINEARS_ERROR_ARGUMENT);
    EXPECT_EQ(Upf<A>::init(&fixture.state, one, two, zero_w, &xv, &pv, &qv, &rv, nullptr, &sampler, init_ws.view()), LINEARS_ERROR_ARGUMENT);
    EXPECT_EQ(Upf<A>::init(&fixture.state, zero_w, two, zero_w, &xv, &pv, &qv, &rv, sample, &sampler, init_ws.view()), LINEARS_ERROR_DOMAIN);
    EXPECT_EQ(Upf<A>::init(&fixture.state, one, two, work_encode<A>(-2), &xv, &pv, &qv, &rv, sample, &sampler, init_ws.view()), LINEARS_ERROR_DOMAIN);
    EXPECT_EQ(Upf<A>::init(&fixture.state, one, two, zero_w, &xv, &zero_pv, &qv, &rv, sample, &sampler, init_ws.view()), LINEARS_ERROR_NOT_POSITIVE_DEFINITE);
    fixture.state.covariances_capacity = count * 4 - 1;
    EXPECT_EQ(Upf<A>::init(&fixture.state, one, two, zero_w, &xv, &pv, &qv, &rv, sample, &sampler, init_ws.view()), LINEARS_ERROR_WORKSPACE);
    fixture.state.covariances_capacity = count * 4;
    fixture.state.covariances.cols = 3;
    EXPECT_EQ(Upf<A>::init(&fixture.state, one, two, zero_w, &xv, &pv, &qv, &rv, sample, &sampler, init_ws.view()), LINEARS_ERROR_DIMENSION);
    fixture.state.covariances.cols = 4;
    auto previous = fixture.state.covariances.data;
    fixture.state.covariances.data = fixture.state.base.particles.data;
    EXPECT_EQ(Upf<A>::init(&fixture.state, one, two, zero_w, &xv, &pv, &qv, &rv, sample, &sampler, init_ws.view()), LINEARS_ERROR_ALIAS);
    fixture.state.covariances.data = previous;
    linears_matrix_t into_covariances{fixture.state.covariances.data, 2, 2};
    EXPECT_EQ(Upf<A>::init(&fixture.state, one, two, zero_w, &xv, &into_covariances, &qv, &rv, sample, &sampler, init_ws.view()), LINEARS_ERROR_ALIAS);
    sampler.fail_at = 2;
    EXPECT_EQ(Upf<A>::init(&fixture.state, one, two, zero_w, &xv, &pv, &qv, &rv, sample, &sampler, init_ws.view()), LINEARS_ERROR_NO_CONVERGENCE);
    EXPECT_EQ(sampler.calls, 3);
    sampler.fail_at = -1; sampler.calls = 0;
    for (auto& v : fixture.covariances) EXPECT_EQ(v, UpfState<A>::work_sentinel);
    for (auto& v : fixture.particles) EXPECT_EQ(v, UpfState<A>::work_sentinel);
    EXPECT_EQ(work_decode<A>(fixture.state.sigma_weights.scale), 0);
    ASSERT_EQ(Upf<A>::init(&fixture.state, one, two, zero_w, &xv, &pv, &qv, &rv, sample, &sampler, init_ws.view()), LINEARS_OK);
    EXPECT_EQ(sampler.calls, static_cast<int>(count));
    for (std::size_t i = 0; i < count; ++i) {
        EXPECT_EQ(fixture.covariance_rows()[i].values, decode<A>(pv).values);
        expect_near<A>(fixture.weight_values()[i], 0.2L);
    }
    expect_near<A>(work_decode<A>(fixture.state.sigma_weights.scale), 2);
    const auto snapshot = fixture.snapshot();
    sampler.calls = 0;
    /* Step validation and pre-callback noise checks. */
    EXPECT_EQ(Upf<A>::step(nullptr, &zv, &model, sample, &sampler, step_ws.view()), LINEARS_ERROR_ARGUMENT);
    EXPECT_EQ(Upf<A>::step(&fixture.state, &zv, nullptr, sample, &sampler, step_ws.view()), LINEARS_ERROR_ARGUMENT);
    EXPECT_EQ(Upf<A>::step(&fixture.state, &zv, &model, nullptr, &sampler, step_ws.view()), LINEARS_ERROR_ARGUMENT);
    EXPECT_EQ(Upf<A>::step(&fixture.state, nullptr, &model, sample, &sampler, step_ws.view()), LINEARS_ERROR_ARGUMENT);
    auto incomplete = model;
    incomplete.transition = nullptr;
    EXPECT_EQ(Upf<A>::step(&fixture.state, &zv, &incomplete, sample, &sampler, step_ws.view()), LINEARS_ERROR_ARGUMENT);
    incomplete = model;
    incomplete.observation = nullptr;
    EXPECT_EQ(Upf<A>::step(&fixture.state, &zv, &incomplete, sample, &sampler, step_ws.view()), LINEARS_ERROR_ARGUMENT);
    auto wrong = zv; wrong.rows = 2;
    EXPECT_EQ(Upf<A>::step(&fixture.state, &wrong, &model, sample, &sampler, step_ws.view()), LINEARS_ERROR_DIMENSION);
    for (int part = 0; part < 3; ++part) {
        auto w = step_ws.view();
        if (part == 0) --w.capacity;
        if (part == 1) --w.basic_bytes;
        if (part == 2) --w.decomposition.capacity;
        EXPECT_EQ(Upf<A>::step(&fixture.state, &zv, &model, sample, &sampler, w), LINEARS_ERROR_WORKSPACE);
    }
    auto w = step_ws.view(); w.decomposition.data = w.data;
    EXPECT_EQ(Upf<A>::step(&fixture.state, &zv, &model, sample, &sampler, w), LINEARS_ERROR_ALIAS);
    auto aliased = zv; aliased.data = step_ws.view().data;
    EXPECT_EQ(Upf<A>::step(&fixture.state, &aliased, &model, sample, &sampler, step_ws.view()), LINEARS_ERROR_ALIAS);
    previous = fixture.state.covariances.data;
    fixture.state.covariances.data = step_ws.view().data;
    EXPECT_EQ(Upf<A>::step(&fixture.state, &zv, &model, sample, &sampler, step_ws.view()), LINEARS_ERROR_ALIAS);
    fixture.state.covariances.data = previous;
    std::memcpy(fixture.state.base.process_noise.data, asymmetric.data.data(), asymmetric.data.size() * sizeof(typename A::Raw));
    EXPECT_EQ(Upf<A>::step(&fixture.state, &zv, &model, sample, &sampler, step_ws.view()), LINEARS_ERROR_DOMAIN);
    std::memcpy(fixture.state.base.process_noise.data, zero.data.data(), zero.data.size() * sizeof(typename A::Raw));
    EXPECT_EQ(Upf<A>::step(&fixture.state, &zv, &model, sample, &sampler, step_ws.view()), LINEARS_ERROR_NOT_POSITIVE_DEFINITE);
    std::memcpy(fixture.state.base.process_noise.data, q.data.data(), q.data.size() * sizeof(typename A::Raw));
    std::memcpy(fixture.state.base.measurement_noise.data, zero_r.data.data(), sizeof(typename A::Raw));
    EXPECT_EQ(Upf<A>::step(&fixture.state, &zv, &model, sample, &sampler, step_ws.view()), LINEARS_ERROR_NOT_POSITIVE_DEFINITE);
    std::memcpy(fixture.state.base.measurement_noise.data, r.data.data(), sizeof(typename A::Raw));
    const auto weights = fixture.state.sigma_weights;
    fixture.state.sigma_weights.scale = zero_w;
    EXPECT_EQ(Upf<A>::step(&fixture.state, &zv, &model, sample, &sampler, step_ws.view()), LINEARS_ERROR_DOMAIN);
    fixture.state.sigma_weights = weights;
    EXPECT_EQ(transition.calls, 0); EXPECT_EQ(observation.calls, 0); EXPECT_EQ(sampler.calls, 0);
    EXPECT_EQ(fixture.snapshot(), snapshot);
    /* Callback, sampler and residual failures in the middle of the particle loop commit nothing. */
    transition.fail_at = 7;
    EXPECT_EQ(Upf<A>::step(&fixture.state, &zv, &model, sample, &sampler, step_ws.view()), LINEARS_ERROR_NO_CONVERGENCE);
    EXPECT_EQ(transition.calls, 8);
    transition.fail_at = -1; transition.calls = 0;
    observation.fail_at = 12; observation.failure = LINEARS_ERROR_SINGULAR;
    EXPECT_EQ(Upf<A>::step(&fixture.state, &zv, &model, sample, &sampler, step_ws.view()), LINEARS_ERROR_SINGULAR);
    EXPECT_EQ(observation.calls, 13);
    observation.fail_at = -1; observation.calls = 0; transition.calls = 0;
    sampler.fail_at = 2; sampler.failure = LINEARS_ERROR_DOMAIN;
    EXPECT_EQ(Upf<A>::step(&fixture.state, &zv, &model, sample, &sampler, step_ws.view()), LINEARS_ERROR_DOMAIN);
    EXPECT_EQ(sampler.calls, 3);
    sampler.fail_at = -1; sampler.calls = 0; transition.calls = 0; observation.calls = 0;
    Model failing_residual;
    failing_residual.residual = [](const Vec& a, const Vec&) { return a; };
    failing_residual.fail_at = 9; failing_residual.failure = LINEARS_ERROR_NO_CONVERGENCE;
    auto with_residual = model;
    with_residual.measurement_residual = residual_callback<A>;
    with_residual.measurement_residual_context = &failing_residual;
    EXPECT_EQ(Upf<A>::step(&fixture.state, &zv, &with_residual, sample, &sampler, step_ws.view()), LINEARS_ERROR_NO_CONVERGENCE);
    EXPECT_EQ(failing_residual.calls, 10);
    transition.calls = 0; observation.calls = 0; sampler.calls = 0;
    if constexpr (A::bits == 0) {
        sampler.nonfinite = true;
        EXPECT_EQ(Upf<A>::step(&fixture.state, &zv, &model, sample, &sampler, step_ws.view()), LINEARS_ERROR_RANGE);
        sampler.nonfinite = false;
        transition.calls = 0; observation.calls = 0; sampler.calls = 0;
    }
    /* A singular per-particle covariance fails the proposal Cholesky. */
    std::fill(fixture.covariances.begin() + 1 + 2 * 4, fixture.covariances.begin() + 1 + 3 * 4, work_encode<A>(0));
    const auto singular = fixture.snapshot();
    EXPECT_EQ(Upf<A>::step(&fixture.state, &zv, &model, sample, &sampler, step_ws.view()), LINEARS_ERROR_NOT_POSITIVE_DEFINITE);
    EXPECT_EQ(fixture.snapshot(), singular);
    std::memcpy(fixture.covariances.data() + 1 + 2 * 4, fixture.covariances.data() + 1, 4 * sizeof(Work<A>));
    EXPECT_EQ(fixture.snapshot(), snapshot);
    /* Resample validation, then a resample that moves covariances with their particles. */
    EXPECT_EQ(Upf<A>::resample(&fixture.state, one, indices.data(), count, resample_ws.view()), LINEARS_ERROR_DOMAIN);
    EXPECT_EQ(Upf<A>::resample(&fixture.state, work_encode<A>(0.5L), nullptr, count, resample_ws.view()), LINEARS_ERROR_ARGUMENT);
    EXPECT_EQ(Upf<A>::resample(&fixture.state, work_encode<A>(0.5L), indices.data(), count - 1, resample_ws.view()), LINEARS_ERROR_WORKSPACE);
    auto short_ws = resample_ws.view(); --short_ws.capacity;
    EXPECT_EQ(Upf<A>::resample(&fixture.state, work_encode<A>(0.5L), indices.data(), count, short_ws), LINEARS_ERROR_WORKSPACE);
    for (auto index : indices) EXPECT_EQ(index, SIZE_MAX);
    EXPECT_EQ(fixture.snapshot(), snapshot);
    for (std::size_t i = 0; i < count; ++i) {
        fixture.covariances[1 + i * 4] = work_encode<A>(0.25L + 0.01L * static_cast<long double>(i));
        fixture.weights[1 + i] = work_encode<A>(i == 1 ? 0.6L : 0.1L);
    }
    const std::vector<Vec> before = fixture.particle_rows();
    const std::vector<Dense> covariances_before = fixture.covariance_rows();
    ASSERT_EQ(Upf<A>::resample(&fixture.state, work_encode<A>(0.25L), indices.data(), count, resample_ws.view()), LINEARS_OK);
    EXPECT_EQ(indices, (std::vector<std::size_t>{0, 1, 1, 1, 3}));
    for (std::size_t i = 0; i < count; ++i) {
        EXPECT_EQ(fixture.particle_rows()[i], before[indices[i]]);
        EXPECT_EQ(fixture.covariance_rows()[i].values, covariances_before[indices[i]].values);
        expect_near<A>(fixture.weight_values()[i], 0.2L);
    }
    ASSERT_EQ(Upf<A>::step(&fixture.state, &zv, &model, sample, &sampler, step_ws.view()), LINEARS_OK);
    fixture.guards(); init_ws.guards(); step_ws.guards(); resample_ws.guards();
}

TEST(UnscentedParticleFilterWorkspace, ConstantExpressionsAndOverflowSentinels) {
    static_assert(FILTERS_UKF_UPDATE_WORK_STORAGE(2, 1) == 75, "kernel scratch inside the step");
    static_assert(FILTERS_UPF_STEP_STORAGE(10, 2, 1) == 10 * 8 + 12 + 12 + 1 + 2 + 1 + 75, "step storage");
    static_assert(FILTERS_UPF_Q_STEP_STORAGE(10, 2, 1) == FILTERS_UPF_STEP_STORAGE(10, 2, 1) + 4 + 1 + 1, "Q step storage");
    static_assert(FILTERS_UPF_STEP_BASIC_BYTES(2, 1) == 10 * sizeof(linears_workspace_unit_t), "step basic");
    static_assert(FILTERS_UPF_STEP_DECOMP_STORAGE(2, 1) == 6, "prior exponent solve dominates");
    static_assert(FILTERS_UPF_STEP_DECOMP_STORAGE(1, 3) == 12, "gain solve dominates");
    static_assert(FILTERS_UPF_RESAMPLE_STORAGE(10, 2) == 60, "resample storage");
    const std::size_t huge = SIZE_MAX / 2 + 1;
    EXPECT_EQ(FILTERS_UPF_STEP_STORAGE(huge, 2, 1), SIZE_MAX);
    EXPECT_EQ(FILTERS_UPF_STEP_STORAGE(1, huge, 1), SIZE_MAX);
    EXPECT_EQ(FILTERS_UPF_STEP_STORAGE(1, 1, huge), SIZE_MAX);
    EXPECT_EQ(FILTERS_UPF_Q_STEP_STORAGE(huge, 1, 1), SIZE_MAX);
    EXPECT_EQ(FILTERS_UPF_STEP_BASIC_BYTES(huge, 1), SIZE_MAX);
    EXPECT_EQ(FILTERS_UPF_STEP_DECOMP_STORAGE(huge, 1), SIZE_MAX);
    EXPECT_EQ(FILTERS_UPF_RESAMPLE_STORAGE(huge, 2), SIZE_MAX);
    float step[FILTERS_UPF_STEP_STORAGE(10, 2, 1)];
    linears_fixed_t q_resample[FILTERS_UPF_RESAMPLE_STORAGE(10, 2)];
    linears_workspace_unit_t basic[LINEARS_WORKSPACE_UNITS(FILTERS_UPF_STEP_BASIC_BYTES(2, 1))];
    EXPECT_EQ(sizeof step / sizeof step[0], 183u);
    EXPECT_EQ(sizeof q_resample / sizeof q_resample[0], 60u);
    EXPECT_EQ(sizeof basic / sizeof basic[0], 10u);
}

} // namespace
