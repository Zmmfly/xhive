#include "ukf_support.h"

namespace {
using namespace ukf_test;

template<class A> class UnscentedKernels : public ::testing::Test {};
TYPED_TEST_SUITE(UnscentedKernels, KernelFormats);
template<class A> class UnscentedKalman : public ::testing::Test {};
TYPED_TEST_SUITE(UnscentedKalman, Formats);

TYPED_TEST(UnscentedKernels, WeightsMatchClosedFormAndRejectInvalidParameters) {
    using A = TypeParam;
    const long double cases[][4] = {{1, 1, 2, 0}, {2, 1, 2, 1}, {3, 0.5L, 0, 0}, {2, 0.25L, 2, -1}, {4, 1, 2, -1}};
    for (const auto& c : cases) {
        const std::size_t n = static_cast<std::size_t>(c[0]);
        const long double alpha = c[1], beta = c[2], kappa = c[3];
        SCOPED_TRACE(::testing::Message() << "n=" << n << " alpha=" << static_cast<double>(alpha) << " kappa=" << static_cast<double>(kappa));
        typename Ukf<A>::Weights w{};
        ASSERT_EQ(UkfKernels<A>::weights(n, work_encode<A>(alpha), work_encode<A>(beta), work_encode<A>(kappa), &w), LINEARS_OK);
        const long double a = work_decode<A>(work_encode<A>(alpha)), b = work_decode<A>(work_encode<A>(beta));
        const long double k = work_decode<A>(work_encode<A>(kappa));
        const long double scale = a * a * (n + k), lambda = scale - n;
        expect_near<A>(work_decode<A>(w.scale), scale);
        expect_near<A>(work_decode<A>(w.mean0), lambda / scale);
        expect_near<A>(work_decode<A>(w.covariance0), lambda / scale + 1 - a * a + b);
        expect_near<A>(work_decode<A>(w.other), 1 / (2 * scale));
        expect_near<A>(work_decode<A>(w.mean0) + 2 * n * work_decode<A>(w.other), 1);
    }
    typename Ukf<A>::Weights w{}, before = w;
    EXPECT_EQ(UkfKernels<A>::weights(2, work_encode<A>(1), work_encode<A>(2), work_encode<A>(0), nullptr), LINEARS_ERROR_ARGUMENT);
    EXPECT_EQ(UkfKernels<A>::weights(0, work_encode<A>(1), work_encode<A>(2), work_encode<A>(0), &w), LINEARS_ERROR_DIMENSION);
    EXPECT_EQ(UkfKernels<A>::weights(2, work_encode<A>(0), work_encode<A>(2), work_encode<A>(0), &w), LINEARS_ERROR_DOMAIN);
    EXPECT_EQ(UkfKernels<A>::weights(2, work_encode<A>(-1), work_encode<A>(2), work_encode<A>(0), &w), LINEARS_ERROR_DOMAIN);
    EXPECT_EQ(UkfKernels<A>::weights(2, work_encode<A>(1), work_encode<A>(2), work_encode<A>(-2), &w), LINEARS_ERROR_DOMAIN);
    EXPECT_EQ(UkfKernels<A>::weights(2, work_encode<A>(1), work_encode<A>(2), work_encode<A>(-3), &w), LINEARS_ERROR_DOMAIN);
    if constexpr (A::bits == 0) {
        const auto nan = std::numeric_limits<Work<A>>::quiet_NaN();
        EXPECT_EQ(UkfKernels<A>::weights(2, nan, work_encode<A>(2), work_encode<A>(0), &w), LINEARS_ERROR_RANGE);
        EXPECT_EQ(UkfKernels<A>::weights(2, work_encode<A>(1), nan, work_encode<A>(0), &w), LINEARS_ERROR_RANGE);
    }
    EXPECT_EQ(bytes(w), bytes(before));
}

TYPED_TEST(UnscentedKalman, LinearModelMatchesLinearKalmanFilter) {
    using A = TypeParam;
    std::mt19937 random(0x554b4631u);
    for (int trial = 0; trial < 36; ++trial) {
        const std::size_t n = trial % 3 + 1, m = (trial / 3) % 3 + 1;
        SCOPED_TRACE(::testing::Message() << "trial=" << trial << " n=" << n << " m=" << m);
        const Dense x0 = random_dense(random, n, 1, A::bits ? 0.1L : 1);
        const Dense p0 = random_spd(random, n, A::bits ? 0.125L : 1, A::bits ? 0.08L : 0.4L);
        const Dense q = Dense::identity(n, A::bits ? 0.002L : 0.01L);
        const Dense f = Dense::identity(n, A::bits ? 0.7L : 1) + random_dense(random, n, n, A::bits ? 0.025L : 0.05L);
        const Dense z = random_dense(random, m, 1, A::bits ? 0.1L : 1);
        const Dense h = random_dense(random, m, n, A::bits ? 0.2L : 0.7L);
        const Dense r = random_spd(random, m, A::bits ? 0.125L : 0.2L, A::bits ? 0.08L : 0.4L);
        Matrix<A> xm(x0), pm(p0), qm(q), fm(f), zm(z), hm(h), rm(r);
        auto xv = xm.view(), pv = pm.view(), qv = qm.view(), fv = fm.view(), zv = zm.view(), hv = hm.view(), rv = rm.view();
        /* Callbacks apply the decoded raw F/H, exactly what the linear filter uses. */
        Model transition{[F = decode<A>(fv)](const Vec& x) { return linear_map(F, x); }};
        Model observation{[H = decode<A>(hv)](const Vec& x) { return linear_map(H, x); }};
        UkfState<A> ukf(n, m);
        MatrixState<A> kf(n);
        auto predict_ws = UkfWorkspace<A>::predict(n);
        auto update_ws = UkfWorkspace<A>::update(n, m);
        Workspace<A> kf_predict(n), kf_update(n, m);
        /* kappa = 3-n keeps the classic fourth-moment match; alpha = 1 keeps Q precision. */
        ASSERT_EQ(Ukf<A>::init(&ukf.state, work_encode<A>(1), work_encode<A>(2), work_encode<A>(3.0L - n),
                               &xv, &pv, &qv, &rv), LINEARS_OK);
        ASSERT_EQ(A::reset_matrix(&kf.state, &xv, &pv), LINEARS_OK);
        ASSERT_EQ(Ukf<A>::predict(&ukf.state, map_callback<A>, &transition, nullptr, nullptr, predict_ws.view()), LINEARS_OK);
        ASSERT_EQ(A::predict_matrix(&kf.state, &fv, &qv, kf_predict.view()), LINEARS_OK);
        EXPECT_EQ(transition.calls, static_cast<int>(2 * n + 1));
        expect_matrix<A>(ukf.state.estimate, decode<A>(kf.state.estimate));
        expect_matrix<A>(ukf.state.covariance, decode<A>(kf.state.covariance));
        /* Continue both from the same prior so only the update step is compared. */
        ASSERT_EQ(A::reset_matrix(&kf.state, &ukf.state.estimate, &ukf.state.covariance), LINEARS_OK);
        ASSERT_EQ(Ukf<A>::update(&ukf.state, &zv, map_callback<A>, &observation, nullptr, nullptr, update_ws.view()), LINEARS_OK);
        ASSERT_EQ(A::update_matrix(&kf.state, &zv, &hv, &rv, kf_update.view()), LINEARS_OK);
        EXPECT_EQ(observation.calls, static_cast<int>(2 * n + 1));
        expect_matrix<A>(ukf.state.estimate, decode<A>(kf.state.estimate));
        expect_matrix<A>(ukf.state.covariance, decode<A>(kf.state.covariance));
        const Dense covariance = decode<A>(ukf.state.covariance);
        for (std::size_t i = 0; i < n; ++i)
            for (std::size_t j = 0; j < n; ++j) EXPECT_EQ(covariance(i, j), covariance(j, i));
        EXPECT_EQ(decode<A>(ukf.state.process_noise).values, decode<A>(qv).values);
        ukf.guards(); kf.guards(); predict_ws.guards(); update_ws.guards(); kf_predict.guards(); kf_update.guards();
    }
}

TYPED_TEST(UnscentedKalman, NonlinearPendulumMatchesIndependentReference) {
    using A = TypeParam;
    const long double dt = 0.1L, gravity = 0.5L;
    Model transition{[=](const Vec& s) { return Vec{s[0] + dt * s[1], s[1] - dt * gravity * std::sin(s[0])}; }};
    Model observation{[](const Vec& s) { return Vec{std::sin(s[0]), s[1]}; }};
    Matrix<A> x0(Dense(2, 1, {0.3L, 0})), p0(Dense::identity(2, 0.01L)), q(Dense::identity(2, 1e-4L));
    Matrix<A> r(Dense::identity(2, 0.01L));
    auto xv = x0.view(), pv = p0.view(), qv = q.view(), rv = r.view();
    UkfState<A> fixture(2, 2), twin(2, 2);
    auto predict_ws = UkfWorkspace<A>::predict(2);
    auto update_ws = UkfWorkspace<A>::update(2, 2);
    const long double alpha = A::bits ? 1 : 0.5L, beta = 2, kappa = 1;
    ASSERT_EQ(Ukf<A>::init(&fixture.state, work_encode<A>(alpha), work_encode<A>(beta), work_encode<A>(kappa),
                           &xv, &pv, &qv, &rv), LINEARS_OK);
    ASSERT_EQ(Ukf<A>::init(&twin.state, work_encode<A>(alpha), work_encode<A>(beta), work_encode<A>(kappa),
                           &xv, &pv, &qv, &rv), LINEARS_OK);
    std::mt19937 random(0x50454e44u);
    std::normal_distribution<double> noise(0, 0.05);
    Vec truth{0.3L, 0};
    for (int step = 0; step < 60; ++step) {
        SCOPED_TRACE(step);
        truth = transition.map(truth);
        /* Reseed the reference from the committed state: compare one transaction at a time. */
        UkfReference reference(work_decode<A>(work_encode<A>(alpha)), work_decode<A>(work_encode<A>(beta)),
                               work_decode<A>(work_encode<A>(kappa)), decode<A>(fixture.state.estimate),
                               decode<A>(fixture.state.covariance));
        reference.predict(transition, decode<A>(qv));
        ASSERT_EQ(Ukf<A>::predict(&fixture.state, map_callback<A>, &transition, nullptr, nullptr, predict_ws.view()), LINEARS_OK);
        ASSERT_EQ(Ukf<A>::predict(&twin.state, map_callback<A>, &transition, nullptr, nullptr, predict_ws.view()), LINEARS_OK);
        expect_matrix<A>(fixture.state.estimate, reference.estimate());
        expect_matrix<A>(fixture.state.covariance, reference.covariance_matrix());
        const Vec observed = observation.map(truth);
        Matrix<A> z(Dense(2, 1, {observed[0] + noise(random), observed[1] + noise(random)}));
        auto zv = z.view();
        reference = UkfReference(work_decode<A>(work_encode<A>(alpha)), work_decode<A>(work_encode<A>(beta)),
                                 work_decode<A>(work_encode<A>(kappa)),
                                 decode<A>(fixture.state.estimate), decode<A>(fixture.state.covariance));
        reference.update(decode<A>(zv), observation, decode<A>(rv));
        ASSERT_EQ(Ukf<A>::update(&fixture.state, &zv, map_callback<A>, &observation, nullptr, nullptr, update_ws.view()), LINEARS_OK);
        ASSERT_EQ(Ukf<A>::update(&twin.state, &zv, map_callback<A>, &observation, nullptr, nullptr, update_ws.view()), LINEARS_OK);
        expect_matrix<A>(fixture.state.estimate, reference.estimate());
        expect_matrix<A>(fixture.state.covariance, reference.covariance_matrix());
        EXPECT_EQ(fixture.x, twin.x); EXPECT_EQ(fixture.p, twin.p);
    }
    /* The filter tracked the truth: the posterior sits within a few sigma of it. */
    const Dense estimate = decode<A>(fixture.state.estimate), covariance = decode<A>(fixture.state.covariance);
    for (std::size_t i = 0; i < 2; ++i) {
        EXPECT_GT(covariance(i, i), 0);
        EXPECT_LT(std::abs(estimate.values[i] - truth[i]), 4 * std::sqrt(covariance(i, i)) + 0.02L);
    }
    fixture.guards(); twin.guards(); predict_ws.guards(); update_ws.guards();
}

TYPED_TEST(UnscentedKalman, AngleResidualAveragesAcrossTheWrapBoundary) {
    using A = TypeParam;
    /* Floating angles are radians on [-pi,pi); Q angles are theta/pi on [-1,1). */
    const long double period = A::bits ? 2 : 2 * std::acos(-1.0L), half = period / 2;
    const long double step = A::bits ? 0.03L : 0.1L, sigma = A::bits ? 0.04L : 0.2L;
    const long double start = half - (A::bits ? 0.05L : 0.15L);
    Model transition{[=](const Vec& s) { return Vec{wrap(s[0] + step, period)}; }};
    Model observation{[=](const Vec& s) { return Vec{wrap(s[0], period)}; }};
    Model angle;
    angle.residual = [=](const Vec& a, const Vec& b) { return Vec{wrap(a[0] - b[0], period)}; };
    Matrix<A> x0(Dense(1, 1, {start})), p0(Dense(1, 1, {sigma * sigma})), q(Dense(1, 1, {0}));
    Matrix<A> r(Dense(1, 1, {sigma * sigma}));
    auto xv = x0.view(), pv = p0.view(), qv = q.view(), rv = r.view();
    const long double x_start = decode<A>(xv).values[0], p_start = decode<A>(pv).values[0];
    const long double r_value = decode<A>(rv).values[0];
    UkfState<A> wrapped(1, 1), naive(1, 1);
    auto predict_ws = UkfWorkspace<A>::predict(1);
    auto update_ws = UkfWorkspace<A>::update(1, 1);
    ASSERT_EQ(Ukf<A>::init(&wrapped.state, work_encode<A>(1), work_encode<A>(2), work_encode<A>(0), &xv, &pv, &qv, &rv), LINEARS_OK);
    ASSERT_EQ(Ukf<A>::init(&naive.state, work_encode<A>(1), work_encode<A>(2), work_encode<A>(0), &xv, &pv, &qv, &rv), LINEARS_OK);
    /* One sigma point crosses the boundary after the shift, the other does not. */
    ASSERT_GT(x_start + std::sqrt(p_start) + step, half);
    ASSERT_LT(x_start + step, half);
    ASSERT_EQ(Ukf<A>::predict(&wrapped.state, map_callback<A>, &transition, residual_callback<A>, &angle, predict_ws.view()), LINEARS_OK);
    const long double predicted = x_start + step;
    expect_matrix<A>(wrapped.state.estimate, Dense(1, 1, {predicted}));
    expect_matrix<A>(wrapped.state.covariance, Dense(1, 1, {p_start}));
    /* Without the residual the two outer points average across the cut. The
     * floating mean lands near zero; the Q covariance (about 2.9) leaves the
     * signal interval, so that filter fails instead of committing garbage. */
    if constexpr (A::bits == 0) {
        ASSERT_EQ(Ukf<A>::predict(&naive.state, map_callback<A>, &transition, nullptr, nullptr, predict_ws.view()), LINEARS_OK);
        EXPECT_GT(std::abs(decode<A>(naive.state.estimate).values[0] - predicted), half / 2);
    } else {
        const auto before = naive.snapshot();
        EXPECT_EQ(Ukf<A>::predict(&naive.state, map_callback<A>, &transition, nullptr, nullptr, predict_ws.view()), LINEARS_ERROR_RANGE);
        EXPECT_EQ(naive.snapshot(), before);
    }
    /* Measure beyond the boundary: the innovation must be the short way around. */
    Matrix<A> z(Dense(1, 1, {wrap(predicted + step, period)}));
    auto zv = z.view();
    ASSERT_LT(decode<A>(zv).values[0], 0);
    ASSERT_GT(predicted, 0);
    ASSERT_EQ(Ukf<A>::update(&wrapped.state, &zv, map_callback<A>, &observation, residual_callback<A>, &angle, update_ws.view()), LINEARS_OK);
    const long double gain = p_start / (p_start + r_value);
    const long double innovation = wrap(decode<A>(zv).values[0] - predicted, period);
    expect_matrix<A>(wrapped.state.estimate, Dense(1, 1, {predicted + gain * innovation}));
    expect_matrix<A>(wrapped.state.covariance, Dense(1, 1, {p_start - gain * p_start}));
    EXPECT_GT(decode<A>(wrapped.state.estimate).values[0], predicted);
    wrapped.guards(); naive.guards(); predict_ws.guards(); update_ws.guards();
}

TYPED_TEST(UnscentedKernels, DecomposedStepsComposeIntoPredictAndAcceptArbitraryWeights) {
    using A = TypeParam;
    const std::size_t n = 2, s = 2 * n + 1;
    Model transition{[](const Vec& x) { return Vec{x[0] + 0.1L * x[1], 0.9L * x[1] - 0.05L * x[0] * x[0]}; }};
    Dense x0(2, 1, {0.2L, -0.1L}), p0(2, 2, {0.02L, 0.005L, 0.005L, 0.01L}), q(Dense::identity(2, 0.001L));
    Dense r(Dense::identity(1, 0.01L));
    Matrix<A> xm(x0), pm(p0), qm(q), rm(r);
    auto xv = xm.view(), pv = pm.view(), qv = qm.view(), rv = rm.view();
    UkfState<A> filter(n, 1);
    auto predict_ws = UkfWorkspace<A>::predict(n);
    ASSERT_EQ(Ukf<A>::init(&filter.state, work_encode<A>(1), work_encode<A>(2), work_encode<A>(1), &xv, &pv, &qv, &rv), LINEARS_OK);
    const typename Ukf<A>::Weights w = filter.state.weights;
    /* Manual composition in working format. */
    WorkMatrix<A> mean_in(decode<A>(xv)), cov_in(decode<A>(pv)), q_in(decode<A>(qv));
    WorkMatrix<A> points(s, n), propagated(s, n), deviations(s, n), mean(n, 1), covariance(n, n), wm(s, 1), wc(s, 1);
    for (std::size_t i = 0; i < s; ++i) {
        wm.data[i] = i == 0 ? w.mean0 : w.other;
        wc.data[i] = i == 0 ? w.covariance0 : w.other;
    }
    auto mean_v = mean_in.view(), cov_v = cov_in.view(), q_v = q_in.view(), points_v = points.view();
    auto prop_v = propagated.view(), dev_v = deviations.view(), out_mean = mean.view(), out_cov = covariance.view();
    auto wm_v = wm.view(), wc_v = wc.view();
    UkfWorkspace<A> sigma_ws(FILTERS_UKF_SIGMA_POINTS_STORAGE(n), FILTERS_UKF_SIGMA_POINTS_BASIC_BYTES(n), FILTERS_UKF_SIGMA_POINTS_DECOMP_STORAGE(n));
    UkfWorkspace<A> mean_ws(FILTERS_UKF_MEAN_STORAGE(s, n), FILTERS_UKF_MEAN_BASIC_BYTES(s, n), 0);
    UkfWorkspace<A> dev_ws(0, FILTERS_UKF_DEVIATIONS_BASIC_BYTES(n), 0);
    UkfWorkspace<A> cov_ws(FILTERS_UKF_WEIGHTED_COVARIANCE_STORAGE(s, n, n), FILTERS_UKF_WEIGHTED_COVARIANCE_BASIC_BYTES(s, n, n), 0);
    ASSERT_EQ(UkfKernels<A>::sigma_points(&mean_v, &cov_v, w.scale, &points_v, sigma_ws.view()), LINEARS_OK);
    {
        /* Rows 1..n and n+1..2n are symmetric about row 0, which is the mean. */
        const Dense pts = decode_work<A>(points_v), center = decode_work<A>(mean_v);
        for (std::size_t j = 0; j < n; ++j) {
            EXPECT_EQ(pts(0, j), center.values[j]);
            for (std::size_t i = 0; i < n; ++i)
                expect_near<A>(pts(1 + i, j) - center.values[j], center.values[j] - pts(1 + n + i, j));
        }
        /* Reconstruct scale*P from the spread: sum over i of (row_i - x)(row_i - x)^T. */
        Dense spread(n, n);
        for (std::size_t i = 0; i < n; ++i)
            for (std::size_t a = 0; a < n; ++a)
                for (std::size_t b = 0; b < n; ++b)
                    spread(a, b) += (pts(1 + i, a) - center.values[a]) * (pts(1 + i, b) - center.values[b]);
        for (std::size_t k = 0; k < n * n; ++k)
            expect_near<A>(spread.values[k], work_decode<A>(w.scale) * decode<A>(pv).values[k]);
    }
    ASSERT_EQ(UkfKernels<A>::propagate(&points_v, map_callback<A>, &transition, &prop_v), LINEARS_OK);
    ASSERT_EQ(UkfKernels<A>::mean(&prop_v, &wm_v, nullptr, nullptr, &out_mean, mean_ws.view()), LINEARS_OK);
    ASSERT_EQ(UkfKernels<A>::deviations(&prop_v, &out_mean, nullptr, nullptr, &dev_v, dev_ws.view()), LINEARS_OK);
    ASSERT_EQ(UkfKernels<A>::weighted_covariance(&dev_v, &dev_v, &wc_v, &out_cov, cov_ws.view()), LINEARS_OK);
    Dense expected_p = decode_work<A>(out_cov) + decode<A>(qv);
    ASSERT_EQ(Ukf<A>::predict(&filter.state, map_callback<A>, &transition, nullptr, nullptr, predict_ws.view()), LINEARS_OK);
    expect_matrix<A>(filter.state.estimate, decode_work<A>(out_mean));
    expect_matrix<A>(filter.state.covariance, expected_p);
    /* The working-format predict kernel is the filter without storage. */
    WorkMatrix<A> work_mean(n, 1), work_cov(n, n);
    auto work_mean_v = work_mean.view(), work_cov_v = work_cov.view();
    UkfWorkspace<A> work_ws(FILTERS_UKF_PREDICT_WORK_STORAGE(n), FILTERS_UKF_PREDICT_BASIC_BYTES(n), FILTERS_UKF_PREDICT_DECOMP_STORAGE(n));
    ASSERT_EQ(Ukf<A>::predict_work(&mean_v, &cov_v, &q_v, &w, map_callback<A>, &transition, nullptr, nullptr,
                                   &work_mean_v, &work_cov_v, work_ws.view()), LINEARS_OK);
    for (std::size_t j = 0; j < n; ++j)
        expect_near<A>(decode_work<A>(work_mean_v).values[j], decode_work<A>(out_mean).values[j]);
    for (std::size_t k = 0; k < n * n; ++k)
        expect_near<A>(decode_work<A>(work_cov_v).values[k], expected_p.values[k]);
    /* And the update kernel reproduces a filter update from the same prior. */
    Model observation{[](const Vec& x) { return Vec{x[0] + x[1]}; }};
    WorkMatrix<A> r_in(decode<A>(rv)), z_in(Dense(1, 1, {0.05L})), post_mean(n, 1), post_cov(n, n);
    Matrix<A> z(Dense(1, 1, {0.05L}));
    auto r_v = r_in.view(), z_v = z_in.view(), post_mean_v = post_mean.view(), post_cov_v = post_cov.view(), zv = z.view();
    UkfWorkspace<A> update_work_ws(FILTERS_UKF_UPDATE_WORK_STORAGE(n, 1), FILTERS_UKF_UPDATE_BASIC_BYTES(n, 1), FILTERS_UKF_UPDATE_DECOMP_STORAGE(n, 1));
    auto update_ws = UkfWorkspace<A>::update(n, 1);
    ASSERT_EQ(Ukf<A>::update_work(&work_mean_v, &work_cov_v, &r_v, &z_v, &w, map_callback<A>, &observation, nullptr, nullptr,
                                  &post_mean_v, &post_cov_v, update_work_ws.view()), LINEARS_OK);
    ASSERT_EQ(Ukf<A>::update(&filter.state, &zv, map_callback<A>, &observation, nullptr, nullptr, update_ws.view()), LINEARS_OK);
    expect_matrix<A>(filter.state.estimate, decode_work<A>(post_mean_v));
    expect_matrix<A>(filter.state.covariance, decode_work<A>(post_cov_v));
    EXPECT_EQ(Ukf<A>::predict_work(&mean_v, &cov_v, &q_v, nullptr, map_callback<A>, &transition, nullptr, nullptr,
                                   &work_mean_v, &work_cov_v, work_ws.view()), LINEARS_ERROR_ARGUMENT);
    EXPECT_EQ(Ukf<A>::update_work(&work_mean_v, &work_cov_v, &r_v, &z_v, &w, map_callback<A>, &observation, nullptr, nullptr,
                                  &work_mean_v, &post_cov_v, update_work_ws.view()), LINEARS_ERROR_ALIAS);
    /* The mean with a plain-subtraction residual reproduces the weighted sum. */
    Model plain;
    plain.residual = [](const Vec& a, const Vec& b) { Vec d(a.size()); for (std::size_t i = 0; i < d.size(); ++i) d[i] = a[i] - b[i]; return d; };
    WorkMatrix<A> mean_again(n, 1);
    auto again_v = mean_again.view();
    ASSERT_EQ(UkfKernels<A>::mean(&prop_v, &wm_v, residual_callback<A>, &plain, &again_v, mean_ws.view()), LINEARS_OK);
    EXPECT_EQ(plain.calls, static_cast<int>(s));
    for (std::size_t j = 0; j < n; ++j)
        expect_near<A>(decode_work<A>(again_v).values[j], decode_work<A>(out_mean).values[j]);
    /* Particle-style use: seven equally weighted points of two different widths. */
    std::mt19937 random(0x50465754u);
    const Dense cloud = random_dense(random, 7, 3, 0.2L), other = random_dense(random, 7, 2, 0.2L);
    WorkMatrix<A> cloud_m(cloud), other_m(other), uniform(Dense(7, 1, Vec(7, 1.0L / 7))), cloud_mean(3, 1), other_mean(2, 1);
    WorkMatrix<A> cloud_dev(7, 3), other_dev(7, 2), cross(3, 2);
    auto cloud_v = cloud_m.view(), other_v = other_m.view(), uniform_v = uniform.view();
    auto cloud_mean_v = cloud_mean.view(), other_mean_v = other_mean.view();
    auto cloud_dev_v = cloud_dev.view(), other_dev_v = other_dev.view(), cross_v = cross.view();
    UkfWorkspace<A> pf_mean_ws(FILTERS_UKF_MEAN_STORAGE(7, 3), FILTERS_UKF_MEAN_BASIC_BYTES(7, 3), 0);
    UkfWorkspace<A> pf_dev_ws(0, FILTERS_UKF_DEVIATIONS_BASIC_BYTES(3), 0);
    UkfWorkspace<A> pf_cov_ws(FILTERS_UKF_WEIGHTED_COVARIANCE_STORAGE(7, 3, 2), FILTERS_UKF_WEIGHTED_COVARIANCE_BASIC_BYTES(7, 3, 2), 0);
    ASSERT_EQ(UkfKernels<A>::mean(&cloud_v, &uniform_v, nullptr, nullptr, &cloud_mean_v, pf_mean_ws.view()), LINEARS_OK);
    ASSERT_EQ(UkfKernels<A>::mean(&other_v, &uniform_v, nullptr, nullptr, &other_mean_v, pf_mean_ws.view()), LINEARS_OK);
    ASSERT_EQ(UkfKernels<A>::deviations(&cloud_v, &cloud_mean_v, nullptr, nullptr, &cloud_dev_v, pf_dev_ws.view()), LINEARS_OK);
    ASSERT_EQ(UkfKernels<A>::deviations(&other_v, &other_mean_v, nullptr, nullptr, &other_dev_v, pf_dev_ws.view()), LINEARS_OK);
    ASSERT_EQ(UkfKernels<A>::weighted_covariance(&cloud_dev_v, &other_dev_v, &uniform_v, &cross_v, pf_cov_ws.view()), LINEARS_OK);
    const Dense cloud_d = decode_work<A>(cloud_v), other_d = decode_work<A>(other_v);
    Dense mean_a(3, 1), mean_b(2, 1), expected(3, 2);
    for (std::size_t i = 0; i < 7; ++i) {
        for (std::size_t a = 0; a < 3; ++a) mean_a.values[a] += cloud_d(i, a) / 7;
        for (std::size_t b = 0; b < 2; ++b) mean_b.values[b] += other_d(i, b) / 7;
    }
    for (std::size_t i = 0; i < 7; ++i)
        for (std::size_t a = 0; a < 3; ++a)
            for (std::size_t b = 0; b < 2; ++b)
                expected(a, b) += (cloud_d(i, a) - mean_a.values[a]) * (other_d(i, b) - mean_b.values[b]) / 7;
    for (std::size_t k = 0; k < 3; ++k) expect_near<A>(decode_work<A>(cloud_mean_v).values[k], mean_a.values[k]);
    for (std::size_t k = 0; k < 6; ++k) expect_near<A>(decode_work<A>(cross_v).values[k], expected.values[k]);
    filter.guards(); predict_ws.guards(); sigma_ws.guards(); mean_ws.guards(); dev_ws.guards(); cov_ws.guards();
    work_ws.guards(); update_work_ws.guards(); update_ws.guards();
    pf_mean_ws.guards(); pf_dev_ws.guards(); pf_cov_ws.guards();
}

TYPED_TEST(UnscentedKalman, InitValidationCapacitiesAndAliasRules) {
    using A = TypeParam;
    Matrix<A> x0(Dense(2, 1, {0.25L, -0.125L})), p0(Dense::identity(2, 0.25L)), q(Dense::identity(2, 0.01L));
    Matrix<A> r(Dense::identity(1, 0.25L));
    Matrix<A> asymmetric(Dense(2, 2, {0.25L, 0.125L, 0, 0.25L})), negative(Dense::identity(2, -0.25L));
    auto xv = x0.view(), pv = p0.view(), qv = q.view(), rv = r.view(), av = asymmetric.view(), nv = negative.view();
    UkfState<A> fixture(2, 1);
    const auto one = work_encode<A>(1), two = work_encode<A>(2), zero = work_encode<A>(0);
    EXPECT_EQ(Ukf<A>::init(nullptr, one, two, zero, &xv, &pv, &qv, &rv), LINEARS_ERROR_ARGUMENT);
    EXPECT_EQ(Ukf<A>::init(&fixture.state, one, two, zero, nullptr, &pv, &qv, &rv), LINEARS_ERROR_ARGUMENT);
    EXPECT_EQ(Ukf<A>::init(&fixture.state, one, two, zero, &xv, &pv, &qv, nullptr), LINEARS_ERROR_ARGUMENT);
    EXPECT_EQ(Ukf<A>::init(&fixture.state, zero, two, zero, &xv, &pv, &qv, &rv), LINEARS_ERROR_DOMAIN);
    EXPECT_EQ(Ukf<A>::init(&fixture.state, one, two, work_encode<A>(-2), &xv, &pv, &qv, &rv), LINEARS_ERROR_DOMAIN);
    EXPECT_EQ(Ukf<A>::init(&fixture.state, one, two, zero, &xv, &av, &qv, &rv), LINEARS_ERROR_DOMAIN);
    EXPECT_EQ(Ukf<A>::init(&fixture.state, one, two, zero, &xv, &pv, &nv, &rv), LINEARS_ERROR_DOMAIN);
    EXPECT_EQ(Ukf<A>::init(&fixture.state, one, two, zero, &xv, &pv, &av, &rv), LINEARS_ERROR_DOMAIN);
    auto wrong = rv; wrong.rows = 2;
    EXPECT_EQ(Ukf<A>::init(&fixture.state, one, two, zero, &xv, &pv, &qv, &wrong), LINEARS_ERROR_DIMENSION);
    wrong = xv; wrong.cols = 2;
    EXPECT_EQ(Ukf<A>::init(&fixture.state, one, two, zero, &wrong, &pv, &qv, &rv), LINEARS_ERROR_DIMENSION);
    wrong = xv; wrong.data = nullptr;
    EXPECT_EQ(Ukf<A>::init(&fixture.state, one, two, zero, &wrong, &pv, &qv, &rv), LINEARS_ERROR_ARGUMENT);
    fixture.state.process_noise_capacity = 3;
    EXPECT_EQ(Ukf<A>::init(&fixture.state, one, two, zero, &xv, &pv, &qv, &rv), LINEARS_ERROR_WORKSPACE);
    fixture.state.process_noise_capacity = 4;
    fixture.state.measurement_noise_capacity = 0;
    EXPECT_EQ(Ukf<A>::init(&fixture.state, one, two, zero, &xv, &pv, &qv, &rv), LINEARS_ERROR_WORKSPACE);
    fixture.state.measurement_noise_capacity = 1;
    auto oversized = fixture.state;
    oversized.covariance.rows = oversized.covariance.cols = SIZE_MAX;
    oversized.estimate.rows = SIZE_MAX;
    EXPECT_EQ(Ukf<A>::init(&oversized, one, two, zero, &xv, &pv, &qv, &rv), LINEARS_ERROR_SIZE);
    for (auto& v : fixture.x) EXPECT_EQ(v, UkfState<A>::sentinel);
    if constexpr (A::bits == 0) {
        typename A::Raw bad = std::numeric_limits<typename A::Raw>::quiet_NaN();
        linears_matrix_t nan{&bad, 1, 1};
        EXPECT_EQ(Ukf<A>::init(&fixture.state, one, two, zero, &xv, &pv, &qv, &nan), LINEARS_ERROR_RANGE);
        EXPECT_EQ(Ukf<A>::init(&fixture.state, std::numeric_limits<typename A::Raw>::infinity(), two, zero, &xv, &pv, &qv, &rv), LINEARS_ERROR_RANGE);
    }
    ASSERT_EQ(Ukf<A>::init(&fixture.state, one, two, zero, &xv, &pv, &qv, &rv), LINEARS_OK);
    expect_matrix<A>(fixture.state.estimate, decode<A>(xv));
    expect_matrix<A>(fixture.state.covariance, decode<A>(pv));
    expect_matrix<A>(fixture.state.process_noise, decode<A>(qv));
    expect_matrix<A>(fixture.state.measurement_noise, decode<A>(rv));
    expect_near<A>(work_decode<A>(fixture.state.weights.scale), 2);
    /* Re-initializing from the state's own descriptors or exact storage is allowed. */
    const auto snapshot = fixture.snapshot();
    ASSERT_EQ(Ukf<A>::init(&fixture.state, one, two, zero, &fixture.state.estimate, &fixture.state.covariance,
                           &fixture.state.process_noise, &fixture.state.measurement_noise), LINEARS_OK);
    auto own_x = fixture.state.estimate, own_p = fixture.state.covariance;
    auto own_q = fixture.state.process_noise, own_r = fixture.state.measurement_noise;
    ASSERT_EQ(Ukf<A>::init(&fixture.state, one, two, zero, &own_x, &own_p, &own_q, &own_r), LINEARS_OK);
    EXPECT_EQ(fixture.snapshot(), snapshot);
    auto partial = own_x; partial.data = static_cast<typename A::Raw*>(own_x.data) + 1;
    EXPECT_EQ(Ukf<A>::init(&fixture.state, one, two, zero, &partial, &pv, &qv, &rv), LINEARS_ERROR_ALIAS);
    auto cross = own_q; cross.data = own_p.data; /* Q source aliasing the P storage. */
    EXPECT_EQ(Ukf<A>::init(&fixture.state, one, two, zero, &xv, &pv, &cross, &rv), LINEARS_ERROR_ALIAS);
    auto inside = xv; inside.data = &fixture.state.estimate;
    EXPECT_EQ(Ukf<A>::init(&fixture.state, one, two, zero, &inside, &pv, &qv, &rv), LINEARS_ERROR_ALIAS);
    auto previous = fixture.state.process_noise.data;
    fixture.state.process_noise.data = fixture.state.covariance.data;
    EXPECT_EQ(Ukf<A>::init(&fixture.state, one, two, zero, &xv, &pv, &qv, &rv), LINEARS_ERROR_ALIAS);
    fixture.state.process_noise.data = previous;
    EXPECT_EQ(fixture.snapshot(), snapshot);
    fixture.guards();
}

TYPED_TEST(UnscentedKalman, WorkspaceBoundsAliasAndFailureAtomicity) {
    using A = TypeParam;
    Model transition{[](const Vec& x) { return Vec{0.5L * x[0] + 0.1L * x[1], 0.5L * x[1]}; }};
    Model observation{[](const Vec& x) { return Vec{0.5L * x[0]}; }};
    Matrix<A> x0(Dense(2, 1, {0.25L, -0.125L})), p0(Dense::identity(2, 0.25L)), q(Dense::identity(2, 0.01L));
    Matrix<A> r(Dense::identity(1, 0.25L)), z(Dense(1, 1, {0.125L}));
    auto xv = x0.view(), pv = p0.view(), qv = q.view(), rv = r.view(), zv = z.view();
    UkfState<A> fixture(2, 1);
    auto predict_ws = UkfWorkspace<A>::predict(2);
    auto update_ws = UkfWorkspace<A>::update(2, 1);
    const auto one = work_encode<A>(1), two = work_encode<A>(2), zero = work_encode<A>(0);
    ASSERT_EQ(Ukf<A>::init(&fixture.state, one, two, zero, &xv, &pv, &qv, &rv), LINEARS_OK);
    const auto snapshot = fixture.snapshot();
    EXPECT_EQ(Ukf<A>::predict(nullptr, map_callback<A>, &transition, nullptr, nullptr, predict_ws.view()), LINEARS_ERROR_ARGUMENT);
    EXPECT_EQ(Ukf<A>::predict(&fixture.state, nullptr, &transition, nullptr, nullptr, predict_ws.view()), LINEARS_ERROR_ARGUMENT);
    EXPECT_EQ(Ukf<A>::update(&fixture.state, &zv, nullptr, &observation, nullptr, nullptr, update_ws.view()), LINEARS_ERROR_ARGUMENT);
    EXPECT_EQ(Ukf<A>::update(&fixture.state, nullptr, map_callback<A>, &observation, nullptr, nullptr, update_ws.view()), LINEARS_ERROR_ARGUMENT);
    for (int part = 0; part < 3; ++part) {
        auto w = predict_ws.view();
        if (part == 1 && w.basic_bytes == 0) continue;
        if (part == 0) --w.capacity;
        if (part == 1) --w.basic_bytes;
        if (part == 2) --w.decomposition.capacity;
        EXPECT_EQ(Ukf<A>::predict(&fixture.state, map_callback<A>, &transition, nullptr, nullptr, w), LINEARS_ERROR_WORKSPACE);
        auto u = update_ws.view();
        if (part == 0) --u.capacity;
        if (part == 1) --u.basic_bytes;
        if (part == 2) --u.decomposition.capacity;
        EXPECT_EQ(Ukf<A>::update(&fixture.state, &zv, map_callback<A>, &observation, nullptr, nullptr, u), LINEARS_ERROR_WORKSPACE);
    }
    auto w = predict_ws.view(); w.data = nullptr;
    EXPECT_EQ(Ukf<A>::predict(&fixture.state, map_callback<A>, &transition, nullptr, nullptr, w), LINEARS_ERROR_WORKSPACE);
    if (predict_ws.basic_bytes != 0) {
        w = predict_ws.view(); w.basic = nullptr;
        EXPECT_EQ(Ukf<A>::predict(&fixture.state, map_callback<A>, &transition, nullptr, nullptr, w), LINEARS_ERROR_WORKSPACE);
    }
    w = predict_ws.view(); w.decomposition.data = nullptr;
    EXPECT_EQ(Ukf<A>::predict(&fixture.state, map_callback<A>, &transition, nullptr, nullptr, w), LINEARS_ERROR_WORKSPACE);
    w = predict_ws.view();
    w.data = reinterpret_cast<Work<A>*>(reinterpret_cast<unsigned char*>(w.data) + 1);
    EXPECT_EQ(Ukf<A>::predict(&fixture.state, map_callback<A>, &transition, nullptr, nullptr, w), LINEARS_ERROR_WORKSPACE);
    w = predict_ws.view(); w.decomposition.data = w.data;
    EXPECT_EQ(Ukf<A>::predict(&fixture.state, map_callback<A>, &transition, nullptr, nullptr, w), LINEARS_ERROR_ALIAS);
    auto u = update_ws.view();
    auto aliased = zv; aliased.data = u.data;
    EXPECT_EQ(Ukf<A>::update(&fixture.state, &aliased, map_callback<A>, &observation, nullptr, nullptr, u), LINEARS_ERROR_ALIAS);
    auto wrong = zv; wrong.rows = 2;
    EXPECT_EQ(Ukf<A>::update(&fixture.state, &wrong, map_callback<A>, &observation, nullptr, nullptr, u), LINEARS_ERROR_DIMENSION);
    const auto previous_x = fixture.state.estimate.data;
    fixture.state.estimate.data = w.data;
    EXPECT_EQ(Ukf<A>::predict(&fixture.state, map_callback<A>, &transition, nullptr, nullptr, predict_ws.view()), LINEARS_ERROR_ALIAS);
    fixture.state.estimate.data = previous_x;
    EXPECT_EQ(transition.calls, 0); EXPECT_EQ(observation.calls, 0);
    EXPECT_EQ(fixture.snapshot(), snapshot);
    /* Callback failures and invalid callback outputs are reported without commits. */
    transition.fail_at = 3;
    EXPECT_EQ(Ukf<A>::predict(&fixture.state, map_callback<A>, &transition, nullptr, nullptr, predict_ws.view()), LINEARS_ERROR_NO_CONVERGENCE);
    EXPECT_EQ(transition.calls, 4);
    transition.fail_at = -1;
    observation.fail_at = 0; observation.failure = LINEARS_ERROR_SINGULAR;
    EXPECT_EQ(Ukf<A>::update(&fixture.state, &zv, map_callback<A>, &observation, nullptr, nullptr, update_ws.view()), LINEARS_ERROR_SINGULAR);
    observation.fail_at = -1; observation.calls = 0;
    Model failing_residual;
    failing_residual.residual = [](const Vec& a, const Vec&) { return a; };
    failing_residual.fail_at = 2; failing_residual.failure = LINEARS_ERROR_DOMAIN;
    EXPECT_EQ(Ukf<A>::update(&fixture.state, &zv, map_callback<A>, &observation, residual_callback<A>, &failing_residual, update_ws.view()), LINEARS_ERROR_DOMAIN);
    EXPECT_EQ(observation.calls, 5);
    transition.nonfinite = true;
    EXPECT_EQ(Ukf<A>::predict(&fixture.state, map_callback<A>, &transition, nullptr, nullptr, predict_ws.view()), LINEARS_ERROR_RANGE);
    transition.nonfinite = false;
    EXPECT_EQ(fixture.snapshot(), snapshot);
    /* A semidefinite P has no Cholesky factor; an indefinite R fails before callbacks. */
    Matrix<A> zero_p(Dense::identity(2, 0)), indefinite_r(Dense(1, 1, {0}));
    auto zero_pv = zero_p.view(), bad_rv = indefinite_r.view();
    UkfState<A> semidefinite(2, 1);
    ASSERT_EQ(Ukf<A>::init(&semidefinite.state, one, two, zero, &xv, &zero_pv, &qv, &rv), LINEARS_OK);
    const auto semidefinite_snapshot = semidefinite.snapshot();
    transition.calls = 0;
    EXPECT_EQ(Ukf<A>::predict(&semidefinite.state, map_callback<A>, &transition, nullptr, nullptr, predict_ws.view()), LINEARS_ERROR_NOT_POSITIVE_DEFINITE);
    EXPECT_EQ(Ukf<A>::update(&semidefinite.state, &zv, map_callback<A>, &observation, nullptr, nullptr, update_ws.view()), LINEARS_ERROR_NOT_POSITIVE_DEFINITE);
    EXPECT_EQ(transition.calls, 0);
    EXPECT_EQ(semidefinite.snapshot(), semidefinite_snapshot);
    UkfState<A> bad_noise(2, 1);
    ASSERT_EQ(Ukf<A>::init(&bad_noise.state, one, two, zero, &xv, &pv, &qv, &bad_rv), LINEARS_OK);
    observation.calls = 0;
    EXPECT_EQ(Ukf<A>::update(&bad_noise.state, &zv, map_callback<A>, &observation, nullptr, nullptr, update_ws.view()), LINEARS_ERROR_NOT_POSITIVE_DEFINITE);
    EXPECT_EQ(observation.calls, 0);
    /* Edited noise contents are re-validated; a zeroed weight block is rejected. */
    Matrix<A> edited(Dense(2, 2, {0.01L, 0.005L, 0, 0.01L}));
    std::memcpy(fixture.state.process_noise.data, edited.data.data(), edited.data.size() * sizeof(typename A::Raw));
    EXPECT_EQ(Ukf<A>::predict(&fixture.state, map_callback<A>, &transition, nullptr, nullptr, predict_ws.view()), LINEARS_ERROR_DOMAIN);
    std::memcpy(fixture.state.process_noise.data, q.data.data(), q.data.size() * sizeof(typename A::Raw));
    const auto weights = fixture.state.weights;
    fixture.state.weights.scale = zero;
    EXPECT_EQ(Ukf<A>::predict(&fixture.state, map_callback<A>, &transition, nullptr, nullptr, predict_ws.view()), LINEARS_ERROR_DOMAIN);
    fixture.state.weights = weights;
    EXPECT_EQ(fixture.snapshot(), snapshot);
    ASSERT_EQ(Ukf<A>::predict(&fixture.state, map_callback<A>, &transition, nullptr, nullptr, predict_ws.view()), LINEARS_OK);
    ASSERT_EQ(Ukf<A>::update(&fixture.state, &zv, map_callback<A>, &observation, nullptr, nullptr, update_ws.view()), LINEARS_OK);
    if constexpr (A::bits == 0) {
        /* Public floating-kernel alias rules; Q work entry points are tested separately. */
        WorkMatrix<A> points(5, 2), mapped(5, 2);
        auto points_v = points.view(), mapped_v = mapped.view();
        EXPECT_EQ(UkfKernels<A>::propagate(&points_v, map_callback<A>, &transition, &points_v), LINEARS_ERROR_ALIAS);
        EXPECT_EQ(UkfKernels<A>::propagate(&points_v, nullptr, &transition, &mapped_v), LINEARS_ERROR_ARGUMENT);
        auto narrow = mapped_v; narrow.rows = 4;
        EXPECT_EQ(UkfKernels<A>::propagate(&points_v, map_callback<A>, &transition, &narrow), LINEARS_ERROR_DIMENSION);
        UkfWorkspace<A> mean_ws(FILTERS_UKF_MEAN_STORAGE(5, 2), FILTERS_UKF_MEAN_BASIC_BYTES(5, 2), 0);
        WorkMatrix<A> weights_m(Dense(5, 1, Vec(5, 0.2L)));
        auto weights_v = weights_m.view();
        linears_matrix_t into_scratch{mean_ws.view().data, 2, 1};
        EXPECT_EQ(UkfKernels<A>::mean(&points_v, &weights_v, nullptr, nullptr, &into_scratch, mean_ws.view()), LINEARS_ERROR_ALIAS);
        auto short_ws = mean_ws.view(); --short_ws.capacity;
        WorkMatrix<A> mean_out(2, 1);
        auto mean_out_v = mean_out.view();
        EXPECT_EQ(UkfKernels<A>::mean(&points_v, &weights_v, nullptr, nullptr, &mean_out_v, short_ws), LINEARS_ERROR_WORKSPACE);
        mean_ws.guards();
    }
    fixture.guards(); semidefinite.guards(); bad_noise.guards(); predict_ws.guards(); update_ws.guards();
}

TYPED_TEST(UnscentedKalman, FixedSigmaPointsMayLeaveTheSignalIntervalButResultsMayNot) {
    using A = TypeParam;
    if constexpr (A::bits == 0) {
        GTEST_SKIP() << "Only fixed formats narrow the result to a signal interval.";
    } else {
        Model contract{[](const Vec& x) { return Vec{0.5L * x[0]}; }};
        Model shift{[](const Vec& x) { return Vec{x[0] + 0.2L}; }};
        Matrix<A> x0(Dense(1, 1, {0.9L})), p0(Dense(1, 1, {0.04L})), q(Dense(1, 1, {0})), r(Dense(1, 1, {0.25L}));
        auto xv = x0.view(), pv = p0.view(), qv = q.view(), rv = r.view();
        UkfState<A> fixture(1, 1);
        auto predict_ws = UkfWorkspace<A>::predict(1);
        ASSERT_EQ(Ukf<A>::init(&fixture.state, work_encode<A>(1), work_encode<A>(2), work_encode<A>(0), &xv, &pv, &qv, &rv), LINEARS_OK);
        const long double x_start = decode<A>(xv).values[0], p_start = decode<A>(pv).values[0];
        ASSERT_GT(x_start + std::sqrt(p_start), 1); /* The upper sigma point exceeds +1 in the working type. */
        ASSERT_EQ(Ukf<A>::predict(&fixture.state, map_callback<A>, &contract, nullptr, nullptr, predict_ws.view()), LINEARS_OK);
        expect_matrix<A>(fixture.state.estimate, Dense(1, 1, {0.5L * x_start}));
        expect_matrix<A>(fixture.state.covariance, Dense(1, 1, {0.25L * p_start}));
        ASSERT_EQ(Ukf<A>::init(&fixture.state, work_encode<A>(1), work_encode<A>(2), work_encode<A>(0), &xv, &pv, &qv, &rv), LINEARS_OK);
        const auto snapshot = fixture.snapshot();
        EXPECT_EQ(Ukf<A>::predict(&fixture.state, map_callback<A>, &shift, nullptr, nullptr, predict_ws.view()), LINEARS_ERROR_RANGE);
        EXPECT_EQ(shift.calls, 3);
        EXPECT_EQ(fixture.snapshot(), snapshot);
        fixture.guards(); predict_ws.guards();
    }
}

TEST(UnscentedKalmanWorkspace, ConstantExpressionsAndOverflowSentinels) {
    static_assert(FILTERS_UKF_SIGMA_COUNT(2) == 5, "2n+1 sigma points");
    static_assert(FILTERS_UKF_PREDICT_STORAGE(2) == 8 * 4 + 8 * 2 + 2, "predict storage");
    static_assert(FILTERS_UKF_Q_PREDICT_STORAGE(2) == 7 * 4 + 8 * 2, "Q local predict storage");
    static_assert(FILTERS_UKF_Q_PREDICT_WORK_STORAGE(2) == 4 * 4 + 6 * 2, "Q predict kernel work");
    static_assert(FILTERS_UKF_Q_PREDICT_SIGNAL_STORAGE(2) == 5 * 2, "Q sigma cloud storage");
    static_assert(FILTERS_UKF_UPDATE_STORAGE(2, 1) == 7 * 4 + 2 + 10 * 2 + 7 * 2 + 5 + 2 + 5 * 2, "update storage");
    static_assert(FILTERS_UKF_UPDATE_STORAGE(1, 3) == 7 + 2 * 9 + 10 * 3 + 7 + 15 + 2 + 3 * 3, "wide update storage");
    static_assert(FILTERS_UKF_Q_UPDATE_STORAGE(2, 1) == 2 * 4 + 2 * 4 + 1 + 3 * 2 + 6 * 2 + 9, "Q local update storage");
    static_assert(FILTERS_UKF_Q_UPDATE_WORK_STORAGE(2, 1) == 2 * 4 + 3 * 2 + 4 * 2 + 8, "Q update kernel work");
    static_assert(FILTERS_UKF_Q_UPDATE_SIGNAL_STORAGE(2, 1) == 4 + 5, "Q update signal storage");
    static_assert(FILTERS_UKF_Q_PREDICT_BASIC_BYTES(2) == 0, "Q statistics do not use floating basic scratch");
    static_assert(FILTERS_UKF_Q_UPDATE_BASIC_BYTES(1, 3) == 0, "Q update has no basic scratch");
    static_assert(FILTERS_UKF_Q_UPDATE_DECOMP_STORAGE(1, 3) == 9, "Q triangular solve scratch");
    static_assert(FILTERS_UKF_PREDICT_WORK_STORAGE(2) == 7 * 4 + 7 * 2 + 2, "predict kernel storage");
    static_assert(FILTERS_UKF_PREDICT_STORAGE(2) == FILTERS_UKF_PREDICT_WORK_STORAGE(2) + 2 + 4, "filter adds staging");
    static_assert(FILTERS_UKF_UPDATE_WORK_STORAGE(2, 1) == 6 * 4 + 2 + 10 * 2 + 6 * 2 + 5 + 2 + 5 * 2, "update kernel storage");
    static_assert(FILTERS_UKF_UPDATE_STORAGE(2, 1) == FILTERS_UKF_UPDATE_WORK_STORAGE(2, 1) + 2 + 4, "filter adds staging");
    static_assert(FILTERS_UKF_PREDICT_BASIC_BYTES(2) == 2 * 5 * sizeof(linears_workspace_unit_t), "predict basic");
    static_assert(FILTERS_UKF_UPDATE_BASIC_BYTES(1, 3) == 3 * 3 * sizeof(linears_workspace_unit_t), "update basic");
    static_assert(FILTERS_UKF_PREDICT_DECOMP_STORAGE(3) == 9, "Cholesky of P");
    static_assert(FILTERS_UKF_UPDATE_DECOMP_STORAGE(3, 1) == 9, "Cholesky dominates");
    static_assert(FILTERS_UKF_UPDATE_DECOMP_STORAGE(1, 3) == 12, "solve dominates");
    static_assert(FILTERS_UKF_MEAN_STORAGE(5, 2) == 20, "mean storage");
    static_assert(FILTERS_UKF_WEIGHTED_COVARIANCE_STORAGE(5, 2, 3) == 25, "covariance storage");
    static_assert(FILTERS_UKF_WEIGHTED_COVARIANCE_BASIC_BYTES(5, 2, 3) == 10 * sizeof(linears_workspace_unit_t), "covariance basic");
    const std::size_t huge = SIZE_MAX / 2;
    EXPECT_EQ(FILTERS_UKF_SIGMA_COUNT(huge), SIZE_MAX);
    EXPECT_EQ(FILTERS_UKF_PREDICT_STORAGE(huge), SIZE_MAX);
    EXPECT_EQ(FILTERS_UKF_Q_PREDICT_STORAGE(huge), SIZE_MAX);
    EXPECT_EQ(FILTERS_UKF_UPDATE_STORAGE(huge, 1), SIZE_MAX);
    EXPECT_EQ(FILTERS_UKF_UPDATE_STORAGE(1, huge), SIZE_MAX);
    EXPECT_EQ(FILTERS_UKF_Q_UPDATE_STORAGE(huge, huge), SIZE_MAX);
    EXPECT_EQ(FILTERS_UKF_PREDICT_WORK_STORAGE(huge), SIZE_MAX);
    EXPECT_EQ(FILTERS_UKF_UPDATE_WORK_STORAGE(huge, 1), SIZE_MAX);
    EXPECT_EQ(FILTERS_UKF_PREDICT_BASIC_BYTES(huge), SIZE_MAX);
    EXPECT_EQ(FILTERS_UKF_UPDATE_BASIC_BYTES(huge, 1), SIZE_MAX);
    EXPECT_EQ(FILTERS_UKF_UPDATE_DECOMP_STORAGE(huge, huge), SIZE_MAX);
    EXPECT_EQ(FILTERS_UKF_MEAN_STORAGE(huge, 3), SIZE_MAX);
    EXPECT_EQ(FILTERS_UKF_WEIGHTED_COVARIANCE_STORAGE(huge, 3, 3), SIZE_MAX);
    float f32_predict[FILTERS_UKF_PREDICT_STORAGE(2)];
    double q31_update[FILTERS_UKF_Q_UPDATE_STORAGE(2, 1)];
    linears_workspace_unit_t basic[LINEARS_WORKSPACE_UNITS(FILTERS_UKF_UPDATE_BASIC_BYTES(2, 1))];
    EXPECT_EQ(sizeof f32_predict / sizeof f32_predict[0], 50u);
    EXPECT_EQ(sizeof q31_update / sizeof q31_update[0], 44u);
    EXPECT_EQ(FILTERS_UKF_Q_PREDICT_SIGNAL_STORAGE(huge), SIZE_MAX);
    EXPECT_EQ(FILTERS_UKF_Q_UPDATE_SIGNAL_STORAGE(1, huge), SIZE_MAX);
    EXPECT_EQ(sizeof basic / sizeof basic[0], 10u);
}

} // namespace
