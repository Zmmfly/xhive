#include "pf_support.h"

namespace {
using namespace pf_test;

template<class A> class ParticleKernels : public ::testing::Test {};
TYPED_TEST_SUITE(ParticleKernels, KernelFormats);
template<class A> class ParticleFilter : public ::testing::Test {};
TYPED_TEST_SUITE(ParticleFilter, Formats);

TYPED_TEST(ParticleKernels, ReweightMatchesLongDoubleOverTwentyTwoNats) {
    using A = TypeParam;
    /* Exponents 0..22 in half-nat steps span the range the Q filters can still
     * store after reweighting; negative exponents exercise the shift. */
    Dense weights(45, 1, Vec(45, 1)), exponents(45, 1);
    for (std::size_t i = 0; i < 45; ++i) exponents.values[i] = 0.5L * static_cast<long double>(i);
    std::mt19937 random(0x50465257u);
    for (int trial = 0; trial < 2; ++trial) {
        if (trial == 1) {
            weights = random_dense(random, 45, 1, 0.5L);
            for (auto& w : weights.values) w = std::abs(w);
            for (auto& e : exponents.values) e -= 3;
        }
        WorkMatrix<A> w(weights), e(exponents);
        auto wv = w.view(), ev = e.view();
        ASSERT_EQ(PfKernels<A>::reweight(&wv, &ev), LINEARS_OK);
        const Vec expected = PfReference::reweighted(weights.values, exponents.values);
        long double sum = 0;
        for (std::size_t i = 0; i < 45; ++i) {
            SCOPED_TRACE(i);
            const long double actual = work_decode<A>(w.data[i]);
            expect_near<A>(actual, expected[i]);
            EXPECT_GE(actual, 0);
            sum += actual;
        }
        expect_near<A>(sum, 1, 45);
    }
    WorkMatrix<A> negative(Dense(2, 1, {0.5L, -0.25L})), zero(Dense(2, 1, {0, 0})), e(Dense(2, 1, {0, 1}));
    WorkMatrix<A> three(Dense(3, 1, {1, 1, 1}));
    auto nv = negative.view(), zv = zero.view(), ev = e.view(), tv = three.view();
    EXPECT_EQ(PfKernels<A>::reweight(&nv, &ev), LINEARS_ERROR_DOMAIN);
    EXPECT_EQ(PfKernels<A>::reweight(&zv, &ev), LINEARS_ERROR_DOMAIN);
    EXPECT_EQ(PfKernels<A>::reweight(&tv, &ev), LINEARS_ERROR_DIMENSION);
    EXPECT_EQ(PfKernels<A>::reweight(&ev, &ev), LINEARS_ERROR_ALIAS);
    EXPECT_EQ(PfKernels<A>::reweight(nullptr, &ev), LINEARS_ERROR_ARGUMENT);
    WorkMatrix<A> bad(Dense(2, 1, {1, 1}));
    bad.data[1] = std::numeric_limits<Work<A>>::infinity();
    auto bv = bad.view();
    EXPECT_EQ(PfKernels<A>::reweight(&bv, &ev), LINEARS_ERROR_RANGE);
    EXPECT_EQ(PfKernels<A>::reweight(&ev, &bv), LINEARS_ERROR_RANGE);
}

TYPED_TEST(ParticleKernels, EffectiveSampleSizeClosedForm) {
    using A = TypeParam;
    std::mt19937 random(0x50464553u);
    Dense uniform(8, 1, Vec(8, 0.125L)), one_hot(8, 1), skewed = random_dense(random, 8, 1, 0.1L);
    one_hot.values[3] = 1;
    for (auto& w : skewed.values) w = std::abs(w) + 0.01L;
    for (const Dense *weights : {&uniform, &one_hot, &skewed}) {
        WorkMatrix<A> w(*weights);
        auto wv = w.view();
        Work<A> ess = work_encode<A>(-1);
        ASSERT_EQ(Pf<A>::effective_sample_size(&wv, &ess), LINEARS_OK);
        expect_near<A>(work_decode<A>(ess), PfReference::effective_sample_size(weights->values));
    }
    WorkMatrix<A> zero(Dense(3, 1, {0, 0, 0})), negative(Dense(3, 1, {0.5L, 0.5L, -0.5L}));
    auto zv = zero.view(), nv = negative.view();
    Work<A> ess = work_encode<A>(-1);
    EXPECT_EQ(Pf<A>::effective_sample_size(&zv, &ess), LINEARS_ERROR_DOMAIN);
    EXPECT_EQ(Pf<A>::effective_sample_size(&nv, &ess), LINEARS_ERROR_DOMAIN);
    EXPECT_EQ(Pf<A>::effective_sample_size(&nv, nullptr), LINEARS_ERROR_ARGUMENT);
    auto wide = nv; wide.cols = 3; wide.rows = 1;
    EXPECT_EQ(Pf<A>::effective_sample_size(&wide, &ess), LINEARS_ERROR_DIMENSION);
    EXPECT_EQ(work_decode<A>(ess), -1);
}

TYPED_TEST(ParticleFilter, RawEffectiveSampleSizeIsScaleFree) {
    using A = TypeParam;
    if constexpr (A::bits == 0) {
        GTEST_SKIP() << "Only the Q filters read raw weights.";
    } else {
        /* Raw weights scaled to the maximum, as the filters store them. */
        Matrix<A> weights(Dense(4, 1, {1, 0.5L, 0.25L, 0}));
        auto wv = weights.view();
        Work<A> ess = -1;
        ASSERT_EQ(Pf<A>::effective_sample_size(&wv, &ess), LINEARS_OK);
        expect_near<A>(ess, PfReference::effective_sample_size(decode<A>(wv).values));
        Matrix<A> zero(Dense(3, 1, {0, 0, 0})), negative(Dense(3, 1, {0.5L, 0.5L, -0.5L}));
        auto zv = zero.view(), nv = negative.view();
        ess = -1;
        EXPECT_EQ(Pf<A>::effective_sample_size(&zv, &ess), LINEARS_ERROR_DOMAIN);
        EXPECT_EQ(Pf<A>::effective_sample_size(&nv, &ess), LINEARS_ERROR_DOMAIN);
        EXPECT_EQ(Pf<A>::effective_sample_size(&nv, nullptr), LINEARS_ERROR_ARGUMENT);
        EXPECT_EQ(Pf<A>::effective_sample_size(nullptr, &ess), LINEARS_ERROR_ARGUMENT);
        auto wide = nv; wide.cols = 3; wide.rows = 1;
        EXPECT_EQ(Pf<A>::effective_sample_size(&wide, &ess), LINEARS_ERROR_DIMENSION);
        EXPECT_EQ(ess, -1);
    }
}

TYPED_TEST(ParticleKernels, SystematicResamplingMatchesReferenceAndCounts) {
    using A = TypeParam;
    std::mt19937 random(0x50465253u);
    const std::size_t count = 40;
    for (int trial = 0; trial < 12; ++trial) {
        SCOPED_TRACE(trial);
        Dense weights = random_dense(random, count, 1, 0.5L);
        long double total = 0;
        for (auto& w : weights.values) total += w = (trial % 3 == 0 ? 0 : 0.0125L) + std::abs(w) * (trial % 4 == 1 ? 0.01L : 1);
        if (trial % 3 == 0) { weights.values[5] = 0; weights.values[6] = 0; }
        /* Only the weights the kernel actually sees define the reference targets. */
        WorkMatrix<A> w(weights);
        const Vec seen = decode_work<A>(w.view()).values;
        const long double uniform = work_decode<A>(work_encode<A>(std::uniform_real_distribution<double>(0, 1)(random)));
        std::vector<std::size_t> indices(count + 1, SIZE_MAX);
        auto wv = w.view();
        ASSERT_EQ(PfKernels<A>::resample_indices(&wv, work_encode<A>(uniform), indices.data(), count), LINEARS_OK);
        EXPECT_EQ(indices.back(), SIZE_MAX);
        indices.pop_back();
        EXPECT_EQ(indices, PfReference::systematic(seen, uniform));
        long double seen_total = 0;
        for (auto value : seen) seen_total += value;
        std::vector<std::size_t> copies(count, 0);
        for (std::size_t i = 0; i < count; ++i) {
            ASSERT_LT(indices[i], count);
            if (i) EXPECT_GE(indices[i], indices[i - 1]);
            ++copies[indices[i]];
        }
        for (std::size_t i = 0; i < count; ++i) {
            const long double expected = static_cast<long double>(count) * seen[i] / seen_total;
            EXPECT_GE(static_cast<long double>(copies[i]), std::floor(expected) - 1e-9L);
            EXPECT_LE(static_cast<long double>(copies[i]), std::ceil(expected) + 1e-9L);
        }
    }
    WorkMatrix<A> w(Dense(3, 1, {0.2L, 0.3L, 0.5L})), zero(Dense(3, 1, {0, 0, 0})), negative(Dense(3, 1, {0.5L, 0.6L, -0.1L}));
    auto wv = w.view(), zv = zero.view(), nv = negative.view();
    std::vector<std::size_t> indices(3, SIZE_MAX);
    const auto zero_draw = work_encode<A>(0), half = work_encode<A>(0.5L);
    EXPECT_EQ(PfKernels<A>::resample_indices(&wv, work_encode<A>(1), indices.data(), 3), LINEARS_ERROR_DOMAIN);
    EXPECT_EQ(PfKernels<A>::resample_indices(&wv, work_encode<A>(-0.5L), indices.data(), 3), LINEARS_ERROR_DOMAIN);
    EXPECT_EQ(PfKernels<A>::resample_indices(&zv, half, indices.data(), 3), LINEARS_ERROR_DOMAIN);
    EXPECT_EQ(PfKernels<A>::resample_indices(&nv, half, indices.data(), 3), LINEARS_ERROR_DOMAIN);
    EXPECT_EQ(PfKernels<A>::resample_indices(&wv, half, indices.data(), 2), LINEARS_ERROR_WORKSPACE);
    EXPECT_EQ(PfKernels<A>::resample_indices(&wv, half, nullptr, 3), LINEARS_ERROR_ARGUMENT);
    EXPECT_EQ(PfKernels<A>::resample_indices(nullptr, half, indices.data(), 3), LINEARS_ERROR_ARGUMENT);
    EXPECT_EQ(PfKernels<A>::resample_indices(&wv, std::numeric_limits<Work<A>>::quiet_NaN(), indices.data(), 3), LINEARS_ERROR_RANGE);
    for (auto index : indices) EXPECT_EQ(index, SIZE_MAX);
    ASSERT_EQ(PfKernels<A>::resample_indices(&wv, zero_draw, indices.data(), 3), LINEARS_OK);
    EXPECT_EQ(indices, (std::vector<std::size_t>{0, 1, 2}));
    /* A one-hot weight vector copies that particle N times. */
    WorkMatrix<A> hot(Dense(3, 1, {0, 1, 0}));
    auto hv = hot.view();
    ASSERT_EQ(PfKernels<A>::resample_indices(&hv, half, indices.data(), 3), LINEARS_OK);
    EXPECT_EQ(indices, (std::vector<std::size_t>{1, 1, 1}));
}

TYPED_TEST(ParticleKernels, GatherCopiesRowsAndRejectsBadIndices) {
    using A = TypeParam;
    std::mt19937 random(0x50464741u);
    const Dense points = random_dense(random, 5, 3, 0.5L);
    WorkMatrix<A> source(points), out(4, 3);
    auto sv = source.view(), ov = out.view();
    const std::size_t indices[4] = {4, 4, 0, 2};
    ASSERT_EQ(PfKernels<A>::gather(&sv, indices, 4, &ov), LINEARS_OK);
    const Dense seen = decode_work<A>(sv), result = decode_work<A>(ov);
    for (std::size_t i = 0; i < 4; ++i)
        for (std::size_t j = 0; j < 3; ++j) EXPECT_EQ(result(i, j), seen(indices[i], j));
    const auto before = out.data;
    const std::size_t bad[4] = {0, 5, 1, 1};
    EXPECT_EQ(PfKernels<A>::gather(&sv, bad, 4, &ov), LINEARS_ERROR_ARGUMENT);
    EXPECT_EQ(PfKernels<A>::gather(&sv, indices, 3, &ov), LINEARS_ERROR_DIMENSION);
    EXPECT_EQ(PfKernels<A>::gather(&sv, indices, 0, &ov), LINEARS_ERROR_DIMENSION);
    EXPECT_EQ(PfKernels<A>::gather(&sv, nullptr, 4, &ov), LINEARS_ERROR_ARGUMENT);
    EXPECT_EQ(PfKernels<A>::gather(&sv, indices, 5, &sv), LINEARS_ERROR_ALIAS);
    auto narrow = ov; narrow.cols = 2;
    EXPECT_EQ(PfKernels<A>::gather(&sv, indices, 4, &narrow), LINEARS_ERROR_DIMENSION);
    EXPECT_EQ(out.data, before);
}

TYPED_TEST(ParticleKernels, PerturbAddsFactoredDrawsInPlaceOrToACopy) {
    using A = TypeParam;
    std::mt19937 random(0x50465054u);
    const Dense points = random_dense(random, 4, 2, 0.5L);
    const Dense covariance = random_spd(random, 2, 0.01L, 0.05L);
    const Dense factor = dense_matrix(PfReference::cholesky(covariance));
    Sampler sampler(1);
    sampler.script = {{0.5L, -1.25L}, {2, 0.75L}, {-0.3L, 0.1L}, {1.5L, -2}};
    WorkMatrix<A> source(points), factor_m(factor), out(4, 2);
    auto sv = source.view(), fv = factor_m.view(), ov = out.view();
    UkfWorkspace<A> ws(FILTERS_PF_PERTURB_STORAGE(2), FILTERS_PF_PERTURB_BASIC_BYTES(2), 0);
    ASSERT_EQ(PfKernels<A>::perturb(&sv, &fv, sample_callback<A>, &sampler, &ov, ws.view()), LINEARS_OK);
    EXPECT_EQ(sampler.calls, 4);
    const Dense seen = decode_work<A>(sv), seen_factor = decode_work<A>(fv);
    const std::vector<Vec> draws = sampler.take_log();
    Dense expected(4, 2);
    for (std::size_t i = 0; i < 4; ++i) {
        Vec row{seen(i, 0), seen(i, 1)};
        row = PfReference::shifted(row, eigen_matrix(seen_factor), draws[i]);
        expected(i, 0) = row[0];
        expected(i, 1) = row[1];
    }
    expect_work<A>(ov, expected);
    /* In place yields the same rows from the same scripted draws. */
    sampler.scripted = 0;
    ASSERT_EQ(PfKernels<A>::perturb(&sv, &fv, sample_callback<A>, &sampler, &sv, ws.view()), LINEARS_OK);
    EXPECT_EQ(source.data, out.data);
    auto partial = sv;
    partial.data = static_cast<Work<A>*>(sv.data) + 1;
    partial.rows = 3;
    auto narrow = sv;
    narrow.rows = 3;
    EXPECT_EQ(PfKernels<A>::perturb(&narrow, &fv, sample_callback<A>, &sampler, &partial, ws.view()), LINEARS_ERROR_ALIAS);
    EXPECT_EQ(PfKernels<A>::perturb(&sv, &fv, sample_callback<A>, &sampler, &narrow, ws.view()), LINEARS_ERROR_DIMENSION);
    EXPECT_EQ(PfKernels<A>::perturb(&sv, &fv, nullptr, &sampler, &ov, ws.view()), LINEARS_ERROR_ARGUMENT);
    auto wrong = fv; wrong.cols = 1;
    EXPECT_EQ(PfKernels<A>::perturb(&sv, &wrong, sample_callback<A>, &sampler, &ov, ws.view()), LINEARS_ERROR_DIMENSION);
    auto short_ws = ws.view(); --short_ws.capacity;
    EXPECT_EQ(PfKernels<A>::perturb(&sv, &fv, sample_callback<A>, &sampler, &ov, short_ws), LINEARS_ERROR_WORKSPACE);
    short_ws = ws.view(); --short_ws.basic_bytes;
    EXPECT_EQ(PfKernels<A>::perturb(&sv, &fv, sample_callback<A>, &sampler, &ov, short_ws), LINEARS_ERROR_WORKSPACE);
    sampler.calls = 0; sampler.fail_at = 2; sampler.failure = LINEARS_ERROR_SINGULAR;
    EXPECT_EQ(PfKernels<A>::perturb(&sv, &fv, sample_callback<A>, &sampler, &ov, ws.view()), LINEARS_ERROR_SINGULAR);
    EXPECT_EQ(sampler.calls, 3);
    sampler.fail_at = -1;
    sampler.nonfinite = true;
    EXPECT_EQ(PfKernels<A>::perturb(&sv, &fv, sample_callback<A>, &sampler, &ov, ws.view()), LINEARS_ERROR_RANGE);
    sampler.nonfinite = false;
    ws.guards();
}

TYPED_TEST(ParticleKernels, GaussianExponentMatchesEigen) {
    using A = TypeParam;
    std::mt19937 random(0x50464745u);
    const Dense deviations = random_dense(random, 6, 3, 0.3L);
    const Dense covariance = random_spd(random, 3, 0.05L, 0.1L);
    const Dense factor = dense_matrix(PfReference::cholesky(covariance));
    WorkMatrix<A> d(deviations), l(factor), out(6, 1);
    auto dv = d.view(), lv = l.view(), ov = out.view();
    UkfWorkspace<A> ws(FILTERS_PF_GAUSSIAN_EXPONENT_STORAGE(3), FILTERS_PF_GAUSSIAN_EXPONENT_BASIC_BYTES,
                       FILTERS_PF_GAUSSIAN_EXPONENT_DECOMP_STORAGE(3));
    ASSERT_EQ(PfKernels<A>::gaussian_exponent(&dv, &lv, &ov, ws.view()), LINEARS_OK);
    /* The reference uses L*L^T of the quantized factor, exactly what the kernel saw. */
    const Dense seen_l = decode_work<A>(lv), seen_d = decode_work<A>(dv);
    std::vector<Vec> rows(6, Vec(3));
    for (std::size_t i = 0; i < 6; ++i)
        for (std::size_t j = 0; j < 3; ++j) rows[i][j] = seen_d(i, j);
    const Vec expected = PfReference::exponents(rows, seen_l * seen_l.transpose());
    for (std::size_t i = 0; i < 6; ++i) {
        SCOPED_TRACE(i);
        expect_near<A>(work_decode<A>(out.data[i]), expected[i]);
        EXPECT_GE(work_decode<A>(out.data[i]), 0);
    }
    Dense singular = factor;
    singular(0, 0) = 0; /* A zero row guarantees an exact zero pivot under any pivoting. */
    WorkMatrix<A> s(singular);
    auto sv = s.view();
    EXPECT_EQ(PfKernels<A>::gaussian_exponent(&dv, &sv, &ov, ws.view()), LINEARS_ERROR_SINGULAR);
    auto wrong = ov; wrong.rows = 5;
    EXPECT_EQ(PfKernels<A>::gaussian_exponent(&dv, &lv, &wrong, ws.view()), LINEARS_ERROR_DIMENSION);
    linears_matrix_t into_input{dv.data, 6, 1};
    EXPECT_EQ(PfKernels<A>::gaussian_exponent(&dv, &lv, &into_input, ws.view()), LINEARS_ERROR_ALIAS);
    auto short_ws = ws.view(); --short_ws.decomposition.capacity;
    EXPECT_EQ(PfKernels<A>::gaussian_exponent(&dv, &lv, &ov, short_ws), LINEARS_ERROR_WORKSPACE);
    ws.guards();
}

TYPED_TEST(ParticleFilter, PendulumTransactionsMatchLongDoubleReference) {
    using A = TypeParam;
    const std::size_t count = 48;
    const long double dt = 0.1L, gravity = 0.5L;
    Model transition{[=](const Vec& s) { return Vec{s[0] + dt * s[1], s[1] - dt * gravity * std::sin(s[0])}; }};
    Model observation{[](const Vec& s) { return Vec{std::sin(s[0]), s[1]}; }};
    const Model plain;
    Matrix<A> x0(Dense(2, 1, {0.3L, 0})), p0(Dense::identity(2, 0.01L)), q(Dense::identity(2, 1e-4L));
    Matrix<A> r(Dense::identity(2, 0.01L)), mean_out(Dense(2, 1)), covariance_out(Dense(2, 2));
    auto xv = x0.view(), pv = p0.view(), qv = q.view(), rv = r.view(), mv = mean_out.view(), cv = covariance_out.view();
    PfState<A> fixture(count, 2, 2);
    auto init_ws = pf_init_workspace<A>(count, 2);
    auto predict_ws = pf_predict_workspace<A>(count, 2);
    auto update_ws = pf_update_workspace<A>(count, 2, 2);
    auto resample_ws = pf_resample_workspace<A>(count);
    auto estimate_ws = pf_estimate_workspace<A>(count, 2);
    Sampler sampler(0x50465044u);
    std::vector<std::size_t> indices(count);
    PfReference reference;
    ASSERT_EQ(Pf<A>::init(&fixture.state, &xv, &pv, &qv, &rv, sample_callback<A>, &sampler, init_ws.view()), LINEARS_OK);
    EXPECT_EQ(sampler.calls, static_cast<int>(count));
    reference.init(decode<A>(xv), decode<A>(pv), sampler.take_log());
    for (std::size_t i = 0; i < count; ++i) {
        for (std::size_t j = 0; j < 2; ++j) expect_near<A>(fixture.particle_rows()[i][j], reference.particles[i][j]);
        expect_near<A>(fixture.weight_values()[i], reference.weights[i]);
    }
    std::mt19937 random(0x50454e44u);
    std::normal_distribution<double> noise(0, 0.1);
    Vec truth{0.3L, 0};
    int resamples = 0;
    for (int step = 0; step < 25; ++step) {
        SCOPED_TRACE(step);
        truth = transition.map(truth);
        /* Reseed the reference from the committed state: one transaction at a time. */
        reference.particles = fixture.particle_rows();
        reference.weights = fixture.weight_values();
        ASSERT_EQ(Pf<A>::predict(&fixture.state, map_callback<A>, &transition, sample_callback<A>, &sampler, predict_ws.view()), LINEARS_OK);
        reference.predict(transition, decode<A>(qv), sampler.take_log());
        for (std::size_t i = 0; i < count; ++i)
            for (std::size_t j = 0; j < 2; ++j) expect_near<A>(fixture.particle_rows()[i][j], reference.particles[i][j]);
        const Vec observed = observation.map(truth);
        Matrix<A> z(Dense(2, 1, {observed[0] + noise(random), observed[1] + noise(random)}));
        auto zv = z.view();
        reference.particles = fixture.particle_rows();
        ASSERT_EQ(Pf<A>::update(&fixture.state, &zv, map_callback<A>, &observation, nullptr, nullptr, update_ws.view()), LINEARS_OK);
        reference.update(decode<A>(zv), observation, decode<A>(rv));
        for (std::size_t i = 0; i < count; ++i) expect_near<A>(fixture.weight_values()[i], reference.weights[i]);
        expect_raw_weight_scale<A>(fixture.weights);
        Work<A> ess = work_encode<A>(0);
        ASSERT_EQ(Pf<A>::effective_sample_size(&fixture.state.weights, &ess), LINEARS_OK);
        expect_near<A>(work_decode<A>(ess), PfReference::effective_sample_size(fixture.weight_values()));
        if (work_decode<A>(ess) < static_cast<long double>(count) / 2) {
            const long double uniform = work_decode<A>(work_encode<A>(sampler.uniform()));
            reference.weights = fixture.weight_values();
            const std::vector<std::size_t> expected = PfReference::systematic(reference.weights, uniform);
            ASSERT_EQ(Pf<A>::resample(&fixture.state, work_encode<A>(uniform), indices.data(), count, resample_ws.view()), LINEARS_OK);
            EXPECT_EQ(indices, expected);
            reference.resample(expected);
            EXPECT_EQ(fixture.particle_rows(), reference.particles);
            for (std::size_t i = 0; i < count; ++i) expect_near<A>(fixture.weight_values()[i], reference.weights[i]);
            expect_raw_weight_scale<A>(fixture.weights);
            ++resamples;
        }
        reference.particles = fixture.particle_rows();
        reference.weights = fixture.weight_values();
        ASSERT_EQ(Pf<A>::estimate(&fixture.state, nullptr, nullptr, &mv, &cv, estimate_ws.view()), LINEARS_OK);
        const Vec mean = reference.mean(plain);
        expect_matrix<A>(mv, Dense(2, 1, mean));
        expect_matrix<A>(cv, reference.covariance(mean, plain));
        const Dense covariance = decode<A>(cv);
        EXPECT_EQ(covariance(0, 1), covariance(1, 0));
    }
    EXPECT_GT(resamples, 0);
    fixture.guards(); init_ws.guards(); predict_ws.guards(); update_ws.guards(); resample_ws.guards(); estimate_ws.guards();
}

TYPED_TEST(ParticleFilter, PendulumTrackingStaysWithinItsOwnSpread) {
    using A = TypeParam;
    const std::size_t count = 400;
    const long double dt = 0.1L, gravity = 0.5L;
    Model transition{[=](const Vec& s) { return Vec{s[0] + dt * s[1], s[1] - dt * gravity * std::sin(s[0])}; }};
    Model observation{[](const Vec& s) { return Vec{std::sin(s[0]), s[1]}; }};
    Matrix<A> x0(Dense(2, 1, {0.3L, 0})), p0(Dense::identity(2, 0.01L)), q(Dense::identity(2, 1e-4L));
    Matrix<A> r(Dense::identity(2, 0.01L)), mean_out(Dense(2, 1)), covariance_out(Dense(2, 2));
    auto xv = x0.view(), pv = p0.view(), qv = q.view(), rv = r.view(), mv = mean_out.view(), cv = covariance_out.view();
    PfState<A> fixture(count, 2, 2);
    auto init_ws = pf_init_workspace<A>(count, 2);
    auto predict_ws = pf_predict_workspace<A>(count, 2);
    auto update_ws = pf_update_workspace<A>(count, 2, 2);
    auto resample_ws = pf_resample_workspace<A>(count);
    auto estimate_ws = pf_estimate_workspace<A>(count, 2);
    Sampler sampler(0x50465452u);
    std::vector<std::size_t> indices(count);
    ASSERT_EQ(Pf<A>::init(&fixture.state, &xv, &pv, &qv, &rv, sample_callback<A>, &sampler, init_ws.view()), LINEARS_OK);
    std::mt19937 random(0x54525554u);
    std::normal_distribution<double> noise(0, 0.1);
    Vec truth{0.3L, 0};
    for (int step = 0; step < 60; ++step) {
        SCOPED_TRACE(step);
        truth = transition.map(truth);
        ASSERT_EQ(Pf<A>::predict(&fixture.state, map_callback<A>, &transition, sample_callback<A>, &sampler, predict_ws.view()), LINEARS_OK);
        const Vec observed = observation.map(truth);
        Matrix<A> z(Dense(2, 1, {observed[0] + noise(random), observed[1] + noise(random)}));
        auto zv = z.view();
        ASSERT_EQ(Pf<A>::update(&fixture.state, &zv, map_callback<A>, &observation, nullptr, nullptr, update_ws.view()), LINEARS_OK);
        Work<A> ess = work_encode<A>(0);
        ASSERT_EQ(Pf<A>::effective_sample_size(&fixture.state.weights, &ess), LINEARS_OK);
        if (work_decode<A>(ess) < static_cast<long double>(count) / 2)
            ASSERT_EQ(Pf<A>::resample(&fixture.state, work_encode<A>(sampler.uniform()), indices.data(), count, resample_ws.view()), LINEARS_OK);
    }
    /* The final posterior sits within a few sigma of the simulated truth. */
    ASSERT_EQ(Pf<A>::estimate(&fixture.state, nullptr, nullptr, &mv, &cv, estimate_ws.view()), LINEARS_OK);
    const Dense estimate = decode<A>(mv), covariance = decode<A>(cv);
    for (std::size_t i = 0; i < 2; ++i) {
        EXPECT_GT(covariance(i, i), 0);
        EXPECT_LT(std::abs(estimate.values[i] - truth[i]), 4 * std::sqrt(covariance(i, i)) + 0.03L);
    }
    fixture.guards(); predict_ws.guards(); update_ws.guards(); resample_ws.guards(); estimate_ws.guards();
}

TYPED_TEST(ParticleFilter, LinearGaussianModelAgreesWithKalmanFilter) {
    using A = TypeParam;
    const std::size_t count = 4000, n = 2, m = 1;
    std::mt19937 random(0x50464b46u);
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
    PfState<A> fixture(count, n, m);
    auto init_ws = pf_init_workspace<A>(count, n);
    auto predict_ws = pf_predict_workspace<A>(count, n);
    auto update_ws = pf_update_workspace<A>(count, n, m);
    auto estimate_ws = pf_estimate_workspace<A>(count, n);
    Sampler sampler(0x4b414c4du);
    ASSERT_EQ(Pf<A>::init(&fixture.state, &xv, &pv, &qv, &rv, sample_callback<A>, &sampler, init_ws.view()), LINEARS_OK);
    ASSERT_EQ(Pf<A>::predict(&fixture.state, map_callback<A>, &transition, sample_callback<A>, &sampler, predict_ws.view()), LINEARS_OK);
    EXPECT_EQ(transition.calls, static_cast<int>(count));
    ASSERT_EQ(Pf<A>::update(&fixture.state, &zv, map_callback<A>, &observation, nullptr, nullptr, update_ws.view()), LINEARS_OK);
    EXPECT_EQ(observation.calls, static_cast<int>(count));
    Work<A> ess = work_encode<A>(0);
    ASSERT_EQ(Pf<A>::effective_sample_size(&fixture.state.weights, &ess), LINEARS_OK);
    EXPECT_GT(work_decode<A>(ess), static_cast<long double>(count) / 5);
    ASSERT_EQ(Pf<A>::estimate(&fixture.state, nullptr, nullptr, &mv, &cv, estimate_ws.view()), LINEARS_OK);
    /* Monte Carlo agreement: the sampling error is a few percent of sigma for N_eff near 1000. */
    const Dense kf_mean = decode<A>(kf.state.estimate), kf_cov = decode<A>(kf.state.covariance);
    const Dense pf_mean = decode<A>(mv), pf_cov = decode<A>(cv);
    for (std::size_t i = 0; i < n; ++i) {
        const long double sigma = std::sqrt(kf_cov(i, i));
        EXPECT_LT(std::abs(pf_mean.values[i] - kf_mean.values[i]), 0.2L * sigma) << i;
        for (std::size_t j = 0; j < n; ++j)
            EXPECT_LT(std::abs(pf_cov(i, j) - kf_cov(i, j)), 0.3L * std::sqrt(kf_cov(i, i) * kf_cov(j, j))) << i << j;
    }
    fixture.guards(); kf.guards(); init_ws.guards(); predict_ws.guards(); update_ws.guards(); estimate_ws.guards();
}

TYPED_TEST(ParticleFilter, AngleResidualWrapsTheEstimateAndTheLikelihood) {
    using A = TypeParam;
    /* Floating angles are radians on [-pi,pi); Q angles are theta/pi on [-1,1). */
    const long double period = A::bits ? 2 : 2 * std::acos(-1.0L), half = period / 2;
    Model angle;
    angle.residual = [=](const Vec& a, const Vec& b) { return Vec{wrap(a[0] - b[0], period)}; };
    Model observation{[](const Vec& s) { return Vec{s[0]}; }};
    const Model plain;
    Matrix<A> x0(Dense(1, 1, {half - 0.1L})), p0(Dense(1, 1, {1e-4L})), q(Dense(1, 1, {1e-4L})), r(Dense(1, 1, {0.01L}));
    Matrix<A> mean_out(Dense(1, 1)), covariance_out(Dense(1, 1));
    auto xv = x0.view(), pv = p0.view(), qv = q.view(), rv = r.view(), mv = mean_out.view(), cv = covariance_out.view();
    PfState<A> fixture(3, 1, 1);
    auto init_ws = pf_init_workspace<A>(3, 1);
    auto update_ws = pf_update_workspace<A>(3, 1, 1);
    auto estimate_ws = pf_estimate_workspace<A>(3, 1);
    Sampler sampler(3);
    sampler.script = {{0}};
    ASSERT_EQ(Pf<A>::init(&fixture.state, &xv, &pv, &qv, &rv, sample_callback<A>, &sampler, init_ws.view()), LINEARS_OK);
    /* Place three particles around the cut by hand: two just below, one just above. */
    const Vec placed{half - 0.1L, half - 0.05L, -half + 0.05L};
    for (std::size_t i = 0; i < 3; ++i) fixture.particles[1 + i] = A::encode(placed[i]);
    PfReference reference;
    reference.particles = fixture.particle_rows();
    reference.weights = fixture.weight_values();
    ASSERT_EQ(Pf<A>::estimate(&fixture.state, residual_callback<A>, &angle, &mv, &cv, estimate_ws.view()), LINEARS_OK);
    const Vec wrapped_mean = reference.mean(angle);
    expect_matrix<A>(mv, Dense(1, 1, wrapped_mean));
    expect_matrix<A>(cv, reference.covariance(wrapped_mean, angle));
    EXPECT_GT(decode<A>(mv).values[0], half - 0.05L);
    EXPECT_LT(decode<A>(mv).values[0], half);
    /* Without the residual the two sides average across the cut. */
    ASSERT_EQ(Pf<A>::estimate(&fixture.state, nullptr, nullptr, &mv, &cv, estimate_ws.view()), LINEARS_OK);
    EXPECT_LT(std::abs(decode<A>(mv).values[0]), half / 2);
    /* A measurement just beyond the cut favors the particle on the other side. */
    Matrix<A> z(Dense(1, 1, {-half + 0.02L}));
    auto zv = z.view();
    ASSERT_EQ(Pf<A>::update(&fixture.state, &zv, map_callback<A>, &observation, residual_callback<A>, &angle, update_ws.view()), LINEARS_OK);
    Model likelihood = observation;
    likelihood.residual = angle.residual;
    reference.update(decode<A>(zv), likelihood, decode<A>(rv));
    const Vec weights = fixture.weight_values();
    for (std::size_t i = 0; i < 3; ++i) expect_near<A>(weights[i], reference.weights[i]);
    EXPECT_GT(weights[2], weights[1]);
    EXPECT_GT(weights[1], weights[0]);
    EXPECT_GT(weights[0], 0.1L);
    fixture.guards(); init_ws.guards(); update_ws.guards(); estimate_ws.guards();
}

TYPED_TEST(ParticleFilter, FixedParticlesMustStayInsideTheSignalInterval) {
    using A = TypeParam;
    if constexpr (A::bits == 0) {
        GTEST_SKIP() << "Only fixed formats store particles in a signal interval.";
    } else {
        Model shift{[](const Vec& x) { return Vec{x[0] + 0.05L}; }};
        Model keep{[](const Vec& x) { return Vec{x[0]}; }};
        Matrix<A> x0(Dense(1, 1, {0.9L})), p0(Dense(1, 1, {0.04L})), q(Dense(1, 1, {1e-4L})), r(Dense(1, 1, {0.25L}));
        Matrix<A> mean_out(Dense(1, 1)), covariance_out(Dense(1, 1));
        auto xv = x0.view(), pv = p0.view(), qv = q.view(), rv = r.view(), mv = mean_out.view(), cv = covariance_out.view();
        PfState<A> fixture(2, 1, 1);
        auto init_ws = pf_init_workspace<A>(2, 1);
        auto predict_ws = pf_predict_workspace<A>(2, 1);
        auto estimate_ws = pf_estimate_workspace<A>(2, 1);
        Sampler sampler(4);
        const long double x_start = decode<A>(xv).values[0], sigma = std::sqrt(decode<A>(pv).values[0]);
        /* A draw of 1.5 sigma lands at 1.2: the whole init fails and writes nothing. */
        sampler.script = {{1.5L}, {0}};
        ASSERT_GT(x_start + 1.5L * sigma, 1);
        const auto untouched = fixture.snapshot();
        EXPECT_EQ(Pf<A>::init(&fixture.state, &xv, &pv, &qv, &rv, sample_callback<A>, &sampler, init_ws.view()), LINEARS_ERROR_RANGE);
        EXPECT_EQ(sampler.calls, 1); // Q fails at the first out-of-range particle.
        EXPECT_EQ(fixture.snapshot(), untouched);
        sampler.script = {{0.4L}, {0}};
        sampler.scripted = 0;
        ASSERT_EQ(Pf<A>::init(&fixture.state, &xv, &pv, &qv, &rv, sample_callback<A>, &sampler, init_ws.view()), LINEARS_OK);
        expect_near<A>(fixture.particle_rows()[0][0], x_start + 0.4L * sigma);
        expect_near<A>(fixture.particle_rows()[1][0], x_start);
        /* Shifting the upper particle past +1 fails predict atomically. */
        sampler.script = {{0}};
        const auto snapshot = fixture.snapshot();
        EXPECT_EQ(Pf<A>::predict(&fixture.state, map_callback<A>, &shift, sample_callback<A>, &sampler, predict_ws.view()), LINEARS_ERROR_RANGE);
        EXPECT_EQ(shift.calls, 1);
        EXPECT_EQ(fixture.snapshot(), snapshot);
        ASSERT_EQ(Pf<A>::predict(&fixture.state, map_callback<A>, &keep, sample_callback<A>, &sampler, predict_ws.view()), LINEARS_OK);
        ASSERT_EQ(Pf<A>::estimate(&fixture.state, nullptr, nullptr, &mv, &cv, estimate_ws.view()), LINEARS_OK);
        const long double a = x_start + 0.4L * sigma, b = x_start;
        expect_matrix<A>(mv, Dense(1, 1, {(a + b) / 2}));
        expect_matrix<A>(cv, Dense(1, 1, {(a - b) * (a - b) / 4}));
        fixture.guards(); init_ws.guards(); predict_ws.guards(); estimate_ws.guards();
    }
}

TYPED_TEST(ParticleFilter, InitValidationCapacitiesAndAliasRules) {
    using A = TypeParam;
    /* Q particles must stay inside [-1,+1], so their prior is tighter. */
    Matrix<A> x0(Dense(2, 1, {0.25L, -0.125L})), p0(Dense::identity(2, A::bits ? 0.04L : 0.25L)), q(Dense::identity(2, 0.01L));
    Matrix<A> r(Dense::identity(1, 0.25L));
    Matrix<A> asymmetric(Dense(2, 2, {0.25L, 0.125L, 0, 0.25L})), negative(Dense::identity(2, -0.25L)), zero(Dense::identity(2, 0));
    auto xv = x0.view(), pv = p0.view(), qv = q.view(), rv = r.view(), av = asymmetric.view(), nv = negative.view(), zv = zero.view();
    PfState<A> fixture(5, 2, 1);
    auto ws = pf_init_workspace<A>(5, 2);
    Sampler sampler(5);
    sampler.script = {{0.5L, -0.5L}, {1, 1}, {-1, 0.25L}, {0, 0}, {2, -2}};
    const auto sample = sample_callback<A>;
    EXPECT_EQ(Pf<A>::init(nullptr, &xv, &pv, &qv, &rv, sample, &sampler, ws.view()), LINEARS_ERROR_ARGUMENT);
    EXPECT_EQ(Pf<A>::init(&fixture.state, nullptr, &pv, &qv, &rv, sample, &sampler, ws.view()), LINEARS_ERROR_ARGUMENT);
    EXPECT_EQ(Pf<A>::init(&fixture.state, &xv, &pv, &qv, nullptr, sample, &sampler, ws.view()), LINEARS_ERROR_ARGUMENT);
    EXPECT_EQ(Pf<A>::init(&fixture.state, &xv, &pv, &qv, &rv, nullptr, &sampler, ws.view()), LINEARS_ERROR_ARGUMENT);
    EXPECT_EQ(Pf<A>::init(&fixture.state, &xv, &av, &qv, &rv, sample, &sampler, ws.view()), LINEARS_ERROR_DOMAIN);
    EXPECT_EQ(Pf<A>::init(&fixture.state, &xv, &pv, &nv, &rv, sample, &sampler, ws.view()), LINEARS_ERROR_DOMAIN);
    EXPECT_EQ(Pf<A>::init(&fixture.state, &xv, &pv, &av, &rv, sample, &sampler, ws.view()), LINEARS_ERROR_DOMAIN);
    EXPECT_EQ(Pf<A>::init(&fixture.state, &xv, &zv, &qv, &rv, sample, &sampler, ws.view()), LINEARS_ERROR_NOT_POSITIVE_DEFINITE);
    auto wrong = rv; wrong.rows = 2;
    EXPECT_EQ(Pf<A>::init(&fixture.state, &xv, &pv, &qv, &wrong, sample, &sampler, ws.view()), LINEARS_ERROR_DIMENSION);
    wrong = xv; wrong.cols = 2;
    EXPECT_EQ(Pf<A>::init(&fixture.state, &wrong, &pv, &qv, &rv, sample, &sampler, ws.view()), LINEARS_ERROR_DIMENSION);
    wrong = xv; wrong.data = nullptr;
    EXPECT_EQ(Pf<A>::init(&fixture.state, &wrong, &pv, &qv, &rv, sample, &sampler, ws.view()), LINEARS_ERROR_ARGUMENT);
    fixture.state.particles_capacity = 9;
    EXPECT_EQ(Pf<A>::init(&fixture.state, &xv, &pv, &qv, &rv, sample, &sampler, ws.view()), LINEARS_ERROR_WORKSPACE);
    fixture.state.particles_capacity = 10;
    fixture.state.weights_capacity = 4;
    EXPECT_EQ(Pf<A>::init(&fixture.state, &xv, &pv, &qv, &rv, sample, &sampler, ws.view()), LINEARS_ERROR_WORKSPACE);
    fixture.state.weights_capacity = 5;
    fixture.state.measurement_noise_capacity = 0;
    EXPECT_EQ(Pf<A>::init(&fixture.state, &xv, &pv, &qv, &rv, sample, &sampler, ws.view()), LINEARS_ERROR_WORKSPACE);
    fixture.state.measurement_noise_capacity = 1;
    auto oversized = fixture.state;
    oversized.particles.rows = SIZE_MAX / 2;
    oversized.particles.cols = 3;
    EXPECT_EQ(Pf<A>::init(&oversized, &xv, &pv, &qv, &rv, sample, &sampler, ws.view()), LINEARS_ERROR_SIZE);
    for (int part = 0; part < 3; ++part) {
        auto w = ws.view();
        if (part == 1 && w.basic_bytes == 0) continue;
        if (part == 0) --w.capacity;
        if (part == 1) --w.basic_bytes;
        if (part == 2) --w.decomposition.capacity;
        EXPECT_EQ(Pf<A>::init(&fixture.state, &xv, &pv, &qv, &rv, sample, &sampler, w), LINEARS_ERROR_WORKSPACE);
    }
    if constexpr (A::bits == 0) {
        typename A::Raw bad = std::numeric_limits<typename A::Raw>::quiet_NaN();
        linears_matrix_t nan{&bad, 1, 1};
        EXPECT_EQ(Pf<A>::init(&fixture.state, &xv, &pv, &qv, &nan, sample, &sampler, ws.view()), LINEARS_ERROR_RANGE);
    }
    EXPECT_EQ(sampler.calls, 0);
    for (auto& v : fixture.particles) EXPECT_EQ(v, PfState<A>::sentinel);
    /* A failing sampler leaves an uninitialized state untouched too. */
    sampler.fail_at = 3;
    EXPECT_EQ(Pf<A>::init(&fixture.state, &xv, &pv, &qv, &rv, sample, &sampler, ws.view()), LINEARS_ERROR_NO_CONVERGENCE);
    EXPECT_EQ(sampler.calls, 4);
    for (auto& v : fixture.particles) EXPECT_EQ(v, PfState<A>::sentinel);
    sampler.fail_at = -1; sampler.scripted = 0; sampler.log.clear();
    ASSERT_EQ(Pf<A>::init(&fixture.state, &xv, &pv, &qv, &rv, sample, &sampler, ws.view()), LINEARS_OK);
    PfReference reference;
    reference.init(decode<A>(xv), decode<A>(pv), sampler.take_log());
    for (std::size_t i = 0; i < 5; ++i) {
        for (std::size_t j = 0; j < 2; ++j) expect_near<A>(fixture.particle_rows()[i][j], reference.particles[i][j]);
        expect_near<A>(fixture.weight_values()[i], 0.2L);
    }
    expect_matrix<A>(fixture.state.process_noise, decode<A>(qv));
    expect_matrix<A>(fixture.state.measurement_noise, decode<A>(rv));
    /* Re-initializing from the state's own noise descriptors or exact storage is allowed. */
    const auto snapshot = fixture.snapshot();
    sampler.scripted = 0;
    ASSERT_EQ(Pf<A>::init(&fixture.state, &xv, &pv, &fixture.state.process_noise, &fixture.state.measurement_noise,
                          sample, &sampler, ws.view()), LINEARS_OK);
    auto own_q = fixture.state.process_noise, own_r = fixture.state.measurement_noise;
    sampler.scripted = 0;
    ASSERT_EQ(Pf<A>::init(&fixture.state, &xv, &pv, &own_q, &own_r, sample, &sampler, ws.view()), LINEARS_OK);
    EXPECT_EQ(fixture.snapshot(), snapshot);
    auto partial = own_q; partial.data = static_cast<typename A::Raw*>(own_q.data) + 1;
    EXPECT_EQ(Pf<A>::init(&fixture.state, &xv, &pv, &partial, &rv, sample, &sampler, ws.view()), LINEARS_ERROR_ALIAS);
    auto cross = qv; cross.data = own_r.data; /* Q source aliasing the R storage. */
    EXPECT_EQ(Pf<A>::init(&fixture.state, &xv, &pv, &cross, &rv, sample, &sampler, ws.view()), LINEARS_ERROR_ALIAS);
    auto inside = xv; inside.data = fixture.state.weights.data;
    EXPECT_EQ(Pf<A>::init(&fixture.state, &inside, &pv, &qv, &rv, sample, &sampler, ws.view()), LINEARS_ERROR_ALIAS);
    auto previous = fixture.state.weights.data;
    fixture.state.weights.data = fixture.state.particles.data;
    EXPECT_EQ(Pf<A>::init(&fixture.state, &xv, &pv, &qv, &rv, sample, &sampler, ws.view()), LINEARS_ERROR_ALIAS);
    fixture.state.weights.data = previous;
    linears_matrix_t into_scratch{ws.view().data, 2, 1};
    EXPECT_EQ(Pf<A>::init(&fixture.state, &into_scratch, &pv, &qv, &rv, sample, &sampler, ws.view()), LINEARS_ERROR_ALIAS);
    EXPECT_EQ(fixture.snapshot(), snapshot);
    fixture.guards(); ws.guards();
}

TYPED_TEST(ParticleFilter, WorkspaceBoundsAliasAndFailureAtomicity) {
    using A = TypeParam;
    Model transition{[](const Vec& x) { return Vec{0.5L * x[0] + 0.1L * x[1], 0.5L * x[1]}; }};
    Model observation{[](const Vec& x) { return Vec{0.5L * x[0]}; }};
    /* Q particles must stay inside [-1,+1], so their prior is tighter. */
    Matrix<A> x0(Dense(2, 1, {0.25L, -0.125L})), p0(Dense::identity(2, A::bits ? 0.04L : 0.25L)), q(Dense::identity(2, 0.01L));
    Matrix<A> r(Dense::identity(1, 0.25L)), z(Dense(1, 1, {0.125L})), mean_out(Dense(2, 1)), covariance_out(Dense(2, 2));
    auto xv = x0.view(), pv = p0.view(), qv = q.view(), rv = r.view(), zv = z.view(), mv = mean_out.view(), cv = covariance_out.view();
    const std::size_t count = 6;
    PfState<A> fixture(count, 2, 1);
    auto init_ws = pf_init_workspace<A>(count, 2);
    auto predict_ws = pf_predict_workspace<A>(count, 2);
    auto update_ws = pf_update_workspace<A>(count, 2, 1);
    auto resample_ws = pf_resample_workspace<A>(count);
    auto estimate_ws = pf_estimate_workspace<A>(count, 2);
    Sampler sampler(6);
    const auto sample = sample_callback<A>;
    std::vector<std::size_t> indices(count, SIZE_MAX);
    ASSERT_EQ(Pf<A>::init(&fixture.state, &xv, &pv, &qv, &rv, sample, &sampler, init_ws.view()), LINEARS_OK);
    const auto snapshot = fixture.snapshot();
    const auto outputs = std::make_pair(mean_out.data, covariance_out.data);
    sampler.calls = 0;
    EXPECT_EQ(Pf<A>::predict(nullptr, map_callback<A>, &transition, sample, &sampler, predict_ws.view()), LINEARS_ERROR_ARGUMENT);
    EXPECT_EQ(Pf<A>::predict(&fixture.state, nullptr, &transition, sample, &sampler, predict_ws.view()), LINEARS_ERROR_ARGUMENT);
    EXPECT_EQ(Pf<A>::predict(&fixture.state, map_callback<A>, &transition, nullptr, &sampler, predict_ws.view()), LINEARS_ERROR_ARGUMENT);
    EXPECT_EQ(Pf<A>::update(&fixture.state, &zv, nullptr, &observation, nullptr, nullptr, update_ws.view()), LINEARS_ERROR_ARGUMENT);
    EXPECT_EQ(Pf<A>::update(&fixture.state, nullptr, map_callback<A>, &observation, nullptr, nullptr, update_ws.view()), LINEARS_ERROR_ARGUMENT);
    EXPECT_EQ(Pf<A>::resample(&fixture.state, work_encode<A>(0.5L), nullptr, count, resample_ws.view()), LINEARS_ERROR_ARGUMENT);
    EXPECT_EQ(Pf<A>::estimate(nullptr, nullptr, nullptr, &mv, &cv, estimate_ws.view()), LINEARS_ERROR_ARGUMENT);
    EXPECT_EQ(Pf<A>::estimate(&fixture.state, nullptr, nullptr, nullptr, &cv, estimate_ws.view()), LINEARS_ERROR_ARGUMENT);
    for (int part = 0; part < 3; ++part) {
        auto w = predict_ws.view();
        auto u = update_ws.view();
        auto e = estimate_ws.view();
        if (part == 0) { --w.capacity; --u.capacity; --e.capacity; }
        if (part == 1) {
            if (w.basic_bytes != 0) --w.basic_bytes;
            if (u.basic_bytes != 0) --u.basic_bytes;
            if (e.basic_bytes != 0) --e.basic_bytes;
        }
        if (part == 2) { --w.decomposition.capacity; --u.decomposition.capacity; }
        if (part != 1 || predict_ws.basic_bytes != 0)
            EXPECT_EQ(Pf<A>::predict(&fixture.state, map_callback<A>, &transition, sample, &sampler, w), LINEARS_ERROR_WORKSPACE);
        if (part != 1 || update_ws.basic_bytes != 0)
            EXPECT_EQ(Pf<A>::update(&fixture.state, &zv, map_callback<A>, &observation, nullptr, nullptr, u), LINEARS_ERROR_WORKSPACE);
        if (part == 0 || (part == 1 && estimate_ws.basic_bytes != 0))
            EXPECT_EQ(Pf<A>::estimate(&fixture.state, nullptr, nullptr, &mv, &cv, e), LINEARS_ERROR_WORKSPACE);
    }
    if constexpr (A::bits != 0) {
        auto short_resample = resample_ws.view();
        if (short_resample.capacity != 0 || short_resample.signal_capacity != 0) {
            if (short_resample.capacity != 0) --short_resample.capacity;
            else --short_resample.signal_capacity;
            EXPECT_EQ(Pf<A>::resample(&fixture.state, work_encode<A>(0.5L), indices.data(), count, short_resample), LINEARS_ERROR_WORKSPACE);
        }
    }
    EXPECT_EQ(Pf<A>::resample(&fixture.state, work_encode<A>(0.5L), indices.data(), count - 1, resample_ws.view()), LINEARS_ERROR_WORKSPACE);
    auto w = predict_ws.view(); w.data = nullptr;
    EXPECT_EQ(Pf<A>::predict(&fixture.state, map_callback<A>, &transition, sample, &sampler, w), LINEARS_ERROR_WORKSPACE);
    w = predict_ws.view();
    w.data = reinterpret_cast<Work<A>*>(reinterpret_cast<unsigned char*>(w.data) + 1);
    EXPECT_EQ(Pf<A>::predict(&fixture.state, map_callback<A>, &transition, sample, &sampler, w), LINEARS_ERROR_WORKSPACE);
    w = predict_ws.view(); w.decomposition.data = w.data;
    EXPECT_EQ(Pf<A>::predict(&fixture.state, map_callback<A>, &transition, sample, &sampler, w), LINEARS_ERROR_ALIAS);
    auto u = update_ws.view();
    auto aliased = zv; aliased.data = u.data;
    EXPECT_EQ(Pf<A>::update(&fixture.state, &aliased, map_callback<A>, &observation, nullptr, nullptr, u), LINEARS_ERROR_ALIAS);
    auto wrong = zv; wrong.rows = 2;
    EXPECT_EQ(Pf<A>::update(&fixture.state, &wrong, map_callback<A>, &observation, nullptr, nullptr, u), LINEARS_ERROR_DIMENSION);
    const auto previous = fixture.state.particles.data;
    fixture.state.particles.data = predict_ws.view().data;
    EXPECT_EQ(Pf<A>::predict(&fixture.state, map_callback<A>, &transition, sample, &sampler, predict_ws.view()), LINEARS_ERROR_ALIAS);
    fixture.state.particles.data = previous;
    linears_matrix_t into_weights{fixture.state.weights.data, 2, 1};
    EXPECT_EQ(Pf<A>::estimate(&fixture.state, nullptr, nullptr, &into_weights, &cv, estimate_ws.view()), LINEARS_ERROR_ALIAS);
    linears_matrix_t into_mean{mv.data, 2, 2};
    EXPECT_EQ(Pf<A>::estimate(&fixture.state, nullptr, nullptr, &mv, &into_mean, estimate_ws.view()), LINEARS_ERROR_ALIAS);
    EXPECT_EQ(Pf<A>::resample(&fixture.state, work_encode<A>(1), indices.data(), count, resample_ws.view()), LINEARS_ERROR_DOMAIN);
    EXPECT_EQ(transition.calls, 0); EXPECT_EQ(observation.calls, 0); EXPECT_EQ(sampler.calls, 0);
    for (auto index : indices) EXPECT_EQ(index, SIZE_MAX);
    EXPECT_EQ(fixture.snapshot(), snapshot);
    EXPECT_EQ(std::make_pair(mean_out.data, covariance_out.data), outputs);
    /* Callback, sampler and residual failures are reported without commits. */
    transition.fail_at = 3;
    EXPECT_EQ(Pf<A>::predict(&fixture.state, map_callback<A>, &transition, sample, &sampler, predict_ws.view()), LINEARS_ERROR_NO_CONVERGENCE);
    EXPECT_EQ(transition.calls, 4); EXPECT_EQ(sampler.calls, A::bits ? 3 : 0);
    transition.fail_at = -1; transition.calls = 0; sampler.calls = 0;
    sampler.fail_at = 2; sampler.failure = LINEARS_ERROR_SINGULAR;
    EXPECT_EQ(Pf<A>::predict(&fixture.state, map_callback<A>, &transition, sample, &sampler, predict_ws.view()), LINEARS_ERROR_SINGULAR);
    EXPECT_EQ(transition.calls, A::bits ? 3 : static_cast<int>(count)); EXPECT_EQ(sampler.calls, 3);
    sampler.fail_at = -1; sampler.calls = 0; transition.calls = 0;
    observation.fail_at = 0; observation.failure = LINEARS_ERROR_SINGULAR;
    EXPECT_EQ(Pf<A>::update(&fixture.state, &zv, map_callback<A>, &observation, nullptr, nullptr, update_ws.view()), LINEARS_ERROR_SINGULAR);
    observation.fail_at = -1; observation.calls = 0;
    Model failing_residual;
    failing_residual.residual = [](const Vec& a, const Vec&) { return a; };
    failing_residual.fail_at = 2; failing_residual.failure = LINEARS_ERROR_DOMAIN;
    EXPECT_EQ(Pf<A>::update(&fixture.state, &zv, map_callback<A>, &observation, residual_callback<A>, &failing_residual, update_ws.view()), LINEARS_ERROR_DOMAIN);
    EXPECT_EQ(observation.calls, A::bits ? 3 : static_cast<int>(count)); EXPECT_EQ(failing_residual.calls, 3);
    observation.calls = 0; failing_residual.calls = 0; failing_residual.fail_at = 1;
    EXPECT_EQ(Pf<A>::estimate(&fixture.state, residual_callback<A>, &failing_residual, &mv, &cv, estimate_ws.view()), LINEARS_ERROR_DOMAIN);
    transition.nonfinite = true;
    EXPECT_EQ(Pf<A>::predict(&fixture.state, map_callback<A>, &transition, sample, &sampler, predict_ws.view()), LINEARS_ERROR_RANGE);
    transition.nonfinite = false; transition.calls = 0;
    sampler.nonfinite = true;
    EXPECT_EQ(Pf<A>::predict(&fixture.state, map_callback<A>, &transition, sample, &sampler, predict_ws.view()), LINEARS_ERROR_RANGE);
    sampler.nonfinite = false; transition.calls = 0; sampler.calls = 0;
    EXPECT_EQ(fixture.snapshot(), snapshot);
    EXPECT_EQ(std::make_pair(mean_out.data, covariance_out.data), outputs);
    /* Noise is re-validated on every call and factored before any callback. */
    Matrix<A> edited(Dense(2, 2, {0.01L, 0.005L, 0, 0.01L})), zero(Dense::identity(2, 0)), zero_r(Dense(1, 1, {0}));
    std::memcpy(fixture.state.process_noise.data, edited.data.data(), edited.data.size() * sizeof(typename A::Raw));
    EXPECT_EQ(Pf<A>::predict(&fixture.state, map_callback<A>, &transition, sample, &sampler, predict_ws.view()), LINEARS_ERROR_DOMAIN);
    std::memcpy(fixture.state.process_noise.data, zero.data.data(), zero.data.size() * sizeof(typename A::Raw));
    EXPECT_EQ(Pf<A>::predict(&fixture.state, map_callback<A>, &transition, sample, &sampler, predict_ws.view()), LINEARS_ERROR_NOT_POSITIVE_DEFINITE);
    std::memcpy(fixture.state.process_noise.data, q.data.data(), q.data.size() * sizeof(typename A::Raw));
    std::memcpy(fixture.state.measurement_noise.data, zero_r.data.data(), sizeof(typename A::Raw));
    EXPECT_EQ(Pf<A>::update(&fixture.state, &zv, map_callback<A>, &observation, nullptr, nullptr, update_ws.view()), LINEARS_ERROR_NOT_POSITIVE_DEFINITE);
    std::memcpy(fixture.state.measurement_noise.data, r.data.data(), sizeof(typename A::Raw));
    EXPECT_EQ(transition.calls, 0); EXPECT_EQ(observation.calls, 0); EXPECT_EQ(sampler.calls, 0);
    EXPECT_EQ(fixture.snapshot(), snapshot);
    /* Zeroed weights make update, resample and the ESS fail; particles stay. */
    std::fill(fixture.weights.begin() + 1, fixture.weights.end() - 1, A::encode(0));
    const auto zeroed = fixture.snapshot();
    EXPECT_EQ(Pf<A>::update(&fixture.state, &zv, map_callback<A>, &observation, nullptr, nullptr, update_ws.view()), LINEARS_ERROR_DOMAIN);
    EXPECT_EQ(Pf<A>::resample(&fixture.state, work_encode<A>(0.5L), indices.data(), count, resample_ws.view()), LINEARS_ERROR_DOMAIN);
    EXPECT_EQ(fixture.snapshot(), zeroed);
    for (auto index : indices) EXPECT_EQ(index, SIZE_MAX);
    std::fill(fixture.weights.begin() + 1, fixture.weights.end() - 1, A::encode(1.0L / count));
    ASSERT_EQ(Pf<A>::predict(&fixture.state, map_callback<A>, &transition, sample, &sampler, predict_ws.view()), LINEARS_OK);
    ASSERT_EQ(Pf<A>::update(&fixture.state, &zv, map_callback<A>, &observation, nullptr, nullptr, update_ws.view()), LINEARS_OK);
    ASSERT_EQ(Pf<A>::resample(&fixture.state, work_encode<A>(0.5L), indices.data(), count, resample_ws.view()), LINEARS_OK);
    ASSERT_EQ(Pf<A>::estimate(&fixture.state, nullptr, nullptr, &mv, &cv, estimate_ws.view()), LINEARS_OK);
    for (auto index : indices) EXPECT_LT(index, count);
    for (std::size_t i = 0; i < count; ++i) expect_near<A>(fixture.weight_values()[i], 1.0L / count);
    fixture.guards(); init_ws.guards(); predict_ws.guards(); update_ws.guards(); resample_ws.guards(); estimate_ws.guards();
}

TEST(ParticleFilterWorkspace, ConstantExpressionsAndOverflowSentinels) {
    static_assert(FILTERS_PF_PERTURB_STORAGE(3) == 6, "perturb storage");
    static_assert(FILTERS_PF_PERTURB_BASIC_BYTES(3) == 3 * sizeof(linears_workspace_unit_t), "perturb basic");
    static_assert(FILTERS_PF_GAUSSIAN_EXPONENT_STORAGE(3) == 3, "exponent storage");
    static_assert(FILTERS_PF_GAUSSIAN_EXPONENT_BASIC_BYTES == sizeof(linears_workspace_unit_t), "exponent basic");
    static_assert(FILTERS_PF_GAUSSIAN_EXPONENT_DECOMP_STORAGE(3) == 12, "exponent solve scratch");
    static_assert(FILTERS_PF_PREDICT_STORAGE(10, 2) == 20 + 4 + 4, "predict storage");
    static_assert(FILTERS_PF_Q_INIT_STORAGE(10, 2) == 2 * 4 + 3 * 2, "Q local init storage");
    static_assert(FILTERS_PF_Q_PREDICT_STORAGE(10, 2) == 2 * 4 + 3 * 2, "Q local predict storage");
    static_assert(FILTERS_PF_Q_INIT_SIGNAL_STORAGE(10, 2) == 20, "Q init raw particle stage");
    static_assert(FILTERS_PF_Q_PREDICT_SIGNAL_STORAGE(10, 2) == 20, "Q predict raw particle stage");
    static_assert(FILTERS_PF_PREDICT_BASIC_BYTES(2) == 2 * sizeof(linears_workspace_unit_t), "predict basic");
    static_assert(FILTERS_PF_PREDICT_DECOMP_STORAGE(2) == 4, "predict Cholesky");
    static_assert(FILTERS_PF_UPDATE_STORAGE(10, 3) == 60 + 20 + 9 + 3, "update storage");
    static_assert(FILTERS_PF_Q_UPDATE_STORAGE(10, 2, 3) == 10 + 2 * 9 + 2 + 3 * 3, "Q local update storage");
    static_assert(FILTERS_PF_Q_UPDATE_SIGNAL_STORAGE(10, 2, 3) == 10, "Q raw weight stage");
    static_assert(FILTERS_PF_UPDATE_BASIC_BYTES(3) == 3 * sizeof(linears_workspace_unit_t), "update basic");
    static_assert(FILTERS_PF_UPDATE_DECOMP_STORAGE(3) == 12, "update solve scratch");
    static_assert(FILTERS_PF_Q_RESAMPLE_STORAGE(10) == 0, "Q resample needs no floating scratch");
    static_assert(FILTERS_PF_Q_RESAMPLE_SIGNAL_STORAGE(10) == 0, "Q resample gathers in place");
    static_assert(FILTERS_PF_ESTIMATE_STORAGE(10, 2) == 60 + 8 + 2, "estimate storage");
    static_assert(FILTERS_PF_ESTIMATE_BASIC_BYTES(10, 2) == 20 * sizeof(linears_workspace_unit_t), "estimate basic");
    static_assert(FILTERS_PF_ESTIMATE_BASIC_BYTES(1, 3) == 9 * sizeof(linears_workspace_unit_t), "estimate basic, n dominates");
    static_assert(FILTERS_PF_Q_ESTIMATE_STORAGE(10, 2) == 4 + 3 * 2, "Q local estimate storage");
    static_assert(FILTERS_PF_Q_ESTIMATE_SIGNAL_STORAGE(10, 2) == 4 + 2 * 2, "Q raw estimate stage");
    static_assert(FILTERS_PF_Q_PREDICT_BASIC_BYTES(2) == 0, "Q predict has no basic scratch");
    static_assert(FILTERS_PF_Q_UPDATE_BASIC_BYTES(3) == 0, "Q update has no basic scratch");
    static_assert(FILTERS_PF_Q_ESTIMATE_BASIC_BYTES(10, 2) == 2 * 4 * sizeof(linears_workspace_unit_t), "Q exact moment accumulators");
    static_assert(FILTERS_PF_Q_UPDATE_DECOMP_STORAGE(3) == 9, "Q triangular solve storage");
    const std::size_t huge = SIZE_MAX / 2 + 1;
    EXPECT_EQ(FILTERS_PF_PERTURB_STORAGE(huge), SIZE_MAX);
    EXPECT_EQ(FILTERS_PF_PREDICT_STORAGE(huge, 2), SIZE_MAX);
    EXPECT_EQ(FILTERS_PF_PREDICT_STORAGE(2, huge), SIZE_MAX);
    EXPECT_EQ(FILTERS_PF_Q_INIT_STORAGE(huge, 2), 14u);
    EXPECT_EQ(FILTERS_PF_Q_INIT_SIGNAL_STORAGE(huge, 2), SIZE_MAX);
    EXPECT_EQ(FILTERS_PF_UPDATE_STORAGE(huge, 1), SIZE_MAX);
    EXPECT_EQ(FILTERS_PF_Q_UPDATE_STORAGE(1, 1, huge), SIZE_MAX);
    EXPECT_EQ(FILTERS_PF_Q_RESAMPLE_STORAGE(huge), 0u);
    EXPECT_EQ(FILTERS_PF_ESTIMATE_STORAGE(huge, 1), SIZE_MAX);
    EXPECT_EQ(FILTERS_PF_ESTIMATE_BASIC_BYTES(huge, 1), SIZE_MAX);
    EXPECT_EQ(FILTERS_PF_Q_ESTIMATE_STORAGE(1, huge), SIZE_MAX);
    EXPECT_EQ(FILTERS_PF_GAUSSIAN_EXPONENT_DECOMP_STORAGE(huge), SIZE_MAX);
    float predict[FILTERS_PF_PREDICT_STORAGE(10, 2)];
    double q31_update[FILTERS_PF_Q_UPDATE_STORAGE(10, 2, 3)];
    linears_workspace_unit_t basic[LINEARS_WORKSPACE_UNITS(FILTERS_PF_ESTIMATE_BASIC_BYTES(1, 3))];
    EXPECT_EQ(sizeof predict / sizeof predict[0], 28u);
    EXPECT_EQ(sizeof q31_update / sizeof q31_update[0], 39u);
    EXPECT_EQ(sizeof basic / sizeof basic[0], 9u);
}

} // namespace
