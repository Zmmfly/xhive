#include "pf_support.h"

#include <type_traits>

namespace {
using namespace pf_test;

template<class A> class ParticlePipeline : public ::testing::Test {};
TYPED_TEST_SUITE(ParticlePipeline, KernelFormats);

TYPED_TEST(ParticlePipeline, PublicGaussianKeepsGeneralDenseFactors) {
    using A = TypeParam;
    /* Both triangles are populated: a triangular-only public implementation
     * cannot compute d^T (L L^T)^-1 d for this valid nonsingular factor. */
    const Dense factor(3, 3, {2, 0.25L, -0.1L,
                             -0.3L, 1.5L, 0.4L,
                             0.2L, -0.2L, 1.2L});
    std::mt19937 random(0x50464950u);
    const Dense deviations = random_dense(random, 7, 3, 0.4L);
    WorkMatrix<A> d(deviations), l(factor), out(7, 1);
    auto dv = d.view(), lv = l.view(), ov = out.view();
    UkfWorkspace<A> ws(FILTERS_PF_GAUSSIAN_EXPONENT_STORAGE(3),
                       FILTERS_PF_GAUSSIAN_EXPONENT_BASIC_BYTES,
                       FILTERS_PF_GAUSSIAN_EXPONENT_DECOMP_STORAGE(3));
    ASSERT_EQ(PfKernels<A>::gaussian_exponent(&dv, &lv, &ov, ws.view()), LINEARS_OK);
    const Dense seen_l = decode_work<A>(lv), seen_d = decode_work<A>(dv);
    const Dense covariance = seen_l * seen_l.transpose();
    std::vector<Vec> rows(7, Vec(3));
    for (std::size_t i = 0; i < rows.size(); ++i)
        for (std::size_t j = 0; j < 3; ++j) rows[i][j] = seen_d(i, j);
    const Vec expected = PfReference::exponents(rows, covariance);
    for (std::size_t i = 0; i < rows.size(); ++i) {
        SCOPED_TRACE(i);
        expect_near<A>(work_decode<A>(out.data[i]), expected[i]);
    }
    ws.guards();
}

TYPED_TEST(ParticlePipeline, CorrelatedNoiseUpdateMatchesLongDouble) {
    using A = TypeParam;
    constexpr std::size_t count = 32, n = 4;
    std::mt19937 random(0x50464647u);
    const Dense particles = random_dense(random, count, n, 0.3L);
    const Dense r = random_spd(random, n, 0.04L, 0.08L);
    const Dense z = random_dense(random, n, 1, 0.2L);
    ASSERT_NE(r(0, 1), 0);
    PfState<A> fixture(count, n, n);
    for (std::size_t i = 0; i < particles.values.size(); ++i)
        fixture.particles[i + 1] = A::encode(particles.values[i]);
    for (std::size_t i = 0; i < r.values.size(); ++i)
        fixture.r[i + 1] = A::encode(r.values[i]);
    for (std::size_t i = 0; i < n * n; ++i)
        fixture.q[i + 1] = A::encode(i % (n + 1) == 0 ? 0.01L : 0);
    std::uniform_real_distribution<long double> weight(0.05L, 1.0L);
    for (std::size_t i = 0; i < count; ++i) fixture.weights[i + 1] = A::encode(weight(random));
    Matrix<A> measurement(z);
    auto zv = measurement.view();
    Model identity{[](const Vec& value) { return value; }};
    auto update_ws = pf_update_workspace<A>(count, n, n);
    PfReference reference;
    reference.particles = fixture.particle_rows();
    reference.weights = fixture.weight_values();
    ASSERT_EQ(Pf<A>::update(&fixture.state, &zv, map_callback<A>, &identity,
                            nullptr, nullptr, update_ws.view()), LINEARS_OK);
    reference.update(decode<A>(zv), identity, decode<A>(fixture.state.measurement_noise));
    const Vec actual = fixture.weight_values();
    for (std::size_t i = 0; i < count; ++i) {
        SCOPED_TRACE(i);
        expect_near<A>(actual[i], reference.weights[i]);
    }
    fixture.guards();
    update_ws.guards();
}

TYPED_TEST(ParticlePipeline, GeneralFactorsKeepLegacyAcceptedExtremeInputs) {
    using A = TypeParam;
    /* Broad-exponent differential probe 0x504647415553: direct forward
     * substitution overflows the squared norm, while the legacy pivoted
     * solve accepts it. Non-diagonal factors must retain that old path. */
    const bool single = A::bits == 0 && sizeof(Work<A>) == sizeof(float);
    const Dense covariance = single
        ? Dense(2, 2, {4.4544191746267394e-32L, 1.0474383780773809e-27L,
                       1.0474383780773809e-27L, 8.5165077559009887e22L})
        : Dense(2, 2, {4.6210604723926343e-181L, 8.2083477171150012e-151L,
                       8.2083477171150012e-151L, 2.3714970701855796e114L});
    const Dense deviation = single
        ? Dense(2, 1, {1.9660570624e12L, 1.6878889578359214e25L})
        : Dense(2, 1, {1.738955937040944e140L, 5.647804412763275e193L});
    PfState<A> fixture(1, 2, 2);
    for (std::size_t i = 0; i < 2; ++i) fixture.particles[i + 1] = A::encode(0);
    fixture.weights[1] = A::encode(1);
    for (std::size_t i = 0; i < 4; ++i) {
        fixture.q[i + 1] = A::encode(i % 3 == 0 ? 0.01L : 0);
        fixture.r[i + 1] = A::encode(covariance.values[i]);
    }
    Matrix<A> z(Dense(2, 1, {-deviation.values[0], -deviation.values[1]}));
    auto zv = z.view();
    Model identity{[](const Vec& value) { return value; }};
    auto update_ws = pf_update_workspace<A>(1, 2, 2);
    const auto before = fixture.snapshot();
    ASSERT_EQ(Pf<A>::update(&fixture.state, &zv, map_callback<A>, &identity,
                            nullptr, nullptr, update_ws.view()), LINEARS_OK);
    EXPECT_EQ(identity.calls, 1);
    EXPECT_EQ(fixture.snapshot(), before);
    fixture.guards();
    update_ws.guards();
}

TYPED_TEST(ParticlePipeline, GaussianRangeFailurePreservesStateAfterAllCallbacks) {
    using A = TypeParam;
    constexpr std::size_t count = 3;
    PfState<A> fixture(count, 1, 1);
    fixture.particles[1] = A::encode(0);
    fixture.particles[2] = std::numeric_limits<Work<A>>::max();
    fixture.particles[3] = A::encode(0);
    fixture.weights[1] = A::encode(0.2L);
    fixture.weights[2] = A::encode(0.3L);
    fixture.weights[3] = A::encode(0.5L);
    fixture.q[1] = A::encode(0.01L);
    fixture.r[1] = A::encode(1);
    Matrix<A> z(Dense(1, 1, {0}));
    auto zv = z.view();
    Model identity{[](const Vec& value) { return value; }}, subtract;
    subtract.residual = [](const Vec& a, const Vec& b) {
        return Vec{a[0] - b[0]};
    };
    auto update_ws = pf_update_workspace<A>(count, 1, 1);
    const auto before = fixture.snapshot();
    EXPECT_EQ(Pf<A>::update(&fixture.state, &zv, map_callback<A>, &identity,
                            residual_callback<A>, &subtract, update_ws.view()), LINEARS_ERROR_RANGE);
    EXPECT_EQ(identity.calls, static_cast<int>(count));
    EXPECT_EQ(subtract.calls, static_cast<int>(count));
    EXPECT_EQ(fixture.snapshot(), before);
    fixture.guards();
    update_ws.guards();
}

/* Compare the state-layer update against the unchanged public Gaussian and
 * reweight kernels, including exact working-format weights. */
template<class A> void expect_legacy_update(const Dense& particles, const Dense& noise) {
    const std::size_t count = particles.rows, n = particles.cols;
    PfState<A> fixture(count, n, n);
    for (std::size_t i = 0; i < count * n; ++i)
        fixture.particles[i + 1] = A::encode(particles.values[i]);
    for (std::size_t i = 0; i < n * n; ++i) {
        fixture.r[i + 1] = A::encode(noise.values[i]);
        fixture.q[i + 1] = A::encode(i % (n + 1) == 0 ? 0.01L : 0);
    }
    for (std::size_t i = 0; i < count; ++i) fixture.weights[i + 1] = A::encode(1.0L / count);
    WorkMatrix<A> residuals(particles), factor(n, n), exponents(count, 1);
    WorkMatrix<A> expected_weights(Dense(count, 1, Vec(count, 1.0L / count)));
    auto dv = residuals.view(), lv = factor.view(), ev = exponents.view(), wv = expected_weights.view();
    auto workspace = pf_update_workspace<A>(count, n, n);
    if constexpr (std::is_same_v<Work<A>, float>)
        ASSERT_EQ(linears_cholesky_f32(&fixture.state.measurement_noise, &lv, workspace.view().decomposition), LINEARS_OK);
    else
        ASSERT_EQ(linears_cholesky_f64(&fixture.state.measurement_noise, &lv, workspace.view().decomposition), LINEARS_OK);
    ASSERT_EQ(PfKernels<A>::gaussian_exponent(&dv, &lv, &ev, workspace.view()), LINEARS_OK);
    ASSERT_EQ(PfKernels<A>::reweight(&wv, &ev), LINEARS_OK);
    Matrix<A> measurement(Dense(n, 1, Vec(n, 0)));
    auto zv = measurement.view();
    Model identity{[](const Vec& value) { return value; }};
    ASSERT_EQ(Pf<A>::update(&fixture.state, &zv, map_callback<A>, &identity,
                            nullptr, nullptr, workspace.view()), LINEARS_OK);
    for (std::size_t i = 0; i < count; ++i)
        EXPECT_EQ(fixture.weights[i + 1], expected_weights.data[i]) << i;
    EXPECT_EQ(identity.calls, static_cast<int>(count));
    fixture.guards(); workspace.guards();
}

TEST(ParticlePipelineAccuracy, CorrelatedCancellationKeepsPivotedSolveWeights) {
    /* Probe seed 0x504647415553, f32 trial 62338. An exact rational reference
     * gives exponent 2.530341569929938...; the legacy relative error is
     * 8.3899e-6, but direct forward substitution's 7.0724e-5 exceeds 2e-5.
     * Both return OK, so a RANGE-only fallback cannot protect this case. */
    const Dense covariance(3, 3, {
        1.3975498882522838e25L, -6098926.5L, 8.4578969108406454e-5L,
        -6098926.5L, 408233344.0L, 65760982466560.0L,
        8.4578969108406454e-5L, 65760982466560.0L, 1.05932260861102e19L});
    const Dense particles(2, 3, {-44182516.0L, -22992.71484375L, -3700297216.0L, 0, 0, 0});
    expect_legacy_update<f32>(particles, covariance);
}

TYPED_TEST(ParticlePipeline, DiagonalNoiseKeepsLegacyWeightsAcrossExponents) {
    using A = TypeParam;
    constexpr std::size_t n = 6, count = 17;
    Dense covariance(n, n), particles(count, n);
    const int single_exponents[n] = {-100, -60, -10, 0, 40, 100};
    const int double_exponents[n] = {-900, -500, -40, 0, 400, 900};
    for (std::size_t j = 0; j < n; ++j) {
        int exponent = std::is_same_v<Work<A>, float> ? single_exponents[j] : double_exponents[j];
        covariance(j, j) = std::ldexp(1.0L, exponent);
        for (std::size_t i = 0; i < count; ++i)
            particles(i, j) = std::ldexp(static_cast<long double>(int((i + j) % 9) - 4) / 4, exponent / 2);
    }
    expect_legacy_update<A>(particles, covariance);
}

} // namespace
