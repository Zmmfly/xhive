#include "kalman_support.h"
#include "linears_work.h"
#include <Eigen/Dense>

namespace {
using namespace kalman_test;

using EigenMatrix = Eigen::Matrix<long double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>;

EigenMatrix eigen_matrix(const Dense& value) {
    return Eigen::Map<const EigenMatrix>(value.values.data(),
        static_cast<Eigen::Index>(value.rows), static_cast<Eigen::Index>(value.cols));
}

Dense dense_matrix(const EigenMatrix& value) {
    return Dense(static_cast<std::size_t>(value.rows()), static_cast<std::size_t>(value.cols()),
                 std::vector<long double>(value.data(), value.data() + value.size()));
}

struct MatrixReference {
    Dense x, p;
    void predict(const Dense& f, const Dense& q) {
        const EigenMatrix transition = eigen_matrix(f), covariance = eigen_matrix(p);
        x = dense_matrix(transition * eigen_matrix(x));
        p = dense_matrix(transition * covariance * transition.transpose() + eigen_matrix(q));
    }
    void update(const Dense& z, const Dense& h, const Dense& r) {
        const EigenMatrix observation = eigen_matrix(h), covariance = eigen_matrix(p);
        const EigenMatrix estimate = eigen_matrix(x), hp = observation * covariance;
        const EigenMatrix innovation = hp * observation.transpose() + eigen_matrix(r);
        const Eigen::LDLT<EigenMatrix> factor(innovation);
        ASSERT_EQ(factor.info(), Eigen::Success);
        const EigenMatrix gain = factor.solve(hp).transpose();
        x = dense_matrix(estimate + gain * (eigen_matrix(z) - observation * estimate));
        /* Independent exact-arithmetic equivalent, not production Joseph code. */
        p = dense_matrix(covariance - gain * hp);
    }
};

template<class Api> class KalmanMatrix : public ::testing::Test {};
TYPED_TEST_SUITE(KalmanMatrix, Formats);

TYPED_TEST(KalmanMatrix, ScalarAgreementAndZeroInnovation) {
    using A = TypeParam;
    Matrix<A> x(Dense(1, 1, {A::bits ? 0.25L : 2})), p(Dense(1, 1, {A::bits ? 0.25L : 4}));
    Matrix<A> f(Dense(1, 1, {A::bits ? 0.5L : 1})), q(Dense(1, 1, {A::bits ? 0.0625L : 1}));
    Matrix<A> z(Dense(1, 1, {A::bits ? 0.0625L : 2}));
    Matrix<A> h(Dense(1, 1, {A::bits ? 0.5L : 1})), r(Dense(1, 1, {A::bits ? 0.125L : 1}));
    auto xv = x.view(), pv = p.view(), fv = f.view(), qv = q.view();
    auto zv = z.view(), hv = h.view(), rv = r.view();
    MatrixState<A> fixture(1);
    Workspace<A> predict(1), update(1, 1);
    typename A::State scalar{};
    ASSERT_EQ(A::reset_matrix(&fixture.state, &xv, &pv), LINEARS_OK);
    ASSERT_EQ(A::reset(&scalar, x.data[0], p.data[0]), LINEARS_OK);
    ASSERT_EQ(A::predict_matrix(&fixture.state, &fv, &qv, predict.view()), LINEARS_OK);
    ASSERT_EQ(A::predict(&scalar, A::coefficient_from_raw(f.data[0]), q.data[0]), LINEARS_OK);
    ASSERT_EQ(A::update_matrix(&fixture.state, &zv, &hv, &rv, update.view()), LINEARS_OK);
    ASSERT_EQ(A::update(&scalar, z.data[0], A::coefficient_from_raw(h.data[0]), r.data[0]), LINEARS_OK);
    expect_near<A>(decode<A>(fixture.state.estimate).values[0], A::decode(scalar.estimate));
    expect_near<A>(decode<A>(fixture.state.covariance).values[0], A::decode(scalar.covariance));
    expect_near<A>(decode<A>(fixture.state.covariance).values[0], A::bits ? 0.1L : 5.0L / 6);
    /* Q MAX coefficients remain 1-LSB, not wide exact unity. */
    f.data[0] = A::encode(1); h.data[0] = A::encode(1);
    ASSERT_EQ(A::reset_matrix(&fixture.state, &xv, &pv), LINEARS_OK);
    ASSERT_EQ(A::reset(&scalar, x.data[0], p.data[0]), LINEARS_OK);
    ASSERT_EQ(A::predict_matrix(&fixture.state, &fv, &qv, predict.view()), LINEARS_OK);
    ASSERT_EQ(A::predict(&scalar, A::coefficient_from_raw(f.data[0]), q.data[0]), LINEARS_OK);
    ASSERT_EQ(A::update_matrix(&fixture.state, &zv, &hv, &rv, update.view()), LINEARS_OK);
    ASSERT_EQ(A::update(&scalar, z.data[0], A::coefficient_from_raw(h.data[0]), r.data[0]), LINEARS_OK);
    expect_near<A>(decode<A>(fixture.state.estimate).values[0], A::decode(scalar.estimate));
    expect_near<A>(decode<A>(fixture.state.covariance).values[0], A::decode(scalar.covariance));
    fixture.guards(); predict.guards(); update.guards();
}

TYPED_TEST(KalmanMatrix, JosephRetainsRepresentableNoise) {
    using A = TypeParam;
    Matrix<A> x(Dense(1, 1, {0})), p(Dense(1, 1, {A::bits ? 0.5L : 1}));
    Matrix<A> z(Dense(1, 1, {0})), h(Dense(1, 1, {1}));
    Matrix<A> r(Dense(1, 1, {std::ldexp(1.0L, A::bits ? -A::bits : -80)}));
    auto xv = x.view(), pv = p.view(), zv = z.view(), hv = h.view(), rv = r.view();
    MatrixState<A> fixture(1);
    Workspace<A> workspace(1, 1);
    ASSERT_EQ(A::reset_matrix(&fixture.state, &xv, &pv), LINEARS_OK);
    ASSERT_EQ(A::update_matrix(&fixture.state, &zv, &hv, &rv, workspace.view()), LINEARS_OK);
    const long double variance = decode<A>(fixture.state.covariance).values[0];
    EXPECT_GT(variance, 0);
    EXPECT_LT(std::abs(variance / decode<A>(rv).values[0] - 1), 1e-5L);
    if constexpr (A::bits != 0) {
        const auto before_x = fixture.x, before_p = fixture.p;
        r.data[0] = A::encode(std::ldexp(1.0L, -80));
        ASSERT_EQ(r.data[0], 0);
        EXPECT_EQ(A::update_matrix(&fixture.state, &zv, &hv, &rv, workspace.view()), LINEARS_ERROR_NOT_POSITIVE_DEFINITE);
        EXPECT_EQ(fixture.x, before_x); EXPECT_EQ(fixture.p, before_p);
    }
    fixture.guards(); workspace.guards();
}

TYPED_TEST(KalmanMatrix, SemidefiniteStateAndFixedFractionEndpoints) {
    using A = TypeParam;
    Matrix<A> x(Dense(2, 1, {-1, 0})), zero(Dense::identity(2, 0));
    Matrix<A> f(Dense(2, 2, {-1, 0, 0, 0.5L}));
    Matrix<A> z(Dense(2, 1, {-0.5L, 0.5L})), h(Dense::identity(2, 0.5L)), r(Dense::identity(2, 0.25L));
    auto xv = x.view(), pv = zero.view(), fv = f.view(), zv = z.view(), hv = h.view(), rv = r.view();
    MatrixState<A> fixture(2);
    Workspace<A> prediction(2), correction(2, 2);
    ASSERT_EQ(A::reset_matrix(&fixture.state, &xv, &pv), LINEARS_OK);
    ASSERT_EQ(A::predict_matrix(&fixture.state, &fv, &pv, prediction.view()), LINEARS_OK);
    const Dense predicted = decode<A>(fixture.state.estimate);
    if constexpr (A::bits != 0) {
        EXPECT_EQ(fixture.x[1], std::numeric_limits<typename A::Raw>::max());
        EXPECT_EQ(1 - predicted.values[0], std::ldexp(1.0L, -A::bits));
    } else {
        EXPECT_EQ(predicted.values[0], 1);
    }
    EXPECT_EQ(predicted.values[1], 0);
    ASSERT_EQ(A::update_matrix(&fixture.state, &zv, &hv, &rv, correction.view()), LINEARS_OK);
    expect_matrix<A>(fixture.state.estimate, predicted);
    expect_matrix<A>(fixture.state.covariance, Dense::identity(2, 0));
    /* Rank-deficient nonzero PSD P is accepted: only R and S need Cholesky. */
    Matrix<A> rank_one(Dense(2, 2, {0.25L, 0.25L, 0.25L, 0.25L}));
    auto rank_one_view = rank_one.view();
    ASSERT_EQ(A::reset_matrix(&fixture.state, &xv, &rank_one_view), LINEARS_OK);
    MatrixReference reference{decode<A>(xv), decode<A>(rank_one_view)};
    reference.update(decode<A>(zv), decode<A>(hv), decode<A>(rv));
    ASSERT_EQ(A::update_matrix(&fixture.state, &zv, &hv, &rv, correction.view()), LINEARS_OK);
    expect_matrix<A>(fixture.state.estimate, reference.x);
    expect_matrix<A>(fixture.state.covariance, reference.p);
    fixture.guards(); prediction.guards(); correction.guards();
}

TYPED_TEST(KalmanMatrix, WideWorkingGainInnovationAndExactIdentity) {
    using A = TypeParam;
    const long double models[2][3] = {{0.5L, 0.25L, 0.015625L}, {0.75L, 0.75L, 0.75L}};
    for (int i = 0; i < 2; ++i) {
        Matrix<A> x(Dense(1, 1, {0})), p(Dense(1, 1, {models[i][0]}));
        Matrix<A> z(Dense(1, 1, {0.125L})), h(Dense(1, 1, {models[i][1]})), r(Dense(1, 1, {models[i][2]}));
        auto xv = x.view(), pv = p.view(), zv = z.view(), hv = h.view(), rv = r.view();
        const long double covariance = decode<A>(pv).values[0], observation = decode<A>(hv).values[0];
        const long double innovation = observation * covariance * observation + decode<A>(rv).values[0];
        if (i == 0) ASSERT_GT(covariance * observation / innovation, 1);
        else ASSERT_GT(innovation, 1);
        MatrixState<A> fixture(1);
        Workspace<A> workspace(1, 1);
        MatrixReference reference{decode<A>(xv), decode<A>(pv)};
        reference.update(decode<A>(zv), decode<A>(hv), decode<A>(rv));
        ASSERT_EQ(A::reset_matrix(&fixture.state, &xv, &pv), LINEARS_OK);
        ASSERT_EQ(A::update_matrix(&fixture.state, &zv, &hv, &rv, workspace.view()), LINEARS_OK);
        expect_matrix<A>(fixture.state.estimate, reference.x);
        expect_matrix<A>(fixture.state.covariance, reference.p);
        fixture.guards(); workspace.guards();
    }
    Matrix<A> x(Dense(2, 1, {0.25L, -0.25L})), p(Dense::identity(2, 0.5L));
    Matrix<A> z(Dense(2, 1, {0.5L, -0.5L})), h(Dense(2, 2)), r(Dense::identity(2, 0.25L));
    auto xv = x.view(), pv = p.view(), zv = z.view(), hv = h.view(), rv = r.view();
    MatrixState<A> fixture(2);
    Workspace<A> workspace(2, 2);
    ASSERT_EQ(A::reset_matrix(&fixture.state, &xv, &pv), LINEARS_OK);
    const auto before_x = fixture.x, before_p = fixture.p;
    for (int i = 0; i < 400; ++i)
        ASSERT_EQ(A::update_matrix(&fixture.state, &zv, &hv, &rv, workspace.view()), LINEARS_OK);
    EXPECT_EQ(fixture.x, before_x); EXPECT_EQ(fixture.p, before_p);
    fixture.guards(); workspace.guards();
}

TYPED_TEST(KalmanMatrix, RandomizedRectangularModelsAgainstIndependentOracle) {
    using A = TypeParam;
    std::mt19937 random(0x4b4d4154u);
    for (int trial = 0; trial < 96; ++trial) {
        const std::size_t n = trial % 4 + 1, m = (trial / 4) % 4 + 1;
        SCOPED_TRACE(::testing::Message() << "trial=" << trial << " n=" << n << " m=" << m);
        Matrix<A> x(random_dense(random, n, 1, A::bits ? 0.1L : 1));
        Matrix<A> p(random_spd(random, n, A::bits ? 0.125L : 1, A::bits ? 0.08L : 0.4L));
        Matrix<A> q(Dense::identity(n, A::bits ? 0.002L : 0.01L));
        Matrix<A> f(Dense::identity(n, A::bits ? 0.7L : 1) + random_dense(random, n, n, A::bits ? 0.025L : 0.05L));
        Matrix<A> z(random_dense(random, m, 1, A::bits ? 0.1L : 1));
        Matrix<A> h(random_dense(random, m, n, A::bits ? 0.2L : 0.7L));
        Matrix<A> r(random_spd(random, m, A::bits ? 0.125L : 0.2L, A::bits ? 0.08L : 0.4L));
        auto xv = x.view(), pv = p.view(), fv = f.view(), qv = q.view();
        auto zv = z.view(), hv = h.view(), rv = r.view();
        const auto f_before = f.data, q_before = q.data, h_before = h.data, r_before = r.data;
        MatrixState<A> fixture(n);
        Workspace<A> prediction(n), correction(n, m);
        ASSERT_EQ(A::reset_matrix(&fixture.state, &xv, &pv), LINEARS_OK);
        MatrixReference reference{decode<A>(xv), decode<A>(pv)};
        reference.predict(decode<A>(fv), decode<A>(qv));
        ASSERT_EQ(A::predict_matrix(&fixture.state, &fv, &qv, prediction.view()), LINEARS_OK);
        expect_matrix<A>(fixture.state.estimate, reference.x);
        expect_matrix<A>(fixture.state.covariance, reference.p);
        reference.update(decode<A>(zv), decode<A>(hv), decode<A>(rv));
        ASSERT_EQ(A::update_matrix(&fixture.state, &zv, &hv, &rv, correction.view()), LINEARS_OK);
        expect_matrix<A>(fixture.state.estimate, reference.x);
        expect_matrix<A>(fixture.state.covariance, reference.p);
        const Dense covariance = decode<A>(fixture.state.covariance);
        for (std::size_t i = 0; i < n; ++i) {
            EXPECT_GE(covariance(i, i), 0);
            for (std::size_t j = 0; j < n; ++j) EXPECT_EQ(covariance(i, j), covariance(j, i));
        }
        for (int i = 0; i < 6; ++i) {
            const Dense u = random_dense(random, n, 1);
            EXPECT_GE((u.transpose() * covariance * u).values[0], -A::tolerance);
        }
        EXPECT_EQ(f.data, f_before); EXPECT_EQ(q.data, q_before);
        EXPECT_EQ(h.data, h_before); EXPECT_EQ(r.data, r_before);
        fixture.guards(); prediction.guards(); correction.guards();
    }
}

TYPED_TEST(KalmanMatrix, LongTrackingSequenceResetAndReentrancy) {
    using A = TypeParam;
    Matrix<A> x(Dense(2, 1, {0, 0})), p(Dense::identity(2, A::bits ? 0.25L : 1));
    Matrix<A> f(Dense(2, 2, {A::bits ? 0.95L : 1, A::bits ? 0.025L : 0.05L, 0, A::bits ? 0.9L : 0.99L}));
    Matrix<A> q(Dense(2, 2, {A::bits ? 0.002L : 0.005L, 0, 0, A::bits ? 0.001L : 0.002L}));
    Matrix<A> h(Dense(1, 2, {A::bits ? 0.75L : 1, 0})), r(Dense(1, 1, {A::bits ? 0.05L : 0.1L}));
    auto xv = x.view(), pv = p.view(), fv = f.view(), qv = q.view(), hv = h.view(), rv = r.view();
    MatrixState<A> first(2), solo(2), other(2);
    Workspace<A> pred(2), upd(2, 1), independent_pred(2), independent_upd(2, 1);
    ASSERT_EQ(A::reset_matrix(&first.state, &xv, &pv), LINEARS_OK);
    ASSERT_EQ(A::reset_matrix(&solo.state, &xv, &pv), LINEARS_OK);
    ASSERT_EQ(A::reset_matrix(&other.state, &xv, &pv), LINEARS_OK);
    MatrixReference reference{decode<A>(xv), decode<A>(pv)};
    std::mt19937 random(0x4b534551u);
    std::uniform_real_distribution<double> noise(A::bits ? -0.005 : -0.05, A::bits ? 0.005 : 0.05);
    for (int i = 0; i < 1200; ++i) {
        SCOPED_TRACE(i);
        Matrix<A> measurement(Dense(1, 1, {(A::bits ? 0.2L : 1) * std::sin(i * 0.01L) + noise(random)}));
        Matrix<A> unrelated(Dense(1, 1, {A::bits ? 0.4L : 4}));
        auto zv = measurement.view(), unrelated_z = unrelated.view();
        ASSERT_EQ(A::predict_matrix(&first.state, &fv, &qv, pred.view()), LINEARS_OK);
        ASSERT_EQ(A::predict_matrix(&other.state, &fv, &qv, independent_pred.view()), LINEARS_OK);
        ASSERT_EQ(A::update_matrix(&first.state, &zv, &hv, &rv, upd.view()), LINEARS_OK);
        ASSERT_EQ(A::update_matrix(&other.state, &unrelated_z, &hv, &rv, independent_upd.view()), LINEARS_OK);
        ASSERT_EQ(A::predict_matrix(&solo.state, &fv, &qv, pred.view()), LINEARS_OK);
        ASSERT_EQ(A::update_matrix(&solo.state, &zv, &hv, &rv, upd.view()), LINEARS_OK);
        reference.predict(decode<A>(fv), decode<A>(qv));
        reference.update(decode<A>(zv), decode<A>(hv), decode<A>(rv));
        expect_matrix<A>(first.state.estimate, reference.x, 3);
        expect_matrix<A>(first.state.covariance, reference.p, 3);
        EXPECT_EQ(first.x, solo.x); EXPECT_EQ(first.p, solo.p);
    }
    ASSERT_EQ(A::reset_matrix(&first.state, &xv, &pv), LINEARS_OK);
    expect_matrix<A>(first.state.estimate, decode<A>(xv));
    expect_matrix<A>(first.state.covariance, decode<A>(pv));
    first.guards(); solo.guards(); other.guards();
    pred.guards(); upd.guards(); independent_pred.guards(); independent_upd.guards();
}

TYPED_TEST(KalmanMatrix, StorageCapacityWorkspaceBoundsAndSentinels) {
    using A = TypeParam;
    Matrix<A> x(Dense(2, 1, {A::bits ? 0.25L : 1, A::bits ? 0.5L : 2})), p(Dense::identity(2, A::bits ? 0.25L : 1));
    Matrix<A> f(Dense::identity(2, A::bits ? 0.5L : 1)), q(Dense::identity(2, A::bits ? 0.01L : 0.1L));
    Matrix<A> z(Dense(1, 1, {A::bits ? 0.25L : 1})), h(Dense(1, 2, {A::bits ? 0.5L : 1, 0}));
    Matrix<A> r(Dense(1, 1, {A::bits ? 0.25L : 1}));
    auto xv = x.view(), pv = p.view(), fv = f.view(), qv = q.view();
    auto zv = z.view(), hv = h.view(), rv = r.view();
    MatrixState<A> fixture(2);
    Workspace<A> prediction(2), correction(2, 1);
    fixture.state.estimate_capacity = 1;
    const auto uninitialized_x = fixture.x, uninitialized_p = fixture.p;
    EXPECT_EQ(A::reset_matrix(&fixture.state, &xv, &pv), LINEARS_ERROR_WORKSPACE);
    EXPECT_EQ(fixture.x, uninitialized_x); EXPECT_EQ(fixture.p, uninitialized_p);
    fixture.state.estimate_capacity = 2;
    fixture.state.covariance_capacity = 3;
    EXPECT_EQ(A::reset_matrix(&fixture.state, &xv, &pv), LINEARS_ERROR_WORKSPACE);
    fixture.state.covariance_capacity = 4;
    ASSERT_EQ(A::reset_matrix(&fixture.state, &xv, &pv), LINEARS_OK);
    const auto before = bytes(fixture.state);
    const auto x_before = bytes(fixture.x[1]);
    const auto data_x = fixture.x, data_p = fixture.p;
    auto w = prediction.view();
    --w.capacity;
    EXPECT_EQ(A::predict_matrix(&fixture.state, &fv, &qv, w), LINEARS_ERROR_WORKSPACE);
    w = prediction.view(); --w.basic_bytes;
    EXPECT_EQ(A::predict_matrix(&fixture.state, &fv, &qv, w), LINEARS_ERROR_WORKSPACE);
    w = prediction.view(); w.data = nullptr;
    EXPECT_EQ(A::predict_matrix(&fixture.state, &fv, &qv, w), LINEARS_ERROR_WORKSPACE);
    w = prediction.view(); w.basic = nullptr;
    EXPECT_EQ(A::predict_matrix(&fixture.state, &fv, &qv, w), LINEARS_ERROR_WORKSPACE);
    for (int part = 0; part < 3; ++part) {
        auto update = correction.view();
        if (part == 0) --update.capacity;
        if (part == 1) --update.basic_bytes;
        if (part == 2) --update.decomposition.capacity;
        EXPECT_EQ(A::update_matrix(&fixture.state, &zv, &hv, &rv, update), LINEARS_ERROR_WORKSPACE);
    }
    auto update = correction.view(); update.decomposition.data = nullptr;
    EXPECT_EQ(A::update_matrix(&fixture.state, &zv, &hv, &rv, update), LINEARS_ERROR_WORKSPACE);
    w = prediction.view();
    w.data = reinterpret_cast<typename A::Intermediate*>(reinterpret_cast<unsigned char*>(w.data) + 1);
    EXPECT_EQ(A::predict_matrix(&fixture.state, &fv, &qv, w), LINEARS_ERROR_WORKSPACE);
    update = correction.view();
    update.basic = reinterpret_cast<linears_workspace_unit_t*>(reinterpret_cast<unsigned char*>(update.basic) + 1);
    EXPECT_EQ(A::update_matrix(&fixture.state, &zv, &hv, &rv, update), LINEARS_ERROR_WORKSPACE);
    update = correction.view();
    update.decomposition.data = reinterpret_cast<typename A::Working*>(
        reinterpret_cast<unsigned char*>(update.decomposition.data) + 1);
    EXPECT_EQ(A::update_matrix(&fixture.state, &zv, &hv, &rv, update), LINEARS_ERROR_WORKSPACE);
    EXPECT_EQ(bytes(fixture.state), before); EXPECT_EQ(bytes(fixture.x[1]), x_before);
    EXPECT_EQ(fixture.x, data_x); EXPECT_EQ(fixture.p, data_p);
    ASSERT_EQ(A::predict_matrix(&fixture.state, &fv, &qv, prediction.view()), LINEARS_OK);
    ASSERT_EQ(A::update_matrix(&fixture.state, &zv, &hv, &rv, correction.view()), LINEARS_OK);
    fixture.guards(); prediction.guards(); correction.guards();
}

TYPED_TEST(KalmanMatrix, AliasRulesIncludeExactResetAndReadOnlySharing) {
    using A = TypeParam;
    Matrix<A> x(Dense(2, 1, {A::bits ? 0.25L : 1, A::bits ? 0.5L : 2}));
    Matrix<A> p(Dense::identity(2, A::bits ? 0.25L : 1)), identity(Dense::identity(2, A::bits ? 0.25L : 1));
    Matrix<A> z(Dense(1, 1, {A::bits ? 0.25L : 1})), h(Dense(1, 2, {A::bits ? 0.5L : 1, 0}));
    Matrix<A> r(Dense(1, 1, {A::bits ? 0.25L : 1}));
    auto xv = x.view(), pv = p.view(), iv = identity.view();
    auto zv = z.view(), hv = h.view(), rv = r.view();
    MatrixState<A> fixture(2);
    Workspace<A> prediction(2), correction(2, 1);
    ASSERT_EQ(A::reset_matrix(&fixture.state, &xv, &pv), LINEARS_OK);
    ASSERT_EQ(A::reset_matrix(&fixture.state, &fixture.state.estimate, &fixture.state.covariance), LINEARS_OK);
    auto own_x = fixture.state.estimate, own_p = fixture.state.covariance;
    ASSERT_EQ(A::reset_matrix(&fixture.state, &own_x, &own_p), LINEARS_OK);
    const auto data_x = fixture.x, data_p = fixture.p;
    const auto context = bytes(fixture.state);
    auto partial = own_x; partial.data = static_cast<typename A::Raw*>(own_x.data) + 1;
    EXPECT_EQ(A::reset_matrix(&fixture.state, &partial, &pv), LINEARS_ERROR_ALIAS);
    auto cross = own_x; cross.data = own_p.data;
    EXPECT_EQ(A::reset_matrix(&fixture.state, &cross, &pv), LINEARS_ERROR_ALIAS);
    auto forbidden = own_x; forbidden.data = &fixture.state.estimate;
    EXPECT_EQ(A::reset_matrix(&fixture.state, &forbidden, &pv), LINEARS_ERROR_ALIAS);
    EXPECT_EQ(A::predict_matrix(&fixture.state, &own_p, &iv, prediction.view()), LINEARS_ERROR_ALIAS);
    auto w = prediction.view();
    const auto previous_x = fixture.state.estimate.data;
    fixture.state.estimate.data = w.data; /* Reject overlap before treating it as signal data. */
    EXPECT_EQ(A::predict_matrix(&fixture.state, &iv, &iv, w), LINEARS_ERROR_ALIAS);
    fixture.state.estimate.data = previous_x;
    auto aliased_input = iv; aliased_input.data = w.data;
    EXPECT_EQ(A::predict_matrix(&fixture.state, &aliased_input, &iv, w), LINEARS_ERROR_ALIAS);
    auto u = correction.view(); u.decomposition.data = u.data;
    EXPECT_EQ(A::update_matrix(&fixture.state, &zv, &hv, &rv, u), LINEARS_ERROR_ALIAS);
    auto previous_p = fixture.state.covariance.data;
    fixture.state.covariance.data = fixture.state.estimate.data;
    EXPECT_EQ(A::reset_matrix(&fixture.state, &xv, &pv), LINEARS_ERROR_ALIAS);
    fixture.state.covariance.data = previous_p;
    EXPECT_EQ(fixture.x, data_x); EXPECT_EQ(fixture.p, data_p);
    EXPECT_EQ(bytes(fixture.state), context);
    /* Read-only inputs may be the same descriptor and array. */
    ASSERT_EQ(A::predict_matrix(&fixture.state, &iv, &iv, prediction.view()), LINEARS_OK);
    expect_matrix<A>(fixture.state.covariance, Dense::identity(2, A::bits ? 0.265625L : 2));
    fixture.guards(); prediction.guards(); correction.guards();
}

TYPED_TEST(KalmanMatrix, DimensionsNoiseAndFailureAtomicity) {
    using A = TypeParam;
    Matrix<A> x(Dense(2, 1, {A::bits ? 0.25L : 1, A::bits ? 0.5L : 2})), p(Dense::identity(2, A::bits ? 0.75L : 10));
    Matrix<A> f(Dense::identity(2)), q(Dense::identity(2, 0));
    Matrix<A> z(Dense(2, 1, {A::bits ? 0.25L : 1, A::bits ? 0.5L : 2}));
    Matrix<A> h(Dense::identity(2, A::bits ? 0.875L : 1)), r(Dense::identity(2, A::bits ? 0.25L : 1));
    Matrix<A> bad_r(Dense(2, 2, {A::bits ? 0.25L : 1, A::bits ? 0.5L : 2, A::bits ? 0.5L : 2, A::bits ? 0.25L : 1}));
    Matrix<A> zero_r(Dense::identity(2, 0)), negative_q(Dense::identity(2, -1));
    Matrix<A> asymmetric(Dense(2, 2, {0.25L, 0.125L, 0, 0.25L}));
    auto xv = x.view(), pv = p.view(), fv = f.view(), qv = q.view();
    auto zv = z.view(), hv = h.view(), rv = r.view(), bad_rv = bad_r.view();
    auto zero_rv = zero_r.view(), negative_qv = negative_q.view(), asymmetric_v = asymmetric.view();
    MatrixState<A> fixture(2);
    Workspace<A> prediction(2), correction(2, 2);
    ASSERT_EQ(A::reset_matrix(&fixture.state, &xv, &pv), LINEARS_OK);
    const auto before = bytes(fixture.state);
    const auto data_x = fixture.x, data_p = fixture.p;
    EXPECT_EQ(A::reset_matrix(nullptr, &xv, &pv), LINEARS_ERROR_ARGUMENT);
    EXPECT_EQ(A::reset_matrix(&fixture.state, nullptr, &pv), LINEARS_ERROR_ARGUMENT);
    EXPECT_EQ(A::predict_matrix(nullptr, &fv, &qv, prediction.view()), LINEARS_ERROR_ARGUMENT);
    EXPECT_EQ(A::update_matrix(nullptr, &zv, &hv, &rv, correction.view()), LINEARS_ERROR_ARGUMENT);
    EXPECT_EQ(A::update_matrix(&fixture.state, nullptr, &hv, &rv, correction.view()), LINEARS_ERROR_ARGUMENT);
    EXPECT_EQ(A::reset_matrix(&fixture.state, &xv, &asymmetric_v), LINEARS_ERROR_DOMAIN);
    EXPECT_EQ(A::reset_matrix(&fixture.state, &xv, &negative_qv), LINEARS_ERROR_DOMAIN);
    EXPECT_EQ(A::predict_matrix(&fixture.state, &fv, &negative_qv, prediction.view()), LINEARS_ERROR_DOMAIN);
    EXPECT_EQ(A::predict_matrix(&fixture.state, &fv, &asymmetric_v, prediction.view()), LINEARS_ERROR_DOMAIN);
    EXPECT_EQ(A::update_matrix(&fixture.state, &zv, &hv, &asymmetric_v, correction.view()), LINEARS_ERROR_DOMAIN);
    EXPECT_EQ(A::update_matrix(&fixture.state, &zv, &hv, &bad_rv, correction.view()), LINEARS_ERROR_NOT_POSITIVE_DEFINITE);
    EXPECT_EQ(A::update_matrix(&fixture.state, &zv, &hv, &zero_rv, correction.view()), LINEARS_ERROR_NOT_POSITIVE_DEFINITE);
    auto wrong = hv; wrong.cols = 1;
    EXPECT_EQ(A::update_matrix(&fixture.state, &zv, &wrong, &rv, correction.view()), LINEARS_ERROR_DIMENSION);
    wrong = fv; wrong.rows = 1;
    EXPECT_EQ(A::predict_matrix(&fixture.state, &wrong, &qv, prediction.view()), LINEARS_ERROR_DIMENSION);
    wrong = zv; wrong.rows = 0;
    EXPECT_EQ(A::update_matrix(&fixture.state, &wrong, &hv, &rv, correction.view()), LINEARS_ERROR_DIMENSION);
    wrong = xv; wrong.data = nullptr;
    EXPECT_EQ(A::reset_matrix(&fixture.state, &wrong, &pv), LINEARS_ERROR_ARGUMENT);
    auto oversized = fixture.state;
    oversized.estimate.rows = SIZE_MAX;
    oversized.covariance.rows = oversized.covariance.cols = SIZE_MAX;
    EXPECT_EQ(A::reset_matrix(&oversized, &xv, &pv), LINEARS_ERROR_SIZE);
    EXPECT_EQ(bytes(fixture.state), before); EXPECT_EQ(fixture.x, data_x); EXPECT_EQ(fixture.p, data_p);
    fixture.guards(); prediction.guards(); correction.guards();
}

TYPED_TEST(KalmanMatrix, NumericRangesAndAtomicLatePredictionFailure) {
    using A = TypeParam;
    Matrix<A> x(Dense(1, 1, {A::bits ? 0.125L : 1})), p(Dense(1, 1, {A::bits ? 0.25L : 1}));
    Matrix<A> f(Dense(1, 1, {A::bits ? -1 : 2})), q(Dense(1, 1, {0}));
    Matrix<A> z(Dense(1, 1, {A::bits ? 0.75L : 1})), h(Dense(1, 1, {A::bits ? 0.125L : 1}));
    Matrix<A> r(Dense(1, 1, {A::bits ? 0.015625L : 1}));
    auto xv = x.view(), pv = p.view(), fv = f.view(), qv = q.view();
    auto zv = z.view(), hv = h.view(), rv = r.view();
    MatrixState<A> fixture(1);
    Workspace<A> prediction(1), correction(1, 1);
    ASSERT_EQ(A::reset_matrix(&fixture.state, &xv, &pv), LINEARS_OK);
    const auto before = bytes(fixture.state);
    const auto data_x = fixture.x, data_p = fixture.p;
    if constexpr (A::bits == 0) {
        for (auto invalid : {std::numeric_limits<typename A::Raw>::quiet_NaN(),
                             std::numeric_limits<typename A::Raw>::infinity(),
                             -std::numeric_limits<typename A::Raw>::infinity()}) {
            typename A::Raw bad_value = invalid;
            linears_matrix_t bad{&bad_value, 1, 1};
            EXPECT_EQ(A::reset_matrix(&fixture.state, &bad, &pv), LINEARS_ERROR_RANGE);
            EXPECT_EQ(A::reset_matrix(&fixture.state, &xv, &bad), LINEARS_ERROR_RANGE);
            EXPECT_EQ(A::predict_matrix(&fixture.state, &bad, &qv, prediction.view()), LINEARS_ERROR_RANGE);
            EXPECT_EQ(A::predict_matrix(&fixture.state, &fv, &bad, prediction.view()), LINEARS_ERROR_RANGE);
            EXPECT_EQ(A::update_matrix(&fixture.state, &bad, &hv, &rv, correction.view()), LINEARS_ERROR_RANGE);
            EXPECT_EQ(A::update_matrix(&fixture.state, &zv, &bad, &rv, correction.view()), LINEARS_ERROR_RANGE);
            EXPECT_EQ(A::update_matrix(&fixture.state, &zv, &hv, &bad, correction.view()), LINEARS_ERROR_RANGE);
        }
    } else {
        MatrixReference reference{decode<A>(xv), decode<A>(pv)};
        reference.update(decode<A>(zv), decode<A>(hv), decode<A>(rv));
        ASSERT_GT(reference.x.values[0], 1);
        EXPECT_EQ(A::update_matrix(&fixture.state, &zv, &hv, &rv, correction.view()), LINEARS_ERROR_RANGE);
        z.data[0] = A::encode(-0.75L);
        MatrixReference negative{decode<A>(xv), decode<A>(pv)};
        negative.update(decode<A>(zv), decode<A>(hv), decode<A>(rv));
        ASSERT_LT(negative.x.values[0], -1);
        EXPECT_EQ(A::update_matrix(&fixture.state, &zv, &hv, &rv, correction.view()), LINEARS_ERROR_RANGE);
    }
    EXPECT_EQ(bytes(fixture.state), before); EXPECT_EQ(fixture.x, data_x); EXPECT_EQ(fixture.p, data_p);
    /* x is valid in scratch; floating P overflows or fixed P reaches +1.5. */
    if constexpr (A::bits == 0) p.data[0] = std::numeric_limits<typename A::Raw>::max();
    else { p.data[0] = A::encode(0.75L); q.data[0] = A::encode(0.75L); }
    pv = p.view();
    ASSERT_EQ(A::reset_matrix(&fixture.state, &xv, &pv), LINEARS_OK);
    const auto large_before = bytes(fixture.state);
    const auto large_x = fixture.x, large_p = fixture.p;
    EXPECT_EQ(A::predict_matrix(&fixture.state, &fv, &qv, prediction.view()), LINEARS_ERROR_RANGE);
    EXPECT_EQ(bytes(fixture.state), large_before);
    EXPECT_EQ(fixture.x, large_x); EXPECT_EQ(fixture.p, large_p);
    fixture.guards(); prediction.guards(); correction.guards();
}

TEST(KalmanMatrixQ31, WorkingRoundingBoundaryDiffersFromDirectSignalContract) {
    using A = q31;
    Matrix<A> x(Dense(5, 1, {0.25L, 0.25L, 0.25L, 0.25L, 0.25L}));
    Matrix<A> p(Dense(5, 5)), f(Dense(5, 5)), q(Dense(5, 5));
    for (std::size_t i = 0; i < 4; ++i) f.data[i] = INT32_MAX;
    f.data[4] = 5;
    auto xv = x.view(), pv = p.view(), fv = f.view(), qv = q.view();

    /* Independent exact integer dot: F62 raw = 2^62 + 2^29, i.e. 1 + 2^-33. */
    int64_t exact_dot = 0;
    for (std::size_t i = 0; i < 5; ++i)
        exact_dot += static_cast<int64_t>(f.data[i]) * static_cast<int64_t>(x.data[i]);
    const int64_t exact_one_f62 = INT64_C(1) << 62;
    ASSERT_EQ(exact_dot, exact_one_f62 + (INT64_C(1) << 29));
    ASSERT_GT(exact_dot, exact_one_f62);
    ASSERT_EQ(exact_dot % LINEARS_FIXED_ONE, INT64_C(1) << 29);
    const int64_t independently_rounded = (exact_dot + LINEARS_FIXED_ONE / 2) / LINEARS_FIXED_ONE;
    ASSERT_EQ(independently_rounded, LINEARS_FIXED_ONE);

    std::array<linears_fixed_t, 25> working_f{};
    std::array<linears_fixed_t, 5> working_x{}, working_result{};
    for (std::size_t i = 0; i < working_f.size(); ++i) working_f[i] = f.data[i];
    for (std::size_t i = 0; i < working_x.size(); ++i) working_x[i] = x.data[i];
    linears_matrix_t wf{working_f.data(), 5, 5}, wx{working_x.data(), 5, 1};
    linears_matrix_t wr{working_result.data(), 5, 1};
    linears_workspace_unit_t basic[LINEARS_WORKSPACE_UNITS(LINEARS_BASIC_WORKSPACE_BYTES(5, 1))];
    ASSERT_EQ(linears_mul_fixed(&wf, &wx, &wr, basic, sizeof basic), LINEARS_OK);
    EXPECT_EQ(working_result[0], independently_rounded);
    for (std::size_t i = 1; i < working_result.size(); ++i) EXPECT_EQ(working_result[i], 0);

    int32_t narrowed = 123;
    ASSERT_EQ(linears_fixed_to_q31(working_result[0], &narrowed), LINEARS_OK);
    EXPECT_EQ(narrowed, INT32_MAX);
    linears_accum_t exact_accumulator{static_cast<uint64_t>(exact_dot), 0};
    int32_t exact_output = 123;
    EXPECT_EQ(linears_accum_to_q31(exact_accumulator, &exact_output), LINEARS_ERROR_RANGE);
    EXPECT_EQ(exact_output, 123);
    std::array<int32_t, 5> direct_output{{123, 123, 123, 123, 123}};
    const auto direct_before = direct_output;
    linears_matrix_t direct{direct_output.data(), 5, 1};
    EXPECT_EQ(linears_mul_q31(&fv, &xv, &direct, basic, sizeof basic), LINEARS_ERROR_RANGE);
    EXPECT_EQ(direct_output, direct_before);

    MatrixState<A> fixture(5);
    Workspace<A> workspace(5);
    ASSERT_EQ(A::reset_matrix(&fixture.state, &xv, &pv), LINEARS_OK);
    ASSERT_EQ(A::predict_matrix(&fixture.state, &fv, &qv, workspace.view()), LINEARS_OK);
    EXPECT_EQ(fixture.x[1], INT32_MAX); /* Exact result asserted; no tolerance masks the policy. */
    for (std::size_t i = 2; i < 6; ++i) EXPECT_EQ(fixture.x[i], 0);
    for (std::size_t i = 1; i < 26; ++i) EXPECT_EQ(fixture.p[i], 0);
    fixture.guards(); workspace.guards();
}

TEST(KalmanWorkspace, ConstantExpressionsAndOverflowSentinels) {
    static_assert(FILTERS_KALMAN_PREDICT_STORAGE(2) == 18, "prediction raw count");
    static_assert(FILTERS_KALMAN_UPDATE_STORAGE(2, 3) == 68, "update raw count");
    static_assert(FILTERS_KALMAN_UPDATE_DECOMP_STORAGE(2, 3) == 15, "solve working count");
    static_assert(FILTERS_KALMAN_PREDICT_BASIC_BYTES(2) == 4 * sizeof(linears_workspace_unit_t), "basic bytes");
    static_assert(FILTERS_KALMAN_Q_PREDICT_STORAGE(2) == 32, "Q F31 prediction count");
    static_assert(FILTERS_KALMAN_Q_UPDATE_STORAGE(2, 3) == 92, "Q F31 update count");
    static_assert(FILTERS_KALMAN_Q_PREDICT_BASIC_BYTES(2) == 6 * sizeof(linears_workspace_unit_t), "Q commit staging");
    EXPECT_EQ(FILTERS_KALMAN_PREDICT_STORAGE(SIZE_MAX), SIZE_MAX);
    EXPECT_EQ(FILTERS_KALMAN_UPDATE_STORAGE(SIZE_MAX, 2), SIZE_MAX);
    EXPECT_EQ(FILTERS_KALMAN_UPDATE_STORAGE(2, SIZE_MAX), SIZE_MAX);
    EXPECT_EQ(FILTERS_KALMAN_UPDATE_BASIC_BYTES(SIZE_MAX, 2), SIZE_MAX);
    EXPECT_EQ(FILTERS_KALMAN_UPDATE_DECOMP_STORAGE(2, SIZE_MAX), SIZE_MAX);
    EXPECT_EQ(FILTERS_KALMAN_Q_PREDICT_STORAGE(SIZE_MAX), SIZE_MAX);
    EXPECT_EQ(FILTERS_KALMAN_Q_UPDATE_STORAGE(SIZE_MAX, 2), SIZE_MAX);
    EXPECT_EQ(FILTERS_KALMAN_Q_UPDATE_STORAGE(2, SIZE_MAX), SIZE_MAX);
    EXPECT_EQ(FILTERS_KALMAN_Q_PREDICT_BASIC_BYTES(SIZE_MAX), SIZE_MAX);
    EXPECT_EQ(FILTERS_KALMAN_Q_UPDATE_BASIC_BYTES(2, SIZE_MAX), SIZE_MAX);
}

} // namespace
