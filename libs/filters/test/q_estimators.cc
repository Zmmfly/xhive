#include "pf_support.h"

namespace {
using namespace pf_test;

template<class A> class FixedEstimators : public ::testing::Test {};
TYPED_TEST_SUITE(FixedEstimators, FixedFormats);

static_assert(std::is_same_v<decltype(filters_ukf_q15_workspace_t{}.signal), int16_t*>);
static_assert(std::is_same_v<decltype(filters_ukf_q31_workspace_t{}.signal), int32_t*>);
static_assert(!std::is_same_v<filters_ukf_q15_workspace_t, filters_ukf_f32_workspace_t>);
static_assert(!std::is_same_v<filters_ukf_q31_workspace_t, filters_ukf_f64_workspace_t>);

/* Exercise each advertised buffer independently. A zero-sized region has no
 * minimum-minus-one case; the fixture passes NULL for that region instead. */
template<class A, class Operation>
void expect_short_workspace(UkfWorkspace<A>& scratch, Operation operation) {
    for (int part = 0; part < 4; ++part) {
        auto workspace = scratch.view();
        std::size_t *capacity = part == 0 ? &workspace.capacity
                              : part == 1 ? &workspace.basic_bytes
                              : part == 2 ? &workspace.decomposition.capacity
                                          : &workspace.signal_capacity;
        if (*capacity == 0) continue;
        SCOPED_TRACE(part);
        --*capacity;
        EXPECT_EQ(operation(workspace), LINEARS_ERROR_WORKSPACE);
    }
}

TYPED_TEST(FixedEstimators, FinalSignalEndpointsAndLastCovarianceFailureAreAtomic) {
    using A = TypeParam;
    const long double lsb = std::ldexp(1.0L, -A::bits);
    Matrix<A> x(Dense(2, 1, {0, 0})), p(Dense::identity(2, 0.25L));
    Matrix<A> q(Dense::identity(2, 0)), r(Dense::identity(1, 0.125L));
    auto xv = x.view(), pv = p.view(), qv = q.view(), rv = r.view();
    UkfState<A> fixture(2, 1);
    auto scratch = UkfWorkspace<A>::predict(2);
    for (long double endpoint : {-1.0L, -1 + lsb, 1 - lsb, 1.0L,
                                 -1 - lsb / 4, 1 + lsb / 4}) {
        SCOPED_TRACE(static_cast<double>(endpoint));
        ASSERT_EQ(Ukf<A>::init(&fixture.state, work_encode<A>(1), work_encode<A>(2), work_encode<A>(0),
                               &xv, &pv, &qv, &rv), LINEARS_OK);
        const auto before = fixture.snapshot();
        Model constant{[=](const Vec&) { return Vec{0.25L, endpoint}; }};
        const auto status = Ukf<A>::predict(&fixture.state, map_callback<A>, &constant,
                                            nullptr, nullptr, scratch.view());
        if (endpoint < -1 || endpoint > 1) {
            EXPECT_EQ(status, LINEARS_ERROR_RANGE);
            EXPECT_EQ(fixture.snapshot(), before);
        } else {
            ASSERT_EQ(status, LINEARS_OK);
            EXPECT_EQ(fixture.x[1], A::encode(0.25L));
            EXPECT_EQ(fixture.x[2], A::encode(endpoint));
            for (std::size_t i = 1; i <= 4; ++i) EXPECT_EQ(fixture.p[i], 0);
        }
        fixture.guards(); scratch.guards();
    }
    /* x is valid and the first three covariance entries fit, but the last
     * diagonal is 4*.25+.125=1.125. Neither x nor any P entry may commit. */
    q.data[3] = A::encode(0.125L);
    ASSERT_EQ(Ukf<A>::init(&fixture.state, work_encode<A>(1), work_encode<A>(2), work_encode<A>(0),
                           &xv, &pv, &qv, &rv), LINEARS_OK);
    const auto before = fixture.snapshot();
    Model expand{[](const Vec& value) { return Vec{0.5L * value[0], 2 * value[1]}; }};
    EXPECT_EQ(Ukf<A>::predict(&fixture.state, map_callback<A>, &expand, nullptr, nullptr, scratch.view()),
              LINEARS_ERROR_RANGE);
    EXPECT_EQ(fixture.snapshot(), before);
    fixture.guards(); scratch.guards();
}

TYPED_TEST(FixedEstimators, DedicatedWorkKernelsKeepExpandedPriorsAndSignedSigmaWeights) {
    using A = TypeParam;
    Matrix<A> x(Dense(1, 1, {0.75L})), p(Dense(1, 1, {0.25L}));
    Matrix<A> q(Dense(1, 1, {0.125L})), r(Dense(1, 1, {0.125L}));
    auto xv = x.view(), pv = p.view(), qv = q.view(), rv = r.view();
    UkfState<A> fixture(1, 1);
    /* alpha=.5 gives Wm0=-3, Wc0=-.25 and Wi=2. None is a Q1 weight. */
    ASSERT_EQ(Ukf<A>::init(&fixture.state, work_encode<A>(0.5L), work_encode<A>(2), work_encode<A>(0),
                           &xv, &pv, &qv, &rv), LINEARS_OK);
    EXPECT_LT(fixture.state.weights.mean0, 0);
    EXPECT_LT(fixture.state.weights.covariance0, 0);
    EXPECT_GT(fixture.state.weights.other, 1);
    WorkMatrix<A> mean(decode<A>(xv)), covariance(decode<A>(pv)), process(decode<A>(qv));
    WorkMatrix<A> noise(decode<A>(rv)), measurement(Dense(1, 1, {0.25L}));
    WorkMatrix<A> prior_mean(1, 1), prior_covariance(1, 1), posterior_mean(1, 1), posterior_covariance(1, 1);
    auto mean_v = mean.view(), covariance_v = covariance.view(), process_v = process.view();
    auto noise_v = noise.view(), measurement_v = measurement.view();
    auto prior_x = prior_mean.view(), prior_p = prior_covariance.view();
    auto posterior_x = posterior_mean.view(), posterior_p = posterior_covariance.view();
    UkfWorkspace<A> predict_ws(FILTERS_UKF_Q_PREDICT_WORK_STORAGE(1), FILTERS_UKF_Q_PREDICT_BASIC_BYTES(1),
                               FILTERS_UKF_Q_PREDICT_DECOMP_STORAGE(1), FILTERS_UKF_Q_PREDICT_SIGNAL_STORAGE(1));
    UkfWorkspace<A> update_ws(FILTERS_UKF_Q_UPDATE_WORK_STORAGE(1, 1), FILTERS_UKF_Q_UPDATE_BASIC_BYTES(1, 1),
                              FILTERS_UKF_Q_UPDATE_DECOMP_STORAGE(1, 1), FILTERS_UKF_Q_UPDATE_SIGNAL_STORAGE(1, 1));
    Model transition{[](const Vec& value) { return Vec{2 * value[0]}; }};
    Model observation{[](const Vec& value) { return value; }};
    ASSERT_EQ(Ukf<A>::predict_work(&mean_v, &covariance_v, &process_v, &fixture.state.weights,
                                   map_callback<A>, &transition, nullptr, nullptr,
                                   &prior_x, &prior_p, predict_ws.view()), LINEARS_OK);
    expect_work<A>(prior_x, Dense(1, 1, {1.5L}));
    expect_work<A>(prior_p, Dense(1, 1, {1.125L}));
    ASSERT_GT(prior_mean.data[0], 1);
    ASSERT_GT(prior_covariance.data[0], 1);
    ASSERT_EQ(Ukf<A>::update_work(&prior_x, &prior_p, &noise_v, &measurement_v, &fixture.state.weights,
                                  map_callback<A>, &observation, nullptr, nullptr,
                                  &posterior_x, &posterior_p, update_ws.view()), LINEARS_OK);
    expect_work<A>(posterior_x, Dense(1, 1, {0.375L}));
    expect_work<A>(posterior_p, Dense(1, 1, {0.1125L}));
    const auto before = std::make_pair(prior_mean.data, prior_covariance.data);
    EXPECT_EQ(Ukf<A>::predict_work(&mean_v, &covariance_v, &process_v, nullptr,
                                   map_callback<A>, &transition, nullptr, nullptr,
                                   &prior_x, &prior_p, predict_ws.view()), LINEARS_ERROR_ARGUMENT);
    EXPECT_EQ(std::make_pair(prior_mean.data, prior_covariance.data), before);
    EXPECT_EQ(Ukf<A>::update_work(&prior_x, &prior_p, &noise_v, &measurement_v, &fixture.state.weights,
                                  map_callback<A>, &observation, nullptr, nullptr,
                                  &prior_x, &posterior_p, update_ws.view()), LINEARS_ERROR_ALIAS);
    predict_ws.guards(); update_ws.guards(); fixture.guards();
}

TYPED_TEST(FixedEstimators, SmallAlphaAndSubSignalResolutionWorkCovariancesStayPositive) {
    using A = TypeParam;
    using Kernel = std::conditional_t<A::bits == 15, f32, f64>;
    const long double cases[][3] = {
        {0, A::bits == 15 ? 1e-10L : 1e-24L, 0.001L},
        {0.75L, A::bits == 15 ? 1e-8L : 1e-16L, 1}
    };
    for (const auto& values : cases) {
        SCOPED_TRACE(static_cast<double>(values[2]));
        /* These are local work matrices, deliberately not raw-state inputs:
         * rounding P to a signal before the Q work call would erase it. */
        WorkMatrix<A> mean(Dense(1, 1, {values[0]})), covariance(Dense(1, 1, {values[1]}));
        WorkMatrix<A> process(Dense(1, 1, {0})), predicted_mean(1, 1), predicted_covariance(1, 1);
        auto xv = mean.view(), pv = covariance.view(), qv = process.view();
        auto out_x = predicted_mean.view(), out_p = predicted_covariance.view();
        typename Ukf<A>::Weights weights{};
        ASSERT_EQ(UkfKernels<Kernel>::weights(1, work_encode<A>(values[2]), work_encode<A>(2),
                                               work_encode<A>(0), &weights), LINEARS_OK);
        UkfWorkspace<A> scratch(FILTERS_UKF_Q_PREDICT_WORK_STORAGE(1), FILTERS_UKF_Q_PREDICT_BASIC_BYTES(1),
                                FILTERS_UKF_Q_PREDICT_DECOMP_STORAGE(1), FILTERS_UKF_Q_PREDICT_SIGNAL_STORAGE(1));
        Model identity{[](const Vec& value) { return value; }};
        ASSERT_EQ(Ukf<A>::predict_work(&xv, &pv, &qv, &weights, map_callback<A>, &identity,
                                       nullptr, nullptr, &out_x, &out_p, scratch.view()), LINEARS_OK);
        EXPECT_EQ(predicted_mean.data[0], mean.data[0]);
        ASSERT_GT(predicted_covariance.data[0], 0);
        const long double expected = covariance.data[0];
        EXPECT_LT(std::abs(static_cast<long double>(predicted_covariance.data[0]) - expected) / expected, 0.001L);
        scratch.guards();
    }
}

TYPED_TEST(FixedEstimators, NonfiniteLastCallbacksAndSamplesNeverCommit) {
    using A = TypeParam;
    constexpr std::size_t count = 5;
    Matrix<A> x(Dense(2, 1, {0.125L, -0.125L})), p(Dense::identity(2, 0.0625L));
    Matrix<A> q(Dense::identity(2, 0.015625L)), r(Dense::identity(1, 0.125L)), z(Dense(1, 1, {0}));
    auto xv = x.view(), pv = p.view(), qv = q.view(), rv = r.view(), zv = z.view();
    UkfState<A> ukf(2, 1);
    PfState<A> pf(count, 2, 1);
    auto ukf_predict = UkfWorkspace<A>::predict(2);
    auto ukf_update = UkfWorkspace<A>::update(2, 1);
    auto init_ws = pf_init_workspace<A>(count, 2);
    auto predict_ws = pf_predict_workspace<A>(count, 2);
    auto update_ws = pf_update_workspace<A>(count, 2, 1);
    Sampler sampler(0x514e414eu);
    sampler.script = {{0, 0}};
    ASSERT_EQ(Ukf<A>::init(&ukf.state, work_encode<A>(1), work_encode<A>(2), work_encode<A>(0),
                           &xv, &pv, &qv, &rv), LINEARS_OK);
    ASSERT_EQ(Pf<A>::init(&pf.state, &xv, &pv, &qv, &rv, sample_callback<A>, &sampler, init_ws.view()), LINEARS_OK);
    const auto ukf_before = ukf.snapshot();
    const auto pf_before = pf.snapshot();
    for (long double bad : {std::numeric_limits<long double>::quiet_NaN(),
                            std::numeric_limits<long double>::infinity(),
                            -std::numeric_limits<long double>::infinity()}) {
        int calls = 0;
        Model transition{[&](const Vec& value) {
            Vec result = value;
            if (++calls == static_cast<int>(count)) result.back() = bad;
            return result;
        }};
        EXPECT_EQ(Ukf<A>::predict(&ukf.state, map_callback<A>, &transition, nullptr, nullptr, ukf_predict.view()),
                  LINEARS_ERROR_RANGE);
        EXPECT_EQ(calls, count);
        EXPECT_EQ(ukf.snapshot(), ukf_before);
        calls = 0;
        EXPECT_EQ(Pf<A>::predict(&pf.state, map_callback<A>, &transition,
                                 sample_callback<A>, &sampler, predict_ws.view()), LINEARS_ERROR_RANGE);
        EXPECT_EQ(calls, count);
        EXPECT_EQ(pf.snapshot(), pf_before);
        Model observation{[](const Vec& value) { return Vec{value[0]}; }};
        Model residual;
        residual.residual = [=](const Vec&, const Vec&) { return Vec{bad}; };
        EXPECT_EQ(Ukf<A>::update(&ukf.state, &zv, map_callback<A>, &observation,
                                 residual_callback<A>, &residual, ukf_update.view()), LINEARS_ERROR_RANGE);
        EXPECT_EQ(Pf<A>::update(&pf.state, &zv, map_callback<A>, &observation,
                                residual_callback<A>, &residual, update_ws.view()), LINEARS_ERROR_RANGE);
        EXPECT_EQ(ukf.snapshot(), ukf_before);
        EXPECT_EQ(pf.snapshot(), pf_before);
        sampler.script.assign(count, Vec{0, 0});
        sampler.script.back().back() = bad;
        sampler.scripted = 0;
        EXPECT_EQ(Pf<A>::init(&pf.state, &xv, &pv, &qv, &rv, sample_callback<A>, &sampler, init_ws.view()),
                  LINEARS_ERROR_RANGE);
        EXPECT_EQ(pf.snapshot(), pf_before);
        sampler.script = {{0, 0}};
        sampler.scripted = 0;
    }
    ukf.guards(); pf.guards(); ukf_predict.guards(); ukf_update.guards();
    init_ws.guards(); predict_ws.guards(); update_ws.guards();
}

TYPED_TEST(FixedEstimators, ParticleCallbacksAreInterleavedWithoutAliasing) {
    using A = TypeParam;
    struct Trace { std::vector<char> events; char map_kind = 'T'; } trace;
    const auto map = +[](const linears_matrix_t *in, linears_matrix_t *out, void *context) -> linears_status_t {
        auto& log = *static_cast<Trace*>(context);
        log.events.push_back(log.map_kind);
        EXPECT_NE(in->data, out->data);
        static_cast<Work<A>*>(out->data)[0] = static_cast<const Work<A>*>(in->data)[0];
        return LINEARS_OK;
    };
    const auto sample = +[](linears_matrix_t *out, void *context) -> linears_status_t {
        static_cast<Trace*>(context)->events.push_back('S');
        static_cast<Work<A>*>(out->data)[0] = 0;
        return LINEARS_OK;
    };
    const auto residual = +[](const linears_matrix_t *a, const linears_matrix_t *b,
                              linears_matrix_t *out, void *context) -> linears_status_t {
        static_cast<Trace*>(context)->events.push_back('R');
        EXPECT_NE(a->data, out->data); EXPECT_NE(b->data, out->data);
        static_cast<Work<A>*>(out->data)[0] = static_cast<const Work<A>*>(a->data)[0]
                                            - static_cast<const Work<A>*>(b->data)[0];
        return LINEARS_OK;
    };
    Matrix<A> x(Dense(1, 1, {0})), p(Dense(1, 1, {0.0625L}));
    Matrix<A> q(Dense(1, 1, {0.015625L})), r(Dense(1, 1, {0.125L})), z(Dense(1, 1, {0.125L}));
    auto xv = x.view(), pv = p.view(), qv = q.view(), rv = r.view(), zv = z.view();
    PfState<A> fixture(3, 1, 1);
    auto init_ws = pf_init_workspace<A>(3, 1);
    auto predict_ws = pf_predict_workspace<A>(3, 1);
    auto update_ws = pf_update_workspace<A>(3, 1, 1);
    ASSERT_EQ(Pf<A>::init(&fixture.state, &xv, &pv, &qv, &rv, sample, &trace, init_ws.view()), LINEARS_OK);
    trace.events.clear();
    ASSERT_EQ(Pf<A>::predict(&fixture.state, map, &trace, sample, &trace, predict_ws.view()), LINEARS_OK);
    EXPECT_EQ(trace.events, (std::vector<char>{'T', 'S', 'T', 'S', 'T', 'S'}));
    trace.events.clear(); trace.map_kind = 'O';
    ASSERT_EQ(Pf<A>::update(&fixture.state, &zv, map, &trace, residual, &trace, update_ws.view()), LINEARS_OK);
    EXPECT_EQ(trace.events, (std::vector<char>{'O', 'R', 'O', 'R', 'O', 'R'}));
    fixture.guards(); init_ws.guards(); predict_ws.guards(); update_ws.guards();
}

TYPED_TEST(FixedEstimators, ExpandedLikelihoodResidualsAndInvalidRawWeights) {
    using A = TypeParam;
    using Raw = typename A::Raw;
    Matrix<A> x(Dense(1, 1, {0})), p(Dense(1, 1, {0.0625L}));
    Matrix<A> q(Dense(1, 1, {0.015625L})), r(Dense(1, 1, {0.125L})), z(Dense(1, 1, {0}));
    Matrix<A> mean(Dense(1, 1, {-0.75L})), covariance(Dense(1, 1, {-0.75L}));
    auto xv = x.view(), pv = p.view(), qv = q.view(), rv = r.view(), zv = z.view();
    auto mv = mean.view(), cv = covariance.view();
    PfState<A> fixture(3, 1, 1);
    auto init_ws = pf_init_workspace<A>(3, 1);
    auto update_ws = pf_update_workspace<A>(3, 1, 1);
    auto estimate_ws = pf_estimate_workspace<A>(3, 1);
    auto resample_ws = pf_resample_workspace<A>(3);
    Sampler sampler(0x514c494bu);
    sampler.script = {{0}};
    ASSERT_EQ(Pf<A>::init(&fixture.state, &xv, &pv, &qv, &rv, sample_callback<A>, &sampler, init_ws.view()), LINEARS_OK);
    fixture.particles[1] = A::encode(-0.5L);
    fixture.particles[2] = A::encode(0.5L);
    fixture.particles[3] = 0;
    fixture.weights[3] = 0;
    Model observation{[](const Vec& value) { return Vec{4 * value[0]}; }};
    Model residual;
    residual.residual = [](const Vec& a, const Vec& b) { return Vec{2 * (a[0] - b[0])}; };
    /* Both supported particles have |h(x)|=2 and |residual|=4, hence e=64.
     * The zero-weight particle has e=0, but must not erase supported weights
     * through early Q1 rounding or saturation of the likelihood exponent. */
    ASSERT_EQ(Pf<A>::update(&fixture.state, &zv, map_callback<A>, &observation,
                            residual_callback<A>, &residual, update_ws.view()), LINEARS_OK);
    EXPECT_EQ(fixture.weights[1], std::numeric_limits<Raw>::max());
    EXPECT_EQ(fixture.weights[2], std::numeric_limits<Raw>::max());
    EXPECT_EQ(fixture.weights[3], 0);
    ASSERT_EQ(Pf<A>::estimate(&fixture.state, nullptr, nullptr, &mv, &cv, estimate_ws.view()), LINEARS_OK);
    EXPECT_EQ(mean.data[0], 0);
    EXPECT_EQ(covariance.data[0], A::encode(0.25L));
    for (bool negative : {true, false}) {
        std::fill(fixture.weights.begin() + 1, fixture.weights.end() - 1, Raw{0});
        if (negative) {
            fixture.weights[1] = std::numeric_limits<Raw>::max();
            fixture.weights[3] = -1;
        }
        const auto before = fixture.snapshot();
        mean.data[0] = covariance.data[0] = A::encode(-0.75L);
        const auto outputs = std::make_pair(mean.data, covariance.data);
        Work<A> ess = -123;
        std::vector<std::size_t> indices(3, SIZE_MAX);
        EXPECT_EQ(Pf<A>::effective_sample_size(&fixture.state.weights, &ess), LINEARS_ERROR_DOMAIN);
        EXPECT_EQ(ess, -123);
        EXPECT_EQ(Pf<A>::estimate(&fixture.state, nullptr, nullptr, &mv, &cv, estimate_ws.view()), LINEARS_ERROR_DOMAIN);
        EXPECT_EQ(Pf<A>::update(&fixture.state, &zv, map_callback<A>, &observation,
                               nullptr, nullptr, update_ws.view()), LINEARS_ERROR_DOMAIN);
        EXPECT_EQ(Pf<A>::resample(&fixture.state, work_encode<A>(0.5L), indices.data(), 3, resample_ws.view()), LINEARS_ERROR_DOMAIN);
        EXPECT_EQ(fixture.snapshot(), before);
        EXPECT_EQ(std::make_pair(mean.data, covariance.data), outputs);
        for (auto index : indices) EXPECT_EQ(index, SIZE_MAX);
    }
    fixture.guards(); init_ws.guards(); update_ws.guards(); estimate_ws.guards(); resample_ws.guards();
}

TYPED_TEST(FixedEstimators, ReweightRetainsPriorAndLikelihoodDynamicRanges) {
    using A = TypeParam;
    using Raw = typename A::Raw;
    const long double exponent = A::bits == 15 ? 15 : 31;
    const long double observed = work_decode<A>(work_encode<A>(std::sqrt(exponent)));
    Matrix<A> x(Dense(1, 1, {0})), p(Dense(1, 1, {0.0625L}));
    Matrix<A> q(Dense(1, 1, {0.125L})), r(Dense(1, 1, {0.5L})), z(Dense(1, 1, {0}));
    auto xv = x.view(), pv = p.view(), qv = q.view(), rv = r.view(), zv = z.view();
    PfState<A> fixture(2, 1, 1);
    auto init_ws = pf_init_workspace<A>(2, 1);
    auto update_ws = pf_update_workspace<A>(2, 1, 1);
    Sampler sampler(0x51455850u);
    sampler.script = {{0}};
    ASSERT_EQ(Pf<A>::init(&fixture.state, &xv, &pv, &qv, &rv, sample_callback<A>, &sampler, init_ws.view()), LINEARS_OK);
    fixture.particles[1] = 0;
    fixture.particles[2] = A::encode(0.5L);
    fixture.weights[1] = 1;
    fixture.weights[2] = std::numeric_limits<Raw>::max();
    Model observation{[=](const Vec& value) { return Vec{value[0] == 0 ? 0 : observed}; }};
    const auto particles_before = fixture.particles;
    ASSERT_EQ(Pf<A>::update(&fixture.state, &zv, map_callback<A>, &observation,
                            nullptr, nullptr, update_ws.view()), LINEARS_OK);
    /* exp(-15/-31) is below one Q15/Q31 raw unit, but multiplying by the
     * prior MAX before final rescaling leaves a nonzero posterior weight. */
    const long double ratio = static_cast<long double>(std::numeric_limits<Raw>::max())
                              * std::exp(-observed * observed);
    const Vec weights = fixture.weight_values();
    EXPECT_EQ(fixture.weights[1], std::numeric_limits<Raw>::max());
    EXPECT_GT(fixture.weights[2], 0);
    expect_near<A>(weights[0], 1 / (1 + ratio));
    expect_near<A>(weights[1], ratio / (1 + ratio));
    EXPECT_EQ(fixture.particles, particles_before);
    fixture.guards(); init_ws.guards(); update_ws.guards();
}

TYPED_TEST(FixedEstimators, FullScaleSignedMomentsDoNotOverflowOrLoseCancellation) {
    using A = TypeParam;
    using Raw = typename A::Raw;
    constexpr Raw minimum = std::numeric_limits<Raw>::min(), maximum = std::numeric_limits<Raw>::max();
    Matrix<A> x(Dense(2, 1)), p(Dense::identity(2, 0.0625L));
    Matrix<A> q(Dense::identity(2, 0.015625L)), r(Dense(1, 1, {0.125L}));
    Matrix<A> mean(Dense(2, 1)), covariance(Dense(2, 2));
    auto xv = x.view(), pv = p.view(), qv = q.view(), rv = r.view(), mv = mean.view(), cv = covariance.view();
    PfState<A> fixture(2, 2, 1);
    auto init_ws = pf_init_workspace<A>(2, 2);
    auto estimate_ws = pf_estimate_workspace<A>(2, 2);
    Sampler sampler(0x5143414eu);
    sampler.script = {{0, 0}};
    ASSERT_EQ(Pf<A>::init(&fixture.state, &xv, &pv, &qv, &rv, sample_callback<A>, &sampler, init_ws.view()), LINEARS_OK);
    fixture.particles[1] = minimum; fixture.particles[2] = maximum;
    fixture.particles[3] = maximum; fixture.particles[4] = minimum;
    ASSERT_EQ(Pf<A>::estimate(&fixture.state, nullptr, nullptr, &mv, &cv, estimate_ws.view()), LINEARS_OK);
    /* Each mean is -0.5 raw units (ties away from zero). Variance rounds to
     * MAX and the signed cross moment to -MAX, not a saturated positive sum. */
    EXPECT_EQ(mean.data[0], -1); EXPECT_EQ(mean.data[1], -1);
    EXPECT_LE(std::abs(static_cast<int64_t>(covariance.data[0]) - maximum), 1);
    EXPECT_LE(std::abs(static_cast<int64_t>(covariance.data[3]) - maximum), 1);
    EXPECT_LE(std::abs(static_cast<int64_t>(covariance.data[1]) + maximum), 1);
    EXPECT_EQ(covariance.data[1], covariance.data[2]);
    fixture.guards(); init_ws.guards(); estimate_ws.guards();
}

TYPED_TEST(FixedEstimators, LargeRawWeightPipelinePreservesSmallTermsAndExactGather) {
    using A = TypeParam;
    using Raw = typename A::Raw;
    constexpr std::size_t count = 65537; // Beyond both Q15 1/N rounding thresholds.
    constexpr Raw maximum = std::numeric_limits<Raw>::max();
    Matrix<A> x(Dense(1, 1, {0})), p(Dense(1, 1, {0.0625L}));
    Matrix<A> q(Dense(1, 1, {0.015625L})), r(Dense(1, 1, {0.125L}));
    Matrix<A> mean(Dense(1, 1, {-0.75L})), covariance(Dense(1, 1, {-0.75L}));
    auto xv = x.view(), pv = p.view(), qv = q.view(), rv = r.view(), mv = mean.view(), cv = covariance.view();
    PfState<A> fixture(count, 1, 1);
    auto init_ws = pf_init_workspace<A>(count, 1);
    auto estimate_ws = pf_estimate_workspace<A>(count, 1);
    auto resample_ws = pf_resample_workspace<A>(count);
    Sampler sampler(0x514c4152u);
    sampler.script = {{0}};
    ASSERT_EQ(Pf<A>::init(&fixture.state, &xv, &pv, &qv, &rv, sample_callback<A>, &sampler, init_ws.view()), LINEARS_OK);
    for (std::size_t i = 1; i <= count; ++i) {
        EXPECT_EQ(fixture.weights[i], maximum);
        fixture.particles[i] = 1;
    }
    Work<A> ess = -1;
    ASSERT_EQ(Pf<A>::effective_sample_size(&fixture.state.weights, &ess), LINEARS_OK);
    EXPECT_NEAR(static_cast<long double>(ess), static_cast<long double>(count),
                8 * std::numeric_limits<Work<A>>::epsilon() * count);
    ASSERT_EQ(Pf<A>::estimate(&fixture.state, nullptr, nullptr, &mv, &cv, estimate_ws.view()), LINEARS_OK);
    EXPECT_EQ(mean.data[0], 1); // Each normalized summand is below one raw unit.
    EXPECT_EQ(covariance.data[0], 0);
    std::vector<std::size_t> indices(count + 1, SIZE_MAX);
    ASSERT_EQ(Pf<A>::resample(&fixture.state, work_encode<A>(0.5L), indices.data(), count, resample_ws.view()), LINEARS_OK);
    for (std::size_t i = 0; i < count; ++i) EXPECT_EQ(indices[i], i);
    EXPECT_EQ(indices.back(), SIZE_MAX);

    /* Maximal Q31 squares cannot be summed in a single 64-bit accumulator.
     * Tiny raw weights remain meaningful alongside two full-scale weights. */
    Vec weights(count);
    std::vector<Raw> particles(count);
    long double total = 0, squares = 0, weighted = 0;
    for (std::size_t i = 0; i < count; ++i) {
        const Raw weight = (i == count / 4 || i == 3 * count / 4) ? maximum : 1;
        particles[i] = A::encode(i % 2 ? -0.25L : 0.25L);
        fixture.weights[i + 1] = weight;
        fixture.particles[i + 1] = particles[i];
        weights[i] = weight;
        total += weight;
        squares += static_cast<long double>(weight) * weight;
        weighted += static_cast<long double>(weight) * A::decode_raw(particles[i]);
    }
    const long double expected_mean = weighted / total;
    long double expected_covariance = 0;
    for (std::size_t i = 0; i < count; ++i) {
        const long double deviation = A::decode_raw(particles[i]) - expected_mean;
        expected_covariance += weights[i] * deviation * deviation / total;
    }
    ASSERT_EQ(Pf<A>::effective_sample_size(&fixture.state.weights, &ess), LINEARS_OK);
    const long double expected_ess = total * total / squares;
    EXPECT_NEAR(static_cast<long double>(ess), expected_ess,
                16 * std::numeric_limits<Work<A>>::epsilon() * expected_ess);
    ASSERT_EQ(Pf<A>::estimate(&fixture.state, nullptr, nullptr, &mv, &cv, estimate_ws.view()), LINEARS_OK);
    EXPECT_LE(std::abs(static_cast<int64_t>(mean.data[0]) - A::encode(expected_mean)), 1);
    EXPECT_LE(std::abs(static_cast<int64_t>(covariance.data[0]) - A::encode(expected_covariance)), 2);
    const auto expected_indices = PfReference::systematic(weights, 0.25L);
    ASSERT_EQ(Pf<A>::resample(&fixture.state, work_encode<A>(0.25L), indices.data(), count, resample_ws.view()), LINEARS_OK);
    for (std::size_t i = 0; i < count; ++i) {
        EXPECT_EQ(indices[i], expected_indices[i]);
        EXPECT_EQ(fixture.particles[i + 1], particles[expected_indices[i]]);
        EXPECT_EQ(fixture.weights[i + 1], maximum);
    }
    EXPECT_EQ(indices.back(), SIZE_MAX);

    const std::size_t hot = count / 2;
    std::fill(fixture.weights.begin() + 1, fixture.weights.end() - 1, Raw{0});
    fixture.weights[hot + 1] = maximum;
    const Raw selected = fixture.particles[hot + 1];
    ASSERT_EQ(Pf<A>::effective_sample_size(&fixture.state.weights, &ess), LINEARS_OK);
    EXPECT_EQ(ess, 1);
    ASSERT_EQ(Pf<A>::resample(&fixture.state, work_encode<A>(0.5L), indices.data(), count, resample_ws.view()), LINEARS_OK);
    for (std::size_t i = 0; i < count; ++i) {
        EXPECT_EQ(indices[i], hot);
        EXPECT_EQ(fixture.particles[i + 1], selected);
    }
    fixture.guards(); init_ws.guards(); estimate_ws.guards(); resample_ws.guards();
}

TYPED_TEST(FixedEstimators, ExactMinimumWorkspacesCoverEachDominantDimension) {
    using A = TypeParam;
    const std::size_t shapes[][3] = {{1, 1, 1}, {3, 1, 3}, {5, 3, 1}};
    for (const auto& shape : shapes) {
        const std::size_t count = shape[0], n = shape[1], m = shape[2];
        SCOPED_TRACE(::testing::Message() << "N=" << count << " n=" << n << " m=" << m);
        Matrix<A> x(Dense(n, 1, Vec(n, 0.0625L))), p(Dense::identity(n, 0.0625L));
        Matrix<A> q(Dense::identity(n, 0.015625L)), r(Dense::identity(m, 0.125L)), z(Dense(m, 1));
        Matrix<A> mean(Dense(n, 1)), covariance(Dense(n, n));
        auto xv = x.view(), pv = p.view(), qv = q.view(), rv = r.view(), zv = z.view();
        auto mv = mean.view(), cv = covariance.view();
        UkfState<A> ukf(n, m);
        PfState<A> pf(count, n, m);
        auto ukf_predict = UkfWorkspace<A>::predict(n);
        auto ukf_update = UkfWorkspace<A>::update(n, m);
        auto pf_init = pf_init_workspace<A>(count, n);
        auto pf_predict = pf_predict_workspace<A>(count, n);
        auto pf_update = pf_update_workspace<A>(count, n, m);
        auto pf_estimate = pf_estimate_workspace<A>(count, n);
        auto pf_resample = pf_resample_workspace<A>(count);
        Model transition{[](const Vec& value) { return value; }};
        Model observation{[=](const Vec& value) { return Vec(m, 0.5L * value[0]); }};
        Sampler sampler(0x514d494eu);
        sampler.script = {Vec(n, 0)};
        ASSERT_EQ(Ukf<A>::init(&ukf.state, work_encode<A>(1), work_encode<A>(2), work_encode<A>(0),
                               &xv, &pv, &qv, &rv), LINEARS_OK);
        const auto uninitialized = pf.snapshot();
        expect_short_workspace(pf_init, [&](auto w) {
            return Pf<A>::init(&pf.state, &xv, &pv, &qv, &rv, sample_callback<A>, &sampler, w);
        });
        EXPECT_EQ(pf.snapshot(), uninitialized);
        EXPECT_EQ(sampler.calls, 0);
        ASSERT_EQ(Pf<A>::init(&pf.state, &xv, &pv, &qv, &rv,
                              sample_callback<A>, &sampler, pf_init.view()), LINEARS_OK);
        const auto ukf_before = ukf.snapshot();
        const auto pf_before = pf.snapshot();
        const auto outputs_before = std::make_pair(mean.data, covariance.data);
        sampler.calls = 0;
        expect_short_workspace(ukf_predict, [&](auto w) {
            return Ukf<A>::predict(&ukf.state, map_callback<A>, &transition, nullptr, nullptr, w);
        });
        expect_short_workspace(ukf_update, [&](auto w) {
            return Ukf<A>::update(&ukf.state, &zv, map_callback<A>, &observation, nullptr, nullptr, w);
        });
        expect_short_workspace(pf_predict, [&](auto w) {
            return Pf<A>::predict(&pf.state, map_callback<A>, &transition, sample_callback<A>, &sampler, w);
        });
        expect_short_workspace(pf_update, [&](auto w) {
            return Pf<A>::update(&pf.state, &zv, map_callback<A>, &observation, nullptr, nullptr, w);
        });
        expect_short_workspace(pf_estimate, [&](auto w) {
            return Pf<A>::estimate(&pf.state, nullptr, nullptr, &mv, &cv, w);
        });
        std::vector<std::size_t> indices(count, SIZE_MAX);
        expect_short_workspace(pf_resample, [&](auto w) {
            return Pf<A>::resample(&pf.state, work_encode<A>(0.5L), indices.data(), count, w);
        });
        EXPECT_EQ(ukf.snapshot(), ukf_before);
        EXPECT_EQ(pf.snapshot(), pf_before);
        EXPECT_EQ(std::make_pair(mean.data, covariance.data), outputs_before);
        EXPECT_EQ(transition.calls, 0); EXPECT_EQ(observation.calls, 0); EXPECT_EQ(sampler.calls, 0);
        for (auto index : indices) EXPECT_EQ(index, SIZE_MAX);
        ASSERT_EQ(Ukf<A>::predict(&ukf.state, map_callback<A>, &transition, nullptr, nullptr, ukf_predict.view()), LINEARS_OK);
        ASSERT_EQ(Ukf<A>::update(&ukf.state, &zv, map_callback<A>, &observation, nullptr, nullptr, ukf_update.view()), LINEARS_OK);
        ASSERT_EQ(Pf<A>::predict(&pf.state, map_callback<A>, &transition, sample_callback<A>, &sampler, pf_predict.view()), LINEARS_OK);
        ASSERT_EQ(Pf<A>::update(&pf.state, &zv, map_callback<A>, &observation, nullptr, nullptr, pf_update.view()), LINEARS_OK);
        ASSERT_EQ(Pf<A>::estimate(&pf.state, nullptr, nullptr, &mv, &cv, pf_estimate.view()), LINEARS_OK);
        ASSERT_EQ(Pf<A>::resample(&pf.state, work_encode<A>(0.5L), indices.data(), count, pf_resample.view()), LINEARS_OK);
        ukf.guards(); pf.guards(); ukf_predict.guards(); ukf_update.guards();
        pf_init.guards(); pf_predict.guards(); pf_update.guards(); pf_estimate.guards(); pf_resample.guards();
    }
}

} // namespace
