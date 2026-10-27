#ifndef FILTERS_TEST_UKF_SUPPORT_H
#define FILTERS_TEST_UKF_SUPPORT_H

/* Shared by ukf.cc, pf.cc and upf.cc: working-format adapters, long double
 * callbacks, guarded state/workspace fixtures and the Eigen UKF reference. */
#include "kalman_support.h"
#include "filters_ukf.h"
#include <Eigen/Dense>

#include <functional>
#include <tuple>

namespace ukf_test {
using namespace kalman_test;

using EigenMatrix = Eigen::Matrix<long double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>;
using Vec = std::vector<long double>;

inline EigenMatrix eigen_matrix(const Dense& value) {
    return Eigen::Map<const EigenMatrix>(value.values.data(),
        static_cast<Eigen::Index>(value.rows), static_cast<Eigen::Index>(value.cols));
}

inline Dense dense_matrix(const EigenMatrix& value) {
    return Dense(static_cast<std::size_t>(value.rows()), static_cast<std::size_t>(value.cols()),
                 Vec(value.data(), value.data() + value.size()));
}

/* UKF entry points per signal format; q15/q31 share the fixed working kernels. */
template<class A> struct Ukf;
#define UKF_TEST_API(tag, kernel) \
template<> struct Ukf<tag> { \
    using State = filters_ukf_##tag##_t; \
    using Weights = filters_ukf_##kernel##_weights_t; \
    using Workspace = filters_ukf_##kernel##_workspace_t; \
    using Fn = filters_ukf_##kernel##_fn; \
    using Residual = filters_ukf_##kernel##_residual_fn; \
    static constexpr auto weights = filters_ukf_##kernel##_weights; \
    static constexpr auto sigma_points = filters_ukf_##kernel##_sigma_points; \
    static constexpr auto propagate = filters_ukf_##kernel##_propagate; \
    static constexpr auto mean = filters_ukf_##kernel##_mean; \
    static constexpr auto deviations = filters_ukf_##kernel##_deviations; \
    static constexpr auto weighted_covariance = filters_ukf_##kernel##_weighted_covariance; \
    static constexpr auto predict_work = filters_ukf_##kernel##_predict_work; \
    static constexpr auto update_work = filters_ukf_##kernel##_update_work; \
    static constexpr auto init = filters_ukf_##tag##_init; \
    static constexpr auto predict = filters_ukf_##tag##_predict; \
    static constexpr auto update = filters_ukf_##tag##_update; \
};
UKF_TEST_API(f32, f32)
UKF_TEST_API(f64, f64)
UKF_TEST_API(q15, fixed)
UKF_TEST_API(q31, fixed)
#undef UKF_TEST_API

/* Working elements seen by callbacks and kernels: float, double or F31. */
template<class A> using Work = typename A::Coefficient;
template<class A> Work<A> work_encode(long double value) { return A::coefficient(value); }
template<class A> long double work_decode(Work<A> value) { return A::decode_coefficient(value); }
template<class A> Dense decode_work(const linears_matrix_t& matrix) {
    Dense result(matrix.rows, matrix.cols);
    const auto *raw = static_cast<const Work<A>*>(matrix.data);
    for (std::size_t i = 0; i < result.values.size(); ++i) result.values[i] = work_decode<A>(raw[i]);
    return result;
}
template<class A> void expect_work(const linears_matrix_t& actual, const Dense& expected,
                                   long double multiplier = 1) {
    ASSERT_EQ(actual.rows, expected.rows);
    ASSERT_EQ(actual.cols, expected.cols);
    const Dense values = decode_work<A>(actual);
    for (std::size_t i = 0; i < values.values.size(); ++i) {
        SCOPED_TRACE(i);
        expect_near<A>(values.values[i], expected.values[i], multiplier);
    }
}
template<class A> struct WorkMatrix {
    std::size_t rows, cols;
    std::vector<Work<A>> data;
    explicit WorkMatrix(const Dense& d) : rows(d.rows), cols(d.cols), data(d.values.size()) {
        for (std::size_t i = 0; i < data.size(); ++i) data[i] = work_encode<A>(d.values[i]);
    }
    WorkMatrix(std::size_t r, std::size_t c) : rows(r), cols(c), data(r * c, work_encode<A>(-0.75L)) {}
    linears_matrix_t view() { return {data.data(), rows, cols}; }
};

/* Test callbacks compute in long double; production never converts. */
struct Model {
    explicit Model(std::function<Vec(const Vec&)> m = {}) : map(std::move(m)) {}
    std::function<Vec(const Vec&)> map;
    std::function<Vec(const Vec&, const Vec&)> residual;
    int calls = 0;
    int fail_at = -1;
    linears_status_t failure = LINEARS_ERROR_NO_CONVERGENCE;
    bool nonfinite = false;
};
template<class A> Vec decode_vector(const linears_matrix_t *matrix) {
    Vec result(matrix->rows);
    for (std::size_t i = 0; i < result.size(); ++i)
        result[i] = work_decode<A>(static_cast<const Work<A>*>(matrix->data)[i]);
    return result;
}
template<class A> void encode_vector(const Vec& values, linears_matrix_t *out) {
    EXPECT_EQ(values.size(), out->rows);
    EXPECT_EQ(out->cols, 1u);
    for (std::size_t i = 0; i < values.size(); ++i)
        static_cast<Work<A>*>(out->data)[i] = work_encode<A>(values[i]);
}
template<class A> linears_status_t map_callback(const linears_matrix_t *in, linears_matrix_t *out, void *context) {
    auto *model = static_cast<Model*>(context);
    if (model->calls++ == model->fail_at) return model->failure;
    encode_vector<A>(model->map(decode_vector<A>(in)), out);
    if constexpr (A::bits == 0) {
        if (model->nonfinite) static_cast<Work<A>*>(out->data)[0] = std::numeric_limits<Work<A>>::quiet_NaN();
    }
    return LINEARS_OK;
}
template<class A> linears_status_t residual_callback(const linears_matrix_t *a, const linears_matrix_t *b,
                                                     linears_matrix_t *out, void *context) {
    auto *model = static_cast<Model*>(context);
    if (model->calls++ == model->fail_at) return model->failure;
    encode_vector<A>(model->residual(decode_vector<A>(a), decode_vector<A>(b)), out);
    return LINEARS_OK;
}

inline Vec linear_map(const Dense& matrix, const Vec& x) {
    Vec result(matrix.rows, 0);
    for (std::size_t r = 0; r < matrix.rows; ++r)
        for (std::size_t c = 0; c < matrix.cols; ++c) result[r] += matrix(r, c) * x[c];
    return result;
}
inline long double wrap(long double value, long double period) {
    while (value >= period / 2) value -= period;
    while (value < -period / 2) value += period;
    return value;
}

template<class A> struct UkfState {
    using Raw = typename A::Raw;
    static constexpr Raw sentinel = static_cast<Raw>(-12345);
    std::vector<Raw> x, p, q, r;
    typename Ukf<A>::State state;
    UkfState(std::size_t n, std::size_t m)
        : x(n + 2, sentinel), p(n * n + 2, sentinel), q(n * n + 2, sentinel), r(m * m + 2, sentinel),
          state{{x.data() + 1, n, 1}, {p.data() + 1, n, n}, {q.data() + 1, n, n}, {r.data() + 1, m, m},
                n, n * n, n * n, m * m, {}} {}
    void guards() const {
        for (const auto *v : {&x, &p, &q, &r}) { EXPECT_EQ(v->front(), sentinel); EXPECT_EQ(v->back(), sentinel); }
    }
    auto snapshot() const { return std::make_tuple(bytes(state), x, p, q, r); }
};

template<class A> struct UkfWorkspace {
    using W = Work<A>;
    std::size_t data_count, basic_bytes, decomp_count;
    std::vector<W> data;
    std::vector<linears_workspace_unit_t> basic;
    std::vector<W> decomp;
    UkfWorkspace(std::size_t count, std::size_t bytes_, std::size_t decomposition)
        : data_count(count), basic_bytes(bytes_), decomp_count(decomposition),
          data(count + 2, static_cast<W>(-23456)), basic(LINEARS_WORKSPACE_UNITS(bytes_) + 2),
          decomp(decomposition + 2, static_cast<W>(-34567)) {
        std::memset(basic.data(), 0x35, basic.size() * sizeof(basic[0]));
    }
    static UkfWorkspace predict(std::size_t n) {
        return A::bits ? UkfWorkspace(FILTERS_UKF_Q_PREDICT_STORAGE(n), FILTERS_UKF_Q_PREDICT_BASIC_BYTES(n),
                                      FILTERS_UKF_PREDICT_DECOMP_STORAGE(n))
                       : UkfWorkspace(FILTERS_UKF_PREDICT_STORAGE(n), FILTERS_UKF_PREDICT_BASIC_BYTES(n),
                                      FILTERS_UKF_PREDICT_DECOMP_STORAGE(n));
    }
    static UkfWorkspace update(std::size_t n, std::size_t m) {
        return A::bits ? UkfWorkspace(FILTERS_UKF_Q_UPDATE_STORAGE(n, m), FILTERS_UKF_Q_UPDATE_BASIC_BYTES(n, m),
                                      FILTERS_UKF_UPDATE_DECOMP_STORAGE(n, m))
                       : UkfWorkspace(FILTERS_UKF_UPDATE_STORAGE(n, m), FILTERS_UKF_UPDATE_BASIC_BYTES(n, m),
                                      FILTERS_UKF_UPDATE_DECOMP_STORAGE(n, m));
    }
    typename Ukf<A>::Workspace view() {
        return {data.data() + 1, data_count, basic.data() + 1, basic_bytes,
                {decomp_count ? decomp.data() + 1 : nullptr, decomp_count}};
    }
    void guards() const {
        EXPECT_EQ(data.front(), static_cast<W>(-23456)); EXPECT_EQ(data.back(), static_cast<W>(-23456));
        EXPECT_EQ(decomp.front(), static_cast<W>(-34567)); EXPECT_EQ(decomp.back(), static_cast<W>(-34567));
        for (auto c : bytes(basic.front())) EXPECT_EQ(c, 0x35);
        for (auto c : bytes(basic.back())) EXPECT_EQ(c, 0x35);
    }
};

/* Independent long-double unscented transform with Eigen's Cholesky. */
struct UkfReference {
    std::size_t n;
    long double scale, wm0, wc0, wi;
    EigenMatrix x, P;
    UkfReference(long double alpha, long double beta, long double kappa, const Dense& x0, const Dense& P0)
        : n(x0.rows), x(eigen_matrix(x0)), P(eigen_matrix(P0)) {
        scale = alpha * alpha * (static_cast<long double>(n) + kappa);
        const long double lambda = scale - static_cast<long double>(n);
        wm0 = lambda / scale;
        wc0 = wm0 + 1 - alpha * alpha + beta;
        wi = 1 / (2 * scale);
    }
    std::vector<Vec> sigma() const {
        const Eigen::LLT<EigenMatrix> llt(P * scale);
        EXPECT_EQ(llt.info(), Eigen::Success);
        const EigenMatrix L = llt.matrixL();
        std::vector<Vec> points(2 * n + 1, Vec(n));
        for (std::size_t j = 0; j < n; ++j) {
            points[0][j] = x(j, 0);
            for (std::size_t i = 0; i < n; ++i) {
                points[1 + i][j] = x(j, 0) + L(j, i);
                points[1 + n + i][j] = x(j, 0) - L(j, i);
            }
        }
        return points;
    }
    static Vec residual_of(const Model& model, const Vec& a, const Vec& b) {
        if (model.residual) return model.residual(a, b);
        Vec d(a.size());
        for (std::size_t i = 0; i < d.size(); ++i) d[i] = a[i] - b[i];
        return d;
    }
    Vec mean(const std::vector<Vec>& points, const Model& model) const {
        Vec result = points[0];
        for (auto& v : result) v = 0;
        for (std::size_t i = 0; i < points.size(); ++i) {
            const Vec d = residual_of(model, points[i], points[0]);
            for (std::size_t j = 0; j < d.size(); ++j) result[j] += (i == 0 ? wm0 : wi) * d[j];
        }
        for (std::size_t j = 0; j < result.size(); ++j) result[j] += points[0][j];
        return result;
    }
    EigenMatrix covariance(const std::vector<Vec>& a, const Vec& ma, const Model& ra,
                           const std::vector<Vec>& b, const Vec& mb, const Model& rb) const {
        EigenMatrix result = EigenMatrix::Zero(static_cast<Eigen::Index>(ma.size()), static_cast<Eigen::Index>(mb.size()));
        for (std::size_t i = 0; i < a.size(); ++i) {
            const Vec da = residual_of(ra, a[i], ma), db = residual_of(rb, b[i], mb);
            for (std::size_t r = 0; r < da.size(); ++r)
                for (std::size_t c = 0; c < db.size(); ++c) result(r, c) += (i == 0 ? wc0 : wi) * da[r] * db[c];
        }
        return result;
    }
    void predict(const Model& f, const Dense& Q) {
        std::vector<Vec> points = sigma();
        for (auto& p : points) p = f.map(p);
        const Vec mean_x = mean(points, f);
        P = covariance(points, mean_x, f, points, mean_x, f) + eigen_matrix(Q);
        for (std::size_t j = 0; j < n; ++j) x(j, 0) = mean_x[j];
    }
    void update(const Dense& z, const Model& h, const Dense& R) {
        const std::vector<Vec> points = sigma();
        std::vector<Vec> measured(points.size());
        for (std::size_t i = 0; i < points.size(); ++i) measured[i] = h.map(points[i]);
        const Vec mean_z = mean(measured, h);
        Vec mean_x(n);
        for (std::size_t j = 0; j < n; ++j) mean_x[j] = x(j, 0);
        const Model plain;
        const EigenMatrix S = covariance(measured, mean_z, h, measured, mean_z, h) + eigen_matrix(R);
        const EigenMatrix C = covariance(points, mean_x, plain, measured, mean_z, h);
        const Eigen::LDLT<EigenMatrix> factor(S);
        ASSERT_EQ(factor.info(), Eigen::Success);
        const EigenMatrix K = factor.solve(C.transpose()).transpose();
        const Vec innovation = residual_of(h, z.values, mean_z);
        const EigenMatrix nu = Eigen::Map<const EigenMatrix>(innovation.data(), static_cast<Eigen::Index>(innovation.size()), 1);
        x = x + K * nu;
        P = P - K * S * K.transpose();
    }
    Dense estimate() const { return dense_matrix(x); }
    Dense covariance_matrix() const { return dense_matrix(P); }
};

} // namespace ukf_test
#endif
