#include "filters_pf.h"

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>
#include <random>
#include <vector>

namespace {

/* The references retain the pre-optimization composition of public linears
 * operations. Compare object representations, not a relaxed numeric tolerance. */
template<class T> struct Api;
#define FLOAT_API(type, tag) \
template<> struct Api<type> { \
    using Workspace = filters_ukf_##tag##_workspace_t; \
    using Residual = filters_ukf_##tag##_residual_fn; \
    using Sample = filters_pf_##tag##_sample_fn; \
    static constexpr auto mean = filters_ukf_##tag##_mean; \
    static constexpr auto deviations = filters_ukf_##tag##_deviations; \
    static constexpr auto covariance = filters_ukf_##tag##_weighted_covariance; \
    static constexpr auto gaussian = filters_pf_##tag##_gaussian_exponent; \
    static constexpr auto perturb = filters_pf_##tag##_perturb; \
    static constexpr auto transpose = linears_transpose_##tag; \
    static constexpr auto mul = linears_mul_##tag; \
    static constexpr auto add = linears_add_##tag; \
    static constexpr auto axpby = linears_axpby_##tag; \
    static constexpr auto solve = linears_solve_##tag; \
};
FLOAT_API(float, f32)
FLOAT_API(double, f64)
#undef FLOAT_API

template<class T> struct Matrix {
    std::vector<T> data;
    linears_matrix_t view;
    Matrix(std::size_t rows, std::size_t cols, T initial = T(19.25))
        : data(rows * cols, initial), view{data.data(), rows, cols} {}
};

template<class T> struct Scratch {
    std::size_t count, units, decomposition_count;
    std::vector<T> data, decomposition;
    std::vector<linears_workspace_unit_t> basic;
    Scratch(std::size_t count_, std::size_t bytes_, std::size_t decomposition_ = 0)
        : count(count_), units(LINEARS_WORKSPACE_UNITS(bytes_)),
          decomposition_count(decomposition_), data(count + 2, T(71.25)),
          decomposition(decomposition_count + 2, T(71.25)), basic(units + 2) {
        for (auto& unit : basic) unit.integer = INT64_C(0x123456789abcdef);
    }
    typename Api<T>::Workspace view() {
        /* Supply one unused extra element in every array, also guarded. */
        return {data.data() + 1, count + 1, basic.data() + 1,
                (units + 1) * sizeof(linears_workspace_unit_t),
                {decomposition.data() + 1, decomposition_count + 1}};
    }
    void guards() const {
        EXPECT_EQ(data.front(), T(71.25));
        EXPECT_EQ(data.back(), T(71.25));
        EXPECT_EQ(decomposition.front(), T(71.25));
        EXPECT_EQ(decomposition.back(), T(71.25));
        EXPECT_EQ(basic.front().integer, INT64_C(0x123456789abcdef));
        EXPECT_EQ(basic.back().integer, INT64_C(0x123456789abcdef));
    }
};

template<class T>
void identical(const Matrix<T>& actual, const Matrix<T>& expected) {
    ASSERT_EQ(actual.data.size(), expected.data.size());
    for (std::size_t i = 0; i < actual.data.size(); ++i)
        EXPECT_EQ(std::memcmp(&actual.data[i], &expected.data[i], sizeof(T)), 0) << "element " << i;
}

#define REF_TRY(expression) do { \
    const auto status = (expression); \
    if (status != LINEARS_OK) return status; \
} while (0)

template<class T>
linears_status_t old_mean(const linears_matrix_t& points, const linears_matrix_t& weights,
                          typename Api<T>::Residual residual, void* context,
                          linears_matrix_t& out, typename Api<T>::Workspace workspace) {
    const auto s = points.rows, k = points.cols;
    workspace.basic_bytes = FILTERS_UKF_MEAN_BASIC_BYTES(s, k);
    linears_matrix_t transposed{workspace.data, k, s};
    if (residual == nullptr) {
        REF_TRY(Api<T>::transpose(&points, &transposed, workspace.basic, workspace.basic_bytes));
        return Api<T>::mul(&transposed, &weights, &out, workspace.basic, workspace.basic_bytes);
    }
    linears_matrix_t deviations{workspace.data, s, k};
    transposed.data = workspace.data + s * k;
    linears_matrix_t reference{points.data, k, 1};
    REF_TRY(Api<T>::deviations(&points, &reference, residual, context, &deviations, workspace));
    REF_TRY(Api<T>::transpose(&deviations, &transposed, workspace.basic, workspace.basic_bytes));
    REF_TRY(Api<T>::mul(&transposed, &weights, &out, workspace.basic, workspace.basic_bytes));
    return Api<T>::add(&reference, &out, &out, workspace.basic, workspace.basic_bytes);
}

template<class T>
linears_status_t old_covariance(const linears_matrix_t& a, const linears_matrix_t& b,
                                const linears_matrix_t& weights, linears_matrix_t& out,
                                typename Api<T>::Workspace workspace) {
    const auto s = a.rows, p = a.cols, q = b.cols;
    workspace.basic_bytes = FILTERS_UKF_WEIGHTED_COVARIANCE_BASIC_BYTES(s, p, q);
    linears_matrix_t scaled{workspace.data, s, q}, transposed{workspace.data + s * q, p, s};
    for (std::size_t i = 0; i < s; ++i) {
        linears_matrix_t source{static_cast<T*>(b.data) + i * q, q, 1};
        linears_matrix_t target{workspace.data + i * q, q, 1};
        REF_TRY(Api<T>::axpby(static_cast<const T*>(weights.data)[i], &source, T(0), &source,
                              &target, workspace.basic, workspace.basic_bytes));
    }
    REF_TRY(Api<T>::transpose(&a, &transposed, workspace.basic, workspace.basic_bytes));
    return Api<T>::mul(&transposed, &scaled, &out, workspace.basic, workspace.basic_bytes);
}

template<class T>
linears_status_t old_gaussian(const linears_matrix_t& deviations, const linears_matrix_t& factor,
                              linears_matrix_t& out, typename Api<T>::Workspace workspace) {
    const auto k = deviations.cols;
    workspace.basic_bytes = FILTERS_PF_GAUSSIAN_EXPONENT_BASIC_BYTES;
    workspace.decomposition.capacity = FILTERS_PF_GAUSSIAN_EXPONENT_DECOMP_STORAGE(k);
    linears_matrix_t solved{workspace.data, k, 1}, transposed{workspace.data, 1, k};
    for (std::size_t i = 0; i < deviations.rows; ++i) {
        linears_matrix_t deviation{static_cast<T*>(deviations.data) + i * k, k, 1};
        linears_matrix_t result{static_cast<T*>(out.data) + i, 1, 1};
        REF_TRY(Api<T>::solve(&factor, &deviation, &solved, workspace.decomposition));
        REF_TRY(Api<T>::mul(&transposed, &solved, &result, workspace.basic, workspace.basic_bytes));
        REF_TRY(Api<T>::axpby(T(0.5), &result, T(0), &result, &result,
                              workspace.basic, workspace.basic_bytes));
    }
    return LINEARS_OK;
}

template<class T>
linears_status_t old_perturb(const linears_matrix_t& points, const linears_matrix_t& factor,
                             typename Api<T>::Sample sample, void* context,
                             linears_matrix_t& out, typename Api<T>::Workspace workspace) {
    const auto k = points.cols;
    workspace.basic_bytes = FILTERS_PF_PERTURB_BASIC_BYTES(k);
    linears_matrix_t draw{workspace.data, k, 1}, noise{workspace.data + k, k, 1};
    for (std::size_t i = 0; i < points.rows; ++i) {
        REF_TRY(sample(&draw, context));
        REF_TRY(Api<T>::mul(&factor, &draw, &noise, workspace.basic, workspace.basic_bytes));
        linears_matrix_t source{static_cast<T*>(points.data) + i * k, k, 1};
        linears_matrix_t target{static_cast<T*>(out.data) + i * k, k, 1};
        REF_TRY(Api<T>::add(&source, &noise, &target, workspace.basic, workspace.basic_bytes));
    }
    return LINEARS_OK;
}
#undef REF_TRY

template<class T> struct Script {
    std::vector<T> values;
    int calls = 0;
    int fail_at = -1;
    int nonfinite_at = -1;
};

template<class T>
linears_status_t draw(linears_matrix_t* out, void* opaque) {
    auto& script = *static_cast<Script<T>*>(opaque);
    const int call = script.calls++;
    if (call == script.fail_at) return LINEARS_ERROR_NO_CONVERGENCE;
    auto* output = static_cast<T*>(out->data);
    for (std::size_t j = 0; j < out->rows; ++j)
        output[j] = script.values[(static_cast<std::size_t>(call) * out->rows + j) % script.values.size()];
    if (call == script.nonfinite_at) output[out->rows - 1] = std::numeric_limits<T>::quiet_NaN();
    return LINEARS_OK;
}

template<class T>
linears_status_t residual(const linears_matrix_t* a, const linears_matrix_t* b,
                           linears_matrix_t* out, void* opaque) {
    auto& script = *static_cast<Script<T>*>(opaque);
    const int call = script.calls++;
    if (call == script.fail_at) return LINEARS_ERROR_NO_CONVERGENCE;
    const auto* av = static_cast<const T*>(a->data);
    const auto* bv = static_cast<const T*>(b->data);
    auto* output = static_cast<T*>(out->data);
    for (std::size_t j = 0; j < a->rows; ++j) output[j] = av[j] - bv[j];
    if (call == script.nonfinite_at) output[a->rows - 1] = std::numeric_limits<T>::infinity();
    return LINEARS_OK;
}

template<class T>
void fill(Matrix<T>& matrix, std::mt19937& random) {
    for (auto& value : matrix.data) {
        const int numerator = static_cast<int>(random() % 20001) - 10000;
        const int exponent = static_cast<int>(random() % 25) - 12;
        value = std::ldexp(T(numerator) / T(8192), exponent);
    }
}

template<class T>
void compare_statistics(Matrix<T>& a, Matrix<T>& b, Matrix<T>& weights) {
    const auto s = a.view.rows, p = a.view.cols, q = b.view.cols;
    Matrix<T> mean(p, 1), reference_mean(p, 1), covariance(p, q), reference_covariance(p, q);
    Scratch<T> mean_ws(FILTERS_UKF_MEAN_STORAGE(s, p), FILTERS_UKF_MEAN_BASIC_BYTES(s, p));
    Scratch<T> reference_mean_ws(FILTERS_UKF_MEAN_STORAGE(s, p), FILTERS_UKF_MEAN_BASIC_BYTES(s, p));
    Scratch<T> cov_ws(FILTERS_UKF_WEIGHTED_COVARIANCE_STORAGE(s, p, q),
                       FILTERS_UKF_WEIGHTED_COVARIANCE_BASIC_BYTES(s, p, q));
    Scratch<T> reference_cov_ws(FILTERS_UKF_WEIGHTED_COVARIANCE_STORAGE(s, p, q),
                                 FILTERS_UKF_WEIGHTED_COVARIANCE_BASIC_BYTES(s, p, q));
    EXPECT_EQ(Api<T>::mean(&a.view, &weights.view, nullptr, nullptr, &mean.view, mean_ws.view()),
              old_mean<T>(a.view, weights.view, nullptr, nullptr, reference_mean.view, reference_mean_ws.view()));
    identical(mean, reference_mean);
    EXPECT_EQ(Api<T>::covariance(&a.view, &b.view, &weights.view, &covariance.view, cov_ws.view()),
              old_covariance<T>(a.view, b.view, weights.view, reference_covariance.view, reference_cov_ws.view()));
    identical(covariance, reference_covariance);
    mean_ws.guards(); reference_mean_ws.guards(); cov_ws.guards(); reference_cov_ws.guards();
}

template<class T> class FloatWorkEquivalence : public ::testing::Test {};
using FloatTypes = ::testing::Types<float, double>;
TYPED_TEST_SUITE(FloatWorkEquivalence, FloatTypes);

TYPED_TEST(FloatWorkEquivalence, RandomRectangularStatisticsAreBitwiseIdentical) {
    using T = TypeParam;
    std::mt19937 random(0x674bee1);
    for (std::size_t s : {1u, 2u, 5u, 9u, 32u}) {
        for (std::size_t p : {1u, 2u, 4u, 7u}) {
            for (std::size_t q : {1u, 3u, 8u}) {
                SCOPED_TRACE(::testing::Message() << "s=" << s << " p=" << p << " q=" << q);
                Matrix<T> a(s, p), b(s, q), weights(s, 1);
                fill(a, random); fill(b, random); fill(weights, random);
                compare_statistics(a, b, weights);
            }
        }
    }
}

TYPED_TEST(FloatWorkEquivalence, SignedZeroSubnormalAndOverflowStatisticsMatch) {
    using T = TypeParam;
    const T values[] = {T(0), -T(0), std::numeric_limits<T>::denorm_min(),
                        -std::numeric_limits<T>::denorm_min(), std::numeric_limits<T>::min(),
                        -std::numeric_limits<T>::min(), T(1), -T(1), T(4096), T(-8192)};
    Matrix<T> a(10, 3), b(10, 4), weights(10, 1);
    for (std::size_t i = 0; i < a.data.size(); ++i) a.data[i] = values[i % 10];
    for (std::size_t i = 0; i < b.data.size(); ++i) b.data[i] = values[(i + 3) % 10];
    for (std::size_t i = 0; i < weights.data.size(); ++i) weights.data[i] = values[(i + 7) % 10];
    compare_statistics(a, b, weights);
    a.data[4] = std::numeric_limits<T>::max();
    weights.data[1] = T(2);
    compare_statistics(a, b, weights);
    a.data[4] = T(1);
    b.data[6] = std::numeric_limits<T>::max();
    compare_statistics(a, b, weights);
    /* Overflow in a dot product after earlier output elements fit is atomic. */
    std::fill(weights.data.begin(), weights.data.end(), T(1));
    std::fill(b.data.begin(), b.data.end(), T(1));
    std::fill(a.data.begin(), a.data.end(), T(1));
    a.data[2] = a.data[5] = std::numeric_limits<T>::max();
    compare_statistics(a, b, weights);
}

TYPED_TEST(FloatWorkEquivalence, ResidualOrderAndFailureOutputMatch) {
    using T = TypeParam;
    std::mt19937 random(0x6137);
    Matrix<T> points(9, 4), weights(9, 1);
    fill(points, random); fill(weights, random);
    for (int mode = 0; mode < 3; ++mode) {
        Matrix<T> result(4, 1), reference(4, 1);
        Scratch<T> ws(FILTERS_UKF_MEAN_STORAGE(9, 4), FILTERS_UKF_MEAN_BASIC_BYTES(9, 4));
        Scratch<T> reference_ws(FILTERS_UKF_MEAN_STORAGE(9, 4), FILTERS_UKF_MEAN_BASIC_BYTES(9, 4));
        Script<T> script, reference_script;
        if (mode == 1) script.fail_at = reference_script.fail_at = 3;
        if (mode == 2) script.nonfinite_at = reference_script.nonfinite_at = 2;
        EXPECT_EQ(Api<T>::mean(&points.view, &weights.view, residual<T>, &script, &result.view, ws.view()),
                  old_mean<T>(points.view, weights.view, residual<T>, &reference_script,
                               reference.view, reference_ws.view()));
        EXPECT_EQ(script.calls, reference_script.calls);
        identical(result, reference);
        ws.guards(); reference_ws.guards();
    }
    /* The legacy residual path commits its dot before the final add can fail. */
    std::fill(points.data.begin(), points.data.end(), T(0));
    std::fill(weights.data.begin(), weights.data.end(), T(0));
    points.data[0] = std::numeric_limits<T>::max();
    points.data[4] = T(0);
    weights.data[1] = T(-1);
    Matrix<T> result(4, 1), reference(4, 1);
    Scratch<T> ws(FILTERS_UKF_MEAN_STORAGE(9, 4), FILTERS_UKF_MEAN_BASIC_BYTES(9, 4));
    Scratch<T> reference_ws(FILTERS_UKF_MEAN_STORAGE(9, 4), FILTERS_UKF_MEAN_BASIC_BYTES(9, 4));
    Script<T> script, reference_script;
    EXPECT_EQ(Api<T>::mean(&points.view, &weights.view, residual<T>, &script, &result.view, ws.view()), LINEARS_ERROR_RANGE);
    EXPECT_EQ(old_mean<T>(points.view, weights.view, residual<T>, &reference_script,
                          reference.view, reference_ws.view()), LINEARS_ERROR_RANGE);
    identical(result, reference);
    EXPECT_EQ(result.data[0], std::numeric_limits<T>::max());
}

TYPED_TEST(FloatWorkEquivalence, DensePivotedGaussianAndPartialFailureMatch) {
    using T = TypeParam;
    std::mt19937 random(0x6511);
    for (std::size_t k : {1u, 2u, 4u, 7u}) {
        Matrix<T> deviations(17, k), factor(k, k);
        fill(deviations, random);
        for (std::size_t i = 0; i < k; ++i)
            for (std::size_t j = 0; j < k; ++j)
                factor.data[i * k + j] = i == j ? T(k + 1) : T(static_cast<int>((i + 2 * j) % 5) - 2) / T(8);
        if (k > 1)
            for (std::size_t j = 0; j < k; ++j) std::swap(factor.data[j], factor.data[(k - 1) * k + j]);
        for (int mode = 0; mode < 3; ++mode) {
            Matrix<T> result(17, 1), reference(17, 1);
            Scratch<T> ws(FILTERS_PF_GAUSSIAN_EXPONENT_STORAGE(k), FILTERS_PF_GAUSSIAN_EXPONENT_BASIC_BYTES,
                            FILTERS_PF_GAUSSIAN_EXPONENT_DECOMP_STORAGE(k));
            Scratch<T> reference_ws(FILTERS_PF_GAUSSIAN_EXPONENT_STORAGE(k), FILTERS_PF_GAUSSIAN_EXPONENT_BASIC_BYTES,
                                      FILTERS_PF_GAUSSIAN_EXPONENT_DECOMP_STORAGE(k));
            if (mode == 1) deviations.data[3 * k] = std::numeric_limits<T>::max();
            if (mode == 2) std::fill(factor.data.begin(), factor.data.end(), T(0));
            const auto expected = old_gaussian<T>(deviations.view, factor.view, reference.view, reference_ws.view());
            EXPECT_EQ(Api<T>::gaussian(&deviations.view, &factor.view, &result.view, ws.view()), expected);
            EXPECT_EQ(expected, mode == 0 ? LINEARS_OK : mode == 1 ? LINEARS_ERROR_RANGE : LINEARS_ERROR_SINGULAR);
            identical(result, reference);
            ws.guards(); reference_ws.guards();
        }
    }
}

TYPED_TEST(FloatWorkEquivalence, DensePerturbInPlaceAndRowAtomicityMatch) {
    using T = TypeParam;
    std::mt19937 random(0x6512);
    for (bool in_place : {false, true}) {
        for (int mode = 0; mode < 5; ++mode) {
            Matrix<T> points(8, 4), reference_points(8, 4), factor(4, 4), result(8, 4), reference(8, 4);
            fill(points, random); fill(factor, random);
            Script<T> script, reference_script;
            script.values = {T(0.125), T(-1.25), -T(0), T(2.5), T(0.25), T(1), T(2), T(-4)};
            if (mode == 1) script.fail_at = 3;
            if (mode == 2) script.nonfinite_at = 2;
            if (mode == 3) {
                script.values[6] = std::numeric_limits<T>::max();
                factor.data[2] = T(2);
            }
            if (mode == 4) {
                std::fill(factor.data.begin(), factor.data.end(), T(0));
                factor.data[0] = std::numeric_limits<T>::max();
                points.data[4] = std::numeric_limits<T>::max();
            }
            reference_script = script;
            std::copy(points.data.begin(), points.data.end(), reference_points.data.begin());
            auto& actual_out = in_place ? points : result;
            auto& reference_out = in_place ? reference_points : reference;
            Scratch<T> ws(FILTERS_PF_PERTURB_STORAGE(4), FILTERS_PF_PERTURB_BASIC_BYTES(4));
            Scratch<T> reference_ws(FILTERS_PF_PERTURB_STORAGE(4), FILTERS_PF_PERTURB_BASIC_BYTES(4));
            const auto expected = old_perturb<T>(reference_points.view, factor.view, draw<T>, &reference_script,
                                                 reference_out.view, reference_ws.view());
            EXPECT_EQ(Api<T>::perturb(&points.view, &factor.view, draw<T>, &script, &actual_out.view, ws.view()), expected);
            EXPECT_EQ(expected, mode == 0 ? LINEARS_OK : mode == 1 ? LINEARS_ERROR_NO_CONVERGENCE : LINEARS_ERROR_RANGE);
            EXPECT_EQ(script.calls, reference_script.calls);
            identical(actual_out, reference_out);
            ws.guards(); reference_ws.guards();
        }
    }
}

TYPED_TEST(FloatWorkEquivalence, SignedZeroAndSubnormalParticleKernelsMatch) {
    using T = TypeParam;
    const T values[] = {T(0), -T(0), std::numeric_limits<T>::denorm_min(),
                        -std::numeric_limits<T>::denorm_min(), std::numeric_limits<T>::min(),
                        -std::numeric_limits<T>::min()};
    Matrix<T> points(6, 3), factor(3, 3, -T(0)), gaussian(6, 1), reference_gaussian(6, 1);
    Matrix<T> perturbed(6, 3), reference_perturbed(6, 3);
    for (std::size_t i = 0; i < points.data.size(); ++i) points.data[i] = values[i % 6];
    for (std::size_t i = 0; i < 3; ++i) factor.data[i * 3 + i] = T(1);
    Scratch<T> gauss_ws(FILTERS_PF_GAUSSIAN_EXPONENT_STORAGE(3), FILTERS_PF_GAUSSIAN_EXPONENT_BASIC_BYTES,
                         FILTERS_PF_GAUSSIAN_EXPONENT_DECOMP_STORAGE(3));
    Scratch<T> reference_gauss_ws(FILTERS_PF_GAUSSIAN_EXPONENT_STORAGE(3), FILTERS_PF_GAUSSIAN_EXPONENT_BASIC_BYTES,
                                   FILTERS_PF_GAUSSIAN_EXPONENT_DECOMP_STORAGE(3));
    EXPECT_EQ(Api<T>::gaussian(&points.view, &factor.view, &gaussian.view, gauss_ws.view()), LINEARS_OK);
    EXPECT_EQ(old_gaussian<T>(points.view, factor.view, reference_gaussian.view, reference_gauss_ws.view()), LINEARS_OK);
    identical(gaussian, reference_gaussian);
    Script<T> script, reference_script;
    script.values.assign(values, values + 6);
    reference_script = script;
    Scratch<T> ws(FILTERS_PF_PERTURB_STORAGE(3), FILTERS_PF_PERTURB_BASIC_BYTES(3));
    Scratch<T> reference_ws(FILTERS_PF_PERTURB_STORAGE(3), FILTERS_PF_PERTURB_BASIC_BYTES(3));
    EXPECT_EQ(Api<T>::perturb(&points.view, &factor.view, draw<T>, &script, &perturbed.view, ws.view()), LINEARS_OK);
    EXPECT_EQ(old_perturb<T>(points.view, factor.view, draw<T>, &reference_script,
                              reference_perturbed.view, reference_ws.view()), LINEARS_OK);
    identical(perturbed, reference_perturbed);
    gauss_ws.guards(); reference_gauss_ws.guards(); ws.guards(); reference_ws.guards();
}

TYPED_TEST(FloatWorkEquivalence, PublicValidationAndRequiredCapacitiesRemain) {
    using T = TypeParam;
    Matrix<T> points(5, 2, T(0.25)), weights(5, 1, T(0.2)), mean(2, 1), covariance(2, 2);
    Scratch<T> mean_ws(FILTERS_UKF_MEAN_STORAGE(5, 2), FILTERS_UKF_MEAN_BASIC_BYTES(5, 2));
    Scratch<T> cov_ws(FILTERS_UKF_WEIGHTED_COVARIANCE_STORAGE(5, 2, 2),
                       FILTERS_UKF_WEIGHTED_COVARIANCE_BASIC_BYTES(5, 2, 2));
    auto short_mean = mean_ws.view(); short_mean.capacity = FILTERS_UKF_MEAN_STORAGE(5, 2) - 1;
    auto short_cov = cov_ws.view(); short_cov.capacity = FILTERS_UKF_WEIGHTED_COVARIANCE_STORAGE(5, 2, 2) - 1;
    EXPECT_EQ(Api<T>::mean(&points.view, &weights.view, nullptr, nullptr, &mean.view, short_mean), LINEARS_ERROR_WORKSPACE);
    EXPECT_EQ(Api<T>::covariance(&points.view, &points.view, &weights.view, &covariance.view, short_cov), LINEARS_ERROR_WORKSPACE);
    auto alias = mean.view; alias.data = points.view.data;
    EXPECT_EQ(Api<T>::mean(&points.view, &weights.view, nullptr, nullptr, &alias, mean_ws.view()), LINEARS_ERROR_ALIAS);
    for (T invalid : {std::numeric_limits<T>::quiet_NaN(), std::numeric_limits<T>::infinity()}) {
        points.data.back() = invalid;
        EXPECT_EQ(Api<T>::mean(&points.view, &weights.view, nullptr, nullptr, &mean.view, mean_ws.view()), LINEARS_ERROR_RANGE);
        EXPECT_EQ(Api<T>::covariance(&points.view, &points.view, &weights.view, &covariance.view, cov_ws.view()), LINEARS_ERROR_RANGE);
    }
    for (T value : mean.data) EXPECT_EQ(value, T(19.25));
    for (T value : covariance.data) EXPECT_EQ(value, T(19.25));
    mean_ws.guards(); cov_ws.guards();
}

} // namespace
