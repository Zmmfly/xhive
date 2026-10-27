#include "../src/ukf_internal.h"
#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <iomanip>
#include <limits>
#include <random>
#include <type_traits>

namespace {

/* References keep the previous native scaling/lround boundary, independently
 * of the production significand extraction and exponent construction. */
int16_t reference_q15(float value) {
    const float scaled = value * 32768.0f;
    return scaled >= 32767.5f ? INT16_MAX : static_cast<int16_t>(std::lround(scaled));
}

int32_t reference_q31(double value) {
    const double scaled = value * 2147483648.0;
    return scaled >= 2147483647.5 ? INT32_MAX : static_cast<int32_t>(std::lround(scaled));
}

void check_q15(float value) {
    if (!(value >= -1 && value <= 1)) return;
    int16_t output[3] = {1234, 2345, 3456};
    ukf_store_q15(&value, 1, output + 1);
    EXPECT_EQ(output[1], reference_q15(value)) << std::hexfloat << value;
    EXPECT_EQ(output[0], 1234);
    EXPECT_EQ(output[2], 3456);
}

void check_q31(double value) {
    if (!(value >= -1 && value <= 1)) return;
    int32_t output[3] = {1234, 2345, 3456};
    ukf_store_q31(&value, 1, output + 1);
    EXPECT_EQ(output[1], reference_q31(value)) << std::hexfloat << value;
    EXPECT_EQ(output[0], 1234);
    EXPECT_EQ(output[2], 3456);
}

void check_widen_q31(int32_t raw) {
    const double expected = static_cast<double>(raw) * 0x1p-31;
    double actual = 123;
    ukf_widen_q31(&raw, 1, &actual);
    EXPECT_EQ(std::memcmp(&actual, &expected, sizeof(actual)), 0) << raw;
}

TEST(QBoundaries, FiniteScalarAndMatrixCoverAllFloatingClasses) {
    const float singles[] = {0.0f, -0.0f, std::numeric_limits<float>::denorm_min(),
        -std::numeric_limits<float>::denorm_min(), std::numeric_limits<float>::min(),
        std::numeric_limits<float>::max(), std::numeric_limits<float>::lowest(),
        std::numeric_limits<float>::infinity(), -std::numeric_limits<float>::infinity(),
        std::numeric_limits<float>::quiet_NaN(), std::numeric_limits<float>::signaling_NaN()};
    const double doubles[] = {0.0, -0.0, std::numeric_limits<double>::denorm_min(),
        -std::numeric_limits<double>::denorm_min(), std::numeric_limits<double>::min(),
        std::numeric_limits<double>::max(), std::numeric_limits<double>::lowest(),
        std::numeric_limits<double>::infinity(), -std::numeric_limits<double>::infinity(),
        std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::signaling_NaN()};
    for (float value : singles) {
        const bool finite = std::isfinite(value);
        EXPECT_EQ(!!ukf_isfinite_f32(value), finite);
        EXPECT_EQ(!!ukf_in_unit_f32(value), value >= -1.0f && value <= 1.0f);
        float values[] = {0, -0.0f, 1, value};
        const linears_matrix_t view{values, 2, 2};
        EXPECT_EQ(ukf_finite_f32(&view), finite ? LINEARS_OK : LINEARS_ERROR_RANGE);
    }
    for (double value : doubles) {
        const bool finite = std::isfinite(value);
        EXPECT_EQ(!!ukf_isfinite_f64(value), finite);
        EXPECT_EQ(!!ukf_in_unit_f64(value), value >= -1.0 && value <= 1.0);
        double values[] = {0, -0.0, 1, value};
        const linears_matrix_t view{values, 2, 2};
        EXPECT_EQ(ukf_finite_f64(&view), finite ? LINEARS_OK : LINEARS_ERROR_RANGE);
    }
}

TEST(QBoundaries, EveryIeeeExponentAndNanPayloadClassesStayClassified) {
    if constexpr (sizeof(float) == sizeof(uint32_t) && std::numeric_limits<float>::is_iec559) {
        for (uint32_t sign : {0u, UINT32_C(0x80000000)}) {
            for (uint32_t exponent = 0; exponent < 256; ++exponent) {
                for (uint32_t fraction : {0u, 1u, UINT32_C(0x3fffff), UINT32_C(0x400000), UINT32_C(0x7fffff)}) {
                    const uint32_t bits = sign | (exponent << 23) | fraction;
                    float value;
                    std::memcpy(&value, &bits, sizeof(value));
                    EXPECT_EQ(!!ukf_isfinite_f32(value), !!std::isfinite(value)) << bits;
                    EXPECT_EQ(!!ukf_in_unit_f32(value), value >= -1.0f && value <= 1.0f) << bits;
                }
            }
        }
    }
    if constexpr (sizeof(double) == sizeof(uint64_t) && std::numeric_limits<double>::is_iec559) {
        for (uint64_t sign : {UINT64_C(0), UINT64_C(0x8000000000000000)}) {
            for (uint64_t exponent = 0; exponent < 2048; ++exponent) {
                for (uint64_t fraction : {UINT64_C(0), UINT64_C(1), UINT64_C(0x7ffffffffffff),
                                           UINT64_C(0x8000000000000), UINT64_C(0xfffffffffffff)}) {
                    const uint64_t bits = sign | (exponent << 52) | fraction;
                    double value;
                    std::memcpy(&value, &bits, sizeof(value));
                    EXPECT_EQ(!!ukf_isfinite_f64(value), !!std::isfinite(value)) << bits;
                    EXPECT_EQ(!!ukf_in_unit_f64(value), value >= -1.0 && value <= 1.0) << bits;
                }
            }
        }
    }
}

TEST(QBoundaries, Q15EveryRawAndHalfCellNeighborMatchesNativeBoundary) {
    for (int32_t raw = INT16_MIN; raw <= INT16_MAX; ++raw) {
        const int16_t stored = static_cast<int16_t>(raw);
        const float exact = static_cast<float>(stored) * 0x1p-15f;
        float widened = 123;
        ukf_widen_q15(&stored, 1, &widened);
        ASSERT_EQ(std::memcmp(&widened, &exact, sizeof(exact)), 0) << raw;
        check_q15(exact);
        const float half = (static_cast<float>(raw) + 0.5f) * 0x1p-15f;
        check_q15(half);
        check_q15(std::nextafter(half, std::numeric_limits<float>::infinity()));
        check_q15(std::nextafter(half, -std::numeric_limits<float>::infinity()));
    }
    for (float value : {-1.0f, 1.0f, -0.0f, 0.0f, std::numeric_limits<float>::denorm_min(),
                         -std::numeric_limits<float>::denorm_min()}) check_q15(value);
}

TEST(QBoundaries, Q31SignedPowersHalfCellNeighborsAndRandomValuesMatch) {
    for (int bit = 0; bit <= 31; ++bit) {
        const int64_t power = INT64_C(1) << bit;
        for (int sign : {-1, 1}) {
            for (int offset : {-1, 0, 1}) {
                const int64_t candidate = sign * power + offset;
                if (candidate < INT32_MIN || candidate > INT32_MAX) continue;
                const int32_t raw = static_cast<int32_t>(candidate);
                check_widen_q31(raw);
                check_q31(static_cast<double>(raw) * 0x1p-31);
                const double half = (static_cast<double>(raw) + 0.5) * 0x1p-31;
                check_q31(half);
                check_q31(std::nextafter(half, std::numeric_limits<double>::infinity()));
                check_q31(std::nextafter(half, -std::numeric_limits<double>::infinity()));
            }
        }
    }
    std::mt19937 random(0x51424f55u);
    for (std::size_t i = 0; i < 65536; ++i) {
        const uint32_t bits = random();
        int32_t raw;
        std::memcpy(&raw, &bits, sizeof(raw));
        check_widen_q31(raw);
        check_q31(static_cast<double>(raw) * 0x1p-31);
        const double half = (static_cast<double>(raw) + 0.5) * 0x1p-31;
        check_q31(half);
        check_q31(std::nextafter(half, std::numeric_limits<double>::infinity()));
        check_q31(std::nextafter(half, -std::numeric_limits<double>::infinity()));
    }
    for (double value : {-1.0, 1.0, -0.0, 0.0, std::numeric_limits<double>::denorm_min(),
                          -std::numeric_limits<double>::denorm_min(), -0x1p-32, 0x1p-32}) {
        check_q31(value);
        check_q31(std::nextafter(value, std::numeric_limits<double>::infinity()));
        check_q31(std::nextafter(value, -std::numeric_limits<double>::infinity()));
    }
}

TEST(QBoundaries, RandomIeeeValuesKeepConversionAndFinitenessResults) {
    std::mt19937_64 random(0x514945454555u);
    for (std::size_t i = 0; i < 65536; ++i) {
        if constexpr (sizeof(float) == sizeof(uint32_t) && std::numeric_limits<float>::is_iec559) {
            const uint32_t bits = static_cast<uint32_t>(random());
            float value;
            std::memcpy(&value, &bits, sizeof(value));
            EXPECT_EQ(!!ukf_isfinite_f32(value), !!std::isfinite(value));
            EXPECT_EQ(!!ukf_in_unit_f32(value), value >= -1.0f && value <= 1.0f);
            check_q15(value);
        }
        if constexpr (sizeof(double) == sizeof(uint64_t) && std::numeric_limits<double>::is_iec559) {
            const uint64_t bits = random();
            double value;
            std::memcpy(&value, &bits, sizeof(value));
            EXPECT_EQ(!!ukf_isfinite_f64(value), !!std::isfinite(value));
            EXPECT_EQ(!!ukf_in_unit_f64(value), value >= -1.0 && value <= 1.0);
            check_q31(value);
        }
    }
}

TEST(QBoundaries, RangeChecksRejectLastElementAndKeepUnitEndpoints) {
    const float singles[] = {std::nextafter(-1.0f, -INFINITY), std::nextafter(1.0f, INFINITY),
        INFINITY, -INFINITY, std::numeric_limits<float>::quiet_NaN()};
    const double doubles[] = {std::nextafter(-1.0, -INFINITY), std::nextafter(1.0, INFINITY),
        INFINITY, -INFINITY, std::numeric_limits<double>::quiet_NaN()};
    for (float bad : singles) {
        float values[] = {-1, 1, 0, bad};
        EXPECT_EQ(ukf_fits_q15(values, 4), LINEARS_ERROR_RANGE);
    }
    for (double bad : doubles) {
        double values[] = {-1, 1, 0, bad};
        EXPECT_EQ(ukf_fits_q31(values, 4), LINEARS_ERROR_RANGE);
    }
    const float singles_ok[] = {-1, 1, -0.0f, 0};
    const double doubles_ok[] = {-1, 1, -0.0, 0};
    int16_t q15[4];
    int32_t q31[4];
    ASSERT_EQ(ukf_fits_q15(singles_ok, 4), LINEARS_OK);
    ASSERT_EQ(ukf_fits_q31(doubles_ok, 4), LINEARS_OK);
    ukf_store_q15(singles_ok, 4, q15);
    ukf_store_q31(doubles_ok, 4, q31);
    EXPECT_EQ((std::array<int16_t, 4>{q15[0], q15[1], q15[2], q15[3]}),
              (std::array<int16_t, 4>{INT16_MIN, INT16_MAX, 0, 0}));
    EXPECT_EQ((std::array<int32_t, 4>{q31[0], q31[1], q31[2], q31[3]}),
              (std::array<int32_t, 4>{INT32_MIN, INT32_MAX, 0, 0}));
}

TEST(QBoundaries, UnitIntervalIncludesEndpointsButNotTheirOutwardNeighbors) {
    for (float value : {-1.0f, 1.0f, -0.0f, 0.0f, std::nextafter(-1.0f, 0.0f),
                         std::nextafter(1.0f, 0.0f)}) EXPECT_TRUE(ukf_in_unit_f32(value));
    for (double value : {-1.0, 1.0, -0.0, 0.0, std::nextafter(-1.0, 0.0),
                          std::nextafter(1.0, 0.0)}) EXPECT_TRUE(ukf_in_unit_f64(value));
    EXPECT_FALSE(ukf_in_unit_f32(std::nextafter(-1.0f, -INFINITY)));
    EXPECT_FALSE(ukf_in_unit_f32(std::nextafter(1.0f, INFINITY)));
    EXPECT_FALSE(ukf_in_unit_f64(std::nextafter(-1.0, -INFINITY)));
    EXPECT_FALSE(ukf_in_unit_f64(std::nextafter(1.0, INFINITY)));
}

template<class W> struct PointScript {
    std::array<W, 3> points;
    std::size_t next = 0;
};

template<class W>
linears_status_t scripted_point(const linears_matrix_t *, linears_matrix_t *out, void *context) {
    auto& script = *static_cast<PointScript<W>*>(context);
    static_cast<W*>(out->data)[0] = script.points[script.next++];
    return LINEARS_OK;
}

template<class W>
void check_mean_error_cells() {
    constexpr bool single = std::is_same_v<W, float>;
    using Raw = std::conditional_t<single, int16_t, int32_t>;
    using Workspace = std::conditional_t<single, filters_ukf_q15_workspace_t, filters_ukf_q31_workspace_t>;
    using Weights = std::conditional_t<single, filters_ukf_f32_weights_t, filters_ukf_f64_weights_t>;
    constexpr W lsb = single ? static_cast<W>(0x1p-15) : static_cast<W>(0x1p-31);
    W work[FILTERS_UKF_Q_PREDICT_WORK_STORAGE(1)];
    W decomp[FILTERS_UKF_Q_PREDICT_DECOMP_STORAGE(1)];
    Raw signals[FILTERS_UKF_Q_PREDICT_SIGNAL_STORAGE(1)];
    Workspace workspace{work, FILTERS_UKF_Q_PREDICT_WORK_STORAGE(1), nullptr, 0,
                        {decomp, FILTERS_UKF_Q_PREDICT_DECOMP_STORAGE(1)},
                        signals, FILTERS_UKF_Q_PREDICT_SIGNAL_STORAGE(1)};
    Weights weights;
    if constexpr (single) ASSERT_EQ(filters_ukf_f32_weights(1, 1, 2, 0, &weights), LINEARS_OK);
    else ASSERT_EQ(filters_ukf_f64_weights(1, 1, 2, 0, &weights), LINEARS_OK);
    const std::array<W, 3> cases[] = {
        {0, 1, static_cast<W>(0.25) + lsb * static_cast<W>(0.25)},
        {0, 1 - lsb, static_cast<W>(0.25) + lsb * static_cast<W>(0.25)},
        {0, 1, 0},
        {0, static_cast<W>(0.5) + lsb * static_cast<W>(0.5),
            static_cast<W>(0.25) + lsb * static_cast<W>(0.5)}
    };
    for (std::size_t i = 0; i < 4; ++i) {
        SCOPED_TRACE(i);
        W x = 0, p = static_cast<W>(0.25), q = 0, mean, covariance;
        linears_matrix_t xv{&x, 1, 1}, pv{&p, 1, 1}, qv{&q, 1, 1};
        linears_matrix_t mean_v{&mean, 1, 1}, covariance_v{&covariance, 1, 1};
        PointScript<W> script{cases[i]};
        if constexpr (single) {
            ASSERT_EQ(filters_ukf_q15_predict_work(&xv, &pv, &qv, &weights, scripted_point<W>, &script,
                nullptr, nullptr, &mean_v, &covariance_v, workspace), LINEARS_OK);
        } else {
            ASSERT_EQ(filters_ukf_q31_predict_work(&xv, &pv, &qv, &weights, scripted_point<W>, &script,
                nullptr, nullptr, &mean_v, &covariance_v, workspace), LINEARS_OK);
        }
        W expected;
        if (i == 0) {
            /* +1 contributes one full LSB of quantization error: the other
             * point pushes weighted error beyond half an LSB, requiring FP. */
            expected = static_cast<W>(0.5) * cases[i][1] + static_cast<W>(0.5) * cases[i][2];
        } else {
            /* MAX by itself must not force FP; both its ordinary cell and
             * +1 with exactly half an LSB of total weighted error still use Q. */
            int64_t sum;
            if constexpr (single) sum = static_cast<int64_t>(reference_q15(cases[i][1])) + reference_q15(cases[i][2]);
            else sum = static_cast<int64_t>(reference_q31(cases[i][1])) + reference_q31(cases[i][2]);
            expected = static_cast<W>(0.5) * (static_cast<W>(sum) * lsb);
        }
        EXPECT_EQ(std::memcmp(&mean, &expected, sizeof(mean)), 0);
        EXPECT_EQ(script.next, 3u);
    }
}

TEST(QBoundaries, MeanBoundKeepsMaximumCellExceptionsAndThresholdDecision) {
    check_mean_error_cells<float>();
    check_mean_error_cells<double>();
}

} // namespace
