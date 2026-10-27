#include "../src/decomp_internal.h"
#define D_FORMAT 1
#include "../src/decomp_template.h"
#include "../src/decomp_cleanup.h"
#define D_FORMAT 2
#include "../src/decomp_template.h"
#include "../src/decomp_cleanup.h"

#include <gtest/gtest.h>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>

namespace {
struct F32 {
    using Real = float;
    using Bits = uint32_t;
    static constexpr unsigned fraction = 23, exponents = 256;
    static constexpr auto classify = linears_decomp_isfinite_f32;
    static constexpr auto finite = linears_decomp_finite_f32;
    static constexpr auto multiply = linears_decomp_mul_f32;
};
struct F64 {
    using Real = double;
    using Bits = uint64_t;
    static constexpr unsigned fraction = 52, exponents = 2048;
    static constexpr auto classify = linears_decomp_isfinite_f64;
    static constexpr auto finite = linears_decomp_finite_f64;
    static constexpr auto multiply = linears_decomp_mul_f64;
};

template<class F> class DecompositionFinite : public ::testing::Test {};
using Formats = ::testing::Types<F32, F64>;
TYPED_TEST_SUITE(DecompositionFinite, Formats);

TYPED_TEST(DecompositionFinite, EveryExponentAndSpecialValueMatchesNativeClassification) {
    using F = TypeParam;
    using R = typename F::Real;
    using B = typename F::Bits;
    if constexpr (!std::numeric_limits<R>::is_iec559 || sizeof(R) != sizeof(B)) {
        GTEST_SKIP() << "IEEE bit-pattern fixture requires a matching native representation";
    } else {
        constexpr B fraction_mask = (B(1) << F::fraction) - 1;
        constexpr B sign_bit = B(1) << (sizeof(B)*8-1);
        const std::array<B, 7> fractions{0, 1, 2, fraction_mask / 3,
                                       B(1) << (F::fraction-1), fraction_mask-1, fraction_mask};
        for (unsigned sign = 0; sign < 2; ++sign) {
            for (unsigned exponent = 0; exponent < F::exponents; ++exponent) {
                for (B fraction : fractions) {
                    const B bits = (sign ? sign_bit : 0) | (B(exponent) << F::fraction) | fraction;
                    R value;
                    std::memcpy(&value, &bits, sizeof(value));
                    const bool expected = std::isfinite(value);
                    ASSERT_EQ(F::classify(value) != 0, expected) << "bits=" << bits;
                    linears_status_t status = LINEARS_OK;
                    const R result = F::finite(value, &status);
                    EXPECT_EQ(status, expected ? LINEARS_OK : LINEARS_ERROR_RANGE) << "bits=" << bits;
                    B returned;
                    std::memcpy(&returned, &result, sizeof(returned));
                    EXPECT_EQ(returned, bits);
                    status = LINEARS_ERROR_DIMENSION;
                    (void)F::finite(value, &status);
                    EXPECT_EQ(status, LINEARS_ERROR_DIMENSION);
                }
            }
        }
    }
}

TYPED_TEST(DecompositionFinite, ArithmeticOverflowRetainsTheFirstFailure) {
    using F = TypeParam;
    using R = typename F::Real;
    linears_status_t status = LINEARS_OK;
    (void)F::multiply(std::numeric_limits<R>::max(), R(2), &status);
    EXPECT_EQ(status, LINEARS_ERROR_RANGE);
    EXPECT_EQ(F::multiply(R(1), R(1), &status), R(0));
    EXPECT_EQ(status, LINEARS_ERROR_RANGE);
    status = LINEARS_ERROR_DIMENSION;
    EXPECT_EQ(F::multiply(std::numeric_limits<R>::max(), R(2), &status), R(0));
    EXPECT_EQ(status, LINEARS_ERROR_DIMENSION);
}
} // namespace
