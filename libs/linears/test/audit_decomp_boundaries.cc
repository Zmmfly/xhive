#include "linears_decomp.h"

#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <cstdint>
#include <limits>

namespace {

// Keep these adapters and mathematical expectations independent of
// decomp_support.h and the production scalar normalization/export helpers.
#define AUDIT_DECOMP_FORMAT(Name, Suffix, RawType, WorkType, Bits) \
    struct Name { \
        using Raw = RawType; \
        using Work = WorkType; \
        using Workspace = linears_##Suffix##_workspace_t; \
        static constexpr int bits = Bits; \
        static linears_status_t svd(const linears_matrix_t *a, linears_matrix_t *u, \
            linears_matrix_t *s, linears_matrix_t *v, size_t budget, Workspace work) { \
            return linears_svd_##Suffix(a, u, s, v, budget, work); \
        } \
        static linears_status_t ldlt(const linears_matrix_t *a, linears_matrix_t *l, \
            linears_matrix_t *d, size_t *permutation, size_t permutation_capacity, \
            uint8_t *blocks, size_t blocks_capacity, Workspace work) { \
            return linears_ldlt_##Suffix(a, l, d, permutation, permutation_capacity, \
                                        blocks, blocks_capacity, work); \
        } \
    };
AUDIT_DECOMP_FORMAT(AuditF32, f32, float, float, 0)
AUDIT_DECOMP_FORMAT(AuditF64, f64, double, double, 0)
AUDIT_DECOMP_FORMAT(AuditQ15, q15, int16_t, linears_q31_t, 15)
AUDIT_DECOMP_FORMAT(AuditQ31, q31, int32_t, linears_q31_t, 31)
#undef AUDIT_DECOMP_FORMAT

using AuditFormats = ::testing::Types<AuditF32, AuditF64, AuditQ15, AuditQ31>;

template <class Format>
typename Format::Raw Encode(long double value, int exponent) {
    if constexpr (Format::bits == 0)
        return static_cast<typename Format::Raw>(value);
    else
        return static_cast<typename Format::Raw>(std::round(std::ldexp(value, Format::bits - exponent)));
}

template <class Format>
long double Decode(typename Format::Raw value, int exponent) {
    if constexpr (Format::bits == 0)
        return static_cast<long double>(value);
    else
        return std::ldexp(static_cast<long double>(value), exponent - Format::bits);
}

template <class Format>
long double WorkingEpsilon() {
    if constexpr (Format::bits == 0)
        return std::numeric_limits<typename Format::Raw>::epsilon();
    else
        return 0x1p-30L;
}

template <class Format>
long double ExportError(int exponent) {
    if constexpr (Format::bits == 0)
        return 0.0L;
    else
        return std::ldexp(0.5L, exponent - Format::bits);
}

template <class Format, size_t Count>
struct GuardedOutput {
    using Raw = typename Format::Raw;
    std::array<Raw, Count + 2> bytes{};
    linears_matrix_t matrix;
    GuardedOutput(size_t rows, size_t cols)
        : matrix{bytes.data() + 1, rows, cols, 73} {
        bytes.fill(static_cast<Raw>(37));
    }
    void CheckUntouched() const {
        for (const auto value : bytes) EXPECT_EQ(value, static_cast<Raw>(37));
        EXPECT_EQ(matrix.exponent, 73);
        EXPECT_EQ(matrix.data, bytes.data() + 1);
    }
    void CheckGuards() const {
        EXPECT_EQ(bytes.front(), static_cast<Raw>(37));
        EXPECT_EQ(bytes.back(), static_cast<Raw>(37));
        EXPECT_EQ(matrix.data, bytes.data() + 1);
    }
    long double At(size_t row, size_t col) const {
        return Decode<Format>(bytes[1 + row * matrix.cols + col], matrix.exponent);
    }
};

template <class Format, size_t Count>
struct GuardedWorkspace {
    using Work = typename Format::Work;
    std::array<Work, Count + 2> data{};
    typename Format::Workspace workspace{data.data() + 1, Count};
    GuardedWorkspace() {
        if constexpr (Format::bits == 0) data.fill(static_cast<Work>(19));
        else data.fill(Work{INT32_C(1431655765), 13});
    }
    void CheckGuards() const {
        if constexpr (Format::bits == 0) {
            EXPECT_EQ(data.front(), static_cast<Work>(19));
            EXPECT_EQ(data.back(), static_cast<Work>(19));
        } else {
            EXPECT_EQ(data.front().mantissa, INT32_C(1431655765));
            EXPECT_EQ(data.front().exponent, 13);
            EXPECT_EQ(data.back().mantissa, INT32_C(1431655765));
            EXPECT_EQ(data.back().exponent, 13);
        }
    }
};

template <class Format> class AuditDecompBoundary : public ::testing::Test {};
TYPED_TEST_SUITE(AuditDecompBoundary, AuditFormats);

TYPED_TEST(AuditDecompBoundary, NegativeLdltPeakKeepsTheFinestRepresentableBlock) {
    using F = TypeParam;
    using Raw = typename F::Raw;
    // Exact LDLT: D00=32767/32768, L10=128/32767,
    // D11=(-32767-16384/32767)/32768. Only its negative peak rounds to
    // -32768 at q15 exponent zero; no positive element needs a carry.
    const std::array<long double, 4> exact_input{
        32767.0L / 32768.0L, 128.0L / 32768.0L,
        128.0L / 32768.0L, -32767.0L / 32768.0L};
    std::array<Raw, 4> input{};
    for (size_t i = 0; i < input.size(); ++i) input[i] = Encode<F>(exact_input[i], 0);
    const auto original = input;
    const linears_matrix_t a{input.data(), 2, 2, 0};
    GuardedOutput<F, 4> l(2, 2), d(2, 2);
    GuardedWorkspace<F, LINEARS_LDLT_WORKSPACE(2)> work;
    std::array<size_t, 2> permutation{91, 92};
    std::array<uint8_t, 2> blocks{81, 82};
    ASSERT_EQ(F::ldlt(&a, &l.matrix, &d.matrix, permutation.data(), 2,
                      blocks.data(), 2, work.workspace), LINEARS_OK);
    EXPECT_EQ(input, original);
    EXPECT_EQ(permutation, (std::array<size_t, 2>{0, 1}));
    EXPECT_EQ(blocks, (std::array<uint8_t, 2>{1, 1}));
    EXPECT_EQ(d.matrix.exponent, 0);
    const long double exact_d11 = (-32767.0L - 16384.0L / 32767.0L) / 32768.0L;
    const long double tolerance = 8.0L * WorkingEpsilon<F>() + ExportError<F>(d.matrix.exponent);
    EXPECT_NEAR(d.At(0, 0), exact_input[0], tolerance);
    EXPECT_NEAR(d.At(1, 1), exact_d11, tolerance);
    EXPECT_EQ(d.At(0, 1), 0.0L);
    EXPECT_EQ(d.At(1, 0), 0.0L);
    EXPECT_EQ(l.At(0, 0), 1.0L);
    EXPECT_EQ(l.At(1, 1), 1.0L);
    EXPECT_EQ(l.At(0, 1), 0.0L);
    EXPECT_NEAR(l.At(1, 0), 128.0L / 32767.0L,
                8.0L * WorkingEpsilon<F>() + ExportError<F>(l.matrix.exponent));
    if constexpr (F::bits == 15) {
        EXPECT_EQ(d.bytes, (std::array<Raw, 6>{37, 32767, 0, 0, INT16_MIN, 37}));
    }
    l.CheckGuards(); d.CheckGuards(); work.CheckGuards();
}

TEST(AuditDecompRegression, Q15NegativeLdltPeakAtExponentLimitMustNotFailRange) {
    std::array<int16_t, 4> input{32767, 128, 128, -32767};
    const linears_matrix_t a{input.data(), 2, 2, LINEARS_EXPONENT_MAX};
    GuardedOutput<AuditQ15, 4> l(2, 2), d(2, 2);
    GuardedWorkspace<AuditQ15, LINEARS_LDLT_WORKSPACE(2)> work;
    std::array<size_t, 2> permutation{91, 92};
    std::array<uint8_t, 2> blocks{81, 82};
    ASSERT_EQ(AuditQ15::ldlt(&a, &l.matrix, &d.matrix, permutation.data(), 2,
                            blocks.data(), 2, work.workspace), LINEARS_OK);
    EXPECT_EQ(l.matrix.exponent, 1);
    EXPECT_EQ(d.matrix.exponent, LINEARS_EXPONENT_MAX);
    EXPECT_EQ(l.bytes, (std::array<int16_t, 6>{37, 16384, 0, 64, 16384, 37}));
    EXPECT_EQ(d.bytes, (std::array<int16_t, 6>{37, 32767, 0, 0, INT16_MIN, 37}));
    EXPECT_EQ(permutation, (std::array<size_t, 2>{0, 1}));
    EXPECT_EQ(blocks, (std::array<uint8_t, 2>{1, 1}));
    work.CheckGuards();
}

TYPED_TEST(AuditDecompBoundary, OneDimensionalSvdHandlesSmallestAndLargestScales) {
    using F = TypeParam;
    using Raw = typename F::Raw;
    std::array<Raw, 2> magnitudes{};
    std::array<int, 2> exponents{};
    if constexpr (F::bits == 0) {
        magnitudes = {std::numeric_limits<Raw>::denorm_min(), std::numeric_limits<Raw>::max()};
    } else {
        magnitudes = {static_cast<Raw>(INT64_C(1) << (F::bits - 1)), std::numeric_limits<Raw>::max()};
        exponents = {LINEARS_EXPONENT_MIN, LINEARS_EXPONENT_MAX};
    }
    for (size_t sample = 0; sample < magnitudes.size(); ++sample) {
        for (const int sign : {-1, 1}) {
            Raw input = static_cast<Raw>(sign * magnitudes[sample]);
            const linears_matrix_t a{&input, 1, 1, exponents[sample]};
            GuardedOutput<F, 1> u(1, 1), s(1, 1), v(1, 1);
            GuardedWorkspace<F, LINEARS_SVD_WORKSPACE(1, 1)> work;
            ASSERT_EQ(F::svd(&a, &u.matrix, &s.matrix, &v.matrix, 0, work.workspace), LINEARS_OK);
            // No norm squaring or absolute-value widening is needed for 1x1.
            EXPECT_EQ(s.bytes[1], magnitudes[sample]);
            EXPECT_EQ(s.matrix.exponent, exponents[sample]);
            EXPECT_EQ(u.At(0, 0) * v.At(0, 0), sign);
            EXPECT_EQ(std::fabs(u.At(0, 0)), 1.0L);
            EXPECT_EQ(std::fabs(v.At(0, 0)), 1.0L);
            u.CheckGuards(); s.CheckGuards(); v.CheckGuards(); work.CheckGuards();
        }
    }
}

TYPED_TEST(AuditDecompBoundary, WideRankOneNeedsOneSweepAndCompletesTheNullBasis) {
    using F = TypeParam;
    using Raw = typename F::Raw;
    // A = [1,2]^T * [1,-2,1], so sigma = sqrt(30),0. This checks
    // the transposed working problem and null-column completion together.
    constexpr std::array<long double, 6> exact{1, -2, 1, 2, -4, 2};
    std::array<Raw, 6> input{};
    for (size_t i = 0; i < input.size(); ++i) input[i] = Encode<F>(exact[i], 3);
    const auto original = input;
    const linears_matrix_t a{input.data(), 2, 3, F::bits == 0 ? 0 : 3};
    GuardedOutput<F, 4> u(2, 2);
    GuardedOutput<F, 2> s(2, 1);
    GuardedOutput<F, 6> v(3, 2);
    GuardedWorkspace<F, LINEARS_SVD_WORKSPACE(2, 3)> work;
    ASSERT_EQ(F::svd(&a, &u.matrix, &s.matrix, &v.matrix, 0, work.workspace), LINEARS_ERROR_NO_CONVERGENCE);
    u.CheckUntouched(); s.CheckUntouched(); v.CheckUntouched();
    ASSERT_EQ(F::svd(&a, &u.matrix, &s.matrix, &v.matrix, 1, work.workspace), LINEARS_OK);
    EXPECT_EQ(input, original);
    const long double work_error = 64.0L * WorkingEpsilon<F>();
    EXPECT_NEAR(s.At(0, 0), std::sqrt(30.0L), work_error * std::sqrt(30.0L) + ExportError<F>(s.matrix.exponent));
    EXPECT_EQ(s.At(1, 0), 0.0L);
    // Account explicitly for final block rounding, separately from the short
    // two-column rotation's working error. No tolerance depends on the error.
    const long double eu = ExportError<F>(u.matrix.exponent);
    const long double es = ExportError<F>(s.matrix.exponent);
    const long double ev = ExportError<F>(v.matrix.exponent);
    const long double product_error = work_error * std::sqrt(30.0L) +
        eu * (s.At(0, 0) + es) * (1.0L + ev) + es * (1.0L + ev) + s.At(0, 0) * ev;
    for (size_t row = 0; row < 2; ++row) {
        for (size_t col = 0; col < 3; ++col) {
            long double reconstructed = 0.0L;
            for (size_t k = 0; k < 2; ++k)
                reconstructed += u.At(row, k) * s.At(k, 0) * v.At(col, k);
            EXPECT_NEAR(reconstructed, exact[row * 3 + col], product_error);
        }
    }
    for (size_t i = 0; i < 2; ++i) {
        for (size_t j = 0; j < 2; ++j) {
            long double left = 0.0L, right = 0.0L;
            for (size_t row = 0; row < 2; ++row) left += u.At(row, i) * u.At(row, j);
            for (size_t row = 0; row < 3; ++row) right += v.At(row, i) * v.At(row, j);
            EXPECT_NEAR(left, i == j ? 1.0L : 0.0L, work_error + 4.0L * eu + 2.0L * eu * eu);
            EXPECT_NEAR(right, i == j ? 1.0L : 0.0L, work_error + 6.0L * ev + 3.0L * ev * ev);
        }
    }
    u.CheckGuards(); s.CheckGuards(); v.CheckGuards(); work.CheckGuards();
}

TYPED_TEST(AuditDecompBoundary, SingularLdltAfterATwoByTwoPivotPreservesBothOutputs) {
    using F = TypeParam;
    using Raw = typename F::Raw;
    std::array<Raw, 9> input{};
    input[1] = input[3] = Encode<F>(1.0L, 1);
    const auto original = input;
    const linears_matrix_t a{input.data(), 3, 3, F::bits == 0 ? 0 : 1};
    GuardedOutput<F, 9> l(3, 3), d(3, 3);
    GuardedWorkspace<F, LINEARS_LDLT_WORKSPACE(3)> work;
    std::array<size_t, 3> permutation{91, 92, 93};
    std::array<uint8_t, 3> blocks{81, 82, 83};
    ASSERT_EQ(F::ldlt(&a, &l.matrix, &d.matrix, permutation.data(), 3,
                      blocks.data(), 3, work.workspace), LINEARS_ERROR_SINGULAR);
    EXPECT_EQ(input, original);
    l.CheckUntouched(); d.CheckUntouched(); work.CheckGuards();
    // Numerical failure may change metadata; do not require rollback that the
    // API does not promise. Nonsymmetry is detected before pivoting starts.
    input[2] = Encode<F>(0.5L, 1);
    permutation = {91, 92, 93}; blocks = {81, 82, 83};
    ASSERT_EQ(F::ldlt(&a, &l.matrix, &d.matrix, permutation.data(), 3,
                      blocks.data(), 3, work.workspace), LINEARS_ERROR_DIMENSION);
    EXPECT_EQ(permutation, (std::array<size_t, 3>{91, 92, 93}));
    EXPECT_EQ(blocks, (std::array<uint8_t, 3>{81, 82, 83}));
    l.CheckUntouched(); d.CheckUntouched(); work.CheckGuards();
}

TYPED_TEST(AuditDecompBoundary, EmptyHugeDimensionsAndOverflowAreCheckedBeforeAccess) {
    using F = TypeParam;
    for (const bool wide : {false, true}) {
        const size_t m = wide ? 0 : SIZE_MAX;
        const size_t n = wide ? SIZE_MAX : 0;
        linears_matrix_t a{nullptr, m, n, 0};
        linears_matrix_t u{nullptr, m, 0, 71};
        linears_matrix_t s{nullptr, 0, 1, 72};
        linears_matrix_t v{nullptr, n, 0, 73};
        ASSERT_EQ(F::svd(&a, &u, &s, &v, SIZE_MAX, {nullptr, 0}), LINEARS_OK);
        EXPECT_EQ(u.exponent, 0); EXPECT_EQ(s.exponent, 0); EXPECT_EQ(v.exponent, 0);
    }
    linears_matrix_t a{nullptr, 0, 0, 0};
    linears_matrix_t l{nullptr, 0, 0, 71};
    linears_matrix_t d{nullptr, 0, 0, 72};
    ASSERT_EQ(F::ldlt(&a, &l, &d, nullptr, 0, nullptr, 0, {nullptr, 0}), LINEARS_OK);
    EXPECT_EQ(l.exponent, 0); EXPECT_EQ(d.exponent, 0);
    l.exponent = 71;
    EXPECT_EQ(F::ldlt(&a, &l, &l, nullptr, 0, nullptr, 0, {nullptr, 0}), LINEARS_ERROR_ALIAS);
    EXPECT_EQ(l.exponent, 71);

    typename F::Raw raw{};
    typename F::Work storage{};
    a = {&raw, SIZE_MAX, SIZE_MAX, 0};
    l = {&raw, SIZE_MAX, SIZE_MAX, 71};
    d = {&raw, SIZE_MAX, SIZE_MAX, 72};
    size_t permutation = 91;
    uint8_t block = 81;
    EXPECT_EQ(F::ldlt(&a, &l, &d, &permutation, SIZE_MAX, &block, SIZE_MAX,
                      {&storage, SIZE_MAX}), LINEARS_ERROR_SIZE);
    EXPECT_EQ(permutation, size_t{91}); EXPECT_EQ(block, 81);
    EXPECT_EQ(l.exponent, 71); EXPECT_EQ(d.exponent, 72);
    a = {&raw, SIZE_MAX, 1, 0};
    l = {&raw, SIZE_MAX, 1, 71};
    d = {&raw, 1, 1, 72};
    linears_matrix_t v{&raw, 1, 1, 73};
    EXPECT_EQ(F::svd(&a, &l, &d, &v, 1, {&storage, SIZE_MAX}), LINEARS_ERROR_SIZE);
    EXPECT_EQ(l.exponent, 71); EXPECT_EQ(d.exponent, 72); EXPECT_EQ(v.exponent, 73);
}

TYPED_TEST(AuditDecompBoundary, MetadataOverlapAndShortCapacityNeverWriteOutputs) {
    using F = TypeParam;
    using Raw = typename F::Raw;
    const Raw unit = Encode<F>(1.0L, 1);
    std::array<Raw, 4> input{unit, 0, 0, unit};
    const linears_matrix_t a{input.data(), 2, 2, F::bits == 0 ? 0 : 1};
    GuardedOutput<F, 4> l(2, 2), d(2, 2);
    GuardedWorkspace<F, LINEARS_LDLT_WORKSPACE(2)> work;
    std::array<size_t, 2> permutation{91, 92};
    std::array<uint8_t, 2> blocks{81, 82};
    auto *overlapping_blocks = reinterpret_cast<uint8_t *>(permutation.data()) + sizeof(size_t);
    EXPECT_EQ(F::ldlt(&a, &l.matrix, &d.matrix, permutation.data(), 2,
                      overlapping_blocks, 2, work.workspace), LINEARS_ERROR_ALIAS);
    EXPECT_EQ(permutation, (std::array<size_t, 2>{91, 92}));
    auto *descriptor_metadata = reinterpret_cast<size_t *>(&d.matrix);
    EXPECT_EQ(F::ldlt(&a, &l.matrix, &d.matrix, descriptor_metadata, 2,
                      blocks.data(), 2, work.workspace), LINEARS_ERROR_ALIAS);
    EXPECT_EQ(F::ldlt(&a, &l.matrix, &d.matrix, permutation.data(), 1,
                      blocks.data(), 2, work.workspace), LINEARS_ERROR_WORKSPACE);
    EXPECT_EQ(F::ldlt(&a, &l.matrix, &d.matrix, permutation.data(), 2,
                      blocks.data(), 1, work.workspace), LINEARS_ERROR_WORKSPACE);
    EXPECT_EQ(permutation, (std::array<size_t, 2>{91, 92}));
    EXPECT_EQ(blocks, (std::array<uint8_t, 2>{81, 82}));
    l.CheckUntouched(); d.CheckUntouched(); work.CheckGuards();
}

TEST(AuditDecompStorage, WorkspaceMacrosRetainZeroAndOverflowBoundaries) {
    static_assert(LINEARS_LDLT_WORKSPACE(0) == 0, "empty LDLT");
    static_assert(LINEARS_LDLT_WORKSPACE(1) == 1, "scalar LDLT");
    static_assert(LINEARS_LDLT_WORKSPACE(SIZE_MAX) == SIZE_MAX, "LDLT overflow");
    static_assert(LINEARS_SVD_WORKSPACE(0, SIZE_MAX) == 0, "empty wide SVD");
    static_assert(LINEARS_SVD_WORKSPACE(SIZE_MAX, 0) == 0, "empty tall SVD");
    static_assert(LINEARS_SVD_WORKSPACE(SIZE_MAX - 2, 1) == SIZE_MAX, "addition sentinel");
    static_assert(LINEARS_SVD_WORKSPACE(SIZE_MAX - 3, 1) == SIZE_MAX - 1, "last finite element count");
    EXPECT_EQ(LINEARS_SVD_WORKSPACE(2, 3), size_t{12});
    EXPECT_EQ(LINEARS_SVD_WORKSPACE(3, 2), size_t{12});
    EXPECT_EQ(LINEARS_SIZE_MUL(LINEARS_SVD_WORKSPACE(SIZE_MAX - 3, 1), sizeof(double)), SIZE_MAX);
}

}  // namespace
