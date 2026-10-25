#include "linears_decomp.h"

#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <cstdint>
#include <limits>

namespace {

// Independent adapters and exact/dyadic references: do not use the author's
// decomp_support or production arithmetic to construct expected factors.
#define AUDIT_DECOMP_FORMAT(Name, Suffix, RawType, WorkType, Bits) \
    struct Name { \
        using Raw = RawType; using Work = WorkType; \
        using Workspace = linears_##Suffix##_workspace_t; \
        static constexpr int bits = Bits; \
        static linears_status_t svd(const linears_matrix_t *a, linears_matrix_t *u, \
            linears_matrix_t *s, linears_matrix_t *v, size_t budget, Workspace work) { \
            return linears_svd_##Suffix(a,u,s,v,budget,work); \
        } \
        static linears_status_t ldlt(const linears_matrix_t *a, linears_matrix_t *l, \
            linears_matrix_t *d, size_t *p, size_t pc, uint8_t *b, size_t bc, Workspace w) { \
            return linears_ldlt_##Suffix(a,l,d,p,pc,b,bc,w); \
        } \
        static linears_status_t inverse(const linears_matrix_t *a, linears_matrix_t *out, Workspace w) { \
            return linears_inverse_##Suffix(a,out,w); \
        } \
    };
AUDIT_DECOMP_FORMAT(AuditF32, f32, float, float, 0)
AUDIT_DECOMP_FORMAT(AuditF64, f64, double, double, 0)
AUDIT_DECOMP_FORMAT(AuditQ15, q15, int16_t, int64_t, 15)
AUDIT_DECOMP_FORMAT(AuditQ31, q31, int32_t, int64_t, 31)
#undef AUDIT_DECOMP_FORMAT
using AuditFormats = ::testing::Types<AuditF32, AuditF64, AuditQ15, AuditQ31>;

template<class F> typename F::Raw Encode(long double value) {
    if constexpr (F::bits == 0) return static_cast<typename F::Raw>(value);
    else {
        long double raw = std::round(std::ldexp(value, F::bits));
        const long double maximum = std::numeric_limits<typename F::Raw>::max();
        if (raw > maximum) raw = maximum;
        return static_cast<typename F::Raw>(raw);
    }
}

template<class F> long double Decode(typename F::Raw value) {
    if constexpr (F::bits == 0) return static_cast<long double>(value);
    else return std::ldexp(static_cast<long double>(value), -F::bits);
}

template<class F> long double WorkingEpsilon() {
    if constexpr (F::bits == 0) return std::numeric_limits<typename F::Raw>::epsilon();
    else return 0x1p-31L;
}

template<class F> long double CellError(typename F::Raw value) {
    if constexpr (F::bits == 0) return 0.0L;
    else return std::ldexp(value == std::numeric_limits<typename F::Raw>::max() ? 1.0L : 0.5L, -F::bits);
}

template<class F, size_t Count> struct GuardedOutput {
    using Raw = typename F::Raw;
    std::array<Raw, Count + 2> data{};
    linears_matrix_t matrix;
    const size_t rows, cols;
    GuardedOutput(size_t r, size_t c) : matrix{data.data() + 1, r, c}, rows(r), cols(c) {
        data.fill(static_cast<Raw>(37));
    }
    void CheckDescriptor() const {
        EXPECT_EQ(matrix.data, data.data() + 1); EXPECT_EQ(matrix.rows, rows); EXPECT_EQ(matrix.cols, cols);
    }
    void CheckUntouched() const {
        for (const auto value : data) EXPECT_EQ(value, static_cast<Raw>(37));
        CheckDescriptor();
    }
    void CheckGuards() const {
        EXPECT_EQ(data.front(), static_cast<Raw>(37)); EXPECT_EQ(data.back(), static_cast<Raw>(37));
        CheckDescriptor();
    }
    long double At(size_t row, size_t col) const { return Decode<F>(data[1 + row * cols + col]); }
    long double QuantizationBound() const {
        long double result = 0.0L;
        for (size_t i = 1; i <= Count; ++i) result = std::fmax(result, CellError<F>(data[i]));
        return result;
    }
};

template<class F, size_t Count> struct GuardedWorkspace {
    using Work = typename F::Work;
    std::array<Work, Count + 2> data{};
    typename F::Workspace workspace{data.data() + 1, Count};
    GuardedWorkspace() { data.fill(static_cast<Work>(19)); }
    void CheckGuards() const { EXPECT_EQ(data.front(), Work{19}); EXPECT_EQ(data.back(), Work{19}); }
};

template<class F> class AuditDecompBoundary : public ::testing::Test {};
TYPED_TEST_SUITE(AuditDecompBoundary, AuditFormats);

TYPED_TEST(AuditDecompBoundary, NegativeLdltEndpointAndUnitFactorsHaveExplicitErrorBounds) {
    using F = TypeParam; using Raw = typename F::Raw;
    const std::array<long double, 4> exact{32767.0L / 32768, 128.0L / 32768,
                                          128.0L / 32768, -32767.0L / 32768};
    std::array<Raw, 4> input{};
    for (size_t i = 0; i < 4; ++i) input[i] = Encode<F>(exact[i]);
    const auto before = input;
    const linears_matrix_t a{input.data(), 2, 2};
    GuardedOutput<F, 4> l(2,2), d(2,2);
    GuardedWorkspace<F, LINEARS_LDLT_WORKSPACE(2)> work;
    std::array<size_t, 2> p{91,92}; std::array<uint8_t, 2> blocks{81,82};
    ASSERT_EQ(F::ldlt(&a,&l.matrix,&d.matrix,p.data(),2,blocks.data(),2,work.workspace), LINEARS_OK);
    EXPECT_EQ(input, before); EXPECT_EQ(p, (std::array<size_t,2>{0,1}));
    EXPECT_EQ(blocks, (std::array<uint8_t,2>{1,1}));
    const long double diagonal = (-32767.0L - 16384.0L / 32767) / 32768;
    const long double error = 8 * WorkingEpsilon<F>();
    EXPECT_NEAR(d.At(0,0), exact[0], error + CellError<F>(d.data[1]));
    EXPECT_NEAR(d.At(1,1), diagonal, error + CellError<F>(d.data[4]));
    EXPECT_EQ(d.At(0,1), 0); EXPECT_EQ(d.At(1,0), 0); EXPECT_EQ(l.At(0,1), 0);
    EXPECT_NEAR(l.At(0,0), 1, CellError<F>(l.data[1]));
    EXPECT_NEAR(l.At(1,1), 1, CellError<F>(l.data[4]));
    EXPECT_NEAR(l.At(1,0), 128.0L / 32767, error + CellError<F>(l.data[3]));
    if constexpr (F::bits == 15) {
        EXPECT_EQ(d.data, (std::array<Raw,6>{37,32767,0,0,INT16_MIN,37}));
        EXPECT_EQ(l.data[1], INT16_MAX); EXPECT_EQ(l.data[4], INT16_MAX);
        EXPECT_EQ(l.data[3], 128);
    }
    l.CheckGuards(); d.CheckGuards(); work.CheckGuards();
}

TYPED_TEST(AuditDecompBoundary, ARealSchurComplementOutsideSignalRangeCannotPartiallyPublish) {
    using F = TypeParam; using Raw = typename F::Raw;
    std::array<Raw,4> input{Encode<F>(0.75L),Encode<F>(0.5L),Encode<F>(0.5L),Encode<F>(-0.75L)};
    const linears_matrix_t a{input.data(),2,2};
    GuardedOutput<F,4> l(2,2), d(2,2);
    GuardedWorkspace<F,LINEARS_LDLT_WORKSPACE(2)> work;
    std::array<size_t,2> p{91,92}; std::array<uint8_t,2> blocks{81,82};
    const auto status = F::ldlt(&a,&l.matrix,&d.matrix,p.data(),2,blocks.data(),2,work.workspace);
    if constexpr (F::bits == 0) {
        ASSERT_EQ(status, LINEARS_OK);
        EXPECT_NEAR(d.At(1,1), -13.0L / 12, 8 * WorkingEpsilon<F>());
    } else {
        EXPECT_EQ(status, LINEARS_ERROR_RANGE);
        l.CheckUntouched(); d.CheckUntouched();
    }
    l.CheckGuards(); d.CheckGuards(); work.CheckGuards();
}

TYPED_TEST(AuditDecompBoundary, OneDimensionalSvdRetainsFloatExtremesAndRawFixedEndpoints) {
    using F = TypeParam; using Raw = typename F::Raw;
    std::array<Raw,5> samples{};
    if constexpr (F::bits == 0) {
        samples = {-std::numeric_limits<Raw>::max(), -std::numeric_limits<Raw>::denorm_min(),
                    Raw{0}, std::numeric_limits<Raw>::denorm_min(), std::numeric_limits<Raw>::max()};
    } else samples = {std::numeric_limits<Raw>::min(), Raw{-1}, Raw{0}, Raw{1}, std::numeric_limits<Raw>::max()};
    for (Raw input : samples) {
        const linears_matrix_t a{&input,1,1};
        GuardedOutput<F,1> u(1,1), s(1,1), v(1,1);
        GuardedWorkspace<F,LINEARS_SVD_WORKSPACE(1,1)> work;
        ASSERT_EQ(F::svd(&a,&u.matrix,&s.matrix,&v.matrix,0,work.workspace), LINEARS_OK);
        if constexpr (F::bits == 0) {
            EXPECT_EQ(s.data[1], std::fabs(input));
            EXPECT_EQ(std::fabs(u.At(0,0)), 1); EXPECT_EQ(std::fabs(v.At(0,0)), 1);
            if (input != 0) {
                EXPECT_EQ(u.At(0,0) * v.At(0,0), input < 0 ? -1 : 1);
            }
        } else {
            const int64_t magnitude = input < 0 ? -static_cast<int64_t>(input) : input;
            const int64_t maximum = std::numeric_limits<Raw>::max();
            EXPECT_EQ(s.data[1], magnitude > maximum ? maximum : magnitude);
            EXPECT_NEAR(std::fabs(u.At(0,0)), 1, u.QuantizationBound());
            EXPECT_NEAR(std::fabs(v.At(0,0)), 1, v.QuantizationBound());
            if (input != 0) {
                EXPECT_NEAR(u.At(0,0) * v.At(0,0), input < 0 ? -1 : 1,
                    u.QuantizationBound() + v.QuantizationBound() + u.QuantizationBound() * v.QuantizationBound());
            }
        }
        u.CheckGuards(); s.CheckGuards(); v.CheckGuards(); work.CheckGuards();
    }
}

TYPED_TEST(AuditDecompBoundary, WideRankOneNeedsOneSweepAndCompletesTheNullBasis) {
    using F = TypeParam; using Raw = typename F::Raw;
    // Float fixture is unchanged. Fixed inputs use exact dyadic values with
    // sigma=sqrt(30)/8 < 1, so every required signal output is representable.
    constexpr long double scale = F::bits == 0 ? 1.0L : 0.125L;
    constexpr std::array<long double,6> unscaled{1,-2,1,2,-4,2};
    std::array<Raw,6> input{};
    for (size_t i=0;i<6;++i) input[i] = Encode<F>(unscaled[i] * scale);
    const auto before = input;
    const linears_matrix_t a{input.data(),2,3};
    GuardedOutput<F,4> u(2,2); GuardedOutput<F,2> s(2,1); GuardedOutput<F,6> v(3,2);
    GuardedWorkspace<F,LINEARS_SVD_WORKSPACE(2,3)> work;
    ASSERT_EQ(F::svd(&a,&u.matrix,&s.matrix,&v.matrix,0,work.workspace), LINEARS_ERROR_NO_CONVERGENCE);
    u.CheckUntouched(); s.CheckUntouched(); v.CheckUntouched();
    ASSERT_EQ(F::svd(&a,&u.matrix,&s.matrix,&v.matrix,1,work.workspace), LINEARS_OK);
    EXPECT_EQ(input, before);
    const long double sigma = std::sqrt(30.0L) * scale;
    const long double arithmetic = 64 * WorkingEpsilon<F>();
    EXPECT_NEAR(s.At(0,0), sigma, arithmetic * sigma + s.QuantizationBound());
    EXPECT_EQ(s.At(1,0), 0);
    const auto eu=u.QuantizationBound(), es=s.QuantizationBound(), ev=v.QuantizationBound();
    const long double product_error = arithmetic * sigma +
        eu * (s.At(0,0) + es) * (1 + ev) + es * (1 + ev) + s.At(0,0) * ev;
    for (size_t row=0;row<2;++row) for (size_t col=0;col<3;++col) {
        long double value=0;
        for (size_t k=0;k<2;++k) value += u.At(row,k) * s.At(k,0) * v.At(col,k);
        EXPECT_NEAR(value, Decode<F>(input[row*3+col]), product_error);
    }
    for (size_t i=0;i<2;++i) for (size_t j=0;j<2;++j) {
        long double left=0,right=0;
        for (size_t row=0;row<2;++row) left += u.At(row,i) * u.At(row,j);
        for (size_t row=0;row<3;++row) right += v.At(row,i) * v.At(row,j);
        EXPECT_NEAR(left, i==j ? 1 : 0, arithmetic + 4*eu + 2*eu*eu);
        EXPECT_NEAR(right, i==j ? 1 : 0, arithmetic + 6*ev + 3*ev*ev);
    }
    u.CheckGuards(); s.CheckGuards(); v.CheckGuards(); work.CheckGuards();
}

TYPED_TEST(AuditDecompBoundary, SingularValuesAboveOneAreRangeOnlyForFixedOutputs) {
    using F=TypeParam; using Raw=typename F::Raw;
    std::array<Raw,2> input{Encode<F>(0.75L),Encode<F>(0.75L)};
    const linears_matrix_t a{input.data(),2,1};
    GuardedOutput<F,2> u(2,1); GuardedOutput<F,1> s(1,1),v(1,1);
    GuardedWorkspace<F,LINEARS_SVD_WORKSPACE(2,1)> work;
    const auto status=F::svd(&a,&u.matrix,&s.matrix,&v.matrix,0,work.workspace);
    if constexpr(F::bits==0) {
        ASSERT_EQ(status,LINEARS_OK);
        EXPECT_NEAR(s.At(0,0),0.75L*std::sqrt(2.0L),8*WorkingEpsilon<F>());
    } else {
        EXPECT_EQ(status,LINEARS_ERROR_RANGE);
        u.CheckUntouched();s.CheckUntouched();v.CheckUntouched();
    }
    u.CheckGuards();s.CheckGuards();v.CheckGuards();work.CheckGuards();
}

TYPED_TEST(AuditDecompBoundary, PositiveSignalIdentityIsNotExactOneWhenInverted) {
    using F=TypeParam; using Raw=typename F::Raw;
    Raw input = F::bits==0 ? Raw{1} : std::numeric_limits<Raw>::max();
    const linears_matrix_t a{&input,1,1};
    GuardedOutput<F,1> inverse(1,1);
    GuardedWorkspace<F,LINEARS_INVERSE_WORKSPACE(1)> work;
    auto status=F::inverse(&a,&inverse.matrix,work.workspace);
    if constexpr(F::bits==0) { ASSERT_EQ(status,LINEARS_OK); EXPECT_EQ(inverse.At(0,0),1); }
    else { EXPECT_EQ(status,LINEARS_ERROR_RANGE); inverse.CheckUntouched(); }
    input=Encode<F>(-1);
    ASSERT_EQ(F::inverse(&a,&inverse.matrix,work.workspace),LINEARS_OK);
    EXPECT_EQ(inverse.At(0,0),-1);
    inverse.CheckGuards();work.CheckGuards();
}

TYPED_TEST(AuditDecompBoundary, HadamardInverseIsRepresentableAndMatchesExactFormula) {
    using F=TypeParam; using Raw=typename F::Raw;
    std::array<Raw,4> input{Encode<F>(0.75L),Encode<F>(0.75L),Encode<F>(0.75L),Encode<F>(-0.75L)};
    const auto before=input;
    const linears_matrix_t a{input.data(),2,2};
    GuardedOutput<F,4> inverse(2,2);
    GuardedWorkspace<F,LINEARS_INVERSE_WORKSPACE(2)> work;
    ASSERT_EQ(F::inverse(&a,&inverse.matrix,work.workspace),LINEARS_OK);
    EXPECT_EQ(input,before);
    for(size_t row=0;row<2;++row) for(size_t col=0;col<2;++col) {
        const long double expected=(row==1 && col==1) ? -2.0L/3 : 2.0L/3;
        EXPECT_NEAR(inverse.At(row,col),expected,8*WorkingEpsilon<F>()+CellError<F>(inverse.data[1+row*2+col]));
    }
    inverse.CheckGuards();work.CheckGuards();
}

TYPED_TEST(AuditDecompBoundary, SingularLdltAfterATwoByTwoPivotPreservesBothOutputs) {
    using F=TypeParam; using Raw=typename F::Raw;
    std::array<Raw,9> input{};
    input[1]=input[3]=Encode<F>(F::bits==0 ? 1.0L : 0.5L);
    const auto before=input;
    const linears_matrix_t a{input.data(),3,3};
    GuardedOutput<F,9> l(3,3),d(3,3);
    GuardedWorkspace<F,LINEARS_LDLT_WORKSPACE(3)> work;
    std::array<size_t,3> p{91,92,93};std::array<uint8_t,3> blocks{81,82,83};
    ASSERT_EQ(F::ldlt(&a,&l.matrix,&d.matrix,p.data(),3,blocks.data(),3,work.workspace),LINEARS_ERROR_SINGULAR);
    EXPECT_EQ(input,before);l.CheckUntouched();d.CheckUntouched();work.CheckGuards();
    // Numerical failure can alter metadata. Nonsymmetry precedes pivoting.
    input[2]=Encode<F>(0.5L);p={91,92,93};blocks={81,82,83};
    ASSERT_EQ(F::ldlt(&a,&l.matrix,&d.matrix,p.data(),3,blocks.data(),3,work.workspace),LINEARS_ERROR_DIMENSION);
    EXPECT_EQ(p,(std::array<size_t,3>{91,92,93}));EXPECT_EQ(blocks,(std::array<uint8_t,3>{81,82,83}));
    l.CheckUntouched();d.CheckUntouched();work.CheckGuards();
}

TYPED_TEST(AuditDecompBoundary, EmptyHugeDimensionsAndOverflowAreCheckedBeforeAccess) {
    using F=TypeParam;
    for(const bool wide : {false,true}) {
        const size_t m=wide?0:SIZE_MAX,n=wide?SIZE_MAX:0;
        linears_matrix_t a{nullptr,m,n},u{nullptr,m,0},s{nullptr,0,1},v{nullptr,n,0};
        ASSERT_EQ(F::svd(&a,&u,&s,&v,SIZE_MAX,{nullptr,0}),LINEARS_OK);
        EXPECT_EQ(u.rows,m);EXPECT_EQ(v.rows,n);EXPECT_EQ(u.data,nullptr);EXPECT_EQ(s.cols,size_t{1});
    }
    linears_matrix_t a{nullptr,0,0},l{nullptr,0,0},d{nullptr,0,0};
    ASSERT_EQ(F::ldlt(&a,&l,&d,nullptr,0,nullptr,0,{nullptr,0}),LINEARS_OK);
    EXPECT_EQ(F::ldlt(&a,&l,&l,nullptr,0,nullptr,0,{nullptr,0}),LINEARS_ERROR_ALIAS);
    typename F::Raw raw{};typename F::Work storage{};
    a={&raw,SIZE_MAX,SIZE_MAX};l=a;d=a;
    size_t p=91;uint8_t block=81;
    EXPECT_EQ(F::ldlt(&a,&l,&d,&p,SIZE_MAX,&block,SIZE_MAX,{&storage,SIZE_MAX}),LINEARS_ERROR_SIZE);
    EXPECT_EQ(p,size_t{91});EXPECT_EQ(block,81);
    a={&raw,SIZE_MAX,1};l=a;d={&raw,1,1};linears_matrix_t v{&raw,1,1};
    EXPECT_EQ(F::svd(&a,&l,&d,&v,1,{&storage,SIZE_MAX}),LINEARS_ERROR_SIZE);
    EXPECT_EQ(l.rows,SIZE_MAX);EXPECT_EQ(d.cols,size_t{1});EXPECT_EQ(v.cols,size_t{1});
}

TYPED_TEST(AuditDecompBoundary, MetadataOverlapAndShortCapacityNeverWriteOutputs) {
    using F=TypeParam;using Raw=typename F::Raw;
    const Raw unit=Encode<F>(1);
    std::array<Raw,4> input{unit,0,0,unit};const linears_matrix_t a{input.data(),2,2};
    GuardedOutput<F,4> l(2,2),d(2,2);GuardedWorkspace<F,LINEARS_LDLT_WORKSPACE(2)> work;
    std::array<size_t,2> p{91,92};std::array<uint8_t,2> blocks{81,82};
    auto *overlap=reinterpret_cast<uint8_t *>(p.data())+sizeof(size_t);
    EXPECT_EQ(F::ldlt(&a,&l.matrix,&d.matrix,p.data(),2,overlap,2,work.workspace),LINEARS_ERROR_ALIAS);
    auto *descriptor=reinterpret_cast<size_t *>(&d.matrix);
    EXPECT_EQ(F::ldlt(&a,&l.matrix,&d.matrix,descriptor,2,blocks.data(),2,work.workspace),LINEARS_ERROR_ALIAS);
    EXPECT_EQ(F::ldlt(&a,&l.matrix,&d.matrix,p.data(),1,blocks.data(),2,work.workspace),LINEARS_ERROR_WORKSPACE);
    EXPECT_EQ(F::ldlt(&a,&l.matrix,&d.matrix,p.data(),2,blocks.data(),1,work.workspace),LINEARS_ERROR_WORKSPACE);
    EXPECT_EQ(p,(std::array<size_t,2>{91,92}));EXPECT_EQ(blocks,(std::array<uint8_t,2>{81,82}));
    l.CheckUntouched();d.CheckUntouched();work.CheckGuards();
}

TEST(AuditDecompStorage, WorkspaceMacrosRetainZeroAndOverflowBoundaries) {
    static_assert(LINEARS_LDLT_WORKSPACE(0)==0,"empty LDLT");
    static_assert(LINEARS_LDLT_WORKSPACE(1)==1,"scalar LDLT");
    static_assert(LINEARS_LDLT_WORKSPACE(SIZE_MAX)==SIZE_MAX,"LDLT overflow");
    static_assert(LINEARS_SVD_WORKSPACE(0,SIZE_MAX)==0,"empty wide SVD");
    static_assert(LINEARS_SVD_WORKSPACE(SIZE_MAX,0)==0,"empty tall SVD");
    static_assert(LINEARS_SVD_WORKSPACE(SIZE_MAX-2,1)==SIZE_MAX,"addition sentinel");
    static_assert(LINEARS_SVD_WORKSPACE(SIZE_MAX-3,1)==SIZE_MAX-1,"last finite count");
    EXPECT_EQ(LINEARS_SVD_WORKSPACE(2,3),size_t{12});
    EXPECT_EQ(LINEARS_SVD_WORKSPACE(3,2),size_t{12});
    EXPECT_EQ(LINEARS_SIZE_MUL(LINEARS_SVD_WORKSPACE(SIZE_MAX-3,1),sizeof(double)),SIZE_MAX);
}

} // namespace
