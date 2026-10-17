#include "decomp_support.h"
#include <array>
#include <cstring>
#include <thread>

namespace {
using namespace decomp_test;

/* The exact input is Matrix::decoded(), never the pre-quantization fixture. */
template<class Format>
double export_error(const Matrix<Format> &matrix)
{
    if constexpr (Format::bits == 0) {
        return 0.0; // Native working values are copied without a second rounding.
    } else {
        return 0.5 * std::scalbn(1.0, matrix.matrix.exponent - Format::bits) *
               std::sqrt(static_cast<double>(matrix.data.size()));
    }
}

template<class Format>
void check_factors(const Eigen::MatrixXd &input, const Matrix<Format> &u,
                   const Matrix<Format> &s, const Matrix<Format> &v,
                   double truncation_bound = 0.0)
{
    const Eigen::Index r = std::min(input.rows(), input.cols());
    const double dimension = static_cast<double>(std::max(input.rows(), input.cols()));
    const Eigen::MatrixXd ud = u.decoded();
    const Eigen::MatrixXd sd = s.decoded();
    const Eigen::MatrixXd vd = v.decoded();
    ASSERT_EQ(sd.rows(), r);
    ASSERT_EQ(sd.cols(), 1);
    ASSERT_TRUE(ud.allFinite());
    ASSERT_TRUE(sd.allFinite());
    ASSERT_TRUE(vd.allFinite());
    if (r == 0) return;
    for (Eigen::Index i = 0; i < r; ++i) {
        EXPECT_GE(sd(i, 0), 0.0);
        if (i != 0) {
            EXPECT_GE(sd(i - 1, 0), sd(i, 0));
        }
    }
    const double qu = export_error(u);
    const double qs = export_error(s);
    const double qv = export_error(v);
    // Conservative product perturbation bound, using Frobenius submultiplicativity.
    const double quantized_product = qu * sd.norm() * vd.norm() +
        (ud.norm() + qu) * qs * vd.norm() +
        (ud.norm() + qu) * (sd.norm() + qs) * qv;
    // Dot products/rotations incur dimension-scaled working roundoff. Truncation
    // is a separate explicit budget, not hidden in the storage quantization term.
    const double working = 32.0 * dimension * static_cast<double>(r) *
                           Format::working_epsilon;
    const Eigen::MatrixXd reconstructed = ud * sd.col(0).asDiagonal() * vd.transpose();
    EXPECT_LE((reconstructed - input).norm(),
              working * input.norm() + truncation_bound + quantized_product);
    const Eigen::MatrixXd identity = Eigen::MatrixXd::Identity(r, r);
    EXPECT_LE((ud.transpose() * ud - identity).norm(),
              working + 2.0 * ud.norm() * qu + qu * qu);
    EXPECT_LE((vd.transpose() * vd - identity).norm(),
              working + 2.0 * vd.norm() * qv + qv * qv);
    const Eigen::JacobiSVD<Eigen::MatrixXd> reference(input);
    EXPECT_LE((sd.col(0) - reference.singularValues()).norm(),
              working * input.norm() + truncation_bound + qs);
}

template<class Format>
void factor_and_check(const Eigen::MatrixXd &values, size_t sweeps = 64, int scale = 0,
                      bool allow_truncation = false)
{
    Matrix<Format> input(values, scale);
    const size_t m = input.matrix.rows;
    const size_t n = input.matrix.cols;
    const size_t r = std::min(m, n);
    Matrix<Format> u(m, r), s(r, 1), v(n, r);
    Scratch<Format> scratch(LINEARS_SVD_WORKSPACE(m, n));
    const auto original = input.data;
    const int original_exponent = input.matrix.exponent;
    const Eigen::MatrixXd represented = input.decoded();
    ASSERT_EQ(Format::svd(&input.matrix, &u.matrix, &s.matrix, &v.matrix,
                          sweeps, scratch.workspace), LINEARS_OK);
    EXPECT_EQ(input.data, original);
    EXPECT_EQ(input.matrix.exponent, original_exponent);
    const double tau = 8.0 * static_cast<double>(std::max(m, n)) * Format::working_epsilon;
    const double loss = allow_truncation ? static_cast<double>(r) * tau * represented.norm() : 0.0;
    check_factors(represented, u, s, v, loss);
    scratch.check();
}

template<class Format>
void expect_sentinel(const Matrix<Format> &matrix)
{
    for (auto value : matrix.data) EXPECT_EQ(value, static_cast<typename Format::Raw>(37));
    EXPECT_EQ(matrix.matrix.exponent, 17);
}

template<class Format> class Svd : public ::testing::Test {};
TYPED_TEST_SUITE(Svd, Formats);

TYPED_TEST(Svd, RandomTallWideSquareAndOneDimensional)
{
    const std::array<std::array<size_t, 2>, 12> shapes = {{{1, 1}, {1, 5}, {5, 1},
        {2, 2}, {3, 5}, {5, 3}, {8, 8}, {11, 6}, {6, 11}, {19, 17}, {71, 3}, {3, 71}}};
    for (const auto &shape : shapes) {
        SCOPED_TRACE(::testing::Message() << shape[0] << "x" << shape[1]);
        for (uint32_t seed = 1; seed <= 3; ++seed)
            factor_and_check<TypeParam>(random_matrix(shape[0], shape[1], seed));
    }
}

TYPED_TEST(Svd, RepeatedSingularValuesNeedNoParticularVectorBasis)
{
    Eigen::MatrixXd input(5, 3);
    input << 1, 0, 0, 0, 1, 0, 0, 0, 0.5, 1, 0, 0, 0, 1, 0;
    factor_and_check<TypeParam>(input, 0);
    factor_and_check<TypeParam>(input.transpose(), 0);
    // Binary-exact Hadamard factors preserve repeated values after q import.
    Eigen::MatrixXd h(4, 4);
    h << 1, 1, 1, 1, 1, -1, 1, -1, 1, 1, -1, -1, 1, -1, -1, 1;
    h *= 0.5;
    Eigen::Vector4d spectrum(2.0, 2.0, 0.5, 0.0);
    factor_and_check<TypeParam>(h * spectrum.asDiagonal() * h.transpose(), 64, 0, true);
}

TYPED_TEST(Svd, RankDeficiencyCompletesOrthogonalColumns)
{
    for (const auto &shape : {std::pair<size_t, size_t>{5, 3}, {3, 5}, {5, 5}})
        factor_and_check<TypeParam>(Eigen::MatrixXd::Zero(shape.first, shape.second), 0);
    Eigen::MatrixXd rank_one(5, 3);
    rank_one << 1, -2, 3, 2, -4, 6, -1, 2, -3, 0, 0, 0, 1, -2, 3;
    factor_and_check<TypeParam>(rank_one, 64, 0, true);
    factor_and_check<TypeParam>(rank_one.transpose(), 64, 0, true);
    Eigen::MatrixXd rank_two = Eigen::MatrixXd::Zero(5, 4);
    rank_two.col(0) << 1, -1, 2, 0, 1;
    rank_two.col(1) << 0, 2, 1, 1, -1;
    rank_two.col(2) = rank_two.col(0) + rank_two.col(1);
    factor_and_check<TypeParam>(rank_two, 64, 0, true);
    Eigen::MatrixXd near_coordinate = Eigen::MatrixXd::Zero(5, 3);
    near_coordinate(0, 0) = 1.0;
    near_coordinate(1, 0) = 0x1p-20;
    factor_and_check<TypeParam>(near_coordinate, 0);
}

TYPED_TEST(Svd, EmptyShapesAllowNullStorageAndWorkspace)
{
    for (const auto &shape : {std::pair<size_t, size_t>{0, 0}, {0, 5}, {5, 0},
                              {0, SIZE_MAX}, {SIZE_MAX, 0}}) {
        Matrix<TypeParam> input(shape.first, shape.second);
        Matrix<TypeParam> u(shape.first, 0), s(0, 1), v(shape.second, 0);
        u.matrix.exponent = s.matrix.exponent = v.matrix.exponent = 17;
        typename TypeParam::Workspace workspace{nullptr, 0};
        ASSERT_EQ(TypeParam::svd(&input.matrix, &u.matrix, &s.matrix, &v.matrix,
                                 0, workspace), LINEARS_OK);
        EXPECT_EQ(u.matrix.exponent, 0);
        EXPECT_EQ(s.matrix.exponent, 0);
        EXPECT_EQ(v.matrix.exponent, 0);
        EXPECT_EQ(LINEARS_SVD_WORKSPACE(shape.first, shape.second), size_t{0});
    }
}

TYPED_TEST(Svd, OneSweepSolvesOneColumnPairAndHugeBudgetIsAccepted)
{
    Eigen::MatrixXd input(3, 2);
    input << 1, 1, 0, 1, 2, 1;
    factor_and_check<TypeParam>(input, 1);
    factor_and_check<TypeParam>(input, std::numeric_limits<size_t>::max());
}

TYPED_TEST(Svd, ExhaustedBudgetDoesNotPublishPartialFactors)
{
    Matrix<TypeParam> input(random_matrix(4, 4, 731));
    Matrix<TypeParam> u(4, 4), s(4, 1), v(4, 4);
    Scratch<TypeParam> scratch(LINEARS_SVD_WORKSPACE(4, 4));
    for (size_t budget : {size_t{0}, size_t{1}}) {
        u.sentinel(); s.sentinel(); v.sentinel();
        ASSERT_EQ(TypeParam::svd(&input.matrix, &u.matrix, &s.matrix, &v.matrix,
                                 budget, scratch.workspace), LINEARS_ERROR_NO_CONVERGENCE);
        expect_sentinel(u); expect_sentinel(s); expect_sentinel(v);
        scratch.check();
    }
    Eigen::MatrixXd diagonal = Eigen::MatrixXd::Zero(3, 3);
    diagonal.diagonal() << -3, 2, 1;
    factor_and_check<TypeParam>(diagonal, 0);
}

TYPED_TEST(Svd, NumericalRankCutoffHasAnExplicitResidualBudget)
{
    Eigen::MatrixXd diagonal = Eigen::MatrixXd::Zero(3, 3);
    diagonal.diagonal() << 1.0, 0.25, 4.0 * TypeParam::working_epsilon;
    Matrix<TypeParam> input(diagonal);
    Matrix<TypeParam> u(3, 3), s(3, 1), v(3, 3);
    Scratch<TypeParam> scratch(LINEARS_SVD_WORKSPACE(3, 3));
    const Eigen::MatrixXd represented = input.decoded();
    const double tau = 8.0 * 3.0 * TypeParam::working_epsilon;
    ASSERT_EQ(TypeParam::svd(&input.matrix, &u.matrix, &s.matrix, &v.matrix,
                             0, scratch.workspace), LINEARS_OK);
    ASSERT_LE(std::abs(represented(2, 2)), tau);
    EXPECT_EQ(s.decoded()(2, 0), 0.0);
    check_factors(represented, u, s, v, 3.0 * tau * represented.norm());
    EXPECT_NEAR((u.decoded() * s.decoded().col(0).asDiagonal() *
                 v.decoded().transpose() - represented).norm(),
                std::abs(represented(2, 2)), 32.0 * TypeParam::storage_epsilon);
}

TYPED_TEST(Svd, ScalingAvoidsSquaringInputMagnitudes)
{
    const Eigen::MatrixXd input = random_matrix(4, 3, 415);
    factor_and_check<TypeParam>(input, 64, -60);
    factor_and_check<TypeParam>(input, 64, 60);
}

TYPED_TEST(Svd, FixedExtremeExponentsPreserveNormalizedFactors)
{
    if constexpr (TypeParam::bits == 0) {
        GTEST_SKIP() << "Fixed-point exponent contract only";
    } else {
        const Eigen::MatrixXd values = random_matrix(4, 3, 82);
        Matrix<TypeParam> baseline(values);
        Matrix<TypeParam> u0(4, 3), s0(3, 1), v0(3, 3);
        Scratch<TypeParam> work0(LINEARS_SVD_WORKSPACE(4, 3));
        ASSERT_EQ(TypeParam::svd(&baseline.matrix, &u0.matrix, &s0.matrix, &v0.matrix,
                                 64, work0.workspace), LINEARS_OK);
        for (int scale : {-15000, 15000}) {
            Matrix<TypeParam> input(values, scale);
            Matrix<TypeParam> u(4, 3), s(3, 1), v(3, 3);
            Scratch<TypeParam> work(LINEARS_SVD_WORKSPACE(4, 3));
            ASSERT_EQ(TypeParam::svd(&input.matrix, &u.matrix, &s.matrix, &v.matrix,
                                     64, work.workspace), LINEARS_OK);
            EXPECT_EQ(u.data, u0.data); EXPECT_EQ(v.data, v0.data); EXPECT_EQ(s.data, s0.data);
            EXPECT_EQ(u.matrix.exponent, u0.matrix.exponent);
            EXPECT_EQ(v.matrix.exponent, v0.matrix.exponent);
            EXPECT_EQ(s.matrix.exponent, s0.matrix.exponent + scale);
            work.check();
        }
    }
}

TYPED_TEST(Svd, RejectsNonfiniteInputsOrInvalidFixedExponentsAtomically)
{
    Matrix<TypeParam> input(Eigen::MatrixXd::Identity(2, 2));
    Matrix<TypeParam> u(2, 2), s(2, 1), v(2, 2);
    Scratch<TypeParam> scratch(LINEARS_SVD_WORKSPACE(2, 2));
    u.sentinel(); s.sentinel(); v.sentinel();
    if constexpr (TypeParam::bits == 0) {
        using Raw = typename TypeParam::Raw;
        for (Raw invalid : {std::numeric_limits<Raw>::quiet_NaN(),
                            std::numeric_limits<Raw>::infinity(),
                            -std::numeric_limits<Raw>::infinity()}) {
            input.data[0] = invalid;
            EXPECT_EQ(TypeParam::svd(&input.matrix, &u.matrix, &s.matrix, &v.matrix,
                                     32, scratch.workspace), LINEARS_ERROR_RANGE);
            expect_sentinel(u); expect_sentinel(s); expect_sentinel(v);
        }
    } else {
        for (int exponent : {LINEARS_EXPONENT_MIN - 1, LINEARS_EXPONENT_MAX + 1}) {
            input.matrix.exponent = exponent;
            EXPECT_EQ(TypeParam::svd(&input.matrix, &u.matrix, &s.matrix, &v.matrix,
                                     32, scratch.workspace), LINEARS_ERROR_RANGE);
            expect_sentinel(u); expect_sentinel(s); expect_sentinel(v);
        }
    }
    scratch.check();
}

TYPED_TEST(Svd, UnrepresentableSingularValueLeavesAllOutputsUnchanged)
{
    Matrix<TypeParam> input(2, 1);
    if constexpr (TypeParam::bits == 0) {
        std::fill(input.data.begin(), input.data.end(),
                  std::numeric_limits<typename TypeParam::Raw>::max());
    } else {
        const auto raw = static_cast<typename TypeParam::Raw>(
            (INT64_C(1) << (TypeParam::bits - 1)) +
            (INT64_C(1) << (TypeParam::bits - 2)));
        std::fill(input.data.begin(), input.data.end(), raw);
        input.matrix.exponent = LINEARS_EXPONENT_MAX;
    }
    Matrix<TypeParam> u(2, 1), s(1, 1), v(1, 1);
    Scratch<TypeParam> scratch(LINEARS_SVD_WORKSPACE(2, 1));
    u.sentinel(); s.sentinel(); v.sentinel();
    EXPECT_EQ(TypeParam::svd(&input.matrix, &u.matrix, &s.matrix, &v.matrix,
                             0, scratch.workspace), LINEARS_ERROR_RANGE);
    expect_sentinel(u); expect_sentinel(s); expect_sentinel(v);
    scratch.check();
}

TYPED_TEST(Svd, WorkspaceCapacityAndAlignmentAreChecked)
{
    Matrix<TypeParam> input(random_matrix(3, 2, 62));
    Matrix<TypeParam> u(3, 2), s(2, 1), v(2, 2);
    const size_t required = LINEARS_SVD_WORKSPACE(3, 2);
    Scratch<TypeParam> scratch(required);
    auto workspace = scratch.workspace;
    u.sentinel(); s.sentinel(); v.sentinel();
    --workspace.capacity;
    EXPECT_EQ(TypeParam::svd(&input.matrix, &u.matrix, &s.matrix, &v.matrix,
                             32, workspace), LINEARS_ERROR_WORKSPACE);
    workspace = {nullptr, required};
    EXPECT_EQ(TypeParam::svd(&input.matrix, &u.matrix, &s.matrix, &v.matrix,
                             32, workspace), LINEARS_ERROR_WORKSPACE);
    std::vector<unsigned char> unaligned(required * sizeof(typename TypeParam::Work) + 1);
    workspace = {reinterpret_cast<typename TypeParam::Work *>(unaligned.data() + 1), required};
    EXPECT_EQ(TypeParam::svd(&input.matrix, &u.matrix, &s.matrix, &v.matrix,
                             32, workspace), LINEARS_ERROR_WORKSPACE);
    expect_sentinel(u); expect_sentinel(s); expect_sentinel(v);
    scratch.check();
}

TYPED_TEST(Svd, RejectsInputOutputWorkspaceAndDescriptorAliases)
{
    Matrix<TypeParam> input(random_matrix(3, 3, 98));
    Matrix<TypeParam> u(3, 3), s(3, 1), v(3, 3);
    Scratch<TypeParam> scratch(LINEARS_SVD_WORKSPACE(3, 3));
    u.sentinel(); s.sentinel(); v.sentinel();
    linears_matrix_t bad_u = u.matrix;
    bad_u.data = input.matrix.data;
    EXPECT_EQ(TypeParam::svd(&input.matrix, &bad_u, &s.matrix, &v.matrix,
                             32, scratch.workspace), LINEARS_ERROR_ALIAS);
    linears_matrix_t bad_v = v.matrix;
    bad_v.data = u.matrix.data;
    EXPECT_EQ(TypeParam::svd(&input.matrix, &u.matrix, &s.matrix, &bad_v,
                             32, scratch.workspace), LINEARS_ERROR_ALIAS);
    linears_matrix_t bad_input = input.matrix;
    bad_input.data = scratch.workspace.data;
    EXPECT_EQ(TypeParam::svd(&bad_input, &u.matrix, &s.matrix, &v.matrix,
                             32, scratch.workspace), LINEARS_ERROR_ALIAS);
    EXPECT_EQ(TypeParam::svd(&input.matrix, &u.matrix, &s.matrix, &u.matrix,
                             32, scratch.workspace), LINEARS_ERROR_ALIAS);
    expect_sentinel(u); expect_sentinel(s); expect_sentinel(v);
    scratch.check();
}

TYPED_TEST(Svd, RejectsWrongShapesNullAndMisalignedDescriptorsOrData)
{
    Matrix<TypeParam> input(random_matrix(3, 2, 32));
    Matrix<TypeParam> u(3, 2), s(2, 1), v(2, 2);
    Scratch<TypeParam> scratch(LINEARS_SVD_WORKSPACE(3, 2));
    u.sentinel(); s.sentinel(); v.sentinel();
    EXPECT_EQ(TypeParam::svd(nullptr, &u.matrix, &s.matrix, &v.matrix,
                             32, scratch.workspace), LINEARS_ERROR_ARGUMENT);
    alignas(linears_matrix_t) std::array<unsigned char, sizeof(linears_matrix_t) + 1> bytes{};
    const auto *misaligned = reinterpret_cast<const linears_matrix_t *>(bytes.data() + 1);
    EXPECT_EQ(TypeParam::svd(misaligned, &u.matrix, &s.matrix, &v.matrix,
                             32, scratch.workspace), LINEARS_ERROR_ARGUMENT);
    linears_matrix_t bad_s = s.matrix;
    bad_s.cols = 2;
    EXPECT_EQ(TypeParam::svd(&input.matrix, &u.matrix, &bad_s, &v.matrix,
                             32, scratch.workspace), LINEARS_ERROR_DIMENSION);
    std::vector<unsigned char> raw(input.data.size() * sizeof(typename TypeParam::Raw) + 1);
    linears_matrix_t bad_input = input.matrix;
    bad_input.data = raw.data() + 1;
    EXPECT_EQ(TypeParam::svd(&bad_input, &u.matrix, &s.matrix, &v.matrix,
                             32, scratch.workspace), LINEARS_ERROR_ARGUMENT);
    expect_sentinel(u); expect_sentinel(s); expect_sentinel(v);
}

TYPED_TEST(Svd, OutputAndUnusedWorkspaceGuardsRemainIntact)
{
    using Raw = typename TypeParam::Raw;
    using Work = typename TypeParam::Work;
    Matrix<TypeParam> input(random_matrix(4, 3, 65));
    std::vector<Raw> u(14, Raw{37}), s(5, Raw{37}), v(11, Raw{37});
    linears_matrix_t um{u.data() + 1, 4, 3, 0};
    linears_matrix_t sm{s.data() + 1, 3, 1, 0};
    linears_matrix_t vm{v.data() + 1, 3, 3, 0};
    const size_t required = LINEARS_SVD_WORKSPACE(4, 3);
    Scratch<TypeParam> scratch(required + 7);
    const std::vector<Work> unused(scratch.data.begin() + 1 + required,
                                   scratch.data.end() - 1);
    ASSERT_EQ(TypeParam::svd(&input.matrix, &um, &sm, &vm, 64, scratch.workspace), LINEARS_OK);
    EXPECT_EQ(u.front(), Raw{37}); EXPECT_EQ(u.back(), Raw{37});
    EXPECT_EQ(s.front(), Raw{37}); EXPECT_EQ(s.back(), Raw{37});
    EXPECT_EQ(v.front(), Raw{37}); EXPECT_EQ(v.back(), Raw{37});
    EXPECT_EQ(std::memcmp(unused.data(), scratch.workspace.data + required,
                          unused.size() * sizeof(Work)), 0);
    scratch.check();
}

TYPED_TEST(Svd, IndependentConcurrentCallsAreReentrant)
{
    Matrix<TypeParam> a(random_matrix(5, 3, 214));
    Matrix<TypeParam> b(random_matrix(3, 5, 215));
    Matrix<TypeParam> ua(5, 3), sa(3, 1), va(3, 3);
    Matrix<TypeParam> ub(3, 3), sb(3, 1), vb(5, 3);
    Scratch<TypeParam> wa(LINEARS_SVD_WORKSPACE(5, 3));
    Scratch<TypeParam> wb(LINEARS_SVD_WORKSPACE(3, 5));
    linears_status_t result_a = LINEARS_OK, result_b = LINEARS_OK;
    std::thread first([&] {
        for (size_t i = 0; i < 12 && result_a == LINEARS_OK; ++i)
            result_a = TypeParam::svd(&a.matrix, &ua.matrix, &sa.matrix, &va.matrix, 64, wa.workspace);
    });
    std::thread second([&] {
        for (size_t i = 0; i < 12 && result_b == LINEARS_OK; ++i)
            result_b = TypeParam::svd(&b.matrix, &ub.matrix, &sb.matrix, &vb.matrix, 64, wb.workspace);
    });
    first.join(); second.join();
    ASSERT_EQ(result_a, LINEARS_OK); ASSERT_EQ(result_b, LINEARS_OK);
    check_factors(a.decoded(), ua, sa, va);
    check_factors(b.decoded(), ub, sb, vb);
    wa.check(); wb.check();
}

TYPED_TEST(Svd, OverflowingShapesAreRejectedBeforeDataAccess)
{
    Matrix<TypeParam> input(1, 1), u(1, 1), s(2, 1), v(2, 2);
    input.matrix.rows = u.matrix.rows = SIZE_MAX;
    input.matrix.cols = u.matrix.cols = 2;
    typename TypeParam::Workspace workspace{nullptr, 0};
    EXPECT_EQ(TypeParam::svd(&input.matrix, &u.matrix, &s.matrix, &v.matrix,
                             0, workspace), LINEARS_ERROR_SIZE);
}

TEST(SvdQ15, FinalBlockQuantizationFailureIsAtomicAcrossAllOutputs)
{
    Matrix<Q15> input(2, 1), u(2, 1), s(1, 1), v(1, 1);
    input.data[0] = 32767;
    input.data[1] = 255;
    input.matrix.exponent = LINEARS_EXPONENT_MAX;
    // The norm is below 2^MAX and fits the working scalar, but rounds to 32768
    // at q15 exponent MAX. Selecting a larger block exponent must fail.
    Scratch<Q15> scratch(LINEARS_SVD_WORKSPACE(2, 1));
    u.sentinel(); s.sentinel(); v.sentinel();
    EXPECT_EQ(Q15::svd(&input.matrix, &u.matrix, &s.matrix, &v.matrix,
                       0, scratch.workspace), LINEARS_ERROR_RANGE);
    expect_sentinel(u); expect_sentinel(s); expect_sentinel(v);
    scratch.check();
}

TEST(SvdWorkspace, FormulaAndOverflowAreConstantExpressions)
{
    static_assert(LINEARS_SVD_WORKSPACE(5, 3) == 27, "thin SVD workspace formula");
    static_assert(LINEARS_SVD_WORKSPACE(3, 5) == 27, "wide SVD workspace formula");
    static_assert(LINEARS_SVD_WORKSPACE(0, SIZE_MAX) == 0, "empty SVD workspace");
    static_assert(LINEARS_SVD_WORKSPACE(SIZE_MAX, 2) == SIZE_MAX, "overflow sentinel");
    EXPECT_EQ(LINEARS_SVD_WORKSPACE(1, 1), size_t{3});
}
} // namespace
