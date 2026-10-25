#include "decomp_support.h"
#include <array>
#include <cstring>
#include <thread>

namespace {
using namespace decomp_test;

/* The exact input is Matrix::decoded(), never the pre-quantization fixture. */
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
    const double qu = quantization_bound(u);
    const double qs = quantization_bound(s);
    const double qv = quantization_bound(v);
    // Conservative product perturbation bound, using Frobenius submultiplicativity.
    const double quantized_product = qu * sd.norm() * vd.norm() +
        (ud.norm() + qu) * qs * vd.norm() +
        (ud.norm() + qu) * (sd.norm() + qs) * qv;
    // Dot products/rotations incur dimension-scaled working roundoff. Truncation
    // is a separate explicit budget, not hidden in the storage quantization term.
    const double working = 32.0 * dimension * static_cast<double>(r) *
                           Format::working_epsilon;
    // Restoring the original input magnitude rounds each fixed singular value
    // once on the absolute F31 grid, even when the represented input is tiny.
    const double restored_rounding = Format::bits == 0 ? 0.0 :
        0.5 * std::sqrt(static_cast<double>(r)) * Format::working_epsilon;
    const Eigen::MatrixXd reconstructed = ud * sd.col(0).asDiagonal() * vd.transpose();
    EXPECT_LE((reconstructed - input).norm(),
              working * input.norm() + truncation_bound + quantized_product +
              (ud.norm() + qu) * restored_rounding * (vd.norm() + qv));
    const Eigen::MatrixXd identity = Eigen::MatrixXd::Identity(r, r);
    EXPECT_LE((ud.transpose() * ud - identity).norm(),
              working + 2.0 * ud.norm() * qu + qu * qu);
    EXPECT_LE((vd.transpose() * vd - identity).norm(),
              working + 2.0 * vd.norm() * qv + qv * qv);
    const Eigen::JacobiSVD<Eigen::MatrixXd> reference(input);
    EXPECT_LE((sd.col(0) - reference.singularValues()).norm(),
              working * input.norm() + truncation_bound + qs + restored_rounding);
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
    const Eigen::MatrixXd represented = input.decoded();
    ASSERT_EQ(Format::svd(&input.matrix, &u.matrix, &s.matrix, &v.matrix,
                          sweeps, scratch.workspace), LINEARS_OK);
    EXPECT_EQ(input.data, original);
    const double tau = 8.0 * static_cast<double>(std::max(m, n)) * Format::working_epsilon;
    const double loss = allow_truncation ? static_cast<double>(r) * tau * represented.norm() : 0.0;
    check_factors(represented, u, s, v, loss);
    scratch.check();
}

template<class Format>
void expect_sentinel(const Matrix<Format> &matrix)
{
    for (auto value : matrix.data) EXPECT_EQ(value, static_cast<typename Format::Raw>(37));
}

template<class Format> class Svd : public ::testing::Test {};
TYPED_TEST_SUITE(Svd, Formats);

TYPED_TEST(Svd, RandomTallWideSquareAndOneDimensional)
{
    const std::array<std::array<size_t, 2>, 12> shapes = {{{1, 1}, {1, 5}, {5, 1},
        {2, 2}, {3, 5}, {5, 3}, {8, 8}, {11, 6}, {6, 11}, {19, 17}, {71, 3}, {3, 71}}};
    for (const auto &shape : shapes) {
        SCOPED_TRACE(::testing::Message() << shape[0] << "x" << shape[1]);
        for (uint32_t seed = 1; seed <= 3; ++seed) {
            Eigen::MatrixXd input = random_matrix(shape[0], shape[1], seed);
            if constexpr (TypeParam::bits != 0) {
                // Change the actual input so ||A||_F <= 1/2 implies s_max <= 1/2.
                input *= 0.5 / std::sqrt(static_cast<double>(shape[0] * shape[1]));
            }
            factor_and_check<TypeParam>(input);
        }
    }
}

TYPED_TEST(Svd, RepeatedSingularValuesNeedNoParticularVectorBasis)
{
    Eigen::MatrixXd input(5, 3);
    input << 1, 0, 0, 0, 1, 0, 0, 0, 0.5, 1, 0, 0, 0, 1, 0;
    if constexpr (TypeParam::bits != 0) input *= 0.25;
    factor_and_check<TypeParam>(input, 0);
    factor_and_check<TypeParam>(input.transpose(), 0);
    // Binary-exact Hadamard factors preserve repeated values after q import.
    Eigen::MatrixXd h(4, 4);
    h << 1, 1, 1, 1, 1, -1, 1, -1, 1, 1, -1, -1, 1, -1, -1, 1;
    h *= 0.5;
    Eigen::Vector4d spectrum(2.0, 2.0, 0.5, 0.0);
    if constexpr (TypeParam::bits != 0) spectrum *= 0.25;
    factor_and_check<TypeParam>(h * spectrum.asDiagonal() * h.transpose(), 64, 0, true);
}

TYPED_TEST(Svd, RankDeficiencyCompletesOrthogonalColumns)
{
    for (const auto &shape : {std::pair<size_t, size_t>{5, 3}, {3, 5}, {5, 5}})
        factor_and_check<TypeParam>(Eigen::MatrixXd::Zero(shape.first, shape.second), 0);
    Eigen::MatrixXd rank_one(5, 3);
    rank_one << 1, -2, 3, 2, -4, 6, -1, 2, -3, 0, 0, 0, 1, -2, 3;
    if constexpr (TypeParam::bits != 0) rank_one *= 0.0625;
    factor_and_check<TypeParam>(rank_one, 64, 0, true);
    factor_and_check<TypeParam>(rank_one.transpose(), 64, 0, true);
    Eigen::MatrixXd rank_two = Eigen::MatrixXd::Zero(5, 4);
    rank_two.col(0) << 1, -1, 2, 0, 1;
    rank_two.col(1) << 0, 2, 1, 1, -1;
    rank_two.col(2) = rank_two.col(0) + rank_two.col(1);
    if constexpr (TypeParam::bits != 0) rank_two *= 0.125;
    factor_and_check<TypeParam>(rank_two, 64, 0, true);
    Eigen::MatrixXd near_coordinate = Eigen::MatrixXd::Zero(5, 3);
    near_coordinate(0, 0) = 1.0;
    near_coordinate(1, 0) = 0x1p-20;
    if constexpr (TypeParam::bits != 0) near_coordinate *= 0.5;
    factor_and_check<TypeParam>(near_coordinate, 0);
}

TYPED_TEST(Svd, EmptyShapesAllowNullStorageAndWorkspace)
{
    for (const auto &shape : {std::pair<size_t, size_t>{0, 0}, {0, 5}, {5, 0},
                              {0, SIZE_MAX}, {SIZE_MAX, 0}}) {
        Matrix<TypeParam> input(shape.first, shape.second);
        Matrix<TypeParam> u(shape.first, 0), s(0, 1), v(shape.second, 0);
        typename TypeParam::Workspace workspace{nullptr, 0};
        ASSERT_EQ(TypeParam::svd(&input.matrix, &u.matrix, &s.matrix, &v.matrix,
                                 0, workspace), LINEARS_OK);
        EXPECT_TRUE(u.data.empty());
        EXPECT_TRUE(s.data.empty());
        EXPECT_TRUE(v.data.empty());
        EXPECT_EQ(LINEARS_SVD_WORKSPACE(shape.first, shape.second), size_t{0});
    }
}

TYPED_TEST(Svd, OneSweepSolvesOneColumnPairAndHugeBudgetIsAccepted)
{
    Eigen::MatrixXd input(3, 2);
    input << 1, 1, 0, 1, 2, 1;
    if constexpr (TypeParam::bits != 0) input *= 0.25;
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
    if constexpr (TypeParam::bits != 0) diagonal *= 0.25;
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
    if constexpr (TypeParam::bits == 0) {
        factor_and_check<TypeParam>(input, 64, -60);
        factor_and_check<TypeParam>(input, 64, 60);
    } else {
        // Both factors change actual values before quantization; there is no
        // scale metadata and no rescaling of the decoded Eigen reference.
        factor_and_check<TypeParam>(input, 64, 4 - TypeParam::bits);
        factor_and_check<TypeParam>(input, 64, -4);
    }
}

TYPED_TEST(Svd, SmallestStoredQuantumRetainsOrthogonalDirections)
{
    if constexpr (TypeParam::bits == 0) {
        GTEST_SKIP() << "Raw fixed-point quantum contract only";
    } else {
        for (size_t columns : {size_t{1}, size_t{2}}) {
            Matrix<TypeParam> input(2, columns), u(2, columns), s(columns, 1), v(columns, columns);
            input.data[0] = 1;
            input.data[1] = 1;
            if (columns == 2) { input.data[2] = 1; input.data[3] = -1; }
            Scratch<TypeParam> work(LINEARS_SVD_WORKSPACE(2, columns));
            ASSERT_EQ(TypeParam::svd(&input.matrix, &u.matrix, &s.matrix, &v.matrix,
                                     0, work.workspace), LINEARS_OK);
            check_factors(input.decoded(), u, s, v);
            EXPECT_GT(s.data[0], 0);
            const double component = std::sqrt(0.5);
            EXPECT_NEAR(std::abs(u.decoded()(0, 0)), component,
                        4.0 * (TypeParam::storage_epsilon + TypeParam::working_epsilon));
            work.check();
        }
    }
}

TYPED_TEST(Svd, SmallNonzeroSingularValueCanRoundToZero)
{
    if constexpr (TypeParam::bits == 0) {
        Eigen::MatrixXd input(2, 2);
        input << 1, 1, 1, 2;
        factor_and_check<TypeParam>(input);
    } else {
        Matrix<TypeParam> input(2, 2), u(2, 2), s(2, 1), v(2, 2);
        input.data = {1, 1, 1, 2};
        input.matrix.data = input.data.data();
        Scratch<TypeParam> work(LINEARS_SVD_WORKSPACE(2, 2));
        ASSERT_EQ(TypeParam::svd(&input.matrix, &u.matrix, &s.matrix, &v.matrix,
                                 64, work.workspace), LINEARS_OK);
        const Eigen::JacobiSVD<Eigen::MatrixXd> reference(input.decoded());
        EXPECT_GT(reference.singularValues()(1), 0.0);
        EXPECT_EQ(s.data[1], 0);
        EXPECT_GT(s.data[0], 0);
        check_factors(input.decoded(), u, s, v);
        work.check();
    }
}

TYPED_TEST(Svd, RejectsNonfiniteOrAcceptsFixedStorageExtrema)
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
        using Raw = typename TypeParam::Raw;
        for (Raw value : {std::numeric_limits<Raw>::min(), std::numeric_limits<Raw>::max(),
                          Raw{1}, Raw{-1}}) {
            input.data[0] = value;
            ASSERT_EQ(TypeParam::svd(&input.matrix, &u.matrix, &s.matrix, &v.matrix,
                                     0, scratch.workspace), LINEARS_OK);
            const Eigen::MatrixXd represented = input.decoded();
            const double rank_bound = 2.0 * 16.0 * TypeParam::working_epsilon * represented.norm();
            check_factors(represented, u, s, v, rank_bound);
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
    }
    Matrix<TypeParam> u(2, 1), s(1, 1), v(1, 1);
    Scratch<TypeParam> scratch(LINEARS_SVD_WORKSPACE(2, 1));
    u.sentinel(); s.sentinel(); v.sentinel();
    EXPECT_EQ(TypeParam::svd(&input.matrix, &u.matrix, &s.matrix, &v.matrix,
                             0, scratch.workspace), LINEARS_ERROR_RANGE);
    expect_sentinel(u); expect_sentinel(s); expect_sentinel(v);
    if constexpr (TypeParam::bits == 0) {
        std::fill(input.data.begin(), input.data.end(), typename TypeParam::Raw{0.75});
        ASSERT_EQ(TypeParam::svd(&input.matrix, &u.matrix, &s.matrix, &v.matrix,
                                 0, scratch.workspace), LINEARS_OK);
        EXPECT_GT(s.data[0], 1);
        check_factors(input.decoded(), u, s, v);
    }
    scratch.check();
}

TYPED_TEST(Svd, ScalarSignsAndUnitEndpointsAreExactStorageContracts)
{
    using Raw = typename TypeParam::Raw;
    Matrix<TypeParam> input(1, 1), u(1, 1), s(1, 1), v(1, 1);
    Scratch<TypeParam> scratch(LINEARS_SVD_WORKSPACE(1, 1));
    if constexpr (TypeParam::bits == 0) {
        for (Raw value : {Raw{-1}, Raw{0}, Raw{1}}) {
            input.data[0] = value;
            ASSERT_EQ(TypeParam::svd(&input.matrix, &u.matrix, &s.matrix, &v.matrix,
                                     0, scratch.workspace), LINEARS_OK);
            EXPECT_EQ(s.data[0], std::abs(value));
            EXPECT_EQ(u.data[0], value < 0 ? Raw{-1} : Raw{1});
            EXPECT_EQ(v.data[0], Raw{1});
        }
    } else {
        for (Raw value : {std::numeric_limits<Raw>::min(), std::numeric_limits<Raw>::max(),
                          Raw{-1}, Raw{0}, Raw{1}}) {
            input.data[0] = value;
            ASSERT_EQ(TypeParam::svd(&input.matrix, &u.matrix, &s.matrix, &v.matrix,
                                     0, scratch.workspace), LINEARS_OK);
            const Raw magnitude = value == std::numeric_limits<Raw>::min() ?
                std::numeric_limits<Raw>::max() : static_cast<Raw>(std::abs(static_cast<int64_t>(value)));
            EXPECT_EQ(s.data[0], magnitude);
            EXPECT_EQ(u.data[0], value < 0 ? std::numeric_limits<Raw>::min() :
                                           std::numeric_limits<Raw>::max());
            EXPECT_EQ(v.data[0], std::numeric_limits<Raw>::max());
        }
    }
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
    Eigen::MatrixXd values = random_matrix(4, 3, 65);
    if constexpr (TypeParam::bits != 0) values *= 0.125;
    Matrix<TypeParam> input(values);
    std::vector<Raw> u(14, Raw{37}), s(5, Raw{37}), v(11, Raw{37});
    linears_matrix_t um{u.data() + 1, 4, 3};
    linears_matrix_t sm{s.data() + 1, 3, 1};
    linears_matrix_t vm{v.data() + 1, 3, 3};
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
    Eigen::MatrixXd values_a = random_matrix(5, 3, 214);
    Eigen::MatrixXd values_b = random_matrix(3, 5, 215);
    if constexpr (TypeParam::bits != 0) { values_a *= 0.125; values_b *= 0.125; }
    Matrix<TypeParam> a(values_a);
    Matrix<TypeParam> b(values_b);
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

TEST(SvdQ15, PositiveRoundingCarryUsesTheDocumentedEndpoint)
{
    Matrix<Q15> input(2, 1), u(2, 1), s(1, 1), v(1, 1);
    input.data[0] = 32767;
    input.data[1] = 255;
    // The true norm is below one but rounds upward to 32768 at F15. The
    // endpoint policy must encode MAX rather than inventing a scale field.
    Scratch<Q15> scratch(LINEARS_SVD_WORKSPACE(2, 1));
    ASSERT_EQ(Q15::svd(&input.matrix, &u.matrix, &s.matrix, &v.matrix,
                       0, scratch.workspace), LINEARS_OK);
    EXPECT_EQ(s.data[0], INT16_MAX);
    check_factors(input.decoded(), u, s, v);
    scratch.check();
}

TEST(SvdQ31, DomainCheckUsesRoundedWorkingValues)
{
    Matrix<Q31> input(2, 1), u(2, 1), s(1, 1), v(1, 1);
    input.data[0] = INT32_MIN;
    input.data[1] = 1;
    // The mathematical norm sqrt(1 + 2^-62) exceeds one, but its rounded F31
    // value is exactly one. The API checks that working value, not an interval
    // certificate for the unrounded mathematical SVD.
    Scratch<Q31> scratch(LINEARS_SVD_WORKSPACE(2, 1));
    ASSERT_EQ(Q31::svd(&input.matrix, &u.matrix, &s.matrix, &v.matrix,
                       0, scratch.workspace), LINEARS_OK);
    EXPECT_EQ(s.data[0], INT32_MAX);
    check_factors(input.decoded(), u, s, v);
    scratch.check();
}

TEST(SvdQ31, UnresolvableFixedRotationReportsPrecisionStagnation)
{
    Matrix<Q31> input(2, 2), u(2, 2), s(2, 1), v(2, 2);
    input.data = {INT32_C(1073741824), 16, INT32_C(805306368), -21};
    input.matrix.data = input.data.data();
    Scratch<Q31> scratch(LINEARS_SVD_WORKSPACE(2, 2));
    u.sentinel(); s.sentinel(); v.sentinel();
    // The second column is above the rank cutoff, but the required sine is
    // below a working F31 quantum. A huge budget must not spin on a zero step.
    EXPECT_EQ(Q31::svd(&input.matrix, &u.matrix, &s.matrix, &v.matrix,
                       SIZE_MAX, scratch.workspace), LINEARS_ERROR_NO_CONVERGENCE);
    expect_sentinel(u); expect_sentinel(s); expect_sentinel(v);
    scratch.check();
}

TEST(SvdQ31, NonzeroRotationCanStillRoundToUnchangedColumns)
{
    Matrix<Q31> input(3, 3), u(3, 3), s(3, 1), v(3, 3);
    input.data = {0, 128, 126, 0, 127, -127, INT32_MIN, 0, 0};
    input.matrix.data = input.data.data();
    Scratch<Q31> scratch(LINEARS_SVD_WORKSPACE(3, 3));
    u.sentinel(); s.sentinel(); v.sentinel();
    // The small column norms round to 180/179 work quanta, above the 24-quantum
    // rank cutoff. Their nonzero rotation still changes no F31 column entry.
    EXPECT_EQ(Q31::svd(&input.matrix, &u.matrix, &s.matrix, &v.matrix,
                       SIZE_MAX, scratch.workspace), LINEARS_ERROR_NO_CONVERGENCE);
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
