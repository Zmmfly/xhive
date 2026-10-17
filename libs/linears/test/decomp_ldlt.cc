#include "decomp_support.h"
#include <array>
#include <thread>

namespace {

using namespace decomp_test;

/** @brief Independent caller-owned factorization storage with metadata guards. */
template<class Tag> struct Factorization {
    Matrix<Tag> a;
    Matrix<Tag> l;
    Matrix<Tag> d;
    Scratch<Tag> scratch;
    std::vector<size_t> permutation;
    std::vector<uint8_t> blocks;
    size_t n;

    explicit Factorization(const Eigen::MatrixXd &input, int scale = 0)
        : a(input, scale), l(static_cast<size_t>(input.rows()), static_cast<size_t>(input.rows())),
          d(static_cast<size_t>(input.rows()), static_cast<size_t>(input.rows())),
          scratch(LINEARS_LDLT_WORKSPACE(static_cast<size_t>(input.rows()))),
          permutation(static_cast<size_t>(input.rows()) + 2, SIZE_MAX),
          blocks(static_cast<size_t>(input.rows()) + 2, 79), n(static_cast<size_t>(input.rows()))
    {
        l.sentinel();
        d.sentinel();
    }

    linears_status_t run()
    {
        return Tag::ldlt(&a.matrix, &l.matrix, &d.matrix, permutation.data() + 1, n,
                         blocks.data() + 1, n, scratch.workspace);
    }

    void check_guards() const
    {
        scratch.check();
        EXPECT_EQ(permutation.front(), SIZE_MAX);
        EXPECT_EQ(permutation.back(), SIZE_MAX);
        EXPECT_EQ(blocks.front(), 79);
        EXPECT_EQ(blocks.back(), 79);
    }

    void check_unchanged() const
    {
        for (auto value : l.data) EXPECT_EQ(value, static_cast<typename Tag::Raw>(37));
        for (auto value : d.data) EXPECT_EQ(value, static_cast<typename Tag::Raw>(37));
        EXPECT_EQ(l.matrix.exponent, 17);
        EXPECT_EQ(d.matrix.exponent, 17);
        check_guards();
    }

    void check_metadata_unchanged() const
    {
        for (auto value : permutation) EXPECT_EQ(value, SIZE_MAX);
        for (auto value : blocks) EXPECT_EQ(value, 79);
    }

    /**
     * @brief Verify structure and reconstruction of the represented input.
     *
     * The working-error term is gamma_(32n) times |L| |D| |L^T|.
     * Fixed storage errors are bounded separately by half an output quantum
     * on each stored entry and expanded through the triple product. Thus q15
     * does not inherit a dimension-multiplied blanket storage tolerance.
     */
    void check() const
    {
        check_guards();
        ASSERT_GT(n, 0u);
        const Eigen::MatrixXd input = a.decoded();
        const Eigen::MatrixXd lower = l.decoded();
        const Eigen::MatrixXd diagonal = d.decoded();
        Eigen::MatrixXd permuted(input.rows(), input.cols());
        std::vector<bool> seen(n, false);
        for (size_t i = 0; i < n; ++i) {
            ASSERT_LT(permutation[i + 1], n);
            ASSERT_FALSE(seen[permutation[i + 1]]);
            seen[permutation[i + 1]] = true;
        }
        for (size_t i = 0; i < n; ++i)
            for (size_t j = 0; j < n; ++j)
                permuted(static_cast<Eigen::Index>(i), static_cast<Eigen::Index>(j)) =
                    input(static_cast<Eigen::Index>(permutation[i + 1]),
                          static_cast<Eigen::Index>(permutation[j + 1]));

        Eigen::MatrixXd allowed_d = Eigen::MatrixXd::Identity(input.rows(), input.cols());
        Eigen::MatrixXd allowed_l = Eigen::MatrixXd::Zero(input.rows(), input.cols());
        for (size_t i = 0; i < n; ++i) {
            EXPECT_DOUBLE_EQ(lower(i, i), 1.0);
            for (size_t j = 0; j < i; ++j) allowed_l(i, j) = 1.0;
            for (size_t j = i + 1; j < n; ++j) EXPECT_DOUBLE_EQ(lower(i, j), 0.0);
        }
        for (size_t k = 0; k < n;) {
            if (blocks[k + 1] == 2) {
                ASSERT_LT(k + 1, n);
                ASSERT_EQ(blocks[k + 2], 0);
                allowed_d(k, k + 1) = allowed_d(k + 1, k) = 1.0;
                allowed_l(k + 1, k) = 0.0;
                EXPECT_DOUBLE_EQ(lower(k + 1, k), 0.0);
                k += 2;
            } else {
                ASSERT_EQ(blocks[k + 1], 1);
                ++k;
            }
        }
        for (size_t i = 0; i < n; ++i)
            for (size_t j = 0; j < n; ++j) {
                EXPECT_DOUBLE_EQ(diagonal(i, j), diagonal(j, i));
                if (allowed_d(i, j) == 0.0) {
                    EXPECT_DOUBLE_EQ(diagonal(i, j), 0.0);
                }
            }

        const double scale = input.cwiseAbs().maxCoeff();
        ASSERT_GT(scale, 0.0);
        const Eigen::MatrixXd normalized_d = diagonal / scale;
        const Eigen::MatrixXd reconstructed = lower * normalized_d * lower.transpose();
        const Eigen::MatrixXd abs_l = lower.cwiseAbs();
        const Eigen::MatrixXd abs_d = normalized_d.cwiseAbs();
        Eigen::MatrixXd error_l = Eigen::MatrixXd::Zero(input.rows(), input.cols());
        Eigen::MatrixXd error_d = error_l;
        if constexpr (Tag::bits != 0) {
            error_l = allowed_l * std::scalbn(0.5, l.matrix.exponent - Tag::bits);
            error_d = allowed_d * (std::scalbn(0.5, d.matrix.exponent - Tag::bits) / scale);
        }
        const Eigen::MatrixXd enlarged =
            (abs_l + error_l) * (abs_d + error_d) * (abs_l + error_l).transpose();
        const Eigen::MatrixXd original_bound = abs_l * abs_d * abs_l.transpose();
        const double quantization_bound = (enlarged - original_bound).norm();
        const double accumulated = 32.0 * static_cast<double>(n) * Tag::working_epsilon;
        const double working_bound = accumulated / (1.0 - accumulated) * enlarged.norm();
        const double residual = (reconstructed - permuted / scale).norm();
        EXPECT_LE(residual, working_bound + quantization_bound)
            << "n=" << n << ", working=" << working_bound
            << ", storage=" << quantization_bound;
    }
};

template<class Tag> class LinearsLdltNumerics : public ::testing::Test {};
template<class Tag> class LinearsLdltContracts : public ::testing::Test {};
TYPED_TEST_SUITE(LinearsLdltNumerics, Formats);
TYPED_TEST_SUITE(LinearsLdltContracts, Formats);

TYPED_TEST(LinearsLdltNumerics, ZeroDiagonalNeedsTwoByTwoPivot)
{
    Eigen::Matrix2d source;
    source << 0, 1, 1, 0;
    Factorization<TypeParam> result(source);
    const auto original = result.a.data;
    ASSERT_EQ(result.run(), LINEARS_OK);
    EXPECT_EQ(result.blocks[1], 2);
    EXPECT_EQ(result.blocks[2], 0);
    EXPECT_EQ(result.a.data, original);
    EXPECT_EQ(result.l.decoded(), Eigen::Matrix2d::Identity());
    EXPECT_EQ(result.d.decoded(), result.a.decoded());
    result.check();
}

TYPED_TEST(LinearsLdltNumerics, NonleadingPairAndLaterPermutations)
{
    Eigen::Matrix4d source;
    source << 0, .2, .1, .3,
              .2, .1, 0, 4,
              .1, 0, -.1, .25,
              .3, 4, .25, .2;
    Factorization<TypeParam> result(source);
    ASSERT_EQ(result.run(), LINEARS_OK);
    EXPECT_EQ(result.permutation[1], 1u);
    EXPECT_EQ(result.permutation[2], 3u);
    EXPECT_EQ(result.blocks[1], 2);
    result.check();
}

TYPED_TEST(LinearsLdltNumerics, SeededGeneralIndefiniteMatrices)
{
    for (size_t n : {2u, 3u, 5u, 8u, 13u}) {
        for (uint32_t sample = 0; sample < 12; ++sample) {
            SCOPED_TRACE(::testing::Message() << "n=" << n << ", sample=" << sample);
            const Eigen::MatrixXd random = random_matrix(n, n, 0x21b7953u + sample * 197u + n);
            Eigen::MatrixXd source = (0.5 * (random + random.transpose())).eval();
            source(0, 0) = .125;
            source(1, 1) = -.125;
            Factorization<TypeParam> result(source);
            ASSERT_EQ(result.run(), LINEARS_OK);
            result.check();
        }
    }
}

TYPED_TEST(LinearsLdltNumerics, PositiveAndNegativeDefiniteAreSupported)
{
    const Eigen::MatrixXd random = random_matrix(6, 6, 0x9831553u);
    const Eigen::MatrixXd positive = random.transpose() * random + Eigen::MatrixXd::Identity(6, 6);
    for (double sign : {-1.0, 1.0}) {
        Factorization<TypeParam> result(sign * positive);
        ASSERT_EQ(result.run(), LINEARS_OK);
        for (size_t i = 0; i < result.n; ++i) EXPECT_EQ(result.blocks[i + 1], 1);
        result.check();
    }
}

TYPED_TEST(LinearsLdltNumerics, RepeatedValuesAndTiesAreDeterministic)
{
    Eigen::Matrix4d source = Eigen::Matrix4d::Zero();
    source(0, 1) = source(1, 0) = source(2, 3) = source(3, 2) = 2;
    Factorization<TypeParam> first(source), second(source);
    ASSERT_EQ(first.run(), LINEARS_OK);
    ASSERT_EQ(second.run(), LINEARS_OK);
    EXPECT_EQ(first.permutation, second.permutation);
    EXPECT_EQ(first.blocks, second.blocks);
    EXPECT_EQ(first.l.data, second.l.data);
    EXPECT_EQ(first.d.data, second.d.data);
    EXPECT_EQ(first.blocks, (std::vector<uint8_t>{79, 2, 0, 2, 0, 79}));
    EXPECT_EQ(first.permutation, (std::vector<size_t>{SIZE_MAX, 0, 1, 2, 3, SIZE_MAX}));
    first.check();
    second.check();
}

TYPED_TEST(LinearsLdltNumerics, InertiaMatchesIndependentEigenSolver)
{
    Eigen::Matrix4d orthogonal;
    orthogonal << 1, 1, 1, 1, 1, -1, 1, -1, 1, 1, -1, -1, 1, -1, -1, 1;
    orthogonal *= .5;
    Eigen::Vector4d eigenvalues;
    eigenvalues << 4, 1, -2, -3;
    Factorization<TypeParam> result(orthogonal * eigenvalues.asDiagonal() * orthogonal.transpose());
    ASSERT_EQ(result.run(), LINEARS_OK);
    result.check();
    const Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> reference(result.a.decoded());
    const Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> blocks(result.d.decoded());
    ASSERT_EQ(reference.info(), Eigen::Success);
    ASSERT_EQ(blocks.info(), Eigen::Success);
    EXPECT_EQ((reference.eigenvalues().array() > 0).count(), 2);
    EXPECT_EQ((blocks.eigenvalues().array() > 0).count(), 2);
    EXPECT_EQ((blocks.eigenvalues().array() < 0).count(), 2);
}

TYPED_TEST(LinearsLdltNumerics, OneByOneBothSignsAndWideBinaryScales)
{
    for (int scale : {-120, -20, 0, 20, 120}) {
        for (double value : {-.75, .75}) {
            Eigen::Matrix<double, 1, 1> source;
            source << value;
            Factorization<TypeParam> result(source, scale);
            ASSERT_EQ(result.run(), LINEARS_OK);
            EXPECT_DOUBLE_EQ(result.l.decoded()(0, 0), 1);
            EXPECT_EQ(result.d.data, result.a.data);
            EXPECT_EQ(result.d.matrix.exponent, result.a.matrix.exponent);
            result.check();
        }
        Eigen::Matrix3d source;
        source << 0, .75, .25, .75, 0, -.125, .25, -.125, .5;
        Factorization<TypeParam> result(source, scale);
        ASSERT_EQ(result.run(), LINEARS_OK);
        result.check();
    }
}

TYPED_TEST(LinearsLdltNumerics, ExactSingularAndZeroMatricesLeaveFactorsUntouched)
{
    Eigen::Matrix3d diagonal = Eigen::Matrix3d::Zero();
    diagonal(0, 0) = 1;
    diagonal(1, 1) = -1;
    const std::array<Eigen::MatrixXd, 4> sources = {
        Eigen::MatrixXd::Zero(3, 3), Eigen::MatrixXd::Ones(2, 2),
        diagonal, Eigen::MatrixXd::Zero(1, 1)};
    for (const auto &source : sources) {
        Factorization<TypeParam> result(source);
        ASSERT_EQ(result.run(), LINEARS_ERROR_SINGULAR);
        result.check_unchanged();
    }
}

TYPED_TEST(LinearsLdltNumerics, DimensionsAreNotArbitrarilyCappedAtSixtyFour)
{
    Eigen::MatrixXd source = Eigen::MatrixXd::Identity(65, 65);
    for (Eigen::Index i = 1; i < 65; i += 2) source(i, i) = -1;
    Factorization<TypeParam> result(source);
    ASSERT_EQ(result.run(), LINEARS_OK);
    result.check();
}

TYPED_TEST(LinearsLdltNumerics, IndependentCallsAreConcurrentAndReusable)
{
    Eigen::Matrix3d source;
    source << 0, .75, .25, .75, 0, -.125, .25, -.125, .5;
    Factorization<TypeParam> first(source), second(source, 7);
    linears_status_t status_first = LINEARS_OK, status_second = LINEARS_OK;
    std::thread one([&] {
        for (int i = 0; i < 16 && status_first == LINEARS_OK; ++i) status_first = first.run();
    });
    std::thread two([&] {
        for (int i = 0; i < 16 && status_second == LINEARS_OK; ++i) status_second = second.run();
    });
    one.join();
    two.join();
    ASSERT_EQ(status_first, LINEARS_OK);
    ASSERT_EQ(status_second, LINEARS_OK);
    first.check();
    second.check();
    EXPECT_EQ(first.permutation, second.permutation);
    EXPECT_EQ(first.blocks, second.blocks);
    EXPECT_EQ(first.l.data, second.l.data);
}

TYPED_TEST(LinearsLdltContracts, EmptyMatrixNeedsNoDataOrScratch)
{
    Matrix<TypeParam> a(0, 0), l(0, 0), d(0, 0);
    l.sentinel();
    d.sentinel();
    const typename TypeParam::Workspace workspace{nullptr, 0};
    ASSERT_EQ(TypeParam::ldlt(&a.matrix, &l.matrix, &d.matrix, nullptr, 0,
                              nullptr, 0, workspace), LINEARS_OK);
    EXPECT_EQ(l.matrix.exponent, 0);
    EXPECT_EQ(d.matrix.exponent, 0);
    EXPECT_EQ(LINEARS_LDLT_WORKSPACE(0), 0u);
}

TYPED_TEST(LinearsLdltContracts, BothInputTrianglesMustAgreeExactly)
{
    Eigen::Matrix2d source;
    source << 2, .25, .5, -1;
    Factorization<TypeParam> result(source);
    ASSERT_EQ(result.run(), LINEARS_ERROR_DIMENSION);
    result.check_unchanged();
    result.check_metadata_unchanged();
}

TYPED_TEST(LinearsLdltContracts, WrongShapesAndNullDescriptors)
{
    Factorization<TypeParam> result(Eigen::Matrix2d::Identity());
    result.l.matrix.rows = 1;
    ASSERT_EQ(result.run(), LINEARS_ERROR_DIMENSION);
    result.l.matrix.rows = 2;
    auto nonsquare = result.a.matrix;
    nonsquare.cols = 1;
    EXPECT_EQ(TypeParam::ldlt(&nonsquare, &result.l.matrix, &result.d.matrix,
        result.permutation.data() + 1, 2, result.blocks.data() + 1, 2,
        result.scratch.workspace), LINEARS_ERROR_DIMENSION);
    EXPECT_EQ(TypeParam::ldlt(nullptr, &result.l.matrix, &result.d.matrix,
        result.permutation.data() + 1, 2, result.blocks.data() + 1, 2,
        result.scratch.workspace), LINEARS_ERROR_ARGUMENT);
    result.check_unchanged();
    result.check_metadata_unchanged();
}

TYPED_TEST(LinearsLdltContracts, ShortScratchAndMetadataKeepSentinels)
{
    Factorization<TypeParam> result(Eigen::Matrix3d::Identity());
    Scratch<TypeParam> short_scratch(LINEARS_LDLT_WORKSPACE(3) - 1);
    EXPECT_EQ(TypeParam::ldlt(&result.a.matrix, &result.l.matrix, &result.d.matrix,
        result.permutation.data() + 1, 3, result.blocks.data() + 1, 3,
        short_scratch.workspace), LINEARS_ERROR_WORKSPACE);
    short_scratch.check();
    EXPECT_EQ(TypeParam::ldlt(&result.a.matrix, &result.l.matrix, &result.d.matrix,
        result.permutation.data() + 1, 2, result.blocks.data() + 1, 3,
        result.scratch.workspace), LINEARS_ERROR_WORKSPACE);
    EXPECT_EQ(TypeParam::ldlt(&result.a.matrix, &result.l.matrix, &result.d.matrix,
        result.permutation.data() + 1, 3, result.blocks.data() + 1, 2,
        result.scratch.workspace), LINEARS_ERROR_WORKSPACE);
    EXPECT_EQ(TypeParam::ldlt(&result.a.matrix, &result.l.matrix, &result.d.matrix,
        nullptr, 3, result.blocks.data() + 1, 3,
        result.scratch.workspace), LINEARS_ERROR_ARGUMENT);
    EXPECT_EQ(TypeParam::ldlt(&result.a.matrix, &result.l.matrix, &result.d.matrix,
        result.permutation.data() + 1, 3, nullptr, 3,
        result.scratch.workspace), LINEARS_ERROR_ARGUMENT);
    result.check_unchanged();
    result.check_metadata_unchanged();
}

TYPED_TEST(LinearsLdltContracts, MatrixAndScratchOverlapsAreRejected)
{
    Factorization<TypeParam> result(Eigen::Matrix3d::Identity());
    const auto original = result.a.data;
    auto output = result.l.matrix;
    output.data = result.a.matrix.data;
    EXPECT_EQ(TypeParam::ldlt(&result.a.matrix, &output, &result.d.matrix,
        result.permutation.data() + 1, 3, result.blocks.data() + 1, 3,
        result.scratch.workspace), LINEARS_ERROR_ALIAS);
    output.data = result.d.matrix.data;
    EXPECT_EQ(TypeParam::ldlt(&result.a.matrix, &output, &result.d.matrix,
        result.permutation.data() + 1, 3, result.blocks.data() + 1, 3,
        result.scratch.workspace), LINEARS_ERROR_ALIAS);
    auto input = result.a.matrix;
    input.data = result.scratch.workspace.data;
    EXPECT_EQ(TypeParam::ldlt(&input, &result.l.matrix, &result.d.matrix,
        result.permutation.data() + 1, 3, result.blocks.data() + 1, 3,
        result.scratch.workspace), LINEARS_ERROR_ALIAS);
    EXPECT_EQ(TypeParam::ldlt(&result.a.matrix, &result.l.matrix, &result.l.matrix,
        result.permutation.data() + 1, 3, result.blocks.data() + 1, 3,
        result.scratch.workspace), LINEARS_ERROR_ALIAS);
    EXPECT_EQ(result.a.data, original);
    result.check_unchanged();
    result.check_metadata_unchanged();
}

TYPED_TEST(LinearsLdltContracts, MetadataCannotOverlapAnyOtherStorage)
{
    Factorization<TypeParam> result(Eigen::Matrix4d::Identity());
    EXPECT_EQ(TypeParam::ldlt(&result.a.matrix, &result.l.matrix, &result.d.matrix,
        result.permutation.data() + 1, 4,
        reinterpret_cast<uint8_t *>(result.permutation.data() + 1), 4,
        result.scratch.workspace), LINEARS_ERROR_ALIAS);
    EXPECT_EQ(TypeParam::ldlt(&result.a.matrix, &result.l.matrix, &result.d.matrix,
        result.permutation.data() + 1, 4, reinterpret_cast<uint8_t *>(result.a.matrix.data), 4,
        result.scratch.workspace), LINEARS_ERROR_ALIAS);
    EXPECT_EQ(TypeParam::ldlt(&result.a.matrix, &result.l.matrix, &result.d.matrix,
        result.permutation.data() + 1, 4, reinterpret_cast<uint8_t *>(&result.l.matrix), 4,
        result.scratch.workspace), LINEARS_ERROR_ALIAS);
    EXPECT_EQ(TypeParam::ldlt(&result.a.matrix, &result.l.matrix, &result.d.matrix,
        result.permutation.data() + 1, 4,
        reinterpret_cast<uint8_t *>(result.scratch.workspace.data), 4,
        result.scratch.workspace), LINEARS_ERROR_ALIAS);
    const uintptr_t start = reinterpret_cast<uintptr_t>(result.scratch.workspace.data);
    const uintptr_t aligned = start + (alignof(size_t) - start % alignof(size_t)) % alignof(size_t);
    EXPECT_EQ(TypeParam::ldlt(&result.a.matrix, &result.l.matrix, &result.d.matrix,
        reinterpret_cast<size_t *>(aligned), 4, result.blocks.data() + 1, 4,
        result.scratch.workspace), LINEARS_ERROR_ALIAS);
    result.check_unchanged();
    result.check_metadata_unchanged();
}

TYPED_TEST(LinearsLdltContracts, MisalignmentIsRejectedBeforeAnyTypedRead)
{
    Factorization<TypeParam> result(Eigen::Matrix2d::Identity());
    auto input = result.a.matrix;
    input.data = reinterpret_cast<unsigned char *>(input.data) + 1;
    EXPECT_EQ(TypeParam::ldlt(&input, &result.l.matrix, &result.d.matrix,
        result.permutation.data() + 1, 2, result.blocks.data() + 1, 2,
        result.scratch.workspace), LINEARS_ERROR_ARGUMENT);
    auto workspace = result.scratch.workspace;
    workspace.data = reinterpret_cast<typename TypeParam::Work *>(
        reinterpret_cast<unsigned char *>(workspace.data) + 1);
    EXPECT_EQ(TypeParam::ldlt(&result.a.matrix, &result.l.matrix, &result.d.matrix,
        result.permutation.data() + 1, 2, result.blocks.data() + 1, 2,
        workspace), LINEARS_ERROR_WORKSPACE);
    alignas(linears_matrix_t) std::array<unsigned char, sizeof(linears_matrix_t) + 1> descriptor{};
    EXPECT_EQ(TypeParam::ldlt(reinterpret_cast<const linears_matrix_t *>(descriptor.data() + 1),
        &result.l.matrix, &result.d.matrix, result.permutation.data() + 1, 2,
        result.blocks.data() + 1, 2, result.scratch.workspace), LINEARS_ERROR_ARGUMENT);
    result.check_unchanged();
    result.check_metadata_unchanged();
}

TYPED_TEST(LinearsLdltContracts, OverflowingWorkspaceFormulaIsRejected)
{
    Factorization<TypeParam> result(Eigen::Matrix2d::Identity());
    auto a = result.a.matrix;
    auto l = result.l.matrix;
    auto d = result.d.matrix;
    a.rows = a.cols = l.rows = l.cols = d.rows = d.cols = SIZE_MAX;
    EXPECT_EQ(LINEARS_LDLT_WORKSPACE(SIZE_MAX), SIZE_MAX);
    EXPECT_EQ(TypeParam::ldlt(&a, &l, &d, nullptr, 0, nullptr, 0,
                              result.scratch.workspace), LINEARS_ERROR_SIZE);
    result.check_unchanged();
    result.check_metadata_unchanged();
}

TYPED_TEST(LinearsLdltContracts, IntermediateRangeFailureDoesNotCommitFactors)
{
    Eigen::Matrix3d source;
    source << 0, .75, .75, .75, 0, -.75, .75, -.75, 0;
    if constexpr (TypeParam::bits == 0)
        source *= static_cast<double>(std::numeric_limits<typename TypeParam::Raw>::max());
    Factorization<TypeParam> result(source);
    if constexpr (TypeParam::bits != 0) result.a.matrix.exponent = LINEARS_EXPONENT_MAX;
    ASSERT_EQ(result.run(), LINEARS_ERROR_RANGE);
    result.check_unchanged();
}

using FloatFormats = ::testing::Types<F32, F64>;
template<class Tag> class LinearsLdltFloating : public ::testing::Test {};
TYPED_TEST_SUITE(LinearsLdltFloating, FloatFormats);

TYPED_TEST(LinearsLdltFloating, NaNAndBothInfinitiesAreRejected)
{
    using Raw = typename TypeParam::Raw;
    for (Raw bad : {std::numeric_limits<Raw>::quiet_NaN(),
                    std::numeric_limits<Raw>::infinity(), -std::numeric_limits<Raw>::infinity()}) {
        Factorization<TypeParam> result(Eigen::Matrix2d::Identity());
        result.a.data[1] = bad;
        ASSERT_EQ(result.run(), LINEARS_ERROR_RANGE);
        result.check_unchanged();
        result.check_metadata_unchanged();
    }
}

TYPED_TEST(LinearsLdltFloating, TwoByTwoPivotDoesNotSquareExtremeMagnitudes)
{
    using Raw = typename TypeParam::Raw;
    for (Raw magnitude : {std::numeric_limits<Raw>::denorm_min(),
                          std::numeric_limits<Raw>::min(),
                          std::numeric_limits<Raw>::max() / static_cast<Raw>(2)}) {
        Eigen::Matrix2d source;
        source << 0, static_cast<double>(magnitude), static_cast<double>(magnitude), 0;
        Factorization<TypeParam> result(source);
        ASSERT_EQ(result.run(), LINEARS_OK);
        EXPECT_EQ(result.blocks[1], 2);
        EXPECT_EQ(result.d.data, result.a.data);
        result.check();
    }
}

TYPED_TEST(LinearsLdltFloating, DescriptorExponentsAreIgnored)
{
    Eigen::Matrix2d source;
    source << 0, 1, 1, 0;
    Factorization<TypeParam> result(source);
    result.a.matrix.exponent = std::numeric_limits<int>::max();
    ASSERT_EQ(result.run(), LINEARS_OK);
    EXPECT_EQ(result.l.matrix.exponent, 0);
    EXPECT_EQ(result.d.matrix.exponent, 0);
    result.check();
}

using FixedFormats = ::testing::Types<Q15, Q31>;
template<class Tag> class LinearsLdltFixed : public ::testing::Test {};
TYPED_TEST_SUITE(LinearsLdltFixed, FixedFormats);

TYPED_TEST(LinearsLdltFixed, ExponentsBeyondFloatingRangeDoNotChangePivotsOrMantissas)
{
    Eigen::Matrix3d source;
    source << 0, .5, .25, .5, 0, -.125, .25, -.125, .375;
    Factorization<TypeParam> reference(source);
    ASSERT_EQ(reference.run(), LINEARS_OK);
    reference.check();
    for (int scale : {-16000, -2000, 2000, 16000}) {
        Factorization<TypeParam> result(source, scale);
        ASSERT_EQ(result.run(), LINEARS_OK) << scale;
        EXPECT_EQ(result.permutation, reference.permutation);
        EXPECT_EQ(result.blocks, reference.blocks);
        EXPECT_EQ(result.l.data, reference.l.data);
        EXPECT_EQ(result.l.matrix.exponent, reference.l.matrix.exponent);
        EXPECT_EQ(result.d.data, reference.d.data);
        EXPECT_EQ(result.d.matrix.exponent, reference.d.matrix.exponent + scale);
        result.check_guards();
    }
}

TYPED_TEST(LinearsLdltFixed, TwoByTwoPivotAtBothWorkingExponentLimits)
{
    Eigen::Matrix2d source;
    source << 0, .5, .5, 0;
    for (int scale : {LINEARS_EXPONENT_MIN, LINEARS_EXPONENT_MAX}) {
        Factorization<TypeParam> result(source, scale);
        ASSERT_EQ(result.run(), LINEARS_OK);
        EXPECT_EQ(result.blocks[1], 2);
        EXPECT_EQ(result.d.data, result.a.data);
        EXPECT_EQ(result.d.matrix.exponent, scale);
        EXPECT_DOUBLE_EQ(result.l.decoded()(0, 0), 1);
        EXPECT_DOUBLE_EQ(result.l.decoded()(1, 1), 1);
        result.check_guards();
    }
}

TYPED_TEST(LinearsLdltFixed, InvalidInputExponentsAreRejectedAtomically)
{
    for (int exponent : {LINEARS_EXPONENT_MIN - 1, LINEARS_EXPONENT_MAX + 1}) {
        Factorization<TypeParam> result(Eigen::Matrix2d::Identity());
        result.a.matrix.exponent = exponent;
        ASSERT_EQ(result.run(), LINEARS_ERROR_RANGE);
        result.check_unchanged();
        result.check_metadata_unchanged();
    }
}

TYPED_TEST(LinearsLdltFixed, FinalBlockQuantizationMayDiscardASmallWorkingPivot)
{
    Factorization<TypeParam> result(Eigen::Matrix3d::Zero());
    using Raw = typename TypeParam::Raw;
    result.a.data[0] = static_cast<Raw>(INT32_C(1) << (TypeParam::bits - 1));
    result.a.data[4] = 8192;
    result.a.data[5] = result.a.data[7] = 8191;
    result.a.data[8] = 8190;
    ASSERT_EQ(result.run(), LINEARS_OK);
    EXPECT_EQ(result.blocks[1], 1);
    EXPECT_EQ(result.blocks[2], 1);
    EXPECT_EQ(result.blocks[3], 1);
    EXPECT_EQ(result.d.data[8], 0);
    result.check();
}

} // namespace
