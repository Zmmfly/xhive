#include "decomp_support.h"

namespace {
using namespace decomp_test;
template<class F> class LinearQr : public ::testing::Test {};
TYPED_TEST_SUITE(LinearQr, Formats);

template<class F> void check_qr(const Eigen::MatrixXd &values, int scale = 0)
{
    const size_t m = static_cast<size_t>(values.rows()), n = static_cast<size_t>(values.cols());
    Matrix<F> a(values, scale), q(m, m), r(m, n);
    Scratch<F> scratch(LINEARS_QR_WORKSPACE(m, n));
    ASSERT_EQ(F::qr(&a.matrix, &q.matrix, &r.matrix, scratch.workspace), LINEARS_OK);
    const Eigen::MatrixXd input = a.decoded(), actual_q = q.decoded(), actual_r = r.decoded();
    EXPECT_LE((actual_q * actual_r - input).norm(), product_bound(q, r, input.norm(), m));
    const double quantization = quantization_bound(q);
    const double orthogonality = 2.0 * actual_q.norm() * quantization + quantization * quantization +
        16.0 * static_cast<double>(std::max<size_t>(m, 1)) * F::working_epsilon * std::sqrt(static_cast<double>(m));
    EXPECT_LE((actual_q.transpose() * actual_q - Eigen::MatrixXd::Identity(m, m)).norm(), orthogonality);
    for (size_t i = 0; i < m; ++i)
        for (size_t j = 0; j < std::min(i, n); ++j) EXPECT_EQ(actual_r(i, j), 0.0);
    if (m != 0 && n != 0) {
        Eigen::HouseholderQR<Eigen::MatrixXd> reference(input);
        // Householder signs are nonunique; compare diagonal magnitudes.
        for (size_t k = 0; k < std::min(m, n); ++k)
            EXPECT_NEAR(std::abs(actual_r(k, k)), std::abs(reference.matrixQR()(k, k)),
                quantization_bound(r) + 16.0 * m * F::working_epsilon *
                (F::bits == 0 ? input.norm() : 1.0));
    }
    scratch.check();
}

TYPED_TEST(LinearQr, TallWideSquareAndScalarMatchEigen)
{
    for (const auto &shape : {std::pair<size_t, size_t>{1, 1}, {1, 7}, {7, 1}, {3, 9}, {9, 3}, {6, 6}, {71, 3}}) {
        Eigen::MatrixXd values = random_matrix(shape.first, shape.second,
            0x84a2u + static_cast<uint32_t>(shape.first * 19 + shape.second));
        if constexpr (TypeParam::bits != 0)
            values *= 0.25 / std::sqrt(static_cast<double>(shape.first * shape.second));
        check_qr<TypeParam>(values);
    }
}

TYPED_TEST(LinearQr, RankDeficientZeroColumnsAndRepeatedColumnsRemainSupported)
{
    Eigen::MatrixXd values = Eigen::MatrixXd::Zero(7, 4);
    values.col(0) << 1, -2, 3, 0, 1, -1, 2;
    values.col(1) = 2.0 * values.col(0);
    values.col(2) = values.col(0);
    if constexpr (TypeParam::bits != 0) values /= 16.0;
    check_qr<TypeParam>(values);
    check_qr<TypeParam>(Eigen::MatrixXd::Zero(4, 7));
}

TYPED_TEST(LinearQr, EmptyShapesAndActualMagnitudeSpans)
{
    check_qr<TypeParam>(Eigen::MatrixXd(0, 0));
    check_qr<TypeParam>(Eigen::MatrixXd(0, 5));
    check_qr<TypeParam>(Eigen::MatrixXd(5, 0));
    const auto values = random_matrix(5, 3, 5517u);
    if constexpr (TypeParam::bits == 0) {
        check_qr<TypeParam>(values, -50);
        check_qr<TypeParam>(values, 50);
    } else {
        check_qr<TypeParam>(values / 16.0);
        check_qr<TypeParam>(values / 16.0, -8);
    }
}

TYPED_TEST(LinearQr, InsufficientWorkspaceAndAliasLeaveAllOutputsUnchanged)
{
    using F = TypeParam;
    Matrix<F> a(random_matrix(4, 3, 712u)), q(4, 4), r(4, 3);
    q.sentinel(); r.sentinel();
    const auto old_q = q.data, old_r = r.data;
    Scratch<F> short_scratch(LINEARS_QR_WORKSPACE(4, 3) - 1);
    EXPECT_EQ(F::qr(&a.matrix, &q.matrix, &r.matrix, short_scratch.workspace), LINEARS_ERROR_WORKSPACE);
    Scratch<F> scratch(LINEARS_QR_WORKSPACE(4, 3));
    linears_matrix_t alias = r.matrix;
    alias.data = a.matrix.data;
    EXPECT_EQ(F::qr(&a.matrix, &q.matrix, &alias, scratch.workspace), LINEARS_ERROR_ALIAS);
    EXPECT_EQ(q.data, old_q); EXPECT_EQ(r.data, old_r);
    short_scratch.check(); scratch.check();
}

template<class F> class FixedQr : public ::testing::Test {};
using FixedQrFormats = ::testing::Types<Q15, Q31>;
TYPED_TEST_SUITE(FixedQr, FixedQrFormats);

TYPED_TEST(FixedQr, MinimumRawColumnKeepsTheUnitDirection)
{
    using F = TypeParam;
    Matrix<F> a(2, 1), q(2, 2), r(2, 1);
    a.data[0] = 1; a.data[1] = 1;
    Scratch<F> scratch(LINEARS_QR_WORKSPACE(2, 1));
    ASSERT_EQ(F::qr(&a.matrix, &q.matrix, &r.matrix, scratch.workspace), LINEARS_OK);
    Eigen::HouseholderQR<Eigen::MatrixXd> reference(a.decoded());
    Eigen::MatrixXd expected = reference.householderQ() * Eigen::MatrixXd::Identity(2, 2);
    EXPECT_LE((q.decoded().cwiseAbs() - expected.cwiseAbs()).norm(),
        quantization_bound(q) + 32.0 * F::working_epsilon);
    EXPECT_EQ(r.data[1], 0);
    scratch.check();
}

TYPED_TEST(FixedQr, UnrepresentableRDoesNotPublishQ)
{
    using F = TypeParam;
    Eigen::Vector2d values; values << 0.75, 0.75;
    Matrix<F> a(values), q(2, 2), r(2, 1);
    q.sentinel(); r.sentinel();
    const auto old_q = q.data, old_r = r.data;
    Scratch<F> scratch(LINEARS_QR_WORKSPACE(2, 1));
    EXPECT_EQ(F::qr(&a.matrix, &q.matrix, &r.matrix, scratch.workspace), LINEARS_ERROR_RANGE);
    EXPECT_EQ(q.data, old_q); EXPECT_EQ(r.data, old_r);
    scratch.check();
}
} // namespace
