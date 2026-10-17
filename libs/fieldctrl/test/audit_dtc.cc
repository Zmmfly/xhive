#include "dtc_support.h"

namespace {
template<class Tag> class AuditDtc : public ::testing::Test {};
TYPED_TEST_SUITE(AuditDtc, fc_test::Types);

/* A sector is a direction, independent of the magnitude of nonzero flux.
 * Integer rays keep the oracle away from every irrational 30-degree boundary.
 * Regression snapshot: dtc_impl.h f5fccca95062912165ce4852b156cc67baf58fe0f0ec9b77aac236fd2dea009b. */
TYPED_TEST(AuditDtc, SectorIsScaleInvariantAtUnderflowAndLargeMagnitude)
{
    using A = dtc_test::Api<TypeParam>;
    using S = typename A::scalar;
    const typename A::config cfg{A::make(0), A::make(0.125), A::make(0.125), 1};
    const typename A::ab zero{A::make(0), A::make(0)};
    const std::array<std::array<int, 2>, 12> rays{{
        {{2, 1}}, {{1, 2}}, {{-1, 2}}, {{-2, 1}}, {{-2, -1}}, {{-1, -2}},
        {{1, -2}}, {{2, -1}}, {{1, 0}}, {{0, 1}}, {{-1, 0}}, {{0, -1}}
    }};
    const auto scaled = [](int component, int level) -> S {
        if constexpr (TypeParam::fixed) {
            S result = A::make(component);
            if (result.mantissa != 0) result.exponent += level;
            return result;
        } else {
            return std::ldexp(static_cast<S>(component), level);
        }
    };
    const int smallest = [] {
        if constexpr (TypeParam::fixed) return LINEARS_EXPONENT_MIN;
        else return std::numeric_limits<S>::min_exponent - std::numeric_limits<S>::digits;
    }();
    const int largest = [] {
        if constexpr (TypeParam::fixed) return LINEARS_EXPONENT_MAX - 3;
        else return std::numeric_limits<S>::max_exponent - 3;
    }();
    for (const int scale : {smallest, -8, 0, largest}) {
        for (const auto& ray : rays) {
            SCOPED_TRACE(::testing::Message() << "scale=" << scale << " ray=("
                << ray[0] << ',' << ray[1] << ')');
            typename A::context ctx{};
            typename A::result result{};
            const typename A::ab flux{scaled(ray[0], scale), scaled(ray[1], scale)};
            ASSERT_EQ(A::init(&ctx, cfg, flux), LINEARS_OK);
            ASSERT_EQ(A::step(&ctx, zero, zero, A::make(1), A::make(1),
                A::make(1), &result), LINEARS_OK);
            const long double angle = std::atan2(static_cast<long double>(ray[1]),
                static_cast<long double>(ray[0]));
            const long double turn = angle / (2 * std::acos(-1.0L));
            const int expected = (static_cast<int>(std::floor(6 * (turn + 1.0L / 12))) + 6) % 6 + 1;
            EXPECT_EQ(result.sector, expected);
        }
    }
}
} // namespace
