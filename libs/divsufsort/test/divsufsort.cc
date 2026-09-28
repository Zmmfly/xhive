#include <divsufsort64.h>
#include <divsufsort.h>
#include <divsufsort64.h>
#include <divsufsort.h>
#include <gtest/gtest.h>
#include <algorithm>
#include <cstdio>
#include <numeric>
#include <random>
#include <type_traits>
#include <vector>

static_assert(sizeof(sauchar_t) == 1);
static_assert(sizeof(saint_t) == 4);
static_assert(sizeof(saidx_t) == 4);
static_assert(sizeof(saidx64_t) == 8);
static_assert(std::is_same_v<decltype(divsufsort64(nullptr, nullptr, 0)), saint_t>);
static_assert(std::is_same_v<decltype(divbwt64(nullptr, nullptr, nullptr, 0)), saidx64_t>);

struct Api32 {
    using index = saidx_t;
    static constexpr auto sort = &divsufsort;
    static constexpr auto bwt = &divbwt;
    static constexpr auto transform = &bw_transform;
    static constexpr auto inverse = &inverse_bw_transform;
    static constexpr auto check = &sufcheck;
    static constexpr auto search = &sa_search;
    static constexpr auto simple = &sa_simplesearch;
    static constexpr auto version = &divsufsort_version;
};

struct Api64 {
    using index = saidx64_t;
    static constexpr auto sort = &divsufsort64;
    static constexpr auto bwt = &divbwt64;
    static constexpr auto transform = &bw_transform64;
    static constexpr auto inverse = &inverse_bw_transform64;
    static constexpr auto check = &sufcheck64;
    static constexpr auto search = &sa_search64;
    static constexpr auto simple = &sa_simplesearch64;
    static constexpr auto version = &divsufsort64_version;
};

template<class Api>
class DivSufSort : public ::testing::Test {};
using Variants = ::testing::Types<Api32, Api64>;
TYPED_TEST_SUITE(DivSufSort, Variants);

TEST(DivSufSortAbi, BothHeadersAndFormatMacrosCoexist) {
    char text[100];
    std::snprintf(text, sizeof(text), "%" PRIdSAINT_T " %" PRIdSAIDX_T " %" PRIdSAIDX64_T,
                  saint_t{-7}, saidx_t{-2147483647}, saidx64_t{1099511627776});
    EXPECT_STREQ(text, "-7 -2147483647 1099511627776");
    ASSERT_NE(divsufsort_version(), nullptr);
    ASSERT_NE(divsufsort64_version(), nullptr);
    EXPECT_STREQ(divsufsort_version(), divsufsort64_version());
}

TYPED_TEST(DivSufSort, BananaAndSearch) {
    using Api = TypeParam;
    using Index = typename Api::index;
    const sauchar_t input[] = "banana";
    const sauchar_t pattern[] = "ana";
    Index sa[6] = {};
    const std::vector<Index> expected{5, 3, 1, 0, 4, 2};
    ASSERT_EQ(Api::sort(input, sa, 6), 0);
    EXPECT_EQ(std::vector<Index>(sa, sa + 6), expected);
    EXPECT_EQ(Api::check(input, sa, 6, 0), 0);
    Index left = -9;
    EXPECT_EQ(Api::search(input, 6, pattern, 3, sa, 6, &left), 2);
    EXPECT_EQ(left, 1);
    EXPECT_EQ(Api::simple(input, 6, sa, 6, 'a', &left), 3);
    EXPECT_EQ(left, 0);
    const sauchar_t missing[] = "z";
    EXPECT_EQ(Api::search(input, 6, missing, 1, sa, 6, &left), 0);
    EXPECT_EQ(left, 6);
    EXPECT_EQ(Api::simple(input, 6, sa, 6, 'z', nullptr), 0);
    EXPECT_EQ(Api::search(input, 6, pattern, 0, sa, 6, &left), 6);
    EXPECT_EQ(left, 0);
    EXPECT_EQ(Api::search(input, 6, pattern, 3, sa, 6, nullptr), 2);
}

TYPED_TEST(DivSufSort, MatchesReferenceSuffixOrdering) {
    using Api = TypeParam;
    using Index = typename Api::index;
    std::mt19937 random(0x12345678u);
    for (size_t length : {0u, 1u, 2u, 3u, 7u, 31u, 128u, 513u, 1025u, 4097u}) {
        for (unsigned alphabet : {1u, 3u, 256u}) {
            SCOPED_TRACE(::testing::Message() << length << '/' << alphabet);
            std::vector<sauchar_t> input(std::max(size_t{1}, length));
            for (size_t i = 0; i < length; ++i) {
                input[i] = static_cast<sauchar_t>(random() % alphabet);
            }
            std::vector<Index> expected(length);
            std::iota(expected.begin(), expected.end(), Index{0});
            std::sort(expected.begin(), expected.end(), [&](Index a, Index b) {
                return std::lexicographical_compare(input.begin() + a, input.begin() + length,
                                                    input.begin() + b, input.begin() + length);
            });
            std::vector<Index> actual(std::max(size_t{1}, length));
            ASSERT_EQ(Api::sort(input.data(), actual.data(), static_cast<Index>(length)), 0);
            ASSERT_EQ(Api::check(input.data(), actual.data(), static_cast<Index>(length), 0), 0);
            actual.resize(length);
            EXPECT_EQ(actual, expected);
        }
    }
}

TYPED_TEST(DivSufSort, BwtRoundTripsWithOwnedAndProvidedScratch) {
    using Api = TypeParam;
    using Index = typename Api::index;
    const std::vector<sauchar_t> input{'b', 'a', 'n', 'a', 'n', 'a', 0, 255, 0};
    const auto n = static_cast<Index>(input.size());
    std::vector<sauchar_t> transformed(input.size()), recovered(input.size());
    std::vector<Index> scratch(input.size() + 2, Index{-99});
    const auto primary = Api::bwt(input.data(), transformed.data(), scratch.data(), n);
    ASSERT_GE(primary, 1);
    ASSERT_LE(primary, n);
    EXPECT_EQ(scratch.back(), -99);
    ASSERT_EQ(Api::inverse(transformed.data(), recovered.data(), nullptr, n, primary), 0);
    EXPECT_EQ(recovered, input);

    auto in_place = input;
    const auto in_place_index = Api::bwt(in_place.data(), in_place.data(), nullptr, n);
    ASSERT_EQ(in_place_index, primary);
    EXPECT_EQ(in_place, transformed);
    ASSERT_EQ(Api::inverse(in_place.data(), in_place.data(), scratch.data(), n, primary), 0);
    EXPECT_EQ(in_place, input);

    std::vector<Index> sa(input.size());
    ASSERT_EQ(Api::sort(input.data(), sa.data(), n), 0);
    Index idx = -1;
    ASSERT_EQ(Api::transform(input.data(), recovered.data(), sa.data(), n, &idx), 0);
    EXPECT_EQ(idx, primary);
    EXPECT_EQ(recovered, transformed);
    in_place = input;
    ASSERT_EQ(Api::transform(in_place.data(), in_place.data(), sa.data(), n, &idx), 0);
    EXPECT_EQ(idx, primary);
    EXPECT_EQ(in_place, transformed);
    ASSERT_EQ(Api::transform(input.data(), recovered.data(), nullptr, n, &idx), 0);
    EXPECT_EQ(idx, primary);
    EXPECT_EQ(recovered, transformed);
}

TYPED_TEST(DivSufSort, EmptyAndSingleByteContracts) {
    using Api = TypeParam;
    using Index = typename Api::index;
    sauchar_t input = 42, output = 99;
    Index sa = -7, idx = -9;
    EXPECT_EQ(Api::sort(&input, &sa, 0), 0);
    EXPECT_EQ(sa, -7);
    EXPECT_EQ(Api::bwt(&input, &output, nullptr, 0), 0);
    EXPECT_EQ(output, 99);
    EXPECT_EQ(Api::transform(&input, &output, nullptr, 0, &idx), 0);
    EXPECT_EQ(idx, 0);
    EXPECT_EQ(Api::inverse(&input, &output, nullptr, 0, 0), 0);
    EXPECT_EQ(output, 99);
    EXPECT_EQ(Api::check(&input, &sa, 0, 0), 0);
    EXPECT_EQ(Api::search(&input, 0, &input, 0, &sa, 0, &idx), 0);
    EXPECT_EQ(idx, -1);
    EXPECT_EQ(Api::simple(&input, 0, &sa, 0, 42, &idx), 0);
    EXPECT_EQ(idx, -1);

    EXPECT_EQ(Api::sort(&input, &sa, 1), 0);
    EXPECT_EQ(sa, 0);
    EXPECT_EQ(Api::bwt(&input, &output, nullptr, 1), 1);
    EXPECT_EQ(output, input);
    output = 99;
    EXPECT_EQ(Api::transform(&input, &output, nullptr, 1, &idx), 0);
    EXPECT_EQ(idx, 1);
    EXPECT_EQ(output, input);
    output = 99;
    EXPECT_EQ(Api::inverse(&input, &output, nullptr, 1, 1), 0);
    EXPECT_EQ(output, 99); // Preserve the upstream small-input no-copy behavior.
}

TYPED_TEST(DivSufSort, InvalidArgumentsAndSuffixArrays) {
    using Api = TypeParam;
    using Index = typename Api::index;
    const sauchar_t input[] = "banana";
    sauchar_t output[6] = {};
    Index sa[6] = {}, idx = 8;
    EXPECT_EQ(Api::sort(nullptr, sa, 0), -1);
    EXPECT_EQ(Api::sort(input, nullptr, 0), -1);
    EXPECT_EQ(Api::sort(input, sa, -1), -1);
    EXPECT_EQ(Api::bwt(nullptr, output, nullptr, 0), -1);
    EXPECT_EQ(Api::bwt(input, nullptr, nullptr, 0), -1);
    EXPECT_EQ(Api::bwt(input, output, nullptr, -1), -1);
    EXPECT_EQ(Api::transform(input, output, nullptr, 0, nullptr), -1);
    EXPECT_EQ(Api::inverse(input, output, nullptr, 1, 0), -1);
    EXPECT_EQ(Api::inverse(input, output, nullptr, 1, 2), -1);
    EXPECT_EQ(Api::inverse(nullptr, output, nullptr, 0, 0), -1);
    EXPECT_EQ(Api::check(input, nullptr, 0, 0), -1);
    EXPECT_EQ(Api::search(input, 6, nullptr, 0, sa, 6, &idx), -1);
    EXPECT_EQ(idx, -1);
    EXPECT_EQ(Api::simple(input, -1, sa, 6, 42, &idx), -1);
    EXPECT_EQ(idx, -1);

    ASSERT_EQ(Api::sort(input, sa, 6), 0);
    sa[0] = 6;
    EXPECT_EQ(Api::check(input, sa, 6, 0), -2);
    ASSERT_EQ(Api::sort(input, sa, 6), 0);
    std::swap(sa[0], sa[5]);
    EXPECT_EQ(Api::check(input, sa, 6, 0), -3);
    ASSERT_EQ(Api::sort(input, sa, 6), 0);
    std::swap(sa[1], sa[2]);
    EXPECT_EQ(Api::check(input, sa, 6, 0), -4);
}
