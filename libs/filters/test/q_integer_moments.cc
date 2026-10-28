#include "filters_pf.h"
#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cstdint>
#include <cstring>
#include <limits>
#include <numeric>
#include <random>
#include <utility>
#include <vector>

namespace {
struct Q15 {
    using Raw = int16_t;
    using Work = float;
    using State = filters_pf_q15_t;
    using Workspace = filters_ukf_q15_workspace_t;
    static constexpr unsigned bits = 15;
    static constexpr auto estimate = filters_pf_q15_estimate;
};
struct Q31 {
    using Raw = int32_t;
    using Work = double;
    using State = filters_pf_q31_t;
    using Workspace = filters_ukf_q31_workspace_t;
    static constexpr unsigned bits = 31;
    static constexpr auto estimate = filters_pf_q31_estimate;
};

template<class F> struct Fixture {
    using R = typename F::Raw;
    using W = typename F::Work;
    static constexpr R guard = R(12345);
    std::size_t count, n, data_count, basic_bytes, signal_count;
    std::vector<R> particles, weights, q, r, mean, covariance, signal;
    std::vector<W> data;
    std::vector<linears_workspace_unit_t> basic;
    typename F::State state;
    typename F::Workspace workspace;
    linears_matrix_t mean_view, covariance_view;

    Fixture(std::size_t count_, std::size_t n_)
        : count(count_), n(n_), data_count(FILTERS_PF_Q_ESTIMATE_STORAGE(count, n)),
          basic_bytes(FILTERS_PF_Q_ESTIMATE_BASIC_BYTES(count, n)),
          signal_count(FILTERS_PF_Q_ESTIMATE_SIGNAL_STORAGE(count, n)),
          particles(count*n + 2, guard), weights(count + 2, guard), q(n*n + 2, guard), r(n*n + 2, guard),
          mean(n + 2, guard), covariance(n*n + 2, guard), signal(signal_count + 2, guard),
          data(data_count + 2, std::numeric_limits<W>::quiet_NaN()),
          basic(LINEARS_WORKSPACE_UNITS(basic_bytes) + 2),
          state{{particles.data()+1, count, n}, {weights.data()+1, count, 1},
                {q.data()+1, n, n}, {r.data()+1, n, n}, count*n, count, n*n, n*n},
          workspace{data.data()+1, data_count, basic.data()+1, basic_bytes, {nullptr, 0},
                    signal.data()+1, signal_count},
          mean_view{mean.data()+1, n, 1}, covariance_view{covariance.data()+1, n, n} {
        std::memset(basic.data(), 0x35, basic.size()*sizeof(basic[0]));
        std::fill(q.begin()+1, q.end()-1, R(0));
        std::fill(r.begin()+1, r.end()-1, R(0));
        for (std::size_t j=0; j<n; ++j) q[1+j*n+j]=r[1+j*n+j]=R(INT64_C(1)<<(F::bits-3));
    }
    linears_status_t run() { return F::estimate(&state, nullptr, nullptr, &mean_view, &covariance_view, workspace); }
    void guards() const {
        for (const auto* values : {&particles,&weights,&q,&r,&mean,&covariance,&signal}) {
            EXPECT_EQ(values->front(),guard); EXPECT_EQ(values->back(),guard);
        }
        const auto* head=reinterpret_cast<const unsigned char*>(basic.data());
        for (std::size_t i=0;i<sizeof(basic[0]);++i) EXPECT_EQ(head[i],0x35);
        const auto* tail=reinterpret_cast<const unsigned char*>(basic.data()+1);
        for (std::size_t i=basic_bytes;i<(basic.size()-1)*sizeof(basic[0]);++i) EXPECT_EQ(tail[i],0x35);
    }
};

template<class F> class IntegerMoments : public ::testing::Test {};
using Formats = ::testing::Types<Q15,Q31>;
TYPED_TEST_SUITE(IntegerMoments,Formats);

#ifdef __SIZEOF_INT128__
using Wide = __int128_t;

/* Independent rational reference. Common weight factors are cancelled first;
 * random cases limit the remaining weights to 16 bits, keeping every reference
 * product below 2^127. Production cannot assume those reduced weights. */
template<class F> typename F::Raw rounded(Wide numerator, Wide denominator) {
    const bool negative=numerator<0;
    Wide magnitude=negative?-numerator:numerator;
    Wide value=magnitude/denominator;
    if ((magnitude%denominator)*2>=denominator) ++value;
    if (!negative && value==(Wide(1)<<F::bits)) value=std::numeric_limits<typename F::Raw>::max();
    return static_cast<typename F::Raw>(negative?-value:value);
}

template<class F> auto reference(const Fixture<F>& f) {
    using R=typename F::Raw;
    std::uint64_t common=0;
    for (std::size_t i=0;i<f.count;++i) common=std::gcd(common,std::uint64_t(f.weights[i+1]));
    std::vector<Wide> sums(f.n,0), remainder(f.n,0);
    std::vector<R> mean(f.n), covariance(f.n*f.n);
    Wide total=0;
    for (std::size_t i=0;i<f.count;++i) {
        Wide weight=std::uint64_t(f.weights[i+1])/common;
        total+=weight;
        for (std::size_t j=0;j<f.n;++j) sums[j]+=Wide(f.particles[1+i*f.n+j])*weight;
    }
    for (std::size_t j=0;j<f.n;++j) {
        mean[j]=rounded<F>(sums[j],total);
        remainder[j]=sums[j]-Wide(mean[j])*total;
    }
    for (std::size_t j=0;j<f.n;++j) {
        for (std::size_t k=0;k<f.n;++k) {
            Wide sum=0;
            for (std::size_t i=0;i<f.count;++i) {
                Wide a=Wide(f.particles[1+i*f.n+j])-mean[j];
                Wide b=Wide(f.particles[1+i*f.n+k])-mean[k];
                Wide product=a*b;
                if constexpr (F::bits==31) {
                    const bool negative=product<0;
                    Wide magnitude=negative?-product:product;
                    magnitude=(magnitude+(Wide(1)<<30))>>31;
                    product=negative?-magnitude:magnitude;
                }
                sum+=product*(std::uint64_t(f.weights[i+1])/common);
            }
            Wide numerator=sum*total;
            if constexpr (F::bits==31) numerator*=Wide(1)<<31;
            numerator-=remainder[j]*remainder[k];
            covariance[j*f.n+k]=rounded<F>(numerator,total*total*(Wide(1)<<F::bits));
        }
    }
    return std::make_pair(mean,covariance);
}

template<class F> void expect_reference(Fixture<F>& f) {
    const auto expected=reference(f);
    const auto particles=f.particles, weights=f.weights, q=f.q, r=f.r;
    std::vector<unsigned char> scratch(f.data.size()*sizeof(typename F::Work));
    std::memcpy(scratch.data(),f.data.data(),scratch.size());
    ASSERT_EQ(f.run(),LINEARS_OK);
    EXPECT_TRUE(std::equal(expected.first.begin(),expected.first.end(),f.mean.begin()+1));
    EXPECT_TRUE(std::equal(expected.second.begin(),expected.second.end(),f.covariance.begin()+1));
    // The default path requires no floating working elements at all.
    EXPECT_EQ(std::memcmp(scratch.data(),f.data.data(),scratch.size()),0);
    EXPECT_EQ(f.particles,particles); EXPECT_EQ(f.weights,weights);
    EXPECT_EQ(f.q,q); EXPECT_EQ(f.r,r);
    f.guards();
}

TYPED_TEST(IntegerMoments, FullSignalRangeMatchesExactRationals) {
    using F=TypeParam; using R=typename F::Raw;
    std::mt19937 random(0x781e52u+F::bits);
    std::uniform_int_distribution<int64_t> signal(std::numeric_limits<R>::min(),std::numeric_limits<R>::max());
    const uint32_t maximum_weight=std::min<uint32_t>(65535u,std::numeric_limits<R>::max());
    for (unsigned trial=0;trial<160;++trial) {
        SCOPED_TRACE(trial);
        Fixture<F> f(1+random()%131,1+random()%4);
        for (std::size_t i=0;i<f.count;++i) {
            f.weights[i+1]=R(random()%(maximum_weight+1));
            for (std::size_t j=0;j<f.n;++j) f.particles[1+i*f.n+j]=R(signal(random));
        }
        f.weights[1]=R(1);
        expect_reference(f);
    }
}

TYPED_TEST(IntegerMoments, RawMeanTiesAndEndpoints) {
    using F=TypeParam; using R=typename F::Raw;
    for (R value : {R(-1),R(1),std::numeric_limits<R>::min(),std::numeric_limits<R>::max()}) {
        Fixture<F> f(2,2);
        f.weights[1]=f.weights[2]=R(1);
        f.particles[1]=0;f.particles[2]=value;
        f.particles[3]=value;f.particles[4]=value;
        expect_reference(f);
    }
}

TEST(IntegerMomentsExact, Q15CenterCorrectionDecidesTheHalfCell) {
    const int16_t inputs[][3]={{23061,-7633,27435},{-6960,-21823,-32036}};
    const int16_t weights[][3]={{25720,28864,1431},{7910,3226,29255}};
    const int16_t means[]={7356,-26310}, variances[]={7303,3011};
    for (unsigned c=0;c<2;++c) {
        Fixture<Q15> f(3,2);
        for (std::size_t i=0;i<3;++i) {
            f.particles[1+2*i]=inputs[c][i];f.particles[2+2*i]=int16_t(-inputs[c][i]);
            f.weights[i+1]=weights[c][i];
        }
        expect_reference(f);
        EXPECT_EQ(f.mean[1],means[c]);EXPECT_EQ(f.mean[2],-means[c]);
        EXPECT_EQ(f.covariance[1],variances[c]);EXPECT_EQ(f.covariance[2],-variances[c]);
        EXPECT_EQ(f.covariance[3],-variances[c]);EXPECT_EQ(f.covariance[4],variances[c]);
    }
}

TEST(IntegerMomentsExact, Q31HalfCellNeedsMoreThan128NumeratorBits) {
    Fixture<Q31> f(128,2);
    const int32_t a=(INT32_C(1)<<30)+1;
    for (std::size_t i=0;i<f.count;++i) {
        const int32_t x=i<32?-a:i<64?a:i<96?1:0;
        f.particles[1+2*i]=x;f.particles[2+2*i]=-x;
        f.weights[i+1]=INT32_MAX;
    }
    /* A=64*W*(2^29+1), T=128*W, r=32*W. The uncancelled q31
     * numerator W^2*(2^73+2^44-2^10) exceeds 2^133. The reference
     * cancels W first; production must handle the original wide product.
     * The exact variance is 2^28+1/2-2^-35 raw cells, not the upper tie. */
    expect_reference(f);
    EXPECT_EQ(f.mean[1],0);EXPECT_EQ(f.mean[2],0);
    EXPECT_EQ(f.covariance[1],INT32_C(1)<<28);
    EXPECT_EQ(f.covariance[2],-(INT32_C(1)<<28));
    EXPECT_EQ(f.covariance[3],-(INT32_C(1)<<28));
    EXPECT_EQ(f.covariance[4],INT32_C(1)<<28);
}

TEST(IntegerMomentsExact, Q15CountBoundaryKeepsTheSameRationalResult) {
    for (std::size_t count : {65540u,65541u}) {
        SCOPED_TRACE(count);
        Fixture<Q15> f(count,2);
        for (std::size_t i=0;i<count;++i) {
            const int16_t x=i%2?INT16_C(16384):-INT16_C(16384);
            f.particles[1+2*i]=x;f.particles[2+2*i]=int16_t(-x);
            f.weights[i+1]=INT16_MAX;
        }
        expect_reference(f);
    }
}
#endif

TYPED_TEST(IntegerMoments, FailureDoesNotPublishAndCustomResidualCanReuseScratch) {
    using F=TypeParam;using R=typename F::Raw;using W=typename F::Work;
    Fixture<F> f(4,2);
    const R half=R(INT64_C(1)<<(F::bits-1));
    for (std::size_t i=0;i<f.count;++i) {
        f.particles[1+2*i]=i%2?half:R(-half);
        f.particles[2+2*i]=R(-f.particles[1+2*i]);
        f.weights[i+1]=std::numeric_limits<R>::max();
    }
    ASSERT_EQ(f.run(),LINEARS_OK);
    const auto mean=f.mean,covariance=f.covariance;
    int calls=0;
    const auto residual=+[](const linears_matrix_t* a,const linears_matrix_t* b,linears_matrix_t* out,void* context)->linears_status_t {
        ++*static_cast<int*>(context);
        for (std::size_t i=0;i<a->rows;++i)
            static_cast<W*>(out->data)[i]=static_cast<const W*>(a->data)[i]-static_cast<const W*>(b->data)[i];
        return LINEARS_OK;
    };
    ASSERT_EQ(F::estimate(&f.state,residual,&calls,&f.mean_view,&f.covariance_view,f.workspace),LINEARS_OK);
    EXPECT_EQ(calls,8);EXPECT_EQ(f.mean,mean);EXPECT_EQ(f.covariance,covariance);
    ASSERT_EQ(f.run(),LINEARS_OK);
    for (int invalid=0;invalid<2;++invalid) {
        std::fill(f.weights.begin()+1,f.weights.end()-1,R(0));
        if (invalid) f.weights[1]=R(-1);
        EXPECT_EQ(f.run(),LINEARS_ERROR_DOMAIN);
        EXPECT_EQ(f.mean,mean);EXPECT_EQ(f.covariance,covariance);
    }
    f.guards();
}
} // namespace
