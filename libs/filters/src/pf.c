/* Particle filters and unscented particle filters. Both families share this
 * translation unit so the UPF reuses the PF state helpers of its tag. */
#include "filters_upf.h"
#include "ukf_internal.h"

U_ALIGNMENT_TYPE(filters_pf_f32_t);
U_ALIGNMENT_TYPE(filters_pf_f64_t);
U_ALIGNMENT_TYPE(filters_pf_q15_t);
U_ALIGNMENT_TYPE(filters_pf_q31_t);
U_ALIGNMENT_TYPE(filters_upf_f32_t);
U_ALIGNMENT_TYPE(filters_upf_f64_t);
U_ALIGNMENT_TYPE(filters_upf_q15_t);
U_ALIGNMENT_TYPE(filters_upf_q31_t);

/* exp(-x) for F31 x >= 0 with integer arithmetic only. x = k*ln2 + r with
 * 0 <= r < ln2; exp(-r) = exp(-r/4)^4 from a degree-8 Taylor polynomial in
 * Horner form (truncation below 4e-13 for r/4 < 0.174); then a rounded right
 * shift by k. Every operand stays in [0,1], so the products cannot overflow.
 * Results below 2^-32 round to zero, which happens for x above about 22. */
static linears_fixed_t pf_fixed_exp_neg(linears_fixed_t x)
{
    static const linears_fixed_t ln2 = INT64_C(1488522236);
    static const linears_fixed_t inverse_factorial[9] = {
        INT64_C(2147483648), INT64_C(2147483648), INT64_C(1073741824), INT64_C(357913941),
        INT64_C(89478485), INT64_C(17895697), INT64_C(2982616), INT64_C(426088), INT64_C(53261)
    };
    linears_fixed_t k = x / ln2, r = x - k * ln2, t = (r + 2) >> 2;
    linears_fixed_t polynomial = inverse_factorial[8], product;
    int j;
    if (k >= 40) return 0;
    for (j = 7; j >= 0; --j) {
        (void)linears_fixed_mul(t, polynomial, &product);
        polynomial = inverse_factorial[j] - product;
    }
    (void)linears_fixed_mul(polynomial, polynomial, &polynomial);
    (void)linears_fixed_mul(polynomial, polynomial, &polynomial);
    return k == 0 ? polynomial : (polynomial + (INT64_C(1) << (k - 1))) >> k;
}

#define U_TAG f32
#define U_WORK float
#define U_FIXED 0
#define U_WORKSPACE filters_ukf_f32_workspace_t
#define U_SAMPLE_FN filters_pf_f32_sample_fn
#define U_ZERO 0.0f
#define U_HALF 0.5f
#define U_FINITE ukf_finite_f32
#define U_EXP_NEG(x) expf(-(x))
#include "pf_work_impl.h"

#define U_TAG f64
#define U_WORK double
#define U_FIXED 0
#define U_WORKSPACE filters_ukf_f64_workspace_t
#define U_SAMPLE_FN filters_pf_f64_sample_fn
#define U_ZERO 0.0
#define U_HALF 0.5
#define U_FINITE ukf_finite_f64
#define U_EXP_NEG(x) exp(-(x))
#include "pf_work_impl.h"

#define U_TAG fixed
#define U_WORK linears_fixed_t
#define U_FIXED 1
#define U_WORKSPACE filters_ukf_fixed_workspace_t
#define U_SAMPLE_FN filters_pf_fixed_sample_fn
#define U_ZERO INT64_C(0)
#define U_HALF (LINEARS_FIXED_ONE / 2)
#include "pf_work_impl.h"

#define U_TAG f32
#define U_KERNEL f32
#define U_SIGNAL float
#define U_WORK float
#define U_FIXED 0
#define U_STATE filters_pf_f32_t
#define U_WORKSPACE filters_ukf_f32_workspace_t
#define U_FN filters_ukf_f32_fn
#define U_RESIDUAL_FN filters_ukf_f32_residual_fn
#define U_SAMPLE_FN filters_pf_f32_sample_fn
#define U_FINITE ukf_finite_f32
#define U_HALF 0.5f
#define U_INIT_STORAGE FILTERS_PF_PREDICT_STORAGE
#define U_PREDICT_STORAGE FILTERS_PF_PREDICT_STORAGE
#define U_UPDATE_STORAGE FILTERS_PF_UPDATE_STORAGE
#define U_ESTIMATE_BASIC FILTERS_PF_ESTIMATE_BASIC_BYTES
#include "pf_filter_impl.h"

#define U_TAG f32
#define U_KERNEL f32
#define U_SIGNAL float
#define U_WORK float
#define U_FIXED 0
#define U_STATE filters_upf_f32_t
#define U_MODEL filters_upf_f32_model_t
#define U_WEIGHTS filters_ukf_f32_weights_t
#define U_WORKSPACE filters_ukf_f32_workspace_t
#define U_SAMPLE_FN filters_pf_f32_sample_fn
#define U_FINITE ukf_finite_f32
#define U_ZERO 0.0f
#define U_NEG_HALF (-0.5f)
#define U_INIT_STORAGE FILTERS_PF_PREDICT_STORAGE
#define U_STEP_STORAGE FILTERS_UPF_STEP_STORAGE
#include "upf_filter_impl.h"

#define U_TAG f64
#define U_KERNEL f64
#define U_SIGNAL double
#define U_WORK double
#define U_FIXED 0
#define U_STATE filters_pf_f64_t
#define U_WORKSPACE filters_ukf_f64_workspace_t
#define U_FN filters_ukf_f64_fn
#define U_RESIDUAL_FN filters_ukf_f64_residual_fn
#define U_SAMPLE_FN filters_pf_f64_sample_fn
#define U_FINITE ukf_finite_f64
#define U_HALF 0.5
#define U_INIT_STORAGE FILTERS_PF_PREDICT_STORAGE
#define U_PREDICT_STORAGE FILTERS_PF_PREDICT_STORAGE
#define U_UPDATE_STORAGE FILTERS_PF_UPDATE_STORAGE
#define U_ESTIMATE_BASIC FILTERS_PF_ESTIMATE_BASIC_BYTES
#include "pf_filter_impl.h"

#define U_TAG f64
#define U_KERNEL f64
#define U_SIGNAL double
#define U_WORK double
#define U_FIXED 0
#define U_STATE filters_upf_f64_t
#define U_MODEL filters_upf_f64_model_t
#define U_WEIGHTS filters_ukf_f64_weights_t
#define U_WORKSPACE filters_ukf_f64_workspace_t
#define U_SAMPLE_FN filters_pf_f64_sample_fn
#define U_FINITE ukf_finite_f64
#define U_ZERO 0.0
#define U_NEG_HALF (-0.5)
#define U_INIT_STORAGE FILTERS_PF_PREDICT_STORAGE
#define U_STEP_STORAGE FILTERS_UPF_STEP_STORAGE
#include "upf_filter_impl.h"

#define U_TAG q15
#define U_KERNEL fixed
#define U_SIGNAL int16_t
#define U_WORK linears_fixed_t
#define U_FIXED 1
#define U_STATE filters_pf_q15_t
#define U_WORKSPACE filters_ukf_fixed_workspace_t
#define U_FN filters_ukf_fixed_fn
#define U_RESIDUAL_FN filters_ukf_fixed_residual_fn
#define U_SAMPLE_FN filters_pf_fixed_sample_fn
#define U_HALF (LINEARS_FIXED_ONE / 2)
#define U_INIT_STORAGE FILTERS_PF_Q_INIT_STORAGE
#define U_PREDICT_STORAGE FILTERS_PF_Q_PREDICT_STORAGE
#define U_UPDATE_STORAGE FILTERS_PF_Q_UPDATE_STORAGE
#define U_ESTIMATE_BASIC FILTERS_PF_Q_ESTIMATE_BASIC_BYTES
#include "pf_filter_impl.h"

#define U_TAG q15
#define U_KERNEL fixed
#define U_SIGNAL int16_t
#define U_WORK linears_fixed_t
#define U_FIXED 1
#define U_STATE filters_upf_q15_t
#define U_MODEL filters_upf_fixed_model_t
#define U_WEIGHTS filters_ukf_fixed_weights_t
#define U_WORKSPACE filters_ukf_fixed_workspace_t
#define U_SAMPLE_FN filters_pf_fixed_sample_fn
#define U_ZERO INT64_C(0)
#define U_NEG_HALF (-(LINEARS_FIXED_ONE / 2))
#define U_INIT_STORAGE FILTERS_PF_Q_INIT_STORAGE
#define U_STEP_STORAGE FILTERS_UPF_Q_STEP_STORAGE
#include "upf_filter_impl.h"

#define U_TAG q31
#define U_KERNEL fixed
#define U_SIGNAL int32_t
#define U_WORK linears_fixed_t
#define U_FIXED 1
#define U_STATE filters_pf_q31_t
#define U_WORKSPACE filters_ukf_fixed_workspace_t
#define U_FN filters_ukf_fixed_fn
#define U_RESIDUAL_FN filters_ukf_fixed_residual_fn
#define U_SAMPLE_FN filters_pf_fixed_sample_fn
#define U_HALF (LINEARS_FIXED_ONE / 2)
#define U_INIT_STORAGE FILTERS_PF_Q_INIT_STORAGE
#define U_PREDICT_STORAGE FILTERS_PF_Q_PREDICT_STORAGE
#define U_UPDATE_STORAGE FILTERS_PF_Q_UPDATE_STORAGE
#define U_ESTIMATE_BASIC FILTERS_PF_Q_ESTIMATE_BASIC_BYTES
#include "pf_filter_impl.h"

#define U_TAG q31
#define U_KERNEL fixed
#define U_SIGNAL int32_t
#define U_WORK linears_fixed_t
#define U_FIXED 1
#define U_STATE filters_upf_q31_t
#define U_MODEL filters_upf_fixed_model_t
#define U_WEIGHTS filters_ukf_fixed_weights_t
#define U_WORKSPACE filters_ukf_fixed_workspace_t
#define U_SAMPLE_FN filters_pf_fixed_sample_fn
#define U_ZERO INT64_C(0)
#define U_NEG_HALF (-(LINEARS_FIXED_ONE / 2))
#define U_INIT_STORAGE FILTERS_PF_Q_INIT_STORAGE
#define U_STEP_STORAGE FILTERS_UPF_Q_STEP_STORAGE
#include "upf_filter_impl.h"
