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

#define U_TAG f32
#define U_WORK float
#define U_WORKSPACE filters_ukf_f32_workspace_t
#define U_SAMPLE_FN filters_pf_f32_sample_fn
#define U_FINITE ukf_finite_f32
#define U_EXP_NEG(x) expf(-(x))
#include "pf_work_impl.h"

#define U_TAG f64
#define U_WORK double
#define U_WORKSPACE filters_ukf_f64_workspace_t
#define U_SAMPLE_FN filters_pf_f64_sample_fn
#define U_FINITE ukf_finite_f64
#define U_EXP_NEG(x) exp(-(x))
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
#include "upf_filter_impl.h"

/* q15 works in float, q31 in double: the significand covers the signal. */
#define U_TAG q15
#define U_KERNEL f32
#define U_SIGNAL int16_t
#define U_SIGNAL_MAX INT16_MAX
#define U_WORK float
#define U_FIXED 1
#define U_STATE filters_pf_q15_t
#define U_WORKSPACE filters_ukf_f32_workspace_t
#define U_FN filters_ukf_f32_fn
#define U_RESIDUAL_FN filters_ukf_f32_residual_fn
#define U_SAMPLE_FN filters_pf_f32_sample_fn
#include "pf_filter_impl.h"

#define U_TAG q15
#define U_KERNEL f32
#define U_SIGNAL int16_t
#define U_WORK float
#define U_FIXED 1
#define U_STATE filters_upf_q15_t
#define U_MODEL filters_upf_f32_model_t
#define U_WEIGHTS filters_ukf_f32_weights_t
#define U_WORKSPACE filters_ukf_f32_workspace_t
#define U_SAMPLE_FN filters_pf_f32_sample_fn
#include "upf_filter_impl.h"

#define U_TAG q31
#define U_KERNEL f64
#define U_SIGNAL int32_t
#define U_SIGNAL_MAX INT32_MAX
#define U_WORK double
#define U_FIXED 1
#define U_STATE filters_pf_q31_t
#define U_WORKSPACE filters_ukf_f64_workspace_t
#define U_FN filters_ukf_f64_fn
#define U_RESIDUAL_FN filters_ukf_f64_residual_fn
#define U_SAMPLE_FN filters_pf_f64_sample_fn
#include "pf_filter_impl.h"

#define U_TAG q31
#define U_KERNEL f64
#define U_SIGNAL int32_t
#define U_WORK double
#define U_FIXED 1
#define U_STATE filters_upf_q31_t
#define U_MODEL filters_upf_f64_model_t
#define U_WEIGHTS filters_ukf_f64_weights_t
#define U_WORKSPACE filters_ukf_f64_workspace_t
#define U_SAMPLE_FN filters_pf_f64_sample_fn
#include "upf_filter_impl.h"
