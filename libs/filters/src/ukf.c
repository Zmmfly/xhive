#include "ukf_internal.h"

U_ALIGNMENT_TYPE(filters_ukf_f32_t);
U_ALIGNMENT_TYPE(filters_ukf_f64_t);
U_ALIGNMENT_TYPE(filters_ukf_q15_t);
U_ALIGNMENT_TYPE(filters_ukf_q31_t);

#define U_TAG f32
#define U_WORK float
#define U_WEIGHTS filters_ukf_f32_weights_t
#define U_WORKSPACE filters_ukf_f32_workspace_t
#define U_FN filters_ukf_f32_fn
#define U_RESIDUAL_FN filters_ukf_f32_residual_fn
#define U_FINITE ukf_finite_f32
#include "ukf_work_impl.h"

#define U_TAG f64
#define U_WORK double
#define U_WEIGHTS filters_ukf_f64_weights_t
#define U_WORKSPACE filters_ukf_f64_workspace_t
#define U_FN filters_ukf_f64_fn
#define U_RESIDUAL_FN filters_ukf_f64_residual_fn
#define U_FINITE ukf_finite_f64
#include "ukf_work_impl.h"

#define U_TAG f32
#define U_KERNEL f32
#define U_SIGNAL float
#define U_WORK float
#define U_FIXED 0
#define U_STATE filters_ukf_f32_t
#define U_WEIGHTS filters_ukf_f32_weights_t
#define U_WORKSPACE filters_ukf_f32_workspace_t
#define U_FN filters_ukf_f32_fn
#define U_RESIDUAL_FN filters_ukf_f32_residual_fn
#define U_FINITE ukf_finite_f32
#define U_PREDICT_STORAGE FILTERS_UKF_PREDICT_STORAGE
#define U_UPDATE_STORAGE FILTERS_UKF_UPDATE_STORAGE
#include "ukf_filter_impl.h"

#define U_TAG f64
#define U_KERNEL f64
#define U_SIGNAL double
#define U_WORK double
#define U_FIXED 0
#define U_STATE filters_ukf_f64_t
#define U_WEIGHTS filters_ukf_f64_weights_t
#define U_WORKSPACE filters_ukf_f64_workspace_t
#define U_FN filters_ukf_f64_fn
#define U_RESIDUAL_FN filters_ukf_f64_residual_fn
#define U_FINITE ukf_finite_f64
#define U_PREDICT_STORAGE FILTERS_UKF_PREDICT_STORAGE
#define U_UPDATE_STORAGE FILTERS_UKF_UPDATE_STORAGE
#include "ukf_filter_impl.h"

/* Q statistics own their raw MAC buffers. Only boundaries and small
 * extended-range linear solves use float/double working values. */
#define U_TAG q15
#define U_FLOAT_TAG f32
#define U_SIGNAL int16_t
#define U_BITS 15
#define U_WORK float
#define U_LSB (1.0f / 32768.0f)
#define U_ABS fabsf
#define U_WEIGHTS filters_ukf_f32_weights_t
#define U_WORKSPACE filters_ukf_q15_workspace_t
#define U_FN filters_ukf_f32_fn
#define U_RESIDUAL_FN filters_ukf_f32_residual_fn
#define U_FINITE ukf_finite_f32
#include "ukf_q_work_impl.h"

#define U_TAG q31
#define U_FLOAT_TAG f64
#define U_SIGNAL int32_t
#define U_BITS 31
#define U_WORK double
#define U_LSB (1.0 / 2147483648.0)
#define U_ABS fabs
#define U_WEIGHTS filters_ukf_f64_weights_t
#define U_WORKSPACE filters_ukf_q31_workspace_t
#define U_FN filters_ukf_f64_fn
#define U_RESIDUAL_FN filters_ukf_f64_residual_fn
#define U_FINITE ukf_finite_f64
#include "ukf_q_work_impl.h"

#define U_TAG q15
#define U_KERNEL f32
#define U_SIGNAL int16_t
#define U_WORK float
#define U_FIXED 1
#define U_STATE filters_ukf_q15_t
#define U_WEIGHTS filters_ukf_f32_weights_t
#define U_WORKSPACE filters_ukf_q15_workspace_t
#define U_FN filters_ukf_f32_fn
#define U_RESIDUAL_FN filters_ukf_f32_residual_fn
#define U_PREDICT_STORAGE FILTERS_UKF_Q_PREDICT_STORAGE
#define U_UPDATE_STORAGE FILTERS_UKF_Q_UPDATE_STORAGE
#include "ukf_filter_impl.h"

#define U_TAG q31
#define U_KERNEL f64
#define U_SIGNAL int32_t
#define U_WORK double
#define U_FIXED 1
#define U_STATE filters_ukf_q31_t
#define U_WEIGHTS filters_ukf_f64_weights_t
#define U_WORKSPACE filters_ukf_q31_workspace_t
#define U_FN filters_ukf_f64_fn
#define U_RESIDUAL_FN filters_ukf_f64_residual_fn
#define U_PREDICT_STORAGE FILTERS_UKF_Q_PREDICT_STORAGE
#define U_UPDATE_STORAGE FILTERS_UKF_Q_UPDATE_STORAGE
#include "ukf_filter_impl.h"
