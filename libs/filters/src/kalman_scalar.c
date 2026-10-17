#include "filters_kalman.h"

#include <math.h>

/* Native scalar operations check the selected format at every intermediate. */
#define K_NATIVE(suffix, type) \
static linears_status_t k_##suffix##_import(type value, type *out) \
{ \
    if (!isfinite(value)) return LINEARS_ERROR_RANGE; \
    *out = value; \
    return LINEARS_OK; \
} \
static linears_status_t k_##suffix##_add(type a, type b, type *out) \
{ return k_##suffix##_import((type)(a + b), out); } \
static linears_status_t k_##suffix##_sub(type a, type b, type *out) \
{ return k_##suffix##_import((type)(a - b), out); } \
static linears_status_t k_##suffix##_mul(type a, type b, type *out) \
{ return k_##suffix##_import((type)(a * b), out); } \
static linears_status_t k_##suffix##_div(type a, type b, type *out) \
{ return k_##suffix##_import((type)(a / b), out); }

K_NATIVE(f32, float)
K_NATIVE(f64, double)
#undef K_NATIVE

static linears_status_t k_q15_export(linears_fixed_t value, linears_q15_t *out)
{
    return linears_fixed_export_q15(&value, 1, &out->mantissa, &out->exponent);
}

static linears_status_t k_q31_export(linears_fixed_t value, linears_q31_t *out)
{
    return linears_fixed_export_q31(&value, 1, &out->mantissa, &out->exponent);
}

#define K_PREFIX filters_kalman_scalar_f32_
#define K_STATE filters_kalman_scalar_f32_t
#define K_SCALAR float
#define K_WORK float
#define K_IMPORT k_f32_import
#define K_EXPORT k_f32_import
#define K_ADD k_f32_add
#define K_SUB k_f32_sub
#define K_MUL k_f32_mul
#define K_DIV k_f32_div
#define K_NEGATIVE(value) ((value) < 0.0f)
#define K_NONPOSITIVE(value) ((value) <= 0.0f)
#define K_ONE 1.0f
#include "kalman_scalar_impl.h"

#define K_PREFIX filters_kalman_scalar_f64_
#define K_STATE filters_kalman_scalar_f64_t
#define K_SCALAR double
#define K_WORK double
#define K_IMPORT k_f64_import
#define K_EXPORT k_f64_import
#define K_ADD k_f64_add
#define K_SUB k_f64_sub
#define K_MUL k_f64_mul
#define K_DIV k_f64_div
#define K_NEGATIVE(value) ((value) < 0.0)
#define K_NONPOSITIVE(value) ((value) <= 0.0)
#define K_ONE 1.0
#include "kalman_scalar_impl.h"

#define K_PREFIX filters_kalman_scalar_q15_
#define K_STATE filters_kalman_scalar_q15_t
#define K_SCALAR linears_q15_t
#define K_WORK linears_fixed_t
#define K_IMPORT linears_fixed_from_q15
#define K_EXPORT k_q15_export
#define K_ADD linears_fixed_add
#define K_SUB linears_fixed_sub
#define K_MUL linears_fixed_mul
#define K_DIV linears_fixed_div
#define K_NEGATIVE(value) ((value).mantissa < 0)
#define K_NONPOSITIVE(value) ((value).mantissa <= 0)
#define K_ONE {INT32_C(1073741824), 1}
#include "kalman_scalar_impl.h"

#define K_PREFIX filters_kalman_scalar_q31_
#define K_STATE filters_kalman_scalar_q31_t
#define K_SCALAR linears_q31_t
#define K_WORK linears_fixed_t
#define K_IMPORT linears_fixed_from_q31
#define K_EXPORT k_q31_export
#define K_ADD linears_fixed_add
#define K_SUB linears_fixed_sub
#define K_MUL linears_fixed_mul
#define K_DIV linears_fixed_div
#define K_NEGATIVE(value) ((value).mantissa < 0)
#define K_NONPOSITIVE(value) ((value).mantissa <= 0)
#define K_ONE {INT32_C(1073741824), 1}
#include "kalman_scalar_impl.h"
