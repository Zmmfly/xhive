#include "filters_kalman.h"

#include <math.h>

/* Preserve native precision and intermediate checks in floating variants. */
#define K_NATIVE(suffix, type) \
static linears_status_t k_##suffix##_import(type value, type *out) \
{ \
    if (!isfinite(value)) return LINEARS_ERROR_RANGE; \
    *out = value; \
    return LINEARS_OK; \
} \
static linears_status_t k_##suffix##_add(type a, type b, type *out) \
{ return k_##suffix##_import((type)(a + b), out); } \
static linears_status_t k_##suffix##_mul(type a, type b, type *out) \
{ return k_##suffix##_import((type)(a * b), out); } \
static linears_status_t k_##suffix##_div(type a, type b, type *out) \
{ return k_##suffix##_import((type)(a / b), out); } \
static linears_status_t k_##suffix##_sub_product(type base, type a, type b, type *out) \
{ \
    type product; \
    linears_status_t status = k_##suffix##_mul(a, b, &product); \
    if (status != LINEARS_OK) return status; \
    return k_##suffix##_import((type)(base - product), out); \
} \
static linears_status_t k_##suffix##_sum_product(type base, type a, type b, type *out) \
{ \
    type product; \
    linears_status_t status = k_##suffix##_mul(a, b, &product); \
    if (status != LINEARS_OK) return status; \
    return k_##suffix##_add(base, product, out); \
} \
static linears_status_t k_##suffix##_two_products(type a, type b, type c, type d, type *out) \
{ \
    type first, second; \
    linears_status_t status = k_##suffix##_mul(a, b, &first); \
    if (status != LINEARS_OK) return status; \
    status = k_##suffix##_mul(c, d, &second); \
    if (status != LINEARS_OK) return status; \
    return k_##suffix##_add(first, second, out); \
}

K_NATIVE(f32, float)
K_NATIVE(f64, double)
#undef K_NATIVE

static linears_status_t k_fixed_sub_product(linears_fixed_t base, linears_fixed_t a,
                                            linears_fixed_t b, linears_fixed_t *out)
{
    linears_accum_t accumulator = {0, 0};
    linears_status_t status = linears_fixed_mac(&accumulator, base, LINEARS_FIXED_ONE);
    if (status != LINEARS_OK) return status;
    status = linears_fixed_msu(&accumulator, a, b);
    if (status != LINEARS_OK) return status;
    return linears_accum_to_fixed(accumulator, out);
}

/* Final Q output expressions stay in F62 until the shared range/rounding check. */
#define K_FIXED_OUTPUT(suffix, type) \
static linears_status_t k_##suffix##_two_products(linears_fixed_t a, linears_fixed_t b, \
    linears_fixed_t c, linears_fixed_t d, type *out) \
{ \
    linears_accum_t accumulator = {0, 0}; \
    linears_status_t status = linears_fixed_mac(&accumulator, a, b); \
    if (status != LINEARS_OK) return status; \
    status = linears_fixed_mac(&accumulator, c, d); \
    if (status != LINEARS_OK) return status; \
    return linears_accum_to_##suffix(accumulator, out); \
} \
static linears_status_t k_##suffix##_product(linears_fixed_t a, linears_fixed_t b, type *out) \
{ \
    linears_accum_t accumulator = {0, 0}; \
    linears_status_t status = linears_fixed_mac(&accumulator, a, b); \
    if (status != LINEARS_OK) return status; \
    return linears_accum_to_##suffix(accumulator, out); \
} \
static linears_status_t k_##suffix##_sum_product(linears_fixed_t base, linears_fixed_t a, \
    linears_fixed_t b, type *out) \
{ return k_##suffix##_two_products(base, LINEARS_FIXED_ONE, a, b, out); }
K_FIXED_OUTPUT(q15, linears_q15_t)
K_FIXED_OUTPUT(q31, linears_q31_t)
#undef K_FIXED_OUTPUT

#define K_PREFIX filters_kalman_scalar_f32_
#define K_STATE filters_kalman_scalar_f32_t
#define K_SCALAR float
#define K_COEFFICIENT float
#define K_WORK float
#define K_LOAD(value, out) K_TRY(k_f32_import((value), (out)))
#define K_LOAD_COEFFICIENT K_LOAD
#define K_EXPORT k_f32_import
#define K_ADD k_f32_add
#define K_MUL k_f32_mul
#define K_GAIN(p, h, ph, s, out) k_f32_div((ph), (s), (out))
#define K_SUB_PRODUCT k_f32_sub_product
#define K_PRODUCT_OUT k_f32_mul
#define K_SUM_PRODUCT_OUT k_f32_sum_product
#define K_TWO_PRODUCTS_OUT k_f32_two_products
#define K_NEGATIVE(value) ((value) < 0.0f)
#define K_NONPOSITIVE(value) ((value) <= 0.0f)
#define K_ONE 1.0f
#include "kalman_scalar_impl.h"

#define K_PREFIX filters_kalman_scalar_f64_
#define K_STATE filters_kalman_scalar_f64_t
#define K_SCALAR double
#define K_COEFFICIENT double
#define K_WORK double
#define K_LOAD(value, out) K_TRY(k_f64_import((value), (out)))
#define K_LOAD_COEFFICIENT K_LOAD
#define K_EXPORT k_f64_import
#define K_ADD k_f64_add
#define K_MUL k_f64_mul
#define K_GAIN(p, h, ph, s, out) k_f64_div((ph), (s), (out))
#define K_SUB_PRODUCT k_f64_sub_product
#define K_PRODUCT_OUT k_f64_mul
#define K_SUM_PRODUCT_OUT k_f64_sum_product
#define K_TWO_PRODUCTS_OUT k_f64_two_products
#define K_NEGATIVE(value) ((value) < 0.0)
#define K_NONPOSITIVE(value) ((value) <= 0.0)
#define K_ONE 1.0
#include "kalman_scalar_impl.h"

#define K_PREFIX filters_kalman_scalar_q15_
#define K_STATE filters_kalman_scalar_q15_t
#define K_SCALAR linears_q15_t
#define K_COEFFICIENT linears_fixed_t
#define K_WORK linears_fixed_t
#define K_LOAD(value, out) (*(out) = linears_fixed_from_q15(value))
#define K_LOAD_COEFFICIENT(value, out) (*(out) = (value))
#define K_EXPORT linears_fixed_to_q15
#define K_ADD linears_fixed_add
#define K_MUL linears_fixed_mul
#define K_GAIN(p, h, ph, s, out) linears_fixed_muldiv((p), (h), (s), (out))
#define K_SUB_PRODUCT k_fixed_sub_product
#define K_PRODUCT_OUT k_q15_product
#define K_SUM_PRODUCT_OUT k_q15_sum_product
#define K_TWO_PRODUCTS_OUT k_q15_two_products
#define K_NEGATIVE(value) ((value) < 0)
#define K_NONPOSITIVE(value) ((value) <= 0)
#define K_ONE LINEARS_FIXED_ONE
#include "kalman_scalar_impl.h"

#define K_PREFIX filters_kalman_scalar_q31_
#define K_STATE filters_kalman_scalar_q31_t
#define K_SCALAR linears_q31_t
#define K_COEFFICIENT linears_fixed_t
#define K_WORK linears_fixed_t
#define K_LOAD(value, out) (*(out) = linears_fixed_from_q31(value))
#define K_LOAD_COEFFICIENT(value, out) (*(out) = (value))
#define K_EXPORT linears_fixed_to_q31
#define K_ADD linears_fixed_add
#define K_MUL linears_fixed_mul
#define K_GAIN(p, h, ph, s, out) linears_fixed_muldiv((p), (h), (s), (out))
#define K_SUB_PRODUCT k_fixed_sub_product
#define K_PRODUCT_OUT k_q31_product
#define K_SUM_PRODUCT_OUT k_q31_sum_product
#define K_TWO_PRODUCTS_OUT k_q31_two_products
#define K_NEGATIVE(value) ((value) < 0)
#define K_NONPOSITIVE(value) ((value) <= 0)
#define K_ONE LINEARS_FIXED_ONE
#include "kalman_scalar_impl.h"
