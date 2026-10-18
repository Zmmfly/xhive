/* Type selection only: fixed-point arithmetic is owned by linears_numeric. */
#include <math.h>
#include <stdint.h>

#define FILTERS_JOIN_(a, b) a##b
#define FILTERS_JOIN(a, b) FILTERS_JOIN_(a, b)
#define FILTERS_NAME_(family, suffix, name) filters_##family##_##suffix##_##name
#define FILTERS_NAME(family, suffix, name) FILTERS_NAME_(family, suffix, name)
#define F_FN(family, name) FILTERS_NAME(family, FILTERS_SUFFIX, name)
#define F_CTX(family) F_FN(family, t)
#define F_LOCAL(name) FILTERS_NAME(internal, FILTERS_SUFFIX, name)
#define F_S FILTERS_SCALAR
#define F_W FILTERS_WORK
#define F_TRY(expression) do { linears_status_t status_ = (expression); \
    if (status_ != LINEARS_OK) return status_; } while (0)

#if FILTERS_FIXED

#define F_C linears_fixed_t
#define F_STATE linears_fixed_t

static F_W F_LOCAL(zero)(void) { return 0; }

static linears_status_t F_LOCAL(load)(F_S value, F_W *out)
{
    *out = FILTERS_LOAD(value);
    return LINEARS_OK;
}

static linears_status_t F_LOCAL(store)(F_W value, F_S *out)
{
    return FILTERS_EXPORT(value, out);
}

static linears_status_t F_LOCAL(coefficient)(F_C value, F_W *out)
{
    *out = value;
    return LINEARS_OK;
}

#define F_ADD linears_fixed_add
#define F_SUB linears_fixed_sub
#define F_MUL linears_fixed_mul
#define F_DIV linears_fixed_div
#define F_CMP linears_fixed_compare

static linears_status_t F_LOCAL(count)(size_t count, F_W *out)
{
    /* Only the median's constant divisor reaches this fixed-point adapter. */
    return linears_fixed_from_int((int64_t)count, out);
}

#else

#define F_C FILTERS_SCALAR
#define F_STATE FILTERS_SCALAR

static F_W F_LOCAL(zero)(void) { return (F_W)0; }
static F_W F_LOCAL(one)(void) { return (F_W)1; }

static linears_status_t F_LOCAL(load)(F_S value, F_W *out)
{
    if (!isfinite(value)) return LINEARS_ERROR_RANGE;
    *out = value;
    return LINEARS_OK;
}

static linears_status_t F_LOCAL(store)(F_W value, F_S *out)
{
    if (!isfinite(value)) return LINEARS_ERROR_RANGE;
    *out = value;
    return LINEARS_OK;
}

static linears_status_t F_LOCAL(coefficient)(F_C value, F_W *out)
{
    return F_LOCAL(load)(value, out);
}

#define FILTERS_FLOAT_BINARY(name, op) \
static linears_status_t F_LOCAL(name)(F_W a, F_W b, F_W *out) \
{ \
    F_W result = a op b; \
    if (!isfinite(result)) return LINEARS_ERROR_RANGE; \
    *out = result; \
    return LINEARS_OK; \
}
FILTERS_FLOAT_BINARY(add, +)
FILTERS_FLOAT_BINARY(sub, -)
FILTERS_FLOAT_BINARY(mul, *)
FILTERS_FLOAT_BINARY(div, /)
#undef FILTERS_FLOAT_BINARY

#define F_ADD F_LOCAL(add)
#define F_SUB F_LOCAL(sub)
#define F_MUL F_LOCAL(mul)
#define F_DIV F_LOCAL(div)

static int F_LOCAL(compare)(F_W a, F_W b)
{
    return (a > b) - (a < b);
}
#define F_CMP F_LOCAL(compare)

static linears_status_t F_LOCAL(count)(size_t count, F_W *out)
{
    *out = (F_W)count;
    return LINEARS_OK;
}

#endif

/* Validate an external signal once; raw Q storage requires no conversion. */
static linears_status_t F_LOCAL(signal)(F_S value, F_S *out)
{
    F_W working;
    F_TRY(F_LOCAL(load)(value, &working));
    *out = value;
    return LINEARS_OK;
}

static linears_status_t F_LOCAL(weight)(F_S value, F_S *out)
{
    F_W working;
    F_TRY(F_LOCAL(load)(value, &working));
    if (F_CMP(working, F_LOCAL(zero)()) < 0) return LINEARS_ERROR_DOMAIN;
#if !FILTERS_FIXED
    if (F_CMP(working, F_LOCAL(one)()) > 0) return LINEARS_ERROR_DOMAIN;
#endif
    *out = value;
    return LINEARS_OK;
}

/* Endpoints are exact. Opposite-sign floating inputs use a weighted sum to
 * avoid x-y overflow; equal-sign inputs use the delta form to preserve a
 * constant signal and avoid rounding two near-maximum terms upward. */
static linears_status_t F_LOCAL(blend)(F_W alpha, F_W previous, F_W input, F_W *out)
{
#if FILTERS_FIXED
    /* The initialized raw alpha is nonnegative and at most INT32_MAX. */
    F_TRY(filters_lowpass_work_step(&previous, (linears_q31_t)alpha, input));
    *out = previous;
    return LINEARS_OK;
#else
    F_W complement, old_part, new_part;
    if (F_CMP(alpha, F_LOCAL(zero)()) == 0) { *out = previous; return LINEARS_OK; }
    if (F_CMP(alpha, F_LOCAL(one)()) == 0 || F_CMP(previous, input) == 0) {
        *out = input;
        return LINEARS_OK;
    }
#if !FILTERS_FIXED
    if ((previous < (F_W)0) == (input < (F_W)0)) {
        F_TRY(F_SUB(input, previous, &new_part));
        F_TRY(F_MUL(alpha, new_part, &new_part));
        return F_ADD(previous, new_part, out);
    }
#endif
    F_TRY(F_SUB(F_LOCAL(one)(), alpha, &complement));
    F_TRY(F_MUL(complement, previous, &old_part));
    F_TRY(F_MUL(alpha, input, &new_part));
    return F_ADD(old_part, new_part, out);
#endif
}
