/* Repeated implementation template; instantiated only by kalman_scalar.c. */
#define K_JOIN_INNER(a, b) a##b
#define K_JOIN(a, b) K_JOIN_INNER(a, b)
#define K_API(name) K_JOIN(K_PREFIX, name)
#define K_TRY(expression) do { \
    status = (expression); \
    if (status != LINEARS_OK) return status; \
} while (0)

linears_status_t K_API(reset)(K_STATE *state, K_SCALAR estimate, K_SCALAR covariance)
{
    linears_status_t status;
    K_WORK x, p;
    K_STATE next;
    if (state == NULL) return LINEARS_ERROR_ARGUMENT;
    K_LOAD(estimate, &x);
    K_LOAD(covariance, &p);
    if (K_NEGATIVE(p)) return LINEARS_ERROR_DOMAIN;
    K_TRY(K_EXPORT(x, &next.estimate));
    K_TRY(K_EXPORT(p, &next.covariance));
    *state = next;
    return LINEARS_OK;
}

linears_status_t K_API(predict)(K_STATE *state, K_COEFFICIENT transition,
                                K_SCALAR process_noise)
{
    linears_status_t status;
    K_WORK x, p, a, q, ap;
    K_STATE next;
    if (state == NULL) return LINEARS_ERROR_ARGUMENT;
    K_LOAD(state->estimate, &x);
    K_LOAD(state->covariance, &p);
    K_LOAD_COEFFICIENT(transition, &a);
    K_LOAD(process_noise, &q);
    if (K_NEGATIVE(p) || K_NEGATIVE(q)) return LINEARS_ERROR_DOMAIN;
    K_TRY(K_PRODUCT_OUT(a, x, &next.estimate));
    /* Multiply by p before the second a: p=0 need not overflow a*a. */
    K_TRY(K_MUL(a, p, &ap));
    K_TRY(K_SUM_PRODUCT_OUT(q, ap, a, &next.covariance));
    *state = next;
    return LINEARS_OK;
}

linears_status_t K_API(update)(K_STATE *state, K_SCALAR measurement,
                               K_COEFFICIENT observation, K_SCALAR measurement_noise)
{
    linears_status_t status;
    const K_WORK one = K_ONE;
    K_WORK x, p, z, h, r, ph, s, k, residual, j, jp, kr;
    K_STATE next;
    if (state == NULL) return LINEARS_ERROR_ARGUMENT;
    K_LOAD(state->estimate, &x);
    K_LOAD(state->covariance, &p);
    K_LOAD(measurement, &z);
    K_LOAD_COEFFICIENT(observation, &h);
    K_LOAD(measurement_noise, &r);
    if (K_NEGATIVE(p) || K_NONPOSITIVE(r)) return LINEARS_ERROR_DOMAIN;
    K_TRY(K_MUL(p, h, &ph));
    K_TRY(K_MUL(ph, h, &s));
    K_TRY(K_ADD(s, r, &s));
    if (K_NONPOSITIVE(s)) return LINEARS_ERROR_DOMAIN;
    K_TRY(K_GAIN(p, h, ph, s, &k));
    K_TRY(K_SUB_PRODUCT(z, h, x, &residual));
    K_TRY(K_SUM_PRODUCT_OUT(x, k, residual, &next.estimate));
    K_TRY(K_SUB_PRODUCT(one, k, h, &j));
    /* Joseph form, not p -= k*h*p: retain both nonnegative terms. */
    K_TRY(K_MUL(j, p, &jp));
    K_TRY(K_MUL(k, r, &kr));
    K_TRY(K_TWO_PRODUCTS_OUT(jp, j, kr, k, &next.covariance));
    *state = next;
    return LINEARS_OK;
}

#undef K_TRY
#undef K_API
#undef K_JOIN
#undef K_JOIN_INNER
#undef K_PREFIX
#undef K_STATE
#undef K_SCALAR
#undef K_COEFFICIENT
#undef K_WORK
#undef K_LOAD
#undef K_LOAD_COEFFICIENT
#undef K_EXPORT
#undef K_ADD
#undef K_MUL
#undef K_GAIN
#undef K_SUB_PRODUCT
#undef K_PRODUCT_OUT
#undef K_SUM_PRODUCT_OUT
#undef K_TWO_PRODUCTS_OUT
#undef K_NEGATIVE
#undef K_NONPOSITIVE
#undef K_ONE
