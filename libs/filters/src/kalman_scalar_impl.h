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
    K_TRY(K_IMPORT(estimate, &x));
    K_TRY(K_IMPORT(covariance, &p));
    if (K_NEGATIVE(p)) return LINEARS_ERROR_DOMAIN;
    K_TRY(K_EXPORT(x, &next.estimate));
    K_TRY(K_EXPORT(p, &next.covariance));
    *state = next;
    return LINEARS_OK;
}

linears_status_t K_API(predict)(K_STATE *state, K_SCALAR transition,
                                K_SCALAR process_noise)
{
    linears_status_t status;
    K_WORK x, p, a, q, nx, np, ap;
    K_STATE next;
    if (state == NULL) return LINEARS_ERROR_ARGUMENT;
    K_TRY(K_IMPORT(state->estimate, &x));
    K_TRY(K_IMPORT(state->covariance, &p));
    K_TRY(K_IMPORT(transition, &a));
    K_TRY(K_IMPORT(process_noise, &q));
    if (K_NEGATIVE(p) || K_NEGATIVE(q)) return LINEARS_ERROR_DOMAIN;
    K_TRY(K_MUL(a, x, &nx));
    /* Multiply by p before the second a: p=0 need not overflow a*a. */
    K_TRY(K_MUL(a, p, &ap));
    K_TRY(K_MUL(ap, a, &np));
    K_TRY(K_ADD(np, q, &np));
    K_TRY(K_EXPORT(nx, &next.estimate));
    K_TRY(K_EXPORT(np, &next.covariance));
    *state = next;
    return LINEARS_OK;
}

linears_status_t K_API(update)(K_STATE *state, K_SCALAR measurement,
                               K_SCALAR observation, K_SCALAR measurement_noise)
{
    linears_status_t status;
    const K_WORK one = K_ONE;
    K_WORK x, p, z, h, r, ph, s, k, residual, temp, nx, j, jp, kr, np;
    K_STATE next;
    if (state == NULL) return LINEARS_ERROR_ARGUMENT;
    K_TRY(K_IMPORT(state->estimate, &x));
    K_TRY(K_IMPORT(state->covariance, &p));
    K_TRY(K_IMPORT(measurement, &z));
    K_TRY(K_IMPORT(observation, &h));
    K_TRY(K_IMPORT(measurement_noise, &r));
    if (K_NEGATIVE(p) || K_NONPOSITIVE(r)) return LINEARS_ERROR_DOMAIN;
    K_TRY(K_MUL(p, h, &ph));
    K_TRY(K_MUL(ph, h, &s));
    K_TRY(K_ADD(s, r, &s));
    if (K_NONPOSITIVE(s)) return LINEARS_ERROR_DOMAIN;
    K_TRY(K_DIV(ph, s, &k));
    K_TRY(K_MUL(h, x, &temp));
    K_TRY(K_SUB(z, temp, &residual));
    K_TRY(K_MUL(k, residual, &temp));
    K_TRY(K_ADD(x, temp, &nx));
    K_TRY(K_MUL(k, h, &temp));
    K_TRY(K_SUB(one, temp, &j));
    /* Joseph form, not p -= k*h*p: retain both nonnegative terms. */
    K_TRY(K_MUL(j, p, &jp));
    K_TRY(K_MUL(jp, j, &np));
    K_TRY(K_MUL(k, r, &kr));
    K_TRY(K_MUL(kr, k, &temp));
    K_TRY(K_ADD(np, temp, &np));
    K_TRY(K_EXPORT(nx, &next.estimate));
    K_TRY(K_EXPORT(np, &next.covariance));
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
#undef K_WORK
#undef K_IMPORT
#undef K_EXPORT
#undef K_ADD
#undef K_SUB
#undef K_MUL
#undef K_DIV
#undef K_NEGATIVE
#undef K_NONPOSITIVE
#undef K_ONE
