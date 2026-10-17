/* Four instantiations share algorithms, but retain native floating precision
 * or the linears integer working scalar selected by fieldctrl_variant.h. */
static FC_W FC_N(base_clamp)(FC_W x, FC_W lo, FC_W hi) {
    return FC_CMP(x, lo) < 0 ? lo : (FC_CMP(x, hi) > 0 ? hi : x);
}

static linears_status_t FC_N(base_limit)(FC_W a, FC_W b, FC_W limit,
                                         FC_W *oa, FC_W *ob) {
    linears_status_t status = LINEARS_OK;
    FC_W magnitude = FC_HYPOT(a, b);
    if (status != LINEARS_OK) return status;
    if (FC_CMP(magnitude, limit) > 0) {
        a = FC_RESCALE(a, limit, magnitude);
        b = FC_RESCALE(b, limit, magnitude);
    }
    if (status == LINEARS_OK) { *oa = a; *ob = b; }
    return status;
}

/* Compute the common mode in unit-bus coordinates. Absolute-voltage math
 * would erase duty information at subnormal bus voltages. If a component
 * exceeds the bus, saturation is certain: scale by that component instead,
 * since the radial limiter then discards the original magnitude anyway. */
static linears_status_t FC_N(base_pwm)(FC_W a, FC_W b, FC_W bus,
                                       FC_TYPE(abc) *out) {
    linears_status_t status = LINEARS_OK;
    FC_W root3 = FC_SQRT(FC_I(3));
    FC_W limit = FC_DIV(FC_I(1), root3), lo, hi, offset, half = FC_RATIO(1, 2);
    FC_W p, q, r, aa = FC_ABS(a), bb = FC_ABS(b), scale;
    FC_TYPE(abc) result;
    scale = FC_CMP(aa, bb) > 0 ? aa : bb;
    if (FC_CMP(scale, bus) < 0) scale = bus;
    a = FC_DIV(a, scale);
    b = FC_DIV(b, scale);
    if (status != LINEARS_OK) return status;
    status = FC_N(base_limit)(a, b, limit, &a, &b);
    p = a;
    q = FC_MUL(half, FC_SUB(FC_MUL(root3, b), a));
    r = FC_MUL(half, FC_NEG(FC_ADD(a, FC_MUL(root3, b))));
    lo = FC_CMP(p, q) < 0 ? p : q;
    lo = FC_CMP(lo, r) < 0 ? lo : r;
    hi = FC_CMP(p, q) > 0 ? p : q;
    hi = FC_CMP(hi, r) > 0 ? hi : r;
    offset = FC_MUL(half, FC_ADD(lo, hi));
    p = FC_ADD(half, FC_SUB(p, offset));
    q = FC_ADD(half, FC_SUB(q, offset));
    r = FC_ADD(half, FC_SUB(r, offset));
    p = FC_N(base_clamp)(p, FC_I(0), FC_I(1));
    q = FC_N(base_clamp)(q, FC_I(0), FC_I(1));
    r = FC_N(base_clamp)(r, FC_I(0), FC_I(1));
    FC_STORE(p, &result.a);
    FC_STORE(q, &result.b);
    FC_STORE(r, &result.c);
    if (status == LINEARS_OK) *out = result;
    return status;
}

linears_status_t FC_N(wrap)(FC_T angle, FC_T *output) {
    linears_status_t status = LINEARS_OK;
    FC_W a;
    FC_T result = {0};
    if (output == NULL) return LINEARS_ERROR_ARGUMENT;
    a = FC_LOAD(angle);
    FC_ANGLE_STORE(a, &result);
    if (status == LINEARS_OK) *output = result;
    return status;
}

linears_status_t FC_N(sincos)(FC_T angle, FC_T *sine, FC_T *cosine) {
    linears_status_t status = LINEARS_OK;
    FC_W a, s = FC_I(0), c = FC_I(0);
    FC_T sout, cout;
    if (sine == NULL || cosine == NULL) return LINEARS_ERROR_ARGUMENT;
    if (fc_overlap(sine, sizeof(*sine), cosine, sizeof(*cosine))) return LINEARS_ERROR_ALIAS;
    a = FC_LOAD(angle);
    FC_SINCOS(a, &s, &c);
    FC_STORE(s, &sout);
    FC_STORE(c, &cout);
    if (status == LINEARS_OK) { *sine = sout; *cosine = cout; }
    return status;
}

linears_status_t FC_N(atan2)(FC_T y, FC_T x, FC_T *angle) {
    linears_status_t status = LINEARS_OK;
    FC_W a, b, result;
    FC_T out = {0};
    if (angle == NULL) return LINEARS_ERROR_ARGUMENT;
    a = FC_LOAD(y); b = FC_LOAD(x);
    result = FC_ATAN2(a, b);
    FC_ANGLE_STORE(result, &out);
    if (status == LINEARS_OK) *angle = out;
    return status;
}

linears_status_t FC_N(clarke)(FC_TYPE(abc) phase, FC_TYPE(ab) *stationary) {
    linears_status_t status = LINEARS_OK;
    FC_W a, b, c, alpha, beta;
    FC_TYPE(ab) result;
    if (stationary == NULL) return LINEARS_ERROR_ARGUMENT;
    a = FC_LOAD(phase.a); b = FC_LOAD(phase.b); c = FC_LOAD(phase.c);
    /* Division before accumulation avoids overflowing 2*a for finite a. */
    alpha = FC_SUB(FC_MUL(a, FC_RATIO(2, 3)), FC_ADD(FC_DIV(b, FC_I(3)), FC_DIV(c, FC_I(3))));
    beta = FC_SUB(FC_DIV(b, FC_SQRT(FC_I(3))), FC_DIV(c, FC_SQRT(FC_I(3))));
    FC_STORE(alpha, &result.alpha); FC_STORE(beta, &result.beta);
    if (status == LINEARS_OK) *stationary = result;
    return status;
}

linears_status_t FC_N(inverse_clarke)(FC_TYPE(ab) stationary, FC_TYPE(abc) *phase) {
    linears_status_t status = LINEARS_OK;
    FC_W a, b, root3half, half;
    FC_TYPE(abc) result;
    if (phase == NULL) return LINEARS_ERROR_ARGUMENT;
    a = FC_LOAD(stationary.alpha); b = FC_LOAD(stationary.beta);
    half = FC_RATIO(1, 2); root3half = FC_MUL(FC_SQRT(FC_I(3)), half);
    FC_STORE(a, &result.a);
    FC_STORE(FC_SUB(FC_MUL(root3half, b), FC_MUL(half, a)), &result.b);
    FC_STORE(FC_NEG(FC_ADD(FC_MUL(half, a), FC_MUL(root3half, b))), &result.c);
    if (status == LINEARS_OK) *phase = result;
    return status;
}

linears_status_t FC_N(park)(FC_TYPE(ab) stationary, FC_T angle, FC_TYPE(dq) *rotating) {
    linears_status_t status = LINEARS_OK;
    FC_W a, b, theta, s = FC_I(0), c = FC_I(0);
    FC_TYPE(dq) result;
    if (rotating == NULL) return LINEARS_ERROR_ARGUMENT;
    a = FC_LOAD(stationary.alpha); b = FC_LOAD(stationary.beta); theta = FC_LOAD(angle);
    FC_SINCOS(theta, &s, &c);
    FC_STORE(FC_ADD(FC_MUL(a, c), FC_MUL(b, s)), &result.d);
    FC_STORE(FC_SUB(FC_MUL(b, c), FC_MUL(a, s)), &result.q);
    if (status == LINEARS_OK) *rotating = result;
    return status;
}

linears_status_t FC_N(inverse_park)(FC_TYPE(dq) rotating, FC_T angle, FC_TYPE(ab) *stationary) {
    linears_status_t status = LINEARS_OK;
    FC_W d, q, theta, s = FC_I(0), c = FC_I(0);
    FC_TYPE(ab) result;
    if (stationary == NULL) return LINEARS_ERROR_ARGUMENT;
    d = FC_LOAD(rotating.d); q = FC_LOAD(rotating.q); theta = FC_LOAD(angle);
    FC_SINCOS(theta, &s, &c);
    FC_STORE(FC_SUB(FC_MUL(d, c), FC_MUL(q, s)), &result.alpha);
    FC_STORE(FC_ADD(FC_MUL(d, s), FC_MUL(q, c)), &result.beta);
    if (status == LINEARS_OK) *stationary = result;
    return status;
}

linears_status_t FC_N(limit_voltage)(FC_TYPE(ab) voltage, FC_T limit, FC_TYPE(ab) *limited) {
    linears_status_t status = LINEARS_OK;
    FC_W a, b, bound;
    FC_TYPE(ab) result;
    if (limited == NULL) return LINEARS_ERROR_ARGUMENT;
    a = FC_LOAD(voltage.alpha); b = FC_LOAD(voltage.beta); bound = FC_LOAD(limit);
    if (status != LINEARS_OK) return status;
    if (FC_CMP(bound, FC_I(0)) < 0) return LINEARS_ERROR_DOMAIN;
    status = FC_N(base_limit)(a, b, bound, &a, &b);
    FC_STORE(a, &result.alpha); FC_STORE(b, &result.beta);
    if (status == LINEARS_OK) *limited = result;
    return status;
}

linears_status_t FC_N(svpwm)(FC_TYPE(ab) voltage, FC_T dc_bus, FC_TYPE(abc) *duty) {
    linears_status_t status = LINEARS_OK;
    FC_W a, b, bus;
    if (duty == NULL) return LINEARS_ERROR_ARGUMENT;
    a = FC_LOAD(voltage.alpha); b = FC_LOAD(voltage.beta); bus = FC_LOAD(dc_bus);
    if (status != LINEARS_OK) return status;
    if (FC_CMP(bus, FC_I(0)) <= 0) return LINEARS_ERROR_DOMAIN;
    return FC_N(base_pwm)(a, b, bus, duty);
}

linears_status_t FC_N(pi_init)(FC_TYPE(pi) *ctx, FC_T kp, FC_T ki,
                              FC_T lower, FC_T upper, FC_T integral) {
    linears_status_t status = LINEARS_OK;
    FC_W p, i, lo, hi, initial;
    FC_TYPE(pi) result;
    if (ctx == NULL) return LINEARS_ERROR_ARGUMENT;
    p = FC_LOAD(kp); i = FC_LOAD(ki); lo = FC_LOAD(lower); hi = FC_LOAD(upper); initial = FC_LOAD(integral);
    if (status != LINEARS_OK) return status;
    if (FC_CMP(p, FC_I(0)) < 0 || FC_CMP(i, FC_I(0)) < 0 || FC_CMP(lo, hi) >= 0 ||
        FC_CMP(initial, lo) < 0 || FC_CMP(initial, hi) > 0) return LINEARS_ERROR_DOMAIN;
    result.kp = kp; result.ki = ki; result.lower = lower; result.upper = upper;
    FC_ACC_STORE(initial, &result.integral);
    if (status == LINEARS_OK) *ctx = result;
    return status;
}

linears_status_t FC_N(pi_step)(FC_TYPE(pi) *ctx, FC_T error, FC_T dt, FC_T *output) {
    linears_status_t status = LINEARS_OK;
    FC_W e, period, p, inc, next, u, old, lo, hi;
    FC_T result;
    FC_TYPE(pi) state;
    if (ctx == NULL || output == NULL) return LINEARS_ERROR_ARGUMENT;
    if (fc_overlap(ctx, sizeof(*ctx), output, sizeof(*output))) return LINEARS_ERROR_ALIAS;
    e = FC_LOAD(error); period = FC_LOAD(dt);
    if (status != LINEARS_OK) return status;
    if (FC_CMP(period, FC_I(0)) <= 0) return LINEARS_ERROR_DOMAIN;
    state = *ctx;
    old = state.integral; lo = FC_LOAD(state.lower); hi = FC_LOAD(state.upper);
    p = FC_MUL(FC_LOAD(state.kp), e);
    inc = FC_MUL(FC_MUL(FC_LOAD(state.ki), period), e);
    next = FC_N(base_clamp)(FC_ADD(old, inc), lo, hi);
    u = FC_ADD(p, next);
    if ((FC_CMP(u, hi) > 0 && FC_CMP(inc, FC_I(0)) > 0) ||
        (FC_CMP(u, lo) < 0 && FC_CMP(inc, FC_I(0)) < 0)) next = old;
    u = FC_N(base_clamp)(FC_ADD(p, next), lo, hi);
    FC_ACC_STORE(next, &state.integral);
    FC_STORE(u, &result);
    if (status == LINEARS_OK) { *ctx = state; *output = result; }
    return status;
}

linears_status_t FC_N(vvvf_init)(FC_TYPE(vvvf) *ctx, FC_T angle) {
    linears_status_t status = LINEARS_OK;
    FC_TYPE(vvvf) result;
    FC_W a;
    if (ctx == NULL) return LINEARS_ERROR_ARGUMENT;
    a = FC_WRAP(FC_LOAD(angle));
    FC_ACC_STORE(a, &result.angle);
    if (status == LINEARS_OK) *ctx = result;
    return status;
}

linears_status_t FC_N(vvvf_step)(FC_TYPE(vvvf) *ctx, FC_T frequency,
                                FC_T amplitude, FC_T dc_bus, FC_T dt,
                                FC_TYPE(vvvf_output) *output) {
    linears_status_t status = LINEARS_OK;
    FC_W f, amp, bus, period, advance, angle, limit, v, a, b, scale;
    FC_W s = FC_I(0), c = FC_I(0);
    FC_TYPE(vvvf_output) result;
    FC_TYPE(vvvf) state;
    if (ctx == NULL || output == NULL) return LINEARS_ERROR_ARGUMENT;
    if (fc_overlap(ctx, sizeof(*ctx), output, sizeof(*output))) return LINEARS_ERROR_ALIAS;
    f = FC_LOAD(frequency); amp = FC_LOAD(amplitude); bus = FC_LOAD(dc_bus); period = FC_LOAD(dt);
    if (status != LINEARS_OK) return status;
    if (FC_CMP(amp, FC_I(0)) < 0 || FC_CMP(bus, FC_I(0)) <= 0 || FC_CMP(period, FC_I(0)) <= 0)
        return LINEARS_ERROR_DOMAIN;
    advance = FC_MUL(f, period);
    if (status != LINEARS_OK) return status;
    if (FC_CMP(FC_ABS(advance), FC_RATIO(1, 2)) > 0) return LINEARS_ERROR_DOMAIN;
    state = *ctx;
    angle = FC_WRAP(FC_ADD(state.angle, advance));
    /* Preserve angle and modulation in dimensionless coordinates even when
     * the final physical-voltage components round to subnormal values. */
    scale = FC_CMP(amp, bus) > 0 ? amp : bus;
    v = FC_DIV(amp, scale);
    limit = FC_DIV(FC_I(1), FC_SQRT(FC_I(3)));
    result.saturated = FC_CMP(v, limit) > 0;
    if (result.saturated) v = limit;
    FC_SINCOS(angle, &s, &c);
    a = FC_MUL(v, c); b = FC_MUL(v, s);
    FC_STORE(FC_MUL(a, bus), &result.voltage.alpha);
    FC_STORE(FC_MUL(b, bus), &result.voltage.beta);
    FC_ANGLE_STORE(angle, &result.angle);
    FC_ACC_STORE(angle, &state.angle);
    if (status != LINEARS_OK) return status;
    status = FC_N(base_pwm)(a, b, FC_I(1), &result.duty);
    if (status == LINEARS_OK) { *ctx = state; *output = result; }
    return status;
}
