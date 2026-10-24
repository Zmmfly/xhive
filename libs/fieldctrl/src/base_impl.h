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

/* Integral bounds of an initialized context: the output range unless the
 * independent limit is enabled. */
static void FC_N(pid_integral_bounds)(const FC_TYPE(pid_config) *config, FC_W *lo, FC_W *hi) {
    if ((config->options & FIELDCTRL_PID_OPT_INTEGRAL_LIMIT) != 0u) {
        *lo = FC_FIELD(config->integral_lower);
        *hi = FC_FIELD(config->integral_upper);
    } else {
        *lo = FC_FIELD(config->lower);
        *hi = FC_FIELD(config->upper);
    }
}

/* Validate a complete tuning and return its integral bounds in working format.
 * Shared by init and retune, so both accept exactly the same configurations. */
static linears_status_t FC_N(pid_check)(const FC_TYPE(pid_config) *config, FC_W *integral_lo,
                                        FC_W *integral_hi) {
    linears_status_t status = LINEARS_OK;
    FC_W zero, kp, ki, kd, lo, hi, filter, weight, ilo, ihi, separation;
    uint32_t options = config->options;
    int positive, negative;
    zero = FC_I(0);
    /* Every field is loaded, so nonfinite floating values are rejected even
     * when the option that reads them is disabled. */
    kp = FC_COEFF_LOAD(config->kp); ki = FC_COEFF_LOAD(config->ki); kd = FC_COEFF_LOAD(config->kd);
    lo = FC_LOAD(config->lower); hi = FC_LOAD(config->upper);
    filter = FC_COEFF_LOAD(config->derivative_filter);
    weight = FC_COEFF_LOAD(config->setpoint_weight);
    ilo = FC_LOAD(config->integral_lower); ihi = FC_LOAD(config->integral_upper);
    separation = FC_LOAD(config->separation);
    if (status != LINEARS_OK) return status;
    /* All gains nonnegative is direct action, all nonpositive reverse action.
     * The step equations are linear in the gains, so both are well defined;
     * mixed signs would make the terms fight each other. */
    positive = FC_CMP(kp, zero) > 0 || FC_CMP(ki, zero) > 0 || FC_CMP(kd, zero) > 0;
    negative = FC_CMP(kp, zero) < 0 || FC_CMP(ki, zero) < 0 || FC_CMP(kd, zero) < 0;
    if ((positive && negative) ||
        FC_CMP(lo, hi) >= 0 || FC_CMP(filter, zero) < 0 ||
        (options & ~(uint32_t)FIELDCTRL_PID_OPTIONS) != 0u ||
        (config->derivative != FIELDCTRL_PID_DERIVATIVE_ON_ERROR &&
         config->derivative != FIELDCTRL_PID_DERIVATIVE_ON_MEASUREMENT))
        return LINEARS_ERROR_DOMAIN;
    if ((options & FIELDCTRL_PID_OPT_SETPOINT_WEIGHT) != 0u &&
        (FC_CMP(weight, zero) < 0 || FC_CMP(weight, FC_I(1)) > 0))
        return LINEARS_ERROR_DOMAIN;
    if ((options & FIELDCTRL_PID_OPT_INTEGRAL_SEPARATION) != 0u && FC_CMP(separation, zero) <= 0)
        return LINEARS_ERROR_DOMAIN;
    if ((options & FIELDCTRL_PID_OPT_INTEGRAL_LIMIT) != 0u) {
        if (FC_CMP(ilo, ihi) > 0) return LINEARS_ERROR_DOMAIN;
        lo = ilo; hi = ihi;
    }
    *integral_lo = lo;
    *integral_hi = hi;
    return LINEARS_OK;
}

linears_status_t FC_N(pid_init)(FC_TYPE(pid) *ctx, const FC_TYPE(pid_config) *config,
                               FC_T integral) {
    linears_status_t status = LINEARS_OK;
    FC_W zero, lo, hi, initial;
    FC_TYPE(pid) next;
    if (ctx == NULL || config == NULL) return LINEARS_ERROR_ARGUMENT;
    zero = FC_I(0);
    initial = FC_LOAD(integral);
    if (status != LINEARS_OK) return status;
    status = FC_N(pid_check)(config, &lo, &hi);
    if (status != LINEARS_OK) return status;
    if (FC_CMP(initial, lo) < 0 || FC_CMP(initial, hi) > 0) return LINEARS_ERROR_DOMAIN;
    next.config = *config;
    next.integral = initial;
    next.previous_error = zero;
    next.previous_measurement = zero;
    next.derivative = zero;
    next.cached_dt = zero;
    next.integral_step = zero;
    next.derivative_gain = zero;
    next.filter_weight = zero;
    next.history = 0;
    *ctx = next;
    return LINEARS_OK;
}

linears_status_t FC_N(pid_retune)(FC_TYPE(pid) *ctx, const FC_TYPE(pid_config) *config) {
    linears_status_t status = LINEARS_OK;
    FC_W zero, lo, hi;
    if (ctx == NULL || config == NULL) return LINEARS_ERROR_ARGUMENT;
    zero = FC_I(0);
    status = FC_N(pid_check)(config, &lo, &hi);
    if (status != LINEARS_OK) return status;
    /* The cached period coefficients are ki*dt, kd/dt and dt/(Tf+dt); a zero
     * cached period forces the next step to recompute them. Bounds, options
     * and kp do not enter the cache, so a pure limit change keeps it. */
    if (FC_CMP(FC_COEFF_FIELD(config->ki), FC_COEFF_FIELD(ctx->config.ki)) != 0 ||
        FC_CMP(FC_COEFF_FIELD(config->kd), FC_COEFF_FIELD(ctx->config.kd)) != 0 ||
        FC_CMP(FC_COEFF_FIELD(config->derivative_filter),
               FC_COEFF_FIELD(ctx->config.derivative_filter)) != 0)
        ctx->cached_dt = zero;
    /* The step holds D(n-1) while kd is zero, so a stale term must not survive. */
    if (FC_CMP(FC_COEFF_FIELD(config->kd), zero) == 0) ctx->derivative = zero;
    ctx->integral = FC_N(base_clamp)(ctx->integral, lo, hi);
    ctx->config = *config;
    return LINEARS_OK;
}

linears_status_t FC_N(pid_reset)(FC_TYPE(pid) *ctx, FC_T integral) {
    linears_status_t status = LINEARS_OK;
    FC_W zero, lo, hi, initial;
    if (ctx == NULL) return LINEARS_ERROR_ARGUMENT;
    zero = FC_I(0);
    initial = FC_LOAD(integral);
    if (status != LINEARS_OK) return status;
    FC_N(pid_integral_bounds)(&ctx->config, &lo, &hi);
    if (FC_CMP(initial, lo) < 0 || FC_CMP(initial, hi) > 0) return LINEARS_ERROR_DOMAIN;
    /* The dt cache depends only on the unchanged tuning and stays valid. */
    ctx->integral = initial;
    ctx->previous_error = zero;
    ctx->previous_measurement = zero;
    ctx->derivative = zero;
    ctx->history = 0;
    return LINEARS_OK;
}

linears_status_t FC_N(pid_step_work)(FC_TYPE(pid) *ctx, FC_W setpoint, FC_W measurement,
                                    FC_W period, FC_TYPE(pid_output) *output) {
    linears_status_t status = LINEARS_OK;
    const FC_TYPE(pid_config) *config = &ctx->config;
    FC_W zero, kd, filter, lo, hi, ilo, ihi, error, p, old, i, d, delta, pd, raw, boundary;
    FC_W integral_step, derivative_gain, filter_weight;
    uint32_t options;
    FC_TYPE(pid_output) result;
    zero = FC_I(0);
    if (FC_CMP(period, zero) <= 0) return LINEARS_ERROR_DOMAIN;
    /* Work in locals and write ctx only after every operation succeeded:
     * copying the whole context in and out would dominate a PI-only step.
     * Configuration fields were validated by init and are read unchecked. */
    options = config->options;
    FC_N(pid_integral_bounds)(config, &ilo, &ihi);
    kd = FC_COEFF_FIELD(config->kd);
    filter = FC_COEFF_FIELD(config->derivative_filter);
    lo = FC_FIELD(config->lower);
    hi = FC_FIELD(config->upper);
    error = FC_SUB(setpoint, measurement);
    p = (options & FIELDCTRL_PID_OPT_SETPOINT_WEIGHT) != 0u
        ? FC_SUB(FC_MUL(FC_COEFF_FIELD(config->setpoint_weight), setpoint), measurement)
        : error;
    p = FC_MUL(FC_COEFF_FIELD(config->kp), p);
    /* Period-derived coefficients are recomputed only when dt changes, which
     * keeps both divisions out of fixed-rate loops. ki*dt is the same product
     * a per-step evaluation would round, so caching does not change results. */
    integral_step = ctx->integral_step;
    derivative_gain = ctx->derivative_gain;
    filter_weight = ctx->filter_weight;
    if (FC_CMP(period, ctx->cached_dt) != 0) {
        integral_step = FC_MUL(FC_COEFF_FIELD(config->ki), period);
        if (FC_CMP(kd, zero) != 0) derivative_gain = FC_DIV(kd, period);
        if (FC_CMP(filter, zero) > 0) filter_weight = FC_DIV(period, FC_ADD(filter, period));
    }
    old = ctx->integral;
    i = old;
    if ((options & FIELDCTRL_PID_OPT_INTEGRAL_SEPARATION) == 0u ||
        FC_CMP(FC_ABS(error), FC_FIELD(config->separation)) <= 0)
        i = FC_N(base_clamp)(FC_ADD(old, FC_MUL(integral_step, error)), ilo, ihi);
    /* The first step after init/reset has no previous sample to differentiate. */
    d = ctx->derivative;
    if (ctx->history && FC_CMP(kd, zero) != 0) {
        delta = config->derivative == FIELDCTRL_PID_DERIVATIVE_ON_MEASUREMENT
            ? FC_SUB(ctx->previous_measurement, measurement)
            : FC_SUB(error, ctx->previous_error);
        delta = FC_MUL(derivative_gain, delta);
        d = FC_CMP(filter, zero) > 0 ? FC_ADD(d, FC_MUL(filter_weight, FC_SUB(delta, d)))
                                     : delta;
    }
    pd = FC_ADD(p, d);
    raw = FC_ADD(pd, i);
    if (status != LINEARS_OK) return status;
    result.saturated = FC_CMP(raw, hi) > 0 || FC_CMP(raw, lo) < 0;
    /* The anti-windup below never moves the command back inside a crossed
     * bound, so the output is the clamped candidate command. */
    FC_STORE(FC_N(base_clamp)(raw, lo, hi), &result.output);
    /* Anti-windup: I may advance up to the value that places the unclamped
     * command exactly on the crossed bound, but not beyond it and never back
     * past I(n-1), so the limiter never creates integral state of its own. */
    if (FC_CMP(raw, hi) > 0 && FC_CMP(i, old) > 0) {
        boundary = FC_SUB(hi, pd);
        i = FC_CMP(boundary, old) > 0 ? boundary : old;
    } else if (FC_CMP(raw, lo) < 0 && FC_CMP(i, old) < 0) {
        boundary = FC_SUB(lo, pd);
        i = FC_CMP(boundary, old) < 0 ? boundary : old;
    }
    FC_ACC_STORE(p, &result.proportional);
    FC_ACC_STORE(i, &result.integral);
    FC_ACC_STORE(d, &result.derivative);
    if (status != LINEARS_OK) return status;
    ctx->cached_dt = period;
    ctx->integral_step = integral_step;
    ctx->derivative_gain = derivative_gain;
    ctx->filter_weight = filter_weight;
    ctx->integral = i;
    ctx->previous_error = error;
    ctx->previous_measurement = measurement;
    ctx->derivative = d;
    ctx->history = 1;
    *output = result;
    return LINEARS_OK;
}

linears_status_t FC_N(pid_step)(FC_TYPE(pid) *ctx, FC_T setpoint, FC_T measurement,
                               FC_T dt, FC_TYPE(pid_output) *output) {
    linears_status_t status = LINEARS_OK;
    FC_W reference, feedback, period;
    if (ctx == NULL || output == NULL) return LINEARS_ERROR_ARGUMENT;
    if (fc_overlap(ctx, sizeof(*ctx), output, sizeof(*output))) return LINEARS_ERROR_ALIAS;
    reference = FC_LOAD(setpoint);
    feedback = FC_LOAD(measurement);
    period = FC_LOAD(dt);
    if (status != LINEARS_OK) return status;
    return FC_N(pid_step_work)(ctx, reference, feedback, period, output);
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
