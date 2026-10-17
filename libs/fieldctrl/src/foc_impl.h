/* Included once per numeric variant; only fieldctrl_arith owns arithmetic. */

/* Limit in working precision so q15 anti-windup does not depend on an
 * intermediate public-vector rounding. This is a circular, not axiswise,
 * actuator constraint. */
static linears_status_t FC_N(foc_radial_limit)(FC_W d, FC_W q, FC_W radius,
    FC_W *limited_d, FC_W *limited_q, int *saturated)
{
    linears_status_t status = LINEARS_OK;
    FC_W magnitude = FC_HYPOT(d, q);
    if (status != LINEARS_OK) return status;
    *saturated = FC_CMP(magnitude, radius) > 0;
    if (*saturated) {
        /* The shared scaled ratio avoids both a vanishing radius/norm and
         * a vanishing small component/norm when the final result is representable. */
        d = FC_RESCALE(d, radius, magnitude);
        q = FC_RESCALE(q, radius, magnitude);
    }
    if (status != LINEARS_OK) return status;
    *limited_d = d;
    *limited_q = q;
    return LINEARS_OK;
}

linears_status_t FC_N(foc_init)(FC_TYPE(foc) *ctx,
    const FC_TYPE(foc_config) *config)
{
    linears_status_t status = LINEARS_OK;
    FC_TYPE(foc) next;
    FC_W kp_d, ki_d, kp_q, ki_q, radius, zero;
    if (ctx == NULL || config == NULL) return LINEARS_ERROR_ARGUMENT;
    kp_d = FC_LOAD(config->kp_d);
    ki_d = FC_LOAD(config->ki_d);
    kp_q = FC_LOAD(config->kp_q);
    ki_q = FC_LOAD(config->ki_q);
    radius = FC_LOAD(config->voltage_limit);
    zero = FC_I(0);
    if (status != LINEARS_OK) return status;
    if (FC_CMP(kp_d, zero) < 0 || FC_CMP(ki_d, zero) < 0 ||
        FC_CMP(kp_q, zero) < 0 || FC_CMP(ki_q, zero) < 0 ||
        FC_CMP(radius, zero) <= 0) return LINEARS_ERROR_DOMAIN;
    next.config = *config;
    next.integral.d = zero;
    next.integral.q = zero;
    *ctx = next;
    return LINEARS_OK;
}

linears_status_t FC_N(foc_step)(FC_TYPE(foc) *ctx,
    FC_TYPE(ab) current, FC_TYPE(dq) reference, FC_T electrical_angle,
    FC_T dc_bus, FC_T dt, FC_TYPE(foc_output) *output)
{
    linears_status_t status = LINEARS_OK;
    FC_TYPE(foc) next;
    FC_TYPE(foc_output) result;
    FC_W zero, one, period, bus, angle, radius, bus_radius;
    FC_W error_d, error_q, delta_d, delta_q, p_d, p_q;
    FC_W candidate_d, candidate_q, raw_d, raw_q, limited_d, limited_q;
    FC_W projection, component, delta_scale, voltage_scale;
    if (ctx == NULL || output == NULL) return LINEARS_ERROR_ARGUMENT;
    if (fc_overlap(ctx, sizeof(*ctx), output, sizeof(*output)))
        return LINEARS_ERROR_ALIAS;
    period = FC_LOAD(dt);
    bus = FC_LOAD(dc_bus);
    angle = FC_LOAD(electrical_angle);
    zero = FC_I(0);
    one = FC_I(1);
    if (status != LINEARS_OK) return status;
    if (FC_CMP(period, zero) <= 0 || FC_CMP(bus, zero) <= 0 ||
        FC_CMP(angle, zero) < 0 || FC_CMP(angle, one) >= 0)
        return LINEARS_ERROR_DOMAIN;
    next = *ctx;
    status = FC_N(park)(current, electrical_angle, &result.current);
    if (status != LINEARS_OK) return status;
    error_d = FC_SUB(FC_LOAD(reference.d), FC_LOAD(result.current.d));
    error_q = FC_SUB(FC_LOAD(reference.q), FC_LOAD(result.current.q));
    p_d = FC_MUL(FC_LOAD(next.config.kp_d), error_d);
    p_q = FC_MUL(FC_LOAD(next.config.kp_q), error_q);
    delta_d = FC_MUL(FC_LOAD(next.config.ki_d), period);
    delta_d = FC_MUL(delta_d, error_d);
    delta_q = FC_MUL(FC_LOAD(next.config.ki_q), period);
    delta_q = FC_MUL(delta_q, error_q);
    candidate_d = FC_ADD(next.integral.d, delta_d);
    candidate_q = FC_ADD(next.integral.q, delta_q);
    raw_d = FC_ADD(p_d, candidate_d);
    raw_q = FC_ADD(p_q, candidate_q);
    radius = FC_LOAD(next.config.voltage_limit);
    bus_radius = FC_DIV(bus, FC_SQRT(FC_I(3)));
    if (FC_CMP(bus_radius, radius) < 0) radius = bus_radius;
    if (status != LINEARS_OK) return status;
    status = FC_N(foc_radial_limit)(raw_d, raw_q, radius,
        &limited_d, &limited_q, &result.saturated);
    if (status != LINEARS_OK) return status;
    if (result.saturated) {
        /* The request is collinear with the saturation residual. Normalize
         * BOTH vectors independently before testing their dot-product sign:
         * multiplying physical units can underflow/overflow even when every
         * controller input and stored integral is a normal finite value. */
        delta_scale = FC_ABS(delta_d);
        component = FC_ABS(delta_q);
        if (FC_CMP(component, delta_scale) > 0) delta_scale = component;
        voltage_scale = FC_ABS(raw_d);
        component = FC_ABS(raw_q);
        if (FC_CMP(component, voltage_scale) > 0) voltage_scale = component;
        projection = zero;
        if (FC_CMP(delta_scale, zero) > 0) {
            projection = FC_MUL(FC_DIV(delta_d, delta_scale),
                FC_DIV(raw_d, voltage_scale));
            component = FC_MUL(FC_DIV(delta_q, delta_scale),
                FC_DIV(raw_q, voltage_scale));
            projection = FC_ADD(projection, component);
        }
        if (status != LINEARS_OK) return status;
        if (FC_CMP(projection, zero) > 0) {
            candidate_d = next.integral.d;
            candidate_q = next.integral.q;
            raw_d = FC_ADD(p_d, candidate_d);
            raw_q = FC_ADD(p_q, candidate_q);
            if (status != LINEARS_OK) return status;
            status = FC_N(foc_radial_limit)(raw_d, raw_q, radius,
                &limited_d, &limited_q, &result.saturated);
            if (status != LINEARS_OK) return status;
        }
    }
    next.integral.d = candidate_d;
    next.integral.q = candidate_q;
    FC_STORE(raw_d, &result.requested_voltage.d);
    FC_STORE(raw_q, &result.requested_voltage.q);
    FC_STORE(limited_d, &result.voltage.d);
    FC_STORE(limited_q, &result.voltage.q);
    if (status != LINEARS_OK) return status;
    status = FC_N(inverse_park)(result.voltage, electrical_angle,
        &result.stationary_voltage);
    if (status != LINEARS_OK) return status;
    status = FC_N(svpwm)(result.stationary_voltage, dc_bus, &result.duty);
    if (status != LINEARS_OK) return status;
    *ctx = next;
    *output = result;
    return LINEARS_OK;
}

linears_status_t FC_N(speed_foc_init)(FC_TYPE(speed_foc) *ctx,
    const FC_TYPE(speed_foc_config) *config)
{
    linears_status_t status = LINEARS_OK;
    FC_TYPE(speed_foc) next;
    FC_W limit, zero;
    FC_T lower, initial;
    if (ctx == NULL || config == NULL) return LINEARS_ERROR_ARGUMENT;
    if (config->pole_pairs == 0) return LINEARS_ERROR_DOMAIN;
    limit = FC_LOAD(config->current_limit);
    zero = FC_I(0);
    if (status != LINEARS_OK) return status;
    if (FC_CMP(limit, zero) <= 0) return LINEARS_ERROR_DOMAIN;
    FC_STORE(FC_NEG(limit), &lower);
    FC_STORE(zero, &initial);
    if (status != LINEARS_OK) return status;
    status = FC_N(foc_init)(&next.current, &config->current);
    if (status != LINEARS_OK) return status;
    status = FC_N(pi_init)(&next.speed, config->kp, config->ki, lower,
        config->current_limit, initial);
    if (status != LINEARS_OK) return status;
    next.pole_pairs = config->pole_pairs;
    *ctx = next;
    return LINEARS_OK;
}

linears_status_t FC_N(speed_foc_step)(FC_TYPE(speed_foc) *ctx,
    FC_TYPE(ab) current, FC_T mechanical_reference, FC_T electrical_speed,
    FC_T electrical_angle, FC_T dc_bus, FC_T dt,
    FC_TYPE(speed_foc_output) *output)
{
    linears_status_t status = LINEARS_OK;
    FC_TYPE(speed_foc) next;
    FC_TYPE(speed_foc_output) result;
    FC_T error;
    FC_W mechanical, speed_error, residual, zero;
    if (ctx == NULL || output == NULL) return LINEARS_ERROR_ARGUMENT;
    if (fc_overlap(ctx, sizeof(*ctx), output, sizeof(*output)))
        return LINEARS_ERROR_ALIAS;
    next = *ctx;
    mechanical = FC_DIV(FC_LOAD(electrical_speed), FC_I(next.pole_pairs));
    speed_error = FC_SUB(FC_LOAD(mechanical_reference), mechanical);
    FC_STORE(mechanical, &result.mechanical_speed);
    FC_STORE(speed_error, &error);
    FC_STORE(FC_I(0), &result.reference.d);
    if (status != LINEARS_OK) return status;
    status = FC_N(pi_step)(&next.speed, error, dt, &result.reference.q);
    if (status != LINEARS_OK) return status;
    status = FC_N(foc_step)(&next.current, current, result.reference,
        electrical_angle, dc_bus, dt, &result.current);
    if (status != LINEARS_OK) return status;
    if (result.current.saturated) {
        residual = FC_SUB(FC_LOAD(result.current.requested_voltage.q),
            FC_LOAD(result.current.voltage.q));
        zero = FC_I(0);
        if (status != LINEARS_OK) return status;
        if ((FC_CMP(speed_error, zero) > 0 && FC_CMP(residual, zero) > 0) ||
            (FC_CMP(speed_error, zero) < 0 && FC_CMP(residual, zero) < 0))
            next.speed.integral = ctx->speed.integral;
    }
    *ctx = next;
    *output = result;
    return LINEARS_OK;
}
