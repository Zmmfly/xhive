/* Four C99 instantiations; no include guard. All q arithmetic uses linears. */

/* Half-open sectors [center-1/12, center+1/12) turns. Comparing slopes avoids
 * rounding atan2 to the public q15 angle precision before choosing a sector. */
static uint8_t FC_N(dtc_sector)(FC_W alpha, FC_W beta,
                                linears_status_t *arithmetic_status) {
    linears_status_t status = *arithmetic_status;
    FC_W zero = FC_I(0);
    FC_W abs_alpha, abs_beta, slope, threshold;
    int sx = FC_CMP(alpha, zero), sy = FC_CMP(beta, zero), slope_compare;
    uint8_t sector;
    if (sy == 0) return (uint8_t)(sx > 0 ? 1 : 4);
    if (sx == 0) return (uint8_t)(sy > 0 ? 3 : 6);
    abs_alpha = FC_ABS(alpha);
    abs_beta = FC_ABS(beta);
    if (FC_CMP(abs_beta, abs_alpha) >= 0) {
        slope_compare = 1;
    } else {
        /* Divide the coordinates before comparing. alpha/sqrt(3) can round
         * to beta for subnormal (2*min, min), incorrectly making a 30-degree
         * boundary from a 26.565-degree vector. This ratio is bounded by 1. */
        slope = FC_DIV(abs_beta, abs_alpha);
        threshold = FC_SQRT(FC_I(3));
        threshold = FC_DIV(FC_I(1), threshold);
        slope_compare = FC_CMP(slope, threshold);
    }
    if (sx > 0) {
        if (sy > 0) sector = (uint8_t)(slope_compare < 0 ? 1 : 2);
        else sector = (uint8_t)(slope_compare > 0 ? 6 : 1);
    } else {
        if (sy > 0) sector = (uint8_t)(slope_compare > 0 ? 3 : 4);
        else sector = (uint8_t)(slope_compare < 0 ? 4 : 5);
    }
    *arithmetic_status = status;
    return sector;
}

linears_status_t FC_N(dtc_init)(FC_TYPE(dtc) *ctx,
                                FC_TYPE(dtc_config) config,
                                FC_TYPE(ab) initial_flux) {
    linears_status_t status = LINEARS_OK;
    FC_TYPE(dtc) next;
    FC_W r, fb, tb, alpha, beta, zero;
    if (ctx == NULL) return LINEARS_ERROR_ARGUMENT;
    r = FC_LOAD(config.resistance);
    fb = FC_LOAD(config.flux_band);
    tb = FC_LOAD(config.torque_band);
    alpha = FC_LOAD(initial_flux.alpha);
    beta = FC_LOAD(initial_flux.beta);
    zero = FC_I(0);
    if (status != LINEARS_OK) return status;
    if (config.pole_pairs == 0 || FC_CMP(r, zero) < 0 ||
        FC_CMP(fb, zero) <= 0 || FC_CMP(tb, zero) <= 0)
        return LINEARS_ERROR_ARGUMENT;
    if (FC_CMP(alpha, zero) == 0 && FC_CMP(beta, zero) == 0)
        return LINEARS_ERROR_DOMAIN;
    next.config = config;
    next.flux_alpha = alpha;
    next.flux_beta = beta;
    next.flux_state = 1;
    next.torque_state = 0;
    next.last_switch = 0;
    *ctx = next;
    return LINEARS_OK;
}

linears_status_t FC_N(dtc_step)(FC_TYPE(dtc) *ctx,
                                FC_TYPE(ab) current,
                                FC_TYPE(ab) applied_voltage,
                                FC_T flux_reference,
                                FC_T torque_reference,
                                FC_T dt,
                                FC_TYPE(dtc_result) *out) {
    linears_status_t status = LINEARS_OK;
    FC_TYPE(dtc_result) result;
    FC_W r, fb, tb, flux_ref, torque_ref, interval;
    FC_W ia, ib, va, vb, alpha, beta, zero;
    FC_W delta, magnitude, torque, cross_a, cross_b, gain;
    FC_W flux_error, torque_error, negative_band;
    int8_t flux_state, torque_state;
    if (ctx == NULL || out == NULL) return LINEARS_ERROR_ARGUMENT;
    if (fc_overlap(ctx, sizeof(*ctx), out, sizeof(*out)))
        return LINEARS_ERROR_ALIAS;
    r = FC_LOAD(ctx->config.resistance);
    fb = FC_LOAD(ctx->config.flux_band);
    tb = FC_LOAD(ctx->config.torque_band);
    flux_ref = FC_LOAD(flux_reference);
    torque_ref = FC_LOAD(torque_reference);
    interval = FC_LOAD(dt);
    ia = FC_LOAD(current.alpha);
    ib = FC_LOAD(current.beta);
    va = FC_LOAD(applied_voltage.alpha);
    vb = FC_LOAD(applied_voltage.beta);
    alpha = ctx->flux_alpha;
    beta = ctx->flux_beta;
    zero = FC_I(0);
    if (status != LINEARS_OK) return status;
    if (FC_CMP(flux_ref, fb) <= 0 || FC_CMP(interval, zero) <= 0)
        return LINEARS_ERROR_ARGUMENT;

    delta = FC_MUL(r, ia);
    delta = FC_SUB(va, delta);
    delta = FC_MUL(interval, delta);
    alpha = FC_ADD(alpha, delta);
    delta = FC_MUL(r, ib);
    delta = FC_SUB(vb, delta);
    delta = FC_MUL(interval, delta);
    beta = FC_ADD(beta, delta);
    if (status != LINEARS_OK) return status;
    if (FC_CMP(alpha, zero) == 0 && FC_CMP(beta, zero) == 0)
        return LINEARS_ERROR_DOMAIN;

    magnitude = FC_HYPOT(alpha, beta);
    cross_a = FC_MUL(alpha, ib);
    cross_b = FC_MUL(beta, ia);
    torque = FC_SUB(cross_a, cross_b);
    gain = FC_MUL(FC_RATIO(3, 2), FC_I(ctx->config.pole_pairs));
    torque = FC_MUL(gain, torque);
    flux_error = FC_SUB(flux_ref, magnitude);
    torque_error = FC_SUB(torque_ref, torque);
    if (status != LINEARS_OK) return status;

    flux_state = ctx->flux_state;
    negative_band = FC_NEG(fb);
    if (FC_CMP(flux_error, fb) >= 0) flux_state = 1;
    else if (FC_CMP(flux_error, negative_band) <= 0) flux_state = -1;

    negative_band = FC_NEG(tb);
    if (FC_CMP(torque_error, tb) >= 0) torque_state = 1;
    else if (FC_CMP(torque_error, negative_band) <= 0) torque_state = -1;
    else if (ctx->torque_state == 1 && FC_CMP(torque_error, zero) > 0)
        torque_state = 1;
    else if (ctx->torque_state == -1 && FC_CMP(torque_error, zero) < 0)
        torque_state = -1;
    else torque_state = 0;

    result.sector = FC_N(dtc_sector)(alpha, beta, &status);
    result.flux_state = flux_state;
    result.torque_state = torque_state;
    result.switch_state = torque_state == 0
        ? dtc_zero_state(ctx->last_switch)
        : dtc_active_table[flux_state > 0][torque_state > 0][result.sector - 1u];
    FC_STORE(alpha, &result.stator_flux.alpha);
    FC_STORE(beta, &result.stator_flux.beta);
    FC_STORE(magnitude, &result.flux_magnitude);
    FC_STORE(torque, &result.torque);
    if (status != LINEARS_OK) return status;

    /* Commit only after every arithmetic operation and output conversion. */
    ctx->flux_alpha = alpha;
    ctx->flux_beta = beta;
    ctx->flux_state = flux_state;
    ctx->torque_state = torque_state;
    ctx->last_switch = result.switch_state;
    *out = result;
    return LINEARS_OK;
}
