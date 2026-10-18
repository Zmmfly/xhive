/* Reused by four C99 instantiations. Fixed variants retain int64_t/F31 state;
 * public Q1 conversion happens only at the API boundary. */
#if FC_VARIANT == 32
#define FC_LP_INIT filters_lowpass_f32_init
#define FC_LP_RESET filters_lowpass_f32_reset
#define FC_LP_STEP filters_lowpass_f32_step
#elif FC_VARIANT == 64
#define FC_LP_INIT filters_lowpass_f64_init
#define FC_LP_RESET filters_lowpass_f64_reset
#define FC_LP_STEP filters_lowpass_f64_step
#else
/* Only bridge storage formats here. The exact F62 low-pass recurrence belongs
 * to filters and is shared with its public Q APIs. */
static linears_status_t FC_N(smo_lowpass_init)(filters_lowpass_q31_t *ctx,
    linears_fixed_t alpha, linears_fixed_t initial)
{
    linears_q31_t a, value;
    linears_status_t status = linears_fixed_to_q31(alpha, &a);
    if (status != LINEARS_OK) return status;
    status = linears_fixed_to_q31(initial, &value);
    if (status != LINEARS_OK) return status;
    return filters_lowpass_q31_init(ctx, a, value);
}

static linears_status_t FC_N(smo_lowpass_reset)(filters_lowpass_q31_t *ctx,
    linears_fixed_t initial)
{
    linears_q31_t value;
    linears_status_t status = linears_fixed_to_q31(initial, &value);
    if (status != LINEARS_OK) return status;
    return filters_lowpass_q31_reset(ctx, value);
}

static linears_status_t FC_N(smo_lowpass_step)(filters_lowpass_q31_t *ctx,
    linears_fixed_t input, linears_fixed_t *output)
{
    linears_status_t status = filters_lowpass_work_step(&ctx->value, ctx->alpha, input);
    if (status == LINEARS_OK) *output = ctx->value;
    return status;
}
#define FC_LP_INIT FC_N(smo_lowpass_init)
#define FC_LP_RESET FC_N(smo_lowpass_reset)
#define FC_LP_STEP FC_N(smo_lowpass_step)
#endif

linears_status_t FC_N(smo_init)(FC_TYPE(smo) *ctx,
    const FC_TYPE(smo_config) *config, FC_TYPE(ab) initial_current)
{
    linears_status_t status = LINEARS_OK;
    FC_TYPE(smo) next;
    FC_W resistance, inductance, period, gain, boundary;
    FC_W emf_alpha, speed_alpha, min_emf, min_speed, max_speed;
    FC_W zero, one, factor, sample_angle;
    if (ctx == NULL || config == NULL) return LINEARS_ERROR_ARGUMENT;
    if (config->lock_samples == 0 ||
        (config->direction != FIELDCTRL_SMO_FORWARD &&
         config->direction != FIELDCTRL_SMO_REVERSE))
        return LINEARS_ERROR_DOMAIN;
    resistance = FC_LOAD(config->resistance);
    inductance = FC_LOAD(config->inductance);
    period = FC_LOAD(config->dt);
    gain = FC_LOAD(config->sliding_gain);
    boundary = FC_LOAD(config->boundary);
    emf_alpha = FC_LOAD(config->emf_alpha);
    speed_alpha = FC_LOAD(config->speed_alpha);
    min_emf = FC_LOAD(config->min_emf);
    min_speed = FC_LOAD(config->min_speed);
    max_speed = FC_LOAD(config->max_speed);
    next.current_hat.alpha = FC_LOAD(initial_current.alpha);
    next.current_hat.beta = FC_LOAD(initial_current.beta);
    zero = FC_I(0);
    one = FC_I(1);
    if (status != LINEARS_OK) return status;
    if (FC_CMP(resistance, zero) < 0 || FC_CMP(inductance, zero) <= 0 ||
        FC_CMP(period, zero) <= 0 || FC_CMP(gain, zero) <= 0 ||
        FC_CMP(boundary, zero) <= 0 || FC_CMP(emf_alpha, zero) <= 0 ||
        FC_CMP(emf_alpha, one) > 0 || FC_CMP(speed_alpha, zero) <= 0 ||
        FC_CMP(speed_alpha, one) > 0 || FC_CMP(min_emf, zero) <= 0 ||
        FC_CMP(min_speed, zero) <= 0 || FC_CMP(max_speed, min_speed) <= 0)
        return LINEARS_ERROR_DOMAIN;
    factor = FC_ADD(resistance, FC_DIV(gain, boundary));
    factor = FC_MUL(period, FC_DIV(factor, inductance));
    sample_angle = FC_MUL(max_speed, period);
    if (status != LINEARS_OK) return status;
    if (FC_CMP(factor, FC_I(2)) >= 0 ||
        FC_CMP(sample_angle, FC_RATIO(1, 4)) >= 0)
        return LINEARS_ERROR_DOMAIN;
    status = FC_LP_INIT(&next.emf_alpha_filter, emf_alpha, zero);
    if (status != LINEARS_OK) return status;
    status = FC_LP_INIT(&next.emf_beta_filter, emf_alpha, zero);
    if (status != LINEARS_OK) return status;
    status = FC_LP_INIT(&next.speed_filter, speed_alpha, zero);
    if (status != LINEARS_OK) return status;
    next.config = *config;
    next.previous_angle = zero;
    next.angle = zero;
    next.has_angle = 0;
    next.lock_count = 0;
    *ctx = next;
    return LINEARS_OK;
}

linears_status_t FC_N(smo_step)(FC_TYPE(smo) *ctx, FC_TYPE(ab) current,
    FC_TYPE(ab) voltage, FC_TYPE(smo_output) *output)
{
    linears_status_t status = LINEARS_OK;
    FC_TYPE(smo) next;
    FC_TYPE(smo_output) result;
    FC_W ia, ib, va, vb, resistance, inductance, period, gain, boundary;
    FC_W error_a, error_b, sat_a, sat_b, za, zb, derivative, step;
    FC_W ea, eb, magnitude, raw_angle, difference, raw_speed, speed;
    FC_W sine, cosine, alpha, rho, numerator, denominator, phase;
    FC_W min_speed, max_speed, raw_magnitude, speed_magnitude;
    FC_W zero, one, negative_one;
#if FC_VARIANT == 15 || FC_VARIANT == 31
    FC_W negative_boundary;
#endif
    int had_angle, direction_ok;
    if (ctx == NULL || output == NULL) return LINEARS_ERROR_ARGUMENT;
    if (fc_overlap(ctx, sizeof(*ctx), output, sizeof(*output)))
        return LINEARS_ERROR_ALIAS;
    next = *ctx;
    ia = FC_LOAD(current.alpha);
    ib = FC_LOAD(current.beta);
    va = FC_LOAD(voltage.alpha);
    vb = FC_LOAD(voltage.beta);
    resistance = FC_LOAD(next.config.resistance);
    inductance = FC_LOAD(next.config.inductance);
    period = FC_LOAD(next.config.dt);
    gain = FC_LOAD(next.config.sliding_gain);
    boundary = FC_LOAD(next.config.boundary);
    zero = FC_I(0);
    one = FC_I(1);
    negative_one = FC_I(-1);
    error_a = FC_SUB(next.current_hat.alpha, ia);
    error_b = FC_SUB(next.current_hat.beta, ib);
#if FC_VARIANT == 15 || FC_VARIANT == 31
    negative_boundary = FC_NEG(boundary);
    if (status != LINEARS_OK) return status;
    /* Saturate before division. A valid F31 current error of +2 divided by
     * a one-LSB Q31 boundary exceeds the working range, although sat is +1. */
    sat_a = FC_CMP(error_a, boundary) >= 0 ? one :
        (FC_CMP(error_a, negative_boundary) <= 0 ? negative_one : FC_DIV(error_a, boundary));
    sat_b = FC_CMP(error_b, boundary) >= 0 ? one :
        (FC_CMP(error_b, negative_boundary) <= 0 ? negative_one : FC_DIV(error_b, boundary));
#else
    /* Preserve the floating API's division/range behavior. */
    sat_a = FC_DIV(error_a, boundary);
    sat_b = FC_DIV(error_b, boundary);
    if (status != LINEARS_OK) return status;
    if (FC_CMP(sat_a, one) > 0) sat_a = one;
    if (FC_CMP(sat_a, negative_one) < 0) sat_a = negative_one;
    if (FC_CMP(sat_b, one) > 0) sat_b = one;
    if (FC_CMP(sat_b, negative_one) < 0) sat_b = negative_one;
#endif
    za = FC_MUL(gain, sat_a);
    zb = FC_MUL(gain, sat_b);
    step = FC_DIV(period, inductance);
    derivative = FC_SUB(va, FC_MUL(resistance, next.current_hat.alpha));
    derivative = FC_SUB(derivative, za);
    next.current_hat.alpha = FC_ADD(next.current_hat.alpha,
        FC_MUL(step, derivative));
    derivative = FC_SUB(vb, FC_MUL(resistance, next.current_hat.beta));
    derivative = FC_SUB(derivative, zb);
    next.current_hat.beta = FC_ADD(next.current_hat.beta,
        FC_MUL(step, derivative));
    if (status != LINEARS_OK) return status;
    status = FC_LP_STEP(&next.emf_alpha_filter, za, &ea);
    if (status != LINEARS_OK) return status;
    status = FC_LP_STEP(&next.emf_beta_filter, zb, &eb);
    if (status != LINEARS_OK) return status;
    magnitude = FC_HYPOT(ea, eb);
    result.observable = 0;
    speed = zero;
    if (status != LINEARS_OK) return status;
    if (FC_CMP(magnitude, FC_LOAD(next.config.min_emf)) < 0) {
        /* A new angle must be acquired after EMF loss; differencing against
         * a stale angle would manufacture a speed spike at reacquisition. */
        next.has_angle = 0;
        status = FC_LP_RESET(&next.speed_filter, zero);
        if (status != LINEARS_OK) return status;
    } else {
        if (next.config.direction == FIELDCTRL_SMO_FORWARD)
            raw_angle = FC_ATAN2(FC_NEG(ea), eb);
        else
            raw_angle = FC_ATAN2(ea, FC_NEG(eb));
        had_angle = next.has_angle;
        raw_speed = zero;
        if (had_angle) {
            difference = FC_SUB(raw_angle, next.previous_angle);
            if (FC_CMP(difference, FC_RATIO(1, 2)) > 0)
                difference = FC_SUB(difference, one);
            if (FC_CMP(difference, FC_RATIO(-1, 2)) < 0)
                difference = FC_ADD(difference, one);
            raw_speed = FC_DIV(difference, period);
            if (status != LINEARS_OK) return status;
            status = FC_LP_STEP(&next.speed_filter, raw_speed, &speed);
            if (status != LINEARS_OK) return status;
        }
        next.previous_angle = raw_angle;
        next.has_angle = 1;
        /* H(z)=a/(1-(1-a)z^-1). Add -arg(H) at the measured signed
         * electrical frequency, not the continuous RC approximation. */
        step = FC_MUL(speed, period);
        sine = zero;
        cosine = one;
        FC_SINCOS(step, &sine, &cosine);
        alpha = FC_LOAD(next.config.emf_alpha);
        rho = FC_SUB(one, alpha);
        numerator = FC_MUL(rho, sine);
        /* alpha + rho*(1-cos) retains a tiny positive alpha even when
         * rounding makes rho exactly one (notably the first, zero-speed sample). */
        denominator = FC_ADD(alpha, FC_MUL(rho, FC_SUB(one, cosine)));
        phase = FC_ATAN2(numerator, denominator);
        next.angle = FC_WRAP(FC_ADD(raw_angle, phase));
        min_speed = FC_LOAD(next.config.min_speed);
        max_speed = FC_LOAD(next.config.max_speed);
        raw_magnitude = FC_ABS(raw_speed);
        speed_magnitude = FC_ABS(speed);
        if (status != LINEARS_OK) return status;
        direction_ok = next.config.direction == FIELDCTRL_SMO_FORWARD
            ? FC_CMP(raw_speed, zero) > 0 && FC_CMP(speed, zero) > 0
            : FC_CMP(raw_speed, zero) < 0 && FC_CMP(speed, zero) < 0;
        result.observable = had_angle && direction_ok &&
            FC_CMP(raw_magnitude, min_speed) >= 0 &&
            FC_CMP(raw_magnitude, max_speed) <= 0 &&
            FC_CMP(speed_magnitude, min_speed) >= 0 &&
            FC_CMP(speed_magnitude, max_speed) <= 0 &&
            FC_CMP(FC_ABS(error_a), boundary) <= 0 &&
            FC_CMP(FC_ABS(error_b), boundary) <= 0;
    }
    if (status != LINEARS_OK) return status;
    if (!result.observable)
        next.lock_count = 0;
    else if (next.lock_count < next.config.lock_samples)
        ++next.lock_count;
    result.locked = next.lock_count == next.config.lock_samples;
    FC_STORE(next.current_hat.alpha, &result.current_hat.alpha);
    FC_STORE(next.current_hat.beta, &result.current_hat.beta);
    FC_STORE(ea, &result.emf.alpha);
    FC_STORE(eb, &result.emf.beta);
    FC_ACC_STORE(speed, &result.electrical_speed);
    FC_ANGLE_STORE(next.angle, &result.electrical_angle);
    if (status != LINEARS_OK) return status;
    *ctx = next;
    *output = result;
    return LINEARS_OK;
}

#undef FC_LP_INIT
#undef FC_LP_RESET
#undef FC_LP_STEP
