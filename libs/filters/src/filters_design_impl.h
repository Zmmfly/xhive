/* Frequency-designed IIR filters (LPF/HPF/BPF) and windowed-sinc FIR taps,
 * instantiated once per scalar format after filters_impl.h. Designs work on
 * prewarped analog prototypes in F_D (float for f32, double otherwise); Q
 * formats round every designed coefficient to F31 exactly once. */

#define F_D FILTERS_REAL
#define F_HZ FILTERS_HZ
#define F_MATH(name) FILTERS_REAL_FN(name)
#define F_SECTION F_FN(iir_section, t)
#define F_PI ((F_D)3.14159265358979323846)
#define F_SQRT2 ((F_D)1.41421356237309504880)

#if FILTERS_FIXED
#define F_C_ONE LINEARS_FIXED_ONE

static F_C F_LOCAL(to_coefficient)(F_D value)
{
    return (F_C)F_MATH(round)(value * (F_D)LINEARS_FIXED_ONE);
}

static int F_LOCAL(representable)(F_D value)
{
    const F_D scaled = value * (F_D)LINEARS_FIXED_ONE;
    /* The comparisons are false for NaN as well. */
    return scaled > -9223372036854775808.0 && scaled < 9223372036854775808.0;
}
#else
#define F_C_ONE ((F_C)1)

static F_C F_LOCAL(to_coefficient)(F_D value)
{
    return (F_C)value;
}

static int F_LOCAL(representable)(F_D value)
{
    return isfinite(value);
}
#endif

static linears_status_t F_LOCAL(quantize)(F_D value, F_C *out)
{
    if (!F_LOCAL(representable)(value)) return LINEARS_ERROR_RANGE;
    *out = F_LOCAL(to_coefficient)(value);
    return LINEARS_OK;
}

/* f/fs, accepted only strictly inside the open Nyquist band. */
static linears_status_t F_LOCAL(ratio)(F_HZ frequency, F_HZ sample_rate, F_D *out)
{
    F_D ratio;
    if (!isfinite(frequency) || !isfinite(sample_rate)) return LINEARS_ERROR_RANGE;
    if (!(sample_rate > 0)) return LINEARS_ERROR_DOMAIN;
    ratio = (F_D)frequency / (F_D)sample_rate;
    if (!(ratio > 0 && ratio < (F_D)0.5)) return LINEARS_ERROR_DOMAIN;
    *out = ratio;
    return LINEARS_OK;
}

/* Prewarped analog frequency for the bilinear map s=(1-z^-1)/(1+z^-1). */
static linears_status_t F_LOCAL(prewarp)(F_HZ frequency, F_HZ sample_rate, F_D *out)
{
    F_D ratio, omega;
    F_TRY(F_LOCAL(ratio)(frequency, sample_rate, &ratio));
    omega = F_MATH(tan)(F_PI * ratio);
    /* In float, pi*ratio for a ratio just below one half can round past the
     * tangent pole and yield a negative value. */
    if (!(omega > 0)) return LINEARS_ERROR_DOMAIN;
    *out = omega;
    return LINEARS_OK;
}

/* Rounded Butterworth denominator 1 + a1*z^-1 + a2*z^-2. */
static linears_status_t F_LOCAL(butterworth)(filters_order_t order, F_HZ cutoff,
                                             F_HZ sample_rate, F_SECTION *section)
{
    F_D omega, a0;
    if (order != FILTERS_ORDER_1 && order != FILTERS_ORDER_2) return LINEARS_ERROR_DOMAIN;
    F_TRY(F_LOCAL(prewarp)(cutoff, sample_rate, &omega));
    if (order == FILTERS_ORDER_1) {
        /* s + w */
        a0 = 1 + omega;
        section->a1 = F_LOCAL(to_coefficient)((omega - 1) / a0);
        section->a2 = 0;
    } else {
        /* s^2 + sqrt(2)*w*s + w^2 */
        const F_D damping = F_SQRT2 * omega;
        const F_D square = omega * omega;
        a0 = 1 + damping + square;
        section->a1 = F_LOCAL(to_coefficient)(2 * (square - 1) / a0);
        section->a2 = F_LOCAL(to_coefficient)((1 - damping + square) / a0);
    }
    return LINEARS_OK;
}

/* The numerator is derived from the rounded denominator: its taps sum to
 * 1 + a1 + a2, so the stored DC gain is exactly one. */
static linears_status_t F_LOCAL(lowpass_design)(filters_order_t order, F_HZ cutoff,
                                                F_HZ sample_rate, F_SECTION *section)
{
    F_C sum;
    F_TRY(F_LOCAL(butterworth)(order, cutoff, sample_rate, section));
    sum = F_C_ONE + section->a1 + section->a2;
    if (order == FILTERS_ORDER_1) {
#if FILTERS_FIXED
        section->b0 = sum / 2;
        section->b1 = sum - section->b0;
#else
        section->b0 = sum / 2;
        section->b1 = section->b0;
#endif
        section->b2 = 0;
    } else {
#if FILTERS_FIXED
        section->b0 = (sum + 2) / 4;
        section->b1 = sum - 2 * section->b0;
#else
        section->b0 = sum / 4;
        section->b1 = sum / 2;
#endif
        section->b2 = section->b0;
    }
    return LINEARS_OK;
}

/* The numerator is scaled from the rounded denominator's Nyquist sum
 * 1 - a1 + a2; its taps cancel exactly, so the DC gain is exactly zero. */
static linears_status_t F_LOCAL(highpass_design)(filters_order_t order, F_HZ cutoff,
                                                 F_HZ sample_rate, F_SECTION *section)
{
    F_C sum;
    F_TRY(F_LOCAL(butterworth)(order, cutoff, sample_rate, section));
    sum = F_C_ONE - section->a1 + section->a2;
    if (order == FILTERS_ORDER_1) {
#if FILTERS_FIXED
        section->b0 = (sum + 1) / 2;
#else
        section->b0 = sum / 2;
#endif
        section->b1 = -section->b0;
        section->b2 = 0;
    } else {
#if FILTERS_FIXED
        section->b0 = (sum + 2) / 4;
#else
        section->b0 = sum / 4;
#endif
        section->b1 = -2 * section->b0;
        section->b2 = section->b0;
    }
    return LINEARS_OK;
}

/* B*s / (s^2 + p*s + q); b2 = -b0 and b1 = 0, so the DC gain is exactly zero. */
static linears_status_t F_LOCAL(bandpass_section)(F_D bandwidth, F_D p, F_D q,
                                                  F_SECTION *section)
{
    const F_D a0 = 1 + p + q;
    F_TRY(F_LOCAL(quantize)(bandwidth / a0, &section->b0));
    F_TRY(F_LOCAL(quantize)(2 * (q - 1) / a0, &section->a1));
    F_TRY(F_LOCAL(quantize)((1 - p + q) / a0, &section->a2));
    section->b1 = 0;
    section->b2 = -section->b0;
    return LINEARS_OK;
}

/* Band-pass around prewarped center w0 with prewarped bandwidth B. Order two
 * maps each pole p=(-1+-j)/sqrt(2) of the Butterworth prototype to the roots
 * of s^2 - p*B*s + w0^2, giving two stagger-tuned sections. The root formulas
 * are arranged so that no step subtracts nearly equal values. */
static linears_status_t F_LOCAL(bandpass_design)(filters_order_t order, F_D center,
                                                 F_D bandwidth, F_SECTION sections[2])
{
    const F_D center2 = center * center;
    if (order == FILTERS_ORDER_1) {
        const F_SECTION unused = {0};
        F_TRY(F_LOCAL(bandpass_section)(bandwidth, bandwidth, center2, &sections[0]));
        sections[1] = unused;
        return LINEARS_OK;
    }
    if (order != FILTERS_ORDER_2) return LINEARS_ERROR_DOMAIN;
    {
        const F_D c = 4 * center2;
        const F_D b2 = bandwidth * bandwidth;
        const F_D h = F_MATH(hypot)(c, b2);
        const F_D m = h + c;
        const F_D imaginary = F_MATH(sqrt)(m / 2);
        const F_D real = b2 / (2 * imaginary);
        const F_D u = bandwidth / F_SQRT2;
        const F_D p_high = u + real;
        const F_D q_high = (p_high * p_high + (u + imaginary) * (u + imaginary)) / 4;
        /* u - real, rewritten without cancellation. */
        const F_D p_low = b2 * (c + c * c / (h + b2)) / (2 * m * p_high);
        /* Each prototype pole's two roots multiply to w0^2. */
        const F_D q_low = center2 * center2 / q_high;
        F_TRY(F_LOCAL(bandpass_section)(bandwidth, p_low, q_low, &sections[0]));
        F_TRY(F_LOCAL(bandpass_section)(bandwidth, p_high, q_high, &sections[1]));
    }
    return LINEARS_OK;
}

static linears_status_t F_LOCAL(band_edges)(F_HZ low, F_HZ high, F_HZ sample_rate,
                                            F_D *center, F_D *bandwidth)
{
    F_D lower, upper;
    F_TRY(F_LOCAL(prewarp)(low, sample_rate, &lower));
    F_TRY(F_LOCAL(prewarp)(high, sample_rate, &upper));
    /* Also rejects distinct edges that round to one prewarped value. */
    if (!(upper > lower)) return LINEARS_ERROR_DOMAIN;
    *center = F_MATH(sqrt)(lower * upper);
    *bandwidth = upper - lower;
    return LINEARS_OK;
}

static linears_status_t F_LOCAL(center_q)(F_HZ center_hz, F_HZ q, F_HZ sample_rate,
                                          F_D *center, F_D *bandwidth)
{
    F_D omega, width;
    if (!isfinite(q)) return LINEARS_ERROR_RANGE;
    F_TRY(F_LOCAL(prewarp)(center_hz, sample_rate, &omega));
    if (!(q > 0)) return LINEARS_ERROR_DOMAIN;
    width = omega / (F_D)q;
    if (!(width > 0)) return LINEARS_ERROR_DOMAIN;
    *center = omega;
    *bandwidth = width;
    return LINEARS_OK;
}

static void F_LOCAL(seed)(F_SECTION *section, F_W input, F_W output)
{
    section->x1 = input;
    section->x2 = input;
    section->y1 = output;
    section->y2 = output;
}

/* Direct form I: every section keeps its actual input and output history, so
 * retuning replaces coefficients without reinterpreting state. Histories are
 * committed only after the final output has been produced. */
static linears_status_t F_LOCAL(cascade)(F_SECTION *sections, size_t count, F_S input,
                                         F_S *output)
{
    F_W x[2], y[2], signal;
    F_S result;
    size_t i;
#if FILTERS_FIXED
    linears_accum_t sum = {0, 0};
#else
    F_W term;
#endif
    F_TRY(F_LOCAL(load)(input, &signal));
    for (i = 0; i < count; ++i) {
        const F_SECTION *s = &sections[i];
        x[i] = signal;
#if FILTERS_FIXED
        /* One exact F62 sum per section; the Q output is rounded from the last
         * sum directly, and the signal between sections stays F31. */
        sum = (linears_accum_t){0, 0};
        F_TRY(linears_fixed_mac(&sum, s->b0, x[i]));
        F_TRY(linears_fixed_mac(&sum, s->b1, s->x1));
        F_TRY(linears_fixed_mac(&sum, s->b2, s->x2));
        F_TRY(linears_fixed_msu(&sum, s->a1, s->y1));
        F_TRY(linears_fixed_msu(&sum, s->a2, s->y2));
        F_TRY(linears_accum_to_fixed(sum, &y[i]));
#else
        F_TRY(F_MUL(s->b0, x[i], &y[i]));
        F_TRY(F_MUL(s->b1, s->x1, &term));
        F_TRY(F_ADD(y[i], term, &y[i]));
        F_TRY(F_MUL(s->b2, s->x2, &term));
        F_TRY(F_ADD(y[i], term, &y[i]));
        F_TRY(F_MUL(s->a1, s->y1, &term));
        F_TRY(F_SUB(y[i], term, &y[i]));
        F_TRY(F_MUL(s->a2, s->y2, &term));
        F_TRY(F_SUB(y[i], term, &y[i]));
#endif
        signal = y[i];
    }
#if FILTERS_FIXED
    F_TRY(FILTERS_ACCUM_EXPORT(sum, &result));
#else
    result = signal;
#endif
    for (i = 0; i < count; ++i) {
        sections[i].x2 = sections[i].x1;
        sections[i].x1 = x[i];
        sections[i].y2 = sections[i].y1;
        sections[i].y1 = y[i];
    }
    *output = result;
    return LINEARS_OK;
}

linears_status_t F_FN(lpf, init)(F_CTX(lpf) *ctx, filters_order_t order, F_HZ cutoff_hz,
                                F_HZ sample_rate_hz, F_S initial)
{
    F_CTX(lpf) result;
    F_W level;
    if (ctx == NULL) return LINEARS_ERROR_ARGUMENT;
    F_TRY(F_LOCAL(load)(initial, &level));
    F_TRY(F_LOCAL(lowpass_design)(order, cutoff_hz, sample_rate_hz, &result.section));
    result.order = order;
    F_LOCAL(seed)(&result.section, level, level);
    *ctx = result;
    return LINEARS_OK;
}

linears_status_t F_FN(lpf, set)(F_CTX(lpf) *ctx, F_HZ cutoff_hz, F_HZ sample_rate_hz)
{
    F_SECTION section;
    if (ctx == NULL) return LINEARS_ERROR_ARGUMENT;
    section = ctx->section;
    F_TRY(F_LOCAL(lowpass_design)(ctx->order, cutoff_hz, sample_rate_hz, &section));
    ctx->section = section;
    return LINEARS_OK;
}

linears_status_t F_FN(lpf, reset)(F_CTX(lpf) *ctx, F_S initial)
{
    F_W level;
    if (ctx == NULL) return LINEARS_ERROR_ARGUMENT;
    F_TRY(F_LOCAL(load)(initial, &level));
    F_LOCAL(seed)(&ctx->section, level, level);
    return LINEARS_OK;
}

linears_status_t F_FN(lpf, step)(F_CTX(lpf) *ctx, F_S input, F_S *output)
{
    F_TRY(F_LOCAL(output)(ctx, sizeof(*ctx), output));
    return F_LOCAL(cascade)(&ctx->section, 1, input, output);
}

linears_status_t F_FN(hpf, init)(F_CTX(hpf) *ctx, filters_order_t order, F_HZ cutoff_hz,
                                F_HZ sample_rate_hz, F_S initial)
{
    F_CTX(hpf) result;
    F_W level;
    if (ctx == NULL) return LINEARS_ERROR_ARGUMENT;
    F_TRY(F_LOCAL(load)(initial, &level));
    F_TRY(F_LOCAL(highpass_design)(order, cutoff_hz, sample_rate_hz, &result.section));
    result.order = order;
    F_LOCAL(seed)(&result.section, level, F_LOCAL(zero)());
    *ctx = result;
    return LINEARS_OK;
}

linears_status_t F_FN(hpf, set)(F_CTX(hpf) *ctx, F_HZ cutoff_hz, F_HZ sample_rate_hz)
{
    F_SECTION section;
    if (ctx == NULL) return LINEARS_ERROR_ARGUMENT;
    section = ctx->section;
    F_TRY(F_LOCAL(highpass_design)(ctx->order, cutoff_hz, sample_rate_hz, &section));
    ctx->section = section;
    return LINEARS_OK;
}

linears_status_t F_FN(hpf, reset)(F_CTX(hpf) *ctx, F_S initial)
{
    F_W level;
    if (ctx == NULL) return LINEARS_ERROR_ARGUMENT;
    F_TRY(F_LOCAL(load)(initial, &level));
    F_LOCAL(seed)(&ctx->section, level, F_LOCAL(zero)());
    return LINEARS_OK;
}

linears_status_t F_FN(hpf, step)(F_CTX(hpf) *ctx, F_S input, F_S *output)
{
    F_TRY(F_LOCAL(output)(ctx, sizeof(*ctx), output));
    return F_LOCAL(cascade)(&ctx->section, 1, input, output);
}

static void F_LOCAL(bpf_seed)(F_CTX(bpf) *ctx, F_W level)
{
    F_LOCAL(seed)(&ctx->sections[0], level, F_LOCAL(zero)());
    F_LOCAL(seed)(&ctx->sections[1], F_LOCAL(zero)(), F_LOCAL(zero)());
}

static linears_status_t F_LOCAL(bpf_init)(F_CTX(bpf) *ctx, filters_order_t order, F_D center,
                                          F_D bandwidth, F_S initial)
{
    F_CTX(bpf) result;
    F_W level;
    F_TRY(F_LOCAL(load)(initial, &level));
    F_TRY(F_LOCAL(bandpass_design)(order, center, bandwidth, result.sections));
    result.order = order;
    F_LOCAL(bpf_seed)(&result, level);
    *ctx = result;
    return LINEARS_OK;
}

static linears_status_t F_LOCAL(bpf_set)(F_CTX(bpf) *ctx, F_D center, F_D bandwidth)
{
    F_SECTION sections[2];
    sections[0] = ctx->sections[0];
    sections[1] = ctx->sections[1];
    F_TRY(F_LOCAL(bandpass_design)(ctx->order, center, bandwidth, sections));
    ctx->sections[0] = sections[0];
    ctx->sections[1] = sections[1];
    return LINEARS_OK;
}

linears_status_t F_FN(bpf, init_band)(F_CTX(bpf) *ctx, filters_order_t order, F_HZ low_hz,
                                     F_HZ high_hz, F_HZ sample_rate_hz, F_S initial)
{
    F_D center, bandwidth;
    if (ctx == NULL) return LINEARS_ERROR_ARGUMENT;
    F_TRY(F_LOCAL(band_edges)(low_hz, high_hz, sample_rate_hz, &center, &bandwidth));
    return F_LOCAL(bpf_init)(ctx, order, center, bandwidth, initial);
}

linears_status_t F_FN(bpf, init_center)(F_CTX(bpf) *ctx, filters_order_t order, F_HZ center_hz,
                                       F_HZ q, F_HZ sample_rate_hz, F_S initial)
{
    F_D center, bandwidth;
    if (ctx == NULL) return LINEARS_ERROR_ARGUMENT;
    F_TRY(F_LOCAL(center_q)(center_hz, q, sample_rate_hz, &center, &bandwidth));
    return F_LOCAL(bpf_init)(ctx, order, center, bandwidth, initial);
}

linears_status_t F_FN(bpf, set_band)(F_CTX(bpf) *ctx, F_HZ low_hz, F_HZ high_hz,
                                    F_HZ sample_rate_hz)
{
    F_D center, bandwidth;
    if (ctx == NULL) return LINEARS_ERROR_ARGUMENT;
    F_TRY(F_LOCAL(band_edges)(low_hz, high_hz, sample_rate_hz, &center, &bandwidth));
    return F_LOCAL(bpf_set)(ctx, center, bandwidth);
}

linears_status_t F_FN(bpf, set_center)(F_CTX(bpf) *ctx, F_HZ center_hz, F_HZ q,
                                      F_HZ sample_rate_hz)
{
    F_D center, bandwidth;
    if (ctx == NULL) return LINEARS_ERROR_ARGUMENT;
    F_TRY(F_LOCAL(center_q)(center_hz, q, sample_rate_hz, &center, &bandwidth));
    return F_LOCAL(bpf_set)(ctx, center, bandwidth);
}

linears_status_t F_FN(bpf, reset)(F_CTX(bpf) *ctx, F_S initial)
{
    F_W level;
    if (ctx == NULL) return LINEARS_ERROR_ARGUMENT;
    F_TRY(F_LOCAL(load)(initial, &level));
    F_LOCAL(bpf_seed)(ctx, level);
    return LINEARS_OK;
}

linears_status_t F_FN(bpf, step)(F_CTX(bpf) *ctx, F_S input, F_S *output)
{
    F_TRY(F_LOCAL(output)(ctx, sizeof(*ctx), output));
    return F_LOCAL(cascade)(ctx->sections, ctx->order == FILTERS_ORDER_2 ? 2 : 1, input, output);
}

/* Hamming-windowed ideal low-pass tap; the window is evaluated from the
 * signed offset to the center tap so mirrored taps match. */
static F_D F_LOCAL(windowed_sinc)(F_D ratio, size_t index, size_t half)
{
    F_D offset, window;
    if (index == half) return 2 * ratio;
    offset = index < half ? -(F_D)(half - index) : (F_D)(index - half);
    window = (F_D)0.54 + (F_D)0.46 * F_MATH(cos)(F_PI * offset / (F_D)half);
    return window * F_MATH(sin)(2 * F_PI * ratio * offset) / (F_PI * offset);
}

/* DC gain of the unnormalized windowed sinc. */
static F_D F_LOCAL(fir_gain)(F_D ratio, size_t taps)
{
    F_D sum = 0;
    size_t i;
    for (i = 0; i < taps; ++i) sum += F_LOCAL(windowed_sinc)(ratio, i, taps / 2);
    return sum;
}

static linears_status_t F_LOCAL(fir_taps)(const F_C *coefficients, size_t taps)
{
    if (coefficients == NULL || taps == 0) return LINEARS_ERROR_ARGUMENT;
    /* An odd length has a center tap and an integer group delay. */
    if (taps % 2 == 0) return LINEARS_ERROR_DOMAIN;
    return LINEARS_OK;
}

/* Writes upper_scale*h(upper) - lower_scale*h(lower) for every non-center tap,
 * then sets the center tap so the stored taps sum to dc. */
static void F_LOCAL(fir_write)(F_C *coefficients, size_t taps, F_D upper, F_D upper_scale,
                               F_D lower, F_D lower_scale, F_C dc)
{
    const size_t half = taps / 2;
    F_C others = 0;
    size_t i;
    for (i = 0; i < taps; ++i) {
        F_D value;
        if (i == half) continue;
        value = upper_scale * F_LOCAL(windowed_sinc)(upper, i, half);
        if (lower_scale != 0) value -= lower_scale * F_LOCAL(windowed_sinc)(lower, i, half);
        coefficients[i] = F_LOCAL(to_coefficient)(value);
        others += coefficients[i];
    }
    coefficients[half] = dc - others;
}

linears_status_t F_FN(fir, design_lpf)(F_C *coefficients, size_t taps, F_HZ cutoff_hz,
                                      F_HZ sample_rate_hz)
{
    F_D ratio;
    F_TRY(F_LOCAL(fir_taps)(coefficients, taps));
    F_TRY(F_LOCAL(ratio)(cutoff_hz, sample_rate_hz, &ratio));
    F_LOCAL(fir_write)(coefficients, taps, ratio, 1 / F_LOCAL(fir_gain)(ratio, taps), ratio, 0,
                       F_C_ONE);
    return LINEARS_OK;
}

linears_status_t F_FN(fir, design_hpf)(F_C *coefficients, size_t taps, F_HZ cutoff_hz,
                                      F_HZ sample_rate_hz)
{
    F_D ratio;
    F_TRY(F_LOCAL(fir_taps)(coefficients, taps));
    F_TRY(F_LOCAL(ratio)(cutoff_hz, sample_rate_hz, &ratio));
    F_LOCAL(fir_write)(coefficients, taps, ratio, -1 / F_LOCAL(fir_gain)(ratio, taps), ratio, 0,
                       0);
    return LINEARS_OK;
}

linears_status_t F_FN(fir, design_bpf)(F_C *coefficients, size_t taps, F_HZ low_hz,
                                      F_HZ high_hz, F_HZ sample_rate_hz)
{
    F_D lower, upper;
    F_TRY(F_LOCAL(fir_taps)(coefficients, taps));
    F_TRY(F_LOCAL(ratio)(low_hz, sample_rate_hz, &lower));
    F_TRY(F_LOCAL(ratio)(high_hz, sample_rate_hz, &upper));
    if (!(upper > lower)) return LINEARS_ERROR_DOMAIN;
    F_LOCAL(fir_write)(coefficients, taps, upper, 1 / F_LOCAL(fir_gain)(upper, taps), lower,
                       1 / F_LOCAL(fir_gain)(lower, taps), 0);
    return LINEARS_OK;
}
