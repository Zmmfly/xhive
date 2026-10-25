/*
 * Shared adapter body of the scalar PID variants.
 *
 * This is not a public header: each src/pid_<suffix>.c includes it exactly once
 * with EZC_PID_TYPE, EZC_PID_SUFFIX and EZC_PID_MAX defined. The suffix selects
 * both the ezctrl API (ezc_pid_<suffix>_*) and the fieldctrl PID that computes
 * it (fieldctrl_<suffix>_pid_*); this file adds no PID arithmetic of its own.
 */

#if !defined(EZC_PID_TYPE) || !defined(EZC_PID_SUFFIX) || !defined(EZC_PID_MAX)
#error "pid_impl.h requires EZC_PID_TYPE, EZC_PID_SUFFIX and EZC_PID_MAX"
#endif

#include <stddef.h>

#define EZC_PID_CAT_(a, b) a##b
#define EZC_PID_CAT(a, b) EZC_PID_CAT_(a, b)

/* ezc_pid_f32, ezc_pid_f64, ... */
#define EZC_PID_NAME EZC_PID_CAT(ezc_pid_, EZC_PID_SUFFIX)
/* ezc_pid_f32_t, ezc_pid_f32_cfg_t, ... */
#define EZC_PID_CONTEXT EZC_PID_CAT(EZC_PID_NAME, _t)
#define EZC_PID_CONFIG EZC_PID_CAT(EZC_PID_NAME, _cfg_t)
#define EZC_PID_RESULT EZC_PID_CAT(EZC_PID_NAME, _result_t)
#define EZC_PID_CALLBACK EZC_PID_CAT(EZC_PID_NAME, _cb_t)
/* ezc_pid_f32_init, ezc_pid_f32_update, ... */
#define EZC_PID_FN(name) EZC_PID_CAT(EZC_PID_NAME, name)

/* fieldctrl_f32_pid, fieldctrl_f64_pid, ... */
#define EZC_FC_NAME EZC_PID_CAT(EZC_PID_CAT(fieldctrl_, EZC_PID_SUFFIX), _pid)
#define EZC_FC_CONFIG EZC_PID_CAT(EZC_FC_NAME, _config_t)
#define EZC_FC_OUTPUT EZC_PID_CAT(EZC_FC_NAME, _output_t)
#define EZC_FC_FN(name) EZC_PID_CAT(EZC_FC_NAME, name)

/* The documented zero state: every member zero, NULL or false. */
static const EZC_PID_CONTEXT EZC_PID_FN(_zero_context) = {0};

/*
 * Whether the fieldctrl controller has been built. A successful fieldctrl init
 * always stores lower < upper, which the all-zero controller of a
 * zero-initialized context (lower == upper == 0) does not satisfy.
 */
static bool EZC_PID_FN(_built)(const EZC_PID_CONTEXT *ctx)
{
    return ctx->pid.config.lower < ctx->pid.config.upper;
}

/*
 * Clamp value into the integral bounds of a fieldctrl tuning: the output range,
 * or the independent integral limit while that option is set. This mirrors the
 * rule fieldctrl applies, because fieldctrl init and reset reject an integral
 * outside those bounds.
 */
static EZC_PID_TYPE EZC_PID_FN(_clamp_integral)(const EZC_FC_CONFIG *fc, EZC_PID_TYPE value)
{
    const bool own_limit = (fc->options & EZC_PID_OPT_INTEGRAL_LIMIT) != 0u;
    const EZC_PID_TYPE lo = own_limit ? fc->integral_lower : fc->lower;
    const EZC_PID_TYPE hi = own_limit ? fc->integral_upper : fc->upper;

    if (value < lo) {
        return lo;
    }
    if (value > hi) {
        return hi;
    }
    return value;
}

/*
 * Map an ezctrl configuration onto the fieldctrl tuning it describes. The
 * callback is not part of the tuning.
 */
static void EZC_PID_FN(_map)(const EZC_PID_CONFIG *cfg, EZC_FC_CONFIG *fc)
{
    fc->kp = cfg->kp;
    fc->ki = cfg->ki;
    fc->kd = cfg->kd;
    /* fieldctrl always clamps; a disabled limit is the whole finite range. */
    fc->lower = cfg->limit_enabled ? cfg->out_min : -EZC_PID_MAX;
    fc->upper = cfg->limit_enabled ? cfg->out_max : EZC_PID_MAX;
    fc->options = cfg->options;
    fc->derivative = (fieldctrl_pid_derivative_t)cfg->deriv_source;
    fc->derivative_filter = cfg->deriv_filter_t;
    fc->setpoint_weight = cfg->weight_p;
    fc->integral_lower = cfg->integral_min;
    fc->integral_upper = cfg->integral_max;
    fc->separation = cfg->integral_separation;
}

/*
 * Make next the configuration in effect. A built controller is retuned in
 * place, which keeps its integral (clamped into the new bounds), derivative
 * history and, for unchanged period coefficients, its dt cache. An unbuilt
 * controller is initialized with zero clamped into the integral bounds.
 * fieldctrl validates every field and leaves ctx->pid unchanged on failure,
 * and cfg is then not committed either.
 */
static ezc_status_t EZC_PID_FN(_apply)(EZC_PID_CONTEXT *ctx, const EZC_PID_CONFIG *next)
{
    EZC_FC_CONFIG fc;
    linears_status_t status;

    EZC_PID_FN(_map)(next, &fc);
    if (EZC_PID_FN(_built)(ctx)) {
        status = EZC_FC_FN(_retune)(&ctx->pid, &fc);
    } else {
        status = EZC_FC_FN(_init)(&ctx->pid, &fc,
                                  EZC_PID_FN(_clamp_integral)(&fc, (EZC_PID_TYPE)0));
    }
    if (status != LINEARS_OK) {
        return EZC_ERR_INVALID_ARG;
    }
    ctx->cfg = *next;
    return EZC_OK;
}

ezc_status_t EZC_PID_FN(_init)(EZC_PID_CONTEXT *ctx, const EZC_PID_CONFIG *cfg)
{
    if (ctx == NULL) {
        return EZC_ERR_INVALID_ARG;
    }
    /* Zero first: a rejected cfg then leaves the documented disabled context. */
    *ctx = EZC_PID_FN(_zero_context);
    if (cfg == NULL) {
        return EZC_ERR_INVALID_ARG;
    }
    return EZC_PID_FN(_apply)(ctx, cfg);
}

ezc_status_t EZC_PID_FN(_set_gains)(EZC_PID_CONTEXT *ctx, EZC_PID_TYPE kp,
                                    EZC_PID_TYPE ki, EZC_PID_TYPE kd)
{
    EZC_PID_CONFIG next;

    if (ctx == NULL) {
        return EZC_ERR_INVALID_ARG;
    }
    next = ctx->cfg;
    next.kp = kp;
    next.ki = ki;
    next.kd = kd;
    return EZC_PID_FN(_apply)(ctx, &next);
}

ezc_status_t EZC_PID_FN(_set_limit)(EZC_PID_CONTEXT *ctx, EZC_PID_TYPE out_min,
                                    EZC_PID_TYPE out_max)
{
    EZC_PID_CONFIG next;

    if (ctx == NULL) {
        return EZC_ERR_INVALID_ARG;
    }
    next = ctx->cfg;
    next.limit_enabled = true;
    next.out_min = out_min;
    next.out_max = out_max;
    return EZC_PID_FN(_apply)(ctx, &next);
}

ezc_status_t EZC_PID_FN(_clear_limit)(EZC_PID_CONTEXT *ctx)
{
    EZC_PID_CONFIG next;

    if (ctx == NULL) {
        return EZC_ERR_INVALID_ARG;
    }
    next = ctx->cfg;
    next.limit_enabled = false;
    return EZC_PID_FN(_apply)(ctx, &next);
}

ezc_status_t EZC_PID_FN(_set_callback)(EZC_PID_CONTEXT *ctx, EZC_PID_CALLBACK callback,
                                       void *user_ctx)
{
    if (ctx == NULL) {
        return EZC_ERR_INVALID_ARG;
    }
    /* The callback is not part of the fieldctrl tuning: nothing is rebuilt. */
    ctx->cfg.callback = callback;
    ctx->cfg.callback_ctx = user_ctx;
    return EZC_OK;
}

ezc_status_t EZC_PID_FN(_set_derivative_source)(EZC_PID_CONTEXT *ctx,
                                                ezc_pid_deriv_source_t source)
{
    EZC_PID_CONFIG next;

    if (ctx == NULL) {
        return EZC_ERR_INVALID_ARG;
    }
    next = ctx->cfg;
    next.deriv_source = source;
    return EZC_PID_FN(_apply)(ctx, &next);
}

ezc_status_t EZC_PID_FN(_set_derivative_filter)(EZC_PID_CONTEXT *ctx, EZC_PID_TYPE deriv_filter_t)
{
    EZC_PID_CONFIG next;

    if (ctx == NULL) {
        return EZC_ERR_INVALID_ARG;
    }
    next = ctx->cfg;
    next.deriv_filter_t = deriv_filter_t;
    return EZC_PID_FN(_apply)(ctx, &next);
}

ezc_status_t EZC_PID_FN(_set_integral_limit)(EZC_PID_CONTEXT *ctx, EZC_PID_TYPE integral_min,
                                             EZC_PID_TYPE integral_max)
{
    EZC_PID_CONFIG next;

    if (ctx == NULL) {
        return EZC_ERR_INVALID_ARG;
    }
    next = ctx->cfg;
    next.integral_min = integral_min;
    next.integral_max = integral_max;
    next.options |= (uint32_t)EZC_PID_OPT_INTEGRAL_LIMIT;
    return EZC_PID_FN(_apply)(ctx, &next);
}

ezc_status_t EZC_PID_FN(_clear_integral_limit)(EZC_PID_CONTEXT *ctx)
{
    EZC_PID_CONFIG next;

    if (ctx == NULL) {
        return EZC_ERR_INVALID_ARG;
    }
    next = ctx->cfg;
    next.options &= ~((uint32_t)EZC_PID_OPT_INTEGRAL_LIMIT);
    return EZC_PID_FN(_apply)(ctx, &next);
}

ezc_status_t EZC_PID_FN(_set_integral_separation)(EZC_PID_CONTEXT *ctx, EZC_PID_TYPE threshold)
{
    EZC_PID_CONFIG next;

    if (ctx == NULL) {
        return EZC_ERR_INVALID_ARG;
    }
    next = ctx->cfg;
    next.integral_separation = threshold;
    next.options |= (uint32_t)EZC_PID_OPT_INTEGRAL_SEPARATION;
    return EZC_PID_FN(_apply)(ctx, &next);
}

ezc_status_t EZC_PID_FN(_clear_integral_separation)(EZC_PID_CONTEXT *ctx)
{
    EZC_PID_CONFIG next;

    if (ctx == NULL) {
        return EZC_ERR_INVALID_ARG;
    }
    next = ctx->cfg;
    next.options &= ~((uint32_t)EZC_PID_OPT_INTEGRAL_SEPARATION);
    return EZC_PID_FN(_apply)(ctx, &next);
}

ezc_status_t EZC_PID_FN(_set_setpoint_weight)(EZC_PID_CONTEXT *ctx, EZC_PID_TYPE weight)
{
    EZC_PID_CONFIG next;

    if (ctx == NULL) {
        return EZC_ERR_INVALID_ARG;
    }
    next = ctx->cfg;
    next.weight_p = weight;
    next.options |= (uint32_t)EZC_PID_OPT_SETPOINT_WEIGHT;
    return EZC_PID_FN(_apply)(ctx, &next);
}

void EZC_PID_FN(_reset)(EZC_PID_CONTEXT *ctx)
{
    if ((ctx == NULL) || !EZC_PID_FN(_built)(ctx)) {
        return;
    }
    /*
     * Zero clamped into the integral bounds is a finite value inside them,
     * which fieldctrl's reset accepts for every built controller.
     */
    (void)EZC_FC_FN(_reset)(&ctx->pid,
                            EZC_PID_FN(_clamp_integral)(&ctx->pid.config, (EZC_PID_TYPE)0));
}

ezc_status_t EZC_PID_FN(_reset_to)(EZC_PID_CONTEXT *ctx, EZC_PID_TYPE integral)
{
    linears_status_t status;

    if (ctx == NULL) {
        return EZC_ERR_INVALID_ARG;
    }
    if (EZC_PID_FN(_built)(ctx)) {
        status = EZC_FC_FN(_reset)(&ctx->pid, integral);
    } else {
        /* Building the controller with this integral is atomic as well. */
        EZC_FC_CONFIG fc;

        EZC_PID_FN(_map)(&ctx->cfg, &fc);
        status = EZC_FC_FN(_init)(&ctx->pid, &fc, integral);
    }
    return (status == LINEARS_OK) ? EZC_OK : EZC_ERR_INVALID_ARG;
}

ezc_status_t EZC_PID_FN(_try_update)(EZC_PID_CONTEXT *ctx, EZC_PID_TYPE setpoint,
                                     EZC_PID_TYPE measure, EZC_PID_TYPE dt,
                                     EZC_PID_TYPE *output)
{
    EZC_FC_OUTPUT out;

    if ((ctx == NULL) || (output == NULL)) {
        return EZC_ERR_INVALID_ARG;
    }
    /*
     * A zero-initialized context builds its disabled controller on first use.
     * Its configuration is the accepted zero one unless cfg was written
     * directly, which the failure branch then treats as a rejected step.
     */
    if (!EZC_PID_FN(_built)(ctx) && (EZC_PID_FN(_apply)(ctx, &ctx->cfg) != EZC_OK)) {
        return EZC_ERR_INVALID_ARG;
    }
    /* fieldctrl rejects the step atomically: no state changed, out unwritten. */
    if (EZC_FC_FN(_step)(&ctx->pid, setpoint, measure, dt, &out) != LINEARS_OK) {
        return EZC_ERR_INVALID_ARG;
    }
    ctx->output = out.output;
    *output = out.output;

    if (ctx->cfg.callback != NULL) {
        EZC_PID_RESULT result;

        result.output = out.output;
        result.setpoint = setpoint;
        result.measure = measure;
        result.error = setpoint - measure;
        result.p_term = out.proportional;
        result.i_term = out.integral;
        result.d_term = out.derivative;
        result.limited = out.saturated != 0;
        ctx->cfg.callback(ctx->cfg.callback_ctx, &result);
    }
    return EZC_OK;
}

EZC_PID_TYPE EZC_PID_FN(_update)(EZC_PID_CONTEXT *ctx, EZC_PID_TYPE setpoint,
                                 EZC_PID_TYPE measure, EZC_PID_TYPE dt)
{
    EZC_PID_TYPE output;

    if (ctx == NULL) {
        return (EZC_PID_TYPE)0;
    }
    if (EZC_PID_FN(_try_update)(ctx, setpoint, measure, dt, &output) != EZC_OK) {
        return ctx->output;
    }
    return output;
}

#undef EZC_FC_FN
#undef EZC_FC_OUTPUT
#undef EZC_FC_CONFIG
#undef EZC_FC_NAME
#undef EZC_PID_FN
#undef EZC_PID_CALLBACK
#undef EZC_PID_RESULT
#undef EZC_PID_CONFIG
#undef EZC_PID_CONTEXT
#undef EZC_PID_NAME
#undef EZC_PID_CAT
#undef EZC_PID_CAT_
