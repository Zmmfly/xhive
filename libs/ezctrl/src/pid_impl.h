/*
 * Shared implementation body of the scalar PID variants.
 *
 * This is not a public header: each src/pid_<suffix>.c includes it exactly once
 * with EZC_PID_TYPE and EZC_PID_SUFFIX defined, and it expands to one complete
 * typed API. A new scalar variant therefore needs a one-line instantiation file
 * instead of a copy of the algorithm.
 */

#if !defined(EZC_PID_TYPE) || !defined(EZC_PID_SUFFIX)
#error "pid_impl.h requires EZC_PID_TYPE and EZC_PID_SUFFIX"
#endif

#include <stddef.h>

#define EZC_PID_CAT_(a, b) a##b
#define EZC_PID_CAT(a, b) EZC_PID_CAT_(a, b)

#define EZC_PID_ABS(v) (((v) < (EZC_PID_TYPE)0) ? -(v) : (v))

/* ezc_pid_f32, ezc_pid_f64, ... */
#define EZC_PID_NAME EZC_PID_CAT(ezc_pid_, EZC_PID_SUFFIX)
/* ezc_pid_f32_t, ezc_pid_f32_cfg_t, ... */
#define EZC_PID_CONTEXT EZC_PID_CAT(EZC_PID_NAME, _t)
#define EZC_PID_CONFIG EZC_PID_CAT(EZC_PID_NAME, _cfg_t)
#define EZC_PID_RESULT EZC_PID_CAT(EZC_PID_NAME, _result_t)
#define EZC_PID_CALLBACK EZC_PID_CAT(EZC_PID_NAME, _cb_t)
/* ezc_pid_f32_init, ezc_pid_f32_update, ... */
#define EZC_PID_FN(name) EZC_PID_CAT(EZC_PID_NAME, name)

/*
 * Return a context to the documented zero state: all gains zero, no limit, no
 * anti-windup, no callback and no controller state.
 */
static void EZC_PID_FN(_zero)(EZC_PID_CONTEXT *ctx)
{
    ctx->kp = (EZC_PID_TYPE)0;
    ctx->ki = (EZC_PID_TYPE)0;
    ctx->kd = (EZC_PID_TYPE)0;
    ctx->options = 0u;
    ctx->deriv_source = EZC_PID_DERIV_ON_ERROR;
    ctx->deriv_filter_t = (EZC_PID_TYPE)0;
    ctx->weight_p = (EZC_PID_TYPE)0;
    ctx->integral_min = (EZC_PID_TYPE)0;
    ctx->integral_max = (EZC_PID_TYPE)0;
    ctx->integral_separation = (EZC_PID_TYPE)0;
    ctx->integral = (EZC_PID_TYPE)0;
    ctx->prev_error = (EZC_PID_TYPE)0;
    ctx->prev_measure = (EZC_PID_TYPE)0;
    ctx->d_filtered = (EZC_PID_TYPE)0;
    ctx->dt_cached = (EZC_PID_TYPE)0;
    ctx->dt_inv_cached = (EZC_PID_TYPE)0;
    ctx->out_min = (EZC_PID_TYPE)0;
    ctx->out_max = (EZC_PID_TYPE)0;
    ctx->limit_enabled = false;
    ctx->anti_windup_enabled = false;
    ctx->callback = NULL;
    ctx->callback_ctx = NULL;
}

/*
 * Validate a configuration before it is applied. Every comparison is written
 * so that a value which is not comparable (NaN) fails it, which keeps NaN out
 * of the controller configuration.
 */
static bool EZC_PID_FN(_config_valid)(const EZC_PID_CONFIG *cfg)
{
    if (cfg->limit_enabled && !(cfg->out_min <= cfg->out_max)) {
        return false;
    }
    if (((cfg->options & EZC_PID_OPT_INTEGRAL_LIMIT) != 0u) &&
        !(cfg->integral_min <= cfg->integral_max)) {
        return false;
    }
    if (((cfg->options & EZC_PID_OPT_INTEGRAL_SEPARATION) != 0u) &&
        !(cfg->integral_separation > (EZC_PID_TYPE)0)) {
        return false;
    }
    if (((cfg->options & EZC_PID_OPT_SETPOINT_WEIGHT) != 0u) &&
        !((cfg->weight_p >= (EZC_PID_TYPE)0) && (cfg->weight_p <= (EZC_PID_TYPE)1))) {
        return false;
    }
    if (!(cfg->deriv_filter_t >= (EZC_PID_TYPE)0)) {
        return false;
    }
    if ((cfg->deriv_source != EZC_PID_DERIV_ON_ERROR) &&
        (cfg->deriv_source != EZC_PID_DERIV_ON_MEASUREMENT)) {
        return false;
    }
    return true;
}

ezc_status_t EZC_PID_FN(_init)(EZC_PID_CONTEXT *ctx, const EZC_PID_CONFIG *cfg)
{
    if (ctx == NULL) {
        return EZC_ERR_INVALID_ARG;
    }

    /*
     * A missing configuration is rejected, and so is any enabled option whose
     * parameters are out of range: every check fails for values that are not
     * comparable (NaN) as well.
     */
    if ((cfg == NULL) || !EZC_PID_FN(_config_valid)(cfg)) {
        EZC_PID_FN(_zero)(ctx);
        return EZC_ERR_INVALID_ARG;
    }

    EZC_PID_FN(_zero)(ctx);
    ctx->kp = cfg->kp;
    ctx->ki = cfg->ki;
    ctx->kd = cfg->kd;
    ctx->options = cfg->options;
    ctx->deriv_source = cfg->deriv_source;
    ctx->deriv_filter_t = cfg->deriv_filter_t;
    ctx->weight_p = cfg->weight_p;
    ctx->integral_min = cfg->integral_min;
    ctx->integral_max = cfg->integral_max;
    ctx->integral_separation = cfg->integral_separation;
    ctx->out_min = cfg->out_min;
    ctx->out_max = cfg->out_max;
    ctx->limit_enabled = cfg->limit_enabled;
    ctx->anti_windup_enabled = cfg->anti_windup_enabled;
    ctx->callback = cfg->callback;
    ctx->callback_ctx = cfg->callback_ctx;
    return EZC_OK;
}

ezc_status_t EZC_PID_FN(_set_gains)(EZC_PID_CONTEXT *ctx, EZC_PID_TYPE kp,
                                    EZC_PID_TYPE ki, EZC_PID_TYPE kd)
{
    if (ctx == NULL) {
        return EZC_ERR_INVALID_ARG;
    }
    ctx->kp = kp;
    ctx->ki = ki;
    ctx->kd = kd;
    return EZC_OK;
}

ezc_status_t EZC_PID_FN(_set_limit)(EZC_PID_CONTEXT *ctx, EZC_PID_TYPE out_min,
                                    EZC_PID_TYPE out_max)
{
    if ((ctx == NULL) || !(out_min <= out_max)) {
        return EZC_ERR_INVALID_ARG;
    }
    ctx->out_min = out_min;
    ctx->out_max = out_max;
    ctx->limit_enabled = true;
    return EZC_OK;
}

ezc_status_t EZC_PID_FN(_clear_limit)(EZC_PID_CONTEXT *ctx)
{
    if (ctx == NULL) {
        return EZC_ERR_INVALID_ARG;
    }
    ctx->limit_enabled = false;
    return EZC_OK;
}

ezc_status_t EZC_PID_FN(_set_anti_windup)(EZC_PID_CONTEXT *ctx, bool enable)
{
    if (ctx == NULL) {
        return EZC_ERR_INVALID_ARG;
    }
    ctx->anti_windup_enabled = enable;
    return EZC_OK;
}

ezc_status_t EZC_PID_FN(_set_callback)(EZC_PID_CONTEXT *ctx, EZC_PID_CALLBACK callback,
                                       void *user_ctx)
{
    if (ctx == NULL) {
        return EZC_ERR_INVALID_ARG;
    }
    ctx->callback = callback;
    ctx->callback_ctx = user_ctx;
    return EZC_OK;
}

ezc_status_t EZC_PID_FN(_set_derivative_source)(EZC_PID_CONTEXT *ctx,
                                                ezc_pid_deriv_source_t source)
{
    if ((ctx == NULL) || ((source != EZC_PID_DERIV_ON_ERROR) &&
                          (source != EZC_PID_DERIV_ON_MEASUREMENT))) {
        return EZC_ERR_INVALID_ARG;
    }
    ctx->deriv_source = source;
    return EZC_OK;
}

ezc_status_t EZC_PID_FN(_set_derivative_filter)(EZC_PID_CONTEXT *ctx, EZC_PID_TYPE deriv_filter_t)
{
    if ((ctx == NULL) || !(deriv_filter_t >= (EZC_PID_TYPE)0)) {
        return EZC_ERR_INVALID_ARG;
    }
    ctx->deriv_filter_t = deriv_filter_t;
    return EZC_OK;
}

ezc_status_t EZC_PID_FN(_set_integral_limit)(EZC_PID_CONTEXT *ctx, EZC_PID_TYPE integral_min,
                                             EZC_PID_TYPE integral_max)
{
    if ((ctx == NULL) || !(integral_min <= integral_max)) {
        return EZC_ERR_INVALID_ARG;
    }
    ctx->integral_min = integral_min;
    ctx->integral_max = integral_max;
    ctx->options |= (uint32_t)EZC_PID_OPT_INTEGRAL_LIMIT;
    return EZC_OK;
}

ezc_status_t EZC_PID_FN(_clear_integral_limit)(EZC_PID_CONTEXT *ctx)
{
    if (ctx == NULL) {
        return EZC_ERR_INVALID_ARG;
    }
    ctx->options &= ~((uint32_t)EZC_PID_OPT_INTEGRAL_LIMIT);
    return EZC_OK;
}

ezc_status_t EZC_PID_FN(_set_integral_separation)(EZC_PID_CONTEXT *ctx, EZC_PID_TYPE threshold)
{
    if ((ctx == NULL) || !(threshold > (EZC_PID_TYPE)0)) {
        return EZC_ERR_INVALID_ARG;
    }
    ctx->integral_separation = threshold;
    ctx->options |= (uint32_t)EZC_PID_OPT_INTEGRAL_SEPARATION;
    return EZC_OK;
}

ezc_status_t EZC_PID_FN(_clear_integral_separation)(EZC_PID_CONTEXT *ctx)
{
    if (ctx == NULL) {
        return EZC_ERR_INVALID_ARG;
    }
    ctx->options &= ~((uint32_t)EZC_PID_OPT_INTEGRAL_SEPARATION);
    return EZC_OK;
}

ezc_status_t EZC_PID_FN(_set_setpoint_weight)(EZC_PID_CONTEXT *ctx, EZC_PID_TYPE weight)
{
    if ((ctx == NULL) ||
        !((weight >= (EZC_PID_TYPE)0) && (weight <= (EZC_PID_TYPE)1))) {
        return EZC_ERR_INVALID_ARG;
    }
    ctx->weight_p = weight;
    ctx->options |= (uint32_t)EZC_PID_OPT_SETPOINT_WEIGHT;
    return EZC_OK;
}

void EZC_PID_FN(_reset)(EZC_PID_CONTEXT *ctx)
{
    if (ctx == NULL) {
        return;
    }
    ctx->integral = (EZC_PID_TYPE)0;
    ctx->prev_error = (EZC_PID_TYPE)0;
    ctx->prev_measure = (EZC_PID_TYPE)0;
    ctx->d_filtered = (EZC_PID_TYPE)0;
}

/*
 * Return 1 / dt and keep it cached while the caller repeats the same dt.
 *
 * Zero is returned for every dt that cannot be inverted: zero, negative, NaN
 * and positive infinity all fail the comparisons below, and the caller treats
 * that zero as a zero-time step. The comparison is bit-exact on purpose,
 * because a fixed-rate loop passes the very same value every step.
 */
static EZC_PID_TYPE EZC_PID_FN(_inverse_dt)(EZC_PID_CONTEXT *ctx, EZC_PID_TYPE dt)
{
    EZC_PID_TYPE inv_dt;

    if (dt == ctx->dt_cached) {
        return ctx->dt_inv_cached;
    }
    if (!(dt > (EZC_PID_TYPE)0)) {
        return (EZC_PID_TYPE)0;
    }
    inv_dt = (EZC_PID_TYPE)1 / dt;
    if (!(inv_dt > (EZC_PID_TYPE)0)) {
        return (EZC_PID_TYPE)0;
    }
    ctx->dt_cached = dt;
    ctx->dt_inv_cached = inv_dt;
    return inv_dt;
}

EZC_PID_TYPE EZC_PID_FN(_update)(EZC_PID_CONTEXT *ctx, EZC_PID_TYPE setpoint,
                                 EZC_PID_TYPE measure, EZC_PID_TYPE dt)
{
    EZC_PID_RESULT result;
    EZC_PID_TYPE error;
    EZC_PID_TYPE p_term;
    EZC_PID_TYPE d_raw;
    EZC_PID_TYPE d_term;
    EZC_PID_TYPE pd_term;
    EZC_PID_TYPE i_term;
    EZC_PID_TYPE inv_dt;
    EZC_PID_TYPE raw;
    EZC_PID_TYPE output;

    if (ctx == NULL) {
        return (EZC_PID_TYPE)0;
    }

    error = setpoint - measure;

    if ((ctx->options & EZC_PID_OPT_SETPOINT_WEIGHT) != 0u) {
        p_term = ctx->kp * ((ctx->weight_p * setpoint) - measure);
    } else {
        p_term = ctx->kp * error;
    }

    i_term = ctx->integral;
    d_term = (EZC_PID_TYPE)0;

    /*
     * A finite positive dt advances the time-dependent terms. Anything else is
     * a zero-time step: the integral holds, the derivative rate is zero, the
     * derivative filter keeps its state, and prev_error/prev_measure are still
     * moved to this sample so that the next valid step sees the change of a
     * single sample interval.
     */
    inv_dt = EZC_PID_FN(_inverse_dt)(ctx, dt);
    if (inv_dt > (EZC_PID_TYPE)0) {
        /*
         * Integral separation first: above the threshold the integral holds,
         * which keeps a large setpoint step from winding it up. The integral
         * limit is the hard bound applied to the result of the integration.
         */
        const bool separated =
            ((ctx->options & EZC_PID_OPT_INTEGRAL_SEPARATION) != 0u) &&
            !(EZC_PID_ABS(error) <= ctx->integral_separation);

        if (!separated) {
            i_term = ctx->integral + (ctx->ki * error * dt);
            if ((ctx->options & EZC_PID_OPT_INTEGRAL_LIMIT) != 0u) {
                if (i_term < ctx->integral_min) {
                    i_term = ctx->integral_min;
                } else if (i_term > ctx->integral_max) {
                    i_term = ctx->integral_max;
                }
            }
        }

        /*
         * Derivative: the configured source, then the optional first-order
         * filter. With the filter off, its state follows the raw derivative so
         * that enabling the filter later starts at the current value.
         */
        if (ctx->deriv_source == EZC_PID_DERIV_ON_MEASUREMENT) {
            d_raw = ctx->kd * (ctx->prev_measure - measure) * inv_dt;
        } else {
            d_raw = ctx->kd * (error - ctx->prev_error) * inv_dt;
        }
        if (ctx->deriv_filter_t > (EZC_PID_TYPE)0) {
            const EZC_PID_TYPE a = dt / (ctx->deriv_filter_t + dt);

            ctx->d_filtered += a * (d_raw - ctx->d_filtered);
        } else {
            ctx->d_filtered = d_raw;
        }
        d_term = ctx->d_filtered;
    }

    pd_term = p_term + d_term;
    raw = pd_term + i_term;
    output = raw;

    if (ctx->limit_enabled) {
        result.limited = (raw < ctx->out_min) || (raw > ctx->out_max);

        if (result.limited && ctx->anti_windup_enabled) {
            /*
             * The integral may advance up to the saturation boundary, which is
             * the value of the integral term that places the unclamped output
             * exactly on a limit. The limiter only stops the integral from
             * moving further into saturation; it never pulls the integral back
             * towards its previous value and therefore never manufactures
             * integral state of its own.
             */
            const EZC_PID_TYPE i_min = ctx->out_min - pd_term;
            const EZC_PID_TYPE i_max = ctx->out_max - pd_term;

            if ((i_term > i_max) && (i_term > ctx->integral)) {
                i_term = (i_max > ctx->integral) ? i_max : ctx->integral;
            } else if ((i_term < i_min) && (i_term < ctx->integral)) {
                i_term = (i_min < ctx->integral) ? i_min : ctx->integral;
            }
            raw = pd_term + i_term;
        }

        output = raw;
        if (output < ctx->out_min) {
            output = ctx->out_min;
        } else if (output > ctx->out_max) {
            output = ctx->out_max;
        }
    } else {
        result.limited = false;
    }

    ctx->integral = i_term;
    ctx->prev_error = error;
    ctx->prev_measure = measure;

    result.output = output;
    result.setpoint = setpoint;
    result.measure = measure;
    result.error = error;
    result.p_term = p_term;
    result.i_term = i_term;
    result.d_term = d_term;

    if (ctx->callback != NULL) {
        ctx->callback(ctx->callback_ctx, &result);
    }
    return output;
}

#undef EZC_PID_ABS
#undef EZC_PID_FN
#undef EZC_PID_CALLBACK
#undef EZC_PID_RESULT
#undef EZC_PID_CONFIG
#undef EZC_PID_CONTEXT
#undef EZC_PID_NAME
#undef EZC_PID_CAT
#undef EZC_PID_CAT_
