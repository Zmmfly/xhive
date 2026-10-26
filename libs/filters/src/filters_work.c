/* Shared fixed-binary-point recurrence for Q filters and observer work state. */
#include "filters_ema.h"

linears_status_t filters_ema_work_step(linears_fixed_t *state,
                                       linears_q31_t alpha,
                                       linears_fixed_t input)
{
    linears_accum_t sum = {0, 0};
    linears_fixed_t result;
    linears_status_t status;
    if (state == NULL) return LINEARS_ERROR_ARGUMENT;
    if (alpha < 0) return LINEARS_ERROR_DOMAIN;
    status = linears_fixed_mac(&sum, LINEARS_FIXED_ONE - alpha, *state);
    if (status != LINEARS_OK) return status;
    status = linears_fixed_mac(&sum, alpha, input);
    if (status != LINEARS_OK) return status;
    status = linears_accum_to_fixed(sum, &result);
    if (status != LINEARS_OK) return status;
    *state = result;
    return LINEARS_OK;
}
