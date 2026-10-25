/* Raw Q1.15 signals with shared wide F31 state/coefficients and F62 MAC. */
#include "filters.h"
#define FILTERS_SUFFIX q15
#define FILTERS_SCALAR linears_q15_t
#define FILTERS_WORK linears_fixed_t
#define FILTERS_FIXED 1
#define FILTERS_LOAD linears_fixed_from_q15
#define FILTERS_EXPORT linears_fixed_to_q15
#define FILTERS_ACCUM_EXPORT linears_accum_to_q15
#define FILTERS_MEAN linears_accum_mean_q15
#include "filters_scalar_impl.h"
#include "filters_impl.h"
