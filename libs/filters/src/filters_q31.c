/* Q31 storage with the shared linears pure-integer Q31 working arithmetic. */
#include "filters.h"
#define FILTERS_SUFFIX q31
#define FILTERS_SCALAR linears_q31_t
#define FILTERS_WORK linears_fixed_t
#define FILTERS_FIXED 1
#define FILTERS_LOAD linears_fixed_from_q31
#define FILTERS_EXPORT linears_fixed_export_q31
#include "filters_scalar_impl.h"
#include "filters_impl.h"
