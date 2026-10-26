/* Native double-precision instantiation of the shared filter recurrences. */
#include "filters.h"
#define FILTERS_SUFFIX f64
#define FILTERS_SCALAR double
#define FILTERS_WORK double
#define FILTERS_FIXED 0
#define FILTERS_HZ double
#define FILTERS_REAL double
#define FILTERS_REAL_FN(name) name
#include "filters_scalar_impl.h"
#include "filters_impl.h"
#include "filters_design_impl.h"
