/* Native double-precision instantiation of the shared filter recurrences. */
#include "filters.h"
#define FILTERS_SUFFIX f64
#define FILTERS_SCALAR double
#define FILTERS_WORK double
#define FILTERS_FIXED 0
#include "filters_scalar_impl.h"
#include "filters_impl.h"
