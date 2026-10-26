/* Native single-precision instantiation of the shared filter recurrences. */
#include "filters.h"
#define FILTERS_SUFFIX f32
#define FILTERS_SCALAR float
#define FILTERS_WORK float
#define FILTERS_FIXED 0
#define FILTERS_HZ float
#define FILTERS_REAL float
#define FILTERS_REAL_FN(name) name##f
#include "filters_scalar_impl.h"
#include "filters_impl.h"
#include "filters_design_impl.h"
