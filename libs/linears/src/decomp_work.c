/* Wide fixed-F31 intermediates shared by Kalman and the raw-Q decompositions. */
#include "decomp_internal.h"

#define D_FORMAT 5
#include "decomp_template.h"
#include "decomp_lu.inc"
#include "decomp_cholesky.inc"
#include "decomp_cleanup.h"
