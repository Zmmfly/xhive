/**
 * @file decomp_svd.c
 * @brief Four scalar instantiations of the bounded one-sided Jacobi template.
 */
#include "decomp_internal.h"

#define D_FORMAT 1
#include "decomp_template.h"
#include "decomp_svd_template.inc"
#include "decomp_cleanup.h"

#define D_FORMAT 2
#include "decomp_template.h"
#include "decomp_svd_template.inc"
#include "decomp_cleanup.h"

#define D_FORMAT 3
#include "decomp_template.h"
#include "decomp_svd_template.inc"
#include "decomp_cleanup.h"

#define D_FORMAT 4
#include "decomp_template.h"
#include "decomp_svd_template.inc"
#include "decomp_cleanup.h"
