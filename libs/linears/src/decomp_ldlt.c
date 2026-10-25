/**
 * @file decomp_ldlt.c
 * @brief Four scalar instantiations of symmetric-indefinite block LDLT.
 */
#include "decomp_internal.h"

#define D_FORMAT 1
#include "decomp_template.h"
#include "decomp_ldlt.inc"
#include "decomp_cleanup.h"

#define D_FORMAT 2
#include "decomp_template.h"
#include "decomp_ldlt.inc"
#include "decomp_cleanup.h"

#define D_FORMAT 3
#include "decomp_template.h"
#include "decomp_ldlt.inc"
#include "decomp_cleanup.h"

#define D_FORMAT 4
#include "decomp_template.h"
#include "decomp_ldlt.inc"
#include "decomp_cleanup.h"
