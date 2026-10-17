#include "fieldctrl_dtc.h"

/* The six active vectors V1..V6 have upper-switch patterns 100, 110, 010,
 * 011, 001, 101. Sectors are centered on those vectors, not on their edges.
 * Axes: [flux decrease/increase][torque decrease/increase][sector 1..6].
 * This is the PMSM BST in Nasr et al., Energies 13 (2020) 1907, Table 1. */
static const uint8_t dtc_active_table[2][2][6] = {
    {{1, 5, 4, 6, 2, 3}, {2, 3, 1, 5, 4, 6}},
    {{5, 4, 6, 2, 3, 1}, {6, 2, 3, 1, 5, 4}}
};

/* V0 and V7 have the same ideal alpha/beta voltage but different switch
 * transitions. Retain either zero state; otherwise select its nearer one. */
static uint8_t dtc_zero_state(uint8_t previous) {
    unsigned ones = (previous & 1u) + ((previous >> 1u) & 1u) +
                    ((previous >> 2u) & 1u);
    return (uint8_t)(ones >= 2u ? 7u : 0u);
}

#define FC_VARIANT 32
#include "fieldctrl_variant.h"
#include "dtc_impl.h"
#include "fieldctrl_variant_end.h"

#define FC_VARIANT 64
#include "fieldctrl_variant.h"
#include "dtc_impl.h"
#include "fieldctrl_variant_end.h"

#define FC_VARIANT 15
#include "fieldctrl_variant.h"
#include "dtc_impl.h"
#include "fieldctrl_variant_end.h"

#define FC_VARIANT 31
#include "fieldctrl_variant.h"
#include "dtc_impl.h"
#include "fieldctrl_variant_end.h"
