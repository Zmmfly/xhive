/* Internal allocation-free range validation; not a public API. */
#ifndef FILTERS_INTERNAL_H
#define FILTERS_INTERNAL_H

#include <stddef.h>
#include <stdint.h>

/* Difference-based comparison avoids pointer-end overflow and relational
 * comparisons of unrelated C objects. All counts have already been checked. */
static int filters_overlap(const void *a, size_t asize, const void *b, size_t bsize)
{
    uintptr_t av = (uintptr_t)a;
    uintptr_t bv = (uintptr_t)b;
    if (asize == 0 || bsize == 0) return 0;
    return av <= bv ? bv - av < asize : av - bv < bsize;
}

#endif
