#ifndef FILTERS_Q_INTEGER_BITS_H
#define FILTERS_Q_INTEGER_BITS_H

#include <limits.h>
#include <stdint.h>

/**
 * @brief Count the significant bits of an unsigned fixed-point magnitude.
 *
 * @param[in] value Any unsigned 64-bit value.
 * @return unsigned Zero for zero, otherwise floor(log2(value)) + 1 (1 to 64).
 */
static inline unsigned filters_q_bit_length(uint64_t value)
{
#if defined(__GNUC__) && UINT_MAX == UINT32_MAX
    uint32_t high = (uint32_t)(value >> 32), low = (uint32_t)value;
    if (high != 0) return 64u - (unsigned)__builtin_clz(high);
    if (low != 0) return 32u - (unsigned)__builtin_clz(low);
    return 0;
#else
    unsigned bits = 0;
    while (value != 0) {
        ++bits;
        value >>= 1;
    }
    return bits;
#endif
}

#endif
