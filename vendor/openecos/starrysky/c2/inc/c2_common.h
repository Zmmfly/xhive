/* Common status and ordered 32-bit MMIO access for StarrySky C2 drivers. */
#ifndef STARRYSKY_C2_COMMON_H
#define STARRYSKY_C2_COMMON_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

typedef enum {
    C2_OK = 0,
    C2_ERROR_INVALID_ARGUMENT = -1,
    C2_ERROR_TIMEOUT = -2,
    C2_ERROR_BUSY = -3,
    C2_ERROR_NACK = -4,
    C2_ERROR_ARBITRATION = -5,
    C2_ERROR_UNSUPPORTED = -6,
    C2_ERROR_NOT_INITIALIZED = -7,
    C2_ERROR_IO = -8
} c2_status_t;

/* poll_limit is a finite count of polling attempts, NOT a duration in us.
 * Individual APIs document its scope and zero behavior. Drivers do not use
 * an OS, heap, or implicit global clock. Caller must serialize transactions
 * and protect contexts shared between threads/interrupts. No driver installs
 * an interrupt handler. GPIO shadow ownership must remain exclusive.
 *
 * Override these two macros consistently for host MMIO simulation. Hardware
 * accesses must remain full-width; never perform MMIO through BITS fields.
 */
#ifndef C2_MMIO_READ32
#define C2_MMIO_READ32(address) (*(volatile const uint32_t *)(address))
#endif
#ifndef C2_MMIO_WRITE32
#define C2_MMIO_WRITE32(address, value) (*(volatile uint32_t *)(address) = (uint32_t)(value))
#endif

static inline void c2_mmio_fence(void)
{
#if defined(__riscv)
    __asm__ volatile ("fence iorw, iorw" ::: "memory");
#elif defined(__GNUC__) || defined(__clang__)
    __asm__ volatile ("" ::: "memory");
#endif
}

static inline uint32_t c2_mmio_read32(volatile const uint32_t *address)
{
    uint32_t value;
    c2_mmio_fence();
    value = C2_MMIO_READ32(address);
    c2_mmio_fence();
    return value;
}

static inline void c2_mmio_write32(volatile uint32_t *address, uint32_t value)
{
    c2_mmio_fence();
    C2_MMIO_WRITE32(address, value);
    c2_mmio_fence();
}

#endif /* STARRYSKY_C2_COMMON_H */
