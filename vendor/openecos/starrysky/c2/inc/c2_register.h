/* Common register-layout contract for StarrySky C2. */
#ifndef STARRYSKY_C2_REGISTER_H
#define STARRYSKY_C2_REGISTER_H

#include <stdint.h>

/*
 * Each register union provides WORD (32-bit access) and BITS (field view).
 * Bit-field allocation and union type-punning are compiler-dependent: these
 * definitions target little-endian GCC/Clang with LSB-first uint32_t fields.
 * Validate field layout and generated MMIO access width for the target ABI.
 * A sizeof assertion alone does not establish bit order or access width.
 *
 * Do not directly write MMIO bit-fields: this can perform a read-modify-write
 * or a narrow access. Build a zero-initialized local register/shadow value,
 * then write WORD once. For GPIO, preserve outputs/directions in software
 * shadows, never by reading DR/DDR. For FIFO, read-clear and changing status,
 * read WORD once and decode the local snapshot (not repeated MMIO bit reads).
 * WO is documentary: C cannot forbid reads of a writable member.
 * UNKNOWN fields are undocumented, NOT confirmed reserved/zero bits.
 * Full-width fields model a 32-bit software word, not necessarily 32 valid
 * hardware bits. No reset values are implied by these declarations.
 */
#if defined(__BYTE_ORDER__) && defined(__ORDER_LITTLE_ENDIAN__)
#if __BYTE_ORDER__ != __ORDER_LITTLE_ENDIAN__
#error "StarrySky C2 register bit-fields require a little-endian target"
#endif
#endif

#if defined(__cplusplus)
#define C2_REG_ASSERT_SIZE(type, size) static_assert(sizeof(type) == (size), #type " size")
#else
#define C2_REG_ASSERT_SIZE(type, size) _Static_assert(sizeof(type) == (size), #type " size")
#endif

#endif /* STARRYSKY_C2_REGISTER_H */
