/* StarrySky C2 architecture information registers.
 * Source: works/docs/periphs/archinfo.md. These registers are writable;
 * reference IP reset values must not be taken as C2 silicon identity.
 */
#ifndef STARRYSKY_C2_ARCHINFO_H
#define STARRYSKY_C2_ARCHINFO_H

#include "c2_register.h"

typedef union {
    uint32_t WORD;
    struct {
        uint32_t SRAM : 8; /* [7:0] */
        uint32_t CLOCK : 12; /* [19:8] */
        uint32_t RESERVED0 : 12; /* [31:20] */
    } BITS;
} C2_ARCHINFO_SYS_TypeDef;
C2_REG_ASSERT_SIZE(C2_ARCHINFO_SYS_TypeDef, 4);

typedef union {
    uint32_t WORD;
    struct {
        uint32_t CUST : 6; /* [5:0] */
        uint32_t PROCESS : 16; /* [21:6] */
        uint32_t VENDOR : 8; /* [29:22] */
        uint32_t TYPE : 2; /* [31:30] */
    } BITS;
} C2_ARCHINFO_IDL_TypeDef;
C2_REG_ASSERT_SIZE(C2_ARCHINFO_IDL_TypeDef, 4);

typedef union {
    uint32_t WORD;
    struct {
        uint32_t DATE : 24; /* [23:0] */
        uint32_t RESERVED0 : 8; /* [31:24] */
    } BITS;
} C2_ARCHINFO_IDH_TypeDef;
C2_REG_ASSERT_SIZE(C2_ARCHINFO_IDH_TypeDef, 4);

typedef struct {
    volatile C2_ARCHINFO_SYS_TypeDef SYS; /* 0x00: system information. */
    volatile C2_ARCHINFO_IDL_TypeDef IDL; /* 0x04: architecture information, low word. */
    volatile C2_ARCHINFO_IDH_TypeDef IDH; /* 0x08: architecture information, high word. */
} C2_ARCHINFO_TypeDef;

#define C2_ARCHINFO0_BASE UINT32_C(0x20001000)
#define C2_ARCHINFO0 ((C2_ARCHINFO_TypeDef *)(uintptr_t)C2_ARCHINFO0_BASE)

#include "c2_common.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uint32_t sys;
    uint32_t idl;
    uint32_t idh;
} c2_archinfo_snapshot_t;

typedef struct {
    uint16_t clock_code; /* Decoded BCD, NOT a measured CPU clock in Hz. */
    uint16_t process_nm;
    uint16_t year;
    uint8_t month;
    uint8_t sram_kb;
    uint8_t vendor;
    uint8_t type;
    uint8_t custom;
} c2_archinfo_info_t;

/* No initialization or enable operation is needed. Each register is read
 * once; caller must exclude concurrent writers for a consistent snapshot. */
c2_status_t c2_archinfo_read(C2_ARCHINFO_TypeDef *reg,
                             c2_archinfo_snapshot_t *snapshot);
/* Explicit writable-IP/debug operation, not a clock/SRAM configuration!
 * Rejects reserved bits. Writes three words, not atomically. Documentation
 * disagrees about silicon writability; OK means writes issued, NOT verified.
 * Use read-back if needed. No driver invokes this during initialization. */
c2_status_t c2_archinfo_write(C2_ARCHINFO_TypeDef *reg,
                              const c2_archinfo_snapshot_t *snapshot);
/* Pure snapshot decode. Invalid BCD returns IO and leaves info unchanged.
 * Calendar month is decoded verbatim (0 or >12 may be unspecified metadata).
 * Register contents are writable metadata, not trusted silicon identity. */
c2_status_t c2_archinfo_decode(const c2_archinfo_snapshot_t *snapshot,
                               c2_archinfo_info_t *info);

#ifdef __cplusplus
}
#endif

#endif /* STARRYSKY_C2_ARCHINFO_H */
