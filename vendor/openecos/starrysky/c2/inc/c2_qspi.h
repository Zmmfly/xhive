/* StarrySky C2 QSPI registers and evidence-bounded basic driver.
 * Sources: works/docs/periphs/spi.md, the StarrySkyC2 SDK qspi.c and start.S.
 * Undocumented fields are deliberately exposed only as raw whole words.
 */
#ifndef STARRYSKY_C2_QSPI_H
#define STARRYSKY_C2_QSPI_H

#include "c2_common.h"
#include "c2_register.h"

/* Read bit 0 is used by the SDK FIFO path. The PSRAM raw path instead waits
 * for the complete STATUS word to equal 1; those are distinct modes. */
typedef union {
    uint32_t WORD;
    struct {
        uint32_t BUSY : 1;       /* [0], SDK interpretation only. */
        uint32_t GO : 1;         /* [1], inferred from raw write 0x102. */
        uint32_t UNKNOWN0 : 2;   /* [3:2] */
        uint32_t RESET : 1;      /* [4], inferred from raw write 0x10. */
        uint32_t UNKNOWN1 : 3;   /* [7:5] */
        uint32_t LOCK : 1;       /* [8], inferred from raw write 0x102. */
        uint32_t UNKNOWN2 : 23;  /* [31:9] */
    } BITS;
} C2_QSPI_STATUS_TypeDef;
C2_REG_ASSERT_SIZE(C2_QSPI_STATUS_TypeDef, 4);

typedef union { uint32_t WORD; struct { uint32_t CLKDIV : 32; } BITS; } C2_QSPI_CLKDIV_TypeDef;
typedef union {
    uint32_t WORD;
    struct {
        uint32_t UNKNOWN0 : 16;
        uint32_t CS : 3;         /* [18:16], observed encodings 1..4. */
        uint32_t UNKNOWN1 : 13;
    } BITS;
} C2_QSPI_CMD_TypeDef;
typedef union { uint32_t WORD; struct { uint32_t ADR : 32; } BITS; } C2_QSPI_ADR_TypeDef;
typedef union { uint32_t WORD; struct { uint32_t UNKNOWN0 : 32; } BITS; } C2_QSPI_LEN_TypeDef;
typedef union { uint32_t WORD; struct { uint32_t DUM : 32; } BITS; } C2_QSPI_DUM_TypeDef;
typedef union { uint32_t WORD; struct { uint32_t DATA : 32; } BITS; } C2_QSPI_TXFIFO_TypeDef;
typedef union { uint32_t WORD; struct { uint32_t DATA : 32; } BITS; } C2_QSPI_RXFIFO_TypeDef;
typedef union { uint32_t WORD; struct { uint32_t UNKNOWN0 : 32; } BITS; } C2_QSPI_INTCFG_TypeDef;
typedef union { uint32_t WORD; struct { uint32_t UNKNOWN0 : 32; } BITS; } C2_QSPI_INTSTA_TypeDef;
C2_REG_ASSERT_SIZE(C2_QSPI_CLKDIV_TypeDef, 4);
C2_REG_ASSERT_SIZE(C2_QSPI_CMD_TypeDef, 4);
C2_REG_ASSERT_SIZE(C2_QSPI_ADR_TypeDef, 4);
C2_REG_ASSERT_SIZE(C2_QSPI_LEN_TypeDef, 4);
C2_REG_ASSERT_SIZE(C2_QSPI_DUM_TypeDef, 4);
C2_REG_ASSERT_SIZE(C2_QSPI_TXFIFO_TypeDef, 4);
C2_REG_ASSERT_SIZE(C2_QSPI_RXFIFO_TypeDef, 4);
C2_REG_ASSERT_SIZE(C2_QSPI_INTCFG_TypeDef, 4);
C2_REG_ASSERT_SIZE(C2_QSPI_INTSTA_TypeDef, 4);

typedef struct {
    volatile C2_QSPI_STATUS_TypeDef STATUS;       /* 0x00: read status/write control. */
    volatile C2_QSPI_CLKDIV_TypeDef CLKDIV;       /* 0x04: SCK=APB/(2*(div+1)). */
    volatile C2_QSPI_CMD_TypeDef CMD;             /* 0x08 */
    volatile C2_QSPI_ADR_TypeDef ADR;             /* 0x0c */
    volatile C2_QSPI_LEN_TypeDef LEN;             /* 0x10 */
    volatile C2_QSPI_DUM_TypeDef DUM;             /* 0x14 */
    volatile C2_QSPI_TXFIFO_TypeDef TXFIFO;       /* 0x18: WO in known use. */
    uint32_t RESERVED0;                           /* 0x1c: do not access. */
    volatile const C2_QSPI_RXFIFO_TypeDef RXFIFO; /* 0x20: raw RO. */
    volatile C2_QSPI_INTCFG_TypeDef INTCFG;       /* 0x24 */
    volatile C2_QSPI_INTSTA_TypeDef INTSTA;       /* 0x28: clear semantics unknown. */
} C2_QSPI_TypeDef;
C2_REG_ASSERT_SIZE(C2_QSPI_TypeDef, 0x2c);
#if defined(__cplusplus)
static_assert(offsetof(C2_QSPI_TypeDef, RXFIFO) == 0x20, "C2 QSPI RXFIFO offset");
#else
_Static_assert(offsetof(C2_QSPI_TypeDef, RXFIFO) == 0x20, "C2 QSPI RXFIFO offset");
#endif

#define C2_QSPI0_BASE UINT32_C(0x20007000)
#define C2_QSPI0 ((C2_QSPI_TypeDef *)(uintptr_t)C2_QSPI0_BASE)

#define C2_QSPI_STATUS_SDK_BUSY_MASK UINT32_C(0x00000001)
#define C2_QSPI_CONTROL_RESET_ASSERT UINT32_C(0x00000010)
#define C2_QSPI_RAW_PSRAM_TRIGGER UINT32_C(0x00000102)
#define C2_QSPI_RAW_PSRAM_DONE UINT32_C(0x00000001)
#define C2_QSPI_RAW_PSRAM_BYTE_LENGTH UINT32_C(0x00080000)

typedef enum {
    C2_QSPI_CS0 = UINT32_C(1) << 16,
    C2_QSPI_CS1 = UINT32_C(2) << 16,
    C2_QSPI_CS2 = UINT32_C(3) << 16,
    C2_QSPI_CS3 = UINT32_C(4) << 16
} c2_qspi_chip_select_t;

/* Raw transaction writes are opt-in because CMD/ADR/LEN/DUM formats are not
 * published. At most one TXFIFO word is submitted; FIFO depth and RX framing
 * are unknown. The caller owns every raw word and the completion predicate. */
#define C2_QSPI_RAW_WRITE_CMD    (UINT32_C(1) << 0)
#define C2_QSPI_RAW_WRITE_ADR    (UINT32_C(1) << 1)
#define C2_QSPI_RAW_WRITE_LEN    (UINT32_C(1) << 2)
#define C2_QSPI_RAW_WRITE_DUM    (UINT32_C(1) << 3)
#define C2_QSPI_RAW_WRITE_TXFIFO (UINT32_C(1) << 4)
#define C2_QSPI_RAW_WRITE_ALL_MASK UINT32_C(0x0000001f)

typedef struct {
    uint32_t write_mask;
    uint32_t command_word;
    uint32_t address_word;
    uint32_t length_word;
    uint32_t dummy_word;
    uint32_t tx_word;
    uint32_t trigger_word;
    uint32_t completion_mask;
    uint32_t completion_value;
} c2_qspi_raw_transaction_t;

#ifdef __cplusplus
extern "C" {
#endif

/* Every API that writes controller state requires exclusive ownership. The
 * caller must ensure there is no active transaction, Flash XIP fetch, mapped
 * PSRAM access, ISR, DMA or other QSPI master unless that API explicitly does
 * an SDK-idle wait. Software call serialization alone is not hardware idle.
 *
 * Mirrors SDK init: writes CLKDIV, then clears CMD/ADR/LEN. It does not reset,
 * wait, configure DUM/interrupts, or prove that the precondition above holds.
 * Effective CLKDIV width is unpublished, so the complete uint32_t is passed. */
c2_status_t c2_qspi_init(C2_QSPI_TypeDef *reg, uint32_t clkdiv);
c2_status_t c2_qspi_set_clkdiv(C2_QSPI_TypeDef *reg, uint32_t clkdiv);
/* Observed reset pulse only: STATUS=0x10 then STATUS=0. No ready indication is
 * known, so callers must provide any required device/controller guard time. */
c2_status_t c2_qspi_reset(C2_QSPI_TypeDef *reg);
/* Mirrors SDK _cs ordering: wait for SDK-idle first, then write the complete
 * CMD chip-select word. poll_limit counts this call's STATUS reads; zero reads
 * nothing and returns C2_ERROR_TIMEOUT. This overwrites unknown CMD fields;
 * only the four enumerated encodings are accepted. The owner must keep the
 * session locked between select and the following FIFO write. */
c2_status_t c2_qspi_select(C2_QSPI_TypeDef *reg,
                           c2_qspi_chip_select_t cs,
                           uint32_t poll_limit);
c2_status_t c2_qspi_read_status(C2_QSPI_TypeDef *reg, uint32_t *status);

/* SDK FIFO mode only. poll_limit counts STATUS reads, not time. zero performs
 * no read and returns C2_ERROR_TIMEOUT. STATUS bit0 clear is considered idle;
 * this must not be used for PSRAM raw completion. */
c2_status_t c2_qspi_wait_sdk_idle(C2_QSPI_TypeDef *reg, uint32_t poll_limit);
/* Waits for SDK-idle and submits one right-aligned 32-bit FIFO word. Success
 * means submitted, not completed. poll_limit covers this call only. */
c2_status_t c2_qspi_fifo_write_word(C2_QSPI_TypeDef *reg,
                                    uint32_t word,
                                    uint32_t poll_limit);
/* Raw RXFIFO pop/snapshot. No ready/valid evidence exists; caller must already
 * know the operation-specific timing, alignment and valid byte count. */
c2_status_t c2_qspi_fifo_read_word_raw(C2_QSPI_TypeDef *reg, uint32_t *word);

/* SDK-compatible byte submission: each uint8_t is zero-extended/right-aligned
 * as in qspi.c, never left-aligned. One total poll budget is shared by every
 * pre-write wait plus the final completion wait. A nonempty transfer requires
 * poll_limit >= length+1; otherwise it is rejected before MMIO. length=0 is a
 * no-op and permits data=NULL/poll_limit=0. On a runtime timeout, earlier bytes
 * may already be submitted and *bytes_submitted reports that count. Neither
 * exact wire bit-count nor CS framing/holding is published by the SDK; success
 * only observes its STATUS-bit convention, not slave acceptance. */
c2_status_t c2_qspi_write_bytes(C2_QSPI_TypeDef *reg,
                                const uint8_t *data,
                                size_t length,
                                uint32_t poll_limit,
                                size_t *bytes_submitted);

/* Full finite raw transaction. poll_limit is the total number of STATUS reads
 * after the trigger; zero is rejected before MMIO. completion_mask must be
 * nonzero. last_status receives the last sampled word when non-NULL. No RXFIFO
 * read is implicit. It does not pre-wait idle or reject a stale status that
 * already satisfies the predicate. Unknown raw encodings, exclusivity and
 * operation-specific readiness remain entirely caller-owned. */
c2_status_t c2_qspi_transaction_raw(C2_QSPI_TypeDef *reg,
                                    const c2_qspi_raw_transaction_t *transaction,
                                    uint32_t poll_limit,
                                    uint32_t *last_status);

#ifdef __cplusplus
}
#endif

#endif /* STARRYSKY_C2_QSPI_H */
