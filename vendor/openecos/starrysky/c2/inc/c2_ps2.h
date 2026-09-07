/* StarrySky C2 PS/2 registers. Source: works/docs/periphs/ps2.md. */
#ifndef STARRYSKY_C2_PS2_H
#define STARRYSKY_C2_PS2_H

#include "c2_register.h"

typedef union {
    uint32_t WORD;
    struct {
        uint32_t ITN : 1; /* [0] */
        uint32_t EN : 1; /* [1] */
        uint32_t RESERVED0 : 30; /* [31:2] */
    } BITS;
} C2_PS2_CTRL_TypeDef;
C2_REG_ASSERT_SIZE(C2_PS2_CTRL_TypeDef, 4);

typedef union {
    uint32_t WORD;
    struct {
        uint32_t DATA : 8; /* [7:0] */
        uint32_t RESERVED0 : 24; /* [31:8] */
    } BITS;
} C2_PS2_DATA_TypeDef;
C2_REG_ASSERT_SIZE(C2_PS2_DATA_TypeDef, 4);

typedef union {
    uint32_t WORD;
    struct {
        uint32_t ITF : 1; /* [0] */
        uint32_t RESERVED0 : 31; /* [31:1] */
    } BITS;
} C2_PS2_STAT_TypeDef;
C2_REG_ASSERT_SIZE(C2_PS2_STAT_TypeDef, 4);

typedef struct {
    volatile C2_PS2_CTRL_TypeDef CTRL;       /* 0x00: receive and interrupt enable. */
    volatile const C2_PS2_DATA_TypeDef DATA; /* 0x04: RO; read pops receive FIFO. */
    volatile const C2_PS2_STAT_TypeDef STAT; /* 0x08: RO; read clears interrupt flag. */
} C2_PS2_TypeDef;

#define C2_PS20_BASE UINT32_C(0x20005000)
#define C2_PS20 ((C2_PS2_TypeDef *)(uintptr_t)C2_PS20_BASE)

#include "c2_common.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Receive-only controller; no transmit/keyboard-command interface is known.
 * init enables reception and selects IRQ gating; no IRQ handler is installed.
 * enable() and irq_enable() preserve the other documented control bit. */
c2_status_t c2_ps2_init(C2_PS2_TypeDef *reg, bool irq_enable);
c2_status_t c2_ps2_deinit(C2_PS2_TypeDef *reg);
c2_status_t c2_ps2_enable(C2_PS2_TypeDef *reg, bool enable);
c2_status_t c2_ps2_irq_enable(C2_PS2_TypeDef *reg, bool enable);
/* Exactly one STAT read; reading acknowledges/clears ITF. It is not a
 * non-destructive FIFO-ready query. Does not consume DATA. */
c2_status_t c2_ps2_status_ack(C2_PS2_TypeDef *reg, bool *pending);
/* One DATA read pops FIFO; zero is the documented empty sentinel, so the
 * try/receive APIs cannot distinguish a received 0x00 from empty. read_raw
 * exposes that ambiguity without discarding zero. No helper reads STAT. */
c2_status_t c2_ps2_read_raw(C2_PS2_TypeDef *reg, uint8_t *data);
c2_status_t c2_ps2_try_read(C2_PS2_TypeDef *reg, uint8_t *data);
/* poll_limit is the maximum total DATA reads for the entire call. Zero
 * means no polling (TIMEOUT for nonempty request). received is mandatory
 * and reports partial progress. length=0 permits data=NULL and returns OK.
 * Reception must already be enabled; helpers can also drain retained FIFO. */
c2_status_t c2_ps2_receive(C2_PS2_TypeDef *reg, uint8_t *data, size_t length,
                           size_t *received, uint32_t poll_limit);

#ifdef __cplusplus
}
#endif

#endif /* STARRYSKY_C2_PS2_H */
