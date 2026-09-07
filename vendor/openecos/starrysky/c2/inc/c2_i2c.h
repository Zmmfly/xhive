/* StarrySky C2 I2C registers. Source: works/docs/periphs/i2c.md.
 * All accesses are 32-bit; PSCR uses 16 bits, other registers use 8 bits.
 * C cannot enforce write-only access: do not read CMD.
 */
#ifndef STARRYSKY_C2_I2C_H
#define STARRYSKY_C2_I2C_H

#include "c2_common.h"
#include "c2_register.h"

typedef union {
    uint32_t WORD;
    struct {
        uint32_t RESERVED0 : 6; /* [5:0] */
        uint32_t IEN : 1; /* [6] */
        uint32_t EN : 1; /* [7] */
        uint32_t RESERVED1 : 24; /* [31:8] */
    } BITS;
} C2_I2C_CTRL_TypeDef;
C2_REG_ASSERT_SIZE(C2_I2C_CTRL_TypeDef, 4);

typedef union {
    uint32_t WORD;
    struct {
        uint32_t PSCR : 16; /* [15:0] */
        uint32_t RESERVED0 : 16; /* [31:16] */
    } BITS;
} C2_I2C_PSCR_TypeDef;
C2_REG_ASSERT_SIZE(C2_I2C_PSCR_TypeDef, 4);

typedef union {
    uint32_t WORD;
    struct {
        uint32_t DATA : 8; /* [7:0] */
        uint32_t RESERVED0 : 24; /* [31:8] */
    } BITS;
} C2_I2C_TXR_TypeDef;
C2_REG_ASSERT_SIZE(C2_I2C_TXR_TypeDef, 4);

typedef union {
    uint32_t WORD;
    struct {
        uint32_t DATA : 8; /* [7:0] */
        uint32_t RESERVED0 : 24; /* [31:8] */
    } BITS;
} C2_I2C_RXR_TypeDef;
C2_REG_ASSERT_SIZE(C2_I2C_RXR_TypeDef, 4);

/* WO command; ACK=0 sends NACK, ACK=1 sends ACK. Write WORD once. */
typedef union {
    uint32_t WORD;
    struct {
        uint32_t IACK : 1; /* [0] */
        uint32_t RESERVED0 : 2; /* [2:1] */
        uint32_t ACK : 1; /* [3] */
        uint32_t WR : 1; /* [4] */
        uint32_t RD : 1; /* [5] */
        uint32_t STO : 1; /* [6] */
        uint32_t STA : 1; /* [7] */
        uint32_t RESERVED1 : 24; /* [31:8] */
    } BITS;
} C2_I2C_CMD_TypeDef;
C2_REG_ASSERT_SIZE(C2_I2C_CMD_TypeDef, 4);

/* RXK=1 means NACK; IF is cleared through CMD.IACK. */
typedef union {
    uint32_t WORD;
    struct {
        uint32_t IF : 1; /* [0] */
        uint32_t TIP : 1; /* [1] */
        uint32_t RESERVED0 : 3; /* [4:2] */
        uint32_t AL : 1; /* [5] */
        uint32_t BSY : 1; /* [6] */
        uint32_t RXK : 1; /* [7] */
        uint32_t RESERVED1 : 24; /* [31:8] */
    } BITS;
} C2_I2C_SR_TypeDef;
C2_REG_ASSERT_SIZE(C2_I2C_SR_TypeDef, 4);

typedef struct {
    volatile C2_I2C_CTRL_TypeDef CTRL;      /* 0x00: controller and IRQ enable. */
    volatile C2_I2C_PSCR_TypeDef PSCR;      /* 0x04: SCL prescaler. */
    volatile C2_I2C_TXR_TypeDef TXR;       /* 0x08: RW; transmit address/data. */
    volatile const C2_I2C_RXR_TypeDef RXR; /* 0x0C: RO; received byte. */
    volatile C2_I2C_CMD_TypeDef CMD;       /* 0x10: WO; transaction commands. */
    volatile const C2_I2C_SR_TypeDef SR;  /* 0x14: RO; controller status. */
} C2_I2C_TypeDef;

#define C2_I2C0_BASE UINT32_C(0x20006000)
#define C2_I2C0 ((C2_I2C_TypeDef *)(uintptr_t)C2_I2C0_BASE)

#define C2_I2C_CTRL_IEN UINT32_C(0x40)
#define C2_I2C_CTRL_EN  UINT32_C(0x80)

#define C2_I2C_CMD_IACK UINT32_C(0x01)
#define C2_I2C_CMD_ACK   UINT32_C(0x08)
#define C2_I2C_CMD_WR    UINT32_C(0x10)
#define C2_I2C_CMD_RD    UINT32_C(0x20)
#define C2_I2C_CMD_STO   UINT32_C(0x40)
#define C2_I2C_CMD_STA   UINT32_C(0x80)

#define C2_I2C_STATUS_IF   UINT32_C(0x01)
#define C2_I2C_STATUS_TIP  UINT32_C(0x02)
#define C2_I2C_STATUS_AL   UINT32_C(0x20)
#define C2_I2C_STATUS_BSY  UINT32_C(0x40)
#define C2_I2C_STATUS_RXK  UINT32_C(0x80)
#define C2_I2C_PRESCALER_MAX UINT32_C(0xFFFF)

typedef struct {
    uint32_t raw;
    bool interrupt_pending;
    bool transfer_in_progress;
    bool arbitration_lost;
    bool bus_busy;
    bool nack_received;
} c2_i2c_status_snapshot_t;

#ifdef __cplusplus
extern "C" {
#endif

/* Configure only while the controller and bus are idle. These functions leave
 * the controller disabled. The clock form uses:
 *
 *   SCL = clock_hz / (5 * (prescaler + 1))
 *
 * and rounds the divider upward so the requested bus_hz is not exceeded.
 * prescaler_out may be NULL. Explicit prescaler values above 0xFFFF and clock
 * ratios outside the 16-bit divider range are rejected.
 */
c2_status_t c2_i2c_configure_prescaler(C2_I2C_TypeDef *regs,
                                         uint32_t prescaler);
c2_status_t c2_i2c_configure(C2_I2C_TypeDef *regs,
                              uint32_t clock_hz,
                              uint32_t bus_hz,
                              uint16_t *prescaler_out);

c2_status_t c2_i2c_enable(C2_I2C_TypeDef *regs);
c2_status_t c2_i2c_disable(C2_I2C_TypeDef *regs);
c2_status_t c2_i2c_set_interrupt_enabled(C2_I2C_TypeDef *regs,
                                          bool enabled);
c2_status_t c2_i2c_get_status(const C2_I2C_TypeDef *regs,
                               c2_i2c_status_snapshot_t *snapshot);

/* Acknowledge the latched IF flag without installing an IRQ handler. */
c2_status_t c2_i2c_ack_interrupt(C2_I2C_TypeDef *regs,
                                  uint32_t poll_limit);

/* Blocking 7-bit master transfers. write_read keeps ownership between phases
 * and emits a repeated START; it is suitable for register-address reads.
 * write_length and read_length must both be nonzero for write_read.
 *
 * Read ACK polarity follows works/docs/periphs/i2c.md: bit3=1 for intermediate
 * ACK, bit3=0 for final NACK. The reference OpenCores bit-controller and pad
 * enable conventions conflict; C2's unpublished top-level wiring cannot be
 * inferred from them. Confirm the ACK/NACK waveform on the actual board.
 *
 * poll_limit is a count of SR reads, not microseconds. Zero is rejected before
 * a transfer is started. The limit applies independently to each issued byte
 * or STOP command and to each bounded IF-clear/bus-idle cleanup phase; total
 * transaction work therefore scales with byte count. The driver never waits
 * forever, uses no OS/heap, and requires external serialization.
 *
 * Arbitration loss never causes a software STOP, because another master may
 * own the bus. NACK and timeout paths make a bounded STOP attempt when AL is
 * not observed. A timeout does not guarantee physical bus release. GPIO bus
 * recovery is unsupported because C2 I2C uses fixed pinmux.
 */
c2_status_t c2_i2c_master_write(C2_I2C_TypeDef *regs,
                                 uint8_t address,
                                 const uint8_t *data,
                                 size_t length,
                                 uint32_t poll_limit);
c2_status_t c2_i2c_master_read(C2_I2C_TypeDef *regs,
                                uint8_t address,
                                uint8_t *data,
                                size_t length,
                                uint32_t poll_limit);
c2_status_t c2_i2c_master_write_read(C2_I2C_TypeDef *regs,
                                      uint8_t address,
                                      const uint8_t *write_data,
                                      size_t write_length,
                                      uint8_t *read_data,
                                      size_t read_length,
                                      uint32_t poll_limit);

#ifdef __cplusplus
}
#endif

#endif /* STARRYSKY_C2_I2C_H */
