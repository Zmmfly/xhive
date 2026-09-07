/* StarrySky C2 UART registers. Source: works/docs/periphs/uart.md.
 * SYS_UART and HP_UART are different IPs, not interchangeable instances.
 * All accesses are 32-bit, even for byte data / 16-bit divider fields.
 */
#ifndef STARRYSKY_C2_UART_H
#define STARRYSKY_C2_UART_H

#include "c2_common.h"
#include "c2_register.h"

typedef union {
    uint32_t WORD;
    struct {
        uint32_t CLKDIV : 32; /* [31:0] */
    } BITS;
} C2_SYS_UART_CLKDIV_TypeDef;
C2_REG_ASSERT_SIZE(C2_SYS_UART_CLKDIV_TypeDef, 4);

/* UNKNOWN0 contains the RX-empty indication; read WORD once, preserving all 32 bits. */
typedef union {
    uint32_t WORD;
    struct {
        uint32_t DATA : 8; /* [7:0] */
        uint32_t UNKNOWN0 : 24; /* [31:8] */
    } BITS;
} C2_SYS_UART_DATA_TypeDef;
C2_REG_ASSERT_SIZE(C2_SYS_UART_DATA_TypeDef, 4);

typedef struct {
    volatile C2_SYS_UART_CLKDIV_TypeDef CLKDIV; /* 0x00: clock / baud (no minus one). */
    volatile C2_SYS_UART_DATA_TypeDef DATA;   /* 0x04: TX write / RX read; keep all 32 bits
                              * when checking the empty indication. */
} C2_SYS_UART_TypeDef;

typedef union {
    uint32_t WORD;
    struct {
        uint32_t RXIE : 1; /* [0] */
        uint32_t TXIE : 1; /* [1] */
        uint32_t PEIE : 1; /* [2] */
        uint32_t WLS : 2; /* [4:3] */
        uint32_t STB : 1; /* [5] */
        uint32_t PEN : 1; /* [6] */
        uint32_t PS : 2; /* [8:7] */
        uint32_t RESERVED0 : 23; /* [31:9] */
    } BITS;
} C2_HP_UART_LCR_TypeDef;
C2_REG_ASSERT_SIZE(C2_HP_UART_LCR_TypeDef, 4);

typedef union {
    uint32_t WORD;
    struct {
        uint32_t DIV : 16; /* [15:0] */
        uint32_t RESERVED0 : 16; /* [31:16] */
    } BITS;
} C2_HP_UART_DIV_TypeDef;
C2_REG_ASSERT_SIZE(C2_HP_UART_DIV_TypeDef, 4);

/* DATA is TX on write, RX on read; read pops FIFO. */
typedef union {
    uint32_t WORD;
    struct {
        uint32_t DATA : 8; /* [7:0] */
        uint32_t RESERVED0 : 24; /* [31:8] */
    } BITS;
} C2_HP_UART_TRX_TypeDef;
C2_REG_ASSERT_SIZE(C2_HP_UART_TRX_TypeDef, 4);

/* WO fields: build a local value and write WORD; never read-modify-write. */
typedef union {
    uint32_t WORD;
    struct {
        uint32_t RF_CLR : 1; /* [0] */
        uint32_t TF_CLR : 1; /* [1] */
        uint32_t RX_TRG_LEVL : 2; /* [3:2] */
        uint32_t RESERVED0 : 28; /* [31:4] */
    } BITS;
} C2_HP_UART_FCR_TypeDef;
C2_REG_ASSERT_SIZE(C2_HP_UART_FCR_TypeDef, 4);

typedef union {
    uint32_t WORD;
    struct {
        uint32_t RXIP : 1; /* [0] */
        uint32_t TXIP : 1; /* [1] */
        uint32_t PEIP : 1; /* [2] */
        uint32_t DR : 1; /* [3] */
        uint32_t PE : 1; /* [4] */
        uint32_t THRE : 1; /* [5] */
        uint32_t TEMT : 1; /* [6] */
        uint32_t EMPT : 1; /* [7] */
        uint32_t FULL : 1; /* [8] */
        uint32_t RESERVED0 : 23; /* [31:9] */
    } BITS;
} C2_HP_UART_LSR_TypeDef;
C2_REG_ASSERT_SIZE(C2_HP_UART_LSR_TypeDef, 4);

typedef struct {
    volatile C2_HP_UART_LCR_TypeDef LCR;       /* 0x00: line control. */
    volatile C2_HP_UART_DIV_TypeDef DIV;       /* 0x04: clock / baud - 1; low 16 bits. */
    volatile C2_HP_UART_TRX_TypeDef TRX;       /* 0x08: TX write / RX read. */
    volatile C2_HP_UART_FCR_TypeDef FCR;       /* 0x0C: FIFO control. */
    volatile const C2_HP_UART_LSR_TypeDef LSR; /* 0x10: line status. */
} C2_HP_UART_TypeDef;

#define C2_UART0_BASE UINT32_C(0x10001000)
#define C2_UART1_BASE UINT32_C(0x20003000)
#define C2_UART0 ((C2_SYS_UART_TypeDef *)(uintptr_t)C2_UART0_BASE)
#define C2_UART1 ((C2_HP_UART_TypeDef *)(uintptr_t)C2_UART1_BASE)

#define C2_HP_UART_IRQ_RX       UINT32_C(0x001)
#define C2_HP_UART_IRQ_TX       UINT32_C(0x002)
#define C2_HP_UART_IRQ_PARITY   UINT32_C(0x004)
#define C2_HP_UART_IRQ_ALL      UINT32_C(0x007)

#define C2_HP_UART_STATUS_RX_IRQ_PENDING     UINT32_C(0x001)
#define C2_HP_UART_STATUS_TX_IRQ_PENDING     UINT32_C(0x002)
#define C2_HP_UART_STATUS_PARITY_IRQ_PENDING UINT32_C(0x004)
#define C2_HP_UART_STATUS_RX_DATA_READY      UINT32_C(0x008)
#define C2_HP_UART_STATUS_PARITY_ERROR       UINT32_C(0x010)
#define C2_HP_UART_STATUS_TX_FIFO_EMPTY      UINT32_C(0x020)
#define C2_HP_UART_STATUS_TX_IDLE            UINT32_C(0x040)
#define C2_HP_UART_STATUS_RX_FIFO_EMPTY      UINT32_C(0x080)
#define C2_HP_UART_STATUS_TX_FIFO_FULL       UINT32_C(0x100)
#define C2_HP_UART_STATUS_ALL                UINT32_C(0x1ff)

typedef enum {
    C2_UART_PARITY_NONE = 0,
    C2_UART_PARITY_ODD,
    C2_UART_PARITY_EVEN
} c2_uart_parity_t;

typedef enum {
    C2_UART_STOP_BITS_1 = 1,
    C2_UART_STOP_BITS_2 = 2
} c2_uart_stop_bits_t;

typedef enum {
    C2_HP_UART_RX_TRIGGER_1_BYTE = 0,
    C2_HP_UART_RX_TRIGGER_2_BYTES = 1,
    C2_HP_UART_RX_TRIGGER_8_BYTES = 2,
    C2_HP_UART_RX_TRIGGER_14_BYTES = 3
} c2_hp_uart_rx_trigger_t;

typedef enum {
    C2_HP_UART_FLUSH_RX = 1,
    C2_HP_UART_FLUSH_TX = 2,
    C2_HP_UART_FLUSH_RX_TX = 3
} c2_hp_uart_flush_t;

typedef struct {
    C2_SYS_UART_TypeDef *regs;
    bool initialized;
} c2_sys_uart_t;

typedef struct {
    uint32_t baud_rate;
    uint32_t irq_enable_mask;
    uint8_t data_bits;
    c2_uart_parity_t parity;
    c2_uart_stop_bits_t stop_bits;
    c2_hp_uart_rx_trigger_t rx_trigger;
} c2_hp_uart_config_t;

typedef struct {
    C2_HP_UART_TypeDef *regs;
    uint32_t lcr_shadow;
    uint32_t fcr_trigger_shadow;
    bool initialized;
} c2_hp_uart_t;

#ifdef __cplusplus
extern "C" {
#endif

/* SYS divider is clock_hz / baud_rate (no minus one). Disable is a software
 * driver gate only: this IP exposes no documented hardware enable bit. */
c2_status_t c2_sys_uart_init(c2_sys_uart_t *uart,
                             C2_SYS_UART_TypeDef *regs,
                             uint32_t clock_hz,
                             uint32_t baud_rate);
c2_status_t c2_sys_uart_configure(c2_sys_uart_t *uart,
                                  uint32_t clock_hz,
                                  uint32_t baud_rate);
c2_status_t c2_sys_uart_disable(c2_sys_uart_t *uart);

/* SYS UART has no TX-ready indication. Writes are immediate, never claim
 * completion on the wire, and no SYS flush API is provided. */
c2_status_t c2_sys_uart_write_byte(c2_sys_uart_t *uart, uint8_t byte);
c2_status_t c2_sys_uart_write(c2_sys_uart_t *uart,
                              const uint8_t *data,
                              size_t length,
                              size_t *written);

/* Nonblocking read performs one DATA read and returns C2_ERROR_BUSY when its
 * undocumented high 24-bit empty/invalid indication is nonzero. Blocking
 * poll_limit is the total number of DATA reads, not microseconds; zero means
 * no MMIO read and immediate C2_ERROR_TIMEOUT. Valid range is 0..UINT32_MAX. */
c2_status_t c2_sys_uart_read_byte_nonblocking(c2_sys_uart_t *uart,
                                              uint8_t *byte);
c2_status_t c2_sys_uart_read_byte(c2_sys_uart_t *uart,
                                  uint8_t *byte,
                                  uint32_t poll_limit);
c2_status_t c2_sys_uart_read(c2_sys_uart_t *uart,
                             uint8_t *data,
                             size_t length,
                             size_t *read_count,
                             uint32_t poll_limit);

/* HP divider is clock_hz / baud_rate - 1 and must fit its documented 16-bit
 * field with DIV >= 2. Configuration pulses both FIFO clears while preserving
 * rx_trigger, then programs the RTL-defined line format. Argument-validation
 * failure occurs before MMIO and leaves an existing context unchanged. */
c2_status_t c2_hp_uart_init(c2_hp_uart_t *uart,
                            C2_HP_UART_TypeDef *regs,
                            uint32_t clock_hz,
                            const c2_hp_uart_config_t *config);
c2_status_t c2_hp_uart_configure(c2_hp_uart_t *uart,
                                 uint32_t clock_hz,
                                 const c2_hp_uart_config_t *config);
/* Clears interrupt enables and gates further driver calls. The documented IP
 * has no complete peripheral-disable or clock-gate bit. */
c2_status_t c2_hp_uart_disable(c2_hp_uart_t *uart);

/* Reading status reads LSR once; hardware also uses an LSR read to clear
 * interrupt-pending state. */
c2_status_t c2_hp_uart_get_status(c2_hp_uart_t *uart, uint32_t *status);
c2_status_t c2_hp_uart_set_irq_mask(c2_hp_uart_t *uart,
                                    uint32_t irq_enable_mask);

/* Flush kind distinguishes RX and TX. The driver writes a set pulse followed
 * by a clear value, so RF_CLR/TF_CLR are never left asserted and the receive
 * trigger level is retained. */
c2_status_t c2_hp_uart_flush(c2_hp_uart_t *uart,
                             c2_hp_uart_flush_t kind);

/* Nonblocking calls make one LSR poll and return C2_ERROR_BUSY for TX-full or
 * RX-empty. Blocking poll_limit is a total LSR-read budget, not microseconds;
 * zero means no status read and immediate C2_ERROR_TIMEOUT. Buffer calls share
 * one budget across the whole transfer (0..UINT32_MAX), report partial count,
 * and return C2_ERROR_TIMEOUT if the budget expires before completion. */
c2_status_t c2_hp_uart_write_byte_nonblocking(c2_hp_uart_t *uart,
                                              uint8_t byte);
c2_status_t c2_hp_uart_write_byte(c2_hp_uart_t *uart,
                                  uint8_t byte,
                                  uint32_t poll_limit);
/* Write calls only enqueue into the TX FIFO. This separate wait polls TEMT
 * (FIFO and shift register both empty) for physical transmission completion.
 * poll_limit has the same finite LSR-read budget and zero semantics above. */
c2_status_t c2_hp_uart_wait_tx_complete(c2_hp_uart_t *uart,
                                        uint32_t poll_limit);
c2_status_t c2_hp_uart_write(c2_hp_uart_t *uart,
                             const uint8_t *data,
                             size_t length,
                             size_t *written,
                             uint32_t poll_limit);
c2_status_t c2_hp_uart_read_byte_nonblocking(c2_hp_uart_t *uart,
                                             uint8_t *byte);
c2_status_t c2_hp_uart_read_byte(c2_hp_uart_t *uart,
                                 uint8_t *byte,
                                 uint32_t poll_limit);
c2_status_t c2_hp_uart_read(c2_hp_uart_t *uart,
                            uint8_t *data,
                            size_t length,
                            size_t *read_count,
                            uint32_t poll_limit);

#ifdef __cplusplus
}
#endif

#endif /* STARRYSKY_C2_UART_H */
