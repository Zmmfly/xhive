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

/** @brief Select which HP_UART FIFO contents to discard, not drain. */
typedef enum {
    C2_HP_UART_FLUSH_RX = 1,
    C2_HP_UART_FLUSH_TX = 2,
    C2_HP_UART_FLUSH_RX_TX = 3
} c2_hp_uart_flush_t;

typedef struct {
    C2_SYS_UART_TypeDef *regs;
    bool initialized;
} c2_sys_uart_t;

/** @brief HP_UART line format and FIFO interrupt configuration. */
typedef struct {
    uint32_t baud_rate;         /**< Requested bits/s; divider uses integer truncation. */
    uint32_t irq_enable_mask;   /**< C2_HP_UART_IRQ_* bits; use zero for polling. */
    uint8_t data_bits;          /**< Data bits per character, from 5 through 8. */
    c2_uart_parity_t parity;    /**< None, odd, or even parity. */
    c2_uart_stop_bits_t stop_bits; /**< One or two stop bits. */
    c2_hp_uart_rx_trigger_t rx_trigger; /**< RX FIFO interrupt threshold encoding. */
} c2_hp_uart_config_t;

/**
 * @brief Caller-owned HP_UART state and control-register shadows.
 *
 * Use one context per peripheral and serialize all access, including ISR and
 * direct register access. No internal lock, software FIFO, ISR, or recovery
 * task is installed. Keep irq_enable_mask zero for polling-only use: every
 * LSR read, including a TX poll, also clears hardware interrupt-pending state.
 */
typedef struct {
    C2_HP_UART_TypeDef *regs;   /**< Register block retained until reinitialization. */
    uint32_t lcr_shadow;       /**< Driver-owned line format and IRQ enables. */
    uint32_t fcr_trigger_shadow; /**< RX trigger bits; FIFO clear bits remain zero. */
    bool initialized;         /**< Software gate, not a hardware enable state. */
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

/**
 * @brief Initialize HP_UART and discard both FIFO contents.
 *
 * @param[out] uart Exclusively owned context; must not be NULL. No prior
 *                  initialization is required. Unchanged on failure.
 * @param[in] regs Accessible HP_UART register block, normally C2_UART1; must
 *                 not be NULL and must remain accessible while uart is used.
 * @param[in] clock_hz Actual peripheral clock in Hz; must be nonzero.
 * @param[in] config Configuration copied during this call; must not be NULL.
 * @return c2_status_t Operation status.
 * @retval C2_OK Configuration writes completed; uart is initialized.
 * @retval C2_ERROR_INVALID_ARGUMENT A pointer, format, IRQ mask, or divider
 *                                  is invalid; no MMIO access is performed.
 *
 * @pre Quiesce both endpoints before initialization or reinitialization. Drain
 *      TX explicitly if its data must be retained, and stop the peer's TX.
 * @note floor(clock_hz / baud_rate) must be 3..65536; DIV is that value minus
 *       one. No baud-error tolerance check or clock reconfiguration is done.
 * @warning This call does not wait for TX/RX idle. It changes the line format
 *          and discards queued data. Nonzero IRQ enables require a caller-
 *          installed interrupt path; use zero for polling-only operation.
 */
c2_status_t c2_hp_uart_init(c2_hp_uart_t *uart,
                            C2_HP_UART_TypeDef *regs,
                            uint32_t clock_hz,
                            const c2_hp_uart_config_t *config);

/**
 * @brief Reconfigure an initialized HP_UART and discard both FIFO contents.
 *
 * @param[in,out] uart Initialized, exclusively owned context; must not be NULL.
 * @param[in] clock_hz Actual peripheral clock in Hz; must be nonzero.
 * @param[in] config Configuration copied during this call; must not be NULL.
 * @return c2_status_t Operation status.
 * @retval C2_OK Configuration writes completed and shadows updated.
 * @retval C2_ERROR_INVALID_ARGUMENT Invalid arguments; no MMIO or state change.
 * @retval C2_ERROR_NOT_INITIALIZED uart is not initialized or has no registers.
 *
 * @pre Quiesce both endpoints as required by c2_hp_uart_init(). This is not a
 *      lossless live reconfiguration; no idle check or implicit wait is made.
 * @note Divider and IRQ constraints are the same as c2_hp_uart_init().
 */
c2_status_t c2_hp_uart_configure(c2_hp_uart_t *uart,
                                 uint32_t clock_hz,
                                 const c2_hp_uart_config_t *config);

/**
 * @brief Mask peripheral interrupts and close the software driver gate.
 *
 * @param[in,out] uart Initialized, exclusively owned context; must not be NULL.
 * @return c2_status_t Operation status.
 * @retval C2_OK IRQ enables cleared and uart marked uninitialized.
 * @retval C2_ERROR_INVALID_ARGUMENT uart is NULL.
 * @retval C2_ERROR_NOT_INITIALIZED uart is not initialized or has no registers.
 *
 * @warning Does not stop the hardware, wait for TX, discard FIFO data, or clear
 *          pending interrupts. Already queued TX and incoming RX may continue.
 */
c2_status_t c2_hp_uart_disable(c2_hp_uart_t *uart);

/**
 * @brief Read one masked LSR snapshot, acknowledging pending interrupts.
 *
 * @param[in] uart Initialized, exclusively owned context; must not be NULL.
 * @param[out] status C2_HP_UART_STATUS_* bitmap; must not be NULL. Unchanged
 *                    on failure. The snapshot describes the pre-clear state.
 * @return c2_status_t Operation status.
 * @retval C2_OK LSR read once and its documented bits returned.
 * @retval C2_ERROR_INVALID_ARGUMENT uart or status is NULL.
 * @retval C2_ERROR_NOT_INITIALIZED uart is not initialized or has no registers.
 *
 * @warning LSR reads clear interrupt-pending state, even outside an ISR.
 */
c2_status_t c2_hp_uart_get_status(c2_hp_uart_t *uart, uint32_t *status);

/**
 * @brief Change peripheral IRQ enables without changing the line format.
 *
 * @param[in,out] uart Initialized, exclusively owned context; must not be NULL.
 * @param[in] irq_enable_mask OR of C2_HP_UART_IRQ_* bits; zero masks all IRQs.
 * @return c2_status_t Operation status.
 * @retval C2_OK LCR and its shadow updated; no FIFO or LSR access performed.
 * @retval C2_ERROR_INVALID_ARGUMENT uart is NULL or the mask has unknown bits.
 * @retval C2_ERROR_NOT_INITIALIZED uart is not initialized or has no registers.
 *
 * @pre Install and coordinate the CPU interrupt path before enabling IRQs.
 * @note No ISR is installed and no pending state is cleared by this call.
 *       Polling I/O also reads LSR and can acknowledge other IRQ sources.
 */
c2_status_t c2_hp_uart_set_irq_mask(c2_hp_uart_t *uart,
                                    uint32_t irq_enable_mask);

/**
 * @brief Discard selected FIFO contents while retaining the RX trigger level.
 *
 * @param[in] uart Initialized, exclusively owned context; must not be NULL.
 * @param[in] kind RX, TX, or both FIFOs; zero is invalid.
 * @return c2_status_t Operation status.
 * @retval C2_OK FIFO clear bits asserted then released by two FCR writes.
 * @retval C2_ERROR_INVALID_ARGUMENT uart is NULL or kind is invalid.
 * @retval C2_ERROR_NOT_INITIALIZED uart is not initialized or has no registers.
 *
 * @warning This is a destructive discard, not a TX drain or a line reset.
 *          It does not wait for or abort a character already being shifted.
 *          Coordinate with the peer when discarding part of a protocol frame.
 */
c2_status_t c2_hp_uart_flush(c2_hp_uart_t *uart,
                             c2_hp_uart_flush_t kind);

/**
 * @brief Try to enqueue one byte after a single LSR read.
 *
 * @param[in] uart Initialized, exclusively owned context; must not be NULL.
 * @param[in] byte Byte to enqueue; the configured data width controls TX bits.
 * @return c2_status_t Operation status.
 * @retval C2_OK Byte enqueued, not necessarily transmitted on the wire.
 * @retval C2_ERROR_BUSY TX FIFO is full; no data written.
 * @retval C2_ERROR_INVALID_ARGUMENT uart is NULL.
 * @retval C2_ERROR_NOT_INITIALIZED uart is not initialized or has no registers.
 *
 * @note The LSR read also acknowledges pending interrupts.
 */
c2_status_t c2_hp_uart_write_byte_nonblocking(c2_hp_uart_t *uart,
                                              uint8_t byte);

/**
 * @brief Poll for TX FIFO space and enqueue one byte.
 *
 * @param[in] uart Initialized, exclusively owned context; must not be NULL.
 * @param[in] byte Byte to enqueue; the configured data width controls TX bits.
 * @param[in] poll_limit Maximum LSR reads, 0..UINT32_MAX, not a time unit.
 *                       Zero performs no MMIO and immediately times out.
 * @return c2_status_t Operation status.
 * @retval C2_OK Byte enqueued, not necessarily transmitted on the wire.
 * @retval C2_ERROR_TIMEOUT Budget exhausted; no data written.
 * @retval C2_ERROR_INVALID_ARGUMENT uart is NULL.
 * @retval C2_ERROR_NOT_INITIALIZED uart is not initialized or has no registers.
 *
 * @note Each LSR read also acknowledges pending interrupts.
 */
c2_status_t c2_hp_uart_write_byte(c2_hp_uart_t *uart,
                                  uint8_t byte,
                                  uint32_t poll_limit);

/**
 * @brief Poll TEMT until both the TX FIFO and transmitter are empty.
 *
 * @param[in] uart Initialized, exclusively owned context; must not be NULL.
 * @param[in] poll_limit Maximum LSR reads, 0..UINT32_MAX, not a time unit.
 *                       Zero performs no MMIO and immediately times out.
 * @return c2_status_t Operation status.
 * @retval C2_OK TEMT observed; local transmission completed, not a peer ACK.
 * @retval C2_ERROR_TIMEOUT Budget exhausted; queued transmission may continue.
 * @retval C2_ERROR_INVALID_ARGUMENT uart is NULL.
 * @retval C2_ERROR_NOT_INITIALIZED uart is not initialized or has no registers.
 *
 * @pre No other writer may enqueue data while completion is being checked.
 * @note Only reads LSR, acknowledging pending IRQs; never clears the TX FIFO.
 */
c2_status_t c2_hp_uart_wait_tx_complete(c2_hp_uart_t *uart,
                                        uint32_t poll_limit);

/**
 * @brief Enqueue a buffer using one shared polling budget.
 *
 * @param[in] uart Initialized, exclusively owned context; must not be NULL.
 * @param[in] data Buffer of at least length bytes, valid throughout this call;
 *                 may be NULL only when length is zero.
 * @param[in] length Bytes to enqueue; zero succeeds without MMIO access.
 * @param[out] written Enqueued byte count; must not be NULL. Unchanged if
 *                     validation fails, otherwise set even on timeout.
 * @param[in] poll_limit Total LSR reads, 0..UINT32_MAX, not a time unit. Zero
 *                       times out without MMIO unless length is zero.
 * @return c2_status_t Operation status.
 * @retval C2_OK All bytes enqueued, or length is zero; not a TX-complete wait.
 * @retval C2_ERROR_TIMEOUT Budget exhausted before all bytes were enqueued.
 * @retval C2_ERROR_INVALID_ARGUMENT Invalid pointer arguments.
 * @retval C2_ERROR_NOT_INITIALIZED uart is not initialized or has no registers.
 *
 * @note LSR reads acknowledge pending IRQs. No buffer pointer is retained.
 * @warning Timeout does not cancel enqueued data. Use written to avoid sending
 *          the accepted prefix twice; retry/recovery is the caller's policy.
 */
c2_status_t c2_hp_uart_write(c2_hp_uart_t *uart,
                             const uint8_t *data,
                             size_t length,
                             size_t *written,
                             uint32_t poll_limit);

/**
 * @brief Try to dequeue one byte after a single LSR read.
 *
 * @param[in] uart Initialized, exclusively owned context; must not be NULL.
 * @param[out] byte Destination; must not be NULL. Written on C2_OK or
 *                  C2_ERROR_IO only, and unchanged on other errors.
 * @return c2_status_t Operation status.
 * @retval C2_OK One byte dequeued without a reported parity error.
 * @retval C2_ERROR_IO One byte dequeued and stored, but its parity is invalid.
 * @retval C2_ERROR_BUSY RX FIFO is empty; TRX is not read.
 * @retval C2_ERROR_INVALID_ARGUMENT uart or byte is NULL.
 * @retval C2_ERROR_NOT_INITIALIZED uart is not initialized or has no registers.
 *
 * @note LSR acknowledges pending IRQs; reading TRX consumes the byte even on
 *       parity failure. No automatic retry or FIFO discard is performed.
 */
c2_status_t c2_hp_uart_read_byte_nonblocking(c2_hp_uart_t *uart,
                                             uint8_t *byte);

/**
 * @brief Poll RX FIFO availability and dequeue one byte.
 *
 * @param[in] uart Initialized, exclusively owned context; must not be NULL.
 * @param[out] byte Destination; must not be NULL. Written on C2_OK or
 *                  C2_ERROR_IO only, and unchanged on other errors.
 * @param[in] poll_limit Maximum LSR reads, 0..UINT32_MAX, not a time unit.
 *                       Zero performs no MMIO and immediately times out.
 * @return c2_status_t Operation status.
 * @retval C2_OK One byte dequeued without a reported parity error.
 * @retval C2_ERROR_IO One byte dequeued and stored, but its parity is invalid.
 * @retval C2_ERROR_TIMEOUT Budget exhausted; no byte consumed.
 * @retval C2_ERROR_INVALID_ARGUMENT uart or byte is NULL.
 * @retval C2_ERROR_NOT_INITIALIZED uart is not initialized or has no registers.
 *
 * @note LSR reads acknowledge pending IRQs. No retry or FIFO discard follows
 *       a parity failure; handling the consumed byte belongs to the caller.
 */
c2_status_t c2_hp_uart_read_byte(c2_hp_uart_t *uart,
                                 uint8_t *byte,
                                 uint32_t poll_limit);

/**
 * @brief Dequeue a fixed byte count using one shared polling budget.
 *
 * @param[in] uart Initialized, exclusively owned context; must not be NULL.
 * @param[out] data Buffer for at least length bytes, valid throughout this
 *                 call; may be NULL only when length is zero. Bytes beyond
 *                 read_count are unchanged.
 * @param[in] length Bytes requested; zero succeeds without MMIO access.
 * @param[out] read_count Bytes consumed and stored, including a parity-failed
 *                       byte; must not be NULL. Unchanged if validation fails.
 * @param[in] poll_limit Total LSR reads, 0..UINT32_MAX, not a time unit. Zero
 *                       times out without MMIO unless length is zero.
 * @return c2_status_t Operation status.
 * @retval C2_OK All requested bytes read, or length is zero.
 * @retval C2_ERROR_IO Stops at the first parity failure. read_count includes
 *                    this byte at data[read_count - 1], which is not trusted.
 * @retval C2_ERROR_TIMEOUT Budget exhausted; read_count reports the prefix read.
 * @retval C2_ERROR_INVALID_ARGUMENT Invalid pointer arguments; outputs unchanged.
 * @retval C2_ERROR_NOT_INITIALIZED uart is not initialized or has no registers.
 *
 * @note LSR reads acknowledge pending IRQs. This is not protocol-frame or idle
 *       detection. No buffer is retained, and no automatic recovery is done.
 */
c2_status_t c2_hp_uart_read(c2_hp_uart_t *uart,
                            uint8_t *data,
                            size_t length,
                            size_t *read_count,
                            uint32_t poll_limit);

#ifdef __cplusplus
}
#endif

#endif /* STARRYSKY_C2_UART_H */
