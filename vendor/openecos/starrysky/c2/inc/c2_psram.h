/* StarrySky C2 external PSRAM controller and cold-start initialization.
 * Sources: StarrySkyC2 start.S and platform_psram.c, cross-checked against
 * works/docs/periphs/psram.md. WC/CHD are observed whole-word tuning writes;
 * their effective field widths and reset values are not published.
 */
#ifndef STARRYSKY_C2_PSRAM_H
#define STARRYSKY_C2_PSRAM_H

#include "c2_common.h"
#include "c2_gpio.h"
#include "c2_qspi.h"
#include "c2_register.h"

typedef union { uint32_t WORD; struct { uint32_t WC : 32; } BITS; } C2_PSRAM_WC_TypeDef;
typedef union { uint32_t WORD; struct { uint32_t CHD : 32; } BITS; } C2_PSRAM_CHD_TypeDef;
C2_REG_ASSERT_SIZE(C2_PSRAM_WC_TypeDef, 4);
C2_REG_ASSERT_SIZE(C2_PSRAM_CHD_TypeDef, 4);

typedef struct {
    volatile C2_PSRAM_WC_TypeDef WC;   /* 0x00: observed wait-cycle tuning. */
    volatile C2_PSRAM_CHD_TypeDef CHD; /* 0x04: observed hold-delay tuning. */
} C2_PSRAM_TypeDef;
C2_REG_ASSERT_SIZE(C2_PSRAM_TypeDef, 8);

#define C2_PSRAM0_BASE UINT32_C(0x10004000)
#define C2_PSRAM0 ((C2_PSRAM_TypeDef *)(uintptr_t)C2_PSRAM0_BASE)
#define C2_PSRAM_MAPPED_BASE UINT32_C(0x40000000)
#define C2_PSRAM_MAPPED_SIZE (UINT32_C(8) * UINT32_C(1024) * UINT32_C(1024))
#define C2_PSRAM_QPI_MODE_GPIO_MASK (UINT32_C(1) << 15)

#define C2_PSRAM_INIT_WC UINT32_C(18)
#define C2_PSRAM_INIT_CHD UINT32_C(4)
#define C2_PSRAM_RUN_WC UINT32_C(8)
#define C2_PSRAM_RUN_CHD UINT32_C(0)

typedef enum {
    /* Optional board/device guard before command 0x66. */
    C2_PSRAM_DELAY_BEFORE_RESET_ENABLE = 0,
    /* Guard after completed 0x66 and before 0x99. */
    C2_PSRAM_DELAY_AFTER_RESET_ENABLE,
    /* Device reset recovery after completed 0x99 and before 0x35. */
    C2_PSRAM_DELAY_AFTER_DEVICE_RESET,
    /* QPI settle guard after completed 0x35 and before GPIO bit15 is raised. */
    C2_PSRAM_DELAY_AFTER_ENTER_QPI
} c2_psram_delay_stage_t;

/* The evidence does not identify the PSRAM part or publish delay minima. A
 * board may supply a bounded callback implementing its datasheet timing. It
 * must use only already-available code/data/stack, must not re-enter or modify
 * the QSPI/PSRAM/GPIO contexts, and returns C2_OK on success. NULL means
 * reproduce the official sequence with no explicit software delay;
 * that is not a claim that every replacement PSRAM part needs no delay. */
typedef c2_status_t (*c2_psram_delay_fn)(void *context,
                                         c2_psram_delay_stage_t stage);

#ifdef __cplusplus
extern "C" {
#endif

/* Initialize the mapped 8 MiB PSRAM without reading/writing mapped memory.
 *
 * Exact observed QSPI/PSRAM values and order are preserved. GPIO bit15 is
 * safely adapted to the caller's authoritative c2_gpio_t shadow: it is first
 * latched low and made output, then raised only after 0x66/0x99/0x35 succeed.
 * Other GPIO shadow bits are preserved. gpio must be the one exclusive GPIO0
 * context; if bit15 is already high the call returns C2_ERROR_BUSY without
 * MMIO, preventing silent runtime reinitialization.
 *
 * poll_limit counts total QSPI STATUS reads shared by all three raw commands,
 * not microseconds. Values below 3 (including zero) return
 * C2_ERROR_INVALID_ARGUMENT before any MMIO because three successful commands
 * require at least one read each. Before starting each new command the driver
 * checks that at least one read remains, so an exhausted budget cannot launch
 * an unobservable command.
 *
 * On timeout/callback/GPIO failure, bit15 is never raised by this function and
 * run tuning WC=8/CHD=0 is not written. Earlier raw commands may already have
 * partially changed the controller/device; failure does not imply rollback or
 * safe immediate retry.
 *
 * EARLY-BOOT EXECUTION CONTRACT: caller must exclusively own QSPI/PSRAM/GPIO0;
 * code, callback, context, constants and stack must be in already usable
 * storage (normally SRAM/ROM). Do not call with a PSRAM stack/global, while
 * executing through Flash XIP on the same QSPI path, or while another master,
 * ISR or DMA user can access QSPI/PSRAM. No ramfunc/linker placement is imposed
 * here because this library cannot know the project's linker script. Rebuild
 * dependent GPIO/QSPI software state after any separate destructive boot
 * sequence. Success is required before any access to C2_PSRAM_MAPPED_BASE.
 */
c2_status_t c2_psram_init(C2_PSRAM_TypeDef *psram,
                          C2_QSPI_TypeDef *qspi,
                          c2_gpio_t *gpio,
                          uint32_t poll_limit,
                          c2_psram_delay_fn delay,
                          void *delay_context);

#ifdef __cplusplus
}
#endif

#endif /* STARRYSKY_C2_PSRAM_H */
