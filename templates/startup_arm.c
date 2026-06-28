/*
Copyright (c) 2025 Zmmfly. All rights reserved.

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <https://www.gnu.org/licenses/>.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
1. Redistributions of source code must retain the above copyright notice,
   this list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.
3. Neither the name of the copyright holder nor the names of its contributors
   may be used to endorse or promote products derived from this software
   without specific prior written permission.
*/

#include <stddef.h>
#include <stdint.h>
#include <xhive_config.h>

/* Highest address of stack */
extern uint32_t _estack;

/* Start address for the initialization values of .data section */
extern uint32_t _sidata;

/* Start address for the .data section */
extern uint32_t _sdata;

/* End address for the .data section */
extern uint32_t _edata;

/* Start address for the .bss section */
extern uint32_t _sbss;

/* End address for the .bss section */
extern uint32_t _ebss;

/*
 * Optional linker-generated initialization tables.
 *
 * copy table entry layout:
 *   [0] source address in ROM
 *   [1] destination runtime address
 *   [2] byte count
 *
 * zero table entry layout:
 *   [0] destination runtime address
 *   [1] byte count
 *
 * The symbols are weak so a custom linker script without these tables can
 * still use the legacy single .data/.bss initialization path below.
 */
extern const uintptr_t __copy_table_start__[] __attribute__((weak));
extern const uintptr_t __copy_table_end__[] __attribute__((weak));
extern const uintptr_t __zero_table_start__[] __attribute__((weak));
extern const uintptr_t __zero_table_end__[] __attribute__((weak));

/*
 * External RAM is often not writable immediately after reset. Board code
 * normally enables the external memory controller in SystemInit(). When
 * external RAM is configured, the startup code can initialize internal and
 * external ranges in separate passes.
 */
#if defined(CONFIG_ENABLE_EXT_RAM) && CONFIG_ENABLE_EXT_RAM
#define XHIVE_HAS_EXT_RAM 1
#define XHIVE_INIT_ALL      (-1)
#define XHIVE_INIT_INTERNAL 0
#define XHIVE_INIT_EXTERNAL 1
#else
#define XHIVE_HAS_EXT_RAM 0
#define XHIVE_INIT_ALL 0
#endif

extern void __libc_init_array(void);
extern void __libc_fini_array(void);

void Default_Handler(void);
void Reset_Handler(void);
#define WEAK_ALIAS __attribute__((weak, alias("Default_Handler")))

/* CPU Interrupt Handlers */

WEAK_ALIAS void NMI_Handler(void);
WEAK_ALIAS void HardFault_Handler(void);
#if !(defined(CONFIG_CPU_CORTEX_M0) || defined(CPU_CORTEX_M0PLUS) || defined(CPU_CORTEX_M1))
WEAK_ALIAS void MemManage_Handler(void);
WEAK_ALIAS void BusFault_Handler(void);
WEAK_ALIAS void UsageFault_Handler(void);
#endif
WEAK_ALIAS void SVC_Handler(void);
WEAK_ALIAS void DebugMon_Handler(void);
WEAK_ALIAS void PendSV_Handler(void);
WEAK_ALIAS void SysTick_Handler(void);

// DO NOT REMOVE THE COMMENTS BELOW, THEY ARE USED TO MARK THE PLACE TO INSERT PERIPHERAL INTERRUPT HANDLERS

/* Peripheral Interrupt Handlers begin */
/* Peripheral Interrupt Handlers end */

/* Vector table */
__attribute__((section(".isr_vector")))
const void* g_pfnVectors[] = {
    &_estack,   /* Initial stack pointer */
    Reset_Handler,                 /* Reset handler */
    NMI_Handler,                   /* NMI handler */
    HardFault_Handler,             /* Hard Fault handler */
#if defined(CONFIG_CPU_CORTEX_M0) || defined(CPU_CORTEX_M0PLUS) || defined(CPU_CORTEX_M1)
    0,                             /* Reserved */
    0,                             /* Reserved */
    0,                             /* Reserved */
#else
    MemManage_Handler,             /* MPU Fault handler */
    BusFault_Handler,              /* Bus Fault handler */
    UsageFault_Handler,            /* Usage Fault handler */
#endif
    0,                             /* Reserved */
    0,                             /* Reserved */
    0,                             /* Reserved */
    0,                             /* Reserved */
    SVC_Handler,                   /* SVCall handler */
#if defined(CONFIG_CPU_CORTEX_M0) || defined(CPU_CORTEX_M0PLUS) || defined(CPU_CORTEX_M1)
    0,                             /* Reserved */
#else
    DebugMon_Handler,              /* Debug Monitor handler */
#endif
    0,                             /* Reserved */
    PendSV_Handler,                /* PendSV handler */
    SysTick_Handler,               /* SysTick handler */

// DO NOT REMOVE THE COMMENTS BELOW, THEY ARE USED TO MARK THE PLACE TO INSERT PERIPHERAL INTERRUPT VECTORS

/* Peripheral Interrupts begin */
/* Peripheral Interrupts end */
};

void Default_Handler(void)
{
    while(1);
}

static void copy_bytes(uint8_t *dst, const uint8_t *src, uintptr_t size)
{
    while (size--) *dst++ = *src++;
}

static void zero_bytes(uint8_t *dst, uintptr_t size)
{
    while (size--) *dst++ = 0;
}

#if XHIVE_HAS_EXT_RAM
/*
 * Treat any range that overlaps EXTRAM as external. This is intentionally
 * conservative: a mixed range must wait until SystemInit() has made EXTRAM
 * usable, otherwise the early copy/zero loop could fault on the external part.
 */
static int is_ext_ram_range(uintptr_t addr, uintptr_t size)
{
    uintptr_t ext_start = (uintptr_t)CONFIG_EXT_RAM_START;
    uintptr_t ext_end = ext_start + ((uintptr_t)CONFIG_EXT_RAM_LENGTH * 1024U);
    uintptr_t range_end;

    if (size == 0)
    {
        return 0;
    }

    range_end = addr + size;
    if (range_end < addr)
    {
        range_end = ~(uintptr_t)0;
    }

    return (addr < ext_end) && (range_end > ext_start);
}

/*
 * target selects which ranges this pass should touch:
 *   XHIVE_INIT_ALL      - initialize every table entry
 *   XHIVE_INIT_INTERNAL - skip entries whose destination is in EXTRAM
 *   XHIVE_INIT_EXTERNAL - initialize only entries whose destination is in EXTRAM
 */
static int should_init_range(uintptr_t addr, uintptr_t size, int target)
{
    int is_external;

    if (target == XHIVE_INIT_ALL)
    {
        return 1;
    }

    is_external = is_ext_ram_range(addr, size);
    return is_external == target;
}
#endif

static void init_datas_region(int target)
{
    /*
     * Prefer the linker table when present. It covers the normal .data
     * section plus optional sections such as .ram_text, .ram_data, .ext_text
     * and .ext_data, each with its own ROM load address and runtime address.
     */
    if (__copy_table_start__ && __copy_table_end__)
    {
        const uintptr_t *entry = __copy_table_start__;
        while (entry < __copy_table_end__)
        {
            const uint8_t *src = (const uint8_t *)entry[0];
            uint8_t *dst = (uint8_t *)entry[1];
            uintptr_t size = entry[2];
            entry += 3;

            #if XHIVE_HAS_EXT_RAM
            if (!should_init_range((uintptr_t)dst, size, target))
            {
                continue;
            }
            #else
            (void)target;
            #endif

            copy_bytes(dst, src, size);
        }
        return;
    }

    /*
     * Fallback for custom linker scripts that only expose the traditional
     * .data symbols. This keeps older bare-metal layouts working.
     */
    uintptr_t size = (uintptr_t)&_edata - (uintptr_t)&_sdata;
    uint8_t *dst = (uint8_t *)&_sdata;

    #if XHIVE_HAS_EXT_RAM
    if (!should_init_range((uintptr_t)dst, size, target))
    {
        return;
    }
    #else
    (void)target;
    #endif

    /* Copy the data segment initializers from flash to runtime memory. */
    copy_bytes(dst, (const uint8_t *)&_sidata, size);
}

static void init_datas(void)
{
    init_datas_region(XHIVE_INIT_ALL);
}

static void fill_zeros_region(int target)
{
    /*
     * Prefer the linker table when present. It covers .bss plus optional
     * zero-filled sections such as .ram_bss and .ext_bss.
     */
    if (__zero_table_start__ && __zero_table_end__)
    {
        const uintptr_t *entry = __zero_table_start__;
        while (entry < __zero_table_end__)
        {
            uint8_t *dst = (uint8_t *)entry[0];
            uintptr_t size = entry[1];
            entry += 2;

            #if XHIVE_HAS_EXT_RAM
            if (!should_init_range((uintptr_t)dst, size, target))
            {
                continue;
            }
            #else
            (void)target;
            #endif

            zero_bytes(dst, size);
        }
        return;
    }

    /*
     * Fallback for custom linker scripts that only expose the traditional
     * .bss symbols.
     */
    uintptr_t size = (uintptr_t)&_ebss - (uintptr_t)&_sbss;
    uint8_t *dst = (uint8_t *)&_sbss;

    #if XHIVE_HAS_EXT_RAM
    if (!should_init_range((uintptr_t)dst, size, target))
    {
        return;
    }
    #else
    (void)target;
    #endif

    /* Zero fill the bss segment. */
    zero_bytes(dst, size);
}

static void fill_zeros(void)
{
    fill_zeros_region(XHIVE_INIT_ALL);
}

__attribute__((weak))
void SystemInit(void)
{
    /* Empty implementation */
}

extern int main(void);

#if defined(CONFIG_THIRD_RTOS_RTTHREAD) && defined(CONFIG_RT_USING_USER_MAIN)

extern int entry(void);
#define USE_ENTRY

#endif // CONFIG_THIRD_RTOS_RTTHREAD && CONFIG_RT_USING_USER_MAIN

__attribute__((noreturn))
void Reset_Handler(void)
{
    #if XHIVE_HAS_EXT_RAM

    #ifdef CONFIG_ENABLE_FAST_STARTUP
    /*
     * Fast startup asks the board code to run before memory initialization.
     * This makes EXTRAM available early, so all copy/zero table entries can be
     * initialized in one pass.
     */
    SystemInit();

    /* Initialize data and bss */
    init_datas();
    fill_zeros();
    #else
    /*
     * Default bare-metal order keeps internal RAM initialization before
     * SystemInit(), but skips EXTRAM destinations because the external memory
     * controller may not be configured yet.
     */
    init_datas_region(XHIVE_INIT_INTERNAL);
    fill_zeros_region(XHIVE_INIT_INTERNAL);

    /*
     * Board code can enable clocks, pins and external memory controllers here.
     */
    SystemInit();

    /*
     * EXTRAM is expected to be writable after SystemInit(), so finish the
     * sections whose runtime addresses live in external RAM.
     */
    init_datas_region(XHIVE_INIT_EXTERNAL);
    fill_zeros_region(XHIVE_INIT_EXTERNAL);
    #endif

    #else

    #ifdef CONFIG_ENABLE_FAST_STARTUP
    /* Call SystemInit first to speed up initialization */
    SystemInit();
    #endif

    /* Initialize data and bss */
    init_datas();
    fill_zeros();

    #ifndef CONFIG_ENABLE_FAST_STARTUP
    /* Call SystemInit */
    SystemInit();
    #endif

    #endif

    #ifdef CONFIG_THIRD_RTOS_NONE
    /* Call C library initialization */
    __libc_init_array();
    #endif

    /* Call entry function */
    #if defined(USE_ENTRY)
    entry();
    #else
    main();
    #endif

    #ifdef CONFIG_THIRD_RTOS_NONE
    /* Call C library cleanup */
    __libc_fini_array();
    #endif

    /* Infinite loop, usually cannot be reached */
    while (1);
}
