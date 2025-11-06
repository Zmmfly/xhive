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
/* Peripheral Interrupts begin */
/* Peripheral Interrupts end */
};

void Default_Handler(void)
{
    while(1);
}

static void init_datas(void)
{
    /* Copy the data segment initializers from flash to SRAM */
    uint32_t *src = &_sidata;
    uint32_t *dst = &_sdata;
    while(dst < &_edata) *dst++ = *src++;
}

static void fill_zeros(void)
{
    /* Zero fill the bss segment. */
    uint32_t *dst = &_sbss;
    while(dst < &_ebss) *dst++ = 0;
}

__attribute__((weak))
void SystemInit(void)
{
    /* Empty implementation */
}

extern int main(void);

#if 1
__attribute__((noreturn))
void Reset_Handler(void)
{
    /* Call SystemInit first to speed up initialization */
    SystemInit();

    /* Initialize data and bss */
    init_datas();
    fill_zeros();

    /* Call C library initialization */
    __libc_init_array();

    /* Call main function */
    main();

    /* Call C library cleanup */
    __libc_fini_array();

    /* Infinite loop */
    while (1);
}
#else
__attribute__((noreturn, naked))
void Reset_Handler(void)
{
    __asm volatile(
    "   ldr r0, =_estack                \n"  /* Load stack pointer address */
    "   mov sp, r0                      \n"  /* Set stack pointer */

    /* Copy the data segment initializers from flash to SRAM */
    "   ldr r0, =_sdata                 \n"  /* Destination address (SRAM) */
    "   ldr r1, =_edata                 \n"  /* End address */
    "   ldr r2, =_sidata                \n"  /* Source address (flash) */
    "   movs r3, #0                     \n"  /* Clear r3 for offset */
    "   b LoopCopyDataInit              \n"  /* Branch to loop */
    "CopyDataInit:                      \n"
    "   ldr r4, [r2, r3]                \n"  /* Load from source */
    "   str r4, [r0, r3]                \n"  /* Store to destination */
    "   adds r3, r3, #4                 \n"  /* Increment offset */
    "LoopCopyDataInit:                  \n"
    "   adds r4, r0, r3                 \n"  /* Current dest address */
    "   cmp r4, r1                      \n"  /* Compare with end */
    "   bcc CopyDataInit                \n"  /* Branch if not done */

    /* Zero fill the bss segment */
    "   ldr r2, =_sbss                  \n"  /* Start address of bss */
    "   ldr r4, =_ebss                  \n"  /* End address of bss */
    "   movs r3, #0                     \n"  /* Clear r3 for offset */
    "   b LoopZeroBSS                   \n"  /* Branch to loop */
    "FillZeroBSS:                       \n"
    "   str r3, [r2]                    \n"  /* Store zero */
    "   adds r2, r2, #4                 \n"  /* Increment address */
    "LoopZeroBSS:                       \n"
    "   cmp r2, r4                      \n"  /* Compare with end */
    "   bcc FillZeroBSS                 \n"  /* Branch if not done */

    /* Call the clock system initialization function */
    "   bl SystemInit                   \n"

    /* Call C library initialization */
    "   bl __libc_init_array            \n"  /* Initialize C library */

    /* Call main function */
    "   bl main                         \n"  /* Branch with link to main */

    /* Call C library cleanup */
    "   bl __libc_fini_array            \n"  /* Cleanup C library */

    /* Infinite loop */
    "LoopForever:                       \n"
    "   b LoopForever                   \n"  /* Branch to self */
    );
}
#endif
