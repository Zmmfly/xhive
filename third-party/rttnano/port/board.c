/*
 * Copyright (c) 2006-2019, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2021-05-24                  the first version
 */

#include <rthw.h>
#include <rtthread.h>
#include "xhive_config.h"

void* rt_bss_end()
{
    extern int __bss_end__;
    return (void*)&__bss_end__;
}

void* rt_ram_end()
{
    return (void*)(CONFIG_RAM_START + CONFIG_RAM_LENGTH*1024);
}

#if defined(RT_USING_USER_MAIN) && defined(RT_USING_HEAP)
/*
 * Please modify RT_HEAP_SIZE if you enable RT_USING_HEAP
 * the RT_HEAP_SIZE max value = (sram size - ZI size), 1024 means 1024 bytes
 */

#ifdef RT_HEAP_USING_STATIC
    #warning "Using static heap, please ensure the RT_HEAP_SIZE is proper!"
    #define RT_HEAP_SIZE (RT_HEAP_SIZE_KB * 1024)
    static rt_uint8_t rt_heap[RT_HEAP_SIZE];
#elif defined(RT_HEAP_USING_BSS_END)
    #warning "Using bss end to ram end as heap, please ensure the sram size is proper!"
#endif

RT_WEAK void *rt_heap_begin_get(void)
{
    #ifdef RT_HEAP_USING_STATIC
    return rt_heap;
    #elif RT_HEAP_USING_BSS_END
    return rt_bss_end();
    #else
    return RT_NULL;
    #endif
}

RT_WEAK void *rt_heap_end_get(void)
{
    #ifdef RT_HEAP_USING_STATIC
    return rt_heap + RT_HEAP_SIZE;
    #elif RT_HEAP_USING_BSS_END
    return rt_ram_end();
    #else
    return RT_NULL;
    #endif
}
#endif

void rt_os_tick_callback(void)
{
    rt_interrupt_enter();
    
    rt_tick_increase();

    rt_interrupt_leave();
}

RT_WEAK
void rt_systick_config(void)
{
    /* empty implementation */
}

/**
 * This function will initial your board.
 */
void rt_hw_board_init(void)
{
    /* 
     * TODO 1: OS Tick Configuration
     * Enable the hardware timer and call the rt_os_tick_callback function
     * periodically with the frequency RT_TICK_PER_SECOND. 
     */
    rt_systick_config();

    /* Call components board initial (use INIT_BOARD_EXPORT()) */
#ifdef RT_USING_COMPONENTS_INIT
    rt_components_board_init();
#endif

#if defined(RT_USING_USER_MAIN) && defined(RT_USING_HEAP)
    rt_system_heap_init(rt_heap_begin_get(), rt_heap_end_get());
#endif
}

#ifdef RT_USING_CONSOLE

RT_WEAK
void rt_hw_console_output(const char *str)
{
    // empty implementation
}

#endif

