#include <stddef.h>
#include <stdint.h>
#include <xmcu_config.h>

/* Highest address of stack */
extern uint32_t _estack[];

/* Start address for the initialization values of .data section */
extern uint32_t _sidata[];

/* Start address for the .data section */
extern uint32_t _sdata[];

/* End address for the .data section */
extern uint32_t _edata[];

/* Start address for the .bss section */
extern uint32_t _sbss[];

/* End address for the .bss section */
extern uint32_t _ebss[];

extern void __libc_init_array(void);
extern void __libc_fini_array(void);

/* Vector table type */
typedef void (*pfunc_t)(void);

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
const pfunc_t g_pfnVectors[] = {
    (pfunc_t)(void *)(&_estack),   /* Initial stack pointer */
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
    uint32_t *src = _sidata;
    uint32_t *dest = _sdata;
    /* Copy the data segment initializers from flash to SRAM */
    while (dest < _edata) *dest++ = *src++;
}

static void fill_zeros(void)
{
    uint32_t *src = _sbss;
    /* Zero fill the bss segment. */
    while (src < _ebss) *src++ = 0;
}

__attribute__((weak))
void SystemInit(void)
{
    /* Empty implementation */
}

extern int main(void);

__attribute__((noreturn))
void Reset_Handler(void)
{
    SystemInit();
    init_datas();
    fill_zeros();
    __libc_init_array();
    main();
    __libc_fini_array();
    while (1);
}
