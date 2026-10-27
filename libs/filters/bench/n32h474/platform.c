/**
 * @file platform.c
 * @brief Bare-metal N32H474RC benchmark platform; only GPIO marker outputs.
 */
#include "bench_platform.h"
#include "bench_result.h"
#include "n32h47x_48x.h"

#if !defined(BENCH_FORMAT) || BENCH_FORMAT < 0 || BENCH_FORMAT > 3
#error "BENCH_FORMAT must be 0=f32, 1=f64, 2=q15 or 3=q31"
#endif

volatile bench_results_t g_bench_results;

/**
 * @brief Configure one analyzer pin without touching other pins on its port.
 *
 * The output latch is cleared before enabling push-pull output. The analyzer
 * input is the only permitted load; no timer alternate function is selected.
 */
static void marker_pin_init(GPIO_Module *gpio, uint32_t pin)
{
    const uint32_t bit = UINT32_C(1) << pin;
    const uint32_t pair = UINT32_C(3) << (pin * 2u);
    gpio->PBC = bit;
    gpio->POTYPE &= ~bit;
    gpio->PUPD &= ~pair;
    gpio->DS &= ~pair; /* Vendor GPIO_DS_2mA encoding. */
    gpio->SR &= ~bit;  /* Vendor GPIO_SR_FAST_SLEW encoding. */
    gpio->AFSEL[pin / 4u] &= ~(GPIO_AFSEL0_AFSEL0 << ((pin % 4u) * 8u));
    gpio->PMODE = (gpio->PMODE & ~pair) | (GPIO_PMODE_OUTPUT << (pin * 2u));
}

void bench_platform_init(void)
{
    /* Reset_Handler masks IRQs while RAM and clock state are initialized. */
    SysTick->CTRL = 0u;
    SysTick->LOAD = 0u;
    SysTick->VAL = 0u;
    for (uint32_t bank = 0u; bank < 4u; ++bank) {
        NVIC->ICER[bank] = UINT32_MAX;
        NVIC->ICPR[bank] = UINT32_MAX;
    }
    SCB->ICSR = SCB_ICSR_PENDSTCLR_Msk | SCB_ICSR_PENDSVCLR_Msk;

    /* Unmodified vendor HSI/PLL path: 8 MHz * 30, Flash latency 5,
     * I-cache enabled/prefetch disabled, APB1/APB2 divide by 2. Its startup
     * voltage sequence selects PVD 1.78 V and waits 1 ms with DWT. */
    SystemInit();
    SystemCoreClockUpdate();

#if BENCH_FORMAT < 2
    SCB->CPACR = UINT32_C(0x00f00000);
#else
    SCB->CPACR = 0u;
#endif
    __DSB();
    __ISB();

    RCC->AHB1PCLKEN |= RCC_AHB1PCLKEN_IOPAEN | RCC_AHB1PCLKEN_IOPBEN |
                      RCC_AHB1PCLKEN_IOPCEN;
    (void)RCC->AHB1PCLKEN;
    for (uint32_t pin = 8u; pin <= 11u; ++pin) marker_pin_init(GPIOA, pin);
    for (uint32_t pin = 12u; pin <= 15u; ++pin) marker_pin_init(GPIOB, pin);
    for (uint32_t pin = 6u; pin <= 9u; ++pin) marker_pin_init(GPIOC, pin);

    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CTRL = 0u;
    DWT->CYCCNT = 0u;
    DWT->CTRL = DWT_CTRL_CYCCNTENA_Msk;
    /* Make NOCP a UsageFault rather than relying only on escalation. Both
     * UsageFault and HardFault vectors preserve CFSR/HFSR in the mailbox. */
    SCB->SHCSR |= SCB_SHCSR_USGFAULTENA_Msk | SCB_SHCSR_BUSFAULTENA_Msk |
                  SCB_SHCSR_MEMFAULTENA_Msk;
    __DSB();
    __ISB();
    __enable_irq(); /* NVIC and SysTick sources remain disabled. */
}

__attribute__((noinline)) uint32_t bench_cycles(void)
{
    return DWT->CYCCNT;
}

void bench_marker(uint32_t channel, uint32_t high)
{
    static GPIO_Module *const ports[12] = {
        GPIOA, GPIOA, GPIOA, GPIOA, GPIOB, GPIOB,
        GPIOB, GPIOB, GPIOC, GPIOC, GPIOC, GPIOC
    };
    static const uint8_t pins[12] = {8, 9, 10, 11, 12, 13, 14, 15, 8, 9, 6, 7};
    const uint32_t bit = UINT32_C(1) << pins[channel];
    if (high != 0u) ports[channel]->PBSC = bit;
    else ports[channel]->PBC = bit;
    __DSB();
}

uint32_t bench_clock_hz(void)
{
    return SystemCoreClock;
}

uint32_t bench_cpacr(void)
{
    return SCB->CPACR;
}

/**
 * @brief Preserve hardware fault status and stop; recovery requires a reset.
 *
 * All exception vectors except Reset converge here. This translation unit is
 * compiled with -mgeneral-regs-only, so NOCP reporting cannot itself use VFP.
 */
__attribute__((noreturn)) void bench_fault(void)
{
    __disable_irq();
    g_bench_results.fault_cfsr = SCB->CFSR;
    g_bench_results.fault_hfsr = SCB->HFSR;
    g_bench_results.cpacr = SCB->CPACR;
    __DMB();
    g_bench_results.phase = 4u;
    __DSB();
    for (;;) __NOP();
}

/* newlib constructor entry points; no syscalls, UART or semihosting. */
void _init(void) {}
void _fini(void) {}

#ifdef BENCH_PLATFORM_ONLY
/* Explicit boot/command smoke image, never a filter measurement. */
void bench_run(void)
{
    g_bench_results.case_count = 0u;
    g_bench_results.phase = 3u;
}
#endif

int main(void)
{
    bench_platform_init();
    g_bench_results.magic = BENCH_MAGIC;
    g_bench_results.version = 1u;
    g_bench_results.format = BENCH_FORMAT;
    g_bench_results.core_hz = bench_clock_hz();
    g_bench_results.cpacr = bench_cpacr();
    __DMB();
    g_bench_results.phase = 1u;
    for (;;) {
        if (g_bench_results.command == 1u) {
            g_bench_results.command = 0u;
            __DMB();
            g_bench_results.phase = 2u;
            bench_run(); /* The workload publishes complete/failed. */
        }
    }
}
