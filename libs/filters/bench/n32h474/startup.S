/* N32H474RC standalone benchmark startup. No RTOS, SysTick or peripheral IRQs. */
.syntax unified
.cpu cortex-m4
.thumb

#if !defined(BENCH_FORMAT) || BENCH_FORMAT < 0 || BENCH_FORMAT > 3
#error "BENCH_FORMAT must be 0=f32, 1=f64, 2=q15 or 3=q31"
#endif

.section .isr_vector,"a",%progbits
.balign 1024
.global g_vectors
.type g_vectors,%object
g_vectors:
    .word __stack_top
    .word Reset_Handler
    .word bench_fault /* NMI */
    .word bench_fault /* HardFault */
    .word bench_fault /* MemManage */
    .word bench_fault /* BusFault */
    .word bench_fault /* UsageFault (including NOCP) */
    .word 0, 0, 0, 0
    .word bench_fault /* SVCall */
    .word bench_fault /* DebugMonitor */
    .word 0
    .word bench_fault /* PendSV */
    .word bench_fault /* SysTick, deliberately disabled */
    .rept 118 /* N32H474 external IRQ numbers 0..117. */
    .word bench_fault
    .endr
.size g_vectors, .-g_vectors

.section .text.Reset_Handler,"ax",%progbits
.balign 4
.global Reset_Handler
.type Reset_Handler,%function
.thumb_func
Reset_Handler:
    cpsid i
    /* Set CPACR before any C or newlib constructor executes. */
    ldr r0, =0xe000ed88
#if BENCH_FORMAT < 2
    ldr r1, =0x00f00000
#else
    movs r1, #0
#endif
    str r1, [r0]
    dsb
    isb
    ldr r0, =0xe000ed08
    ldr r1, =g_vectors
    str r1, [r0]
    dsb
    isb

    ldr r0, =__data_load
    ldr r1, =__data_start
    ldr r2, =__data_end
1:
    cmp r1, r2
    bcs 2f
    ldr r3, [r0], #4
    str r3, [r1], #4
    b 1b
2:
    ldr r1, =__bss_start
    ldr r2, =__bss_end
    movs r3, #0
3:
    cmp r1, r2
    bcs 4f
    str r3, [r1], #4
    b 3b
4:
    bl __libc_init_array
    bl main
    b bench_fault
.size Reset_Handler, .-Reset_Handler

.section .note.GNU-stack,"",%progbits
