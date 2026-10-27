/* N32H474RC board limits, matching the existing demo's 256/96 KiB config.
 * Do not replace these with the vendor family-wide 192 KiB SRAM default. */
ENTRY(Reset_Handler)
MEMORY
{
    FLASH (rx)  : ORIGIN = 0x08000000, LENGTH = 256K
    RAM   (rwx) : ORIGIN = 0x20000000, LENGTH = 96K
}

__stack_size = 8K;
__stack_top = ORIGIN(RAM) + LENGTH(RAM);
__stack_limit = __stack_top - __stack_size;

SECTIONS
{
    .isr_vector :
    {
        KEEP(*(.isr_vector))
    } > FLASH
    .text :
    {
        *(.text .text.*)
        *(.rodata .rodata.*)
        KEEP(*(.init))
        KEEP(*(.fini))
        . = ALIGN(4);
    } > FLASH
    .ARM.extab : { *(.ARM.extab* .gnu.linkonce.armextab.*) } > FLASH
    .ARM.exidx :
    {
        __exidx_start = .;
        *(.ARM.exidx* .gnu.linkonce.armexidx.*)
        __exidx_end = .;
    } > FLASH
    .preinit_array :
    {
        __preinit_array_start = .;
        KEEP(*(.preinit_array))
        __preinit_array_end = .;
    } > FLASH
    .init_array :
    {
        __init_array_start = .;
        KEEP(*(SORT(.init_array.*)))
        KEEP(*(.init_array))
        __init_array_end = .;
    } > FLASH
    .fini_array :
    {
        __fini_array_start = .;
        KEEP(*(SORT(.fini_array.*)))
        KEEP(*(.fini_array))
        __fini_array_end = .;
    } > FLASH
    .data :
    {
        . = ALIGN(8);
        __data_start = .;
        *(.data .data.*)
        . = ALIGN(4);
        __data_end = .;
    } > RAM AT > FLASH
    __data_load = LOADADDR(.data);
    __flash_end = LOADADDR(.data) + SIZEOF(.data);
    .bss (NOLOAD) :
    {
        . = ALIGN(8);
        __bss_start = .;
        *(.bss.g_bench_results)
        *(.bss .bss.* COMMON)
        . = ALIGN(8);
        __bss_end = .;
        __end = .;
    } > RAM
    .stack __stack_limit (NOLOAD) :
    {
        . += __stack_size;
    } > RAM
    .ARM.attributes 0 : { *(.ARM.attributes) }
    /DISCARD/ : { *(.note.GNU-stack) *(.comment) }
}
ASSERT(__bss_end <= __stack_limit, "benchmark data overlaps reserved 8 KiB stack")
ASSERT(__flash_end <= ORIGIN(FLASH) + LENGTH(FLASH), "benchmark exceeds 256 KiB Flash")
