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

#ifndef CONFIG_FLASH_START
#define CONFIG_FLASH_START 0x08000000
#endif

#ifndef CONFIG_RAM_START
#define CONFIG_RAM_START 0x20000000
#endif

#ifndef CONFIG_FLASH_LENGTH
#define CONFIG_FLASH_LENGTH 64 /* 64KB */
#endif

#ifndef CONFIG_RAM_LENGTH
#define CONFIG_RAM_LENGTH 16 /* 16KB */
#endif

#define FLASH_LENGTH CONFIG_FLASH_LENGTH * 1024
#define RAM_LENGTH CONFIG_RAM_LENGTH * 1024

#if CONFIG_ENABLE_EXT_RAM

// For external ram start
#ifndef CONFIG_EXT_RAM_START
#define CONFIG_EXT_RAM_START 0x40000000
#endif

// For external ram length
#ifndef CONFIG_EXT_RAM_LENGTH
#define CONFIG_EXT_RAM_LENGTH 8192
#endif

#define EXT_RAM_LENGTH CONFIG_EXT_RAM_LENGTH * 1024

#endif // CONFIG_ENABLE_EXT_RAM

#if CONFIG_ENABLE_EXT_RAM && CONFIG_ENABLE_LINK_TO_EXT_RAM
#define DEFAULT_RAM_REGION EXTRAM
#else
#define DEFAULT_RAM_REGION RAM
#endif

#if CONFIG_CPU_64BITS
#define LINK_TABLE_ENTRY QUAD
#define LINK_TABLE_ALIGN 8
#else
#define LINK_TABLE_ENTRY LONG
#define LINK_TABLE_ALIGN 4
#endif

MEMORY
{

    ROM (rx) : ORIGIN = CONFIG_FLASH_START, LENGTH = FLASH_LENGTH /* flash */

    #if CONFIG_ENABLE_EXEC_IN_RAM || (CONFIG_ENABLE_EXT_RAM && CONFIG_ENABLE_LINK_TO_EXT_RAM)
    RAM (rwx) : ORIGIN = CONFIG_RAM_START,   LENGTH = RAM_LENGTH /* sram */
    #else
    RAM (rw) : ORIGIN = CONFIG_RAM_START,   LENGTH = RAM_LENGTH /* sram */
    #endif

    #if CONFIG_ENABLE_EXT_RAM
    EXTRAM (rwx) : ORIGIN = CONFIG_EXT_RAM_START,   LENGTH = EXT_RAM_LENGTH /* external ram */
    #endif
}
ENTRY(Reset_Handler)
_system_stack_size = CONFIG_STACK_LENGTH;

SECTIONS
{
    .text :
    {
        . = ALIGN(4);
        _stext = .;
        #if CONFIG_CPU_ARM
        KEEP(*(.isr_vector))            /* ISR Vector */
        #elif CONFIG_CPU_RISCV
        #if CONFIG_ARCH_RISCV_HAS_ISR_VECTOR
        KEEP(*(.isr_vector))            /* ISR Vector */
        #else
        KEEP(*(.startup_code))          /* Startup code */
        #endif
        #endif

        . = ALIGN(4);
        *(.text)                        /* remaining code */
        *(.text.*)                      /* remaining code */
        *(.rodata)                      /* read-only data (constants) */
        *(.rodata*)
        #if CONFIG_CPU_ARM
        *(.glue_7)
        *(.glue_7t)
        #endif
        *(.gnu.linkonce.t*)

        . = ALIGN(4);
        PROVIDE(__preinit_array_start = .);
        KEEP (*(SORT(.preinit_array.*)))
        KEEP (*(.preinit_array))
        PROVIDE(__preinit_array_end = .);

        . = ALIGN(4);
        PROVIDE(__init_array_start = .);
        PROVIDE(__ctors_start__ = .);
        KEEP (*(SORT(.init_array.*)))
        KEEP (*(.init_array))
        PROVIDE(__init_array_end = .);
        PROVIDE(__ctors_end__ = .);

        . = ALIGN(4);
        PROVIDE(__fini_array_start = .);
        KEEP (*(SORT(.fini_array.*)))
        KEEP (*(.fini_array))
        PROVIDE(__fini_array_end = .);

        /* used for RT-Thread */

#ifdef CONFIG_USING_RTTHREAD

        /* section information for finsh shell */
        . = ALIGN(4);
        __fsymtab_start = .;
        KEEP(*(FSymTab))
        __fsymtab_end = .;

        . = ALIGN(4);
        __vsymtab_start = .;
        KEEP(*(VSymTab))
        __vsymtab_end = .;

        /* section information for initial. */
        . = ALIGN(4);
        __rt_init_start = .;
        KEEP(*(SORT(.rti_fn*)))
        __rt_init_end = .;

#endif /* CONFIG_USING_RTTHREAD */

        . = ALIGN(4);

        _etext = .;
    } > ROM = 0

#ifdef CONFIG_CPP_EXCEPTION_SUPPORT

	.except_ordered : 
    {
		*(.gcc_except_table *.gcc_except_table.*)

        #if CONFIG_CPU_ARM
			*(.ARM.extab* .gnu.linkonce.armextab.*)
        #endif
	} > ROM
    
	.eh_frame_hdr : 
    {
		PROVIDE_HIDDEN ( __eh_frame_hdr_start = . );
		*(.eh_frame_hdr) *(.eh_frame_entry .eh_frame_entry.*)
		PROVIDE_HIDDEN ( __eh_frame_hdr_end = . );
	} > ROM

	.eh_frame : 
    {
		PROVIDE_HIDDEN ( __eh_frame_start = . );
		KEEP (*(.eh_frame .eh_frame.*))
		PROVIDE_HIDDEN ( __eh_frame_end = . );
	} > ROM

#else /* CONFIG_CPP_EXCEPTION_SUPPORT */

	/DISCARD/ : {
		*(.note .note.*)
		*(.eh_frame .eh_frame.*)

        #if CONFIG_CPU_ARM
			*(.ARM.extab* .gnu.linkonce.armextab.*)
			*(.ARM.exidx*)
        #endif
	}

#endif /* CONFIG_CPP_EXCEPTION_SUPPORT */

    #if CONFIG_CPU_ARM
	    /* .ARM.exidx is sorted, so has to go in its own output section.  */
	    __exidx_start = .;
	    .ARM.exidx :
	    {
	        *(.ARM.exidx* .gnu.linkonce.armexidx.*)
	    } > ROM
	    __exidx_end = .;
    #endif

    .copy.table :
    {
        . = ALIGN(LINK_TABLE_ALIGN);
        __copy_table_start__ = .;

        #if CONFIG_ENABLE_EXEC_IN_RAM || (CONFIG_ENABLE_EXT_RAM && CONFIG_ENABLE_LINK_TO_EXT_RAM)
        LINK_TABLE_ENTRY(LOADADDR(.ram_text));
        LINK_TABLE_ENTRY(ADDR(.ram_text));
        LINK_TABLE_ENTRY(SIZEOF(.ram_text));
        #endif

        #if CONFIG_ENABLE_EXT_RAM
        LINK_TABLE_ENTRY(LOADADDR(.ext_text));
        LINK_TABLE_ENTRY(ADDR(.ext_text));
        LINK_TABLE_ENTRY(SIZEOF(.ext_text));
        #endif

        LINK_TABLE_ENTRY(LOADADDR(.data));
        LINK_TABLE_ENTRY(ADDR(.data));
        LINK_TABLE_ENTRY(SIZEOF(.data));

        #if CONFIG_ENABLE_EXT_RAM && CONFIG_ENABLE_LINK_TO_EXT_RAM
        LINK_TABLE_ENTRY(LOADADDR(.ram_data));
        LINK_TABLE_ENTRY(ADDR(.ram_data));
        LINK_TABLE_ENTRY(SIZEOF(.ram_data));
        #endif

        #if CONFIG_ENABLE_EXT_RAM
        LINK_TABLE_ENTRY(LOADADDR(.ext_data));
        LINK_TABLE_ENTRY(ADDR(.ext_data));
        LINK_TABLE_ENTRY(SIZEOF(.ext_data));
        #endif

        __copy_table_end__ = .;
    } > ROM

    .zero.table :
    {
        . = ALIGN(LINK_TABLE_ALIGN);
        __zero_table_start__ = .;

        #if CONFIG_ENABLE_EXT_RAM && CONFIG_ENABLE_LINK_TO_EXT_RAM
        LINK_TABLE_ENTRY(ADDR(.ram_bss));
        LINK_TABLE_ENTRY(SIZEOF(.ram_bss));
        #endif

        #if CONFIG_ENABLE_EXT_RAM
        LINK_TABLE_ENTRY(ADDR(.ext_bss));
        LINK_TABLE_ENTRY(SIZEOF(.ext_bss));
        #endif

        LINK_TABLE_ENTRY(ADDR(.bss));
        LINK_TABLE_ENTRY(SIZEOF(.bss));

        __zero_table_end__ = .;
    } > ROM

    #if CONFIG_ENABLE_EXEC_IN_RAM || (CONFIG_ENABLE_EXT_RAM && CONFIG_ENABLE_LINK_TO_EXT_RAM)
    .ram_text :
    {
        . = ALIGN(4);
        _sram_text = .;
        *(.ram_text)
        *(.ram_text.*)
        *(.ramfunc)
        *(.ramfunc.*)
        . = ALIGN(4);
        _eram_text = .;
    } > RAM AT > ROM
    #endif

    #if CONFIG_ENABLE_EXT_RAM
    .ext_text :
    {
        . = ALIGN(4);
        _sext_text = .;
        *(.ext_text)
        *(.ext_text.*)
        *(.extram_text)
        *(.extram_text.*)
        . = ALIGN(4);
        _eext_text = .;
    } > EXTRAM AT > ROM
    #endif

    /* This is used by the startup in order to initialize the .data secion */
    _sidata = LOADADDR(.data);

    /* .data section which is used for initialized data */

    .data :
    {
        . = ALIGN(4);
        /* This is used by the startup in order to initialize the .data secion */
        _sdata = . ;

        *(.data)
        *(.data.*)
        *(.gnu.linkonce.d*)
        #if !(CONFIG_ENABLE_EXT_RAM && CONFIG_ENABLE_LINK_TO_EXT_RAM)
        *(.ram_data)
        *(.ram_data.*)
        #endif

        #if CONFIG_CPU_RISCV
        . = ALIGN(4);
        PROVIDE(__global_pointer$ = . + 0x800);
        *(.sdata)
        *(.sdata.*)
        *(.sdata2)
        *(.sdata2.*)
        #endif

        /* PROVIDE(__dtors_start__ = .);
        KEEP(*(SORT(.dtors.*)))
        KEEP(*(.dtors))
        PROVIDE(__dtors_end__ = .); */

        . = ALIGN(4);
        /* This is used by the startup in order to initialize the .data secion */
        _edata = . ;
    } > DEFAULT_RAM_REGION AT > ROM

    #if CONFIG_ENABLE_EXT_RAM && CONFIG_ENABLE_LINK_TO_EXT_RAM
    .ram_data :
    {
        . = ALIGN(4);
        _sram_data = .;
        *(.ram_data)
        *(.ram_data.*)
        . = ALIGN(4);
        _eram_data = .;
    } > RAM AT > ROM
    #endif

    #if CONFIG_ENABLE_EXT_RAM
    .ext_data :
    {
        . = ALIGN(4);
        _sext_data = .;
        *(.ext_data)
        *(.ext_data.*)
        *(.extram_data)
        *(.extram_data.*)
        *(.ext_ram)
        *(.ext_ram.*)
        *(.extram)
        *(.extram.*)
        . = ALIGN(4);
        _eext_data = .;
    } > EXTRAM AT > ROM
    #endif

    .stack (NOLOAD) :
    {
        . = ALIGN(4);
        _sstack = .;
        . = . + _system_stack_size;
        . = ALIGN(4);
        _estack = .;
    } > RAM

    #if CONFIG_ENABLE_EXT_RAM && CONFIG_ENABLE_LINK_TO_EXT_RAM
    .ram_bss (NOLOAD) :
    {
        . = ALIGN(4);
        _sram_bss = .;
        *(.ram_bss)
        *(.ram_bss.*)
        . = ALIGN(4);
        _eram_bss = .;
    } > RAM
    #endif

    #if CONFIG_ENABLE_EXT_RAM
    .ext_bss (NOLOAD) :
    {
        . = ALIGN(4);
        _sext_bss = .;
        *(.ext_bss)
        *(.ext_bss.*)
        *(.extram_bss)
        *(.extram_bss.*)
        . = ALIGN(4);
        _eext_bss = .;
    } > EXTRAM
    #endif

    __bss_start = .;
    .bss (NOLOAD) :
    {
        . = ALIGN(4);
        /* This is used by the startup in order to initialize the .bss secion */
        _sbss = .;

        *(.bss)
        *(.bss.*)
        #if !(CONFIG_ENABLE_EXT_RAM && CONFIG_ENABLE_LINK_TO_EXT_RAM)
        *(.ram_bss)
        *(.ram_bss.*)
        #endif
        #if CONFIG_CPU_RISCV
        *(.sbss)
        *(.sbss.*)
        *(.scommon)
        #endif
        *(COMMON)

        . = ALIGN(4);
        /* This is used by the startup in order to initialize the .bss secion */
        _ebss = . ;

        *(.bss.init)
    } > DEFAULT_RAM_REGION
    __bss_end = .;

    _end = .;

    /* Stabs debugging sections.  */
    .stab          0 : { *(.stab) }
    .stabstr       0 : { *(.stabstr) }
    .stab.excl     0 : { *(.stab.excl) }
    .stab.exclstr  0 : { *(.stab.exclstr) }
    .stab.index    0 : { *(.stab.index) }
    .stab.indexstr 0 : { *(.stab.indexstr) }
    .comment       0 : { *(.comment) }

    #if CONFIG_CPU_ARM
    .ARM.attributes 0 : { *(.ARM.attributes) }
    #elif CONFIG_CPU_RISCV
    .riscv.attributes 0 : { *(.riscv.attributes) }
    #endif

    /* DWARF debug sections.
     * Symbols in the DWARF debugging sections are relative to the beginning
     * of the section so we begin them at 0.  */
    /* DWARF 1 */
    .debug          0 : { *(.debug) }
    .line           0 : { *(.line) }
    /* GNU DWARF 1 extensions */
    .debug_srcinfo  0 : { *(.debug_srcinfo) }
    .debug_sfnames  0 : { *(.debug_sfnames) }
    /* DWARF 1.1 and DWARF 2 */
    .debug_aranges  0 : { *(.debug_aranges) }
    .debug_pubnames 0 : { *(.debug_pubnames) }
    /* DWARF 2 */
    .debug_info     0 : { *(.debug_info .gnu.linkonce.wi.*) }
    .debug_abbrev   0 : { *(.debug_abbrev) }
    .debug_line     0 : { *(.debug_line) }
    .debug_frame    0 : { *(.debug_frame) }
    .debug_str      0 : { *(.debug_str) }
    .debug_loc      0 : { *(.debug_loc) }
    .debug_macinfo  0 : { *(.debug_macinfo) }
    /* SGI/MIPS DWARF 2 extensions */
    .debug_weaknames 0 : { *(.debug_weaknames) }
    .debug_funcnames 0 : { *(.debug_funcnames) }
    .debug_typenames 0 : { *(.debug_typenames) }
    .debug_varnames  0 : { *(.debug_varnames) }
}
