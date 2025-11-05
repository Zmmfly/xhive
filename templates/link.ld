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

/*
    ROM_START default to 0x08000000
    RAM_START default to 0x20000000

    ROM_LENGTH defined by Kconfig
    RAM_LENGTH defined by Kconfig

    STACK_LENGTH defined by Kconfig
    HEAP_LENGTH defined by Kconfig
 */
#ifndef CONFIG_ROM_START
#define CONFIG_ROM_START 0x08000000
#endif

#ifndef CONFIG_RAM_START
#define CONFIG_RAM_START 0x20000000
#endif

#ifndef CONFIG_ROM_LENGTH
#define CONFIG_ROM_LENGTH 0x00010000 /* 64KB */
#endif

#ifndef CONFIG_RAM_LENGTH
#define CONFIG_RAM_LENGTH 0x00005000 /* 20KB */
#endif

MEMORY
{

    ROM (rx) : ORIGIN = CONFIG_ROM_START, LENGTH = CONFIG_ROM_LENGTH /* flash */
    RAM (rw) : ORIGIN = CONFIG_RAM_START, LENGTH = CONFIG_RAM_LENGTH /* sram */
}
ENTRY(Reset_Handler)
_system_stack_size = CONFIG_STACK_LENGTH;

SECTIONS
{
    .text :
    {
        . = ALIGN(4);
        _stext = .;
        KEEP(*(.isr_vector))            /* Startup code */

        . = ALIGN(4);
        *(.text)                        /* remaining code */
        *(.text.*)                      /* remaining code */
        *(.rodata)                      /* read-only data (constants) */
        *(.rodata*)
        *(.glue_7)
        *(.glue_7t)
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
		*(.ARM.extab* .gnu.linkonce.armextab.*)
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

#else

	/DISCARD/ : {
		*(.note .note.*)
		*(.eh_frame .eh_frame.*)
		*(.ARM.extab* .gnu.linkonce.armextab.*)
		*(.ARM.exidx*)
	}

#endif /* CONFIG_CPP_EXCEPTION_SUPPORT */

    /* .ARM.exidx is sorted, so has to go in its own output section.  */
    __exidx_start = .;
    .ARM.exidx :
    {
        *(.ARM.exidx* .gnu.linkonce.armexidx.*)

        /* This is used by the startup in order to initialize the .data secion */
        _sidata = .;
    } > ROM
    __exidx_end = .;

    /* .data section which is used for initialized data */

    .data : AT (_sidata)
    {
        . = ALIGN(4);
        /* This is used by the startup in order to initialize the .data secion */
        _sdata = . ;

        *(.data)
        *(.data.*)
        *(.gnu.linkonce.d*)

        /* PROVIDE(__dtors_start__ = .);
        KEEP(*(SORT(.dtors.*)))
        KEEP(*(.dtors))
        PROVIDE(__dtors_end__ = .); */

        . = ALIGN(4);
        /* This is used by the startup in order to initialize the .data secion */
        _edata = . ;
    } >RAM

    .stack :
    {
        . = ALIGN(4);
        _sstack = .;
        . = . + _system_stack_size;
        . = ALIGN(4);
        _estack = .;
    } >RAM

    __bss_start = .;
    .bss :
    {
        . = ALIGN(4);
        /* This is used by the startup in order to initialize the .bss secion */
        _sbss = .;

        *(.bss)
        *(.bss.*)
        *(COMMON)

        . = ALIGN(4);
        /* This is used by the startup in order to initialize the .bss secion */
        _ebss = . ;

        *(.bss.init)
    } > RAM
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
