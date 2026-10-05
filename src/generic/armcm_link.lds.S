// Generic ARM Cortex-M linker script
//
// Copyright (C) 2019  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include "autoconf.h" // CONFIG_FLASH_APPLICATION_ADDRESS

#if CONFIG_ARMCM_FLASH_SIZE_IS_TOTAL
#define ARMCM_ROM_LENGTH (CONFIG_FLASH_SIZE - (CONFIG_FLASH_APPLICATION_ADDRESS - CONFIG_FLASH_BOOT_ADDRESS))
#else
#define ARMCM_ROM_LENGTH CONFIG_FLASH_SIZE
#endif

OUTPUT_FORMAT("elf32-littlearm", "elf32-littlearm", "elf32-littlearm")
OUTPUT_ARCH(arm)

#if CONFIG_ARMCM_EXPLICIT_RESET_ENTRY
ENTRY(ResetHandler)
#endif

MEMORY
{
  rom (rx) : ORIGIN = CONFIG_FLASH_APPLICATION_ADDRESS , LENGTH = ARMCM_ROM_LENGTH
  ram (rwx) : ORIGIN = CONFIG_RAM_START , LENGTH = CONFIG_RAM_SIZE
}

SECTIONS
{
    .text : {
        . = ALIGN(4);
        _text_vectortable_start = .;
        KEEP(*(.vector_table))
        _text_vectortable_end = .;
        *(.text .text.*)
        *(.rodata .rodata*)
    } > rom

    . = ALIGN(4);
    _data_flash = .;

#if CONFIG_ARMCM_RAM_VECTORTABLE
    .ram_vectortable (NOLOAD) : {
        _ram_vectortable_start = .;
        . = . + ( _text_vectortable_end - _text_vectortable_start ) ;
        _ram_vectortable_end = .;
    } > ram
#endif

    .data : AT (_data_flash)
    {
        . = ALIGN(4);
        _data_start = .;
        *(.ramfunc .ramfunc.*);
        *(.data .data.*);
        . = ALIGN(4);
        _data_end = .;
    } > ram

#if CONFIG_ARMCM_FLASH_SIZE_IS_TOTAL
    ASSERT(LOADADDR(.data) + SIZEOF(.data) <= ORIGIN(rom) + LENGTH(rom),
           "region `rom' overflowed by initialized data")
#endif

    .bss (NOLOAD) :
    {
        . = ALIGN(4);
        _bss_start = .;
        *(.bss .bss.*)
        *(COMMON)
        . = ALIGN(4);
        _bss_end = .;
    } > ram

    _stack_start = CONFIG_RAM_START + CONFIG_RAM_SIZE - CONFIG_STACK_SIZE ;
    .stack _stack_start (NOLOAD) :
    {
        . = . + CONFIG_STACK_SIZE;
        _stack_end = .;
    } > ram

    /DISCARD/ : {
        // The .init/.fini sections are used by __libc_init_array(), but
        // that isn't needed so no need to include them in the binary.
        *(.init)
        *(.fini)
        // Don't include exception tables
        *(.ARM.extab)
        *(.ARM.exidx)
    }
}
