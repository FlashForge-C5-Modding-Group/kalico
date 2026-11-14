/* Linker script for the X2600 RISC-V core: the image loads at 0x0F000000
   and must stay below the shared window at 0x0F052000 */

OUTPUT_ARCH(riscv)
ENTRY(_header)

MEMORY {
    RAM (rwx) : ORIGIN = 0x0F000000, LENGTH = 0x52000
}

SECTIONS
{
    __load_start = ORIGIN(RAM);

    .text : ALIGN(4) {
        KEEP(*(.text.start))
        *(.text .text.*)
    } > RAM

    .rodata : ALIGN(4) {
        *(.rodata .rodata.* .srodata .srodata.*)
    } > RAM

    .data : ALIGN(4) {
        __data_start = .;
        *(.data .data.* .sdata .sdata.*)
        . = ALIGN(4);
        __data_end = .;
    } > RAM

    .copy (NOLOAD) : ALIGN(4) {
        __copy_start = .;
        . += __data_end - __data_start;
        . = ALIGN(4);
    } > RAM

    .bss (NOLOAD) : ALIGN(4) {
        __bss_start = .;
        *(.bss .bss.* .sbss .sbss.* COMMON)
        . = ALIGN(4);
        __bss_end = .;
    } > RAM

    .stack (NOLOAD) : ALIGN(16) {
        . += 0x2000;
        __stack_end = .;
    } > RAM

    _end = .;

    /DISCARD/ : {
        *(.compile_time_request)
        *(.comment*)
        *(.note*)
        *(.eh_frame*)
    }
}
