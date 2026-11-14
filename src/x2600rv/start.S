// Image header and entry point for the X2600 RISC-V core.  The host's
// soc_mcu driver reads the header (magic "TPEL") when the image is loaded
// and programs the core's memory windows from it before starting it.

#include "internal.h"

        .section .text.start, "ax"
        .option push
        .option norvc
        .globl _header
_header:
        j       _start                  // core starts at the header
        .word   0x4C455054              // "TPEL"
        .word   __load_start            // load address
        .word   __load_start            // header address
        .word   _end                    // end of image
        .word   0
        .word   RING_RX_HDR             // host -> riscv ring
        .word   RING_TX_HDR             // riscv -> host ring
        .word   SHARED_BASE             // shared (uncached) window
        .word   0x1000                  // shared window size
        .option pop

        .globl _start
_start:
        la      sp, __stack_end
        la      t0, __bss_start
        la      t1, __bss_end
1:      bgeu    t0, t1, 2f
        sw      zero, 0(t0)
        addi    t0, t0, 4
        j       1b
2:      call    main
3:      j       3b
