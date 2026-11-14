// setjmp/longjmp for rv32 (no FPU)

        .text
        .globl setjmp
setjmp:
        sw      ra, 0(a0)
        sw      sp, 4(a0)
        sw      s0, 8(a0)
        sw      s1, 12(a0)
        sw      s2, 16(a0)
        sw      s3, 20(a0)
        sw      s4, 24(a0)
        sw      s5, 28(a0)
        sw      s6, 32(a0)
        sw      s7, 36(a0)
        sw      s8, 40(a0)
        sw      s9, 44(a0)
        sw      s10, 48(a0)
        sw      s11, 52(a0)
        li      a0, 0
        ret

        .globl longjmp
longjmp:
        lw      ra, 0(a0)
        lw      sp, 4(a0)
        lw      s0, 8(a0)
        lw      s1, 12(a0)
        lw      s2, 16(a0)
        lw      s3, 20(a0)
        lw      s4, 24(a0)
        lw      s5, 28(a0)
        lw      s6, 32(a0)
        lw      s7, 36(a0)
        lw      s8, 40(a0)
        lw      s9, 44(a0)
        lw      s10, 48(a0)
        lw      s11, 52(a0)
        seqz    a0, a1
        add     a0, a0, a1
        ret
