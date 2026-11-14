#ifndef __X2600RV_SETJMP_H
#define __X2600RV_SETJMP_H
// ra, sp and s0-s11 (see setjmp.S)

typedef unsigned long jmp_buf[14];

int setjmp(jmp_buf env);
void longjmp(jmp_buf env, int val) __attribute__((noreturn));

#endif // setjmp.h
