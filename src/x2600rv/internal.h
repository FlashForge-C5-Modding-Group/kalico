#ifndef __X2600RV_INTERNAL_H
#define __X2600RV_INTERNAL_H
// Ingenic X2600 RISC-V core: shared definitions

#ifndef __ASSEMBLER__
#include <stdint.h>
#endif

// RISC-V core control block (also mapped on the host at the same address)
#define RV_BASE           0x12A00000
#define RV_HOST_DOORBELL  (RV_BASE + 0x08)  // host sets 1 after writing; we clear
#define RV_HOST_IRQ       (RV_BASE + 0x0C)  // we set >0 to interrupt the host
#define RV_MTIME_LO       (RV_BASE + 0x10)  // 64-bit timer at CONFIG_CLOCK_FREQ
#define RV_MTIME_HI       (RV_BASE + 0x14)

// Shared window (uncached), described by the image header in start.S
#define SHARED_BASE       0x0F052000
#define RING_RX_HDR       (SHARED_BASE + 0x000)  // host -> riscv
#define RING_TX_HDR       (SHARED_BASE + 0x020)  // riscv -> host
#define RING_RX_BUF       (SHARED_BASE + 0x040)
#define RING_TX_BUF       (SHARED_BASE + 0x640)
#define RING_SIZE         0x600

#ifndef __ASSEMBLER__
// timer.c
uint8_t timer_interrupt_pending(void);
void timer_clear_interrupt(void);
void timer_set(uint32_t value);

// ring.c
void ring_init(void);
uint32_t ring_readable(void);
uint32_t ring_read(uint8_t *buf, uint32_t len);
void ring_write(const uint8_t *buf, uint32_t len);
void ring_poll(void);
#endif

#endif // internal.h
