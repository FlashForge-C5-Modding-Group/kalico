// Host link over the soc_mcu shared memory rings
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include <string.h> // memcpy
#include "board/io.h" // readl
#include "internal.h" // RING_RX_HDR

// Layout used by the host's ring_mem_* helpers (utils.ko); the indexes
// count up forever and are taken modulo the size
struct ring_mem {
    uint32_t buf;
    uint32_t size;
    uint32_t write_idx;
    uint32_t read_idx;
    uint32_t write_virt;
    uint32_t read_virt;
    uint32_t pad[2];
};

#define RX ((volatile struct ring_mem *)RING_RX_HDR)
#define TX ((volatile struct ring_mem *)RING_TX_HDR)

static void
ring_setup(volatile struct ring_mem *r, uint32_t buf)
{
    r->buf = buf;
    r->size = RING_SIZE;
    r->write_idx = 0;
    r->read_idx = 0;
    r->write_virt = buf;
    r->read_virt = buf;
    r->pad[0] = r->pad[1] = 0;
}

void
ring_init(void)
{
    ring_setup(RX, RING_RX_BUF);
    ring_setup(TX, RING_TX_BUF);
    writel((void *)RV_HOST_DOORBELL, 0);
}

uint32_t
ring_readable(void)
{
    return RX->write_idx - RX->read_idx;
}

uint32_t
ring_read(uint8_t *buf, uint32_t len)
{
    uint32_t avail = ring_readable();
    if (len > avail)
        len = avail;
    uint32_t pos = RX->read_idx % RING_SIZE, first = RING_SIZE - pos;
    const uint8_t *data = (const uint8_t *)RING_RX_BUF;
    if (first > len)
        first = len;
    memcpy(buf, data + pos, first);
    memcpy(buf + first, data, len - first);
    __asm__ __volatile__("fence rw, rw" ::: "memory");
    RX->read_idx += len;
    return len;
}

void
ring_write(const uint8_t *buf, uint32_t len)
{
    uint8_t *data = (uint8_t *)RING_TX_BUF;
    while (len) {
        uint32_t room = RING_SIZE - (TX->write_idx - TX->read_idx);
        if (!room)
            continue; // host reader drains it
        uint32_t n = len < room ? len : room;
        uint32_t pos = TX->write_idx % RING_SIZE, first = RING_SIZE - pos;
        if (first > n)
            first = n;
        memcpy(data + pos, buf, first);
        memcpy(data, buf + first, n - first);
        __asm__ __volatile__("fence rw, rw" ::: "memory");
        TX->write_idx += n;
        buf += n;
        len -= n;
    }
    writel((void *)RV_HOST_IRQ, 1);
}

// Acknowledge the host's doorbell so it can ring it again
void
ring_poll(void)
{
    if (readl((void *)RV_HOST_DOORBELL))
        writel((void *)RV_HOST_DOORBELL, 0);
}
