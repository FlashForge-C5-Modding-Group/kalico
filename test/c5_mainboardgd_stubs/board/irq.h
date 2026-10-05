// Host-build stub for interrupt control
//
// Copyright (C) 2026
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#ifndef __C5_MAINBOARDGD_TEST_IRQ_H
#define __C5_MAINBOARDGD_TEST_IRQ_H

#include <stdint.h>

typedef uint32_t irqstatus_t;

irqstatus_t irq_save(void);
void irq_restore(irqstatus_t flag);

#endif // c5_mainboardgd_stubs/board/irq.h
