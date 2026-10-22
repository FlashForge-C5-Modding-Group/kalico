// Reset command for the Creator 5 eBoard and heaterBoard
//
// Copyright (C) 2026
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include "board/internal.h" // NVIC_SystemReset
#include "command.h" // DECL_COMMAND_FLAGS
#include "generic/armcm_reset.h" // try_request_canboot

// The stock "reset" handler leaves this value in backup data register 1 for
// the stock boot stage before requesting a system reset.  Without it, the
// board does not return to the application after a warm reset.
#define C5_RESET_FLAG 0x1234

// The stock boot stage is not Katapult, so there is no request to leave.
void
try_request_canboot(void)
{
}

static void
reset_with_flag(uint16_t flag)
{
    RCC->APB1ENR |= RCC_APB1ENR_PWREN | RCC_APB1ENR_BKPEN;
    PWR->CR |= PWR_CR_DBP;
    BKP->DR1 = flag;
    NVIC_SystemReset();
}

void
command_reset(uint32_t *args)
{
    (void)args;
    reset_with_flag(C5_RESET_FLAG);
}
DECL_COMMAND_FLAGS(command_reset, HF_IN_SHUTDOWN, "reset");

// Without the flag the boot stage stays resident, as after power-on, so the
// stock IAPCommand can flash the board without a power cycle.
void
command_c5_reset_bootloader(uint32_t *args)
{
    (void)args;
    reset_with_flag(0);
}
DECL_COMMAND_FLAGS(command_c5_reset_bootloader, HF_IN_SHUTDOWN,
                   "c5_reset_bootloader");
