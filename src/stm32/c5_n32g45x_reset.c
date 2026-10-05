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

void
command_reset(uint32_t *args)
{
    (void)args;
    RCC->APB1ENR |= RCC_APB1ENR_PWREN | RCC_APB1ENR_BKPEN;
    PWR->CR |= PWR_CR_DBP;
    BKP->DR1 = C5_RESET_FLAG;
    NVIC_SystemReset();
}
DECL_COMMAND_FLAGS(command_reset, HF_IN_SHUTDOWN, "reset");
