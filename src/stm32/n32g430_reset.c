// Reset command for the Creator 5 N32G430F8S7 levelboard
//
// Copyright (C) 2026
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include "board/internal.h" // NVIC_SystemReset
#include "command.h" // DECL_COMMAND_FLAGS

void
command_reset(uint32_t *args)
{
    (void)args;
    NVIC_SystemReset();
}
DECL_COMMAND_FLAGS(command_reset, HF_IN_SHUTDOWN, "reset");
