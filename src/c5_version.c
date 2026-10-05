// Creator 5 firmware version report (shared by all C5 boards)
//
// Copyright (C) 2026
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include <stdint.h> // uint32_t
#include "command.h" // DECL_COMMAND_FLAGS

// Release label reported to the stock host as "V<year><MMDD><number>".
// All boards are built from one tree, so they share one label; bump it
// for every release so a board that was not reflashed stands out.
#define C5_VERSION_YEAR 2026
#define C5_VERSION_DATE 924
#define C5_VERSION_NUMBER 1

void
command_get_mcu_version(uint32_t *args)
{
    sendf("mcu_version year=%u date=%u version=%u"
          , C5_VERSION_YEAR, C5_VERSION_DATE, C5_VERSION_NUMBER);
}
DECL_COMMAND_FLAGS(command_get_mcu_version, HF_IN_SHUTDOWN, "get_mcu_version");
