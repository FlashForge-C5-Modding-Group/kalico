// Free watchdog support for GD32H737
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include "internal.h" // FWDGT_CTL
#include "sched.h" // DECL_TASK

void
watchdog_reset(void)
{
    FWDGT_CTL = FWDGT_KEY_RELOAD;
}
DECL_TASK(watchdog_reset);

void
watchdog_init(void)
{
    RCU_RSTSCK |= RCU_RSTSCK_IRC32KEN;
    while (!(RCU_RSTSCK & RCU_RSTSCK_IRC32KSTB))
        ;
    FWDGT_CTL = FWDGT_WRITEACCESS_ENABLE;
    FWDGT_PSC = FWDGT_PSC_DIV4;
    FWDGT_RLD = FWDGT_RLD_RLD;
    FWDGT_WND = FWDGT_WND_WND;
    while (FWDGT_STAT & (FWDGT_STAT_PUD | FWDGT_STAT_RUD | FWDGT_STAT_WUD))
        ;
    FWDGT_CTL = FWDGT_KEY_RELOAD;
    FWDGT_CTL = FWDGT_KEY_ENABLE;
}
DECL_INIT(watchdog_init);
