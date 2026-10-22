# Creator 5: put the eBoard, heaterBoard and levelboard into their boot
# stage for a live firmware update (oc-mcu-flash on the SoC does the rest)
#
# Copyright (C) 2026
#
# This file may be distributed under the terms of the GNU GPLv3 license.

BOARDS = ('eboard', 'eheaterboard', 'levelboard')


class C5McuFlash:
    def __init__(self, config):
        self.printer = config.get_printer()
        gcode = self.printer.lookup_object('gcode')
        gcode.register_command('C5_MCU_BOOTLOADER', self.cmd_BOOTLOADER,
                               desc=self.cmd_BOOTLOADER_help)

    cmd_BOOTLOADER_help = ("Reset boards into their boot stage for a live "
                           "firmware update; Klipper shuts down")
    def cmd_BOOTLOADER(self, gcmd):
        names = [n.strip().lower() for n in
                 gcmd.get('BOARDS', ",".join(BOARDS)).split(',') if n.strip()]
        for name in names:
            if name not in BOARDS:
                raise gcmd.error("BOARDS: unknown board '%s' (%s)"
                                 % (name, ", ".join(BOARDS)))
        stats = self.printer.lookup_object('print_stats', None)
        eventtime = self.printer.get_reactor().monotonic()
        if stats is not None and stats.get_status(eventtime)['state'] in (
                'printing', 'paused'):
            raise gcmd.error("Not during a print")
        cmds = {}
        for name in names:
            mcu = self.printer.lookup_object('mcu ' + name, None)
            if mcu is None:
                raise gcmd.error("No [mcu %s]" % (name,))
            cmd = mcu.try_lookup_command("c5_reset_bootloader")
            if cmd is None:
                raise gcmd.error("%s firmware has no c5_reset_bootloader; "
                                 "update it from USB once first" % (name,))
            cmds[name] = cmd
        gcmd.respond_info("Resetting %s into the boot stage"
                          % (", ".join(names),))
        for name in names:
            cmds[name].send([])


def load_config(config):
    return C5McuFlash(config)
