# FlashForge Creator 5 eddy trigger threshold (eboard / levelboard)
#
# This file may be distributed under the terms of the GNU GPLv3 license.


class FFEddy:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.name = config.get_name().split()[-1]
        self.mcu = self.printer.lookup_object(config.get('mcu'))
        self.trigger_threshold = config.getint(
            'trigger_threshold', -15, minval=-200, maxval=200)
        self.value = self.diff = 0
        self.last_update_time = 0.
        self.mcu.register_config_callback(self._build_config)
        gcode = self.printer.lookup_object('gcode')
        suffix = self.name.upper()
        for cmd, func, desc in (
                ('GET_BASIC_PARAM', self.cmd_GET_BASIC_PARAM,
                 "Get eddy basic param"),
                ('REMOVE_PEEL', self.cmd_REMOVE_PEEL,
                 "Remove peel and get a data"),
                ('SET_TRIGGER_THRESHOLD', self.cmd_SET_TRIGGER_THRESHOLD,
                 "Set eddy trigger threshold")):
            gcode.register_command('%s_%s' % (cmd, suffix), func, desc=desc)

    def _build_config(self):
        self.get_basic_param_cmd = self.mcu.lookup_query_command(
            "get_basic_param num=%u", "param_value value=%u reserve=%u")
        self.peel_cmd = self.mcu.lookup_query_command(
            "remove_peel action=%u", "peel_data value=%i")
        self.set_trigger_threshold_cmd = self.mcu.lookup_query_command(
            "set_trigger_threshold threshold=%i",
            "trigger_threshold threshold=%i")
        self.set_trigger_threshold_cmd.send([self.trigger_threshold])

    def _updated(self):
        self.last_update_time = self.printer.get_reactor().monotonic()

    def cmd_GET_BASIC_PARAM(self, gcmd):
        result = self.get_basic_param_cmd.send([gcmd.get_int('NUM', 0)])
        self.value, self.diff = result['value'], result['reserve']
        self._updated()
        gcmd.respond_info("%s result: value=%d,diff=%d"
                          % (self.name, self.value, self.diff))

    def cmd_REMOVE_PEEL(self, gcmd):
        result = self.peel_cmd.send([gcmd.get_int('ACTION', 0)])
        self.diff = result['value']
        self._updated()
        gcmd.respond_info('%s peel_data=%d' % (self.name, self.diff))

    def cmd_SET_TRIGGER_THRESHOLD(self, gcmd):
        threshold = gcmd.get_int('THRESHOLD', self.trigger_threshold,
                                 minval=-200, maxval=200)
        self.set_trigger_threshold_cmd.send([threshold])
        self.trigger_threshold = threshold
        gcmd.respond_info("%s trigger_threshold=%d" % (self.name, threshold))

    def get_status(self, eventtime=None):
        return {'value': self.value, 'diff': self.diff,
                'trigger_threshold': self.trigger_threshold,
                'last_update_time': self.last_update_time}


def load_config_prefix(config):
    return FFEddy(config)
