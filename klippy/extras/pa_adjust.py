# Creator 5 eboard pressure-advance measurement
#
# This file may be distributed under the terms of the GNU GPLv3 license.
from . import pulse_counter, output_pin

class PrinterPA:
    def __init__(self, config):
        # Register commands
        self.printer = config.get_printer()
        gcode = config.get_printer().lookup_object('gcode')
        gcode.register_command("PA_GET", self.cmd_PA_GET)
        gcode.register_command("PA_ACTION", self.cmd_PA_ACTION)
        gcode.register_command("PA_STREAM", self.cmd_PA_STREAM)

        self.mcu = self.printer.lookup_object('mcu eboard')
        self.mcu.register_config_callback(self.build_config)
        self._pa_value_get_cmd = None
        self._pa_action_cmd = None
        self._pa_stream_cmd = None
        #start end
    def build_config(self):
        self._pa_value_get_cmd = self.mcu.lookup_query_command(
            "get_emcu_pa_value",
            "pa_value value=%u")

        self._pa_action_cmd = self.mcu.lookup_command(
            "pa_action action=%u pc=%u")
        # Only OpenCreator eboard firmware records the readings themselves.
        if self.mcu.try_lookup_command("get_pa_stream index=%hu") is not None:
            self._pa_stream_cmd = self.mcu.lookup_query_command(
                "get_pa_stream index=%hu",
                "pa_stream index=%hu total=%hu start=%u data=%*s")
    def cmd_PA_ACTION(self, gcmd):
        #0:PA_close  1:PA_start
        action = gcmd.get_int('ACTION', 0)
        #材料
        pc = gcmd.get_int('PC', 0)
        self.pa_action(action, pc)

    def pa_action(self, action, pc):
        self._pa_action_cmd.send([action, pc])

    def pa_get_value(self):
        return self._pa_value_get_cmd.send()["value"]

    def has_stream(self):
        return self._pa_stream_cmd is not None

    def read_stream(self):
        """[(print_time, SG_RESULT)] of the last PA_ACTION capture."""
        if self._pa_stream_cmd is None:
            raise self.printer.command_error(
                'eboard firmware cannot report its PA readings; update it')
        samples, index = [], 0
        while True:
            result = self._pa_stream_cmd.send([index])
            data = bytearray(result['data'])
            for i in range(0, len(data) - 5, 6):
                clock = self.mcu.clock32_to_clock64(
                    int.from_bytes(data[i + 2:i + 6], 'little'))
                samples.append((self.mcu.clock_to_print_time(clock),
                                data[i] | data[i + 1] << 8))
            index += len(data) // 6
            if not data or index >= result['total']:
                return samples

    def cmd_PA_STREAM(self, gcmd):
        if self._pa_stream_cmd is None:
            raise gcmd.error('eboard firmware has no PA stream')
        first = self._pa_stream_cmd.send([0])
        data = bytearray(first['data'])
        values = [data[i] | data[i + 1] << 8 for i in range(0, len(data), 6)]
        gcmd.respond_info('PA stream: %d readings, start clock %d, first %s'
                          % (first['total'], first['start'], values))

    def cmd_PA_GET(self, gcmd):
        # Get PA value
        gcmd.respond_info("Result is value=%s" % (self.pa_get_value(),))
def load_config(config):
    return PrinterPA(config)
