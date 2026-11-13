# Creator 5 mainboard and power supply fans, from the mainboard MCU's
# temperature and the printer's load
#
# This file may be distributed under the terms of the GNU GPLv3 license.

CHECK_TIME = 1.0
MIN_CHANGE = 0.02

# Options that C5_BOARD_FANS can change until restart: name -> (min, max)
SETTINGS = {
    "on_temp": (0.0, 150.0),
    "max_temp": (0.0, 150.0),
    "min_speed": (0.0, 1.0),
    "hysteresis": (0.0, 50.0),
    "motor_min": (0.0, 1.0),
    "heater_watts": (0.0, 1000.0),
    "motor_watts": (0.0, 1000.0),
    "full_watts": (1.0, 5000.0),
    "psu_min_speed": (0.0, 1.0),
    "heating_min": (0.0, 1.0),
    "runon": (0.0, 3600.0),
    "runon_speed": (0.0, 1.0),
}
DEFAULTS = {
    "on_temp": 54.5,
    "max_temp": 65.0,
    "min_speed": 0.3,
    "hysteresis": 2.0,
    "motor_min": 0.4,
    "heater_watts": 45.0,
    "motor_watts": 30.0,
    "full_watts": 120.0,
    "psu_min_speed": 0.3,
    "heating_min": 0.5,
    "runon": 60.0,
    "runon_speed": 0.8,
}


class C5BoardFans:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.reactor = self.printer.get_reactor()
        self.mainboard_name = config.get("mainboard_fan", "pcb_fan")
        self.psu_name = config.get("psu_fan", "heat_ctl_fan")
        self.sensor_name = config.get(
            "sensor", "temperature_sensor mainboard_mcu"
        )
        self.stepper_names = config.getlist(
            "steppers", ("stepper_x", "stepper_y", "stepper_z")
        )
        self.heater_names = config.getlist(
            "heaters", ("extruder", "extruder1", "extruder2", "extruder3")
        )
        self.settings = {}
        for name, (lo, hi) in SETTINGS.items():
            self.settings[name] = config.getfloat(
                name, DEFAULTS[name], minval=lo, maxval=hi
            )
        if self.settings["max_temp"] <= self.settings["on_temp"]:
            raise config.error("max_temp must be above on_temp")
        self.stepper_enable = self.printer.load_object(config, "stepper_enable")
        self.printer.load_object(config, "heaters")
        self.fans = self.heaters = self.sensor = None
        self.hold = None
        self.runon_left = 0.0
        self.last_check = None
        self.status = {"mainboard_speed": 0.0, "psu_speed": 0.0,
                       "temperature": 0.0, "load_watts": 0.0}
        self.printer.register_event_handler("klippy:connect", self._connect)
        self.printer.register_event_handler("klippy:ready", self._ready)
        gcode = self.printer.lookup_object("gcode")
        gcode.register_command(
            "C5_BOARD_FANS", self.cmd_C5_BOARD_FANS,
            desc=self.cmd_C5_BOARD_FANS_help,
        )

    def _connect(self):
        self.fans = {}
        for role, name in (("mainboard", self.mainboard_name),
                           ("psu", self.psu_name)):
            obj = self.printer.lookup_object("fan_generic " + name, None)
            if obj is None:
                raise self.printer.config_error(
                    "c5_board_fans: no [fan_generic %s]" % (name,))
            self.fans[role] = obj.fan
        self.sensor = self.printer.lookup_object(self.sensor_name, None)
        if self.sensor is None:
            raise self.printer.config_error(
                "c5_board_fans: no [%s]" % (self.sensor_name,))
        pheaters = self.printer.lookup_object("heaters")
        self.heaters = [pheaters.lookup_heater(n) for n in self.heater_names
                        if n in pheaters.heaters]
        steppers = self.stepper_enable.get_steppers()
        unknown = [n for n in self.stepper_names if n not in steppers]
        if unknown:
            raise self.printer.config_error(
                "c5_board_fans: unknown steppers %s" % (", ".join(unknown),))

    def _ready(self):
        self.reactor.register_timer(self._check, self.reactor.NOW)

    def _curve(self, temp, active, current):
        s = self.settings
        if not active:
            return 0.0
        if temp >= s["max_temp"]:
            return 1.0
        if temp > s["on_temp"]:
            return s["min_speed"] + (1.0 - s["min_speed"]) * (
                temp - s["on_temp"]) / (s["max_temp"] - s["on_temp"])
        if current > 0.0 and temp > s["on_temp"] - s["hysteresis"]:
            return s["min_speed"]
        return 0.0

    def _check(self, eventtime):
        s = self.settings
        elapsed = (CHECK_TIME if self.last_check is None
                   else eventtime - self.last_check)
        self.last_check = eventtime
        temp = self.sensor.get_status(eventtime).get("temperature", 0.0)
        motors = any(
            self.stepper_enable.lookup_enable(n).is_motor_enabled()
            for n in self.stepper_names)
        watts = motor_watts = s["motor_watts"] if motors else 0.0
        heating = False
        for heater in self.heaters:
            status = heater.get_status(eventtime)
            watts += s["heater_watts"] * status.get("power", 0.0)
            heating |= status.get("target", 0.0) > 0.0

        mainboard_now = self.fans["mainboard"].get_status(eventtime)["speed"]
        psu_now = self.fans["psu"].get_status(eventtime)["speed"]
        # The mainboard fan runs the plain curve, so the hysteresis goes by it
        mainboard = self._curve(temp, motors or heating, mainboard_now)
        if motors:
            mainboard = max(mainboard, s["motor_min"])

        if watts > 0.0 or heating:
            psu = min(max(watts / s["full_watts"], s["psu_min_speed"]), 1.0)
            if heating:
                psu = max(psu, s["heating_min"])
            self.runon_left = s["runon"]
        elif self.runon_left > 0.0:
            psu = min(psu_now, s["runon_speed"])
            self.runon_left = max(self.runon_left - elapsed, 0.0)
        else:
            psu = 0.0
        psu = max(psu, mainboard)

        if self.hold is not None:
            mainboard = psu = self.hold
        for role, want, now in (("mainboard", mainboard, mainboard_now),
                                ("psu", psu, psu_now)):
            if abs(want - now) >= MIN_CHANGE or (want == 0.0 and now > 0.0):
                self.fans[role].set_speed(want)
        self.status = {"mainboard_speed": mainboard, "psu_speed": psu,
                       "temperature": temp, "load_watts": watts}
        return eventtime + CHECK_TIME

    def get_status(self, eventtime):
        status = dict(self.status)
        status.update(self.settings)
        status["hold"] = -1.0 if self.hold is None else self.hold
        status["runon_left"] = self.runon_left
        return status

    cmd_C5_BOARD_FANS_help = (
        "Show the board fans, HOLD=<0-1> to hold both at one speed "
        "(HOLD=-1 to release), or change a setting until restart")

    def cmd_C5_BOARD_FANS(self, gcmd):
        hold = gcmd.get_float("HOLD", None, maxval=1.0)
        if hold is not None:
            self.hold = None if hold < 0.0 else hold
        for name, (lo, hi) in SETTINGS.items():
            value = gcmd.get_float(name.upper(), None, minval=lo, maxval=hi)
            if value is not None:
                self.settings[name] = value
        if self.settings["max_temp"] <= self.settings["on_temp"]:
            raise gcmd.error("MAX_TEMP must be above ON_TEMP")
        st = self.get_status(self.reactor.monotonic())
        gcmd.respond_info(
            "Mainboard fan %.0f%%, power supply fan %.0f%%; MCU %.1f C, "
            "load %.0f W%s" % (
                st["mainboard_speed"] * 100.0, st["psu_speed"] * 100.0,
                st["temperature"], st["load_watts"],
                "" if self.hold is None
                else "; held at %.0f%%" % (self.hold * 100.0,)))


def load_config(config):
    return C5BoardFans(config)
