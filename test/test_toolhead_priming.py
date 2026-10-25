# Toolhead restart behaviour with a slow G-code parser, using the real
# lookahead, trapq and background step flushing (no MCU).
import types

import pytest

from klippy import toolhead as th_mod
from klippy.extras import danger_options
from klippy.extras import motion_queuing as mq_mod


class FakeReactor:
    NEVER = 9999999999999999.
    NOW = 0.

    def __init__(self):
        self.now = 100.
        self.timers = []

    def monotonic(self):
        return self.now

    def register_timer(self, callback, waketime=NEVER):
        timer = [callback, waketime]
        self.timers.append(timer)
        return timer

    def update_timer(self, timer, waketime):
        timer[1] = waketime

    def unregister_timer(self, timer):
        if timer in self.timers:
            self.timers.remove(timer)

    def _run_due(self):
        while True:
            due = [t for t in self.timers if t[1] <= self.now]
            if not due:
                return
            timer = min(due, key=lambda t: t[1])
            timer[1] = self.NEVER
            waketime = timer[0](self.now)
            if timer in self.timers:
                timer[1] = waketime

    def pause(self, waketime):
        while True:
            self._run_due()
            pending = [t[1] for t in self.timers if t[1] < self.NEVER]
            if pending and min(pending) < waketime:
                self.now = max(self.now, min(pending))
                continue
            break
        self.now = max(self.now, waketime)
        self._run_due()
        return self.now


class FakeMcu:
    def __init__(self, fileoutput):
        self.fileoutput = fileoutput

    def estimated_print_time(self, eventtime):
        return eventtime

    def is_fileoutput(self):
        return self.fileoutput


class FakePrinter:
    def __init__(self, reactor, mcu):
        self.reactor = reactor
        self.mcu = mcu
        self.shutdowns = []

    def get_reactor(self):
        return self.reactor

    def lookup_objects(self, module=None):
        return [("mcu", self.mcu)]

    def register_event_handler(self, *args):
        pass

    def send_event(self, *args):
        pass

    def invoke_shutdown(self, msg):
        self.shutdowns.append(msg)


@pytest.fixture(autouse=True)
def danger(monkeypatch):
    monkeypatch.setattr(danger_options, "DANGER_OPTIONS",
                        types.SimpleNamespace(bgflush_extra_time=0.25),
                        raising=False)


def make_toolhead(can_pause=True):
    reactor = FakeReactor()
    mcu = FakeMcu(fileoutput=not can_pause)
    printer = FakePrinter(reactor, mcu)
    mq = mq_mod.PrinterMotionQueuing(
        types.SimpleNamespace(get_printer=lambda: printer))
    th = th_mod.ToolHead.__new__(th_mod.ToolHead)
    th.printer = printer
    th.reactor = reactor
    th.mcu = mcu
    th.lookahead = th_mod.LookAheadQueue()
    th.lookahead.set_flush_time(th_mod.BUFFER_TIME_HIGH)
    th.commanded_pos = [0., 0., 0., 0.]
    th.max_velocity = 600.
    th.max_accel = 30000.
    th.min_cruise_ratio = 0.5
    th.square_corner_velocity = 9.
    th._calc_junction_deviation()
    th.check_stall_time = 0.
    th.print_stall = 0
    th.can_pause = can_pause
    th.need_check_pause = -1.
    th.print_time = 0.
    th.special_queuing_state = "NeedPrime"
    th.priming_timer = None
    th.wall_flush_deadline = None
    th.motion_queuing = mq
    mq.register_flush_callback(th._handle_step_flush, can_add_trapq=True)
    th.trapq = mq.allocate_trapq()
    real_append = mq.lookup_trapq_append()
    th.appended = []  # (host time, print time, duration, end velocity)

    def record_append(tq, t, accel_t, cruise_t, decel_t, *rest):
        start_v, cruise_v, accel = rest[-3:]
        th.appended.append((reactor.now, t, accel_t + cruise_t + decel_t,
                            cruise_v - accel * decel_t))
        real_append(tq, t, accel_t, cruise_t, decel_t, *rest)

    th.trapq_append = record_append
    th.extra_axes = []
    th.kin = types.SimpleNamespace(check_move=lambda move: None)
    reactor.pause(reactor.now + 1.)  # idle head, flush timer drained
    return th


def stream_segments(th, count, host_ms, seg=0.3, until_moving=False):
    # Zigzag segments at 600 mm/s (90 degree corners, like infill), parsed at
    # host_ms per move and yielding every 10 ms like virtual_sdcard.
    reactor = th.reactor
    x = y = 0.
    next_yield = reactor.now + 0.010
    for i in range(count):
        if i % 2:
            x += seg
        else:
            y = seg - y
        th.move([x, y, 0., 0.], 600.)
        if until_moving and th.appended:
            return
        reactor.now += host_ms / 1000.
        if reactor.now >= next_yield:
            reactor.pause(reactor.NOW)
            next_yield = reactor.now + 0.010


def full_stops(th):
    return [a for a in th.appended if a[3] < 1e-6]


def test_slow_parser_starts_motion_quickly_and_never_stops_the_head():
    # ~1.2x real time: the base 2.0s move-time budget never released any of
    # these 1500 moves, and releasing small batches with only the 0.25s lead
    # tripped the run-out flush (decelerate to a stop) repeatedly.
    th = make_toolhead()
    start = th.reactor.now
    stream_segments(th, 1500, host_ms=5.)
    assert th.appended
    assert th.appended[0][0] - start <= th_mod.LOOKAHEAD_WALL_FLUSH_TIME + 0.02
    assert full_stops(th) == []
    assert th.print_stall == 0
    assert th.printer.shutdowns == []


def test_restart_generates_first_steps_without_waiting_for_the_reactor():
    th = make_toolhead()
    stream_segments(th, 1500, host_ms=5., until_moving=True)
    # Returned straight from the move() that released the first batch, so no
    # reactor timer has run since: steps past its start already exist.
    first_move_print_time = th.appended[0][1]
    assert th.motion_queuing.last_step_gen_time > first_move_print_time


def test_batch_mode_keeps_move_time_budget():
    th = make_toolhead(can_pause=False)
    stream_segments(th, 500, host_ms=5.)
    assert th.special_queuing_state == "Priming"
    assert th.appended == []


def test_idle_priming_flush_generates_steps_immediately():
    th = make_toolhead()
    stream_segments(th, 3, host_ms=1.)
    before = th.motion_queuing.last_step_gen_time
    th._priming_handler(th.reactor.now)
    assert th.special_queuing_state == "NeedPrime"
    assert th.appended
    assert th.motion_queuing.last_step_gen_time > before
