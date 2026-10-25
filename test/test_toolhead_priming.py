import types

from klippy import toolhead as th_mod


class FakeReactor:
    NEVER = 9999999999999999.
    NOW = 0.

    def __init__(self):
        self.now = 100.

    def monotonic(self):
        return self.now

    def register_timer(self, callback, waketime=NEVER):
        return object()

    def update_timer(self, timer, waketime):
        pass

    def unregister_timer(self, timer):
        pass

    def pause(self, waketime):
        self.now = max(self.now, waketime)
        return self.now


class FakeMcu:
    def estimated_print_time(self, eventtime):
        return eventtime


class FakeMotionQueuing:
    def __init__(self):
        self.flushes = 0

    def calc_step_gen_restart(self, est_print_time):
        return est_print_time + 0.1

    def note_mcu_movequeue_activity(self, mq_time, is_step_gen=True):
        pass

    def flush_due_steps(self):
        self.flushes += 1


def make_toolhead(can_pause=True):
    th = th_mod.ToolHead.__new__(th_mod.ToolHead)
    th.printer = types.SimpleNamespace(send_event=lambda *a: None)
    th.reactor = FakeReactor()
    th.mcu = FakeMcu()
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
    th.prime_flush_deadline = None
    th.motion_queuing = FakeMotionQueuing()
    th.trapq = object()
    th.appended = []
    th.trapq_append = lambda *args: th.appended.append(args)
    th.extra_axes = []
    th.kin = types.SimpleNamespace(check_move=lambda move: None)
    return th


def stream_segments(th, count, host_time_per_move=0.002):
    # Zigzag 0.2 mm segments at 600 mm/s (90 degree corners, like infill):
    # 1/3000 s of move time each, so the 2.0 s lookahead budget alone would
    # need 6000 segments. Corners give lazy flush its cut points.
    x = y = 0.
    for i in range(count):
        if i % 2:
            x += 0.2
        else:
            y = 0.2 - y
        th.move([x, y, 0., 0.], 600.)
        th.reactor.now += host_time_per_move
        if not th.special_queuing_state:
            return


def test_streamed_input_starts_motion_within_bounded_host_time():
    th = make_toolhead()
    stream_segments(th, 500)
    assert th.special_queuing_state == ""
    assert th.appended
    assert th.motion_queuing.flushes == 1
    assert th.reactor.now - 100. <= th_mod.PRIME_FLUSH_WALL_TIME + 0.01


def test_batch_mode_keeps_move_time_budget():
    th = make_toolhead(can_pause=False)
    stream_segments(th, 500)
    assert th.special_queuing_state == "Priming"
    assert th.appended == []
    assert th.motion_queuing.flushes == 0


def test_idle_priming_flush_generates_steps_immediately():
    th = make_toolhead()
    stream_segments(th, 3)
    th._priming_handler(th.reactor.now)
    assert th.special_queuing_state == "NeedPrime"
    assert th.appended
    assert th.motion_queuing.flushes == 1
