# Host-side tests for the VFA calibration helpers in stepper_resonance_tester.
import importlib.util
import pathlib
import unittest

import numpy as np


MODULE_PATH = (pathlib.Path(__file__).resolve().parents[1]
               / "klippy/extras/stepper_resonance_tester.py")
spec = importlib.util.spec_from_file_location("c5_srt_host", MODULE_PATH)
srt = importlib.util.module_from_spec(spec)
spec.loader.exec_module(srt)


class CruiseWindowTest(unittest.TestCase):
    def test_skips_ramps_and_settling(self):
        t0, t1 = srt.cruise_window(10.0, 12.0, vel=100.0, accel=5000.0)
        self.assertAlmostEqual(t0, 10.0 + 0.02 + srt.CRUISE_SETTLE)
        self.assertAlmostEqual(t1, 12.0 - 0.02 - srt.CRUISE_SETTLE)

    def test_too_short_gives_none(self):
        self.assertIsNone(srt.cruise_window(0.0, 0.4, vel=100.0, accel=5000.0))

    def test_fast_move_needs_only_enough_cycles(self):
        # 300 mm/s over 100 mm at 10000 mm/s²: about 0.36 s, 0.1 s of it steady.
        # Enough for a 525 Hz ripple (40 cycles = 0.076 s), not for 50 Hz
        t_end = 2 * 0.03 + 91. / 300.
        self.assertIsNotNone(srt.cruise_window(0., t_end, 300., 10000., freq=525.))
        self.assertIsNone(srt.cruise_window(0., t_end, 300., 10000., freq=50.))


class CombineAmpsTest(unittest.TestCase):
    def test_minimises_summed_parabolas(self):
        # forward best at 0.02, backward at 0.04, backward twice as steep
        fwd = (0.0, -2 * 1.0 * 0.02, 1.0)
        bwd = (0.0, -2 * 2.0 * 0.04, 2.0)
        amp = srt.combine_direction_amps([0.02, 0.04], [fwd, bwd])
        self.assertAlmostEqual(amp, (0.02 * 1.0 + 0.04 * 2.0) / 3.0)

    def test_stays_between_the_two(self):
        fwd = (0.0, -2 * 0.02, 1.0)
        bwd = (0.0, 0.5, 1.0)  # its own minimum is negative
        amp = srt.combine_direction_amps([0.02, 0.03], [fwd, bwd])
        self.assertGreaterEqual(amp, 0.02)
        self.assertLessEqual(amp, 0.03)

    def test_falls_back_to_mean(self):
        self.assertAlmostEqual(
            srt.combine_direction_amps([0.02, 0.04], [None, (0, 0, 1)]), 0.03)
        self.assertAlmostEqual(
            srt.combine_direction_amps([0.02, 0.04], [(0, 0, -1), (0, 0, 1)]),
            0.03)

    def test_not_max(self):
        fwd = (0.0, -2 * 0.02, 1.0)
        bwd = (0.0, -2 * 0.04, 1.0)
        self.assertLess(srt.combine_direction_amps([0.02, 0.04], [fwd, bwd]),
                        0.04)


class DampingHelpsTest(unittest.TestCase):
    def test_clear_reduction_is_kept(self):
        self.assertTrue(srt.damping_helps([1000, 1200], [400, 500]))

    def test_worse_or_marginal_is_dropped(self):
        self.assertFalse(srt.damping_helps([1000, 1000], [1100, 900]))
        self.assertFalse(srt.damping_helps([1000], [980]))
        self.assertFalse(srt.damping_helps([1000], [1300]))

    def test_no_bare_vibration(self):
        self.assertFalse(srt.damping_helps([0.0], [0.0]))


class PickResonanceTest(unittest.TestCase):
    def sweep(self, centre, vels=None):
        vels = np.arange(20., 125., 5.) if vels is None else vels
        amps = 100. + 2000. / (1. + ((vels - centre) / 6.) ** 2)
        return vels, amps

    def test_finds_peak_between_samples(self):
        vels, amps = self.sweep(57.3)
        vel, amp, trusted, _ = srt.pick_resonance(vels, amps)
        self.assertTrue(trusted)
        self.assertAlmostEqual(vel, 57.3, delta=1.0)

    def test_peak_at_edge_is_not_trusted(self):
        vels, amps = self.sweep(140.)
        self.assertFalse(srt.pick_resonance(vels, amps)[2])

    def test_flat_sweep_is_not_trusted(self):
        vels = np.arange(20., 125., 5.)
        amps = 500. + 10. * np.sin(vels)
        self.assertFalse(srt.pick_resonance(vels, amps)[2])

    def test_frequency_from_speed(self):
        # C5: 40.4 mm per turn, a CoreXY motor at 45 degrees (ratio sqrt 2)
        f = srt.vel_to_freq(57.13, 40.4, np.sqrt(2.), 2)
        self.assertAlmostEqual(f, 200., delta=0.1)


class CurveTestCommandTest(unittest.TestCase):
    def make(self, res_hz):
        import tempfile
        from unittest import mock
        t = srt.StepperResonanceTester.__new__(srt.StepperResonanceTester)
        t.vel_min, t.vel_max, t.vel_step = 20., 120., 5.
        t.sweep_spacing = srt.SWEEP_SPACING
        t.tdx, t.main_tdx = [1, 2, 4], 2
        t.stepper, t.dir = 'stepper_x', 1
        t.rotation_dis, t.vel_ratio, t.res_freq = 40.4, np.sqrt(2.), 200.
        t.td_config = {'stepper_x': {'freq': 200.}}
        t.prepare_test = mock.Mock()
        t.end_test = mock.Mock()
        t.update_move = mock.Mock()
        self.configfile = mock.Mock()
        t.printer = mock.Mock()
        t.printer.lookup_object.return_value = self.configfile
        self.moves = []

        def measure(gcmd, vel, tdxs):
            self.moves.append(vel)
            out = []
            for tdx in tdxs:
                f = srt.vel_to_freq(vel, 40.4, np.sqrt(2.), tdx)
                if f > srt.MAX_MEASURABLE_HZ:
                    out.append(float('nan'))
                    continue
                # a big response far above the limit would be aliasing noise
                out.append(100. + 2000. / (1. + ((f - res_hz) / 15.) ** 2))
            return out
        t._measure_amps = measure
        self.tmp = tempfile.mkdtemp()
        self._old_dir, srt.CSV_DIR = srt.CSV_DIR, self.tmp + '/'
        self.addCleanup(setattr, srt, 'CSV_DIR', self._old_dir)
        gcmd = mock.Mock()
        gcmd.get_int.side_effect = lambda name, default=None: {'APPLY': 1}.get(name, default)
        gcmd.get.side_effect = lambda name, default=None: {'NAME': 'before'}.get(name, default)
        return t, gcmd

    def test_measures_and_applies_the_resonance(self):
        t, gcmd = self.make(187.)
        t.cmd_STEPPER_RESONANCE_CURVE_TEST(gcmd)
        self.assertAlmostEqual(t.td_config['stepper_x']['freq'], 187., delta=3.)
        self.configfile.set.assert_called_once()
        # coarse sweep (21 speeds) plus a fine one around the peak
        self.assertGreater(len(self.moves), 21)
        info = pathlib.Path(self.tmp, 'vfa_stepper_x_curve_before.info').read_text()
        self.assertIn('trusted=1', info)
        rows = pathlib.Path(self.tmp, 'vfa_stepper_x_curve_before.csv').read_text().splitlines()
        self.assertEqual(rows[0], 'vel,td1_amp,td2_amp,td4_amp')
        t.end_test.assert_called_once()

    def test_keeps_configured_frequency_without_a_clear_peak(self):
        t, gcmd = self.make(900.)  # far outside the sweep
        t.cmd_STEPPER_RESONANCE_CURVE_TEST(gcmd)
        self.assertEqual(t.td_config['stepper_x']['freq'], 200.)
        self.configfile.set.assert_not_called()
        self.assertIn('trusted=0',
                      pathlib.Path(self.tmp, 'vfa_stepper_x_curve_before.info').read_text())


class SweepSpeedsTest(unittest.TestCase):
    def test_proportional_spacing_covers_the_range(self):
        v = srt.sweep_speeds(20., 300.)
        self.assertEqual(v[0], 20.)
        self.assertEqual(v[-1], 300.)
        ratios = [b / a for a, b in zip(v, v[1:])]
        self.assertTrue(all(1.0 < r <= 1.081 for r in ratios))
        self.assertLess(v[1] - v[0], 2.)       # fine at low speed
        self.assertGreater(v[-1] - v[-2], 20.)  # wide at high speed

    def test_zero_spacing_gives_fixed_steps(self):
        self.assertEqual(srt.sweep_speeds(20., 60., 0., 10.), [20., 30., 40., 50., 60.])


class NarrowPeakTest(CurveTestCommandTest):
    # Like the C5's X motor: a narrow 84 Hz peak (2x ripple at 24 mm/s) and a
    # wider, slightly weaker one at 285 Hz (81 mm/s)
    def make_two_peaks(self, spacing, step):
        t, gcmd = self.make(84.4)
        t.vel_min, t.vel_max, t.vel_step = 20., 300., step
        t.sweep_spacing = spacing

        def measure(gcmd_, vel, tdxs):
            self.moves.append(vel)
            out = []
            for tdx in tdxs:
                f = srt.vel_to_freq(vel, 40.4, np.sqrt(2.), tdx)
                if f > srt.MAX_MEASURABLE_HZ:
                    out.append(float('nan'))
                    continue
                out.append(100. + 1650. / (1. + ((f - 84.4) / 3.) ** 2)
                           + 1000. / (1. + ((f - 284.6) / 25.) ** 2))
            return out
        t._measure_amps = measure
        return t, gcmd

    def test_proportional_spacing_finds_the_narrow_peak(self):
        t, gcmd = self.make_two_peaks(srt.SWEEP_SPACING, 10.)
        t.cmd_STEPPER_RESONANCE_CURVE_TEST(gcmd)
        self.assertAlmostEqual(t.td_config['stepper_x']['freq'], 84.4, delta=3.)

    def test_even_fixed_10mm_steps_find_it_as_a_candidate(self):
        # This is what picked 285 Hz on the printer before every candidate
        # was refined
        t, gcmd = self.make_two_peaks(0., 10.)
        t.cmd_STEPPER_RESONANCE_CURVE_TEST(gcmd)
        self.assertAlmostEqual(t.td_config['stepper_x']['freq'], 84.4, delta=3.)


class CandidatesTest(unittest.TestCase):
    def test_local_peaks_strongest_first(self):
        amps = [900, 600, 300, 450, 700, 1100, 800, 500, 650, 400]
        self.assertEqual(srt.resonance_candidates(amps), [5, 0, 8])

    def test_at_most_three(self):
        amps = [1, 5, 1, 6, 1, 7, 1, 8, 1]
        self.assertEqual(srt.resonance_candidates(amps), [7, 5, 3])


class CheckSweepTest(CurveTestCommandTest):
    def test_check_sweep_reports_without_applying(self):
        t, gcmd = self.make(187.)
        gcmd.get_int.side_effect = lambda name, default=None: default
        gcmd.get.side_effect = lambda name, default=None: {'NAME': 'after'}.get(name, default)
        t.cmd_STEPPER_RESONANCE_CURVE_TEST(gcmd)
        self.assertEqual(t.td_config['stepper_x']['freq'], 200.)
        self.configfile.set.assert_not_called()
        said = " ".join(str(c[0][0]) for c in gcmd.respond_info.call_args_list)
        self.assertIn("strongest remaining vibration", said)
        self.assertNotIn("resonance:", said)
        self.assertNotIn("No clear resonance", said)


class FastSweepTest(CurveTestCommandTest):
    def test_sweep_to_300_finds_low_resonance(self):
        t, gcmd = self.make(87.5)
        t.vel_min, t.vel_max, t.vel_step = 20., 300., 10.
        t.cmd_STEPPER_RESONANCE_CURVE_TEST(gcmd)
        self.assertAlmostEqual(t.td_config['stepper_x']['freq'], 87.5, delta=3.)
        rows = pathlib.Path(self.tmp, 'vfa_stepper_x_curve_before.csv').read_text().splitlines()
        last = rows[-1].split(',')
        self.assertEqual(float(last[0]), 300.)
        self.assertNotEqual(last[1], 'nan')   # 1x at 300 mm/s: 525 Hz, measurable
        self.assertEqual(last[2], 'nan')      # 2x: 1050 Hz, not
        self.assertEqual(last[3], 'nan')      # 4x


class MeasureAmpsTest(unittest.TestCase):
    def make(self):
        from unittest import mock
        t = srt.StepperResonanceTester.__new__(srt.StepperResonanceTester)
        t.rotation_dis, t.vel_ratio = 40.4, np.sqrt(2.)
        t.move_accel, t.fast_move_speed, t.fast_move_accel = 5000., 150., 10000.
        t.saved_limits = (30000., 0.5)
        t.move_angle, t.axis, t.stepper = 45., 'A', 'stepper_x'
        t.td_config = {'stepper_x': {'accel_chip': 'lis2dw'}}
        t.gcode = mock.Mock()
        t.do_move_and_accel_measure = mock.Mock(return_value=([], 0., 1.))
        t.calculate_resonance_amps = mock.Mock(
            side_effect=lambda samples, freqs, *a, **k: [10. * (i + 1) for i in range(len(freqs))])
        return t

    def test_unmeasurable_ripples_are_nan_and_not_analysed(self):
        t = self.make()
        amps = t._measure_amps(None, 300., [1, 2, 4])
        self.assertEqual(amps[0], 10.)
        self.assertTrue(np.isnan(amps[1]) and np.isnan(amps[2]))
        freqs = t.calculate_resonance_amps.call_args[0][1]
        self.assertEqual(len(freqs), 1)

    def test_fast_moves_accelerate_harder_then_restore(self):
        t = self.make()
        t._measure_amps(None, 300., [1])
        cmds = [c[0][0] for c in t.gcode.run_script_from_command.call_args_list]
        self.assertEqual(cmds, ["SET_VELOCITY_LIMIT ACCEL=10000.000",
                                "SET_VELOCITY_LIMIT ACCEL=5000.000"])

    def test_slow_moves_keep_the_test_accel(self):
        t = self.make()
        t._measure_amps(None, 50., [1, 2, 4])
        t.gcode.run_script_from_command.assert_not_called()


class StrongResonancesTest(unittest.TestCase):
    def data(self, peaks):
        vels = np.arange(20., 125., 1.)
        amps = 100. + sum(h / (1. + ((vels - c) / w) ** 2) for c, h, w in peaks)
        return vels, amps

    def test_two_strong_peaks_strongest_first(self):
        vels, amps = self.data([(24., 1650., 1.), (81., 1000., 6.)])
        found = srt.strong_resonances(vels, amps, [(20., 30.), (70., 90.)])
        self.assertEqual(len(found), 2)
        self.assertAlmostEqual(found[0][0], 24., delta=0.6)
        self.assertAlmostEqual(found[1][0], 81., delta=0.6)

    def test_weak_second_peak_is_dropped(self):
        vels, amps = self.data([(24., 1650., 1.), (81., 500., 6.)])
        found = srt.strong_resonances(vels, amps, [(20., 30.), (70., 90.)])
        self.assertEqual([round(v) for v, _ in found], [24])

    def test_peak_at_the_edge_is_not_a_resonance(self):
        vels, amps = self.data([(10., 1650., 4.), (81., 1000., 6.)])
        found = srt.strong_resonances(vels, amps, [(20., 25.), (70., 90.)])
        self.assertEqual([round(v) for v, _ in found], [81])

    def test_overlapping_regions_count_once(self):
        vels, amps = self.data([(81., 1000., 6.)])
        found = srt.strong_resonances(vels, amps, [(70., 90.), (75., 95.)])
        self.assertEqual(len(found), 1)

    def test_score_ignores_unmeasurable(self):
        self.assertEqual(srt.sweep_score({20.: [3., 4., float('nan')]}), 25.)


class AutoCalibrateTest(unittest.TestCase):
    # Like the C5's X motor: the 84 Hz peak is stronger, but calibrating at the
    # 285 Hz one leaves less vibration across the speeds, so that one is kept
    def make(self):
        import tempfile
        from unittest import mock
        t = srt.StepperResonanceTester.__new__(srt.StepperResonanceTester)
        t.vel_min, t.vel_max, t.vel_step = 20., 300., 10.
        t.sweep_spacing = srt.SWEEP_SPACING
        t.tdx, t.main_tdx = [1, 2, 4], 2
        t.stepper, t.dir = 'stepper_x', 1
        t.rotation_dis, t.vel_ratio, t.res_freq = 40.4, np.sqrt(2.), 200.
        t.td_config = {'stepper_x': {'freq': 200.}}
        t.update_move = mock.Mock()
        self.configfile = mock.Mock()
        t.printer = mock.Mock()
        t.printer.lookup_object.return_value = self.configfile
        t._set_damping = mock.Mock()
        t._keep_result = mock.Mock()
        t.tuned = None

        def measure(gcmd, vel, tdxs):
            out = []
            for tdx in tdxs:
                f = srt.vel_to_freq(vel, 40.4, np.sqrt(2.), tdx)
                if f > srt.MAX_MEASURABLE_HZ:
                    out.append(float('nan'))
                elif t.tuned is None:
                    out.append(100. + 1650. / (1. + ((f - 84.4) / 3.) ** 2)
                               + 1000. / (1. + ((f - 284.6) / 25.) ** 2))
                else:
                    out.append(300. if t.tuned < 150. else 100.)
            return out
        t._measure_amps = measure

        def search(gcmd):
            t.tuned = t.res_freq
            t.last_results = {2: (0.02 if t.res_freq < 150. else 0.03, 1., 2.)}
            return [[2, 1, 50., t.res_freq, 1000., 50., 1, 0.02, 1.]]
        t._search_calibrate = mock.Mock(side_effect=search)
        tmp = tempfile.mkdtemp()
        old, srt.CSV_DIR = srt.CSV_DIR, tmp + '/'
        self.addCleanup(setattr, srt, 'CSV_DIR', old)
        return t

    def test_keeps_the_calibration_that_leaves_less_vibration(self):
        from unittest import mock
        t = self.make()
        t._auto_calibrate(mock.Mock())
        self.assertEqual(t._search_calibrate.call_count, 2)
        self.assertAlmostEqual(t.td_config['stepper_x']['freq'], 284.6, delta=10.)
        t._keep_result.assert_called_with(2, 0.03, 1., 2.)
        staged = self.configfile.set.call_args_list[-1][0]
        self.assertEqual(staged[:2], ('stepper_resonance_tester', 'stepper_x_freq'))
        self.assertAlmostEqual(float(staged[2]), 284.6, delta=10.)

    def test_checks_both_directions(self):
        from unittest import mock
        t = self.make()
        t._auto_calibrate(mock.Mock())
        dirs = [c[0][1] for c in t.update_move.call_args_list]
        self.assertIn(0, dirs)
        self.assertEqual(dirs[-1], 1)

    def test_graph_reuses_the_winners_check_as_after(self):
        from unittest import mock
        t = self.make()
        moves = []
        orig = t._measure_amps
        t._measure_amps = lambda g, v, tdxs: (moves.append(v), orig(g, v, tdxs))[1]
        t._auto_calibrate(mock.Mock(), graph=True)
        after = pathlib.Path(srt.CSV_DIR, 'vfa_stepper_x_curve_after.csv')
        self.assertTrue(after.exists())
        # no separate after sweep: the last moves are the winner's check
        speeds = srt.sweep_speeds(20., 300.)
        self.assertEqual(moves[-len(speeds):], speeds)
        rows = after.read_text().splitlines()
        self.assertEqual(len(rows), len(speeds) + 1)

    def test_one_resonance_graph_runs_an_after_sweep(self):
        from unittest import mock
        t = self.make()
        orig = t._measure_amps

        def one_peak(gcmd, vel, tdxs):
            if t.tuned is not None:
                return orig(gcmd, vel, tdxs)
            out = []
            for tdx in tdxs:
                f = srt.vel_to_freq(vel, 40.4, np.sqrt(2.), tdx)
                out.append(float('nan') if f > srt.MAX_MEASURABLE_HZ else
                           100. + 1650. / (1. + ((f - 245.) / 20.) ** 2))
            return out
        t._measure_amps = one_peak
        t._auto_calibrate(mock.Mock(), graph=True)
        self.assertTrue(pathlib.Path(srt.CSV_DIR, 'vfa_stepper_x_curve_after.csv').exists())

    def test_one_strong_resonance_calibrates_once(self):
        from unittest import mock
        t = self.make()
        orig = t._measure_amps

        def one_peak(gcmd, vel, tdxs):
            if t.tuned is not None:
                return orig(gcmd, vel, tdxs)
            out = []
            for tdx in tdxs:
                f = srt.vel_to_freq(vel, 40.4, np.sqrt(2.), tdx)
                out.append(float('nan') if f > srt.MAX_MEASURABLE_HZ else
                           100. + 1650. / (1. + ((f - 245.) / 20.) ** 2))
            return out
        t._measure_amps = one_peak
        t._auto_calibrate(mock.Mock())
        self.assertEqual(t._search_calibrate.call_count, 1)
        self.assertAlmostEqual(t.td_config['stepper_x']['freq'], 245., delta=8.)


class PhaseProbeRetryTest(unittest.TestCase):
    # A ripple of size D, cancelled by a test strength of D/K at phase pi
    def run_search(self, bare, k=48485.):
        from unittest import mock
        t = srt.StepperResonanceTester.__new__(srt.StepperResonanceTester)
        t.stepper, t.vel = 'stepper_y', None
        t.res_freq, t.rotation_dis, t.vel_ratio = 244., 40.4, np.sqrt(2.)
        t.dir_inverted = False
        t.td_amp_initial = [0.01, 0.02, 0.01]
        t.initial_trial_amp = 0.05
        t.min_distortion_threshold = 10.
        t.update_move = mock.Mock()
        t.move_angle = 45.
        amps = []

        def measure(gcmd, vel, tdx, direction, amp, phase):
            amps.append(round(amp, 4))
            return abs(bare + k * amp * np.exp(1j * phase))
        t._apply_comp_and_measure = measure
        gcmd = mock.Mock()
        amp, phase, history, fit = t.search_calibrate_one_direction(gcmd, 1, 1)
        probes = [round(h[1], 4) for h in history if h[0] == 1]
        return amp, phase, probes, gcmd

    def test_small_ripple_gets_a_smaller_probe(self):
        # bare 160: a 0.01 probe overshoots at every phase, 0.002 doesn't
        amp, phase, probes, _ = self.run_search(160.)
        self.assertEqual(probes, [0.01] * 4 + [0.002] * 4)
        self.assertAlmostEqual(amp, 160. / 48485., delta=0.0006)
        self.assertAlmostEqual(phase, np.pi, delta=0.15)

    def test_no_repeat_at_the_same_strength(self):
        # bare 50: even the smallest probe overshoots. It says so and carries
        # on, instead of probing again at the same strength
        amp, phase, probes, gcmd = self.run_search(50.)
        self.assertEqual(probes, [0.01] * 4 + [0.002] * 4)
        said = " ".join(str(c[0][0]) for c in gcmd.respond_info.call_args_list)
        self.assertIn("already the smallest", said)
        self.assertNotIn("Reducing 0.0020 -> 0.0020", said)


class ResonanceAmpWindowTest(unittest.TestCase):
    def setUp(self):
        self.tester = srt.StepperResonanceTester.__new__(
            srt.StepperResonanceTester)

    def samples(self):
        fs = 1600.0
        t = np.arange(0, 3.0, 1 / fs)
        accel = np.zeros_like(t)
        # big spikes at the move's ends, a steady 200 Hz tone of 50 between
        accel[(t > 0.5) & (t < 0.52)] = 5000.0
        accel[(t > 2.48) & (t < 2.5)] = -5000.0
        cruise = (t > 0.6) & (t < 2.4)
        accel[cruise] += 50.0 * np.sin(2 * np.pi * 200.0 * t[cruise])
        return np.column_stack([t, accel, np.zeros_like(t), np.zeros_like(t)])

    def test_window_measures_the_steady_tone(self):
        amp = self.tester.calculate_resonance_amps(
            self.samples(), [200.0], 0.0, 'X', window=(0.7, 2.3))[0]
        self.assertAlmostEqual(amp, 50.0, delta=2.5)

    def test_short_window_falls_back(self):
        amp = self.tester.calculate_resonance_amps(
            self.samples(), [200.0], 0.0, 'X', window=(1.0, 1.01))
        self.assertIsNotNone(amp)


if __name__ == "__main__":
    unittest.main()
