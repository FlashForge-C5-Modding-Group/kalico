import importlib.util
import math
import random
import unittest
from pathlib import Path

MODULE_PATH = (Path(__file__).resolve().parents[1] / 'klippy' / 'extras'
               / 'c5_pa_analysis.py')
SPEC = importlib.util.spec_from_file_location('c5_pa_analysis', MODULE_PATH)
pa = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(pa)

DT = .002
FAST = 10.4


PERIOD = .0589


def make_run(advance, true_tau, strokes=6, gain=-2., noise=4., seed=1,
             ripple=10.):
    """Slow 1 mm/s with fast plateaus; the reading follows speed, minus a
    load proportional to the nozzle flow that lags the motor by TRUE_TAU."""
    rng = random.Random(seed)
    accel = 750.
    edges, t = [], .4
    for _ in range(strokes):
        edges.append(t)
        t += .5 + .4
    end = t
    grid_t = [i * DT for i in range(int(end / DT))]

    def nominal(t):
        for start in edges:
            ramp = (FAST - 1.) / accel
            if start <= t < start + ramp:
                return 1. + accel * (t - start)
            if start + ramp <= t < start + .5 - ramp:
                return FAST
            if start + .5 - ramp <= t < start + .5:
                return FAST - accel * (t - start - .5 + ramp)
        return 1.
    vn = [nominal(t) for t in grid_t]
    grid_v = [v + advance * (vn[min(i + 1, len(vn) - 1)] - vn[max(i - 1, 0)])
              / (2 * DT) for i, v in enumerate(vn)]
    alpha = 1. - math.exp(-DT / true_tau)
    flow, acc = [], 1.
    for v in grid_v:
        acc += alpha * (v - acc)
        flow.append(acc)
    e, pos = [], 0.
    for v in grid_v:
        pos += v * DT
        e.append(pos)
    values = [30. + 300. * (1. - math.exp(-abs(v) / 12.)) + gain * q
              + ripple * math.sin(2. * math.pi * p / PERIOD)
              + rng.gauss(0., noise) for v, q, p in zip(grid_v, flow, e)]
    return grid_t, values, grid_v, e


def points_for(advances, true_tau, **kw):
    points = []
    for i, a in enumerate(advances):
        run = make_run(a, true_tau, seed=i, **kw)
        points.extend((a, d) for d in pa.stroke_drifts(*run, fast=FAST,
                                                        period=PERIOD))
    return points


class DriftTest(unittest.TestCase):
    def test_finds_strokes(self):
        t, _, v, _ = make_run(0., .04)
        self.assertEqual(len(pa.strokes(t, v, FAST)), 6)

    def test_drift_changes_sign_around_true_lag(self):
        low = pa.stroke_drifts(*make_run(0., .04), fast=FAST, period=PERIOD)
        high = pa.stroke_drifts(*make_run(.08, .04), fast=FAST,
                                period=PERIOD)
        self.assertEqual(len(low), 5)
        self.assertTrue(all(d > 0 for d in low))
        self.assertTrue(all(d < 0 for d in high))

    def test_recovers_lag(self):
        for true_tau in (.02, .04, .06):
            fit = pa.fit_zero(points_for((0., .05, .10), true_tau))
            self.assertAlmostEqual(fit.advance, true_tau, delta=.006)
            self.assertIsNone(fit.problem())

    def test_ripple_is_removed(self):
        fit = pa.fit_zero(points_for((0., .05, .10), .04, ripple=40.))
        self.assertAlmostEqual(fit.advance, .04, delta=.006)

    def test_single_bad_stroke_is_dropped(self):
        points = points_for((0., .05, .10), .04)
        points.append((.10, -.16))
        fit = pa.fit_zero(points)
        self.assertEqual(fit.dropped, 1)
        self.assertAlmostEqual(fit.advance, .04, delta=.006)

    def test_rejects_unresponsive_load(self):
        fit = pa.fit_zero(points_for((0., .05, .10), .04, gain=0.))
        self.assertIsNotNone(fit.problem())

    def test_needs_two_advances(self):
        with self.assertRaises(ValueError):
            pa.fit_zero([(.01, .1), (.01, .2), (.01, .3), (.01, .4)])


if __name__ == '__main__':
    unittest.main()
