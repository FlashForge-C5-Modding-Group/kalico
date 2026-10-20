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


def nominal_velocity(t, accel=284.):
    """Slow 1 mm/s with two fast 10.4 mm/s strokes, trapezoidal edges."""
    def stroke(t, start, length=.22, high=10.4):
        ramp = (high - 1.) / accel
        if t < start or t > start + length + 2 * ramp:
            return 0.
        if t < start + ramp:
            return accel * (t - start)
        if t < start + ramp + length:
            return high - 1.
        return high - 1. - accel * (t - start - ramp - length)
    if t < 0. or t > 3.:
        return 0.
    return 1. + stroke(t, 1.) + stroke(t, 2.)


def make_run(advance, true_tau, delay=.008, coef=-3., noise=5., seed=1):
    rng = random.Random(seed)
    grid_t = [-.3 + i * DT for i in range(int(3.6 / DT))]
    vn = [nominal_velocity(t) for t in grid_t]
    # Motor velocity with pressure advance (Klipper's v + advance * dv/dt).
    grid_v = [v + advance * (vn[min(i + 1, len(vn) - 1)] - vn[max(i - 1, 0)])
              / (2 * DT) for i, v in enumerate(vn)]
    alpha = 1. - math.exp(-DT / true_tau)
    flow, acc = [], 0.
    for v in grid_v:
        acc += alpha * (v - acc)
        flow.append(acc)
    sample_t, sample_value, t = [], [], -.25
    while t < 3.2:
        i = int((t - delay - grid_t[0]) / DT)
        speed = abs(grid_v[i])
        value = (30. + 300. * (1. - math.exp(-speed / 12.)) + coef * flow[i]
                 + rng.gauss(0., noise))
        sample_t.append(t)
        sample_value.append(value)
        t += .0077 + rng.uniform(-.001, .001)
    return pa.Run(grid_t, grid_v, sample_t, sample_value)


class EstimatorTest(unittest.TestCase):
    def test_recovers_lag_and_delay(self):
        runs = [make_run(a, .045, seed=i)
                for i, a in enumerate((0., .04, .08, 0., .04, .08))]
        result = pa.estimate(runs)
        self.assertAlmostEqual(result.advance, .045, delta=.007)
        self.assertAlmostEqual(result.delay, .008, delta=.0041)
        self.assertLessEqual(result.low, .045)
        self.assertGreaterEqual(result.high, .045)
        self.assertIsNone(result.problem())

    def test_result_not_limited_to_tested_advances(self):
        runs = [make_run(a, .06, seed=i) for i, a in enumerate((0., .04, .08))]
        self.assertAlmostEqual(pa.estimate(runs).advance, .06, delta=.007)

    def test_known_inertia_is_removed(self):
        runs = [make_run(a, .045, seed=i) for i, a in enumerate((0., .04, .08))]
        self.assertAlmostEqual(pa.estimate(runs, inertia=0.).advance, .045,
                               delta=.007)

    def test_rejects_trace_without_pressure_term(self):
        runs = [make_run(a, .045, coef=0., seed=i)
                for i, a in enumerate((0., .04, .08))]
        self.assertIsNotNone(pa.estimate(runs).problem())

    def test_needs_samples(self):
        with self.assertRaises(ValueError):
            pa.estimate([pa.Run([0., DT], [0., 0.], [0.], [1.])])


if __name__ == '__main__':
    unittest.main()
