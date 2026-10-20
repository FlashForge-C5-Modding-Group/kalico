# Creator 5 pressure advance from a flow-test sensor trace
#
# This file may be distributed under the terms of the GNU GPLv3 license.
#
# Klipper's pressure advance assumes nozzle flow trails the extruder motor by
# a first-order lag whose time constant is the pressure advance itself.  The
# melt pressure follows that lagged flow, so a sensor that feels it reads
#
#     value(t) = run offset + g(|v|) + k * a + c * q,  all at t - delay
#
# with v and a the commanded motor velocity and acceleration, q = v through a
# first-order lag of time constant tau, g the sensor's own response to motor
# speed (a cubic), k the rotor's inertia and delay the sensor's read
# latency.  For each (delay, tau) on a
# grid the rest is linear least squares; the tau that explains the trace best
# is the pressure advance, whatever advance the strokes themselves ran with.

import math

MAX_SPEED = 40.
DELAYS = tuple(i * .004 for i in range(7))      # 0-24 ms
COARSE_TAUS = tuple(i * .005 for i in range(41))  # 0-0.2 s
FINE_STEP = .0005
MIN_T_STAT = 6.
MAX_INTERVAL = .02


class Run:
    """One stroke sequence.

    GRID_T/GRID_V: commanded motor velocity on a uniform time grid.
    SAMPLE_T/SAMPLE_VALUE: sensor readings (unread ones already dropped).
    """
    def __init__(self, grid_t, grid_v, sample_t, sample_value):
        self.grid_t = grid_t
        self.grid_v = grid_v
        self.sample_t = sample_t
        self.sample_value = sample_value


def _lag(np, v, dt, tau):
    if tau <= 0.:
        return v.copy()
    alpha = 1. - math.exp(-dt / tau)
    q = np.empty_like(v)
    acc = v[0]
    for i, value in enumerate(v):
        acc += alpha * (value - acc)
        q[i] = acc
    return q


def _speed_basis(np, speed):
    """Cubic in |v| (offsets are per run).  Speeds above the strokes' own
    only occur while pressure advance accelerates, so a flexible curve would
    soak up the lag it is meant to leave for the flow term."""
    s = np.minimum(speed, MAX_SPEED) / 10.
    return [s, s * s, s * s * s]


class Estimator:
    def __init__(self, runs, inertia=None):
        self.inertia = inertia
        self._inertia_part = 0.
        import numpy as np
        self.np = np
        self.runs = [r for r in runs if len(r.sample_t) > 20]
        if len(self.runs) < 2:
            raise ValueError('not enough sensor samples')
        self.y = np.concatenate([np.asarray(r.sample_value, float)
                                 for r in self.runs])
        offsets = []
        for i, r in enumerate(self.runs):
            col = np.zeros(len(self.y))
            start = sum(len(o.sample_t) for o in self.runs[:i])
            col[start:start + len(r.sample_t)] = 1.
            offsets.append(col)
        self.offsets = offsets
        self.grids = []
        for r in self.runs:
            gt = np.asarray(r.grid_t, float)
            gv = np.asarray(r.grid_v, float)
            self.grids.append((gt, gv, np.gradient(gv, gt)))
        self._lags = {}

    def _columns(self, delay, tau):
        np = self.np
        speeds, accels, flows = [], [], []
        for i, (r, (gt, gv, ga)) in enumerate(zip(self.runs, self.grids)):
            key = (i, tau)
            if key not in self._lags:
                self._lags[key] = _lag(np, gv, gt[1] - gt[0], tau)
            t = np.asarray(r.sample_t, float) - delay
            speeds.append(np.abs(np.interp(t, gt, gv)))
            accels.append(np.interp(t, gt, ga))
            flows.append(np.interp(t, gt, self._lags[key]))
        basis = _speed_basis(np, np.concatenate(speeds))
        if self.inertia is not None:
            # Known rotor inertia: remove it rather than fit it, since a free
            # acceleration term is indistinguishable from the lag itself.
            self._inertia_part = self.inertia * np.concatenate(accels)
        return basis, np.concatenate(flows)

    def fit(self, delay, tau):
        """(sse, flow coefficient, its t statistic) at DELAY, TAU."""
        np = self.np
        basis, flow = self._columns(delay, tau)
        x = np.column_stack(self.offsets + basis + [flow])
        y = self.y - self._inertia_part
        coef, _, rank, _ = np.linalg.lstsq(x, y, rcond=None)
        resid = y - x @ coef
        sse = float(resid @ resid)
        # Neighbouring readings are not independent (and model error is
        # smooth), so count the samples the residuals' correlation leaves.
        rho = float(resid[1:] @ resid[:-1]) / sse if sse else 0.
        rho = min(max(rho, 0.), .99)
        dof = max(len(self.y) * (1. - rho) / (1. + rho) - x.shape[1], 1.)
        cov = np.linalg.pinv(x.T @ x) * (sse / dof)
        t_stat = coef[-1] / math.sqrt(cov[-1, -1]) if cov[-1, -1] > 0 else 0.
        return sse, float(coef[-1]), float(t_stat), dof

    def estimate(self):
        best = None
        for delay in DELAYS:
            for tau in COARSE_TAUS:
                sse = self.fit(delay, tau)[0]
                if best is None or sse < best[0]:
                    best = (sse, delay, tau)
        _, delay, coarse = best
        lo = max(0., coarse - COARSE_TAUS[1])
        hi = min(COARSE_TAUS[-1], coarse + COARSE_TAUS[1])
        taus = [lo + i * FINE_STEP
                for i in range(int(round((hi - lo) / FINE_STEP)) + 1)]
        profile = [(tau,) + self.fit(delay, tau) for tau in taus]
        tau, sse, coef, t_stat, dof = min(profile, key=lambda p: p[1])
        # Taus whose fit is within the 95% profile-likelihood band.
        limit = sse * (1. + 3.84 / dof)
        inside = [p[0] for p in profile if p[1] <= limit]
        return Result(tau, delay, coef, t_stat, min(inside), max(inside),
                      coarse in (COARSE_TAUS[0], COARSE_TAUS[-1]))


class Result:
    def __init__(self, advance, delay, coef, t_stat, low, high, at_edge):
        self.advance = advance
        self.delay = delay
        self.coef = coef
        self.t_stat = t_stat
        self.low = low
        self.high = high
        self.at_edge = at_edge

    def problem(self):
        """Why the result should not be used, or None."""
        if abs(self.t_stat) < MIN_T_STAT:
            return 'no clear pressure response (t=%.1f)' % self.t_stat
        if self.at_edge:
            return 'best fit at the edge of the searched range'
        if self.high - self.low > MAX_INTERVAL:
            return 'too uncertain (%.4f-%.4f)' % (self.low, self.high)
        return None


def estimate(runs, inertia=None):
    return Estimator(runs, inertia).estimate()
