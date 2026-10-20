# Creator 5 pressure advance from extruder-load drift
#
# This file may be distributed under the terms of the GNU GPLv3 license.
#
# Pressure advance assumes nozzle pressure trails the extruder motor by a
# first-order lag; the right advance is that lag.  Once a fast stroke's
# speed-up (including the advance's own overshoot) is over, the motor turns
# at a constant speed, so any change in its load reading can only be the
# pressure still settling.  Too little advance leaves pressure building
# (load rising) and too much leaves it falling, so the early-minus-late
# difference of the reading crosses zero at the right advance.  Each tested
# advance gives one such difference per stroke; a line through them finds the
# crossing.

import math

SETTLE = .03          # s from a stroke's start to its first counted reading
EARLY_LEN = .06       # s
RIPPLE_HARMONICS = 4

MIN_STROKE = .3       # s
MIN_T_STAT = 4.
MAX_INTERVAL = .03


def strokes(t, v, fast):
    """(start, end) of each stretch commanded faster than half of FAST."""
    out, start = [], None
    for ti, vi in zip(t, v):
        if vi > fast / 2.:
            if start is None:
                start = ti
            last = ti
        elif start is not None:
            if last - start >= MIN_STROKE:
                out.append((start, last))
            start = None
    if start is not None and last - start >= MIN_STROKE:
        out.append((start, last))
    return out


def stroke_drifts(t, value, v, e, fast, period, skip=1):
    """(early - late) / late of each fast stroke after the first SKIP.

    The windows sit at fixed times from each stroke's start, so every
    advance's overshoot ends the same way relative to them.  The reading's
    ripple with the motor's electrical cycle (PERIOD mm of filament) is
    fitted across the run's strokes and removed.
    """
    import numpy as np
    t, y, v, e = (np.asarray(x, float) for x in (t, value, v, e))
    used = strokes(t, v, fast)[skip:]
    if not used:
        return []
    offsets, earlies, sel = [], [], np.zeros(len(t), bool)
    for a, b in used:
        window = (t >= a + SETTLE) & (t <= b - .02)
        sel |= window
        offsets.append(window.astype(float))
        earlies.append(((t >= a + SETTLE)
                        & (t <= a + SETTLE + EARLY_LEN)).astype(float))
    ripple = []
    if period:
        for k in range(1, RIPPLE_HARMONICS + 1):
            phase = 2. * math.pi * k * e / period
            ripple += [np.cos(phase), np.sin(phase)]
    x = np.column_stack(offsets + earlies + ripple)[sel]
    coef = np.linalg.lstsq(x, y[sel], rcond=None)[0]
    n = len(used)
    # Offsets are the late level; the early columns are early - late.
    return [float(coef[n + i] / coef[i]) for i in range(n) if coef[i] > 0.]


class Fit:
    def __init__(self, advance, se, t_stat, points, tested):
        self.advance = advance
        self.low = advance - 1.96 * se
        self.high = advance + 1.96 * se
        self.t_stat = t_stat
        self.points = points
        self.tested = tested

    def problem(self):
        """Why the result should not be used, or None."""
        if abs(self.t_stat) < MIN_T_STAT:
            return 'load does not respond to pressure advance (t=%.1f)' % (
                self.t_stat,)
        if self.t_stat > 0:
            return 'load drifts the wrong way as pressure advance rises'
        lo, hi = min(self.tested), max(self.tested)
        if not lo - .02 <= self.advance <= hi + .02:
            return 'result %.4f is outside the tested range' % self.advance
        if self.high - self.low > MAX_INTERVAL:
            return 'too uncertain (%.4f-%.4f)' % (self.low, self.high)
        return None


def fit_zero(points):
    """Line through (advance, drift) points; where it crosses zero.

    A stroke now and then reads far off (a missed reading burst, a bubble);
    up to a tenth of the points further than four robust standard
    deviations from the line are dropped before the final fit.
    """
    fit = _fit_line(points)
    for _ in range(2):
        resid = [(abs(d - fit.icpt - fit.slope * a), (a, d))
                 for a, d in fit.points]
        spread = 1.4826 * sorted(r for r, _ in resid)[len(resid) // 2]
        if not spread:
            break
        far = sorted((r for r in resid if r[0] > 4. * spread), reverse=True)
        far = far[:len(points) // 10]
        if not far:
            break
        drop = set(id(p) for _, p in far)
        fit = _fit_line([p for _, p in resid if id(p) not in drop])
    fit.dropped = len(points) - len(fit.points)
    return fit


def _fit_line(points):
    n = len(points)
    tested = sorted(set(a for a, _ in points))
    if n < 4 or len(tested) < 2:
        raise ValueError('need drifts at two or more pressure advances')
    mx = sum(a for a, _ in points) / n
    my = sum(d for _, d in points) / n
    sxx = sum((a - mx) ** 2 for a, _ in points)
    sxy = sum((a - mx) * (d - my) for a, d in points)
    slope = sxy / sxx
    if not slope:
        raise ValueError('drift does not change with pressure advance')
    icpt = my - slope * mx
    sse = sum((d - icpt - slope * a) ** 2 for a, d in points)
    s2 = sse / (n - 2) if n > 2 else 0.
    var_slope = s2 / sxx
    var_icpt = s2 * (1. / n + mx * mx / sxx)
    cov = -mx * s2 / sxx
    advance = -icpt / slope
    # Delta method for -icpt/slope.
    var = (var_icpt + advance * advance * var_slope
           + 2. * advance * cov) / (slope * slope)
    t_stat = slope / math.sqrt(var_slope) if var_slope > 0 else (
        -math.inf if slope < 0 else math.inf)
    fit = Fit(advance, math.sqrt(max(var, 0.)), t_stat, points, tested)
    fit.slope, fit.icpt = slope, icpt
    return fit
