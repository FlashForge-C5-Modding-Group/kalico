# Creator 5 X/Y motor tuning

The mainboard runs the X/Y/Z motor current loops itself (`src/c5_mclib.c`). These commands are for
testing changes on a real printer. Nothing here changes behaviour until it is turned on.

## Feedforward

At speed, the motor's back-EMF and inductance push the current off its target. Without feedforward,
the current PI has to catch up every time the speed changes, so the current lags during acceleration
and deceleration. `MCLIB_SET_FEEDFORWARD` adds the voltage those effects need up front, worked out
from the commanded speed, so the PI only corrects what's left.

It is off by default (`feedforward: 0`). To try it:

1. Run some constant-speed moves (for example 150 mm/s back and forth on X), then
   `MCLIB_TIMING STEPPER=stepper_x`. Note the `d-axis PI integral avg`.
2. `MCLIB_SET_FEEDFORWARD STEPPER=stepper_x GAIN=100` (and the same for `stepper_y`), repeat the
   same moves, and query again.
3. If the integral drops a lot, feedforward is doing its job. If it roughly doubles, the sign is
   wrong for this motor: set `GAIN=0` and report it.
4. Listen to the moves and print the same test with feedforward off and on.

To keep it, add `feedforward: 100` (or the gain that worked best) under `[mclib stepper_x]` and
`[mclib stepper_y]`. Z ignores it.

## Microsteps

The crossover into closed-loop fast mode, the cutoff for stock resonance damping, and the slow-speed
threshold now follow the configured `microsteps`. At the default of 16, nothing changes. At 32 or 64
they stay at the same motor speeds, so higher microstep settings can be tested without moving those
switch points. Recalibrate ripple correction after changing microsteps.

## Injection scale

`MCLIB_SET_INJECTION` scales the observer's sliding-mode correction term. That term is internal to
the position estimate and never reaches the motor, so lowering it does not make the motors quieter.
It only makes the angle estimate (used in fast mode and for stall and slip detection) worse. Leave
it at 100 outside of bench tests.

## Measuring ripple

The measurement is adapted from the U1 phase stepping port's calibration
([justinh-rahb/u1-klipper](https://github.com/justinh-rahb/u1-klipper), `phase_calibrate.py` and
`motor_harmonics.py`):

- Each measurement is a move out and back from the same start, repeated `PASSES` times (default 2,
  `passes` in `[mclib_ripple]`). Every pass starts at the same motor angle, so each harmonic is taken as a
  complex amplitude in phase with the electrical angle and averaged. The motor's own vibration adds up and
  vibration from anything else averages out.
- Results are displacement in nm, from all three accelerometer axes, with a noise level measured at the
  half orders either side of each harmonic (h-0.5, h+0.5), where no motor harmonic falls.
- Harmonics above 40% of the accelerometer's sample rate (640 Hz at 1600 Hz) aren't measured.

Commands:

| Command | |
| --- | --- |
| `MCLIB_RIPPLE_MEASURE STEPPER=stepper_x SPEED=80 [PASSES=3]` | Every harmonic at one motor speed, with its noise level. |
| `MCLIB_RIPPLE_SWEEP STEPPER=stepper_x [SPEED_MIN=10 SPEED_MAX=150 SPEED_STEP=10] [HARMONICS=1,2,3,4] [PASSES=1]` | Every harmonic over a range of speeds; prints each harmonic's peak above 3x noise and writes all of it to `/tmp/mclib_ripple_<stepper>_<time>.csv`. The default range is `sweep` in `[mclib_ripple]`. |
| `MCLIB_RIPPLE_CALIBRATE STEPPER=stepper_x HARMONICS=2,4,6,8 SPEEDS=AUTO` | Calibrate at the speeds where each harmonic shows. |

With `SPEEDS=AUTO`, calibration clears the harmonics it calibrates, sweeps, and for each harmonic picks up to 5
speeds where it is at least 5x its noise level and 5 nm in both directions, spread over its peaks. A
harmonic that's in the noise everywhere keeps its old table. `SPEEDS=8,15,30` still calibrates at fixed speeds.

At each speed the vibration responds linearly to the correction, so it is measured with the correction as it
is and with two small probes, and the correction that cancels it follows directly. A probe that doesn't move
the vibration to at least twice its noise level means the harmonic can't be corrected there, and that
speed is skipped. Two more measurements refine it; a speed's correction is kept only if it takes off at
least 20%. In reverse the electrical angle runs backwards, so it is followed backwards there; otherwise the
response depends on the conjugate of the correction and the probes cancel out.

A point that isn't kept in one direction is left at zero, which also turns off the stock damping there. So
once a harmonic is done, its new table and the table it replaces are measured at the same speeds, and the new
one is only kept if it leaves at least 10% less. On the printer used here, `SPEEDS=AUTO` on X for h2, 4, 6 and
8 kept the old h2 table (new 1128 nm against 816 nm, summed over its speeds and both directions) and replaced
h4 (534 to 362 nm), h6 (204 to 128 nm) and h8 (180 to 86 nm).

Before this, measurements were power at each harmonic from a single move and varied by up to 2x between runs.
Now three runs of `PASSES=3` at 80 mm/s agree within about 10% for harmonics well above noise.

h1 usually sits in the noise: at its low frequency, frame resonances (55-80 Hz on the Creator 5 Pro used
here) and accelerometer noise turn into a lot of displacement.

## Current sense noise

`MCLIB_TIMING` also reports the current error while the motor holds still (`current noise while holding`),
the noise in the current measurement. Query it once to reset, wait a few seconds with the motor enabled and
still, and query again. Here it is about 1.5 mA RMS at 1.8 A, so ADC noise isn't a source of motor noise.
The first query after moves includes the settling at the end of the move, so it reads higher.

It also shows, while the motor moves, how long each control update takes, the longest gap between two
updates (one control period is 50 us at 20 kHz), how many updates came a period or more late, the longest
update counting only its own time, and the most a single update lost to each interrupt that can preempt it
(serial, Klipper's timer and the 100 Hz stall tick). Measured on a Creator 5 Pro with fast X/Y moves at
600 mm/s: about 2,800 cycles per update on X, 2,400 on Y and 1,500 on Z (600 MHz), so the three motors use about
a fifth of each control period. The longest update is 9 us, the longest gap 53 us, and no update is late.

The control path (the motor interrupts and everything they call) runs from RAM (`C5_FAST`, the `.ramfunc`
section). From flash, an update whose code had dropped out of the cache took up to 28 us while moving and
136 us after boot, and lost control periods. The step hooks stay in flash: in RAM they stop being inlined into
Klipper's timer dispatch, and steps then took up to 18 us out of single control updates.

## Stepping

- Each step is timed from its scheduled clock, not from when the timer interrupt got to it, so the step
  periods the interpolation and the fast-mode switch use don't pick up interrupt latency.
- With firmware that declares `C5_MAINBOARD_STEP_BOTH_EDGE`, `[mclib]` asks for step-on-both-edges on its
  stepper, so a step is one timer event instead of a step and an unstep. Older firmware counts only rising
  edges, and the host then keeps the step and unstep events. Host and firmware need to match: firmware without
  the constant on a host that forced both edges would move half the distance.
- The mainboard firmware is built for the hard-float ABI (`-mfloat-abi=hard`), which made the control update
  about 5% faster than `softfp`.
- Between steps the motor angle moves linearly to the next step's scheduled time. `interpolation_mode:
  quadratic` in `[mclib]` (or `MCLIB_SET_INTERPOLATION STEPPER=... MODE=quadratic`) instead follows the parabola
  through the previous, current and next step, which is exact at constant acceleration with perfect step times.
  It measured about 9% more toolhead vibration at 50-200 Hz in short accelerating moves (two runs each,
  repeatable): Klipper's step times carry up to 25 us of compression error, and the quadratic term, taken from
  the ratio of two step periods, amplifies it. Linear is the default.

## Zero-crossing compensation (tried, not kept)

Adding a voltage in the direction of each phase's current near its zero crossing (dead-time compensation)
made h4 and h8 worse in proportion to the voltage: 0.3 V raised h4 from 20-110 to about 250, 1.5 V to about
1700. Before it, h4 differed between directions, which dead-time loss wouldn't do. The current loop already
corrects the bridge's own error at these frequencies, so this was removed.

## Homing speed

Homing is one long constant-speed move, so its speed decides what it sounds like. `MCLIB_RIPPLE_SWEEP` on both
motors from 30 to 120 mm/s, with h2-h8 added up as acceleration (closest to how loud it is), on the printer used
here:

| mm/s | 30 | 40 | 50 | 60 | 70 | 80 | 90 | 100 | 110 | 120 |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| h2-h8, mm/s^2 | 243 | 341 | 278 | 289 | 294 | **394** | 285 | 320 | 385 | 363 |
| h1, nm | 636 | 1037 | **2446** | 963 | 715 | 346 | 261 | 191 | 326 | 748 |

The stock 80 mm/s is the loudest of these, and 40-60 mm/s put h1 on the frame resonance (55-80 Hz). 90 mm/s is
low on both, so `homing_speed` is 90 here. The endstops are switches, so the speed doesn't change where they
trigger. `second_homing_speed` stays 10.

## Skipped steps at low speed and holding still

Skipped steps come from the gap between the commanded angle and the observer's rotor angle. The observer
gets that angle from the motor's back-EMF, which is close to zero when holding still or moving slowly.
There, what it sees is mostly the bridge's dead time and the difference between the configured and real
coil resistance. That vector can be small, so its angle wanders. With `slip_watch: holding` or `always`
this counted thousands of false slips (a 10% resistance mismatch was enough), and slow moves around
10-15 mm/s gained a false cycle every few tenths of a second.

Below the fast-move speed the firmware now:

- learns that standstill part in the command frame (about 0.1 s, only from samples close to it, so a slip
  in progress isn't learned) and takes it out, along with the back-EMF expected at the commanded speed,
- follows the angle of what is left (low-passed at about 130 Hz, or unfiltered above 1 V so a rotor
  running away is still followed) only while it is over 0.25 V, and keeps the reference through the gaps,
- takes whichever of the two possible angles (back-EMF flips with the rotor's direction) is nearer the
  last, and skips samples where the angle turns faster than the rotor speed that back-EMF means,
- lets the drift off the nearest whole cycle fade with a 1 s time constant, so the small offsets left
  between fast moves and slow parts can't add up over a print. A real slip takes a few tens of ms.

Fast moves use the observer's angle as before.

In a simulated motor (dead time 0.1-0.6 V, resistance 80-130% of the configured value, 3-8 mA sense
noise), 24 runs of 2 minutes of print-like X and Z moves with `slip_watch: always` counted no slips, where
the old code counted up to 17,000. Pushes that made the rotor slip 1-3 cycles while holding or at 8-30 mm/s
counted exactly in most cases. Larger runaways count fewer than the real number, but always trip.

`slip_watch` now applies during prints as well, not only in test mode. It stays `moving` by default until
`holding`/`always` have run clean on real prints.
