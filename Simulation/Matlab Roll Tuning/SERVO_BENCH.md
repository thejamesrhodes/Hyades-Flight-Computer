# Measuring the BMS-127WV+ — what to do, in order

Everything below is measurable in an afternoon with parts you already have.
The one rule that matters: **put the angle sensor on the canard, not on the
servo horn.** Backlash lives between them, so a horn-mounted sensor cannot see
the quantity you most need.

## Rig

| item | note |
|---|---|
| angle sensor | **AS5048A** (SPI, 14-bit = 0.022°) on the canard shaft with a diametric magnet. AS5600 (I²C, 12-bit) works for the static tests but is too slow for chirps. |
| driver | your own H743 at the real 250 Hz frame — do not use a hobby servo tester, its timing is not yours |
| supply | the actual flight battery/BEC at the actual voltage. Both torque and slew scale with V, and a sagging BEC changes every number here |
| load | a leaf-spring or torsion bar sized to give the max-q hinge moment at full deflection. Size it from the bench's `hinge N·m` column |
| logging | ≥ 1 kHz, command and measurement on the same timebase |

## 1. Backlash — 10 minutes, highest value of anything here

Command a fixed position and leave it. Push the canard by hand to one flank,
record; push to the other, record. The difference is total lash. Repeat at
five or six commanded positions and in both directions.

Do **not** try to get this from a hysteresis loop — a slow triangular command
gives you a loop whose width is deadband *and* lash *and* a bit of phase lag
mixed together (the model gives 0.73° for a 0.21° deadband and 0.15° lash).
Two nonlinearities, two separate tests.

## 2. Dead zone — 20 minutes

Ramp the command in single-microsecond steps and log the canard angle. The
output moves in discrete jumps; the command increment between jumps is the
dead zone. Repeat with the spring load fitted — the dead zone referred to
position grows with load, because the servo must break friction as well as
close its own error.

## 3. Rotational stiffness K_δ — 20 minutes

Command a fixed position. Hang known masses on a lever arm on the canard shaft
and record deflection against applied torque. Slope = K_δ.

Take it **twice**: once starting from rest inside the dead zone, and once after
a small deflection has already engaged the loop. They differ by a large factor,
which is the point — the divergence calculation assumes a single linear spring
and there isn't one.

## 4. Frequency response — the informative one

Drive a logarithmic sine chirp, 0.5 → 60 Hz over ~20 s, and log command and
measurement. `fit_servo_bench.m` returns ω_n, ζ and the transport delay in one
shot: the delay appears as linear phase and separates cleanly from the
second-order phase.

**Run it at three amplitudes — 0.5°, 3° and 12°.** A linear actuator gives the
same answer at all three. Yours will not, and the spread is the measurement.

`verify_fit.m` runs the whole identification against the servo model with a
*known* f_n = 18.50 Hz, ζ = 0.620, τ_d = 11.0 ms, at your current `P.Linkage.n = 1`:

| chirp amplitude | fitted f_n | fitted ζ | fitted τ_d |
|---|---|---|---|
| 0.5° | 30.63 Hz | 2.876 | 17.56 ms |
| **3°** | **18.28 Hz** | **0.681** | **11.63 ms** |
| **12°** | **18.58 Hz** | **0.638** | **11.10 ms** |

The 0.5° fit is worthless — f_n 66 % high, ζ 4.6× high — because the 0.20° dead
zone is 40 % of the drive amplitude. The other two recover the truth almost
exactly. **Drive well outside the dead zone.**

That top end is only clean because n = 1 leaves you 1355 °/s of slew. At n = 4
the 12° case rate-limits and the fit collapses the other way, returning
f_n ≈ 12 Hz for the same servo — **35 % slow**. So the usable window depends on
your linkage ratio. Practical rule: run three amplitudes, keep the ones that
agree with each other, and carry their spread as the uncertainty band.

Run `verify_fit.m` before you build the rig, so you know what good looks like.

Worth noting: your Simulink model uses ω_n = 80 rad/s (12.7 Hz) while
`optimise_roll_schedule.m` uses 2π·20 (20 Hz) and calls it bench-measured.
12.7 Hz is about what a large-amplitude chirp would return for a servo whose
true value is near 19 Hz — that is a hypothesis, not a diagnosis, but it is
worth ten minutes to rule in or out.

## 5. Slew rate under load

Large steps (±30°) with the spring fitted. Measure the constant-slope region
and compare against the derived torque–speed line:

| hinge moment | predicted peak rate at n = 4 |
|---|---|
| 0 | 339 °/s |
| 0.5 N·m | 243 °/s |
| 1.0 N·m | 147 °/s |
| 1.5 N·m | 51 °/s |

If the measured derating is steeper than this, the servo is current-limiting
below its published stall torque and `T_stall` should come down.

## 6. Stall torque

Clamp the canard, command a large offset, read the reaction with a gauge.
Confirms the datasheet at your voltage and confirms where the current limit
sits. Do it briefly — a stalled coreless motor cooks.

## What to do with the numbers

Everything goes into `servo_params_from_datasheet(..., opts)` as overrides:
`deadband_deg`, `lash_deg`, `wn`, `zeta`, `tau_d`, `K_servo`. Everything else —
slew rate, reflected inertia, peak power, acceleration limit — is already
derived from the published table and does not need measuring.
