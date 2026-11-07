# Actuator model reduction — what can be dropped, and what cannot

Status: settled by measurement, 2026-08-24. Supersedes the informal claim in
ADR-001 §2.4 that command quantisation is negligible.

## Question

Is the eight-element canard servo model (`canard_servo.m`) too complex for Monte
Carlo, and can the deadband/backlash behaviour be recovered from a linearisation
or a simpler equivalent?

## Answer

**A linearisation cannot substitute.** A describing function has no DC component,
so it reproduces neither the trim hang-off nor the limit cycle — the two
phenomena the Monte Carlo exists to quantify. It also cannot represent the
quantiser hunting described below, which is not amplitude-continuous at all.

**Two of the eight elements are provably inert** and one of the "obviously
droppable" ones is load-bearing. Measured by ablation at the current operating
point (ref = 0, Cl_0 = 0.03, 7.4 V servo, `compare_actuators.m`):

| model | p mean (rad/s) | p pp (rad/s) |
|---|---|---|
| all eight | 0.2444 | 0.2028 |
| no acceleration limit | 0.2444 | 0.2028 |
| no load derating of the rate limit | 0.2444 | 0.2028 |
| no rate limit | 0.2444 | 0.2028 |
| **no quantiser** | **0.2175** | **0.0000** |

- **Acceleration limit** — a_max ≈ 5.8e3 rad/s² against a peak demand ≈ 6.4e2.
  Bit identical when removed.
- **Load derating of the rate limit** — hinge moment at 40 kPa and full travel
  is 3.7 mN·m against ~390 mN·m available (<1 %), and it derates a limiter that
  never engages (peak slew 127 °/s against ~1100 °/s). Bit identical.
- **Rate limit** — also never engages, but it is free as an integrator
  saturation, so keep it as a guard.
- **Command quantisation — keep it.** The quantum is 0.0292° against a 0.2005°
  dead zone, seven times finer, and the intuition that it therefore cannot bind
  is wrong: it does not compete with the dead zone, it hunts against the
  *continuous* trim integrator. It is the sole source of residual roll-rate
  ripple. Its magnitude is a lattice effect and is not smooth in `Res`
  (7.3 °/s pp at Res = 5.1e-4, 0.35 °/s at 1.0e-3), which is an argument for
  dispersing it in Monte Carlo rather than trusting any single value.

## Why this reduction is the useful one

The load derating is the only element that needs the aerodynamic hinge moment,
and therefore the only reason the actuator model has to know the canard
geometry, the flight condition, or its own past output. Removing it turns the
actuator into a plain signal chain that maps one-for-one onto native Simulink
blocks (`build_reduced_actuator.m`):

    In → Quantizer → Delay(N) → Sum(+−) → Dead Zone → Gain f_N² → Sum(+−)
       → DTI [limit ±Rate/n] → DTI [limit ±Travel] → Backlash → Out

That is where the sim-speed win is. A native chain compiles under Accelerator; a
MATLAB Function block holding persistents runs interpreted every step of every
run. The element count barely changes; the execution model changes completely.

## Do not merge the dead zone into the backlash

Collapsing dead zone + backlash into one Backlash block of width
(Deadband + Lash) is the obvious simplification and it is wrong. Open-loop gain
at 5 Hz, referenced to the full model:

| drive amplitude | full | merged dz+lash | merged 2·dz+lash |
|---|---|---|---|
| 0.3° | 0.102 | 0.496 (+387 %) | 0.091 (−10 %) |
| 0.5° | 0.552 | 0.738 (+34 %) | 0.532 (−4 %) |
| 1.0° | 0.816 | 0.889 (+9 %) | 0.806 (−1 %) |
| ≥2° | — | +3 % or less | −1 % or less |

The loop lives at fractions of a degree, so the naive merge is wrong exactly
where it matters, and wrong in the flattering direction — the model looks better
damped than the hardware. If a single block is unavoidable, the kinematically
correct width is **2·Deadband + Lash**: on a reversal the command must cross the
dead zone twice before the output moves again.

## Two parameter bugs found while doing this

1. **`P.Aero.y_Cp = 0.03975` was stale.** That is the spanwise area centroid of
   the *old* canard (s = 0.020, C_r = 0.044, C_t = 0.020). The current panel
   (s = 0.012, C_r = 0.030, C_t = 0.015) gives 0.03633, so C_lδ had been running
   9 % high. Now derived in the parameters file as
   `y_bar = r + s(C_r + 2C_t) / (3(C_r + C_t))` so it cannot go stale again.

2. **`P.Trim.Clamp = ±3°` was undersized.** With the geometry-consistent
   C_lδ = 0.2975, cancelling Cl_0 = 0.03 needs 5.78° of trim (3.85° even at the
   file's Cl_0 = 0.02). The trim saturated at the clamp and left a standing roll
   rate. Raised to ±6°, which covers Cl_0 ≤ 0.031 and still leaves 6° of travel
   for damping.

Consequence for tuning: C_lδ has fallen from 0.8398 (old canard) to 0.2975, so
open-loop gain is **2.82× lower** than `optimise_roll_schedule.m` assumed.
`P.Controller.c = 100` would need ~282 to restore the old crossover, but the
delay margin has not moved, so the optimum may not scale linearly — re-run the
sweep rather than taking that number on trust.
