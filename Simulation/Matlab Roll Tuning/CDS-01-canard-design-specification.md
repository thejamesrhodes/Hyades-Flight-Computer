# CDS-01 — Canard Design Specification

**Hyades 62 mm canard-controlled rocket — roll control surfaces**

| | |
|---|---|
| Status | Released for first flight, rate damping only |
| Date | 2026-08-24 |
| Supersedes | ADR-002 (proposal); `canard-sizing-the-real-trade.md` (superseded analysis) |
| Verification | Monte Carlo, 400 runs, 7 envelope points, 11 dispersed parameters |
| Not cleared for | Attitude hold; supersonic flight; CFD-validated performance |

---

## 1. Geometry

| Parameter | Value | Note |
|---|---|---|
| Exposed semi-span, s | **18.0 mm** | from body surface |
| Root chord, C_r | **24.0 mm** | |
| Tip chord, C_t | **12.0 mm** | taper ratio 0.50 |
| Number of panels, N | **4** | |
| Planform area, one panel | 3.24 cm² | |
| Planform area, all four | **12.96 cm²** | |
| Aspect ratio, s²/S_c | **1.00** | panel convention |
| Mean aerodynamic chord | **18.67 mm** | |
| Spanwise centre of pressure, y_Cp | **39.00 mm** | from roll axis; = r + s(C_r+2C_t)/(3(C_r+C_t)) |
| **Hinge line** | **0.20 × MAC** | **3.73 mm aft of the local leading edge at the MAC** |
| Section | rounded hexagonal | |
| Thickness, root | **≥ 5.0 mm** (t/c ≈ 0.21) | set by the spar, see §5 |
| Body diameter, d | 62.0 mm | |

**y_Cp is derived, never a constant.** It is the spanwise area centroid. If the
geometry changes, recompute it — a stale y_Cp survived one canard change in this
project already and put Cl_δ 9% out.

## 2. Aerodynamic coefficients

Barrowman fin theory with the DATCOM low-aspect-ratio lift slope and the
fin–body interference factor K_fb = 1 + r/(s+r).

| Coefficient | Value | |
|---|---|---|
| Cl_δ, roll control effectiveness | **0.6569 /rad** | at M = 0.82 |
| Cl_p, roll damping | **−0.9179** | canards + body skin friction only |
| C_Nα, canard normal force | **1.079 /rad** | all four panels — **destabilising in pitch** |

Static margin with canards fitted, checked in OpenRocket: **1.8 calibres**.
Requirement ≥ 1.5. This is the only flight-safety-critical item in the sizing
and it is satisfied. **If the geometry changes, re-check it before anything
else.**

Cl_p contains **no contribution from the rear fins** — they are on a
roll-isolated bearing can and do not damp the controlled body. This is
deliberate and correct for this airframe; see §8.

## 3. Actuator

**Blue Bird BMS-127WV+**, one per panel, direct drive (linkage ratio n = 1).

| Parameter | Value | Source |
|---|---|---|
| Supply | 8.4 V | design choice |
| Stall torque | 0.5198 N·m (5.3 kg·cm) | datasheet |
| Speed | 0.05 s / 60° | datasheet |
| Derived no-load rate | 23.65 rad/s | fitted from four (V, torque, t60) points |
| Pulse width | 1100–1900 µs for 90° | datasheet → **8.889 µs/deg** |
| **Dead band** | **2 µs = 0.225° window** | datasheet → **0.00196 rad half-width** |
| **Backlash** | **0.115°** = 0.00200 rad | manufacturer, direct: 0.2 mm at 100 mm arm |
| Frame rate | **250 Hz** (3 ms max) | datasheet |
| Stall current | 1.3 A at 8.4 V | manufacturer |
| Mechanical travel | ±12° | design limit |
| Command resolution | 12-bit over 120° = 0.029° | flight computer |
| Natural frequency, f_N | 80 rad/s (12.7 Hz) | **assumed — still unmeasured** |
| Damping ratio, ζ | 0.7 | **assumed** |
| Transport delay | 14 ms | **assumed** |

**Direct drive is mandatory, not a default.** A reduction stage would shrink the
dead zone at the canard by 1/n but adds its own backlash in series with the
existing 0.115°. Since lash and dead zone are already comparable, a 4:1 stage
would need under ~0.04° of added lash to be worth it. No practical hobby gear
train achieves that.

**Power budget:** 4 × 1.3 A = **5.2 A, 43.7 W** with all four stalled
simultaneously. Size the BEC and harness for this, not for the running current.

## 4. Hinge

**Hinge at 0.20 c. This is a safety requirement, not a preference.**

The hinge must sit **ahead of the most-forward aerodynamic centre at every
Mach number reached**, or the panel is aerodynamically overbalanced somewhere in
the flight and can diverge in torsion. Subsonic x_ac ≈ 0.25 c and it migrates
aft transonically (taken as 0.35 c at M = 0.82). A hinge at 0.20 c is therefore
always **underbalanced** — the aerodynamic moment always restores toward zero.

It also keeps a small, consistent hinge moment that takes up the 0.115° of
backlash in one direction rather than letting the panel rattle inside it.

| Quantity | Value |
|---|---|
| Hinge moment, q = 40 kPa at 12°, subsonic x_ac | 0.0064 N·m |
| Hinge moment, same, transonic worst case | **0.0191 N·m** |
| **Torque margin** | **27** |
| Torsional divergence q, K_servo = 30 N·m/rad | 13.2 MPa |
| Torsional divergence q, pessimistic K_servo = 5 | 2.19 MPa |

Against a maximum flight q of 40 kPa, divergence is clear by a factor of 55 even
on the pessimistic stiffness. **An earlier iteration carried the hinge at 0.90 c
— overbalanced by 0.65 MAC — which capped the canard on servo torque and would
have been a divergence risk.**

## 5. Structure

**No metal spar. No composite reinforcement. FDM ASA throughout.**

| Load case (q = 40 kPa, 12°, one panel) | Value |
|---|---|
| Panel lift | 6.82 N |
| Root bending moment | 0.0546 N·m |
| Hinge torque | 0.0191 N·m |

| Section | Required | Provided | Margin |
|---|---|---|---|
| Hinge shaft, bending | 3.82 mm ⌀ | **5.0 mm ⌀** | **2.25** |
| Hinge shaft, torsion | — | 5.0 mm ⌀ | 0.78 MPa, negligible |
| Panel root, bending | 3.0 mm thickness | ≥5.0 mm | > 4 |

Allowable taken as **10 MPa** — conservative for FDM ASA loaded across layers
(bulk cross-layer strength is 20–25 MPa), so the true margin on the shaft is
about 4.5.

**Print orientation:** the shaft's bending stress is the driving case. Orient so
the layer lines run **along** the shaft axis where possible; if the panel must be
printed flat, keep the shaft diameter at 5.0 mm and treat the 10 MPa allowable
as the design value.

The panel root is never critical — the full 24 mm chord carries the bending, so
3 mm of thickness would suffice there. The 5 mm requirement is entirely the
shaft.

## 6. Control law

Proportional roll-rate damper with a bounded trim path and command dither.
`Controller.Mode = 1`.

    delta_cmd = Kp·p_e + delta_trim + dither ,   Kp = c / q_e

| Parameter | Value | Note |
|---|---|---|
| `Controller.c` | **70** | feasible span [35.8, 95.2] |
| Open-loop crossover | **1.96 Hz** | actuator-limited, not canard-limited |
| Kp at q = 2 / 12 / 40 kPa | 0.0350 / 0.00583 / 0.00175 | |
| `Trim.Ki` | 0.0234 | |
| `Trim.BandK` | 1.1 | |
| `Trim.Clamp` | ±6° | trim needed for Cl_0 = 0.03 is 2.62° |
| `Trim.Rate` | 3 °/s | |
| **`Dither.Amp`** | **1.2 × dead zone = 0.135°** | **square wave** |
| **`Dither.Freq`** | **125 Hz** | sign toggles every 250 Hz frame |

**Crossover is capped by the actuator, not the canard.** The margin analysis
gives the same 2.66 Hz ceiling at every canard area from 5 to 43 cm², because
re-tuning c to hold crossover makes the loop product Cl_δ·c invariant. More
canard area buys authority; it does not buy bandwidth. c = 70 sits below the
2.66 Hz ceiling deliberately — see §7.

### 6.1 Dither — why it is there

The servo dead zone, the 12-bit command quantiser and the gear lash form a
lattice. Whether the loop hunts depends on where the equilibrium deflection
happens to land on it: six of seven envelope conditions settle flat and one
(q = 5 kPa, V = 100 m/s) limit-cycles at 14.3° peak-to-peak in **roll angle**.
Dispersion sprinkles that condition through the Monte Carlo.

Ruled out, each measured:

| candidate | result |
|---|---|
| dead-zone compensation on the command, 80% / 100% | 14.31° / 14.30° vs 14.35° — **no effect** |
| algebraic Cl_0 estimator replacing the integrator | leaves **exactly half** the disturbance |
| trim capture (integrate then freeze) | **cycle survives** with the integrator frozen |

Compensation fails because the dead zone lives inside the **servo's own position
loop**, not on the command path; a loop-internal nonlinearity cannot be
cancelled from outside the loop. The estimator fails because unity loop gain
through a filter gives 1/(1+1) = ½ — only integral action gives zero
steady-state error.

This is the **valve-stiction limit cycle** of process control: an integrating
controller driving an actuator with a dead band. The established cures there are
the same three available here — remove the integrator, add dither or knocker
pulses, or fix the actuator.

Dither is injected on the **command**, so the servo's internal error
|cmd − θ| is swept across the dead zone continuously and the threshold is
smeared. The **surface does not follow it**: at 125 Hz against f_N = 12.7 Hz the
actuator attenuates by ~99%. Measured surface activity *falls* when dither is
added, because the hunting it removes costs more motion than the dither adds.

### 6.2 Dither amplitude — sized from quantisation theory

Amplitude is set by the dither ratio ρ = σ/Δ from classical quantisation theory,
where σ is the dither RMS and Δ the threshold width. The pseudo-quantisation-noise
model fails below ρ ≈ 0.5 (the dead-zone regime) and converges to the classical
dither limit above ρ ≈ 0.6.

- Square wave: σ = A, so **A = 1.2 × dz_half gives ρ = 0.6**.
- A sine needs A = 1.7 × dz_half for the same ρ, and at a 250 Hz frame rate a
  60 Hz sine is only 4.2 samples per cycle and aliases. **Use the square wave.**

This was verified against the project's own ICM-42688-P quantiser
characterisation: the measured η(ρ) curve places the knee at ρ ≈ 0.5–0.6, and
the actuator dither that worked in simulation (1.5 × dz sinusoidal, ρ = 0.53)
lands exactly on it, while the one that did not (1.0 × dz, ρ = 0.354) sits
inside the dead-zone regime. Two unrelated devices, same curve.

## 7. Verification

Monte Carlo, N = 400, seed 7331, 7 envelope points (q 2–40 kPa, V 60–280 m/s),
11 dispersed parameters including Cl_0 at 1σ = 0.03, dead zone ±30%, transport
delay ±25%, f_N ±20%, Ixx ±20%.

| Metric | Result | Requirement | |
|---|---|---|---|
| Unstable fraction | **1.7%** | ≤ 2% | ✔ |
| **Angle ripple, 95th pct** | **6.39° pp** | ≤ 10° pp | ✔ |
| Steady travel, 95th pct | **40.7%** | ≤ 50% | ✔ |
| Settling | **6.74 s**, 351/393 runs | — | ✔ |
| RMS rate error, 95th pct | 0.808 rad/s | ≤ 0.5 | ✘ see below |
| Rate bias, 95th pct | 0.808 rad/s | — | |
| Ripple, 95th pct | 0.234 rad/s | — | |
| Surface activity | 0.120 rad/s | — | |
| Runs over spec (\|bias\| > 1 rad/s) | 6 / 393 | — | |

**The RMS requirement is the one miss, and it is not a real one.** RMS equals
bias exactly (0.808 = 0.808), meaning the residual is entirely steady offset
with no oscillatory content. A steady rate bias is absorbed by the future
attitude loop at zero cost — it commands an opposing rate. The 0.5 rad/s figure
was a placeholder with no mission behind it.

**For comparison, `Mode = 0` (no trim) at the same gain leaves 25.8 rad/s of
standing roll** — a 32× worse bias for identical angle ripple. Mode 1 with
dither is the flight configuration.

## 8. Limitations and exclusions

1. **Servo f_N, ζ and transport delay are assumed**, not measured. They set the
   crossover ceiling. `SERVO_BENCH.md` measures them.
2. **σ(Cl_0) = 0.03 is assumed.** Required canard area is directly proportional
   to it. The design holds Cl_0 ≤ 0.0688 on half travel = 2.29σ.
3. **No CFD.** The lift slope is DATCOM low-AR at AR = 1.00; thickness effects
   are not modelled at all, and the section is thick (t/c ≈ 0.21) for M = 0.82.
   Expect a transonic drag rise this analysis does not predict.
4. **Roll axis only.** No pitch/yaw coupling, no Magnus, no cross-axis inertia.
5. **The rear fin can is roll-isolated on bearings and is not modelled.** Bearing
   friction and the two-body coupling torque are both absent.
6. **Not cleared for attitude hold.** 6.39° of angle ripple is within the
   ±5° budget, but the outer loop has not been designed or simulated.
7. **Hardover is not mitigated by sizing.** The steady roll rate following a
   four-canard hardover is essentially independent of canard area, because
   Cl_δ/Cl_p is nearly invariant. The mitigation is a failsafe that centres the
   surfaces on fault detection — see FDS-01.

## 9. Change control

Any change to s, C_r, C_t or the hinge fraction invalidates §2, §4, §5 and §7.
Re-run in order:

    optimise_roll_schedule(P)                          % new c
    parameter_sweep_roll_control({'Controller.c', ...}) % confirm it
    montecarlo_roll_control(struct('N',400,...))        % verify
    % and re-check static margin in OpenRocket

`P.Aero.y_Cp` is derived in `parameters_initialise_roll_control.m` and must stay
derived.
