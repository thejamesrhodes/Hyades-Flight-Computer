# MC-01: SITL 6DOF Monte Carlo, assumption register, requirements and results

**Date:** 8 November 2026.
**Tool:** `Firmware/FC-V4/Tools/sitl/` (`sitl.c` plant + the flight core; `mc.py` driver; `mc_figures.py`).
**Raw data:** `Tools/logs/sitl/runs/` (gitignored; the `summary.json` files are committed).
**Decisions it supports:** ADR-004.
**Figures:** `Images/adr004/`.

## 1. What is simulated

### 1.1 The flight code

Not a model: `Core/hyades/mission/flight.c` and everything it calls (IEKF, NAV-01, detectors, FDIR, phase machine, roll law with trim, boost and dither) is compiled for the host and stepped at `FRAME_HZ` exactly as `app.c` steps it on the STM32.

The boot is replayed as on the target: 1 s gyro calibration, IEKF seed, CALIBRATE → SETTLING → READY, ignition after `pad_hold` seconds in READY.

### 1.2 The plant (`sitl.c`)

All in double precision, 0.25 ms steps in ascent and 1 ms in descent.

| Block | Model | Source |
|---|---|---|
| Rigid body | Forward body 6DOF with quaternion attitude. The can's angular momentum is in the Euler equations (gyroscopic coupling). | — |
| Fin can | 1 roll DOF. Driven by fin cant aero (strip theory Cl_δ 8.75, Cl_p −17.3); coupled to the body by bearing friction T = T_c·tanh(slip/0.5) + c_v·slip. | Free-Spinning Fin Can spec; `rollopt.py` strip theory |
| Mass, CG, inertia | Dry 773.6 g at 0.428 m; motor at 0.827 m; transverse I from OpenRocket with parallel-axis motor changes; forward-body Ixx 2.2e-4. | OpenRocket "62mm Canard Control V1.0.0" export |
| Drag | Cd(Mach) along the air-relative velocity. | OpenRocket (`aero.csv`) |
| Normal force | Total CN_α 11.8 /rad and CP(Mach) from OpenRocket, less the canards' share. | Barrowman check: nose 2.0 + canards 0.91 + fins 8.47 = 11.4 |
| Pitch damping | Nose and rear fins see α_local = ω × r. | Barrowman components |
| Canards | Four fins, each with its own local flow (vehicle α, body rates, deflection) and lift k_f = Cl_δ·A·d/(4 y_cp), stall at 15°. Roll damping, roll/pitch/yaw cross-coupling and saturation all come out of the geometry. | CDS-01 geometry; Cl_δ 0.657 matches CDS-01 |
| Asymmetries | Body Cl₀; net rear-fin cant (spins the can); one misaligned rear fin (a side force rotating with the can, the lock-in path); thrust misalignment; lateral CG offset. | Dispersed, §2 |
| Wind | Power-law shear (exponent 0.14); Gauss–Markov turbulence (τ 1 s, σ = turb × U); optional 1−cos crosswind gust. | — |
| Launch | Rail of 1.5 m, tilted 0–5°. | — |
| Recovery | Canopy (Cd·A 0.198 m²; OpenRocket chute 0.6 m, Cd 0.7) on the shock cord at 0.18 m from the nose, so the body swings. 0.4 s inflation; Gauss–Markov swing of 3–12°; ±10 % breathing. | OpenRocket .ork |
| Ground | Spring–damper (~30 g peak at 6 m/s); canopy collapses 2–30 s after touchdown (until then it drags). | — |
| IMU | QMI8658A. Internal 7.17 kHz: misalignment, scale factor, bias, g-sensitivity, measured noise (ARW 8.5 mdps/√Hz, VRW about 70 µg/√Hz); vibration (broadband plus a Q = 5 resonance); **clip at ±16 g / ±2048 dps before** a 2nd-order 60 Hz LPF; decimate to 448.4 Hz; 16-bit LSB; lever arm. | Datasheet; ADR-003 noise capture |
| Servos | `canard_servo_reduced` port: transport delay, 2nd order, dead band, rate limit (× supply factor), lash, 12-bit quantisation, travel ±12°. The command reaches the servo 1.9 ms after the frame (compute + pulse read). | `parameters_initialise_roll_control.m`; BMS-127WV+ data |
| Servo faults | Stuck, hard-over, slow (¼ rate), floating (unpowered, follows the local flow), offset. On one random fin at a random time 0–6 s. | §5 |
| GPS | 1 Hz altitude, σ 4 m, not usable under thrust; absent in 10 % of flights. | — |

### 1.3 Verification of the plant

| Check | Result |
|---|---|
| G79W, 3 m/s wind, 3° rail | Apogee 432.7 m vs OpenRocket 439 m (OpenRocket wind about 2 m/s); t_apogee 9.59 vs 9.66 s. |
| I205W | 1 291.6 vs 1 303 m. |
| Same flight code on hardware HITL (G79W nominal) | SITL roll p_max 60–120 °/s against HITL 105 °/s, canards about 1° in both. Same q̂ behaviour. |
| Linear margins vs nonlinear SITL | c sweep: the linear edge (c ≈ 22) is benign in SITL because saturation masks it. That's why the gains are chosen on margins (ADR-004 §3.2). |
| Deterministic re-run | `rerun.py runs/X/runs.csv SEED trace.csv` reproduces any run bit for bit. |

## 2. Assumption register

Status codes:
- **M** = measured;
- **D** = derived from project data;
- **A** = assumed (needs a measurement);
- **S** = standard or maker data.

| ID | Quantity | Nominal | MC dispersion | Status | Source / how to close |
|---|---|---|---|---|---|
| A1 | Forward-body roll inertia Ixx | 2.2e-4 kg m² | ×U(0.68, 1.6) | D | OpenRocket 4.78e-4 less can 2.6e-4. OpenRocket fin mass is overridden (6 g for 4 fins). **Trifilar pendulum.** |
| A2 | Transverse inertia I_t | 0.065 dry (0.073 with G79W) | ×N(1, 0.10) | D | OpenRocket. **Bifilar pendulum.** |
| A3 | Can roll inertia I_can | 2.6e-4 | U(1.5, 3.0)e-4 | D | Fin can spec. |
| A4 | Mass and CG | 773.6 g, 0.428 m | ×N(1, 0.03); CG ±N(0, 10 mm) | D | OpenRocket. **Weigh and balance.** |
| A5 | Cd | OpenRocket | ×N(1, 0.10) | D | — |
| A6 | CN_α, CP | 11.8 /rad, OpenRocket CP | ×N(1, 0.10); CP ±N(0, 10 mm) | D | Barrowman cross-check (§1.2). |
| A7 | Canard Cl_δ | 0.657 /rad | ×U(0.7, 1.3) | D | CDS-01; strip theory. Flight data later. |
| A8 | Can Cl_p (rear fins) | −17.3 | ×N(1, 0.15) | D | Strip theory. |
| A9 | Body Cl₀ | 0 | N(0, 0.005) | A | Canard zeroing. Laser alignment (FDS polarity check). |
| A10 | Net rear-fin cant | 0 | **N(0, 0.1°)** build / N(0, 0.25°) wide | A | **Build tolerance ≤ 0.25° (§6).** Fin jig plus laser check. |
| A11 | Single rear-fin misalignment | 0 | N(0, 0.1°) / N(0, 0.2°) | A | As A10. |
| A12 | Bearing Coulomb torque | 5–10 mN m (spec) | **U(3, 6) mN m** build / U(3, 12) wide | A | **Spin-down test (BP-01 §4.3).** |
| A13 | Thrust misalignment | 0 | |N(0, 0.25°)|, random azimuth | A | Typical hobby-motor value. |
| A14 | Thrust | thrustcurve.org / OpenRocket | ×N(1, 0.03) | S | NAR/TRA certification data. |
| A15 | Wind | — | U(0, 8) m/s (U(0, 12) in V_wind12); turbulence 0.1–0.25 U; gust in 30 % (2–8 m/s, 0.5–2 s) | A | Club limits; on-site anemometer. |
| A16 | Rail | 1.5 m | tilt U(0, 5°) | A | MRC rail data. |
| A17 | Servo τ_d | 14 ms | U(10, 21) ms | A | **Laser bench (SERVO_BENCH.md).** |
| A18 | Servo f_N, ζ | 80 rad/s, 0.7 | U(52, 100), U(0.5, 0.9) | A | Bench. A large-amplitude chirp biases f_N low by about 35 %. |
| A19 | Servo dead band | 0.00196 rad | U(0.001, 0.004) | S/A | Maker dead band 2 µs (BMS-126WV sheet). Bench. |
| A20 | Servo lash | 0.002 rad | U(0.001, 0.003) | S | Maker 0.115°. |
| A21 | Servo slew | 23.65 rad/s at 8.4 V | ×U(0.85, 1.0) (2S sag) | D | `servo_params_from_datasheet.m`. |
| A22 | IMU noise | measured | per-run seeds | M | ADR-003 capture, 2.46 h. |
| A23 | IMU bias / scale / misalignment | — | σ 0.03 m/s², 0.05 °/s / 0.5 % / 0.3° | S/A | Datasheet class; residual after boot cal. |
| A24 | Vibration | — | Broadband U(0.5, 3) g RMS; resonance U(0.3, 2) g at U(150, 900) Hz; gyro U(1, 6) °/s | **A** | **No measured PSD yet (BP-01 §3).** |
| A25 | Recovery | 0.6 m chute | delay U(0.5, 3) s; swing U(3, 12)°; collapse U(2, 30) s | A | OpenRocket .ork; Phase 2 recovery sim. |
| A26 | Site elevation | 110 m | — | A | MRC survey or GPS on the day. The density effect is about 1 % per 100 m. |

## 3. Requirements (confirmed by James on 8 Nov; roll and lock-in tightened to the verified design)

| ID | Requirement | Rationale |
|---|---|---|
| R1 | Phase sequence BOOST → COAST → DESCENT → LANDED, no LOCKOUT or CATO, no READY demotion at liftoff | FDS-01 §3 |
| R2 | Liftoff detected ≤ 0.35 s after ignition | Inside the 2.56 s pre-trigger ring with 7× margin |
| R3 | Apogee detected within −0.5 / +1.5 s of truth | Recovery timing (Phase 2) |
| R4a | LANDED **never** before touchdown | Data and beacon safety |
| R4b | LANDED ≤ 15 s after touchdown (≤ 40 s with no GPS), or ≤ 5 s (≤ 35 s) after coming to rest | Operations |
| R5a | \|p\| ≤ **240 °/s** at all times in active flight (was 360) | Tightened to the verified maximum (222 °/s) plus margin |
| R5b | Coast roll-rate RMS ≤ **45 °/s** (was 60) | Verified maximum 43.7 °/s |
| R6 | Max \|body roll\| / ω_pitch ≤ **0.6**, and max \|can spin\| / ω_pitch ≤ **0.6** (was 0.7) | Lock-in margin; verified maxima 0.45 / 0.58 |
| G1, G2 | **Statistical goals:** p95 peak \|p\| ≤ 120 °/s; p95 coast RMS ≤ 25 °/s | Verified 112 / 22 °/s |
| R8 | Canard command saturated ≤ 5 % of the time | Authority margin |
| R9 | q̂ RMS error ≤ 25 % | Inside the ±30 % gain uncertainty the loop was designed for |

## 4. Results

The final firmware includes dither, boost, the torque trim and the STM-01 fixes. 7 680 flights in total.

Wind is U(0, 8) m/s unless stated. "Build tolerance" means A10–A12 at the build values.

| Campaign | n | All-pass | Notes |
|---|---|---|---|
| **Active, build tolerance, 6 motors** | 1 200 | **100 % (tightened R5/R6 too; G1 and G2 met)** | Roll peak p95 112 °/s (max 222); coast RMS p95 22 °/s (max 44); canard max p95 2.8°; AoA (q > 500 Pa) p95 11°; q·α p95 12.7 kPa·°; lateral load p95 0.58 g; q_max p95 8.3 kPa |
| Active, wide tolerance | 1 200 | 92.5 % (tightened) | R6 93.8 %, R5a 98.5 %, R5b 98.7 %: failures trace to cant > 0.5° or friction > 8 mN m, so the build tolerance is mandatory |
| Active, wind 0–12 m/s | 600 | 98.5 % | R9 98.5 % (q̂ uses ground speed; wind error); roll unaffected |
| Flight-1 **shadow** (canards held at 0) | 600 | 94.7 % | Body roll p95 760 °/s, max 1 930 °/s. **5.3 % cross pitch resonance** (ratio max 1.7) |
| Flight-1 **damping only** (trim off) | 600 | 68.3 % | R5b only (coast RMS p95 124 °/s). **Lock-in ratio max 0.50**, canards ≤ 2° |
| Servo stuck / slow / offset | 240 each | 99.6 % | Absorbed by the other three canards and the trim |
| Servo floating (unpowered) | 240 | 99.2 % | One R6 marginal (0.76) |
| **Servo hard-over** | 240 | 35.4 % | Roll peak p95 648 °/s, max about 1 000 °/s; R6 97.5 %; **AoA ≤ 14°: no pitch-over** |

Per motor, active, build tolerance (all 100 %):

| Motor | Apogee p50 | q_max p95 | Coast RMS p95 | Roll peak p95 | Apogee backup (coast p99 × 1.2) |
|---|---|---|---|---|---|
| G40W | 376 m | 5.2 kPa | 21 °/s | 85 °/s | 10.2 s |
| G53FJ | 337 m | 4.6 kPa | 19 °/s | 97 °/s | 8.9 s |
| G79W | 427 m | 7.0 kPa | 21 °/s | 102 °/s | 10.5 s |
| G80T | 503 m | 9.0 kPa | 27 °/s | 123 °/s | 11.2 s |
| G77R | 402 m | 6.9 kPa | 21 °/s | 118 °/s | 10.1 s |
| G104T | 328 m | 5.2 kPa | 22 °/s | 115 °/s | 9.6 s |

**Statistical reading.** 1 200 flights with 0 failures bounds the per-flight failure probability at **≤ 0.25 % with 95 % confidence** (rule of three: 3/n). That applies to the dispersed model, not to the unmodelled.

## 5. Failure modes found and what was done

| # | Mode | Mechanism | Fix / status |
|---|---|---|---|
| FM1 | Roll instability at c = 70 | Ixx 3× lower than assumed, plus 20 ms of unmodelled delay | c = 15, 200 Hz synced (ADR-004) |
| FM2 | Roll hang at 40–60 °/s | Trim band sized for c = 70; friction torque needs 1/q trim | Band 0.3, torque-space trim, dither |
| FM3 | **Lock-in through the free fin can** | Can spin ≈ 1.13 × \|cant°\| × f_pitch, constant through the flight because both scale with V | **Build tolerance:** \|cant\| ≤ 0.25°, single fin ≤ 0.25° (`mc_lockin.png`) |
| FM4 | Friction drag-along | Can spun by cant drags the body through the bearing (T/(k c) ≈ 290 °/s at 7.5 mN m) | Bearing ≤ 6 mN m (spin-down). A can-spin sensor for feedforward was considered for V6 and not adopted |
| FM5 | Apogee 7 s late on high-thrust motors | One vibration-clipped frame latched NAV saturation | NAV clip budget (20 ms) |
| FM6 | False LANDED under a calm canopy (10 s still) | Steady descent looks like ground | 30 s rule; impact-shock rule; t_min 15 s |
| FM7 | Deployment shock latched "touchdown" | Motor-delay deployment after t_min | t_min 15 s |
| FM8 | Passive flight 1 crosses resonance in 5 % of flights | The free forward body has very low roll damping | Flight 1 = **full rate law** (§7), which keeps the body furthest from resonance |
| FM9 | Hard-over: roll up to about 1 000 °/s | The P gain is too low to use the authority of the remaining canards | Boost 4× (p95 920 → 648). **Residual hazard** (servo feedback not adopted for V6) |

## 6. Build and measurement requirements derived

1. Net rear-fin cant ≤ 0.25°; each fin ≤ 0.25° from the body axis. Verify with a jig and laser.
2. Bearing Coulomb torque ≤ 6 mN m. Spin-down test; it must be repeatable after assembly.
3. Canard zero ≤ 0.1° (Cl₀). Laser zeroing.
4. Measure Ixx and I_t (trifilar and bifilar), and servo τ_d, f_N, dead band (laser bench). Then re-run MC-01 with the measured values and consider c = 18–20.
5. Set `DET_T_APOGEE_MAX_S` from the table above for the chosen motor.

## 7. Flight 1 mode: **decided (James, 8 Nov): full rate law, zero-rate target, no angle hold**

The options weighed:

| Option | Lock-in (ratio max) | Roll | Risk |
|---|---|---|---|
| Shadow (canards locked at 0) | **1.7, 5.3 % of flights > 0.7** | p95 760 °/s | Passes through pitch resonance |
| **Damping only** (trim off, dither on) | **0.50** | Coast RMS p95 124 °/s | Minimal authority (≤ 2°); verifies the actuation path |
| **Full law (chosen)** | 0.45 | Coast RMS p95 22 °/s | Most capability on a first flight. It is also the option that keeps the forward body furthest from resonance. |

## 8. How to run

```
cd Firmware/FC-V4/Tools/sitl
python make_tables.py <motor-database/data/thrustcurve.org>   # aero.csv, motors/*.csv
make                                                          # sitl (the flight core is compiled in)
python mc.py --n 200 --cant_sd 0.1 --fin1_sd 0.1 --bear_max 0.006 --out runs/V_active
python mc.py --n 40 --fault hardover --out runs/V_fault_hardover
python mc.py --n 30 "--variant=-DCTRL_ROLL_C=18.0f" --out runs/c18      # gain sweeps
python rerun.py runs/V_active/runs.csv <seed> trace.csv                 # one run with a trace
python mc_figures.py
```

## 9. Limits of this model (what is not covered)

- No structural flexibility.
- No aeroelastic canards.
- No servo thermal or current limit (stall).
- No body Magnus effect.
- Linear aero to stall.
- No rail friction or tip-off.
- Vibration is synthetic (A24).
- Recovery dynamics are simplified: ejection forces, line stretch and inflation loads are Phase 2.
- Structural loads are reported only as q·α and lateral load factor; Phase 2 adds bending moments.
- The motor shortlist excludes anything above 14 g by design.
