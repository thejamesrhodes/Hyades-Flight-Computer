# LD-01: Flight structural loads from the 6DOF Monte Carlo, mass-property correction, and the physical test plan

**Date:** 8 November 2026. Phase 2 of the ADR-004 / MC-01 work (agreed at the 8 Nov checkpoint).
**Companion:** REC-01 (ejection, shock cord and parachute loads).
**Tools:**
- `Tools/sitl/sitl.c` (loads model, `loads_init` / `loads_eval`);
- `Tools/sitl/rerun_ld01.sh` (campaigns);
- `Tools/recovery/ld_rec_figures.py`.

**Figures:** `Images/ld01/`.

Markers used in this document:
- **[fact]**: from a cited source;
- **[inference]**: my reasoning;
- **[assumption]**: unverified.

## 1. Summary

1. **The plant's mass properties were wrong**, and the error is now fixed. The SITL used a dry CG of 0.428 m, a motor CG of 0.827 m and a dry I_t of 0.065. The OpenRocket export, solved at ignition and at burnout, gives 0.401 m, 0.835 m and 0.053. With the old values the loaded CG sat 2.2 cm aft (about 0.35 cal less static margin) and I_t was 12 % high. Both lowered the pitch frequency, so the MC-01 lock-in results were **conservative**. Every MC-01 campaign was re-run (§3).
2. **Ascent loads are small.** Over 3 840 dispersed flights, the largest values are:
   - bending moment 0.56 N m (wind 0–12 m/s);
   - axial compression 92 N (at the fin-can joint, at peak thrust).
   
   Stresses are below 1 MPa in every section.
3. **Recovery and landing size the structure, not ascent.** Ejection puts up to 685 N on the coupler bulkhead and on the motor retention. The shock cord sees up to 640 N with the current Kevlar cord. Landing reaches p99 49 g (max 63 g) at about 8 m/s descent. See REC-01.
4. **Two corner flights fail the tightened roll requirements** with the corrected plant: 2 of 1 200, both at the low-Cl_δ, slow-servo corner. **This needs a decision from James** (§3.2).

## 2. Mass-property correction

The OpenRocket G79W simulation export (`Simulation/or_sim1_G79W.csv`) was solved at two points, ignition and burnout:

| | Mass (g) | Motor mass (g) | CG (cm) | I_t (kg m²) |
|---|---|---|---|---|
| Ignition | 897.6 | 124.0 | 46.07 | 0.0734 |
| Burnout | 837.5 | 64.0 | 43.39 | 0.0643 |

Solving the pair gives a motor CG of **0.835 m** and a dry vehicle of **773.6 g at 0.401 m**, with dry I_t about its own CG of **0.053**.

| Quantity | SITL before | SITL now | OpenRocket |
|---|---|---|---|
| Loaded CG | 0.4830 m | 0.4607 m | 0.4607 m |
| Burnout CG | — | 0.4339 m | 0.4339 m |
| Loaded I_t | 0.0820 | 0.0732 | 0.0734 |

The G79W apogee check against OpenRocket (3 m/s wind, 3° rail) moves from 432.7 m to 431.8 m (OpenRocket 439 m).

The error existed only in the SITL defaults. HITL (`hitl_model.py`) and MATLAB do not use these values.

## 3. Re-run of the MC-01 campaigns (corrected plant, 3 840 flights)

### 3.1 Results

| Campaign | n | All-pass now (was) | Notes |
|---|---|---|---|
| Active, build tolerance | 1 200 | **99.8 %** (100 %) | Roll peak p95 111 °/s; coast RMS p95 21.7 °/s, so G1 and G2 are still met. Lock-in max 0.42 / 0.41 (was 0.45 / 0.58). AoA p95 11°. |
| Active, wind 0–12 m/s | 600 | 96.2 % (98.5 %) | R9 q̂ error 96.3 %: q̂ uses ground speed, a known limitation. Roll unaffected. |
| Active, wide tolerance | 1 200 | 95.1 % (92.5 %) | R6 96.6 %. The build tolerance is still required. |
| Shadow (canards held) | 600 | 96.5 % (94.7 %) | Body lock-in ratio max 1.24. Shadow is still unsuitable. |
| Servo hard-over | 240 | 25.8 % (35.4 %) | Roll p95 615 °/s. **AoA max 15°: no pitch-over.** |

### 3.2 The two failures (decision for James)

| Seed | Motor | Failed | Value | Corner |
|---|---|---|---|---|
| 201168 | G79W | R5a (≤ 240 °/s) | 253 °/s during boost | Cl_δ × 0.71, servo f_N 57 rad/s, 7.3 m/s gust |
| 301174 | G80T | R5b (≤ 45 °/s) | 46.1 °/s coast RMS | Ixx × 1.38, Cl_δ × 0.79, servo τ_d 21 ms |

The thresholds were set on 8 Nov to the maximum then achieved plus a small margin. The corrected plant moves the extreme tail by 5–15 %.

The options are:

- **(a)** Reset R5a to 260 °/s and R5b to 50 °/s. The statistical goals G1 and G2 (p95) are unaffected and still met.
- **(b)** Keep the thresholds and accept 0.17 % as the dispersed-model failure rate.
- **(c)** Wait for the servo-bench and pendulum measurements. These would shrink the Cl_δ, Ixx and τ_d corners that drive both cases, and are the right long-term fix anyway (BP-01 §4).

My recommendation is (c), with (a) in the meantime **[inference]**.

## 4. Loads model

### 4.1 Cut method

At each ascent step (off the rail, before deployment), the internal resultant at a station is the sum of two parts over everything forward of the cut:

- the external non-gravity forces;
- the inertial reactions of the mass nodes, −m_j f_j, where f_j = f_cg + ω̇ × r_j + ω × (ω × r_j). Gravity cancels.

The result gives axial force (compression positive), shear, and bending moment. A cut aft of the whole vehicle must carry nothing, and that closure is checked on every step.

### 4.2 Force placement

| Force | Where it is applied | Status |
|---|---|---|
| Body normal force | Split between the nose CP (0.047 m) and the fins (0.812 m), with the same total and moment as the SITL's CN_α / CP | [inference] |
| Drag | 40 % forebody at the nose, 30 % skin friction in 5 stations, 30 % base | **A-L1 [assumption]** |
| Canards | At their station | — |
| Pitch damping | At the nose and the fins | — |
| Misaligned-fin side force | At the fins | — |
| Thrust | At the nozzle | — |

### 4.3 Mass model

- 20 components from the OpenRocket tree (`rocket.ork`): tubes as 4 nodes, the motor as 4 nodes over its length.
- 29.6 g that the tree lists only through overrides is placed to hit the OpenRocket CG; it lands at 0.84 m.
- The model is calibrated at nominal mass, then scaled and shifted for each run's mass and CG dispersion.
- Its I_t is 0.0545 against OpenRocket's 0.053. The rotational inertial term is scaled to the dynamics' I_t, so the cut still closes.

### 4.4 Verification

| Check | Result |
|---|---|
| Closure residual | p99 3.7 %, max 4.7 % of m g L. The remainder is the gyroscopic term of the spinning can and the canards' radial offsets. |
| Hand check, axial | Compression at the fin-can joint at peak thrust ≈ T × m_fwd / m = 0.7 × 122 ≈ 86 N; the SITL p50 at G80T is 88 N. |
| Bug found and fixed in verification | With mass_scale < 1, the calibration node landed metres away (I_t 0.73). It is now calibrated at nominal mass. Residual went from 39 % to under 5 %. |

## 5. Results: ascent limit loads

The limit load is the maximum over all campaigns (3 840 flights, about p99.97). Figure: `Images/ld01/ld_envelope.png`.

| Station (m from tip) | Joint | Bending p99 / max (N m) | Compression p99 / max (N) | Equivalent axial P + 2M/R, max (N) |
|---|---|---|---|---|
| 0.10 | Nose / avionics bay | 0.11 / 0.16 | 8.3 / 8.8 | 17 |
| 0.18 | Avionics bay / canard module | 0.25 / 0.36 | 27 / 28 | 47 |
| 0.26 | Canard module / body tube | 0.37 / 0.51 | 48 / 51 | 75 |
| 0.32 | Coupler aft end | 0.41 / 0.56 | 64 / 66 | 94 |
| 0.51 | Body tube mid | 0.39 / 0.56 | 74 / 76 | 100 |
| 0.76 | Body / fin can | 0.24 / 0.31 | 88 / 92 | 106 |

p99 is over all 3 840 flights pooled.

Other ascent maxima:
- shear ≤ 3.6 N;
- lateral load factor 0.92 g;
- AoA (q > 500 Pa) p99 16°, max 23° (wind 0–12 m/s);
- q·α p99 16.3 kPa·°, max 24.1 kPa·°;
- SITL landing impact p99 49 g, max 63 g at 7–10 m/s descent.

**Stress check [inference].**

| Section | Area | I | Peak stress at max loads |
|---|---|---|---|
| Carbon body tube (62 × 1 mm) | 1.9 cm² | 8.9e-8 m⁴ | 0.48 MPa axial + 0.19 MPa bending |
| 2 mm PLA sections | 3.8 cm² | 1.7e-7 m⁴ | Below 0.3 MPa |

Both are two orders of magnitude below any credible allowable for these materials, including printed-PLA interlayer strength, which is unknown here **[assumption]**. **The ascent environment does not size anything.** The canards' own hinge and servo loads are in CDS-01.

## 6. Physical test plan (what to verify against)

**Factors of safety.** Sandia's mechanical specification uses an ultimate factor of 1.5 with qualification testing and 2.0 without, and a 1.2 development/qualification test factor **[fact]** ([JAS toolbox](https://jastoolbox.sandia.gov/?p=520)). Spacecraft use an ultimate factor of 1.4 and aircraft 1.5 **[fact]** (Zipay, Modlin and Larsen, [NTRS 20150003482](https://ntrs.nasa.gov/citations/20150003482)).

For Hyades **[inference]**:
- **qualification test load = 1.25 × limit**;
- **design ultimate = 2.0 × limit** for printed and bonded parts, which have no allowables data;
- the factor can drop to 1.5 once a coupon or qualification test has been passed.

| # | Test | Load (1.25 × limit) | Pass criterion | Driving case |
|---|---|---|---|---|
| T1 | **Motor retention, aft pull** (spring scale or weights on the motor case) | **860 N** | No slip, no cracking | REC-01 ejection, 1.0 g charge |
| T2 | **Coupler bulkhead push-out** (static load on the bulkhead from inside the body tube) | **860 N** | No movement of the bond | REC-01 |
| T3 | **Shock-cord attachment eyes** (forward bulkhead, aft anchor), pull test | 1.25 × cord limit: **800 N** (current 4 m Kevlar) / **270 N** (REC-01 §5.5 recommended cord, covering the shroud load too) | No failure; repeat ×3 | REC-01 snatch |
| T4 | Shroud-line / canopy attachment | **500 N** (current) / **270 N** (recommended) | No failure | REC-01 opening shock |
| T5 | Axial compression of the full stack (weights, motor case in place) | 115 N (1.25 × 92) | Trivial: do once with T6 | Ascent |
| T6 | Bending at the coupler joint (cantilever, weight at the nose) | 0.7 N m (1.25 × 0.56) | Trivial | Ascent |
| T7 | **Drop test**, assembled sections, onto the field surface, at the descent rate (8 m/s now, about 5.4 m/s with REC-01's larger canopy) | Energy-equivalent drop height v²/2g: 3.3 m now, 1.5 m with the larger canopy | Fins and canards survive, FC stays seated | REC-01 §6, SITL impact p99 49 g |
| T8 | **Ejection ground test** (required by the UKRA Safety Code 4.5) | Flight charge | Clean separation; chute out; no motor movement | REC-01 |

T1 to T4 and T7–T8 are the ones that matter. T5–T6 confirm the ascent analysis cheaply.

**Instrumented verification.**
- T1–T3: a luggage scale or a load cell gives the force.
- T8: the FC logs the separation shock, or film at 240 fps gives the separation speed (REC-01 predicts 10.8 m/s p50).
- In flight, V6-3 (raw IMU ≥ 1 kHz) and V6-5 (high-g) give the boost load factor and the recovery shocks, which can be compared directly with LD-01 and REC-01.

## 7. Assumption register (loads)

| ID | Assumption | Effect | How to close |
|---|---|---|---|
| A-L1 | Drag split 40 / 30 / 30 | Axial force distribution ±10 % | Not worth closing: ascent loads are tiny |
| A-L2 | Rigid body, no structural modes | Dynamic amplification at the coupler joint not modelled | Tap test (BP-01 §3) finds the first bending mode. If it is above about 50 Hz, the rigid assumption holds for the aero loads. |
| A-L3 | 29.6 g of untracked mass placed at 0.84 m | Local distribution near the fin can | Weigh the sections (forward and aft of the coupler) and balance each |
| A-L4 | Landing via the SITL spring–damper (k 1 500 N/m, c 30 N s/m) | Impact g is only indicative | Drop test T7 with the FC logging |

## 8. Files

- `Tools/sitl/sitl.c`:
  - corrected mass properties;
  - `loads_init` / `loads_eval`;
  - deployment-state outputs (`dep_V`, `dep_gam`, `dep_aoa`, `dep_h`) and `v_desc`;
  - `LD_DEBUG=1` prints the closure history.
- `Tools/sitl/mc.py`: `loads` block in `summary.json`.
- `Tools/sitl/rerun_ld01.sh`.
- `runs/L_*/summary.json`.
