# REC-01: recovery sub-simulation (motor ejection, single chute): ejection, separation, shock cord and parachute loads, and sizing

**Date:** 8 November 2026. Phase 2, as agreed at the 8 Nov checkpoint (James: "Motor ejection, single chute").
**Tools:** `Firmware/FC-V4/Tools/recovery/`:
- `rec.py` (model and Monte Carlo);
- `rec_campaign.sh`, `rec_campaign2.sh` (campaigns);
- `ld_rec_figures.py` (figures).

**Inputs:** deployment states from the LD-01 SITL Monte Carlo (`sitl/runs/L_active`, 1 200 dispersed flights over 6 motors).
**Figures:** `Images/ld01/rec_*.png`.

Markers used in this document:
- **[fact]**: from a cited source;
- **[inference]**: my reasoning;
- **[assumption]**: unverified.

## 1. Summary

| Item | Current design (0.6 m chute, 4 m Kevlar) | **Recommended** (0.9 m chute, 6 m cord with EA ≈ 300 N) |
|---|---|---|
| Descent rate | **8.1 m/s** (SITL p50; 7.4–9.3 p5–p99) | **≈ 5.4 m/s** |
| Peak cord tension, p99 / max | **457 / 639 N** | **116 / 146 N** (152 N if deployed at 25–40 m/s) |
| Peak shroud-line (opening) load, p99 / max | 278 / 399 N | 145 / 211 N |
| Peak section acceleration, max | **156 g** (forward), 153 g (aft) | **37 g** (forward), 24 g (aft) |
| Ejection pressure, p99 / max (charge 0.3–1.0 g) | 193 / 242 kPa | Same |
| Bulkhead and motor-retention load, p99 / max | **546 / 685 N** | Same; set by the charge (§4) |
| Separation speed (p50) | 10.8 m/s | Same |

**Main findings.**

1. **The stiff Kevlar cord drives the snatch loads.**
   - Peak tension scales roughly as Δv·√(EA·μ/L).
   - A 4 m Kevlar cord gives over 100 g snatches. That is enough to unseat parts, it clips the FC-V4 ±16 g IMU, and it may break printed attachments.
   - A cord with EA ≈ 200–500 N cuts the tension 4× and the accelerations 4–6×.
2. **The 0.6 m chute is undersized** for an 838 g recovered mass.
   - It gives 8.1 m/s and 29 J of landing energy, against 12 J at 5.4 m/s.
   - UKRA sets no descent-rate limit but requires every part to land fit to fly again (Safety Code 4.5) **[fact]** ([UKRA Safety Code](https://ukra.org.uk/safety/safety-code/)).
   - A 0.9 m canopy does not raise the opening loads, because the snatch, not the opening, dominates.
3. **The ejection charge decides the bulkhead and motor-retention loads.**
   - Up to 685 N, with the charge size unknown (0.3–1.0 g assumed).
   - Weigh the reload's charge, and proof-test the motor retention and the bulkhead (LD-01 T1/T2).
   - Separation fails in under 1 % of cases only above about 0.3 g of black powder with an 80 N fit.
4. **After the canopy opens, the aft section overruns the forward one** (the chute is tied nearer the forward section). In a planar point-mass model the sections pass within 0.4 m in 30–75 % of runs at about 10 m/s, depending on the cord. **This is a contact risk, not a certainty** [inference]: the model has no body rotation or lateral separation. A longer cord reduces it.

## 2. Model

### 2.1 Bodies and links

The model is planar, with 4 point masses, vectorised over the Monte Carlo runs:

| Body | Contents | Mass |
|---|---|---|
| **F** | Forward section: nose, avionics bay, canard module, coupler and bulkhead | 435 g |
| **A** | Aft section: body tube, fin can, engine block, spent motor | 373 g |
| **K** | Cord knot / quick link | 3–10 g |
| **C** | Canopy and lines | 20–35 g |

The links are spring–dampers that carry tension only:
- F–K, length a·L;
- K–A, length (1 − a)·L;
- K–C, the shroud.

### 2.2 Phase 1: ejection (time step 1 µs)

- **Charge.** Black powder burns into the free volume between the engine block and the coupler bulkhead.
  - Gas yield uses the hobby ideal-gas constant R T = 22.16 ft·lbf/(lbm·°R) × 3 307 °R, i.e. **219 J of P·V per gram** **[fact: widely used value]** ([BP calculator](https://d21c22pth15itr.cloudfront.net/tools/bpcharge_calculator/)).
  - Its origin is not documented: Nakka notes he could not find a reference for it ([Nakka, Appendix E](https://www.nakka-rocketry.net/Appendix-E.htm)). So it is multiplied by an efficiency η ∈ U(0.4, 0.9), and the burn and leakage time constants are dispersed.
- **Coupler.** The friction fit breaks at F_hold. The coupler then strokes out over its 60 mm engagement and the gas vents.

### 2.3 Phase 2: free flight (time step 25 µs)

- Drag on each body and gravity.
- Cord and shroud stretch.
- **Canopy inflation** starts at shroud line stretch. It follows a fill distance n·D₀ with n ∈ U(4, 10), and CdS grows as (t / t_fill)² **[assumption: Knacke-type fill model, constants not taken from a source]**.

### 2.4 Verification

| Check | Result |
|---|---|
| Unit conversion | 219.0 J/g reproduced (a 1.8× slip was caught and fixed in development) |
| Separation speed vs work–energy | P·A·L_eng / μ → 11.8 m/s; model p50 10.8 m/s |
| Snatch tension vs Δv·√(kμ) | 230 N hand estimate; model p50 226 N (Kevlar) |
| Time-step convergence (halved steps, 60 runs) | P, v_sep, accelerations < 1 %; cord tension p50 0.1 % (max 2 %); shroud peak p50 0.2 % (max 11 %, single short spikes) |
| Deployment speed vs OpenRocket | SITL p50 22.6 m/s; OpenRocket warns of 21.6 m/s at deployment |

**Not modelled:**
- body rotation and tumbling;
- lateral offset at separation (so the zipper metric is inconclusive: < 5° in the planar model);
- canopy added mass and porosity;
- wind shear;
- cord heating;
- strain limits of elastic cord (checked after the fact, §5).

## 3. Assumption register

| ID | Quantity | Range | Status | How to close |
|---|---|---|---|---|
| A-R1 | Motor ejection charge | U(0.3, 1.0) g black powder | **A** | **Weigh the charge supplied with the reload** (or read it from the instructions) |
| A-R2 | Combustion efficiency η | U(0.4, 0.9) | A | Ground test: film the separation and compare v_sep with the predicted p50 of 10.8 m/s |
| A-R3 | Free volume | U(0.5, 0.9) L (0.40 m bay, pack 50 % porous) | D/A | Measure the bay with the pack in place |
| A-R4 | Friction-fit break force | U(10, 50) N (80 N in the low-charge study) | **A** | Spring scale on the assembled coupler |
| A-R5 | Section CdA (tumbling) | F U(0.004, 0.012), A U(0.006, 0.020) m² | A | — |
| A-R6 | Canopy CdS | 0.198 m² (OpenRocket: 0.6 m, Cd 0.7) ±7 % | S | — |
| A-R7 | Fill constant n | U(4, 10) | A | Film the deployment |
| A-R8 | Knot position a | U(0.2, 0.5) of L from F | A | Set by the build |
| A-R9 | Cord EA (N) | Kevlar U(6e3, 2.5e4); nylon U(1.5e3, 5e3); elastic U(40, 150) | **A** | **Hang a known weight W on a measured length L and measure the stretch ΔL: EA = W·L / ΔL** |
| A-R10 | Cord damping ζ | U(0.05, 0.15); 0.3–0.5 in a sensitivity case | A | Sensitivity: tension −7 %, accelerations ±3 % |
| A-R11 | Deployment state | SITL L_active: V p50 22.6, p95 33, max 41 m/s; path angle −83 to −14°; stress case 25–40 m/s | D | Motor delay choice (§6) |

## 4. Ejection and separation

### 4.1 Results

| | p50 | p95 | p99 | max |
|---|---|---|---|---|
| Peak gauge pressure (kPa) | 82 | 156 | 193 | 242 |
| Bulkhead and motor-retention force, P × 28.3 cm² (N) | 231 | 441 | 546 | 685 |
| Separation time (ms) | 13.5 | 20 | 23 | 26 |
| Separation speed (m/s) | 10.8 | 15.2 | 17.0 | 18.8 |

Figure: `rec_pressure.png`. The usual hobby design point is 10–15 psi (69–103 kPa) **[fact: common practice, same calculator source]**. The 0.3–1.0 g range spans 1–3× that.

The same pressure acts on two parts:
- **the coupler bulkhead**, pushing the forward section out;
- **the engine block and the motor's forward closure**, pushing the motor aft.

**If the retention is weaker than the coupler fit, the motor is ejected instead of the nose [inference].** Hence LD-01 tests T1 and T2 at 860 N.

**Hoop stress** in the carbon tube at 242 kPa: P·r/t = 7.3 MPa, which is negligible.

### 4.2 Separation margin

Low-charge study: charge 0.05–0.4 g, fit up to 80 N, 1 000 runs.

| Charge (g) | Separation failure rate |
|---|---|
| 0.05–0.10 | 69 % |
| 0.10–0.15 | 37 % |
| 0.15–0.20 | 15 % |
| 0.20–0.30 | 4.3 % |
| 0.30–0.40 | 0.7 % |

**Requirement [inference]:** a charge of at least 0.35 g for a fit of 50 N or less. Ground test (UKRA 4.5) with the real reload charge.

## 5. Shock cord and canopy loads

### 5.1 Cord comparison (2 000 runs each)

Figure: `rec_cord_cdf.png`.

| Cord (4 m, 0.6 m chute) | Tension p99 / max (N) | Shroud p99 / max (N) | Forward / aft accel p99 (g) | Elastic strain p99 / max |
|---|---|---|---|---|
| Kevlar (EA 6–25 kN) | 457 / 639 | 278 / 399 | 109 / 115 | — |
| Nylon (EA 1.5–5 kN) | 227 / 295 | 225 / 315 | 53 / 54 | 8 % / 10 % |
| Elastic (EA 40–150 N) | 80 / 101 | 146 / 281 | 19 / 13 | **116 % / 160 %**: beyond a bungee's usable extension, so this is optimistic |

### 5.2 Stiffness sweep (400 runs per point)

Figure: `rec_ea_sweep.png`.

| EA (N) | 50 | 200 | 500 | 1 000 | 5 000 | 20 000 |
|---|---|---|---|---|---|---|
| Tension p99 (N) | 64 | 89 | 116 | 133 | 255 | 449 |
| Aft accel p99 (g) | 10 | 16 | 21 | 27 | 57 | 111 |

The curve is flat below about 500 N and steep above about 2 kN.

### 5.3 Cord length (Kevlar)

| L (m) | 2 | 3 | 4 | 6 | 8 |
|---|---|---|---|---|---|
| Tension p99 (N) | 724 | 567 | 459 | 354 | 321 |

Longer is better: the snatch energy is spread over more stretch.

### 5.4 Other studies

| Study | Result |
|---|---|
| Deployment at 25–40 m/s (early or late motor delay) | Kevlar p99 460 N, unchanged. Separation speed, not flight speed, sets the snatch. Elastic p99 97 N. |
| Cord damping ζ 0.3–0.5 | Kevlar p99 424 N (−7 %); elastic 76 N |

### 5.5 Recommended design point: 6 m, EA ≈ 300 N, 1 000 runs each

| Case | Tension p99 / max | Shroud p99 / max | Forward / aft accel max | Strain max | Close-pass |
|---|---|---|---|---|---|
| 0.6 m chute | 100 / 121 N | 152 / 194 N | 30 / 17 g | 40 % | 41 % |
| **0.9 m chute** | **116 / 146 N** | **145 / 211 N** | **35 / 24 g** | 49 % | 45 % |
| 0.9 m chute, deployment 25–40 m/s | 134 / 152 N | 159 / 208 N | 37 / 24 g | 51 % | 29 % |

**How to build it [inference].**
- A 0.3–0.5 m Kevlar leader at the motor end, for heat (UKRA 4.5 requires protection from ejection heat).
- A main length with a measured EA of 200–500 N, about 5.5 m. In series, the soft element sets the stiffness.
- Verify the main cord's EA with the hanging-weight test (A-R9). Check that its rated extension exceeds about 60 %.

**Ratings, at 2.0 × limit for untested soft goods and attachments (LD-01 §6):**
- limit loads: cord 152 N, shroud and canopy attachment 211 N;
- **every recovery element rated ≥ 420 N** (cord, quick link, shroud bundle, both attachment eyes);
- **proof test of the attachments at 1.25 × limit ≈ 270 N.**

## 6. Parachute size and descent

Figure: `rec_descent.png`.

Recovered mass 838 g (dry plus spent G79W case), ρ = 1.2 kg/m³.

| Canopy | CdS (m²) | Descent rate | Landing energy | Time from 420 m | Drift at 8 m/s wind |
|---|---|---|---|---|---|
| 0.6 m, Cd 0.7 (current) | 0.198 | 8.3 m/s (SITL 8.1 p50) | 29 J | ~52 s | ~410 m |
| **0.9 m, Cd 0.75** | 0.477 | **5.4 m/s** | 12 J | ~78 s | ~620 m |
| 1.0 m, Cd 0.7 | 0.55 | 5.0 m/s | 10 J | ~84 s | ~670 m |

The 4.5–6 m/s target is my inference (typical hobby practice; no UKRA limit).

The trade is drift. Check the recovery-area size at the club field and the wind limit you fly to (UKRA 5.2, 6.2.3).

The drop test (LD-01 T7) settles whether 8 m/s is acceptable for the printed fins and canards. If it is not, adopt the 0.9 m canopy.

## 7. Consequences for firmware and V6

1. **V6-5 high-g accelerometer: range ±200 g.**
   - With the current Kevlar cord, snatches reach about 155 g, so ±100 g would clip.
   - With the recommended cord, about 37 g. That still exceeds the ±16 g IMU, so the high-g part is needed either way.
2. **Sample rate ≥ 1 kHz around deployment.**
   - The Kevlar snatch is a 15–20 ms half-sine (ω ≈ √(k/μ) ≈ 160–230 rad/s). The soft cord's is about 0.2 s.
   - V6-3's ≥ 1 kHz raw log should run through apogee + 5 s, not just boost and coast.
   - **Firmware:** extend the high-rate window to "BOOST start → DESCENT + 5 s", or trigger a second pre-trigger capture on a > 5 g shock in DESCENT.
3. **Landing detection is unaffected.**
   - Deployment and snatch occur within about 1 s of ejection, which is at most about 3 s after apogee, well inside `DET_T_LAND_MIN_S` = 15 s.
   - The overrun snatch, about 0.75 s after ejection, is also inside it (ADR-004 §5).
4. **Motor delay choice:** the shortest delay that lands deployment within −1 to +3 s of the predicted apogee time for the chosen motor (MC-01 table). Ejection pressure and separation do not depend on flight speed. With the recommended cord, peak tension rises about 15 % for a 25–40 m/s deployment (§5.5), and a late deployment costs altitude and adds drift.
5. **FC mounting:** it must take the recovery shock. That is 37 g with the recommended cord, against 156 g as built: check the FC tray and the battery retention.

## 8. What to measure, in order

1. Weigh the reload's ejection charge (A-R1).
2. Spring-scale the coupler fit (A-R4).
3. Hang-test the chosen cord for EA (A-R9).
4. Proof tests LD-01 T1–T4.
5. Ejection ground test with video (T8): the predicted separation speed is p50 10.8 m/s and p95 15 m/s, and the gas must not move the motor.
6. Drop test T7.
7. Re-run `rec_campaign.sh` with the measured values, narrowing A-R1, A-R4 and A-R9.

## 9. How to run

```
cd Firmware/FC-V4/Tools/recovery
python rec.py --n 2000 --cord kevlar --dep ../sitl/runs/L_active/runs.csv --out runs/R_kevlar
python rec.py --n 1000 --ea 300 --cord_len 6 --chute_d 0.9 --chute_cds 0.477 --dep ../sitl/runs/L_active/runs.csv --out runs/REC_design_09
python ld_rec_figures.py
```
