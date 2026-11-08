# FIG-01: roll and recovery time histories (SITL Monte Carlo, hardware HITL, ejection sim)

**Date:** 8 November 2026.
**Purpose:** show the actual roll rate, roll angle and canard deflection behind the MC-01 / LD-01 statistics, and the recovery time histories behind REC-01, so that no behaviour is hidden by the scalar metrics.
**Figures:** `Images/fig01/` (16 PNGs).
**Scripts:**
- `Tools/sitl/trace_figs.py`: re-runs seeds from each campaign's `runs.csv` with a 50 Hz trace, cached in `runs/traces/`;
- `Tools/recovery/rec_trace_figs.py`.

Markers used in this document:
- **[fact]**: from a cited source;
- **[inference]**: my reasoning;
- **[assumption]**: unverified.

## 1. How the SITL figures are built

Each campaign figure shows the same set of flights:

- **60 random seeds plus every worst case**: the top 3 by boost peak, coast peak, coast RMS and lock-in ratio, and every run that failed a requirement. All are re-run bit-exact from `runs.csv` on the corrected plant (LD-01 §2).
- **Grey:** individual flights.
- **Blue band:** p5–p95 across the traced flights. **Blue line:** median.
- **Red:** the worst-case flights.
- Dotted lines mark the median burnout and apogee.
- Roll angle is the integral of the body roll rate from ignition.
- Deflection is the mean |δ| of the four canards, in degrees.

## 2. Figure list

| Figure | Content |
|---|---|
| `sitl_roll_L_active.png` | Active, build tolerance (flight configuration) |
| `sitl_roll_L_wind12.png` | Wind 0–12 m/s |
| `sitl_roll_L_wide.png` | Wide build tolerance |
| `sitl_roll_L_shadow.png` | Shadow (canards held at 0) |
| `sitl_roll_L_fault_hardover.png` | One canard hard-over |
| `sitl_roll_overview.png` | All five campaigns side by side, plus a coast-RMS histogram |
| `sitl_roll_by_motor.png` | Active, by motor (six shortlist motors, about 25 flights each) |
| `sitl_roll_faults.png` | Stuck, slow, offset, floating and hard-over servo faults, plus q(t) |
| `sitl_roll_worst_cases.png` | The failed and worst flights in detail: body rate, fin-can spin, mean \|δ\|, dispersed parameters |
| `sitl_metric_crosscheck.png` | Summary metrics recomputed from the traces (§3) |
| `hitl_roll_all_cases.png` | All 13 hardware HITL flights of 8 Nov, pre-dither and dither builds |
| `hitl_vs_sitl_nominal.png` | HITL G79W nominal flight over the SITL G79W dispersion band |
| `hitl_roll_whole_flight.png` | HITL whole flight (dither build) with the descent artefact labelled |
| `rec_ejection_pressure.png` | Ejection pressure and coupler stroke, 60 dispersed runs |
| `rec_cord_timeseries.png` | Separation, cord tension, shroud load and section g: as built vs recommended, 60 runs each, same seeds |
| `rec_single_deployment.png` | One deployment with the event sequence |

## 3. Cross-check: do the statistics miss behaviour?

| Check | Result |
|---|---|
| Coast RMS, MC metric (truth window) vs trace (board COAST) | Points lie on the line in every campaign. **The metric is faithful.** |
| Active-flight peak, MC metric vs trace (board BOOST + COAST) | On the line. The metric's truth windows (thrust > 0; thrust off and climbing) cover the board's active phases closely, including the short window between true apogee and detected apogee. |
| **Post-apogee roll (DESCENT, fins OFF, before or at deployment)** | **Not covered by any requirement.** Active / build tolerance: most flights stay below 50 °/s, a tail reaches about 200 °/s. Wide tolerance: up to about 400 °/s. Mechanism **[inference]**: with control off and q low, the fin can, spun by its cant, drags the body through bearing friction (the MC-01 FM4 path). Consequence: cord twist at deployment and camera footage. The build tolerances bound it. |

## 4. Findings

### F1: the I205W HITL run limit-cycled at about ±450 °/s

This was recorded as "OK" in HITL-02 and ADR-004 §8, but only its phase sequence was judged.

- **Evidence:** `hitl_roll_all_cases.png`, last row. Roll oscillates at about 2–3 Hz, ±450 °/s, through boost and most of the coast, and decays as q falls.
- **Cause chain** (from the log):
  - the IMU clips at +16 g (az_meas pinned at 156.9 m/s²);
  - NAV's velocity, and so q̂, goes wrong: q̂ reads about half the true q in boost, and in late coast reads 0 while the true q is still 3 kPa;
  - the law is therefore scheduled on the wrong q̂, or on its 40 kPa fallback when q̂ is flagged invalid.
- **The current flight code does the same.** In SITL with the I205W substituted into 40 dispersed G79W flights: q̂ RMS error 79 %; boost peak p50 418 °/s, max 1 204 °/s; coast peak max 1 597 °/s.
- **Scope:** the I205W is outside the ≤ 14 g shortlist (ADR-004), so it is not a flight case. **Nothing in the firmware prevents flying it.**
- **Actions:**
  - the pre-flight sheet must check motor thrust / mass ≤ 14 g;
  - V6's ±200 g accelerometer (BP-01 V6-5) also fixes NAV for high-thrust motors;
  - root-cause the controller's behaviour when NAV saturates (fallback gain vs corrupted q̂) before any motor above 16 g is considered (PLAN-01 S9).

### F2: the trim winds to its clamp before apogee in the wide-tolerance failures

- **Evidence:** `sitl_roll_worst_cases.png`, rows 5 and 6.
- The torque-space trim (δ = trim_T / q̂) grows as q falls. With a rear-fin cant of +0.4 to +0.7°, it reaches the ±6° clamp about 2 s before apogee, and roll then creeps to about 80 °/s.
- This is the trim-clamp question raised earlier. With the build tolerance (cant ≤ 0.25°) the clamp is not reached in any traced flight: the active campaign peaks at about 5° (`sitl_roll_L_active.png`, bottom panel).
- **No change recommended [inference]:** a larger clamp buys little authority at q below 1 kPa, and the cant tolerance is the real control.

### F3: the HITL nominal flight sits at the edge of the SITL band during boost

- **Evidence:** `hitl_vs_sitl_nominal.png`.
- HITL peaks at 95 °/s against a SITL p95 of about 70 °/s. Its canards hold about −0.9° throughout.
- The HITL plant carries a fixed roll disturbance (its "nominal" body asymmetry) that needs that trim. The SITL nominal is undisturbed and only the dispersed asymmetries create roll.
- Both stay inside the SITL worst-case envelope. **This is a rig-model difference, not a defect.**

### F4: the HITL descent spin is a rig artefact

- In DESCENT the fins are OFF and the servo rests about 0.12° from zero (its dead band).
- The HITL plant has no parachute or cord torsion, so the body spins up to about 140 °/s at q ≈ 40 Pa (G79W) or 420 °/s (wind case). It is labelled as such in `hitl_roll_whole_flight.png`.
- **It is still a reminder:** canard zero (G4, ≤ 0.1°) and the servo dead band set the residual torque in any unpowered phase.

### F5: hard-over boost oscillation

- **Evidence:** `sitl_roll_faults.png`, hard-over panel.
- Some hard-over flights oscillate at about ±750 °/s for 1–2 s in boost. The 4× gain boost then sits at the delay limit, as ADR-004 §3.4 anticipated.
- This is consistent with MC-01 (hard-over is the residual hazard), but the traces show its oscillatory character, which the peak statistic does not.
- **No pitch-over occurs** (AoA ≤ 15°).

### F6: recovery

- **Pressure:** peak pressure arrives 4–12 ms after ignition and the coupler leaves the tube at 8–30 ms. The as-built charge range spans 0.3–2.2× the 10–15 psi design band.
- **Cord and section loads:** the as-built Kevlar cord gives 15–20 ms snatch spikes to 500 N and section accelerations above 100 g (log scale in the figure). The recommended cord gives 50–100 N, smooth, below 16 g in the median.
- **Overrun:** in both configurations the separation trace shows the aft section's overrun after the canopy fills, sometimes back to the joined spacing.

## 5. Corrections to earlier documents

- **ADR-004 §8 and HITL-02:** "I205W OK" means the phase sequence was correct. Roll limit-cycled (F1). The I205W is not a flight motor.
- **PLAN-01:** S9 added (NAV saturation and the roll schedule; motor thrust check on the pre-flight sheet) and S10 added (post-apogee roll).
