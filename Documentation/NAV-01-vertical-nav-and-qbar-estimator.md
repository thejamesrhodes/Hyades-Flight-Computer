# NAV-01: Vertical navigation and q̄ estimator (no barometer)

**Status:** implemented and HITL-verified, 7 November 2026. **Applies to:** FC-V4 now, H743 unchanged.
**Code:** `Core/hyades/gnc/nav.c/h`, `Core/hyades/mission/detect.c/h`, wiring in `Core/Src/app.c`, constants in `board.h` §10.
**Tests:** `Tests/test_nav.c` (10), `Tests/test_detect.c` (5); HITL runs in HITL-01.

**Decisions (James, 7 Nov):**
- Scope: strapdown velocity/height, q̂, phase detectors, plus GPS aiding.
- No altitude control gate until UKRA S&T rule on §8.3 in writing.
- Motor not chosen yet, so the ±16 g IMU range is a constraint on motor choice.
- HITL host is Python, using the MATLAB parameters.

## Why it is needed
- The roll design is q-scheduled: `Kp = c/q̂` (`optimise_roll_schedule.m`, `controller.m`). The envelope runs from (2 kPa, 60 m/s) to (40 kPa, 280 m/s).
- The firmware still uses a fixed roll effectiveness `CTRL_EFF_ROLL`. HITL shows why that matters: see HITL-01, H3 finding.
- The phase FSM had the BOOST, COAST and DESCENT transitions but nothing requested them. The detectors now do.

## Architecture (as built)
A loosely coupled block, kept separate from the IEKF. Nothing aids velocity in flight, so making it SE₂(3) states inside the filter would add cost without improving attitude.

1. **Propagation, 100 Hz, trapezoidal.**
   $$\dot v^n = R\,f^b - g_\text{ref}\,e_\text{up}, \qquad \dot p^n = v^n$$
   where:
   - \(R\) is the body-to-ENU rotation from the IEKF;
   - \(f^b\) is the measured specific force (m/s²);
   - \(g_\text{ref}\) is the gravity magnitude **measured on the pad**: the mean up-component of \(Rf\) while stationary. This cancels the vertical accel bias and the scale error at 1 g.
2. **Pad.** In SETTLING and READY the state integrates every frame but is zeroed whenever the sample is stationary:
   - | |f| − g_ref | < 0.5 m/s² and |ω| < 0.1 rad/s;
   - liftoff is therefore integrated from its first high-g sample.

   HITL confirms it: G79W liftoff detected at 0.09 s with no velocity lost.
3. **q̂.**
   $$\hat q = \tfrac12\,\rho_\text{ISA}(h_\text{site}+h)\,\lVert v\rVert^2$$
   - Reported as 0 below 5 m/s.
   - Ground speed stands in for airspeed.
   - **Site:** MRC, near Twycross, `NAV_SITE_ELEV_M` = 110 m. This is an estimate and is not verified. The GPS pad average replaces it whenever there is a fix (HDOP ≤ 3, ≥ 6 satellites).
4. **Saturation.** If any accel axis is at ≥ 15.5 g, `NAV_F_SAT` is set. v, h and q̂ are then invalid until the next pad reset.
5. **GPS.** Only in DESCENT and LANDED:
   - a 2-state [h, v_up] Kalman update with a χ²(1) = 16 gate;
   - re-anchors to the fix (h = GPS, v_up = 0, P reset) if the solution is saturated, or after 5 consecutive gate rejections.

   Without the re-anchor, a saturated flight gated GPS out for the whole descent. The I205W HITL run showed this: 13 km height error and no aiding.
6. **Detectors** (`detect.c`). Each one requests a phase; `phase.c`'s one-way table decides. They are persistence counters, a simplification of the FDS-01 §5.3 SPRT, which is the H2 upgrade.

   | Edge | Rule | Constants |
   |---|---|---|
   | READY → BOOST | axial f > 3 g for 50 ms | G79W peak 8.3 g, I205W 30 g |
   | BOOST → COAST | after 0.1 s, axial f < 0 for 50 ms | |
   | COAST → DESCENT | v_up < 0 for 50 ms (nav valid), else timer 15 s after COAST entry | I205W coast 12.2 s + 20 % |
   | DESCENT → LANDED | after 5 s: \|f\| ≈ g, \|ω\| < 0.2 rad/s, GPS-aided \|v_up\| < 1.5 m/s for 2 s; else timer 300 s | |

   **Arbiter order (fixed during HITL).** Detectors run **before** the READY → SETTLING demotion. Liftoff on a fast motor clips the accelerometer, which trips `accel_rng` (SOFT) in the same frames. Under the old order the board demoted itself and flew the whole I205W flight in SETTLING with control off.
   - A HARD fault still blocks every request except landing.
   - DESCENT has no CATO edge, so a sensor lost under the parachute previously left the board in a flight phase. A flight phase refuses log pull and reboot until power is cycled.

## HITL verification (7 Nov, board build `d86713e6+…`, open loop)

| Case | Liftoff | Burnout (board / truth) | Apogee (board / truth) | Landing | q̂ error, rail → apogee | h at apogee (board / truth) |
|---|---|---|---|---|---|---|
| G79W | 0.09 s | 1.71 / 1.93 s | 9.65 / 9.61 s | 63.2 s (touchdown ≈ 61.1 s + 2 s window) | −0.9 % mean, 1.4 % max | 436.6 / 437.0 m |
| G79W, accel SF +2 % | 0.09 s | 1.71 / 1.93 s | 9.65 / 9.61 s | 63.2 s | +3.1 % mean, 3.4 % max | 445.3 / 437.0 m |
| I205W (30 g) | 0.05 s | 1.53 / 1.89 s | **16.5 / 14.0 s (timer)** | 167.0 s | **invalid, saturated** | — |

- **Error budget confirmed.** A 2 % scale-factor error gives about 3 % in q̂. The prediction was 2δv/V ≈ 4 %. Both are well inside the ±30 % the schedule tolerates.
- **Early burnout is real physics, not a bug.** Axial f goes negative when drag exceeds thrust during tail-off, about 0.2–0.35 s before OpenRocket's burnout event.
- **Descent height error with GPS aiding:** 2.3 m RMS, 5.4 m max (G79W).

## Constraint on motor selection (confirmed)
- Peak boost acceleration must be ≤ 14 g.
- The I205W (30 g peak in OpenRocket) saturates the IMU for the whole boost: v, h and q̂ are lost. Apogee then comes only from the timer (2.5 s late), and the roll schedule has no q̂.
- Options:
  - choose a ≤ 14 g motor (the G79W is 8.3 g);
  - add a q(t) table fallback from the trajectory sim (not implemented);
  - add a ±32 g+ accelerometer on the H743 board.
