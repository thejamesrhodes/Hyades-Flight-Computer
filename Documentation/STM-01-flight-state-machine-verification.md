# STM-01: flight phase machine, written out and verified by hand

**Date:** 8 November 2026.
**Code:**
- `Core/hyades/mission/phase.c` (allowed-transition table `k_allowed`, protected word);
- `detect.c` (event detectors);
- `flight.c` (arbiter);
- `app.c` (boot sequence, logger reactions).

**Diagram:** `Images/adr004/stm01_phase_machine.png` (source `.dot` alongside).

![STM-01](Images/adr004/stm01_phase_machine.png)

## 1. Method

1. Each phase's allowed exits were copied from `k_allowed` and each guard from the code that requests the transition. Paths: `Detect_Step`, the `monitor_and_arbitrate` worst-severity branch, the operator clear, and `app.c` boot.
2. For every (phase, event) pair: is the transition allowed, is it requested, and is the result what FDS-01 §3 intends?
3. Every disagreement became a finding (F1–F3), a code change and a test.
4. The same paths were then exercised by:
   - host tests (`test_phase`, `test_detect`, `test_flight`);
   - 7 680 SITL flights (MC-01 R1: 100 % in every active campaign);
   - hardware HITL (8 Nov: 9 / 9 sequences correct).

The phase word is stored with its complement and uses 16-bit codes with pairwise Hamming distance ≥ 6. Any read that fails the check is treated as corrupt (F1).

## 2. Transition table (as built)

| From | To | Guard (source) | Who requests | Notes |
|---|---|---|---|---|
| BOOT | CALIBRATE | drivers up | `app.c` boot | |
| BOOT | LOCKOUT | IMU init or self-test fail | `app.c` | |
| CALIBRATE | SETTLING | gyro bias averaged 1 s, IEKF seeded | `app.c` | |
| CALIBRATE | LOCKOUT | previous reset was IWDG | `app.c` | FDS-01 §5.2 |
| SETTLING | READY | worst = INFO for `READY_HOLD_MS`, and `ready_inhibit` false (flight build: ≥ 200 KB erased log) | arbiter | Log ring arms on entry |
| READY | SETTLING | any SOFT, **not** while a liftoff is pending | arbiter | Ring discarded |
| READY | BOOST | f_ax > 3 g for 100 ms and v_up > 8 m/s, or f_ax > 3 g for 300 ms (no nav) | `Detect_Step` | Log commits |
| **SETTLING** | **BOOST** | same liftoff rule | `Detect_Step` | **F3, new** |
| SETTLING, READY | LOCKOUT | any HARD on the pad | arbiter | |
| BOOST | COAST | t ≥ 0.1 s and f_ax < 0 for 50 ms | `Detect_Step` | |
| BOOST, COAST | CATO_SAFE | any HARD in flight | arbiter | Fins OFF |
| COAST | DESCENT | v_up < 0 for 50 ms (nav valid), or t_coast ≥ `DET_T_APOGEE_MAX_S` | `Detect_Step` | Per-motor timer (MC-01 §4) |
| DESCENT | LANDED | t ≥ 15 s and (GPS quiet 2 s, or shock > 3 g then still 2 s, or still 30 s), or t ≥ 300 s. Runs despite HARD. | `Detect_Step` | ADR-004 §5 |
| **CATO_SAFE** | **LANDED** | same landing rules | `Detect_Step` | **F2, new** |
| LOCKOUT | SETTLING | operator `faults clear`: pad only, no HARD active, `Phase_Recover` | arbiter | Full hold needed again |
| any | LOCKOUT / CATO_SAFE | phase word corrupt → `Phase_ForceSafe(in_flight)` | arbiter | **F1, fixed**: in_flight from the detector's last good phase |

**Terminal states:** LANDED, and LOCKOUT in flight (now unreachable from flight).

**Control actuation:** BOOST and COAST only (plus READY on bench builds). The mode is FULL, RATE_DAMP or OFF from the FDIR capability ladder.

## 3. Findings

### F1: corrupt phase word in flight went to LOCKOUT

`Phase_ForceSafe(&ph, false)` was called unconditionally, and LOCKOUT has no in-flight exits. A single corrupted word in coast would therefore have:
- frozen the board in LOCKOUT;
- left the canards OFF (correct);
- never declared LANDED, so the log was never closed and the beacon never came on.

**Fix:** flight versus pad is now taken from `det.last`, the detector's last good phase, held in separate RAM. A corrupt word in flight goes to CATO_SAFE, which can reach LANDED.

**Test:** `test_corrupt_word_in_flight_goes_cato_then_lands`.

### F2: CATO_SAFE could never reach LANDED

The table allowed CATO_SAFE → LANDED, but `Detect_Step` had no CATO_SAFE case. The detector call was also skipped whenever a HARD fault was present, which is the usual reason for being in CATO_SAFE.

**Fix:** CATO_SAFE uses the DESCENT landing rules, and the detector runs despite HARD in CATO_SAFE as it already does in DESCENT.

### F3: a launch from SETTLING was never sequenced

If ignition happened while the board was in SETTLING, the whole flight stayed in SETTLING: no BOOST, no log commit, no landing. That happens, for example, when a SOFT fault resets the hold just before the button is pressed. FDS-01 P5 makes "not READY" a NO-GO indication, but the vehicle cannot stop the club's ignition, and P4 says to protect the data.

**Fix:** SETTLING → BOOST is allowed on the same liftoff rule; the log starts directly. The control mode still comes from FDIR, so a SOFT estimator fault flies RATE_DAMP, not FULL.

**Tests:** `test_liftoff_from_settling_is_sequenced`, plus updated `test_phase` and `test_detect`.

**Not extended to LOCKOUT:** a board in LOCKOUT has impeached itself, so it neither actuates nor sequences.

## 4. Pairs checked and found correct

- **Liftoff while READY with a SOFT fault in the same frame:** the detectors run before the demotion, so liftoff wins (HITL-01 defect 1, still pinned).
- **HARD in DESCENT:** the CATO transition is refused (not in the table), so the board stays in DESCENT and landing still runs (HITL-01 defect 3).
- **READY demotion during a pending liftoff:** held (`Detect_LiftoffPending`, now also for SETTLING).
- **Pad knock (6 g, 60 ms):** rejected by the velocity gate. A 10 g, 200 ms knock still passes the 300 ms acc-only path only if nav is invalid. Known limit (HITL-02).
- **Operator clear in flight:** refused (`FCLR_IN_FLIGHT`).
- **Reboot or firmware update in flight:** refused by the console (unchanged).
- **Burnout chuffing:** the 50 ms persistence plus t ≥ 0.1 s gave no false burnouts in 7 680 SITL flights.
- **Apogee without nav (saturated):** the timer path was exercised by the I205W HITL case.

## 5. Open

- SPRT detectors (FDS-01 §5.3) remain a planned upgrade over the persistence counters.
- The LED state patterns (FDS-01 §5.3) are not yet implemented on FC-V4.
