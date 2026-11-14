# CAL-01 — Canard zero alignment (firmware and ground station)

**Status:** implemented 9 Nov 2026. Revised 9 Nov: dedicated ALIGN phase (enterable from READY), zero range ±90°, no timeout. Revised 10 Nov: servo scale corrected to 8.889 µs/° (§9), so the zero range is ±65°, full authority needs ±35°, and records are version 3. Host tests, simulator end-to-end tests and the target build all pass. **Not yet run on the board** (§8).

**Why.** A 0.5–3° zero error on one canard costs +27 % coast roll RMS p95 and pushes the worst peak roll past the 240 °/s requirement (ALLOC-01 §4). Servo spline indexing and linkage set each canard's zero, so it has to be measured on the assembled vehicle, not set in the code.

**Decisions (9 Nov):**
- the zero is found by jogging each canard onto a jig or laser line;
- it is stored in a flash journal;
- commands are allowed on the pad, in all builds;
- READY is held while no valid calibration is stored.

**Revision (9 Nov):**
- a session puts the board in its own phase, **ALIGN**, entered from SETTLING or READY and left only by save or cancel (both to SETTLING, so the full READY hold runs again), a HARD fault (LOCKOUT) or liftoff (BOOST, the T9 rule). No timeout;
- any zero within ±65° (the servo travel from neutral; ±90° until the 10 Nov scale correction) can be stored, so horn indexing on the spline can be taken out in software;
- READY is held while any stored zero is beyond ±35° (±60° until 10 Nov), where the pulse limit would clip the ±30° command authority.

## 1. Design

| Part | File | Role |
|---|---|---|
| Record and journal format | `Core/hyades/svc/fincal.c/.h` | 32-byte record with magic, sequence number, version and CRC-32. Scans for the newest valid record. Pure, host-tested. |
| Alignment session | `Core/hyades/mission/align.c/.h` | Phase gating, limits, session end on phase exit, angle parser. Pure, host-tested. |
| Flash storage | `Core/Src/calstore.c/.h` | Appends records to the journal and rewrites it after a log erase. |
| PWM zero | `Core/Src/pwm.c` | `PWM_SetZeroMdeg`: millidegrees to whole ticks, clamped to ±65°. The only way the zero changes. |
| ALIGN phase | `Core/hyades/mission/phase.c`, `detect.c`, `flight.c` | `PH_ALIGN`: allowed edges SETTLING/READY → ALIGN → SETTLING, BOOST, LOCKOUT (STM-01 T17, T18). Liftoff detection runs in ALIGN. |
| Integration | `Core/Src/app.c` | `cal` command, READY inhibit, log event, status token, boot report. |
| Ground station | `Tools/fc_console.py` (Canards tab, Ctrl+4), `fc_link.py`, `fc_sim.py`, `fc_pull.py` | Operator UI, reply parser, simulator, log decode. |

**The control path is untouched.**
- The session only edits the per-fin zero that `PWM_SetFin` adds to every command.
- Outside BOOST and COAST the flight loop commands every fin to 0, so in ALIGN the canard sits exactly on the trial zero being jogged.
- No servo override exists and nothing writes the PWM directly.
- In flight the zero is the stored calibration, loaded once at boot.

**Storage.**
- The journal is 4 KB at the start of log sector 5, which is 128 records.
- Records are version 3: int32 millidegree zeros at 8.889 µs/°. Version 2 records (zeros at the earlier 6.667 µs/°) are still read and converted × 3/4, which is the same pulse, so an alignment made before the scale correction holds. v1 int16 records are not read.
- A save appends one record: 8 word writes, with the CPU stalled about 0.13 ms. It never erases.
- The newest valid record wins. A torn or foreign record is skipped and never trusted.
- A firmware update only erases sectors 0–4, so the calibration survives reflashing.
- A log erase wipes sector 5, so the app writes the stored calibration back as the first record straight after the erase.
- The log now starts 4 KB later and holds 389 088 B (8 106 records).

## 2. Safety case

| Hazard | Mitigation | Where |
|---|---|---|
| A canard moves during flight | A session exists only in the ALIGN phase, which can only be entered from SETTLING or READY with no HARD fault, cannot reach READY and never actuates. `Align_Poll` ends the session the frame the phase leaves ALIGN. | `phase.c` `k_allowed`, `align.c`, `app.c` task_flight |
| Ignition during a session | Liftoff is detected in ALIGN and sequenced to BOOST (T9), the session ends and the stored zero is restored before that frame's PWM write. The log starts as in F3. | `detect.c`, `Align_Poll` |
| A session left half-done | The board stays in ALIGN, fins static, READY unreachable, until save or cancel. A USB unplug changes nothing; reconnect and carry on. A power cycle discards the trial (RAM only). Only an explicit save keeps the trial values. No timeout, so the phase can never drift into READY unattended. | `Align_Poll`, `phase.c` |
| READY reached straight after a session | Save and cancel go to SETTLING, and the READY hold counter is cleared on every SETTLING entry (STM-01 F4). | `Flight_GoPhase` |
| A zero so large the ±30° command clips at the pulse limit | READY held while any |zero| > 35°; status shows `cal=CLIPPED`, the boot report warns. | `cal_ready_ok()` |
| A wild value from a typo or a corrupt link | Each jog is limited to ±10° and each zero to ±65°. Out-of-range input is refused, not clamped. Angles are parsed as integers, with no float parsing. | `Align_Jog/Set/ParseMdeg` |
| A corrupt or torn calibration used in flight | CRC-32 plus magic, version and range checks on every read. After a save, the record is read back from flash before it is used. | `FinCal_Unpack`, `CalStore` |
| Flying uncalibrated | READY is held while no valid record exists (`CFG_REQUIRE_FIN_CAL 1`). The flight build fails to compile with it off. | `board.h`, `app.c` |
| Not knowing what flew | The calibration is written into every flight log at arming, as an `EV_CAL` event. `fc_pull.py` decodes it. | `log_cal()` |
| The log erase also erases the calibration | The stored calibration is rewritten straight after the erase. A power cut between the two leaves no calibration, which holds READY: a safe failure that needs re-alignment. | `maintenance()` |

## 3. Commands (firmware console)

```
cal                     show
cal begin               enter ALIGN (from SETTLING or READY, no HARD fault)
cal jog <fin> <deg>     nudge one canard's trial zero, |deg| <= 10
cal set <fin> <deg>     set it outright, |deg| <= 65
cal save                store the trial zeros, return to SETTLING
cal cancel              restore the stored zeros, return to SETTLING
```

Fins: 0 top, 1 right, 2 bottom, 3 left. Angles are in degrees at the servo, up to 3 decimals. One 8 MHz PWM tick is 0.014°.

Every reply ends with one machine-readable line:

```
CAL state=ALIGN stored=OK seq=3 zero=+0.120,-0.040,+0.000,+0.310 trial=+0.140,-0.040,+0.000,+0.310 journal=3/128 auth=full ready=held
```

`auth` is `full` when every zero is within ±35°, else `clipped`. The 5 Hz status line ends with `cal=OK|NONE|ALIGN|CLIPPED`. Zeros are logged in the `EV_CAL` event as int16 centidegrees.

## 4. Procedure (PLAN-01 G4)

1. Fit the alignment jig or laser line to each canard root. Restrain the airframe and keep the fin arc clear.
2. Power the flight computer, connect the ground station and wait for SETTLE or READY.
3. Open the Canards tab (Ctrl+4) and press Begin session. The phase shows ALIGN. The canards hold the stored zero, or 0 if none is stored.
4. Jog each canard onto its reference. Use Set or ±10° for a coarse offset (horn indexing), ±1° and ±0.1° to get close, then ±0.02° (1.4 ticks). Offsets beyond ±35° hold READY: refit the horn instead.
   - **Bench note, 10 Nov (BENCH-01 rev 2):** the servo takes a new target only after a 2.0 µs change, which is 0.225° or 12 of the ±0.02° jogs, and lands about ±0.15° from it. Finish with ±0.1° jogs from one side and expect about ±0.2°.
5. Press Save and check the before and after table in the confirmation. The board returns to SETTLING and reaches READY after the full hold; the tab shows the stored sequence number and READY allowed.
6. Re-check by power-cycling. The boot line prints the stored zeros, and the canards return to the jig line.
7. Polarity check (FDS-01 §6.6) as before.

## 5. Ground station

![Canards tab](Images/cal01/cal01_canards_tab.png)

- **Tiles:** session (ALIGN phase or idle), stored calibration, journal use, command authority (full or CLIPPED), and READY gate.
- **Controls:**
  - jog buttons and a Set field per canard, enabled only while a session runs;
  - Begin, enabled only in SETTLE or READY;
  - Save, which asks for confirmation and lists every change.
- **On connect:** with no calibration stored, the console warns that READY is held.
- **Simulator:** `python fc_console.py --sim` starts with no calibration. It runs the same rules and replies as the firmware (`fc_sim.py`), so the whole workflow can be rehearsed without a board.

## 6. Verification

| Check | Result |
|---|---|
| `test_fincal` | 9 tests: version 2 record converted × 3/4 to the same pulse; old record out of range rejected; round trip; every single-bit flip rejected; range; empty journal; torn record skipped; foreign data reported full and invalid; append position |
| `test_align` | 6 tests: phase and fault gating; no command without a session; 10° step and 65° range refusal; save keeps, cancel reverts; each phase exit ends the session (fault, liftoff, other); no timeout over 100 000 polls; parser accept and reject table |
| `test_phase`, `test_detect`, `test_flight` | ALIGN edges allowed and refused; liftoff from ALIGN; ALIGN holds READY, save returns through the full hold, liftoff from ALIGN sequenced (fails without the F4 fix) |
| Full host suite | 15 suites pass |
| `test_tools.py` | `CanardAlignment`: parser, status token, simulator session (ALIGN from SETTLE and READY, no timeout, 10°/90° limits, save and cancel return through SETTLE, clipped zero holds READY). 16 tests pass. |
| GUI | Driven under Xvfb against the simulator: begin, jogs including +10°, set −45°, save, READY reached (figure) |
| Target build | arm-none-eabi -O2, -Werror, clean. The flight-build guard rejects `CFG_REQUIRE_FIN_CAL 0`. |
| SITL | Unchanged: the SITL drives `Flight` directly and does not use the app layer |

## 7. Migration (read before flashing this firmware)

1. **Pull the flight log with the old firmware first.** The log now starts 4 KB later, so records written by older firmware no longer line up and cannot be decoded after the update.
2. Flash, then run **`log erase`**. Until then the old log data sits where the journal lives: the journal reads as full, and a save is refused with that message.
3. Align and save (§4). READY is held until you do.

## 8. Open items

- **Bench run on the board.** Check that:
  - save, power-cycle and reload work;
  - `log erase` rewrites the calibration;
  - a USB unplug during a session leaves the board in ALIGN with the fins static;
  - a ±30° set moves the horn as expected and a ±40° zero holds READY;
  - the servo still follows at a ±60° zero: pulses outside 1100–1900 µs are not on the data sheet (REVIEW-01 F22);
  - the jog step is visible on a protractor or laser.
- **Firmware image size.** At -O2 the image is 96 320 B against the 96 KB (98 304 B) USB-update staging buffer, leaving about 2 KB. CAL-01 added about 5 KB. The same code at -Os is 88 944 B. Before the next feature, either build at -Os or move the bench-only code (`boot_verify`, `servo_sweep`, the IMU capture) behind `CFG_FLIGHT_BUILD`.

## 9. Servo scale correction, 10 Nov 2026

- **Finding (REVIEW-01 F1).** The firmware mapped 900–2100 µs to 180°, 6.667 µs/°. The maker gives 90° for 1100–1900 µs, 8.889 µs/°. James's bench check found the canard moving 0.75 of the command, which confirms the maker's figure.
- **Change.** `SERVO_RANGE_DEG` is 135. A command of 30° is now 266.7 µs (2 133 ticks), was 200 µs.
- **Zero limits, from the pulse range.** The neutral is 1520 µs, so the travel to the nearer pulse limit is 580 µs = 65.25°.
  - largest storable zero: ±65° (`FINCAL_LIMIT_MDEG`);
  - full ±30° authority: zero within ±35° (`FIN_ZERO_FULL_AUTH_MDEG`);
  - both are compile-time checks in `pwm.c`, which now allow for the neutral not being at mid-range (REVIEW-01 F11).
- **Stored calibrations.** A version 2 record is converted × 3/4 when read. The pulse, and so the canard position found at alignment, is unchanged. The boot report says so, and the next save writes version 3.
- **On the board.** Build `75638673+01b712` flashed over USB on 10 Nov: the stored version 2 record (all zeros) was read and converted, and READY was reached.
- **Still to check by hand:** a 10° jog is 10° at the canard.
