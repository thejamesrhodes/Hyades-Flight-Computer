# BENCH-02: servo identification, bench procedure and tools

**Rev 1, 14 November 2026.** **Status:** procedure, firmware and analysis tools written and host-tested. **Not yet run on the board, and no recordings yet** (§8 lists exactly what has and has not been checked).
**Applies to:** FC-V4, Blue Bird BMS-127WV+, direct drive, 5 V servo rail from USB (this board's limit).
**Code:** `Core/hyades/svc/bench.c/h`, `bench_profiles.c` (generated), `bench` command in `Core/Src/app.c`, `CFG_BENCH_PROFILE` in `board.h`; tools in `Tools/bench/servo_id/`.
**Tests:** `Tests/test_bench.c` (6), `Tests/test_tools.py::BenchProfiles` (3).
**Supersedes for method:** BENCH-01 (hand jogs, no timing mark). BENCH-01's numbers stand until this test replaces them.

## 1. Why

BENCH-01 gave the static behaviour of the servo (0.225° threshold, landing scatter, lash) from hand jogs, and only a bound on timing (dead time < about 13 ms), because nothing tied a video frame to a command. The roll loop's margins depend on the timing and the small-signal response, and a simulated flight commands the canards within 0 to −1.8° with frame-to-frame changes of one or two timer ticks (0.014–0.028°) plus the dither. That regime, and the dynamics, are what this test measures.

| Model parameter (SITL name) | Profile | Taken from |
|---|---|---|
| Threshold `sv_sod` | `creep` | command travelled between two jumps of the canard |
| Landing error `sv_land` | `stair`, `creep` | scatter of settled levels about the command |
| Lash `sv_lash` | `stair` | same level approached from below and from above |
| Dead time `sv_tau` | `steps` | time from pulse start to 10 % of the move, less 1.5 ms pulse read |
| `sv_wn`, `sv_zeta` | `steps` 0.5–5°, `sine03/1/3` | rise, overshoot; gain and phase against frequency |
| Rate limit `sv_rate` | `steps` 20°, 30° | peak rate |
| What the dither does | `creepd`, `sine1d`, `flight` vs `flightn` | error and 50 Hz ripple with and without |
| Pulse scale (10° is 10°) | `bench hold` and a tape measure | §4 |
| The whole model, as flown | `flight` | 6 s of a simulated flight's roll command replayed |

## 2. What was built

- **Firmware.** `bench run <profile> <fin>` plays a stored command profile to the chosen canard at 200 Hz. Both board LEDs (D6, D8) become a timing mark: off for 1.5 s, then on/off every 97 frames (0.485 s) for the length of the profile, then off for 1.5 s. The LED and the servo pulse of a frame change at the same instant (`PWM_Commit`).
  - Every profile's length is 97 × a different odd number, so the count of LED half periods tells the analysis which profile a video holds.
  - Profiles are integer tick sequences. The firmware prints the CRC-32 of what it played (`BENCH done ... ok`); the analysis replays the same ticks from `profiles.py`. `test_bench.c` checks the two agree for every profile.
  - Only in SETTLING or READY with no HARD fault. A liftoff, a fault or an alignment session ends a run at once and the flight law's commands stand.
  - **Bench builds only:** `CFG_BENCH_PROFILE 1`. A build with `CFG_FLIGHT_BUILD` refuses it (`#error`).
- **`bench hold <fin> <deg>`** holds one angle until `bench stop` (for §4).
- **Tools** (`Tools/bench/servo_id/`): `profiles.py` (definitions, firmware table, replay), `track.py` (video to dot and LED series), `analyse.py` (timing, angle, results, figure), `synth.py` (synthetic video for checking the tools).

Cost: about 7.4 KB of flash in a cross-build (GCC 13.2): 82 288 B without, 89 640 B with. The CubeIDE image was 79 352 B on 10 Nov, so about 86.7 KB is expected against the 98 304 B USB-update limit (inference: the two compilers differ by about 3 KB on the same source).

## 3. Setup

| Item | Do this | Why |
|---|---|---|
| Airframe | Clamp it. Laser on canard 2 (bottom), dot on a flat wall 1.5–3 m away, beam roughly square to the wall at zero | Any movement of the rocket reads as canard angle |
| Dot's path | ±30° sweeps a line 1.15 × the wall distance long (2.3 m at 2 m). Keep all of it on the wall | `stair` and `steps` go to ±30° |
| Phone | Tripod or wedged solid, not hand-held. Turn it so the dot travels along the **long** side of the picture and the path fills most of it | Resolution; and see rolling shutter below |
| Board LEDs | In the picture, at the **same height in the picture as the dot's path** (beyond one end of it, not on it). Out of focus is fine | The sensor reads out line by line. LEDs on another line of the picture are timed up to 4 ms differently. The analysis reports the offset |
| Laser module | Keep the module itself out of the picture if you can | Its red glow moves with the canard; the analysis rejects it, but less to reject is better |
| Light | Dim the room. Lock exposure and focus on the wall (hold to lock), then pull exposure down until the dot is a small spot, not a white blob | A clipped blob loses precision |
| Camera mode | Slo-mo 240 fps, 1080p if offered | 4.2 ms per frame |
| Power | USB 5 V. Run **one fin** (`2`), not `all` | Four servos stepping together can sag the USB rail. If the board resets on a 30° step, say so |
| During a run | Do not touch the bench, the cable or the phone | |

A lit room also works (the detector was checked on the BENCH-01 video, lit and cluttered: dot found in 99.5 % of frames), but dim is better.

## 4. Scale check with a tape measure (10 minutes, once)

Closes the open item "a 10° command is 10° at the canard" (REVIEW-01 F1).

1. Tape a tape measure to the wall along the dot's path. Measure D, the distance from the canard's hinge line to the wall along the beam at zero.
2. `bench hold 2 0`, read the dot's position. Then `bench hold 2 10`, `bench hold 2 -10`, `bench hold 2 20`, `bench hold 2 -20`, reading each. `bench stop`.
3. Angle = atan(distance from the zero reading / D).

Expect each within about 1 % of the command plus the servo's ±0.15° landing scatter. At D = 2 m, 10° is 353 mm and 20° is 728 mm; 1 mm is 0.03°. Unequal + and − readings mean the wall is not square to the beam (1° of wall angle gives about 0.3 % at 10°).

## 5. Recording

One video per profile. For each: start recording, type the command, wait for `BENCH done ... ok`, stop recording. Several profiles in one video also work (each run is found from its own LED start).

| Order | Command | Run time | What it is |
|---|---|---|---|
| 1 | `bench run sync 2` | 7 s | No motion. Checks the LEDs are seen |
| 2 | `bench run stair 2` | 40 s | Levels to ±12° in 1.5° steps, twice, then ±30° in 5°. **Calibrates the picture for every later run** |
| 3 | `bench run creep 2` | 29 s | 0.4°/s triangle to ±2° |
| 4 | `bench run creepd 2` | 30 s | The same with the flight dither |
| 5 | `bench run steps 2` | 38 s | Steps of 0.1 to 30° both ways |
| 6 | `bench run sine03 2` | 31 s | 0.3° sines, 1 to 40 Hz |
| 7 | `bench run sine1 2` | 32 s | 1° sines |
| 8 | `bench run sine1d 2` | 33 s | 1° sines with dither |
| 9 | `bench run sine3 2` | 34 s | 3° sines |
| 10 | `bench run flight 2` | 12 s | Simulated flight command with dither, as flown |
| 11 | `bench run flightn 2` | 13 s | The same without dither |

- `bench` alone lists the profiles. `bench stop` ends a run.
- **After 1 and 2, stop and have them checked** before recording the rest. They are the first real footage through the tools (§8).
- If the phone or the rocket moves after `stair`, record `stair` again: the picture-to-angle map comes from it.
- Every profile except `sync` starts with 0, +10°, −10°, 0. The analysis uses it to check the picture against the `stair` map.
- Files: copy the **original** 240 fps files (not a version with the slow-motion effect baked in) into `Firmware/FC-V4/Tools/logs/servo_id/<date>/`. `Tools/logs/` is ignored by git. The console log saves itself in `Tools/logs/console/`.

## 6. Analysis

    python track.py   logs/servo_id/<date>/stair.mov        # about 1 min per minute of video; resumable
    python analyse.py logs/servo_id/<date>/*.track.npz

- `track.py` keeps, per frame, the eight strongest laser-like spots (red rim, white core where clipped, compact) and the brightness of a 16 px grid.
- `analyse.py`:
  1. finds the cells that blink at 0.485 s and reads each run: profile, and a straight-line fit of camera time against flight-computer frame through all LED edges (the two clocks need not agree);
  2. picks the dot: not a fixed light, not the LEDs, on the longest straight run of places a spot has been;
  3. `stair`: fits position to angle from its 88 levels, s = (p₀ + p₁u)/(1 + p₂u) with u = tan θ (exact for a flat wall and a pinhole camera), and writes `cal.json`. Other runs use `cal.json`;
  4. writes `<video>.runN.json` (results), `.npz` (aligned series) and `.png` (command, measured, error, LED).
- Times are flight-computer time from RUN frame 0, whose pulse starts with the first LED edge. A servo reads a pulse at its end, about 1.5 ms later. "Delay from pulse start" includes that.
- The joint fit of the servo model to all runs is **not written yet**. It needs real recordings to be worth tuning; the per-profile numbers in §1 come out now.

## 7. Limits

- **Supply:** USB 5 V. Speed and torque are lower than on a stiff rail, and the result is for this board only (later boards: 7.4 V).
- **No air load.** The canard is free; in flight the hinge moment loads the servo.
- **Laser on the canard** adds mass and a lead. BENCH-01 setup B suspected the lead of snagging.
- **Rolling shutter:** removed only by putting the LEDs level with the dot's path. The analysis reports the offset as a fraction of the picture height; the worst case is that fraction × one frame time.
- **Frame time 4.2 ms.** Timing of a single edge is to one frame; the line fit through all LED edges brings the run's time base to about ±0.1–0.2 ms (synthetic check, §8).

## 8. What has been checked, and what has not

| Check | Result |
|---|---|
| Host tests (`make`, 16 suites) | pass. `test_bench.c`: the firmware replay gives the CRC `profiles.py` computed for all 11 profiles; LED timing; mask, hold, stop |
| Tool tests (`make tools`) | pass. The firmware table is the one `profiles.py` generates |
| Cross-build with the CubeIDE flags (`-Werror`), public ST HAL | compiles and links; sizes in §2 |
| Synthetic videos through `track.py` + `analyse.py`, six profiles, known servo, camera clock 80 ppm off | profile identified every time; timing error −0.1 to +0.2 ms; angle error 0.014–0.017° rms; threshold 0.225° recovered as 0.225°, rate limit 600°/s as 601°/s, lash 0.135° as 0.143° |
| Dot detection on real footage | BENCH-01 video 7983 (lit room, red LEDs and the laser module in view): found in 99.5 % of frames, 0.066 px rms at rest |
| Tools in the PC's Linux shell (ffmpeg 4.4) | run |
| **CubeIDE build, flash, a run on the board** | **not done** (the console was closed and the board unplugged when this was written) |
| **LED detection on real footage** | **not done.** The synthetic LEDs are clean; real ones may be small or clipped. `sync` and `stair` first, then check |
| **Phone file format** (HEVC, variable frame timing) | not seen yet. The tools read the file's own timestamps |

## 9. Files

- `Documentation/BENCH-02-servo-identification-bench-procedure.md` (this)
- `Firmware/FC-V4/Core/hyades/svc/bench.h`, `bench.c`, `bench_profiles.c`
- `Firmware/FC-V4/Core/Src/app.c` (`bench_frame`, `cmd_bench`), `pwm.c` (`PWM_SetFinTicks`), `leds.c` (`Leds_Bench`), `Core/Inc/board.h` (`CFG_BENCH_PROFILE`)
- `Firmware/FC-V4/Tests/test_bench.c`, `test_tools.py`
- `Firmware/FC-V4/Tools/bench/servo_id/`: `profiles.py`, `flight_cmd.csv`, `track.py`, `analyse.py`, `synth.py`, `README.md`
- `Firmware/FC-V4/Tools/sitl/sitl.c`: `trace_dec` and the `roll_cmd` trace column (source of `flight_cmd.csv`)
