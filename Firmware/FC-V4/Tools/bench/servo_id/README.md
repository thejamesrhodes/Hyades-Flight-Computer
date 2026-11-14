# Servo identification bench tools (BENCH-02)

Procedure, setup and what each number means: `Documentation/BENCH-02-servo-identification-bench-procedure.md`.

| File | What it does |
|---|---|
| `profiles.py` | The command profiles. `--emit` writes the firmware table `Core/hyades/svc/bench_profiles.c`; the analysis replays the same ticks. |
| `flight_cmd.csv` | Roll command of one simulated flight, per 200 Hz frame (source of the `flight` profiles). |
| `track.py` | Video to laser-dot candidates and LED brightness per frame (`VIDEO.track.npz`). Resumable. |
| `analyse.py` | Track file to runs, timing, angle, per-profile results (`.runN.json`, `.runN.npz`, `.runN.png`). |
| `synth.py` | Synthetic bench video with a known servo, to check the two scripts above. |

    python profiles.py                       # list profiles and durations
    python track.py   logs/servo_id/day/stair.mov
    python analyse.py logs/servo_id/day/*.track.npz

`track.py`, `analyse.py` and `synth.py` need numpy, OpenCV, matplotlib, ffmpeg and ffprobe. `profiles.py` needs nothing.
Keep videos out of git: `Tools/logs/` is ignored.

Changing a profile: edit `profiles.py`, run `python profiles.py --emit`, rebuild and flash, and run `make` and
`make tools` in `Tests/` (they check the firmware replay against this file).
