# BENCH-01 rev 2 re-analysis scripts

These produce every number in `Documentation/BENCH-01-canard-jog-laser-test.md` rev 2.
They are working scripts, kept as run. `7983` is setup A (bare), `7984` is setup B (flight-like).

## Data kept here

| File | What |
|---|---|
| `trk_<n>.npz` | Per-frame dot track. `out[:,0:2]` core centroid x, y (px, portrait frame), `out[:,8:10]` halo centroid, used where the core is missing |
| `pts_<n>.txt` | Frame timestamps from `ffprobe -show_entries packet=pts_time` |
| `on_<n>.npy`, `burst_<n>.npy` | Audio click onsets, raw and merged within 70 ms (video time, container audio offset applied) |

The videos and the 48 kHz audio are not in the repo.

## Run order

From the tracks (no video needed):

```
python3 table.py > table_out.txt      # per-jog table, tab_<n>.pkl
python3 dyn2.py                       # second-order fits, dynfit2.pkl, dynfit.png
python3 statics.py                    # scale, 1 deg jog sizes, 3-jog lattice, repeat visits
python3 relcheck.py                   # press against release click timing
python3 logcheck.py ../data/console_2026-11-09.txt   # jog sequence against the console log
python3 fig.py                        # bench01_reanalysis.png
```

From the videos (only to regenerate the data above):

```
python3 track.py 7983.mp4 7983 435 520 60 720 0,72,240,420 540,620,330,540
python3 track.py 7984.mp4 7984 610 700 100 720 210,290,360,520 280,520,40,150
ffmpeg -i 7983.mp4 -vn -ac 1 -ar 48000 a_7983.wav     # and 7984
python3 ev.py && python3 ev2.py       # click onsets and bursts
python3 servo_sound2.py               # slew-noise onset after the release click (BENCH-01 §3.6)
```

## Session-specific constants

- `table.py`: jog sequences `SEQ`, start angles `START`, and `FINE83`, the hand-checked press-click times of A's 0.1° run (one click at 39.68 s is inferred; it lies between two moves, so no result depends on it).
- `ev.py`: `A0`, the audio stream start in each file (0.181 s and 0.054 s).
- `dyn.py`: `TEXP`, the 3.5 ms exposure used to blur the model.
- B's restricted zone (commanded above −21°) is excluded by index in `statics.py` (`kr`) and by jog number in `dyn2.py`.

## Needs

Python 3 with numpy, scipy, matplotlib and OpenCV (`track.py` only).
