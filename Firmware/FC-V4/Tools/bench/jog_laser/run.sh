#!/bin/sh
# BENCH-01 canard jog laser test: video -> dot track -> per-jog metrics -> figures.
# Usage: ./run.sh <dir with 7983.mp4 and 7984.mp4>   (videos are not in the repo; tracks are in data/)
# Session-specific constants: the log windows in match.py and the starting angles in steps2.py.
set -e
cd "$(dirname "$0")/data"
V=${1:-}
if [ -n "$V" ]; then
  python3 ../track.py "$V/7983.mp4" t83.npy 560
  python3 ../track.py "$V/7984.mp4" t84.npy 740
  for f in 7983 7984; do ffmpeg -v error -y -i "$V/$f.mp4" -vn -ac 1 -ar 48000 -f s16le a$f.raw; done
  python3 ../audio.py
fi
export PYTHONPATH=..
python3 ../match.py console_2026-11-09.txt
python3 ../steps2.py > /dev/null
python3 ../analyse.py
python3 ../an2.py
python3 ../fig_final.py
