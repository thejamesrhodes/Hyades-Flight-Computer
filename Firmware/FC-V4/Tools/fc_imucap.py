#!/usr/bin/env python3
"""
fc_imucap.py -- record a raw IMU noise capture from the FC-V4 over USB.

    python fc_imucap.py --hours 8            # overnight static run -> logs/imucap/<time>.imu/.json
    python fc_imucap.py --minutes 10         # quick look
    python fc_noise.py logs/imucap/<time> --fig   # then analyse

Puts the board into 'imu capture' (firmware leaves the flight loop: control and
FDIR stop, fins centred, LEDs show LOCKOUT) and records every ODR sample as raw
counts with the sensor's own sample counter, so gaps are detectable and the
time base is the sensor clock. At the end (or Ctrl+C) it sends 'reboot' and
the board restarts normally. Close the console first: one program per port.

Before an overnight run: board flat and still on something that will not be
touched or warmed (no laptop vents, no sun), USB from a powered port with
sleep/USB suspend disabled. The IMU configuration (ODR, LPF, full scale) is
the flight one and is recorded in the .json, so the result applies directly.

Files: <stem>.imu = N x 18-byte records '<Ih3h3h' (ts, temp, a[3], g[3]);
<stem>.json = metadata (IMUCAP line, config, timing, gaps). Written
incrementally (flushed every second): an interrupted run is still usable.
Requires: pip install pyserial
"""
from __future__ import annotations

import argparse
import json
import os
import re
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import fc_common as fc                                      # noqa: E402
import fc_link as link                                      # noqa: E402

LPF_FRAC = {0: 2.66, 1: 3.63, 2: 5.39, 3: 13.37}           # CTRL5 aLPF/gLPF mode -> % of ODR


def parse_imucap(line: str) -> dict:
    kv = dict(re.findall(r"(\w+)=(\S+)", line))
    ctrl = re.search(r"ctrl=((?:[0-9A-F]{2} ){6}[0-9A-F]{2})", line)
    meta = {"line": line, "odr_hz": int(kv["odr_mHz"]) / 1000.0, "a_lsb_per_g": float(kv["a_lsb_per_g"]),
            "g_lsb_per_dps": int(kv["g_lsb_per_dps_x1000"]) / 1000.0, "build": kv.get("build", "?")}
    if ctrl:
        c = [int(b, 16) for b in ctrl.group(1).split()]       # CTRL1..CTRL7
        meta["ctrl"] = ctrl.group(1)
        meta["accel_fs_g"] = 2 << ((c[1] >> 4) & 7)
        meta["gyro_fs_dps"] = 16 << ((c[2] >> 4) & 7)
        meta["lpf_pct_odr"] = LPF_FRAC[(c[4] >> 5) & 3] if c[4] & 0x10 else None
    return meta


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    g = ap.add_mutually_exclusive_group(required=True)
    g.add_argument("--hours", type=float)
    g.add_argument("--minutes", type=float)
    ap.add_argument("--port")
    ap.add_argument("--out", default=os.path.join(HERE, "logs", "imucap"))
    ap.add_argument("--note", default="", help="free text stored in the .json (setup, surface, room)")
    args = ap.parse_args()
    fc.require_serial()
    dur = (args.hours or 0) * 3600 + (args.minutes or 0) * 60
    os.makedirs(args.out, exist_ok=True)
    stem = os.path.join(args.out, time.strftime("%Y%m%d-%H%M%S"))
    port = fc.find_port(args.port) or fc.die("board not found (USB connected? console closed?)")

    with fc.open_port(port) as ser:
        fc.quiet(ser)
        fc.send_cmd(ser, "imu capture")
        line = fc.wait_for(ser, ("IMUCAP", "ERR"), 5.0)
        if not line or not line.startswith("IMUCAP"):
            fc.die(f"board refused: {line or 'no reply'} (firmware too old? in flight?)")
        meta = parse_imucap(line)
        meta.update(port=port, note=args.note, start_utc=time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
                    requested_s=dur)
        print(f"capture: {meta['odr_hz']:.1f} Hz, +/-{meta.get('accel_fs_g')} g, +/-{meta.get('gyro_fs_dps')} dps, "
              f"LPF {meta.get('lpf_pct_odr')} % ODR, build {meta['build']}")
        print(f"writing {stem}.imu  for {dur / 3600:.2f} h  (Ctrl+C stops early and keeps the data)")

        demux = link.StreamDemux()
        n = gaps = lost = bad_len = 0
        last_ts = None
        t0 = time.monotonic()
        t_print = t_flush = t0
        ser.timeout = 0.2
        try:
            with open(stem + ".imu", "wb") as fh:
                while time.monotonic() - t0 < dur:
                    chunk = ser.read(4096)
                    for ev in demux.feed(chunk):
                        if ev[0] != "frame" or ev[1] != link.MSG_IMU:
                            continue
                        p = ev[2]
                        if len(p) != 18:
                            bad_len += 1
                            continue
                        fh.write(p)
                        ts = int.from_bytes(p[0:4], "little") & 0xFFFFFF
                        if last_ts is not None:
                            d = (ts - last_ts) & 0xFFFFFF
                            if d > 1:
                                gaps += 1
                                lost += d - 1
                        last_ts = ts
                        n += 1
                    now = time.monotonic()
                    if now - t_flush > 1.0:
                        fh.flush()
                        t_flush = now
                    if now - t_print > 10.0:
                        t_print = now
                        el = now - t0
                        print(f"\r  {el / 3600:5.2f} h  {n:,} samples  {n / el:6.1f} /s  gaps {gaps} "
                              f"({lost} lost, {100.0 * lost / max(1, n + lost):.3f} %)   ", end="", flush=True)
        except KeyboardInterrupt:
            print("\nstopped by user")
        finally:
            el = time.monotonic() - t0
            meta.update(duration_s=round(el, 1), samples=n, gaps=gaps, lost=lost, bad_frames=demux.bad_frames + bad_len)
            with open(stem + ".json", "w", encoding="utf-8") as fh:
                json.dump(meta, fh, indent=1)
            fc.send_cmd(ser, "reboot")
    print(f"\n{n:,} samples in {el / 3600:.2f} h, {lost} lost in {gaps} gaps -> {stem}.imu/.json")
    print(f"board rebooting. Analyse with:  python {os.path.join('Tools', 'fc_noise.py')} \"{stem}\" --fig")


if __name__ == "__main__":
    main()
