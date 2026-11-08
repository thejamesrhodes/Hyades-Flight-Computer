#!/usr/bin/env python3
"""
fc_pull.py -- download and decode the FC-V4 flight log over USB.

    python fc_pull.py                 # pull -> logs/<timestamp>/ (raw .bin + CSVs)
    python fc_pull.py --decode f.bin  # decode a saved raw log, no board needed

Firmware side: Core/Src/flashlog.c (protocol), Core/hyades/svc/logrec.h (format).
Each "log start" begins a session (BOOT record); one CSV pair per session:
  session<N>_state.csv   t_s, phase, mode, flags, q0..q3, w*_dps, a*_ms2,
                         x0..x3 (v1: bias xyz dps; v2: h m, v_up m/s, q_hat Pa, trim rad), fin0..3, loop_us, faults; v2 adds sessionN_aux.csv (1 Hz bias, GPS, dropped)
  session<N>_events.csv  t_s, phase, code, arg0, arg1, text
Records failing their CRC (torn by power loss) are counted and skipped.
Requires: pip install pyserial  (decoding needs only the standard library)
"""
from __future__ import annotations

import argparse
import binascii
import csv
import os
import struct
import time
import zlib

REC = 48
T_STATE, T_EVENT, T_BOOT, T_AUX = 0x51, 0xE5, 0xB7, 0xA7
PHASES = ["BOOT", "CAL", "SETTLE", "READY", "BOOST", "COAST", "DESCENT", "LANDED", "LOCKOUT", "CATO"]
MODES = ["OFF", "RDAMP", "FULL"]


def crc_ok(r: bytes) -> bool:
    return binascii.crc_hqx(r[:46], 0xFFFF) == struct.unpack_from("<H", r, 46)[0]


def decode(raw: bytes, outdir: str) -> None:
    os.makedirs(outdir, exist_ok=True)
    sessions: list[dict] = []
    bad = 0
    for off in range(0, len(raw) - REC + 1, REC):
        r = raw[off:off + REC]
        if r[0] == 0xFF:
            break
        if r[0] not in (T_STATE, T_EVENT, T_BOOT, T_AUX) or not crc_ok(r):
            bad += 1
            continue
        if r[0] == T_BOOT or not sessions:
            build = r[12:44].split(b"\0")[0].decode("ascii", "replace") if r[0] == T_BOOT else "?"
            hz = struct.unpack_from("<H", r, 10)[0] if r[0] == T_BOOT else 0
            ver = r[1] if r[0] == T_BOOT else 1
            sessions.append({"build": build, "log_hz": hz, "ver": ver, "state": [], "event": [], "aux": []})
            if r[0] == T_BOOT:
                continue
        s = sessions[-1]
        t_us = struct.unpack_from("<I", r, 4)[0]
        if r[0] == T_STATE:
            q = [v / 32767 for v in struct.unpack_from("<4h", r, 10)]
            w = [v / 10 for v in struct.unpack_from("<3h", r, 18)]
            a = [v / 100 for v in struct.unpack_from("<3h", r, 24)]
            f = [v / 127 for v in struct.unpack_from("<4b", r, 36)]
            loop_us, faults = struct.unpack_from("<HH", r, 40)
            ph = PHASES[r[1]] if r[1] < len(PHASES) else str(r[1])
            md = MODES[r[2]] if r[2] < len(MODES) else str(r[2])
            if s["ver"] >= 2:      # ADR-004: h, v_up, q_hat, trim replace the bias
                h, vz = struct.unpack_from("<hh", r, 30); qb = struct.unpack_from("<H", r, 34)[0] * 2
                trim = struct.unpack_from("<h", r, 44)[0] / 1e4
                x = [h / 10, vz / 100, qb, trim]
            else:
                x = [v / 100 for v in struct.unpack_from("<3h", r, 30)] + [""]
            s["state"].append([t_us / 1e6, ph, md, r[3], *q, *w, *a, *x, *f, loop_us, f"{faults:04X}"])
        elif r[0] == T_AUX:
            b = [v / 100 for v in struct.unpack_from("<3h", r, 10)]
            alt = struct.unpack_from("<h", r, 16)[0]
            dropped, wcet, fails = struct.unpack_from("<IHH", r, 20)
            ph = PHASES[r[1]] if r[1] < len(PHASES) else str(r[1])
            s["aux"].append([t_us / 1e6, ph, r[2], r[3], *b, alt, r[18] / 10, r[19], dropped, wcet, fails])
        else:
            arg0, arg1 = struct.unpack_from("<II", r, 12)
            text = r[20:46].split(b"\0")[0].decode("ascii", "replace")
            ph = PHASES[r[2]] if r[2] < len(PHASES) else str(r[2])
            s["event"].append([t_us / 1e6, ph, r[1], arg0, arg1, text])
    hdr_s = ["t_s", "phase", "mode", "flags", "q0", "q1", "q2", "q3", "wx_dps", "wy_dps", "wz_dps",
             "ax_ms2", "ay_ms2", "az_ms2", "x0", "x1", "x2", "x3", "fin0", "fin1", "fin2", "fin3",
             "loop_us", "faults"]
    hdr_e = ["t_s", "phase", "code", "arg0", "arg1", "text"]
    hdr_a = ["t_s", "phase", "nav_flags", "sats", "bx_dps", "by_dps", "bz_dps", "gps_alt_m", "hdop", "mode",
             "log_dropped", "loop_us", "imu_fails"]
    for i, s in enumerate(sessions, 1):
        with open(os.path.join(outdir, f"session{i}_state.csv"), "w", newline="") as fh:
            w = csv.writer(fh); w.writerow(hdr_s); w.writerows(s["state"])
        with open(os.path.join(outdir, f"session{i}_events.csv"), "w", newline="") as fh:
            w = csv.writer(fh); w.writerow(hdr_e); w.writerows(s["event"])
        if s["aux"]:
            with open(os.path.join(outdir, f"session{i}_aux.csv"), "w", newline="") as fh:
                w = csv.writer(fh); w.writerow(hdr_a); w.writerows(s["aux"])
        print(f"session {i}: build '{s['build']}', {s['log_hz']} Hz, "
              f"{len(s['state'])} state, {len(s['event'])} events")
    print(f"{len(sessions)} session(s), {bad} bad record(s) -> {outdir}")


def pull(port_arg: str | None) -> bytes:
    import fc_common as fc
    fc.require_serial()
    port = fc.find_port(port_arg) or fc.die("board not found")
    with fc.open_port(port) as ser:
        fc.quiet(ser)
        fc.send_cmd(ser, "log pull")
        line = fc.wait_for(ser, ("LOGDATA", "ERR"), 5.0)
        if not line or not line.startswith("LOGDATA"):
            fc.die(f"no LOGDATA: {line}")
        n = int(line.split()[1])
        print(f"pulling {n} bytes...")
        ser.timeout = 2.0
        t0 = time.monotonic()
        raw = bytearray()
        while len(raw) < n:
            chunk = ser.read(n - len(raw))
            if not chunk:
                fc.die(f"stalled at {len(raw)}/{n} bytes")
            raw += chunk
        ser.timeout = 0.2
        trailer = fc.wait_for(ser, ("LOGEND",), 5.0)
        fc.send_cmd(ser, "stream on")
    if not trailer:
        fc.die("no LOGEND trailer")
    want = int(trailer.split()[1], 16)
    got = zlib.crc32(raw) & 0xFFFFFFFF
    print(f"{n} bytes in {time.monotonic() - t0:.1f} s, crc32 {got:08X} "
          f"({'OK' if got == want else 'MISMATCH, want %08X' % want})")
    if got != want:
        fc.die("transfer corrupted; retry")
    return bytes(raw)


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port")
    ap.add_argument("--decode", help="decode an existing raw log file instead of pulling")
    ap.add_argument("--out", default=os.path.join("logs", time.strftime("%Y%m%d-%H%M%S")))
    args = ap.parse_args()
    if args.decode:
        raw = open(args.decode, "rb").read()
    else:
        raw = pull(args.port)
        os.makedirs(args.out, exist_ok=True)
        with open(os.path.join(args.out, "flightlog.bin"), "wb") as fh:
            fh.write(raw)
    decode(raw, args.out)


if __name__ == "__main__":
    main()
