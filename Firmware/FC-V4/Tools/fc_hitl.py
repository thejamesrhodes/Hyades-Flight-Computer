#!/usr/bin/env python3
"""
fc_hitl.py -- PC-simulation-in-the-loop host for the FC-V4 (HITL-01).

    python fc_hitl.py static --frames 100000          # H0 exit: link soak on the pad
    python fc_hitl.py flight --motor I205W            # H1: open-loop flight replay
    python fc_hitl.py flight --motor G79W --closed    # fins fed back into the plant
    python fc_hitl.py flight --motor I205W --acc-sf 0.02 --seed 3

Needs a CFG_HITL=1 build on the board and nothing else holding the port
(close fc_console). Each run reboots the board first (phases are one-way), so
it starts from BOOT -> SETTLING with the real IMU, then goes live on the first
host sample: from then on every 10 ms frame consumes exactly one simulated IMU
sample and returns one REPLY (lockstep), and the GPS parser reads simulated
NMEA instead of the receiver.

Flight runs hold on the pad (nose up, static) until the board reports READY,
then ignite and run to landing + 15 s. Results go to logs/hitl/<time>_<run>.csv
(truth and board, one row per frame) and a .json summary that is also printed:
event times board vs truth, q_hat error rail-exit -> apogee, apogee height
error, link statistics.

Plant: hitl_model.py (OpenRocket trajectory inputs + the Matlab roll model).
Requires: pip install pyserial numpy
"""
from __future__ import annotations

import argparse
import json
import math
import os
import struct
import re
import sys
import time

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import fc_common as fc                                      # noqa: E402
import fc_link as link                                      # noqa: E402
import hitl_model as hm                                     # noqa: E402

MSG_SAMPLE, MSG_REPLY, MSG_NMEA = 0x10, 0x11, 0x12
SAMPLE = struct.Struct("<I3f3fI")                            # 32 B
REPLY = struct.Struct("<IBBHHBB4f4f4f")                      # 60 B
REPLY_FIELDS = ("idx", "phase", "mode", "faults", "loop_us", "nav_flags", "hitl_flags",
                "f0", "f1", "f2", "f3", "qw", "qx", "qy", "qz", "h", "v_up", "speed", "qbar")
NAV_F_INIT, NAV_F_SAT, NAV_F_GPS_AIDED = 1, 2, 4
SF_FAIL = 1
MOTORS = {"I205W": "or_sim2_I205W.csv", "G79W": "or_sim1_G79W.csv"}


def _board_frame_hz() -> int:
    """FRAME_HZ from Core/Inc/board.h, so the host steps the world by the board's frame."""
    p = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "Core", "Inc", "board.h")
    with open(p, encoding="utf-8") as f:
        m = re.search(r"#define\s+FRAME_HZ\s+(\d+)u", f.read())
    return int(m.group(1)) if m else 100


FRAME_HZ = _board_frame_hz()
FRAME_DT = 1.0 / FRAME_HZ


def frame(msg_id: int, payload: bytes) -> bytes:
    body = bytes([msg_id, len(payload)]) + payload
    return b"\xAA\x55" + body + struct.pack("<H", link.crc16(body))


class Link:
    """Synchronous lockstep link (one program owns the port)."""

    def __init__(self, port: str | None, log_lines) -> None:
        self.port_arg = port
        self.ser = None
        self.dmx = link.StreamDemux(resync=True)
        self.log_lines = log_lines
        self.replies: list[tuple] = []
        self.timeouts = 0

    def open(self, timeout_s: float = 15.0) -> None:
        p = fc.wait_port(self.port_arg, timeout_s)
        if not p:
            fc.die("FC-V4 not found (close fc_console; is the board plugged in?)")
        self.ser = fc.open_port(p)
        self.ser.timeout = 0.002
        self.dmx = link.StreamDemux(resync=True)

    def close(self) -> None:
        if self.ser:
            try:
                self.ser.close()
            except Exception:
                pass
            self.ser = None

    def pump(self) -> None:
        n = self.ser.in_waiting
        data = self.ser.read(n if n else 1)
        for ev in self.dmx.feed(data):
            if ev[0] == "line":
                self.log_lines(ev[1])
            elif ev[0] == "frame" and ev[1] == MSG_REPLY and len(ev[2]) == REPLY.size:
                self.replies.append(REPLY.unpack(ev[2]))

    def cmd(self, s: str) -> None:
        self.ser.write((s + "\r").encode())
        self.ser.flush()

    def wait_line(self, text: str, timeout_s: float) -> bool:
        seen = []
        orig = self.log_lines
        self.log_lines = lambda l: (seen.append(l), orig(l))
        t_end = time.monotonic() + timeout_s
        try:
            while time.monotonic() < t_end:
                self.pump()
                if any(t in l for t in ((text,) if isinstance(text, str) else text) for l in seen):
                    return True
            return False
        finally:
            self.log_lines = orig

    def step(self, idx: int, acc, gyro_dps, nmea: bytes, flags: int = 0, timeout_s: float = 0.25):
        out = b""
        for k in range(0, len(nmea), 96):
            out += frame(MSG_NMEA, nmea[k:k + 96])
        out += frame(MSG_SAMPLE, SAMPLE.pack(idx, *map(float, acc), *map(float, gyro_dps), flags))
        self.ser.write(out)
        t_end = time.monotonic() + timeout_s
        while time.monotonic() < t_end:
            self.pump()
            while self.replies:
                r = self.replies.pop(0)
                if r[0] == idx:
                    return dict(zip(REPLY_FIELDS, r))
        self.timeouts += 1
        return None


def reboot_to_settle(lk: Link, log) -> None:
    """Every run starts from a fresh boot (phases are one-way). A HITL build
    accepts 'reboot' in a flight phase while the seam is live; anything else
    refusing it is a hard stop, not a warning."""
    lk.open()
    lk.cmd("stream off")
    lk.cmd("reboot")
    if not lk.wait_line("rebooting", 3.0):
        lk.close()
        fc.die("board refused 'reboot' (in flight without HITL live?) -- power-cycle it")
    lk.close()
    time.sleep(2.0)
    lk.open(20.0)
    if not lk.wait_line(("->SETTLE", "->READY"), 15.0):
        lk.close()
        fc.die("board did not reach SETTLING after the reboot")
    lk.cmd("stream off")
    time.sleep(0.3)


def run(args) -> dict:
    stamp = time.strftime("%Y%m%d-%H%M%S")
    out_dir = os.path.join(HERE, "logs", "hitl")
    os.makedirs(out_dir, exist_ok=True)
    tag = f"{stamp}_{args.mode}" + (f"_{args.motor}" if args.mode == "flight" else "")
    if getattr(args, "name", None):
        tag += "_" + args.name
    out_dir = getattr(args, "out_dir", None) or out_dir
    os.makedirs(out_dir, exist_ok=True)
    lines_f = open(os.path.join(out_dir, tag + ".txt"), "w", encoding="utf-8")
    events = []

    def on_line(l: str) -> None:
        lines_f.write(l + "\n")
        e = link.parse_event(l)
        if e:
            events.append((time.monotonic(), e))
            print("   board:", l)

    P, psrc = hm.load_params(args.params)
    motor = hm.load_openrocket(os.path.join(hm.SIM_DIR, MOTORS[args.motor]))
    sim = dict(getattr(args, "sim", None) or {})
    if "vib" in sim:
        sim["vib"] = hm.VibCfg(**sim["vib"])
    cfg = hm.SimCfg(site_elev=args.elev, wind_enu=(args.wind, 0.0, 0.0), cl0=args.cl0,
                    use_fins=args.closed, seed=args.seed, **sim)
    imu = hm.ImuCfg(acc_sf=(args.acc_sf, args.acc_sf, args.acc_sf))
    gcfg = hm.GpsCfg(**(getattr(args, "gps_cfg", None) or {}))
    veh = hm.Vehicle(motor, P, cfg, imu, gcfg)
    injections = list(getattr(args, "inject", None) or [])     # [{"kind","t0","t1"}], s after ignition
    gps_fault = dict(getattr(args, "gps_fault", None) or {})
    print(f"plant: {motor.name}, params={psrc}, site {args.elev:.0f} m, wind {args.wind} m/s, "
          f"{'closed' if args.closed else 'open'} loop, acc SF {args.acc_sf:+.3f}")

    lk = Link(args.port, on_line)
    reboot_to_settle(lk, print)
    rows, idx, t_wall0 = [], 0, time.monotonic()
    fin = np.zeros(4)
    t_world = 0.0
    phase_t: dict[str, float] = {}
    pre_ign: list[str] = []
    ignite_at = None
    max_frames = args.frames if args.mode == "static" else int((args.pad_max + 400.0) * FRAME_HZ)
    landed_t = None
    try:
        while idx < max_frames:
            veh.frame(fin, FRAME_DT)
            t_world += FRAME_DT
            nmea = veh.gps_bytes(t_world) if args.gps else b""
            acc, gyr, flags = veh.imu.out_a.copy(), veh.imu.out_g.copy(), 0
            tf = veh.t if veh.ignited else -1e9
            for inj in injections:
                if inj["t0"] <= tf < inj["t1"]:
                    if inj["kind"] == "fail":
                        flags |= SF_FAIL
                    elif inj["kind"] == "nan":
                        acc[:] = float("nan")
                    elif inj["kind"] == "sat":
                        acc[:] = 16.0 * hm.G0 - hm.G0 / 2048.0
                    elif inj["kind"] == "gyro_spike":
                        gyr[2] = 2047.0
            if nmea:
                off = gps_fault.get("off")
                garb = gps_fault.get("garbage")
                if off and off[0] <= tf < off[1]:
                    nmea = b""
                elif garb and garb[0] <= tf < garb[1]:
                    nmea = bytes(veh.rng.integers(32, 127, len(nmea), dtype=np.uint8)) + b"\r\n$GPGGA,1,2,3*00\r\n"
            r = lk.step(idx, acc, gyr, nmea, flags)
            idx += 1
            if r is None:
                continue
            if args.closed:
                fin = np.array([r["f0"], r["f1"], r["f2"], r["f3"]])
            ph = link.PHASES[r["phase"]] if r["phase"] < len(link.PHASES) else "?"
            if ph not in phase_t:
                phase_t[ph] = veh.t if veh.ignited else -1.0
            if not veh.ignited and ph not in pre_ign:
                pre_ign.append(ph)
            tr = veh.truth()
            rows.append([idx, veh.t if veh.ignited else -veh.t_pad, ph, r["mode"], r["faults"], r["loop_us"],
                         r["nav_flags"], r["hitl_flags"], r["h"], r["v_up"], r["speed"], r["qbar"],
                         r["f0"], r["qw"], r["qx"], r["qy"], r["qz"],
                         tr["h"], tr["vz"], tr["speed"], tr["qbar"], tr["mach"], tr["p_dps"], tr["delta_deg"],
                         tr["qw"], tr["qx"], tr["qy"], tr["qz"], float(gyr[2]), float(acc[2]), r["f1"], r["f2"], r["f3"]])
            if args.mode == "flight":
                if ignite_at is None:
                    force = getattr(args, "ignite_regardless", False) and veh.t_pad >= args.pad_min
                    if force or (ph == "READY" and (r["nav_flags"] & NAV_F_INIT) and veh.t_pad >= args.pad_min):
                        ignite_at = idx
                        print(f"   host: READY after {veh.t_pad:.1f} s on the pad -> IGNITION")
                        veh.ignite()
                    elif veh.t_pad > args.pad_max:
                        print("   host: board never reached READY -- abort")
                        break
                elif veh.landed:
                    landed_t = landed_t if landed_t is not None else veh.t
                    if veh.t > landed_t + args.post_land:
                        break
                if ph in ("LOCKOUT", "CATO") and not getattr(args, "run_through", False):
                    print(f"   host: board in {ph} -- stopping")
                    break
            if idx % 1000 == 0:
                print(f"   {idx:7d} frames  t={veh.t:6.1f}  {ph:8s} h={r['h']:7.1f} (true {tr['h']:7.1f}) "
                      f"q={r['qbar']:7.0f} (true {tr['qbar']:7.0f})  loop={r['loop_us']} us")
    except KeyboardInterrupt:
        print("interrupted")
    wall = time.monotonic() - t_wall0
    lk.cmd("stream on")
    lk.close()
    lines_f.close()

    hdr = ["frame", "t", "phase", "mode", "faults", "loop_us", "nav_flags", "hitl_flags", "h", "v_up", "speed",
           "qbar", "fin0", "qw", "qx", "qy", "qz", "h_true", "vz_true", "speed_true", "qbar_true", "mach",
           "p_true_dps", "delta_true_deg", "qw_true", "qx_true", "qy_true", "qz_true", "gz_meas_dps",
           "az_meas", "fin1", "fin2", "fin3"]
    csv_path = os.path.join(out_dir, tag + ".csv")
    with open(csv_path, "w") as f:
        f.write(",".join(hdr) + "\n")
        for r in rows:
            f.write(",".join(f"{x:.6g}" if isinstance(x, float) else str(x) for x in r) + "\n")

    summ = summarise(rows, hdr, phase_t, veh, motor, idx, lk.timeouts, wall, psrc, args)
    summ["name"] = getattr(args, "name", "")
    summ["pre_ignition_phases"] = pre_ign
    summ["csv"] = csv_path
    summ["imu_internal_clips"] = veh.imu.clip_count
    summ["board_events"] = [e[1] for e in events]
    with open(os.path.join(out_dir, tag + ".json"), "w") as f:
        json.dump(summ, f, indent=2)
    print(json.dumps(summ, indent=2))
    print(f"log: {csv_path}")
    return summ


def summarise(rows, hdr, phase_t, veh, motor, n_sent, timeouts, wall, psrc, args) -> dict:
    s = {"mode": args.mode, "frames_sent": n_sent, "replies": len(rows), "reply_timeouts": timeouts,
         "wall_s": round(wall, 1), "frames_per_s": round(n_sent / max(wall, 1e-9), 1), "params": psrc}
    if not rows:
        return s
    a = {h: [r[i] for r in rows] for i, h in enumerate(hdr)}
    loop = np.array(a["loop_us"], float)
    missing = int(np.sum(np.array(a["hitl_flags"]) & 1))
    s.update({"board_missing_samples": missing, "loop_us_max": int(loop.max()), "loop_us_p99": int(np.percentile(loop, 99)),
              "phases_seen": list(phase_t.keys())})
    if args.mode != "flight" or not veh.ignited:
        return s
    ev_true = {"BOOST": 0.0, "COAST": motor.t_burnout, "DESCENT": veh.t_apogee}
    s["events"] = {k: {"board_s": round(phase_t[k], 3) if k in phase_t else None,
                       "truth_s": None if v is None else round(v, 3)} for k, v in ev_true.items()}
    s["events"]["LANDED"] = {"board_s": round(phase_t.get("LANDED", float("nan")), 3) if "LANDED" in phase_t else None}
    t = np.array(a["t"], float)
    qb, qt = np.array(a["qbar"], float), np.array(a["qbar_true"], float)
    asc = (t > 0.1) & (t < (veh.t_apogee or 0)) & (qt > 2000.0)
    if asc.any():
        e = (qb[asc] - qt[asc]) / qt[asc] * 100
        s["qhat_err_pct"] = {"window": "t>0.1 s to apogee, q_true>2 kPa", "n": int(asc.sum()),
                             "mean": round(float(e.mean()), 2), "rms": round(float(np.sqrt((e ** 2).mean())), 2),
                             "max_abs": round(float(np.abs(e).max()), 2)}
    h, ht = np.array(a["h"], float), np.array(a["h_true"], float)
    ia = int(np.argmin(np.abs(t - (veh.t_apogee or 0))))
    s["apogee"] = {"true_m": round(float(ht.max()), 1), "board_at_true_apogee_m": round(float(h[ia]), 1),
                   "openrocket_m": round(motor.ref["apogee"], 1)}
    pt = np.array(a["p_true_dps"], float)
    fl = (t > 0.0) & (t < (veh.t_apogee or 0))
    if fl.any():
        fins = np.array(a["fin0"], float)[fl] * 30.0
        s["roll"] = {"p_rms_dps": round(float(np.sqrt(np.mean(pt[fl] ** 2))), 2),
                     "p_max_abs_dps": round(float(np.abs(pt[fl]).max()), 1),
                     "p_final_coast_mean_dps": round(float(pt[fl][-100:].mean()), 2),
                     "fin_cmd_sat_frac": round(float(np.mean(np.abs(fins) >= 11.99)), 3),
                     "fin_cmd_max_deg": round(float(np.abs(fins).max()), 2),
                     "roll_revs": round(float(np.sum(pt[fl]) * FRAME_DT / 360.0), 2)}
    sat = np.array(a["nav_flags"]) & NAV_F_SAT
    s["nav_saturated_frames"] = int((sat > 0).sum())
    gps = np.array(a["nav_flags"]) & NAV_F_GPS_AIDED
    s["gps_aided_frames"] = int((gps > 0).sum())
    desc = (t > (veh.t_apogee or 1e9) + 5) & (np.array(a["phase"]) == "DESCENT")
    if desc.any():
        s["descent_h_err_m"] = {"rms": round(float(np.sqrt(((h[desc] - ht[desc]) ** 2).mean())), 1),
                                "max_abs": round(float(np.abs(h[desc] - ht[desc]).max()), 1)}
    return s


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("mode", choices=("static", "flight"))
    ap.add_argument("--port")
    ap.add_argument("--frames", type=int, default=100000, help="static: frames to run")
    ap.add_argument("--motor", choices=tuple(MOTORS), default="I205W")
    ap.add_argument("--closed", action="store_true", help="feed the board's fin commands to the plant")
    ap.add_argument("--elev", type=float, default=110.0, help="site elevation m MSL (MRC estimate)")
    ap.add_argument("--wind", type=float, default=2.2)
    ap.add_argument("--cl0", type=float, default=0.0, help="roll forcing coefficient")
    ap.add_argument("--acc-sf", type=float, default=0.0, help="accel scale-factor error (fraction)")
    ap.add_argument("--seed", type=int, default=1)
    ap.add_argument("--no-gps", dest="gps", action="store_false")
    ap.add_argument("--pad-min", type=float, default=10.0, help="s on the pad before ignition (min)")
    ap.add_argument("--pad-max", type=float, default=60.0)
    ap.add_argument("--post-land", type=float, default=15.0)
    ap.add_argument("--params", help="sim_params.json (default Tools/sim_params.json)")
    for stream in (sys.stdout, sys.stderr):              # cp1252 consoles vs U+FFFD from the link
        try:
            stream.reconfigure(errors="replace")
        except (AttributeError, ValueError):
            pass
    fc.require_serial()
    run(ap.parse_args())


if __name__ == "__main__":
    main()
