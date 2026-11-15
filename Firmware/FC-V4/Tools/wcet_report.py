#!/usr/bin/env python3
"""WCET report from the board's own cycle counters (app.c 'tasks', prof.h; WCET-01).

    python wcet_report.py logs/hitl/campaign-<time> --pad logs/console/<file>.txt --out <dir>

Reads each HITL case's 'tasks' reply (fc_hitl.py stores it as tasks_text) and the last
'tasks' reply in a console log taken on the pad with the real IMU. Writes wcet_summary.json,
wcet.md (tables) and fig02_wcet.png (cycles against the frame and the 80 % budget).
"""
import argparse, json, os, re

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

MHZ = 96
FRAME = 5000 * MHZ          # 480 000 cycles
BUDGET = 4000 * MHZ         # 384 000 cycles (80 %, software guideline B1)
TASK_RE = re.compile(r"^\s*(sense|flight|log|telem|comms|leds)\s+\d+\s+\d+\s+(\d+)\s+(\d+)\s*$")
STEP_RE = re.compile(r"step:(\w+)\s+last_cyc\s+\d+\s+wcet_cyc\s+(\d+)")
FRAME_RE = re.compile(r"frame:(\w+)\s+wcet_cyc\s+(\d+)")


def parse(lines):
    out = {"task": {}, "step": {}, "frame": {}}
    for l in lines:
        l = re.sub(r"^\d\d:\d\d:\d\d(\.\d+)?\s+", "", l)      # console timestamps
        m = TASK_RE.match(l)
        if m:
            out["task"][m.group(1)] = int(m.group(3)); continue
        m = STEP_RE.search(l)
        if m:
            out["step"][m.group(1)] = int(m.group(2)); continue
        m = FRAME_RE.search(l)
        if m:
            out["frame"][m.group(1)] = int(m.group(2))
    return out


def last_tasks_block(path):
    lines = open(path, encoding="utf-8", errors="replace").read().splitlines()
    idx = [i for i, l in enumerate(lines) if l.rstrip().endswith("tasks")]
    return lines[idx[-1]:idx[-1] + 20] if idx else []


def merge_max(dst, src):
    for k in ("task", "step", "frame"):
        for n, v in src[k].items():
            if v > dst[k].get(n, (0, ""))[0]:
                dst[k][n] = (v, src.get("case", ""))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("campaign"); ap.add_argument("--pad", required=True); ap.add_argument("--out", required=True)
    a = ap.parse_args()
    res = json.load(open(os.path.join(a.campaign, "campaign.json")))
    hitl = {"task": {}, "step": {}, "frame": {}}
    per_case = []
    for r in res:
        p = parse(r.get("tasks_text", [])); p["case"] = r.get("name", "?")
        if p["task"]:
            per_case.append(p); merge_max(hitl, p)
    pad = parse(last_tasks_block(a.pad)); pad["case"] = "pad (real IMU)"
    worst_flight_frame = max(v for ph, (v, _) in hitl["frame"].items() if ph in ("BOOST", "COAST"))
    # The HITL seam replaces the I2C read in flight phases. Its own cost (a copy, a few hundred
    # cycles) is not separated out, so it is left in and the real read is added on top: conservative.
    # (hitl task 'sense' WCET is the real read in CAL/SETTLE before the seam goes live: not subtracted.)
    sense_real = max(pad["task"]["sense"], hitl["task"]["sense"][0])
    bound_obs = worst_flight_frame + sense_real
    worst = {t: max(pad["task"].get(t, 0), hitl["task"].get(t, (0, ""))[0]) for t in ("sense", "flight", "log", "telem", "comms", "leds")}
    bound_sum = sum(worst.values())
    # Not measured: the log writer at the data-sheet maximum word-programming time (24 words x 100 us,
    # STM32F4 family figure after 100 k erase cycles), telemetry idle (no USB in flight). WCET-01 4.2.
    corner = worst["sense"] + worst["flight"] + 24 * 100 * MHZ + worst["comms"] + worst["leds"]
    summ = {"mhz": MHZ, "frame_cyc": FRAME, "budget_cyc": BUDGET, "cases": len(per_case),
            "hitl_max": {k: {n: {"cyc": v, "case": c} for n, (v, c) in d.items()} for k, d in hitl.items()},
            "pad": {k: v for k, v in pad.items() if k != "case"},
            "flight_frame_bound_observed_cyc": bound_obs, "flight_frame_bound_sum_cyc": bound_sum,
            "flash_max_corner_cyc": corner}
    os.makedirs(a.out, exist_ok=True)
    json.dump(summ, open(os.path.join(a.out, "wcet_summary.json"), "w"), indent=2)

    with open(os.path.join(a.out, "wcet.md"), "w") as f:
        f.write("| Task | Pad, real IMU (cyc) | HITL max over cases (cyc) | case | µs |\n|---|---|---|---|---|\n")
        for t in ("sense", "flight", "log", "telem", "comms", "leds"):
            hv, hc = hitl["task"].get(t, (0, ""))
            f.write(f"| {t} | {pad['task'].get(t, 0):,} | {hv:,} | {hc} | {max(hv, pad['task'].get(t, 0)) / MHZ:.0f} |\n")
        f.write("\n| Flight_Step section | Pad (cyc) | HITL max (cyc) | case |\n|---|---|---|---|\n")
        for s in ("estimate", "monitor", "control", "alloc"):
            hv, hc = hitl["step"].get(s, (0, ""))
            f.write(f"| {s} | {pad['step'].get(s, 0):,} | {hv:,} | {hc} |\n")
        f.write("\n| Frame, by phase | WCET (cyc) | µs | % of frame | case |\n|---|---|---|---|---|\n")
        for ph, v in sorted(pad["frame"].items()):
            f.write(f"| {ph} (pad, real IMU) | {v:,} | {v / MHZ:.0f} | {100 * v / FRAME:.1f} | pad |\n")
        for ph, (v, c) in sorted(hitl["frame"].items()):
            f.write(f"| {ph} (HITL) | {v:,} | {v / MHZ:.0f} | {100 * v / FRAME:.1f} | {c} |\n")
        f.write(f"\nFlight-frame bound, HITL worst BOOST/COAST frame plus the worst real I2C read: {bound_obs:,} cyc "
                f"({bound_obs / MHZ:.0f} µs, {100 * bound_obs / FRAME:.1f} % of the frame).\n"
                f"Sum of every task's WCET, worse of pad and HITL (all coinciding in one frame): {bound_sum:,} cyc ({bound_sum / MHZ:.0f} µs, "
                f"{100 * bound_sum / FRAME:.1f} %). Budget (guideline B1, 80 %): {BUDGET:,} cyc.\n"
                f"Corner, not measured (log writer at the data-sheet 100 µs/word, telemetry idle): {corner:,} cyc "
                f"({corner / MHZ:.0f} µs, {100 * corner / FRAME:.1f} %).\n")

    # figure
    tasks = ["sense", "flight", "log", "telem", "comms", "leds"]
    cols = {"sense": "#0072B2", "flight": "#D55E00", "log": "#009E73", "telem": "#E69F00", "comms": "#CC79A7",
            "leds": "#56B4E9", "frame": "#8C8C8C"}
    ink = {"sense": "white", "flight": "white"}                      # dark text on the light fills
    names = {"frame": "whole frame, measured"}
    flash_max = 24 * 100 * MHZ          # 2 records x 48 B = 24 words, 100 us/word data-sheet max (WCET-01 4.2)
    rows = [("Pad, READY, real IMU:\nwhole frame", [("frame", pad["frame"].get("READY", 0))]),
            ("Flight, BOOST/COAST: HITL frame\n+ worst real I2C read", [("frame", worst_flight_frame), ("sense", sense_real)]),
            (f"Every task's worst case at once\n(pad or HITL, {len(per_case)} cases)", [(t, worst[t]) for t in tasks]),
            ("Corner, not measured: flash at\ndata-sheet max, no USB (telem idle)",
             [("sense", worst["sense"]), ("flight", worst["flight"]), ("log", flash_max), ("comms", worst["comms"]), ("leds", worst["leds"])])]
    fig, (ax, bx) = plt.subplots(2, 1, figsize=(11, 7.4), gridspec_kw=dict(height_ratios=[1.35, 1.0]))
    seen = set(); labels = []
    for i, (lab, segs) in enumerate(rows):
        x0 = 0
        for k, v in segs:
            ax.barh(i, v, left=x0, color=cols[k], height=0.56, edgecolor="white", linewidth=1.0,
                    label=None if k in seen else names.get(k, k))
            seen.add(k)
            if v > 22000:
                ax.text(x0 + v / 2, i, f"{names.get(k, k).split(',')[0] if k == 'frame' else k}\n{v / 1000:.0f}k",
                        ha="center", va="center", fontsize=7.5, color=ink.get(k, "#1a1a1a"))
            x0 += v
        labels.append(f"{lab}\nΣ {x0:,} cyc = {x0 / MHZ:,.0f} µs ({100 * x0 / FRAME:.0f} %)")
    for xv, txt, ls in ((BUDGET, "budget 80 % (B1)", "--"), (FRAME, "frame 5 ms", "-")):
        ax.axvline(xv, color="#222", ls=ls, lw=1)
        ax.text(xv, 1.01, txt, transform=ax.get_xaxis_transform(), fontsize=8, ha="center", va="bottom")
    ax.text(pad["frame"].get("READY", 0) + 25000, 0, "Not shown, off scale: an I2C bus held busy\nmakes the HAL wait 25 ms per read\n(2.4 M cycles; WCET-01 §4)",
            ha="left", va="center", fontsize=7.5, color="#333")
    ax.set_yticks(range(len(rows)), labels, fontsize=8); ax.invert_yaxis(); ax.set_xlim(0, FRAME * 1.04)
    ax.set_xlabel("core cycles at 96 MHz")
    ax.legend(loc="upper center", bbox_to_anchor=(0.5, -0.17), fontsize=7.5, ncol=7, frameon=False)
    ax.set_title("Worst-case execution time per 5 ms frame (DWT cycle counter, -O2, interrupts included)",
                 fontsize=10.5, loc="left", pad=16)
    secs = ["estimate", "monitor", "control", "alloc"]
    xs = range(len(secs)); w = 0.38
    bx.bar([x - w / 2 for x in xs], [pad["step"].get(s, 0) for s in secs], w, color="#8C8C8C", label="pad, READY, real IMU")
    bx.bar([x + w / 2 for x in xs], [hitl["step"][s][0] for s in secs], w, color="#D55E00", label=f"HITL max ({len(per_case)} cases, all phases)")
    for x, s in zip(xs, secs):
        v = hitl["step"][s][0]
        bx.text(x + w / 2, v + 600, f"{v:,}\n({v / MHZ:.0f} µs)", ha="center", va="bottom", fontsize=7.5)
    bx.set_ylim(0, max(hitl["step"][s][0] for s in secs) * 1.25)
    bx.set_xticks(list(xs), ["estimate\n(IEKF + NAV)", "monitor\n(FDIR, detectors, phase)", "control\n(q schedule, roll test, law + observer)", "alloc\n(allocation, quantisation)"], fontsize=8)
    bx.set_ylabel("WCET (cycles)"); bx.legend(fontsize=8, frameon=False, loc="upper right")
    bx.set_title("Inside the flight task: Flight_Step sections", fontsize=9.5, loc="left")
    for a_ in (ax, bx):
        [a_.spines[s].set_visible(False) for s in ("top", "right")]
    fig.tight_layout()
    fig.savefig(os.path.join(a.out, "fig02_wcet.png"), dpi=160, bbox_inches="tight")
    print(open(os.path.join(a.out, "wcet.md")).read())


if __name__ == "__main__":
    main()
