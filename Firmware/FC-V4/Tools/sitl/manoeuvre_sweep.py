"""Worst-case manoeuvre study: canards forced to a fixed deflection for the whole flight (no roll).
Compares the trajectory envelope against the same conditions with the canards neutral."""
import itertools, json, os, re, subprocess, sys
from concurrent.futures import ThreadPoolExecutor
HERE = os.path.dirname(os.path.abspath(__file__))
KEYS = ("apogee", "aoa_max", "nlat_max", "tilt_max", "gam_min_pow", "gam_min", "x_apo", "x_land", "q_max")


def run(args):
    o = subprocess.run([os.path.join(HERE, "sitl")] + args, capture_output=True, text=True, cwd=HERE).stdout
    return {k: float(re.search(rf"\b{k}=([-\d.]+)", o).group(1)) for k in KEYS}


motors = ["G40W", "G53FJ", "G79W", "G80T", "G77R", "G104T"]
cases = []
for m, w, wd, rt in itertools.product(motors, (0, 6, 12), (0, 90, 180, 270), (0, 5)):
    base = [f"motor=motors/{m}.csv", f"wind={w}", f"wind_dir={wd}", f"rail_tilt={rt}", f"rail_az={wd}", "turb=0",
            "Cld_scale=1.3", "CNa_scale=0.9", "cg_shift=0.02", "cp_shift=-0.02"]
    for mode, fm, deg, trav in (("none", 0, 0, 0.2094), ("diag12", 2, 12, 0.2094), ("diag15", 2, 15, 0.2618), ("pump15", 4, 15, 0.2618)):
        a = base + [f"sv_travel={trav}"] + ([f"force_mode={fm}", f"force_deg={deg}"] if deg else [])
        cases.append((m, w, wd, rt, mode, a))
with ThreadPoolExecutor(max_workers=os.cpu_count()) as ex:
    res = list(ex.map(lambda c: run(c[5]), cases))
rows = [dict(motor=c[0], wind=c[1], wind_dir=c[2], rail_tilt=c[3], mode=c[4], **r) for c, r in zip(cases, res)]
json.dump(rows, open(os.path.join(HERE, "runs", "manoeuvre_sweep.json"), "w"), indent=0)
for mode in ("none", "diag12", "diag15", "pump15"):
    rr = [r for r in rows if r["mode"] == mode]
    w = min(rr, key=lambda r: r["gam_min_pow"])
    print(f"{mode:7s} n={len(rr)} min gam under thrust {w['gam_min_pow']:.1f} ({w['motor']} wind {w['wind']} dir {w['wind_dir']} rail {w['rail_tilt']}); "
          f"max aoa {max(r['aoa_max'] for r in rr):.1f}; max nlat {max(r['nlat_max'] for r in rr):.2f} g; "
          f"apogee min {min(r['apogee'] for r in rr):.0f}; x_apo max {max(r['x_apo'] for r in rr):.0f}; x_land max {max(r['x_land'] for r in rr):.0f}")
# paired deltas: forced minus neutral, same conditions
import collections
pair = collections.defaultdict(dict)
for r in rows:
    pair[(r["motor"], r["wind"], r["wind_dir"], r["rail_tilt"])][r["mode"]] = r
for mode in ("diag12", "diag15", "pump15"):
    dg = [p[mode]["gam_min_pow"] - p["none"]["gam_min_pow"] for p in pair.values()]
    dx = [p[mode]["x_apo"] - p["none"]["x_apo"] for p in pair.values()]
    dh = [p[mode]["apogee"] - p["none"]["apogee"] for p in pair.values()]
    print(f"{mode}: change in min powered flight-path angle {min(dg):.1f}..{max(dg):.1f} deg; extra horizontal at apogee {min(dx):.0f}..{max(dx):.0f} m; apogee change {min(dh):.0f}..{max(dh):.0f} m")
