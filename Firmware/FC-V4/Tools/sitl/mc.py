"""Monte Carlo driver for the SITL 6DOF harness (ADR-004 §6, MC-01).

Each run samples the dispersion table below (the assumption register in
MC-01 lists the source of every bound), runs ./sitl (or a gain-variant
build), and parses its one-line metrics. Results: <out>/runs.csv and
<out>/summary.json with pass rates against the MC-01 requirement set.

    python mc.py --n 200 --out runs/nominal                 # all motors, active
    python mc.py --n 200 --shadow --out runs/shadow         # flight 1 (canards held)
    python mc.py --n 60 --variant "-DCTRL_ROLL_C=15.0f" --out runs/c15
    python mc.py --n 40 --fault stuck --out runs/stuck      # servo fault campaign
"""
import argparse, csv, json, os, subprocess, sys, hashlib
from concurrent.futures import ProcessPoolExecutor
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
MOTORS = ["G40W", "G53FJ", "G79W", "G80T", "G77R", "G104T"]

# name: (sampler, description) -- all bounds documented in MC-01 §2
def sample(rng, motor, a):
    U, N = rng.uniform, rng.normal
    d = dict(
        motor=f"motors/{motor}.csv", aero="aero.csv",
        thrust_scale=N(1, 0.03), mass_scale=N(1, 0.03), cg_shift=N(0, 0.01),
        ixx_scale=U(0.68, 1.6), it_scale=N(1, 0.10), I_can=U(1.5e-4, 3.0e-4),
        Cd_scale=N(1, 0.10), CNa_scale=N(1, 0.10), cp_shift=N(0, 0.01), Cld_scale=U(0.7, 1.3), Clp_scale=N(1, 0.15),
        cl0_body=N(0, 0.005), can_cant_deg=N(0, a.cant_sd), fin1_mis_deg=N(0, a.fin1_sd),
        thrust_mis_deg=abs(N(0, 0.25)), thrust_az_deg=U(0, 360), cg_off_y=N(0, 0.001),
        bear_coul=U(0.003, a.bear_max),
        wind=U(0, a.wind_max), wind_dir=U(0, 360), turb=U(0.1, 0.25),
        rail_tilt=U(0, 5), rail_az=U(0, 360),
        vib_boost_g=U(0.5, 3.0), vib_res_g=U(0.3, 2.0), vib_res_hz=U(150, 900), vib_gyro_dps=U(1, 6),
        sv_tau=U(0.010, 0.021), sv_wn=U(52, 100), sv_zeta=U(0.5, 0.9), sv_db=U(0.001, 0.004),
        sv_lash=U(0.001, 0.003), sv_volt_k=U(0.85, 1.0),
        chute_delay=U(0.5, 3.0), canopy_swing=U(3, 12), collapse_t=U(2, 30), gps_on=int(rng.uniform() < 0.9),
    )
    if rng.uniform() < 0.3:     # discrete crosswind gust in 30 % of flights
        d.update(gust_t=U(0.3, 6.0), gust_amp=U(2, 8), gust_dur=U(0.5, 2.0))
    if a.n_gear != 1:
        n = a.n_gear
        d.update(sv_db=d["sv_db"] / n, sv_lash=d["sv_lash"] / n, sv_res=5.1e-4 / n, sv_rate=23.65 * d["sv_volt_k"] / n, sv_volt_k=1.0)
    if a.fault:
        d.update(fault_kind=a.fault, fault_fin=int(rng.integers(0, 4)), fault_t=U(0.0, 6.0),
                 fault_val=float(rng.choice([-1.0, 1.0]) * (U(0.5, 3.0) if a.fault == "offset" else 1.0)))
    if a.shadow:
        d["shadow"] = 1
    for kv in a.set:
        k, v = kv.split("=", 1); d[k] = float(v) if v.replace(".", "", 1).replace("-", "", 1).isdigit() else v
    return d


def build(variant):
    if not variant:
        exe = os.path.join(HERE, "sitl")
        subprocess.run(["make", "-s", "-C", HERE], check=True)
        return exe
    tag = hashlib.md5(variant.encode()).hexdigest()[:8]
    exe = os.path.join(HERE, f"sitl_{tag}")
    core = os.path.join(HERE, "..", "..", "Core")
    src = ["sitl.c"] + [os.path.join(core, "hyades", p) for p in
           ("mission/flight.c", "mission/phase.c", "mission/fdir.c", "mission/detect.c",
            "gnc/control.c", "gnc/iekf.c", "gnc/nav.c")]
    inc = [f"-I{os.path.join(core, 'hyades', s)}" for s in ("math", "gnc", "mission", "svc")] + [f"-I{os.path.join(core, 'Inc')}"]
    subprocess.run(["gcc", "-std=c11", "-O2", "-D_DEFAULT_SOURCE", *variant.split(), *inc, "-o", exe, *src, "-lm"],
                   cwd=HERE, check=True)
    return exe


def run_one(job):
    exe, seed, params = job
    args = [exe, f"seed={seed}"] + [f"{k}={v}" for k, v in params.items()]
    r = subprocess.run(args, cwd=HERE, capture_output=True, text=True, timeout=120)
    out = {}
    for tok in r.stdout.split():
        k, v = tok.split("=", 1)
        try: out[k] = float(v)
        except ValueError: out[k] = v
    out.update({f"in_{k}": v for k, v in params.items()})
    out["seed"] = seed
    return out


# MC-01 §3 requirement set (proposed; to be confirmed by James)
def judge(r, shadow):
    lift = r["det_liftoff"]; apo = r["det_apogee"] - r["t_apogee"]
    land = r["det_landed"] - r["t_ground"] if r["t_ground"] > 0 else 99
    land_rest = r["det_landed"] - r["t_rest"] if r["t_rest"] > 0 else 99
    gps = r.get("in_gps_on", 1) == 1
    checks = {
        "R1_sequence": r["ok_seq"] == 1 and r["phase_bad"] == 0,
        "R2_liftoff_le_0.35s": 0 <= lift <= 0.35,
        "R3_apogee_-0.5_+1.5s": -0.5 <= apo <= 1.5,
        "R4a_landing_never_early": land >= 0,
        "R4b_landing_latency": (land <= (15 if gps else 40)) or (land_rest <= (5 if gps else 35)),
        "R6_lockin_ratio_le_0.7": r["can_pitch_ratio"] <= 0.7 and r["body_pitch_ratio"] <= 0.7,
        "R9_qhat_rms_le_25pct": r["qhat_rms"] <= 25,
    }
    if not shadow:
        checks.update({
            "R5a_roll_max_le_360dps": max(r["p_max_boost"], r["p_max_coast"]) <= 360,
            "R5b_coast_rms_le_60dps": r["p_rms_coast"] <= 60,
            "R8_sat_le_5pct": r["sat_frac"] <= 0.05,
        })
    return checks


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--n", type=int, default=50, help="runs per motor")
    ap.add_argument("--motors", default=",".join(MOTORS))
    ap.add_argument("--out", required=True)
    ap.add_argument("--variant", default="")
    ap.add_argument("--shadow", action="store_true")
    ap.add_argument("--fault", default="")
    ap.add_argument("--wind_max", type=float, default=8.0)
    ap.add_argument("--n_gear", type=float, default=1.0)
    ap.add_argument("--cant_sd", type=float, default=0.25, help="rear-fin net cant 1-sigma, deg")
    ap.add_argument("--fin1_sd", type=float, default=0.2, help="single-fin misalignment 1-sigma, deg")
    ap.add_argument("--bear_max", type=float, default=0.012, help="bearing Coulomb torque upper bound, N m")
    ap.add_argument("--seed0", type=int, default=1000)
    ap.add_argument("--set", action="append", default=[], help="key=value fixed override")
    ap.add_argument("--jobs", type=int, default=os.cpu_count() or 2)
    a = ap.parse_args()
    exe = build(a.variant)
    jobs = []
    for mi, m in enumerate(a.motors.split(",")):
        for i in range(a.n):
            seed = a.seed0 + mi * 100000 + i
            rng = np.random.default_rng(seed)
            jobs.append((exe, seed, sample(rng, m, a)))
    with ProcessPoolExecutor(a.jobs) as ex:
        rows = list(ex.map(run_one, jobs, chunksize=4))
    os.makedirs(a.out, exist_ok=True)
    keys = sorted({k for r in rows for k in r})
    with open(os.path.join(a.out, "runs.csv"), "w", newline="") as f:
        w = csv.DictWriter(f, keys); w.writeheader(); w.writerows(rows)
    summ = {"n": len(rows), "variant": a.variant, "cant_sd": a.cant_sd, "fin1_sd": a.fin1_sd, "bear_max": a.bear_max, "shadow": a.shadow, "fault": a.fault, "n_gear": a.n_gear,
            "wind_max": a.wind_max, "pass": {}, "pct": {}, "by_motor": {}}
    allc = [judge(r, a.shadow) for r in rows]
    for k in allc[0]:
        summ["pass"][k] = round(100.0 * np.mean([c[k] for c in allc]), 1)
    summ["all_pass_pct"] = round(100.0 * np.mean([all(c.values()) for c in allc]), 1)
    for k in ("p_max_boost", "p_max_coast", "p_rms_coast", "aoa_max", "qalpha_max", "nlat_max", "defl_max",
              "sat_frac", "qhat_rms", "pcan_max", "can_pitch_ratio", "body_pitch_ratio", "impact_g", "apogee", "q_max", "mach_max", "wlat_max"):
        v = np.array([r[k] for r in rows], float)
        summ["pct"][k] = {"p50": round(float(np.percentile(v, 50)), 3), "p95": round(float(np.percentile(v, 95)), 3),
                          "p99": round(float(np.percentile(v, 99)), 3), "max": round(float(v.max()), 3)}
    for m in a.motors.split(","):
        rr = [r for r in rows if r["motor"].endswith(f"/{m}.csv")]
        cc = [judge(r, a.shadow) for r in rr]
        summ["by_motor"][m] = {"all_pass_pct": round(100.0 * np.mean([all(c.values()) for c in cc]), 1),
                               "apogee_p50": round(float(np.median([r["apogee"] for r in rr])), 1),
                               "q_max_p95": round(float(np.percentile([r["q_max"] for r in rr], 95)), 0),
                               "p_rms_coast_p95": round(float(np.percentile([r["p_rms_coast"] for r in rr], 95)), 1),
                               "p_max_p95": round(float(np.percentile([max(r["p_max_boost"], r["p_max_coast"]) for r in rr], 95)), 1)}
    fails = {}
    for r, c in zip(rows, allc):
        for k, ok in c.items():
            if not ok:
                fails.setdefault(k, []).append(int(r["seed"]))
    summ["fail_seeds"] = {k: v[:10] for k, v in fails.items()}
    json.dump(summ, open(os.path.join(a.out, "summary.json"), "w"), indent=1)
    print(json.dumps({k: summ[k] for k in ("n", "all_pass_pct", "pass")}, indent=None))
    print(json.dumps(summ["by_motor"]))


if __name__ == "__main__":
    main()
