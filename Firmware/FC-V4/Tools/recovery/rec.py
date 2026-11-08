"""REC-01 recovery sub-simulation: motor ejection, single chute.

Planar 4-body model, vectorised across Monte Carlo runs:
  F  forward section (nose, avionics bay, canard module, coupler + bulkhead)
  A  aft section (body tube, fin can, engine block, spent motor)
  K  cord knot / quick link where the chute is tied to the shock cord
  C  canopy + shroud lines (point mass with a growing drag area)
Links (tension only, spring-damper): F-K (a L), K-A ((1-a) L), K-C (shroud).

Phase 1, ejection. The motor's black-powder charge pressurises the free volume
between the engine block and the coupler bulkhead. Gauge pressure from the
ideal-gas hobby constant (R T = 22.16 ft lbf/lbm/R x 3307 R, i.e. 219 J of P V
per gram) times an efficiency eta, with finite burn and leakage. The friction
fit breaks at F_hold; the coupler strokes out over L_eng, then the gas vents.
Phase 2, free flight. Drag on each body, gravity, cord and shroud stretch,
canopy inflation (Knacke-type fill: fill distance n D0, CdS ~ t^2) from shroud
line stretch.

Outputs per run: peak ejection pressure, bulkhead / motor-retention force,
separation speed, peak tension per link (snatch vs opening), peak acceleration
of each section, recontact, cord angle to the aft body axis at peak tension.
Every input range is listed in REC-01 §2 (assumption register).

    python rec.py --n 2000 --cord kevlar --dep ../sitl/runs/L_active/runs.csv --out runs/kevlar
"""
import argparse, csv, json, os
import numpy as np

G0 = 9.80665
RT_BP = 22.16 * 1.3558179 / 0.45359237 * 3307          # J/kg: R (ft-lbf/lbm/R -> J/kg/R) x T (R) = 219 kJ/kg
RT_BP_G = RT_BP / 1000.0                              # J per gram (= 219 J/g)
D_ID = 0.060; A_T = np.pi * D_ID ** 2 / 4             # carbon body tube bore
CORDS = {   # axial stiffness EA (N) ranges per cord class: REC-01 A-R9
    "kevlar":  (6e3, 2.5e4),   # braided aramid, 3-5 % strain at break
    "nylon":   (1.5e3, 5e3),   # flat / tubular nylon, ~15-25 % at break
    "elastic": (40.0, 150.0),  # braided elastic (bungee), ~100 % at break
}


def sample(rng, n, a, dep):
    U, N = rng.uniform, rng.normal
    s = {}
    i = rng.integers(0, len(dep["dep_V"]), n) if dep is not None else None
    s["V0"] = dep["dep_V"][i] if a.v_dep is None else U(a.v_dep[0], a.v_dep[1], n)
    s["gam0"] = np.radians(dep["dep_gam"][i]) if a.v_dep is None else np.radians(U(-60, 10, n))
    s["aoa0"] = np.radians(np.minimum(dep["dep_aoa"][i] if a.v_dep is None else U(0, 30, n), 60)) * rng.choice([-1, 1], n)
    s["h0"] = dep["dep_h"][i] if a.v_dep is None else U(300, 450, n)
    s["mF"] = 0.435 * N(1, 0.05, n); s["mA"] = 0.373 * N(1, 0.05, n)
    s["mK"] = U(0.003, 0.010, n); s["mC"] = U(0.020, 0.035, n)       # quick link; 0.6 m ripstop + lines
    s["m_bp"] = U(a.bp[0], a.bp[1], n)                                # g, motor ejection charge
    s["eta"] = U(0.4, 0.9, n)                                         # combustion/heat-loss efficiency
    s["tau_b"] = U(0.002, 0.010, n); s["tau_leak"] = U(0.02, 0.10, n); s["tau_v"] = U(0.001, 0.004, n)
    s["V0g"] = U(0.5e-3, 0.9e-3, n)                                   # m^3 free volume (pack 50 % porous)
    s["F_hold"] = U(a.hold[0], a.hold[1], n); s["L_eng"] = np.full(n, 0.06)
    s["cdaF"] = U(0.004, 0.012, n); s["cdaA"] = U(0.006, 0.020, n); s["cdaP"] = np.full(n, 0.002)
    s["CdS0"] = a.chute_cds * N(1, 0.07, n); s["D0"] = np.full(n, a.chute_d)
    s["n_fill"] = U(4, 10, n); s["L"] = np.full(n, a.cord_len); s["a_knot"] = U(0.2, 0.5, n)
    s["L_s"] = np.full(n, a.chute_d) * U(0.9, 1.2, n)                   # shroud length ~ 1 D0
    lo, hi = CORDS[a.cord]; s["EA"] = rng.uniform(lo, hi, n) if a.ea is None else np.full(n, a.ea)
    s["EA_s"] = U(2e3, 6e3, n)                                        # nylon shroud bundle
    s["zeta"] = U(a.zeta[0], a.zeta[1], n); s["ej_frac"] = U(0.0, 0.5, n)         # pack lag behind F at exit
    return s


def link(pa, pb, va, vb, L0, EA, zeta, mu):
    d = pb - pa; dist = np.sqrt((d ** 2).sum(0)) + 1e-9; u = d / dist
    k = EA / L0; c = 2 * zeta * np.sqrt(k * mu)
    rate = ((vb - va) * u).sum(0)
    T = np.where(dist > L0, np.maximum(0.0, k * (dist - L0) + c * rate), 0.0)
    return T, u, dist


def simulate(s, t_end=4.0, dt=2.5e-5, rho=1.2, dt1=1e-6, trace=None):
    n = len(s["V0"])
    e = np.vstack([np.cos(s["gam0"] + s["aoa0"]), np.sin(s["gam0"] + s["aoa0"])])   # body forward axis
    v = np.vstack([s["V0"] * np.cos(s["gam0"]), s["V0"] * np.sin(s["gam0"])])
    m1 = s["mF"] + s["mK"] + s["mC"]; m2 = s["mA"]; mt = m1 + m2
    # ---- phase 1: ejection (relative coordinate along e) ----
    E = np.zeros(n); sx = np.zeros(n); sv = np.zeros(n); t = np.zeros(n)
    broke = np.zeros(n, bool); out = np.zeros(n, bool); Pmax = np.zeros(n); Pg = np.zeros(n)
    t_sep = np.full(n, np.nan); v_sep = np.zeros(n); Pout = np.zeros(n)
    for _ in range(int(0.08 / dt1)):
        burn = s["m_bp"] / s["tau_b"] * np.exp(-t / s["tau_b"]) * s["eta"] * RT_BP_G
        V = s["V0g"] + A_T * sx
        Pg = np.where(out, Pout * np.exp(-(t - np.nan_to_num(t_sep)) / s["tau_v"]), np.maximum(E / V, 0.0))
        E = E + (burn - E / s["tau_leak"]) * dt1
        Pmax = np.maximum(Pmax, Pg)
        Fp = Pg * A_T
        broke |= Fp > s["F_hold"]
        fr = np.where(out, 0.0, 0.5 * s["F_hold"])
        acc = np.where(broke, (Fp - fr) * (1 / m1 + 1 / m2), 0.0)
        sv = np.where(broke, np.maximum(sv + acc * dt1, 0.0), 0.0); sx = sx + sv * dt1
        newly = (~out) & (sx >= s["L_eng"])
        t_sep = np.where(newly, t, t_sep); v_sep = np.where(newly, sv, v_sep); Pout = np.where(newly, Pg, Pout)
        out |= newly
        t += dt1
    fail_sep = ~out
    v_sep = np.where(out, v_sep, sv)
    # ---- phase 2: four bodies ----
    pF = np.zeros((2, n)); pA = -e * 0.40                       # attach points 0.40 m apart when joined
    vF = v + e * v_sep * m2 / mt; vA = v - e * v_sep * m1 / mt
    pK = pF.copy(); pC = pF.copy(); vK = vF - e * v_sep * s["ej_frac"]; vC = vK.copy()
    infl = np.zeros(n, bool); t_ls = np.full(n, np.nan); t2 = 0.0
    keys = ("FK", "KA", "KC")
    Tm = {k: np.zeros(n) for k in keys}; Ts = {k: np.zeros(n) for k in keys}      # peak, peak before inflation
    gF = np.zeros(n); gA = np.zeros(n); ang = np.zeros(n); dmin = np.full(n, 9.0); T_tot_max = np.zeros(n)
    v_hit = np.zeros(n); t_hit = np.full(n, np.nan); GAP0 = 0.40     # attach-point spacing when joined
    mu_FK = s["mF"] * s["mK"] / (s["mF"] + s["mK"]); mu_KA = s["mA"] * s["mK"] / (s["mA"] + s["mK"])
    mu_KC = s["mC"] * s["mK"] / (s["mC"] + s["mK"])
    g = np.array([[0.0], [-G0]])
    for _ in range(int(t_end / dt)):
        T1, u1, d1 = link(pF, pK, vF, vK, s["a_knot"] * s["L"], s["EA"], s["zeta"], mu_FK)
        T2, u2, d2 = link(pK, pA, vK, vA, (1 - s["a_knot"]) * s["L"], s["EA"], s["zeta"], mu_KA)
        T3, u3, d3 = link(pK, pC, vK, vC, s["L_s"], s["EA_s"], s["zeta"], mu_KC)
        newly = (~infl) & (d3 >= 0.98 * s["L_s"]) & (~fail_sep)
        t_ls = np.where(newly, t2, t_ls); infl |= newly
        VC = np.sqrt((vC ** 2).sum(0)) + 1e-6
        frac = np.where(infl, np.clip((t2 - np.nan_to_num(t_ls)) * VC / (s["n_fill"] * s["D0"]), 0, 1) ** 2, 0.0)
        cdsC = s["cdaP"] + (s["CdS0"] - s["cdaP"]) * frac
        def drag(vv, cda):
            sp = np.sqrt((vv ** 2).sum(0)) + 1e-9
            return -0.5 * rho * cda * sp * vv
        fF = drag(vF, s["cdaF"]) + T1 * u1
        fA = drag(vA, s["cdaA"]) - T2 * u2
        fK = -T1 * u1 + T2 * u2 + T3 * u3
        fC = drag(vC, cdsC) - T3 * u3
        aF = fF / s["mF"]; aA = fA / s["mA"]
        gF = np.maximum(gF, np.sqrt((aF ** 2).sum(0)) / G0); gA = np.maximum(gA, np.sqrt((aA ** 2).sum(0)) / G0)
        vF = vF + (aF + g) * dt; vA = vA + (aA + g) * dt
        vK = vK + (fK / s["mK"] + g) * dt; vC = vC + (fC / s["mC"] + g) * dt
        pF = pF + vF * dt; pA = pA + vA * dt; pK = pK + vK * dt; pC = pC + vC * dt
        for k, T in zip(keys, (T1, T2, T3)):
            Tm[k] = np.maximum(Tm[k], T); Ts[k] = np.where(infl, Ts[k], np.maximum(Ts[k], T))
        hi = (T2 > T_tot_max) & (~infl)                  # snatch only: body rotation is not modelled
        # angle between the cord at the aft attach and the aft body axis (zipper risk)
        ca = np.clip(-(u2 * e).sum(0), -1, 1)
        ang = np.where(hi, np.degrees(np.arccos(ca)), ang); T_tot_max = np.where(infl, T_tot_max, np.maximum(T_tot_max, T2))
        if t2 > 0.05:
            dFA = pF - pA; dd = np.sqrt((dFA ** 2).sum(0))
            dmin = np.minimum(dmin, dd)
            # section ends meet again (attach points closer than when joined): closing speed
            hit = (dd < GAP0) & np.isnan(t_hit)
            vclose = -((vF - vA) * dFA).sum(0) / (dd + 1e-9)
            v_hit = np.where(hit, vclose, v_hit); t_hit = np.where(hit, t2, t_hit)
            if trace is not None and int(t2 / dt) % 200 == 0:
                trace.append((t2, dd[0], T1[0], T2[0], T3[0], frac[0], *(vF[:, 0]), *(vA[:, 0])))
        t2 += dt
    vfin = np.sqrt((vC ** 2).sum(0))
    return dict(P_max_kPa=Pmax / 1e3, F_bulk_N=Pmax * A_T, v_sep=v_sep, fail_sep=fail_sep.astype(float),
                t_sep_ms=t_sep * 1e3, T_FK=Tm["FK"], T_KA=Tm["KA"], T_KC=Tm["KC"],
                Tsn_FK=Ts["FK"], Tsn_KA=Ts["KA"], T_max=np.maximum(Tm["FK"], Tm["KA"]),
                g_F=gF, g_A=gA, zip_ang=ang, recontact_min_m=dmin, v_end=vfin,
                recontact=(~np.isnan(t_hit)).astype(float), v_hit=v_hit, t_hit=t_hit,
                t_ls=t_ls, inflated=infl.astype(float))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--n", type=int, default=1000)
    ap.add_argument("--cord", default="kevlar", choices=list(CORDS))
    ap.add_argument("--ea", type=float, default=None, help="fixed EA (N) for a sweep")
    ap.add_argument("--cord_len", type=float, default=4.0)
    ap.add_argument("--chute_d", type=float, default=0.6)
    ap.add_argument("--chute_cds", type=float, default=0.7 * np.pi * 0.09)
    ap.add_argument("--bp", type=float, nargs=2, default=[0.3, 1.0], help="ejection charge range, g")
    ap.add_argument("--hold", type=float, nargs=2, default=[10.0, 50.0], help="friction-fit break force, N")
    ap.add_argument("--v_dep", type=float, nargs=2, default=None, help="override deploy speed range (stress case)")
    ap.add_argument("--dep", default="../sitl/runs/L_active/runs.csv")
    ap.add_argument("--zeta", type=float, nargs=2, default=[0.05, 0.15], help="cord damping ratio range")
    ap.add_argument("--seed", type=int, default=1)
    ap.add_argument("--out", required=True)
    a = ap.parse_args()
    rows = list(csv.DictReader(open(a.dep)))
    dep = {k: np.array([float(r[k]) for r in rows if float(r.get("dep_V", 0)) > 0]) for k in ("dep_V", "dep_gam", "dep_aoa", "dep_h")}
    rng = np.random.default_rng(a.seed)
    s = sample(rng, a.n, a, dep)
    r = simulate(s)
    os.makedirs(a.out, exist_ok=True)
    with open(os.path.join(a.out, "runs.csv"), "w", newline="") as f:
        w = csv.writer(f); ks = list(s) + list(r); w.writerow(ks)
        for i in range(a.n):
            w.writerow([float(s[k][i]) for k in s] + [float(r[k][i]) for k in r])
    summ = {"n": a.n, "args": vars(a), "pct": {}}
    for k, v in r.items():
        v = v[np.isfinite(v)]
        if len(v):
            summ["pct"][k] = {q: round(float(np.percentile(v, p)), 3) for q, p in (("p50", 50), ("p95", 95), ("p99", 99))}
            summ["pct"][k]["max"] = round(float(v.max()), 3)
    json.dump(summ, open(os.path.join(a.out, "summary.json"), "w"), indent=1)
    for k in ("P_max_kPa", "F_bulk_N", "v_sep", "fail_sep", "T_max", "Tsn_KA", "T_KC", "g_F", "g_A", "zip_ang", "recontact_min_m", "recontact", "v_hit"):
        print(f"{k:16s}", summ["pct"].get(k))


if __name__ == "__main__":
    main()
