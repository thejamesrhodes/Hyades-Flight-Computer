"""Build the SITL input tables (ADR-004 §6) from the project's own sources.

aero.csv   Mach, Cd, x_cp (m from the nose), CN_alpha (/rad): OpenRocket
           exports of "62mm Canard Control V1.0.0" (G79W and I205W runs cover
           Mach 0-0.8), binned by Mach. CN_alpha = median(CN/alpha) where
           0.5 < alpha < 10 deg, else the overall median.
motors/*.csv  t (s), thrust (N), motor mass (kg). G79W from the OpenRocket
           export; the others from thrustcurve.org RASP files (openrocket/
           motor-database), mass burned in proportion to impulse delivered.
"""
import glob, os, sys
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
SIM = os.path.join(HERE, "..", "..", "..", "..", "Simulation")


def load_or(path):
    hdr, rows = None, []
    for line in open(path, encoding="utf-8"):
        if line.startswith("# Time"):
            hdr = [h.strip() for h in line[2:].split(",")]
        elif line.startswith("#") or not line.strip():
            continue
        else:
            rows.append([float(x) if x not in ("NaN", "") else np.nan for x in line.split(",")])
    a = np.array(rows)
    return {h.split(" (")[0]: a[:, i] for i, h in enumerate(hdr)}


def aero(out):
    M, Cd, xcp, cn = [], [], [], []
    for f in ("or_sim1_G79W.csv", "or_sim2_I205W.csv"):
        d = load_or(os.path.join(SIM, f))
        t = d["Time"]; asc = d["Vertical velocity"] > 1.0
        ok = asc & np.isfinite(d["Mach number"]) & np.isfinite(d["CP location"]) & (d["Mach number"] > 0.03)
        M += list(d["Mach number"][ok]); Cd += list(d["Drag coefficient"][ok]); xcp += list(d["CP location"][ok] / 100)
        a = np.radians(d["Angle of attack"]); c = d["Normal force coefficient"]
        m = ok & (a > np.radians(0.5)) & (a < np.radians(10)) & np.isfinite(c)
        cn += list(c[m] / a[m])
    M, Cd, xcp = map(np.array, (M, Cd, xcp)); cna = float(np.median(cn))
    with open(out, "w") as f:
        f.write("# Mach Cd xcp_m CNa  (OpenRocket 62mm Canard Control V1.0.0; CNa median %.2f)\n" % cna)
        for lo in np.arange(0.0, 0.85, 0.05):
            m = (M >= lo) & (M < lo + 0.05)
            if m.sum() < 3:
                continue
            f.write("%.3f %.4f %.4f %.3f\n" % (M[m].mean(), np.median(Cd[m]), np.median(xcp[m]), cna))
    print("aero.csv: CNa %.2f, %d Mach bins" % (cna, sum(1 for _ in open(out)) - 1))


def motor_or(name, path, out):
    d = load_or(path)
    t, F, mm = d["Time"], d["Thrust"], d["Motor mass"] / 1000
    ok = np.isfinite(F) & np.isfinite(mm)
    t, F, mm = t[ok], F[ok], mm[ok]
    tb = t[np.nonzero(F > 0.01)[0][-1]] + 0.02
    k = t <= tb
    np.savetxt(out, np.column_stack([t[k], F[k], mm[k]]), fmt="%.4f", header=f"{name} from OpenRocket: t thrust motor_mass")


def motor_eng(name, path, out):
    L = [l.strip() for l in open(path, errors="replace") if l.strip() and not l.startswith(";")]
    h = L[0].split(); prop, tot = float(h[4]), float(h[5])
    pts = np.array([[float(x) for x in l.split()[:2]] for l in L[1:] if len(l.split()) >= 2])
    t = np.concatenate([[0.0], pts[:, 0]]); F = np.concatenate([[0.0], pts[:, 1]])
    I = np.concatenate([[0.0], np.cumsum(np.diff(t) * 0.5 * (F[1:] + F[:-1]))])
    m = tot - prop * I / I[-1]
    np.savetxt(out, np.column_stack([t, F, m]), fmt="%.4f",
               header=f"{name} {os.path.basename(path)} I={I[-1]:.1f} Ns: t thrust motor_mass")
    return I[-1], F.max(), t[-1], tot


if __name__ == "__main__":
    aero(os.path.join(HERE, "aero.csv"))
    motor_or("G79W", os.path.join(SIM, "or_sim1_G79W.csv"), os.path.join(HERE, "motors", "G79W.csv"))
    motor_or("I205W", os.path.join(SIM, "or_sim2_I205W.csv"), os.path.join(HERE, "motors", "I205W.csv"))
    db = sys.argv[1] if len(sys.argv) > 1 else None
    if db:
        pick = {"G40W": "AeroTech/G40_5f4294d20002e90000000463.eng"}
        want = ["G40", "G53", "G77", "G80", "G104"]
        for f in sorted(glob.glob(os.path.join(db, "AeroTech", "G*.eng"))):
            L = [l for l in open(f, errors="replace") if l.strip() and not l.startswith(";")]
            nm, dia, ln = L[0].split()[0], float(L[0].split()[1]), float(L[0].split()[2])
            base = nm.rstrip("ABCDEFGHIJKLMNOPQRSTUVWXYZ-0123456789") or nm
            if abs(dia - 29) > 0.6 or ln > 130:
                continue
            key = next((w for w in want if nm.startswith(w) and not nm[len(w):len(w)+1].isdigit()), None)
            if key and not os.path.exists(os.path.join(HERE, "motors", nm + ".csv")):
                I, Fm, tb, tot = motor_eng(nm, f, os.path.join(HERE, "motors", nm + ".csv"))
                print(f"{nm:8s} I {I:6.1f} Ns  Fmax {Fm:6.1f} N  tb {tb:.2f} s  total {tot*1000:.0f} g  <- {os.path.relpath(f, db)}")
