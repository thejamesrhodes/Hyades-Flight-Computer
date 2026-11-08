"""STM-01 flight phase machine diagram: fixed grid, flow left to right, orthogonal routing.

Every transition and guard is taken from the code (phase.c k_allowed, detect.c, flight.c
monitor_and_arbitrate, app.c boot and log_on_phase) and cross-referenced by its T-number
in STM-01 §2. Run: python stm01_phase_machine.py  ->  stm01_phase_machine.png / .svg
"""
import os
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import FancyBboxPatch

HERE = os.path.dirname(os.path.abspath(__file__))
PX, BW, BH = 3.4, 2.05, 1.05          # column pitch, box width, box height
Y_MAIN, Y_FAULT = 0.0, -3.3
C = {  # fill, edge
    "pad":   ("#eef3fb", "#4a6fa5"),
    "armed": ("#e6f5e6", "#3b8a3b"),
    "fly":   ("#fff4e0", "#c07a00"),
    "fault": ("#fde8e8", "#b03030"),
    "end":   ("#ececec", "#555555"),
}
RED, ORG, BLK, GRY = "#b03030", "#c07a00", "#222222", "#555555"

# name: (column, row y, style, title, line 1, line 2)
S = {
    "BOOT":    (0, Y_MAIN, "pad",   "BOOT",      "reset, drivers up",      "fins centred"),
    "CAL":     (1, Y_MAIN, "pad",   "CALIBRATE", "gyro bias (1 s)",        "IEKF seeded"),
    "SET":     (2, Y_MAIN, "pad",   "SETTLING",  "estimator converging",   "fins OFF"),
    "READY":   (3, Y_MAIN, "armed", "READY",     "fins OFF (flight build)", "log ring ARMED"),
    "BOOST":   (4, Y_MAIN, "fly",   "BOOST",     "roll law FULL / RDAMP",  "log COMMITTED"),
    "COAST":   (5, Y_MAIN, "fly",   "COAST",     "roll law FULL / RDAMP",  "log 100 Hz"),
    "DESC":    (6, Y_MAIN, "fly",   "DESCENT",   "fins OFF",               "log 20 Hz"),
    "LAND":    (7, Y_MAIN, "end",   "LANDED",    "terminal",               "log closes +60 s"),
    "LOCK":    (2, Y_FAULT, "fault", "LOCKOUT",  "pad, latched, fins OFF", "exit: operator clear"),
    "CATO":    (5, Y_FAULT, "fault", "CATO_SAFE", "in flight, latched",    "fins OFF; landing runs"),
}


def X(n):
    return S[n][0] * PX


def Y(n):
    return S[n][1]


def box(ax, n):
    c, y, st, t, l1, l2 = S[n]
    x = c * PX
    fc, ec = C[st]
    ax.add_patch(FancyBboxPatch((x - BW / 2, y - BH / 2), BW, BH, boxstyle="round,pad=0.02,rounding_size=0.12",
                                fc=fc, ec=ec, lw=1.6, zorder=3))
    ax.text(x, y + 0.27, t, ha="center", va="center", fontsize=10.5, weight="bold", zorder=4)
    ax.text(x, y - 0.06, l1, ha="center", va="center", fontsize=8, zorder=4)
    ax.text(x, y - 0.30, l2, ha="center", va="center", fontsize=8, zorder=4, color="#333")


def path(ax, pts, color=BLK, ls="-", lw=1.4, head=True):
    xs, ys = zip(*pts)
    ax.plot(xs[:-1] + (xs[-1],), ys, color=color, ls=ls, lw=lw, zorder=2, solid_capstyle="butt")
    if head:
        ax.annotate("", xy=pts[-1], xytext=pts[-2],
                    arrowprops=dict(arrowstyle="-|>", color=color, lw=lw, mutation_scale=13, shrinkA=0, shrinkB=0),
                    zorder=2)


def tag(ax, x, y, s, color=BLK, ha="center", va="bottom"):
    ax.text(x, y, s, ha=ha, va=va, fontsize=8, color=color, zorder=5,
            bbox=dict(boxstyle="round,pad=0.15", fc="white", ec="none"))


fig = plt.figure(figsize=(17, 10.6))
ax = fig.add_axes([0.02, 0.37, 0.96, 0.62])
ax.set_xlim(-1.4, 7 * PX + 1.4); ax.set_ylim(Y_FAULT - 1.55, 3.35)
ax.set_aspect("equal"); ax.axis("off")
for n in S:
    box(ax, n)

hw, hh = BW / 2, BH / 2
# --- nominal sequence, main row (T1-T7) ---
main = [("BOOT", "CAL", "T1  boot ok"), ("CAL", "SET", "T2  seeded"), ("SET", "READY", "T3  hold met"),
        ("READY", "BOOST", "T4  LIFTOFF"), ("BOOST", "COAST", "T5  BURNOUT"), ("COAST", "DESC", "T6  APOGEE"),
        ("DESC", "LAND", "T7  LANDING")]
for a, b, s in main:
    lw = 2.6 if a == "READY" else 1.5
    path(ax, [(X(a) + hw, 0.0), (X(b) - hw, 0.0)], lw=lw)
    tag(ax, (X(a) + X(b)) / 2, 0.12, s)

# --- top lanes: demotion (inner) and launch from SETTLING (outer) ---
yA, yB = 1.25, 2.05
path(ax, [(X("READY") - 0.45, hh), (X("READY") - 0.45, yA), (X("SET") + 0.45, yA), (X("SET") + 0.45, hh)],
     ls=(0, (5, 3)))
tag(ax, (X("SET") + X("READY")) / 2, yA + 0.06, "T8  SOFT fault → demote (not if liftoff pending)")
path(ax, [(X("SET") - 0.45, hh), (X("SET") - 0.45, yB), (X("BOOST"), yB), (X("BOOST"), hh)],
     color=ORG, ls=(0, (5, 3)))
tag(ax, (X("SET") + X("BOOST")) / 2, yB + 0.06, "T9  LIFTOFF while not READY  (STM-01 F3)", color=ORG)

# --- pad faults into LOCKOUT ---
yBus = -1.55
lx_in = X("LOCK") - 0.55
for n, s in (("BOOT", "T10  IMU init /\nself-test fail"), ("CAL", "T11  last reset\nwas IWDG")):
    path(ax, [(X(n), -hh), (X(n), yBus)], color=RED, head=False)
    tag(ax, X(n) + 0.08, (yBus - hh) / 2, s, color=RED, ha="left", va="center")
path(ax, [(X("BOOT"), yBus), (lx_in, yBus), (lx_in, Y_FAULT + hh)], color=RED)
ax.plot([X("CAL")], [yBus], "o", color=RED, ms=4, zorder=4)
path(ax, [(X("SET"), -hh), (X("SET"), Y_FAULT + hh)], color=RED)
tag(ax, X("SET") + 0.08, -1.25, "T12  HARD", color=RED, ha="left", va="center")
path(ax, [(X("READY"), -hh), (X("READY"), Y_FAULT - 0.2), (X("LOCK") + hw, Y_FAULT - 0.2)], color=RED)
tag(ax, X("READY") + 0.08, -1.25, "T12  HARD", color=RED, ha="left", va="center")
# operator clear back to SETTLING
path(ax, [(X("LOCK") + 0.55, Y_FAULT + hh), (X("LOCK") + 0.55, -hh)], color=GRY, ls=(0, (1.5, 2.5)), lw=1.6)
tag(ax, X("LOCK") + 0.63, -2.35, "T13  operator clear\n(pad, no HARD active)", color=GRY, ha="left", va="center")

# --- flight faults into CATO_SAFE, CATO_SAFE to LANDED ---
path(ax, [(X("BOOST"), -hh), (X("BOOST"), Y_FAULT), (X("CATO") - hw, Y_FAULT)], color=RED)
tag(ax, X("BOOST") + 0.08, -1.25, "T14  HARD", color=RED, ha="left", va="center")
path(ax, [(X("COAST"), -hh), (X("COAST"), Y_FAULT + hh)], color=RED)
tag(ax, X("COAST") + 0.08, -1.25, "T14  HARD", color=RED, ha="left", va="center")
path(ax, [(X("CATO") + hw, Y_FAULT), (X("LAND"), Y_FAULT), (X("LAND"), -hh)], color=RED)
tag(ax, (X("CATO") + X("LAND")) / 2 + 0.6, Y_FAULT + 0.06, "T15  same landing rules as T7  (F2)", color=RED)
# HARD in DESCENT: refused
tag(ax, X("DESC"), -1.25, "HARD here: CATO refused,\nstays DESCENT, landing runs", color=GRY, va="center")

# --- corrupt protected word (from any phase) ---
for n, s in (("LOCK", "T16  phase word corrupt, on pad"), ("CATO", "T16  phase word corrupt, in flight (F1)")):
    path(ax, [(X(n), Y_FAULT - 1.15), (X(n), Y_FAULT - hh)], color=RED, ls=(0, (2, 2)))
    tag(ax, X(n), Y_FAULT - 1.2, s, color=RED, va="top")

ax.text(-1.3, 3.2, "STM-01  Hyades FC-V4 flight phase machine (as built, firmware 8 Nov 2026)",
        fontsize=13, weight="bold", ha="left", va="center")
ax.text(-1.3, 2.88, "Solid: nominal   Dashed: pad recovery / bypass   Red: fault handling   Dotted grey: operator   "
        "Thick: launch commit", fontsize=8.5, ha="left", va="center", color="#444")

# --- guard table ---
G = [
    ("T1", "BOOT → CALIBRATE", "IMU init and self-test pass", "app.c boot"),
    ("T2", "CALIBRATE → SETTLING", "1 s gyro-bias average done, IEKF seeded, previous reset not IWDG", "app.c boot"),
    ("T3", "SETTLING → READY", "worst severity = INFO for READY_HOLD_MS; ready_inhibit false (flight build: ≥ 200 KB erased log)", "arbiter"),
    ("T4", "READY → BOOST", "f_ax > 3 g for 100 ms and nav v_up > 8 m/s,  or  f_ax > 3 g for 300 ms", "Detect_Step"),
    ("T5", "BOOST → COAST", "t_boost ≥ 0.1 s and f_ax < 0 for 50 ms", "Detect_Step"),
    ("T6", "COAST → DESCENT", "nav valid and v_up < 0 for 50 ms,  or  t_coast ≥ DET_T_APOGEE_MAX_S (11.5 s; set per motor)", "Detect_Step"),
    ("T7", "DESCENT → LANDED", "t ≥ 15 s and [GPS quiet 2 s | shock > 3 g then still 2 s | still 30 s],  or  t ≥ 300 s; runs despite HARD", "Detect_Step"),
    ("T8", "READY → SETTLING", "worst = SOFT and no liftoff pending (detectors run first); log ring discarded", "arbiter"),
    ("T9", "SETTLING → BOOST", "T4 rule; log starts at once; control mode still capped by FDIR", "Detect_Step"),
    ("T10–12", "BOOT, CAL, SETTLING, READY → LOCKOUT", "IMU init fail / IWDG reset cause / any HARD on the pad; armed log ring discarded", "app.c, arbiter"),
    ("T13", "LOCKOUT → SETTLING", "operator 'faults clear' on the pad with no HARD active (Phase_Recover); full READY hold again", "serve_clear"),
    ("T14", "BOOST, COAST → CATO_SAFE", "any HARD in flight", "arbiter"),
    ("T15", "CATO_SAFE → LANDED", "T7 rules", "Detect_Step"),
    ("T16", "any → LOCKOUT / CATO_SAFE", "complement check fails: in flight per the detector's last good phase (det.last)", "arbiter"),
]
tb = fig.add_axes([0.02, 0.05, 0.96, 0.31]); tb.axis("off")
cols = [0.0, 0.055, 0.25, 0.905]
for i, h in enumerate(("#", "Transition", "Guard (code)", "Requested by")):
    tb.text(cols[i], 1.0, h, fontsize=8.5, weight="bold", va="top")
for r, row in enumerate(G):
    y = 0.93 - r * 0.066
    for i, v in enumerate(row):
        tb.text(cols[i], y, v, fontsize=8, va="top", family="DejaVu Sans")
tb.text(0.0, -0.035, "Actuation only in BOOST and COAST (READY too on bench builds, CFG_BENCH_ACTUATE_READY). Mode FULL / RATE_DAMP / OFF "
        "from the FDIR capability ladder; CFG_CTRL_SHADOW 1 logs the law with canards held at 0.  LANDED is the only terminal state.",
        fontsize=8, va="top", color="#444")

for ext in ("png", "svg"):
    fig.savefig(os.path.join(HERE, f"stm01_phase_machine.{ext}"), dpi=150)
print("ok")
