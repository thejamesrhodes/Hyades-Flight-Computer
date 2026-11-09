"""STM-01 flight phase machine diagram: fixed grid, flow left to right, orthogonal routing.

Every transition is taken from the code (phase.c k_allowed, detect.c, flight.c
monitor_and_arbitrate, app.c boot) and carries its T-number from the STM-01 §2 guard table.
Colours: Okabe-Ito palette (colour-vision-deficiency safe), same scheme as sw01_flight_dataflow.py.
Run: python stm01_phase_machine.py  ->  stm01_phase_machine.png / .svg
"""
import os
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import FancyBboxPatch

HERE = os.path.dirname(os.path.abspath(__file__))
PX, BW, BH = 4.4, 2.75, 1.35          # column pitch, box width, box height
Y_MAIN, Y_FAULT = 0.0, -3.6
C = {  # fill, edge
    "pad":   ("#9cc3e4", "#00507d"),   # blue
    "armed": ("#5fc4a3", "#00664a"),   # bluish green
    "fly":   ("#f6ee8a", "#8a7f00"),   # yellow
    "fault": ("#f1a77a", "#a34700"),   # vermillion
    "end":   ("#8c8c8c", "#303030"),   # grey
}
BLK, GRY = "#222222", "#555555"
FLT = C["fault"][1]                    # fault transitions
REC = "#8e4573"                        # recovery and bypass transitions (reddish purple)

# name: (column, row y, style, title, line 1, line 2)
S = {
    "BOOT":  (0, Y_MAIN, "pad",   "BOOT",      "Drivers initialised",   "Fins centred"),
    "CAL":   (1, Y_MAIN, "pad",   "CALIBRATE", "Gyro bias over 1 s",    "IEKF seeded"),
    "SET":   (2, Y_MAIN, "pad",   "SETTLING",  "Estimator converging",  "Fins off"),
    "READY": (3, Y_MAIN, "armed", "READY",     "Log buffer armed",      "Fins off"),
    "BOOST": (4, Y_MAIN, "fly",   "BOOST",     "Roll control active",   "Log committed"),
    "COAST": (5, Y_MAIN, "fly",   "COAST",     "Roll control active",   "Log at 100 Hz"),
    "DESC":  (6, Y_MAIN, "fly",   "DESCENT",   "Fins off",              "Log at 20 Hz"),
    "LAND":  (7, Y_MAIN, "end",   "LANDED",    "Terminal state",        "Log closes after 60 s"),
    "LOCK":  (2, Y_FAULT, "fault", "LOCKOUT",  "Pad fault, latched",    "Fins off"),
    "CATO":  (5, Y_FAULT, "fault", "CATO_SAFE", "Flight fault, latched", "Fins off"),
}


def X(n):
    return S[n][0] * PX


def box(ax, n):
    c, y, st, t, l1, l2 = S[n]
    x = c * PX
    fc, ec = C[st]
    ax.add_patch(FancyBboxPatch((x - BW / 2, y - BH / 2), BW, BH, boxstyle="round,pad=0.02,rounding_size=0.12",
                                fc=fc, ec=ec, lw=1.8, zorder=3))
    ax.text(x, y + 0.30, t, ha="center", va="center", fontsize=10.5, weight="bold", zorder=4)
    ax.text(x, y - 0.03, l1, ha="center", va="center", fontsize=8.6, zorder=4)
    ax.text(x, y - 0.30, l2, ha="center", va="center", fontsize=8.6, zorder=4)


def path(ax, pts, color=BLK, lw=1.5, head=True):
    xs, ys = zip(*pts)
    ax.plot(xs, ys, color=color, lw=lw, zorder=2, solid_capstyle="butt")
    if head:
        ax.annotate("", xy=pts[-1], xytext=pts[-2],
                    arrowprops=dict(arrowstyle="-|>", color=color, lw=lw, mutation_scale=14, shrinkA=0, shrinkB=0),
                    zorder=2)


def tag(ax, x, y, s, color=BLK, ha="center", va="center", fs=8.4):
    ax.text(x, y, s, ha=ha, va=va, fontsize=fs, color=color, zorder=5, linespacing=1.15,
            bbox=dict(boxstyle="round,pad=0.2", fc="white", ec="none"))


fig = plt.figure(figsize=(22, 6.55))
ax = fig.add_axes([0.01, 0.0, 0.98, 0.93])
ax.set_xlim(-1.9, 7 * PX + 1.9); ax.set_ylim(-6.45, 3.35)
ax.set_aspect("equal"); ax.axis("off")
for n in S:
    box(ax, n)
hw, hh = BW / 2, BH / 2
Y_V = -1.25                                   # label height on the downward fault lines

# nominal sequence (T1 to T7), T4 drawn heavy as the launch commit
main = [("BOOT", "CAL", "T1\nBoot OK"), ("CAL", "SET", "T2\nCalibrated"), ("SET", "READY", "T3\nHold met"),
        ("READY", "BOOST", "T4\nLiftoff"), ("BOOST", "COAST", "T5\nBurnout"), ("COAST", "DESC", "T6\nApogee"),
        ("DESC", "LAND", "T7\nLanding")]
for a, b, s in main:
    path(ax, [(X(a) + hw, 0.0), (X(b) - hw, 0.0)], lw=2.8 if a == "READY" else 1.5)
    tag(ax, (X(a) + X(b)) / 2, 0.0, s)

# demotion (T8) and launch from SETTLING (T9)
yA, yB = 1.3, 2.1
path(ax, [(X("READY") - 0.55, hh), (X("READY") - 0.55, yA), (X("SET") + 0.55, yA), (X("SET") + 0.55, hh)], REC)
tag(ax, (X("SET") + X("READY")) / 2, yA, "T8  SOFT fault", REC)
path(ax, [(X("SET") - 0.55, hh), (X("SET") - 0.55, yB), (X("BOOST"), yB), (X("BOOST"), hh)], REC)
tag(ax, (X("SET") + X("READY")) / 2, yB, "T9  Liftoff before READY", REC)

# pad faults into LOCKOUT
yBus = -1.95
lx_in = X("LOCK") - 0.7
for n, s in (("BOOT", "T10\nIMU init fail"), ("CAL", "T11\nWatchdog reset")):
    path(ax, [(X(n), -hh), (X(n), yBus)], FLT, head=False)
    tag(ax, X(n), Y_V, s, FLT)
path(ax, [(X("BOOT"), yBus), (lx_in, yBus), (lx_in, Y_FAULT + hh)], FLT)
ax.plot([X("CAL")], [yBus], "o", color=FLT, ms=4.5, zorder=4)
path(ax, [(X("SET"), -hh), (X("SET"), Y_FAULT + hh)], FLT)
tag(ax, X("SET"), Y_V, "T12\nHARD fault", FLT)
path(ax, [(X("READY"), -hh), (X("READY"), Y_FAULT - 0.25), (X("LOCK") + hw, Y_FAULT - 0.25)], FLT)
tag(ax, X("READY"), Y_V, "T12\nHARD fault", FLT)
path(ax, [(X("LOCK") + 0.7, Y_FAULT + hh), (X("LOCK") + 0.7, -hh)], REC)
tag(ax, X("LOCK") + 0.7, -2.45, "T13\nOperator clear", REC)

# flight faults into CATO_SAFE, then CATO_SAFE to LANDED
path(ax, [(X("BOOST"), -hh), (X("BOOST"), Y_FAULT), (X("CATO") - hw, Y_FAULT)], FLT)
tag(ax, X("BOOST"), Y_V, "T14\nHARD fault", FLT)
path(ax, [(X("COAST"), -hh), (X("COAST"), Y_FAULT + hh)], FLT)
tag(ax, X("COAST"), Y_V, "T14\nHARD fault", FLT)
path(ax, [(X("CATO") + hw, Y_FAULT), (X("LAND"), Y_FAULT), (X("LAND"), -hh)], FLT)
tag(ax, (X("CATO") + hw + X("LAND")) / 2, Y_FAULT, "T15  Landing", FLT)
ax.text(X("DESC"), Y_V, "HARD fault ignored,\nlanding detection continues", ha="center", va="center",
        fontsize=8.4, color=GRY)

# corrupt protected phase word, from any phase
for n in ("LOCK", "CATO"):
    path(ax, [(X(n), Y_FAULT - 1.25), (X(n), Y_FAULT - hh)], FLT)
    tag(ax, X(n), Y_FAULT - 1.4, "T16  Phase word corrupt", FLT, va="top")

# liftoff divider: HARD fault outcomes differ either side
XD = 0.5 * (X("READY") + X("BOOST"))
ax.plot([XD, XD], [-5.95, 2.75], color="#7a7a7a", lw=2.0, zorder=1, solid_capstyle="butt")
ax.text(XD - 0.3, 3.05, "Before liftoff", ha="right", va="center", fontsize=10, weight="bold")
ax.text(XD - 0.3, 2.68, "Hold safe on pad", ha="right", va="center", fontsize=8.6)
ax.text(XD + 0.3, 3.05, "After liftoff", ha="left", va="center", fontsize=10, weight="bold")
ax.text(XD + 0.3, 2.68, "Fins off, recovery continues", ha="left", va="center", fontsize=8.6)

# key, stacked, bottom left
KX, KY, KDY = -hw, -2.3, 0.42
ax.text(KX, KY, "Key", ha="left", va="center", fontsize=9.2, weight="bold")
KEYS = [("box", "pad", "Pad preparation"), ("box", "armed", "Armed on pad"), ("box", "fly", "Flight"),
        ("box", "fault", "Fault, latched"), ("box", "end", "Terminal"),
        ("line", (BLK, 1.5), "Nominal transition"), ("line", (BLK, 2.8), "Launch commit"),
        ("line", (FLT, 1.5), "Fault transition"), ("line", (REC, 1.5), "Recovery or bypass")]
for k, (kind, v, name) in enumerate(KEYS):
    yk = KY - (k + 1) * KDY
    if kind == "box":
        fc, ec = C[v]
        ax.add_patch(FancyBboxPatch((KX, yk - 0.15), 0.55, 0.30, boxstyle="round,pad=0.01,rounding_size=0.05",
                                    fc=fc, ec=ec, lw=1.4))
    else:
        ax.plot([KX, KX + 0.55], [yk, yk], color=v[0], lw=v[1], solid_capstyle="butt")
    ax.text(KX + 0.75, yk, name, ha="left", va="center", fontsize=8.8)

fig.suptitle("Hyades FC-V4 flight phase machine", x=0.02, y=0.985, ha="left", fontsize=13, weight="bold")
for ext in ("png", "svg"):
    fig.savefig(os.path.join(HERE, f"stm01_phase_machine.{ext}"), dpi=160)
print("ok")
