"""SW-01 flight software data flow: one 200 Hz frame, fixed grid, flow left to right.

Every block and arrow is taken from the code: app.c (frame loop, s_tasks[], task_sense,
task_flight, task_log, watchdog check-in), flight.c (Flight_Step: estimate ->
monitor_and_arbitrate -> Ctrl_SetQbar -> Ctrl_Update -> shadow), fdir.c (monitors,
Fdir_Capability), control.c (Ctrl_RollLaw, rate_damp), phase.c (Phase_ActuationAllowed),
board.h (rates). Same style as stm01_phase_machine.py.
Run: python sw01_flight_dataflow.py  ->  sw01_flight_dataflow.png / .svg
"""
import os
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import FancyBboxPatch, Rectangle

HERE = os.path.dirname(os.path.abspath(__file__))
PX, BW, BH = 5.0, 3.3, 1.35          # column pitch, box width, main-row box height
Y_MAIN, Y_TOP, Y_BOT, Y_SCH = 0.0, 2.55, -3.35, 4.75
BH_S = 1.05                            # top / bottom row box height
C = {  # fill, edge. Okabe-Ito palette (colour-vision-deficiency safe), mid-strength tints
    "hw":   ("#8c8c8c", "#303030"),   # grey
    "pad":  ("#f6ee8a", "#8a7f00"),   # yellow
    "est":  ("#5fc4a3", "#00664a"),   # bluish green
    "fdir": ("#f1a77a", "#a34700"),   # vermillion
    "fly":  ("#9cc3e4", "#00507d"),   # blue
    "io":   ("#d9a0c2", "#8e4573"),   # reddish purple
}
BLK, GRY = "#222222", "#555555"
ORG, PUR, GRN = "#00507d", C["io"][1], C["est"][1]


def cx(c):
    return c * PX


def box(ax, c, y, st, title, lines, h=BH, w=BW, fs=7.6):
    x = cx(c)
    fc, ec = C[st]
    ax.add_patch(FancyBboxPatch((x - w / 2, y - h / 2), w, h, boxstyle="round,pad=0.02,rounding_size=0.12",
                                fc=fc, ec=ec, lw=1.6, zorder=3))
    n = len(lines)
    top = y + 0.5 * 0.27 * n + 0.04
    ax.text(x, top, title, ha="center", va="center", fontsize=9.6, weight="bold", zorder=4)
    step = (h - 0.48) / max(n, 1)
    for i, s in enumerate(lines):
        ax.text(x, top - 0.32 - i * 0.27, s, ha="center", va="center", fontsize=fs, zorder=4,
                color="#000")


def path(ax, pts, color=BLK, ls="-", lw=1.4, head=True):
    xs, ys = zip(*pts)
    ax.plot(xs, ys, color=color, ls=ls, lw=lw, zorder=2, solid_capstyle="butt")
    if head:
        ax.annotate("", xy=pts[-1], xytext=pts[-2],
                    arrowprops=dict(arrowstyle="-|>", color=color, lw=lw, mutation_scale=13, shrinkA=0, shrinkB=0),
                    zorder=2)


def tag(ax, x, y, s, color=BLK, ha="center", va="bottom", fs=7.4):
    ax.text(x, y, s, ha=ha, va=va, fontsize=fs, color=color, zorder=5,
            bbox=dict(boxstyle="round,pad=0.15", fc="white", ec="none"))


fig = plt.figure(figsize=(26, 6.25))
ax = fig.add_axes([0.01, 0.0, 0.98, 0.935])
ax.set_xlim(-2.0, 8 * PX + 2.0); ax.set_ylim(Y_BOT - 1.25, Y_SCH + 0.5)
ax.set_aspect("equal"); ax.axis("off")

hw, hh, hs = BW / 2, BH / 2, BH_S / 2
M = dict(fontsize=8.8)

# frame schedule, s_tasks[] order
SCH = [("Frame tick", "5 ms deadline"), ("Sense", ""), ("Flight", ""), ("Log", ""), ("Telemetry", "100 Hz"),
       ("Comms", ""), ("LEDs", ""), ("Watchdog", "kicked if frame complete"), ("Maintenance", "log erase only")]
for i, (t, sub) in enumerate(SCH):
    x = cx(i)
    ax.add_patch(FancyBboxPatch((x - 1.5, Y_SCH - 0.36), 3.0, 0.72, boxstyle="round,pad=0.02,rounding_size=0.1",
                                fc="#f7f7f7", ec=GRY, lw=1.1, zorder=3))
    ax.text(x, Y_SCH + (0.11 if sub else 0.0), t, ha="center", va="center", fontsize=8.8, weight="bold", zorder=4)
    if sub:
        ax.text(x, Y_SCH - 0.15, sub, ha="center", va="center", fontsize=7.6, color="#333", zorder=4)
    if i < len(SCH) - 1:
        path(ax, [(x + 1.5, Y_SCH), (cx(i + 1) - 1.5, Y_SCH)], GRY, lw=1.1)

# task_flight and Flight_Step groupings
Y_RT, Y_RB = 1.50, -1.80
Y_IT, Y_IB = 1.25, -1.55
ax.add_patch(Rectangle((cx(2) - hw - 0.32, Y_RB), cx(7) - cx(2) + BW + 0.64, Y_RT - Y_RB,
                       fc="#f4f8fc", ec=ORG, lw=1.2, ls="--", zorder=0))
ax.add_patch(Rectangle((cx(2) - hw - 0.16, Y_IB), cx(6) - cx(2) + BW + 0.32, Y_IT - Y_IB,
                       fc="#fbfbfb", ec=GRY, lw=1.0, ls=":", zorder=1))
ax.text(cx(7) + hw + 0.25, Y_RT - 0.06, "task_flight", ha="right", va="top", fontsize=8.8, color=ORG, weight="bold")
ax.text(cx(2) - hw - 0.06, Y_IB + 0.06, "Flight_Step()", ha="left", va="bottom", fontsize=8.8, color=GRY, weight="bold")
ax.text(cx(1), hh + 0.12, "task_sense", ha="center", va="bottom", fontsize=8.8, color=C["pad"][1], weight="bold")

# main row
MAINBOX = [
    ("hw", "SENSORS", ["QMI8658A IMU, 448 Hz", "GPS receiver"]),
    ("pad", "SENSE", ["IMU read, gyro bias removed", "GPS poll"]),
    ("est", "ESTIMATE", ["IEKF attitude", r"Vertical navigation, $\bar{q}$"]),
    ("fdir", "FDIR MONITORS", ["Sensor, timing, estimator", "and GPS health"]),
    ("fly", "DETECTORS", ["Liftoff, burnout", "apogee, landing"]),
    ("fdir", "ARBITER", ["Phase transitions", "Control mode limit"]),
    ("fly", "ROLL CONTROL", [r"$\delta = -(c/\bar{q})\,p$ + trim", "Rate damping fallback", "Allocation, null-space dither"]),
    ("fly", "ACTUATION", ["Four fin commands", "Synchronous PWM, 8 MHz tick"]),
    ("hw", "SERVOS", [r"4$\times$ BMS-127WV+", "Canard deflection"]),
]
for i, (st, t, lines) in enumerate(MAINBOX):
    box(ax, i, Y_MAIN, st, t, lines, fs=8.2)

MAIN = ["IMU,\nGPS", "Sensor\ndata", "State\nestimate", "Fault\nstatus", "Phase\nevents", "Control\nmode",
        "Fin\ncommands", "PWM"]
for i, a in enumerate(MAIN):
    path(ax, [(cx(i) + hw, Y_MAIN), (cx(i + 1) - hw, Y_MAIN)])
    tag(ax, 0.5 * (cx(i) + cx(i + 1)), Y_MAIN, a, va="center", fs=8.0)

# estimate to control: attitude, rates and dynamic pressure for the gain schedule
Y_L = -1.0
path(ax, [(cx(2), -hh), (cx(2), Y_L), (cx(6), Y_L), (cx(6), -hh)], GRN)
tag(ax, cx(4), Y_L - 0.08, r"Attitude, body rates, $\bar{q}$", GRN, va="top", fs=8.0)

# top row
box(ax, 1, Y_TOP, "io", "USB HOST", ["Ground station"], h=BH_S, fs=8.2)
box(ax, 3, Y_TOP, "io", "COMMS", ["Console commands"], h=BH_S, fs=8.2)
box(ax, 6, Y_TOP, "io", "TELEMETRY", ["Vehicle state"], h=BH_S, fs=8.2)
path(ax, [(cx(1) + hw, Y_TOP), (cx(3) - hw, Y_TOP)], PUR)
tag(ax, cx(2), Y_TOP + 0.06, "Commands", PUR, fs=8.0)
path(ax, [(cx(3), Y_TOP - hs), (cx(3), hh)], PUR)
tag(ax, cx(3) - 0.12, 1.78, "Fault clear", PUR, ha="right", va="center", fs=8.0)
path(ax, [(cx(6), Y_RT), (cx(6), Y_TOP - hs)], PUR)
tag(ax, cx(6) + 0.12, 1.78, "Frame state", PUR, ha="left", va="center", fs=8.0)
Y_TL = Y_TOP + hs + 0.42
path(ax, [(cx(6), Y_TOP + hs), (cx(6), Y_TL), (cx(1), Y_TL), (cx(1), Y_TOP + hs)], PUR)
tag(ax, cx(3.5), Y_TL + 0.05, "Telemetry", PUR, fs=8.0)

# bottom row
box(ax, 3, Y_BOT, "io", "EVENTS", ["Faults, phase, mode"], h=BH_S, fs=8.2)
box(ax, 4, Y_BOT, "io", "LOG", ["Pre-trigger buffer"], h=BH_S, fs=8.2)
box(ax, 5, Y_BOT, "hw", "SPI FLASH", ["Flight record"], h=BH_S, fs=8.2)
box(ax, 7, Y_BOT, "io", "LEDS", ["Phase and fault status"], h=BH_S, fs=8.2)
for c, lab in ((3, "Events"), (4, "State records")):
    path(ax, [(cx(c), Y_IB), (cx(c), Y_BOT + hs)], PUR)
    tag(ax, cx(c) + 0.12, 0.5 * (Y_IB + Y_BOT + hs), lab, PUR, ha="left", va="center", fs=8.0)
path(ax, [(cx(3) + hw, Y_BOT), (cx(4) - hw, Y_BOT)], PUR)
path(ax, [(cx(4) + hw, Y_BOT), (cx(5) - hw, Y_BOT)], PUR)
path(ax, [(cx(7), Y_RB), (cx(7), Y_BOT + hs)], PUR)
tag(ax, cx(7) + 0.12, 0.5 * (Y_RB + Y_BOT + hs), "Phase", PUR, ha="left", va="center", fs=8.0)

# legend
LEG = [("hw", "Hardware"), ("pad", "Sensing"), ("est", "Estimation"), ("fdir", "Fault handling"),
       ("fly", "Guidance and control"), ("io", "I/O and logging")]
# key: stacked, bottom left
KDY = 0.44
KX, KY = cx(0) - hw, (Y_BOT - hs + 0.17) + 6 * KDY
ax.text(KX, KY, "Key", ha="left", va="center", fontsize=9.0, weight="bold")
for k, (st, name) in enumerate(LEG):
    yk = KY - (k + 1) * KDY
    fc, ec = C[st]
    ax.add_patch(FancyBboxPatch((KX, yk - 0.17), 0.5, 0.34, boxstyle="round,pad=0.01,rounding_size=0.05",
                                fc=fc, ec=ec, lw=1.4))
    ax.text(KX + 0.7, yk, name, ha="left", va="center", fontsize=8.6)

fig.suptitle("FC-V4 flight software data flow, one 200 Hz frame", x=0.02, y=0.985, ha="left",
             fontsize=13, weight="bold")
for ext in ("png", "svg"):
    fig.savefig(os.path.join(HERE, "sw01_flight_dataflow." + ext), dpi=160)
print("ok")
