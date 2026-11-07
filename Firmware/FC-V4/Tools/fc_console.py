#!/usr/bin/env python3
"""
fc_console.py -- bench and field console for the Hyades FC-V4 (replaces Tera Term).

    python fc_console.py              # find the board (USB 0483:5740) and connect
    python fc_console.py --port COM8  # a specific port
    python fc_console.py --sim        # no board: built-in simulator (fc_sim.py)

One window for everything that used to need a terminal plus fc_flash.py and
fc_pull.py: live status, flight phase and faults always on screen, a terminal
with history/completion/find, firmware flashing and log pull+decode over USB,
and an automatic session log of every line the board sent.

Design rules (ISO 9241-110 dialogue principles, Nielsen's heuristics, WCAG 2.1):
  * System status always visible: link, phase, control mode, faults, data age
    and build are in a header that never scrolls or changes with the tab.
  * Colour is never the only cue: every coloured state is also a word.
  * Field theme is dark-on-light (readable in sunlight); bench theme is dark.
    Text contrast >= 4.5:1 in both. Ctrl +/- scales every font.
  * Large targets (>= 32 px tall buttons), everything reachable by keyboard,
    shortcuts shown next to the action.
  * Error prevention: destructive actions (flash, erase, reboot, inject) ask
    first, say what will happen, default to Cancel, and are disabled in
    flight phases, while disconnected or while another operation runs.
  * The UI thread never blocks: one link thread owns the port, operations run
    in a worker thread and report progress.

Requires Python 3.9+ with tkinter, and pyserial (pip install pyserial).
"""
from __future__ import annotations

import argparse
import collections
import glob
import contextlib
import io
import json
import os
import queue
import re
import subprocess
import sys
import threading
import time
import tkinter as tk
import zlib
from tkinter import filedialog, messagebox, ttk
import tkinter.font as tkfont

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import fc_link as link                                          # noqa: E402

APP = "Hyades FC-V4 Console"
SETTINGS = os.path.join(os.path.expanduser("~"), ".fc_console.json")
FW_ROOT = os.path.normpath(os.path.join(HERE, ".."))
DEFAULT_ELF = os.path.join(FW_ROOT, "Debug", "Flight Controller V4.elf")
LOG_DIR = os.path.join(HERE, "logs")
ERR_RE = re.compile(r"^(ERR|FWABORT|usage)|\bFAIL(ED)?\b|\bbad\b(?!=)|CROSSED|no signal|CORRUPT|held low|floating")
STALE_S = 1.5                       # status older than this is shown as STALE

# ---------------------------------------------------------------------------
# Themes. Roles, not colours, are referenced everywhere else.
# Contrast (text on bg): field 15.4:1 / dim 7.0:1; bench 12.9:1 / dim 6.0:1.
# ---------------------------------------------------------------------------
THEMES = {
    "field": dict(bg="#ffffff", panel="#eceff3", edge="#9aa4b0", text="#111418", dim="#4a5360",
                  term="#ffffff", sel="#b9d4ff", ok="#0b6b2e", warn="#8a5200", bad="#b00020",
                  info="#0b4f9c", sent="#5b2a86", okbg="#d6f2df", warnbg="#ffe8bf", badbg="#ffd6dc",
                  infobg="#d8e7fb", idlebg="#e3e6ea", btn="#dfe4ea", btnhot="#cdd5df", off="#7d8590"),
    "bench": dict(bg="#14181e", panel="#1c222b", edge="#3a4452", text="#e3e8ee", dim="#a3adb9",
                  term="#0e1115", sel="#2e4a6e", ok="#5fd18f", warn="#f0b54a", bad="#ff6b6b",
                  info="#6fb3ff", sent="#c39bff", okbg="#173d27", warnbg="#4a3510", badbg="#55181d",
                  infobg="#13324f", idlebg="#2a313b", btn="#2a323d", btnhot="#36404d", off="#7a8592"),
}
PHASE_STYLE = {"READY": "ok", "LANDED": "idle", "LOCKOUT": "bad", "CATO": "bad",
               "BOOST": "info", "COAST": "info", "DESCENT": "info"}       # others: warn


def load_settings() -> dict:
    try:
        with open(SETTINGS, encoding="utf-8") as fh:
            return json.load(fh)
    except (OSError, ValueError):
        return {}


def save_settings(d: dict) -> None:
    try:
        with open(SETTINGS, "w", encoding="utf-8") as fh:
            json.dump(d, fh, indent=1)
    except OSError:
        pass


def newest_source() -> float:
    t = 0.0
    for sub in ("Core", "USB_DEVICE"):
        for dp, _d, files in os.walk(os.path.join(FW_ROOT, sub)):
            for f in files:
                if f.endswith((".c", ".h", ".s", ".ld")) and f != "build_id.h":
                    with contextlib.suppress(OSError):
                        t = max(t, os.path.getmtime(os.path.join(dp, f)))
    return t


def find_cubeide(hint: str = "") -> str:
    """stm32cubeidec.exe (the console-mode launcher) for headless builds."""
    pats = [hint] if hint else []
    pats += [os.path.join(os.path.dirname(os.path.dirname(FW_ROOT)), "..", "STM32_IDE", "STM32CubeIDE_*",
                          "STM32CubeIDE", "stm32cubeidec.exe"),
             r"C:\Hyades Rocket Files\STM32_IDE\STM32CubeIDE_*\STM32CubeIDE\stm32cubeidec.exe",
             r"C:\ST\STM32CubeIDE_*\STM32CubeIDE\stm32cubeidec.exe",
             "/opt/st/stm32cubeide_*/stm32cubeide"]
    for pat in pats:
        hits = sorted(glob.glob(os.path.normpath(pat)))
        if hits:
            return hits[-1]
    return ""


def open_folder(path: str) -> None:
    os.makedirs(path, exist_ok=True)
    if sys.platform.startswith("win"):
        os.startfile(path)                                       # noqa: S606
    else:
        subprocess.Popen(["open" if sys.platform == "darwin" else "xdg-open", path])


class Tile(tk.Frame):
    """Caption over a value. The value carries a word AND a background colour."""

    def __init__(self, app, master, caption: str, width: int = 10):
        super().__init__(master, bd=0, highlightthickness=1)
        self.app = app
        self.cap = tk.Label(self, text=caption.upper(), anchor="w", font=app.f_cap, padx=8, pady=1)
        self.val = tk.Label(self, text="--", anchor="w", font=app.f_tile, padx=8, width=1)  # grid sets width
        self.cap.pack(fill="x")
        self.val.pack(fill="x", pady=(0, 3))
        self.style = "idle"
        app.themed.append(self)

    def set(self, text: str, style: str = "idle") -> None:
        if text != self.val.cget("text"):
            self.val.configure(text=text)
        if style != self.style:
            self.style = style
            self.retheme(self.app.c)

    def retheme(self, c: dict) -> None:
        bg = c.get(self.style + "bg", c["idlebg"])
        fg = c.get(self.style, c["text"]) if self.style != "idle" else c["text"]
        self.configure(bg=bg, highlightbackground=c["edge"])
        self.cap.configure(bg=bg, fg=c["text"] if self.style != "idle" else c["dim"])
        self.val.configure(bg=bg, fg=fg)


class VScroll(tk.Frame):
    """A frame that scrolls vertically only when its content does not fit
    (small field laptops, large text). Content goes in .body."""

    def __init__(self, master, **kw):
        super().__init__(master, **kw)
        self.cv = tk.Canvas(self, highlightthickness=0, bd=0)
        self.sb = ttk.Scrollbar(self, orient="vertical", command=self.cv.yview)
        self.cv.configure(yscrollcommand=self._sb_set)
        self.cv.pack(side="left", fill="both", expand=True)
        self.body = tk.Frame(self.cv)
        self._win = self.cv.create_window((0, 0), window=self.body, anchor="nw")
        self.body.bind("<Configure>", lambda _e: self.cv.configure(scrollregion=self.cv.bbox("all")))
        self.cv.bind("<Configure>", lambda e: self.cv.itemconfigure(self._win, width=e.width))
        self.bind("<Enter>", lambda _e: self._wheel(True))
        self.bind("<Leave>", lambda _e: self._wheel(False))

    def _sb_set(self, lo, hi) -> None:
        if float(lo) <= 0.0 and float(hi) >= 1.0:
            self.sb.pack_forget()
        elif not self.sb.winfo_ismapped():
            self.sb.pack(side="right", fill="y", before=self.cv)
        self.sb.set(lo, hi)

    def _wheel(self, on: bool) -> None:
        for seq in ("<MouseWheel>", "<Button-4>", "<Button-5>"):
            if on:
                self.cv.bind_all(seq, self._scroll)
            else:
                self.cv.unbind_all(seq)

    def _scroll(self, e) -> None:
        if self.sb.winfo_ismapped():
            n = -1 if (getattr(e, "num", 0) == 4 or getattr(e, "delta", 0) > 0) else 1
            self.cv.yview_scroll(n, "units")

    def retheme(self, c: dict) -> None:
        for w in (self, self.cv, self.body):
            w.configure(bg=c["bg"])


# Okabe-Ito colour-blind-safe series colours (legend text carries the name too).
SERIES = ("#0072B2", "#D55E00", "#009E73", "#CC79A7")


class StripChart(tk.Canvas):
    """Scrolling time plot of a few series, newest at the right edge.

    Fixed symmetric y-range by default (a jumping auto-scale hides the size of
    a disturbance); values beyond it are drawn clipped with a marker. The
    legend shows each series' current value, so it is readable without
    hovering.
    """

    def __init__(self, app, master, title: str, unit: str, names: tuple, keys: tuple, yrange: float,
                 scales: tuple = (), ranges: tuple = ()):
        super().__init__(master, height=170, highlightthickness=1, bd=0)
        self.app, self.title, self.unit = app, title, unit
        self.names, self.keys, self.yr = names, keys, yrange
        self.scales = scales or (1.0,) * len(keys)        # plotted = value * scale; legend shows value
        self.ranges = ranges or (yrange,)                 # click the chart to step through these
        self.bind("<Configure>", lambda _e: self.draw())
        self.bind("<Button-1>", lambda _e: self._next_range())

    def _next_range(self) -> None:
        r = self.ranges
        self.yr = r[(r.index(self.yr) + 1) % len(r)] if self.yr in r else r[0]
        self.draw()

    def draw(self) -> None:
        a, c = self.app, self.app.c
        self.delete("all")
        self.configure(bg=c["term"], highlightbackground=c["edge"])
        w, h = max(self.winfo_width(), 200), max(self.winfo_height(), 120)
        x0, x1, y0, y1 = 56, w - 10, 26, h - 20
        mid = (y0 + y1) / 2
        span = a.plot_window
        now = a.plot_now()
        self.create_text(8, 4, anchor="nw", text=f"{self.title} ({self.unit})", fill=c["text"], font=a.f_head)
        for frac, lab in ((0, f"+{self.yr:g}"), (0.5, "0"), (1, f"-{self.yr:g}")):
            y = y0 + frac * (y1 - y0)
            self.create_line(x0, y, x1, y, fill=c["edge"] if frac != 0.5 else c["dim"],
                             dash=(2, 4) if frac != 0.5 else ())
            self.create_text(x0 - 6, y, anchor="e", text=lab, fill=c["dim"], font=a.f_ui)
        for k in range(1, 5):
            x = x1 - (x1 - x0) * k / 4
            self.create_line(x, y0, x, y1, fill=c["edge"], dash=(2, 4))
            self.create_text(x, y1 + 3, anchor="n", text=f"-{span * k / 4:g}s", fill=c["dim"], font=a.f_ui)
        hist = [(t, d) for t, d in a.plot_hist if now - t <= span]
        lx = x0 + 160
        for j, (name, key, k) in enumerate(zip(self.names, self.keys, self.scales)):
            col = SERIES[j % len(SERIES)]
            pts, clipped = [], []
            for t, d in hist:
                v = d.get(key)
                if v is None or v != v:
                    continue
                vc = max(-self.yr, min(self.yr, v * k))
                x, y = x1 - (now - t) / span * (x1 - x0), mid - vc / self.yr * (y1 - y0) / 2
                pts += [x, y]
                if vc != v * k:
                    clipped.append((x, y))
            if len(pts) >= 4:
                self.create_line(*pts, fill=col, width=2)
            for x, y in clipped:                               # off-scale samples: triangle on the rail
                dy = 6 if y <= mid else -6
                self.create_polygon(x - 4, y + dy, x + 4, y + dy, x, y, fill=col, outline="")
            cur = hist[-1][1].get(key) if hist else None
            txt = f"{name} {cur:+.1f}" if isinstance(cur, float) and cur == cur else f"{name} --"
            self.create_rectangle(lx, 8, lx + 10, 18, fill=col, outline="")
            self.create_text(lx + 14, 13, anchor="w", text=txt, fill=c["text"], font=a.f_ui)
            lx += 120
        if a.plot_frozen:
            self.create_text(x1, 4, anchor="ne", text="FROZEN (Space)", fill=c["warn"], font=a.f_head)


class App:
    def __init__(self, root: tk.Tk, args):
        self.root = root
        self.cfg = load_settings()
        self.q: queue.Queue = queue.Queue()
        self.themed: list = []          # widgets with retheme(c)
        self.c = THEMES[self.cfg.get("theme", "field")]
        self.busy: str | None = None
        self.status: dict | None = None
        self.status_t = 0.0
        self.frame_t = 0.0
        self.build = "unknown"
        self.reset_cause = ""
        self.stream_on = True
        self.link_state, self.link_detail = "waiting", "starting"
        self.recent_sent: collections.deque = collections.deque(maxlen=8)
        self.history: list[str] = list(self.cfg.get("history", []))[-200:]
        self.hist_pos: int | None = None
        self.elf = self.cfg.get("elf") or DEFAULT_ELF
        self.session = None
        self.rx_mark, self.rate_t = 0, time.monotonic()
        self.last_pull_dir: str | None = None
        self._showing_why = False
        self.plot_hist: collections.deque = collections.deque(maxlen=12000)
        self.plot_window = float(self.cfg.get("plot_window", 20))
        self.plot_frozen = False
        self.plot_freeze_t = 0.0
        self.fault_list: list[str] = []
        self.ev_phase: str | None = None

        self._fonts()
        self._build()
        self.apply_theme(self.cfg.get("theme", "field"))
        root.protocol("WM_DELETE_WINDOW", self.on_close)
        if self.cfg.get("session_log", True):
            self.start_session_log()

        if args.sim:
            import fc_sim
            finder, opener, self.sim = fc_sim.make(binary=args.sim_binary)
            self.board = link.Board(self._from_link, finder=finder, opener=opener)
        else:
            self.board = link.Board(self._from_link, port=args.port)
        self.note(f"{APP} - waiting for the board. F1 lists shortcuts; 'help' lists firmware commands.")
        root.after(40, self.pump)
        root.after(250, self.tick)

    # ------------------------------------------------------------------ fonts
    def _fonts(self) -> None:
        self.scale = float(self.cfg.get("scale", 1.0))
        ui = "Segoe UI" if sys.platform.startswith("win") else "DejaVu Sans"
        mono = "Consolas" if sys.platform.startswith("win") else "DejaVu Sans Mono"
        spec = {"f_ui": (ui, 10, "normal"), "f_btn": (ui, 10, "normal"), "f_cap": (ui, 8, "bold"),
                "f_tile": (ui, 14, "bold"), "f_mono": (mono, 10, "normal"), "f_big": (mono, 26, "bold"),
                "f_mid": (mono, 15, "bold"), "f_head": (ui, 9, "bold")}
        self.font_base = {}
        for name, (fam, size, weight) in spec.items():
            f = tkfont.Font(family=fam, size=round(size * self.scale), weight=weight)
            setattr(self, name, f)
            self.font_base[name] = (f, size)

    def zoom(self, step: float) -> None:
        self.scale = max(0.7, min(2.2, self.scale + step))
        for f, size in self.font_base.values():
            f.configure(size=max(6, round(size * self.scale)))
        self.cfg["scale"] = self.scale

    # ------------------------------------------------------------ construction
    def _build(self) -> None:
        r = self.root
        r.title(APP)
        with contextlib.suppress(tk.TclError):             # taskbar/title icon
            if sys.platform.startswith("win"):
                r.iconbitmap(default=os.path.join(HERE, "fc_console.ico"))
            else:
                self._icon = tk.PhotoImage(file=os.path.join(HERE, "fc_console.png"))
                r.iconphoto(True, self._icon)
        geo = self.cfg.get("geometry", "1280x800")
        try:                                    # never restore a window larger than this screen
            w, h = (int(v) for v in geo.split("+")[0].split("x"))
            if w > r.winfo_screenwidth() or h > r.winfo_screenheight():
                geo = "1280x800"
        except ValueError:
            geo = "1280x800"
        r.geometry(geo)
        if self.cfg.get("zoomed"):
            with contextlib.suppress(tk.TclError):
                r.state("zoomed")               # Windows: maximised
        r.minsize(980, 600)
        self.labels_: list = []
        self._menu()

        self.style = ttk.Style(r)
        with contextlib.suppress(tk.TclError):
            self.style.theme_use("clam")

        # Header: always visible, independent of the tab.
        head = tk.Frame(r)
        head.pack(fill="x", padx=8, pady=(8, 4))
        self.t_link = Tile(self, head, "USB link", 18)
        self.t_phase = Tile(self, head, "Phase", 8)
        self.t_mode = Tile(self, head, "Control", 6)
        self.t_fault = Tile(self, head, "Faults (active / latched)", 22)
        self.t_data = Tile(self, head, "Data", 9)
        self.t_build = Tile(self, head, "Firmware build", 14)
        for i, (t, w) in enumerate(((self.t_link, 3), (self.t_phase, 2), (self.t_mode, 2), (self.t_fault, 4),
                                    (self.t_data, 2), (self.t_build, 2))):
            t.grid(row=0, column=i, sticky="nsew", padx=(0, 6))
            head.columnconfigure(i, weight=w, uniform="tiles")
        self.frames_ = [head]

        body = tk.Frame(r)
        body.pack(side="bottom", fill="both", expand=True, padx=8, pady=(0, 4))
        self.frames_.append(body)
        side = tk.Frame(body, width=236)
        side.pack(side="left", fill="y", padx=(0, 8))
        side.pack_propagate(False)
        self.frames_.append(side)
        self.rail_outer = side
        rail = VScroll(side)
        self.themed.append(rail)
        self._sidebar(rail.body)
        rail.pack(fill="both", expand=True)          # after the op panel claims the bottom

        self.nb = ttk.Notebook(body)
        self.nb.pack(side="left", fill="both", expand=True)
        self.p_console = tk.Frame(self.nb)
        self.p_dash = tk.Frame(self.nb)
        self.p_plots = tk.Frame(self.nb)
        self.frames_ += [self.p_console, self.p_dash, self.p_plots]
        self.nb.add(self.p_console, text="  Console  (Ctrl+1)  ")
        self.nb.add(self.p_dash, text="  Dashboard  (Ctrl+2)  ")
        self.nb.add(self.p_plots, text="  Plots  (Ctrl+3)  ")
        self._console(self.p_console)
        self._dashboard(self.p_dash)
        self._plots(self.p_plots)
        self.nb.select(min(int(self.cfg.get("tab", 0)), 2))
        self.nb.bind("<<NotebookTabChanged>>", lambda _e: self._tab_changed())

        foot = tk.Frame(r)
        foot.pack(side="bottom", fill="x", padx=8, pady=(0, 6), before=body)   # packed first: never squeezed out
        self.frames_.append(foot)
        self.foot_l = tk.Label(foot, anchor="w", font=self.f_ui)
        self.foot_l.pack(side="left", fill="x", expand=True)
        self.foot_r = tk.Label(foot, anchor="e", font=self.f_ui)
        self.foot_r.pack(side="right")
        self.labels_ += [self.foot_l, self.foot_r]

        for i in (1, 2, 3):
            r.bind_all(f"<Control-Key-{i}>", lambda _e, k=i - 1: self.nb.select(k))
        r.bind_all("<F1>", lambda _e: self.show_keys())
        r.bind_all("<F2>", lambda _e: self.toggle_stream())
        r.bind_all("<F4>", lambda _e: self.on_build())
        r.bind_all("<F5>", lambda _e: self.on_flash())
        r.bind_all("<F6>", lambda _e: self.on_pull())
        r.bind_all("<F7>", lambda _e: self.apply_theme("bench" if self.c is THEMES["field"] else "field"))
        r.bind_all("<Control-l>", lambda _e: self.clear_term())
        r.bind_all("<Control-f>", lambda _e: self._focus_find())
        r.bind_all("<Control-plus>", lambda _e: self.zoom(0.1))
        r.bind_all("<Control-equal>", lambda _e: self.zoom(0.1))
        r.bind_all("<Control-minus>", lambda _e: self.zoom(-0.1))
        r.bind_all("<Escape>", lambda _e: self.entry.focus_set())
        r.bind_all("<F8>", lambda _e: self.on_clear_faults())
        r.bind_all("<space>", self._space)

    def _menu(self) -> None:
        m = tk.Menu(self.root)
        f = tk.Menu(m, tearoff=0)
        f.add_command(label="Open logs folder", command=lambda: open_folder(LOG_DIR))
        f.add_command(label="Save terminal as...", command=self.save_term)
        self.v_session = tk.BooleanVar(value=self.cfg.get("session_log", True))
        f.add_checkbutton(label="Record session log (logs/console/)", variable=self.v_session,
                          command=self._session_toggled)
        f.add_command(label="Decode a saved flightlog.bin...", command=self.decode_file)
        f.add_separator()
        f.add_command(label="Exit", command=self.on_close)
        m.add_cascade(label="File", menu=f)
        b = tk.Menu(m, tearoff=0)
        b.add_command(label="Flash firmware...", accelerator="F5", command=self.on_flash)
        b.add_command(label="Build firmware (headless CubeIDE)", accelerator="F4", command=self.on_build)
        b.add_command(label="Choose firmware ELF...", command=self.choose_elf)
        b.add_command(label="Locate STM32CubeIDE...", command=self.choose_ide)
        b.add_separator()
        b.add_command(label="Pull + decode flight log", accelerator="F6", command=self.on_pull)
        b.add_command(label="Erase flight log...", command=self.on_erase)
        b.add_separator()
        b.add_command(label="Pause / resume status stream", accelerator="F2", command=self.toggle_stream)
        b.add_command(label="Reboot board...", command=self.on_reboot)
        m.add_cascade(label="Board", menu=b)
        v = tk.Menu(m, tearoff=0)
        v.add_command(label="Field theme (light, sunlight)", accelerator="F7", command=lambda: self.apply_theme("field"))
        v.add_command(label="Bench theme (dark)", accelerator="F7", command=lambda: self.apply_theme("bench"))
        v.add_separator()
        v.add_command(label="Larger text", accelerator="Ctrl++", command=lambda: self.zoom(0.1))
        v.add_command(label="Smaller text", accelerator="Ctrl+-", command=lambda: self.zoom(-0.1))
        m.add_cascade(label="View", menu=v)
        h = tk.Menu(m, tearoff=0)
        h.add_command(label="Keyboard shortcuts", accelerator="F1", command=self.show_keys)
        h.add_command(label="Firmware command list (help)", command=lambda: self.send("help"))
        m.add_cascade(label="Help", menu=h)
        self.root.configure(menu=m)

    def _button(self, master, text, cmd, danger=False, primary=False) -> tk.Button:
        b = tk.Button(master, text=text, command=cmd, font=self.f_btn, relief="flat", bd=0,
                      padx=8, pady=6, anchor="w", cursor="hand2", highlightthickness=2)
        b.kind = "danger" if danger else ("primary" if primary else "plain")
        self.buttons.append(b)
        return b

    def _heading(self, master, text) -> None:
        lab = tk.Label(master, text=text.upper(), anchor="w", font=self.f_head)
        lab.pack(fill="x", pady=(10, 3))
        self.dim_labels.append(lab)

    def _sidebar(self, s) -> None:
        self.buttons: list[tk.Button] = []
        self.dim_labels: list[tk.Label] = []
        self._heading(s, "Status stream")
        self.b_stream = self._button(s, "Pause status   F2", self.toggle_stream)
        self.b_stream.pack(fill="x")

        self._heading(s, "Diagnostics")
        g = tk.Frame(s)
        g.pack(fill="x")
        self.frames_.append(g)
        self.query_btns = []
        for i, (label, cmds) in enumerate((("Faults", ("faults",)), ("Task timing", ("tasks",)),
                                           ("GPS diagnose", ("gps probe", "gps raw")), ("IMU", ("imu",)))):
            b = self._button(g, label, lambda c=cmds: self.send_many(c))
            b.grid(row=i // 2, column=i % 2, sticky="ew", padx=(0, 4) if i % 2 == 0 else 0, pady=(0, 4))
            self.query_btns.append(b)
        g.columnconfigure(0, weight=1)
        g.columnconfigure(1, weight=1)

        self._heading(s, "Flight log")
        g2 = tk.Frame(s)
        g2.pack(fill="x")
        self.frames_.append(g2)
        self.b_log_start = self._button(g2, "Start", lambda: self.send("log start"))
        self.b_log_stop = self._button(g2, "Stop", lambda: self.send("log stop"))
        self.b_log_start.grid(row=0, column=0, sticky="ew", padx=(0, 4), pady=(0, 4))
        self.b_log_stop.grid(row=0, column=1, sticky="ew", pady=(0, 4))
        g2.columnconfigure(0, weight=1)
        g2.columnconfigure(1, weight=1)
        self.b_pull = self._button(s, "Pull + decode   F6", self.on_pull, primary=True)
        self.b_pull.pack(fill="x", pady=(0, 4))
        self.b_open = self._button(s, "Open last pulled log", self.open_last_pull)
        self.b_open.pack(fill="x", pady=(0, 4))
        self.b_erase = self._button(s, "Erase log...", self.on_erase, danger=True)
        self.b_erase.pack(fill="x")

        self._heading(s, "Firmware")
        self.elf_l = tk.Label(s, anchor="w", justify="left", font=self.f_ui, wraplength=220)
        self.elf_l.pack(fill="x")
        self.dim_labels.append(self.elf_l)
        self.b_elf = self._button(s, "Choose ELF...", self.choose_elf)
        self.b_elf.pack(fill="x", pady=(3, 4))
        self.b_build = self._button(s, "Build (CubeIDE)   F4", self.on_build)
        self.b_build.pack(fill="x", pady=(0, 4))
        self.b_flash = self._button(s, "Flash firmware   F5", self.on_flash, primary=True)
        self.b_flash.pack(fill="x")

        self._heading(s, "Board")
        self.b_clear_faults = self._button(s, "Clear faults...   F8", self.on_clear_faults, danger=True)
        self.b_clear_faults.pack(fill="x", pady=(0, 4))
        self.b_reboot = self._button(s, "Reboot...", self.on_reboot, danger=True)
        self.b_reboot.pack(fill="x")

        # Operation progress, pinned to the bottom of the rail.
        op = tk.Frame(self.rail_outer)
        op.pack(side="bottom", fill="x", pady=(8, 0))
        self.frames_.append(op)
        self.op_l = tk.Label(op, text="", anchor="w", justify="left", font=self.f_ui, wraplength=220)
        self.op_l.pack(fill="x")
        self.labels_.append(self.op_l)
        self.prog = ttk.Progressbar(op, mode="determinate", maximum=100)
        self.prog.pack(fill="x", pady=(3, 0))
        self._update_elf_label()

    def _console(self, p) -> None:
        bar = tk.Frame(p)
        bar.pack(fill="x", padx=6, pady=(6, 2))
        self.frames_.append(bar)
        self.v_show_status = tk.BooleanVar(value=self.cfg.get("show_status", False))
        self.v_stamp = tk.BooleanVar(value=self.cfg.get("stamps", False))
        self.v_follow = tk.BooleanVar(value=True)
        self.checks = []
        for text, var in (("Status lines in terminal", self.v_show_status), ("Timestamps", self.v_stamp),
                          ("Follow output", self.v_follow)):
            cb = tk.Checkbutton(bar, text=text, variable=var, font=self.f_ui, padx=4)
            cb.pack(side="left", padx=(0, 10))
            self.checks.append(cb)
        self.b_clear = self._button(bar, "Clear  Ctrl+L", self.clear_term)
        self.b_clear.pack(side="right")
        self.find_var = tk.StringVar()
        self.find_e = tk.Entry(bar, textvariable=self.find_var, font=self.f_mono, width=18, relief="flat",
                               highlightthickness=1)
        self.find_e.pack(side="right", padx=(4, 8), ipady=3)
        self.find_e.bind("<Return>", lambda _e: self.find_next())
        self.find_e.bind("<KeyRelease>", lambda e: e.keysym != "Return" and self.find_all())
        fl = tk.Label(bar, text="Find  Ctrl+F", font=self.f_ui)
        fl.pack(side="right")
        self.dim_labels.append(fl)

        # The latest status line, updated in place instead of scrolling the terminal.
        self.pin = tk.Label(p, anchor="w", font=self.f_mono, padx=8, pady=3, text="status: (none yet)")
        self.pin.pack(fill="x", padx=6, pady=(2, 2))

        wrap = tk.Frame(p)
        wrap.pack(fill="both", expand=True, padx=6)
        self.frames_.append(wrap)
        self.term = tk.Text(wrap, font=self.f_mono, wrap="none", relief="flat", padx=8, pady=6,
                            undo=False, highlightthickness=1)
        ys = ttk.Scrollbar(wrap, orient="vertical", command=self.term.yview)
        xs = ttk.Scrollbar(wrap, orient="horizontal", command=self.term.xview)
        self.term.configure(yscrollcommand=ys.set, xscrollcommand=xs.set)
        ys.pack(side="right", fill="y")
        xs.pack(side="bottom", fill="x")
        self.term.pack(side="left", fill="both", expand=True)
        # Read-only but selectable/copyable: swallow edits, keep navigation.
        self.term.bind("<Key>", self._term_key)
        self.term.bind("<MouseWheel>", lambda _e: self.root.after(10, self._check_follow))
        self.term.bind("<Button-4>", lambda _e: self.root.after(10, self._check_follow))
        self.term.bind("<Button-5>", lambda _e: self.root.after(10, self._check_follow))
        ys.bind("<B1-Motion>", lambda _e: self.root.after(10, self._check_follow), add="+")

        row = tk.Frame(p)
        row.pack(fill="x", padx=6, pady=6)
        self.frames_.append(row)
        pr = tk.Label(row, text="Command ›", font=self.f_head)
        pr.pack(side="left", padx=(0, 6))
        self.dim_labels.append(pr)
        self.entry = tk.Entry(row, font=self.f_mono, relief="flat", highlightthickness=2)
        self.entry.pack(side="left", fill="x", expand=True, ipady=6)
        self.entry.bind("<Return>", self.on_enter)
        self.entry.bind("<Up>", lambda _e: self._hist(-1))
        self.entry.bind("<Down>", lambda _e: self._hist(+1))
        self.entry.bind("<Tab>", self._complete)
        self.b_send = self._button(row, "Send  Enter", self.on_enter, primary=True)
        self.b_send.pack(side="left", padx=(6, 0))
        self.hint = tk.Label(p, anchor="w", font=self.f_ui, padx=6,
                             text="Up/Down: history    Tab: complete    Esc: back to this box")
        self.hint.pack(fill="x", pady=(0, 4))
        self.dim_labels.append(self.hint)
        self.entry.focus_set()

    def _dashboard(self, p) -> None:
        self.dash = {}
        sc = VScroll(p)
        sc.pack(fill="both", expand=True)
        self.themed.append(sc)
        grid = tk.Frame(sc.body)
        grid.pack(fill="both", expand=True, padx=10, pady=10)
        self.frames_.append(grid)

        def group(r, c, title, rows, cs=1):
            fr = tk.Frame(grid, highlightthickness=1, padx=10, pady=6)
            fr.grid(row=r, column=c, columnspan=cs, sticky="nsew", padx=5, pady=5)
            self.panels_.append(fr)
            h = tk.Label(fr, text=title.upper(), font=self.f_head, anchor="w")
            h.grid(row=0, column=0, columnspan=6, sticky="w")
            self.panel_labels.append((h, "dim"))
            for i, (key, cap, unit, big) in enumerate(rows):
                cl = tk.Label(fr, text=cap, font=self.f_ui, anchor="w")
                cl.grid(row=1 + i, column=0, sticky="w", padx=(0, 10))
                vl = tk.Label(fr, text="--", font=self.f_big if big else self.f_mid, anchor="e", width=8)
                vl.grid(row=1 + i, column=1, sticky="e")
                ul = tk.Label(fr, text=unit, font=self.f_ui, anchor="w")
                ul.grid(row=1 + i, column=2, sticky="w", padx=(6, 0))
                self.panel_labels += [(cl, "dim"), (vl, "text"), (ul, "dim")]
                self.dash[key] = vl
            return fr

        self.panels_: list = []
        self.panel_labels: list = []
        group(0, 0, "Attitude (IEKF)", [("roll", "Roll", "deg", True), ("pitch", "Pitch", "deg", True),
                                         ("yaw", "Yaw", "deg", True)])
        group(0, 1, "Body rates", [("wx", "x", "deg/s", False), ("wy", "y", "deg/s", False),
                                   ("wz", "z", "deg/s", False), ("an", "|a|", "g", False)])
        fins = group(0, 2, "Fin commands", [])
        self.fin_canvas = tk.Canvas(fins, width=280, height=150, highlightthickness=0)
        self.fin_canvas.grid(row=1, column=0, columnspan=3, sticky="nsew")
        group(1, 0, "Loop / IMU", [("loop", "Loop time", "us", False), ("ovr", "Overruns", "", False),
                                   ("imu", "IMU", "", False), ("imu_fails", "IMU fails", "", False)])
        group(1, 1, "GPS", [("gps", "State", "", False), ("sats", "Satellites", "", False),
                            ("alt", "Alt MSL", "m", False), ("gps_rx", "Bytes rx", "", False)])
        group(1, 2, "Flight log", [("log", "Used", "%", False)])
        ff = tk.Frame(grid, highlightthickness=1, padx=10, pady=6)
        ff.grid(row=2, column=0, columnspan=3, sticky="nsew", padx=5, pady=5)
        self.panels_.append(ff)
        h = tk.Label(ff, text="FDIR MONITORS", font=self.f_head, anchor="w")
        h.grid(row=0, column=0, columnspan=8, sticky="w")
        self.panel_labels.append((h, "dim"))
        self.mon_cells = []
        for i, n in enumerate(link.MONITORS):
            cell = Tile(self, ff, n, 7)
            cell.grid(row=1, column=i, padx=3, pady=(4, 0), sticky="ew")
            ff.columnconfigure(i, weight=1)
            self.mon_cells.append(cell)
        ev = tk.Frame(grid, highlightthickness=1, padx=10, pady=6)
        ev.grid(row=3, column=0, columnspan=3, sticky="nsew", padx=5, pady=5)
        self.panels_.append(ev)
        h = tk.Label(ev, text="EVENTS  (phase changes, trips, log; newest last)", font=self.f_head, anchor="w")
        h.pack(fill="x")
        self.panel_labels.append((h, "dim"))
        self._events(ev)
        for c in range(3):
            grid.columnconfigure(c, weight=1)
        self.dash_note = tk.Label(p, anchor="w", font=self.f_ui, padx=12,
                                  text="Values come from the 5 Hz status line (or 0xAA55 frames). Grey = stale.")
        self.dash_note.pack(fill="x", pady=(0, 6))
        self.dim_labels.append(self.dash_note)

    def _events(self, p) -> None:
        cols = ("pc", "board", "phase", "text")
        wrap = tk.Frame(p)
        wrap.pack(fill="both", expand=True, pady=(4, 0))
        self.frames_.append(wrap)
        self.ev = ttk.Treeview(wrap, columns=cols, show="headings", style="Ev.Treeview", height=7)
        for col, title, w in (("pc", "PC time", 100), ("board", "Board t (s)", 100), ("phase", "Phase", 90),
                              ("text", "Event", 500)):
            self.ev.heading(col, text=title, anchor="w")
            self.ev.column(col, width=w, anchor="w", stretch=(col == "text"))
        ys = ttk.Scrollbar(wrap, orient="vertical", command=self.ev.yview)
        self.ev.configure(yscrollcommand=ys.set)
        ys.pack(side="right", fill="y")
        self.ev.pack(fill="both", expand=True)

    def _plots(self, p) -> None:
        bar = tk.Frame(p)
        bar.pack(fill="x", padx=8, pady=6)
        self.frames_.append(bar)
        l = tk.Label(bar, text="Window", font=self.f_ui)
        l.pack(side="left")
        self.dim_labels.append(l)
        self.plot_btns = {}
        for sec in (10, 20, 60):
            b = self._button(bar, f"{sec} s", lambda v=sec: self._set_window(v))
            b.pack(side="left", padx=(6, 0))
            self.plot_btns[sec] = b
        self.b_freeze = self._button(bar, "Freeze   Space", self.toggle_freeze)
        self.b_freeze.pack(side="left", padx=(16, 0))
        note = tk.Label(bar, font=self.f_ui, text="Fixed scales; off-scale samples marked with a triangle on the rail.")
        note.pack(side="left", padx=(16, 0))
        self.dim_labels.append(note)
        self.charts = [
            StripChart(self, p, "Attitude", "deg; yaw drawn at half scale", ("roll", "pitch", "yaw"),
                       ("roll", "pitch", "yaw"), 90.0, (1.0, 1.0, 0.5)),
            StripChart(self, p, "Body rates", "deg/s; click to rescale", ("x", "y", "z"), ("wx", "wy", "wz"),
                       500.0, ranges=(100.0, 500.0, 2000.0)),
            StripChart(self, p, "Fin commands", "x100 %", ("0 top", "1 right", "2 bottom", "3 left"),
                       ("f0", "f1", "f2", "f3"), 100.0),
            StripChart(self, p, "Nav (NAV-01)", "h m, v_up m/s, q kPa drawn x10; click to rescale",
                       ("h", "v_up", "q kPa"), ("h", "vz", "qbar"), 1500.0, (1.0, 1.0, 10.0),
                       ranges=(50.0, 300.0, 1500.0)),
        ]
        for ch in self.charts:
            ch.pack(fill="both", expand=True, padx=8, pady=(0, 6))

    def _set_window(self, sec: float) -> None:
        self.plot_window = self.cfg["plot_window"] = float(sec)
        self._style_plot_btns()
        self.draw_plots()

    def _style_plot_btns(self) -> None:
        for sec, b in self.plot_btns.items():
            on = abs(sec - self.plot_window) < 0.5
            b.configure(relief="sunken" if on else "flat", bg=self.c["sel"] if on else self.c["btn"])

    def toggle_freeze(self) -> None:
        self.plot_frozen = not self.plot_frozen
        self.plot_freeze_t = time.monotonic()
        self.b_freeze.configure(text="Resume   Space" if self.plot_frozen else "Freeze   Space")
        self.draw_plots()

    def _space(self, e):
        if self.nb.index("current") == 2 and e.widget not in (self.entry, self.find_e):
            self.toggle_freeze()
            return "break"
        return None

    def plot_now(self) -> float:
        return self.plot_freeze_t if self.plot_frozen else time.monotonic()

    def _plot_add(self, st: dict) -> None:
        if not self.plot_frozen:
            d = {k: st.get(k) for k in ("roll", "pitch", "yaw", "wx", "wy", "wz", "h", "vz")}
            q = st.get("qbar")
            d["qbar"] = q / 1000.0 if isinstance(q, float) else None
            for i in range(4):
                f = st.get(f"f{i}")
                d[f"f{i}"] = f * 100 if isinstance(f, float) else None
            self.plot_hist.append((time.monotonic(), d))

    def draw_plots(self) -> None:
        if self.nb.index("current") == 2:
            for ch in self.charts:
                ch.draw()

    # ------------------------------------------------------------------ theme
    def apply_theme(self, name: str) -> None:
        c = self.c = THEMES[name]
        self.cfg["theme"] = name
        self.root.configure(bg=c["bg"])
        for f in self.frames_:
            f.configure(bg=c["bg"])
        for b in self.buttons:
            self._style_button(b)
        for lab in self.dim_labels:
            lab.configure(bg=c["bg"], fg=c["dim"])
        for lab in self.labels_:
            lab.configure(bg=c["bg"], fg=c["text"])
        for cb in self.checks:
            cb.configure(bg=c["bg"], fg=c["text"], activebackground=c["bg"], activeforeground=c["text"],
                         selectcolor=c["term"])
        for fr in self.panels_:
            fr.configure(bg=c["panel"], highlightbackground=c["edge"])
        for lab, role in self.panel_labels:
            lab.configure(bg=c["panel"], fg=c[role])
        self.fin_canvas.configure(bg=c["panel"])
        self.pin.configure(bg=c["panel"], fg=c["text"])
        self.term.configure(bg=c["term"], fg=c["text"], insertbackground=c["text"], selectbackground=c["sel"],
                            selectforeground=c["text"], highlightbackground=c["edge"], highlightcolor=c["info"])
        for e in (self.entry, self.find_e):
            e.configure(bg=c["term"], fg=c["text"], insertbackground=c["text"], highlightbackground=c["edge"],
                        highlightcolor=c["info"], selectbackground=c["sel"], selectforeground=c["text"])
        for tag, role in (("sent", "sent"), ("evt", "info"), ("err", "bad"), ("ok", "ok"), ("boot", "dim"),
                          ("status", "dim"), ("note", "warn"), ("stamp", "dim")):
            self.term.tag_configure(tag, foreground=c[role])
        self.term.tag_configure("find", background=c["warnbg"])
        self.term.tag_configure("findcur", background=c["warn"], foreground=c["bg"])
        st = self.style
        st.configure(".", background=c["bg"], foreground=c["text"], fieldbackground=c["term"],
                     bordercolor=c["edge"], troughcolor=c["panel"], font=self.f_ui)
        st.configure("TNotebook", background=c["bg"], borderwidth=0)
        st.configure("TNotebook.Tab", background=c["panel"], foreground=c["dim"], padding=(10, 6), font=self.f_head)
        st.map("TNotebook.Tab", background=[("selected", c["bg"])], foreground=[("selected", c["text"])])
        st.configure("TProgressbar", background=c["info"], troughcolor=c["panel"], bordercolor=c["edge"])
        st.configure("Ev.Treeview", background=c["term"], fieldbackground=c["term"], foreground=c["text"],
                     font=self.f_mono, rowheight=int(self.f_mono.metrics("linespace") * 1.5))
        st.configure("Ev.Treeview.Heading", background=c["panel"], foreground=c["text"], font=self.f_head)
        st.map("Ev.Treeview", background=[("selected", c["sel"])], foreground=[("selected", c["text"])])
        st.configure("TScrollbar", background=c["btn"], troughcolor=c["panel"], arrowcolor=c["text"])
        self.ev.tag_configure("bad", foreground=c["bad"])
        self.ev.tag_configure("ok", foreground=c["ok"])
        self.ev.tag_configure("warn", foreground=c["warn"])
        for t in self.themed:
            t.retheme(c)
        self.draw_fins()
        if hasattr(self, "plot_btns"):
            self._style_plot_btns()
            self.draw_plots()
        self.refresh_controls()

    def _style_button(self, b: tk.Button) -> None:
        c = self.c
        fg = {"danger": c["bad"], "primary": c["info"]}.get(b.kind, c["text"])
        b.configure(bg=c["btn"], fg=fg, activebackground=c["btnhot"], activeforeground=fg,
                    disabledforeground=c["off"], highlightbackground=c["bg"], highlightcolor=c["info"])
        if b.kind == "primary":
            b.configure(font=self.f_head)

    # ------------------------------------------------------------- link input
    def _from_link(self, kind, *a) -> None:
        """Link thread: never touch Tk here."""
        self.q.put((kind, a))

    def pump(self) -> None:
        t_end = time.monotonic() + 0.03
        lines = []
        try:
            while time.monotonic() < t_end:
                kind, a = self.q.get_nowait()
                if kind == "line":
                    lines.append(a[0])
                    if len(lines) >= 200:
                        self._lines(lines)
                        lines = []
                elif kind == "frame":
                    if a[0] == link.MSG_STATE and len(a[1]) == link.STATE.size:
                        self._frame(link.decode_state(a[1]))
                elif kind == "link":
                    self._link(*a)
                elif kind == "capture":
                    pass                                        # consumed by the worker
                elif kind == "ui":
                    if lines:                                   # keep board/console order
                        self._lines(lines)
                        lines = []
                    a[0]()
        except queue.Empty:
            pass
        if lines:
            self._lines(lines)
        self.root.after(40, self.pump)

    def ui(self, fn) -> None:
        """Run fn on the UI thread (from a worker)."""
        self.q.put(("ui", (fn,)))

    def _link(self, state: str, detail: str) -> None:
        self.link_state, self.link_detail = state, detail
        if state == "connected":
            self.note(f"connected on {detail}", "ok")
            self.stream_on = True
            self.root.after(600, lambda: self.busy is None and self.board.connected.is_set()
                            and self.tx("ver"))
        elif state == "waiting" and detail == "board disconnected":
            self.note("board disconnected (reset, reflash or cable) - reconnecting automatically", "note")
            self.status = None
            self.fault_list = []
        elif state == "busy":
            self.note(f"{detail}. Close Tera Term / the CubeIDE serial console; retrying.", "err")
        self.refresh_controls()

    def _lines(self, lines: list[str]) -> None:
        out = []
        now = time.time()
        for s in lines:
            if self.session:
                with contextlib.suppress(OSError, ValueError):
                    self.session.write(time.strftime("%H:%M:%S", time.localtime(now)) + f".{int(now * 1000) % 1000:03d} {s}\n")
            st = link.parse_status(s)
            if st:
                self.status, self.status_t = st, time.monotonic()
                self._plot_add(st)
                self.stream_on = True
                self.pin.configure(text="status: " + s[s.find("t="):])
                if self.v_show_status.get():
                    out.append((s, "status"))
                continue
            ev = link.parse_event(s)
            if ev:
                self._event(ev)
                out.append((s, "evt"))
                continue
            b = link.parse_build(s)
            if b:
                self.build = b
                i = s.find("reset:")
                self.reset_cause = s[i + 6:].strip(" =,") if i >= 0 else ""
                if "last reset" in s:
                    self.reset_cause = s.rsplit("last reset:", 1)[1].strip()
            if s in self.recent_sent:                          # the firmware's echo of a command
                self.recent_sent.remove(s)
                out.append(("\u203a " + s, "sent"))
                continue
            if s in ("stream off", "stream on"):                # its reply
                self.stream_on = s == "stream on"
            if ERR_RE.search(s):
                tag = "err"
            elif s.startswith("[BOOT]") or s.startswith("==="):
                tag = "boot"
            elif s.endswith(": OK") or s.startswith(("FWCRC ok", "FWVEC ok", "FWPROG")):
                tag = "ok"
            else:
                tag = ""
            out.append((s, tag))
        if out:
            self._write(out)
        self.refresh_header()

    def _frame(self, f: dict) -> None:
        """Binary STATE frame (CFG_TELEM_BINARY=1): feed the dashboard."""
        self.frame_t = time.monotonic()
        h = f["health"]
        st = dict(self.status or {})
        st.update(roll=f["roll"], pitch=f["pitch"], yaw=f["yaw"], wx=f["wx"], wy=f["wy"], wz=f["wz"],
                  an=(f["ax"] ** 2 + f["ay"] ** 2 + f["az"] ** 2) ** 0.5 / 9.80665,
                  f0=f["f0"], f1=f["f1"], f2=f["f2"], f3=f["f3"], loop=f["loop_us"],
                  imu="OK" if h & 3 == 3 else "FAIL", mode=("OFF", "RDAMP", "FULL", "?")[(h >> 10) & 3])
        # Frames carry no phase or per-monitor bits: phase comes from the last [EVT]
        # transition, faults only as "any active" (type 'faults' for detail).
        st["phase"] = "LOCKOUT" if h & (1 << 9) else (self.ev_phase or "?")
        st["active"] = st["latched"] = None
        st["fault_any"] = bool(h & (1 << 8))
        self.status, self.status_t = st, self.frame_t
        if f["frame"] % 4 == 0:                                   # 25 Hz is plenty for a strip plot
            self._plot_add(st)

    def _event(self, ev: dict) -> None:
        to = ev["text"].rsplit("->", 1)[-1].strip()
        if "->" in ev["text"] and to in link.PHASES:
            self.ev_phase = to
        up = ev["text"].upper()
        bad = any(w in up for w in ("TRIP", "LOCKOUT", "CATO", "FAIL", "CORRUPT", "WATCHDOG"))
        advisory = bad and any(n in ev["text"] for n in link.ADVISORY) and "LOCKOUT" not in up
        tag = "warn" if advisory else ("bad" if bad else ("ok" if ev["text"].endswith("READY") else ""))
        iid = self.ev.insert("", "end", values=(time.strftime("%H:%M:%S"), f"{ev['ms'] / 1000:.2f}", ev["phase"],
                                                ev["text"]), tags=(tag,))
        self.ev.see(iid)
        kids = self.ev.get_children()
        if len(kids) > 500:
            self.ev.delete(*kids[:len(kids) - 500])

    def _tab_changed(self) -> None:
        self.draw_plots()

    # ---------------------------------------------------------------- terminal
    def _write(self, items: list[tuple[str, str]]) -> None:
        t = self.term
        follow = self.v_follow.get()
        stamp = self.v_stamp.get()
        ts = time.strftime("%H:%M:%S ") if stamp else ""
        for s, tag in items:
            if ts:
                t.insert("end", ts, "stamp")
            t.insert("end", s + "\n", tag)
        n = int(t.index("end-1c").split(".")[0])
        if n > 20000:
            t.delete("1.0", f"{n - 15000}.0")
        if follow:
            t.see("end")

    def note(self, text: str, tag: str = "note") -> None:
        """Console-generated line (never confused with board output: '»' prefix)."""
        self._write([("» " + text, tag)])
        if self.session:
            with contextlib.suppress(OSError, ValueError):
                self.session.write(time.strftime("%H:%M:%S") + f" [console] {text}\n")

    def _term_key(self, e):
        if (e.state & 0x4) and e.keysym.lower() in ("c", "a"):
            if e.keysym.lower() == "a":
                self.term.tag_add("sel", "1.0", "end")
                return "break"
            return None
        if e.keysym in ("Up", "Down", "Prior", "Next", "Home", "End", "Left", "Right"):
            self.root.after(10, self._check_follow)
            return None
        if len(e.char) == 1 and e.char.isprintable():           # typing in the terminal goes to the entry
            self.entry.focus_set()
            self.entry.insert("end", e.char)
        return "break"

    def _check_follow(self) -> None:
        """Scrolling up pauses following; scrolling back to the bottom resumes it."""
        self.v_follow.set(self.term.yview()[1] >= 0.999)

    def clear_term(self) -> None:
        self.term.delete("1.0", "end")

    def save_term(self) -> None:
        p = filedialog.asksaveasfilename(defaultextension=".txt", initialdir=LOG_DIR,
                                         initialfile=time.strftime("fc_terminal_%Y%m%d-%H%M%S.txt"))
        if p:
            with open(p, "w", encoding="utf-8") as fh:
                fh.write(self.term.get("1.0", "end"))
            self.note(f"terminal saved to {p}", "ok")

    def _focus_find(self) -> None:
        self.nb.select(0)
        self.find_e.focus_set()
        self.find_e.select_range(0, "end")

    def find_all(self) -> None:
        t = self.term
        t.tag_remove("find", "1.0", "end")
        t.tag_remove("findcur", "1.0", "end")
        needle = self.find_var.get()
        if len(needle) < 2:
            return
        idx, n = "1.0", 0
        while n < 2000:
            idx = t.search(needle, idx, nocase=True, stopindex="end")
            if not idx:
                break
            end = f"{idx}+{len(needle)}c"
            t.tag_add("find", idx, end)
            idx, n = end, n + 1
        self.foot_l.configure(text=f"find: {n} match{'es' if n != 1 else ''} for '{needle}'")

    def find_next(self) -> None:
        t = self.term
        needle = self.find_var.get()
        if len(needle) < 2:
            return
        r = t.tag_ranges("findcur")
        start = str(r[1]) if r else "1.0"
        idx = t.search(needle, start, nocase=True, stopindex="end") or t.search(needle, "1.0", nocase=True,
                                                                                 stopindex="end")
        t.tag_remove("findcur", "1.0", "end")
        if idx:
            t.tag_add("findcur", idx, f"{idx}+{len(needle)}c")
            self.v_follow.set(False)
            t.see(idx)

    # ------------------------------------------------------------------ input
    def tx(self, cmd: str) -> bool:
        """Every command goes through here so its echo can be recognised (any thread)."""
        self.recent_sent.append(cmd)
        return self.board.send_line(cmd)

    def send(self, cmd: str) -> bool:
        if self.busy:
            self.note(f"busy ({self.busy}) - wait for it to finish", "err")
            return False
        if not self.board.connected.is_set():
            self.note("not connected - command not sent", "err")
            return False
        head = cmd.split()[:2]
        if head[:1] == ["fw"]:
            self.note("use Flash firmware (F5): 'fw' expects a raw image right after it", "err")
            return False
        if head == ["log", "pull"]:
            self.on_pull()                                      # raw binary: route to the decoder
            return True
        if head == ["log", "erase"]:
            self.on_erase()
            return True
        if head[:1] in (["reboot"], ["inject"]) and not self._confirm_board_action(cmd):
            return False
        if cmd.startswith("stream"):
            self.stream_on = not cmd.endswith("off")
        self.tx(cmd)
        self.refresh_controls()
        return True

    def _confirm_board_action(self, cmd: str) -> bool:
        return messagebox.askokcancel(APP, f"Send '{cmd}'?\n\nThis resets the board or forces a fault "
                                           f"(LOCKOUT, fins centred).", icon="warning", default="cancel")

    def on_enter(self, _e=None):
        cmd = self.entry.get().strip()
        if not cmd:
            return "break"
        if not self.history or self.history[-1] != cmd:
            self.history.append(cmd)
        self.hist_pos = None
        if self.send(cmd):
            self.entry.delete(0, "end")
        return "break"

    def _hist(self, d: int):
        if not self.history:
            return "break"
        if self.hist_pos is None:
            self.hist_pos = len(self.history)
        self.hist_pos = max(0, min(len(self.history), self.hist_pos + d))
        self.entry.delete(0, "end")
        if self.hist_pos < len(self.history):
            self.entry.insert(0, self.history[self.hist_pos])
        return "break"

    def _complete(self, _e):
        cur = self.entry.get()
        hits = [c for c in link.COMMANDS if c.startswith(cur)] if cur else []
        if len(hits) == 1:
            self.entry.delete(0, "end")
            self.entry.insert(0, hits[0])
        elif hits:
            common = os.path.commonprefix(hits)
            self.entry.delete(0, "end")
            self.entry.insert(0, common)
            self.foot_l.configure(text="  ".join(hits))
        return "break"

    def toggle_stream(self) -> None:
        self.send("stream off" if self.stream_on else "stream on")

    # ----------------------------------------------------------- state display
    def tick(self) -> None:
        self.refresh_header()
        self.refresh_dashboard()
        self.refresh_controls()
        self.draw_plots()
        now = time.monotonic()
        if now - self.rate_t >= 1.0:
            rate = (self.board.rx_bytes - self.rx_mark) / (now - self.rate_t)
            self.rx_mark, self.rate_t = self.board.rx_bytes, now
            sess = os.path.relpath(self.session.name, HERE) if self.session else "session log off"
            self.foot_r.configure(text=f"rx {rate:6.0f} B/s   total rx {self.board.rx_bytes:,} B  "
                                       f"tx {self.board.tx_bytes:,} B   {sess}")
        self.root.after(250, self.tick)

    def _age(self) -> float:
        return time.monotonic() - self.status_t if self.status else float("inf")

    def phase(self) -> str | None:
        return self.status["phase"] if self.status and self._age() < 10 else None

    def refresh_header(self) -> None:
        st, age = self.status, self._age()
        conn = self.link_state == "connected"
        if conn:
            self.t_link.set(f"CONNECTED {self.link_detail}", "ok")
        elif self.link_state == "busy":
            self.t_link.set("PORT IN USE", "bad")
        else:
            self.t_link.set("NO BOARD", "warn")
        if st and conn:
            ph = st["phase"]
            self.t_phase.set(ph, PHASE_STYLE.get(ph, "warn"))
            self.t_mode.set(st["mode"], {"FULL": "ok", "RDAMP": "warn"}.get(st["mode"], "idle"))
            a, l = st.get("active", 0), st.get("latched", 0)
            if a is None:                                       # binary frames: summary bit only
                self.fault_list = []
                self.t_fault.set("FAULT (see Faults)" if st.get("fault_any") else "none",
                                 "bad" if st.get("fault_any") else "ok")
            else:
                text, style, self.fault_list = link.fault_summary(a, l, ph)
                if age >= STALE_S:                              # do not present old faults as current
                    text, style = f"{text}  (stale)", "idle"
                self.t_fault.set(text, style)
        else:
            for t in (self.t_phase, self.t_mode, self.t_fault):
                t.set("--", "idle")
        if not conn:
            self.t_data.set("--", "idle")
        elif not self.stream_on:
            self.t_data.set("PAUSED", "warn")
        elif age < STALE_S:
            self.t_data.set(f"live {age:.1f}s", "ok")
        elif st:
            self.t_data.set(f"STALE {min(age, 999):.0f}s", "bad")
        else:
            self.t_data.set("no status", "warn")
        self.t_build.set(self.build if conn else "--", "idle")
        self.b_stream.configure(text="Resume status   F2" if self.stream_on is False else "Pause status   F2")

    def refresh_dashboard(self) -> None:
        if self.nb.index("current") != 1:
            return
        st = self.status
        fresh = st is not None and self._age() < STALE_S and self.link_state == "connected"
        c = self.c
        fg = c["text"] if fresh else c["edge"]

        def fmt(v, dp=1):
            if isinstance(v, str):
                return v
            if v != v:
                return "nan"
            return f"{v:+.{dp}f}" if dp else f"{v:.0f}"
        for k, lab in self.dash.items():
            v = st.get(k) if st else None
            txt = "--" if v is None else fmt(v, 2 if k == "an" else (0 if k in ("loop", "ovr", "sats", "gps_rx",
                                                                                 "imu_fails", "log") else 1))
            colour = fg
            if fresh and k == "imu" and v != "OK":
                colour = c["bad"]
            if fresh and k == "ovr" and v:
                colour = c["warn"]
            lab.configure(text=txt, fg=colour)
        a = st.get("active", 0) if st else 0
        l = st.get("latched", 0) if st else 0
        for i, cell in enumerate(self.mon_cells):
            if not fresh or a is None:
                cell.set("--", "idle")
            elif (l >> i) & 1:
                cell.set("FAIL" if (a >> i) & 1 else "latched", "bad")
            elif (a >> i) & 1:
                sev = link.SEV[link.MONITORS[i]][1 if st.get("phase") in link.IN_FLIGHT else 0]
                cell.set("FAIL" if sev != "INFO" else "info", {"HARD": "bad", "SOFT": "warn"}.get(sev, "info"))
            else:
                cell.set("ok", "ok")
        self.draw_fins()

    def draw_fins(self) -> None:
        cv = self.fin_canvas
        cv.delete("all")
        c = self.c
        st = self.status
        fresh = st is not None and self._age() < STALE_S
        w = max(int(cv.winfo_width()), 280)
        names = ("0 top", "1 right", "2 bottom", "3 left")
        x0, x1 = 80, w - 70
        mid = (x0 + x1) / 2
        for i, n in enumerate(names):
            y = 14 + i * 34
            v = st.get(f"f{i}", 0.0) if st else 0.0
            v = 0.0 if v != v else max(-1.0, min(1.0, v))
            cv.create_text(4, y + 9, text=n, anchor="w", fill=c["dim"], font=self.f_ui)
            cv.create_rectangle(x0, y, x1, y + 18, outline=c["edge"], fill=c["term"])
            cv.create_line(mid, y - 3, mid, y + 21, fill=c["dim"])
            if fresh:
                xe = mid + v * (x1 - mid)
                cv.create_rectangle(min(mid, xe), y + 3, max(mid, xe), y + 15, outline="",
                                    fill=c["warn"] if abs(v) > 0.95 else c["info"])
            cv.create_text(w - 4, y + 9, text=f"{v:+.2f}" if fresh else "--", anchor="e",
                           fill=c["text"] if fresh else c["edge"], font=self.f_mono)

    def refresh_controls(self) -> None:
        if not hasattr(self, "board"):
            return
        conn = self.board.connected.is_set()
        ph = self.phase()
        flying = ph in link.IN_FLIGHT
        idle = self.busy is None
        ok = conn and idle
        for b in self.query_btns + [self.b_stream, self.b_log_start, self.b_log_stop, self.b_send]:
            b.configure(state="normal" if ok else "disabled")
        risky = ok and not flying
        for b in (self.b_pull, self.b_erase, self.b_flash, self.b_reboot):
            b.configure(state="normal" if risky else "disabled")
        clearable = ok and ph not in link.IN_FLIGHT and ph != "CATO"
        self.b_clear_faults.configure(state="normal" if clearable else "disabled")
        self.b_open.configure(state="normal" if self.last_pull_dir else "disabled")
        self.b_elf.configure(state="normal" if idle else "disabled")
        self.b_build.configure(state="normal" if idle else "disabled")
        if self.busy:
            why = f"Busy: {self.busy}"
        elif not conn:
            why = "Board actions need a connected board."
        elif flying:
            why = f"In flight ({ph}): flash, pull, erase and reboot are locked out."
        else:
            why = ""
        if not self.busy:                    # a lock-out reason shows instead of the last result
            if why:
                if self.op_l.cget("text") != why:
                    self.op_l.configure(text=why, fg=self.c["warn"])
                self._showing_why = True
            elif self._showing_why:
                self.op_l.configure(text="")
                self._showing_why = False

    # -------------------------------------------------------------- operations
    def _start(self, name: str, fn, *args) -> None:
        self.busy = name
        self.prog.configure(mode="determinate", value=0)
        self.op_l.configure(text=f"{name}...", fg=self.c["info"])
        self.refresh_controls()

        def run():
            try:
                fn(*args)
            except SystemExit:
                self.ui(lambda: self._op_msg("failed (see terminal)", "err"))
            except Exception as exc:                            # noqa: BLE001
                self.ui(lambda e=exc: self._op_msg(f"error: {e}", "err"))
            finally:
                self.board.proto_end()
                self.ui(self._done)
        threading.Thread(target=run, name=name, daemon=True).start()

    def _done(self) -> None:
        self.busy = None
        self.prog.stop()
        self.prog.configure(mode="determinate")
        self.refresh_controls()

    def _op_msg(self, text: str, tag: str = "note") -> None:
        self.note(text, tag)
        self.op_l.configure(text=text, fg={"err": self.c["bad"], "ok": self.c["ok"]}.get(tag, self.c["info"]))

    def _progress(self, pct: float, text: str | None = None) -> None:
        def f():
            self.prog.configure(value=pct)
            if text:
                self.op_l.configure(text=text, fg=self.c["info"])
        self.ui(f)

    def _quiet(self) -> bool:
        """Pause the stream so a protocol exchange is not interleaved. Returns prior state."""
        was = self.stream_on
        self.tx("stream off")
        time.sleep(0.3)
        return was

    # flash -------------------------------------------------------------------
    def _update_elf_label(self) -> None:
        if os.path.isfile(self.elf):
            age = (time.time() - os.path.getmtime(self.elf)) / 60
            self.elf_l.configure(text=f"{os.path.basename(self.elf)}\nbuilt {age:.0f} min ago")
        else:
            self.elf_l.configure(text=f"{os.path.basename(self.elf)}\nNOT FOUND - build first or choose")

    def choose_ide(self) -> str:
        p = filedialog.askopenfilename(title="stm32cubeidec.exe (in the STM32CubeIDE install folder)",
                                       filetypes=[("CubeIDE console launcher", "stm32cubeidec*"), ("All", "*.*")])
        if p:
            self.cfg["ide"] = p
        return p

    def on_build(self) -> None:
        if self.busy:
            return
        ide = find_cubeide(self.cfg.get("ide", ""))
        if not ide:
            messagebox.showinfo(APP, "STM32CubeIDE was not found. Point me at stm32cubeidec.exe "
                                     "(STM32CubeIDE install folder).")
            ide = self.choose_ide()
            if not ide:
                return
        self._start("building", self._build_fw, ide)
        self.ui(lambda: (self.prog.configure(mode="indeterminate"), self.prog.start(15)))

    def _build_fw(self, ide: str) -> None:
        """Headless CDT build of the Debug config in a scratch workspace, so it
        works whether or not the CubeIDE GUI is open (its workspace stays locked)."""
        import tempfile
        ws = os.path.join(tempfile.gettempdir(), "fcv4_headless_ws")
        cmd = [ide, "--launcher.suppressErrors", "-nosplash", "-application",
               "org.eclipse.cdt.managedbuilder.core.headlessbuild", "-data", ws,
               "-import", FW_ROOT, "-build", "Flight Controller V4/Debug"]
        self.ui(lambda: self.note("build: headless CubeIDE (Debug), ~20 s"))
        t0 = time.monotonic()
        flags = 0x08000000 if sys.platform.startswith("win") else 0         # CREATE_NO_WINDOW
        p = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, stdin=subprocess.DEVNULL,
                             text=True, errors="replace", creationflags=flags)
        result, size_next = None, False
        for line in p.stdout:
            line = line.rstrip()
            show = size_next or bool(re.search(r"\b(error|warning):", line)) or "Build Finished" in line
            size_next = line.lstrip().startswith("text")                # the arm-none-eabi-size table
            if show or size_next:
                self.ui(lambda l=line: self._write([(l, "err" if " error:" in l else "")]))
            m = re.search(r"Build Finished\. (\d+) errors?, (\d+) warnings?", line)
            if m:
                result = (int(m.group(1)), int(m.group(2)))
        p.wait()
        dt = time.monotonic() - t0
        if result and result[0] == 0:
            self.ui(lambda: (self._update_elf_label(),
                             self._op_msg(f"build OK in {dt:.0f} s ({result[1]} warnings) - F5 to flash", "ok")))
        else:
            self.ui(lambda: self._op_msg(f"build FAILED (exit {p.returncode}) - see terminal", "err"))

    def choose_elf(self) -> None:
        p = filedialog.askopenfilename(title="Firmware ELF", initialdir=os.path.dirname(self.elf),
                                       filetypes=[("ELF", "*.elf"), ("All", "*.*")])
        if p:
            self.elf = self.cfg["elf"] = p
            self._update_elf_label()

    def on_flash(self) -> None:
        if self.b_flash.cget("state") == "disabled":
            return
        import fc_flash
        self._update_elf_label()
        if not os.path.isfile(self.elf):
            messagebox.showerror(APP, f"No firmware image at\n{self.elf}\n\nBuild in STM32CubeIDE (Debug) "
                                      f"or use Choose ELF.")
            return
        try:
            img = fc_flash.elf_to_image(self.elf)
        except SystemExit:
            messagebox.showerror(APP, f"{self.elf} is not a usable F411 ELF.")
            return
        crc = zlib.crc32(img) & 0xFFFFFFFF
        build = img[fc_flash.BUILD_ID_OFFSET:fc_flash.BUILD_ID_OFFSET + 32].split(b"\0")[0].decode("ascii", "replace")
        stale = ""
        src = newest_source()
        if src > os.path.getmtime(self.elf) + 2:
            stale = (f"\n\nWARNING: sources changed {(src - os.path.getmtime(self.elf)) / 60:.0f} min after this "
                     f"ELF was built. Rebuild first or you will flash the old code.")
        msg = (f"Flash {os.path.basename(self.elf)} over USB?\n\n"
               f"  size   {len(img):,} bytes   crc32 {crc:08x}\n"
               f"  build  {build or '?'}   (running: {self.build})\n"
               f"  phase  {self.phase() or 'unknown'}\n\n"
               f"The board stops logging, reprograms itself (~3 s, fins centred) and re-enumerates. "
               f"If the new image does not boot, recover over SWD (ST-LINK).{stale}")
        if not messagebox.askokcancel(APP, msg, icon="warning" if stale else "question", default="cancel"):
            return
        self._start("flashing", self._flash, img, crc, build)

    def _flash(self, img: bytes, crc: int, build: str) -> None:
        b = self.board
        self._quiet()
        b.proto_begin()
        self.ui(lambda: self.note(f"flash: {len(img):,} B crc32={crc:08x} build {build}"))
        self.tx(f"fw {len(img)} {crc:08x}")
        line = b.expect(("FWREADY", "FWABORT", "ERR"), 5.0)
        if not line or not line.startswith("FWREADY"):
            self.ui(lambda: self._op_msg(f"flash: board refused: {line or 'no reply'}", "err"))
            self.tx("stream on")
            return
        t0 = time.monotonic()
        for i in range(0, len(img), 4096):
            if not b.write_raw(img[i:i + 4096]):
                self.ui(lambda: self._op_msg("flash: link dropped during transfer (old firmware intact)", "err"))
                return
            n = min(i + 4096, len(img))
            self._progress(80 * n / len(img), f"sending {n // 1024}/{len(img) // 1024} KB")
        dt = time.monotonic() - t0
        for k, want in enumerate(("FWRECV", "FWCRC", "FWVEC", "FWPROG")):
            line = b.expect((want, "FWABORT"), 10.0)
            if not line or line.startswith("FWABORT") or " bad" in line:
                self.ui(lambda l=line: self._op_msg(f"flash aborted: {l or 'timeout'}; old firmware intact", "err"))
                self.tx("stream on")
                return
            self._progress(82 + 4 * k, f"{line}")
        self._progress(90, "programming - do not unplug")
        self.ui(lambda: self.note(f"sent in {dt:.1f} s; board is programming itself - do not unplug"))
        b.wait_disconnect(8.0)
        if not b.wait_connect(40.0):
            self.ui(lambda: self._op_msg("flash: board did not come back - recover over SWD", "err"))
            return
        time.sleep(1.0)
        b.proto_begin()
        self.tx("ver")
        line = b.expect((link.VER_PREFIX,), 5.0)
        self._progress(100)
        if line and build and build in line:
            self.ui(lambda: self._op_msg(f"flash OK - running {build}", "ok"))
        elif line:
            self.ui(lambda: self._op_msg(f"flashed, but running build does not match '{build}': {line}", "err"))
        else:
            self.ui(lambda: self._op_msg("flashed; no reply to 'ver'", "err"))

    # log ---------------------------------------------------------------------
    def on_pull(self) -> None:
        if self.b_pull.cget("state") == "disabled":
            return
        self._start("pulling log", self._pull)

    def _pull(self) -> None:
        import fc_pull
        b = self.board
        was = self._quiet()
        b.arm_capture()
        b.proto_begin()
        self.tx("log pull")
        line = b.expect(("LOGDATA", "ERR"), 5.0)
        if not line or not line.startswith("LOGDATA"):
            b.demux.disarm()
            self.ui(lambda: self._op_msg(f"pull: {line or 'no reply'}", "err"))
        else:
            n = int(line.split()[1])
            t0 = time.monotonic()
            raw = b.wait_capture(3.0, lambda got, want: self._progress(
                90 * got / want, f"pulling {got // 1024}/{want // 1024} KB"))
            trailer = b.expect(("LOGEND",), 5.0) if raw is not None else None
            if raw is None or not trailer:
                self.ui(lambda: self._op_msg("pull: transfer stalled or link lost - retry", "err"))
            else:
                want = int(trailer.split()[1], 16)
                got = zlib.crc32(raw) & 0xFFFFFFFF
                if got != want:
                    self.ui(lambda: self._op_msg(f"pull: CRC mismatch {got:08X} != {want:08X} - retry", "err"))
                else:
                    out = os.path.join(LOG_DIR, time.strftime("%Y%m%d-%H%M%S"))
                    os.makedirs(out, exist_ok=True)
                    with open(os.path.join(out, "flightlog.bin"), "wb") as fh:
                        fh.write(raw)
                    buf = io.StringIO()
                    with contextlib.redirect_stdout(buf):
                        fc_pull.decode(raw, out)
                    dt = time.monotonic() - t0
                    self.last_pull_dir = out

                    def report():
                        for s in buf.getvalue().splitlines():
                            self.note("  " + s, "ok")
                        self._op_msg(f"pulled {n:,} B in {dt:.1f} s, CRC OK -> logs/{os.path.basename(out)}", "ok")
                    self.ui(report)
                    self._progress(100)
        if was:
            self.tx("stream on")

    def open_last_pull(self) -> None:
        if self.last_pull_dir:
            open_folder(self.last_pull_dir)

    def decode_file(self) -> None:
        p = filedialog.askopenfilename(title="Raw flight log", initialdir=LOG_DIR,
                                       filetypes=[("Flight log", "*.bin"), ("All", "*.*")])
        if not p:
            return
        import fc_pull
        buf = io.StringIO()
        with contextlib.redirect_stdout(buf):
            fc_pull.decode(open(p, "rb").read(), os.path.dirname(p))
        for s in buf.getvalue().splitlines():
            self.note("  " + s, "ok")
        self.last_pull_dir = os.path.dirname(p)
        self.refresh_controls()

    def on_erase(self) -> None:
        if self.b_erase.cget("state") == "disabled":
            return
        if not messagebox.askokcancel(APP, "Erase the flight log (384 KB)?\n\nEvery recorded session is lost. "
                                           "This cannot be undone - pull it first (F6) if you need it.\n\n"
                                           "Takes ~3-6 s; fins are centred meanwhile.",
                                      icon="warning", default="cancel"):
            return
        self._start("erasing log", self._erase)

    def _erase(self) -> None:
        b = self.board
        b.proto_begin()
        self.ui(lambda: (self.prog.configure(mode="indeterminate"), self.prog.start(15)))
        self.tx("log erase")
        line = b.expect(("log erase: OK", "log erase: FAILED", "ERR"), 20.0)
        ok = bool(line and line.endswith("OK"))
        self.ui(lambda: self._op_msg(line or "log erase: no reply", "ok" if ok else "err"))

    # board -------------------------------------------------------------------
    def on_reboot(self) -> None:
        if self.b_reboot.cget("state") != "disabled" and messagebox.askokcancel(
                APP, "Reboot the board?\n\nIt re-calibrates the gyro on start: keep it still.",
                icon="warning", default="cancel"):
            self.tx("reboot")

    def on_clear_faults(self) -> None:
        if self.b_clear_faults.cget("state") == "disabled":
            return
        listed = "\n".join("  " + f for f in self.fault_list) or "  (none reported)"
        if messagebox.askokcancel(APP, f"Clear latched faults?\n\n{listed}\n\nPad only. Refused while a HARD "
                                       f"monitor is still failing. From LOCKOUT the board re-enters SETTLE and "
                                       f"needs the full fault-free hold before READY.", icon="warning",
                                  default="cancel"):
            self.tx("faults clear")

    def send_many(self, cmds) -> None:
        for c in cmds:
            if not self.send(c):
                return

    # ---------------------------------------------------------------- misc
    def show_keys(self) -> None:
        messagebox.showinfo(APP, "F1  this list\nF2  pause / resume status stream\nF4  build firmware (CubeIDE)\n"
                                 "F5  flash firmware\n"
                                 "F6  pull + decode flight log\nF7  field / bench theme\nF8  clear latched faults (pad)\n\n"
                                 "Ctrl+1/2/3  Console / Dashboard / Plots\nSpace  freeze plots (Plots tab)\n"
                                 "Ctrl+F  find (Enter = next)\n"
                                 "Ctrl+L  clear terminal\nCtrl+ +/-  text size\nEsc  back to the command box\n\n"
                                 "Command box: Up/Down history, Tab completes.\n"
                                 "Scroll the terminal up to pause it; scroll to the end to follow again.")

    def start_session_log(self) -> None:
        d = os.path.join(LOG_DIR, "console")
        with contextlib.suppress(OSError):
            os.makedirs(d, exist_ok=True)
            self.session = open(os.path.join(d, time.strftime("%Y%m%d-%H%M%S.txt")), "a", encoding="utf-8",
                                buffering=1)

    def _session_toggled(self) -> None:
        self.cfg["session_log"] = self.v_session.get()
        if self.v_session.get() and not self.session:
            self.start_session_log()
        elif not self.v_session.get() and self.session:
            self.session.close()
            self.session = None

    def on_close(self) -> None:
        if self.busy and not messagebox.askokcancel(
                APP, f"{self.busy} is in progress. Quitting now may leave the board needing an SWD "
                     f"reflash.\n\nQuit anyway?", icon="warning", default="cancel"):
            return
        zoomed = self.root.state() == "zoomed"
        if not zoomed:
            self.cfg["geometry"] = self.root.geometry()
        self.cfg.update(zoomed=zoomed, plot_window=self.plot_window, tab=self.nb.index("current"), history=self.history[-200:],
                        show_status=self.v_show_status.get(), stamps=self.v_stamp.get(), elf=self.elf)
        save_settings(self.cfg)
        self.board.stop()
        if self.session:
            self.session.close()
        self.root.destroy()


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", help="serial port (default: find USB 0483:5740)")
    ap.add_argument("--sim", action="store_true", help="no board: run against the built-in simulator")
    ap.add_argument("--sim-binary", action="store_true", help="simulator sends 0xAA55 binary frames")
    args = ap.parse_args()
    if not args.sim:
        try:
            import serial                                       # noqa: F401
        except ImportError:
            sys.exit("pyserial is required:  pip install pyserial")
    if sys.platform.startswith("win"):
        import ctypes
        with contextlib.suppress(Exception):                    # crisp text on high-DPI laptops
            ctypes.windll.shcore.SetProcessDpiAwareness(1)
        with contextlib.suppress(Exception):                    # own taskbar icon, not python's
            ctypes.windll.shell32.SetCurrentProcessExplicitAppUserModelID("Hyades.FCV4.Console")
    root = tk.Tk()
    App(root, args)
    root.mainloop()


if __name__ == "__main__":
    main()
