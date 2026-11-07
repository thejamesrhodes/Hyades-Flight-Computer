"""
hitl_model.py -- the physical world for HITL (HITL-01, NAV-01): vehicle, roll
plant, canard actuator, IMU and GPS models. Pure Python + numpy; no I/O to the
board (fc_hitl.py drives that).

Sources (single source of truth, no retyped constants where avoidable):
  * Trajectory inputs (thrust, mass, roll inertia, drag coefficient vs Mach,
    rail length, descent rate) from an OpenRocket CSV export of the
    '62mm Canard Control V1.0.0' model (Simulation/or_sim*.csv).
  * Roll aerodynamics, actuator and canard geometry from the Matlab roll-tuning
    parameters: sim_params.json written by export_sim_params.m. Without it a
    built-in copy of parameters_initialise_roll_control.m (2026-08-24) is used
    and the run is flagged 'params=FALLBACK'.
  * IMU noise from the 2.46 h static capture (Docs/noise, board.h IEKF block).

Model (deliberate simplifications, see HITL-01):
  * 3-DOF translation in ENU with thrust/drag along the body axis; drag
    coefficient Cd(Mach) interpolated from OpenRocket; ISA atmosphere at the
    site elevation; constant wind.
  * Attitude: the body axis weathercocks toward the air-relative velocity with
    a first-order lag (tau_wc) after the rail -- a stand-in for a stable
    short period (2.8 cal), not a pitch/yaw model. Roll is fully dynamic:
        Ixx p_dot = qbar A d (Cl_delta delta + Cl_p p d/(2V) + Cl_0)
    with Cl_delta, Cl_p from the Matlab Barrowman/DATCOM expressions
    (optimise_roll_schedule.m roll_coeffs) at the current Mach.
  * Actuator: port of canard_servo_reduced.m (quantised command, transport
    delay, deadband, 2nd-order with rate and travel limits, backlash).
  * IMU: QMI8658A at 224.2 Hz ODR, white noise at the measured density, a
    1st-order ~30 Hz LPF, constant + random-walk gyro bias, 16-bit
    quantisation, +/-16 g / +/-2048 dps saturation; the board takes the latest
    ODR sample per 100 Hz frame. Accel scale-factor error per axis (default 0).
  * GPS: 1 Hz GGA+RMC, white + slow position error, loses lock above
    gps_lock_g and reacquires gps_reacq_s later.
"""
from __future__ import annotations

import json
import math
import os
from dataclasses import dataclass, field

import numpy as np

G0 = 9.80665
D2R, R2D = math.pi / 180.0, 180.0 / math.pi
HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.abspath(os.path.join(HERE, "..", "..", ".."))
SIM_DIR = os.path.join(REPO, "Simulation")

# ---------------------------------------------------------------------------
# Parameters
# ---------------------------------------------------------------------------
# Copy of parameters_initialise_roll_control.m (only the fields used here),
# used only when sim_params.json (export_sim_params.m) is absent.
FALLBACK_P = {
    "Aero": {"Cl_0": 0.0, "C_f": 0.003},
    "Vehicle": {"d": 0.062, "s": 0.018, "C_r": 0.024, "C_t": 0.012, "N": 4, "L": 0.8, "Ixx": 0.0007},
    "Linkage": {"n": 1.0},
    "Servo": {"f_N": 80.0, "Zeta": 0.7, "Tau_d": 0.014, "Deadband": 0.00196, "Lash": 0.0020,
              "Res": 5.1e-4, "Travel": 0.2094, "Rate": 23.65, "dt": 5e-4},
}


def load_params(path: str | None = None) -> tuple[dict, str]:
    path = path or os.path.join(HERE, "sim_params.json")
    if os.path.exists(path):
        with open(path) as f:
            return json.load(f), "matlab:" + os.path.basename(path)
    return json.loads(json.dumps(FALLBACK_P)), "FALLBACK"


def y_cp(P: dict) -> float:
    V = P["Vehicle"]
    return V["d"] / 2 + V["s"] * (V["C_r"] + 2 * V["C_t"]) / (3 * (V["C_r"] + V["C_t"]))


def roll_coeffs(P: dict, mach: float) -> tuple[float, float]:
    """Port of roll_coeffs (optimise_roll_schedule.m). Matlab uses beta from
    V/343; here the local Mach is passed in (pass V/343 to match exactly)."""
    V = P["Vehicle"]
    d, s, Cr, Ct, N = V["d"], V["s"], V["C_r"], V["C_t"], V["N"]
    S_c = (Cr + Ct) * s * 0.5
    AR = s * s / S_c
    r = d / 2
    A = math.pi * r * r
    beta = math.sqrt(max(1 - mach * mach, 1e-6))
    a = 2 * math.pi * AR / (2 + math.sqrt(4 + (AR * beta) ** 2))
    K = 1 + r / (s + r)
    ycp = P["Aero"].get("y_Cp", y_cp(P))
    Cl_delta = K * N * a * (S_c / A) * (ycp / d)
    k = (Ct - Cr) / s
    I3 = ((r + s) ** 3 - r ** 3) / 3
    I4 = ((r + s) ** 4 - r ** 4) / 4
    Cl_p = -2 * K * N * a * (Cr * I3 + k * (I4 - r * I3)) / (A * d * d) - 2 * P["Aero"]["C_f"] * (V["L"] / d)
    return Cl_delta, Cl_p


class Servo:
    """Port of canard_servo_reduced.m, one call per P.Servo.dt."""

    def __init__(self, P: dict) -> None:
        S, self.n = P["Servo"], float(P["Linkage"]["n"])
        self.dt = S["dt"]
        self.q_cmd = S["Res"] / self.n
        self.nd = max(int(round(S["Tau_d"] / self.dt)), 1)
        self.buf = [0.0] * (self.nd + 1)
        self.w = 0
        self.dz = S["Deadband"] / self.n
        self.wn, self.z = S["f_N"], S["Zeta"]
        self.rate = S["Rate"] / self.n
        self.travel = S["Travel"]
        self.b = 0.5 * S["Lash"] / self.n
        self.th = self.thd = self.lash = 0.0

    def step(self, cmd: float) -> float:
        u = round(cmd / self.q_cmd) * self.q_cmd
        self.buf[self.w] = u
        cmd_d = self.buf[(self.w - self.nd) % len(self.buf)]
        self.w = (self.w + 1) % len(self.buf)
        e = cmd_d - self.th
        e_dz = 0.0 if abs(e) <= self.dz else e - math.copysign(self.dz, e)
        thdd = self.wn * self.wn * e_dz - 2 * self.z * self.wn * self.thd
        self.thd = max(min(self.thd + thdd * self.dt, self.rate), -self.rate)
        self.th += self.thd * self.dt
        if self.th > self.travel:
            self.th, self.thd = self.travel, 0.0
        if self.th < -self.travel:
            self.th, self.thd = -self.travel, 0.0
        if self.th - self.lash > self.b:
            self.lash = self.th - self.b
        elif self.lash - self.th > self.b:
            self.lash = self.th + self.b
        return self.lash


# ---------------------------------------------------------------------------
# OpenRocket trajectory inputs
# ---------------------------------------------------------------------------
@dataclass
class Motor:
    name: str
    t: np.ndarray            # s
    thrust: np.ndarray       # N
    mass: np.ndarray         # kg
    ixx: np.ndarray          # kg m^2 (roll)
    t_burnout: float
    mach_cd: np.ndarray      # [[M, Cd], ...] sorted
    a_ref: float             # m^2
    rail_m: float
    v_descent: float         # m/s at ground hit
    ej_delay: float          # s, apogee -> deployment (OpenRocket)
    events: dict = field(default_factory=dict)
    ref: dict = field(default_factory=dict)   # apogee, vmax, amax for validation


def load_openrocket(path: str) -> Motor:
    hdr, rows, ev = None, [], {}
    with open(path, encoding="utf-8") as f:
        for line in f:
            if line.startswith("# Time"):
                hdr = [h.strip() for h in line[2:].split(",")]
            elif line.startswith("# Event"):
                w = line.split()
                ev[w[2]] = float(w[-2].split("=")[1])
            elif not line.startswith("#") and line.strip():
                rows.append([float(x) for x in line.split(",")])
    a = np.array(rows)
    col = {h.split(" (")[0]: i for i, h in enumerate(hdr)}
    t = a[:, col["Time"]]
    m = a[:, col["Mass"]] / 1000.0
    ixx = a[:, col["Rotational moment of inertia"]]
    thrust = a[:, col["Thrust"]]
    asc = t < ev.get("APOGEE", t[-1])
    mach, cd = a[asc, col["Mach number"]], a[asc, col["Drag coefficient"]]
    ok = np.isfinite(mach) & np.isfinite(cd) & (mach > 0.02)
    order = np.argsort(mach[ok])
    mcd = np.column_stack([mach[ok][order], cd[ok][order]])
    alt = a[:, col["Altitude"]]
    rail = float(np.interp(ev["LAUNCHROD"], t, alt))
    ixx_ok = np.where(np.isfinite(ixx), ixx, np.nan)
    name = os.path.splitext(os.path.basename(path))[0]
    valid = t <= ev["BURNOUT"] + 0.5
    return Motor(
        name=name, t=t[valid], thrust=thrust[valid], mass=m[valid], ixx=ixx_ok[valid],
        t_burnout=ev["BURNOUT"], mach_cd=mcd, a_ref=a[0, col["Reference area"]] * 1e-4,
        rail_m=rail, v_descent=abs(a[-1, col["Vertical velocity"]]),
        ej_delay=ev.get("RECOVERY_DEVICE_DEPLOYMENT", ev["APOGEE"]) - ev["APOGEE"], events=ev,
        ref={"apogee": float(np.nanmax(alt)), "vmax": float(np.nanmax(a[:, col["Total velocity"]])),
             "amax": float(np.nanmax(a[:, col["Total acceleration"]])), "t_apogee": ev["APOGEE"]})


# ---------------------------------------------------------------------------
# Atmosphere
# ---------------------------------------------------------------------------
def isa(h_msl: float) -> tuple[float, float]:
    """(rho kg/m^3, speed of sound m/s), ISA troposphere."""
    T = 288.15 - 0.0065 * min(max(h_msl, -500.0), 11000.0)
    p = 101325.0 * (T / 288.15) ** 5.255877
    return p / (287.053 * T), math.sqrt(1.4 * 287.053 * T)


# ---------------------------------------------------------------------------
# Quaternion helpers (w x y z, body -> world, Hamilton; as the firmware)
# ---------------------------------------------------------------------------
def q_to_R(q) -> np.ndarray:
    w, x, y, z = q
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                     [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                     [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


def q_mul(p, q) -> np.ndarray:
    pw, px, py, pz = p
    qw, qx, qy, qz = q
    return np.array([pw * qw - px * qx - py * qy - pz * qz, pw * qx + px * qw + py * qz - pz * qy,
                     pw * qy - px * qz + py * qw + pz * qx, pw * qz + px * qy - py * qx + pz * qw])


def q_exp(rv) -> np.ndarray:
    a = float(np.linalg.norm(rv))
    if a < 1e-12:
        return np.array([1.0, rv[0] / 2, rv[1] / 2, rv[2] / 2])
    s = math.sin(a / 2) / a
    return np.array([math.cos(a / 2), rv[0] * s, rv[1] * s, rv[2] * s])


# ---------------------------------------------------------------------------
# Sensors
# ---------------------------------------------------------------------------
@dataclass
class ImuCfg:
    odr_hz: float = 224.2
    lpf_hz: float = 224.2 * 0.1337
    arw_dps_rthz: float = 7.9e-3        # measured 7.37-8.51 mdps/rtHz
    vrw_g_rthz: float = 72e-6           # measured 66-84 ug/rtHz
    gyro_bias_dps: float = 0.02         # 1-sigma residual after boot cal
    gyro_rw_dps_rts: float = 1.29e-5 ** 0.5 * R2D * 0.0   # bias RW off by default
    acc_sf: tuple = (0.0, 0.0, 0.0)     # per-axis scale-factor error (fraction)
    acc_fs: float = 16.0 * G0
    gyro_fs: float = 2048.0
    acc_lsb: float = G0 / 2048.0
    gyro_lsb: float = 1.0 / 16.0


@dataclass
class VibCfg:
    """Flight vibration at the IMU (ASSUMED levels -- no Hyades flight or
    static-fire data yet; set from a measured PSD when there is one).
    Broadband: flat PSD between band[0] and band[1] Hz. Resonance: a narrow
    peak (airframe/fin mode) during boost. Gyro: angular vibration, broadband.
    Levels are g RMS per axis (accel) and dps RMS per axis (gyro)."""
    boost_g: float = 0.0
    coast_g: float = 0.0
    chute_g: float = 0.0
    res_hz: float = 180.0
    res_g: float = 0.0            # boost only
    band: tuple = (20.0, 1500.0)
    gyro_dps: float = 0.0         # boost; coast at coast_g/boost_g of it
    internal_clip: bool = True    # clip at FS BEFORE the decimation filter (worst case)


class VibGen:
    """Unit-RMS band-limited Gaussian noise at the sensor's internal rate,
    generated in chunks by FFT shaping (no scipy)."""

    CHUNK = 1 << 16

    def __init__(self, fs: float, band: tuple, res_hz: float, rng: np.random.Generator) -> None:
        self.fs, self.band, self.res_hz, self.rng = fs, band, res_hz, rng
        self.buf = np.zeros((0, 9))
        self.pos = 0

    def _shape(self, lo: float, hi: float) -> np.ndarray:
        n = self.CHUNK
        X = np.fft.rfft(self.rng.normal(0.0, 1.0, (n, 3)), axis=0)
        f = np.fft.rfftfreq(n, 1.0 / self.fs)
        X[(f < lo) | (f > hi)] = 0.0
        x = np.fft.irfft(X, n=n, axis=0)
        return x / max(float(np.sqrt(np.mean(x ** 2))), 1e-12)

    def take(self, k: int) -> np.ndarray:
        """k x 9: [accel broadband(3) | accel resonance(3) | gyro broadband(3)]."""
        while self.buf.shape[0] - self.pos < k:
            hi = min(self.band[1], 0.45 * self.fs)
            res = self._shape(self.res_hz * 0.95, self.res_hz * 1.05)
            self.buf = np.vstack([self.buf[self.pos:], np.hstack([self._shape(self.band[0], hi), res,
                                                                  self._shape(self.band[0], hi)])])
            self.pos = 0
        out = self.buf[self.pos:self.pos + k]
        self.pos += k
        return out


class Imu:
    DECIM = 32                    # QMI8658A: ODR = internal rate / 32 (7174.4 / 224.2 Hz)

    def __init__(self, cfg: ImuCfg, rng: np.random.Generator, vib: VibCfg | None = None) -> None:
        self.c, self.rng = cfg, rng
        self.vib = vib or VibCfg()
        self.vgen = VibGen(cfg.odr_hz * self.DECIM, self.vib.band, self.vib.res_hz, rng)
        self.level = (0.0, 0.0, 0.0)      # set by the vehicle: (bb g, res g, gyro dps)
        self.clip_count = 0
        self.ts = 1.0 / cfg.odr_hz
        self.alpha = 1.0 - math.exp(-2 * math.pi * cfg.lpf_hz * self.ts)
        self.fa = np.zeros(3)
        self.fg = np.zeros(3)
        self.bg = rng.normal(0.0, cfg.gyro_bias_dps, 3)
        self.next_t = 0.0
        self.out_a = np.zeros(3)
        self.out_g = np.zeros(3)
        self.init = False

    def sample(self, t: float, f_body: np.ndarray, w_dps: np.ndarray) -> None:
        """Advance the sensor to time t with the truth at t (call at >= ODR)."""
        c = self.c
        while self.next_t <= t:
            na = self.rng.normal(0.0, c.vrw_g_rthz * G0 * math.sqrt(c.odr_hz), 3)
            ng = self.rng.normal(0.0, c.arw_dps_rthz * math.sqrt(c.odr_hz), 3)
            a_true = f_body * (1.0 + np.array(c.acc_sf))
            bb, rs, gy = self.level
            if bb > 0.0 or rs > 0.0 or gy > 0.0:
                # internal-rate samples: truth + vibration, (optionally) clipped at
                # full scale BEFORE decimation -- clipping a vibrating signal
                # rectifies it into a DC error the filter cannot remove.
                v = self.vgen.take(self.DECIM)
                ai = a_true + G0 * (bb * v[:, 0:3] + rs * v[:, 3:6])
                gi = w_dps + gy * v[:, 6:9]
                if self.vib.internal_clip:
                    self.clip_count += int(np.sum(np.abs(ai) >= c.acc_fs))
                    ai = np.clip(ai, -c.acc_fs, c.acc_fs)
                    gi = np.clip(gi, -c.gyro_fs, c.gyro_fs)
                a_true, g_mean = ai.mean(axis=0), gi.mean(axis=0)
            else:
                g_mean = w_dps
            a_in = a_true + na
            g_in = g_mean + self.bg + ng
            if not self.init:
                self.fa, self.fg, self.init = a_in.copy(), g_in.copy(), True
            self.fa += self.alpha * (a_in - self.fa)
            self.fg += self.alpha * (g_in - self.fg)
            self.out_a = np.clip(np.round(self.fa / c.acc_lsb) * c.acc_lsb, -c.acc_fs, c.acc_fs - c.acc_lsb)
            self.out_g = np.clip(np.round(self.fg / c.gyro_lsb) * c.gyro_lsb, -c.gyro_fs, c.gyro_fs - c.gyro_lsb)
            self.next_t += self.ts


def nmea_ck(body: str) -> str:
    c = 0
    for ch in body:
        c ^= ord(ch)
    return f"${body}*{c:02X}\r\n"


@dataclass
class GpsCfg:
    lat0: float = 52.64          # MRC, near Twycross -- approximate, unverified
    lon0: float = -1.53
    sigma_h: float = 5.0         # m, white
    sigma_n: float = 2.5
    drift_h: float = 0.3         # m/sqrt(s) random walk
    sats: int = 9
    hdop: float = 1.0
    lock_g: float = 4.0          # loses lock above this |a| (dynamics)
    alt_bias: float = 0.0        # m, constant altitude error (worst-case receiver)
    reacq_s: float = 5.0


class Gps:
    def __init__(self, cfg: GpsCfg, site_elev: float, rng: np.random.Generator) -> None:
        self.c, self.elev, self.rng = cfg, site_elev, rng
        self.next_t = 0.0
        self.bias = np.zeros(3)
        self.lost_until = -1.0

    def step(self, t: float, p_enu: np.ndarray, v_enu: np.ndarray, a_mag: float) -> bytes:
        """Returns NMEA bytes due at time t (1 Hz), else b''."""
        if a_mag > self.c.lock_g * G0:
            self.lost_until = t + self.c.reacq_s
        if t < self.next_t:
            return b""
        self.next_t += 1.0
        self.bias += self.rng.normal(0.0, self.c.drift_h, 3) * np.array([0.5, 0.5, 1.0])
        hh, mm = divmod(int(43200 + t), 3600)
        mm, ss = divmod(mm, 60)
        utc = f"{hh:02d}{mm:02d}{ss:02d}.00"
        if t < self.lost_until:
            return (nmea_ck(f"GPGGA,{utc},,,,,0,00,99.9,,M,,M,,") + nmea_ck(f"GPRMC,{utc},V,,,,,,,,,,N")).encode()
        n = p_enu + self.bias + self.rng.normal(0.0, [self.c.sigma_n, self.c.sigma_n, self.c.sigma_h])
        lat = self.c.lat0 + n[1] / 111320.0
        lon = self.c.lon0 + n[0] / (111320.0 * math.cos(self.c.lat0 * D2R))
        alt = self.elev + n[2] + self.c.alt_bias

        def dm(v: float, deg_w: int) -> tuple[str, float]:
            a = abs(v)
            d = int(a)
            return f"{d:0{deg_w}d}{(a - d) * 60:07.4f}", v
        la, _ = dm(lat, 2)
        lo, _ = dm(lon, 3)
        ns, ew = ("N" if lat >= 0 else "S"), ("E" if lon >= 0 else "W")
        spd_kn = float(np.hypot(v_enu[0], v_enu[1])) / 0.514444
        crs = (math.degrees(math.atan2(v_enu[0], v_enu[1])) + 360) % 360
        gga = f"GPGGA,{utc},{la},{ns},{lo},{ew},1,{self.c.sats:02d},{self.c.hdop:.1f},{alt:.1f},M,48.0,M,,"
        rmc = f"GPRMC,{utc},A,{la},{ns},{lo},{ew},{spd_kn:.2f},{crs:.1f},071126,,,A"
        return (nmea_ck(gga) + nmea_ck(rmc)).encode()


# ---------------------------------------------------------------------------
# Vehicle
# ---------------------------------------------------------------------------
@dataclass
class SimCfg:
    site_elev: float = 110.0      # MRC estimate (board.h NAV_SITE_ELEV_M), unverified
    wind_enu: tuple = (2.2, 0.0, 0.0)
    tau_wc: float = 0.08          # s, weathercock lag (stand-in for short period)
    dt: float | None = None       # dynamics step; default = P.Servo.dt
    cl0: float = 0.0              # roll forcing (fin misalignment), Cl
    roll_ixx: float | None = None # override; default OpenRocket roll inertia
    fin_deg: float = 30.0         # firmware FIN_DEFLECT_DEG: fin=+/-1 -> +/-30 deg command
    use_fins: bool = True
    seed: int = 1
    # dispersions / worst cases
    cld_scale: float = 1.0        # control effectiveness error (aero model)
    clp_scale: float = 1.0        # roll damping error
    ixx_scale: float = 1.0
    thrust_scale: float = 1.0     # motor-to-motor impulse variation
    servo: dict = field(default_factory=dict)   # overrides of P.Servo (f_N, Zeta, Tau_d, Deadband, Lash)
    gust: tuple | None = None     # (t_start s, duration s, peak m/s, from-direction deg): 1-cosine gust
    pad_knocks: tuple = ()        # ((t_pad s, peak g, duration s, axis 0..2), ...) handling on the pad
    pad_sway_deg: float = 0.0     # rail/airframe sway amplitude on the pad (0.7 Hz)
    vib: VibCfg = field(default_factory=VibCfg)


class Vehicle:
    """Integrates the world at cfg.dt; the board frame is 10 ms."""

    def __init__(self, motor: Motor, P: dict, cfg: SimCfg, imu: ImuCfg | None = None,
                 gps: GpsCfg | None = None) -> None:
        self.m, self.P, self.c = motor, P, cfg
        self.dt = cfg.dt or P["Servo"]["dt"]
        self.rng = np.random.default_rng(cfg.seed)
        if cfg.servo:
            P = json.loads(json.dumps(P))
            P["Servo"].update(cfg.servo)
            self.P = P
        self.imu = Imu(imu or ImuCfg(), self.rng, cfg.vib)
        self.gps = Gps(gps or GpsCfg(), cfg.site_elev, self.rng)
        self.servos = [Servo(self.P) for _ in range(4)]
        self.t = 0.0                 # time since ignition (negative = pad hold)
        self.ignited = False
        self.p = np.zeros(3)
        self.v = np.zeros(3)
        self.a = np.zeros(3)
        self.q = np.array([1.0, 0.0, 0.0, 0.0])   # nose up
        self.p_roll = 0.0            # roll rate, rad/s (body z)
        self.w_body = np.zeros(3)
        self.f_body = np.array([0.0, 0.0, G0])
        self.delta = 0.0             # roll-effective canard deflection, rad
        self.on_rail = True
        self.chute = False
        self.landed = False
        self.touchdown = None        # time of ground contact
        self.v_td = np.zeros(3)
        self.t_apogee = None
        self.qbar = 0.0
        self.mach = 0.0
        self.apogee = 0.0

    # -- inputs ------------------------------------------------------------
    def _interp(self, arr: np.ndarray, t: float) -> float:
        tt = self.m.t
        return float(np.interp(t, tt, arr, left=arr[0], right=arr[-1]))

    def mass(self) -> float:
        return self._interp(self.m.mass, max(self.t, 0.0)) if self.ignited else float(self.m.mass[0])

    def ixx(self) -> float:
        if self.c.roll_ixx:
            return self.c.roll_ixx
        ix = self.m.ixx[np.isfinite(self.m.ixx)]
        return self._interp(np.nan_to_num(self.m.ixx, nan=float(ix[-1])), max(self.t, 0.0))

    def thrust(self) -> float:
        if not self.ignited or self.t > self.m.t_burnout:
            return 0.0
        return self.c.thrust_scale * self._interp(self.m.thrust, self.t)

    def cd(self, mach: float) -> float:
        mc = self.m.mach_cd
        return float(np.interp(mach, mc[:, 0], mc[:, 1]))

    # -- one dynamics step ---------------------------------------------------
    def step(self, fin_norm: np.ndarray) -> None:
        dt, g = self.dt, np.array([0.0, 0.0, -G0])
        if self.ignited:
            self.t += dt
        R = q_to_R(self.q)
        zb = R[:, 2]
        h_msl = self.c.site_elev + self.p[2]
        rho, a_snd = isa(h_msl)
        wind = np.array(self.c.wind_enu, float)
        if self.c.gust is not None and self.ignited:
            t0, dur, amp, frm = self.c.gust
            if t0 <= self.t <= t0 + dur:
                u = 0.5 * amp * (1.0 - math.cos(2 * math.pi * (self.t - t0) / dur))
                wind = wind + u * np.array([-math.sin(math.radians(frm)), -math.cos(math.radians(frm)), 0.0])
        v_air = self.v - wind * (0.0 if self.on_rail else 1.0)
        V = float(np.linalg.norm(v_air))
        self.mach = V / a_snd
        self.qbar = 0.5 * rho * V * V
        m = self.mass()

        F = self.thrust() * zb
        if V > 0.1:
            F = F - self.qbar * self.m.a_ref * self.cd(self.mach) * (v_air / V)
        if self.chute and V > 0.1:
            k = m * G0 / (self.m.v_descent ** 2)          # matches OpenRocket ground-hit speed
            F = F - k * V * v_air
        acc = F / m + g

        if self.touchdown is not None:
            # ground contact: stop over T_STOP (soft field), so the board sees
            # the impact as specific force instead of a velocity step it cannot
            # observe; then at rest.
            if self.t < self.touchdown + self.T_STOP:
                acc = -self.v_td / self.T_STOP
            else:
                acc = np.zeros(3)
                self.v[:] = 0.0
                self.landed = True
        elif self.on_rail:
            along = float(acc @ zb)
            acc = max(along, 0.0) * zb if self.ignited else np.zeros(3)
            if self.ignited and float(self.p @ zb) >= self.m.rail_m:
                self.on_rail = False
        self.a = acc
        v0 = self.v.copy()
        self.v = self.v + acc * dt
        self.p = self.p + 0.5 * (v0 + self.v) * dt
        if self.ignited and self.touchdown is None and self.p[2] < 0.0 and self.t > 1.0:
            self.touchdown, self.v_td = self.t, self.v.copy()
        if self.touchdown is not None:
            self.p[2] = max(self.p[2], 0.0)
        if self.ignited and self.t_apogee is None and self.v[2] < 0.0 and self.t > 0.5:
            self.t_apogee = self.t
        self.apogee = max(self.apogee, self.p[2])
        if self.t_apogee is not None and not self.chute and self.t >= self.t_apogee + self.m.ej_delay:
            self.chute = True

        # roll: canards (mean deflection of the four fins, firmware mixing)
        cmd = (float(np.mean(fin_norm)) * self.c.fin_deg * D2R) if self.c.use_fins else 0.0
        defl = [s.step(cmd) for s in self.servos]
        self.delta = float(np.mean(defl))
        ixx = self.ixx() * self.c.ixx_scale
        if V > 1.0 and not self.on_rail and self.touchdown is None:
            Cld, Clp = roll_coeffs(self.P, self.mach)
            Cld, Clp = Cld * self.c.cld_scale, Clp * self.c.clp_scale
            A, d = math.pi * (self.P["Vehicle"]["d"] / 2) ** 2, self.P["Vehicle"]["d"]
            Lm = self.qbar * A * d * (Cld * self.delta + Clp * self.p_roll * d / (2 * V) + self.c.cl0)
            self.p_roll += Lm / ixx * dt
        elif self.on_rail or self.landed:
            self.p_roll = 0.0

        # attitude: weathercock the body axis toward v_air (after the rail)
        w_perp = np.zeros(3)
        if not self.on_rail and self.touchdown is None and V > 5.0:
            target = v_air / V
            err = np.cross(zb, target)                   # world-frame small rotation
            w_perp = err / self.c.tau_wc
        w_world = w_perp + zb * self.p_roll
        w_b = R.T @ w_world
        self.q = q_mul(self.q, q_exp(w_b * dt))
        self.q /= np.linalg.norm(self.q)
        self.w_body = w_b
        self.f_body = q_to_R(self.q).T @ (acc - g)
        w_sens = self.w_body.copy()
        if not self.ignited:
            tp = self.t_pad
            for (tk, gk, dk, ax) in self.c.pad_knocks:          # half-sine handling knocks
                if tk <= tp <= tk + dk:
                    self.f_body = self.f_body.copy()
                    self.f_body[int(ax)] += gk * G0 * math.sin(math.pi * (tp - tk) / dk)
            if self.c.pad_sway_deg > 0.0:
                wsw = 2 * math.pi * 0.7
                ang = math.radians(self.c.pad_sway_deg)
                w_sens[0] += ang * wsw * math.cos(wsw * tp)         # rad/s about body X
                self.f_body = self.f_body.copy()
                self.f_body[1] += G0 * ang * math.sin(wsw * tp)     # gravity tilt seen by accel
        vc = self.c.vib
        if self.ignited and self.touchdown is None:
            if self.thrust() > 0.0:
                self.imu.level = (vc.boost_g, vc.res_g, vc.gyro_dps)
            elif self.chute:
                self.imu.level = (vc.chute_g, 0.0, 0.0)
            else:
                k = vc.coast_g / vc.boost_g if vc.boost_g > 0 else 0.0
                self.imu.level = (vc.coast_g, 0.0, vc.gyro_dps * k)
        else:
            self.imu.level = (0.0, 0.0, 0.0)
        self.imu.sample(self.t if self.ignited else self.t_pad, self.f_body, w_sens * R2D)

    t_pad = 0.0
    T_STOP = 0.1                     # s, touchdown deceleration (8.6 m/s -> ~9 g)

    def ignite(self) -> None:
        self.ignited = True
        self.imu.next_t -= self.t_pad      # continue the ODR clock from the pad
        self.t_pad = 0.0

    def frame(self, fin_norm: np.ndarray, frame_dt: float = 0.01) -> None:
        n = int(round(frame_dt / self.dt))
        for _ in range(n):
            if not self.ignited:
                self.t_pad += self.dt
            self.step(fin_norm)

    def gps_bytes(self, t_world: float) -> bytes:
        return self.gps.step(t_world, self.p, self.v, float(np.linalg.norm(self.a + [0, 0, G0])))

    def truth(self) -> dict:
        return {"h": float(self.p[2]), "vz": float(self.v[2]), "speed": float(np.linalg.norm(self.v)),
                "qbar": self.qbar, "mach": self.mach, "p_dps": self.p_roll * R2D, "delta_deg": self.delta * R2D,
                "ax": float(self.f_body[0]), "ay": float(self.f_body[1]), "az": float(self.f_body[2]),
                "qw": float(self.q[0]), "qx": float(self.q[1]), "qy": float(self.q[2]), "qz": float(self.q[3])}
