"""
canard_model.py — open-loop plant characterisation for an all-moving canard set.

Scope (ADR-001): geometry x flight condition x linkage x bearing coupling ->
roll authority, hinge moment, actuator-limited rate, plant poles, constraint
margins.  NO control law, NO gains, NO closed-loop response — those live in the
MATLAB/Simulink toolchain.  This module only defines the plant it is handed.

All SI unless a name ends in _deg.  Angles internally in radians.

Model provenance
----------------
C_Nalpha   subsonic  : DATCOM/Diederich low-AR wing formula (reduces exactly to
                       the slender-wing limit pi*AR/2 as AR -> 0, which is the
                       built-in cross-check).
           supersonic: linearised 2-D 4/beta with the rectangular-wing tip-loss
                       factor for beta*AR >= 1; below that a documented blend to
                       the slender limit, continuous at beta*AR = 1.
           transonic : NOT MODELLED.  M in [0.8, 1.2] returns a nominal plus a
                       +/-30% band and transonic=True.  Per the project lit
                       review, semi-empirical codes exceed 20-30% error here and
                       cannot predict control-effectiveness sign reversal at all.
C_ldelta, C_lp       : strip loading proportional to chord (Barrowman form);
                       cross-checked against the published RocketPy equations.
hinge moment         : x_ac 0.25 c_bar subsonic -> 0.50 c_bar supersonic.
error floor          : p_floor = pdot_granularity / omega_c.  This is a SCALING
                       LAW, not a limit-cycle solution.  omega_c (closed-loop
                       crossover) is an INPUT because it is a control-design
                       quantity this module deliberately does not know.  A dead
                       zone cannot sustain a limit cycle in a linearly stable
                       loop -- its describing function is real and <= 1 -- so
                       what this estimates is bounded hang-off error, not
                       oscillation.  Backlash CAN sustain one; see backlash_df.
divergence           : q_div = K_delta / (S_f c_bar C_Nalpha (x_ac - x_h)).
                       NOTE the NACA TN 4197 fin-flutter formula used by rocketry
                       calculators is for FIXED fins and does not apply here.
servo                : DC torque-speed line, rate derated linearly by load.
"""
from __future__ import annotations

import json
from dataclasses import dataclass, asdict, field

import numpy as np

GAMMA, R_AIR, G0 = 1.4, 287.053, 9.80665
_trapz = getattr(np, "trapezoid", getattr(np, "trapz", None))  # numpy 1.x / 2.x
TRANSONIC = (0.80, 1.20)
TRANSONIC_BAND = 0.30          # +/- fractional uncertainty flagged in that window


# ----------------------------------------------------------------- atmosphere
def isa(alt: float):
    """ISA troposphere/lower stratosphere. Returns (T, p, rho, a)."""
    if alt < 11000.0:
        T = 288.15 - 0.0065*alt
        p = 101325.0*(T/288.15)**(-G0/(-0.0065*R_AIR))
    else:
        T = 216.65
        p = 22632.1*np.exp(-G0*(alt-11000.0)/(R_AIR*T))
    rho = p/(R_AIR*T)
    return T, p, rho, np.sqrt(GAMMA*R_AIR*T)


# ------------------------------------------------------------------ geometry
SECTIONS = {
    # name: (x_ac shift on c_bar vs flat plate, wave-drag factor k in Cd0w = k (t/c)^2 / beta)
    "smoothed hexagon": (0.00, 4.0),
    "sharp double wedge": (0.00, 4.0),
    "biconvex": (0.00, 5.33),
    "NACA 4-digit symmetric": (-0.01, 5.5),
}


@dataclass
class Canard:
    span: float = 0.025             # exposed semi-span s [m]
    root_chord: float = 0.055
    tip_chord: float = 0.035
    sweep_le_deg: float = 25.0
    thickness_ratio: float = 0.08   # t/c
    hinge_frac: float = 0.20        # x_hinge / c_bar
    n_fins: int = 4
    section: str = "smoothed hexagon"
    material_density: float = 1600.0   # kg/m^3 (CF laminate ~1600)
    ar_convention: str = "panel"       # "panel" = s^2/S_f, "image" = 2 s^2/S_f

    # --- derived ---
    @property
    def taper(self): return self.tip_chord/self.root_chord

    @property
    def area(self):  # exposed panel area, one panel
        return 0.5*(self.root_chord+self.tip_chord)*self.span

    @property
    def ar_eff(self):
        """Aspect ratio fed to the low-AR lift-slope formula.

        Two conventions are in circulation and they differ by 2x, which is a
        ~1.9x difference in C_Nalpha at AR ~ 1:

          "panel" : AR = s^2/S_f  -- the exposed panel's own aspect ratio.
          "image" : AR = 2 s^2/S_f -- the panel mirrored in the body surface,
                    treated as half of a wing of span 2s and area 2 S_f.

        The image-plane argument is the textbook one for a surface on a large
        flat wall, but a fin sits on a slender body of comparable size to its
        own span, not on a wall. Checked against Barrowman's own closed form
        for a fin set (4N(s/d)^2 / [1 + sqrt(1 + (2 l_m/(c_r+c_t))^2)]), the
        panel convention lands within ~10% and the image convention is ~107%
        high. "panel" is therefore the default, and it is what the Simulink
        plant function uses.
        """
        base = self.span**2/self.area
        return base if self.ar_convention == "panel" else 2.0*base

    @property
    def mac(self):
        lam = self.taper
        return (2.0/3.0)*self.root_chord*(1+lam+lam*lam)/(1+lam)

    @property
    def y_centroid(self):  # spanwise area centroid measured from the root
        return self.span*(self.root_chord+2*self.tip_chord)/(3*(self.root_chord+self.tip_chord))

    def sweep_half_chord(self):
        dx = (0.5*self.tip_chord - 0.5*self.root_chord)
        return np.arctan((self.span*np.tan(np.radians(self.sweep_le_deg)) + dx)/self.span)

    def chord(self, xi):
        """Chord at spanwise station xi measured from the root."""
        return self.root_chord + (self.tip_chord-self.root_chord)*xi/self.span

    def inertia_about_hinge(self):
        """Panel mass moment of inertia about the hinge line [kg m^2].
        Solid section of equivalent thickness 0.7*(t/c)*c (hexagon/double-wedge
        area factor); hinge assumed at the same chord fraction at every station.

            I = rho * t_eq * INT_span INT_0^c (x - x_h)^2 dx dxi
              = rho * t_eq * INT_span [ ((c-x_h)^3 + x_h^3) / 3 ] dxi
        """
        xi = np.linspace(0.0, self.span, 400)
        c = self.chord(xi)
        xh = self.hinge_frac*c                      # hinge line follows the chord
        t_eq = 0.7*self.thickness_ratio*c
        integrand = self.material_density*t_eq*((c - xh)**3 + xh**3)/3.0
        return float(_trapz(integrand, xi))

    def static_unbalance(self):
        """First moment of panel mass about the hinge, S_delta [kg m].
        Positive means the section CG lies AFT of the hinge line, which is the
        inertial coupling that drives control-surface flutter."""
        xi = np.linspace(0.0, self.span, 400)
        c = self.chord(xi)
        xh = self.hinge_frac*c
        t_eq = 0.7*self.thickness_ratio*c
        return float(_trapz(self.material_density*t_eq*c*(0.5*c - xh), xi))

    def x_cg_frac(self):
        """Chordwise CG of the section as a fraction of the local chord."""
        return 0.5

    def mass(self):
        """Panel mass [kg], same equivalent-thickness assumption."""
        xi = np.linspace(0.0, self.span, 400)
        c = self.chord(xi)
        return float(_trapz(self.material_density*0.7*self.thickness_ratio*c*c, xi))


@dataclass
class Vehicle:
    # Defaults are the Hyades 62 mm CV as parameterised in
    # parameters_initialise_roll_control.m (Ixx 0.00061 kg m^2 total, split
    # here between the controlled forebody and the fin can).
    diameter: float = 0.062
    Ixx_fore: float = 5.0e-4        # controlled forebody roll inertia [kg m^2]
    Ixx_tail: float = 1.1e-4        # aft fin can roll inertia [kg m^2]
    fin_Clp: float = -12.0          # tail-fin roll damping, per rad (p*d/2V)
    body_Clp: float = -0.15         # bare-body skin-friction roll damping
    fin_cant_deg: float = 0.20      # residual tail cant / build error
    fin_Cldelta: float = 8.0        # tail roll forcing per rad of cant
    canard_rig_error_deg: float = 0.15   # forebody-side disturbance, always present
    # --- longitudinal stations, all from the nose tip [m] ---
    x_cg: float = 0.700             # centre of gravity
    x_canard: float = 0.260         # canard quarter-MAC
    x_cp_rest: float = 0.900        # CP of everything except the canards
    cna_rest: float = 22.0          # C_Nalpha of everything except the canards, on S_ref

    @property
    def s_ref(self): return 0.25*np.pi*self.diameter**2


def backlash_df(ratio):
    """Describing function of backlash for input amplitude / lash half-width.

    Returns (|N|/K, phase [deg]).  Gelb & Vander Velde form.  Unlike a dead
    zone this is complex: backlash injects phase lag (up to -90 deg as the
    amplitude approaches the lash width) and is therefore the nonlinearity that
    can actually destabilise an otherwise stable loop.  Reported as a flag for
    the MATLAB side; nothing here consumes it.
    """
    if ratio <= 1.0:
        return 0.0, -90.0
    u = 1.0 - 2.0/ratio
    re = (1.0/np.pi)*(np.pi/2 + np.arcsin(u) + u*np.sqrt(max(1-u*u, 0.0)))
    im = -(4.0/(np.pi*ratio))*(1.0 - 1.0/ratio)
    return float(np.hypot(re, im)), float(np.degrees(np.arctan2(im, re)))


def omega_c_delay_ceiling(servo, phase_budget_deg):
    """Crossover above which the transport delay alone eats more than
    `phase_budget_deg` of phase margin.  omega <= budget / tau."""
    tau = servo.delay_s + 1.0/servo.frame_hz
    return np.radians(phase_budget_deg)/tau if tau > 0 else np.inf


# Blue Bird BMS-127WV+ manufacturer data, four voltage points.
# https://www.blue-bird-model.com/products_detail/413.htm
BMS127WV_V   = np.array([3.7, 6.0, 7.4, 8.4])
BMS127WV_T   = np.array([2.5, 4.0, 4.7, 5.3])*9.80665e-2   # N m stall
BMS127WV_T60 = np.array([0.11, 0.07, 0.06, 0.05])          # s per 60 deg, no load


def servo_from_datasheet(voltage, V=BMS127WV_V, T=BMS127WV_T, t60=BMS127WV_T60):
    """Derive the torque-speed line and reflected output inertia from a servo's
    published (voltage, stall torque, 60-degree time) table.

    The 60-degree time is a TRAVEL time, not a slew rate: it contains an
    acceleration and deceleration phase.  Fitting t60 = a/V + b separates them --
    `a/V` is the slew part and `b` is the accel/decel overhead.  Two things fall
    out that the raw spec does not give you:

      * the true no-load slew rate, 60 deg / (t60 - b), which is faster than the
        headline figure;
      * the output-referred inertia, J_out ~ b*T_stall/(2*w0), i.e. the reflected
        rotor + gearbox inertia, which for a high-ratio hobby servo dominates the
        control surface's own inertia by orders of magnitude.

    Checking the scaling also tells you WHICH limit the spec figure represents:
    slew-limited travel scales as 1/V, inertia-limited travel as 1/sqrt(V).

    Returns a dict; stall torque and slew rate are linearly interpolated in V.
    """
    A = np.vstack([1.0/V, np.ones_like(V)]).T
    (a_fit, b_fit), *_ = np.linalg.lstsq(A, t60, rcond=None)
    w0_true = np.radians(60.0)/(t60 - b_fit)
    J_out = float(np.mean(b_fit*T/(2*w0_true)))
    return dict(
        stall_torque=float(np.interp(voltage, V, T)),
        rate_noload=float(np.interp(voltage, V, w0_true)),
        rate_spec=float(np.interp(voltage, V, np.radians(60.0)/t60)),
        overhead_s=float(b_fit),
        J_out=J_out,
        alpha_max=float(np.interp(voltage, V, T)/J_out),
        slew_scaling=float((t60[0]/t60[-1])),
        slew_pred=float(V[-1]/V[0]),
        inertia_pred=float(np.sqrt(V[-1]/V[0])),
        peak_power=float(np.interp(voltage, V, T)*np.interp(voltage, V, w0_true)/4),
    )


@dataclass
class Servo:
    """BMS-127WV+ at 8.4 V.  Defaults derived in `servo_from_datasheet`, not
    copied off the headline spec — see that docstring."""
    stall_torque: float = 0.5198       # N m  (5.3 kg cm at 8.4 V)
    rate_noload_deg: float = 1355.0    # deg/s, slew-only (spec 1200 includes accel)
    deadband_deg: float = 0.21         # 2 us on 90 deg / 800 us (mfr) or 120 deg / 1200 us
    resolution_deg: float = 0.029      # 12 bit, 4096 steps -- 7x finer than the deadband
    backlash_deg: float = 0.15         # MEASURE THIS
    delay_s: float = 0.006             # transport/processing delay
    frame_hz: float = 250.0            # servo command frame rate
    stiffness: float = 30.0            # N m / rad at the servo output — MEASURE THIS
    efficiency: float = 0.85           # linkage efficiency
    J_out: float = 6.76e-5             # kg m^2, reflected rotor+gearbox at the output

    @property
    def rate_noload(self): return np.radians(self.rate_noload_deg)


@dataclass
class Linkage:
    ratio: float = 1.0                 # n = delta_servo / delta_canard

    def refer(self, servo: Servo):
        """Servo capability referred to the canard shaft."""
        n = self.ratio
        return dict(torque=n*servo.stall_torque*servo.efficiency,
                    rate=servo.rate_noload/n,
                    deadband=np.radians(servo.deadband_deg)/n,
                    resolution=np.radians(servo.resolution_deg)/n,
                    backlash=np.radians(servo.backlash_deg)/n,
                    stiffness=n*n*servo.stiffness*servo.efficiency)


@dataclass
class Bearing:
    """Aft fin-can roll bearing. c_visc = 0, T_coulomb = 0 is a free tail;
    c_visc -> inf is a seized (rigid) tail."""
    c_visc: float = 0.0        # N m / (rad/s)
    t_coulomb: float = 0.0     # N m


# ------------------------------------------------------------ aerodynamics
def cn_alpha(M: float, ar: float, sweep_half_chord: float = 0.0):
    """Panel normal-force slope per radian, referenced to panel area.

    Returns (nominal, lo, hi, transonic_flag).
    """
    def subsonic(Mx):
        b2 = max(1.0 - Mx*Mx, 1e-6)
        return 2*np.pi*ar/(2 + np.sqrt(4 + ar*ar*b2*(1 + np.tan(sweep_half_chord)**2/b2)))

    def supersonic(Mx):
        b = np.sqrt(max(Mx*Mx - 1.0, 1e-9))
        bar = b*ar
        if bar >= 1.0:
            return (4.0/b)*(1.0 - 1.0/(2.0*bar))
        # continuous blend to the slender-wing limit; equals 2*ar at bar = 1
        return (np.pi*ar/2.0)*(1.0 + (4.0/np.pi - 1.0)*bar)

    if M < TRANSONIC[0]:
        v = subsonic(M); return v, v*0.9, v*1.1, False
    if M > TRANSONIC[1]:
        v = supersonic(M); return v, v*0.9, v*1.1, False
    lo_v, hi_v = subsonic(TRANSONIC[0]), supersonic(TRANSONIC[1])
    f = (M - TRANSONIC[0])/(TRANSONIC[1] - TRANSONIC[0])
    v = lo_v + f*(hi_v - lo_v)
    return v, v*(1-TRANSONIC_BAND), v*(1+TRANSONIC_BAND), True


def x_ac_frac(M: float):
    """Chordwise aerodynamic centre as a fraction of c_bar."""
    if M < TRANSONIC[0]: return 0.25
    if M > TRANSONIC[1]: return 0.50
    return 0.25 + 0.25*(M - TRANSONIC[0])/(TRANSONIC[1] - TRANSONIC[0])


def interference(canard: Canard, veh: Vehicle):
    """Barrowman fin-body interference factor."""
    r = 0.5*veh.diameter
    return 1.0 + r/(canard.span + r)


def roll_derivatives(canard: Canard, veh: Vehicle, M: float):
    """(C_ldelta, C_lp_canard, transonic_flag) referenced to S_ref and d."""
    a, _, _, tflag = cn_alpha(M, canard.ar_eff, canard.sweep_half_chord())
    a *= interference(canard, veh)
    r = 0.5*veh.diameter
    ybar = r + canard.y_centroid
    cld = canard.n_fins*a*canard.area*ybar/(veh.s_ref*veh.diameter)

    xi = np.linspace(0.0, canard.span, 400)
    integ = float(_trapz(canard.chord(xi)*(r + xi)**2, xi))
    clp = -2.0*canard.n_fins*a*integ/(veh.s_ref*veh.diameter**2)
    return cld, clp, tflag


def hinge_moment(canard: Canard, veh: Vehicle, M: float, q: float, delta: float):
    """Aerodynamic hinge moment per panel [N m] (positive = restoring)."""
    a, _, _, _ = cn_alpha(M, canard.ar_eff, canard.sweep_half_chord())
    a *= interference(canard, veh)
    arm = (x_ac_frac(M) - canard.hinge_frac)*canard.mac
    return q*canard.area*a*delta*arm


def q_divergence(canard: Canard, veh: Vehicle, M: float, k_delta: float):
    """Torsional divergence dynamic pressure about the hinge [Pa]. inf if the
    hinge is at or ahead of the aerodynamic centre with a restoring arm."""
    a, _, _, _ = cn_alpha(M, canard.ar_eff, canard.sweep_half_chord())
    a *= interference(canard, veh)
    arm = (x_ac_frac(M) - canard.hinge_frac)*canard.mac
    if arm <= 0:
        return np.inf
    return k_delta/(canard.area*a*arm)


# --------------------------------------------------------------- actuator
def available_rate(canard, veh, M, q, delta, servo: Servo, link: Linkage):
    """Deflection rate available at the canard under aerodynamic load [rad/s]."""
    cap = link.refer(servo)
    load = abs(hinge_moment(canard, veh, M, q, delta))
    frac = min(load/cap["torque"], 1.0) if cap["torque"] > 0 else 1.0
    return cap["rate"]*max(1.0 - frac, 0.0), frac


# ------------------------------------------------------- two-body roll plant
def roll_plant(canard, veh: Vehicle, brg: Bearing, M: float, q: float, V: float):
    """Linearised two-body roll dynamics.

        x = [p_fore, p_tail],  xdot = A x + B delta

    Returns dict with A, B, eigenvalues, forebody time constant and the
    steady-state forebody roll rate per radian of canard deflection.
    """
    cld, clp_c, tflag = roll_derivatives(canard, veh, M)
    k = q*veh.s_ref*veh.diameter
    hd = veh.diameter/(2.0*max(V, 1e-6))          # p -> p_hat

    d_fore = k*(clp_c + veh.body_Clp)*hd          # N m per (rad/s)
    d_tail = k*veh.fin_Clp*hd
    cb = brg.c_visc

    A = np.array([[(d_fore - cb)/veh.Ixx_fore,  cb/veh.Ixx_fore],
                  [cb/veh.Ixx_tail,            (d_tail - cb)/veh.Ixx_tail]])
    B = np.array([[k*cld/veh.Ixx_fore], [0.0]])
    eig = np.linalg.eigvals(A)
    try:
        pss = float(np.linalg.solve(-A, B.flatten())[0])
    except np.linalg.LinAlgError:
        pss = np.inf
    slow = float(np.min(np.abs(eig.real))) if np.all(eig.real < 0) else 0.0
    return dict(A=A, B=B, eig=eig, tau_fore=(1.0/slow if slow > 0 else np.inf),
                p_ss_per_rad=pss, C_ldelta=cld, C_lp_canard=clp_c,
                pdot_per_rad=float(B[0, 0]), transonic=tflag)


# ------------------------------------------------------------- constraints
def evaluate(canard: Canard, veh: Vehicle, servo: Servo, link: Linkage,
             brg: Bearing, envelope, req):
    """Evaluate one design over a flight envelope.

    Returns per-point results plus normalised margins (>1 = satisfied) and the
    maximin screening score.  Read `MARGIN_NOTES` before treating any of these
    as a pass mark -- several are indicators, one is pure policy, and only the
    geometric ones are exact.
    """
    cap = link.refer(servo)
    dmax = np.radians(req["delta_max_deg"])
    mu = 1.79e-5                                   # dynamic viscosity, sea level
    pts = []

    worst = dict(authority=np.inf, rate=np.inf, granularity=np.inf, torque=np.inf,
                 divergence=np.inf, root_bending=np.inf)
    sm_worst = np.inf
    J_total = canard.inertia_about_hinge() + link.ratio**2*servo.J_out

    # --- steady roll disturbance the canards must trim -----------------------
    # Tail cant reaches the forebody only through the bearing; canard rigging
    # error is always on the forebody.  Both are q-independent AS A DEFLECTION,
    # because disturbance and control scale with q together.
    cl_dist_tail = veh.fin_Cldelta*np.radians(veh.fin_cant_deg)
    cl_dist_can = np.radians(veh.canard_rig_error_deg)      # per unit C_ldelta

    for e in envelope:
        _, _, rho, a_s = isa(e["alt"])
        V = e["V"]; M = V/a_s; q = 0.5*rho*V*V
        pl = roll_plant(canard, veh, brg, M, q, V)
        cld = pl["C_ldelta"]

        # deflection to trim the steady disturbance (q-independent)
        d_trim_rigid = (cl_dist_tail/cld + cl_dist_can) if cld > 0 else np.inf
        d_trim_free = cl_dist_can
        d_trim = d_trim_rigid if brg.c_visc > 0 else d_trim_free
        # deflection for the commanded manoeuvre acceleration
        d_man = (np.radians(req["pdot_req_deg"])/pl["pdot_per_rad"]) if pl["pdot_per_rad"] > 0 else np.inf
        d_req = d_trim + d_man

        # error floor
        step = cap["deadband"] + cap["backlash"]
        pdot_gran = pl["pdot_per_rad"]*step
        p_lc = pdot_gran/req["omega_c"]
        b_half = 0.5*cap["backlash"]
        a_over_b = (min(d_req, dmax)/b_half) if b_half > 0 else np.inf
        _, bl_phase = backlash_df(a_over_b) if np.isfinite(a_over_b) else (1.0, 0.0)

        # actuator: rate demanded by the loop, and torque including inertia
        d_rate_req = req["omega_c"]*min(d_man, dmax)
        d_accel_req = req["omega_c"]**2*min(d_man, dmax)
        h_dmax = hinge_moment(canard, veh, M, q, dmax)
        t_req_op = abs(hinge_moment(canard, veh, M, q, min(d_req, dmax))) + J_total*d_accel_req
        frac_max = min(abs(h_dmax)/cap["torque"], 1.0) if cap["torque"] > 0 else 1.0
        rate_avail, _ = available_rate(canard, veh, M, q, min(d_req, dmax), servo, link)

        qdiv = q_divergence(canard, veh, M, cap["stiffness"])

        # root bending moment on the panel at full deflection
        a_cn = cn_alpha(M, canard.ar_eff, canard.sweep_half_chord())[0]*interference(canard, veh)
        m_root = q*canard.area*a_cn*dmax*canard.y_centroid

        # static margin in calibres, canards included
        cna_can = canard.n_fins*a_cn*canard.area/veh.s_ref
        cna_tot = cna_can + veh.cna_rest
        x_cp = (veh.cna_rest*veh.x_cp_rest + cna_can*veh.x_canard)/cna_tot
        sm_cal = (x_cp - veh.x_cg)/veh.diameter

        re_c = rho*V*canard.mac/mu

        worst["authority"] = min(worst["authority"], dmax/d_req if d_req > 0 else np.inf)
        worst["rate"] = min(worst["rate"], rate_avail/d_rate_req if d_rate_req > 0 else np.inf)
        worst["granularity"] = min(worst["granularity"],
                                   np.radians(req["p_floor_budget_deg"])/p_lc if p_lc > 0 else np.inf)
        worst["torque"] = min(worst["torque"], 1.0/frac_max if frac_max > 0 else np.inf)
        worst["divergence"] = min(worst["divergence"],
                                  qdiv/(q*req["div_margin"]) if np.isfinite(qdiv) else np.inf)
        worst["root_bending"] = min(worst["root_bending"],
                                    req["m_root_allow"]/m_root if m_root > 0 else np.inf)
        sm_worst = min(sm_worst, sm_cal)

        pts.append(dict(name=e["name"], M=float(M), q=float(q), V=float(V),
                        C_ldelta=cld, C_lp_canard=pl["C_lp_canard"],
                        pdot_per_rad=pl["pdot_per_rad"],
                        pdot_at_dmax_deg=float(np.degrees(pl["pdot_per_rad"]*dmax)),
                        delta_trim_deg=float(np.degrees(d_trim)),
                        delta_man_deg=float(np.degrees(d_man)),
                        delta_req_deg=float(np.degrees(d_req)),
                        pdot_granularity_deg=float(np.degrees(pdot_gran)),
                        p_floor_deg=float(np.degrees(p_lc)),
                        backlash_A_over_b=float(a_over_b),
                        backlash_phase_lag_deg=float(bl_phase),
                        rate_req_deg=float(np.degrees(d_rate_req)),
                        rate_avail_deg=float(np.degrees(rate_avail)),
                        hinge_Nm_at_dmax=float(h_dmax),
                        torque_req_op_Nm=float(t_req_op),
                        torque_frac_at_dmax=float(frac_max),
                        m_root_Nm=float(m_root),
                        static_margin_cal=float(sm_cal),
                        reynolds=float(re_c),
                        q_div_kPa=float(qdiv/1000.0),
                        tau_fore=pl["tau_fore"],
                        p_ss_deg_per_rad=float(np.degrees(pl["p_ss_per_rad"])),
                        transonic=pl["transonic"]))

    margins = dict(authority=worst["authority"], rate=worst["rate"],
                   granularity=worst["granularity"], torque=worst["torque"],
                   divergence=worst["divergence"],
                   span=req["span_max"]/canard.span,
                   static_margin=sm_worst/req["sm_min_cal"],
                   root_bending=worst["root_bending"])
    score = float(min(margins.values()))
    return dict(points=pts, margins={k: float(v) for k, v in margins.items()},
                score=score, capability=cap,
                geometry=dict(S_f_mm2=canard.area*1e6, AR_eff=canard.ar_eff,
                              MAC_mm=canard.mac*1e3, taper=canard.taper,
                              y_centroid_mm=canard.y_centroid*1e3,
                              J_hinge=canard.inertia_about_hinge(),
                              J_total_at_canard=J_total,
                              S_delta=canard.static_unbalance(),
                              mass_g=canard.mass()*1e3))


MARGIN_NOTES = {
    "authority": "EXACT given the disturbance model. delta_trim is q-INDEPENDENT "
                 "(disturbance and control both scale with q); only the manoeuvre "
                 "part varies. Weak point: linear C_Nalpha to delta_max, and no "
                 "Reynolds correction at the low-q end where this binds.",
    "rate": "Demand estimated as omega_c * delta_manoeuvre. Scales as 1/n, so it is "
            "what should stop you raising the linkage ratio without limit.",
    "granularity": "SCALING LAW, NOT A BOUND. See ADR-001 section 2.2. Rank with it; "
                   "do not treat 1.0 as a pass mark.",
    "torque": "A LIMIT CHECK, not an operating condition -- it asks whether the servo "
              "can hold delta_max at max q. Normal operation uses a few percent. "
              "Inertial term included via J_total = J_canard + n^2 J_servo_out.",
    "divergence": "Static divergence about the hinge with a LINEAR spring. Real "
                  "stiffness is ~zero inside the servo deadband and high outside, so "
                  "this is not a single number. Binary flutter can occur below the "
                  "divergence speed; check S_delta (mass balance) and run a GVT.",
    "span": "NOT A COMPUTED MARGIN -- it is the policy cap you set, standing in for "
            "canard-tail vortex interference, which is not semi-empirically predictable.",
    "static_margin": "Barrowman-style CP shift in calibres with the canards included. "
                     "Depends entirely on x_cg, x_canard, x_cp_rest and cna_rest being "
                     "right; it is only as good as those four inputs.",
    "root_bending": "Panel root bending moment at delta_max against an allowable you "
                    "set. Shaft torsion is the hinge moment; check both.",
}

DEFAULT_REQ = dict(
    delta_max_deg=12.0,          # mechanical/stall deflection limit
    pdot_req_deg=500.0,          # roll acceleration the loop must be able to command
    p_floor_budget_deg=5.0,      # tolerable residual roll-rate error [deg/s]
    sm_min_cal=2.0,              # minimum static margin [calibres]
    m_root_allow=3.0,            # allowable panel root bending moment [N m]
    omega_c=30.0,                # ASSUMED closed-loop crossover [rad/s]. Replace
                                 # with the real figure from the MATLAB design --
                                 # the error floor scales as 1/omega_c.
    phase_budget_deg=20.0,       # phase margin you are willing to spend on delay
    div_margin=2.0,              # required q_div / q
    span_max=0.030,              # exposed semi-span cap (sign-reversal mitigation)
)

DEFAULT_ENVELOPE = [
    dict(name="rail exit", alt=200.0, V=30.0),
    dict(name="mid boost", alt=500.0, V=150.0),
    dict(name="max q", alt=1200.0, V=270.0),
    dict(name="burnout", alt=2000.0, V=310.0),
    dict(name="coast", alt=3000.0, V=150.0),
]


def sweep(veh, servo, link, brg, envelope, req, spans, roots, taper, **kw):
    """Grid over (exposed semi-span, root chord). Returns arrays of score and margins."""
    S = np.zeros((len(spans), len(roots)))
    M = {k: np.zeros_like(S) for k in
         ("authority", "granularity", "torque", "divergence", "span", "static_margin")}
    A = np.zeros_like(S)
    for i, s in enumerate(spans):
        for j, cr in enumerate(roots):
            c = Canard(span=s, root_chord=cr, tip_chord=taper*cr, **kw)
            r = evaluate(c, veh, servo, link, brg, envelope, req)
            S[i, j] = r["score"]; A[i, j] = c.area*1e6
            for k in M: M[k][i, j] = r["margins"][k]
    return S, M, A


def export(canard, veh, servo, link, brg, result, path):
    """Write the parameter set the MATLAB stability loop consumes."""
    out = dict(canard=asdict(canard), vehicle=asdict(veh), servo=asdict(servo),
               linkage=asdict(link), bearing=asdict(brg),
               capability={k: float(v) for k, v in result["capability"].items()},
               geometry=result["geometry"], margins=result["margins"],
               score=result["score"], points=result["points"],
               note="C_ldelta and C_lp are referenced to S_ref = pi d^2/4 and "
                    "length d, with p_hat = p d / (2 V). Points flagged "
                    "transonic=true carry +/-30% uncertainty and no sign-reversal model.")
    with open(path, "w") as fh:
        json.dump(out, fh, indent=2)
    return path
