function delta = canard_servo(delta_cmd, P)
%CANARD_SERVO  Nonlinear servo + linkage + gear lash, one fixed rate.
%
%   delta = canard_servo(delta_cmd, P)
%
%   delta_cmd  commanded canard deflection  [rad]
%   P          the struct from PARAMETERS_INITIALISE_ROLL_CONTROL
%   delta      actual canard deflection     [rad]
%
%   Drop-in replacement for the Aerospace Blockset Nonlinear Second-Order
%   Actuator: one input, one output.  Put it in a MATLAB Function block with a
%   DISCRETE sample time of P.Servo.dt.  Everything is explicit, so there is no
%   solver chatter, no zero-crossing storm, and the structure matches what runs
%   on the H743.
%
%   The aerodynamic hinge moment is computed internally from P and the surface's
%   own current deflection, so nothing else in the model needs rewiring and no
%   algebraic loop is created.
%
%   Nonlinearities, in the order the physical signal meets them:
%     1  command quantisation      (12-bit controller)
%     2  transport delay           (frame + processing)
%     3  dead zone ON THE POSITION ERROR, inside the loop -- NOT on the command.
%        A dead zone on the command is a different and wrong system: it would
%        ignore small commands but track perfectly once outside them.  The real
%        servo stops driving when ITS OWN error is small, which is what produces
%        hang-off.
%     4  second-order position loop
%     5  acceleration limit        (stall torque / total reflected inertia)
%     6  rate limit DERATED BY LOAD, off the DC torque-speed line -- the coupling
%        the Aerospace Blockset actuator does not have
%     7  travel limit
%     8  gear/linkage backlash between the gearbox and the surface
%
%   Servo-side quantities are referred to the canard through n = P.Linkage.n:
%   torque x n, rate / n, dead zone / n, quantum / n, inertia x n^2.
%
%   SIMULINK SETUP -- see setup_actuator_block.m, or by hand:
%     * paste this whole file (main function + both local functions) into a
%       MATLAB Function block, exactly as you did for plant and controller;
%     * in the Symbols pane set P to scope PARAMETER, so it resolves from the
%       workspace and does NOT become an input port. The block is then one in,
%       one out -- a true drop-in for the Aerospace Blockset actuator;
%     * set the block SAMPLE TIME to P.Servo.dt. It must be discrete and
%       explicit: this function holds persistent state, which is meaningless
%       on an inherited continuous sample time.
%
%   Note the transport delay is inside this function. Do not also place a
%   Transport Delay block in the path.

%#codegen
% Fixed buffer length so the block is codegen-safe. It bounds the transport
% delay: Tau_d/dt must stay below NBUF-1. At dt = 1e-4 that is 204 ms, which is
% two orders clear of anything real -- but the assert below catches it rather
% than silently truncating the delay if you ever shorten dt.
NBUF = 2048;

% Geometry-derived constants are cached on the first call. Simulink resets
% persistents at the start of every sim run, so a Monte Carlo that varies P
% re-derives them per run.
persistent th thd buf widx lash_pos init J_tot K_H
if isempty(init)
    th = 0; thd = 0; lash_pos = 0; widx = 1;
    buf = zeros(1, NBUF);
    J_tot = canard_inertia(P) + P.Linkage.n^2 * P.Servo.J_out;
    K_H   = canard_hinge_coeff(P);          % H = K_H * q * delta
    init = true;
end

n   = P.Linkage.n;
eta = P.Linkage.eta;
dt  = P.Servo.dt;

% ---- 1: quantise the command (quantum referred to the canard) -------------
q_cmd = P.Servo.Res / n;
u_q   = round(delta_cmd / q_cmd) * q_cmd;

% ---- 2: transport delay through a circular buffer -------------------------
nd = max(round(P.Servo.Tau_d / dt), 1);
assert(nd <= NBUF - 1, ...
    'canard_servo: Tau_d/dt exceeds the delay buffer -- raise NBUF or raise dt.');
buf(widx) = u_q;
ridx = widx - nd;
if ridx < 1, ridx = ridx + NBUF; end
cmd_d = buf(ridx);
widx = widx + 1;
if widx > NBUF, widx = 1; end

% ---- 3: dead zone on the POSITION ERROR -----------------------------------
dz = P.Servo.Deadband / n;
e  = cmd_d - th;
if abs(e) <= dz
    e_dz = 0;
else
    e_dz = e - sign(e)*dz;
end

% ---- 6: load-dependent rate limit (needs the hinge moment first) ----------
H       = K_H * P.Aero.q * lash_pos;
T_avail = n * P.Servo.T_stall * eta;
frac    = min(abs(H)/T_avail, 1);
rate_max = (P.Servo.Rate / n) * max(1 - frac, 0);

% ---- 4 + 5: second-order loop with an acceleration limit ------------------
a_max = T_avail / J_tot;
thdd  = P.Servo.f_N^2 * e_dz - 2*P.Servo.Zeta*P.Servo.f_N*thd;
thdd  = max(min(thdd, a_max), -a_max);

thd = thd + thdd*dt;
thd = max(min(thd, rate_max), -rate_max);
th  = th + thd*dt;

% ---- 7: travel limit ------------------------------------------------------
th_max = P.Servo.Travel;
if th >  th_max, th =  th_max; thd = 0; end
if th < -th_max, th = -th_max; thd = 0; end

% ---- 8: backlash -- the surface is dragged only when a flank is engaged ----
b = 0.5 * P.Servo.Lash / n;
if th - lash_pos > b
    lash_pos = th - b;
elseif lash_pos - th > b
    lash_pos = th + b;
end

delta = lash_pos;
end

% ==========================================================================
function K_H = canard_hinge_coeff(P)
%CANARD_HINGE_COEFF  Hinge moment on ONE panel is H = K_H * q * delta  [N m].
%   Same low-AR lift slope and interference factor as the plant function, so
%   the actuator and the plant cannot drift apart.
S_c  = (P.Vehicle.C_r + P.Vehicle.C_t) * P.Vehicle.s * 0.5;
AR   = P.Vehicle.s^2 / S_c;
r    = P.Vehicle.d / 2;
mach = P.Vehicle.V / 343;
beta = sqrt(max(1 - mach^2, 1e-6));
a    = 2*pi*AR / (2 + sqrt(4 + (AR*beta)^2));
K_fb = 1 + r/(P.Vehicle.s + r);

lam = P.Vehicle.C_t / P.Vehicle.C_r;
mac = (2/3) * P.Vehicle.C_r * (1 + lam + lam^2) / (1 + lam);
arm = (P.Canard.x_ac - P.Canard.hinge_frac) * mac;

K_H = S_c * K_fb * a * arm;
end

% ==========================================================================
function J = canard_inertia(P)
%CANARD_INERTIA  Panel inertia about the hinge line [kg m^2].
%   Solid section of equivalent thickness 0.7*(t/c)*c; hinge at the same chord
%   fraction at every station.
NQ = 41;
s  = P.Vehicle.s;
J  = 0;
for k = 1:NQ
    xi = (k-1)*s/(NQ-1);
    c  = P.Vehicle.C_r + (P.Vehicle.C_t - P.Vehicle.C_r)*xi/s;
    xh = P.Canard.hinge_frac * c;
    t  = 0.7 * P.Canard.t_c * c;
    f  = P.Canard.rho * t * ((c - xh)^3 + xh^3) / 3;
    w  = (k == 1 || k == NQ) * 0.5 + (k > 1 && k < NQ) * 1.0;
    J  = J + w * f;
end
J = J * s/(NQ-1);
end
