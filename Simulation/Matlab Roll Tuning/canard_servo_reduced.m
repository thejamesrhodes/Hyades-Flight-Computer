function delta = canard_servo_reduced(delta_cmd, P)
%CANARD_SERVO_REDUCED  Drop-in for CANARD_SERVO with the aero coupling removed.
%
%   Same signature, same P, same sample time. Two of the eight elements in
%   CANARD_SERVO are gone. Both were measured, not assumed, at the current
%   operating point (see COMPARE_ACTUATORS):
%
%     REMOVED  acceleration limit.  a_max = n*T_stall*eta/J_tot ~ 5.8e3 rad/s^2
%              against a peak demand of ~6.4e2. Removing it leaves the
%              closed-loop roll rate bit identical: 0.2444 both ways.
%     REMOVED  load derating of the rate limit.  The hinge moment at 40 kPa and
%              full travel is 3.7 mN m against ~390 mN m available: under 1%.
%              And it derates a limiter that never engages -- peak slew is
%              127 deg/s against ~1100 deg/s available. Also bit identical.
%
%   KEPT: quantisation, transport delay, dead zone on the position error,
%   second-order loop, fixed rate limit, travel limit, backlash.
%
%   WHY THIS IS THE USEFUL CUT rather than a bigger one: the load derating is
%   the ONLY element that needs the aerodynamic hinge moment, and therefore the
%   only reason this model has to know the canard geometry, the flight
%   condition, or its own past output through K_H. With it gone the actuator is
%   a plain signal chain that maps one-for-one onto native Simulink blocks --
%   see BUILD_REDUCED_ACTUATOR. That is where the sim-speed win is: a native
%   chain compiles under Accelerator, a MATLAB Function block with persistents
%   does not.
%
%   DO NOT ALSO DROP THE QUANTISER. It looks droppable -- the quantum is
%   0.0292 deg against a 0.2005 deg dead zone, seven times finer -- and that
%   argument is wrong. The quantiser does not compete with the dead zone; it
%   hunts against the TRIM INTEGRATOR, which is continuous. It is the sole
%   source of the residual roll-rate ripple: 0.203 rad/s peak to peak with it,
%   0.000 without, and it shifts the mean rate by 0.027 rad/s. Treat the magnitude with suspicion though -- it is a lattice
%   effect, so it is not smooth in the parameters (7.3 deg/s pp at Res = 5.1e-4,
%   0.35 deg/s at Res = 1.0e-3). That non-smoothness is precisely why it belongs
%   in the Monte Carlo rather than in a hand calculation.
%
%   DO NOT MERGE THE DEAD ZONE INTO THE BACKLASH. Replacing both with one
%   backlash of width (Deadband + Lash) overstates the gain at 0.3 deg drive by
%   387%, which is the amplitude the loop actually lives at. If a single block
%   is unavoidable the kinematically correct width is 2*Deadband + Lash: on a
%   reversal the command must cross the dead zone twice before the output moves.
%   COMPARE_ACTUATORS prints both.

%#codegen
NBUF = 2048;
persistent th thd buf widx lash_pos init
if isempty(init)
    th = 0; thd = 0; lash_pos = 0; widx = 1;
    buf = zeros(1, NBUF);
    init = true;
end

n  = P.Linkage.n;
dt = P.Servo.dt;

% ---- 1: command quantisation ---------------------------------------------
q_cmd = P.Servo.Res / n;
u_q   = round(delta_cmd / q_cmd) * q_cmd;

% ---- 2: transport delay ---------------------------------------------------
nd = max(round(P.Servo.Tau_d / dt), 1);
assert(nd <= NBUF - 1, ...
    'canard_servo_reduced: Tau_d/dt exceeds the delay buffer.');
buf(widx) = u_q;
ridx = widx - nd;
if ridx < 1, ridx = ridx + NBUF; end
cmd_d = buf(ridx);
widx = widx + 1;
if widx > NBUF, widx = 1; end

% ---- 3: dead zone on the POSITION ERROR, inside the loop ------------------
dz = P.Servo.Deadband / n;
e  = cmd_d - th;
if abs(e) <= dz
    e_dz = 0;
else
    e_dz = e - sign(e)*dz;
end

% ---- 4: second-order position loop ---------------------------------------
thdd = P.Servo.f_N^2 * e_dz - 2*P.Servo.Zeta*P.Servo.f_N*thd;

% ---- 5: fixed rate limit (never active at present -- kept as a guard) -----
rate_max = P.Servo.Rate / n;
thd = thd + thdd*dt;
thd = max(min(thd, rate_max), -rate_max);
th  = th + thd*dt;

% ---- 6: travel limit ------------------------------------------------------
if th >  P.Servo.Travel, th =  P.Servo.Travel; thd = 0; end
if th < -P.Servo.Travel, th = -P.Servo.Travel; thd = 0; end

% ---- 7: backlash ----------------------------------------------------------
b = 0.5 * P.Servo.Lash / n;
if th - lash_pos > b
    lash_pos = th - b;
elseif lash_pos - th > b
    lash_pos = th + b;
end

delta = lash_pos;
end
