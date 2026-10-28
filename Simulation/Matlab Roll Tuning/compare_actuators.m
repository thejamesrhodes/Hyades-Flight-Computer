function compare_actuators(P, Cl_0)
%COMPARE_ACTUATORS  Which parts of the actuator model can be thrown away?
%
%   compare_actuators                 % uses parameters_initialise_roll_control
%   compare_actuators(P, 0.03)
%
%   Answers three questions with numbers rather than intuition.
%
%   1  ABLATION. Remove one element at a time from the full eight-element model
%      and see what the closed-loop roll rate does. An element that changes
%      nothing is an element you can delete.
%
%      Measured on the current parameters (ref = 0, Cl_0 = 0.03, 7.4 V servo):
%        all eight               p = 0.2444   p pp = 0.2028
%        no acceleration limit   p = 0.2444   p pp = 0.2028   <- identical, drop
%        no load derating        p = 0.2444   p pp = 0.2028   <- identical, drop
%        no rate limit           p = 0.2444   p pp = 0.2028   <- identical, but
%                                                                keep: it is free
%        no quantiser            p = 0.2175   p pp = 0.0000   <- DO NOT DROP
%
%      The quantiser is the surprise. It is seven times finer than the dead zone
%      and looks irrelevant, but it does not compete with the dead zone -- it
%      hunts against the trim integrator, and it is the only thing in the model
%      producing residual roll-rate ripple. Note the magnitude is a lattice
%      effect and is not smooth in Res, which is an argument for dispersing it
%      in Monte Carlo rather than trusting any single value.
%
%   2  MERGE TEST. Open-loop gain against drive amplitude for the full model and
%      for two ways of collapsing dead zone + backlash into one backlash block.
%      Width (dz + lash) is the obvious choice and is badly wrong at small
%      amplitude. Width (2*dz + lash) is the kinematically correct one.
%
%   3  CROSS-CHECK against CANARD_SERVO itself, so the local transcription used
%      here is provably the same system.

if nargin < 1 || isempty(P),    P    = parameters_initialise_roll_control(); end
if nargin < 2 || isempty(Cl_0), Cl_0 = 0.03; end
DEG = pi/180;

% ------------------------------------------------------- 3. cross-check first
clear canard_servo
dt = P.Servo.dt; u = 1*DEG*ones(1, round(0.5/dt));
a = zeros(size(u));
for k = 1:numel(u), a(k) = canard_servo(u(k), P); end
b = trace_variant('full', u, P);
fprintf('cross-check vs canard_servo.m : max |diff| = %.3e rad\n', max(abs(a-b)));
fprintf('Cl_delta = %.4f, Cl_p = %.4f, trim needed for Cl_0 = %.3f is %.2f deg ', ...
        plant_cld(P), plant_clp(P), Cl_0, rad2deg(Cl_0/plant_cld(P)));
fprintf('(clamp %.2f deg)\n\n', rad2deg(P.Trim.Clamp));

% ------------------------------------------------------------- 1. ablation
fprintf('=== ablation: closed loop, ref = %.1f rad/s, Cl_0 = %.3f ===\n', ...
        P.Controller.ref, Cl_0);
fprintf('%-26s%10s%12s%14s\n', 'model', 'p mean', 'p pp rad/s', 'delta pp deg');
opts = {'all eight',            struct(); ...
        'no quantiser',         struct('noquant',1); ...
        'no acceleration limit',struct('noaccel',1); ...
        'no rate limit',        struct('norate',1); ...
        'no load derating',     struct('noderate',1); ...
        'none of the four',     struct('noquant',1,'noaccel',1,'norate',1,'noderate',1)};
for i = 1:size(opts,1)
    m = closed_loop(P, Cl_0, opts{i,2});
    fprintf('%-26s%10.4f%12.4f%14.4f\n', opts{i,1}, m.p_mean, m.p_pp, m.d_pp);
end

% ------------------------------------------------------------ 2. merge test
fprintf('\n=== open loop: gain at 5 Hz vs drive amplitude ===\n');
names = {'FULL','R1 no accel/derate','R2 merged dz+lash','R3 merged 2dz+lash'};
modes = {'full','r1','r2','r3'};
fprintf('%8s', 'amp deg'); fprintf('%22s', names{:}); fprintf('\n');
t = 0:dt:2; h = floor(numel(t)/2);
for amp = [0.1 0.2 0.3 0.5 1.0 2.0 5.0 20.0]
    u = amp*DEG*sin(2*pi*5*t); g = zeros(1,4);
    for i = 1:4
        o = trace_variant(modes{i}, u, P);
        g(i) = sqrt(mean(o(h:end).^2))/sqrt(mean(u(h:end).^2));
    end
    fprintf('%8.1f%13.3f%9s', amp, g(1), '');
    for i = 2:4
        if g(1) < 1e-6
            fprintf('%13.3f%9s', g(i), '--');
        else
            fprintf('%13.3f%8.0f%%', g(i), 100*(g(i)-g(1))/g(1));
        end
    end
    fprintf('\n');
end
fprintf(['\nR2 is the trap. At 0.3 deg drive -- the amplitude the loop lives at --\n' ...
         'it passes several times what the real actuator passes, so the model\n' ...
         'looks better damped than the hardware. Use R1, or R3 if a single block\n' ...
         'is unavoidable.\n']);
end

% =========================================================================
function y = trace_variant(mode, u, P)
n = P.Linkage.n; dt = P.Servo.dt; dz = P.Servo.Deadband/n;
switch mode
    case {'full','r1'}, b = 0.5*P.Servo.Lash/n;          usedz = true;
    case 'r2',          b = 0.5*(dz + P.Servo.Lash/n);   usedz = false;
    case 'r3',          b = 0.5*(2*dz + P.Servo.Lash/n); usedz = false;
end
nd = max(round(P.Servo.Tau_d/dt),1); NB = 2048; buf = zeros(1,NB); widx = 1;
th = 0; thd = 0; lp = 0;
J = canard_inertia_local(P) + n^2*P.Servo.J_out;
Tav = n*P.Servo.T_stall*P.Linkage.eta; amax = Tav/J;
KHq = hinge_local(P)*P.Aero.q;
y = zeros(size(u));
for k = 1:numel(u)
    c = u(k);
    if strcmp(mode,'full'), q = P.Servo.Res/n; c = round(c/q)*q; end
    buf(widx) = c; ridx = widx - nd;
    if ridx < 1, ridx = ridx + NB; end
    cd = buf(ridx); widx = widx + 1; if widx > NB, widx = 1; end
    e = cd - th;
    if usedz
        if abs(e) <= dz, edz = 0; else, edz = e - sign(e)*dz; end
    else
        edz = e;
    end
    thdd = P.Servo.f_N^2*edz - 2*P.Servo.Zeta*P.Servo.f_N*thd;
    if strcmp(mode,'full')
        thdd = max(min(thdd,amax),-amax);
        rmax = (P.Servo.Rate/n)*max(1-min(abs(KHq*lp)/Tav,1),0);
    else
        rmax = P.Servo.Rate/n;
    end
    thd = max(min(thd + thdd*dt, rmax), -rmax);
    th  = th + thd*dt;
    if th >  P.Servo.Travel, th =  P.Servo.Travel; thd = 0; end
    if th < -P.Servo.Travel, th = -P.Servo.Travel; thd = 0; end
    if th - lp > b, lp = th - b; elseif lp - th > b, lp = th + b; end
    y(k) = lp;
end
end

% =========================================================================
function m = closed_loop(P, Cl_0, o)
DEG = pi/180; dt = P.Servo.dt; T = 20; n = P.Linkage.n;
if ~isfield(o,'noquant'),  o.noquant  = 0; end
if ~isfield(o,'noaccel'),  o.noaccel  = 0; end
if ~isfield(o,'norate'),   o.norate   = 0; end
if ~isfield(o,'noderate'), o.noderate = 0; end
Cld = plant_cld(P); Clp = plant_clp(P); A = pi*(P.Vehicle.d/2)^2;
Kp  = P.Controller.c/P.Controller.q_e;
dz  = P.Servo.Deadband/n; eb = P.Trim.BandK*dz/max(Kp,eps);
nd  = max(round(P.Servo.Tau_d/dt),1); NB = 2048; buf = zeros(1,NB); widx = 1;
th = 0; thd = 0; lp = 0; p = 0; trim = 0; b = 0.5*P.Servo.Lash/n;
J = canard_inertia_local(P) + n^2*P.Servo.J_out;
Tav = n*P.Servo.T_stall*P.Linkage.eta; amax = Tav/J; KHq = hinge_local(P)*P.Aero.q;
N = round(T/dt); pv = zeros(1,N); dv = zeros(1,N);
for k = 1:N
    e  = P.Controller.ref - p;
    dd = Kp*e;
    dtot = max(min(dd+trim, P.Servo.Travel), -P.Servo.Travel);
    if abs(e) > eb, drive = sign(e)*(abs(e)-eb); else, drive = 0; end
    if abs(dd+trim) < P.Servo.Travel*0.999
        st = max(min(P.Trim.Ki*drive, P.Trim.Rate), -P.Trim.Rate);
        trim = max(min(trim + st*dt, P.Trim.Clamp), -P.Trim.Clamp);
    end
    c = dtot;
    if ~o.noquant, q = P.Servo.Res/n; c = round(c/q)*q; end
    buf(widx) = c; ridx = widx - nd;
    if ridx < 1, ridx = ridx + NB; end
    cd = buf(ridx); widx = widx + 1; if widx > NB, widx = 1; end
    e2 = cd - th;
    if abs(e2) <= dz, edz = 0; else, edz = e2 - sign(e2)*dz; end
    thdd = P.Servo.f_N^2*edz - 2*P.Servo.Zeta*P.Servo.f_N*thd;
    if ~o.noaccel, thdd = max(min(thdd,amax),-amax); end
    if o.norate
        rmax = inf;
    elseif o.noderate
        rmax = P.Servo.Rate/n;
    else
        rmax = (P.Servo.Rate/n)*max(1-min(abs(KHq*lp)/Tav,1),0);
    end
    thd = max(min(thd+thdd*dt,rmax),-rmax);
    th  = th + thd*dt;
    if th >  P.Servo.Travel, th =  P.Servo.Travel; thd = 0; end
    if th < -P.Servo.Travel, th = -P.Servo.Travel; thd = 0; end
    if th-lp > b, lp = th-b; elseif lp-th > b, lp = th+b; end
    pdot = (P.Aero.q*A*P.Vehicle.d*(Cld*lp + Cl_0) + ...
            P.Aero.q*A*P.Vehicle.d^2*Clp*p/(2*P.Vehicle.V)) / P.Vehicle.Ixx;
    p = p + pdot*dt;
    pv(k) = p; dv(k) = lp;
end
w = round(6/dt);
m.p_mean = mean(pv(end-w:end));
m.p_pp   = max(pv(end-w:end)) - min(pv(end-w:end));
m.d_pp   = (max(dv(end-w:end)) - min(dv(end-w:end)))/DEG;
end

% =========================================================================
function a = lift_slope(P)
S_c = (P.Vehicle.C_r + P.Vehicle.C_t)*P.Vehicle.s*0.5;
AR  = P.Vehicle.s^2/S_c;
beta = sqrt(max(1-(P.Vehicle.V/343)^2,1e-6));
a = 2*pi*AR/(2+sqrt(4+(AR*beta)^2));
end

function Cld = plant_cld(P)
S_c = (P.Vehicle.C_r + P.Vehicle.C_t)*P.Vehicle.s*0.5;
r = P.Vehicle.d/2; A = pi*r^2; K = 1 + r/(P.Vehicle.s+r);
Cld = K*P.Vehicle.N*lift_slope(P)*(S_c/A)*(P.Aero.y_Cp/P.Vehicle.d);
end

function Clp = plant_clp(P)
r = P.Vehicle.d/2; A = pi*r^2; s = P.Vehicle.s;
K = 1 + r/(s+r); k = (P.Vehicle.C_t-P.Vehicle.C_r)/s;
I3 = ((r+s)^3-r^3)/3; I4 = ((r+s)^4-r^4)/4;
Clp = -2*K*P.Vehicle.N*lift_slope(P)*(P.Vehicle.C_r*I3 + k*(I4-r*I3)) ...
      /(A*P.Vehicle.d^2) - 2*P.Aero.C_f*(P.Vehicle.L/P.Vehicle.d);
end

function K_H = hinge_local(P)
S_c = (P.Vehicle.C_r + P.Vehicle.C_t)*P.Vehicle.s*0.5;
r = P.Vehicle.d/2; K_fb = 1 + r/(P.Vehicle.s+r);
lam = P.Vehicle.C_t/P.Vehicle.C_r;
mac = (2/3)*P.Vehicle.C_r*(1+lam+lam^2)/(1+lam);
K_H = S_c*K_fb*lift_slope(P)*(P.Canard.x_ac - P.Canard.hinge_frac)*mac;
end

function J = canard_inertia_local(P)
NQ = 41; s = P.Vehicle.s; J = 0;
for k = 1:NQ
    xi = (k-1)*s/(NQ-1);
    c  = P.Vehicle.C_r + (P.Vehicle.C_t-P.Vehicle.C_r)*xi/s;
    xh = P.Canard.hinge_frac*c;
    t  = 0.7*P.Canard.t_c*c;
    f  = P.Canard.rho*t*((c-xh)^3 + xh^3)/3;
    if k == 1 || k == NQ, w = 0.5; else, w = 1.0; end
    J  = J + w*f;
end
J = J*s/(NQ-1);
end
