function hitl_crosscheck()
%HITL_CROSSCHECK  Reference traces for the HITL Python port (HITL-01 H0b).
%   Runs the SAME code the Simulink model runs -- canard_servo_reduced.m and
%   the 'Plant' MATLAB Function of Simulink_Roll_Model_1 (copied below
%   verbatim as plant_ref) -- on a fixed input set, integrated as the model
%   does (fixed step P.Servo.dt, discrete Euler integrators), and writes
%   Firmware/FC-V4/Tools/hitl_crosscheck_matlab.csv. hitl_crosscheck.py
%   repeats the run with hitl_model.py and reports the RMS difference.
%
%   Inputs (columns): t, cmd (rad), delta (rad, servo output), p (rad/s, roll
%   plant driven by delta at the P.Vehicle.V / P.Aero.q design point).
%   Segments: 0-0.5 s step +5 deg; 0.5-1.0 s step -2 deg; 1.0-3.0 s chirp
%   0.5->20 Hz, 3 deg amplitude; 3.0-3.5 s zero.
here = fileparts(mfilename('fullpath'));
P = parameters_initialise_roll_control();
dt = P.Servo.dt;
t = (0:dt:3.5-dt)';
cmd = zeros(size(t));
cmd(t < 0.5) = 5*pi/180;
cmd(t >= 0.5 & t < 1.0) = -2*pi/180;
k = t >= 1.0 & t < 3.0;
tc = t(k) - 1.0; f0 = 0.5; f1 = 20; T = 2.0;
cmd(k) = 3*pi/180 * sin(2*pi*(f0*tc + (f1-f0)/(2*T)*tc.^2));
clear canard_servo_reduced
delta = zeros(size(t)); p = zeros(size(t)); pk = 0;
for i = 1:numel(t)
    delta(i) = canard_servo_reduced(cmd(i), P);
    p(i) = pk;
    pk = pk + plant_ref(delta(i), pk, P) * dt;
end
out = fullfile(here, '..', '..', 'Firmware', 'FC-V4', 'Tools', 'hitl_crosscheck_matlab.csv');
writematrix([t cmd delta p], out);
fprintf('wrote %s (%d rows)\n', out, numel(t));
end

function p_dot = plant_ref(delta, p, P)
% Verbatim copy of the Simulink_Roll_Model_1 'Plant' MATLAB Function block.
V = P.Vehicle.V; Cl_0 = P.Aero.Cl_0; q = P.Aero.q; d = P.Vehicle.d; Ixx = P.Vehicle.Ixx;
s = P.Vehicle.s; C_r = P.Vehicle.C_r; C_t = P.Vehicle.C_t; N = P.Vehicle.N;
C_f = P.Aero.C_f; L = P.Vehicle.L; y_Cp = P.Aero.y_Cp;
S_c = (C_r + C_t) * s * 0.5; AR = s^2/S_c; r = d/2; A = pi * r^2;
mach = V / 343; beta = sqrt(1 - mach^2);
a = 2*pi*AR / (2 + sqrt(4 + (AR*beta)^2));
K_fb = 1 + r/(s + r);
Cl_delta = K_fb * N * a * (S_c / A) * (y_Cp / d);
k = (C_t - C_r) / s;
I3 = ((r+s)^3 - r^3)/3; I4 = ((r+s)^4 - r^4)/4;
span_int = C_r*I3 + k*(I4 - r*I3);
Cl_p = -2 * C_f * (L/d) + -2 * K_fb * N * a * span_int / (A * d^2);
p_dot = (q*A*d*Cl_delta*delta + (q*A*d^2*Cl_p*p)/(2*V) + q*A*d*Cl_0) / Ixx;
end
