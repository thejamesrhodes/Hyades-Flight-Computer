function P = parameters_initialise_roll_control()

P = struct();

%% Aerodynamics
P.Aero.Cl_0 = 0.00;
P.Aero.q = 6125;
P.Aero.C_f = 0.003;
P.Aero.y_Cp = 0.03975;   %OVERWRITTEN below from the geometry -- see the note
                         % after the Vehicle block. Left here so the section
                         % order is unchanged.

%% Vehicle
P.Vehicle.V = 100;  %Airspeed / m/s
P.Vehicle.Mass = 0.9;   %Mass / kg      STALE -- UPDATE FROM CAD
P.Vehicle.Ixx = 0.0007; %Inertia / kg m^2 STALE -- UPDATE FROM CAD
                        % These predate the current build, which is expected to
                        % come out nearer 1.2-1.5 kg all-up including motor.
                        % Ixx sets the plant gain b = A*d*Cl_delta*q/Ixx
                        % directly, so every gain, margin and Monte Carlo result
                        % below is conditional on it. Take both from the
                        % SolidWorks mass properties of the as-built assembly
                        % before any result from this file is quoted.
                        optimise_roll_schedule.m now reads Ixx
                        % from here rather than keeping its own copy, so fixing
                        % it here fixes the schedule too.
P.Vehicle.d = 0.062;    %Body Diameter / m
P.Vehicle.s = 0.018;     %Exposed Semi-Span / m
P.Vehicle.C_r = 0.024;  %Root Chord / m
P.Vehicle.C_t = 0.012;   %Tip Chord / m
P.Vehicle.N = 4;        %Number of canards
P.Vehicle.L = 0.8;      %Body Length / m

% y_Cp is DERIVED, not a free constant: it is the spanwise area centroid of the
% exposed panel, y_bar = r + s*(C_r + 2*C_t)/(3*(C_r + C_t)).
%
% The 0.03975 above is exactly the value for the OLD canard (s = 0.020,
% C_r = 0.044, C_t = 0.020). It did not follow the geometry down when the panel
% was shrunk to s = 0.012, so Cl_delta has been running 9% high. Recomputed here
% so it cannot go stale again -- change the geometry and this follows.
P.Aero.y_Cp = P.Vehicle.d/2 + P.Vehicle.s*(P.Vehicle.C_r + 2*P.Vehicle.C_t) ...
                            / (3*(P.Vehicle.C_r + P.Vehicle.C_t));

%% Canard section and structure (feeds the hinge moment and the servo inertia)
P.Canard.t_c        = 0.2;   %Thickness ratio t/c
P.Canard.rho        = 1600;   %Panel material density / kg m^-3 (CF laminate)
P.Canard.hinge_frac = 0.2;   %Hinge line as a fraction of MAC
P.Canard.x_ac       = 0.25;   %Chordwise aero centre / MAC (subsonic thin section)

%% Linkage
% n = delta_servo / delta_canard. Everything servo-side is referred to the
% canard through it: torque x n, rate / n, deadband / n, lash / n, inertia x n^2.
% n = 1 is direct drive, i.e. the build as it stands. The sizing bench puts the
% knee at n ~ 4-5: deadband and torque improve, slew rate is what stops you.
P.Linkage.n   = 1;
P.Linkage.eta = 0.85;   %Linkage mechanical efficiency

%% Servo
% Blue Bird BMS-127WV+.  Torque, slew rate, reflected inertia and the
% acceleration limit are DERIVED from the four published (V, stall torque,
% 60 deg time) points -- see servo_params_from_datasheet.  Fields marked
% MEASURE are still assumptions; SERVO_BENCH.md says how to pin them down.
P.Servo.Voltage = 8.4;      %Supply at the servo / V
D = servo_params_from_datasheet(P.Servo.Voltage, struct('quiet',1));

P.Servo.T_stall  = D.T_stall;    %Stall torque / N m            (derived)
P.Servo.Rate     = D.w0;         %No-load slew rate / rad s^-1  (derived: 23.65,
                                 % not the 20.9 the headline 0.05 s/60 deg implies --
                                 % that figure includes a 5.7 ms accel/decel phase)
P.Servo.J_out    = D.J_out;      %Reflected rotor+gearbox inertia / kg m^2 (derived)
P.Servo.Alpha    = D.alpha_max;  %Max output acceleration / rad s^-2 (derived)

P.Servo.f_N      = 80;       %Natural Frequency / rad/s (12.7 Hz)   MEASURE
                             % This is now the single source of truth:
                             % optimise_roll_schedule.m reads it from here. It
                             % used to keep its own 2*pi*20 = 125.7 rad/s, 57%
                             % higher, which moved the phase margin directly.
                             % Still unmeasured -- fit_servo_bench.m settles it,
                             % and note a large-amplitude chirp biases f_N LOW
                             % by ~35%.
P.Servo.Zeta     = 0.7;      %Damping Ratio                MEASURE
P.Servo.Tau_d    = 0.014;    %Transport delay / s          MEASURE
                             % (optimise_roll_schedule.m already uses 14 ms; the
                             %  Simulink actuator had no delay at all)
P.Servo.Deadband = 0.00196;   %Dead zone at the servo shaft / rad   STILL ASSUMED
                             % The manufacturer had NO step-response data and
                             % did not quote a positional dead zone. Their
                             % "62.5-125 us" figure cannot be one: on this
                             % servo's scale (800 us spans 90 deg) that would be
                             % 7-14 deg of dead zone on a unit with 0.115 deg of
                             % backlash. 62.5 us and 125 us are 16 kHz and
                             % 8 kHz -- an internal loop period, not a position
                             % band. 0.0035 rad = 0.2 deg is 1.8 us on the same
                             % scale, typical for a digital servo, but it is a
                             % GUESS and the canard sizing hinges on it.
                             % SERVO_BENCH.md measures it.
                             % 2 us on 90 deg / 800 us = 0.225 deg = 0.0039 rad
P.Servo.Lash     = 0.0020;   %Gear + linkage backlash / rad   MANUFACTURER DATA
                             % Blue Bird, direct from the maker: 0.115 deg,
                             % quoted as "a 10 cm lever arm gives a 0.2 mm gap".
                             % 0.2/100 = 0.0020 rad = 0.1146 deg -- self-
                             % consistent. Replaces the assumed 0.0026 rad
                             % (0.149 deg), so the model was 30% pessimistic on
                             % the single most sensitive parameter in it.
                             % dominant sensitivity in the whole model
P.Servo.Res      = 5.1e-4;   %Command quantum / rad (12 bit, 4096 steps over 120 deg)
P.Servo.Travel   = 0.2094;   %Mechanical travel limit / rad (+/- 12 deg)
P.Servo.I_stall  = 1.3;      %Stall current at 8.4 V / A     MANUFACTURER DATA
                             % 10.9 W per servo at stall; 5.2 A and 43.7 W with
                             % four canards stalled together. A power-budget and
                             % BEC-sizing input, not a control parameter, but it
                             % is the worst case the harness and regulator must
                             % survive.
P.Servo.K_servo  = 30;       %Rotational stiffness at the output / N m rad^-1  MEASURE
P.Servo.dt       = 5e-4;     %Actuator model step / s -- set the MATLAB Function
                             % block's sample time to this

%% Controller
P.Controller.c = 70;   %Proportional q-scheduled constant: Kp = c/q_e
                        % STALE -- RE-DERIVE. This value predates the canard
                        % resize and the schedule fix. optimise_roll_schedule.m
                        % now derives Cl_delta, Cl_p, Ixx and f_N from this file
                        % rather than from its own hard-coded copies, and the
                        % copies it used to hold (Cl_delta = 0.6694 against the
                        % derived 0.2975) made the C it returned 2.25x too low.
                        % Since wc = c*A*d*Cl_delta/Ixx, c is a direct
                        % multiplicative function of Cl_delta.
                        % Do NOT scale c by hand -- the transport delay does not
                        % scale with Cl_delta, so the margin-limited optimum is
                        % not a linear rescale. Re-run optimise_roll_schedule
                        % and take c from its output.
P.Controller.Ti = 7;  %Ratio of integral to proportional gain -- LEGACY PI ONLY.
                      % Has NO effect in the flown configuration (Mode = 1),
                      % where bias rejection comes from the bounded trim path,
                      % not from integral action in the damping loop. Read only
                      % when Mode = 2, which is kept so the Monte Carlo can A/B
                      % the two architectures.
P.Controller.ref = 0; %Target Rate
P.Controller.q_e = P.Aero.q; %Estimated q value
P.Controller.Mode = 1;  %0 = P only, 1 = P + slow trim (default), 2 = legacy PI

%% Trim  (cancels a standing Cl_0 without an integrator in the damping loop)
% delta_ff = -Cl_0/Cl_delta is INDEPENDENT of q and V, so one fixed deflection
% cancels the bias everywhere. Sized against Cl_0 = 0.03 (about 0.21 deg of
% equivalent fin cant), which needs 2.05 deg of trim; the +/-3 deg clamp then
% covers Cl_0 up to ~0.044 and still leaves 9 deg of travel for damping.
P.Trim.Mode     = 2;        %0 = off, 1 = external estimate, 2 = internal integrator
P.Trim.Ki       = 0.0234;   %rad of trim per (rad/s of error) per second.
                            % Sized for ~0.3 Hz at q = 10 kPa, an order below
                            % the 3.3 Hz stick-slip frequency. Uses the
                            % highest-q (fastest) case, so it is slower, never
                            % faster, than 0.3 Hz elsewhere in the envelope.
P.Trim.Clamp    = 0.1047;   %+/- 6 deg.  MUST exceed Cl_0/Cl_delta or the trim
                            % saturates and a standing roll rate survives.
                            % With the current canard Cl_delta = 0.2975 (was
                            % 0.8398 at s = 0.020), so Cl_0 = 0.02 needs 3.85 deg
                            % and Cl_0 = 0.03 needs 5.78 deg. The old +/-3 deg
                            % clamp covered neither. +/-6 deg covers Cl_0 up to
                            % 0.031 and still leaves 6 deg of travel for damping.
P.Trim.Rate     = 0.0524;   %3 deg/s -- clamp reached in ~2 s, as before. This is
                            % gate 4, a bound only; P.Trim.Ki sets the bandwidth.
P.Trim.BandK    = 1.1;      %Dead zone on the integrator input, as a multiple
                            % of deadband/Kp. Below 1.0 a sustained ~0.45 Hz
                            % cycle reappears (band x0.5 -> 0.77 rad/s pp);
                            % 1.0 and above settle flat. Costs a steady
                            % hang-off of roughly +/-5% of the rate command --
                            % that is the actuator's own resolution floor and
                            % the only way to shrink it is P.Linkage.n.
%% Dither  (breaks the dead-zone / quantiser lattice limit cycle)
% Injected on the COMMAND inside CONTROLLER, so it sweeps the servo's internal
% position error across its dead zone and smears the threshold. The surface does
% not follow it -- at 125 Hz against f_N = 80 rad/s (12.7 Hz) the second-order
% actuator attenuates by ~99% -- so it costs no measurable surface activity, and
% measured activity does not rise when dither is on: the hunting it removes
% costs more motion than the dither adds.
%
% Effect, quoted in the RATE domain over four 400-run campaigns at matched
% dispersion: ripple_pp at the 95th percentile falls by an order of magnitude
% between dither off and 1.5x dead zone, with activity flat. The cost shows up
% as a small steady rate bias rather than as oscillation.
%
% Do not re-record the campaign figures in this comment -- the last set went
% stale here without anyone noticing. They belong with the Monte Carlo output.
% The roll-ANGLE ripple figures that used to sit here are WITHDRAWN: that metric
% is the detrended integral of rate over the tail and its resolution guard
% cannot separate "has not converged" from "nothing is oscillating".
%
% Size against the UPPER end of the plausible dead zone, not the nominal:
% Deadband is still unmeasured, and too much dither costs only bias while too
% little leaves the cycle.
%
% FRAME RATE -- OPEN: 125 Hz dither needs roughly 600 Hz of servo frame rate to
% be reconstructed cleanly. Confirm the BMS-127WV+ maximum frame rate; if it
% caps at 333 Hz, drop to 60-80 Hz, still two decades above the roll mode.
P.Dither.On   = 1;
P.Dither.Amp  = 1.5 * P.Servo.Deadband;   % rad at the canard
P.Dither.Freq = 125;                       %Hz

P.Trim.External = 0;        %Trim from an external estimator, used when Mode = 1

end
