function P = parameters_initialise_roll_control()

P = struct();

%% Aerodynamics
P.Aero.Cl_0 = 0;
P.Aero.q = 10000;
P.Aero.C_f = 0.003;
P.Aero.y_Cp = 0.0315;

%% Vehicle
P.Vehicle.V = 100;  %Airspeed / m/s
P.Vehicle.Mass = 0.6;   %Mass / kg
P.Vehicle.Ixx = 0.00061;%Inertia
P.Vehicle.d = 0.062;    %Body Diameter / m
P.Vehicle.s = 0.02;     %Exposed Semi-Span / m
P.Vehicle.C_r = 0.044;  %Root Chord / m
P.Vehicle.C_t = 0.02;   %Tip Chord / m
P.Vehicle.N = 4;        %Number of canards
P.Vehicle.L = 0.8;      %Body Length / m

%% Servo
P.Servo.f_N = 80;      %Natural Frequency / Rad/s
P.Servo.Zeta = 0.7;     %Damping Ratio
P.Servo.Rate = 20.9;      %Rate Limit / Rad/s

%% Controller
P.Controller.c = 120;   %Proportional q schedueled constant
P.Controller.Ti = 7;  %Ratio of Integral gain to proportional gain
P.Controller.ref = 5; %Target Rate
P.Controller.q_e = P.Aero.q; %Estimated q value

end
