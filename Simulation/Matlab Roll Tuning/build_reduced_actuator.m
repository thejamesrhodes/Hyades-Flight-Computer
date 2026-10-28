function build_reduced_actuator(model, subpath)
%BUILD_REDUCED_ACTUATOR  The actuator as NATIVE Simulink blocks, no MATLAB
%   Function block, no persistent state.
%
%   build_reduced_actuator
%   build_reduced_actuator('Simulink_Roll_Model_1', ...
%                          'Simulink_Roll_Model_1/Actuator Model/Reduced Servo')
%
%   Equivalent to CANARD_SERVO_REDUCED: the same seven elements, in the same
%   order. This is the version to use for Monte Carlo. The reason is not that
%   it is a smaller model -- it is barely smaller -- but that nothing in it
%   needs the hinge moment, so nothing needs MATLAB in the loop. A native chain
%   compiles under Accelerator; a MATLAB Function block holding persistents
%   runs interpreted, once per step, per run.
%
%   Chain:
%
%     In -> Quantizer -> Delay(N) -> Sum(+-) -> DeadZone -> Gain f_N^2 -> Sum(+-)
%        -> DTI [limit +-Rate/n] -> DTI [limit +-Travel] -> Backlash -> Out
%                 |                        |
%                 +-> Gain 2*Zeta*f_N -----+   (velocity feedback, minus)
%                                          |
%           +--------------------------------+ (position feedback to Sum 1, minus)
%
%   Gains are written as P EXPRESSIONS, not baked-in literals, so a Monte Carlo
%   that disperses Deadband, Lash, f_N, Zeta, Rate, Travel or Linkage.n picks
%   the new values up on the next compile without rebuilding the subsystem.
%   Tau_d is the exception: Delay needs an integer length, so it is resolved at
%   build time. If you disperse Tau_d, either re-run this function inside the
%   loop or swap Delay for a Variable Integer Delay driven by
%   round(P.Servo.Tau_d/P.Servo.dt).
%
%   KNOWN DIFFERENCE from CANARD_SERVO_REDUCED: at the travel stops the
%   handwritten model also zeroes the velocity state; the Discrete-Time
%   Integrator only holds its own. They differ by one sample on exit from a
%   hard stop. If your case sits on the stops, drive the velocity integrator's
%   external reset from a saturation detector.
%
%   NOTE: written against the standard Simulink API but NOT executed here --
%   there is no Simulink in this environment. Every add_block names the block
%   it creates, so anything that fails is straightforward to finish by hand.

if nargin < 1 || isempty(model),   model   = 'Simulink_Roll_Model_1'; end
if nargin < 2 || isempty(subpath), subpath = [model '/Actuator Model/Reduced Servo']; end

if ~evalin('base', 'exist(''P'',''var'')')
    error('build_reduced_actuator:noP', ...
          'Run P = parameters_initialise_roll_control(); first.');
end
P = evalin('base', 'P');

load_system(model);
if getSimulinkBlockHandle(subpath) > 0, delete_block(subpath); end
add_block('built-in/Subsystem', subpath);
delete_line(subpath, 'In1/1', 'Out1/1');
S = @(name) [subpath '/' name];

nd = max(round(P.Servo.Tau_d / P.Servo.dt), 1);
dt = 'P.Servo.dt';

add_block('simulink/Discontinuities/Quantizer', S('Quant'), ...
          'QuantizationInterval', 'P.Servo.Res/P.Linkage.n');
add_block('simulink/Discrete/Delay', S('Delay'), ...
          'DelayLength', num2str(nd), 'SampleTime', dt);
add_block('simulink/Math Operations/Sum', S('Err'), 'Inputs', '+-');
add_block('simulink/Discontinuities/Dead Zone', S('DeadZone'), ...
          'LowerValue', '-P.Servo.Deadband/P.Linkage.n', ...
          'UpperValue',  'P.Servo.Deadband/P.Linkage.n');
add_block('simulink/Math Operations/Gain', S('wn2'), 'Gain', 'P.Servo.f_N^2');
add_block('simulink/Math Operations/Sum', S('Acc'), 'Inputs', '+-');
add_block('simulink/Discrete/Discrete-Time Integrator', S('IntRate'), ...
          'SampleTime', dt, 'IntegratorMethod', 'Integration: Forward Euler', ...
          'LimitOutput', 'on', ...
          'UpperSaturationLimit',  'P.Servo.Rate/P.Linkage.n', ...
          'LowerSaturationLimit', '-P.Servo.Rate/P.Linkage.n');
add_block('simulink/Math Operations/Gain', S('Damp'), ...
          'Gain', '2*P.Servo.Zeta*P.Servo.f_N');
add_block('simulink/Discrete/Discrete-Time Integrator', S('IntPos'), ...
          'SampleTime', dt, 'IntegratorMethod', 'Integration: Forward Euler', ...
          'LimitOutput', 'on', ...
          'UpperSaturationLimit',  'P.Servo.Travel', ...
          'LowerSaturationLimit', '-P.Servo.Travel');
add_block('simulink/Discontinuities/Backlash', S('Backlash'), ...
          'BacklashWidth', 'P.Servo.Lash/P.Linkage.n', 'InitialOutput', '0');

L = {'In1/1','Quant/1'; 'Quant/1','Delay/1'; 'Delay/1','Err/1'; ...
     'Err/1','DeadZone/1'; 'DeadZone/1','wn2/1'; 'wn2/1','Acc/1'; ...
     'Acc/1','IntRate/1'; 'IntRate/1','Damp/1'; 'Damp/1','Acc/2'; ...
     'IntRate/1','IntPos/1'; 'IntPos/1','Err/2'; 'IntPos/1','Backlash/1'; ...
     'Backlash/1','Out1/1'};
for i = 1:size(L,1)
    add_line(subpath, L{i,1}, L{i,2}, 'autorouting', 'on');
end
Simulink.BlockDiagram.arrangeSystem(subpath);

n = P.Linkage.n;
fprintf('Reduced actuator built at %s\n', subpath);
fprintf('  quantiser  %.4f deg  (KEEP IT -- it is the only source of residual ripple)\n', ...
        rad2deg(P.Servo.Res/n));
fprintf('  delay      %d taps (%.1f ms)   << resolved at build time\n', nd, 1e3*nd*P.Servo.dt);
fprintf('  dead zone  +/- %.4f deg at the canard\n', rad2deg(P.Servo.Deadband/n));
fprintf('  f_N %.1f rad/s, zeta %.2f\n', P.Servo.f_N, P.Servo.Zeta);
fprintf('  rate limit +/- %.0f deg/s  (inactive at present -- kept as a guard)\n', ...
        rad2deg(P.Servo.Rate/n));
fprintf('  travel     +/- %.2f deg\n', rad2deg(P.Servo.Travel));
fprintf('  backlash   %.4f deg total\n', rad2deg(P.Servo.Lash/n));
fprintf('\nNow delete the old MATLAB Function actuator and route through this one,\n');
fprintf('then set the model to Accelerator: set_param(''%s'', ''SimulationMode'', ''accelerator'')\n', model);
end
