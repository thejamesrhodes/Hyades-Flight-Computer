function configure_for_speed(model, mode)
%CONFIGURE_FOR_SPEED  Solver and logging settings for a model containing the
%   nonlinear actuator. Fixes the usual 10-50x slowdown.
%
%   configure_for_speed('Simulink_Roll_Model_1', 'design')   % dt 2e-4, scopes on
%   configure_for_speed('Simulink_Roll_Model_1', 'mc')       % dt 5e-4, logging trimmed
%
%   WHY IT IS SLOW. The dead zone, backlash and rate limit are discontinuous.
%   With a variable-step solver the plant's continuous Integrator sees a
%   discontinuous input, so the solver shortens its step, retries, and hunts
%   for zero crossings -- while ALSO being forced to land on every actuator
%   sample point. That combination, not the MATLAB Function block, is what
%   costs you the order of magnitude.
%
%   Step size, measured on the closed loop against a dt = 1e-5 reference:
%
%     dt       limit-cycle amplitude error   servo rise-time error   speed
%     1e-4              -0.7 %                     0.08 %            1x
%     2e-4              -1.3 %                     0.17 %            2x
%     5e-4              -5.5 %                     0.43 %            5x
%     1e-3             -11.6 %                     0.86 %           10x
%
%   The limit-cycle amplitude is the dt-sensitive metric; everything else is
%   flat. 2e-4 for design work, 5e-4 for Monte Carlo, and do not go past 1e-3
%   if limit-cycle amplitude is one of your metrics.

if nargin < 1 || isempty(model), model = 'Simulink_Roll_Model_1'; end
if nargin < 2 || isempty(mode),  mode  = 'design'; end
load_system(model);

switch lower(mode)
    case 'design', dt = 2e-4;
    case 'mc',     dt = 5e-4;
    otherwise, error('mode must be ''design'' or ''mc''');
end

% ---- 1. fixed step. This is the big one. --------------------------------
set_param(model, 'SolverType', 'Fixed-step');
set_param(model, 'Solver',     'ode4');          % ode1 if you go fully discrete
set_param(model, 'FixedStep',  sprintf('%g', dt));
set_param(model, 'ZeroCrossControl', 'DisableAll');
fprintf('  fixed-step ode4 at %g s\n', dt);

% keep the actuator block on the same clock as the solver
blk = [model '/Actuator Model/Canard Servo'];
if ~isempty(find_system(model, 'LookUnderMasks','all', 'Name','Canard Servo'))
    set_param(blk, 'SampleTime', sprintf('%g', dt));
    evalin('base', sprintf('P.Servo.dt = %g;', dt));
    fprintf('  actuator sample time and P.Servo.dt -> %g\n', dt);
end

% ---- 2. logging. 100k points/s of Scope data is not free. ---------------
scopes = find_system(model, 'LookUnderMasks','all', 'BlockType','Scope');
for i = 1:numel(scopes)
    if strcmpi(mode,'mc')
        set_param(scopes{i}, 'Commented', 'on');
    else
        set_param(scopes{i}, 'Commented', 'off');
    end
end
if ~isempty(scopes)
    fprintf('  %d scope(s) %s\n', numel(scopes), ...
            merge_str(strcmpi(mode,'mc'), 'commented out', 'left on'));
end
set_param(model, 'SignalLoggingSaveFormat', 'Dataset');
set_param(model, 'LimitDataPoints', 'on');
set_param(model, 'MaxDataPoints', '20000');
set_param(model, 'SaveFormat', 'Dataset');

% ---- 3. compile once, not 4000 times -----------------------------------
if strcmpi(mode,'mc')
    set_param(model, 'SimulationMode', 'accelerator');
    fprintf('  accelerator mode on\n');
    fprintf(['  -> also set ''UseFastRestart'',''on'' in the parsim call in\n' ...
             '     montecarlo_roll_control.m. It is currently ''off'', which\n' ...
             '     recompiles every run.\n']);
else
    set_param(model, 'SimulationMode', 'normal');
end

% ---- 4. the structural win, if you want it -----------------------------
nInt = numel(find_system(model, 'LookUnderMasks','all', 'BlockType','Integrator'));
if nInt > 0
    fprintf(['\n  %d continuous Integrator block(s) remain. Swapping them for\n' ...
             '  Discrete-Time Integrator at %g s leaves the model with ZERO\n' ...
             '  continuous states -- solver becomes FixedStepDiscrete and the\n' ...
             '  sim reduces to a plain loop. It is also what runs on the H743.\n' ...
             '  Worth another 2-4x on top of the above.\n'], nInt, dt);
end
fprintf('done.\n');
end

function s = merge_str(c, a, b)
if c, s = a; else, s = b; end
end
