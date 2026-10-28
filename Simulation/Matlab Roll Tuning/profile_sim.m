function profile_sim(model)
%PROFILE_SIM  Find what is actually costing the wall-clock time.
%
%   Run this before guessing. It times the model under each setting in turn so
%   you can see which one moves the needle, rather than changing five things
%   and hoping.
%
%   Order of suspects for a model containing the nonlinear actuator, once the
%   solver is already fixed-step:
%     1  Normal mode. Every MATLAB Function block call crosses the
%        Simulink/MATLAB boundary each step. Accelerator compiles the whole
%        model to a MEX -- usually the single biggest remaining win.
%     2  Simulation Data Inspector. SDI records every logged signal at the
%        solver rate and is notoriously expensive. Turn it off for sweeps.
%     3  Scopes. 5000 points per simulated second each, unbounded by default.
%     4  Leftover continuous states. One surviving Integrator makes ode4 call
%        every block FOUR times per step.

if nargin < 1 || isempty(model), model = 'Simulink_Roll_Model_1'; end
load_system(model);
evalin('base', 'P = parameters_initialise_roll_control();');

fprintf('\n--- baseline ---\n');
fprintf('  solver        %s / %s, step %s\n', get_param(model,'SolverType'), ...
        get_param(model,'Solver'), get_param(model,'FixedStep'));
fprintf('  sim mode      %s\n', get_param(model,'SimulationMode'));
nCont = numel(find_system(model,'LookUnderMasks','all','BlockType','Integrator'));
nScope = numel(find_system(model,'LookUnderMasks','all','BlockType','Scope'));
fprintf('  continuous Integrator blocks: %d   (want 0)\n', nCont);
fprintf('  Scope blocks: %d\n', nScope);
if nCont > 0
    fprintf('  ** ode4 calls every block 4x per step while any continuous state remains **\n');
end

cases = {
  'as-is',                        @() deal()
  'SDI logging off',              @() Simulink.sdi.setAutoArchiveMode(false)
  'scopes commented',             @() setScopes(model, 'on')
  'accelerator mode',             @() set_param(model,'SimulationMode','accelerator')
};

t0 = get_param(model,'StopTime');
fprintf('\n--- timing (StopTime = %s) ---\n', t0);
base = NaN;
for i = 1:size(cases,1)
    try, cases{i,2}(); catch, end
    tic; sim(model); w = toc;
    if isnan(base), base = w; end
    fprintf('  %-26s %6.2f s wall   %5.2f s/s   %4.1fx\n', ...
            cases{i,1}, w, w/str2double(t0), base/w);
end

fprintf(['\nIf accelerator gives the big jump, keep it and set\n' ...
         '''UseFastRestart'',''on'' in the parsim call -- yours is ''off'',\n' ...
         'which recompiles on every one of the 4000 runs.\n']);
end

function setScopes(model, state)
s = find_system(model,'LookUnderMasks','all','BlockType','Scope');
for i = 1:numel(s), set_param(s{i}, 'Commented', state); end
end
