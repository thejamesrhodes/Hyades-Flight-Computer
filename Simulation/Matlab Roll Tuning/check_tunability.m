function check_tunability(model, fast_restart)
%CHECK_TUNABILITY  Does the Monte Carlo dispersion actually reach the model?
%
%   check_tunability
%   check_tunability('Simulink_Roll_Model_1', true)   % test WITH fast restart
%
%   THE FAILURE THIS CATCHES IS SILENT, WHICH IS WHY IT NEEDS A TEST.
%
%   Dead Zone start/end, Backlash width, Delay length and Quantizer interval
%   are STRUCTURAL block parameters. When they are written as expressions of a
%   workspace struct (P.Servo.Deadband and friends, as BUILD_REDUCED_ACTUATOR
%   writes them) Simulink resolves them at compile time. Under fast restart the
%   model is compiled once and reused, so a run that sets a different
%   P.Servo.Deadband gets the OLD value -- no warning, no error, just a Monte
%   Carlo that reports the dead zone has no influence.
%
%   The same trap does not apply to the MATLAB Function implementations
%   (CANARD_SERVO, CANARD_SERVO_REDUCED): those read P every step, so the value
%   follows. Which is exactly why it is worth knowing which actuator is wired
%   in before trusting a sensitivity ranking.
%
%   This runs the model three times -- nominal, dead zone x4, lash x4 -- and
%   reports whether the outputs differ. If they do not, the dispersion is not
%   reaching the model.

if nargin < 1 || isempty(model), model = 'Simulink_Roll_Model_1'; end
if nargin < 2, fast_restart = false; end

load_system(model);
P0 = parameters_initialise_roll_control();
P0.Aero.Cl_0 = 0.03;                       % give the loop something to chew on

Pdz = P0; Pdz.Servo.Deadband = 4*P0.Servo.Deadband;
Plh = P0; Plh.Servo.Lash     = 4*P0.Servo.Lash;
cases = { 'nominal',     P0
          'Deadband x4', Pdz
          'Lash x4',     Plh };

if fast_restart
    set_param(model, 'FastRestart', 'on');
    fr = 'ON';
else
    fr = 'OFF';
end
c = onCleanup(@() set_param(model, 'FastRestart', 'off')); %#ok<NASGU>

% Simulink.SimulationInput, NOT assignin + sim(model).
%
% This was a real bug in an earlier version of this file and it produced a
% false alarm. assignin('base','P',...) followed by sim(model) does not force
% Simulink to re-resolve a struct-valued workspace parameter: the model keeps
% whatever P it compiled with, so EVERY case returns a bit-identical trace and
% this function reports that no dispersion reaches the model. It was measured:
% tripling Aero.Cl_0 -- which triples the disturbance -- changed the result by
% exactly zero through that path, and by 200+ rad/s through this one.
% setVariable is the supported mechanism and is what MONTECARLO_ROLL_CONTROL
% uses, so this test now exercises the same path the analysis does.
sig = cell(size(cases,1),1);
fprintf('\n--- dispersion reachability, fast restart %s ---\n', fr);
for k = 1:size(cases,1)
    in  = Simulink.SimulationInput(model);
    in  = in.setVariable('P', cases{k,2});
    out = sim(in);
    if ~isempty(out.ErrorMessage)
        error('check_tunability:simfail', 'Case "%s" failed: %s', ...
              cases{k,1}, out.ErrorMessage);
    end
    d = out.logsout.get('delta_real_log').Values.Data(:);
    p = out.logsout.get('p_log').Values.Data(:);
    sig{k} = [d(:); p(:)];
    fprintf('  %-14s  RMS delta %.6f rad   final p %.5f rad/s\n', ...
            cases{k,1}, sqrt(mean(d.^2)), p(end));
end

fprintf('\n');

% ---- POSITIVE CONTROL FIRST -------------------------------------------
% Without this the test cannot tell "the dispersion is ignored" from "nothing
% is happening at all", and it will confidently report the wrong cause.
nomRMS = sqrt(mean(sig{1}(1:round(numel(sig{1})/2)).^2));
if nomRMS < 1e-9
    fprintf(['  ** THE NOMINAL RUN IS DEAD: zero deflection and zero roll rate\n' ...
             '     with Cl_0 = 0.03 applied. Every case will match every other\n' ...
             '     case, so the dispersion result below means nothing.\n' ...
             '     Check that Plant reads P.Aero.Cl_0 and that P actually\n' ...
             '     reached the model, before reading anything else here.\n\n']);
end

ok = true;
for k = 2:numel(sig)
    n  = min(numel(sig{1}), numel(sig{k}));
    dz = max(abs(sig{1}(1:n) - sig{k}(1:n)));
    if dz < 1e-12
        fprintf('  ** "%s" produced an IDENTICAL trace to nominal.\n', cases{k,1});
        ok = false;
    else
        fprintf('  "%s" changed the trace by %.3e -- dispersion reaches the model.\n', ...
                cases{k,1}, dz);
    end
end

if ok
    fprintf('\nAll dispersions reach the model. Safe to proceed.\n\n');
    return
end

% ---- DIAGNOSE, most likely cause first --------------------------------
fprintf('\n--- diagnosis, in the order worth checking ---\n');

% (a) is the element even in the model?
asub = [model '/Actuator Model'];
fprintf('(a) what is actually inside "%s":\n', asub);
try
    b = find_system(asub, 'SearchDepth', 1, 'LookUnderMasks','all');
    for i = 2:numel(b)
        fprintf('      %-28s [%s]\n', get_param(b{i},'Name'), get_param(b{i},'BlockType'));
    end
    ref = find_system(asub,'SearchDepth',1,'BlockType','Reference');
    if ~isempty(ref)
        fprintf(['    ** A library Reference block is the actuator. If that is the\n' ...
                 '       Aerospace Blockset "Nonlinear Second-Order Actuator", it has\n' ...
                 '       NO dead zone, NO backlash, NO quantiser and NO delay -- the\n' ...
                 '       parameters you are dispersing do not exist in the model.\n' ...
                 '       THIS IS THE USUAL CAUSE. Fix: wire_model(''%s'', ''apply'')\n'], model);
    end
catch e
    fprintf('      (could not inspect: %s)\n', e.message);
end

% (b) structural block parameters under fast restart
if fast_restart
    fprintf(['(b) fast restart is ON. Dead Zone limits, Backlash width, Delay\n' ...
             '    length and Quantizer interval are structural: they are resolved\n' ...
             '    at compile and are not re-evaluated between fast-restart runs.\n' ...
             '    Re-run this with fast_restart = false to separate the two causes.\n']);
else
    fprintf('(b) fast restart is OFF, so it is not the cause here.\n');
end

% (c) P not reaching the block
fprintf(['(c) if the actuator IS a MATLAB Function block, check that P is a\n' ...
         '    Parameter-scope symbol resolving from the base workspace, not a\n' ...
         '    stale copy in the model workspace:\n' ...
         '        get_param(bdroot, ''ModelWorkspace'')\n\n']);
end
