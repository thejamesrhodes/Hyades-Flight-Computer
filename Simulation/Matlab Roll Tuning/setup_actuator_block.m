function setup_actuator_block(model, blkpath)
%SETUP_ACTUATOR_BLOCK  Configure the canard servo MATLAB Function block the same
%   way your plant and controller blocks are configured: P as a PARAMETER, not
%   an input port, and an explicit discrete sample time.
%
%   setup_actuator_block                                   % uses the defaults below
%   setup_actuator_block('Simulink_Roll_Model_1', 'Simulink_Roll_Model_1/Actuator Model/Canard Servo')
%
%   Do the two manual steps first:
%     1. In the Actuator Model subsystem, delete the Aerospace Blockset
%        "Nonlinear Second-Order Actuator" block.
%     2. Drop in a MATLAB Function block, name it "Canard Servo", and paste the
%        WHOLE of canard_servo.m into it -- main function and both local
%        functions. Simulink names the ports from the signature, so you get
%        delta_cmd in and delta out, plus a P port it should not keep.
%   Then run this. It removes the P port and pins the sample time.
%
%   NOTE: written against the standard Stateflow API but not executed here --
%   I have no Simulink in this environment. If a line fails, the manual
%   equivalent is in the comment above it.

if nargin < 1 || isempty(model),   model   = 'Simulink_Roll_Model_1'; end
if nargin < 2 || isempty(blkpath), blkpath = [model '/Actuator Model/Canard Servo']; end

load_system(model);

% ---- 1. P from Input scope to Parameter scope ---------------------------
% Manual equivalent: open the block, View > Symbols Pane, set P's Scope to
% "Parameter" and its Type to "Inherit: Same as Simulink".
rt   = sfroot;
chart = rt.find('-isa','Stateflow.EMChart','Path',blkpath);
if isempty(chart)
    error('setup_actuator_block:notfound', ...
          'No MATLAB Function block at "%s". Check the path and the block name.', blkpath);
end
d = chart.find('-isa','Stateflow.Data','Name','P');
if isempty(d)
    error('setup_actuator_block:noP', ...
          'The block has no symbol named P. Paste canard_servo.m in first.');
end
d.Scope    = 'Parameter';
d.DataType = 'Inherit: Same as Simulink';
fprintf('  P  -> scope Parameter (no longer an input port)\n');

% ---- 2. explicit discrete sample time -----------------------------------
% Manual equivalent: right-click the block > Block Parameters > Sample time.
set_param(blkpath, 'SampleTime', 'P.Servo.dt');
fprintf('  sample time -> P.Servo.dt\n');

% ---- 3. checks that catch the usual traps -------------------------------
P = evalin('base', 'exist(''P'',''var'')') && evalin('base','isstruct(P)');
if ~P
    warning(['P is not a struct in the base workspace. Run ' ...
             'P = parameters_initialise_roll_control(); before compiling.']);
else
    Pb = evalin('base','P');
    bad = find_char_fields(Pb, 'P');
    if ~isempty(bad)
        warning(['These P fields are char/string and will break codegen in ' ...
                 'every MATLAB Function block that takes P: %s'], strjoin(bad, ', '));
    end
    nd = round(Pb.Servo.Tau_d / Pb.Servo.dt);
    fprintf('  delay buffer: %d taps (limit 2047)\n', nd);
end

st = get_param(model, 'SolverType');
if strcmp(st, 'Variable-step')
    fprintf(['  NOTE solver is Variable-step. The dead zone, backlash and rate\n' ...
             '       limit are discontinuous; a fixed-step solver (ode4 at\n' ...
             '       %g s) is steadier and matches what runs on the H743.\n'], ...
             evalin('base','P.Servo.dt'));
end
fprintf('done.\n');
end

% =========================================================================
function out = find_char_fields(s, prefix)
out = {};
f = fieldnames(s);
for i = 1:numel(f)
    v = s.(f{i});
    if isstruct(v)
        out = [out, find_char_fields(v, [prefix '.' f{i}])]; %#ok<AGROW>
    elseif ischar(v) || isstring(v)
        out{end+1} = [prefix '.' f{i}]; %#ok<AGROW>
    end
end
end
