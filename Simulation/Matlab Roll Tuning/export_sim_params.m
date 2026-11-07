function export_sim_params()
%EXPORT_SIM_PARAMS  Write P (parameters_initialise_roll_control) to JSON for
%   the HITL plant (Firmware/FC-V4/Tools/hitl_model.py), so the Python model
%   reads the same numbers as Simulink instead of a retyped copy (HITL-01).
%   Re-run after any change to parameters_initialise_roll_control.m.
here = fileparts(mfilename('fullpath'));
P = parameters_initialise_roll_control();
out = fullfile(here, '..', '..', 'Firmware', 'FC-V4', 'Tools', 'sim_params.json');
fid = fopen(out, 'w');
fprintf(fid, '%s', jsonencode(P, 'PrettyPrint', true));
fclose(fid);
fprintf('wrote %s\n', out);
end
