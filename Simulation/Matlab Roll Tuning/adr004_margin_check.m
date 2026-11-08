% adr004_margin_check.m -- ADR-004 cross-check of the Python port (rollopt.py)
% Margins of the q-scheduled roll law for the forward-body inertia (fin can
% decoupled) under the old and new digital chains, and at the firmware gains.
P = parameters_initialise_roll_control();
P.Vehicle.Ixx = 2.2e-4;          % forward body only (ADR-004 §3); measure (trifilar)
chains = { 'MATLAB as before (no digital delay)', 0,      0
           '7 Nov firmware 100 Hz / 50 Hz async', 0.0192, 30
           'ADR-004 200 Hz synced, 448 Hz IMU',   0.0066, 60 };
for k = 1:size(chains,1)
    cfg = struct('quiet',true,'plot',false,'tau_extra',chains{k,2},'lpf_hz',chains{k,3});
    [C, info] = optimise_roll_schedule(P, cfg);
    f = info.PM >= 35 & info.GM >= 2 & info.Ms <= 2;
    [~,i70] = min(abs(info.Cvec - 70)); [~,i12] = min(abs(info.Cvec - 12));
    if any(f), sp = sprintf('%.1f-%.1f', min(info.Cvec(f)), max(info.Cvec(f))); else, sp = 'none'; end
    fprintf('%-38s C_opt %6.1f  feasible C %s | C=70: PM %6.1f GM %5.2f | C=12: PM %5.1f GM %5.2f Ms %4.2f\n', ...
        chains{k,1}, C, sp, info.PM(i70), info.GM(i70), info.PM(i12), info.GM(i12), info.Ms(i12));
end
