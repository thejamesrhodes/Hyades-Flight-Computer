%RUN_ALL  Everything, in order, from a cold MATLAB.
%   Run section by section with Ctrl+Enter. Sections 0-3 are cheap checks that
%   must pass before any Monte Carlo number is worth reading. Sections 4-7 are
%   the actual design loop. Section 8 is the sign-off.
%
%   cd to "...\Flight Controller V6\Testing\Matlab Roll Tuning" first.
%
%   Total runtime for 4-8 at the suggested N is roughly 2-4 hours depending on
%   how fast a single 20 s sim is. Cut every N by 3 for a first pass -- the
%   shapes appear long before the percentiles are tight.

%% 0. Clean path -- kills the shadowed-model warning
clear; clc; close all
restoredefaultpath; rehash toolboxcache
addpath(pwd);
% If 'Simulink_Roll_Model_1' still resolves to two files, find the other with:
%   which -all Simulink_Roll_Model_1

%% 1. Parameters, and confirm the servo derivation
P = parameters_initialise_roll_control();
servo_params_from_datasheet(P.Servo.Voltage);
fprintf('Cl_delta check: y_Cp = %.5f m (derived from geometry)\n', P.Aero.y_Cp);

%% 2. Actuator: is the reduced model still faithful?
%    Cross-check must print 0.000e+00. The quantiser must show up as the only
%    element whose removal changes the answer.
compare_actuators(P, 0.03);

%% 3. Does the dispersion actually reach the model?
%    Run this ONCE after any change to how the actuator is wired. If it says a
%    dispersion produced an identical trace, every sensitivity number below is
%    meaningless. This is the single most important check in the file.
check_tunability('Simulink_Roll_Model_1', false);

%% 4. Linear starting point for the gain
%    Reads Cl_delta and Cl_p from P, so it can no longer go stale.
%    Expect C ~ 280 at the current canard (the old hard-coded 0.6694 gave ~125).
[C0, sched] = optimise_roll_schedule(P);

%% 5. Coarse 1-D sweep of Controller.c  -- bracket the answer
%    Log grid spanning a factor of ~8 around the linear estimate. 100 runs per
%    point is enough to see the shape; it is NOT enough to trust pFail.
%    ~15 min.
cgrid  = logspace(log10(C0/3), log10(C0*3), 9);
sweepC = parameter_sweep_roll_control({'Controller.c', cgrid}, 100, 2025);

%% 6. Fine 1-D sweep of Controller.c  -- pick it
%    Narrow to the feasible span from section 5. 300 runs per point.
%    ~45 min.
cbest  = sweepC.Controller_c(find(sweepC.feasible, 1, 'first'));
cfine  = linspace(0.6*cbest, 1.8*cbest, 7);
sweepC2 = parameter_sweep_roll_control({'Controller.c', cfine}, 300, 2025);
c_sel  = sweepC2.Controller_c(sweepC2.rms_p95 == min(sweepC2.rms_p95(sweepC2.feasible)));
fprintf('\n>>> selected Controller.c = %.4g\n', c_sel(1));

%% 7. 2-D sweep of the trim path, at the chosen c
%    These are the only other free parameters in Mode 1. Ti is NOT free -- it
%    is read only in Mode 2 (legacy PI), so sweeping it proves nothing.
%    Trim.Ki    trim bandwidth: too slow and Cl_0 survives the flight, too fast
%               and the trim re-enters the loop and the stick-slip returns.
%    Trim.BandK dead zone on the trim integrator input. Below ~1.0 the 0.45 Hz
%               cycle reappears; above it, the price is hang-off.
%    ~50 min.
sweepT = parameter_sweep_roll_control( ...
    {'Trim.Ki',    [0.008 0.015 0.023 0.040 0.070]; ...
     'Trim.BandK', [0.8 1.0 1.1 1.4 2.0]}, 200, 2025);
% Read the selection off the feasible-region figure, then:
%   P.Controller.c = c_sel;  P.Trim.Ki = ...;  P.Trim.BandK = ...;
% and put those numbers into parameters_initialise_roll_control.m.

%% 8. Is this canard the right size?
%    Re-tunes c at every size so the comparison is at EQUAL CROSSOVER -- a
%    bigger canard judged on a gain sized for a smaller one is not a fair test.
%    Look for the knee in travel_p95: that is the design point, and it is a
%    hard constraint in a way that rms_err is not.
%    ~40 min.
sizes = canard_size_study([0.7 0.85 1.0 1.2 1.4 1.7], 150, 2025);
disp(sizes);

%% 9. Sign-off Monte Carlo at the frozen gains
%    Only after 6, 7 and 8 have settled. 2000 runs, full envelope.
%    ~1 h.
results = montecarlo_roll_control(struct('N', 2000, 'seed', 7331));

%% 10. Backlash scenarios -- NOT a dispersion
%    Lash is a per-unit build and wear quantity. Dispersing it Gaussian
%    averages away the worn case, which is the one that matters. Run it as
%    three discrete builds: new, nominal, worn.
for lash = [0, 0.0026, 0.0052]
    fprintf('\n########## Servo.Lash = %.4f rad (%.3f deg) ##########\n', ...
            lash, rad2deg(lash));
    montecarlo_roll_control(struct('N', 500, 'seed', 7331, ...
                                   'overrides', {{'Servo.Lash', lash}}));
end

%% 11. Linkage ratio scenarios -- hardware, not a gain
%    n moves the dead zone, the torque and the slew rate together, so it does
%    not belong on the gain grid. The sizing bench puts the knee at n ~ 4-5.
for n = [1 2 4]
    fprintf('\n########## Linkage.n = %d ##########\n', n);
    montecarlo_roll_control(struct('N', 500, 'seed', 7331, ...
                                   'overrides', {{'Linkage.n', n}}));
end

%% 12. Sizing bench, if the geometry moved
%    python Verify.py && node test_physics.mjs   (in ..\Canard Sizing)
%    then open canard_bench_standalone.html, set omega_c from section 4's
%    crossover, and re-export.

% =========================================================================
% WHAT TO READ ON EACH FIGURE
%
% Monte Carlo, "Performance distributions" (4 ECDFs)
%   ECDF not histogram: percentiles read straight off the y axis, no bin width
%   to argue about, and these are skewed distributions where a histogram hides
%   the tail. The dashed line is the 95th percentile, red is the requirement.
%   Bad sign: a long flat tail on travel -- a few corners saturating.
%
% Monte Carlo, "Envelope"
%   One marker per (q, V) condition, area = number of runs there, colour = mean
%   outcome. This is the plot that says whether the canard is big enough,
%   because authority runs out at ONE end of the envelope, not on average.
%   Bad sign: travel near 1.0 at the low-q corner.
%
% Monte Carlo, "Authority vs accuracy"
%   Each dot is a run. A vertical wall at travel = 1 means saturation is
%   setting the error, and no amount of gain tuning fixes that -- only more
%   area or more linkage ratio does.
%
% Monte Carlo, "Sensitivity" (SRC bars)
%   SRC^2 partitions output variance for independent inputs, so bar length is
%   importance. A scatter matrix was dropped: 13 inputs is 169 panels.
%   Check R^2. Below ~0.5 the response is nonlinear and the bars understate
%   whatever is causing it -- go back to the envelope panels.
%
% Sweep, 1-D
%   Figure 1 is robustness and accuracy, figure 2 is what it costs the servo.
%   Grey bands are infeasible. The knee in figure 1 and the rise in activity in
%   figure 2 usually bracket the answer between them: activity climbing faster
%   than travel means the extra gain is going into chatter, not control.
%
% Sweep, 2-D
%   Index axes with real values as tick labels, so a logspace grid is drawn
%   correctly -- imagesc(v, v, Z) would map the axes linearly and put every
%   contour in the wrong place. Red contour is the requirement boundary; the
%   star is the selection. The "Feasible region" figure is the decision.
%
% Canard size study
%   Everything against AREA, because area is what costs drag, mass and hinge
%   moment. Top panel is the hard constraint. Bottom panel: if rms_err has gone
%   flat, extra area is buying nothing.
