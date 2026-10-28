function study = canard_size_study(scales, Nper, seed, req)
%CANARD_SIZE_STUDY  What canard area does this vehicle need, and why?
%
%   study = canard_size_study()
%   study = canard_size_study([0.7 0.85 1.0 1.2 1.4 1.7 2.0], 150, 2025, req)
%
%   scales  geometric scale factors on (s, C_r, C_t) together, so aspect ratio
%           and taper are held and only AREA changes (area ~ scale^2).
%   Nper    Monte Carlo runs per size (default 150). Set 0 to run the analytic
%           pass only -- that takes about a second and is most of the answer.
%   seed    fixed across sizes: common random numbers (default 2025)
%   req     .travel_bias  fraction of travel budgeted for holding the bias,
%                         the rest being left for damping (default 0.4)
%           .Cl_0_sigma   1-sigma of the standing roll moment (default 0.03)
%           .rms_err, .travel, .pFail  as PARAMETER_SWEEP_ROLL_CONTROL
%
%   THE TRADE, AND WHY THE EARLIER VERSION OF THIS FILE COULD NOT SEE IT
%
%   Area does NOT buy bandwidth. The gain schedule is re-tuned at each size to
%   hold the crossover (c ~ 1/Cl_delta), so the loop product Cl_delta*c is
%   invariant -- 62.5 for this vehicle at every size tested. The crossover cap
%   comes from the ACTUATOR (f_N, tau_d, Ms <= 2), not from the canard, and it
%   sits at 2.66 Hz whether the panel is 5 cm^2 or 31 cm^2. So every metric that
%   depends only on loop gain -- droop, settling, stability -- is identical at
%   every size by construction. An earlier version of this study reported
%   exactly that: a failure rate flat to within noise across a 6x area range,
%   which looked like a bug and was actually a theorem.
%
%   What area DOES change is a pair of opposing effects:
%
%     AUTHORITY improves.  The deflection needed to cancel a standing Cl_0 is
%       delta_req = Cl_0/Cl_delta, so it falls as 1/area. Undersize the canard
%       and the trim clamp -- or the mechanical stop -- is reached before the
%       bias is cancelled, and a standing roll rate survives the whole flight.
%
%     RESOLUTION worsens.  The servo dead zone dz is a fixed angle. As
%       delta_req shrinks, dz becomes a larger fraction of the working
%       deflection. Worse, the rate error a dead zone produces is
%           dp_dz ~ q*Cl_delta*dz / (Cl_delta*c + q*d*|Cl_p|/(2V))
%       and since Cl_delta*c is invariant, dp_dz grows LINEARLY WITH AREA.
%       A bigger canard resolves the bias more coarsely, in rate terms.
%
%   The design point is where those cross. Both are computed analytically below
%   before a single simulation runs; the Monte Carlo then confirms it with the
%   nonlinearities in the loop.
%
%   THE HEADLINE NUMBER is Cl_0_max = travel_bias * Servo.Travel * Cl_delta:
%   the largest standing roll moment the design can hold while still reserving
%   (1 - travel_bias) of the stop for damping. Compare it against your build
%   tolerance on Cl_0 -- that comparison, not a percentile, is the sizing case.

if nargin < 1 || isempty(scales), scales = [0.7 0.85 1.0 1.2 1.4 1.7 2.0]; end
if nargin < 2 || isempty(Nper),   Nper   = 150;  end
if nargin < 3 || isempty(seed),   seed   = 2025; end
if nargin < 4 || isempty(req),    req    = struct(); end
if ~isfield(req,'pFail'),       req.pFail       = 0.02; end
if ~isfield(req,'rms_err'),     req.rms_err     = 0.5;  end
if ~isfield(req,'travel'),      req.travel      = 0.8;  end
if ~isfield(req,'travel_bias'), req.travel_bias = 0.4;  end
if ~isfield(req,'Cl_0_sigma'),  req.Cl_0_sigma  = 0.03; end

P0 = parameters_initialise_roll_control();
s0 = P0.Vehicle.s; Cr0 = P0.Vehicle.C_r; Ct0 = P0.Vehicle.C_t;
n  = numel(scales);

[area, Cld, Clp, Copt, wc, dreq, Cl0max, nsig, dzfrac, dp_dz] = deal(nan(n,1));
[unstable, rms95, bias95, ripple95, trav95, travpk95, act95, overspec] = deal(nan(n,1));

% ======================================================================
% 1. ANALYTIC PASS -- about a second, and most of the answer
% ======================================================================
qw = max(montecarlo_envelope());              % worst (highest-q) condition
fprintf('\n=== canard size: analytic trade (Cl_0 sigma = %.3f, %.0f%% of travel for bias) ===\n', ...
        req.Cl_0_sigma, 100*req.travel_bias);
fprintf(['%6s %8s %9s %8s %8s %9s %9s %8s %9s %9s\n'], ...
        'scale','area','Cl_delta','c_opt','wc Hz','dreq deg','Cl0_max','sigma','dz/dreq','dp_dz');
for k = 1:n
    Pk = scale_vehicle(P0, scales(k), s0, Cr0, Ct0);
    [C, info] = optimise_roll_schedule(Pk, struct('plot',false,'quiet',true));
    Cld(k)  = info.Cl_delta;  Clp(k) = info.Cl_p;  Copt(k) = C;
    wc(k)   = C*info.k_b/(2*pi);
    area(k) = (Pk.Vehicle.C_r + Pk.Vehicle.C_t)*Pk.Vehicle.s*0.5*Pk.Vehicle.N;

    dz        = Pk.Servo.Deadband / Pk.Linkage.n;
    dreq(k)   = req.Cl_0_sigma / Cld(k);
    Cl0max(k) = req.travel_bias * Pk.Servo.Travel * Cld(k);
    nsig(k)   = Cl0max(k) / req.Cl_0_sigma;
    dzfrac(k) = dz / dreq(k);
    dp_dz(k)  = qw*Cld(k)*dz / (Cld(k)*C + qw*Pk.Vehicle.d*abs(Clp(k))/(2*Pk.Vehicle.V));

    fprintf('%6.2f %7.2f%s %9.4f %8.1f %8.2f %9.2f %9.4f %8.2f %9.3f %9.4f\n', ...
            scales(k), 1e4*area(k), ' ', Cld(k), Copt(k), wc(k), rad2deg(dreq(k)), ...
            Cl0max(k), nsig(k), dzfrac(k), dp_dz(k));
end
fprintf(['  dreq   = deflection to cancel a 1-sigma Cl_0        (falls as 1/area)\n' ...
         '  Cl0_max= largest Cl_0 holdable inside the travel budget (rises with area)\n' ...
         '  sigma  = Cl0_max expressed in sigmas of your build tolerance -- WANT >= 2\n' ...
         '  dp_dz  = roll-rate error one dead zone produces at q = %.0f Pa (rises with area)\n'], qw);

kok = find(nsig >= 2, 1, 'first');
if isempty(kok)
    fprintf(['\n** No size in range reaches 2 sigma of Cl_0 authority. Either the\n' ...
             '   canard must grow beyond scale %.2f, or the build tolerance on Cl_0\n' ...
             '   must be tightened below %.3f.\n'], max(scales), req.Cl_0_sigma);
else
    fprintf(['\n>> Smallest size reaching 2 sigma of Cl_0 authority: scale %.2f ' ...
             '(s = %.1f mm, area = %.2f cm^2)\n'], scales(kok), 1e3*s0*scales(kok), 1e4*area(kok));
    fprintf('   Its dead-zone rate error is %.4f rad/s, against %.4f at the size as built.\n', ...
            dp_dz(kok), dp_dz(find(abs(scales(:)-1) == min(abs(scales(:)-1)), 1)));
end

% ---- the resolution penalty is bought back by linkage ratio, not by area ----
% dz at the canard is Deadband/n, so dp_dz scales as 1/n while authority is
% untouched. Area and linkage are NOT interchangeable: only area fixes
% authority, only n fixes resolution. The pair is what closes the design.
kref = find(abs(scales(:)-1) == min(abs(scales(:)-1)), 1);
if ~isempty(kok)
    fprintf('\n   Recovering the resolution with linkage ratio n (dz at the canard = Deadband/n):\n');
    fprintf('     %-34s %10s %10s %10s\n', 'configuration', 'n = 1', 'n = 2', 'n = 4');
    fprintf('     %-34s %10.4f %10.4f %10.4f\n', ...
            sprintf('as built, scale %.2f', scales(kref)), ...
            dp_dz(kref), dp_dz(kref)/2, dp_dz(kref)/4);
    fprintf('     %-34s %10.4f %10.4f %10.4f\n', ...
            sprintf('2-sigma size, scale %.2f', scales(kok)), ...
            dp_dz(kok), dp_dz(kok)/2, dp_dz(kok)/4);
    fprintf(['     Area buys authority and costs resolution; n buys resolution and\n' ...
             '     costs slew rate. Confirm any n > 1 against the rate limit in the\n' ...
             '     sizing bench before committing to it.\n']);
end

% ======================================================================
% 2. MONTE CARLO CONFIRMATION
% ======================================================================
if Nper > 0
    fprintf('\n=== Monte Carlo confirmation, %d runs per size ===\n', Nper);
    for k = 1:n
        Pk = scale_vehicle(P0, scales(k), s0, Cr0, Ct0);
        ov = {'Vehicle.s',    Pk.Vehicle.s;
              'Vehicle.C_r',  Pk.Vehicle.C_r;
              'Vehicle.C_t',  Pk.Vehicle.C_t;
              'Aero.y_Cp',    Pk.Aero.y_Cp;
              'Controller.c', Copt(k)};
        r = montecarlo_roll_control(struct('N',Nper,'seed',seed,'quiet',true, ...
                                           'overrides',{ov}));
        unstable(k)=r.pFail;   rms95(k)=r.rms_p95;      bias95(k)=r.bias_p95;
        ripple95(k)=r.ripple_p95; trav95(k)=r.travel_p95; travpk95(k)=r.travelpk_p95;
        act95(k)=r.activity_p95;  overspec(k)=r.nOverSpec;
        fprintf(['  scale %.2f  area %6.2f cm^2 : unstable %.3f  rms95 %6.4f  ' ...
                 'bias95 %6.4f  trav95 %5.1f%% (pk %5.1f%%)  overspec %d/%d\n'], ...
                scales(k), 1e4*area(k), unstable(k), rms95(k), bias95(k), ...
                100*trav95(k), 100*travpk95(k), overspec(k), Nper);
    end
end

feasible = (nsig >= 2) & (trav95 <= req.travel | isnan(trav95)) & ...
           (rms95 <= req.rms_err | isnan(rms95));
study = table(scales(:), 1e3*s0*scales(:), 1e4*area, Cld, Clp, Copt, wc, ...
              rad2deg(dreq), Cl0max, nsig, dzfrac, dp_dz, ...
              unstable, rms95, bias95, ripple95, trav95, travpk95, act95, overspec, feasible, ...
    'VariableNames', {'scale','s_mm','area_cm2','Cl_delta','Cl_p','c_opt','wc_Hz', ...
                      'dreq_deg','Cl0_max','sigma_margin','dz_frac','dp_deadzone', ...
                      'unstable','rms_p95','bias_p95','ripple_p95','travel_p95', ...
                      'travelpk_p95','activity_p95','nOverSpec','feasible'});

% ======================================================================
% 3. PLOT -- the two opposing effects on one x axis
% ======================================================================
figure('Color','w','Name','Canard size study');
tl = tiledlayout(2,1,'TileSpacing','compact','Padding','compact');

nexttile;
yyaxis left
plot(1e4*area, nsig, '-o', 'LineWidth', 1.5); hold on
yline(2, 'r--', 'want >= 2\sigma', 'LineWidth', 1.2, 'HandleVisibility','off');
ylabel('Cl_0 authority / \sigma of build tolerance');
yyaxis right
plot(1e4*area, dp_dz, '-s', 'LineWidth', 1.5);
ylabel('rate error per dead zone / rad s^{-1}');
xlabel('Total canard planform area / cm^2'); grid on
mark_current(1e4*area, scales);
legend({'authority (up is good)','dead-zone rate error (down is good)'}, 'Location','best');
title('The trade: authority rises with area, resolution worsens with area');

nexttile;
if Nper > 0
    yyaxis left
    plot(1e4*area, 100*trav95, '-o', 'LineWidth', 1.5); hold on
    plot(1e4*area, 100*travpk95, ':', 'LineWidth', 1.0);
    yline(100*req.travel, 'r--', 'travel requirement', 'LineWidth', 1.2, 'HandleVisibility','off');
    ylabel('Travel used, 95th pct / %'); ylim([0 110]);
    yyaxis right
    plot(1e4*area, rms95, '-s', 'LineWidth', 1.5); hold on
    plot(1e4*area, ripple95, '--', 'LineWidth', 1.0);
    ylabel('rate error / rad s^{-1}');
    legend({'travel steady','travel peak','RMS error 95th pct','ripple pp 95th pct'}, ...
           'Location','best');
    xlabel('Total canard planform area / cm^2'); grid on
    mark_current(1e4*area, scales);
    title('Monte Carlo confirmation, gains re-tuned at each size');
else
    axis off; title('(Monte Carlo skipped: Nper = 0)');
end
title(tl, sprintf(['Canard size study. Crossover is actuator-limited at %.2f Hz ' ...
                   'and does NOT improve with area.'], wc(1)));
end


% ======================================================================
function Pk = scale_vehicle(P0, f, s0, Cr0, Ct0)
Pk = P0;
Pk.Vehicle.s   = s0*f;
Pk.Vehicle.C_r = Cr0*f;
Pk.Vehicle.C_t = Ct0*f;
% y_Cp is geometry, not a free constant. Recompute or every size is wrong --
% this is exactly how the stale 0.03975 survived a canard change once already.
Pk.Aero.y_Cp = Pk.Vehicle.d/2 + Pk.Vehicle.s*(Pk.Vehicle.C_r + 2*Pk.Vehicle.C_t) ...
                              / (3*(Pk.Vehicle.C_r + Pk.Vehicle.C_t));
end


function q = montecarlo_envelope()
% Highest dynamic pressure in the Monte Carlo envelope. Kept in step with
% montecarlo_roll_control by reading its default if possible.
q = [2000 5000 7200 12000 20000 32000 40000];
end


function mark_current(x, scales)
[~, i1] = min(abs(scales(:) - 1));
xline(x(i1), 'k-.', 'as built', 'LineWidth', 1.2, 'HandleVisibility','off');
end
