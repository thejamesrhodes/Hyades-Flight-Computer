function sweep = parameter_sweep_roll_control(spec, Nper, seed, req)
% PARAMETER_SWEEP_ROLL_CONTROL  Deterministic sweep of DESIGN parameters, with
% a Monte Carlo dispersion run nested at each grid point.
%
%   sweep = parameter_sweep_roll_control(spec)
%   sweep = parameter_sweep_roll_control(spec, Nper, seed, req)
%   sweep = parameter_sweep_roll_control(cvec, ...)      % legacy: c only
%
%     spec  1- or 2-row cell {name, values; name, values}, e.g.
%             {'Controller.c', logspace(log10(60),log10(600),9)}
%             {'Controller.c', [150 220 280 360]; 'Trim.Ki', [0.01 0.023 0.05]}
%     Nper  runs per grid point (default 150)
%     seed  RNG seed, FIXED across all points -- common random numbers, so
%           differences between grid points are attributable to the design
%           parameter alone and not to sampling noise (default 2025)
%     req   struct of requirement thresholds used for feasibility and for the
%           selection rule. Defaults:
%             req.pFail    = 0.02      max acceptable failure probability
%             req.rms_err  = 0.5       rad/s, 95th pct RMS rate error
%             req.travel   = 0.8       fraction of mechanical stop, 95th pct
%
%   WHICH PARAMETERS ARE ACTUALLY FREE NOW
%
%   Controller.Ti is NOT one of them. It is read only in Controller.Mode == 2
%   (legacy PI). In Mode 1 -- the design -- the integral path is gone and Ti is
%   dead code, so a c x Ti grid spends every run proving that Ti does nothing.
%   The genuinely free parameters of the current controller are:
%
%     Controller.c   proportional schedule constant. Sets the crossover:
%                    wc = c * A*d*Cl_delta/Ixx. THE ONE THAT MATTERS.
%     Trim.Ki        trim bandwidth. Too slow and Cl_0 is not cancelled inside
%                    the flight; too fast and it re-enters the loop dynamics
%                    and the stick-slip cycle comes back.
%     Trim.BandK     dead zone on the trim integrator input, in multiples of
%                    (Deadband/Kp). Below ~1.0 the 0.45 Hz cycle reappears;
%                    above it, the cost is hang-off. A genuine trade.
%     Linkage.n      not a gain but a hardware choice, and it moves the dead
%                    zone, the torque and the slew rate together. Sweep it
%                    separately as scenarios, not on this grid.
%
%   The right order is c first (1-D), then Trim.Ki x Trim.BandK at the chosen c.
%   Sweeping all three at once is 3-D and mostly wasted: c dominates.

% ---- argument handling ---------------------------------------------------
if isnumeric(spec)                       % legacy call signature
    spec = {'Controller.c', spec(:)'};
end
if nargin < 2 || isempty(Nper), Nper = 150;  end
if nargin < 3 || isempty(seed), seed = 2025; end
if nargin < 4 || isempty(req)
    req = struct();
end
if ~isfield(req,'pFail'),    req.pFail    = 0.02;         end
if ~isfield(req,'rms_err'),  req.rms_err  = 0.5;          end
if ~isfield(req,'travel'),   req.travel   = 0.5;          end
if ~isfield(req,'theta_pp'), req.theta_pp = deg2rad(10);  end

nDim = size(spec,1);
if nDim < 1 || nDim > 2
    error('parameter_sweep_roll_control: spec must have 1 or 2 rows.');
end
name1 = spec{1,1};  v1 = spec{1,2}(:);
if nDim == 2
    name2 = spec{2,1}; v2 = spec{2,2}(:);
else
    name2 = ''; v2 = NaN;
end
n1 = numel(v1); n2 = numel(v2); nPt = n1*n2;
is2D = (nDim == 2) && (n1 > 1) && (n2 > 1);

[G1, G2] = meshgrid(v1, v2);             % n2-by-n1
F1 = G1(:); F2 = G2(:);

cols = {'pFail','pFail_se','nStable','rms_p95','bias_p95','ripple_p95', ...
        'travel_p95','travelpk_p95','activity_p95','settle_p95','nOverSpec', ...
        'theta_p95','freq_med'};
for i = 1:numel(cols), R.(cols{i}) = nan(nPt,1); end

fprintf('Sweeping %d point(s), %d runs each (seed %d, common random numbers)\n', ...
        nPt, Nper, seed);
fprintf(['  requirements: unstable <= %.3f, ANGLE ripple <= %.1f deg pp, ' ...
         'steady travel <= %.0f%%\n'], req.pFail, rad2deg(req.theta_pp), 100*req.travel);
fprintf('  selection is on angle ripple: a rate bias is absorbed by the future attitude loop.\n');

for idx = 1:nPt
    opts       = struct('N', Nper, 'seed', seed, 'quiet', true);
    ov         = {name1, F1(idx)};
    if nDim == 2, ov = [ov; {name2, F2(idx)}]; end %#ok<AGROW>
    opts.overrides = ov;

    r = montecarlo_roll_control(opts);
    for i = 1:numel(cols), R.(cols{i})(idx) = r.(cols{i}); end

    if nDim == 2
        fprintf(['  %s=%-9.4g %s=%-9.4g : unstable=%.3f  rms95=%7.4f  ' ...
                 'theta95=%5.1f deg @ %4.2f Hz  trav95=%5.1f%%  act95=%6.3f\n'], ...
                name1, F1(idx), name2, F2(idx), R.pFail(idx), R.rms_p95(idx), ...
                rad2deg(R.theta_p95(idx)), R.freq_med(idx), ...
                100*R.travel_p95(idx), R.activity_p95(idx));
    else
        fprintf(['  %s=%-9.4g : unstable=%.3f  rms95=%7.4f  ' ...
                 'theta95=%5.1f deg @ %4.2f Hz  trav95=%5.1f%%  act95=%6.3f\n'], ...
                name1, F1(idx), R.pFail(idx), R.rms_p95(idx), ...
                rad2deg(R.theta_p95(idx)), R.freq_med(idx), ...
                100*R.travel_p95(idx), R.activity_p95(idx));
    end
end

% ---- feasibility and selection ------------------------------------------
feasible = (R.pFail <= req.pFail) & (R.theta_p95 <= req.theta_pp) & ...
           (R.travel_p95 <= req.travel);
% Selection is on ANGLE ripple, not RMS rate error: a steady rate bias is
% absorbed by the future attitude loop, oscillation is not.
score = R.theta_p95;  score(~feasible) = Inf;
[~, best] = min(score);
if ~any(feasible)
    warning(['No grid point meets all three requirements. Reporting the ' ...
             'lowest ANGLE ripple instead -- relax a requirement or widen the grid.']);
    [~, best] = min(R.theta_p95);
end

fprintf('\n--- selected ---\n');
fprintf('%s = %.6g', name1, F1(best));
if nDim == 2, fprintf('   %s = %.6g', name2, F2(best)); end
fprintf('\n  unstable %.3f | ANGLE ripple %.2f deg pp at %.2f Hz\n', ...
        R.pFail(best), rad2deg(R.theta_p95(best)), R.freq_med(best));
fprintf('  rate: rms %.4f  bias %.4f  ripple %.4f rad/s\n', ...
        R.rms_p95(best), R.bias_p95(best), R.ripple_p95(best));
fprintf('  travel %.1f%% steady | activity %.3f rad/s\n', ...
        100*R.travel_p95(best), R.activity_p95(best));
fprintf('  feasible points: %d of %d\n\n', sum(feasible), nPt);

% ---- table ---------------------------------------------------------------
sweep = table(F1, F2, R.nStable, R.pFail, R.pFail_se, R.rms_p95, R.bias_p95, ...
              R.ripple_p95, rad2deg(R.theta_p95), R.freq_med, R.travel_p95, ...
              R.travelpk_p95, R.activity_p95, R.settle_p95, R.nOverSpec, feasible, ...
    'VariableNames', {matlab.lang.makeValidName(name1), ...
                      matlab.lang.makeValidName(ternary(nDim==2, name2, 'unused')), ...
                      'nStable','pFail','pFail_se','rms_p95','bias_p95', ...
                      'ripple_p95','theta_deg_p95','ripple_Hz','travel_p95', ...
                      'travelpk_p95','activity_p95','settle_p95','nOverSpec','feasible'});
sweep.Properties.UserData = struct('spec',{spec},'req',req,'best',best, ...
                                   'v1',v1,'v2',v2,'is2D',is2D);

% ---- plots ---------------------------------------------------------------
if is2D
    plot_2d(v1, v2, name1, name2, R, feasible, best, req);
else
    plot_1d(v1, name1, R, feasible, best, req);
end
end


% ======================================================================
function out = ternary(c, a, b)
if c, out = a; else, out = b; end
end


% ======================================================================
function plot_1d(xv, xlab, R, feasible, best, req)
% Two figures, each answering one question. Both use a linear y and, where the
% grid is logarithmically spaced, a log x -- otherwise the knee sits squashed
% against the left edge and looks like a cliff.
logx = numel(xv) > 2 && ...
       std(diff(log(xv))) < 0.25*std(diff(xv))/mean(xv);

% --- figure 1: does it work, and how well -------------------------------
figure('Color','w','Name','Sweep: robustness and accuracy');
yyaxis left
h1 = errorbar(xv, R.pFail, R.pFail_se, '-o', 'LineWidth', 1.3);
ylabel('P_{fail}');
ylim([0, max(0.05, min(1, max(R.pFail)+max(R.pFail_se)+eps))]);
yline(req.pFail, ':', 'P_{fail} requirement', 'HandleVisibility','off');
yyaxis right
h2 = plot(xv, R.rms_p95, '-s', 'LineWidth', 1.3); hold on;
h3 = plot(xv, R.bias_p95, '--', 'LineWidth', 1.0);
h4 = plot(xv, R.ripple_p95, ':', 'LineWidth', 1.0);
yline(req.rms_err, 'r-', 'accuracy requirement', 'LineWidth', 1.2, 'HandleVisibility','off');
ylabel('rate error / rad s^{-1}');
xlabel(xlab);
if logx, set(gca,'XScale','log'); end
mark_best(xv(best));
shade_infeasible(xv, feasible);
legend([h1 h2 h3 h4], {'P_{fail}','RMS error 95th pct','|bias| 95th pct', ...
                       'ripple pp 95th pct'}, 'Location','best');
grid on; title('Robustness and steady accuracy vs design parameter');

% --- figure 2: what it costs the actuator -------------------------------
figure('Color','w','Name','Sweep: actuator cost');
yyaxis left
plot(xv, 100*R.travel_p95, '-o', 'LineWidth', 1.3); hold on;
yline(100*req.travel, 'r--', 'travel requirement', 'LineWidth', 1.2, 'HandleVisibility','off');
ylabel('Travel used, 95th pct / % of stop'); ylim([0 105]);
yyaxis right
plot(xv, R.activity_p95, '-s', 'LineWidth', 1.3);
ylabel('Surface activity, 95th pct / rad s^{-1}');
xlabel(xlab);
if logx, set(gca,'XScale','log'); end
mark_best(xv(best));
shade_infeasible(xv, feasible);
grid on;
title({'Actuator cost vs design parameter', ...
       'Activity rising faster than travel means the extra gain is going into chatter, not control.'});
end


% ======================================================================
function plot_2d(v1, v2, n1, n2, R, feasible, best, req)
% Heatmaps are drawn on INDEX axes with the real values as tick labels.
% imagesc(v1, v2, Z) maps the axes linearly, so a logspace grid would be drawn
% with the wrong spacing and every contour would land in the wrong place. Index
% axes are correct for any spacing.
nx = numel(v1); ny = numel(v2);
Z = @(v) reshape(v, ny, nx);

panels = { R.rms_p95,      'RMS rate error 95th pct / rad s^{-1}', req.rms_err
           R.travel_p95,   'Travel used 95th pct / fraction',      req.travel
           R.pFail,        'P_{fail}',                             req.pFail
           R.activity_p95, 'Surface activity 95th pct / rad s^{-1}', [] };

figure('Color','w','Name','Sweep: 2-D grid');
tiledlayout(2,2,'TileSpacing','compact','Padding','compact');
for k = 1:size(panels,1)
    nexttile;
    M = Z(panels{k,1});
    imagesc(1:nx, 1:ny, M); axis xy; colorbar;
    set(gca,'XTick',1:nx,'XTickLabel',compose('%.3g',v1), ...
            'YTick',1:ny,'YTickLabel',compose('%.3g',v2));
    xlabel(n1,'Interpreter','none'); ylabel(n2,'Interpreter','none');
    title(panels{k,2});
    lim = panels{k,3};
    if ~isempty(lim) && any(M(:) < lim) && any(M(:) > lim)
        hold on;
        contour(1:nx, 1:ny, M, [lim lim], 'r', 'LineWidth', 2, 'ShowText','on');
    end
    hold on;
    [by, bx] = ind2sub([ny nx], best);
    plot(bx, by, 'kp', 'MarkerSize', 14, 'MarkerFaceColor','w', 'LineWidth',1.2);
end
sgtitle({'2-D design grid. Red contour = requirement boundary, star = selection.', ...
         'Index axes, so the tick labels are the real values whatever the spacing.'});

% --- feasible region, the actual decision output ------------------------
figure('Color','w','Name','Feasible region');
imagesc(1:nx, 1:ny, double(Z(feasible))); axis xy;
colormap(gca, [0.85 0.85 0.85; 0.55 0.75 0.55]); caxis([0 1]);
set(gca,'XTick',1:nx,'XTickLabel',compose('%.3g',v1), ...
        'YTick',1:ny,'YTickLabel',compose('%.3g',v2));
xlabel(n1,'Interpreter','none'); ylabel(n2,'Interpreter','none');
hold on;
[by, bx] = ind2sub([ny nx], best);
plot(bx, by, 'kp', 'MarkerSize', 16, 'MarkerFaceColor','y', 'LineWidth',1.5);
title({'Feasible region (green) and selection (star)', ...
       sprintf('pFail<=%.3f, RMS<=%.3f rad/s, travel<=%.0f%%', ...
               req.pFail, req.rms_err, 100*req.travel)});
end


% ======================================================================
function mark_best(x)
xline(x, 'k-.', 'selected', 'LineWidth', 1.2, 'HandleVisibility','off');
end

function shade_infeasible(xv, feasible)
% Grey bands where the requirements are not met, so the feasible span is
% readable without cross-referencing three curves.
bad = ~feasible(:)';
if ~any(bad), return; end
yl = ylim;
d  = [true diff(bad)~=0];
starts = find(d & bad);
for s = starts
    e = s; while e < numel(bad) && bad(e+1), e = e+1; end
    x1 = xv(max(s-0,1)); x2 = xv(min(e,numel(xv)));
    patch([x1 x2 x2 x1], [yl(1) yl(1) yl(2) yl(2)], [0.9 0.9 0.9], ...
          'EdgeColor','none','FaceAlpha',0.35,'HandleVisibility','off');
end
set(gca,'Children', flipud(get(gca,'Children')));
end
