function sweep = parameter_sweep_roll_control(cvec, tivec, Nper, seed, settle_req)
% PARAMETER_SWEEP_ROLL_CONTROL  Deterministic sweep of the controller gains
% Controller.c and Controller.Ti, with a Monte Carlo dispersion run nested at
% each grid point.
%
%   Separates the two analyses that share the same machinery:
%     - inner (stochastic): plant/servo/aero dispersion -> robustness (pFail)
%     - outer (deterministic): the gains are DESIGN CHOICES, swept on a grid
%
%   USAGE
%     sweep = parameter_sweep_roll_control(cvec)                       % c only
%     sweep = parameter_sweep_roll_control(cvec, tivec)               % c x Ti grid
%     sweep = parameter_sweep_roll_control(cvec, tivec, Nper, seed, settle_req)
%
%     cvec       vector of Controller.c values   (required)
%     tivec      vector of Controller.Ti values  (default [] -> hold Ti at nominal)
%     Nper       runs per grid point             (default 200)
%     seed       RNG seed, FIXED across all points (default 2025)
%     settle_req settling requirement / s, drawn on 1-D plots (optional)
%       cvec  = 50:50:300;        % 6 values
%       tivec = 2:2:12;           % 6 values
%       sweep = parameter_sweep_roll_control(cvec, tivec, 200, 2025, 0.5);
%
%
%   If both cvec and tivec have length > 1, a full 2-D grid is run and the
%   results are shown as heatmaps. If only one varies, a 1-D line sweep is run.
%
%   Common random numbers: the same seed and Nper are used at every grid point,
%   so the dispersed plant samples are identical across gains. Differences in
%   the outputs are then attributable to the gains alone, not to sampling noise.
%
%   Returns a table with one row per grid point: the gains, pFail and its
%   standard error, and the 95th-percentile settling, overshoot and peak
%   deflection. For the 2-D case the same data is also returned reshaped as
%   matrices in sweep.Properties.UserData for convenience.

% ---- argument handling ----
if nargin < 2,                  tivec = [];      end
if nargin < 3 || isempty(Nper), Nper  = 200;     end
if nargin < 4 || isempty(seed), seed  = 2025;    end
if nargin < 5,                  settle_req = [];  end

cvec = cvec(:);                    % force column (this was the bug: do NOT
                                   % re-index cvec with a literal range here)
sweep_ti = ~isempty(tivec);
if sweep_ti, tivec = tivec(:); else, tivec = NaN; end   % NaN -> nominal Ti

nC  = numel(cvec);
nT  = numel(tivec);
nPt = nC * nT;

is2D = (nC > 1) && sweep_ti && (nT > 1);

% ---- preallocate flat result columns ----
[Cgrid, Tgrid] = meshgrid(cvec, tivec);   % nT-by-nC
Cflat = Cgrid(:);  Tflat = Tgrid(:);
pFail      = nan(nPt,1);  pFail_se   = nan(nPt,1);
settleMean = nan(nPt,1);  settleP95  = nan(nPt,1);
overP95    = nan(nPt,1);  deltaP95   = nan(nPt,1);
nStable    = nan(nPt,1);
delta_limit = NaN;

fprintf('Sweeping %d point(s): %d c-value(s) x %d Ti-value(s), %d runs each (seed %d, CRN)...\n', ...
        nPt, nC, nT, Nper, seed);

% ---- main loop ----
for idx = 1:nPt
    opts          = struct();
    opts.N        = Nper;
    opts.seed     = seed;          % SAME every point -> common random numbers
    opts.quiet    = true;

    if sweep_ti
        opts.overrides = {'Controller.c',  Cflat(idx); ...
                          'Controller.Ti', Tflat(idx)};
    else
        opts.overrides = {'Controller.c',  Cflat(idx)};
    end

    R = montecarlo_roll_control(opts);

    pFail(idx)      = R.pFail;
    pFail_se(idx)   = R.pFail_se;
    settleMean(idx) = R.settle_mean;
    settleP95(idx)  = R.settle_p95;
    overP95(idx)    = R.over_p95;
    deltaP95(idx)   = R.delta_p95;
    nStable(idx)    = R.nStable;
    if idx == 1, delta_limit = R.cfg.delta_limit; end

    if sweep_ti
        fprintf('  c=%7.2f Ti=%6.2f : pFail=%.3f+/-%.3f  settle95=%6.3f s  over95=%6.1f%%  delta95=%.3f\n', ...
                Cflat(idx), Tflat(idx), pFail(idx), pFail_se(idx), settleP95(idx), overP95(idx), deltaP95(idx));
    else
        fprintf('  c=%7.2f : pFail=%.3f+/-%.3f  settle95=%6.3f s  over95=%6.1f%%  delta95=%.3f\n', ...
                Cflat(idx), pFail(idx), pFail_se(idx), settleP95(idx), overP95(idx), deltaP95(idx));
    end
end

% ---- assemble table ----
sweep = table(Cflat, Tflat, nStable, pFail, pFail_se, settleMean, settleP95, overP95, deltaP95, ...
    'VariableNames', {'c','Ti','nStable','pFail','pFail_se','settle_mean','settle_p95','over_p95','delta_p95'});

% ---- plotting ----
if is2D
    plot_grid(cvec, tivec, reshapeT(pFail,nT,nC),     'P_{fail}',                  delta_limit, false);
    plot_grid(cvec, tivec, reshapeT(settleP95,nT,nC), 'Settling 95th pct / s',     delta_limit, false);
    plot_grid(cvec, tivec, reshapeT(overP95,nT,nC),   'Overshoot 95th pct / %',    delta_limit, false);
    plot_grid(cvec, tivec, reshapeT(deltaP95,nT,nC),  'Peak |\delta| 95th pct / rad', delta_limit, true);
    sweep.Properties.UserData = struct('cvec',cvec,'tivec',tivec, ...
        'pFail',reshapeT(pFail,nT,nC),'settle_p95',reshapeT(settleP95,nT,nC), ...
        'over_p95',reshapeT(overP95,nT,nC),'delta_p95',reshapeT(deltaP95,nT,nC));
else
    % 1-D: x-axis is whichever gain varies
    if sweep_ti && nT > 1
        xv = tivec;  xlab = 'Integral setting  Ti';
    else
        xv = cvec;   xlab = 'Proportional gain constant  c';
    end
    plot_1d(xv, xlab, pFail, pFail_se, settleP95, settleMean, overP95, deltaP95, ...
            delta_limit, settle_req);
end
end


% ======================================================================
% LOCAL FUNCTIONS
% ======================================================================

function M = reshapeT(v, nT, nC)
% Flat column (meshgrid order, nT-by-nC) back to an nT-by-nC matrix.
M = reshape(v, nT, nC);
end


function plot_grid(cvec, tivec, Z, name, delta_limit, mark_limit)
% Heatmap of a metric over the (c, Ti) grid.
figure;
imagesc(cvec, tivec, Z); axis xy; colorbar;
xlabel('Proportional gain constant  c');
ylabel('Integral setting  Ti');
title(name);
if mark_limit && ~isnan(delta_limit)
    hold on;
    contour(cvec, tivec, Z, [delta_limit delta_limit], 'r', 'LineWidth', 2, ...
            'ShowText', 'on');
    title({name, sprintf('red contour = servo limit %.3f rad', delta_limit)});
end
end


function plot_1d(xv, xlab, pFail, pFail_se, settleP95, settleMean, overP95, deltaP95, ...
                 delta_limit, settle_req)
% Knee plot + overshoot/actuator plot for a 1-D sweep.
figure;
yyaxis left
errorbar(xv, pFail, pFail_se, '-o', 'LineWidth', 1.2);
ylabel('P_{fail}');
ylim([0, max(0.05, min(1, max(pFail)+max(pFail_se)+eps))]);
yyaxis right
plot(xv, settleP95, '-s', 'LineWidth', 1.2); hold on;
plot(xv, settleMean, '--', 'LineWidth', 1.0);
ylabel('Settling time / s');
if ~isempty(settle_req)
    yline(settle_req, 'r:', 'requirement', 'LineWidth', 1.2);
end
xlabel(xlab); title('Sweep: robustness vs speed');
legend({'P_{fail}','settling 95th pct','settling mean'}, 'Location','best');
grid on;

figure;
yyaxis left
plot(xv, overP95, '-o', 'LineWidth', 1.2);
ylabel('Overshoot 95th pct / %');
yyaxis right
plot(xv, deltaP95, '-s', 'LineWidth', 1.2); hold on;
if ~isnan(delta_limit)
    yline(delta_limit, 'r--', 'servo limit', 'LineWidth', 1.2);
end
ylabel('Peak |\delta| 95th pct / rad');
xlabel(xlab); title('Sweep: overshoot and actuator demand');
grid on;
end