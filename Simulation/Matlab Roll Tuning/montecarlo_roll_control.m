function results = montecarlo_roll_control(opts)
% MONTECARLO_ROLL_CONTROL  Monte Carlo dispersion analysis of the roll-control model.
%
%   results = montecarlo_roll_control() samples the parameter struct P, runs N
%   simulations of the Simulink model, extracts per-run performance metrics,
%   and returns a struct of raw and aggregated results.
%
%   results = montecarlo_roll_control(opts) accepts an options struct used by
%   the gain-sweep wrapper (and any caller):
%     opts.overrides : M-by-2 cell {'Controller.c', value; ...} applied to the
%                      nominal P0 BEFORE dispersion. Use for design variables
%                      that are chosen, not dispersed (e.g. controller gains).
%     opts.N         : override cfg.N for this call.
%     opts.seed      : override cfg.seed for this call.
%     opts.quiet     : true to suppress all printing and figures (for sweeps).
%
%   Dependencies
%     - Simulink                       (required)
%     - Parallel Computing Toolbox     (optional; parsim runs serially without it)
%     - Control System Toolbox         (stepinfo)
%     - Statistics & ML Toolbox        (gplotmatrix; only if SAMPLER = "lhs")
%
%   Before running, set everything in the CONFIG block to match your model:
%   model name, logged signal names, the commanded reference, and the servo
%   deflection limit. These are the only project-specific values.

% ======================================================================
% CONFIG  -- edit these to match your model
% ======================================================================
cfg.model        = 'Simulink_Roll_Model_1';  % model file name (no .slx)
cfg.N            = 4000;                        % number of runs
cfg.seed         = 2025;                       % RNG seed (record this)
cfg.sampler      = "mc";                        % "mc" or "lhs"

cfg.sig_rate     = 'p_log';            % logged signal name: roll rate  / rad s^-1
cfg.sig_delta    = 'delta_real_log';   % logged signal name: canard deflection / rad
cfg.ref          = [];        % commanded roll rate the step targets / rad s^-1
cfg.delta_limit  = 0.2618;     % servo travel limit (+/-) for margin check / rad (~15 deg)
cfg.div_factor   = 3;          % divergence test: |p| > div_factor*scale => unstable

% Dispersion specification:
%   {field path , type , spread , [lo hi] physical guard}
%   rel_gauss : nominal*(1 + spread*randn)   (fractional uncertainty)
%   abs_gauss : nominal +  spread*randn       (additive uncertainty, same units)
cfg.disp_spec = {
    'Vehicle.Mass',  'rel_gauss', 0.05,   [0.3    Inf]
    'Vehicle.Ixx',   'rel_gauss', 0.20,   [1e-5   Inf]
    'Servo.f_N',     'rel_gauss', 0.20,   [1      Inf]
    'Servo.Zeta',    'abs_gauss', 0.10,   [0.1    1.5]
    'Servo.Rate',    'rel_gauss', 0.25,   [1      Inf]
    'Aero.C_f',      'rel_gauss', 0.20,   [0      Inf]
    'Aero.y_Cp',     'abs_gauss', 0.005,  [0      Inf]
    'Aero.Cl_0',     'abs_gauss', 0.01,   [-0.25 0.25]
    'Controller.q_e','rel_gauss', 0.2,    [100,   Inf]
};
% ======================================================================

%% 1. Setup
if nargin < 1 || isempty(opts), opts = struct(); end
if isfield(opts,'N')    && ~isempty(opts.N),    cfg.N    = opts.N;    end
if isfield(opts,'seed') && ~isempty(opts.seed), cfg.seed = opts.seed; end
cfg.quiet = isfield(opts,'quiet') && opts.quiet;   % suppress prints/plots

load_system(cfg.model);
P0 = parameters_initialise_roll_control();          % nominal parameters

% Apply caller overrides to the nominal P0 BEFORE dispersion. Used to sweep
% design variables (e.g. Controller.c) that are NOT in disp_spec, so they
% inherit straight into every dispersed copy Pk = P0.
if isfield(opts,'overrides') && ~isempty(opts.overrides)
    ov = opts.overrides;
    for iov = 1:size(ov,1)
        s = strsplit(ov{iov,1}, '.');
        P0 = setfield(P0, s{:}, ov{iov,2}); %#ok<SFLD>
    end
end

rng(cfg.seed);                                       % reproducibility

cfg.ref = P0.Controller.ref; %Consistency

%% 2. Build the sample set as an array of SimulationInput objects
[in, X, Xnames] = build_inputs(cfg, P0);

%% 3. Run
if license('test','Distrib_Computing_Toolbox')
    out = parsim(in, 'ShowProgress','on', ...
                     'UseFastRestart','off', ...
                     'TransferBaseWorkspaceVariables','on');
else
    warning('No Parallel Computing Toolbox; running serially via sim().');
    out = sim(in);                                   % same interface, serial
end

%% 4. Extract per-run metrics
m = extract_metrics(out, cfg);

%% 5. Aggregate, report, plot
results = aggregate_report(m, cfg);
results.cfg     = cfg;
results.raw     = m;
results.X       = X;
results.Xnames  = Xnames;

%% 6. Sensitivity / diagnostic plots (need the sampled inputs)
if ~cfg.quiet
    sensitivity_plots(X, Xnames, m, cfg);
end
end


% ======================================================================
% LOCAL FUNCTIONS
% ======================================================================

function [in, X, Xnames] = build_inputs(cfg, P0)
% Construct N SimulationInput objects, each with a dispersed copy of P.
% Also return X (N-by-nP matrix of the sampled values) and Xnames, so the
% inputs can be correlated against outcomes for sensitivity analysis.
spec   = cfg.disp_spec;
names  = spec(:,1);
types  = spec(:,2);
spread = cell2mat(spec(:,3));
bounds = cell2mat(spec(:,4));
nP     = numel(names);
N      = cfg.N;

% Nominal value for each dispersed field, pulled from P0
nom = zeros(nP,1);
for i = 1:nP
    s = strsplit(names{i},'.');
    nom(i) = getfield(P0, s{:}); %#ok<GFLD>
end

% Standard-normal draws: rows = runs, cols = parameters
switch lower(cfg.sampler)
    case "mc"
        Z = randn(N, nP);
    case "lhs"
        if ~license('test','Statistics_Toolbox')
            error('LHS sampler requires the Statistics & ML Toolbox.');
        end
        U = lhsdesign(N, nP);        % uniform [0,1] LHS
        Z = norminv(U);              % map to standard normal
    otherwise
        error('Unknown sampler "%s" (use "mc" or "lhs").', cfg.sampler);
end

Xnames = names;
X      = zeros(N, nP);
in(1:N) = Simulink.SimulationInput(cfg.model);
for k = 1:N
    Pk = P0;
    for i = 1:nP
        switch types{i}
            case 'rel_gauss', v = nom(i)*(1 + spread(i)*Z(k,i));
            case 'abs_gauss', v = nom(i) +  spread(i)*Z(k,i);
            otherwise, error('Unknown dispersion type "%s".', types{i});
        end
        v = min(max(v, bounds(i,1)), bounds(i,2));   % physical guard (clip)
        X(k,i) = v;                                  % record the sampled value
        s = strsplit(names{i},'.');
        Pk = setfield(Pk, s{:}, v); %#ok<SFLD>
    end
    in(k) = in(k).setVariable('P', Pk);
end
end


function m = extract_metrics(out, cfg)
% Pull settling time, overshoot, steady-state error and peak deflection
% from each run. Failed or divergent runs are flagged ok(k)=false and
% leave NaN metrics.
N = numel(out);
m.settle    = nan(N,1);
m.overshoot = nan(N,1);
m.ssErr     = nan(N,1);
m.peakDelta = nan(N,1);
m.ok        = false(N,1);
m.errored   = false(N,1);
m.diverged  = false(N,1);    % finite but exceeded the magnitude threshold
m.nonfinite = false(N,1);    % NaN or Inf in the response

for k = 1:N
    if ~isempty(out(k).ErrorMessage)
        m.errored(k) = true;            % sim threw -> counts as failure
        continue;
    end

    L  = out(k).logsout;
    pr = L.get(cfg.sig_rate).Values;    t = pr.Time;  p = pr.Data;
    dd = L.get(cfg.sig_delta).Values;   d = dd.Data;

    % Divergence / NaN test. Scale the threshold by the larger of the
    % reference and the initial condition, so a regulator (ref = 0)
    % decaying from a nonzero initial rate is handled correctly. Using
    % ref alone collapses the threshold to zero when ref = 0.
    p0    = abs(p(1));
    scale = max(abs(cfg.ref), p0);
    if scale == 0, scale = 1; end                  % guard: all-zero response
    if any(~isfinite(p))
        m.nonfinite(k) = true;          % NaN/Inf -> numerical blow-up
        continue;
    end
    if max(abs(p)) > cfg.div_factor*scale
        m.diverged(k) = true;           % grew beyond threshold -> unstable
        continue;
    end
    m.ok(k) = true;

    % stepinfo needs the initial value passed explicitly. For a regulator
    % (ref = 0) the default yinit = 0 implies a zero-amplitude step and
    % returns NaN for settling time and overshoot. p(1) is the initial rate.
    si             = stepinfo(p, t, cfg.ref, p(1));   % Control System Toolbox
    m.settle(k)    = si.SettlingTime;
    m.overshoot(k) = si.Overshoot;
    m.ssErr(k)     = p(end) - cfg.ref;
    m.peakDelta(k) = max(abs(d));
end
end


function R = aggregate_report(m, cfg)
% Summarise, print, and plot the key distributions. Percentiles are taken
% over the stable runs only, with NaN explicitly removed (a run can be ok
% yet have NaN settling time if it never settled within the sim window).
N     = numel(m.ok);
nOK   = sum(m.ok);
pfail = 1 - nOK/N;
se    = sqrt(pfail*(1-pfail)/N);        % 1-sigma std error of the failure rate

R.N        = N;
R.nStable  = nOK;
R.pFail    = pfail;
R.pFail_se = se;
R.settle_mean   = mean(m.settle,'omitnan');
R.settle_p95    = pct(m.settle(m.ok), 95);
R.over_mean     = mean(m.overshoot,'omitnan');
R.over_p95      = pct(m.overshoot(m.ok), 95);
R.delta_p95     = pct(m.peakDelta(m.ok), 95);
R.ssErr_mean    = mean(m.ssErr,'omitnan');
R.ssErr_std     = std(m.ssErr,'omitnan');

R.nErrored   = sum(m.errored);
R.nDiverged  = sum(m.diverged);
R.nNonfinite = sum(m.nonfinite);

if cfg.quiet, return; end   % sweep mode: compute R but skip prints/figures

fprintf('\n===== Monte Carlo summary (N = %d, seed = %d) =====\n', N, cfg.seed);
fprintf('Stable runs      : %d / %d   (P_fail = %.3f +/- %.3f, 1sigma)\n', ...
         nOK, N, pfail, se);
fprintf('Failures by mode : %d sim-error | %d diverged | %d non-finite\n', ...
         R.nErrored, R.nDiverged, R.nNonfinite);
fprintf('Settling time    : mean %.3f   95th pct %.3f  s\n', R.settle_mean, R.settle_p95);
fprintf('Overshoot        : mean %.1f   95th pct %.1f  %%\n', R.over_mean, R.over_p95);
fprintf('Peak |delta|     : 95th pct %.3f  rad   (limit %.3f)\n', R.delta_p95, cfg.delta_limit);
if R.delta_p95 > cfg.delta_limit
    fprintf('  ** WARNING: 95th-pct deflection exceeds servo limit -- control authority margin thin.\n');
end
fprintf('Steady-state err : mean %.4f   std %.4f  rad/s\n', R.ssErr_mean, R.ssErr_std);
fprintf('====================================================\n\n');

figure; histogram(m.settle);    xlabel('Settling time / s');       ylabel('Count'); title('Settling time');
figure; histogram(m.overshoot); xlabel('Overshoot / %');           ylabel('Count'); title('Overshoot');
figure; histogram(m.peakDelta); xlabel('Peak |\delta| / rad');     ylabel('Count'); title('Peak deflection');
hold on; xline(cfg.delta_limit,'r--','servo limit'); hold off;
end


function sensitivity_plots(X, Xnames, m, cfg)
% Two complementary importance analyses:
%   (A) Stability drivers (point-biserial) -- only meaningful when the run set
%       contains BOTH stable and unstable cases. Conditional on the assumed
%       dispersions (a deliberately wide spread will rank higher because it
%       exercises more of the parameter's effect).
%   (B) Standardised Regression Coefficients (SRC) on the continuous metrics
%       over the stable population. Inputs and output are standardised to unit
%       variance, so each coefficient is effect-per-input-SD and (for
%       independent inputs) SRC^2 partitions output variance. This is the
%       distribution-normalised importance measure and is the right view when
%       nothing fails (pFail = 0), where (A) is degenerate.
nP     = size(X,2);
stab   = m.ok;
nU     = sum(~stab);
nS     = sum(stab);
labels = strrep(Xnames, '_', '\_');
both_classes = (nU > 0) && (nU < numel(stab));

% Standardise inputs once (zero mean, unit variance)
mu = mean(X); sd = std(X); sd(sd==0) = 1;
Xs = (X - mu) ./ sd;

% ---- (A1) Stability scatter matrix, coloured stable/unstable -------------
if both_classes
    if exist('gplotmatrix','file') == 2
        grp = repmat("unstable", numel(stab), 1); grp(stab) = "stable";
        figure;
        gplotmatrix(X, [], grp, [], [], [], 'on', 'hist', labels);
        sgtitle('Input scatter matrix, coloured by stability');
    else
        figure;
        for i = 1:nP
            subplot(ceil(nP/2), 2, i);
            scatter(X(stab,i),  ones(nS,1),  12, 'b', 'filled'); hold on;
            scatter(X(~stab,i), zeros(nU,1), 12, 'r', 'filled');
            yticks([0 1]); yticklabels({'unstable','stable'}); ylim([-0.5 1.5]);
            xlabel(labels{i}); title(labels{i});
        end
        sgtitle('Stability outcome vs each input parameter');
    end

    % ---- (A2) Stability driver ranking (point-biserial) ------------------
    r = corr(X, double(stab));
    [~, ord] = sort(abs(r), 'descend');
    figure;
    barh(r(ord));
    yticks(1:nP); yticklabels(labels(ord)); set(gca,'YDir','reverse');
    xlabel('Point-biserial r  with stability');
    title({'Stability influence (conditional on assumed dispersions)', ...
           'wider-dispersed params rank higher by construction'});
    grid on;
else
    fprintf(['sensitivity_plots: all runs share one outcome (%d stable, ' ...
             '%d unstable). Stability scatter and point-biserial ranking ' ...
             'are undefined here -- showing metric SRC instead.\n'], nS, nU);
end

% ---- (B) SRC of the continuous metrics over the stable runs --------------
mets   = {m.settle(stab), m.overshoot(stab), m.peakDelta(stab)};
mnames = {'Settling time', 'Overshoot', 'Peak |\delta|'};
Xs_s   = Xs(stab,:);
figure;
for j = 1:numel(mets)
    y    = mets{j};
    good = ~isnan(y);
    if nnz(good) < nP + 2          % too few points for a stable regression
        subplot(1,numel(mets),j); axis off;
        title({mnames{j}, '(insufficient data)'}); continue;
    end
    Xj = Xs_s(good,:);
    ys = (y(good) - mean(y(good))) / std(y(good));
    A  = [ones(nnz(good),1), Xj];
    b  = A \ ys;
    beta = b(2:end);                        % standardised regression coeffs
    R2   = 1 - sum((ys - A*b).^2) / sum(ys.^2);
    [~, ord] = sort(abs(beta), 'descend');
    subplot(1, numel(mets), j);
    barh(beta(ord));
    yticks(1:nP); yticklabels(labels(ord)); set(gca,'YDir','reverse');
    xlabel('SRC (\beta per input SD)');
    title(sprintf('%s  (R^2 = %.2f)', mnames{j}, R2));
    grid on;
end
sgtitle({'Standardised regression coefficients (stable runs)', ...
         'distribution-normalised importance; low R^2 => linear model weak, read scatter matrix'});

% ---- (C) Empirical CDFs of the metrics over the stable population --------
figure;
subplot(1,3,1); plot_ecdf(m.settle(stab));    xlabel('Settling time / s');  title('Settling');
subplot(1,3,2); plot_ecdf(m.overshoot(stab)); xlabel('Overshoot / %');      title('Overshoot');
subplot(1,3,3); plot_ecdf(m.peakDelta(stab)); xlabel('Peak |\delta| / rad'); title('Peak deflection');
hold on; xline(cfg.delta_limit,'r--','limit'); hold off;
sgtitle('Empirical CDFs (stable runs)');

% ---- (D) Trade-off cloud: overshoot vs settling, colour = peak deflection
figure;
scatter(m.settle(stab), m.overshoot(stab), 18, m.peakDelta(stab), 'filled');
xlabel('Settling time / s'); ylabel('Overshoot / %');
cb = colorbar; cb.Label.String = 'Peak |\delta| / rad';
title('Performance trade-off (stable runs)');
end


function p = pct(x, q)
% NaN-safe percentile: drop NaNs, return NaN if nothing left.
x = x(~isnan(x));
if isempty(x), p = NaN; else, p = prctile(x, q); end
end


function plot_ecdf(x)
% Minimal empirical CDF without the Statistics Toolbox ecdf().
x = sort(x(~isnan(x)));
if isempty(x), return; end
y = (1:numel(x))' / numel(x);
stairs(x, y); ylabel('F(x)'); ylim([0 1]); grid on;
end