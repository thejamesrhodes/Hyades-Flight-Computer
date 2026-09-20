function results = montecarlo_roll_control(opts)
% MONTECARLO_ROLL_CONTROL  Monte Carlo dispersion analysis of the roll loop.
%
%   results = montecarlo_roll_control()
%   results = montecarlo_roll_control(opts)
%     opts.overrides : M-by-2 cell {'Controller.c', value; ...} applied to the
%                      nominal P0 BEFORE dispersion. For design variables that
%                      are chosen, not dispersed.
%     opts.N, opts.seed, opts.quiet
%
%   WHAT CHANGED AND WHY (2026-08-24)
%
%   1  THE FLIGHT ENVELOPE IS NOW SAMPLED. Previously every one of the N runs
%      sat at the single nominal (q, V). That tests the controller at one point
%      and says nothing about the 1/q gain schedule, which is the entire reason
%      the schedule exists. Each run now draws a point from cfg.envelope.
%
%   2  Controller.q_e IS DERIVED FROM THE SAMPLED q, not from the nominal. It
%      is the controller's ESTIMATE of dynamic pressure, so its error must
%      multiply the truth. Dispersing it about a fixed nominal while q itself
%      moved would have conflated schedule error with estimation error.
%
%   3  THE DEAD ZONE, THE DELAY AND THE TRIM GAIN ARE DISPERSED. They were not,
%      and they are the elements that set the residual behaviour.
%
%   4  cfg.delta_limit IS GONE. It was 0.2618 rad against a mechanical travel
%      of 0.2094 rad, so the "authority margin" check compared the deflection
%      against a limit 25% beyond the stop the servo model already clamps to.
%      It could never fire. The metric is now travel = max|delta|/Servo.Travel,
%      which saturates at 1 and is a real margin.
%
%   5  stepinfo IS GONE -- see ROLL_METRICS for why (it returns NaN for every
%      run when ref = 0, and settling time is undefined for a loop that
%      converges to a band rather than a point).
%
%   6  THE DIVERGENCE TEST IS NO LONGER A MAGNITUDE THRESHOLD. It used to be
%      max|p| > 3*scale, and with ref = 0 and p(1) = 0 the scale guard made
%      that a flat 3 rad/s. But before the trim converges the loop sits at
%      p_droop = q*Cl_0/(Cl_delta*c + q*d*|Cl_p|/(2V)), which is 14 rad/s at
%      q = 40 kPa -- so four of the seven envelope points were reported as
%      "diverged" on every run, stably, whatever the gains. The measured
%      symptom was a pFail that did not respond to c, to the trim gains, or to
%      a 6x change in canard area. Stability is now judged by GROWTH in the
%      tail, which is what stability actually means.
%
%   FAST RESTART IS OFF BY DEFAULT AND MUST STAY OFF while Servo.Deadband,
%   Servo.Lash, Servo.Res or Servo.Tau_d are dispersed AND the actuator is the
%   native block build. Dead Zone limits, Backlash width and Delay length are
%   structural block parameters: under fast restart they are not re-evaluated,
%   so the dispersion would silently do nothing and the results would look
%   beautifully insensitive to the dominant nonlinearity. Run CHECK_TUNABILITY
%   once to confirm this on your install before trusting any deadband result.
%
%   Dependencies: Simulink. Parallel Computing Toolbox optional. No Control
%   System Toolbox, no Statistics Toolbox (unless sampler = "lhs").

% ======================================================================
% CONFIG
% ======================================================================
cfg.model   = 'Simulink_Roll_Model_1';
cfg.N       = 500;          % start here; go to 2000+ once the setup is trusted
cfg.seed    = 2025;
cfg.sampler = "mc";         % "mc" or "lhs"

cfg.sig_rate  = 'p_log';
cfg.sig_delta = 'delta_real_log';

cfg.fast_restart = false;   % see the note above. Leave false while the
                            % dead zone / lash / delay are dispersed.

% Flight envelope: each run draws one (q, V) pair, uniformly over these rows.
% Replace with points off your actual trajectory when you have one -- the
% controller is only ever required to work where the rocket actually goes.
cfg.envelope = [ ...
    2000   60
    5000  100
    7200  100
   12000  150
   20000  200
   32000  250
   40000  280 ];

% Requirement lines drawn on the plots. Set to [] to omit.
cfg.req.rms_err  = 0.5;     % rad/s RMS steady rate error
cfg.req.travel   = 0.5;     % fraction of mechanical travel, steady. Half the
                            % surface is reserved for the future angle loop.
cfg.req.theta_pp = deg2rad(10);  % rad pp. THE attitude-hold requirement: the
                            % angle excursion left after an outer loop absorbs
                            % the bias. 10 deg pp = +/-5 deg of hold.
cfg.req.activity = [];      % rad/s RMS surface rate, if you have a duty limit

% STABILITY TEST. Not a magnitude threshold -- see below.
cfg.growth_max = 1.15;   % tail-second-half RMS / tail-first-half RMS
cfg.p_cap      = 50;     % rad/s. Beyond ~8 rev/s the single-body, small-angle
                         % aero model is not valid anyway, so a run past this
                         % is discarded as out of model scope, not as a design
                         % failure.
cfg.req.p_max  = 1.0;    % rad/s. Steady |bias| above this is a PERFORMANCE
                         % failure (stable but useless), counted separately.

% Dispersion spec: {field, type, spread, [lo hi], base}
%   rel_gauss     v = nominal*(1 + spread*randn)
%   abs_gauss     v = nominal +  spread*randn
%   rel_gauss_of  v = (already-sampled value of 'base')*(1 + spread*randn)
cfg.disp_spec = {
  % --- vehicle / aero -----------------------------------------------------
    'Vehicle.Ixx',    'rel_gauss',    0.20,  [1e-5  Inf], ''
    'Aero.C_f',       'rel_gauss',    0.20,  [0     Inf], ''
    'Aero.y_Cp',      'rel_gauss',    0.10,  [0     Inf], ''
    'Aero.Cl_0',      'abs_gauss',    0.03,  [-0.25 0.25],''
  % --- actuator -----------------------------------------------------------
    'Servo.f_N',      'rel_gauss',    0.20,  [1     Inf], ''
    'Servo.Zeta',     'abs_gauss',    0.10,  [0.1   1.5], ''
    'Servo.Rate',     'rel_gauss',    0.25,  [1     Inf], ''
    'Servo.Deadband', 'rel_gauss',    0.30,  [0     Inf], ''
    'Servo.Tau_d',    'rel_gauss',    0.25,  [1e-4  Inf], ''
  % --- controller / trim --------------------------------------------------
    'Trim.Ki',        'rel_gauss',    0.20,  [0     Inf], ''
    'Controller.q_e', 'rel_gauss_of', 0.20,  [100   Inf], 'Aero.q'
};
% NOTE Servo.Lash is deliberately NOT here. Backlash is a per-unit build and
% wear quantity, not run-to-run noise: dispersing it Gaussian averages away the
% worn case that actually matters. Sweep it as discrete scenarios instead --
% see the SCENARIOS block in run_all.m.
%
% NOTE Aero.y_Cp is now rel_gauss 10%, not abs_gauss 0.005 m. On the old
% s = 20 mm canard 0.005 m was 13% of the arm; on the current s = 12 mm panel
% the same absolute spread would be 14% -- but it would no longer track if the
% geometry moves again. Relative is the invariant form.
% ======================================================================

%% 1. Setup
if nargin < 1 || isempty(opts), opts = struct(); end
if isfield(opts,'N')    && ~isempty(opts.N),    cfg.N    = opts.N;    end
if isfield(opts,'seed') && ~isempty(opts.seed), cfg.seed = opts.seed; end
cfg.quiet = isfield(opts,'quiet') && opts.quiet;

load_system(cfg.model);
P0 = parameters_initialise_roll_control();

cfg.ovtext = '';
if isfield(opts,'overrides') && ~isempty(opts.overrides)
    ov = opts.overrides;
    parts = cell(1,size(ov,1));
    for iov = 1:size(ov,1)
        s  = strsplit(ov{iov,1}, '.');
        P0 = setfield(P0, s{:}, ov{iov,2}); %#ok<SFLD>
        parts{iov} = sprintf('%s=%.6g', ov{iov,1}, ov{iov,2});
    end
    cfg.ovtext = strjoin(parts, ', ');
end

rng(cfg.seed);
cfg.ref = P0.Controller.ref;
cfg.P0  = P0;               % so the report can echo what was actually run

%% 2. Build the sample set
[in, X, Xnames, Pset] = build_inputs(cfg, P0);

%% 3. Run
if license('test','Distrib_Computing_Toolbox')
    out = parsim(in, 'ShowProgress','on', ...
                     'UseFastRestart', onoff(cfg.fast_restart), ...
                     'TransferBaseWorkspaceVariables','on');
else
    warning('No Parallel Computing Toolbox; running serially via sim().');
    out = sim(in);
end

%% 4. Metrics
m = extract_metrics(out, cfg, Pset);

%% 5. Aggregate and report
results        = aggregate_report(m, cfg);
results.cfg    = cfg;
results.raw    = m;
results.X      = X;
results.Xnames = Xnames;

%% 6. Plots
if ~cfg.quiet
    mc_plots(X, Xnames, m, cfg);
end
end


% ======================================================================
% LOCAL FUNCTIONS
% ======================================================================

function s = onoff(tf)
if tf, s = 'on'; else, s = 'off'; end
end


function [in, X, Xnames, Pset] = build_inputs(cfg, P0)
spec   = cfg.disp_spec;
names  = spec(:,1);
types  = spec(:,2);
spread = cell2mat(spec(:,3));
bounds = cell2mat(spec(:,4));
bases  = spec(:,5);
nP     = numel(names);
N      = cfg.N;
nEnv   = size(cfg.envelope,1);

nom = zeros(nP,1);
for i = 1:nP
    s = strsplit(names{i},'.');
    nom(i) = getfield(P0, s{:}); %#ok<GFLD>
end

switch lower(cfg.sampler)
    case "mc"
        Z = randn(N, nP);
    case "lhs"
        if ~license('test','Statistics_Toolbox')
            error('LHS sampler requires the Statistics & ML Toolbox.');
        end
        Z = norminv(lhsdesign(N, nP));
    otherwise
        error('Unknown sampler "%s" (use "mc" or "lhs").', cfg.sampler);
end
envIdx = randi(nEnv, N, 1);

% X carries the envelope columns too, so sensitivity analysis can see them.
Xnames  = [{'Aero.q'; 'Vehicle.V'}; names(:)];
X       = zeros(N, nP+2);
Pset    = cell(N,1);
in(1:N) = Simulink.SimulationInput(cfg.model);

for k = 1:N
    Pk = P0;
    % --- flight condition first: everything derived from q must see it -----
    Pk.Aero.q    = cfg.envelope(envIdx(k),1);
    Pk.Vehicle.V = cfg.envelope(envIdx(k),2);
    X(k,1) = Pk.Aero.q;  X(k,2) = Pk.Vehicle.V;

    for i = 1:nP
        switch types{i}
            case 'rel_gauss'
                v = nom(i)*(1 + spread(i)*Z(k,i));
            case 'abs_gauss'
                v = nom(i) +  spread(i)*Z(k,i);
            case 'rel_gauss_of'
                sb = strsplit(bases{i},'.');
                v  = getfield(Pk, sb{:}) * (1 + spread(i)*Z(k,i)); %#ok<GFLD>
            otherwise
                error('Unknown dispersion type "%s".', types{i});
        end
        v = min(max(v, bounds(i,1)), bounds(i,2));
        X(k,i+2) = v;
        s  = strsplit(names{i},'.');
        Pk = setfield(Pk, s{:}, v); %#ok<SFLD>
    end
    Pset{k} = Pk;
    in(k)   = in(k).setVariable('P', Pk);
end
end


function m = extract_metrics(out, cfg, Pset)
N = numel(out);
f = {'rms_err','bias','ripple_pp','theta_ripple_pp','ripple_freq', ...
     'ripple_unresolved', ...
     'peak_delta','travel_peak','travel_steady','activity','growth', ...
     'peak_rate','settle','overshoot'};
for i = 1:numel(f), m.(f{i}) = nan(N,1); end
m.ok        = false(N,1);
m.errored   = false(N,1);
m.diverged  = false(N,1);
m.nonfinite = false(N,1);
m.pinned    = false(N,1);   % touched the mechanical stop at any point
m.overspec  = false(N,1);   % stable, but steady error beyond the requirement

for k = 1:N
    if ~isempty(out(k).ErrorMessage)
        m.errored(k) = true; continue;
    end
    L  = out(k).logsout;
    pr = L.get(cfg.sig_rate).Values;   t = pr.Time;  p = pr.Data(:);
    dd = L.get(cfg.sig_delta).Values;  d = dd.Data(:);

    if any(~isfinite(p)) || any(~isfinite(d))
        m.nonfinite(k) = true; continue;
    end
    r = roll_metrics(t, p, d, cfg.ref, Pset{k});

    % Unstable = still growing at the end, or past model validity. NOT "big".
    if r.growth > cfg.growth_max || r.peak_rate > cfg.p_cap
        m.diverged(k) = true;
        for i = 1:numel(f), m.(f{i})(k) = r.(f{i}); end   % keep them for the plots
        continue;
    end

    for i = 1:numel(f), m.(f{i})(k) = r.(f{i}); end
    m.pinned(k)  = r.travel_peak >= 0.999;
    m.overspec(k)= abs(r.bias) > cfg.req.p_max;
    m.ok(k)      = true;
end
end


function R = aggregate_report(m, cfg)
N     = numel(m.ok);
nOK   = sum(m.ok);
pfail = 1 - nOK/N;

R.N        = N;
R.nStable  = nOK;
R.pFail    = pfail;
R.pFail_se = sqrt(pfail*(1-pfail)/N);
R.nErrored   = sum(m.errored);
R.nDiverged  = sum(m.diverged);
R.nNonfinite = sum(m.nonfinite);
R.nPinned    = sum(m.pinned);
R.nOverSpec  = sum(m.overspec);

R.rms_mean     = mean(m.rms_err(m.ok),'omitnan');
R.rms_p95      = pct(m.rms_err(m.ok),   95);
R.bias_p95     = pct(abs(m.bias(m.ok)), 95);
R.ripple_p95   = pct(m.ripple_pp(m.ok), 95);
R.theta_p95    = pct(m.theta_ripple_pp(m.ok), 95);
R.freq_med     = pct(m.ripple_freq(m.ok), 50);
R.fracUnres    = mean(m.ripple_unresolved(m.ok) > 0);
R.travel_p95   = pct(m.travel_steady(m.ok), 95);   % steady: the design question
R.travelpk_p95 = pct(m.travel_peak(m.ok),   95);   % peak: transient saturation
R.activity_p95 = pct(m.activity(m.ok),  95);
R.settle_p95   = pct(m.settle(m.ok),    95);
R.over_p95     = pct(m.overshoot(m.ok), 95);
% A regulator with residual ripple never enters a band tighter than the ripple,
% so settle is legitimately Inf for most runs. Report the COUNT that settled --
% a percentile over a column of Infs is just NaN and looks like a bug.
R.nSettled     = sum(isfinite(m.settle(m.ok)));

if cfg.quiet, return; end

fprintf('\n===== Monte Carlo (N = %d, seed = %d, %d envelope points) =====\n', ...
        N, cfg.seed, size(cfg.envelope,1));
if isempty(cfg.ovtext)
    fprintf('Configuration    : nominal (no overrides)\n');
else
    fprintf('Configuration    : %s\n', cfg.ovtext);
end
fprintf('Key parameters   : c=%.4g  Trim.Ki=%.4g  BandK=%.3g  Clamp=%.2f deg  n=%.3g\n', ...
        cfg.P0.Controller.c, cfg.P0.Trim.Ki, cfg.P0.Trim.BandK, ...
        rad2deg(cfg.P0.Trim.Clamp), cfg.P0.Linkage.n);
fprintf('Geometry         : s=%.1f mm  Cr=%.1f mm  Ct=%.1f mm  y_Cp=%.5f m\n', ...
        1e3*cfg.P0.Vehicle.s, 1e3*cfg.P0.Vehicle.C_r, 1e3*cfg.P0.Vehicle.C_t, ...
        cfg.P0.Aero.y_Cp);
fprintf('Stable runs      : %d / %d   (P_fail = %.3f +/- %.3f, 1sigma)\n', ...
        nOK, N, R.pFail, R.pFail_se);
fprintf('Failures by mode : %d sim-error | %d unstable/out-of-scope | %d non-finite\n', ...
        R.nErrored, R.nDiverged, R.nNonfinite);
fprintf('Runs on the stop : %d  (travel = 100%% -- no authority margin left)\n', R.nPinned);
fprintf('--- steady behaviour, 95th percentile over stable runs ---\n');
fprintf('RMS rate error   : %8.4f rad/s   (mean %.4f)\n', R.rms_p95, R.rms_mean);
fprintf('  |bias|         : %8.4f rad/s   hang-off alone\n', R.bias_p95);
fprintf('  ripple pp      : %8.4f rad/s   limit cycle alone\n', R.ripple_p95);
fprintf('ANGLE ripple     : %8.2f deg pp  <- what an outer attitude loop is\n', ...
        rad2deg(R.theta_p95));
fprintf('                                 left with (bias detrended out).\n');
fprintf('  at             : %8.2f Hz      median dominant ripple frequency\n', R.freq_med);
if R.fracUnres > 0.2
    fprintf('  ** %.0f%% of runs are DRIFT-DOMINATED: the ripple frequency sits in the\n', 100*R.fracUnres);
    fprintf('     lowest FFT bins, so the tail has not converged and the angle\n');
    fprintf('     number above is measuring drift, not oscillation. Raise StopTime\n');
    fprintf('     (tail = 30%% of it; bin spacing = 1/tail) and re-run before\n');
    fprintf('     believing it. Rate metrics below are still valid.\n');
end
fprintf('Travel, steady   : %8.1f %%       <- the authority question\n', 100*R.travel_p95);
fprintf('Travel, peak     : %8.1f %%       (transient; saturation here is\n', 100*R.travelpk_p95);
fprintf('                                 normal while the trim converges)\n');
fprintf('Over spec        : %8d runs    stable but |bias| > %.2f rad/s\n', ...
        R.nOverSpec, cfg.req.p_max);
fprintf('Surface activity : %8.3f rad/s   RMS surface rate\n', R.activity_p95);
fprintf('Settling         : %8.3f s   (%d of %d runs ever entered the band;\n', ...
        R.settle_p95, R.nSettled, nOK);
fprintf('                                a low count is expected -- the residual\n');
fprintf('                                ripple is wider than the band. Read RMS.)\n');
if ~isempty(cfg.req.rms_err) && R.rms_p95 > cfg.req.rms_err
    fprintf('  ** 95th-pct RMS rate error exceeds the %.3f rad/s requirement\n', cfg.req.rms_err);
end
if ~isempty(cfg.req.theta_pp) && R.theta_p95 > cfg.req.theta_pp
    fprintf('  ** angle ripple %.1f deg pp exceeds the %.1f deg attitude-hold budget\n', ...
            rad2deg(R.theta_p95), rad2deg(cfg.req.theta_pp));
end
if ~isempty(cfg.req.travel) && R.travel_p95 > cfg.req.travel
    fprintf('  ** 95th-pct travel exceeds %.0f%% -- the canard is undersized or c is too high\n', ...
            100*cfg.req.travel);
end
fprintf('=============================================================\n\n');
end


function mc_plots(X, Xnames, m, cfg)
stab   = m.ok;
labels = strrep(Xnames, '_', '\_');

% ---- 1. Empirical CDFs of the four decision metrics ---------------------
% ECDF, not a histogram: percentiles are read off directly, there is no bin
% width to choose, and these distributions are strongly skewed (a few bad
% corners in a long tail) which is exactly where histograms mislead.
figure('Color','w','Name','Performance distributions');
tiledlayout(2,2,'TileSpacing','compact','Padding','compact');
plot_one_ecdf(m.rms_err(stab),   'RMS rate error / rad s^{-1}',            cfg.req.rms_err);
plot_one_ecdf(m.travel_steady(stab), 'Travel used, steady / fraction of stop', cfg.req.travel);
plot_one_ecdf(rad2deg(m.theta_ripple_pp(stab)), 'Angle ripple / deg pp', ...
              rad2deg(cfg.req.theta_pp));
plot_one_ecdf(m.activity(stab),  'Surface activity, RMS rate / rad s^{-1}',cfg.req.activity);
sgtitle(sprintf('Stable runs (%d of %d). Dashed = 95th pct, red = requirement.', ...
        sum(stab), numel(stab)));

% ---- 2. Where in the envelope does it hurt ------------------------------
% The plot the old script had no equivalent of, and the one that answers
% "are these canards big enough": performance against flight condition.
figure('Color','w','Name','Envelope');
tiledlayout(1,2,'TileSpacing','compact','Padding','compact');
nexttile;
envelope_panel(X(stab,1), X(stab,2), m.rms_err(stab), 'RMS rate error / rad s^{-1}');
nexttile;
envelope_panel(X(stab,1), X(stab,2), m.travel_steady(stab), 'Travel used, steady / fraction');
sgtitle('Performance across the sampled flight envelope (stable runs)');

% ---- 3. Authority vs accuracy ------------------------------------------
figure('Color','w','Name','Authority vs accuracy');
scatter(m.travel_steady(stab), m.rms_err(stab), 14, X(stab,1)/1000, 'filled', ...
        'MarkerFaceAlpha', 0.35);
xlabel('Travel used, steady / fraction of mechanical stop');
ylabel('RMS rate error / rad s^{-1}');
cb = colorbar; cb.Label.String = 'dynamic pressure q / kPa';
if ~isempty(cfg.req.travel),  xline(cfg.req.travel, 'r--','travel requirement');  end
if ~isempty(cfg.req.rms_err), yline(cfg.req.rms_err,'r--','accuracy requirement'); end
grid on;
title({'Authority vs accuracy', ...
       'Bottom-left is what you want. A wall on the right means the surface is saturating.'});

% ---- 4. Sensitivity: standardised regression coefficients ---------------
% SRC rather than a scatter matrix: with 13 inputs a gplotmatrix is 169 panels
% and unreadable. SRC^2 partitions output variance for independent inputs, so
% the bars are directly interpretable as importance.
mu = mean(X); sd = std(X); sd(sd==0) = 1;
Xs = (X - mu) ./ sd;
mets   = {m.rms_err(stab), m.travel_steady(stab), m.activity(stab)};
mnames = {'RMS rate error', 'Travel used (steady)', 'Surface activity'};
figure('Color','w','Name','Sensitivity');
tiledlayout(1,numel(mets),'TileSpacing','compact','Padding','compact');
nIn = size(X,2);
Xstab = Xs(stab,:);
for j = 1:numel(mets)
    y = mets{j}; good = ~isnan(y) & isfinite(y);
    nexttile;
    if nnz(good) < nIn + 2
        axis off; title({mnames{j}, '(insufficient data)'}); continue;
    end
    Xj = Xstab(good,:);
    ys = (y(good) - mean(y(good))) / max(std(y(good)), eps);
    A  = [ones(nnz(good),1), Xj];
    b  = A \ ys;
    beta = b(2:end);
    R2   = 1 - sum((ys - A*b).^2) / sum(ys.^2);
    [~, ord] = sort(abs(beta), 'descend');
    barh(beta(ord));
    yticks(1:nIn); yticklabels(labels(ord)); set(gca,'YDir','reverse');
    xlabel('SRC (\beta per input SD)');
    title(sprintf('%s  (R^2 = %.2f)', mnames{j}, R2));
    grid on;
end
sgtitle({'Standardised regression coefficients, stable runs', ...
         'Low R^2 means the response is nonlinear -- read the envelope panels instead.'});

% ---- 5. Failure breakdown, only when there is something to show ---------
nF = [sum(m.errored) sum(m.diverged) sum(m.nonfinite) sum(m.pinned) sum(m.overspec)];
if any(nF)
    cats = {'sim error','unstable','non-finite','touched stop','over spec'};
    figure('Color','w','Name','Failures');
    bar(categorical(cats, cats), nF);
    ylabel('Runs'); grid on;
    title('Failure and saturation counts');
end
end


function plot_one_ecdf(x, lab, req)
nexttile;
x = sort(x(~isnan(x) & isfinite(x)));
if isempty(x), axis off; title({lab,'(no data)'}); return; end
y = (1:numel(x))'/numel(x);
stairs(x, y, 'LineWidth', 1.2); hold on;
p95 = x(max(1,ceil(0.95*numel(x))));
xline(p95, 'k--', sprintf('p95 = %.3g', p95));
if ~isempty(req), xline(req, 'r-', 'requirement', 'LineWidth', 1.2); end
ylabel('F(x)'); xlabel(lab); ylim([0 1]); grid on;
end


function envelope_panel(q, V, z, lab)
% Mean of z in each (q, V) cell. The envelope is a small discrete set of
% points, so a scatter of raw runs just overplots -- one marker per condition,
% sized by count and coloured by the mean, shows both coverage and outcome.
[u, ~, ic] = unique([q V], 'rows');
mz = accumarray(ic, z, [], @mean);
nz = accumarray(ic, 1);
scatter(u(:,1)/1000, u(:,2), 40 + 200*nz/max(nz), mz, 'filled', ...
        'MarkerEdgeColor','k');
xlabel('dynamic pressure q / kPa'); ylabel('airspeed V / m s^{-1}');
cb = colorbar; cb.Label.String = lab;
grid on; title(lab);
for i = 1:size(u,1)
    text(u(i,1)/1000, u(i,2), sprintf('  %.3g', mz(i)), 'FontSize', 8);
end
end


function p = pct(x, qq)
x = x(~isnan(x) & isfinite(x));
if isempty(x), p = NaN; return; end
x = sort(x);
p = x(max(1, ceil(qq/100*numel(x))));
end
