function [C_opt, info] = optimise_roll_schedule(P, cfg)
%OPTIMISE_ROLL_SCHEDULE  Pick the proportional schedule constant C.
%
%   [C_opt, info] = optimise_roll_schedule()
%   [C_opt, info] = optimise_roll_schedule(P)
%   [C_opt, info] = optimise_roll_schedule(P, cfg)
%
%   Control law:  delta_cmd = -Kp(q)*p ,  Kp(q) = C/q
%
%   The 1/q schedule holds the open-loop CROSSOVER constant at wc = C*k_b.
%   That is the design target, NOT the closed-loop -3 dB bandwidth, which also
%   depends on the plant pole a = k_a*q/V and so varies across the envelope
%   wherever passive damping (Cl_p) is strong.
%     * analytic:  C0 = w_target/k_b
%     * numerical: confirm the stability margins near C0 survive independent
%                  +/-30% errors in Cl_delta and Cl_p across the envelope.
%
%   No integral term: rate damping has no setpoint error to integrate out. A
%   standing Cl_0 is cancelled by the bounded trim path (see CONTROLLER), not
%   by integral action in the damping loop.
%
%   WHAT CHANGED (2026-08-24). Every aero and actuator constant is now read
%   from P instead of being hard-coded. The previous version carried
%       Cl_delta_nom = 0.6694   (correct value 0.2975)
%       Cl_p_nom     = -1.18    (correct value -0.4291)
%       Ixx          = 0.00061  (P has 0.0007)
%       wn           = 2*pi*20  (P.Servo.f_N is 80)
%   and a comment quoting a third figure, Cl_p = -5.778. Since C_opt scales as
%   1/Cl_delta, a stale Cl_delta was a direct multiplicative error on the
%   answer -- the 0.6694 above is 2.25x the truth, so C was 2.25x too low. The
%   coefficients are derived here from the same Barrowman expressions the plant
%   and the actuator use, so the three cannot drift apart again.

if nargin < 1 || isempty(P),   P   = parameters_initialise_roll_control(); end
if nargin < 2 || isempty(cfg), cfg = struct(); end

% ---- targets and uncertainties -------------------------------------------
if ~isfield(cfg,'w_target'),  cfg.w_target = 2*pi*4; end   % crossover / rad s^-1
if ~isfield(cfg,'PM_min'),    cfg.PM_min   = 35;     end   % deg
if ~isfield(cfg,'GM_min'),    cfg.GM_min   = 2.0;    end   % ratio (6 dB)
if ~isfield(cfg,'Ms_max'),    cfg.Ms_max   = 2.0;    end
if ~isfield(cfg,'unc_delta'), cfg.unc_delta = 0.30;  end
if ~isfield(cfg,'unc_p'),     cfg.unc_p     = 0.30;  end
if ~isfield(cfg,'plot'),      cfg.plot      = true;  end
if ~isfield(cfg,'quiet'),     cfg.quiet     = false; end
% Flight condition set. cfg.env is an N-by-2 list of PAIRED (q, V) points and
% is the default, because the outer product of a q list and a V list is mostly
% fiction: with q from 2 to 40 kPa and V from 40 to 280 m/s, 59% of the grid
% implies an air density outside 0.15-1.3 kg/m^3, and the corner that used to
% set the gain -- q = 2 kPa at V = 280 m/s -- implies rho = 0.051 kg/m^3, about
% 35 km altitude. Sizing the controller against a point the vehicle cannot
% reach costs real gain everywhere it can.
% Set cfg.env = [] to fall back to the outer product of cfg.qbar and cfg.V.
if ~isfield(cfg,'env')
    cfg.env = [ 2000  60
                5000 100
                7200 100
               12000 150
               20000 200
               32000 250
               40000 280 ];
end
if ~isfield(cfg,'qbar'), cfg.qbar = linspace(2e3, 4e4, 8); end
if ~isfield(cfg,'V'),    cfg.V    = linspace(40,  280, 8); end

% ---- coefficients, derived from P ----------------------------------------
[Cl_delta, Cl_p] = roll_coeffs(P);
A   = pi*(P.Vehicle.d/2)^2;
d   = P.Vehicle.d;
Ixx = P.Vehicle.Ixx;

% Actuator. P.Servo.f_N is the model's truth, so use it. The old 2*pi*20 was
% never reconciled with it; the difference is 57% in bandwidth and it moves the
% phase margin directly.
wn    = P.Servo.f_N;
zeta  = P.Servo.Zeta;
tau_d = P.Servo.Tau_d;

k_b = A*d   * Cl_delta / Ixx;        % b   = k_b*q
k_a = A*d^2 * Cl_p     / (2*Ixx);    % a   = k_a*q/V   (<0)
C0  = cfg.w_target / k_b;

if ~cfg.quiet
fprintf('\n--- proportional roll-rate damper:  Kp(q) = C/q ---\n');
fprintf('derived from P: Cl_delta = %.4f, Cl_p = %.4f, Ixx = %.3e\n', Cl_delta, Cl_p, Ixx);
fprintf('actuator      : f_N = %.1f rad/s, zeta = %.2f, tau_d = %.1f ms\n', wn, zeta, 1e3*tau_d);
end

% ---- frequency and design grids ------------------------------------------
Cvec = logspace(log10(C0/4), log10(C0*4), 120);
w    = logspace(-2, log10(30*wn), 2500);
jw   = 1i*w;
Hact = wn^2 ./ (jw.^2 + 2*zeta*wn.*jw + wn^2) .* exp(-jw*tau_d);

scd = [1-cfg.unc_delta, 1, 1+cfg.unc_delta];
scp = [1-cfg.unc_p,     1, 1+cfg.unc_p];
if ~isempty(cfg.env)
    QB = cfg.env(:,1);  VV = cfg.env(:,2);       % paired points
else
    [QB, VV] = ndgrid(cfg.qbar, cfg.V);          % outer product (legacy)
end
if ~cfg.quiet
fprintf('flight conditions: %d %s points, q %.0f-%.0f Pa, V %.0f-%.0f m/s\n', ...
        numel(QB), ternary(~isempty(cfg.env),'paired','grid'), ...
        min(QB(:)), max(QB(:)), min(VV(:)), max(VV(:)));
end

nC = numel(Cvec);
PMw = nan(nC,1); GMw = nan(nC,1); Msw = nan(nC,1); bind = zeros(nC,2);

for ic = 1:nC
    C = Cvec(ic); pm = inf; gm = inf; ms = -inf; bq = 0; bv = 0;
    for id = 1:numel(scd)
        for ip = 1:numel(scp)
            for kk = 1:numel(QB)
                q = QB(kk); v = VV(kk);
                b = scd(id)*k_b*q;  a = scp(ip)*k_a*q/v;
                L = (C/q) .* Hact .* (b ./ (jw - a));
                [G, Pm] = margins_fr(w, L);
                if Pm < pm, pm = Pm; bq = q; bv = v; end
                gm = min(gm, G);
                ms = max(ms, max(abs(1./(1+L))));
            end
        end
    end
    PMw(ic) = pm; GMw(ic) = gm; Msw(ic) = ms; bind(ic,:) = [bq bv];
end

% ---- feasibility, then the C closest to the analytic target --------------
feasible = (PMw >= cfg.PM_min) & (GMw >= cfg.GM_min) & (Msw <= cfg.Ms_max);
cost = abs(log(Cvec(:)) - log(C0)); cost(~feasible) = inf;
[~, ic] = min(cost);
if ~any(feasible)
    warning('No C meets the stability margins; lower w_target or the actuator delay.');
    [~, ic] = max(PMw);
end
C_opt = Cvec(ic);

q = bind(ic,1); v = bind(ic,2);
Lc = (C_opt/q).*Hact.*((k_b*q)./(jw-(k_a*q/v)));
BW_active = bw_from_T(w, abs(Lc./(1+Lc)))/(2*pi);

% ---- report ---------------------------------------------------------------
if ~cfg.quiet
fprintf('C            = %.4g     (analytic C0 = %.4g,  ratio %.2f)\n', C_opt, C0, C_opt/C0);
fprintf('crossover    = %.2f Hz  (target %.2f Hz)\n', C_opt*k_b/(2*pi), cfg.w_target/(2*pi));
fprintf('worst case   : PM = %.1f deg   GM = %.2f   Ms = %.2f\n', PMw(ic), GMw(ic), Msw(ic));
fprintf('worst-PM corner: q = %.0f Pa, V = %.0f m/s  (closed-loop BW there %.2f Hz)\n', ...
        bind(ic,1), bind(ic,2), BW_active);
if any(feasible)
    fprintf('feasible span: C in [%.4g, %.4g]\n', min(Cvec(feasible)), max(Cvec(feasible)));
else
    fprintf('** infeasible -- showing the max-PM fallback **\n');
end
fprintf('\nresulting gains Kp = C/q:\n');
for qq = [min(QB(:)) median(QB(:)) max(QB(:))]
    fprintf('   q = %6.0f Pa  ->  Kp = %.4g\n', qq, C_opt/qq);
end
fprintf('\nSet P.Controller.c = %.4g, then CONFIRM in the nonlinear sim.\n', C_opt);
fprintf('This is a linear margin calculation: it knows nothing about the dead\n');
fprintf('zone, the backlash or the quantiser, so it is a starting point for the\n');
fprintf('sweep, not an answer.\n\n');
end

info = struct('C0',C0,'Cvec',Cvec,'PM',PMw,'GM',GMw,'Ms',Msw, ...
              'feasible',feasible,'Cl_delta',Cl_delta,'Cl_p',Cl_p, ...
              'k_b',k_b,'k_a',k_a,'BW_active',BW_active,'cfg',cfg);

% ---- plot -----------------------------------------------------------------
if ~cfg.plot, return; end
% Two stacked panels sharing the x axis rather than one twin-axis plot: three
% margin quantities with three different units cannot share a y axis honestly,
% and the previous version simply did not draw GM or Ms at all -- so a C that
% failed on gain margin looked fine.
figure('Color','w','Name','Roll schedule');
tl = tiledlayout(2,1,'TileSpacing','compact','Padding','compact');

nexttile;
yyaxis left
semilogx(Cvec, PMw, '-', 'LineWidth', 1.6); hold on
yline(cfg.PM_min, '--', 'PM min');
ylabel('worst-case phase margin / deg'); ylim([0 90]);
yyaxis right
semilogx(Cvec, GMw, '-',  'LineWidth', 1.3); hold on
semilogx(Cvec, Msw, ':',  'LineWidth', 1.3);
yline(cfg.GM_min, '--', 'GM min');
yline(cfg.Ms_max, '-.',  'M_s max');
ylabel('gain margin / ratio,   M_s'); ylim([0 max(4, cfg.Ms_max+1)]);
shade_feasible(Cvec, feasible);
xline(C_opt, 'k-.', 'C_{opt}', 'LineWidth', 1.3);
xline(C0,    'k:',  'C_0');
grid on; title('Worst-case stability margins over the envelope and \pm30% aero');
legend({'PM','','GM','M_s'}, 'Location','best');

nexttile;
loglog(Cvec, Cvec*k_b/(2*pi), '-', 'LineWidth', 1.6); hold on
yline(cfg.w_target/(2*pi), '--', 'crossover target');
xline(C_opt, 'k-.', 'C_{opt}', 'LineWidth', 1.3);
xline(C0,    'k:',  'C_0');
shade_feasible(Cvec, feasible);
xlabel('schedule constant  C'); ylabel('crossover / Hz');
grid on; title('Crossover is linear in C by construction (wc = C k_b)');
title(tl, sprintf('Cl_\\delta = %.4f, Cl_p = %.4f, f_N = %.0f rad/s', Cl_delta, Cl_p, wn));
end


% ======================================================================
function [Cl_delta, Cl_p] = roll_coeffs(P)
% Barrowman fin aerodynamics with the DATCOM low-AR lift slope and the
% fin-body interference factor. Identical expressions to the plant and the
% hinge-moment model, so the three cannot disagree.
V = P.Vehicle.V; d = P.Vehicle.d; s = P.Vehicle.s;
Cr = P.Vehicle.C_r; Ct = P.Vehicle.C_t; N = P.Vehicle.N;
S_c = (Cr + Ct)*s*0.5;
AR  = s^2/S_c;                       % panel convention, s^2/S_f
r   = d/2;  A = pi*r^2;
beta = sqrt(max(1 - (V/343)^2, 1e-6));
a    = 2*pi*AR/(2 + sqrt(4 + (AR*beta)^2));
K    = 1 + r/(s + r);
Cl_delta = K*N*a*(S_c/A)*(P.Aero.y_Cp/d);
k  = (Ct - Cr)/s;
I3 = ((r+s)^3 - r^3)/3;
I4 = ((r+s)^4 - r^4)/4;
Cl_p = -2*K*N*a*(Cr*I3 + k*(I4 - r*I3))/(A*d^2) - 2*P.Aero.C_f*(P.Vehicle.L/d);
end


function shade_feasible(Cvec, feasible)
if ~any(feasible), return; end
yl = ylim;
patch([min(Cvec(feasible)) max(Cvec(feasible)) max(Cvec(feasible)) min(Cvec(feasible))], ...
      [yl(1) yl(1) yl(2) yl(2)], [0.55 0.75 0.55], ...
      'EdgeColor','none','FaceAlpha',0.15,'HandleVisibility','off');
set(gca,'Children', flipud(get(gca,'Children')));
end


function bw = bw_from_T(w, magT)
ref = magT(1)/sqrt(2); idx = find(magT < ref, 1, 'first');
if isempty(idx), bw = w(end);
elseif idx == 1,  bw = w(1);
else
    x1 = log(w(idx-1)); x2 = log(w(idx));
    y1 = log(magT(idx-1)); y2 = log(magT(idx));
    bw = exp(x1 + (log(ref)-y1)*(x2-x1)/(y2-y1));
end
end


function [GM, PM] = margins_fr(w, L)
mag = abs(L); ph = unwrap(angle(L));
PM = 180; k = find(mag(1:end-1) >= 1 & mag(2:end) < 1, 1);
if ~isempty(k)
    t = (1-mag(k))/(mag(k+1)-mag(k));
    PM = 180 + (ph(k) + t*(ph(k+1)-ph(k)))*180/pi;
end
GM = Inf; j = find(ph(1:end-1) >= -pi & ph(2:end) < -pi, 1);
if ~isempty(j)
    t = (-pi-ph(j))/(ph(j+1)-ph(j));
    GM = 1/(mag(j) + t*(mag(j+1)-mag(j)));
end
end


function out = ternary(c, a, b)
if c, out = a; else, out = b; end
end
