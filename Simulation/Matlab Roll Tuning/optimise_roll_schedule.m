%% optimise_roll_schedule.m  --  PROPORTIONAL roll-rate damper schedule
% Pure rate damping:   delta_cmd = -Kp(qbar) * p ,   Kp(qbar) = C / qbar
%
% The 1/qbar schedule holds the open-loop CROSSOVER constant: wc = C*k_b.
% That is the quantity to target -- NOT the closed-loop -3 dB bandwidth, which
% also depends on the plant pole |a| = |k_a|*qbar/V and therefore varies hugely
% across the envelope when passive damping (Cl_p) is strong. So:
%   * design target  = crossover wc = C*k_b  ->  C = w_target/k_b = C0 (analytic)
%   * numerical job  = CONFIRM stability margins at/around C0 are robust to the
%                      independent +-30% Cl_delta and Cl_p uncertainties.
%
% No toolboxes: margins from the open-loop frequency response, delay exact.
%
% Physical note for THIS vehicle: with Cl_p = -5.778 the open-loop roll pole
% ranges ~0.4 rad/s (low-q/high-V, nearly undamped -- control needed) to ~55
% rad/s (high-q/low-V, passively damped at ~8.7 Hz -- controller nearly idle,
% sub-unity loop gain). Active roll control engages only at the low-q corner.
% NO INTEGRAL: rate damping has no setpoint/steady error; cancel any standing
% roll bias by FEED-FORWARD of Cl0, not integral action.

% ---------- vehicle / aero (your values) ----------
A=0.003019; d=0.062; Ixx=0.00061;
Cl_delta_nom = 0.6694;          % control effectiveness [1/rad]
Cl_p_nom     = -1.18;          % roll damping derivative [-]
unc_delta    = 0.30;            % +-30% on Cl_delta
unc_p        = 0.30;            % +-30% on Cl_p

% ---------- actuator (bench-measured) ----------
wn = 2*pi*20; zeta = 0.7; tau_d = 14e-3;

% ---------- operating envelope (trajectory tube) ----------
qbar = linspace(2e3, 4e4, 8);
V    = linspace(40, 280, 8);
[QB,VV] = ndgrid(qbar, V);

% ---------- targets ----------
w_target = 2*pi*4;              % crossover target [rad/s]  (~wn/5)
PM_min   = 35;                  % deg
GM_min   = 2.0;                 % ratio (6 dB)
Ms_max   = 2.0;

% ---------- derived constants ----------
k_b = A*d   * Cl_delta_nom / Ixx;        % b = k_b*qbar
k_a = A*d^2 * Cl_p_nom      /(2*Ixx);    % a = k_a*qbar/V   (<0)
C0  = w_target / k_b;                     % analytic schedule constant (= w_target*Ixx/(A*d*Cl_delta))

% ---------- 1-D design grid + frequency grid ----------
Cvec = logspace(log10(C0/4), log10(C0*4), 120);
w  = logspace(-2, log10(30*wn), 2500);
jw = 1i*w;
Hact = wn^2 ./ (jw.^2 + 2*zeta*wn.*jw + wn^2) .* exp(-jw*tau_d);

scd = [1-unc_delta, 1, 1+unc_delta];     % independent Cl_delta multipliers
scp = [1-unc_p,     1, 1+unc_p];         % independent Cl_p     multipliers

nC=numel(Cvec);
PMw=nan(nC,1); GMw=nan(nC,1); Msw=nan(nC,1); bind=zeros(nC,2);

for ic=1:nC
  C=Cvec(ic); pm=inf; gm=inf; ms=-inf; bq=0; bv=0;
  for id=1:numel(scd)
    for ip=1:numel(scp)
      for kk=1:numel(QB)
        q=QB(kk); v=VV(kk);
        b=scd(id)*k_b*q;  a=scp(ip)*k_a*q/v;
        L = (C/q) .* Hact .* (b ./ (jw - a));        % proportional open loop
        [G,P]=margins_fr(w,L);
        if P<pm, pm=P; bq=q; bv=v; end               % worst-PM corner
        gm=min(gm,G); ms=max(ms,max(abs(1./(1+L))));
      end
    end
  end
  PMw(ic)=pm; GMw(ic)=gm; Msw(ic)=ms; bind(ic,:)=[bq bv];
end

% ---------- feasibility (stability only) + pick C closest to C0 ----------
feasible = (PMw>=PM_min) & (GMw>=GM_min) & (Msw<=Ms_max);
cost = abs(log(Cvec(:)) - log(C0));  cost(~feasible)=inf;   % crossover -> target
[~,ic]=min(cost);
if ~any(feasible)
  warning('No C meets the stability margins; lower w_target or actuator delay.');
  [~,ic]=max(PMw);
end
C_opt=Cvec(ic);

% closed-loop bandwidth at the (active) worst-PM corner, nominal -- info only
q=bind(ic,1); v=bind(ic,2);
Lc = (C_opt/q).*Hact.*((k_b*q)./(jw-(k_a*q/v)));
BW_active = bw_from_T(w, abs(Lc./(1+Lc)))/(2*pi);

% ---------- report ----------
fprintf('\n--- proportional roll-rate damper:  Kp(qbar) = C / qbar ---\n');
fprintf('C            = %.4g     (analytic C0 = %.4g,  ratio %.2f)\n',C_opt,C0,C_opt/C0);
fprintf('crossover    = %.2f Hz  (design target = %.2f Hz)\n',C_opt*k_b/(2*pi),w_target/(2*pi));
fprintf('worst-case   : PM = %.1f deg   GM = %.2f   Ms = %.2f\n',PMw(ic),GMw(ic),Msw(ic));
fprintf('worst-PM corner: qbar = %.0f Pa , V = %.0f m/s   (closed-loop BW here = %.2f Hz)\n',...
        bind(ic,1),bind(ic,2),BW_active);
if ~any(feasible), fprintf('** infeasible -- showing max-PM fallback **\n'); end
fprintf('\nresulting gains  Kp = C/qbar:\n');
for qq=[min(qbar) median(qbar) max(qbar)]
  fprintf('   qbar = %5.0f Pa  ->  Kp = %.4g\n', qq, C_opt/qq);
end
fprintf('\nConfirm C_opt in a NONLINEAR sim (saturation + rate limit) before use.\n');

% ---------- 1-D trade plot ----------
figure('Color','w');
yyaxis left
plot(Cvec,PMw,'-','LineWidth',1.6); hold on
yline(PM_min,'--'); ylabel('worst-case phase margin [deg]'); ylim([0 90])
yyaxis right
plot(Cvec, Cvec*k_b/(2*pi),'-','LineWidth',1.6);
yline(w_target/(2*pi),'--'); ylabel('crossover [Hz]')
xline(C_opt,':k','C_{opt}'); xline(C0,':','C_0');
set(gca,'XScale','log'); xlabel('schedule constant  C'); grid on
title('proportional rate damper: phase margin & crossover vs C')

% ===================== helpers =====================
function bw = bw_from_T(w, magT)
ref=magT(1)/sqrt(2); idx=find(magT<ref,1,'first');
if isempty(idx), bw=w(end); elseif idx==1, bw=w(1);
else
  x1=log(w(idx-1)); x2=log(w(idx)); y1=log(magT(idx-1)); y2=log(magT(idx));
  bw=exp(x1+(log(ref)-y1)*(x2-x1)/(y2-y1));
end
end

function [GM,PM] = margins_fr(w, L)
mag=abs(L); ph=unwrap(angle(L));
PM=180; k=find(mag(1:end-1)>=1 & mag(2:end)<1,1);
if ~isempty(k), t=(1-mag(k))/(mag(k+1)-mag(k)); PM=180+(ph(k)+t*(ph(k+1)-ph(k)))*180/pi; end
GM=Inf;  j=find(ph(1:end-1)>=-pi & ph(2:end)<-pi,1);
if ~isempty(j), t=(-pi-ph(j))/(ph(j+1)-ph(j)); GM=1/(mag(j)+t*(mag(j+1)-mag(j))); end
end
