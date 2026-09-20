function m = roll_metrics(t, p, delta, ref, P, opt)
%ROLL_METRICS  Per-run performance metrics for the roll-control model.
%
%   m = roll_metrics(t, p, delta, ref, P)
%   m = roll_metrics(t, p, delta, ref, P, opt)
%
%   t      time / s;  p roll rate / rad s^-1;  delta canard deflection / rad
%   ref    commanded roll rate / rad s^-1
%   P      parameter struct (needs Servo.Travel and Servo.dt)
%   opt    .tail_frac    fraction of the run treated as steady (default 0.3)
%          .settle_band  absolute band / rad s^-1
%
%   TRANSIENT AND STEADY ARE REPORTED SEPARATELY, DELIBERATELY.
%
%   This loop has a large, benign, physically unavoidable transient. Before the
%   trim integrator converges, proportional damping alone leaves a droop
%
%       p_droop = q*Cl_0 / (Cl_delta*c + q*d*|Cl_p|/(2V))
%
%   which at q = 40 kPa with Cl_0 = 0.03 is 14 rad/s, and the surface saturates
%   while that is being removed. A peak-over-the-whole-run metric therefore
%   reports "saturated, 99% of travel" for EVERY design, and a magnitude-based
%   divergence test reports "diverged" for every high-q run whatever the gains.
%   Both are true statements about the transient and useless ones about the
%   design. An earlier version of this file made exactly that mistake.
%
%   So travel_peak answers "did the surface ever hit the stop", travel_steady
%   answers "what does holding the trim actually cost", and growth -- not
%   magnitude -- answers "is it unstable".
%
%   METRICS
%     rms_err        RMS of (p - ref) over the tail. The headline number.
%     bias           mean(p) - ref over the tail. Hang-off alone, signed.
%     ripple_pp      max(p) - min(p) over the tail. Limit cycle alone.
%     travel_peak    max|delta| over the WHOLE run / Servo.Travel.
%     travel_steady  max|delta| over the TAIL / Servo.Travel.
%     activity       RMS surface rate, std(diff(delta))/dt. Catches stick-slip,
%                    which no amplitude metric does.
%     growth         RMS of the last half of the tail over the first half.
%                    <=1 converging or steady, >>1 diverging. The stability test.
%     theta_ripple_pp  Angle excursion, rad, left after detrending the integral
%                    of the tail. THE metric for "can an angle-hold loop sit on
%                    top of this". A steady rate bias detrends away because an
%                    outer loop absorbs it; oscillation does not.
%     ripple_freq    Dominant frequency of the tail ripple / Hz. Ripple below
%                    the outer loop's bandwidth is rejected by it; ripple above
%                    passes straight through to attitude.
%     ripple_unresolved  true when that frequency landed in the lowest two FFT
%                    bins, i.e. the tail is drift-dominated or the cycle is too
%                    slow for the window. theta_ripple_pp is NOT trustworthy
%                    when this is set -- lengthen StopTime and re-run.
%     peak_rate      max|p| over the run, for the model-validity check.
%     settle         time |p-ref| last left the band. Inf if never inside.
%     overshoot      peak past ref as % of initial error. NaN if that is zero.
%
%   WHY NOT STEPINFO: with ref = 0 and p(1) = 0 it sees a zero-amplitude step
%   and returns NaN for every run, silently. And settling time is undefined for
%   a loop that converges to a band rather than a point.

if nargin < 6, opt = struct(); end
if ~isfield(opt,'tail_frac'), opt.tail_frac = 0.3; end

t = t(:); p = p(:); delta = delta(:);
e  = p - ref;
e0 = e(1);

if ~isfield(opt,'settle_band') || isempty(opt.settle_band)
    opt.settle_band = max(0.05*abs(e0), 0.05);
end

% ---- tail window ---------------------------------------------------------
t_tail = t(end) - opt.tail_frac*(t(end) - t(1));
kt     = t >= t_tail;
if nnz(kt) < 4, kt = false(size(t)); kt(max(1,numel(t)-3):end) = true; end

pt = p(kt);  dtail = delta(kt);
m.rms_err   = sqrt(mean((pt - ref).^2));
m.bias      = mean(pt) - ref;
m.ripple_pp = max(pt) - min(pt);

% ---- authority: peak and steady are different questions ------------------
m.peak_delta    = max(abs(delta));
m.travel_peak   = m.peak_delta / P.Servo.Travel;
m.travel_steady = max(abs(dtail)) / P.Servo.Travel;
m.peak_rate     = max(abs(p));

% ---- surface activity ----------------------------------------------------
if numel(delta) > 2
    dt = median(diff(t));
    if ~isfinite(dt) || dt <= 0, dt = P.Servo.dt; end
    m.activity = std(diff(delta)) / dt;
else
    m.activity = 0;
end

% ---- ANGLE: what an outer attitude loop would actually be left with -----
% A constant rate bias is NOT a problem for an angle-hold loop: the outer loop
% simply commands an opposing rate and absorbs it. What the outer loop CANNOT
% remove is oscillation above its own bandwidth, and with the inner crossover
% capped at 2.66 Hz the outer loop can only run at roughly a third of that.
%
% So integrate the tail to get angle, then DETREND -- removing the ramp the
% bias produces is exactly the job the outer loop does for free. What is left
% is the angle excursion that survives into attitude hold. This is the metric
% that decides whether an angle loop is worth building on this rate loop, and
% no amplitude-only rate metric can substitute: the same rate ripple at 0.5 Hz
% and at 5 Hz produces a ten-to-one difference in angle.
if nnz(kt) > 8
    th_tail = cumtrapz(t(kt), pt - ref);
    th_tail = detrend(th_tail);
    m.theta_ripple_pp = max(th_tail) - min(th_tail);

    % Dominant ripple frequency, for the cascade-separation check.
    y  = pt - mean(pt);
    ny = numel(y);
    dtm = median(diff(t(kt)));
    Y  = abs(fft(y));
    half = floor(ny/2);
    if half > 2 && isfinite(dtm) && dtm > 0
        [~, ipk] = max(Y(2:half));
        m.ripple_freq = (ipk) / (ny*dtm);
        % RESOLUTION GUARD. The FFT bin spacing is 1/(tail length). A peak in
        % the lowest one or two bins means the tail is dominated by something
        % with a period at least half the window -- which is either a cycle too
        % slow to resolve, or, far more commonly, a response that has not
        % finished converging. Either way theta_ripple_pp is then measuring
        % drift, not oscillation, and detrending cannot separate them because a
        % converging tail is curved, not linear.
        %
        % This is not hypothetical: at StopTime 35 s the reported frequency came
        % back as EXACTLY 0.095147 Hz = 1/10.5 s = bin 1, identically at every
        % point of a 7-point gain sweep and a 25-point trim grid. A physical
        % limit cycle would have moved with the gains. Extend the run until the
        % flag clears before believing any angle number.
        m.ripple_unresolved = (ipk <= 2);
    else
        m.ripple_freq = NaN;
        m.ripple_unresolved = true;
    end
else
    m.theta_ripple_pp   = NaN;
    m.ripple_freq       = NaN;
    m.ripple_unresolved = true;
end

% ---- growth: the instability test ---------------------------------------
% Split the tail in half and compare RMS. A converging run shrinks, a limit
% cycle holds, a divergent one grows. None of that is visible in max|p|.
idx = find(kt);
h   = idx(max(1, round(numel(idx)/2)));
r1  = sqrt(mean((p(idx(1):h)   - ref).^2));
r2  = sqrt(mean((p(h:idx(end)) - ref).^2));
m.growth = (r2 + 1e-12) / (r1 + 1e-12);

% ---- settling, from the LAST exit from the band -------------------------
outside = abs(e) > opt.settle_band;
kout    = find(outside, 1, 'last');
if isempty(kout)
    m.settle = t(1);
elseif kout == numel(t)
    m.settle = Inf;
else
    m.settle = t(kout+1);
end
m.settle_band = opt.settle_band;

% ---- overshoot -----------------------------------------------------------
if abs(e0) < eps
    m.overshoot = NaN;
else
    past = -sign(e0) * e;
    m.overshoot = 100 * max(max(past), 0) / abs(e0);
end
end
