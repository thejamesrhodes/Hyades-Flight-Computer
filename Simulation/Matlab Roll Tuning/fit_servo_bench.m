function F = fit_servo_bench(t, cmd, meas, opts)
%FIT_SERVO_BENCH  Identify wn, zeta and transport delay from a bench chirp.
%
%   F = fit_servo_bench(t, cmd, meas)
%   F = fit_servo_bench(t, cmd, meas, struct('fmax',60,'nband',40))
%
%   t     uniform time vector [s]        (log at >= 1 kHz)
%   cmd   commanded angle    [rad or deg -- units cancel]
%   meas  measured angle at the CANARD, not the servo horn
%
%   Returns wn [rad/s], zeta, tau_d [s], the fitted coherence-weighted
%   residual, and the empirical FRF so you can plot it.
%
%   Method: Welch-averaged cross- and auto-spectra give H(f) = Sxy/Sxx.  The
%   transport delay shows up as LINEAR phase, so it separates cleanly from the
%   second-order phase -- fit all three together by nonlinear least squares on
%   the complex FRF, weighted by coherence.
%
%   Run the chirp at THREE amplitudes (say 0.3, 2 and 10 deg).  A linear
%   actuator gives the same H(f) at all three.  Yours will not: the small
%   amplitude loses gain to the dead zone and the large one loses gain to the
%   rate limit.  The amplitude at which the gain is flattest is where the
%   describing function is closest to unity -- fit wn and zeta THERE, and use
%   the spread between amplitudes as your uncertainty band.

if nargin < 4, opts = struct(); end
fmax  = getdef(opts,'fmax', 60);          % Hz, top of the fit band
fmin  = getdef(opts,'fmin', 0.5);
nseg  = getdef(opts,'nseg', 8);           % Welch segments

dt = mean(diff(t));  fs = 1/dt;
x = cmd(:) - mean(cmd);  y = meas(:) - mean(meas);
N = numel(x);  L = floor(N/nseg);  L = L - mod(L,2);
w = 0.5 - 0.5*cos(2*pi*(0:L-1)'/(L-1));   % Hann
Sxx = zeros(L/2+1,1); Sxy = zeros(L/2+1,1); Syy = zeros(L/2+1,1);
nblk = 0;
for k = 1:floor(L/2):N-L+1
    X = fft(x(k:k+L-1).*w);  Y = fft(y(k:k+L-1).*w);
    X = X(1:L/2+1);          Y = Y(1:L/2+1);
    Sxx = Sxx + abs(X).^2;   Syy = Syy + abs(Y).^2;
    Sxy = Sxy + conj(X).*Y;  nblk = nblk + 1;
end
f  = (0:L/2)'*fs/L;
H  = Sxy./Sxx;
coh = abs(Sxy).^2 ./ max(Sxx.*Syy, eps);

b = f >= fmin & f <= fmax & coh > 0.8;
if nnz(b) < 8, error('fit_servo_bench: too few coherent bins -- raise the drive amplitude or widen the band'); end
fb = f(b); Hb = H(b); wb = coh(b);

% ---- nonlinear least squares on the complex FRF -------------------------
mdl = @(p, ff) (p(1)^2) ./ ((2i*pi*ff).^2 + 2*p(2)*p(1)*(2i*pi*ff) + p(1)^2) ...
               .* exp(-2i*pi*ff*p(3));
cost = @(p) sum(wb .* abs(mdl(p, fb) - Hb).^2);
p = [2*pi*20, 0.7, 5e-3];                       % start
step = [2*pi*8, 0.25, 4e-3];
for it = 1:400                                   % coordinate descent, no toolbox
    improved = false;
    for j = 1:3
        for s = [-1 1]
            q = p; q(j) = p(j) + s*step(j);
            if q(1) <= 0 || q(2) <= 0.02 || q(2) > 3 || q(3) < 0, continue; end
            if cost(q) < cost(p), p = q; improved = true; end
        end
    end
    if ~improved, step = step/2; end
    if all(step < [1e-4 1e-5 1e-8]), break; end
end

F = struct('wn', p(1), 'zeta', p(2), 'tau_d', p(3), ...
           'fn_hz', p(1)/(2*pi), 'resid', sqrt(cost(p)/sum(wb)), ...
           'f', f, 'H', H, 'coh', coh, 'band', b);
fprintf('\n--- servo FRF fit ---\n');
fprintf('  wn    = %.1f rad/s  (%.2f Hz)\n', F.wn, F.fn_hz);
fprintf('  zeta  = %.3f\n', F.zeta);
fprintf('  tau_d = %.2f ms\n', F.tau_d*1e3);
fprintf('  weighted residual %.4f over %d bins, %.1f-%.1f Hz\n', ...
        F.resid, nnz(b), min(fb), max(fb));
fprintf('  repeat at three drive amplitudes -- the spread IS your uncertainty.\n');
end

function v = getdef(s, f, d)
if isfield(s, f), v = s.(f); else, v = d; end
end
