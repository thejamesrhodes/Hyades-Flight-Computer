function D = servo_params_from_datasheet(voltage, opts)
%SERVO_PARAMS_FROM_DATASHEET  Derive a servo model from published (V, stall
%   torque, 60-degree time) points instead of assuming it.
%
%   D = servo_params_from_datasheet(8.4)
%   D = servo_params_from_datasheet(8.4, struct('quiet',1))
%
%   The 60-degree figure is a TRAVEL time, not a slew rate: it contains an
%   acceleration and a deceleration phase.  Fitting  t60 = a/V + b  separates
%   them.  Two things fall out that the headline spec does not give you:
%     * the true no-load slew rate, 60deg/(t60 - b);
%     * the reflected rotor+gearbox inertia, J ~ b*T_stall/(2*w0), which for a
%       high-ratio hobby servo swamps the control surface's own inertia.
%   Checking the scaling also says WHICH limit the spec figure represents:
%   slew-limited travel scales as 1/V, inertia-limited travel as 1/sqrt(V).
%
%   Defaults are the Blue Bird BMS-127WV+:
%   https://www.blue-bird-model.com/products_detail/413.htm
%
%   Override the table via opts.V / opts.Tkg / opts.t60 for a different servo.

if nargin < 2, opts = struct(); end
V   = getdef(opts,'V',   [3.7 6.0 7.4 8.4]);
Tkg = getdef(opts,'Tkg', [2.5 4.0 4.7 5.3]);        % kg.cm stall
t60 = getdef(opts,'t60', [0.11 0.07 0.06 0.05]);    % s per 60 deg, no load
T   = Tkg*9.80665e-2;                                % N m

A  = [1./V(:), ones(numel(V),1)];
ab = A \ t60(:);
b_fit   = ab(2);
w0_true = (pi/3) ./ (t60 - b_fit);                   % rad/s, slew only (60 deg = pi/3)

D = struct();
D.T_stall   = interp1(V, T,       voltage, 'linear', 'extrap');
D.w0        = interp1(V, w0_true, voltage, 'linear', 'extrap');
D.w0_spec   = interp1(V, (pi/3)./t60, voltage, 'linear', 'extrap');
D.J_out     = mean(b_fit .* T ./ (2*w0_true));
D.overhead  = b_fit;
D.alpha_max = D.T_stall / D.J_out;
D.peak_power = D.T_stall * D.w0 / 4;

D.slew_ratio   = t60(1)/t60(end);
D.slew_pred    = V(end)/V(1);
D.inertia_pred = sqrt(V(end)/V(1));
if abs(D.slew_ratio - D.inertia_pred) < abs(D.slew_ratio - D.slew_pred)
    D.limit = 'inertia';
else
    D.limit = 'slew';
end

if ~getdef(opts,'quiet',false)
  fprintf('\n--- servo derived from %d datasheet points, at %.1f V ---\n', numel(V), voltage);
  fprintf('  t60 ratio %.2f : slew predicts %.2f, inertia predicts %.2f  -> %s-limited\n', ...
          D.slew_ratio, D.slew_pred, D.inertia_pred, D.limit);
  fprintf('  accel/decel overhead      %.1f ms\n', D.overhead*1e3);
  fprintf('  true no-load slew         %.0f deg/s   (spec figure %.0f deg/s)\n', ...
          D.w0*180/pi, D.w0_spec*180/pi);
  fprintf('  stall torque              %.4f N m\n', D.T_stall);
  fprintf('  reflected output inertia  %.3e kg m^2\n', D.J_out);
  fprintf('  max output acceleration   %.1f krad/s^2\n', D.alpha_max/1e3);
  fprintf('  peak mechanical power     %.2f W\n', D.peak_power);
end
end

function v = getdef(s, f, d)
if isfield(s, f), v = s.(f); else, v = d; end
end
