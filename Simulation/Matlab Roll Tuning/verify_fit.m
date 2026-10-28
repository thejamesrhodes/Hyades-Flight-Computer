% Verify the identification procedure against the servo model itself: drive a
% chirp through canard_servo with KNOWN f_N / Zeta / Tau_d, then check that
% fit_servo_bench recovers them. This is the rehearsal for the bench test --
% run it before you build the rig so you know what good looks like.
P = parameters_initialise_roll_control();
P.Servo.f_N = 2*pi*18.5; P.Servo.Zeta = 0.62; P.Servo.Tau_d = 11e-3;
printf('truth: f_N %.1f rad/s (%.2f Hz), zeta %.3f, tau_d %.1f ms\n', ...
       P.Servo.f_N, P.Servo.f_N/(2*pi), P.Servo.Zeta, P.Servo.Tau_d*1e3);

dt = P.Servo.dt; T = 20; t = (0:dt:T-dt)';
f0 = 0.5; f1 = 60;                                   % logarithmic chirp
ph = 2*pi*f0*T/log(f1/f0)*(exp(t/T*log(f1/f0))-1);
for amp = [0.5 3 12]
  clear canard_servo
  u = (amp*pi/180)*sin(ph); y = zeros(size(u));
  for k = 1:numel(t), y(k) = canard_servo(u(k), P); end
  F = fit_servo_bench(t, u, y, struct('fmax',45));
  printf('   amplitude %4.1f deg -> f_N %.2f Hz  zeta %.3f  tau_d %.2f ms\n\n', ...
         amp, F.fn_hz, F.zeta, F.tau_d*1e3);
end
