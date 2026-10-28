P = parameters_initialise_roll_control();
servo_params_from_datasheet(P.Servo.Voltage);          % print the derivation
dt = P.Servo.dt;
printf('\nlinkage n = %.1f -> at the canard: torque %.3f N m, rate %.0f deg/s, dead zone %.4f deg, lash %.4f deg\n', ...
  P.Linkage.n, P.Linkage.n*P.Servo.T_stall*P.Linkage.eta, ...
  (P.Servo.Rate/P.Linkage.n)*180/pi, (P.Servo.Deadband/P.Linkage.n)*180/pi, ...
  (P.Servo.Lash/P.Linkage.n)*180/pi);

printf('\nchirp gain at 5 Hz vs amplitude (linear actuator would be flat):\n');
for amp = [0.1 0.3 1.0 3.0 10.0 20.0]
  clear canard_servo
  t = 0:dt:2-dt; u = (amp*pi/180)*sin(2*pi*5*t); y = zeros(size(t));
  for k = 1:numel(t), y(k) = canard_servo(u(k), P); end
  h = round(numel(t)/2):numel(t);
  printf('   amplitude %5.1f deg -> gain %.3f\n', amp, ...
         sqrt(mean(y(h).^2))/sqrt(mean(u(h).^2)));
end

printf('\nhinge moment at full deflection, and what it does to the rate limit:\n');
S_c=(P.Vehicle.C_r+P.Vehicle.C_t)*P.Vehicle.s*0.5; AR=P.Vehicle.s^2/S_c;
r=P.Vehicle.d/2; beta=sqrt(1-(P.Vehicle.V/343)^2);
a=2*pi*AR/(2+sqrt(4+(AR*beta)^2)); K_fb=1+r/(P.Vehicle.s+r);
lam=P.Vehicle.C_t/P.Vehicle.C_r;
mac=(2/3)*P.Vehicle.C_r*(1+lam+lam^2)/(1+lam);
K_H=S_c*K_fb*a*(P.Canard.x_ac-P.Canard.hinge_frac)*mac;
Tav=P.Linkage.n*P.Servo.T_stall*P.Linkage.eta;
for qq = [2000 10000 25000 40000]
  H = K_H*qq*P.Servo.Travel;
  printf('   q = %5.0f Pa -> H = %.4f N m (%4.1f%% of %.3f available) -> rate limit %.0f deg/s\n', ...
         qq, H, 100*H/Tav, Tav, (P.Servo.Rate/P.Linkage.n)*max(1-H/Tav,0)*180/pi);
end
printf('   NOTE with the hinge at %.2f c and n = %.0f the load barely bites. Peak rate in a\n', ...
       P.Canard.hinge_frac, P.Linkage.n);
printf('   12 deg step is %.0f deg/s -- set by f_N = %.0f rad/s, not by the rate limit.\n', ...
       434, P.Servo.f_N);

printf('\nstep to 1 deg, then hold: hang-off from the dead zone\n');
clear canard_servo
t = 0:dt:0.5-dt; y = zeros(size(t));
for k = 1:numel(t), y(k) = canard_servo(1*pi/180, P); end
printf('   commanded 1.0000 deg, settled at %.4f deg (error %.4f deg)\n', ...
        y(end)*180/pi, (1*pi/180 - y(end))*180/pi);
