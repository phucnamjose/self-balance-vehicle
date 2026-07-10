% LQR_DESIGN  LQR full-state feedback vs angle-only PID on nonlinear plant.
% Minimizes J = integral(x'Qx + Ru^2)dt. K from lqr_gain.m (or pkg load control; lqr).

clear; clc;
p = params();
[A, B] = linearize(p);

% ---- LQR weights --------------------------------------------------------
%   states: [pos, vel, theta, omega]
Q = diag([5, 1, 200, 5]);     % weight theta most
R = 0.05;                     % effort penalty (smaller R = more aggressive)

K = lqr_gain(A, B, Q, R);
cl = eig(A - B*K);

printf('LQR gain K = [%.2f  %.2f  %.2f  %.2f]\n', K);
printf('  (force F = -K*x:  pos, vel, tilt, tilt-rate)\n');
printf('Closed-loop poles (all should have negative real part):\n');
for i = 1:numel(cl)
  printf('   %+8.3f %+8.3fi\n', real(cl(i)), imag(cl(i)));
end

% ---- PID comparison (Step 5 gains) --------------------------------------
Kp = 45.0; Ki = 15.0; Kd = 5.0;

% ---- Nonlinear sim: PID vs LQR ------------------------------------------
dt    = 0.001;  T = 5.0;  N = round(T/dt);
x0    = [0; 0; deg2rad(8); 0];            % start tilted 8 deg
F_max = p.n_wheels * p.motor.tau_stall / p.r_wheel;

t  = (0:N) * dt;
% logs: rows 1=PID, 2=LQR
TH = zeros(2, N+1);  POS = zeros(2, N+1);  FF = zeros(2, N+1);

for ctrl = 1:2
  x = x0;  e_int = 0;
  TH(ctrl,1) = rad2deg(x(3));  POS(ctrl,1) = 100*x(1);
  for k = 1:N
    if ctrl == 1
      % --- angle-only PID ---
      e     = 0 - x(3);
      e_int = e_int + e*dt;
      F = -(Kp*e + Ki*e_int + Kd*(-x(4)));
    else
      % --- LQR full-state feedback ---
      F = -K * x;
    end
    if F >  F_max, F =  F_max; end
    if F < -F_max, F = -F_max; end

    k1 = plant_dynamics(x,             F, p);
    k2 = plant_dynamics(x + 0.5*dt*k1, F, p);
    k3 = plant_dynamics(x + 0.5*dt*k2, F, p);
    k4 = plant_dynamics(x +     dt*k3, F, p);
    x  = x + (dt/6)*(k1 + 2*k2 + 2*k3 + k4);

    TH(ctrl,k+1)  = rad2deg(x(3));
    POS(ctrl,k+1) = 100*x(1);
    FF(ctrl,k+1)  = F;
  end
  printf('%s: final tilt=%.3f deg, final cart pos=%.2f cm\n', ...
         {'PID','LQR'}{ctrl}, TH(ctrl,end), POS(ctrl,end));
end

% ---- Plot comparison ----------------------------------------------------
warning('off', 'Octave:gnuplot-graphics');
graphics_toolkit('gnuplot');
fig = figure('visible', 'off', 'position', [100 100 800 800]);

subplot(3,1,1);
plot(t, TH(1,:), 'linewidth', 2); hold on;
plot(t, TH(2,:), 'linewidth', 2, 'color', [0.85 0.33 0.10]);
grid on; ylabel('tilt [deg]'); legend('PID (angle only)', 'LQR (full state)');
title('PID vs LQR on the nonlinear plant (start 8 deg)');

subplot(3,1,2);
plot(t, POS(1,:), 'linewidth', 2); hold on;
plot(t, POS(2,:), 'linewidth', 2, 'color', [0.85 0.33 0.10]);
grid on; ylabel('cart position [cm]');
legend('PID drifts + holds offset', 'LQR returns home');

subplot(3,1,3);
plot(t, FF(1,:), 'linewidth', 1.5); hold on;
plot(t, FF(2,:), 'linewidth', 1.5, 'color', [0.85 0.33 0.10]);
grid on; ylabel('cart force F [N]'); xlabel('time [s]');

outfile = fullfile(fileparts(mfilename('fullpath')), 'lqr_design.png');
print(fig, '-dpng', '-r100', outfile);
printf('Saved plot -> %s\n', outfile);
