% SIM_CLOSEDLOOP_PID  Balance the body with an angle PID on the nonlinear plant.
%
% Now we close the loop. A PID controller looks at the tilt error (we want
% theta = 0) and commands a cart force F to drive it back upright:
%
%       e   = theta_ref - theta          (theta_ref = 0: stay upright)
%       F   = -(Kp*e + Ki*Int(e) + Kd*de/dt)
%
% Sign intuition (from linearize.m): a positive tilt needs a positive force to
% drive the wheels under the body. With the error e = -theta, F = Kp*theta + ...
% comes out positive for a forward tilt - correct.
%
% This step uses the FULL state directly (perfect angle + rate), integrates the
% true nonlinear dynamics at a fine step, and clamps F to what the motors can
% physically produce. Sensor noise, sampling at 200 Hz, and the motor model come
% in Step 6 (sim_discrete.m). Here we just want gains that balance.

clear; clc;
p = params();

% ---- PID gains (start from the linear model, then tune) -----------------
% Kp must be strong enough to beat gravity (it overcomes the m*g*l*sin(theta)
% destabilizing torque); Kd adds damping so it does not overshoot; Ki removes
% any slow steady tilt offset. These were tuned against the nonlinear plant.
Kp = 45.0;     % force per rad of tilt         [N/rad]
Ki = 15.0;     % force per rad*s of tilt        [N/(rad*s)]
Kd = 5.0;      % force per rad/s of tilt rate   [N/(rad/s)]

% ---- Simulation settings ------------------------------------------------
dt    = 0.001;                 % integration step [s]
T     = 4.0;                   % total time [s]
N     = round(T / dt);
theta_ref = 0;                 % upright target
x = [0; 0; deg2rad(5); 0];     % start tilted 5 deg, at rest

% Physical force ceiling: both motors at stall, force = n*tau_stall / r.
F_max = p.n_wheels * p.motor.tau_stall / p.r_wheel;

t  = zeros(1, N+1);
X  = zeros(4, N+1);  X(:,1) = x;
Fh = zeros(1, N+1);
e_int = 0;                     % integral of the tilt error

for k = 1:N
  % --- PID control from the current (full) state ---
  theta = x(3);
  omega = x(4);
  e     = theta_ref - theta;
  e_int = e_int + e*dt;
  % de/dt of the error = -omega (ref is constant).
  F = -(Kp*e + Ki*e_int + Kd*(-omega));

  % Saturate to the motors' capability (and anti-windup: stop integrating
  % when saturated so the integral term can't blow up).
  if F >  F_max, F =  F_max; e_int = e_int - e*dt; end
  if F < -F_max, F = -F_max; e_int = e_int - e*dt; end

  % --- RK4 step of the nonlinear plant with this (held) force ---
  k1 = plant_dynamics(x,             F, p);
  k2 = plant_dynamics(x + 0.5*dt*k1, F, p);
  k3 = plant_dynamics(x + 0.5*dt*k2, F, p);
  k4 = plant_dynamics(x +     dt*k3, F, p);
  x  = x + (dt/6) * (k1 + 2*k2 + 2*k3 + k4);

  t(k+1)   = k*dt;
  X(:,k+1) = x;
  Fh(k+1)  = F;
end

theta_deg = rad2deg(X(3,:));
pos_cm    = 100 * X(1,:);

% ---- Report -------------------------------------------------------------
settled = find(abs(theta_deg) > 0.5, 1, 'last');   % last time tilt > 0.5 deg
if isempty(settled)
  ts = 0;
else
  ts = t(settled);
end
printf('PID balance: Kp=%.1f Ki=%.1f Kd=%.2f, F_max=%.1f N\n', Kp, Ki, Kd, F_max);
printf('Peak tilt    = %.2f deg\n', max(abs(theta_deg)));
printf('Settling (|tilt|<0.5 deg) at t = %.2f s\n', ts);
printf('Final tilt   = %.3f deg, final cart pos = %.1f cm\n', ...
       theta_deg(end), pos_cm(end));

% ---- Plot ---------------------------------------------------------------
warning('off', 'Octave:gnuplot-graphics');
graphics_toolkit('gnuplot');
fig = figure('visible', 'off', 'position', [100 100 800 800]);

subplot(3,1,1);
plot(t, theta_deg, 'linewidth', 2); grid on;
ylabel('tilt \theta [deg]');
title(sprintf('PID balance (Kp=%.1f Ki=%.1f Kd=%.2f)', Kp, Ki, Kd));

subplot(3,1,2);
plot(t, pos_cm, 'linewidth', 2, 'color', [0.85 0.33 0.10]); grid on;
ylabel('cart position [cm]');

subplot(3,1,3);
plot(t, Fh, 'linewidth', 2, 'color', [0.20 0.55 0.20]); hold on;
plot([t(1) t(end)], [ F_max  F_max], '--', 'color', [0.5 0.5 0.5]);
plot([t(1) t(end)], [-F_max -F_max], '--', 'color', [0.5 0.5 0.5]);
grid on; ylabel('cart force F [N]'); xlabel('time [s]');

outfile = fullfile(fileparts(mfilename('fullpath')), 'sim_closedloop_pid.png');
print(fig, '-dpng', '-r100', outfile);
printf('Saved plot -> %s\n', outfile);
