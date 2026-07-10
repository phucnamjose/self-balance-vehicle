% SIM_OPENLOOP  Uncontrolled plant: tiny tilt must topple (F=0, RK4 integration).

clear; clc;
p = params();

% ---- Simulation settings ------------------------------------------------
dt    = 0.001;                 % integration step [s]
T     = 2.0;                   % total time [s]
N     = round(T / dt);
theta0_deg = 3;                % initial tilt from upright [deg]

% State: start nearly upright, at rest.
x = [0; 0; deg2rad(theta0_deg); 0];
F = 0;                         % open loop: no control force

t   = zeros(1, N+1);
X   = zeros(4, N+1);
X(:,1) = x;

t_fall = NaN;                  % time past 90 deg

for k = 1:N
  % RK4 step
  k1 = plant_dynamics(x,             F, p);
  k2 = plant_dynamics(x + 0.5*dt*k1, F, p);
  k3 = plant_dynamics(x + 0.5*dt*k2, F, p);
  k4 = plant_dynamics(x +     dt*k3, F, p);
  x  = x + (dt/6) * (k1 + 2*k2 + 2*k3 + k4);

  t(k+1)   = k*dt;
  X(:,k+1) = x;

  if isnan(t_fall) && abs(x(3)) >= pi/2
    t_fall = k*dt;
  end
end

theta_deg = rad2deg(X(3,:));
pos_cm    = 100 * X(1,:);

% ---- Report -------------------------------------------------------------
printf('Open-loop drop test: start tilt = %.1f deg, F = 0\n', theta0_deg);
if isnan(t_fall)
  printf('Body had not reached 90 deg by t = %.2f s (final tilt %.1f deg)\n', ...
         T, theta_deg(end));
else
  printf('Body reached 90 deg (fully fallen) at t = %.3f s\n', t_fall);
end

% ---- Plot (saved to PNG; works headless) --------------------------------
warning('off', 'Octave:gnuplot-graphics');
graphics_toolkit('gnuplot');
fig = figure('visible', 'off', 'position', [100 100 800 600]);

subplot(2,1,1);
plot(t, theta_deg, 'linewidth', 2); hold on;
plot([t(1) t(end)], [ 90  90], '--', 'color', [0.5 0.5 0.5]);
plot([t(1) t(end)], [-90 -90], '--', 'color', [0.5 0.5 0.5]);
grid on; ylabel('tilt \theta [deg]');
title(sprintf('Open-loop: uncontrolled body falls (start %.1f deg)', theta0_deg));

subplot(2,1,2);
plot(t, pos_cm, 'linewidth', 2, 'color', [0.85 0.33 0.10]);
grid on; ylabel('cart position [cm]'); xlabel('time [s]');

outfile = fullfile(fileparts(mfilename('fullpath')), 'sim_openloop.png');
print(fig, '-dpng', '-r100', outfile);
printf('Saved plot -> %s\n', outfile);
