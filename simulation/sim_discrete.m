% SIM_DISCRETE  Realistic loop: 200 Hz, IMU, complementary filter, motor model.
% Adds sampling/ZOH, noisy sensors, back-EMF feedforward, deadband compensation.

clear; clc;
p = params();

% ---- PID gains (force units; carried over from sim_closedloop_pid.m) ----
Kp = 45.0; Ki = 15.0; Kd = 5.0;

% ---- Complementary filter + sensor model --------------------------------
alpha       = 0.995;                % gyro weight (higher = less accel contamination)
gyro_bias   = deg2rad(0.1);         % residual gyro offset AFTER calibration [rad/s]
gyro_noise  = deg2rad(1.0);         % gyro noise std [rad/s]
accel_noise = deg2rad(2.0);         % accel-angle noise std [rad]
vel_noise   = 0.01;                 % encoder velocity noise std [m/s]

% ---- Motor model --------------------------------------------------------
F_stall = p.n_wheels * p.motor.tau_stall / p.r_wheel;                 % [N] at w=0
K_bemf  = p.n_wheels * p.motor.tau_stall / (p.motor.w_noload * p.r_wheel^2); % [N/(m/s)]

% ---- Timing -------------------------------------------------------------
dt_ctrl = p.dt_ctrl;          % 5 ms control period
n_sub   = 5;                  % plant substeps per control tick
dt_sub  = dt_ctrl / n_sub;    % 1 ms integration step
T       = 5.0;
Nticks  = round(T / dt_ctrl);

% ---- State + logs -------------------------------------------------------
x         = [0; 0; deg2rad(5); 0];      % true state, start tilted 5 deg
theta_est = x(3);                        % filter estimate (seed at truth)
e_int     = 0;
vel_prev  = x(2);

randn('seed', 1);                       % reproducible noise

t          = zeros(1, Nticks+1);
theta_true = zeros(1, Nticks+1);  theta_true(1) = rad2deg(x(3));
theta_hat  = zeros(1, Nticks+1);  theta_hat(1)  = rad2deg(theta_est);
pos_cm     = zeros(1, Nticks+1);  pos_cm(1)     = 100*x(1);
u_cmd_log  = zeros(1, Nticks+1);

for n = 1:Nticks
  % ===== SENSE (200 Hz) =====
  gyro_meas = x(4) + gyro_bias + gyro_noise*randn();        % tilt rate
  a_cart    = (x(2) - vel_prev) / dt_ctrl;                  % cart accel estimate
  theta_acc = x(3) + a_cart/p.g + accel_noise*randn();      % accel tilt (noisy)
  vel_prev  = x(2);
  vel_meas  = x(2) + vel_noise*randn();                     % encoder cart speed

  % ===== ESTIMATE: complementary filter =====
  theta_est = alpha*(theta_est + gyro_meas*dt_ctrl) + (1-alpha)*theta_acc;

  % ===== CONTROL: angle PID on the ESTIMATE =====
  e     = 0 - theta_est;
  e_int = e_int + e*dt_ctrl;
  F_des = -(Kp*e + Ki*e_int + Kd*(-gyro_meas));   % D term uses measured rate

  % ===== ACTUATE =====
  % Back-EMF feedforward: cancel speed-dependent torque droop
  F_cmd = F_des + K_bemf*vel_meas;

  u = F_cmd / F_stall;                              % desired duty in [-1,1]
  u = max(-p.motor.u_max, min(p.motor.u_max, u));   % saturate
  if abs(u) >= p.motor.u_max, e_int = e_int - e*dt_ctrl; end   % anti-windup

  % Deadband remap [0,1] -> [deadband,1]
  db = p.motor.deadband;
  if abs(u) > 1e-3
    u_applied = sign(u) * (db + (1-db)*abs(u));
  else
    u_applied = 0;
  end
  u_applied = max(-p.motor.u_max, min(p.motor.u_max, u_applied));

  % ===== PLANT: integrate continuously with the duty held (ZOH) =====
  for s = 1:n_sub
    w_wheel = x(2) / p.r_wheel;
    tau     = u_applied*p.motor.tau_stall ...
              - (p.motor.tau_stall/p.motor.w_noload)*w_wheel;   % back-EMF droop
    F       = p.n_wheels * tau / p.r_wheel;                     % force on cart

    k1 = plant_dynamics(x,                F, p);
    k2 = plant_dynamics(x + 0.5*dt_sub*k1, F, p);
    k3 = plant_dynamics(x + 0.5*dt_sub*k2, F, p);
    k4 = plant_dynamics(x +     dt_sub*k3, F, p);
    x  = x + (dt_sub/6) * (k1 + 2*k2 + 2*k3 + k4);
  end

  t(n+1)          = n*dt_ctrl;
  theta_true(n+1) = rad2deg(x(3));
  theta_hat(n+1)  = rad2deg(theta_est);
  pos_cm(n+1)     = 100*x(1);
  u_cmd_log(n+1)  = u_applied;

  if abs(x(3)) > pi/2
    idx = 1:n+1;
    t=t(idx); theta_true=theta_true(idx); theta_hat=theta_hat(idx);
    pos_cm=pos_cm(idx); u_cmd_log=u_cmd_log(idx);
    printf('FELL OVER at t = %.2f s\n', n*dt_ctrl);
    break;
  end
end

% ---- Report -------------------------------------------------------------
printf('Discrete loop @ %d Hz, complementary alpha=%.2f, deadband=%.2f\n', ...
       p.f_ctrl, alpha, p.motor.deadband);
printf('Back-EMF feedforward K_bemf = %.1f N/(m/s), F_stall = %.1f N\n', K_bemf, F_stall);
printf('Peak true tilt = %.2f deg\n', max(abs(theta_true)));
printf('Final true tilt = %.3f deg, final est tilt = %.3f deg\n', ...
       theta_true(end), theta_hat(end));
printf('Estimator RMS error = %.3f deg\n', sqrt(mean((theta_true - theta_hat).^2)));
printf('Final cart pos = %.1f cm\n', pos_cm(end));

% ---- Plot ---------------------------------------------------------------
warning('off', 'Octave:gnuplot-graphics');
graphics_toolkit('gnuplot');
fig = figure('visible', 'off', 'position', [100 100 800 800]);

subplot(3,1,1);
plot(t, theta_true, 'linewidth', 2); hold on;
plot(t, theta_hat, 'linewidth', 1, 'color', [0.85 0.33 0.10]);
grid on; ylabel('tilt [deg]'); legend('true', 'estimate (filter)');
title(sprintf('Discrete 200 Hz loop: IMU + motor model (Kp=%.0f Ki=%.0f Kd=%.1f)', Kp, Ki, Kd));

subplot(3,1,2);
plot(t, pos_cm, 'linewidth', 2, 'color', [0.20 0.55 0.20]); grid on;
ylabel('cart position [cm]');

subplot(3,1,3);
plot(t, 100*u_cmd_log, 'linewidth', 1.0, 'color', [0.40 0.20 0.60]); grid on;
ylabel('motor duty [%]'); xlabel('time [s]'); ylim([-110 110]);

outfile = fullfile(fileparts(mfilename('fullpath')), 'sim_discrete.png');
print(fig, '-dpng', '-r100', outfile);
printf('Saved plot -> %s\n', outfile);
