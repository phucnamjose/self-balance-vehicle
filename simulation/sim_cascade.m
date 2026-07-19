% SIM_CASCADE  Firmware cascade: balance PID -> wheel PI -> motor -> plant.
% Ports balance_pid_sim / wheel_pi_sim (not direct force like sim_discrete.m).
% Multi-rate, mirroring the two-task firmware:
%   motor + wheel-PI @ f_motor (500 Hz), IMU read + complementary filter @ f_imu
%   (250 Hz), balance PID @ f_bal (250 Hz, BALANCE_DIV=1). Estimate + w_common hold
%   (ZOH) between their ticks, exactly as the firmware hands them across tasks.
% Simplifications: one lumped inner loop; w=v/r (no body-rotation term).

clear; clc;
here = fileparts(mfilename('fullpath'));
addpath(here);                 % params, plant_dynamics, wheel_pi_sim, balance_pid_sim
p = params();

% ---- Inner loop: firmware wheel_pi gains (lumped avg) --------------------
% tau_f = 0 (measurement LPF OFF, current firmware): the VEL_WIN window already
% de-quantizes speed, so the extra pole is gone -> faster, better-damped inner loop.
inner.kp        = 0.150;                      % avg(0.1455, 0.1554)
inner.ki        = 0.750;                      % avg(0.6737, 0.8265)
inner.kff       = 33.3;                       % avg(34.36, 32.18)  [rad/s per duty]
inner.tau_f     = 0.0;                        % WHEEL_PI_TAU_F (off)
inner.u_max     = 0.95;                       % WHEEL_PI_DUTY_MAX
inner.i_max     = 0.95;                       % WHEEL_PI_I_MAX
inner.brake_max = 0.4;                        % WHEEL_PI_BRAKE_MAX
inner.ff_en     = true;                       % feedforward ON (current firmware default)
inner.db_en     = false;                      % deadband comp off (firmware default)
inner.neutral   = 0.02;                       % WHEEL_PI_NEUTRAL
inner.db_floor  = p.motor.deadband;           % WHEEL_PI_DB_FLOOR

% ---- Outer balance loop -------------------------------------------------
% Velocity cascade: Ki=stiffness (need r*Ki>g, Ki>~302), Kp=damping, Kd=inertia.
% Re-tuned for the faster inner loop + 250 Hz balance rate by tune_cascade.m.
outer.kp        = 45.0;
outer.ki        = 450.0;
outer.kd        = 3.0;
outer.theta_ref = 0.0;
outer.w_max     = 20.0;                       % BALANCE_W_MAX
outer.i_max     = 20.0;                       % BALANCE_I_MAX
BAL_MAX_TILT    = 0.6;                        % BALANCE_MAX_TILT (fall cutoff, rad)

% ---- Estimator + sensor model -------------------------------------------
alpha       = 0.98;                           % complementary-filter weight (~0.2 s tau at 250 Hz)
gyro_bias   = deg2rad(0.1);
gyro_noise  = deg2rad(1.0);
accel_noise = deg2rad(2.0);
w_meas_noise = 0.3;                           % wheel-speed measurement noise [rad/s]

% ---- Timing (multi-rate) ------------------------------------------------------
dt_m    = 1 / p.f_motor;  n_sub = 4;  dt_sub = dt_m / n_sub;   % plant integrates at motor rate
imu_div = round(p.f_motor / p.f_imu);         % motor ticks per IMU/estimator update (=2)
bal_div = round(p.f_motor / p.f_bal);         % motor ticks per balance step (=2)
dt_imu  = 1 / p.f_imu;  dt_bal = 1 / p.f_bal;
T = 8.0;  Nticks = round(T / dt_m);

% Residual pure transport delay [s]: IMU sample age (FIFO) + I2C read + PWM update.
% Smaller than the old single-rate 15 ms lump because the estimator/balance ZOHs
% are now modelled explicitly by the sub-rate holds below. 0 = off.
loop_delay = 0.004;                           % [s]  (~2 motor ticks)
n_delay    = max(0, round(loop_delay / dt_m));
duty_q     = zeros(1, n_delay + 1);           % FIFO of pending duty commands

% ---- State + controller memory ------------------------------------------------
x         = [0; 0; deg2rad(5); 0];            % start tilted 5 deg
theta_est = x(3);                             % held between IMU ticks
gm        = x(4);                             % last gyro reading (held for balance D-term)
vel_prev  = x(2);
bal_i     = 0;                                % balance integrator
w_common  = 0;                                % balance output, held between balance ticks
wp_i      = 0;                                % inner integrator
wp_wf     = x(2) / p.r_wheel;                 % inner measurement filter (seed at truth)
u_eff     = 0;                                % actuator-lag state (effective duty)

randn('seed', 1);

t        = (0:Nticks) * dt_m;
th_true  = zeros(1, Nticks+1);  th_true(1) = rad2deg(x(3));
th_hat   = zeros(1, Nticks+1);  th_hat(1)  = rad2deg(theta_est);
wcmd_log = zeros(1, Nticks+1);
wmea_log = zeros(1, Nticks+1);
duty_log = zeros(1, Nticks+1);
pos_log  = zeros(1, Nticks+1);
fell = false;

for n = 1:Nticks
  % ===== SENSE + ESTIMATE @ f_imu (250 Hz) =====
  if mod(n-1, imu_div) == 0
    gm        = x(4) + gyro_bias + gyro_noise*randn();       % tilt rate [rad/s]
    a_cart    = (x(2) - vel_prev) / dt_imu;                  % cart accel estimate
    theta_acc = x(3) + a_cart/p.g + accel_noise*randn();     % accel tilt [rad]
    vel_prev  = x(2);
    theta_est = alpha*(theta_est + gm*dt_imu) + (1-alpha)*theta_acc;
  end

  % ===== BALANCE (outer) @ f_bal (250 Hz) -> common wheel-speed command =====
  if mod(n-1, bal_div) == 0
    if abs(theta_est) < BAL_MAX_TILT
      [w_common, bal_i] = balance_pid_sim(theta_est, gm, bal_i, dt_bal, outer);
    else
      w_common = 0; bal_i = 0;                 % fallen: cut (mirrors control.c)
    end
  end

  % ===== INNER wheel-speed PI @ f_motor (500 Hz) =====
  w_meas = x(2)/p.r_wheel + w_meas_noise*randn();
  if abs(theta_est) < BAL_MAX_TILT
    [duty, wp_i, wp_wf] = wheel_pi_sim(w_common, w_meas, wp_i, wp_wf, dt_m, inner);
  else
    duty = 0; wp_i = 0; wp_wf = w_meas;        % cut + reset inner
  end

  % ===== TRANSPORT DELAY =====
  duty_q   = [duty_q(2:end), duty];
  duty_app = duty_q(1);

  % ===== ACTUATE + PLANT (ZOH duty, first-order u_eff lag) =====
  for s = 1:n_sub
    u_eff   = u_eff + (dt_sub / p.motor.tau_e) * (duty_app - u_eff);   % first-order actuator lag
    u_m     = u_eff;
    if abs(u_m) < p.motor.deadband, u_m = 0.0; end   % dead zone
    w_wheel = x(2) / p.r_wheel;
    tau     = u_m*p.motor.tau_stall - (p.motor.tau_stall/p.motor.w_noload)*w_wheel;
    F       = p.n_wheels * tau / p.r_wheel;
    k1 = plant_dynamics(x,                 F, p);
    k2 = plant_dynamics(x + 0.5*dt_sub*k1, F, p);
    k3 = plant_dynamics(x + 0.5*dt_sub*k2, F, p);
    k4 = plant_dynamics(x +     dt_sub*k3, F, p);
    x  = x + (dt_sub/6)*(k1 + 2*k2 + 2*k3 + k4);
  end

  th_true(n+1)  = rad2deg(x(3));
  th_hat(n+1)   = rad2deg(theta_est);
  wcmd_log(n+1) = w_common;
  wmea_log(n+1) = x(2)/p.r_wheel;
  duty_log(n+1) = duty;
  pos_log(n+1)  = 100 * x(1);

  if abs(x(3)) > pi/2
    idx = 1:n+1;
    t=t(idx); th_true=th_true(idx); th_hat=th_hat(idx);
    wcmd_log=wcmd_log(idx); wmea_log=wmea_log(idx); duty_log=duty_log(idx); pos_log=pos_log(idx);
    fell = true; printf('FELL OVER at t = %.2f s\n', n*dt_m);
    break;
  end
end

% ---- Report -------------------------------------------------------------------
printf('Cascade sim: motor %d Hz / imu %d Hz / balance %d Hz\n', p.f_motor, p.f_imu, p.f_bal);
printf('Inner PI (firmware wheel_pi): Kp=%.4f Ki=%.4f  (K=%.2f rad/s/duty, tau_f=%.3f, motor tau_e=%.3f s)\n', ...
       inner.kp, inner.ki, inner.kff, inner.tau_f, p.motor.tau_e);
printf('Balance PID: Kp=%.1f Ki=%.1f Kd=%.1f, w_max=%.0f rad/s, delay=%.0f ms\n', ...
       outer.kp, outer.ki, outer.kd, outer.w_max, 1e3*loop_delay);
if ~fell
  printf('BALANCED. Peak tilt = %.2f deg, final tilt = %.3f deg, final pos = %.1f cm\n', ...
         max(abs(th_true)), th_true(end), pos_log(end));
end

% ---- Plot ---------------------------------------------------------------------
warning('off', 'Octave:gnuplot-graphics');
graphics_toolkit('gnuplot');
fig = figure('visible', 'off', 'position', [100 100 820 900]);

subplot(4,1,1);
plot(t, th_true, 'linewidth', 2); hold on;
plot(t, th_hat, 'linewidth', 1, 'color', [0.85 0.33 0.10]); grid on;
ylabel('tilt [deg]'); legend('true', 'estimate');
title(sprintf('Cascade: balance PID %d Hz -> wheel PI %d Hz (Kp=%.0f Ki=%.0f Kd=%.0f)', ...
      p.f_bal, p.f_motor, outer.kp, outer.ki, outer.kd));

subplot(4,1,2);
plot(t, wcmd_log, 'linewidth', 1.5, 'color', [0.00 0.45 0.74]); hold on;
plot(t, wmea_log, 'linewidth', 1.2, 'color', [0.47 0.16 0.51]); grid on;
ylabel('wheel speed [rad/s]'); legend('w\_common (setpoint)', 'measured');

subplot(4,1,3);
plot(t, 100*duty_log, 'linewidth', 1.0, 'color', [0.20 0.55 0.20]); grid on;
ylabel('motor duty [%]'); ylim([-110 110]);

subplot(4,1,4);
plot(t, pos_log, 'linewidth', 2, 'color', [0.85 0.33 0.10]); grid on;
ylabel('cart pos [cm]'); xlabel('time [s]');

outfile = fullfile(here, 'sim_cascade.png');
print(fig, '-dpng', '-r100', outfile);
printf('Saved plot -> %s\n', outfile);

% Velocity-command cascade: a=r*d(w_common)/dt => Ki=stiffness, Kp=damping, Kd=inertia.
% Opposite of force-PID intuition; tilt-only still drifts (no position spring).
