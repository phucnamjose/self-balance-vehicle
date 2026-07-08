% SIM_CASCADE  The firmware's cascade, in simulation: balance PID -> wheel-speed
%              PI -> motor -> plant. NO direct force control.
%
% sim_discrete.m (Step 6) closes the balance loop by computing a cart FORCE and
% turning it into a duty (F/F_stall + back-EMF feedforward). The firmware does
% NOT work that way: the balance loop outputs a common WHEEL-SPEED command
% w_common, and an inner per-wheel PI turns that into duty (the cascade in
% docs/theory/README.md). This script mirrors that exact structure so gains can
% be tuned in the units the firmware actually uses:
%
%   pitch estimate  --> balance_pid_sim --> w_common --> wheel_pi_sim --> duty
%        ^                (outer loop)                     (inner loop)     |
%        |                                                                  v
%        +----------------------- complementary filter <-- IMU <-- PLANT <-+
%
% The two controllers are the SAME code the firmware runs
% (balance_pid_sim.m / wheel_pi_sim.m are line-for-line ports of
% balance_pid.c / wheel_pi.c), so tuning here transfers directly.
%
% Simplifications (documented, to be lifted later):
%   * ONE lumped inner loop (the plant is a single symmetric cart), not two.
%   * Wheel speed = v/r (ground-relative); the -omega body-rotation term the real
%     encoder sees is neglected, matching the existing motor model in
%     sim_discrete.m.
%   * The sim motor is an ideal first-order torque-speed curve, so its speed
%     response (tau_w ~ 37 ms below) is faster than the real motor (~190 ms from
%     motor-identification.md). The inner gains here are therefore tuned to the
%     SIM motor; on hardware use the firmware wheel_pi defaults.

clear; clc;
here = fileparts(mfilename('fullpath'));
addpath(here);                 % params, plant_dynamics, wheel_pi_sim, balance_pid_sim
p = params();

% ---- Inner wheel-speed loop: the ACTUAL firmware wheel_pi gains -----------------
% The sim motor now carries the real actuator lag (p.motor.tau_e), so the inner
% loop must be the firmware's, not an idealized IMC design - otherwise the sim
% inner loop is ~5x too fast and balance gains won't transfer. These are the
% lumped average of the per-wheel gains in firmware/main/wheel_pi.c.
inner.kp        = 0.150;                      % avg(0.1455, 0.1554)
inner.ki        = 0.750;                      % avg(0.6737, 0.8265)
inner.kff       = 33.3;                       % avg(34.36, 32.18)  [rad/s per duty]
inner.tau_f     = 0.02;                       % WHEEL_PI_TAU_F
inner.u_max     = 0.95;                       % WHEEL_PI_DUTY_MAX
inner.i_max     = 0.95;                       % WHEEL_PI_I_MAX
inner.brake_max = 0.4;                        % WHEEL_PI_BRAKE_MAX
inner.ff_en     = true;
inner.db_en     = false;                      % firmware default (off)
inner.neutral   = 0.02;                       % WHEEL_PI_NEUTRAL
inner.db_floor  = p.motor.deadband;           % WHEEL_PI_DB_FLOOR
Kw = p.motor.w_noload;

% ---- Outer balance loop -------------------------------------------------------
% In THIS cascade the gain roles rotate (see the note at the bottom of the file):
% Ki is the restoring STIFFNESS (needs r*Ki > g, i.e. Ki > g/r ~= 302), Kp is
% DAMPING, Kd adds effective inertia (slows the loop so the motor lag hurts less).
% With the REALISTIC motor lag (p.motor.tau_e) + the firmware inner loop, the loop
% needs a lot of damping: tune_cascade.m lands around Kp~60 Ki~500 Kd~8. (The
% ideal-motor optimum Kp=18 Ki=750 Kd=0 limit-cycles / falls once the lag is in -
% exactly what the robot does.)
outer.kp        = 60.0;
outer.ki        = 500.0;
outer.kd        = 8.0;
outer.theta_ref = 0.0;
outer.w_max     = 30.0;                       % BALANCE_W_MAX
outer.i_max     = 30.0;                       % BALANCE_I_MAX
BAL_MAX_TILT    = 0.6;                        % BALANCE_MAX_TILT (fall cutoff, rad)

% ---- Estimator (complementary) + sensor model (as in sim_discrete.m) ----------
alpha       = 0.98;                           % firmware COMP_ALPHA_DEFAULT
gyro_bias   = deg2rad(0.1);
gyro_noise  = deg2rad(1.0);
accel_noise = deg2rad(2.0);
w_meas_noise = 0.3;                           % wheel-speed measurement noise [rad/s]

% ---- Timing -------------------------------------------------------------------
dt_ctrl = p.dt_ctrl;  n_sub = 5;  dt_sub = dt_ctrl / n_sub;
T = 8.0;  Nticks = round(T / dt_ctrl);

% ---- State + controller memory ------------------------------------------------
x         = [0; 0; deg2rad(5); 0];            % start tilted 5 deg
theta_est = x(3);
vel_prev  = x(2);
bal_i     = 0;                                % balance integrator
wp_i      = 0;                                % inner integrator
wp_wf     = x(2) / p.r_wheel;                 % inner measurement filter (seed at truth)
u_eff     = 0;                                % actuator-lag state (effective duty)

% Lumped loop DEAD TIME (pure transport delay): IMU sample + I2C read + estimator
% + control compute + PWM update + driver. Distinct from the motor's first-order
% lag (tau_e) - a pure delay is far more destabilizing. Set 0 to disable.
loop_delay = 0.015;                           % [s]  (~3 ticks at 200 Hz)
n_delay    = max(0, round(loop_delay / dt_ctrl));
duty_q     = zeros(1, n_delay + 1);           % FIFO of pending duty commands

randn('seed', 1);

t        = (0:Nticks) * dt_ctrl;
th_true  = zeros(1, Nticks+1);  th_true(1) = rad2deg(x(3));
th_hat   = zeros(1, Nticks+1);  th_hat(1)  = rad2deg(theta_est);
wcmd_log = zeros(1, Nticks+1);
wmea_log = zeros(1, Nticks+1);
duty_log = zeros(1, Nticks+1);
pos_log  = zeros(1, Nticks+1);
fell = false;

for n = 1:Nticks
  % ===== SENSE (200 Hz) =====
  gyro_meas = x(4) + gyro_bias + gyro_noise*randn();          % tilt rate [rad/s]
  a_cart    = (x(2) - vel_prev) / dt_ctrl;                    % cart accel estimate
  theta_acc = x(3) + a_cart/p.g + accel_noise*randn();        % accel tilt [rad]
  vel_prev  = x(2);
  w_meas    = x(2)/p.r_wheel + w_meas_noise*randn();          % wheel speed [rad/s]

  % ===== ESTIMATE: complementary filter =====
  theta_est = alpha*(theta_est + gyro_meas*dt_ctrl) + (1-alpha)*theta_acc;

  % ===== CONTROL: balance (outer) -> wheel speed (inner) =====
  if abs(theta_est) < BAL_MAX_TILT
    [w_common, bal_i] = balance_pid_sim(theta_est, gyro_meas, bal_i, dt_ctrl, outer);
    [duty, wp_i, wp_wf] = wheel_pi_sim(w_common, w_meas, wp_i, wp_wf, dt_ctrl, inner);
  else
    % Fallen: cut motors and reset both controllers (mirrors control.c).
    w_common = 0; duty = 0; bal_i = 0; wp_i = 0; wp_wf = w_meas;
  end

  % ===== TRANSPORT DELAY: the duty computed now is applied n_delay ticks later ==
  duty_q   = [duty_q(2:end), duty];
  duty_app = duty_q(1);

  % ===== ACTUATE + PLANT: hold duty over the tick (ZOH); motor lag on u_eff =====
  for s = 1:n_sub
    u_eff   = u_eff + (dt_sub / p.motor.tau_e) * (duty_app - u_eff);   % first-order actuator lag
    u_m     = u_eff;
    if abs(u_m) < p.motor.deadband, u_m = 0.0; end   % motor dead zone: no drive torque below ~10% duty
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
    fell = true; printf('FELL OVER at t = %.2f s\n', n*dt_ctrl);
    break;
  end
end

% ---- Report -------------------------------------------------------------------
printf('Cascade sim @ %d Hz\n', p.f_ctrl);
printf('Inner PI (firmware wheel_pi): Kp=%.4f Ki=%.4f  (K=%.2f rad/s/duty, motor tau_e=%.3f s)\n', ...
       inner.kp, inner.ki, inner.kff, p.motor.tau_e);
printf('Balance PID: Kp=%.1f Ki=%.1f Kd=%.1f, w_max=%.0f rad/s\n', ...
       outer.kp, outer.ki, outer.kd, outer.w_max);
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
title(sprintf('Cascade: balance PID -> wheel PI (Kp=%.0f Ki=%.0f Kd=%.0f)', ...
      outer.kp, outer.ki, outer.kd));

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

% ------------------------------------------------------------------------------
% WHY THE GAIN ROLES ROTATE (velocity-command balancing)
%
% With a wheel-SPEED inner loop the base linear acceleration is the derivative of
% our command:  a = r * d(w_common)/dt.  With w_common = Kp*th + Ki*Int(th) + Kd*th_dot,
%
%     a = r*(Kp*th_dot + Ki*th + Kd*th_ddot).
%
% Small-angle pendulum  L*th_ddot = g*th - a  gives
%
%     (L + r*Kd)*th_ddot + r*Kp*th_dot + (r*Ki - g)*th = 0.
%
% All coefficients must share a sign, so:
%   * Ki gives the RESTORING STIFFNESS and MUST satisfy  r*Ki > g  =>  Ki > g/r (~302).
%   * Kp gives DAMPING (the th_dot term).
%   * Kd adds EFFECTIVE INERTIA.
% This is the opposite of force-based PID intuition, and is the key thing the
% cascade restructure makes visible. Tilt-only still drifts (no r*Ki-g "spring"
% on POSITION) - a velocity/position outer loop is the next layer.
% ------------------------------------------------------------------------------
