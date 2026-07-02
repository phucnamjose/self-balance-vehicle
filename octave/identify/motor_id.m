function R = motor_id(csvfile)
% MOTOR_ID  Identify per-wheel motor parameters from a recorded step response.
%
%   R = motor_id()            uses the default recording under experiments/
%   R = motor_id(csvfile)     uses the given 'motors' topic CSV
%
% octave --eval "motor_id('../../experiments/motors_identify/motors.csv')"
%
% Expects a CSV recorded by the web app's "motors" topic:
%
%     t, velL, velR, velL_sp, velR_sp, mL, mR
%
% where t is seconds (from boot), velL/velR are measured wheel speeds [rad/s],
% and mL/mR are the commanded PWM duties in -1..+1 (the playback script). The
% setpoint columns are empty and ignored.
%
% MODEL (per wheel). Treating each free-spinning wheel as first order from duty
% u to angular speed w:
%
%       tau * dw/dt + w = K * u          <=>     G(s) = K / (tau*s + 1)
%
%   K   [rad/s per unit duty]  steady-state gain = no-load speed at full duty
%   tau [s]                    mechanical time constant
%
% This matches the sim's torque-speed line tau(u,w) = u*tau_stall
% - (tau_stall/w_noload)*w (see octave/simulation/params.m): for a free wheel
%   w_noload = K   and   tau_stall = I_eff * K / tau,
% with I_eff the reflected wheel inertia (uses p.I_wheel from params()).
%
% Two independent estimates are produced per wheel:
%   1) STATIC : from each constant-duty segment's steady-state speed, fit
%      w_ss = K*(|u| - deadband)*sign(u)  ->  gain K and the deadband.
%   2) DYNAMIC: one global least-squares fit of  dw/dt = a*w + b*u + c  over the
%      whole run  ->  tau = -1/a, K = -b/a  (c captures static friction).
%
% A figure overlays the measured speed with the identified model's simulation.

  if nargin < 1 || isempty(csvfile)
    here = fileparts(mfilename('fullpath'));
    csvfile = fullfile(here, '..', '..', 'experiments', 'motors_identify', ...
                       'motors.csv');
  end
  % Reuse the sim's physical constants (wheel inertia) for the torque mapping.
  addpath(fullfile(fileparts(mfilename('fullpath')), '..', 'simulation'));

  D = dlmread(csvfile, ',', 1, 0);         % skip the header row
  t  = D(:,1) - D(1,1);                    % seconds from the start of the file
  w  = struct('L', D(:,2), 'R', D(:,3));
  u  = struct('L', D(:,6), 'R', D(:,7));
  dt = median(diff(t));
  printf('\nloaded %d samples, %.2f s, dt = %.4f s (%.0f Hz)\n', ...
         numel(t), t(end), dt, 1/dt);

  R.L = fit_wheel('LEFT ', t, w.L, u.L);
  R.R = fit_wheel('RIGHT', t, w.R, u.R);

  % ---- map the dynamic fit back to the sim's motor torque parameters --------
  p = params();
  for side = {'L','R'}
    s = side{1}; f = R.(s);
    w_noload  = abs(f.K_dyn);                 % no-load speed at |u| = 1
    tau_stall = p.I_wheel * w_noload / f.tau; % from tau = I*w_noload/tau_stall
    R.(s).w_noload  = w_noload;
    R.(s).tau_stall = tau_stall;
    printf(['  -> params.m (%s): w_noload = %.1f rad/s (%.0f rpm), ' ...
            'tau_stall ~ %.3f N*m  [I_wheel=%.2e]\n'], ...
           s, w_noload, w_noload*60/(2*pi), tau_stall, p.I_wheel);
  end

  outpng = fullfile(fileparts(csvfile), 'motor_id_fit.png');
  plot_fits(t, w, u, R, outpng);
end

% ---------------------------------------------------------------------------
function f = fit_wheel(name, t, w, u)
  dt = median(diff(t));

  % Smooth the (heavily quantized) speed before differentiating. One encoder
  % count per tick is ~0.95 rad/s, so a short zero-phase moving average tames
  % the finite-difference noise without smearing the ~0.1 s time constant.
  N  = 5;
  ws = movavg(w, N);

  % ---- DYNAMIC fit: dw/dt = a*w + b*u + c  (global least squares) -----------
  dw = diff(ws) / dt;
  Phi = [ ws(1:end-1), u(1:end-1), ones(numel(dw),1) ];
  theta = Phi \ dw;                       % [a; b; c]
  a = theta(1); b = theta(2);
  tau = -1 / a;
  Kd  = -b / a;

  % ---- STATIC fit: steady-state speed per constant-duty segment -------------
  [useg, wss] = segment_steadystate(t, ws, u);
  % Fit gain + deadband on each drive direction from the static points.
  [Ks, deadband] = static_gain_deadband(useg, wss);

  % ---- fit quality: simulate the first-order model and compare --------------
  wh = simulate_first_order(u, dt, a, b, theta(3));
  ss_res = sum((w - wh).^2);
  ss_tot = sum((w - mean(w)).^2);
  R2 = 1 - ss_res / max(ss_tot, eps);

  printf('\n[%s]\n', name);
  printf('  dynamic : K = %+7.2f rad/s/duty,  tau = %6.3f s   (fit R^2 = %.3f)\n', ...
         Kd, tau, R2);
  printf('  static  : K = %+7.2f rad/s/duty,  deadband = %.3f duty\n', Ks, deadband);
  if Kd < 0
    printf('  note    : negative gain -> this wheel''s encoder/motor sign is flipped\n');
  end

  f = struct('K_dyn', Kd, 'tau', tau, 'K_static', Ks, ...
             'deadband', deadband, 'R2', R2, 'a', a, 'b', b, 'c', theta(3));
end

% ---------------------------------------------------------------------------
function y = movavg(x, N)
  if N <= 1, y = x; return; end
  k = ones(N,1) / N;
  y = filter(k, 1, x);
  s = floor(N/2);                          % shift back to make it zero-phase
  y = [y(s+1:end); x(end-s+1:end)];
end

% Split into runs of constant duty; return each run's duty and steady-state
% speed (mean over the last 40% of the run, once the transient has settled).
function [useg, wss] = segment_steadystate(t, w, u)
  starts = [1; find(diff(u) ~= 0) + 1; numel(u)+1];
  useg = []; wss = [];
  for s = 1:numel(starts)-1
    i0 = starts(s); i1 = starts(s+1) - 1;
    if i1 - i0 < 10, continue; end         % skip too-short segments
    tail = i0 + floor(0.6*(i1 - i0)) : i1;
    useg(end+1,1) = u(i0);
    wss(end+1,1)  = mean(w(tail));
  end
end

% Fit w_ss = K*(|u| - db)*sign(u) using the drive segments (|u| large enough to
% be past the deadband). Uses a simple linear fit of |w_ss| vs |u|.
function [K, db] = static_gain_deadband(useg, wss)
  m = abs(useg) > 1e-3;
  if nnz(m) < 2, K = NaN; db = NaN; return; end
  au = abs(useg(m)); aw = abs(wss(m));
  P  = [au, ones(numel(au),1)] \ aw;       % aw = slope*au + intercept
  K  = P(1);                               % rad/s per unit duty (magnitude)
  db = -P(2) / P(1);                       % |u| where speed extrapolates to 0
  K  = sign(mean(wss(m)./useg(m))) * K;    % restore sign of the wheel
end

function wh = simulate_first_order(u, dt, a, b, c)
  wh = zeros(size(u));
  for k = 1:numel(u)-1
    wh(k+1) = wh(k) + dt * (a*wh(k) + b*u(k) + c);
  end
end

% ---------------------------------------------------------------------------
function plot_fits(t, w, u, R, outpng)
  try
    fig = figure('Name', 'motor identification', 'NumberTitle', 'off', ...
                 'Visible', 'off');
    sides = {'L','R'}; names = {'LEFT','RIGHT'};
    for i = 1:2
      s = sides{i}; f = R.(s);
      dt = median(diff(t));
      wh = simulate_first_order(u.(s), dt, f.a, f.b, f.c);

      subplot(2,1,i); hold on; grid on;
      plot(t, w.(s), 'Color', [.7 .7 .7], 'DisplayName', 'measured');
      plot(t, wh, 'r', 'LineWidth', 1.2, 'DisplayName', 'model');
      plot(t, u.(s)*max(abs(w.(s))), 'b--', 'DisplayName', 'duty (scaled)');
      xlabel('t [s]'); ylabel('\omega [rad/s]');
      title(sprintf('%s wheel:  K=%.1f rad/s/duty,  \\tau=%.3f s,  R^2=%.3f', ...
                    names{i}, f.K_dyn, f.tau, f.R2));
      legend('Location', 'northeast');
    end
    print(fig, outpng, '-dpng', '-r110');
    printf('\nsaved fit plot -> %s\n', outpng);
  catch e
    printf('\n(plot skipped: %s)\n', e.message);
  end
end
