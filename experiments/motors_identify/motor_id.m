function R = motor_id(csvfile)
% MOTOR_ID  Identify per-wheel motor parameters from a recorded step response.
%   R = motor_id()            default recording under experiments/
%   R = motor_id(csvfile)     given 'motors' topic CSV
%
% CSV columns (header-matched): t, velL, velR, velL_sp, velR_sp, mL, mR, uL, uR
% velL/R [rad/s], mL/R PWM duty [-1..+1]. vel*_sp/u* ignored during open-loop ID.
%
% Model per wheel: tau*dw/dt + w = K*u  <=>  G(s) = K/(tau*s+1)
%   K [rad/s per duty], tau [s]
% Maps to sim torque line: w_noload = K, tau_stall = I_eff*K/tau.
% Two fits per wheel: static (K, deadband from steady-state segments);
% dynamic (global LS on dw/dt = a*w + b*u + c -> tau=-1/a, K=-b/a).

  if nargin < 1 || isempty(csvfile)
    here = fileparts(mfilename('fullpath'));
    csvfile = fullfile(here, '..', '..', 'experiments', 'motors_identify', ...
                       'motors.csv');
  end
  % sim wheel inertia for torque mapping
  addpath(fullfile(fileparts(mfilename('fullpath')), '..', '..', 'simulation'));

  [D, names] = read_named_csv(csvfile);    % header-mapped columns
  col = @(nm) D(:, colidx(names, nm));
  t  = col('t') - D(1, colidx(names, 't'));% seconds from the start of the file
  w  = struct('L', col('velL'), 'R', col('velR'));
  u  = struct('L', col('mL'),   'R', col('mR'));
  dt = median(diff(t));
  printf('\nloaded %d samples, %.2f s, dt = %.4f s (%.0f Hz)\n', ...
         numel(t), t(end), dt, 1/dt);

  R.L = fit_wheel('LEFT ', t, w.L, u.L);
  R.R = fit_wheel('RIGHT', t, w.R, u.R);

  % map dynamic fit -> sim motor torque parameters
  p = params();
  for side = {'L','R'}
    s = side{1}; f = R.(s);
    w_noload  = abs(f.K_dyn);                 % no-load speed at |u| = 1
    tau_stall = p.I_wheel * w_noload / f.tau;
    R.(s).w_noload  = w_noload;
    R.(s).tau_stall = tau_stall;
    printf(['  -> params.m (%s): w_noload = %.1f rad/s (%.0f rpm), ' ...
            'tau_stall ~ %.3f N*m  [I_wheel=%.2e]\n'], ...
           s, w_noload, w_noload*60/(2*pi), tau_stall, p.I_wheel);
  end

  [csvdir, csvbase] = fileparts(csvfile);
  outpng = fullfile(csvdir, [csvbase '.png']);   % same name as the CSV, .png
  plot_fits(t, w, u, R, outpng);
end

% Read header-mapped CSV; empty fields -> 0 (dlmread behaviour)
function [D, names] = read_named_csv(csvfile)
  fid = fopen(csvfile, 'r');
  if fid < 0, error('motor_id: cannot open %s', csvfile); end
  hdr = fgetl(fid);
  fclose(fid);
  names = strtrim(strsplit(hdr, ','));
  D = dlmread(csvfile, ',', 1, 0);         % skip the header row
end

% Named column index (case-sensitive)
function i = colidx(names, nm)
  i = find(strcmp(names, nm), 1);
  if isempty(i), error('motor_id: column "%s" not found in CSV header', nm); end
end

% ---------------------------------------------------------------------------
function f = fit_wheel(name, t, w, u)
  dt = median(diff(t));

  % smooth quantized speed before differentiating (~0.95 rad/s/count)
  N  = 5;
  ws = movavg(w, N);

  % dynamic: dw/dt = a*w + b*u + c
  dw = diff(ws) / dt;
  Phi = [ ws(1:end-1), u(1:end-1), ones(numel(dw),1) ];
  theta = Phi \ dw;                       % [a; b; c]
  a = theta(1); b = theta(2);
  tau = -1 / a;
  Kd  = -b / a;

  % static: steady-state speed per constant-duty segment
  [useg, wss] = segment_steadystate(t, ws, u);
  [Ks, deadband] = static_gain_deadband(useg, wss);

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
  s = floor(N/2);                          % zero-phase shift
  y = [y(s+1:end); x(end-s+1:end)];
end

% Constant-duty segments; steady-state = mean of last 40% of each run
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

% w_ss = K*(|u| - db)*sign(u); linear fit of |w_ss| vs |u|
function [K, db] = static_gain_deadband(useg, wss)
  m = abs(useg) > 1e-3;
  if nnz(m) < 2, K = NaN; db = NaN; return; end
  au = abs(useg(m)); aw = abs(wss(m));
  P  = [au, ones(numel(au),1)] \ aw;       % aw = slope*au + intercept
  K  = P(1);
  db = -P(2) / P(1);
  K  = sign(mean(wss(m)./useg(m))) * K;
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
      title(sprintf('%s wheel:  K=%.2f rad/s/duty,  \\tau=%.3f s,  R^2=%.3f', ...
                    names{i}, f.K_dyn, f.tau, f.R2));
      legend('Location', 'northeast');
    end
    print(fig, outpng, '-dpng', '-r110');
    printf('\nsaved fit plot -> %s\n', outpng);
  catch e
    printf('\n(plot skipped: %s)\n', e.message);
  end
end
