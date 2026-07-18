function R = closed_loop_id(csvfile)
% CLOSED_LOOP_ID  Identify the inner wheel-speed closed loop (setpoint -> speed).
%   R = closed_loop_id()          default 'motors.csv' in this folder
%   R = closed_loop_id(csvfile)   given a 'motors' topic CSV
%
% Records come from TEST_MOTOR_CONTROLLERS driven by a speed playback script, so the
% input is the setpoint velL_sp/velR_sp [rad/s] and the output is the measured speed
% velL/velR [rad/s]. CSV columns (header-matched):
%   t, velL, velR, velL_sp, velR_sp, mL, mR, rawL, rawR
%
% Per wheel it fits two models of the closed loop T(s) = w / w_sp:
%   1st order  tau_cl*dw/dt + w = K*w_sp      -> K_cl, tau_cl   (the IMC design target)
%   2nd order  ARX(2,2) least squares         -> wn, zeta, DC gain, and the model's
%                                                step overshoot / rise / settle
% and measures the empirical overshoot straight off each setpoint step, so the model
% can be sanity-checked against the raw response.

  if nargin < 1 || isempty(csvfile)
    here = fileparts(mfilename('fullpath'));
    csvfile = fullfile(here, 'motors.csv');
  end

  [D, names] = read_named_csv(csvfile);
  col = @(nm) D(:, colidx(names, nm));
  t  = col('t') - D(1, colidx(names, 't'));    % seconds from the start of the file
  w  = struct('L', col('velL'),    'R', col('velR'));      % measured speed [rad/s]
  r  = struct('L', col('velL_sp'), 'R', col('velR_sp'));   % setpoint (input) [rad/s]
  dt = median(diff(t));
  printf('\nloaded %d samples, %.2f s, dt = %.4f s (%.0f Hz)\n', ...
         numel(t), t(end), dt, 1/dt);

  R.L = fit_wheel('LEFT ', t, w.L, r.L, dt);
  R.R = fit_wheel('RIGHT', t, w.R, r.R, dt);

  [csvdir, csvbase] = fileparts(csvfile);
  outpng = fullfile(csvdir, [csvbase '.png']);   % same name as the CSV, .png
  plot_fits(t, w, r, R, outpng);
end

% ---------------------------------------------------------------------------
function f = fit_wheel(name, t, w, r, dt)
  ws = movavg(w, 5);                 % tame speed quantization before differentiating

  % ---- 1st order: dw/dt = a*w + b*r + c  (IMC target: T = 1/(tau_cl*s+1)) ----
  dw  = diff(ws) / dt;
  Phi = [ ws(1:end-1), r(1:end-1), ones(numel(dw),1) ];
  th  = Phi \ dw;                    % [a; b; c]
  a = th(1); b = th(2); c = th(3);
  tau_cl = -1 / a;  K1 = -b / a;
  w1   = sim_first_order(r, dt, a, b, c);
  R2_1 = rsq(w, w1);

  % ---- 2nd order: ARX(2,2), y[k] = -a1 y[k-1] - a2 y[k-2] + b1 r[k-1] + b2 r[k-2] ----
  y = ws;  N = numel(y);
  P2 = [ -y(2:N-1), -y(1:N-2), r(2:N-1), r(1:N-2) ];
  q  = P2 \ y(3:N);                  % [a1; a2; b1; b2]
  den = [1, q(1), q(2)];  num = [q(3), q(4)];
  dc  = sum(num) / sum(den);         % T(1): steady-state tracking gain (want ~1)
  [wn, zeta, tau2] = disc2cont(den, dt);
  w2   = sim_arx(r, den, num);
  R2_2 = rsq(w, w2);
  [OSm, trm, tsm] = step_char(den, num, dt);

  % ---- empirical step character straight off the data ----
  E = step_metrics(ws, r, dt);

  printf('\n[%s]\n', name);
  printf('  1st order : K_cl = %.3f, tau_cl = %.4f s        (R^2 = %.3f)\n', ...
         K1, tau_cl, R2_1);
  if isnan(zeta)
    printf('  2nd order : overdamped, tau = [%.4f, %.4f] s, DC = %.3f  (R^2 = %.3f)\n', ...
           tau2(1), tau2(2), dc, R2_2);
  else
    printf('  2nd order : wn = %.1f rad/s, zeta = %.3f, DC = %.3f      (R^2 = %.3f)\n', ...
           wn, zeta, dc, R2_2);
  end
  printf('  model step: overshoot = %5.1f %%, rise(10-90) = %.3f s, settle(2%%) = %.3f s\n', ...
         OSm, trm, tsm);
  printf('  data  step: overshoot mean = %.1f %%, max = %.1f %% over %d step(s), ', ...
         E.OS_mean, E.OS_max, E.n);
  printf('rise = %.3f s, settle = %.3f s (%.0f rad/s step)\n', ...
         E.big.tr, E.big.ts, E.big.size);
  printf('  overshoot by step:');
  for i = 1:size(E.tbl, 1)
    printf('  %.0f->%.0f: %.0f%%', E.tbl(i,1), E.tbl(i,2), E.tbl(i,3));
  end
  printf('\n');

  f = struct('K1', K1, 'tau_cl', tau_cl, 'R2_1', R2_1, 'a', a, 'b', b, 'c', c, ...
             'den', den, 'num', num, 'wn', wn, 'zeta', zeta, 'tau2', tau2, ...
             'dc', dc, 'R2_2', R2_2, 'OS_model', OSm, 'tr_model', trm, ...
             'ts_model', tsm, 'E', E);
end

% ============================ model helpers ================================
function wh = sim_first_order(r, dt, a, b, c)
  wh = zeros(size(r));
  for k = 1:numel(r)-1
    wh(k+1) = wh(k) + dt * (a*wh(k) + b*r(k) + c);
  end
end

% Free-run ARX(2,2) simulation (uses its own past outputs, not the measured ones).
function yh = sim_arx(u, den, num)
  a1 = den(2); a2 = den(3); b1 = num(1); b2 = num(2);
  yh = zeros(size(u));
  for k = 3:numel(u)
    yh(k) = -a1*yh(k-1) - a2*yh(k-2) + b1*u(k-1) + b2*u(k-2);
  end
end

% Map the ARX denominator's discrete poles to a continuous 2nd-order character.
function [wn, zeta, tau2] = disc2cont(den, dt)
  s  = log(roots(den)) / dt;         % z -> s = ln(z)/dt
  wn = NaN; zeta = NaN; tau2 = [NaN, NaN];
  if any(abs(imag(s)) > 1e-6)        % complex pair -> underdamped
    wn   = abs(s(1));
    zeta = -real(s(1)) / wn;
  else                               % real poles -> two time constants
    tau2 = sort(-1 ./ real(s))';
  end
end

% Unit-step response of the identified discrete model: overshoot %, 10-90% rise, 2% settle.
function [OS, tr, ts] = step_char(den, num, dt)
  N = 4000;
  y = sim_arx(ones(N,1), den, num);
  yf = y(end);
  OS = max(0, (max(y) - yf) / yf * 100);
  tr = crossing_time((y - 0) * sign(yf), 0.1*abs(yf), 0.9*abs(yf), dt);
  out = find(abs(y - yf) > 0.02*abs(yf), 1, 'last');
  ts  = isempty(out) * 0 + ~isempty(out) * out * dt;
end

% ========================== empirical step metrics =========================
% Split the run at each setpoint change and, per step big enough to trust,
% measure overshoot past steady state; also rise/settle on the biggest step.
function E = step_metrics(w, r, dt)
  ch = [1; find(abs(diff(r)) > 1e-6) + 1; numel(r)+1];
  minlen = round(0.6 / dt);          % need a settled tail to trust a step
  tbl = []; big = struct('OS', NaN, 'tr', NaN, 'ts', NaN, 'size', 0);
  for s = 1:numel(ch)-1
    i0 = ch(s); i1 = ch(s+1) - 1;
    if i1 - i0 < minlen, continue; end
    sp_from = r(max(i0-1, 1)); sp_to = r(i0);
    w0   = w(i0);
    tail = i0 + floor(0.7*(i1 - i0)) : i1;
    ss   = mean(w(tail));
    step = ss - w0;
    if abs(step) < 1.0, continue; end          % skip tiny / repeated setpoints
    seg = w(i0:i1);
    pk  = (step > 0) * max(seg) + (step <= 0) * min(seg);
    OS  = max(0, (pk - ss) / step * 100);
    tbl(end+1, :) = [sp_from, sp_to, OS];
    if abs(step) >= big.size
      big.size = abs(step); big.OS = OS;
      big.tr = crossing_time((seg - w0) * sign(step), 0.1*abs(step), 0.9*abs(step), dt);
      tol = max(0.02*abs(step), 1.0);          % settle band, floored at ~one speed count
      out = find(abs(seg - ss) > tol, 1, 'last');
      big.ts = isempty(out) * 0 + ~isempty(out) * out * dt;
    end
  end
  if isempty(tbl), tbl = [NaN, NaN, NaN]; end
  E.tbl = tbl;
  E.OS_mean = mean(tbl(:,3)); E.OS_max = max(tbl(:,3)); E.n = size(tbl,1); E.big = big;
end

% Time from the first crossing of ylo to the first crossing of yhi (positive-going).
function tc = crossing_time(y, ylo, yhi, dt)
  i_lo = find(y >= ylo, 1, 'first');
  i_hi = find(y >= yhi, 1, 'first');
  if isempty(i_lo) || isempty(i_hi), tc = NaN; else tc = (i_hi - i_lo) * dt; end
end

% ============================== utilities ==================================
function r2 = rsq(y, yh)
  ss_res = sum((y - yh).^2);
  ss_tot = sum((y - mean(y)).^2);
  r2 = 1 - ss_res / max(ss_tot, eps);
end

function y = movavg(x, N)
  if N <= 1, y = x; return; end
  k = ones(N,1) / N;
  y = filter(k, 1, x);
  s = floor(N/2);                              % zero-phase shift
  y = [y(s+1:end); x(end-s+1:end)];
end

% Read header-mapped CSV; empty fields -> 0 (dlmread behaviour)
function [D, names] = read_named_csv(csvfile)
  fid = fopen(csvfile, 'r');
  if fid < 0, error('closed_loop_id: cannot open %s', csvfile); end
  hdr = fgetl(fid);
  fclose(fid);
  names = strtrim(strsplit(hdr, ','));
  D = dlmread(csvfile, ',', 1, 0);             % skip the header row
end

% Named column index (case-sensitive)
function i = colidx(names, nm)
  i = find(strcmp(names, nm), 1);
  if isempty(i), error('closed_loop_id: column "%s" not found in CSV header', nm); end
end

% ---------------------------------------------------------------------------
function plot_fits(t, w, r, R, outpng)
  try
    fig = figure('Name', 'closed-loop identification', 'NumberTitle', 'off', ...
                 'Visible', 'off');
    sides = {'L','R'}; names = {'LEFT','RIGHT'};
    dt = median(diff(t));
    for i = 1:2
      s = sides{i}; f = R.(s);
      w1 = sim_first_order(r.(s), dt, f.a, f.b, f.c);
      w2 = sim_arx(r.(s), f.den, f.num);

      subplot(2,1,i); hold on; grid on;
      plot(t, w.(s), 'Color', [.7 .7 .7], 'DisplayName', 'measured');
      plot(t, r.(s), 'k--', 'DisplayName', 'setpoint');
      plot(t, w1, 'b',  'LineWidth', 1.0, 'DisplayName', '1st-order model');
      plot(t, w2, 'r',  'LineWidth', 1.2, 'DisplayName', '2nd-order model');
      xlabel('t [s]'); ylabel('\omega [rad/s]');
      if isnan(f.zeta)
        title(sprintf(['%s:  \\tau_{cl}=%.3f s (R^2=%.2f)  |  2nd overdamped, ' ...
                       'DC=%.2f (R^2=%.2f)'], names{i}, f.tau_cl, f.R2_1, f.dc, f.R2_2));
      else
        title(sprintf(['%s:  \\tau_{cl}=%.3f s (R^2=%.2f)  |  \\omega_n=%.1f, ' ...
                       '\\zeta=%.2f, OS=%.0f%% (R^2=%.2f)'], names{i}, f.tau_cl, ...
                      f.R2_1, f.wn, f.zeta, f.OS_model, f.R2_2));
      end
      legend('Location', 'southeast');
    end
    print(fig, outpng, '-dpng', '-r110');
    printf('\nsaved fit plot -> %s\n', outpng);
  catch e
    printf('\n(plot skipped: %s)\n', e.message);
  end
end
