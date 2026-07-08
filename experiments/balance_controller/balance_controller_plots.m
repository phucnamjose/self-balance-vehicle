% BALANCE_CONTROLLER_PLOTS  Figures for docs/theory/balance-controller.md.
%
% The balance (standing) controller is the OUTER loop of the cascade: it reads
% the tilt estimate (theta, theta_dot) and drives the wheels to hold the body
% upright, actively stabilizing the right-half-plane pole of the inverted
% pendulum. These figures show:
%   1. it works: recovery from an initial tilt + a mid-run disturbance kick,
%      next to the open-loop fall it is fighting;
%   2. why tilt alone is not enough: angle-only PID balances but the cart
%      drifts, while full-state LQR balances AND returns home;
%   3. how feedback stabilizes: the open-loop poles (one in the RHP) are dragged
%      into the left half-plane by the LQR gain;
%   4. the stabilization condition: the tilt P-gain must exceed a threshold
%      (it must "beat gravity") before the closed-loop pole crosses into the LHP.
%
% The plant + design are reused from ../../simulation (params, plant_dynamics,
% linearize, lqr_gain) so these figures always match the sim. Base Octave only.
%
% Generates four PNGs into docs/theory/ (referenced by balance-controller.md):
%   balance-recovery.png    - closed-loop recovery + disturbance vs open-loop fall
%   balance-pid-vs-lqr.png  - angle-only PID drifts; full-state LQR returns home
%   balance-poles.png       - s-plane: open-loop poles -> LQR closed-loop poles
%   balance-threshold.png   - rightmost tilt pole vs Kp: the stabilization limit
%
% Run:  cd experiments/balance_controller && octave --eval balance_controller_plots

function balance_controller_plots()

  here   = fileparts(mfilename('fullpath'));
  outdir = fullfile(here, '..', '..', 'docs', 'theory');
  addpath(fullfile(here, '..', '..', 'simulation'));   % params, plant, linearize, lqr_gain
  warning('off', 'Octave:gnuplot-graphics');
  graphics_toolkit('gnuplot');

  p       = params();
  [A, B]  = linearize(p);
  F_max   = p.n_wheels * p.motor.tau_stall / p.r_wheel;   % force ceiling [N]

  % ---- shared colours (match the other theory figures) --------------------
  col_ol   = [0.85 0.33 0.10];       % open-loop / unstable (orange)
  col_cl   = [0.00 0.45 0.74];       % closed-loop / stable (blue)
  col_lqr  = [0.47 0.16 0.51];       % LQR (purple)
  col_eff  = [0.20 0.55 0.20];       % effort (green)
  col_grey = [0.55 0.55 0.55];

  % ---- angle-only PID gains (force units), from sim_closedloop_pid.m -------
  Kp = 45.0; Ki = 15.0; Kd = 5.0;

  % ======================================================================
  % Fig 1: it works - closed-loop recovery + disturbance vs open-loop fall
  % ======================================================================
  dt = 0.001; T = 3.0; N = round(T/dt); t = (0:N)*dt;
  th0 = deg2rad(4);                      % start tilted 4 deg
  t_kick = 1.5; k_kick = round(t_kick/dt);   % a shove at t = 1.5 s
  dw_kick = 3.0;                         % sudden +3 rad/s tilt-rate disturbance

  % closed loop (PID on the nonlinear plant)
  x = [0;0;th0;0]; e_int = 0;
  TH = zeros(1,N+1); FF = zeros(1,N+1); TH(1) = rad2deg(x(3));
  for k = 1:N
    if k == k_kick, x(4) = x(4) + dw_kick; end   % external disturbance kick
    e = 0 - x(3); e_int = e_int + e*dt;
    F = -(Kp*e + Ki*e_int + Kd*(-x(4)));
    if F >  F_max, F =  F_max; e_int = e_int - e*dt; end
    if F < -F_max, F = -F_max; e_int = e_int - e*dt; end
    x = rk4(x, F, dt, p);
    TH(k+1) = rad2deg(x(3)); FF(k+1) = F;
  end

  % open loop (no control) - it just tips over
  xo = [0;0;th0;0]; THO = zeros(1,N+1); THO(1) = rad2deg(xo(3));
  for k = 1:N
    xo = rk4(xo, 0, dt, p);
    THO(k+1) = rad2deg(xo(3));
  end

  fig = figure('visible','off','position',[100 100 820 640]);

  subplot(2,1,1); hold on; grid on;
  plot(t, THO, '--', 'color', col_ol,  'linewidth', 2);
  plot(t, TH,        'color', col_cl,  'linewidth', 2);
  plot([t_kick t_kick], [-30 30], ':', 'color', col_grey, 'linewidth', 1.2);
  ylim([-15 30]); xlim([0 T]);
  ylabel('tilt \theta [deg]');
  title('Balance controller: recovery vs the open-loop fall it fights');
  text(t_kick+0.03, 26, 'disturbance kick', 'color', [0.3 0.3 0.3], 'fontsize', 9);
  text(0.30, 24, 'open loop: tips over', 'color', col_ol, 'fontsize', 10);
  text(1.05, -9, 'closed loop: recovers', 'color', col_cl, 'fontsize', 10);
  legend('open loop (no control)', 'closed loop (PID)', 'location', 'east');

  subplot(2,1,2); hold on; grid on;
  plot(t, FF, 'color', col_eff, 'linewidth', 1.8);
  plot([0 T], [ F_max  F_max], ':', 'color', col_grey);
  plot([0 T], [-F_max -F_max], ':', 'color', col_grey);
  plot([t_kick t_kick], [-F_max F_max], ':', 'color', col_grey, 'linewidth', 1.2);
  xlim([0 T]);
  xlabel('time [s]'); ylabel('drive effort F [N]');
  text(0.05, F_max*0.75, 'motor ceiling \pmF_{max}', 'color', [0.3 0.3 0.3], 'fontsize', 9);

  save_fig(fig, outdir, 'balance-recovery.png');

  % ======================================================================
  % Fig 2: why tilt alone is not enough - PID drifts, LQR returns home
  % ======================================================================
  Q = diag([5, 1, 200, 5]);  R = 0.05;      % LQR weights (lqr_design.m)
  K = lqr_gain(A, B, Q, R);

  dt = 0.001; T = 5.0; N = round(T/dt); t = (0:N)*dt;
  x0 = [0;0;deg2rad(8);0];
  TH2 = zeros(2,N+1); POS = zeros(2,N+1);
  for c = 1:2
    x = x0; e_int = 0;
    TH2(c,1) = rad2deg(x(3)); POS(c,1) = 100*x(1);
    for k = 1:N
      if c == 1
        e = 0 - x(3); e_int = e_int + e*dt;
        F = -(Kp*e + Ki*e_int + Kd*(-x(4)));
      else
        F = -K*x;
      end
      if F >  F_max, F =  F_max; end
      if F < -F_max, F = -F_max; end
      x = rk4(x, F, dt, p);
      TH2(c,k+1) = rad2deg(x(3)); POS(c,k+1) = 100*x(1);
    end
  end

  fig = figure('visible','off','position',[100 100 820 640]);

  subplot(2,1,1); hold on; grid on;
  plot(t, TH2(1,:), 'color', col_cl,  'linewidth', 2);
  plot(t, TH2(2,:), 'color', col_lqr, 'linewidth', 2);
  ylabel('tilt \theta [deg]'); xlim([0 T]);
  title('Angle-only PID vs full-state LQR (start 8 deg)');
  legend('PID (angle only)', 'LQR (full state)', 'location', 'northeast');

  subplot(2,1,2); hold on; grid on;
  plot(t, POS(1,:), 'color', col_cl,  'linewidth', 2);
  plot(t, POS(2,:), 'color', col_lqr, 'linewidth', 2);
  plot([0 T], [0 0], ':', 'color', col_grey);
  xlabel('time [s]'); ylabel('cart position [cm]'); xlim([0 T]);
  legend('PID: balances but drifts / holds offset', ...
         'LQR: balances AND returns home', 'location', 'east');

  save_fig(fig, outdir, 'balance-pid-vs-lqr.png');

  % ======================================================================
  % Fig 3: how feedback stabilizes - open-loop poles dragged into the LHP
  % ======================================================================
  ol = eig(A);              % open-loop (has a RHP pole)
  cl = eig(A - B*K);        % LQR closed-loop (all LHP)

  fig = figure('visible','off','position',[100 100 820 560]);
  hold on; grid on;

  xl = 25; yl = 3.5;
  % shade the unstable (right) half-plane lightly
  patch([0 xl xl 0], [-yl -yl yl yl], col_ol, 'facealpha', 0.06, 'edgecolor', 'none');
  plot([0 0], [-yl yl], 'color', col_grey, 'linewidth', 1.0);   % jw axis
  plot([-xl xl], [0 0], 'color', col_grey, 'linewidth', 1.0);   % real axis

  % arrow (above the axis) showing the RHP pole is pulled left across the jw axis
  ru = max(real(ol));                       % the unstable open-loop pole (~ +20.6)
  annotation_arrow(ru, 1.1, -6.4, 1.1, col_lqr);

  % open-loop poles (x), closed-loop poles (o)
  plot(real(ol), imag(ol), 'x', 'color', col_ol,  'markersize', 14, 'linewidth', 3);
  plot(real(cl), imag(cl), 'o', 'color', col_lqr, 'markersize', 11, 'linewidth', 2.5);

  xlim([-xl xl]); ylim([-yl yl]);
  xlabel('Re(s) [rad/s]'); ylabel('Im(s) [rad/s]');
  title('Feedback drags the unstable pole into the left half-plane');
  text( 8.5, -2.9, 'UNSTABLE (RHP)', 'color', col_ol,  'fontsize', 11);
  text(-23,  -2.9, 'STABLE (LHP)',   'color', col_lqr, 'fontsize', 11);
  text(ru-8.5, 0.55, sprintf('open-loop pole +%.1f', ru), 'color', col_ol, 'fontsize', 9);
  text(1.5, 1.55, 'pulled left by feedback', 'color', col_lqr, 'fontsize', 9);
  text(-24, 2.9, 'one fast closed-loop pole at -466 is off-scale to the left', ...
       'color', [0.4 0.4 0.4], 'fontsize', 8);
  legend('open-loop poles (eig A)', 'LQR closed-loop (eig A-BK)', 'location', 'southeast');

  save_fig(fig, outdir, 'balance-poles.png');

  % ======================================================================
  % Fig 4: the stabilization condition - tilt P-gain must beat gravity
  % ======================================================================
  % Reduced tilt sub-loop [theta; omega] with proportional-derivative feedback
  % F = Kp_t*theta + Kd_t*omega (positive F for a forward tilt drives the wheels
  % under the body). Using the linearized tilt row a43,a44 and drive b4:
  a43 = A(4,3); a44 = A(4,4); b4 = B(4);
  Kd_t = 5.0;                             % fix the damping gain
  KpG  = linspace(0, 25, 400);
  maxre = zeros(size(KpG));
  for i = 1:numel(KpG)
    Acl = [0 1; a43 + b4*KpG(i), a44 + b4*Kd_t];
    maxre(i) = max(real(eig(Acl)));
  end
  kp_thr = -a43 / b4;                     % crossing: a43 + b4*Kp = 0

  fig = figure('visible','off','position',[100 100 780 560]);
  hold on; grid on;
  % shade stable / unstable regions as full-height bands split at the threshold
  yl2 = [min(maxre)*1.1, max(maxre)*1.1];
  patch([0 kp_thr kp_thr 0],   [yl2(1) yl2(1) yl2(2) yl2(2)], col_ol,  'facealpha', 0.07, 'edgecolor','none');
  patch([kp_thr 25 25 kp_thr], [yl2(1) yl2(1) yl2(2) yl2(2)], col_eff, 'facealpha', 0.07, 'edgecolor','none');
  plot(KpG, maxre, 'color', col_cl, 'linewidth', 2.2);
  plot([0 25], [0 0], 'color', col_grey, 'linewidth', 1.0);
  plot([kp_thr kp_thr], yl2, ':', 'color', col_ol, 'linewidth', 1.5);
  xlim([0 25]); ylim(yl2);
  xlabel('tilt proportional gain  K_p  [N/rad]');
  ylabel('rightmost closed-loop pole  Re(s) [rad/s]');
  title('Stabilization condition: K_p must beat gravity');
  text(kp_thr+0.4, yl2(2)*0.6, sprintf('threshold K_p = %.1f', kp_thr), ...
       'color', col_ol, 'fontsize', 10);
  text(1.0,  yl2(2)*0.35, 'too weak: still UNSTABLE', 'color', col_ol,  'fontsize', 9);
  text(13,   yl2(1)*0.6,  'stable (pole in LHP)',      'color', col_eff, 'fontsize', 9);

  save_fig(fig, outdir, 'balance-threshold.png');

  printf('LQR gain K = [%.2f %.2f %.2f %.2f]\n', K);
  printf('open-loop poles:  '); printf('%+.2f ', real(ol)); printf('\n');
  printf('closed-loop max Re = %+.3f\n', max(real(cl)));
  printf('tilt P-gain threshold Kp = %.2f N/rad\n', kp_thr);
  printf('Done. Figures written to %s\n', outdir);
end

% ---- helpers -----------------------------------------------------------
function x = rk4(x, F, dt, p)
  k1 = plant_dynamics(x,           F, p);
  k2 = plant_dynamics(x+0.5*dt*k1, F, p);
  k3 = plant_dynamics(x+0.5*dt*k2, F, p);
  k4 = plant_dynamics(x+    dt*k3, F, p);
  x  = x + (dt/6)*(k1 + 2*k2 + 2*k3 + k4);
end

function annotation_arrow(x0, y0, x1, y1, col)
  plot([x0 x1], [y0 y1], '-', 'color', col, 'linewidth', 1.3);
  ang = atan2(y1-y0, x1-x0); L = 1.4;
  for da = [2.7 -2.7]
    plot([x1 x1+L*cos(ang+da)], [y1 y1+L*sin(ang+da)], '-', 'color', col, 'linewidth', 1.3);
  end
end

function save_fig(fig, outdir, name)
  outfile = fullfile(outdir, name);
  print(fig, '-dpng', '-r100', outfile);
  printf('  saved %s\n', name);
  close(fig);
end
