% INVERTED_PENDULUM_PLOTS  Figures for docs/theory/inverted-pendulum.md.
% Cart-pendulum model: geometry, open-loop fall, poles, phase plane, linearization.
% Reuses ../../simulation (params, plant_dynamics, linearize). Base Octave only.
% Outputs: ip-schematic/fall/poles/phase/linearization.png -> docs/theory/
% Run:  cd experiments/inverted_pendulum && octave --eval inverted_pendulum_plots

function inverted_pendulum_plots()

  here   = fileparts(mfilename('fullpath'));
  outdir = fullfile(here, '..', '..', 'docs', 'theory');
  addpath(fullfile(here, '..', '..', 'simulation'));   % params, plant_dynamics, linearize
  warning('off', 'Octave:gnuplot-graphics');
  graphics_toolkit('gnuplot');

  p = params();

  % ---- shared colours ----------------------------------------------------
  col_true = [0.15 0.15 0.15];
  col_inv  = [0.85 0.33 0.10];
  col_hang = [0.20 0.55 0.20];
  col_lin  = [0.00 0.45 0.74];
  col_grey = [0.55 0.55 0.55];

  % ======================================================================
  % Fig 1: cart-pendulum coordinates and forces
  % ======================================================================
  th = deg2rad(25);                              % draw a 25 deg tilt for clarity
  l  = 1.0;                                       % CoM arm (drawing units)
  cart = [0 0];                                   % pivot / axle at origin
  com  = cart + l*[sin(th) cos(th)];             % CoM up the body

  fig = figure('visible','off','position',[100 100 780 700]);
  hold on; axis equal; axis off;

  % ground + wheels (the "cart")
  plot([-1.5 1.5], [0 0], 'color', col_grey, 'linewidth', 1.5);
  text(1.15, -0.13, 'ground');
  rectangle('Position', [-0.5 -0.05 1.0 0.12], 'Curvature', 0.4, ...
            'FaceColor', [0.90 0.92 0.98], 'EdgeColor', col_true, 'linewidth', 1.2);
  for xc = [-0.32 0.32]
    rr = 0.11;
    aa = linspace(0, 2*pi, 40);
    patch(xc+rr*cos(aa), 0.0+rr*sin(aa), [0.80 0.82 0.88], ...
          'edgecolor', col_true, 'linewidth', 1.2);
    plot(xc, 0.0, 'k.', 'markersize', 8);
  end

  % body: a slab from the axle up to the CoM and beyond, rotated by theta
  R = [cos(th) -sin(th); sin(th) cos(th)];
  body = [ -0.05 -0.05 0.05 0.05; 0 1.35 1.35 0 ];
  b = R*body;
  patch(b(1,:), b(2,:), [0.96 0.94 0.90], 'edgecolor', col_true, 'linewidth', 1.4);

  % upright reference through the pivot
  plot([0 0], [0 1.5], '--', 'color', col_grey, 'linewidth', 1.1);
  text(-0.30, 1.5, 'upright');

  % pivot + CoM
  plot(0, 0, 'k.', 'markersize', 18);
  text(0.06, -0.16, 'pivot / axle');
  plot(com(1), com(2), 'o', 'markersize', 10, ...
       'markerfacecolor', col_inv, 'markeredgecolor', col_true);
  text(com(1)+0.08, com(2), 'CoM (mass m)');

  % arm length l along the body
  midb = 0.5*com;
  text(midb(1)-0.16, midb(2)+0.04, 'l');

  % theta arc between upright and the body
  aa = linspace(pi/2, pi/2 - th, 40);
  plot(0.55*cos(aa), 0.55*sin(aa), 'k', 'linewidth', 1.2);
  text(0.14, 0.63, '\theta');

  % gravity at the CoM (down) and its tipping component
  arrow(com, com + [0 -0.55], col_inv, 2.5);
  text(com(1)+0.05, com(2)-0.42, 'm g  (gravity)');

  % control force on the cart (horizontal), = wheels pushing the base
  arrow(cart + [-0.5 0.32], cart + [-0.02 0.32], col_lin, 2.5);
  text(-0.62, 0.45, 'F  (drive the base)');

  % cart position coordinate
  arrow([0 -0.30], [0.5 -0.30], col_grey, 1.5);
  text(0.52, -0.30, 'pos (x)');

  title('Inverted pendulum on a cart:  tilt \theta from upright, drive force F');
  xlim([-1.5 1.5]); ylim([-0.5 2.05]);
  print(fig, fullfile(outdir, 'ip-schematic.png'), '-dpng', '-r110');

  % ======================================================================
  % Fig 2: open-loop (F=0) — upright runs away, hanging oscillates
  % ======================================================================
  th0 = deg2rad(3);

  [ti, xi] = simulate([0;0; th0;      0], 1.4, p);   % inverted
  [thg, xg] = simulate([0;0; pi+th0;  0], 3.0, p);   % hanging

  theta_inv  = xi(:,3) * 180/pi;                      % deg from upright
  theta_hang = (xg(:,3) - pi) * 180/pi;              % deg from hanging equilibrium

  % unstable-pole exponential envelope
  [A, ~]  = linearize(p);
  lam     = max(real(eig(A)));
  t2      = log(2)/lam;                               % time-to-double
  env     = 3 * exp(lam * ti);

  fig = figure('visible','off','position',[100 100 860 640]);

  subplot(2,1,1); hold on; grid on;
  plot(ti, theta_inv, 'linewidth', 2.2, 'color', col_inv);
  plot(ti, env, '--', 'linewidth', 1.4, 'color', col_grey);
  plot([t2 t2], [0 6], 'k:', 'linewidth', 1.2);
  text(t2+0.01, 5.2, sprintf('time-to-double = %.0f ms', t2*1000));
  ylim([0 60]); xlim([0 1.4]);
  ylabel('tilt from upright [deg]');
  legend('nonlinear fall (F=0)', ...
         sprintf('e^{+%.1f t} envelope', lam), 'location', 'northwest');
  title('Upright is UNSTABLE: a 3\circ nudge runs away exponentially');

  subplot(2,1,2); hold on; grid on;
  plot(thg, theta_hang, 'linewidth', 2.2, 'color', col_hang);
  plot([thg(1) thg(end)], [0 0], 'color', col_grey);
  xlim([0 3]);
  xlabel('time [s]'); ylabel('angle from hanging [deg]');
  legend('nonlinear swing (F=0)', 'location', 'northeast');
  title('Same pendulum hung DOWN is STABLE: a 3\circ nudge just oscillates');
  print(fig, fullfile(outdir, 'ip-fall.png'), '-dpng', '-r110');

  % ======================================================================
  % Fig 3: open-loop poles — inverted (upright) vs hanging (theta = pi)
  % ======================================================================
  [Ai, ~] = linearize(p);
  Ah = jacobian_about([0;0;pi;0], p);
  ei = eig(Ai);
  eh = eig(Ah);

  fig = figure('visible','off','position',[100 100 760 620]);
  hold on; grid on; axis equal;

  % axes
  plot([-14 14], [0 0], 'color', col_grey, 'linewidth', 1.0);
  plot([0 0], [-14 14], 'color', col_grey, 'linewidth', 1.0);
  % shade the unstable (right) half-plane
  patch([0 14 14 0], [-14 -14 14 14], [1.0 0.90 0.86], ...
        'edgecolor', 'none', 'facealpha', 0.6);
  text(7.5, 12.5, 'unstable half-plane (Re > 0)', 'color', col_inv);

  % hanging poles: a lightly damped pair near the imaginary axis
  hh = plot(real(eh), imag(eh), 's', 'markersize', 11, ...
       'markerfacecolor', col_hang, 'markeredgecolor', col_true, 'linewidth', 1);
  % inverted poles: a real pair (one in the RHP)
  ii = plot(real(ei), imag(ei), 'o', 'markersize', 11, ...
       'markerfacecolor', col_inv, 'markeredgecolor', col_true, 'linewidth', 1);

  % annotate the culprit
  [~, k] = max(real(ei));
  plot(real(ei(k)), imag(ei(k)), 'o', 'markersize', 18, ...
       'markeredgecolor', col_inv, 'linewidth', 2);
  text(real(ei(k))-0.3, 2.6, sprintf('+%.1f rad/s', real(ei(k))), ...
       'color', col_inv, 'horizontalalignment', 'center');
  text(real(ei(k))-0.3, 1.4, '(tips over)', ...
       'color', col_inv, 'horizontalalignment', 'center');

  xlabel('Re\{s\}  [rad/s]  (growth / decay rate)');
  ylabel('Im\{s\}  [rad/s]  (oscillation)');
  legend([ii hh], {'inverted (upright)', 'hanging (down)'}, 'location', 'southwest');
  title('Open-loop poles: the inverted pendulum has a pole in the right half-plane');
  xlim([-14 14]); ylim([-14 14]);
  print(fig, fullfile(outdir, 'ip-poles.png'), '-dpng', '-r110');

  % ======================================================================
  % Fig 4: phase portrait of the (undamped, undriven) pendulum angle
  % ======================================================================
  % theta'' = (g/l) sin(theta); upright (0,0) saddle, hanging (+-pi,0) centres
  w2 = p.g / p.l;                                    % (rad/s)^2
  wn = sqrt(w2);

  [TH, W] = meshgrid(linspace(-pi-0.6, pi+0.6, 26), ...
                     linspace(-2.2*wn, 2.2*wn, 26));
  dTH = W;
  dW  = w2 * sin(TH);
  M   = hypot(dTH, dW); M(M==0) = 1;

  fig = figure('visible','off','position',[100 100 860 620]);
  hold on; grid on;

  % direction field (normalised arrows)
  quiver(TH, W, dTH./M, dW./M, 0.5, 'color', col_grey, 'maxheadsize', 0.4);

  % energy level-set trajectories
  thc = linspace(-pi-0.6, pi+0.6, 600);
  for E = [-0.9 -0.5 0.0 0.6 1.4 2.4] * w2
    val = 2*(E + w2*cos(thc));                        % w^2 = 2(E + (g/l)cos th)
    val(val < 0) = NaN;
    wc = sqrt(val);
    plot(thc,  wc, 'color', col_lin, 'linewidth', 1.0);
    plot(thc, -wc, 'color', col_lin, 'linewidth', 1.0);
  end

  % separatrix through the saddle (E = w2): highlighted
  val = 2*w2*(1 + cos(thc)); val(val<0)=NaN;
  plot(thc,  sqrt(val), 'color', col_inv, 'linewidth', 2.2);
  plot(thc, -sqrt(val), 'color', col_inv, 'linewidth', 2.2);

  % equilibria
  plot(0, 0, 'o', 'markersize', 12, 'markerfacecolor', col_inv, ...
       'markeredgecolor', col_true, 'linewidth', 1);
  plot([-pi pi], [0 0], 's', 'markersize', 12, 'markerfacecolor', col_hang, ...
       'markeredgecolor', col_true, 'linewidth', 1);
  text(0.12, 0.8, 'upright: SADDLE (unstable)', 'color', col_inv);
  text(pi-2.2, -1.2, 'hanging: CENTRE (stable)', 'color', col_hang);

  xlabel('tilt \theta [rad]   (0 = upright, \pm\pi = hanging)');
  ylabel('tilt rate \omega [rad/s]');
  title('Phase portrait: upright is a saddle - almost every path leaves it');
  xlim([-pi-0.6 pi+0.6]); ylim([-2.2*wn 2.2*wn]);
  set(gca, 'xtick', [-pi -pi/2 0 pi/2 pi], ...
           'xticklabel', {'-\pi','-\pi/2','0','\pi/2','\pi'});
  print(fig, fullfile(outdir, 'ip-phase.png'), '-dpng', '-r110');

  % ======================================================================
  % Fig 5: sin(theta) ~ theta; linear model valid at small tilt
  % ======================================================================
  fig = figure('visible','off','position',[100 100 900 420]);

  subplot(1,2,1); hold on; grid on;
  thd = linspace(0, 60, 400);              % degrees
  thr = deg2rad(thd);
  plot(thd, sin(thr),  'linewidth', 2.2, 'color', col_true);
  plot(thd, thr,       '--', 'linewidth', 2.0, 'color', col_lin);
  % 5% error crossing
  err = abs(thr - sin(thr)) ./ abs(sin(thr));
  k5  = find(err >= 0.05, 1);
  plot([thd(k5) thd(k5)], [0 1.1], 'k:', 'linewidth', 1.2);
  text(thd(k5)+1, 0.30, '5% error');
  text(thd(k5)+1, 0.20, sprintf('at %.0f deg', thd(k5)));
  xlabel('tilt \theta [deg]'); ylabel('value');
  legend('sin\theta (true)', '\theta (linear)', 'location', 'northwest');
  title('Small-angle: sin\theta \approx \theta');
  ylim([0 1.15]);

  subplot(1,2,2); hold on; grid on;
  [tn, xn] = simulate([0;0; deg2rad(2); 0], 0.6, p);   % nonlinear fall
  theta_nl = xn(:,3)*180/pi;
  % linear model x_dot = A x from the same start
  x = [0;0; deg2rad(2); 0]; dt = tn(2)-tn(1);
  theta_ln = zeros(size(tn));
  for k = 1:numel(tn)
    theta_ln(k) = x(3)*180/pi;
    x = x + dt * (Ai * x);                              % Euler on the linear model
  end
  plot(tn, theta_nl, 'linewidth', 2.4, 'color', col_true);
  plot(tn, theta_ln, '--', 'linewidth', 2.0, 'color', col_lin);
  xlabel('time [s]'); ylabel('tilt [deg]');
  legend('nonlinear plant', 'linear model  x'' = A x', 'location', 'northwest');
  title('Linear \approx nonlinear while \theta stays small');
  print(fig, fullfile(outdir, 'ip-linearization.png'), '-dpng', '-r110');

  printf('saved 5 figures into %s\n', outdir);
  printf('unstable pole lambda = %+.3f rad/s -> time-to-double %.1f ms\n', ...
         lam, t2*1000);

end

% RK4 integration, F = 0
function [t, X] = simulate(x0, T, p)
  dt = 1e-3;
  t  = (0:dt:T)';
  N  = numel(t);
  X  = zeros(N, 4);
  x  = x0(:);
  for k = 1:N
    X(k,:) = x';
    k1 = plant_dynamics(x,            0, p);
    k2 = plant_dynamics(x + 0.5*dt*k1, 0, p);
    k3 = plant_dynamics(x + 0.5*dt*k2, 0, p);
    k4 = plant_dynamics(x + dt*k3,     0, p);
    x  = x + (dt/6)*(k1 + 2*k2 + 2*k3 + k4);
  end
end

% Jacobian d(f)/d(x) about an equilibrium (central diff)
function A = jacobian_about(x0, p)
  n = numel(x0); A = zeros(n);
  h = 1e-6;
  for i = 1:n
    dx = zeros(n,1); dx(i) = h;
    A(:,i) = (plant_dynamics(x0+dx,0,p) - plant_dynamics(x0-dx,0,p)) / (2*h);
  end
end

% manual 2D arrow
function arrow(p0, p1, col, lw)
  p0 = p0(:); p1 = p1(:);
  d  = p1 - p0; Ld = norm(d);
  plot([p0(1) p1(1)], [p0(2) p1(2)], 'color', col, 'linewidth', lw);
  if Ld < 1e-9, return; end
  u  = d/Ld; n = [-u(2); u(1)];
  hl = 0.09; hw = 0.04;
  base = p1 - hl*u;
  h1 = base + hw*n; h2 = base - hw*n;
  patch([p1(1) h1(1) h2(1)], [p1(2) h1(2) h2(2)], col, 'edgecolor', col);
end
