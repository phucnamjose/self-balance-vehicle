% ANGLE_ESTIMATION_PLOTS  Figures for docs/theory/angle-estimation.md.
%
% Illustrates how the body tilt angle theta is estimated by fusing the two
% imperfect IMU signals (accelerometer angle + gyro rate). Base Octave only
% (no toolboxes): every transfer function is evaluated on a grid by hand and
% the filters are run as plain difference equations.
%
% Generates five PNGs into docs/theory/ (referenced by angle-estimation.md):
%   angle-est-geometry.png     - accelerometer tilt geometry (gravity projection)
%   angle-est-sensors.png      - the two failure modes: accel noise/spikes, gyro drift
%   angle-est-comp-bode.png    - complementary filter: LP(accel)+HP(gyro) = 1
%   angle-est-fusion.png       - fused estimate vs accel-only / gyro-only / truth
%   angle-est-kalman.png       - Kalman filter: tracks theta AND learns the gyro bias
%
% Run:  cd experiments/angle_estimation && octave --eval angle_estimation_plots

function angle_estimation_plots()

  here   = fileparts(mfilename('fullpath'));
  outdir = fullfile(here, '..', '..', 'docs', 'theory');
  warning('off', 'Octave:gnuplot-graphics');
  graphics_toolkit('gnuplot');

  % ---- shared colours -----------------------------------------------------
  col_true = [0.15 0.15 0.15];       % truth (near-black)
  col_acc  = [0.85 0.33 0.10];       % accelerometer (orange)
  col_gyro = [0.20 0.55 0.20];       % gyroscope (green)
  col_fuse = [0.00 0.45 0.74];       % fused estimate (blue)
  col_grey = [0.55 0.55 0.55];

  dt = 1/200;                         % 200 Hz control tick (firmware CONTROL_HZ)

  % ======================================================================
  % Fig 1: accelerometer tilt geometry
  % ======================================================================
  % A body tilted by theta about the wheel axle. At rest the accelerometer
  % measures the SPECIFIC FORCE, which points opposite gravity (+g "up"). In the
  % body frame its components are a_z = g cos(theta), a_x = g sin(theta), so
  % theta = atan2(a_x, a_z).
  th = deg2rad(28);                   % draw a 28 deg tilt for clarity
  R  = [cos(th) -sin(th); sin(th) cos(th)];
  L  = 1.0;                            % accel vector length (= 1 g)

  fig = figure('visible','off','position',[100 100 760 680]);
  hold on; axis equal; axis off;

  % ground + pivot
  plot([-1.7 1.7], [0 0], 'color', col_grey, 'linewidth', 1.5);
  plot(0, 0, 'k.', 'markersize', 16);
  text(1.42, -0.12, 'ground');

  % body: a slab from the pivot up, rotated by theta
  body = [ -0.16 -0.16 0.16 0.16; 0 1.3 1.3 0 ];
  b = R*body;
  patch(b(1,:), b(2,:), [0.90 0.92 0.98], 'edgecolor', col_true, 'linewidth', 1.5);

  % vertical (upright) reference through the pivot
  plot([0 0], [0 1.45], '--', 'color', col_grey, 'linewidth', 1.2);
  text(-0.28, 1.45, 'upright (z_{world})');

  % CoM / IMU location
  com = R*[0; 0.72];
  plot(com(1), com(2), 'k.', 'markersize', 15);
  text(com(1)-0.52, com(2)-0.02, 'CoM / IMU');

  % body axes at the CoM: z_b up along the slab, x_b across it
  zb = R*[0; 1];  xb = R*[1; 0];
  arrow(com, com + 0.9*L*zb, col_grey, 1.5);
  arrow(com, com + 0.55*L*xb, col_grey, 1.5);
  text_at(com + 0.95*L*zb, 'z_b', col_grey);
  text_at(com + 0.60*L*xb, 'x_b', col_grey);

  % measured specific force a: points straight UP (world), magnitude 1 g
  tip = com + [0; L];
  arrow(com, tip, col_acc, 2.5);
  text(tip(1)+0.03, tip(2)+0.02, 'a  (accel reads +1g, opposite gravity)');

  % projections of a onto the body axes -> the two measured components
  az = (L*zb'*[0;1]) * zb;            % component along z_b
  ax = (L*xb'*[0;1]) * xb;            % component along x_b
  plot([com(1)+az(1) tip(1)], [com(2)+az(2) tip(2)], ':', 'color', col_fuse, 'linewidth', 1.3);
  plot([com(1)+ax(1) tip(1)], [com(2)+ax(2) tip(2)], ':', 'color', col_fuse, 'linewidth', 1.3);
  arrow(com, com + az, col_fuse, 2);
  arrow(com, com + ax, col_fuse, 2);
  text_at(com + 0.5*az + [-0.16;0.02], 'a_z = g cos\theta', col_fuse);
  text_at(com + 0.5*ax + [0.02;-0.12], 'a_x = g sin\theta', col_fuse);

  % theta arc between a (world up) and z_b (body up)
  aa = linspace(pi/2, pi/2 - th, 40);
  rr = 0.45;
  plot(com(1)+rr*cos(aa), com(2)+rr*sin(aa), 'k', 'linewidth', 1.2);
  text(com(1)-0.02, com(2)+rr+0.06, '\theta');

  title('Accelerometer sees gravity:  \theta_{acc} = atan2(a_x, a_z)');
  xlim([-1.7 1.7]); ylim([-0.25 2.0]);
  print(fig, fullfile(outdir, 'angle-est-geometry.png'), '-dpng', '-r110');

  % ======================================================================
  % Simulate a common truth for the time-domain figures (Figs 2, 4, 5)
  % ======================================================================
  T = 12; t = 0:dt:T; N = numel(t);
  randn('seed', 7);

  % true tilt: a smooth, realistic balancing wobble (a few small sinusoids)
  theta_true = 3*sin(2*pi*0.25*t) + 1.5*sin(2*pi*0.6*t + 1.0);
  omega_true = gradient(theta_true, dt);           % true tilt rate [deg/s]

  % --- gyro: clean rate, but a constant BIAS + a little noise -------------
  gyro_bias  = 1.2;                                % deg/s residual bias
  gyro_noise = 0.30;                               % deg/s
  gyro_meas  = omega_true + gyro_bias + gyro_noise*randn(1,N);

  % --- accel angle: no drift, but NOISY and corrupted by body linear accel.
  % A horizontal shove at t~4 s makes the accelerometer briefly report a large
  % false tilt (it cannot tell gravity from acceleration). The gyro is immune.
  accel_noise = 1.2;                               % deg
  bump = 9*exp(-((t-4).^2)/0.05);                  % motion-induced accel error
  theta_acc = theta_true + bump + accel_noise*randn(1,N);

  % --- integrate the gyro alone (drifts) ---------------------------------
  theta_gyro = zeros(1,N);
  for k = 2:N
    theta_gyro(k) = theta_gyro(k-1) + gyro_meas(k)*dt;
  end

  % ======================================================================
  % Fig 2: why neither sensor works alone
  % ======================================================================
  fig = figure('visible','off','position',[100 100 860 640]);

  subplot(2,1,1); hold on; grid on;
  plot(t, theta_true, 'linewidth', 2.5, 'color', col_true);
  plot(t, theta_acc, 'linewidth', 0.7, 'color', col_acc);
  ylabel('tilt [deg]');
  legend('true \theta', 'accel-only \theta_{acc}', 'location', 'northeast');
  title('Accelerometer: no drift, but noisy and fooled by motion (t \approx 4 s shove)');
  ylim([-8 12]);

  subplot(2,1,2); hold on; grid on;
  plot(t, theta_true, 'linewidth', 2.5, 'color', col_true);
  plot(t, theta_gyro, 'linewidth', 2, 'color', col_gyro);
  xlabel('time [s]'); ylabel('tilt [deg]');
  legend('true \theta', 'gyro-only \int\omega dt', 'location', 'northwest');
  title(sprintf('Gyroscope: smooth, but a %.1f deg/s bias makes it drift away', gyro_bias));
  print(fig, fullfile(outdir, 'angle-est-sensors.png'), '-dpng', '-r110');

  % ======================================================================
  % Fig 3: the complementary filter in the frequency domain
  % ======================================================================
  % theta = a*(theta + gyro*dt) + (1-a)*theta_acc  is a first-order blend with
  % time constant tau = a*dt/(1-a): a LOW-PASS on the accel angle plus a
  % complementary HIGH-PASS on the gyro angle. They sum to 1 at every frequency.
  alpha = 0.98;                       % illustrative (firmware/sim uses ~0.995)
  tau   = alpha*dt/(1-alpha);         % crossover time constant [s]
  fc    = 1/(2*pi*tau);               % crossover frequency [Hz]

  f  = logspace(-2, 2, 2000);
  w  = 2*pi*f;
  LP = 1./(1 + 1i*w*tau);             % accel path  (trust at LOW freq)
  HP = (1i*w*tau)./(1 + 1i*w*tau);    % gyro path   (trust at HIGH freq)

  fig = figure('visible','off','position',[100 100 820 480]);
  hold on; grid on;
  semilogx(f, 20*log10(abs(LP)), 'linewidth', 2.2, 'color', col_acc);
  semilogx(f, 20*log10(abs(HP)), 'linewidth', 2.2, 'color', col_gyro);
  semilogx(f, 20*log10(abs(LP+HP)), '--', 'linewidth', 2, 'color', col_fuse);
  plot([fc fc], [-40 3], 'k:', 'linewidth', 1.2);
  text(fc*1.15, -30, sprintf('f_c = %.2f Hz', fc));
  text(fc*1.15, -34, sprintf('\\tau = %.2f s', tau));
  set(gca,'xscale','log');
  ylim([-40 3]); xlim([f(1) f(end)]);
  xlabel('frequency [Hz]'); ylabel('magnitude [dB]');
  legend('accel path: low-pass  1/(\tau s+1)', ...
         'gyro path: high-pass  \tau s/(\tau s+1)', ...
         'sum (all-pass, 0 dB)', 'location', 'east');
  title('Complementary filter: accel below f_c, gyro above f_c, sum \equiv 1');
  print(fig, fullfile(outdir, 'angle-est-comp-bode.png'), '-dpng', '-r110');

  % ======================================================================
  % Fig 4: the fused estimate (complementary filter)
  % ======================================================================
  theta_comp = zeros(1,N); theta_comp(1) = theta_acc(1);
  for k = 2:N
    theta_comp(k) = alpha*(theta_comp(k-1) + gyro_meas(k)*dt) ...
                    + (1-alpha)*theta_acc(k);
  end

  fig = figure('visible','off','position',[100 100 860 480]);
  hold on; grid on;
  plot(t, theta_acc,  'linewidth', 0.6, 'color', col_acc);
  plot(t, theta_gyro, 'linewidth', 1.5, 'color', col_gyro);
  plot(t, theta_true, 'linewidth', 2.8, 'color', col_true);
  plot(t, theta_comp, 'linewidth', 1.8, 'color', col_fuse);
  xlabel('time [s]'); ylabel('tilt [deg]'); ylim([-8 12]);
  legend('accel-only (noisy/spiky)', 'gyro-only (drifts)', ...
         'true \theta', 'complementary fusion', 'location', 'northeast');
  title(sprintf('Fusion: smooth like the gyro, drift-free like the accel (\\alpha=%.2f)', alpha));
  print(fig, fullfile(outdir, 'angle-est-fusion.png'), '-dpng', '-r110');

  % ======================================================================
  % Fig 5: Kalman filter - tracks theta AND estimates the gyro bias
  % ======================================================================
  % 2-state KF: x = [theta; bias].  Predict with the gyro, correct with accel.
  %   theta[k] = theta[k-1] + (gyro - bias)*dt ;  bias[k] = bias[k-1]
  Q = [1e-3 0; 0 1e-5];               % process noise (angle wander, slow bias)
  Rm = 4.0;                           % accel-angle measurement variance [deg^2]
  A  = [1 -dt; 0 1];
  B  = [dt; 0];
  H  = [1 0];
  xk = [theta_acc(1); 0]; P = eye(2);
  theta_kf = zeros(1,N); bias_kf = zeros(1,N);
  for k = 1:N
    xk = A*xk + B*gyro_meas(k);       % predict using the gyro
    P  = A*P*A' + Q;
    S  = H*P*H' + Rm;                 % correct using the accel angle
    Kk = (P*H')/S;
    xk = xk + Kk*(theta_acc(k) - H*xk);
    P  = (eye(2) - Kk*H)*P;
    theta_kf(k) = xk(1); bias_kf(k) = xk(2);
  end

  fig = figure('visible','off','position',[100 100 860 640]);

  subplot(2,1,1); hold on; grid on;
  plot(t, theta_true, 'linewidth', 2.8, 'color', col_true);
  plot(t, theta_comp, 'linewidth', 1.4, 'color', col_fuse);
  plot(t, theta_kf,   'linewidth', 1.6, 'color', col_acc);
  ylabel('tilt [deg]'); ylim([-8 12]);
  legend('true \theta', 'complementary', 'Kalman', 'location', 'northeast');
  title('Kalman filter vs complementary: similar tracking, but the KF is self-tuning');

  subplot(2,1,2); hold on; grid on;
  plot([t(1) t(end)], [gyro_bias gyro_bias], '--', 'linewidth', 2, 'color', col_grey);
  plot(t, bias_kf, 'linewidth', 2, 'color', col_gyro);
  xlabel('time [s]'); ylabel('gyro bias [deg/s]');
  legend('true bias', 'KF bias estimate', 'location', 'southeast');
  title('The KF also identifies the gyro bias online (a state the complementary filter cannot)');
  print(fig, fullfile(outdir, 'angle-est-kalman.png'), '-dpng', '-r110');

  printf('saved 5 figures into %s\n', outdir);
  printf('complementary: alpha=%.3f -> tau=%.3f s, f_c=%.3f Hz\n', alpha, tau, fc);

end

% ---- clean 2D arrow (line + manual arrowhead), no quiver autoscale --------
function arrow(p0, p1, col, lw)
  p0 = p0(:); p1 = p1(:);
  d  = p1 - p0; Ld = norm(d);
  plot([p0(1) p1(1)], [p0(2) p1(2)], 'color', col, 'linewidth', lw);
  if Ld < 1e-9, return; end
  u  = d/Ld;                       % direction
  n  = [-u(2); u(1)];              % normal
  hl = 0.10; hw = 0.045;           % head length / half-width
  base = p1 - hl*u;
  h1 = base + hw*n; h2 = base - hw*n;
  patch([p1(1) h1(1) h2(1)], [p1(2) h1(2) h2(2)], col, 'edgecolor', col);
end

function text_at(p, s, col)
  text(p(1), p(2), s, 'color', col);
end
