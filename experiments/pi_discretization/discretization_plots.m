% DISCRETIZATION_PLOTS  Figures for docs/theory/pi-discretization.md.
% Continuous -> discrete PI conversion; hand-evaluated TFs. Base Octave only.
% Outputs: pi-disc-sampling/splane/bode/step.png -> docs/theory/
% Run:  cd experiments/pi_discretization && octave --eval discretization_plots

function discretization_plots()

  here   = fileparts(mfilename('fullpath'));
  outdir = fullfile(here, '..', '..', 'docs', 'theory');
  warning('off', 'Octave:gnuplot-graphics');
  graphics_toolkit('gnuplot');

  % ---- shared plant / controller / timing (see pi-tuning.md) --------------
  K = 34; tau = 0.19; tau_cl = tau/5;
  Kp = tau/(K*tau_cl);            % 0.147
  Ki = 1/(K*tau_cl);             % 0.774
  dt = 1/500;                     % 500 Hz tick
  wN = pi/dt;                     % Nyquist frequency [rad/s] ~ 1571
  wc = 23.8;                      % gain crossover of the loop [rad/s]

  col_c = [0.55 0.55 0.55];
  col_f = [0.85 0.33 0.10];
  col_b = [0.20 0.55 0.20];
  col_t = [0.00 0.45 0.74];

  % ======================================================================
  % Fig 1: sampling + zero-order hold
  % ======================================================================
  f  = 20;                                    % a 20 Hz signal
  t  = linspace(0, 0.05, 1000);
  x  = sin(2*pi*f*t);
  tk = 0:dt:0.05;
  xk = sin(2*pi*f*tk);

  fig = figure('visible','off','position',[100 100 820 420]);
  hold on; grid on;
  plot(t*1000,  x, 'linewidth', 2, 'color', col_c);
  stairs(tk*1000, xk, 'linewidth', 2, 'color', col_t);
  plot(tk*1000, xk, 'o', 'markersize', 7, 'markerfacecolor', col_f, ...
       'markeredgecolor', col_f);
  xlabel('time [ms]'); ylabel('signal');
  ylim([-1.3 1.3]);
  legend('continuous e(t)', 'held output (ZOH)', 'samples e[k]', ...
         'location', 'southeast');
  title('Sampling + zero-order hold at 500 Hz (\Delta t = 2 ms)');
  print(fig, fullfile(outdir, 'pi-disc-sampling.png'), '-dpng', '-r110');

  % ======================================================================
  % Fig 2: image of the s-plane imaginary axis under each rule (z-plane)
  % ======================================================================
  th = linspace(0, 2*pi, 400);
  fig = figure('visible','off','position',[100 100 640 620]);
  hold on; grid on; axis equal;
  % unit circle = discrete stability boundary
  plot(cos(th), sin(th), 'k', 'linewidth', 1.5);
  % forward Euler: jw -> Re(z) = 1
  plot([1 1], [-1.6 1.6], 'linewidth', 2, 'color', col_f);
  % backward Euler: jw -> |z - 0.5| = 0.5
  plot(0.5 + 0.5*cos(th), 0.5*sin(th), 'linewidth', 2, 'color', col_b);
  % Tustin: jw -> unit circle
  plot(cos(th), sin(th), '--', 'linewidth', 2, 'color', col_t);
  plot(1, 0, 'k.', 'markersize', 14);                 % z = 1  <-  s = 0
  text(1.03, 0.12, 'z=1 (s=0)');
  xlabel('Re\{z\}'); ylabel('Im\{z\}');
  legend('unit circle |z|=1', 'forward: j\omega \rightarrow Re=1', ...
         'backward: j\omega \rightarrow |z-0.5|=0.5', ...
         'Tustin: j\omega \rightarrow |z|=1', 'location', 'southoutside');
  title('Where the stable-region boundary (j\omega axis) lands in the z-plane');
  xlim([-1.6 2.1]); ylim([-1.7 1.7]);
  print(fig, fullfile(outdir, 'pi-disc-splane.png'), '-dpng', '-r110');

  % ======================================================================
  % Fig 3: integrator Ki/s vs 3 discrete rules
  % ======================================================================
  w  = logspace(0, log10(wN), 3000);
  s  = 1i*w;
  z  = exp(1i*w*dt);
  Ic = Ki./s;                        % continuous integrator Ki/s
  If = Ki*dt./(z - 1);               % forward  Euler
  Ib = Ki*dt.*z./(z - 1);            % backward Euler
  It = (Ki*dt/2).*(z + 1)./(z - 1);  % Tustin

  fig = figure('visible','off','position',[100 100 820 700]);

  subplot(2,1,1); hold on; grid on;
  semilogx(w, 20*log10(abs(Ic)), 'linewidth', 3.5, 'color', col_c);
  semilogx(w, 20*log10(abs(If)), 'linewidth', 1.8, 'color', col_f);
  semilogx(w, 20*log10(abs(Ib)), 'linewidth', 1.8, 'color', col_b);
  semilogx(w, 20*log10(abs(It)), '--', 'linewidth', 1.8, 'color', col_t);
  yl = [-70 5];
  plot([wc wc], yl, 'k:',  'linewidth', 1.2);
  plot([wN wN], yl, 'k--', 'linewidth', 1.2);
  ylim(yl); ylabel('|K_i / s|  [dB]'); set(gca,'xscale','log'); xlim([w(1) w(end)]);
  legend('continuous  K_i/s', 'forward Euler', 'backward Euler', 'Tustin', ...
         'location', 'northeast');
  title('Bode of the integrator: continuous vs discrete (500 Hz)');

  subplot(2,1,2); hold on; grid on;
  semilogx(w, angle(Ic)*180/pi, 'linewidth', 3.5, 'color', col_c);
  semilogx(w, angle(If)*180/pi, 'linewidth', 1.8, 'color', col_f);
  semilogx(w, angle(Ib)*180/pi, 'linewidth', 1.8, 'color', col_b);
  semilogx(w, angle(It)*180/pi, '--', 'linewidth', 1.8, 'color', col_t);
  yl = [-185 5];
  plot([w(1) w(end)], [-90 -90], 'color', [.8 .8 .8]);
  plot([wc wc], yl, 'k:',  'linewidth', 1.2);
  plot([wN wN], yl, 'k--', 'linewidth', 1.2);
  text(wc*1.08, -150, '\omega_c'); text(wN*0.4, -150, 'Nyquist');
  ylim(yl);
  xlabel('\omega [rad/s]'); ylabel('phase [deg]');
  set(gca,'xscale','log'); xlim([w(1) w(end)]);
  print(fig, fullfile(outdir, 'pi-disc-bode.png'), '-dpng', '-r110');

  % ======================================================================
  % Fig 4: closed-loop step, continuous ideal vs discrete (ZOH) loop
  % ======================================================================
  tc    = linspace(0, 0.30, 1000);
  wcont = 10*(1 - exp(-tc/tau_cl));           % continuous first-order closed loop

  ad = exp(-dt/tau); bd = K*(1 - ad);          % exact ZOH plant
  N  = round(0.30/dt); td = (0:N-1)*dt;
  w_ = 0; I = 0; wsp = 10; W = zeros(1,N);
  for k = 1:N
    e  = wsp - w_;
    u  = Kp*e + I;
    us = max(-1, min(1, u));
    if u == us, I = I + Ki*e*dt; end
    w_ = ad*w_ + bd*us;  W(k) = w_;
  end

  fig = figure('visible','off','position',[100 100 820 420]);
  hold on; grid on;
  plot(tc*1000, wcont, 'linewidth', 3.5, 'color', col_c);
  stairs(td*1000, W, 'linewidth', 1.8, 'color', col_f);
  plot([tau_cl tau_cl]*1000, [0 10], 'k:');
  text(tau_cl*1000+2, 5.5, '\tau_{cl}');
  xlabel('time [ms]'); ylabel('\omega [rad/s]');
  ylim([0 11]);
  legend('continuous closed loop  1/(\tau_{cl}s+1)', ...
         'discrete PI (forward Euler) + ZOH plant', 'location', 'southeast');
  title('Closed-loop step: discretization preserves the behaviour');
  print(fig, fullfile(outdir, 'pi-disc-step.png'), '-dpng', '-r110');

  printf('saved 4 figures into %s\n', outdir);

end
