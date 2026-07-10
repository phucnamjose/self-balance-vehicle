% LOOP_RATE_PLOTS  Figures for docs/theory/loop-rates.md.
% Control-system frequency stack. Base Octave only (no control package).
% Outputs: loop-rates-map/cascade/delay/antialias.png -> docs/theory/
% Run:  cd experiments/loop_rates && octave --eval loop_rate_plots

function loop_rate_plots()

  here   = fileparts(mfilename('fullpath'));
  outdir = fullfile(here, '..', '..', 'docs', 'theory');
  warning('off', 'Octave:gnuplot-graphics');
  graphics_toolkit('gnuplot');

  % ---- key frequencies [rad/s] -------------------------------------------
  w_vel   = 2;        % outer velocity/position loop crossover (target)
  w_wpole = 5.3;      % open-loop wheel pole 1/tau
  w_unst  = 20.57;    % unstable body pole (RHP)
  w_bal   = 32;       % balance loop crossover (target, above the unstable pole)
  w_wheel = 25.8;     % inner wheel-speed loop crossover (tuned)
  w_wheel2= 49;       % wheel loop retuned tighter (tau_cl = tau/10)
  w_dlpf  = 2*pi*44;  % IMU DLPF cutoff ~ 276
  w_nyq   = pi/(1/200);   % control Nyquist ~ 628
  w_fs    = 2*pi*200; % control sample rate ~ 1257
  w_pwm   = 2*pi*1e4; % PWM carrier ~ 62800

  col_plant = [0.40 0.40 0.40];
  col_unst  = [0.85 0.10 0.10];
  col_loop  = [0.00 0.45 0.74];
  col_dig   = [0.20 0.55 0.20];
  col_carr  = [0.55 0.30 0.65];

  % ======================================================================
  % Fig 1: the system frequency map
  % ======================================================================
  fig = figure('visible','off','position',[100 100 1000 500]);
  hold on;
  set(gca, 'xscale', 'log');
  xlim([1 1e5]); ylim([0.2 4.9]);

  % lane guide lines (lane 4 = plant ... lane 1 = carrier)
  lanes = {'carrier', 'digital / sensor', 'loop crossovers', 'plant dynamics'};
  for L = 1:4
    plot([1 1e5], [L L], '-', 'color', [.88 .88 .88], 'linewidth', 8);
  end
  set(gca, 'ytick', 1:4, 'yticklabel', lanes);

  % control band: up to ~50 rad/s
  patch([1 50 50 1], [0.2 0.2 4.9 4.9], [0.90 0.95 0.90], ...
        'edgecolor', 'none', 'facealpha', 0.5);

  function stem_label(w, y, txt, c, dy)
    plot(w, y, 'o', 'markersize', 9, 'markerfacecolor', c, 'markeredgecolor', c);
    text(w, y+dy, txt, 'rotation', 22, 'color', c, ...
         'horizontalalignment', 'left', 'fontsize', 9);
  end

  stem_label(w_wpole, 4, ' wheel pole 5.3',        col_plant, 0.12);
  stem_label(w_unst,  4, ' UNSTABLE 20.6',         col_unst,  0.12);
  stem_label(w_vel,   3, ' velocity 2',            col_loop,  0.12);
  stem_label(w_wheel, 3, ' wheel 25.8',            col_loop,  0.12);
  stem_label(w_bal,   3, ' balance 32 (squeeze)',  col_loop, -0.30);
  stem_label(w_dlpf,  2, ' DLPF 44 Hz',            col_dig,   0.12);
  stem_label(w_nyq,   2, ' Nyquist 100 Hz',        col_dig,  -0.40);
  stem_label(w_fs,    2, ' f_s 200 Hz',            col_dig,   0.12);
  stem_label(w_pwm,   1, ' PWM 10 kHz',            col_carr,  0.12);

  % oversampling gap: fastest loop vs Nyquist (rule 3)
  yb = 2.40;
  plot([w_wheel w_nyq], [yb yb], 'k-', 'linewidth', 1);
  plot(w_wheel, yb, 'k<', w_nyq, yb, 'k>', 'markersize', 6);
  text(sqrt(w_wheel*w_nyq), yb+0.14, '24x gap  (rule 3: sample >> crossover)', ...
       'horizontalalignment', 'center', 'fontsize', 9);

  xlabel('frequency  \omega  [rad/s]   (Hz = \omega / 2\pi)');
  title('System frequency map: plant, loops, sampling, carrier (log axis)');
  grid on; set(gca, 'xgrid', 'on', 'ygrid', 'off');
  print(fig, fullfile(outdir, 'loop-rates-map.png'), '-dpng', '-r110');

  % ======================================================================
  % Fig 2: nested closed-loop bandwidths (separation of timescales)
  % ======================================================================
  w = logspace(-0.5, 2.2, 3000);
  T = @(wc) 1 ./ sqrt(1 + (w./wc).^2);        % first-order closed loop |T(jw)|

  fig = figure('visible','off','position',[100 100 900 520]);
  hold on; grid on;
  semilogx(w, 20*log10(T(w_vel)),   'linewidth', 2, 'color', col_dig);
  semilogx(w, 20*log10(T(w_bal)),   'linewidth', 2.4, 'color', col_loop);
  semilogx(w, 20*log10(T(w_wheel)), 'linewidth', 1.2, 'color', 'k');
  semilogx(w, 20*log10(T(w_wheel2)),'--','linewidth', 2, 'color', [0.85 0.33 0.10]);
  plot([w_unst w_unst], [-40 3], 'r:', 'linewidth', 1.5);
  plot([w(1) w(end)], [-3 -3], 'k:');          % -3 dB bandwidth line
  text(w_unst*0.97, -34, 'unstable pole 20.6', 'color', col_unst, ...
       'fontsize', 9, 'rotation', 90);
  text(w_vel*0.8, 2, 'velocity', 'color', col_dig, 'fontsize', 9);
  text(w_bal*1.15, 2.2, 'balance', 'color', col_loop, 'fontsize', 9);
  text(w_wheel*0.42, 2.2, 'wheel', 'color', col_plant, 'fontsize', 9);
  ylim([-40 4]); xlim([w(1) w(end)]);
  set(gca,'xscale','log');
  xlabel('\omega [rad/s]'); ylabel('closed-loop |T| [dB]');
  legend('velocity (2)', 'balance (32)', 'wheel-speed (25.8)', ...
         'wheel retuned (49)', 'location', 'southwest');
  title('Cascade bandwidths: clean gap velocity to balance, but balance and wheel overlap');
  print(fig, fullfile(outdir, 'loop-rates-cascade.png'), '-dpng', '-r110');

  % ======================================================================
  % Fig 3: sample-and-hold phase penalty vs frequency
  % ======================================================================
  w = logspace(0, 2, 2000);
  Td = @(fs) 1.5 / fs;                          % ZOH + compute delay
  ph = @(fs) -w * Td(fs) * 180/pi;              % phase lag [deg]

  fig = figure('visible','off','position',[100 100 900 500]);
  hold on; grid on;
  semilogx(w, ph(100), 'linewidth', 2, 'color', [0.85 0.10 0.10]);
  semilogx(w, ph(200), 'linewidth', 2, 'color', [0.00 0.45 0.74]);
  semilogx(w, ph(400), 'linewidth', 2, 'color', [0.20 0.55 0.20]);
  plot([w_wheel w_wheel], [-60 0], 'k:', 'linewidth', 1.2);
  text(w_wheel*1.03, -3, 'loop crossovers ~25-32', 'fontsize', 9);
  % mark the phase eaten at the crossover for each rate
  for fs = [100 200 400]
    p = -w_wheel*Td(fs)*180/pi;
    plot(w_wheel, p, 'ko', 'markersize', 6, 'markerfacecolor', 'k');
    text(w_wheel*1.05, p-2.5, sprintf('%.0f Hz: %.0f\\circ', fs, p), 'fontsize', 9);
  end
  ylim([-60 2]); xlim([w(1) w(end)]);
  set(gca,'xscale','log');
  xlabel('\omega [rad/s]'); ylabel('phase lag from delay  -\omega T_d  [deg]');
  legend('100 Hz  (T_d=15 ms)', '200 Hz  (T_d=7.5 ms)', '400 Hz  (T_d=3.75 ms)', ...
         'location', 'southwest');
  title('Why 200 Hz: sample-and-hold phase penalty at the crossover');
  print(fig, fullfile(outdir, 'loop-rates-delay.png'), '-dpng', '-r110');

  % ======================================================================
  % Fig 4: IMU anti-aliasing (DLPF vs Nyquist)
  % ======================================================================
  f  = logspace(0, log10(500), 3000);          % Hz
  fc = 44;                                       % DLPF cutoff
  H  = 1 ./ sqrt(1 + (f./fc).^4);               % ~2nd-order rolloff (illustrative)

  fig = figure('visible','off','position',[100 100 900 520]);
  hold on; grid on;
  % shaded signal band (balance loop uses up to ~5 Hz)
  patch([1 5 5 1], [-60 -60 5 5], [0.90 0.95 0.90], 'edgecolor','none','facealpha',0.6);
  semilogx(f, 20*log10(H), 'linewidth', 2.5, 'color', col_dig);
  plot([100 100], [-60 5], 'k--', 'linewidth', 1.2);   % Nyquist
  plot([200 200], [-60 5], 'k:',  'linewidth', 1.2);   % sample rate
  plot([fc fc],   [-60 5], 'color', col_dig, 'linestyle','--');
  % 150 Hz vibration above Nyquist (folds to 50 Hz)
  fv = 150; Hv = 20*log10(1/sqrt(1+(fv/fc)^4));
  plot(fv, Hv, 'o', 'markersize', 8, 'markerfacecolor', col_unst, 'markeredgecolor', col_unst);
  text(fv*0.92, Hv+7, sprintf('150 Hz vibration cut %.0f dB', Hv), ...
       'color', col_unst, 'fontsize', 9, 'horizontalalignment', 'right');
  text(fv*0.92, Hv+3.5, 'before it folds to 50 Hz', ...
       'color', col_unst, 'fontsize', 9, 'horizontalalignment', 'right');
  text(2.0, -52, 'signal band', 'fontsize', 9, 'color', [0.1 0.4 0.1]);
  text(46, 2, 'DLPF 44 Hz', 'fontsize', 9, 'color', col_dig, 'rotation', 90);
  text(103, -20, 'Nyquist 100 Hz', 'fontsize', 9, 'rotation', 90);
  text(205, -20, 'f_s 200 Hz', 'fontsize', 9, 'rotation', 90);
  ylim([-60 5]); xlim([f(1) f(end)]);
  set(gca,'xscale','log');
  xlabel('frequency [Hz]'); ylabel('IMU response |H| [dB]');
  title('Anti-aliasing: the DLPF attenuates above-Nyquist content before it folds in');
  print(fig, fullfile(outdir, 'loop-rates-antialias.png'), '-dpng', '-r110');

  printf('saved 4 figures into %s\n', outdir);

end
