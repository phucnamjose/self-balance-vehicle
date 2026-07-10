% PI_TUNING_ANALYSIS  Nyquist + Bode margins for the wheel-speed PI loop.
% Hand-evaluated L(jw) on a grid (no control package).
% Plant G=K/(tau*s+1), PI C=Kp+Ki/s (IMC, Ti=tau), plus ZOH delay and meas LPF.
%
%   pi_tuning_analysis                      % defaults K=34, tau=0.19
%   pi_tuning_analysis(K, tau)
%   pi_tuning_analysis(K, tau, tau_cl)
%   pi_tuning_analysis(K, tau, tau_cl, tag)  % custom PNG suffix
%
% Outputs: pi-tuning-nyquist/bode[(-tag)].png in this folder.

function pi_tuning_analysis(K, tau, tau_cl, tag)

  % ---- arguments (fall back to identified defaults) ---------------------
  K0 = 34.0; tau0 = 0.19;                 % identified defaults
  if nargin < 1 || isempty(K),      K   = K0;   end   % rad/s per unit duty
  if nargin < 2 || isempty(tau),    tau = tau0; end   % s
  if nargin < 3 || isempty(tau_cl), tau_cl = tau/5; end % desired closed-loop [s]

  % accept string args from CLI
  if ischar(K),      K      = str2double(K);      end
  if ischar(tau),    tau    = str2double(tau);    end
  if ischar(tau_cl), tau_cl = str2double(tau_cl); end

  % PNG suffix: base names only for exact defaults
  is_default = (K == K0) && (tau == tau0) && (abs(tau_cl - tau/5) <= eps(tau));
  if nargin >= 4 && ~isempty(tag)
    suffix = ['-' tag];
  elseif is_default
    suffix = '';
  else
    suffix = sprintf('-K%g_tau%g_tcl%g', K, tau, tau_cl);
  end

  % PI: pole-zero cancellation / IMC (Ti = tau)
  Ti = tau;
  Kp = tau / (K * tau_cl);
  Ki = 1   / (K * tau_cl);
  printf('plant: K = %.3f rad/s/duty, tau = %.4f s\n', K, tau);
  printf('IMC tuning: tau_cl = %.3f s -> Kp = %.4f, Ki = %.4f (Ti = %.3f)\n', ...
         tau_cl, Kp, Ki, Ti);

  % ---- practical lags in the discrete loop ----------------------------
  dt    = 0.002;                  % 500 Hz control tick
  Td    = 1.5*dt;                 % ZOH (~0.5 tick) + compute (~1 tick)
  tau_f = 0.02;                   % encoder-rate LPF (WHEEL_PI_TAU_F)

  % ---- open-loop frequency response L(jw) -----------------------------
  w  = logspace(-1, 3, 6000);
  s  = 1i*w;
  C  = Kp + Ki./s;
  G  = K ./ (tau*s + 1);
  Dl = exp(-s*Td);
  Hf = 1 ./ (tau_f*s + 1);
  L  = C .* G .* Dl .* Hf;

  mag   = abs(L);
  phdeg = angle(L)*180/pi;
  phu   = unwrap(angle(L))*180/pi;

  % ---- phase margin: at the gain crossover |L| = 1 --------------------
  k  = find(mag(1:end-1) >= 1 & mag(2:end) < 1, 1, 'first');
  wc = interp1(mag(k:k+1), w(k:k+1), 1);
  pc = interp1(w(k:k+1), phdeg(k:k+1), wc);
  PM = 180 + pc;
  printf('gain crossover wc = %.2f rad/s, phase margin PM = %.1f deg\n', wc, PM);

  % ---- gain margin: where phase = -180 deg ----------------------------
  GM_dB = Inf; wp = NaN;
  j = find(phu(1:end-1) >= -180 & phu(2:end) < -180, 1, 'first');
  if isempty(j)
    printf('gain margin: infinite (phase never reaches -180 deg)\n');
  else
    wp = interp1(phu(j:j+1), w(j:j+1), -180);
    GM_dB = -20*log10(interp1(w(j:j+1), mag(j:j+1), wp));
    printf('phase crossover wp = %.2f rad/s, gain margin GM = %.1f dB\n', wp, GM_dB);
  end

  here = fileparts(mfilename('fullpath'));
  nyq_png  = fullfile(here, ['pi-tuning-nyquist' suffix '.png']);
  bode_png = fullfile(here, ['pi-tuning-bode'    suffix '.png']);
  warning('off', 'Octave:gnuplot-graphics');
  graphics_toolkit('gnuplot');

  % ======================= Nyquist =====================================
  fig1 = figure('visible', 'off', 'position', [100 100 720 680]);
  hold on; grid on; axis equal;
  th = linspace(0, 2*pi, 200);
  plot(cos(th), sin(th), ':', 'color', [.6 .6 .6]);        % unit circle
  plot(real(L),  imag(L),  'linewidth', 2, 'color', [0.00 0.45 0.74]);
  plot(real(L), -imag(L), '--', 'linewidth', 1, 'color', [0.00 0.45 0.74]); % mirror
  plot(-1, 0, 'rx', 'markersize', 12, 'linewidth', 2);
  text(-1.02, 0.14, '-1', 'color', 'r');
  xlabel('Re\{L(j\omega)\}'); ylabel('Im\{L(j\omega)\}');
  title(sprintf('Nyquist of wheel-speed PI loop  (PM = %.0f deg, \\tau_{cl}=%.3f s)', PM, tau_cl));
  xlim([-2 1]); ylim([-2.5 0.5]);
  print(fig1, nyq_png, '-dpng', '-r110');

  % ======================= Bode ========================================
  fig2 = figure('visible', 'off', 'position', [100 100 760 680]);

  subplot(2,1,1); hold on; grid on;
  semilogx(w, 20*log10(mag), 'linewidth', 2, 'color', [0.00 0.45 0.74]);
  plot([w(1) w(end)], [0 0], 'k:');                         % 0 dB
  plot([wc wc], ylim, 'r--');                               % gain crossover
  xlabel('\omega [rad/s]'); ylabel('|L| [dB]');
  title(sprintf('Bode of L(j\\omega):  wc=%.1f rad/s, PM=%.0f deg, GM=%.1f dB', wc, PM, GM_dB));
  set(gca, 'xscale', 'log'); xlim([w(1) w(end)]);

  subplot(2,1,2); hold on; grid on;
  semilogx(w, phu, 'linewidth', 2, 'color', [0.00 0.45 0.74]);
  plot([w(1) w(end)], [-180 -180], 'k:');                   % -180 deg
  plot([wc wc], ylim, 'r--');                               % gain crossover
  if ~isnan(wp), plot([wp wp], ylim, 'm--'); end            % phase crossover
  xlabel('\omega [rad/s]'); ylabel('phase [deg]');
  set(gca, 'xscale', 'log'); xlim([w(1) w(end)]);
  print(fig2, bode_png, '-dpng', '-r110');

  printf('saved -> %s , %s\n', nyq_png, bode_png);

end
