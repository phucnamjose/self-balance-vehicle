% COM_HEIGHT_STUDY  How the unstable pole (=> how hard to balance) depends on the
% CoM height l. Uses the real linearize.m on the actual plant, so it stays
% consistent with the rest of the sim.
%
% The unstable pole p_u sets the time-to-double td = ln2/p_u: the smaller p_u
% (the longer td), the more time the lagging inner loop has to react, i.e. the
% EASIER to balance. Two physically distinct ways to "raise the CoM":
%
%   A) Shift mass within the CURRENT chassis (body height fixed at 0.0875 m, so
%      the body inertia I is fixed; l can only range 0..body_height).
%   B) Build a TALLER robot with a roughly uniform body (body_height = 2*l, so
%      I = (1/12) m (2l)^2 grows with height). This is the realistic way to get
%      a large l.
%
% Reference: the inner wheel loop closes to ~37 ms; you want td comfortably
% larger than that. Rough guide: td < ~40 ms marginal, ~70 ms comfortable,
% >100 ms easy.

clear; clc;
here = fileparts(mfilename('fullpath')); addpath(here);
p0 = params();

  function [pu, td] = pole_of(p)
    A  = linearize(p);
    ev = eig(A);
    pu = max(real(ev));        % unstable (largest real part) pole [rad/s]
    td = log(2) / pu;          % time-to-double [s]
  end

% ---- Scenario A: fixed current body, just move the CoM -----------------------
printf('== Scenario A: fixed chassis (body_height = %.4f m, I fixed) ==\n', p0.body_height);
printf('   l (m)   pole (rad/s)   t_double (ms)\n');
lA = 0.02:0.01:p0.body_height;
puA = zeros(size(lA)); tdA = zeros(size(lA));
for k = 1:numel(lA)
  p = p0; p.l = lA(k);                       % I_body unchanged (chassis unchanged)
  [puA(k), tdA(k)] = pole_of(p);
  printf('   %.3f      %6.2f         %6.1f\n', lA(k), puA(k), 1e3*tdA(k));
end

% ---- Scenario B: taller uniform robot (body_height = 2*l) --------------------
printf('\n== Scenario B: taller uniform body (body_height = 2*l, I grows) ==\n');
printf('   l (m)   height*(m)  pole (rad/s)  t_double (ms)\n');
lB = 0.05:0.025:0.30;
puB = zeros(size(lB)); tdB = zeros(size(lB));
for k = 1:numel(lB)
  p = p0;
  p.l           = lB(k);
  p.body_height = 2*lB(k);                   % uniform body, CoM at mid-height
  p.I_body      = (1/12) * p.m_body * p.body_height^2;
  [puB(k), tdB(k)] = pole_of(p);
  Htot = p.r_wheel + p.body_height;          % axle-to-top + wheel radius ~ overall height
  printf('   %.3f     %.3f       %6.2f        %6.1f\n', lB(k), Htot, puB(k), 1e3*tdB(k));
end

% current operating point
[pu_now, td_now] = pole_of(p0);
printf('\nCurrent: l = %.3f m -> pole %.2f rad/s, t_double %.1f ms\n', ...
       p0.l, pu_now, 1e3*td_now);

% ---- Plot --------------------------------------------------------------------
warning('off','Octave:gnuplot-graphics'); graphics_toolkit('gnuplot');
fig = figure('visible','off','position',[100 100 820 520]);

subplot(2,1,1);
plot(lB, puB, '-o','linewidth',2,'color',[0.00 0.45 0.74]); hold on;
plot(lA, puA, '--','linewidth',1.5,'color',[0.5 0.5 0.5]);
plot(p0.l, pu_now, 'rp','markersize',12,'markerfacecolor','r'); grid on;
ylabel('unstable pole [rad/s]');
legend('B: taller uniform body','A: fixed chassis','current (l=0.05)');
title('How the unstable pole depends on CoM height l (lower = easier)');

subplot(2,1,2);
plot(lB, 1e3*tdB, '-o','linewidth',2,'color',[0.20 0.55 0.20]); hold on;
plot(p0.l, 1e3*td_now, 'rp','markersize',12,'markerfacecolor','r');
xl = [min(lB) max(lB)];
plot(xl, [37 37], '--','color',[0.85 0.33 0.10]); text(0.20, 40, '37 ms (inner-loop response)','color',[0.85 0.33 0.10]);
plot(xl, [70 70], ':','color',[0.3 0.3 0.3]);       text(0.20, 66, '70 ms (comfortable)','color',[0.3 0.3 0.3]);
grid on;
xlabel('CoM height l above axle [m]'); ylabel('time-to-double [ms]');

outfile = fullfile(here,'com_height_study.png');
print(fig,'-dpng','-r100',outfile);
printf('Saved plot -> %s\n', outfile);
