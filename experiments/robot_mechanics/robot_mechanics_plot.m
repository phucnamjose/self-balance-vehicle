% ROBOT_MECHANICS_PLOT  Dimensioned robot illustration for docs/hardware/robot-mechanics.md.
% Side view (X front, Z up) and front view (Y left, Z up); lengths in mm, to scale.
% Output: robot-mechanics.png -> docs/hardware/
% Run:  cd experiments/robot_mechanics && octave --eval robot_mechanics_plot

function robot_mechanics_plot()

  here   = fileparts(mfilename('fullpath'));
  outdir = fullfile(here, '..', '..', 'docs', 'hardware');
  warning('off', 'Octave:gnuplot-graphics');
  graphics_toolkit('gnuplot');

  % ---- dimensions (mm) ----------------------------------------------------
  D  = 65;    r = D/2;         % wheel diameter / radius
  W  = 190;                    % track (centre-to-centre)
  H  = 120;                    % overall height
  l  = 52;                     % CoM height above axle (measure!)

  % ---- colours ------------------------------------------------------------
  col_body  = [0.86 0.91 0.98];
  col_edge  = [0.20 0.24 0.30];
  col_tyre  = [0.30 0.32 0.36];
  col_hub   = [0.72 0.74 0.78];
  col_com   = [0.85 0.33 0.10];
  col_imu   = [1.00 0.95 0.80];
  col_dim   = [0.35 0.35 0.35];
  col_grnd  = [0.55 0.55 0.55];
  ax_x = [0.80 0.15 0.15];     % X red
  ax_y = [0.15 0.55 0.20];     % Y green
  ax_z = [0.15 0.35 0.80];     % Z blue

  fig = figure('visible','off','position',[100 100 1200 480]);

  % SIDE VIEW (left side; front = +X right)
  subplot(1,2,1); hold on; axis equal; axis off;

  bw = 74;                                   % body depth drawn in side view
  % ground
  plot([-95 120], [0 0], 'color', col_grnd, 'linewidth', 2);
  hatch_ground(-95, 120, 0, col_grnd);

  % body (rounded slab from just above ground to H)
  roundrect(-bw/2, 12, bw, H-12, 9, col_body, col_edge, 1.6);

  % wheel (both wheels overlap in side view) - tyre + hub
  circle(0, r, r,      col_tyre, col_edge, 1.6);
  circle(0, r, r*0.42, col_hub,  col_edge, 1.2);
  plot(0, r, 'k.', 'markersize', 10);        % axle centre

  % CoM arm l : dashed line from the axle up to the CoM
  plot([0 0], [r r+l], '--', 'color', col_com, 'linewidth', 1.2);
  text(-10, r+l-6, 'l', 'color', col_com, 'fontweight', 'bold', ...
       'horizontalalignment', 'center', 'fontangle', 'italic');

  % IMU board (on the body, above the wheel)
  roundrect(-14, 66, 28, 12, 2, col_imu, col_edge, 1.2);
  text(0, 72, 'MPU6050', 'fontsize', 7.5, 'horizontalalignment', 'center');

  % CoM (above the wheel, on the body centre-line)
  plot(0, r+l, 'o', 'markersize', 12, 'markerfacecolor', col_com, ...
       'markeredgecolor', col_edge, 'linewidth', 1.2);
  text(9, r+l, 'CoM', 'color', col_com, 'fontweight', 'bold');

  % body-axis triad (X front, Z up; Y out of page)
  o = [62; 74];
  arrow(o, o+[24;0], ax_x, 2.2); text(o(1)+27, o(2),    'X (front)', 'color', ax_x);
  arrow(o, o+[0;24], ax_z, 2.2); text(o(1)-4,  o(2)+30, 'Z (up)',    'color', ax_z);
  ocirc(o(1), o(2), 3.2, ax_y);              % Y out of page
  text(o(1)+6, o(2)-11, 'Y (out of page)', 'color', ax_y, 'fontsize', 8);

  % dimensions
  dim_v( 96, 0, H,   sprintf('H = %d mm', H), col_dim);        % height (right)
  dim_v(-52, 0, 2*r, sprintf('Ø %d mm', D),   col_dim);        % wheel dia (left)
  text(-70, r+22, 'wheel', 'color', col_dim, 'fontsize', 8, ...
       'horizontalalignment', 'center');
  % axle-height note with a short leader from the axle centre
  plot([0 46], [r 46], ':', 'color', col_dim, 'linewidth', 0.9);
  text(48, 47, 'axle @ 32.5 mm', 'color', col_dim, 'fontsize', 8);

  % front-direction hint
  arrow([-6; H+16], [34; H+16], col_edge, 2);
  text(37, H+16, 'drives forward (+X)', 'fontsize', 8);

  title('Side view  (robot''s left side)', 'fontsize', 11);
  xlim([-95 122]); ylim([-18 H+30]);

  % FRONT VIEW (+X toward viewer; +Y = screen left)
  subplot(1,2,2); hold on; axis equal; axis off;

  % ground
  plot([-140 140], [0 0], 'color', col_grnd, 'linewidth', 2);
  hatch_ground(-140, 140, 0, col_grnd);

  % body
  bwf = 150;
  roundrect(-bwf/2, 30, bwf, H-30, 9, col_body, col_edge, 1.6);

  % two wheels seen edge-on (thin tyres) at +/- W/2
  tw = 22;                                    % tyre thickness drawn
  for xc = [-W/2 W/2]
    roundrect(xc-tw/2, 0, tw, 2*r, 4, col_tyre, col_edge, 1.6);
    plot(xc, r, 'k.', 'markersize', 9);      % axle centre
  end

  % IMU triad (Y left, Z up, X out of page)
  o = [0; r+l];
  plot(0, r+l, 'o', 'markersize', 9, 'markerfacecolor', col_com, ...
       'markeredgecolor', col_edge);
  arrow(o, o+[-30;0], ax_y, 2.2); text(o(1)-64, o(2), 'Y (left)', 'color', ax_y);
  arrow(o, o+[0;28],  ax_z, 2.2); text(o(1)+4,  o(2)+32, 'Z (up)', 'color', ax_z);
  ocirc(o(1)+34, o(2), 3.2, ax_x);
  text(o(1)+40, o(2), 'X (out)', 'color', ax_x, 'fontsize', 8);

  % dimensions: track (centre-to-centre) and wheel diameter
  dim_h(-W/2, W/2, -10, sprintf('W = %d mm  (track)', W), col_dim);
  dim_v(W/2+tw/2+16, 0, 2*r, sprintf('Ø %d', D), col_dim);

  title('Front view  (looking from the front, +X toward you)', 'fontsize', 11);
  xlim([-150 165]); ylim([-30 H+30]);

  print(fig, fullfile(outdir, 'robot-mechanics.png'), '-dpng', '-r120');
  printf('saved robot-mechanics.png into %s\n', outdir);

end

% rounded rectangle
function roundrect(x, y, w, h, rad, fc, ec, lw)
  rectangle('Position', [x y w h], 'Curvature', [2*rad/w 2*rad/h], ...
            'FaceColor', fc, 'EdgeColor', ec, 'linewidth', lw);
end

% filled circle
function circle(cx, cy, rad, fc, ec, lw)
  a = linspace(0, 2*pi, 80);
  patch(cx+rad*cos(a), cy+rad*sin(a), fc, 'edgecolor', ec, 'linewidth', lw);
end

% "out of page" symbol
function ocirc(cx, cy, rad, col)
  a = linspace(0, 2*pi, 40);
  plot(cx+rad*cos(a), cy+rad*sin(a), 'color', col, 'linewidth', 1.6);
  plot(cx, cy, '.', 'color', col, 'markersize', 12);
end

% vertical dimension line
function dim_v(x, y0, y1, lbl, col)
  plot([x x], [y0 y1], 'color', col, 'linewidth', 1);
  tick(x, y0, 'h', col); tick(x, y1, 'h', col);
  arrowhead([x; y0], [0; 1],  col);
  arrowhead([x; y1], [0; -1], col);
  text(x+3, (y0+y1)/2, lbl, 'color', col, 'rotation', 90, ...
       'horizontalalignment', 'center', 'fontsize', 9);
end

% horizontal dimension line
function dim_h(x0, x1, y, lbl, col)
  plot([x0 x1], [y y], 'color', col, 'linewidth', 1);
  tick(x0, y, 'v', col); tick(x1, y, 'v', col);
  arrowhead([x0; y], [1; 0],  col);
  arrowhead([x1; y], [-1; 0], col);
  text((x0+x1)/2, y-9, lbl, 'color', col, ...
       'horizontalalignment', 'center', 'fontsize', 9);
end

function tick(x, y, dir, col)
  if dir == 'h', plot([x-3 x+3], [y y], 'color', col, 'linewidth', 1);
  else,          plot([x x], [y-3 y+3], 'color', col, 'linewidth', 1); end
end

% arrow (line + head)
function arrow(p0, p1, col, lw)
  p0 = p0(:); p1 = p1(:);
  plot([p0(1) p1(1)], [p0(2) p1(2)], 'color', col, 'linewidth', lw);
  d = p1 - p0; if norm(d) < 1e-9, return; end
  arrowhead(p1, d/norm(d), col);
end

function arrowhead(tip, u, col)
  u = u(:)/norm(u); n = [-u(2); u(1)];
  hl = 6; hw = 3;
  b = tip - hl*u;
  patch([tip(1) b(1)+hw*n(1) b(1)-hw*n(1)], ...
        [tip(2) b(2)+hw*n(2) b(2)-hw*n(2)], col, 'edgecolor', col);
end

% ground hatching
function hatch_ground(x0, x1, y, col)
  for xx = x0:12:x1
    plot([xx xx-7], [y y-7], 'color', col, 'linewidth', 0.8);
  end
end
