function [A, B] = linearize(p)
% LINEARIZE  x_dot ~= A*x + B*F about upright equilibrium (small x, F=0).
%   [A, B] = linearize(p)     (p optional; defaults to params())
% A, B via central differences on plant_dynamics.m. No outputs -> prints poles + controllability.

  if nargin < 1
    p = params();
  end

  x0 = [0; 0; 0; 0];     % upright equilibrium state
  F0 = 0;                % equilibrium input (no force)
  n  = numel(x0);

  % --- A = d(f)/d(x) by central differences ---
  A = zeros(n, n);
  for i = 1:n
    dx = zeros(n, 1);
    h  = 1e-6;
    dx(i) = h;
    fp = plant_dynamics(x0 + dx, F0, p);
    fm = plant_dynamics(x0 - dx, F0, p);
    A(:, i) = (fp - fm) / (2*h);
  end

  % --- B = d(f)/d(F) by central differences ---
  hF = 1e-6;
  fp = plant_dynamics(x0, F0 + hF, p);
  fm = plant_dynamics(x0, F0 - hF, p);
  B  = (fp - fm) / (2*hF);

  % --- Analysis (only when run for its own sake) ---
  if nargout == 0
    disp('A ='); disp(A);
    disp('B ='); disp(B);

    ev = eig(A);
    printf('Open-loop eigenvalues (poles):\n');
    for i = 1:numel(ev)
      printf('   %+9.4f %+9.4fi\n', real(ev(i)), imag(ev(i)));
    end
    if any(real(ev) > 1e-6)
      printf('-> at least one pole has POSITIVE real part: upright is UNSTABLE.\n');
      printf('   (a small tilt grows ~ e^(%.2f t): time-to-double ~ %.3f s)\n', ...
             max(real(ev)), log(2)/max(real(ev)));
    else
      printf('-> all poles stable (unexpected for an inverted pendulum).\n');
    end

    % Controllability of single force input
    Co = [B, A*B, A*A*B, A*A*A*B];
    r  = rank(Co);
    printf('Controllability matrix rank = %d of %d', r, n);
    if r == n
      printf('  -> CONTROLLABLE (a controller can place all poles).\n');
    else
      printf('  -> NOT fully controllable.\n');
    end
  end
end
