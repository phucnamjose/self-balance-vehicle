function K = lqr_gain(A, B, Q, R)
% LQR_GAIN  Continuous-time LQR via Hamiltonian eigendecomposition (no control pkg).
%   K = lqr_gain(A, B, Q, R)   minimizes J = integral(x'Qx + u'Ru)dt; u = -K*x.
% K = R^-1 B'X where X solves CARE: A'X + XA - XBR^-1B'X + Q = 0.
% H = [A, -BR^-1B'; -Q, -A']; stable eigenvectors give X = U2*U1^-1.

  n = size(A, 1);
  Rinv = inv(R);

  H = [ A,        -B*Rinv*B';
       -Q,        -A'        ];

  [V, D] = eig(H);
  ev = diag(D);

  % Stable eigenvectors only
  stable = find(real(ev) < 0);
  if numel(stable) ~= n
    error('lqr_gain: expected %d stable eigenvalues, found %d (check Q,R,(A,B))', ...
          n, numel(stable));
  end
  Vs  = V(:, stable);
  U1  = Vs(1:n,     :);
  U2  = Vs(n+1:2*n, :);

  X = real(U2 / U1);              % solve CARE
  K = Rinv * (B' * X);            % optimal feedback gain
end
