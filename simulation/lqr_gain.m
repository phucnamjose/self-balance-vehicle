function K = lqr_gain(A, B, Q, R)
% LQR_GAIN  Continuous-time LQR gain, self-contained (no control package).
%
%   K = lqr_gain(A, B, Q, R)
%
% Returns the state-feedback gain K so that  u = -K*x  minimizes the cost
%
%       J = integral( x'*Q*x + u'*R*u ) dt
%
% This is the Linear-Quadratic Regulator. Q penalizes state error (how much we
% care about each state staying near zero); R penalizes control effort (how much
% we want to avoid big commands). Bigger Q -> snappier; bigger R -> gentler.
%
% The optimal K = R^-1 B' X, where X solves the continuous algebraic Riccati
% equation (CARE):   A'X + XA - XBR^-1B'X + Q = 0.
%
% We solve the CARE WITHOUT the control package, via the Hamiltonian matrix
%
%       H = [ A,           -B R^-1 B' ;
%            -Q,           -A'        ]
%
% whose stable eigenvectors (negative real part) span the solution subspace.
% Partition those eigenvectors [U1; U2]; then X = U2 * U1^-1.
%
% (If you have the control package, `K = lqr(A,B,Q,R)` does the same thing.)

  n = size(A, 1);
  Rinv = inv(R);

  H = [ A,        -B*Rinv*B';
       -Q,        -A'        ];

  [V, D] = eig(H);
  ev = diag(D);

  % Pick the n eigenvectors whose eigenvalues have negative real part.
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
