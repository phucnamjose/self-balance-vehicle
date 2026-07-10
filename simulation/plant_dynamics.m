function dx = plant_dynamics(x, F, p)
% PLANT_DYNAMICS  Nonlinear inverted-pendulum cart: dx = plant_dynamics(x, F, p).
% x=[pos;vel;theta;omega], F=cart force [N], dx=[vel;accel;omega;alpha].
% Mq*[vel_dot;om_dot]=rhs (see eom_derive.m). m*g*l*sin(th) makes upright unstable.

  vel = x(2);
  th  = x(3);
  om  = x(4);

  M = p.M; m = p.m_body; l = p.l; I = p.I_body;
  g = p.g; b = p.b; c = p.c;

  s  = sin(th);
  ct = cos(th);

  % Mass matrix and RHS
  Mq  = [ M + m,        m*l*ct;
          m*l*ct,       I + m*l^2 ];
  rhs = [ F - b*vel + m*l*s*om^2;
          m*g*l*s - c*om ];

  acc = Mq \ rhs;          % [vel_dot; om_dot]

  dx = [ vel;              % d(pos)/dt
         acc(1);           % d(vel)/dt   (cart acceleration)
         om;               % d(theta)/dt
         acc(2) ];         % d(omega)/dt (angular acceleration)
end
