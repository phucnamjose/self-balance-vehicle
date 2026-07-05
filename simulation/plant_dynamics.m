function dx = plant_dynamics(x, F, p)
% PLANT_DYNAMICS  Nonlinear state derivative of the inverted-pendulum cart.
%
%   dx = plant_dynamics(x, F, p)
%
%   x  = [pos; vel; theta; omega]   (see params.m for the convention)
%   F  = horizontal force applied to the cart by the wheels [N]
%   p  = parameter struct from params()
%   dx = time derivative of x = [vel; accel; omega; alpha]
%
% Equations of motion (derived symbolically in eom_derive.m). With
%   M = effective cart mass, m = body mass, l = CoM height, I = body inertia:
%
%   (M+m) * vel_dot  +  m*l*cos(th) * om_dot  =  F - b*vel + m*l*sin(th)*om^2
%   m*l*cos(th)*vel_dot + (I + m*l^2)*om_dot  =  m*g*l*sin(th) - c*om
%
% That is a 2x2 linear system  Mq * [vel_dot; om_dot] = rhs  which we solve
% at every call. The m*g*l*sin(th) term is what makes the upright position
% unstable: a small +theta produces a +om_dot that tips it further.

  vel = x(2);
  th  = x(3);
  om  = x(4);

  M = p.M; m = p.m_body; l = p.l; I = p.I_body;
  g = p.g; b = p.b; c = p.c;

  s  = sin(th);
  ct = cos(th);

  % Mass (inertia) matrix and right-hand side.
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
