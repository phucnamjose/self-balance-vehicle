% EOM_DERIVE  Derive the inverted-pendulum-cart equations of motion symbolically.
%
% This is a LEARNING / VERIFICATION script. It uses the Lagrangian method to
% derive the same equations that plant_dynamics.m implements numerically, prints
% them, and then checks that plant_dynamics.m agrees at a sample state.
%
% Needs the symbolic package:  pkg install -forge symbolic   (once), then it is
% loaded below.
%
% Generalized coordinates:  q  = [pos; theta]      (cart position, body tilt)
% Generalized velocities:   qd = [vel; omega]
%
% Geometry (theta measured from the UPWARD vertical, 0 = upright):
%   cart at (pos, 0);  body CoM at (pos + l*sin(theta),  l*cos(theta)).

clear; clc;
pkg load symbolic;
% Substituting numeric parameters into symbolic expressions triggers a noisy
% "floating-point values to sym" warning; it is harmless for our cross-check.
warning('off', 'all');

syms M m l I g b c F real          % parameters + input force
syms pos vel th om real            % state: position, velocity, tilt, tilt-rate

% ---- Energies -----------------------------------------------------------
% CoM velocity components (d/dt of the CoM position, with d(pos)/dt = vel,
% d(theta)/dt = om):
xc_dot =  vel + l*cos(th)*om;      % horizontal CoM velocity
yc_dot =       -l*sin(th)*om;      % vertical   CoM velocity

T = 1/2*M*vel^2 ...                       % cart translation
  + 1/2*m*(xc_dot^2 + yc_dot^2) ...       % body translation
  + 1/2*I*om^2;                           % body rotation
V = m*g*l*cos(th);                        % gravity PE (max upright -> unstable)

T = simplify(T);
disp('Kinetic energy T ='); pretty(T);
disp('Potential energy V ='); pretty(V);

% ---- Manipulator form  Mq*qdd = rhs ------------------------------------
% Rather than carry symbolic time-functions, we assemble the well-known
% manifold form directly from T and V. The inertia matrix is the Hessian of
% T with respect to the velocities:
Mq = [ diff(diff(T,vel),vel), diff(diff(T,vel),om);
       diff(diff(T,om ),vel), diff(diff(T,om ),om) ];
Mq = simplify(Mq);

% Gravity generalized forces:  -dV/dq
Gq = [ -diff(V,pos); -diff(V,th) ];        % = [0; m*g*l*sin(th)]

% Velocity (Coriolis/centrifugal) term for this system enters only the cart
% equation as +m*l*sin(th)*om^2 (from d/dt of the cross term). Build the rhs
% to match: applied force + gravity + velocity term - damping.
rhs = [ F + m*l*sin(th)*om^2 - b*vel;
        Gq(2)                 - c*om ];
rhs = simplify(rhs);

disp('Inertia matrix Mq ='); pretty(Mq);
disp('Right-hand side rhs ='); pretty(rhs);

qdd = simplify(Mq \ rhs);
disp('Cart acceleration  vel_dot ='); pretty(qdd(1));
disp('Angular accel.     om_dot  ='); pretty(qdd(2));

% ---- Numeric cross-check against plant_dynamics.m ----------------------
p = params();
xtest = [0.0; 0.3; 0.2; -0.4];     % arbitrary [pos vel theta omega]
Ftest = 1.7;                       % arbitrary force [N]

subs_list = {M, m, l, I, g, b, c, F, pos, vel, th, om};
vals      = { p.M, p.m_body, p.l, p.I_body, p.g, p.b, p.c, Ftest, ...
              xtest(1), xtest(2), xtest(3), xtest(4) };

acc_sym = double(subs(qdd, subs_list, vals));      % [vel_dot; om_dot] (symbolic)
dx_num  = plant_dynamics(xtest, Ftest, p);         % numeric implementation
acc_num = [dx_num(2); dx_num(4)];

printf('\n--- cross-check at x=[%.2f %.2f %.2f %.2f], F=%.2f ---\n', xtest, Ftest);
printf('symbolic  vel_dot=%.6f  om_dot=%.6f\n', acc_sym(1), acc_sym(2));
printf('numeric   vel_dot=%.6f  om_dot=%.6f\n', acc_num(1), acc_num(2));
printf('max abs difference = %.3e  (should be ~0)\n', max(abs(acc_sym - acc_num)));
