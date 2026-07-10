% EOM_DERIVE  Symbolic EOM via Lagrangian; cross-checks plant_dynamics.m.
% Needs symbolic package. q=[pos;theta], theta from upward vertical (0=upright).

clear; clc;
pkg load symbolic;
% Suppress harmless "float to sym" warnings during numeric cross-check.
warning('off', 'all');

syms M m l I g b c F real          % parameters + input force
syms pos vel th om real            % state: position, velocity, tilt, tilt-rate

% ---- Energies -----------------------------------------------------------
xc_dot =  vel + l*cos(th)*om;      % horizontal CoM velocity
yc_dot =       -l*sin(th)*om;      % vertical   CoM velocity

T = 1/2*M*vel^2 ...                       % cart translation
  + 1/2*m*(xc_dot^2 + yc_dot^2) ...       % body translation
  + 1/2*I*om^2;                           % body rotation
V = m*g*l*cos(th);                        % gravity PE

T = simplify(T);
disp('Kinetic energy T ='); pretty(T);
disp('Potential energy V ='); pretty(V);

% ---- Manipulator form  Mq*qdd = rhs ------------------------------------
% Inertia matrix = Hessian of T w.r.t. velocities.
Mq = [ diff(diff(T,vel),vel), diff(diff(T,vel),om);
       diff(diff(T,om ),vel), diff(diff(T,om ),om) ];
Mq = simplify(Mq);

% Gravity generalized forces:  -dV/dq
Gq = [ -diff(V,pos); -diff(V,th) ];        % = [0; m*g*l*sin(th)]

% Cart equation gets +m*l*sin(th)*om^2 (Coriolis); rhs = force + gravity + vel term - damping.
rhs = [ F + m*l*sin(th)*om^2 - b*vel;
        Gq(2)                 - c*om ];
rhs = simplify(rhs);

disp('Inertia matrix Mq ='); pretty(Mq);
disp('Right-hand side rhs ='); pretty(rhs);

qdd = simplify(Mq \ rhs);
disp('Cart acceleration  vel_dot ='); pretty(qdd(1));
disp('Angular accel.     om_dot  ='); pretty(qdd(2));

% ---- Cross-check vs plant_dynamics.m -----------------------------------
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
