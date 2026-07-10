function p = params()
% PARAMS  Single source of truth for sim parameters (SI units).
% Inverted pendulum on a cart: wheels=cart, body=pendulum about axle.
% x = [pos; vel; theta; omega]  theta from upright [rad], 0 = balanced.

  % ---- World ----------------------------------------------------------
  p.g = 9.81;                 % gravity [m/s^2]

  % ---- Wheels (cart) ---------------------------------------------------
  % Measured: 65 mm diameter wheel.
  p.r_wheel = 0.0325;         % wheel radius [m]  (65 mm diameter, measured)
  p.m_wheel = 0.060;          % mass of ONE wheel [kg]  (ESTIMATE)
  p.n_wheels = 2;             % two driven wheels
  p.track    = 0.190;         % distance between the two wheels [m] (centre-to-centre, measured)
  % I = 1/2 m r^2 per wheel
  p.I_wheel = 0.5 * p.m_wheel * p.r_wheel^2;   % [kg*m^2]

  % M includes translational + rotational inertia reflected through r
  p.M = p.n_wheels * (p.m_wheel + p.I_wheel / p.r_wheel^2);   % [kg]

  % ---- Body (pendulum) -------------------------------------------------
  % CoM height l is the most sensitive balance parameter (measure on robot).
  p.m_body      = 0.800;      % body mass [kg]  (ESTIMATE)
  p.body_height = 0.0875;     % body extent above the axle [m] = H - r (120 - 32.5 mm)
  p.l           = 0.050;      % CoM height above axle [m] (ESTIMATE)
  % Uniform rod about center: I = (1/12) m H^2
  p.I_body = (1/12) * p.m_body * p.body_height^2;   % [kg*m^2]

  % ---- Friction / damping ----------------------------------------------
  p.b = 0.01;                 % cart viscous damping  [N/(m/s)]
  p.c = 0.001;                % pivot viscous damping [N*m/(rad/s)]

  % ---- Motors (GB37-520, 12 V, 30:1) ---------------------------------
  % tau(u,w) = u*tau_stall - (tau_stall/w_noload)*w; u in [-1,1], w [rad/s]
  kgfcm_to_Nm     = 9.80665e-2;                 % 1 kgf*cm -> N*m
  p.motor.tau_stall = 5.0 * kgfcm_to_Nm;        % stall torque per motor [N*m]
  p.motor.w_noload  = 333 * 2*pi/60;            % no-load output speed [rad/s]
  p.motor.deadband  = 0.10;                     % |u| below this -> no motion (measured ~10%)
  p.motor.u_max     = 1.0;                       % PWM saturation (100% duty)
  % First-order duty lag: u_eff_dot = (u_cmd - u_eff)/tau_e; torque uses u_eff.
  % Without it sim inner loop is ~5x too fast vs measured ~0.19 s.
  p.motor.tau_e     = 0.19;                      % duty->speed time constant [s] (measured)

  % ---- Control loop ---------------------------------------------------
  p.f_ctrl  = 200;            % control rate [Hz]  (matches firmware CONTROL_HZ)
  p.dt_ctrl = 1 / p.f_ctrl;   % control period [s] = 5 ms

  % ---- Encoder (firmware units mapping) --------------------------------
  p.counts_per_wheel_rev = 1320;   % 4x quadrature, GB37-520 (11 PPR * 30 * 4)

end
