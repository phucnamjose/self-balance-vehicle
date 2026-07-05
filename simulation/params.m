function p = params()
% PARAMS  Single source of truth for the balance-bot simulation.
%
%   p = params()  returns a struct of physical + control parameters in SI
%   units (meters, kilograms, seconds, radians, Newtons).
%
% Modeling choice (see simulation/README.md): the robot is a classic
% INVERTED PENDULUM ON A CART.
%   - The "cart" is the wheels + axle (it translates along the ground).
%   - The "pendulum" is the whole body above the axle (battery, ESP32,
%     frame), free to tip about the wheel axle.
%
% State vector used everywhere in the sim:
%
%       x = [ pos ;        % cart position along the ground   [m]
%             vel ;        % cart velocity                    [m/s]
%             theta ;      % body tilt from UPRIGHT vertical  [rad], 0 = balanced
%             omega ];     % body tilt rate                   [rad/s]
%
%   theta > 0  means the body has tipped FORWARD (in +pos direction).
%   theta = 0  is the (unstable) upright equilibrium we want to hold.
%
% Many of these numbers are ESTIMATES. Measure them on the real robot and
% refine here - every other script reads from this one file, so the whole
% study updates at once.

  % ---- World ----------------------------------------------------------
  p.g = 9.81;                 % gravity [m/s^2]

  % ---- Wheels (the "cart") -------------------------------------------
  % GB37-520 with a typical ~67 mm diameter wheel. MEASURE your wheel.
  p.r_wheel = 0.0335;         % wheel radius [m]
  p.m_wheel = 0.060;          % mass of ONE wheel [kg]
  p.n_wheels = 2;             % two driven wheels
  % Solid-disk approximation for a wheel's rotational inertia, I = 1/2 m r^2.
  p.I_wheel = 0.5 * p.m_wheel * p.r_wheel^2;   % [kg*m^2]

  % Effective translational mass of the cart. A rolling wheel resists
  % acceleration both by its mass AND by its rotational inertia reflected
  % through the radius (I_wheel / r^2). Both wheels count.
  p.M = p.n_wheels * (p.m_wheel + p.I_wheel / p.r_wheel^2);   % [kg]

  % ---- Body (the "pendulum") -----------------------------------------
  % Everything above the axle (frame, battery, ESP32, driver). MEASURE the
  % total mass, the body height, and the height of the center of mass (CoM)
  % above the axle. These three set how fast the robot tips - shorter/lighter
  % is twitchier and harder to balance.
  p.m_body      = 0.800;      % body mass [kg]
  p.body_height = 0.300;      % overall body height [m] (~30 cm)
  p.l           = 0.150;      % CoM height above the wheel axle [m] (~mid-body)
  % Body inertia about its OWN CoM, approximated as a uniform rod of height
  % body_height about its center:  I = (1/12) m H^2.
  p.I_body = (1/12) * p.m_body * p.body_height^2;   % [kg*m^2]

  % ---- Friction / damping (small; refine from real coast-down tests) --
  p.b = 0.01;                 % cart viscous damping  [N/(m/s)]
  p.c = 0.001;                % pivot viscous damping [N*m/(rad/s)]

  % ---- Motors (GB37-520, 12 V, 30:1) ---------------------------------
  % Linear torque-speed model per motor at full battery voltage:
  %     tau(u, w) = u * tau_stall  -  (tau_stall / w_noload) * w
  % where u in [-1,1] is the PWM duty command and w is the wheel
  % (output-shaft) angular speed [rad/s]. The second term is the back-EMF
  % effect: torque falls off linearly to zero at no-load speed.
  kgfcm_to_Nm     = 9.80665e-2;                 % 1 kgf*cm -> N*m
  p.motor.tau_stall = 5.0 * kgfcm_to_Nm;        % stall torque per motor [N*m]
  p.motor.w_noload  = 333 * 2*pi/60;            % no-load output speed [rad/s]
  p.motor.deadband  = 0.10;                     % |u| below this -> no motion (measured, Phase 4 deadband sweep; both motors ~10%)
  p.motor.u_max     = 1.0;                       % PWM saturation (100% duty)

  % ---- Control loop ---------------------------------------------------
  p.f_ctrl  = 200;            % control rate [Hz]  (matches firmware CONTROL_HZ)
  p.dt_ctrl = 1 / p.f_ctrl;   % control period [s] = 5 ms

  % ---- Encoder (for the firmware-units mapping, Step 8) --------------
  p.counts_per_wheel_rev = 1320;   % 4x quadrature, GB37-520 (11 PPR * 30 * 4)

end
