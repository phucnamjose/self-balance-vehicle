function [duty, i_acc, w_filt, u_raw] = wheel_pi_sim(w_set, w_meas, i_acc, w_filt, dt, c)
% WHEEL_PI_SIM  Inner wheel-speed PI, a faithful port of firmware/main/wheel_pi.c.
%
%   [duty, i_acc, w_filt, u_raw] = wheel_pi_sim(w_set, w_meas, i_acc, w_filt, dt, c)
%
% One step of the inner loop: measurement low-pass, feedforward u_ff = w_set/K,
% PI on the speed error, optional deadband compensation, output saturation, a
% brake cap when commanded to stop, and conditional-integration anti-windup -
% the same sequence (and order) as the C. The sim runs ONE lumped instance (the
% plant is a single symmetric cart) rather than the firmware's per-wheel pair.
%
% State carried by the caller between ticks:  i_acc (integral, duty), w_filt
% (filtered speed, rad/s).  Config struct c:
%   kp, ki        - PI gains (duty per rad/s, duty per rad)
%   kff           - feedforward steady gain K [rad/s per duty]  (u_ff = w_set/kff)
%   tau_f         - measurement low-pass time constant [s]
%   u_max         - output duty ceiling
%   i_max         - integral clamp (duty)
%   brake_max     - reverse-effort cap when w_set == 0
%   ff_en, db_en  - enable feedforward / deadband compensation (bool)
%   neutral, db_floor - deadband compensation zone + floor (duty)

  % Measurement low-pass (backward-Euler exponential smoother).
  a      = dt / (c.tau_f + dt);
  w_filt = w_filt + a * (w_meas - w_filt);

  e = w_set - w_filt;

  % Model-based feedforward (the duty that holds w_set in steady state).
  u_ff = 0.0;
  if c.ff_en && c.kff ~= 0.0, u_ff = w_set / c.kff; end

  % FF + P + I, before deadband and saturation.
  u     = u_ff + c.kp * e + i_acc;
  u_raw = u;

  duty = u;
  if c.db_en, duty = deadband_compensate(duty, c.neutral, c.db_floor); end

  duty_sat = max(-c.u_max, min(c.u_max, duty));

  % Brake cap: when told to stop but still rolling, limit reverse effort.
  if w_set == 0.0
    if     w_filt > 0.0, duty_sat = max(duty_sat, -c.brake_max);
    elseif w_filt < 0.0, duty_sat = min(duty_sat,  c.brake_max);
    end
  end

  % Conditional-integration anti-windup.
  pushing_high = (duty > duty_sat) && (e > 0.0);
  pushing_low  = (duty < duty_sat) && (e < 0.0);
  if ~pushing_high && ~pushing_low
    i_acc = i_acc + c.ki * e * dt;
    i_acc = max(-c.i_max, min(c.i_max, i_acc));
  end

  duty = duty_sat;
end

function u = deadband_compensate(u, neutral, floor_mag)
  if abs(u) < neutral, u = 0.0; return; end
  if u > 0.0, u = max(u,  floor_mag); else, u = min(u, -floor_mag); end
end
