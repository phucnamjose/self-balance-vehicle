function [duty, i_acc, w_filt, u_raw] = wheel_pi_sim(w_set, w_meas, i_acc, w_filt, dt, c)
% WHEEL_PI_SIM  Port of firmware/main/wheel_pi.c (one lumped inner loop).
%   [duty, i_acc, w_filt, u_raw] = wheel_pi_sim(w_set, w_meas, i_acc, w_filt, dt, c)
% LPF -> feedforward -> PI -> deadband -> saturate -> brake cap -> anti-windup.
% Config c: kp, ki, kff, tau_f, u_max, i_max, brake_max, ff_en, db_en, neutral, db_floor.

  % Backward-Euler LPF
  a      = dt / (c.tau_f + dt);
  w_filt = w_filt + a * (w_meas - w_filt);

  e = w_set - w_filt;

  % Steady-state feedforward duty
  u_ff = 0.0;
  if c.ff_en && c.kff ~= 0.0, u_ff = w_set / c.kff; end

  % FF + P + I before deadband/saturation
  u     = u_ff + c.kp * e + i_acc;
  u_raw = u;

  duty = u;
  if c.db_en, duty = deadband_compensate(duty, c.neutral, c.db_floor); end

  duty_sat = max(-c.u_max, min(c.u_max, duty));

  % Brake cap when w_set=0 but still rolling
  if w_set == 0.0
    if     w_filt > 0.0, duty_sat = max(duty_sat, -c.brake_max);
    elseif w_filt < 0.0, duty_sat = min(duty_sat,  c.brake_max);
    end
  end

  % Conditional-integration anti-windup
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
