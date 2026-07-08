function [w_common, i_acc, u_raw] = balance_pid_sim(theta, theta_dot, i_acc, dt, c)
% BALANCE_PID_SIM  Outer balance (tilt) PID, a faithful port of
%                  firmware/main/balance_pid.c.
%
%   [w_common, i_acc, u_raw] = balance_pid_sim(theta, theta_dot, i_acc, dt, c)
%
% Reads the tilt @p theta (rad, 0 = upright) and tilt rate @p theta_dot (rad/s,
% the gyro) and returns the COMMON wheel-speed command w_common (rad/s) that both
% inner loops track:
%
%     e        = theta - theta_ref
%     w_common = Kp*e + Ki*Int(e) + Kd*theta_dot     (saturated to +/- w_max)
%
% with conditional-integration anti-windup - identical to the C. State carried by
% the caller: i_acc (integral, rad/s). Config struct c:
%   kp, ki, kd  - PID gains (rad/s per rad, per rad*s, per rad/s)
%   theta_ref   - upright trim [rad]
%   w_max       - output saturation [rad/s]
%   i_max       - integral clamp [rad/s]

  e = theta - c.theta_ref;

  u     = c.kp * e + i_acc + c.kd * theta_dot;
  u_raw = u;

  u_sat = max(-c.w_max, min(c.w_max, u));

  pushing_high = (u > u_sat) && (e > 0.0);
  pushing_low  = (u < u_sat) && (e < 0.0);
  if ~pushing_high && ~pushing_low
    i_acc = i_acc + c.ki * e * dt;
    i_acc = max(-c.i_max, min(c.i_max, i_acc));
  end

  w_common = u_sat;
end
