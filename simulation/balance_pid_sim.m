function [w_common, i_acc, u_raw] = balance_pid_sim(theta, theta_dot, i_acc, dt, c)
% BALANCE_PID_SIM  Port of firmware/main/balance_pid.c.
%   [w_common, i_acc, u_raw] = balance_pid_sim(theta, theta_dot, i_acc, dt, c)
% theta, theta_dot [rad, rad/s] -> w_common [rad/s]. Caller holds i_acc.
% w_common = Kp*e + Ki*Int(e) + Kd*theta_dot (saturated); conditional anti-windup.
% Config c: kp, ki, kd, theta_ref, w_max, i_max.

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
