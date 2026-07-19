% TUNE_CASCADE  Sweep balance-PID gains on the multi-rate cascade sim; score tilt
% RMS, saturation, drift. Mirrors sim_cascade.m: motor+wheel-PI @ f_motor, IMU +
% complementary filter @ f_imu, balance PID @ f_bal, inner tau_f = 0 (current firmware).
clear; clc;
here = fileparts(mfilename('fullpath')); addpath(here);
p = params();

% Inner loop = firmware wheel_pi gains, measurement LPF OFF (tau_f=0).
inner = struct('kp',0.150,'ki',0.750,'kff',33.3,'tau_f',0.0, ...
               'u_max',0.95,'i_max',0.95,'brake_max',0.4, ...
               'ff_en',true,'db_en',false,'neutral',0.02,'db_floor',p.motor.deadband);

function [fell, rms_tilt, drift, sat] = run_one(p, inner, outer, BAL_MAX_TILT)
  alpha=0.98; gyro_bias=deg2rad(0.1); gyro_noise=deg2rad(1.0);
  accel_noise=deg2rad(2.0); w_meas_noise=0.3;
  dt_m=1/p.f_motor; n_sub=2; dt_sub=dt_m/n_sub;
  imu_div=round(p.f_motor/p.f_imu); bal_div=round(p.f_motor/p.f_bal);
  dt_imu=1/p.f_imu; dt_bal=1/p.f_bal;
  T=6.0; N=round(T/dt_m);
  x=[0;0;deg2rad(5);0]; th_est=x(3); gm=x(4); vp=x(2);
  bi=0; wc=0; wi=0; wf=x(2)/p.r_wheel; ue=0;
  nd=max(0,round(0.004/dt_m)); dq=zeros(1,nd+1);   % 4 ms residual transport delay
  randn('seed',1);
  fell=false; sat_cnt=0; tail=[];
  for n=1:N
    if mod(n-1,imu_div)==0
      gm=x(4)+gyro_bias+gyro_noise*randn();
      ac=(x(2)-vp)/dt_imu; ta=x(3)+ac/p.g+accel_noise*randn(); vp=x(2);
      th_est=alpha*(th_est+gm*dt_imu)+(1-alpha)*ta;
    end
    if mod(n-1,bal_div)==0
      if abs(th_est)<BAL_MAX_TILT
        [wc,bi]=balance_pid_sim(th_est,gm,bi,dt_bal,outer);
      else
        wc=0; bi=0;
      end
    end
    wm=x(2)/p.r_wheel + w_meas_noise*randn();
    if abs(th_est)<BAL_MAX_TILT
      [duty,wi,wf]=wheel_pi_sim(wc,wm,wi,wf,dt_m,inner);
    else
      duty=0; wi=0; wf=wm;
    end
    if abs(duty)>0.94, sat_cnt=sat_cnt+1; end
    dq=[dq(2:end),duty]; da=dq(1);
    for s=1:n_sub
      ue=ue+(dt_sub/p.motor.tau_e)*(da-ue);
      um=ue; if abs(um)<p.motor.deadband, um=0.0; end
      ww=x(2)/p.r_wheel;
      tau=um*p.motor.tau_stall-(p.motor.tau_stall/p.motor.w_noload)*ww;
      F=p.n_wheels*tau/p.r_wheel;
      k1=plant_dynamics(x,F,p); k2=plant_dynamics(x+0.5*dt_sub*k1,F,p);
      k3=plant_dynamics(x+0.5*dt_sub*k2,F,p); k4=plant_dynamics(x+dt_sub*k3,F,p);
      x=x+(dt_sub/6)*(k1+2*k2+2*k3+k4);
    end
    if n*dt_m>=T-3.0, tail(end+1)=x(3); end
    if abs(x(3))>pi/2, fell=true; break; end
  end
  if fell, rms_tilt=Inf; drift=Inf; sat=1;
  else rms_tilt=rad2deg(sqrt(mean(tail.^2))); drift=100*abs(x(1)); sat=sat_cnt/N; end
end

% Ki must clear the gravity stiffness g/r ~ 302 (else no restoring term). With the
% faster tau_f=0 inner loop + 250 Hz balance, hot gains chatter the duty into
% saturation (and the cart accel then corrupts the tilt estimate) -> favour the
% lowest gains that still hold, i.e. least saturation.
Kps=[30 40 50];
Kds=[0 1 2 3 4];
Kis=[350 400 450 500];
best={}; rows=[];
for kp=Kps, for kd=Kds, for ki=Kis
  outer=struct('kp',kp,'ki',ki,'kd',kd,'theta_ref',0,'w_max',20,'i_max',20);
  [fell,rms_t,drift,sat]=run_one(p,inner,outer,0.6);
  rows(end+1,:)=[kp ki kd rms_t sat drift fell];
end, end, end

% Score: only non-fell; prefer low saturation then low tilt RMS + small drift.
printf('total=%d  fell=%d  ok=%d\n', size(rows,1), sum(rows(:,7)==1), sum(rows(:,7)==0));
ok = rows(rows(:,7)==0, :);
if isempty(ok), printf('none survived; dumping all rows:\n'); disp(rows); return; end
clean = ok(ok(:,5) < 0.05, :);
if isempty(clean), printf('(no unsaturated survivors; showing least-saturated)\n');
  [~,so]=sort(ok(:,5)); clean = ok(so(1:min(12,end)),:); end
cost = clean(:,4) + 0.03*clean(:,6) + 5.0*clean(:,5);   % tiltRMS[deg] + drift[cm] + sat penalty
[~,ord]=sort(cost);
clean = clean(ord,:);
printf('  Kp    Ki    Kd | tiltRMS(deg)  satFrac  drift(cm)   (best first)\n');
for i=1:min(12,size(clean,1))
  r=clean(i,:);
  printf('%5.0f %5.0f %5.0f | %10.3f  %7.3f  %8.1f\n', r(1),r(2),r(3),r(4),r(5),r(6));
end
