% matlab quick script examining the finer points of velocity matching


% the main goal is to first match velocity to target's and use the remaining velocity budget to travel along the line of sight to the obstacle, resulting in the time optimal interception of a constant velocity target. 
% NOTE: when it is possible to intercept, next velocity is such that maximum deceleration will match target position and velocity at some time t_r. Since t_r may not land on a discrete timestep, the controller will 'wiggle' instead of have zero velocity when the intercept occurs at the next timestep. This simply means that the target has been aquired and the controller is matching velocity during some non-integral time.


clear;
pos_k =[0;0];
vel_k =[2;0];
pos_t =[4;0];
vel_t =[-1;-1];
dt    = .1;
pos = pos_k;
pos_t_hist = pos_t;
vel = vel_k;
accel_hist = [0;0];
T = 0:dt:50;

for i=T(2:end)
  acc=vel_match(pos_k,vel_k,pos_t,vel_t,dt);
  [pos_k,vel_k] = dyn(pos_k,vel_k,acc,dt);
  pos_t = pos_t+vel_t*dt;
  accel_hist(:,end+1) = acc;
  pos(:,end+1) = pos_k;
  vel(:,end+1) = vel_k;
  pos_t_hist(:,end+1) = pos_t;
end

figure(1); clf; hold on;
plot(pos(1,:),pos(2,:),'LineWidth',2, 'DisplayName','interceptor');
plot(pos_t_hist(1,:),pos_t_hist(2,:),'--','LineWidth',2,'DisplayName','target')
xlabel('x');ylabel('y'); 
legend('Location','Best');
axis equal;
figure(2); clf; hold on;
plot(T,vecnorm(pos_t_hist-pos),'LineWidth',2,'DisplayName','dist');
plot(T,vecnorm(vel_t-vel),'LineWidth',2,'DisplayName','vel');
xlabel('t'); ylabel('norm difference');
legend('Location','Best');




function [pos_k1,vel_k1] = dyn(pos_k,vel_k,acc,dt)
  pos_k1 = eye(2)*pos_k + dt*eye(2)*(vel_k) + 0.5*dt^2*eye(2)*acc;
  vel_k1 = vel_k + dt*eye(2)*acc;

end

function acc = vel_match(pos_k,vel_k,pos_t, vel_t, dt)
  vel_k1 = vel_match_vel(pos_k,vel_k,pos_t,vel_t,dt);
  acc = (vel_k1-vel_k)/dt;
  % issues arise when target velocity can change
  if(norm(acc) > 1)
    acc = acc / norm(acc);
  end
end

function vel_k1 = vel_match_vel(pos_k,vel_k,pos_t, vel_t, dt)
  % assume there is an accel limit:
  a_max = 1;
  % assume there is a velocity limit:
  v_max = 2;

  r = pos_t-pos_k;
  alpha =0;

  v_del = vel_t-vel_k;

  if (norm(vel_t)>=v_max)
    disp("warning: target moving faster than interceptor max velocity!");
    vel_k1 = [nan;nan];
    return 
  end

  if (norm(r) <1e-2 && norm(v_del) < 1e-2)
    disp("intercepted!")
    vel_k1 = vel_t;
    return
  end
  a = norm(r)^2;
  b = 2*(v_del(1)*r(1) + v_del(2)*r(2));
  c = norm(v_del)^2 - a_max^2*dt^2;

  det = b^2-4*a*c;

  if(det<0)
    vel_dir = [-r(2);r(1)];
    if (dot(vel_dir,vel_t)<0)
      vel_dir = -vel_dir;
    end
    vel_k1 = vel_k + vel_dir*a_max/norm(vel_dir)*dt;
    if(norm(vel_k1)>v_max)
      vel_k1 = v_max*vel_k1/norm(vel_k1);
    end
    return

  else
    alpha_a = (-b+sqrt(det))/(2*a);
  end
  % velocity limit
  a = norm(r)^2;
  b = 2*dot(vel_t,r);
  c = norm(vel_t)^2 - v_max^2;

  det = b^2-4*a*c;

  if(det<0)
    disp("no vel solution can be found!");
  else
    alpha_v = (-b+sqrt(det))/(2*a);
  end
  % there is always a position limit
  a = 1;
  b = a_max*dt;
  c = dot(vel_k,r)/norm(r)*a_max*dt - 2*a_max*norm(r);
  det = b^2-4*a*c;
  if(c>0)
    %keyboard
  end
  if(det<0)
    % position limit will then be the one to whip backwards
    a = norm(r)^2;
    b = 2*(v_del(1)*r(1) + v_del(2)*r(2));
    c = norm(v_del)^2 - a_max^2*dt^2;
    det = b^2-4*a*c;
    alpha_p = (-b-sqrt(det))/(2*a);
    %keyboard
  else
    V_k1p(1) = (-b-sqrt(det))/(2*a);
    V_k1p(2) = (-b+sqrt(det))/(2*a);
    alpha_p = V_k1p(2)/norm(r);
  end
  % position limited velocity
  vel_k1_plim = vel_t+alpha_p*r;

  alpha = min(min(alpha_a,alpha_v),alpha_p); % also should include alpha from vel limit

  vel_k1=vel_t + r*alpha;

end
