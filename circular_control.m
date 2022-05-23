%{ 
Robot: Single Link
Idea:
1. We have a trajectory that provides desired q, qdot, qddot at each time;
and the desired q at this point. 
2. Using the q, qdot, and qddot; We compute the torque required at that
time step through inverse dynamics/whatever controller we use.
3. We apply this torque to the robot through forward dynamics with ode45
4. This solves for the q_resulting which we compare against q_des
%}
clear;
clear inverse_dynamics;
load 1link_sine_accel.mat
T = T(1:int32(length(T)));
q_traj = q_traj(:, :, 1:length(T));

% Robot Properties
robot_config.L1 = 1; % m
robot_config.M = 1; % kg 

% Tuning Parameters
% Kv = 7.5e6;
% Kp = 5000;
Kv = 7.5e6;
Kp = 5000;

% Initial Condition
Xtheta0 = [pi, 10];

torques = [];

[t,Xres] = ode45(@(t,Xtheta) single_link_dynamics(t, Xtheta, T, squeeze(q_traj(1,:,:)), robot_config, Kv, Kp), T, Xtheta0);

figure(1);
plot(T, Xres(:, 1));
hold on;
plot(T,qDes(1, 1:length(T)));
title("Calculated $q$ vs Desired $q$ (rad)", 'interpreter', 'latex');
legend("$q_{res}$", "$q_{des}$", 'interpreter', 'latex');
hold off;

animate(Xres, qDes(1, :).', robot_config);

function dXdt = single_link_dynamics(t, Xcurr, runtime, q_des_v, robot_config, Kv, Kp)
    robot = single_link(Xcurr, robot_config);
    
    q_des_curr = interp1(runtime,q_des_v.',ones(3,1).*t);
    tau = inverse_dynamics(t, robot, Xcurr, q_des_curr, Kv, Kp);
    dXdt = forward_dynamics(robot, tau, Xcurr);
end