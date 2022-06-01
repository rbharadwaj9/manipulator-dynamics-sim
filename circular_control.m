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
clear; close all;
clear inverse_dynamics;
load 1link_const_vel.mat
T = T(1:int32(length(T)));
q_traj = wrapToPi(q_traj(:, :, 1:length(T)));
qDes = wrapToPi(qDes(:, 1:length(T)));

% Robot Properties
robot_config.L1 = 1; % m
robot_config.M = 1; % kg 

% Tuning Parameters
Kv = 7.5e6;
Kp = 5000;
% Kv = 30000;
% Kp = 50;

% Initial Condition
Xtheta0 = [pi/2+pi/4, 0];

%% Apply Control
[t,Xres] = ode45(@(t,Xtheta) single_link_dynamics(t, Xtheta, T, q_traj, robot_config, Kv, Kp), T, Xtheta0);
Xres = Xres';

%% Run Xres values to find out torques for plotting. 
% Can't use vectorization because rnea only supports evaluation at one timeframe.
goalTorques = zeros(dof, length(t));
for i = 1:length(t)
    ti = t(i);
    Xtheta = Xres(:, i);
    [dydX, tau] = single_link_dynamics(ti, Xtheta, T, q_traj, robot_config, Kv, Kp);
    goalTorques(:, i) = tau;
end

%% Plot results
figure(1);
for i = 1:dof
    subplot(dof, 2, 2*i-1);
    plot(t, Xres(i,:));
    hold on;
    plot(T,qDes(i, 1:length(T)));
    hold off;
    title(num2str(i));
end


% Plot torques calculated for the robot.
kinova_torque_limits = [39.0;39.0;39.0;39.0;9.0;9.0;9.0]; % N-m from documentation
for i = 1:dof
    subplot(dof, 2, i*2);
    plot(goalTorques(i,:));
    hold on;
    yline(kinova_torque_limits(i), 'LineWidth', 1.5);
    yline(-1*kinova_torque_limits(i), 'LineWidth', 1.5);
    ylim([-1*kinova_torque_limits(i)-5, kinova_torque_limits(i)+5]);
    title(strcat("Joint: ", num2str(i), "; Max Torque: ", num2str(max(abs(goalTorques(i,:)))), " N-m"));
end
sgtitle("Calculated $q$ vs Desired $q$ (rad); Calculated Torques (N-m)", 'interpreter', 'latex');

%% Animate
animate(Xres, qDes', robot_config);

%%
function [dXdt, tau] = single_link_dynamics(t, Xcurr, runtime, q_des_v, robot_config, Kv, Kp)
    robot = single_link(Xcurr, robot_config);
    
    q_des_v = permute(q_des_v, [3, 2, 1]); % (timesteps x dof x [q, qd, qdd])
    q_des_curr = interp1(runtime,q_des_v, t);
    
    robot.pos = wrapToPi(Xcurr(1:robot.dof, :));
    robot.vel = Xcurr(robot.dof+1:end, :);
    
    tau = inverse_dynamics(t, robot, q_des_curr, Kv, Kp);
    dXdt = forward_dynamics(robot, tau, Xcurr);
end