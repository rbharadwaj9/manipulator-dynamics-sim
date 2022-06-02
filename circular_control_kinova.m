%{ 
Robot: Kinova
Idea:
1. We have a trajectory that provides desired q, qdot, qddot at each time;
and the desired q at this point. 
2. Using the q, qdot, and qddot; We compute the torque required at that
time step through inverse dynamics/whatever controller we use.
3. We apply this torque to the robot through forward dynamics with ode45
4. This solves for the q_resulting which we compare against q_des
%}
clear; clc; close all;
load kinova_zero_vel.mat
T = T(1:int32(length(T))); % To restrict amount of timesteps evaluated
q_traj = q_traj(:, :, 1:length(T));
q_traj(1, :, :) = (q_traj(1, :, :));
qDes = (qDes(:, 1:length(T)));

% Robot Properties
[kinova, robot_config]=loadrobot('kinovaGen3');
% [robot_params, kinova] = get_robot_params('./gen3_robotiq_2f_85_modified.urdf');
[robot_params, kinova] = get_robot_params(kinova);
kinova.Gravity = [0, 0, -9.81];
dof = robot_params.num_joints
%%
% Tuning Parameters
% Kv = 7.5e6;
% Kp = 5000;
% Kv = 10000*eye(dof);
% Kp = diag([3,3,3,3,10,10,10]);
Kd = 10*eye(dof);
Kp = 30*eye(dof);


% Initial Condition
% Xtheta0 = zeros(dof, 2);
% Xtheta0(3, 1) = 0.019;
% Xtheta0 = Xtheta0(:);
Xtheta0 = zeros(dof, 2);
% Xtheta0(:, 1) = wrapToPi([0., -0.26179939, 3.14159265, -4.01425728, 0., -0.95993109, 1.57079633])';
Xtheta0(4, 1) = Xtheta0(4, 1) + pi/2;
Xtheta0 = Xtheta0(:);


%% Solve Control Problem
clear inverse_dynamics;
% tspan = [min(T), max(T)];
[t,Xres] = ode45(@(t,Xtheta) kinova_dynamics(t, Xtheta, T, q_traj, kinova, robot_params, Kd, Kp), T, Xtheta0);
Xres = Xres';

%Run Xres values to find out torques for plotting. 
% Can't use vectorization because rnea only supports evaluation at one timeframe.
goalTorques = [];
for i = 1:length(t)
    ti = t(i);
    Xtheta = Xres(:, i);
    [dydX, tau] = kinova_dynamics(ti, Xtheta, T, q_traj, kinova, robot_params, Kd, Kp);
    goalTorques = [goalTorques, tau];
end
%% Plot Resulting Joint Angles vs Desired
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
%% Animate on Kinova
figure
show(kinova,Xres(1:dof,1));
view(3)
ax = gca;
ax.Projection = 'perspective';

hold on
% plot(points(:,1),points(:,2),'k')

framesPerSecond = 15;
r = robotics.Rate(framesPerSecond);
for i = 1:length(Xres)
%     show(kinova,qDes(1:dof,i),'Frames', 'off', 'PreservePlot',false);
    show(kinova,Xres(1:dof,i),'Frames', 'off', 'PreservePlot',false);
    drawnow
    waitfor(r);
end
%%
function [dXdt, tau] = kinova_dynamics(t, Xcurr, runtime, q_des_v, kinova, robot_params, Kv, Kp)
    % Simplification to reduce variables sent to inverse Dynamics   
    robot.dof = robot_params.num_joints;
    robot.joint_limits = robot_params.joint_limits;
    
    % Linear interpolation of desired trajectory at current time
    q_des_v = permute(q_des_v, [3, 2, 1]); % (timesteps x dof x [q, qd, qdd])
    q_des_curr = squeeze(interp1(runtime,q_des_v, t)); 
    
    % Extract current pos and velocity
    % Kinova Outputs Angles from 0 to 2pi, 
    % However, Kinova URDF joint limits are from -pi to pi. Thus, convert
    robot.pos = (Xcurr(1:robot.dof, :));
    robot.vel = Xcurr(robot.dof+1:end, :);
    
    % Calculate mass matrix, coriolis matrix, gravity vector
    % CLASSICAL BUGGY CURRENTLY
    robot.MassMat = spatial_rnea_mass(robot.pos, robot_params);
    robot.CorMat = spatial_rnea_coriolis(robot.pos, robot.vel, robot_params);
    robot.GravityVec = spatial_rnea_gravity(robot.pos, robot_params); 
    
    tau = inverse_dynamics(t, robot, robot_params, q_des_curr, Kv, Kp);
    dXdt = forward_dynamics(robot, tau);
    % ODE Solver needs a column vector as return value
%     tau = robot.GravityVec;
end