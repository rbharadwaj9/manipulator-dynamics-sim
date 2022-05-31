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
clear inverse_dynamics;
load kinova_complex_vel.mat
T = T(1:int32(length(T)));
q_traj = q_traj(:, :, 1:length(T));
q_traj(1, :, :) = wrapTo2Pi(q_traj(1, :, :));
qDes = wrapTo2Pi(qDes(:, 1:length(T)));

% Robot Properties
dof = 7;
[kinova, robot_config]=loadrobot('kinovaGen3');
[robot_params, kinova] = get_robot_params(kinova);

%%
% Tuning Parameters
% Kv = 7.5e6;
% Kp = 5000;
Kv = 10000*eye(dof);
Kp = diag([3,3,3,3,10,10,10]);

% Initial Condition
% Xtheta0 = zeros(dof, 2);
% Xtheta0(3, 1) = 0.019;
% Xtheta0 = Xtheta0(:);
Xtheta0 = zeros(dof, 2);
Xtheta0(4, 1) = pi/2+0.001;
Xtheta0(:, 1) = deg2rad([0.000762939453125, 0.1, 0.07003647089004517, 0.06445464491844177, 0.01, 0.05, 0.08])
Xtheta0 = Xtheta0(:);

torques = [];

%% Solve Control Problem
[t,Xres] = ode45(@(t,Xtheta) single_link_dynamics(t, Xtheta, T, q_traj, kinova, robot_params, Kv, Kp), T, Xtheta0);
Xres = Xres';

%% Run Xres values to find out torques for plotting. 
% Can't use vectorization because rnea only supports evaluation at one timeframe.
goalTorques = [];
for i = 1:length(t)
    ti = t(i);
    Xtheta = Xres(:, i);
    [dydX, tau] = single_link_dynamics(ti, Xtheta, T, q_traj, kinova, robot_params, Kv, Kp);
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
function [dXdt, tau] = single_link_dynamics(t, Xcurr, runtime, q_des_v, kinova, robot_params, Kv, Kp)
    robot.dof = kinova.NumBodies - 1;
    
    q_des_v = permute(q_des_v, [3, 2, 1]);
    q_des_curr = squeeze(interp1(runtime,q_des_v, t));
    robot.MassMat = rnea_mass(Xcurr(1:robot.dof, :), robot_params);
    robot.CorMat = rnea_coriolis(Xcurr(1:robot.dof, :), Xcurr(robot.dof+1:end, :), robot_params);
    robot.GravityMat = rnea_gravity(Xcurr(1:robot.dof, 1), robot_params);
    tau = inverse_dynamics(t, robot, Xcurr, q_des_curr, Kv, Kp);


    dXdt = zeros(robot.dof*2, 1);
    dXdt(1:robot.dof) = Xcurr(robot.dof+1:end, :);
%     dXdt(dof+1:end) = forwardDynamics(kinova, Xcurr(:, 1), Xcurr(:, 2),
%     tau).'; % Why not required?
    
    % update acceleration 
    qdd = robot.MassMat\(tau-robot.CorMat*Xcurr(robot.dof+1:end, :)-robot.GravityMat);
   
    % update state derivative
    dXdt(robot.dof+1:end) = qdd;
end