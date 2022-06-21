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
load kinova_complex_all_jt.mat;
T = T(1:int32(length(T))); % To restrict amount of timesteps evaluated
q_traj = q_traj(:, :, 1:length(T));
q_traj(1, :, :) = wrapToPi(q_traj(1, :, :));
qDes_plot = qDes(:, 1:length(T));
qDes = wrapToPi(qDes(:, 1:length(T)));

% Robot Properties
% [kinova, robot_config]=loadrobot('kinovaGen3');
[robot_params, kinova] = get_robot_params('gen3_robotiq_2f_85_modified.urdf');
% [robot_params, kinova] = get_robot_params(kinova);
kinova.Gravity = [0, 0, -9.81];
dof = robot_params.num_joints
%%
% Tuning Parameters
% Kv = 7.5e6;
% Kp = 5000;
% Kv = 10000*eye(dof);
% Kp = diag([3,3,3,3,10,10,10]);
Kd = diag([100,100,100,100,100,100,100]);
Kp = 1*diag([500,500,500,500,2000,2000,2000]);


% Initial Condition
% Xtheta0 = zeros(dof, 2);
% Xtheta0(3, 1) = 0.019;
% Xtheta0 = Xtheta0(:);
Xtheta0 = zeros(dof, 2);
% Xtheta0(:, 1) = wrapToPi([0., -0.26179939, 3.14159265, -4.01425728, 0., -0.95993109, 1.57079633])';
% Xtheta0(4, 1) = Xtheta0(4, 1) + pi/2;
% Xtheta0 = Xtheta0(:);


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
    [dydX, tau] = kinova_dynamics_inbuilt(ti, Xtheta, T, q_traj, kinova, robot_params, Kd, Kp);
    goalTorques = [goalTorques, tau];
end
%% Plot Resulting Joint Angles vs Desired
figure(1);
for i = 1:dof
    subplot(dof, 2, 2*i-1);
    plot(t, Xres(i,:));
    hold on;
    plot(T,qDes_plot(i, 1:length(T)));
    hold off;
    title(num2str(i));
    legend("Calculated", "Desired");
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
ax.Projection = 'perspect ive';

hold on
% plot(points(:,1),points(:,2),'k')

framesPerSecond = 15;
r = robotics.Rate(framesPerSecond);
for i = 1:length(Xres)
%     show(kinova,qDes(1:dof,i),'Frames', 'off', 'PreservePlot',false);
    show(kinova,Xres(1:dof,i),'Frames', 'on', 'PreservePlot',false);
    drawnow
    waitfor(r);
end