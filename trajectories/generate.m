%% File containing methods to generate trajectories for simulation
% q_des(t) = q_0 + q_0(t) + 1/2 k^* t^2
%% 1 revolute link stationary, against gravity. 
T = (0:0.01:2*pi);
dof = 1; % Num degrees of freedom
qInit = [0; 0];
accel = zeros(dof, length(T));
q_traj = motion_eqn(T, qInit, accel);
qDes = q_traj(1, :, :);

filename = '1link_zero_vel.mat';
save(filename, 'T', 'dof', 'qDes', 'q_traj');
plot_traj(dof, q_traj, "Single Link Zero Velocity, Against Gravity");

%% 1 revolute link moving in a circle, const velocity, acceleration
T = (0:0.01:2*pi);
dof = 1; % Num degrees of freedom
qInit = [-pi/2-pi/4; 1]; % 1 rad/s
accel = zeros(dof, length(T));
q_traj = motion_eqn(T, qInit, accel);
qDes = q_traj(1, :, :);

filename = '1link_const_vel.mat';
save(filename, 'T', 'dof', 'qDes', 'q_traj');

plot_traj(dof, q_traj, "Single Link Const Velocity Trajectory Beginning in 3rd Quadrant");

%% 1 revolute link with sine wave acceleration
T = (0:0.01:2*pi);
dof = 1; % Num degrees of freedom
qInit = [0; 1]; % 1 rad/s
accel = zeros(dof, length(T));
accel(1, :) = sin(T);

q_traj = motion_eqn(T, qInit, accel);
qDes = q_traj(1, :, :);

filename = '1link_sine_accel.mat';
save(filename, 'T', 'dof', 'qDes', 'q_traj');
plot_traj(dof, q_traj, "Single Link Sine Accel Trajectory");

%% Kinova with zero velocity, accel
T = (0:0.01:2*pi);
dof = 7; % Num degrees of freedom
% Zero velocity
qInit = zeros(2, 7);
qInit(1, 4) = pi/2;
accel = zeros(dof, length(T));

q_traj = motion_eqn(T, qInit, accel);
qDes = squeeze(q_traj(1, :, :));

filename = 'kinova_zero_vel.mat';
save(filename, 'T', 'dof', 'qDes', 'q_traj');
plot_traj(dof, q_traj, "Kinova Zero Velocity Trajectory");

%% Kinova with single joint const velocity
T = (0:0.01:2*pi);
dof = 7; % Num degrees of freedom
% Const velocity or 1 rad/second on joint 4
qInit = zeros(2, 7); 
qInit(2, 1) = 1;
accel = zeros(dof, length(T));

q_traj = motion_eqn(T, qInit, accel);
qDes = squeeze(q_traj(1, :, :));

filename = 'kinova_const_vel.mat';
save(filename, 'T', 'dof', 'qDes', 'q_traj');
plot_traj(dof, q_traj, "Kinova Const Velocity Trajectory");


%% Kinova with multiple joint const velocity
T = (0:0.01:2*pi);
dof = 7; % Num degrees of freedom
% Const velocity or 1 rad/second on joint 4
qInit = zeros(2, 7); 
qInit(2, 1) = 5;
qInit(2, 4) = 1;
accel = zeros(dof, length(T));

q_traj = motion_eqn(T, qInit, accel);
qDes = squeeze(q_traj(1, :, :));

filename = 'kinova_two_const_vel.mat';
save(filename, 'T', 'dof', 'qDes', 'q_traj');
plot_traj(dof, q_traj, "Kinova Two Joints Const Velocity Trajectory");

%% Kinova with multiple joint const velocity and accel
T = (0:0.01:2*pi);
dof = 7; % Num degrees of freedom
% Const velocity or 1 rad/second on joint 4
qInit = zeros(2, 7); 
qInit(1, 2) = 0.019; % assume simulation always starts at 0s
qInit(2, 4) = 1;
accel = zeros(dof, length(T));
accel(1, :) = ones(1, length(T));

q_traj = motion_eqn(T, qInit, accel);
qDes = squeeze(q_traj(1, :, :));

filename = 'kinova_complex_vel.mat';
save(filename, 'T', 'dof', 'qDes', 'q_traj');
plot_traj(dof, q_traj, "Kinova Complex Trajectory");

%% Kinova static in "home" position
T = (0:0.01:2*pi);
dof = 7; % Num degrees of freedom
% Const velocity or 1 rad/second on joint 4
qInit = zeros(2, dof); 
qInit(1, :) = [0., -0.26179939, 3.14159265, -4.01425728, 0., -0.95993109, 1.57079633];
accel = zeros(dof, length(T));

q_traj = motion_eqn(T, qInit, accel);
qDes = squeeze(q_traj(1, :, :));

filename = 'kinova_const_home.mat';
save(filename, 'T', 'dof', 'qDes', 'q_traj');
plot_traj(dof, q_traj, "Kinova Const at Home Trajectory");

%%
% qInit = ([q;qdot] x dof) 
% accel = (dof x timesteps)
% T = (1 x timesteps)
function q_traj = motion_eqn(T, qInit, accel)
    qDes = qInit(1, :)' + (qInit(2, :)' .* T) + 0.5.*accel.*(T.^2);
    qDes_dot = qInit(2, :)' + accel.*T;
    q_traj = cat(3, qDes, qDes_dot, accel);
    q_traj = permute(q_traj, [3,1,2]);
end

%%
function fig = plot_traj(dof, q_traj, filename)
    % Plot each of q0, qdot, qddot, and qdes in separate subplots
    % Each line within the subplot indicates a single DOF
    fig = figure();
    subplot(3,1,1);
    plot(squeeze(q_traj(1,:,:))');
    title('$q$','Interpreter','latex','FontSize', 19);
    hold on;
    labels = strings();
    for i = 1:dof
        labels(i) = num2str(i);
    end
    legend(labels);
    subplot(3,1,2);
    plot(squeeze(q_traj(2,:,:))')
    title('$\dot{q}$','Interpreter','latex','FontSize', 19);
    subplot(3,1,3);
    plot(squeeze(q_traj(3,:,:))')
    title('$\ddot{q}$','Interpreter','latex','FontSize', 19);
    
    sgtitle(filename);
end