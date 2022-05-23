%% Test animation and plot for dynamics
tspan = [0, 30];

% Robot Properties
robot_config.L1 = 1; % m
robot_config.M = 1; % kg 

% Initial Condition
Xtheta0 = [0 0];

% Input torques
tau_t = linspace(tspan(1),tspan(2),500);
tau = 0.1.*tau_t; % N/m

[t,Xres] = ode45(@(t,Xtheta) single_link_dynamics(t, Xtheta, tau_t, tau, robot_config), tspan, Xtheta0);

% Plot Response
figure(1);
subplot(2,1,1);
plot(t,Xres(:,1),'-o')
title("Theta");
subplot(2,1,2);
plot(t,Xres(:,2),'-.');
title("Omega");

% Convert theta to cartesian
x_pos = robot.L1.*cos(Xres(:,1));
y_pos = robot.L1.*sin(Xres(:,1));
plot(x_pos,y_pos);

% Animate
for k = 1 : length(x_pos)
  figure(2);
  plot(x_pos(k), y_pos(k), 'r*-', 'LineWidth', 2);
  xlim([-2, 2]);
  ylim([-2, 2]);
  grid on;
  hold on;
  line([0,x_pos(k)], [0, y_pos(k)], 'LineWidth', 1);
  hold off;
  title("Live");
  pause(0.05);  
end

function dxdf = single_link_dynamics(t, Xcurr, tau_t, tau, robot_config)
    g = 9.8; % m/s^2
    tau = interp1(tau_t, tau, t);
    
    robot = single_link(Xcurr, robot_config);
    dxdf = forward_dynamics(robot, tau, Xcurr);
end