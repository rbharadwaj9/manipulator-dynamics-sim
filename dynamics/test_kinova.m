%% Test animation and plot for dynamics
clear;
tspan = [0, 5];

% Robot Properties
dof = 7;
[kinova, robot_config]=loadrobot('kinovaGen3');
kinova.DataFormat = 'row';
lbr.Gravity = [0 0 -9.8]; % Since default gravity is 0

% Initial Condition
% (dof x [theta, thetadot])
Xtheta0 = zeros(dof, 2);
Xtheta0 = Xtheta0(:);

% Input torques
tau_t = linspace(tspan(1),tspan(2),500);
tau = zeros(dof, length(tau_t));
tau(1, :) = 0.1.*tau_t; % N/m

%%
[t,Xres] = ode45(@(t,Xtheta) kinova_dynamics(t, Xtheta, tau_t, tau, kinova), tspan, Xtheta0);
Xres = Xres.';

%% Plot Response
figure(1);
subplot(2,1,1);
plot(t,Xres(1:dof,:),'-o');
title("Theta");
subplot(2,1,2);
plot(t,Xres(dof+1:end,:),'-.');
title("Omega");

%%
figure
show(kinova,Xres(1:dof,1)');
view(3)
ax = gca;
ax.Projection = 'perspective';

hold on
% plot(points(:,1),points(:,2),'k')

framesPerSecond = 30;
r = robotics.Rate(framesPerSecond);
for i = 1:length(Xres)
    show(kinova,Xres(1:dof,i)','Frames', 'off', 'PreservePlot',false);
    plot(kinova.getBody("EndEffector_Link").CenterOfMass);
    drawnow
    waitfor(r);
end

%%
function dXdt = kinova_dynamics(t, Xcurr, tau_t, tau, kinova)
    % Tau: (dof x timesteps)
    % Xcurr: (dof x [theta, thetadot])
    dof = kinova.NumBodies - 1;
    tau = interp1(tau_t, tau.', t);
    Xcurr = reshape(Xcurr, dof, []);

    dXdt = zeros(dof*2, 1);
    dXdt(1:dof) = Xcurr(:, 2);
    dXdt(dof+1:end) = forwardDynamics(kinova, Xcurr(:, 1).', Xcurr(:, 2).', tau).';
end