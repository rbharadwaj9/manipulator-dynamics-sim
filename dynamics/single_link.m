%% Dynamics for a single link arm
function robot = single_link(Xcurr, robot_config)
    g = 9.8; % m/s^2
    
    robot.dof = 1;
%     robot.joint_limits = [-pi/2 - pi/4 - pi/8; pi/2+pi/4+pi/8];
    robot.joint_limits = [-Inf; Inf];
    robot.MassMat = [(4/3)*robot_config.M*(robot_config.L1.^2)];
    robot.CorMat = [0];
    robot.GravityVec = [robot_config.M .* g .* robot_config.L1 .* cos(Xcurr(1))];
end