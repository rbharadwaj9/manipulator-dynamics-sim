%% Dynamics for a single link arm
function robot = single_link(Xcurr, robot_config)
    g = 9.8; % m/s^2
    
    robot.MassMat = [(4/3)*robot_config.M*(robot_config.L1.^2)];
    robot.CorMat = [0];
    robot.GravityMat = [robot_config.M .* g .* robot_config.L1 .* cos(Xcurr(1))];
end

function robot = single_link_dynamics(t, q_curr_v, q_des_v, tau_t, tau, robot, Kp, Kd)
    g = 9.8; % m/s^2
    tau = interp1(tau_t, tau, t);
    
    robot.MassMat = [(4/3)*robot.M*(robot.L1.^2)];
    robot.CorMat = [0];
    robot.GravityMat = [robot.M .* g .* robot.L1 .* cos(q_curr_v(1))];
    
    dxdf = forward_dynamics(robot, tau, q_v);
    inv_tau = inverse_dynamics(robot, q_curr_v, q_des_v);
end