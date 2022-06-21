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
    Kr = zeros(7);
    
    error = calcPosError(robot, q_des_curr(:, 1));
    derr_dt = robot.vel - q_des_curr(:, 2);
    
    qd_a = q_des_curr(:, 2);
    qdd_a = q_des_curr(:, 3) - Kp*error - Kv*derr_dt;
    
    tau = spatial_rnea(robot.pos, robot.vel, qd_a, qdd_a, true, zeros(6,1), robot_params);

    % Calculate mass matrix, coriolis matrix, gravity vector
    % robot.MassMat = spatial_rnea_mass(robot.pos, robot_params);
    % robot.CorMat = spatial_rnea_coriolis(robot.pos, robot.vel, robot_params);
    % robot.GravityVec = spatial_rnea_gravity(robot.pos, robot_params); 
    % dXdt = forward_dynamics(robot, tau);

    dXdt = zeros(robot.dof*2, 1);
    dXdt(1:robot.dof,:) = robot.vel';
    dXdt(robot.dof+1:end, :) = forwardDynamics(kinova, robot.pos, robot.vel, tau);
end
