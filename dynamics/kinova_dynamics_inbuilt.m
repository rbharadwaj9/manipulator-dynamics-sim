function [dXdt, tau] = kinova_dynamics_inbuilt(t, Xcurr, runtime, q_des_v, kinova, robot_params, Kv, Kp)
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
    
    error = calcPosError(robot, q_des_curr(:, 1));
    derr_dt = robot.vel - q_des_curr(:, 2);

    qdd = q_des_curr(:,3) - Kp*error - Kv*derr_dt;
    tau = inverseDynamics(kinova, q_des_curr(:, 1), q_des_curr(:, 2), qdd); 

    dXdt = zeros(robot.dof*2, 1);
    dXdt(1:robot.dof,:) = robot.vel';
    dXdt(robot.dof+1:end, :) = forwardDynamics(kinova, robot.pos, robot.vel, tau);
end