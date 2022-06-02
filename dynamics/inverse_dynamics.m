function tau = inverse_dynamics(t, robot, robot_params, q_des_v, Kd, Kp)
    % Compute Discrete Derivative
    % Xcurr: [theta, theta dot]
    
    % error = robot.pos - q_des_v(:, 1);
    % TODO: when qdes == curr == pi, it thinks it's at 0 and tries to go
    % there for a split second.
    error = calcPosError(robot, q_des_v(:, 1));
    derr_dt = robot.vel - q_des_v(:, 2);
    
%     q = q_des_v(:, 1);
%     qd = q_des_v(:, 2);
%     qdd = q_des_v(:, 3) - Kd*derr_dt - Kp*error;
%     tau = rnea(q, qd, qd, qdd, 1, robot_params);
    
    tau = robot.MassMat*(q_des_v(:, 3) - Kd * derr_dt - Kp * error) + robot.CorMat*robot.vel + robot.GravityVec;
end

