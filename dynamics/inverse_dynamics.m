function tau = inverse_dynamics(t, robot, q_des_v, Kd, Kp)
    % Compute Discrete Derivative
    % Xcurr: [theta, theta dot]
    persistent prevErr prevTime;
    if isempty(prevErr)
       prevErr = zeros(size(robot.pos)); 
    end
    if isempty(prevTime)
       prevTime = zeros(size(robot.pos)); 
    end
    
%     error = robot.pos - q_des_v(:, 1);
    % TODO: when qdes == curr == pi, it thinks it's at 0 and tries to go
    % there for a split second.
    error = calcPosError(robot, q_des_v(:, 1)); 
    derr_dt = robot.vel - q_des_v(:, 2);

    prevErr = error;
    prevTime = t;
    
    tau = robot.MassMat*(q_des_v(:, 3) - Kd * derr_dt - Kp * error) + robot.CorMat*robot.vel + robot.GravityVec; 
end

