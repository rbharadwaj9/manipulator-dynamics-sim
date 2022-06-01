function tau = inverse_dynamics(t, robot, q_des_v, Kv, Kp)
    % Compute Discrete Derivative
    % Xcurr: [theta, theta dot]
    persistent prevErr prevTime;
    if isempty(prevErr)
       prevErr = zeros(size(robot.pos, 1), 1); 
    end
    if isempty(prevTime)
       prevTime = zeros(size(robot.pos, 1), 1); 
    end
    
    error = calcPosError(robot, q_des_v(:, 1)); % robot.pos - q_des_v(:, 1);
    
    derr_dt = (error - prevErr) / 1;
    prevErr = error;
    prevTime = t;
    
    tau = robot.MassMat*(q_des_v(:, 3) - Kv * derr_dt - Kp * error) + robot.CorMat*robot.vel + robot.GravityVec; 
end

