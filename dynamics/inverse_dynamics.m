function tau = inverse_dynamics(t, robot, Xcurr, q_des_v, Kv, Kp)
    % Compute Discrete Derivative
    % Xcurr: [theta, theta dot]
    persistent prevErr prevTime;
    if isempty(prevErr)
       prevErr = zeros(size(Xcurr, 1)/2, 1); 
    end
    if isempty(prevTime)
       prevTime = zeros(size(Xcurr, 1)/2, 1); 
    end
    
    error = calcAngleError(Xcurr(1:robot.dof, :), q_des_v(:, 1));
    
    derr_dt = (error - prevErr) / 1;
    prevErr = error;
    prevTime = t;
    
    tau = robot.MassMat*(q_des_v(:, 3) - Kv * derr_dt - Kp * error) + robot.CorMat*Xcurr(robot.dof+1:end, 1) + robot.GravityMat; 
end

function Delta = calcAngleError(X, Y)
    Delta = min((2 * pi) - (X - Y), (X - Y));
end