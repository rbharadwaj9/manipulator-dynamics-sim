function tau = inverse_dynamics(t, robot, Xcurr, q_des_v, Kv, Kp)
    % Compute Discrete Derivative
    % Xcurr: [theta, theta dot]
    persistent prevErr prevTime;
    if isempty(prevErr)
       prevErr = 0; 
    end
    if isempty(prevTime)
       prevTime = 0; 
    end
    
    error = Xcurr(1) - q_des_v(1);
    
    derr_dt = (error - prevErr) / 1;
    prevErr = error;
    prevTime = t;
    
    tau = robot.MassMat*(q_des_v(3) - Kv .* derr_dt - Kp .* error) + robot.CorMat*Xcurr(2) + robot.GravityMat; 
end