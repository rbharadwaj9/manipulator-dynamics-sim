function dXdt = forward_dynamics(robot, tau, X)
    % Input and output is in state space
    % [ theta; theta dot ]
    dXdt = zeros(2,1);
    dXdt(1) = X(2);
    dXdt(2) = inv(robot.MassMat) * (tau - robot.CorMat*X(2) - robot.GravityMat); 
end