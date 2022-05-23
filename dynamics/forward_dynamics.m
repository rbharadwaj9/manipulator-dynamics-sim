function Xres = forward_dynamics(robot, tau, X)
    % Input and output is in state space
    % [ theta; theta dot ]
    Xres = zeros(2,1);
    Xres(1) = X(2);
    Xres(2) = inv(robot.MassMat) * (tau - robot.CorMat*X(2) - robot.GravityMat); 
end