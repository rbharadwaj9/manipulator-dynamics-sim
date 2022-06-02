function dXdt = forward_dynamics(robot, tau)
    % Input and output is in state space
    % [ theta; theta dot ]
    dXdt = zeros(2,robot.dof);
    dXdt(1,:) = robot.vel';
    dXdt(2,:) = inv(robot.MassMat) * (tau - robot.CorMat*robot.vel - robot.GravityVec); 
end