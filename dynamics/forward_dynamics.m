function dXdt = forward_dynamics(robot, tau)
    % Input and output is in state space
    % [ theta; theta dot ]
    dXdt = zeros(robot.dof*2, 1);
    dXdt(1:robot.dof,:) = robot.vel';
    dXdt(robot.dof+1:end,:) = inv(robot.MassMat) * (tau - robot.CorMat*robot.vel - robot.GravityVec);
end