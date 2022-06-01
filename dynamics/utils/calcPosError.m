function [Delta] = calcPosError(robot, des)
%CALCPOSERROR Position error of robot and desired joint angles
%   For continous joints, wrap angle between 0 and 2pi and calculate error in
%   both directions. 
%   For revolute joints, direction can be calculated only in one direction

%     Delta = zeros(robot.dof, 1);
%     for i = 1:robot.dof
%        if robot.joint_limits(1, i) == -Inf
%            Delta(i, 1) = calcContinuousAngleError(robot.pos(i, 1), des(i, 1));
%        else
%            Delta(i, 1) = robot.pos(i, 1) - des(i, 1);
%        end
%     end
    
% Vectorized version is faster by almost 2x based on naive profiling
    contAngleError = calcContinuousAngleError(robot.pos, des);
    revAngleError = robot.pos - des;
    joints_with_limits = robot.joint_limits(1, :)' ~= -Inf;
    
    Delta = revAngleError .* joints_with_limits + contAngleError .* ~joints_with_limits;
end