function [Delta] = calcPosError(robot, des)
%CALCPOSERROR Position error of robot and desired joint angles
%   For continous joints, wrap angle between 0 and 2pi and calculate error in
%   both directions. 
%   For revolute joints, direction can be calculated only in one direction

    contAngleError = calcContinuousAngleError(robot.pos, des);
    revAngleError = robot.pos - des;
    joints_with_limits = robot.joint_limits(1, :)' ~= -Inf;
    
    Delta = revAngleError .* joints_with_limits + contAngleError .* ~joints_with_limits;
end