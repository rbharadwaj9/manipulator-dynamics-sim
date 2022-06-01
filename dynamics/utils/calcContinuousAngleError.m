function [Delta] = calcContinuousAngleError(X,Y)
%CALCCONTINUOUSANGLEERROR Calculate error for continous angles in both
%directions
%   
% Moving Delta amount from X should get you to Y.
% https://stackoverflow.com/a/28037434/19250186
    diff = mod(X - Y + pi, 2*pi) - pi;
    if diff < -pi
        Delta = diff + 2*pi;
    else 
        Delta = diff;
    end
end

