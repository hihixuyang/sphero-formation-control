function [ angle ] = anglePythagora( dki, dkj, dji )
%DEGREES calculates the angle between three points in place using the
%generalizes Pythagorean theorem, k being the tip point
%   INPUTS:
% - 3 distances:
%OUTPUTS:
% - angle: angle in radians [0 pi]
temp = (dki^2 +dkj^2-dji^2)/(2*dki*dkj);
angle = acos(temp);

end

