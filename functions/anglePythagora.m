function [ angle ] = anglePythagora( dki, dkj, dji )
%ANGLEPYTHAGORA calculates the angle between three points in place using the
%generalizes Pythagorean theorem, k being the tip point
%   INPUTS:
% - dki: distance ki
% - dkj: distance kj
% - dji: distance ji
%OUTPUTS:
% - angle: angle in degrees [0 180]
angle = acosd((dki^2 +dkj^2-dji^2)/(2*dki*dkj));

end

