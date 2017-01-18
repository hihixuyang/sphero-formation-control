function [ dVadP ] = avoidanceFunctionDerivative( d, r, R, iPosition, jPosition )
%AVOIDANCEFUNCTIONDERIVATIVE Calculates the partial derivative of the avoidance
%function, with respect to the position of the agents
%
%   INPUTS:
% d: distance between agents
% r: protection radius
% R: detection radius
% iPosition: reference agent position [2*1]
% jPosition: second agent position [2*1]
% OUTPUT
% dVadP: partial derivative of Va [2*1]
if d>=R %zeros
    dVadP = [0;0];
elseif d<=r %signed infinte
    dVadP = 1e+06*sign(iPosition - jPosition);
else
    dVadP = 4*(R^2-r^2)*(d^2-R^2)*(iPosition - jPosition)/((d^2-r^2)^3);
end

end

