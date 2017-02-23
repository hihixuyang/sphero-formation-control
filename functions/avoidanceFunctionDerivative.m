function [Va,  dVadP ] = avoidanceFunctionDerivative(iPosition, jPosition, r, R)
%AVOIDANCEFUNCTIONDERIVATIVE Calculates the partial directional derivative
%of the avoidance function, with respect to the position of the agents
%A linear version of the avoidance function is implemented with values in
%[0, 1]. The original nonlinear function is commented out, because all the
%other components of the control law have been scaled to the same magnitude
%
%   INPUTS:
% r: protection radius
% R: detection radius
% iPosition: agent position [2*1]
% jPosition: second agent or obstacle position [2*1]
% OUTPUT
% Va: avoidance function 
% dVadP: partial directional derivative of Va [2*1]

d = norm(iPosition-jPosition);
%% nonlinear avoidance function

% if d>=R %zeros
%     dVadP = [0;0];
% elseif d<=r %signed infinty
%     dVadP = 10^6*(iPosition - jPosition)/d;
% else
%     dVadP = -(R^2-r^2)*(d^2-R^2)*(iPosition - jPosition)/((d^2-r^2)^3);
%     dVadP = min(10^6, max(-10^6, dVadP));
% end

%% linear avoidance function

if d>=R %outside the detection range Va = 0
    Va = 0;
    dVadP = 0 * (iPosition - jPosition)/d;
elseif d<=r %inside the protection radius, Va = 1
    Va = 1;
    dVadP = 1 * (iPosition - jPosition)/d;
else % inside the detection range, 0 < Va < 1 
    Va = (R-d)^2/(R-r);
    dVadP = (R-d)/(R-r) * (iPosition - jPosition)/d;
end
end

