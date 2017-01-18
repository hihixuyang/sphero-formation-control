function [ Va ] = avoidanceFunction(d, r, R )
%AVOIDANCEFUNCTION Calculates collision avoidance considering the radii of
%protection (r) and detection (R), r<R
%if d<=r then Va = Inf
%if d>=R then Va = 0
%
%   INPUTS:
% d: distance between objects
% r: radius of protection
% R: radius of detection
%   OUTPUTS:
% Va: avoidance value

v_temp = ((d^2) - (R^2))/((d^2) - (r^2));
Va = min(0, v_temp)^2;

end

