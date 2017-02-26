function [Vo, dVodP ] = orientationControl(positionRef, spheroPos )
%ORIENTATIONCONTROL Calculates formation orientation error for agents 1 and 2
%
%   INPUTS:
% -positionRef: formation position reference [2*N]
% -spheroPos: current agent positions [2*N]
%
%   OUTPUTS:
% -Vo :orientation error in degrees [-180 180]
% -dVoDp: gradient of formation error for the first 2 elements [2*2]


p12Ref = positionRef(:, 1) - positionRef(:, 2);
orientationRef = atan2d(p12Ref(2), p12Ref(1));

p12 = spheroPos(:, 1) - spheroPos(:, 2);
orientation = atan2d(p12(2), p12(1));
d12 = norm(p12);

x1 = spheroPos(1, 1);
x2 = spheroPos(1, 2);

y1 = spheroPos(2, 1);
y2 = spheroPos(2, 2);

orientationError = orientation - orientationRef;

if orientationError < -180
   orientationError = 360+orientationError; 
end
if orientationError > 180
    orientationError = orientationError-360;
end
Vo = orientationError;
dVodP1 = Vo/180*[(y1-y2); -(x1-x2)]/d12;

dVodP2 = -dVodP1;
dVodP = [dVodP1 dVodP2];
% Vo = Vo^2;
end

