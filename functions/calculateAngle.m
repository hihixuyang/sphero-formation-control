function [controlAngle] = calculateAngle(spheroPos, agentError, movementThreshold)
persistent lastSpheroPos lastControlAngle;
if isempty(lastSpheroPos)
    lastSpheroPos = 0;
end

displacement = spheroPos - lastSpheroPos;
movement = sqrt(sum(displacement.^2, 1));
hasMoved = movement > movementThreshold; 
displacement = displacement.*hasMoved;
lastSpheroPos = spheroPos.*hasMoved + lastSpheroPos.*~hasMoved;

desiredDirection = atan2d (agentError(2,:), agentError(1, :));
actualDirection = atan2d (displacement(2,:), displacement(1,:));
agentAngleError = actualDirection - desiredDirection;

if isempty(lastControlAngle)
    lastControlAngle = 0;    
    hasMoved = zeros(1, size(spheroPos, 2));
end
controlAngle = lastControlAngle - agentAngleError.*hasMoved;
controlAngle = wrapTo360(controlAngle);
lastControlAngle = controlAngle;
end