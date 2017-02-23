function [controlHeading, desiredHeading, actualHeading] =...
    calculateAngle(spheroPos, formationVelocity, movementThreshold)
%CALCULATEANGLE calculates the angles of the agents
%INPUT:
%-spheroPos [2*N]
%-desiredHeading [1*N]
%-movementThreshold 
%OUTPUT:
%-controlHeading [1*N] :considers the offset of the Sphero
%-actualHeading [1*N] :the direction in which the Sphero moves

persistent lastControlHeading;
if isempty(lastControlHeading)
    lastControlHeading = 0;    
end

persistent lastActualHeading;
if isempty(lastActualHeading)
    lastActualHeading = 0;    
end

[displacement, hasMoved] = detectMovement(spheroPos, movementThreshold);
desiredHeading = atan2d (formationVelocity(2,:), formationVelocity(1,:));
heading = atan2d (displacement(2,:), displacement(1,:));
actualHeading = heading.*hasMoved + lastActualHeading.*~hasMoved;
headingError = desiredHeading - actualHeading;

controlHeading = lastControlHeading + headingError.*hasMoved;
%controlHeading = lastControlHeading + headingError;
controlHeading = wrapTo360(controlHeading);

lastControlHeading = controlHeading;
lastActualHeading = actualHeading;
end