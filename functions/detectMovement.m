function [displacement, hasMoved] = detectMovement(spheroPos, movementThreshold)
%%DETECTMOVEMENT detects if the agents have moved over a certain threshold,
%%considered measurement error, and outputs the displacement and a hasMoved
%%flag, which is 1 if true, and 0 if false.

persistent lastSpheroPos
if isempty(lastSpheroPos)
    lastSpheroPos = spheroPos;
end

displacement = spheroPos - lastSpheroPos;
movement = sqrt(sum(displacement.^2, 1));
hasMoved = movement > movementThreshold; 
displacement = displacement.*hasMoved;

lastSpheroPos = spheroPos.*hasMoved + lastSpheroPos.*~hasMoved;
end