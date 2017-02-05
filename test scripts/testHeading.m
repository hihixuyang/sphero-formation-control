clear all;
%spheroPos = [-2 -2 0 5; 0 -2 -2 -2];
spheroPos = [-2 -2 -2 -2 -2; 0 -1 -2 0 5];
desiredDirection = 90;
lastSpheroPos = [0;0];
hasMoved = 0;
lastControlAngle = 0;
counter = 1;

while true
    displacement = spheroPos(:, counter) - lastSpheroPos;
    movement = sqrt(sum(displacement.^2, 1));
    hasMoved = movement > 1; %2 cm movement
    displacement = displacement.*hasMoved;
    lastSpheroPos = spheroPos(:, counter).*hasMoved + lastSpheroPos.*~hasMoved;
    
    actualDirection = atan2d (displacement(2,:), displacement(1,:));
    agentAngleError = actualDirection - desiredDirection;

    controlAngle = lastControlAngle - agentAngleError.*hasMoved;
    
    if counter == 1
        controlAngle = 0;
    end
    lastControlAngle = controlAngle
    counter = counter+1;
end