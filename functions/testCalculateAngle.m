lastSpheroPos = [1; 1];
spheroPos = [3; 3];
displacement = spheroPos - lastSpheroPos;

desiredAngle = 10;
spheroAngle = atan2d (displacement(2,:), displacement(1,:));

controlAngle = desiredAngle - spheroAngle
heading = arrayfun(@(angle) (wrapTo360(angle)), controlAngle)
