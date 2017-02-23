clear all;
clc;
formationVelocity = [1; 1];
movementThreshold = 1;

spheroPos = [0; 0];
[controlHeading, desiredHeading, actualHeading] =...
    calculateAngle(spheroPos, formationVelocity, movementThreshold)

%%
spheroPos = [0; 2];
[controlHeading, desiredHeading, actualHeading] =...
    calculateAngle(spheroPos, formationVelocity, movementThreshold)

%%
spheroPos = [2; 4];
[controlHeading, desiredHeading, actualHeading] =...
    calculateAngle(spheroPos, formationVelocity, movementThreshold)
%%
spheroPos = [2.1; 4.1];
[controlHeading, desiredHeading, actualHeading] =...
    calculateAngle(spheroPos, formationVelocity, movementThreshold)
%%
spheroPos = [2.5; 4.5];
[controlHeading, desiredHeading, actualHeading] =...
    calculateAngle(spheroPos, formationVelocity, movementThreshold)