clear all
spheroPos = [11 ; 10 ];
obstaclePos = [];
positionRef = [10 ; 10 ];
angleRef = [NaN];
%angleRef = [0, 0];
r = 0;
R = 0;
doOrientation = false;
k_a = 1; % angle error gain
k_v = 1; % distance error gain
k_o = 1; % obstacle error gain
formationGains = [k_a, k_v, k_o];

[formationError, d, angle, Phi, Psi, dVadP, dVodP] =...
    formationController(spheroPos, obstaclePos, positionRef, angleRef, r, R,...
    formationGains, doOrientation);