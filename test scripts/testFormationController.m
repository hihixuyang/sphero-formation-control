clear all
spheroPos = [11 20 ; 11 20];
obstaclePos = [];
positionRef = [10 20; 10 20 ];
angleRef = [NaN, NaN];
%angleRef = [0, 0];
r = 0;
R = 0;
doOrientation = false;
k_a = 0; % angle error gain
k_v = 1; % distance error gain
k_o = 0; % obstacle error gain
formationGains = [k_a, k_v, k_o];

[agentError, d, angle, Phi, Psi, dVadP, dVodP] =...
    formationController(spheroPos, obstaclePos, positionRef, angleRef, r, R,...
    formationGains, doOrientation);