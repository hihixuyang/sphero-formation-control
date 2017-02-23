clc;
spheroPos = [1 1.6 2; 1 1.5 2];
N = size(spheroPos, 2);
%the detection and avoidance region
R = 0.9;
r = 0.2;

%timesteps, just initial value
delta_t = 0.1;

%formation control parameters
k_d = 0.5; % distance error gain
k_a = 1; % angle error gain
k_oa = 1; % obstacle avoidance
k_o = 0; %orientation
k_r = 0; %reference tracking

beta = 45;
Vr = [cosd(beta); sind(beta)];
formationGains = [k_d, k_a, k_oa, k_o, k_r];
%PID tuning
Kp = 0.3; Ki = 0; Kd = 0;
PIDgains = [Kp Ki Kd];
saturation = 0.1;

%position reference for each agent [2*N]
p1 = [1;1];
p2 = [2;2];
positionRef = [p1, p2];
distanceRef = [0.5 0.5 0.5];

%angle reference for agents [1*N], NaN for no reference for that agent
angleRef = [NaN NaN NaN];

%static obstacles to prove collision avoidance [2*M]
obstacles =  [ ];
obstacles = horzcat(obstacles,[1.1 1.1; 3.2 2.4]');
M = size(obstacles, 2);

%initial value for threshold, smaller means more sensitive
movementThreshold = 0.15;
angleOffset = true;
if angleOffset
    training = true;
    agent = 1;
    offset = zeros(1, N);
end
trainingSpeed = 0.05;
xRange = 3.2; yRange = 2.4;
scaleMatrix = [xRange; yRange];

[u, Phi, Psi, Va, dVadP, Vo, dVodP, phi, psi] = ...
    formationController(spheroPos, obstacles, distanceRef, positionRef, angleRef, Vr,...
    r, R, formationGains, scaleMatrix)