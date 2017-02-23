xRange = 3.2;
yRange = 2.4;

scaleMatrix = [xRange; yRange];

% iPos =[1.6 1.2]';
iPos =[0 0]';
kPos =[3.2 2.4]';
distanceRef = 0;
[psi, Psi] = distanceErrorGradient(distanceRef, iPos, kPos, scaleMatrix )