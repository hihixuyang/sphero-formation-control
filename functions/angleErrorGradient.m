function [ phi, Phi ] = angleErrorGradient(angleRef, iPosition, kPosition, jPosition)
%ANGLEERRORGRADIENT Calculates the gradient of the formation angle error.
% it uses the anti-trigonometric angle convention (left-hand rule)
%  INPUTS:
% angleRef : angle reference in degrees [-180, 180]
% iPos: i position
% kPos: k position
% jPos: j position
%OUTPUTS:
% phi: angle error [1*N]
% Phi: angle error gradient [2*N]

xi = iPosition(1);
xk = kPosition(1);

yi = iPosition(2);
yk = kPosition(2);

dki = norm(iPosition - kPosition);
dkj = norm(jPosition - kPosition);
dji = norm(iPosition - jPosition);

angle = anglePythagora(dki, dkj, dji);
angle = angle * signSi(iPosition, kPosition, jPosition);

phi = angle - angleRef;
if phi < -180
   phi = 360+phi; 
end
if phi > 180
   phi = phi-360; 
end
%phi = phi;
Phi = phi/180*[-(yi-yk); (xi-xk)]/dki;
% phi = phi^2;
end

