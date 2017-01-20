function [ Phi ] = angleErrorGradient( angle, angleRef,...
                           iPosition, kPosition, jPosition)
%ANGLEERRORGRADIENT Calculates the gradient of the formation angle error.
%  INPUTS: 
%- angle
% - angleRef
% - iPos
% - kPos
% -jPos
%OUTPUTS: 
% - Phi [2*N]

xi = iPosition(1);
xk = kPosition(1);
xj = jPosition(1);

yi = iPosition(2);
yk = kPosition(2);
yj = jPosition(2);

angleError = (angle - angleRef)*signSi(iPosition, kPosition, jPosition);

dki = norm(iPosition - kPosition);
Phi = [-(yi-yk); (xi-xk)]*angleError/dki^2;

end

