function [psi, Psi] = distanceErrorGradient(distanceRef, iPos, kPos, scaleMatrix )
%DISTANCEERRORGRADIENT Calculates the distance error and error gradient of 
%the formation and scales the gradient
%   INPUTS:
% -distanceRef: distance reference [1*N]
% -iPos: iPosition
% -kPos: kPosition
% -scaleMatrix
%OUTPUT:
% - Psi: distance error gradient, with sum of squared elements in [0..1]

dki = norm(iPos-kPos);
scaling = norm(scaleMatrix);
psi = dki-distanceRef;

Psi = -psi/scaling*(iPos - kPos)/dki;
psi = psi^2;
end

