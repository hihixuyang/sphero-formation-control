function [ Psi ] = distanceErrorGradient(dki, distanceRef, iPos, kPos )
%DISTANCEERRORGRADIENT Calculates the distance error gradient of the
%formation.
%   INPUTS:
% -iPos: 
% -kPos:
% -distanceRef:
% -dki:
%OUTPUT:
% - Psi:

Psi = (iPos - kPos)*(distanceRef-dki)/dki;

end

