function [ Psi ] = distanceErrorGradient(dki, distanceRef, iPos, kPos )
%DISTANCEERRORGRADIENT Calculates the distance error gradient of the
%formation.
%   INPUTS:
% -dki:distance between agents [1*N]
% -distanceRef: distance reference [1*N]
% -iPos: iPosition
% -kPos: kPosition
%OUTPUT:
% - Psi: distance error gradient

Psi = (iPos - kPos)*(distanceRef-dki)/dki;

end

