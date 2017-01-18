function [ dVodP ] = orientationControl(positionRef, spheroPos )
%ORIENTATIONCONTROL Calculates formation orientation error for agents 1 and 2
%
%   INPUTS:
% -positionRef: formation position reference [2*N]
% -spheroPos: current agent positions [2*N]
%
%   OUTPUTS:
% -dVoDp: gradient of formation error for the first 2 elements [2*2]


p12Ref = positionRef(:, 1) - positionRef(:, 2);
%p21Ref = positionRef(:, 2) - positionRef(:, 1);
%p21Ref = -p12Ref;

p12 = spheroPos(:, 1) - spheroPos(:, 2);
%p21 = spheroPos(:, 2) - spheroPos(:, 1);
%p21 = - p12;

L = [ 1 -1; -1 1];

Vo1 = (p12-p12Ref)'*L*(p12-p12Ref);
%Vo2 = (p21-p21Ref)'*L*(p21-p21Ref);
%Vo2 = Vo1;

dVodP1 = Vo1./p12;
%dVodP2 = Vo2./p21
dVodP2 = -dVodP1;
dVodP = [dVodP1 dVodP2];

end

