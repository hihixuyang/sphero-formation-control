function [ angleError ] = angleError( angle, angleRef,...
                           iPosition, kPosition, jPosition )
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here

angleError = (angle - angleRef)*signSi(iPosition, kPosition, jPosition);

end

