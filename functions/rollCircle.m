function [ ready ] = rollCircle( sphero, speed )
%ROLLCIRCLE Summary of this function goes here
%   Detailed explanation goes here
    for i = 0:30:360
       i
       sphero.Roll(speed, i);
       pause(0.2);
    end
    sphero.Roll(0, 0);
    ready = true;
end

