function [ speed ] = toSpeed( centroid, lastCentroid, dt )
%toVel calcualtes the speed from the current position and the last
%postion, given the time step between the two samples

speed = sqrt(sum(((centroid - lastCentroid) ./  dt).^2, 2));
%vel = sqrt(sum((vel).^2, 2));

end

