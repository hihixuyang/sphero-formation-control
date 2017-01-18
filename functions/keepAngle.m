function [ angle_out ] = keepAngle( hasMoved, angle, old_angle )
%keepAngle shall return the last angle if the object as not moved in the
%current timestep
    if ~hasMoved
        angle_out = old_angle;
    else
        angle_out = angle;
    end
end

