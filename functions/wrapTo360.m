function wrappedAngle = wrapTo360(ang)
%Substitute for the wrapTo360 function from the mapping toolbox. Only works
%with scalars! Wraps angles in degrees to [0, 360) degrees

wrappedAngle = mod(ang, 360);

end