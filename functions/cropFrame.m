function [croppedFrame] = cropFrame(frame, coords, frameSize)
%Extracts a quadratic image around the the GIVEN coordinates, with the 
%GIVEN size from the GIVEN frame. RETURNs extracted picture.

if nargin < 3 && isEmpty(frameSize)
    frameSize = 400;
end

offset = zeros(1,2); %offsets on x and y

%calculate offset values and create a rectangle to crop from the frame
%check for y-component
if coords(1)-(frameSize/2) < 0
    offset(1) = 0;
else
    offset(1) = coords(1)-(frameSize/2);
end

%check for x-component
if coords(2)-(frameSize/2) < 0
    offset(2) = 0;
else
    offset(2) = coords(2)-(frameSize/2);
end
%first XY corner and opposing XY corner of the rectangle
rectSphero = [offset(1) offset(2)  frameSize frameSize];
croppedFrame  = imcrop(frame,rectSphero);