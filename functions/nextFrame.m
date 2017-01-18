function [ frame ] = nextFrame(cam, isWebcam)
%nextFrame returns the next Frame from either an already recorded video
%file which is passed as a videoReader, or live imagery from a webcam
%passed as a webcam object. The second parameter is a boolean, which is
%true if a webcam is used.

if isWebcam
   frame = getsnapshot(cam);
else
   frame = readFrame(cam);
end

end

