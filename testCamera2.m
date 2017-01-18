imaqreset;
%vidobj = videoinput('winvideo', 1);
vidobj = videoinput('winvideo', 2, 'BYRG_1600x1200');
vidobj.FramesPerTrigger = 1;

%%
tic
for i = 1:20
    snapshot = getsnapshot(vidobj);
end

elapsedTime = toc

% Compute the time per frame and effective frame rate.
timePerFrame = elapsedTime/20
effectiveFrameRate = 1/timePerFrame

%%
% Configure the object for manual trigger mode.
triggerconfig(vidobj, 'manual');

% Now that the device is configured for manual triggering, call START.
% This will cause the device to send data back to MATLAB, but will not log
% frames to memory at this point.
start(vidobj)

% Measure the time to acquire 20 frames.
tic
for i = 1:20
    snapshot = getsnapshot(vidobj);
end

elapsedTime = toc

% Compute the time per frame and effective frame rate.
timePerFrame = elapsedTime/20
effectiveFrameRate = 1/timePerFrame

% Call the STOP function to stop the device.
stop(vidobj)