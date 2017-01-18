%% Identifying Available Devices
%
% Copyright 2001-2012 The MathWorks, Inc.

%% Identifying Installed Adaptors
% This example shows how to identify the available devices on your system and obtain device information.
%
% The |imaqhwinfo| function provides a structure with an
% |InstalledAdaptors| field that lists all adaptors on the current system 
% that the toolbox can access.
imaqInfo = imaqhwinfo

%%
imaqInfo.InstalledAdaptors

%% Obtaining Device Information
% Calling |imaqhwinfo| with an adaptor name returns a structure that provides
% information on all accessible image acquisition devices.
hwInfo = imaqhwinfo('winvideo')

%%
hwInfo.DeviceInfo

%%
% Information on a specific device can be obtained by simply indexing into 
% the device information structure array.
device1 = hwInfo.DeviceInfo(1)

%%
% The |DeviceName| field contains the image acquisition device name.
device1.DeviceName

%%
% The |DeviceID| field contains the image acquisition device identifier.
device1.DeviceID

%%
% The |DefaultFormat| field contains the image acquisition device's default 
% video format.
device1.DefaultFormat

%%
% The |SupportedFormats| field contains a cell array of all valid video 
% formats supported by the image acquisition device.
device1.SupportedFormats

%%
% Configure the number of frames to log upon triggering.
vidobj.FramesPerTrigger = 1;
triggerconfig(vidobj, 'manual')
trigger(vidobj);
% Wait for the acquisition to end.
wait(vidobj, 2);

% Determine the number frames acquired.
frameslogged = vidobj.FramesAcquired;

% vid = videoinput('winvideo', 2, 'BYRG_1024x768');
% vid1 = videoinput('gige', 1, 'Mono8');
imaqsupport;
imaqhwinfo
vid = videoinput('winvideo');
src = getselectedsource(vid);
%getsnapshot(src);
tic
frame = getsnapshot(vid);
% 
% start(vid);
% frame = peekdata(vid, 1);

%himage = preview(vid)
toc
imshow(frame);
image(frame);
imagesc(frame);

%%
for i = 1:5
    snapshot = getsnapshot(vidobj);
    imagesc(snapshot);
end

%%
% Measure the time to acquire 20 frames.
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