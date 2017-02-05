%% Initialization and user input
clc;
delete (instrfindall); %delete all instruments
clear all;
close all;
disp('Starting...')
sph = Sphero('Sphero-OWW')
N = 1;
M = 0;
sph.SetRGBLEDOutput([1 1 1], false);

sph.SetBackLEDOutput(1)
%% CONSTANTS
disp('Initializing constants...')
%timesteps, just initial value
cycleTime = 0.1;
delta_t = cycleTime;
%velocity reference tracking controller gains: Kp, Ki, Kd
velocityKp = 0.04; velocityKi = 0.00; velocityKd = 0.00;
velocity_PID_gains = [velocityKp velocityKi velocityKd];

meterPerPixel = 2e-3;

%initial value for threshold, smaller means more sensitive
threshold = .35;
spheroPos = zeros(2, N);
movementThreshold = 0.1;
saturation = 0.3;
positionRef = [0.1 0.1]';
%% SWITCHES

%Toggles whether or not to display the tracking results in Windowed Players
doDisplay = false;

%toggles between spherodetection using foreground detection and the
%detection using a threshold and an areafilter
doForegroundSubtraction = false;

%switches between a webcam input and a recorded input
isWebcam = true;

%toggles formation orientation control
doOrientation = false;

%% VIDEO INITIALIZATIONS FOR TRACKING
imaqreset;
%camera used. Normally the index >>1<< refers to the USB-Camera
obj.cam = videoinput('winvideo', 2, 'BYRG_1600x1200');
obj.cam.FramesPerTrigger = 1;
% Configure the object for manual trigger mode.
triggerconfig(obj.cam, 'manual');
obj.cam.LoggingMode = 'memory';

% vid.LoggingMode = 'disk&memory';
% diskLogger = VideoWriter('C:\Users\mihai\Dropbox\Uni TUHH\Sem 3\Project\doc\images and videos\test_0001.avi', 'Motion JPEG AVI');
% vid.DiskLogger = diskLogger;
% diskLogger.FrameRate = 30;
% diskLogger.Quality = 60;

% Now that the device is configured for manual triggering, call START.
% This will cause the device to send data back to MATLAB, but will not log
% frames to memory at this point.
start(obj.cam);

% %Player for the video and the mask
% obj.videoPlayer = vision.VideoPlayer('Position', [20, 400, 700, 400]);
% obj.maskPlayer = vision.VideoPlayer('Position', [740, 400, 700, 400]);
if doForegroundSubtraction
    %initialize a Forground detector to substract the foreground (moving
    %objects) from the background. The output is a binary mask.
    obj.detector = vision.ForegroundDetector('NumGaussians', 3,...
        'NumTrainingFrames', 40,...
        'MinimumBackgroundRatio', 0.7);
    
    %Initialize a Blobanalyzer to find groups of ON-pixels which very likely
    %refer to a moving object. The Analyzer calculates their characteristics
    obj.blobAnalyser = vision.BlobAnalysis('BoundingBoxOutputPort', true,...
        'AreaOutputPort', true,...
        'CentroidOutputPort', true,...
        'MinimumBlobArea', 400);
end

%% DATALOGGING INITIALIZATIONS
disp('Initializing logging variables...')
%timings of the individual processes
% 1: time needed for acquiring an image
% 2: time needed for detecting spheros inside this image
% 3: time needed for the prediction of future positions
% 4: time needed to assign current detections to existing tracks
% 5: time needed to update the tracks with the current positions
% 6: time needed to display the detections if enabled
% 7: time needed to calculate the reference velocity value for each Sphero
% using the formation control law
% 8: time needed to calculate actual direction
% 9: time needed to evaluate the output of the "local" controller
%10: time needed to log relevant data
%11: time needed to send the results to the individual units
%
timings = zeros(1, 11);

%maximum size of buffers
MAX_LOG = 2500;

%current write index
logCounter = 1;

logImages = cell(MAX_LOG, 1);
logNumDetections = zeros(MAX_LOG, 1);
logPosition = zeros(MAX_LOG, 2, N);
logDistance = zeros(MAX_LOG, N, N+M);
logAgentAngle = zeros(MAX_LOG, N) ;

logAgentError = zeros(MAX_LOG, 2, N);
logPhi = zeros(MAX_LOG, 2, N);
logPsi = zeros(MAX_LOG, 2, N) ;
logphi = zeros(MAX_LOG, N);
logpsi = zeros(MAX_LOG, N);
logdVadP = zeros( MAX_LOG, 2, N);
logdVodP = zeros( MAX_LOG, 2, N);

logControlVelocity = zeros(MAX_LOG, 2, N);
logPVelocity = zeros(MAX_LOG, 2, N);
logIVelocity = zeros(MAX_LOG, 2, N);
logDVelocity = zeros(MAX_LOG, 2, N);
logControlSpeed = zeros(MAX_LOG, N);

logControlAngle = zeros(MAX_LOG, N);
logDesiredDirection = zeros(MAX_LOG, N);
logActualDirection = zeros(MAX_LOG, N);
logPAngleOut = zeros(MAX_LOG, N);
logIAngleOut = zeros(MAX_LOG, N);
logDAngleOut = zeros(MAX_LOG, N);

logTiming = zeros(MAX_LOG, 11);

%% Initial detection and association
disp('Initial detection...');
frame = nextFrame(obj.cam, isWebcam);
%to avoid false recognitions
[centroids, mask] = findSpheroCentroid(frame, threshold);
numberOfDetections = size(centroids, 1);

while numberOfDetections < N
    %imshow(mask);
    threshold = threshold -.01; %makes it more sensitive
    disp('Attempting initial detection, searching for Spheros, decreasing threshold');
    frame = nextFrame(obj.cam, isWebcam);
    [centroids, mask] = findSpheroCentroid(frame, threshold);
    numberOfDetections = size(centroids, 1);
    % pause();
end
threshold = threshold - 0.01;
%% tracking loop
disp(' Tracking loop...')

while true
    tic
    frame = nextFrame(obj.cam, isWebcam);
    t_capture = toc;
    
    tic
    %detecting the spheros in the image
    if doForegroundSubtraction
        %calculate foreground mask. mask is a binary image.
        %this routine is normally turned off in favour of the thresholding
        %approach which has better performance if the SPheros do not move or
        %only move very little
        
        mask = obj.detector.step(frame);
        
        % Apply morphological operations to remove noise and fill in holes.
        mask = imopen(mask, strel('rectangle', [3,3]));
        mask = imclose(mask, strel('rectangle', [15, 15]));
        mask = imfill(mask, 'holes');
        
        %Perform blobanalysis on binary image and find connected components.
        %centroids shold contain the centroid of every detected object and
        %bboxes should contain the bounding boxes around these objects.
        [~, centroids, bboxes] = obj.blobAnalyser.step(mask);
    else
        %the second option is the one described in the report. Here the main
        %principle relies on extracting bright objects of the correct size
        %finding Spheros in an image
        [centroids, mask] = findSpheroCentroid(frame, threshold);
    end
    numberOfDetections = size(centroids, 1);
    
    %error messages if the detections do not comply with the number of robots
    if numberOfDetections == 0
        disp('No agents detected');
    else
        if numberOfDetections > N
            disp('Too many detections')
        else
            if numberOfDetections < N
                disp('Not enough detections')
            end
        end
    end
    t_find = toc;
    
    tic

    t_predict = toc;
    
    tic

    t_assign = toc;
    
    tic;

    t_update = toc;
    
    tic;
    %Display the results of detection
    if doDisplay
        image(frame);
    end
    t_display = toc;
    
    %% formation control
    tic
    %stack the positions in meters in a [2*N] matrix, as in the paper
    spheroPos = centroids'*meterPerPixel;
    %change direction of y axis and move x axis on the bottom
    %spheroPos = [spheroPos(1, :); (ones(1, N)*2.4-spheroPos(2, :))];

    t_formation = toc;
    
    agentError = positionRef - spheroPos;
    
    tic
    controlAngle = calculateAngle(spheroPos, agentError, movementThreshold);
    t_direction = toc;
    
    tic
    %PID Controller used to track the reference velocity input
    [controlVelocity, pVelocityOut, iVelocityOut, dVelocityOut] =...
        PIDVelocityController(agentError, delta_t, velocity_PID_gains, saturation);
    t_controller = toc;
    %% communication
    tic
    %send the roll command
    controlSpeed = hypot(controlVelocity(1), controlVelocity(2));
    
    sph.Roll(controlSpeed, -controlAngle, 'normal');
        
    t_comm = toc;
%%    
    tic
    logCounter = logCounter + 1;
    
    %     logImages{mod(logCounter, MAX_LOG)} = frame;
    logNumDetections(mod(logCounter, MAX_LOG)) = numberOfDetections;
    logPosition( mod(logCounter, MAX_LOG), :, :) = spheroPos;
    %logDistance(mod(logCounter, MAX_LOG), :, :) = elementDistance;
%     logAgentAngle(mod(logCounter, MAX_LOG), :) = agentAngle;
    
    logAgentError(mod(logCounter, MAX_LOG), :, :) = agentError;
%     logPhi(mod(logCounter, MAX_LOG), :, :) = Phi;
%     logPsi(mod(logCounter, MAX_LOG), :, :) = Psi;
%     logphi(mod(logCounter, MAX_LOG), :) = phi;
%     logpsi(mod(logCounter, MAX_LOG), :) = psi';
%     logdVadP(mod(logCounter, MAX_LOG), :, :) = dVadP;
%     logdVodP(mod(logCounter, MAX_LOG), :, :) = dVodP;
    
    logPVelocity(mod(logCounter, MAX_LOG), :, :) = pVelocityOut;
    logIVelocity(mod(logCounter, MAX_LOG), :, :) = iVelocityOut;
    logDVelocity(mod(logCounter, MAX_LOG), :, :) = dVelocityOut;
    logControlVelocity(mod(logCounter, MAX_LOG), :, :) = controlVelocity;
    logControlSpeed(mod(logCounter, MAX_LOG), :) = controlSpeed;
    
%     logDesiredDirection(mod(logCounter, MAX_LOG), :) = desiredDirection;
%     logActualDirection(mod(logCounter, MAX_LOG), :) = actualDirection;
%     logPAngleOut(mod(logCounter, MAX_LOG), :) = pAngleOut;
%     logIAngleOut(mod(logCounter, MAX_LOG), :) = iAngleOut;
%     logDAngleOut(mod(logCounter, MAX_LOG), :) = dAngleOut;
    logControlAngle(mod(logCounter, MAX_LOG), :) = controlAngle();
    
    logTiming(mod(logCounter, MAX_LOG)-1, :) = timings;
    
    t_logging = toc;
    
    timings = [t_capture, t_find, t_predict, t_assign, t_update, t_display,...
        t_formation, t_direction, t_controller, t_comm, t_logging];
    delta_t = sum(timings);     %calculate complete time needed for a whole cycle
  %  pause(cycleTime - delta_t);
end
%%
stop(obj.cam);