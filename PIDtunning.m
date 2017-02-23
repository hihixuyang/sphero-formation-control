%% CONNECT SPHEROS (USER INPUT)
clc;
delete (instrfindall); %delete all instruments
clear all;
close all;
disp('Starting...')
sph = Sphero('Sphero-OWW');
% sph = Sphero('Sphero-RGB');
% sph = Sphero('Sphero-GBG');
N = 1;
M = 0;
sph.SetRGBLEDOutput([1 1 1], false);

sph.SetBackLEDOutput(1)
%% CONSTANTS
disp('Initializing constants...')
delta_t = 0.1;
%velocity reference tracking controller gains: Kp, Ki, Kd
Kp = 0.3; Ki = 0; Kd = 0;
PIDgains = [Kp Ki Kd];

meterPerPixel = 2e-3;
spheroPos = zeros(2, N);
movementThreshold = 0.15;
saturation = 0.1;
positionRef1 = [3 2.2]'; %in meters
positionRef2 = [2 2]';
distanceRef = 0.2;
angleRef = 90;
flag = 'fast';
mode = 1; %1 tunning, 2 distance, 3 angle, 4 combination, 5 combination+obstacle, 6 orientation+obstacle
% 7 tracking
initialMode = mode;
Ka=0.5; Kd=2; Ko=0.5;

obstaclePos = [2 1]';
r = 0.05;
R = 0.3;

angleOffset = true;
hasMoved = false;
desiredHeading = 0;
actualHeading = 0;
offset = 0;
if angleOffset
    angleTraining = true;
else
    angleTraining = false;
end;
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

%% VIDEO OFFSET
%o-->x   ^y
%|       |
%y   to  o-->x
resolution = obj.cam.VideoResolution;

xRange = resolution(1)*meterPerPixel;
yRange = resolution(2)*meterPerPixel;
scaleMatrix = [xRange; yRange];

if mode == 7
    movingRef = zeros(2, 2500);
    movingRef(:,1) = positionRef1;
    center = scaleMatrix/2;
    radius = 1;
    alpha = 0;
    rate = 5; %angle (degrees) change rate per second;
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
psi = zeros(1, N);
phi = zeros(1, N);
Va = zeros(1, N);
Vo = zeros(1, N);

Phi = zeros(2, N);
Psi = zeros(2, N);
dVadP = zeros(2, N);
dVodP = zeros(2, N);

agentVelocity = [0 0]';

timings = zeros(1, 11);

%maximum size of buffers
MAX_LOG = 2500;

%current write index
logCounter = 1;

logPosition = zeros(MAX_LOG, 2, N);

logphi = zeros(MAX_LOG, N);
logpsi = zeros(MAX_LOG, N);
logVa = zeros(MAX_LOG, N);
logVo = zeros(MAX_LOG, N);

logPhi = zeros(MAX_LOG, 2, N);
logPsi = zeros(MAX_LOG, 2, N);
logdVadP = zeros(MAX_LOG, 2, N);
logdVodP = zeros(MAX_LOG, 2, 2);

logControlVelocity = zeros(MAX_LOG, 2, N);
logAgentVelocity = zeros(MAX_LOG, 2, N);
logAgentSpeed = zeros(MAX_LOG, N);

logAgentHeading = zeros(MAX_LOG, N);
logDesiredHeading = zeros(MAX_LOG, N);

logTiming = zeros(MAX_LOG, 11);

%% Initial detection and association
disp('Initial detection...');
frame = nextFrame(obj.cam, isWebcam);
[centroids, ~] = findSpheroCentroid(frame);


%% TRACKING LOOP
disp('Tracking loop...')

while true
    tic
    frame = nextFrame(obj.cam, isWebcam);
    t_capture = toc;
    
    tic
    %detecting the spheros in the image
    if doForegroundSubtraction
        %calculate foreground mask. mask is a binary image.
        %this routine is normally turned off in favour of the thresholding
        %approach which has better performance if the Spheros do not move or
        %only move very little
        mask = obj.detector.step(frame);
        % Apply morphological operations to remove noise and fill in holes.
        mask = imopen(mask, strel('rectangle', [3,3]));
        mask = imclose(mask, strel('rectangle', [15, 15]));
        mask = imfill(mask, 'holes');
        %Perform blobanalysis on binary image and find connected components.
        [~, centroids, ~] = obj.blobAnalyser.step(mask);
    else
        [centroids, ~] = findSpheroCentroid(frame);
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
    %stack the positions in meters in a [2*N] matrix, as in the paper
    spheroPos = centroids'*meterPerPixel; % transform to meters
    spheroPos(2, :) =  yRange -  spheroPos(2, :); % tranform to RHCS
    t_update = toc;
    tic;
    %Display the results of detection
    if doDisplay
        image(frame);
    end
    t_display = toc;
    
    %% FORMATION AND LOCAL CONTROLLERS
    tic
    if angleTraining
        [displacement, hasMoved] = detectMovement(spheroPos, movementThreshold);
        if ~hasMoved
            mode = 0;
        else
            sph.Roll(0, 0, flag);
            mode = initialMode;
            offset = atan2d (displacement(2,:), displacement(1,:));
            angleTraining = false;
        end
    end
    
    switch mode
        case 0  % ANGLE OFFSET
            u = [0.05 0]';
        case 1
            positionError = positionRef1 - spheroPos;
            u = positionError./[xRange; yRange]; %scales error to [0..1]
            
        case 2
            [psi, Psi] = distanceErrorGradient(distanceRef, spheroPos, positionRef1, scaleMatrix);
            u = Psi;
            
        case 3
            [phi, Phi] = angleErrorGradient(angleRef, spheroPos, positionRef1, positionRef2);
            u = Phi;
        case 4
            [phi, Phi] = angleErrorGradient(angleRef, spheroPos, positionRef1, positionRef2);
            [psi, Psi] = distanceErrorGradient(distanceRef, spheroPos, positionRef1, scaleMatrix);
            
            u = Kd*Psi+Ka*Phi;
        case 5
            [phi, Phi]= angleErrorGradient(angleRef, spheroPos, positionRef1, positionRef2);
            [psi, Psi] = distanceErrorGradient(distanceRef, spheroPos, positionRef1, scaleMatrix);
            [Va,  dVadP ] = avoidanceFunctionDerivative(spheroPos, obstaclePos,r, R);
            
            u = Kd*Psi+Ka*Phi+Ko*dVadP;
        case 6
            positionRef = [positionRef1 positionRef2];
            spheroPosOrientation = [spheroPos(:, 1) positionRef2];
            [Vo, dVodP ] = orientationControl(positionRef, spheroPosOrientation );
            dVodP = dVodP (:, 1);
            [Va,  dVadP ] = avoidanceFunctionDerivative(spheroPos, obstaclePos,r, R);
            
            u = Ka*dVodP + Ko*dVadP;
        case 7
            alpha = alpha + rate*delta_t;
            movingRef(:,logCounter) = movingRef(:,logCounter-1) + [1;1]*0.1*delta_t;
            [psi, Psi] = distanceErrorGradient(distanceRef, spheroPos, movingRef(:,logCounter), scaleMatrix);
            u = Psi;
    end
    t_formation = toc;
    
    tic
    if angleOffset
        desiredHeading = atan2d (u(2,:), u(1,:));
        agentHeading = desiredHeading - offset;
    else
        [agentHeading, desiredHeading, actualHeading] =...
            calculateAngle(spheroPos, u, movementThreshold);
    end
    t_direction = toc;
    
    tic
    if angleTraining
        agentSpeed = hypot(u(1), u(2));
    else
        [agentVelocity, ~, ~, ~] = PIDController(u, delta_t, PIDgains);
        agentSpeed = hypot(agentVelocity(1), agentVelocity(2));
        agentSpeed = min(saturation, agentSpeed);
    end
    t_controller = toc;
    %% COMMUNICATION
    tic
    sph.Roll(agentSpeed, agentHeading, flag);
    t_comm = toc;
    %% DATALOGGING
    tic
    logCounter = logCounter + 1;
    
    logPosition( mod(logCounter, MAX_LOG), :, :) = spheroPos;
    
    logphi(mod(logCounter, MAX_LOG), :) = phi;
    logpsi(mod(logCounter, MAX_LOG), :) = psi;
    logVa(mod(logCounter, MAX_LOG), :) = Va;
    logVo(mod(logCounter, MAX_LOG), :) = Vo;
    
    logPhi(mod(logCounter, MAX_LOG), :, :) = Phi;
    logPsi(mod(logCounter, MAX_LOG), :, :) = Psi;
    logdVadP(mod(logCounter, MAX_LOG), :, :) = dVadP;
    logdVodP(mod(logCounter, MAX_LOG), :, :) = dVodP;
    
    logControlVelocity(mod(logCounter, MAX_LOG), :, :) = u;    
    logAgentVelocity(mod(logCounter, MAX_LOG), :, :) = agentVelocity;
    logAgentSpeed(mod(logCounter, MAX_LOG), :) = agentSpeed;
    
    logDesiredHeading(mod(logCounter, MAX_LOG), :) = desiredHeading;
    logAgentHeading(mod(logCounter, MAX_LOG), :) = agentHeading;
    
    logTiming(mod(logCounter, MAX_LOG)-1, :) = timings;
    
    t_logging = toc;
    
    timings = [t_capture, t_find, t_predict, t_assign, t_update, t_display,...
        t_formation, t_direction, t_controller, t_comm, t_logging];
    delta_t = sum(timings);     %calculate complete time needed for a whole cycle
end
%%
stop(obj.cam);