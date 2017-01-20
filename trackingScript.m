%% Initialization and user input
delete (instrfindall); %delete all instruments
clear all
clc
disp('Starting...')
spheros  = connectSpheros();

%% CONSTANTS
disp('Initializing constants...')

N = length(spheros); %number of robots

%the communication laplacian:
%Create a symmetric adjacency matrix, A, that creates a complete
%graph of order N.
% A = ones(N) - eye(N);
% G = graph(A);
% L = full(G.laplacian);

meterPerPixel = 2e-3; %transfer from pixel to meters

%the detection and avoidance region
R = 0.2 / meterPerPixel;
r = 0.05 / meterPerPixel;

%timesteps, just initial value
delta_t = 0.1;

%formation control parameters
k_a = 0; % angle error gain
k_v = 0.05; % distance error gain
k_o = 0; % obstacle error gain
formationGains = [k_a, k_v, k_o];

%position reference for each agent [2*N]
switch N
    case 1
        positionRef = [1 1]'/ meterPerPixel;
    case 2
        positionRef = [[1 1]; [0.9 0.9]]'/ meterPerPixel;
    case 3
        positionRef = [[0 0]; [0.5 0.5]; [1 1]]'/ meterPerPixel;
    case 4
        positionRef = [[0 0]; [0.5 0.5]; [1 1]; [1.5 1.5]]'/ meterPerPixel;
end

%angle reference for agents [1*N]
angleRef = zeros(1, N);
angleRef(angleRef == 0) = NaN;

%static obstacles to prove collision avoidance [2*M]
obstacles =  [ ];%(0.75 0.75)]'; %(1.25 1.25)]'./ meterPerPixel;
M = size(obstacles, 2);

%velocity reference tracking controller gains: Kp, Ki, Kd
velocityKp = 0.003; velocityKi = 0; velocityKd = 0;
velocity_PID_gains = [velocityKp velocityKi velocityKd];

angleKp = 0.06; angleKi = 0; angleKd = 0.00\0;
angle_PID_gains =[angleKp angleKi angleKd];

%initial value for threshold, smaller means more sensitive
threshold = .67;
spheroPos = zeros(2, N);
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

% associate hardware with tracking
success = false;
while ~success
    disp('Associate Spheros...');
    [association, success] = associateSpheros(spheros, centroids, obj.cam);
    if ~success
        disp('Could not associate bluetooth object with spheros')
        continue
    end
end

% create Tracks for tracking
tracks = createTracks(association, centroids, spheros);

for i = 1:length(tracks)
    tracks(i).Sphero.SetRGBLEDOutput([1 1 1], false);
end

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
    %predict the locations of the robots
    for i = 1:length(tracks)
        predict(tracks(i).kalmanFilter);
    end
    t_predict = toc;
    
    tic
    %assign detections to best fitting tracks
    matches = assignTracks(tracks, centroids);
    t_assign = toc;
    
    tic;
    for i = 1:size(matches, 1)
        trackIndex = matches(i, 1);
        detectionIndex = matches(i, 2);
        
        %correct the prediction with the measurement
        correct(tracks(trackIndex).kalmanFilter, centroids(detectionIndex, :));
        
        %replace prediction with measurement
        tracks(trackIndex).centroid = centroids(detectionIndex, :);
        
    end
    t_update = toc;
    
    tic;
    %Display the results of detection
    if doDisplay
        image(frame);
    end
    t_display = toc;
    
    %% formation control    
    tic
    lastSpheroPos = spheroPos;
    spheroPos = vertcat(tracks(:).centroid)';
    displacement = spheroPos - lastSpheroPos;
    
    [agentError, elementDistance, agentAngle, Phi, Psi, dVadP, dVodP, phi, psi] = ...
        formationController(spheroPos, obstacles, positionRef, angleRef,...
        r, R, formationGains, doOrientation);
    t_formation = toc;
    
    tic
    desiredDirection = atan2d (agentError(2,:), agentError(1, :));
    actualDirection = atan2d (displacement(2,:), displacement(1,:));
    agentAngleError = desiredDirection - actualDirection;
    [controlAngle, pAngleOut, iAngleOut, dAngleOut] = PIDAngleController(...
        agentAngleError, delta_t, angle_PID_gains);
    controlAngle = controlAngle(1, :);
    %controlAngle = arrayfun(@(angle) (wrapTo360(angle)), controlAngle);
    t_direction = toc;
    
    tic
    %PID Controller used to track the reference velocity input
    [controlVelocity, pVelocityOut, iVelocityOut, dVelocityOut] =...
        PIDVelocityController(agentError, delta_t, velocity_PID_gains);
    t_controller = toc;
    %% communication
    tic
    %send the roll command
    controlSpeed = hypot(controlVelocity(1, :), controlVelocity(2, :));
    if logCounter > 2
        for i = 1:N
            tracks(i).Sphero.Roll(controlSpeed(i), controlAngle(i), 'normal');
        end
    end
    t_comm = toc;
    
    tic
    logCounter = logCounter + 1;
    
%     logImages{mod(logCounter, MAX_LOG)} = frame;
    logNumDetections(mod(logCounter, MAX_LOG)) = numberOfDetections;
    logPosition( mod(logCounter, MAX_LOG), :, :) = spheroPos;
    logDistance(mod(logCounter, MAX_LOG), :, :) = elementDistance;
    logAgentAngle(mod(logCounter, MAX_LOG), :) = agentAngle;    
    
    logAgentError(mod(logCounter, MAX_LOG), :, :) = agentError;
    logPhi(mod(logCounter, MAX_LOG), :, :) = Phi;    
    logPsi(mod(logCounter, MAX_LOG), :, :) = Psi;
    logphi(mod(logCounter, MAX_LOG), :) = phi;
    logpsi(mod(logCounter, MAX_LOG), :) = psi';
    logdVadP(mod(logCounter, MAX_LOG), :, :) = dVadP;
    logdVodP(mod(logCounter, MAX_LOG), :, :) = dVodP;

    logPVelocity(mod(logCounter, MAX_LOG), :, :) = pVelocityOut;
    logIVelocity(mod(logCounter, MAX_LOG), :, :) = iVelocityOut;
    logDVelocity(mod(logCounter, MAX_LOG), :, :) = dVelocityOut; 
    logControlVelocity(mod(logCounter, MAX_LOG), :, :) = controlVelocity;
    logControlSpeed(mod(logCounter, MAX_LOG), :) = controlSpeed;
    
    logDesiredDirection(mod(logCounter, MAX_LOG), :) = desiredDirection;
    logActualDirection(mod(logCounter, MAX_LOG), :) = actualDirection;
    logPAngleOut(mod(logCounter, MAX_LOG), :) = pAngleOut;
    logIAngleOut(mod(logCounter, MAX_LOG), :) = iAngleOut;
    logDAngleOut(mod(logCounter, MAX_LOG), :) = dAngleOut;
    logControlAngle(mod(logCounter, MAX_LOG), :) = controlAngle();
    
    logTiming(mod(logCounter, MAX_LOG)-1, :) = timings;
    
    t_logging = toc;
    
    timings = [t_capture, t_find, t_predict, t_assign, t_update, t_display,...
        t_formation, t_direction, t_controller, t_comm, t_logging];
    delta_t = sum(timings);     %calculate complete time needed for a whole cycle
    
end