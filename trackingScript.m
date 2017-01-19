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

%speed reference tracking controller gains: Kp, Ki, Kd
Kp = 0.003; Ki = 0; Kd = 0;
PIDgains = [Kp Ki Kd];

%initial value for threshold, smaller means more sensitive
threshold = .6;

%% SWITCHES

%Toggles whether or not to display the tracking results in Windowed Players
doDisplay = false;

%toggles between spherodetection using foreground detection and the
%detection using a threshold and an areafilter
foregroundSubtraction = false;

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
if foregroundSubtraction
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

%timings of the individual processes
% 1: time needed for acquiring an image
% 2: time needed for detecting spheros inside this image
% 3: time needed for the prediction of future positions
% 4: time needed to assign current detections to existing tracks
% 5: time needed to update the tracks with the current positions
% 6: time needed to display the detections if enabled
% 7: time needed to calculate the reference speed value for each Sphero
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
logFormationError = zeros(MAX_LOG, 2, N);
logHeading = zeros(MAX_LOG, N);
logDistance = zeros(MAX_LOG, N, N+M);
logAngle = zeros(MAX_LOG, N) ;
logPhi = zeros(MAX_LOG, 2, N);
logPsi = zeros(MAX_LOG, 2, N) ;
logphi = zeros(MAX_LOG, N);
logpsi = zeros(MAX_LOG, N);
logdVadP = zeros( MAX_LOG, 2, N);
logdVodP = zeros( MAX_LOG, 2, N);
logControlSpeed = zeros(MAX_LOG, 2, N);
logProportional = zeros(MAX_LOG, 2, N);
logIntegral = zeros(MAX_LOG, 2,  N);
logDerivative = zeros(MAX_LOG, 2, N);
logTiming = zeros(MAX_LOG, 11);

%% Initial detection and association
disp('Initial detection...');
frame = nextFrame(obj.cam, isWebcam);
%to avoid false recognitions
[centers, noDetections, bboxes, mask] = findSpheroBB(frame, threshold);
while noDetections < N
    %imshow(mask);
    threshold = threshold -.01; %makes it more sensitive
    disp('Attempting initial detection, searching for Spheros, decreasing threshold');
    frame = nextFrame(obj.cam, isWebcam);
    [centers, noDetections, bboxes, mask] = findSpheroBB(frame, threshold);
   % pause();
end
threshold = threshold - 0.01;

% associate hardware with tracking
success = false;
while ~success
    disp('Associate Spheros...');
    [association, success] = associateSpheros(spheros, centers, obj.cam);
    if ~success
        disp('Could not associate bluetooth object with spheros')
        continue
    end
end

% create Tracks for tracking
tracks = createTracks(association, bboxes, centers, spheros);

for i = 1:length(tracks)
    tracks(i).Sphero.SetRGBLEDOutput([1 1 1], false);
end

%% tracking loop
disp('Starting Tracking...')

while true
    tic
    frame = nextFrame(obj.cam, isWebcam); 
    t_capture = toc;
    
    tic
    %detecting the spheros in the image
    if foregroundSubtraction
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
        predictedCentroid = predict(tracks(i).kalmanFilter);        
    end
    t_predict = toc;
    
    tic
    %assign detections to best fitting tracks
    costOfNonAssignment = 20;
    matches = assignTracks(tracks, centroids, costOfNonAssignment);
    t_assign = toc;
    
    tic;
    for i = 1:size(matches, 1)
        trackIndex = matches(i, 1);
        detectionIndex = matches(i, 2);
        centroid = centroids(detectionIndex, :);
        
        %correct the prediction with the measurement
        correct(tracks(trackIndex).kalmanFilter, centroid);
        
        %replace prediction with measurement
        %tracks(trackIndex).lastCentroid = tracks(trackIndex).centroid;
        tracks(trackIndex).centroid = centroid;
        
        % Update track's age.
        tracks(trackIndex).age = tracks(trackIndex).age + 1;
        
        % Update visibility.
        tracks(trackIndex).totalVisibleCount = tracks(trackIndex).totalVisibleCount + 1;
        tracks(trackIndex).consecutiveInvisibleCount = 0;
                
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
    centroids = vertcat(tracks(:).centroid);
    
    [formationError, d, angle, Phi, Psi, dVadP, dVodP, phi, psi] = ...
        formationController(centroids', obstacles, positionRef, angleRef,...
        r, R, formationGains, doOrientation);
    t_formation = toc;
    
    tic
    %calculating angle. IMPORTANT: negative!!!
    desiredAngle = atan2d (formationError(2,:),formationError(1, :));
    heading = arrayfun(@(angle) (wrapTo360(angle)),-desiredAngle);
    
    %TODO not used
    %     %calculate which Sphero have moved from the last samples. Used to
    %     %cancel out small errors in detection.
    
    %     last_centroids = vertcat(lastTracks(:).centroid);
    %     displacement = centroids - last_centroids;
    %     hasMoved = sqrt(sum(displacement.^2, 2)); %not optimal : z=sqrt(x'*x);
    %     hasMoved = hasMoved > 15;
    %     %calculate current rolling direction, wrap to used angle convention,
    %     %and add an offsetangle(90°) to match the Spheros anglesystem
    %
    %     rawDirection = atan2d(yDisplacement,xDisplacement); %calculates the unwrapped direction of movement (-180°, 180°)
    %     direction = arrayfun(@(ang)(wrapTo360(ang)),rawDirection); %.* hasMoved
    %     %     direction = direction + (~hasMoved) * direction_old;
    %     %     direction_old = direction;
    %     %     theta = direction + omega * ( 360 / (2 * pi)) * delta_t;
    %     %PID controller for the reference tracking of the heading angle
    %     %[dir, previous_angles] = controller( theta_desired_old, previous_angles, delta_t, K_angle);
    %
    t_direction = toc;
    
    tic
    %PID Controller used to track the reference speed input
    [controlSpeed, proportional, integral, derivative] = PIDController(formationError,...
        delta_t, PIDgains);
    t_controller = toc;
    %% communication
    tic
    %send the roll command
    if logCounter > 2
        for i = 1:N
            tracks(i).Sphero.Roll(norm(controlSpeed(:, i)), heading(i), 'normal');
        end
    end
    t_comm = toc;
    
    tic
    logCounter = logCounter + 1;
    
    %logImages{mod(logCounter, MAX_LOG)} = frame;
    logNumDetections(mod(logCounter, MAX_LOG)) = numberOfDetections;
    logPosition( mod(logCounter, MAX_LOG), :, :) = centroids;
    logFormationError(mod(logCounter, MAX_LOG), :, :) = formationError;
    logDistance(mod(logCounter, MAX_LOG), :, :) = d;
    logAngle(mod(logCounter, MAX_LOG), :) = angle;
    logPhi(mod(logCounter, MAX_LOG), :, :) = Phi;
    logPsi(mod(logCounter, MAX_LOG), :, :) = Psi;
    logphi(mod(logCounter, MAX_LOG), :) = phi;
    logpsi(mod(logCounter, MAX_LOG), :) = psi';
    logdVadP(mod(logCounter, MAX_LOG), :, :) = dVadP;
    logdVodP(mod(logCounter, MAX_LOG), :, :) = dVodP;
    logHeading(mod(logCounter, MAX_LOG), :) = heading;
    logControlSpeed(mod(logCounter, MAX_LOG), :, :) = controlSpeed;
    logProportional(mod(logCounter, MAX_LOG), :, :) = proportional;
    logIntegral(mod(logCounter, MAX_LOG), :, :) = integral;
    logDerivative(mod(logCounter, MAX_LOG), :, :) = derivative;
    logTiming(mod(logCounter, MAX_LOG)-1, :) = timings;
    
    t_logging = toc;
    
    timings = [t_capture, t_find, t_predict, t_assign, t_update, t_display,...
        t_formation, t_direction, t_controller, t_comm, t_logging];
    delta_t = sum(timings);     %calculate complete time needed for a whole cycle
    
end