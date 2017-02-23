%% Initialization and user input
clc;
delete (instrfindall); %delete all instruments
clear all;
close all;
disp('Starting...')
spheros  = connectSpheros();

%% CONSTANTS
disp('Initializing constants...')

N = length(spheros); %number of robots
meterPerPixel = 2e-3; %transfer from pixel to meters

%the detection and avoidance region
R = 0.3;
r = 0.1;

%timesteps, just initial value
delta_t = 0.1;

%formation control parameters
k_d = 0.5; % distance error gain
k_a = 0.5; % angle error gain
k_oa = 0; % obstacle avoidance 
k_o = 0; %orientation
k_r = 0; %reference tracking

beta = 45;
Vr = [cosd(beta); sind(beta)];
formationGains = [k_d, k_a, k_oa, k_o, k_r];
%PID tuning
Kp = 0.3; Ki = 0; Kd = 0;
PIDgains = [Kp Ki Kd];
saturation = 0.1;
flag = 'fast';

%position reference for each agent [2*N]
p1 = [1;1];
p2 = [2;2];
positionRef = [p1, p2];
distanceRef = [NaN NaN NaN];

%angle reference for agents [1*N], NaN for no reference for that agent
angleRef = [60 60 60];

%static obstacles to prove collision avoidance [2*M]
obstacles =  [ ];
obstacles = horzcat(obstacles,[0 0; 3.2 2.4]'); 
M = size(obstacles, 2);

%initial value for threshold, smaller means more sensitive
movementThreshold = 0.15;
angleOffset = true;
if angleOffset
    training = true;
    agent = 1;
    offset = zeros(1, N);
end
trainingSpeed = 0.05;

%% SWITCHES
%Toggles whether or not to display the tracking results in Windowed Players
doDisplay = false;

%% VIDEO INITIALIZATIONS FOR TRACKING
imaqreset;
%camera used. Normally the index >>1<< refers to the USB-Camera
camera = videoinput('winvideo', 2, 'BYRG_1600x1200');
camera.FramesPerTrigger = 1;
% Configure the object for manual trigger mode.
triggerconfig(camera, 'manual');
camera.LoggingMode = 'memory';
start(camera);
%% VIDEO OFFSET
%o-->x   ^y
%|       |
%y   to  o-->x
resolution = camera.VideoResolution;

xRange = resolution(1)*meterPerPixel;
yRange = resolution(2)*meterPerPixel;
scaleMatrix = [xRange; yRange];

%% DATALOGGING INITIALIZATIONS
disp('Initializing logging variables...')
timings = zeros(1, 11);
MAX_LOG = 2500;%maximum size of buffers
logCounter = 1;%current write index

logNumDetections = zeros(MAX_LOG, 1);
logPosition = zeros(MAX_LOG, 2, N);

logControlVelocity = zeros(MAX_LOG, 2, N);
logphi = zeros(MAX_LOG, N);
logpsi = zeros(MAX_LOG, N);
logVo = zeros(MAX_LOG, 2);
logVa = zeros(MAX_LOG, N);
logPhi = zeros(MAX_LOG, 2, N);
logPsi = zeros(MAX_LOG, 2, N) ;
logdVadP = zeros(MAX_LOG, 2, N);
logdVodP = zeros(MAX_LOG, 2, N);

logAgentVelocity = zeros(MAX_LOG, 2, N);
logAgentSpeed = zeros(MAX_LOG, N);

logAgentHeading = zeros(MAX_LOG, N);
logDesiredHeading = zeros(MAX_LOG, N);
logActualHeading = zeros(MAX_LOG, N);

logTiming = zeros(MAX_LOG, 11);

%% Initial detection and association
disp('Initial detection...');
frame = getsnapshot(camera);
[centroids, ~] = findSpheroCentroid(frame);

%% associate hardware with tracking
success = false;
while ~success
    disp('Associate Spheros...');
    [association, success] = associateSpheros(spheros, centroids, camera);
    if ~success
        disp('Could not associate bluetooth object with spheros')
        continue
    end
end

%% create tracks
tracks = createTracks(association, centroids, spheros);

for i = 1:length(tracks) %set the color of the spheros
    tracks(i).Sphero.SetRGBLEDOutput([1 1 1], false);
end

%% tracking loop
disp('Tracking loop...')

while true
    %% Image processing
    tic
    frame = getsnapshot(camera);
    t_capture = toc;
    
    tic
    %detecting the spheros in the image
    [centroids, ~] = findSpheroCentroid(frame); 
    numberOfDetections = size(centroids, 1);    
    %error messages if the detections do not comply with the number of robots    
    if numberOfDetections > N
        disp('Too many detections')
    else
        if numberOfDetections < N
            disp('Not enough detections')
        end
    end    
    t_find = toc;
    
    tic    
    for i = 1:length(tracks) %predict the locations of the robots
        predict(tracks(i).kalmanFilter);
    end
    t_predict = toc;
    
    tic    
    matches = assignTracks(tracks, centroids);%assign detections tracks
    t_assign = toc;
    
    tic;
    for i = 1:size(matches, 1)
        trackIndex = matches(i, 1);
        detectionIndex = matches(i, 2);        
        %???correct the prediction with the measurement
        correct(tracks(trackIndex).kalmanFilter, centroids(detectionIndex, :));        
        %replace prediction with measurement
        tracks(trackIndex).centroid = centroids(detectionIndex, :);        
    end    
    spheroPos = vertcat(tracks(:).centroid)'*meterPerPixel;%transf to meters
    spheroPos(2, :) =  yRange -  spheroPos(2, :); %tranform to RHCS
    t_update = toc;
    
    tic;    
    if doDisplay %Display the results of detection
        image(frame);
    end
    t_display = toc;

    %% Formation control
    tic
    if agent > N
        training = false;
    end
    if training
        u = zeros(2, N);
        agentVelocity = zeros(2, N);        
        agentVelocity(:, agent) = [trainingSpeed; 0];
        [displacement, hasMoved] = detectMovement(spheroPos(:,agent), movementThreshold);
        if hasMoved
            agentVelocity(:, agent) = [0; 0];
            offset(agent) = atan2d (displacement(2,:), displacement(1,:));
            agent = agent + 1;
            clear detectMovement;
        end
    else
        [u, Phi, Psi, Va, dVadP, Vo, dVodP, phi, psi] = ...
            formationController(spheroPos, obstacles, distanceRef, positionRef, angleRef, Vr,...
            r, R, formationGains, scaleMatrix);
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
    
    if training
    else
        [agentVelocity, ~, ~, ~] = PIDController(u, delta_t, PIDgains);
    end
    agentSpeed = hypot(agentVelocity(1, :), agentVelocity(2, :));
    agentSpeed = min(saturation, agentSpeed);
    t_controller = toc;
    

    %% communication
    tic
    %send the roll command    
    for i = 1:N
        tracks(i).Sphero.Roll(agentSpeed(i), agentHeading(i), flag);
    end        
    t_comm = toc;
    
    %%
    tic
    logCounter = logCounter + 1;
    
    logNumDetections(mod(logCounter, MAX_LOG)) = numberOfDetections;
    logPosition( mod(logCounter, MAX_LOG), :, :) = spheroPos;

    
    logControlVelocity(mod(logCounter, MAX_LOG), :, :) = u;
    logphi(mod(logCounter, MAX_LOG), :) = phi;
    logpsi(mod(logCounter, MAX_LOG), :) = psi;
    logVo(mod(logCounter, MAX_LOG), :) = Vo;
    logVa(mod(logCounter, MAX_LOG), :) = Va;
    logPhi(mod(logCounter, MAX_LOG), :, :) = Phi;
    logPsi(mod(logCounter, MAX_LOG), :, :) = Psi;
    logdVadP(mod(logCounter, MAX_LOG), :, :) = dVadP;
    logdVodP(mod(logCounter, MAX_LOG), :, :) = dVodP;    

    logAgentVelocity(mod(logCounter, MAX_LOG), :, :) = agentVelocity;
    logAgentSpeed(mod(logCounter, MAX_LOG), :) = agentSpeed;
    
    logDesiredHeading(mod(logCounter, MAX_LOG), :) = desiredDirection;
    logActualHeading(mod(logCounter, MAX_LOG), :) = actualDirection;
    logAgentHeading(mod(logCounter, MAX_LOG), :) = agentHeading;
    
    logTiming(mod(logCounter, MAX_LOG)-1, :) = timings;
    
    t_logging = toc;
    
    timings = [t_capture, t_find, t_predict, t_assign, t_update, t_display,...
        t_formation, t_direction, t_controller, t_comm, t_logging];
    delta_t = sum(timings);     %calculate complete time needed for a whole cycle
    
end
%%
stop(camera);