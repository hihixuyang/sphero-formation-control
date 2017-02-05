clear;
close all;
load('PID1feb13');
%%
logPcontrolSpeed = hypot(logPVelocity(:, 1), logPVelocity(:, 2));
logIcontrolSpeed = hypot(logIVelocity(:, 1), logIVelocity(:, 2));
logDcontrolSpeed = hypot(logDVelocity(:, 1), logDVelocity(:, 2));
%% Load and remove zeros from the data

logCounter = logCounter - 2;

%eliminate the first row which is 0
logImages = circshift(logImages, -1);
logNumDetections = circshift(logNumDetections, -1);
logPosition = circshift(logPosition, -1);
logDistance = circshift(logDistance, -1);
logAgentAngle = circshift(logAgentAngle, -1);

logAgentError = circshift(logAgentError, -1);
logPhi = circshift(logPhi, -1);
logPsi = circshift(logPsi, -1);
logphi = circshift(logphi, -1);
logpsi = circshift(logpsi, -1);
logdVadP = circshift(logdVadP, -1);
logdVodP = circshift(logdVodP, -1);

logPVelocity = circshift(logPVelocity, -1);
logIVelocity = circshift(logIVelocity, -1);
logDVelocity = circshift(logDVelocity, -1);
logControlVelocity = circshift(logControlVelocity, -1);
logControlSpeed = circshift(logControlSpeed, -1);

logPcontrolSpeed = circshift(logPcontrolSpeed, -1);
logIcontrolSpeed = circshift(logIcontrolSpeed, -1);
logDcontrolSpeed = circshift(logDcontrolSpeed, -1);

logDesiredDirection = circshift(logDesiredDirection, -1);
logActualDirection = circshift(logActualDirection, -1);
logPAngleOut = circshift(logPAngleOut, -1);
logIAngleOut = circshift(logIAngleOut, -1);
logDAngleOut = circshift(logDAngleOut, -1);
logControlAngle = circshift(logControlAngle, -1);

logTiming = circshift(logTiming, -1);
%%
logImages = logImages(1:logCounter, :);
logNumDetections = logNumDetections(1:logCounter, :);
logPosition = logPosition(1:logCounter, :, :);
logDistance = logDistance(1:logCounter, :, :);
logAgentAngle = logAgentAngle(1:logCounter, :);

logAgentError = logAgentError(1:logCounter, :, :);
logPhi = logPhi(1:logCounter, :, :);
logPsi = logPsi(1:logCounter, :, :);
logphi = logphi(1:logCounter, :);
logpsi = logpsi(1:logCounter, :);%distance error
logdVadP = logdVadP( 1:logCounter, :, :);
logdVodP = logdVodP( 1:logCounter, :, :);

logPVelocity = logPVelocity(1:logCounter, :, :);
logIVelocity = logIVelocity(1:logCounter, :, :);
logDVelocity = logDVelocity(1:logCounter, :, :);
logControlVelocity = logControlVelocity(1:logCounter, :, :);
logControlSpeed = logControlSpeed(1:logCounter, :);
logPcontrolSpeed = logPcontrolSpeed(1:logCounter, :);
logIcontrolSpeed = logIcontrolSpeed(1:logCounter, :);
logDcontrolSpeed = logDcontrolSpeed(1:logCounter, :);

logDesiredDirection = logDesiredDirection(1:logCounter, :);
logActualDirection = logActualDirection(1:logCounter, :);
logPAngleOut = logPAngleOut(1:logCounter, :);
logIAngleOut = logIAngleOut(1:logCounter, :);
logDAngleOut = logDAngleOut(1:logCounter, :);
logControlAngle = logControlAngle(1:logCounter, :);

logTiming = logTiming(1:logCounter, :);
logTiming(:, 12) = sum(logTiming, 2); %delta_t for each step

duration = cumsum(logTiming(:, 12));
%% distribution of sampling times
cycletime = logTiming(:, 12);
meanCycle = mean(cycletime)
histogram(cycletime, 100)
title('Histogram on cycle time')
hold on
%print -deps epsFig
trackingTime = logTiming(:, 1) + logTiming(:, 2);
meanTracking = mean(trackingTime)
histogram(trackingTime, 100);
meanTracking/meanCycle

%%
x1 = logPosition(:, 1, 1);
y1 = logPosition(:, 2, 1);
u1 = logControlSpeed(:, 1);
up = logPcontrolSpeed ;
ui = logIcontrolSpeed ;
ud = logDcontrolSpeed ;
ex1 = logAgentError(:, 1, 1);
ey1 = logAgentError(:, 2, 1);
e1 = ex1.^2 + ey1.^2;
desiredDir1 = logDesiredDirection(:, 1);
actualDir1 = logActualDirection(:, 1);
controlAngle1 = logControlAngle(:, 1);

de1 = logpsi(:, 1);

%% PID Tunning
figure('Name','agent position and target','NumberTitle','off')
plot(x1, y1, 'r', x1(end), y1(end),'ro', positionRef(1,1), positionRef(2,1), 'r*');
xlabel('x')
ylabel('y')
legend('agent1')
title ('agent position and target');
axis([0, 3.2, 0, 2.4]);
set(gca, 'XAxisLocation','top');
set(gca,'YDir','reverse');
%%
%figure(1);
figure('Name','control output and error','NumberTitle','off')
subplot(2,1,1);
plot(duration, u1, duration, up, duration, ui, duration, ud);
legend('control output', 'P', 'I', 'D');
title('PID output');
subplot(2, 1, 2);
plot(duration, e1);
legend ('error');
title('squared position error');

%%
%figure(3);
figure('Name','position errors','NumberTitle','off')
plot(duration, ex1, duration, ey1);
title('errors')

%%
figure('Name','position errors vs control input on x','NumberTitle','off')
plot(duration, ex1, duration, logControlVelocity(:, 1), duration, ...
    logPVelocity(:, 1), duration,logIVelocity(:, 1), duration, logDVelocity(:, 1));
title('errors vs control input x')
legend('error', 'control output', 'up', 'ui', 'ud')

%%
figure('Name','position errors vs control input on y','NumberTitle','off')
plot(duration, ey1, duration, logControlVelocity(:, 2), duration, ...
    logPVelocity(:, 2), duration,logIVelocity(:, 2), duration, logDVelocity(:, 2));
title('errors vs control input on y')
legend('error', 'control output', 'up', 'ui', 'ud');
%%
figure('Name','angles','NumberTitle','off')
plot(duration, desiredDir1,duration,actualDir1,duration, controlAngle1);
title('agent1');
legend('desired angle','actual angle', 'control angle')
%%
tilefigs;
