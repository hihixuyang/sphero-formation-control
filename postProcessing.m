clear;
close all;
load('PID31jan9');
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

x2 = logPosition(:, 1, 2);
y2 = logPosition(:, 2, 2);

u1 = logControlSpeed(:, 1);
u2 = logControlSpeed(:, 2);

ex1 = logAgentError(:, 1, 1);
ey1 = logAgentError(:, 2, 1);
e1 = ex1.^2 + ey1.^2;

ex2 = logAgentError(:, 1, 2);
ey2 = logAgentError(:, 2, 2);
e2 = ex2.^2 + ey2.^2;

desiredDir1 = logDesiredDirection(:, 1);
desiredDir2 = logDesiredDirection(:, 2);

actualDir1 = logActualDirection(:, 1);
actualDir2 = logActualDirection(:, 2);

controlAngle1 = logControlAngle(:, 1);
controlAngle2 = logControlAngle(:, 2);

de1 = logpsi(:, 1);
de2 = logpsi(:, 2);

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
figure('Name','control effort','NumberTitle','off')
ax1 = subplot(2,1,1);
ax2 = subplot(2,1,2);
plot(ax1, duration, u1);
title(ax1,'agent1');
plot(ax2, duration, u2);
title(ax2, 'agent2');

%%
%figure(2);
figure('Name','formation error','NumberTitle','off')
ax3 = subplot(2,1,1);
ax4 = subplot(2,1,2);
plot(ax3, duration, e1);
title(ax3,'agent1');
plot(ax4, duration, e2);
title(ax4, 'agent2');
% figure(1);
% %ax1 = subplot(3,1,1);
% ax2 = subplot(2,1,1);
% ax3 = subplot(2,1,2);
% 
% %set(ax2,'Ydir','reverse')
% 
% %(:, 1, 1) first coordinate for first agent
% % plot(ax1, duration , logPSpeed(:, 1, 1), 'g', duration, logISpeed(:, 1, 1), 'r',...
% %  duration, logDSpeed(:, 1, 1) ,'b', duration, logControlSpeed(:, 1, 1), 'k');
% % title(ax1, 'PID Tunning')
% % xlabel(ax1, duration)
% % legend('P','I', 'D', 'control');
% 
% plot(ax2, x1, y1, 'r', x1(end), y1(end),'ro', positionRef(1,1), positionRef(2,1), 'r*');
% title(ax2, 'Phase-plane diagram for 1st agent')
% xlabel(ax2, 'x')
% ylabel(ax2, 'y')
% axis(ax2, [0, 1200, 0, 1600]);
% set(ax2,'YAxisLocation','right');
% view(ax2, 90,90); %rotate the plot 90 deg CW to suit the image of the cameara
% 
% 
% plot(ax3, x2, y2, 'b', x2(end), y2(end),'bo', positionRef(1,2), positionRef(2,2), 'b*');
% title(ax3, 'Phase-plane diagram for 2nd agent')
% xlabel(ax3, 'x')
% ylabel(ax3, 'y')
% axis(ax3, [0, 1200, 0, 1600]);
% set(ax3,'YAxisLocation','right');
% view(ax3, 90,90);
%%
%figure(3);
figure('Name','position errors','NumberTitle','off')
ax5 = subplot(2,1,1);
ax6 = subplot(2,1,2);
plot(ax5, duration, logAgentError(:,1, 1), duration, logAgentError(:,1, 2));
title(ax5, 'x error')
plot(ax6, duration, logAgentError(:, 2, 1), duration, logAgentError(:,2, 2));
title(ax6, 'y error')
%%
figure('Name','agent position and target','NumberTitle','off')
plot(x1, y1, 'r', x1(end), y1(end),'ro', positionRef(1,1), positionRef(2,1), 'r*',...
     x2, y2, 'b', x2(end), y2(end),'bo', positionRef(1,2), positionRef(2,2), 'b*');
xlabel('x')
ylabel('y')
legend('agent1','', '', 'agent 2')
title ('agent position and target');
axis([0, 3.2, 0, 2.4]);
% set(gca, 'XAxisLocation','top');
% set(gca,'YDir','reverse');
%view(90,90)

%%
figure(5)
ax7 = subplot(2,1,1);
ax8 = subplot(2,1,2);
plot(ax7, duration, de1);
title(ax7, 'agent 1 distance error');
plot(ax8, duration, de2);
title(ax8, 'agent 2 distance error');
%%
figure('Name','angles','NumberTitle','off')
ax9 = subplot(2,1,1);
ax10 = subplot(2,1,2);
plot(ax9, duration, desiredDir1,duration,actualDir1,duration, controlAngle1);
title(ax9,'agent1');
legend('desired angle','actual angle', 'control angle')
plot(ax10, duration, desiredDir2,duration,actualDir2,duration, controlAngle2);
title(ax10, 'agent2');
legend('desired angle','actual angle', 'control angle')