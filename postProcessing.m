clear;
close all;
load('jan18test4');
%% Load and remove zeros from the data

logCounter = logCounter - 2;

%eliminate the first row which is 0
logImages = circshift(logImages, -1);
logNumDetections = circshift(logNumDetections, -1);
logPosition = circshift(logPosition, -1);
logFormationError = circshift(logFormationError, -1);
logHeading = circshift(logHeading, -1);
logDistance = circshift(logDistance, -1);
logAngle = circshift(logAngle, -1);
logPhi = circshift(logPhi, -1);
logPsi = circshift(logPsi, -1);
logphi = circshift(logphi, -1);
logpsi = circshift(logpsi, -1);
logdVadP = circshift(logdVadP, -1);
logdVodP = circshift(logdVodP, -1);
logControlSpeed = circshift(logControlSpeed, -1);
logProportional = circshift(logProportional, -1);
logIntegral = circshift(logIntegral, -1);
logDerivative = circshift(logDerivative, -1);
logTiming = circshift(logTiming, -1);

logImages = logImages(1:logCounter, :);
logNumDetections = logNumDetections(1:logCounter, :);
logPosition = logPosition(1:logCounter, :, :);
logFormationError = logFormationError(1:logCounter, :, :);
logHeading = logHeading(1:logCounter, :);
logDistance = logDistance(1:logCounter, :, :);
logAngle = logAngle(1:logCounter, :);
logPhi = logPhi(1:logCounter, :, :);
logPsi = logPsi(1:logCounter, :, :);
logphi = logphi(1:logCounter, :);
logpsi = logpsi(1:logCounter, :);%distance error
logdVadP = logdVadP( 1:logCounter, :, :);
logdVodP = logdVodP( 1:logCounter, :, :);
logControlSpeed = logControlSpeed(1:logCounter, :, :);
logProportional = logProportional(1:logCounter, :, :);
logIntegral = logIntegral(1:logCounter, :,  :);
logDerivative = logDerivative(1:logCounter, :, :);
logTiming = logTiming(1:logCounter, :);

logTiming(:, 12) = sum(logTiming, 2); %delta_t for each step

duration = cumsum(logTiming(:, 12));

%%
x1 = logPosition(:, 1, 1);
y1 = logPosition(:, 2, 1);

x2 = logPosition(:, 1, 2);
y2 = logPosition(:, 2, 2);

ux1 = logControlSpeed(:, 1, 1);
uy1 = logControlSpeed(:, 2, 1);
u1 = ux1.^2 + uy1.^2;

ux2 = logControlSpeed(:, 1, 2);
uy2 = logControlSpeed(:, 2, 2);
u2 = ux2.^2 + uy2.^2;

ex1 = logFormationError(:, 1, 1);
ey1 = logFormationError(:, 2, 1);
e1 = ex1.^2 + ey1.^2;

ex2 = logFormationError(:, 1, 2);
ey2 = logFormationError(:, 2, 2);
e2 = ex2.^2 + ey2.^2;

de1 = logpsi(:, 1);
de2 = logpsi(:, 2);

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
% % plot(ax1, duration , logProportional(:, 1, 1), 'g', duration, logIntegral(:, 1, 1), 'r',...
% %  duration, logDerivative(:, 1, 1) ,'b', duration, logControlSpeed(:, 1, 1), 'k');
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
plot(ax5, duration, logFormationError(:,1, 1), duration, logFormationError(:,1, 2));
title(ax5, 'x error')
plot(ax6, duration, logFormationError(:, 2, 1), duration, logFormationError(:,2, 2));
title(ax6, 'y error')
%%
figure('Name','agent position and target','NumberTitle','off')
plot(x1, y1, 'r', x1(end), y1(end),'ro', positionRef(1,1), positionRef(2,1), 'r*',...
     x2, y2, 'b', x2(end), y2(end),'bo', positionRef(1,2), positionRef(2,2), 'b*');
xlabel('x')
ylabel('y')
legend('agent1','', '', 'agent 2')
title ('agent position and target');
axis([0, 1200, 0, 1600]);
set(gca, 'YAxisLocation','right');
view(90,90)

%%
figure(5)
ax7 = subplot(2,1,1);
ax8 = subplot(2,1,2);
plot(ax7, duration, de1);
title(ax7, 'agent 1 distance error');
plot(ax8, duration, de2);
title(ax8, 'agent 2 distance error');