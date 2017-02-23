clear;
close all;
load('PID31jan9');
%% Load and remove zeros from the data
logCounter = logCounter - 2;
%eliminate the first row which is 0

logNumDetections = circshift(logNumDetections, -1);
logPosition = circshift(logPosition, -1);

logControlVelocity = circshift(logControlVelocity, -1);
logphi = circshift(logphi, -1);
logpsi = circshift(logpsi, -1);
logVo = circshift(logVo, -1);
logVa = circshift(logVa, -1);
logPhi = circshift(logPhi, -1);
logPsi = circshift(logPsi, -1);
logdVadP = circshift(logdVadP, -1);
logdVodP = circshift(logdVodP, -1);

logAgentVelocity = circshift(logAgentVelocity, -1);
logAgentSpeed = circshift(logAgentSpeed, -1);

logAgentHeading = circshift(logAgentHeading, -1);
logDesiredHeading = circshift(logDesiredHeading, -1);
logActualHeading = circshift(logActualHeading, -1);

logTiming = circshift(logTiming, -1);
%%
logNumDetections = logNumDetections(1:logCounter, :);
logPosition = logPosition(1:logCounter, :, :);

logControlVelocity = logControlVelocity(1:logCounter, :, :);
logphi = logphi(1:logCounter, :);
logpsi = logpsi(1:logCounter, :);
logVo = logVo(1:logCounter, :);
logVa = logVa(1:logCounter, :);
logPhi = logPhi(1:logCounter, :, :);
logPsi = logPsi(1:logCounter, :, :);
logdVadP = logdVadP(1:logCounter, :, :);
logdVodP = logdVodP(1:logCounter, :, :);

logAgentVelocity = logAgentVelocity(1:logCounter, :, :);
logAgentSpeed = logAgentSpeed(1:logCounter, :, :);

logAgentHeading = logAgentHeading(1:logCounter, :);
logDesiredHeading = logDesiredHeading(1:logCounter, :);
logActualHeading = logActualHeading(1:logCounter, :);

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

%% PID Tunning
figure('Name','agent position','NumberTitle','off')
hold on
for i = 1 : N
    h(i, 1) = plot(logPosition(:, 1, i), logPosition(:, 2, i), '-');
    h(i, 2) = plot(logPosition(1, 1, i), logPosition(1, 2, i), 'Marker','o','Color',h(i, 1).Color);
    h(i, 3) = plot(logPosition(end, 1, i), logPosition(end, 2, i), 'Marker','*','Color',h(i, 1).Color);
end
n = 10;
radius = linspace(r, R, n);
viscircles(obstaclePos'.* ones(n,2),radius,'Color','k','LineWidth', 0.5, 'LineStyle', ':');
hold off
Legend=cell(N,1);
for iter=1:N
    Legend{iter}=strcat('agent ', num2str(iter));
end
legend(h(:,1), Legend)

xlabel('x')
ylabel('y')
title ('agent position');
axis([0, 3.2, 0, 2.4]);
set(gca, 'XAxisLocation','top');
set(gca,'YDir','reverse');
%%
%figure(1);
figure('Name','errors','NumberTitle','off')
ax1 = subplot(4,1,1);
ax2 = subplot(4,1,2);
ax3 = subplot(4,1,3);
ax4 = subplot(4,1,4);

hold on
for i = 1 : N
    h(i) = plot(ax1, duration, logphi(:, i), '-');
end
hold off
title(ax1,'phi');

hold on
for i = 1 : N
    h(i) = plot(ax2, duration, logpsi(:, i), '-');
end
hold off
title(ax2,'psi');

hold on
for i = 1 : N
    h(i) = plot(ax3, duration, logVo(:, i), '-');
end
hold off
title(ax3,'Vo');

hold on
for i = 1 : N
    h(i) = plot(ax4, duration, logVa(:, i), '-');
end
hold off
title(ax4,'Va');

Legend = cell(N,1);
for iter = 1:N
    Legend{iter} = strcat('agent ', num2str(iter));
end
legend(ax1, Legend)
