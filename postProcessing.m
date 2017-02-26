clear;
close all;
load('formation02_24_23');
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

%% display
formationGains
distanceRef
angleRef
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
end
for i = 1 : N
    h(i, 2) = plot(logPosition(1, 1, i), logPosition(1, 2, i), 'Marker','o','Color',h(i, 1).Color);
end
for i = 1 : N
    h(i, 3) = plot(logPosition(end, 1, i), logPosition(end, 2, i), 'Marker','*','Color',h(i, 1).Color);
end
if ~isempty(obstacles)
    n = 10;
    radius = linspace(r, R, n);
    viscircles(obstacles'.* ones(n,2),radius,'Color','k','LineWidth', 0.5, 'LineStyle', ':');
end
hold off
Legend=cell(N,1);
for iter=1:N
    Legend{iter}=strcat('agent ', num2str(iter));
end
legend(h(:,1), Legend)

xlabel('x')
ylabel('y')
title ('agent position');
grid on
axis equal;
axis([0, xRange, 0, yRange]);
% set(gca, 'XAxisLocation','top');
% set(gca,'YDir','reverse');
%%
%figure(1);
figure('Name','errors','NumberTitle','off')

ax1 = subplot(4,1,1);
ax2 = subplot(4,1,2);
ax3 = subplot(4,1,3);
ax4 = subplot(4,1,4);

hold(ax1, 'on')
for i = 1 : N
    h(i) = plot(ax1, duration, logphi(:, i), '-');
    
end
hold(ax1, 'off');
title(ax1,'phi');
grid (ax1, 'on');

hold(ax2, 'on')
for i = 1 : N
    h(i) = plot(ax2, duration, logpsi(:, i), '-');
end
hold(ax2, 'off');
title(ax2,'psi');
grid (ax2, 'on');

hold(ax3, 'on')
for i = 1 : 2
    h(i) = plot(ax3, duration, logVo(:, i), '-');
end
hold(ax3, 'off')
title(ax3,'Vo');
grid (ax3, 'on');

hold(ax4, 'on')
for i = 1 : N
    h(i) = plot(ax4, duration, logVa(:, i), '-');
end
hold(ax4, 'off')
title(ax4,'Va');
grid (ax4, 'on');

Legend = cell(N,1);
for iter = 1:N
    Legend{iter} = strcat('agent ', num2str(iter));
end
legend(ax1, Legend)

%%
figure
for i = 1 : N
    hold on
    h(i) = plot(duration, logAgentSpeed(:, i), '-');
end

Legend = cell(N,1);
for iter = 1:N
    Legend{iter} = strcat('agent ', num2str(iter));
end
grid on
legend(Legend)
title('agent speed')

%%
figure
for i = 1 : N
    hold on
    h(i) = plot(duration, logAgentHeading(:, i), '-');
end

Legend = cell(N,1);
for iter = 1:N
    Legend{iter} = strcat('agent ', num2str(iter));
end
grid on
legend(Legend)
title('agent angle')

%%
figure
ax1 = subplot(3,1,1);
ax2 = subplot(3,1,2);
ax3 = subplot(3,1,3);
plot(ax1,duration, logPosition(:, 1, 1), duration, logPosition(:, 2, 1))
title(ax1,'agent 1')
plot(ax2,duration, logPosition(:, 1, 2), duration, logPosition(:, 2, 2))
title(ax2,'agent 2')
plot(ax3,duration, logPosition(:, 1, 3), duration, logPosition(:, 2, 3))
title(ax3,'agent 3')
grid on
legend('x', 'y')
xlabel('time')
ylabel('distance')
%%
tilefigs
