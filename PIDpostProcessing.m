clear;
close all;
load('PID02_15_35');
clc;
if exist('mode') ~= 1
    mode = 1;
end
mode
PIDgains
saturation
movementThreshold

 angleRef
 distanceRef
 offset
 Ka
 Kd
 Ko
%% remove first values from the data

logCounter = logCounter - 2;

%eliminate the first row which is 0
logPosition = circshift(logPosition, -1);

logControlVelocity = circshift(logControlVelocity, -1);
logPhi = circshift(logPhi, -1);
logPsi = circshift(logPsi, -1);
logphi = circshift(logphi, -1);
logpsi = circshift(logpsi, -1);
%logVa = circshift(logVa, -1);
%logVo = circshift(logVo, -1);
logdVadP = circshift(logdVadP, -1);
logdVodP = circshift(logdVodP, -1);

logAgentVelocity = circshift(logAgentVelocity, -1);
logAgentSpeed = circshift(logAgentSpeed, -1);

logDesiredHeading = circshift(logDesiredHeading, -1);
logAgentHeading = circshift(logAgentHeading, -1);

logTiming = circshift(logTiming, -1);
%% remove zeros
logPosition = logPosition(1:logCounter, :, :);

logControlVelocity = logControlVelocity(1:logCounter, :, :);
logPhi = logPhi(1:logCounter, :, :);
logPsi = logPsi(1:logCounter, :, :);
logphi = logphi(1:logCounter, :);
logpsi = logpsi(1:logCounter, :);%distance error
%logVa = logVa(1:logCounter, :);
%logVo = logVo(1:logCounter, :);
logdVadP = logdVadP( 1:logCounter, :, :);
logdVodP = logdVodP( 1:logCounter, :, :);

logAgentVelocity = logAgentVelocity(1:logCounter, :, :);
logAgentSpeed = logAgentSpeed(1:logCounter, :);

logDesiredHeading = logDesiredHeading(1:logCounter, :);
logAgentHeading = logAgentHeading(1:logCounter, :);

logTiming = logTiming(1:logCounter, :);
logTiming(:, 12) = sum(logTiming, 2); %delta_t for each step

duration = cumsum(logTiming(:, 12));
%% rename log varaibles
x1 = logPosition(:, 1, 1);
y1 = logPosition(:, 2, 1);

% positionRef(2,:) =  yRange - positionRef(2,:);
agentSpeed = logAgentSpeed(:, 1);

ux = logControlVelocity(:, 1, 1);
uy = logControlVelocity(:, 2, 1);
u = ux.^2 + uy.^2;
desiredDir1 = logDesiredHeading(:, 1);
actualDir1 = logActualHeading(:, 1);
controlAngle1 = logAgentHeading(:, 1);

de1 = logpsi(:, 1);

dnx = positionRef1(1,1);
dny = positionRef1(2,1);
anx = positionRef2(1,1);
any = positionRef2(2,1);
%%
% Defaults for this blog post
%width = 2;     % Width in inches
%height = 5;    % Height in inches
alw = 0.4;    % AxesLineWidth
fsz = 12;      % Fontsize
lw = 0.5;      % LineWidth
msz = 6;       % MarkerSize

% The properties we've been using in the figures
set(0,'defaultLineLineWidth',lw);   % set the default line width to lw
set(0,'defaultLineMarkerSize',msz); % set the default line marker size to msz
set(0,'defaultLineLineWidth',lw);   % set the default line width to lw
set(0,'defaultLineMarkerSize',msz); % set the default line marker size to msz

% Set the default Size for display
defpos = get(0,'defaultFigurePosition');
%set(0,'defaultFigurePosition', [defpos(1) defpos(2) width*100, height*100]);
set(0,'defaultFigurePosition', 'factory');

% Set the defaults for saving/printing to a file
set(0,'defaultFigureInvertHardcopy','on'); % This is the default anyway
set(0,'defaultFigurePaperUnits','inches'); % This is the default anyway
%defsize = get(gcf, 'PaperSize');
% left = (defsize(1)- width)/2;
% bottom = (defsize(2)- height)/2;
% defsize = [left, bottom, width, height];
% set(0, 'defaultFigurePaperPosition', defsize);
set(0, 'defaultFigurePaperPosition', 'factory');
gca.XAxisLocation = 'origin';
gca.YAxisLocation = 'origin';
%% distribution of sampling times
% figure('Name', 'cycle time')    %cycleTime = deleteoutliers(cycleTime);
% cycleTime = logTiming(:, 12);
%     histogram(cycleTime, 50);
%     title('Cycle Time vs. Detection Time ')
%     hold on
%     %print -depsc epsFig
%     detectionTime = logTiming(:, 1) + logTiming(:, 2);
%     meanDetectionTime = mean(detectionTime)
%     histogram(detectionTime, 50);
%     %
%     % findTime = logTiming(:, 2);
%     % meanFindTime = mean(findTime)
%     % histogram(findTime, 50);
%     %
%     % captureTime = logTiming(:, 1);
%     % meanCaptureTime = mean(captureTime)
%     % histogram(captureTime, 50);
% %     
% %     meanDetectionTime/meanCycleTime
%     legend('show')
%     xlabel('seconds')
%      print('C:\Users\mihai\Dropbox\Uni TUHH\Sem 3\Project\Latex documentation\figures\cycletime',...
%          '-depsc', '-tiff' );
% 
% 
% meanCycleTime = mean(cycleTime)
% hold off
%% PID Tunning
%figure('Name','agent position and target','NumberTitle','off')
%subplot(1,2,2)
if mode == 1
    plot(x1, y1, 'r', x1(1), y1(1),'ro', x1(end), y1(end),'r*', dnx, dny, 'kx');
    legend('agent path','start position', 'end position', 'target')
end
if mode == 2
    plot(x1, y1, 'r', x1(1), y1(1),'ro', x1(end), y1(end),'r*', dnx, dny, 'kx');
    rectangle('Position', [dnx-distanceRef, dny-distanceRef,...
        2*distanceRef,2*distanceRef], 'Curvature', [1, 1]) ;
    legend('path','start', 'end', 'target')
end;
if mode == 3
    plot(x1, y1, 'r', x1(1), y1(1),'ro', x1(end), y1(end),'r*',...
        dnx, dny, 'kx', anx, any, 'k<');
    if angleRef < 0
        line ([dnx 0], [dny (dny-dnx*tand(angleRef+atan2d(any-dny, anx-dnx)))],'Color','k');
    else
        line ([dnx 3.2], [dny dny+(3.2-dnx)*tand(angleRef+atan2d(any-dny, anx-dnx))],'Color','k');
    end
    legend('path','start', 'end', 'dist neigh', 'angle neigh','angle target')
end
if mode == 4
    plot(x1, y1, 'r', x1(1), y1(1),'ro', x1(end), y1(end),'r*',...
        dnx, dny, 'kx', anx, any, 'k<');
    rectangle('Position', [dnx-distanceRef, dny-distanceRef,...
        2*distanceRef,2*distanceRef],'Curvature', [1, 1]);
    if angleRef < 0
        line ([dnx 0], [dny (dny-dnx*tand(angleRef+atan2d(any-dny, anx-dnx)))],'Color','k');
    else
        line ([dnx 3.2], [dny dny+(3.2-dnx)*tand(angleRef+atan2d(any-dny, anx-dnx))],'Color','k');
    end
end

if mode == 5
    plot(x1, y1, 'r', x1(1), y1(1),'ro', x1(end), y1(end),'r*',...
        dnx, dny, 'kx', anx, any, 'k<',obstaclePos(1), obstaclePos(2),'ko');
    rectangle('Position', [dnx-distanceRef, dny-distanceRef,...
        2*distanceRef,2*distanceRef],'Curvature', [1, 1]);
    n = 10;
    radius = linspace(r, R, n);
    viscircles(obstaclePos'.* ones(n,2),radius,'Color','k','LineWidth', 0.5, 'LineStyle', ':');
    if angleRef < 0
        line ([dnx 0], [dny (dny-dnx*tand(angleRef+atan2d(any-dny, anx-dnx)))] ,'Color','k');
    else
        line ([dnx 3.2], [dny dny+(3.2-dnx)*tand(angleRef+atan2d(any-dny, anx-dnx))],'Color','k');
    end
    line([dnx anx], [dny any],'Color','k')
    legend('agent path','start position', 'end position', 'distance neighbour',...
       'angle neighbour', 'obstacle' )
end
if mode == 6
    plot(x1, y1, 'r', x1(1), y1(1),'ro', x1(end), y1(end),'r*',...
        dnx, dny, 'kx', anx, any, 'k<');
    line ([anx anx+2*(dnx-anx)], [any any+2*(dny-any)]);
    n = 10;
    radius = linspace(r, R, n);
    viscircles(obstaclePos'.* ones(n,2),radius,'Color',[0 0 0],'LineWidth', 0.5, 'LineStyle', ':');
end
if mode == 7
    plot(x1, y1, 'r', x1(1), y1(1),'ro', x1(end), y1(end),'r*', movingRef(1, :), movingRef(2, :), 'b');
    legend('agent path','start position', 'end position', 'target')
end
xlabel('x')
ylabel('y')

grid on
axis equal;
axis([0, xRange, 0, yRange]);
% print('C:\Users\mihai\Dropbox\Uni TUHH\Sem 3\Project\Latex documentation\figures\stepResponse'...
% ,'-depsc' );

%%
%figure(1);
figure('Name','Local control input and u^2','NumberTitle','off')
% subplot(2,1,1);
plot(duration, agentSpeed);
axis([0 inf 0 1.1*saturation ])
legend('u^*');
title('Local control effort');
xlabel('time [s]')
ylabel('u^*[%]')

% subplot(2, 1, 2);
% plot(duration, u);
% legend ('u_x^2+u_y^2');
% title('Formation control effort');
% xlabel('time [s]')
% ylabel('u')
axis normal

%%
%figure(3);
figure('Name','Formation control input','NumberTitle','off')
plot(duration, ux, duration, uy, duration, u);
legend('u_x', 'u_y', 'u_x^2+u_y^2');
title('Formation control effort')
xlabel('time [s]');
ylabel('u')

%%
figure('Name','formation vs local control input x','NumberTitle','off')
plot(duration, ux, duration, logAgentVelocity(:, 1));
title('formation vs local control input x')
legend('formation vel', 'agent vel')

%%
figure('Name','formation vs local control input x','NumberTitle','off')
plot(duration, uy, duration, logAgentVelocity(:, 2));
title('formation vs local control input y')
legend('formation vel', 'agent vel')
%%
figure('Name','angles','NumberTitle','off')
plot(duration, desiredDir1,duration, controlAngle1);
legend('desired angle', 'control angle')
xlabel('time [s]');
ylabel('angle[degrees]')
%%
if mode == 4
    figure('Name','distance and angle error','NumberTitle','off');
    Psix = logPsi(:, 1);
    Psiy = logPsi(:, 2);
    Phix = logPhi(:, 1);
    Phiy = logPhi(:, 2);
    plot(duration, Psix, duration, Psiy,duration, Phix, duration, Phiy);
    legend('psix','psiy','phix','phiy');
    title('error gradients')
    xlabel('time [s]');
    ylabel('e[%]')
end;

%%
if mode == 5
    figure('Name','distance and angle error','NumberTitle','off');
    Psix = logPsi(:, 1);
    Psiy = logPsi(:, 2);
    Phix = logPhi(:, 1);
    Phiy = logPhi(:, 2);
    dVadPx = logdVadP(:, 1);
    dVadPy = logdVadP(:, 2);
    plot(duration, Psix, duration, Psiy,duration, Phix, duration, Phiy, duration, dVadPx, duration, dVadPy);
    legend('psix','psiy','phix','phiy', 'dVadPx', 'dVadPx');
    title('gradients gradients')
    xlabel('time [s]');
    ylabel('e[%]')
end;
%%
tilefigs;
