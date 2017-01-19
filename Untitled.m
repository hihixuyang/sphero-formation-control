% centroids = [1 3; 5 7; 4 9; 2 5; 7 7];
% last_Centroids = 2* centroids;
% 
%     tic
%     displacement = centroids - last_Centroids;
%     hasMoved = sqrt(sum((centroids - last_Centroids).^2, 2));
%     toc;
%     d=gpuDevice(1);
% test = 10000*rand(1000,1000);
%     tic 
%     inv(test);
%     toc
%     G = gpuArray(test);
%     tic 
%     gputimeit(@()inv(G))
%     wait(d)
%     toc
    
maxIterations = 500;
gridSize = 1000;
xlim = [-0.748766713922161, -0.748766707771757];
ylim = [ 0.123640844894862,  0.123640851045266];

% Setup
t = tic();
x = linspace( xlim(1), xlim(2), gridSize );
y = linspace( ylim(1), ylim(2), gridSize );
[xGrid,yGrid] = meshgrid( x, y );
z0 = xGrid + 1i*yGrid;
count = ones( size(z0) );

% Calculate
z = z0;
for n = 0:maxIterations
    z = z.*z + z0;
    inside = abs( z )<=2;
    count = count + inside;
end
count = log( count );

% Show
cpuTime = toc( t );
fig = gcf;
fig.Position = [200 200 600 600];
imagesc( x, y, count );
colormap( [jet();flipud( jet() );0 0 0] );
axis off
title( sprintf( '%1.2fsecs (without GPU)', cpuTime ) );

%%
% Setup
t = tic();
x = gpuArray.linspace( xlim(1), xlim(2), gridSize );
y = gpuArray.linspace( ylim(1), ylim(2), gridSize );
[xGrid,yGrid] = meshgrid( x, y );
z0 = complex( xGrid, yGrid );
count = ones( size(z0), 'gpuArray' );

% Calculate
z = z0;
for n = 0:maxIterations
    z = z.*z + z0;
    inside = abs( z )<=2;
    count = count + inside;
end
count = log( count );

% Show
count = gather( count ); % Fetch the data back from the GPU
naiveGPUTime = toc( t );
imagesc( x, y, count )
axis off
title( sprintf( '%1.3fsecs (naive GPU) = %1.1fx faster', ...
    naiveGPUTime, cpuTime/naiveGPUTime ) )

%%
timeBench = bench(3);
%%
tStart= tic;
distance = 0;
day = 0;
while (distance < 4.5)
    if (distance ~= 0)
        distance = 0.9*distance;
    end;
    distance = distance + 0.5;
    day = day + 1;
end
day;
distance;
toc
tStop = toc(tStart)
%%
global L;
L = 10;
sumfunction(2);
%%
logmax = 100;
logSpeed=zeros(logmax, 1);
logPosition = zeros(logmax, 5,2);
logvar = struct('logSpeed',logSpeed, 'logPosition', logPosition);
logvaraux = struct('speed', 0, 'position' , zeros(1,2));
for i = 1: logmax
    logvar2(i) = logvaraux;
end;
%%
logStruct = struct(...
    'logSpeedReference', zeros(1, N),...
    'logControllerOutput', zeros(1, N), ...
    'logDirection', zeros(1, N),...
    'logDesiredHeading', zeros(1, N),...
    'logPosition', zeros(1, N, 2), ...
    'logSpeed', zeros(1, N), ...
    'logDistError', zeros(1, N), ...
    'logNumDetections', zeros(1, 1), ...
    'logTiming' , zeros(1, 11), ...
    'logImages', cell(1, 1));
for i = 1 : MAX_LOG
    logvar3(i) = logStruct;
end

%%
x = rand(1, 1000);
 tic;z=sqrt(x'*x);toc
 tic;y=sqrt(sum(x.^2));toc
 tic;w=norm(x); toc;
 
 %%
 x = 1000*rand(1, 1e6);
 tic; sum1 = sum(x); toc;
 tic; sum2 = norm(x,1); toc;
 
 %%
 
 