function [ sumdVadP ] = avoidanceFunctionGradient( pos, N, M, d, r, R)
%AVOIDANCEFUNCTION Calculates the sum of derivatives of the avoidance function
%   Detailed explanation goes here
%INPUTS:
% pos: element positions [2*(N+M)]
% N: number of agents
% M: number of obstacles
% d: distance between elements
% r: protection radius
% R: detection radius
%OUTPUTS:
%sumdVadP : avoidance function gradient for each agent [2*N]

Va = zeros(N, N+M);
dVadP = zeros(N, N+M, 2);

for i = 1:N
    for j = (i+1):N % the distance matrix is symmetric in case of agents      
        Va(i,j) = avoidanceFunction(d(i,j), r, R);
        dVadP(i, j, :) = avoidanceFunctionDerivative(d(i,j), r, R, pos(:, i), pos(:, j));
        
        Va(j, i) = Va(i, j);
        %dVadP(j, i, :) = avoidanceFunctionDerivative(d(i,j), r, R, pos(:, j), pos(:, i));
        dVadP(j, i, :) = - dVadP(i, j, :);
    end    
    for k = (N+1):N+M %calculating Va for the obstacles
        Va(i,k) = avoidanceFunction( d(i,k), r, R);
        dVadP(i, k, :) = avoidanceFunctionDerivative(d(i,k), r, R, pos(:, i), pos(:, k));        
    end
end

%summing the values for each agent
sumdVadP = squeeze(sum(dVadP, 2))';
end

