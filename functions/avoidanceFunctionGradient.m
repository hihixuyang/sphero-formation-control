function [sumVa, sumdVadP ] = avoidanceFunctionGradient( pos, N, M, r, R)
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

dVadP = zeros(N, N+M, 2);
Va = zeros(N, N+M);
for i = 1:N
    for j = (i+1):N % the distance matrix is symmetric in case of agents      

        [Va(i,j), dVadP(i, j, :)] = avoidanceFunctionDerivative(pos(:, i), pos(:, j), r, R);
        
        Va(j, i) = Va(i, j);
        dVadP(j, i, :) = - dVadP(i, j, :);
    end    
    for k = (N+1):N+M %calculating Va for the obstacles
        [Va(i,k), dVadP(i, k, :)] = avoidanceFunctionDerivative(pos(:, i), pos(:, k), r, R);
    end
end
%summing the values for each agent
sumdVadP = squeeze(sum(dVadP, 2))';
sumVa = sum(Va, 2)';
end

