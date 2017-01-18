function [ d ] = distanceElements( elementPos, N, M )
%DISTANCE Calculates the distance matrix between the elements(agents and
%obstacles. The distance matrix is not rectangulat because only the
%distance from the agents to the obstacles is calculated. The distance
%matrix has zeros on the main diagonal 
%   INPUTS:
% -elementPos: the positions of the elements [2*(N+M)]
% OUTPUTS:
% -d: the distance matrix between agents and other elements [N*(N+M)]

d = zeros(N, N+M);

for i = 1:N
    for j = (i+1):N %the distance between the agents is 'symmetric'
        d(i,j) = norm(elementPos(:, i) - elementPos(:, j));
        d(j,i) = d(i,j);
    end
    
    for j = (N+1):N+M
        d(i,j) = norm(elementPos(:, i) - elementPos(:, j));
    end
end

end

