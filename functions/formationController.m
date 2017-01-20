function [formationError, d, angle, Phi, Psi, dVadP, dVodP, phi, psi] =...
    formationController(spheroPos, obstaclePos, positionRef, angleRef, r, R,...
    formationGains, doOrientation)
%formationController is used to calculate the speed values according to the
%applied control law.
%INPUTS:
%   -positions: current positions of the agents [2*N]
%   -obstacles: positions of obstacles [2*M]

k_a = formationGains(1); % angle error gain
k_v = formationGains(2); % distance error gain
k_o = formationGains(3); % obstacle error gain

%create a combined vector for the positions of robots and obstacles
N = size(spheroPos, 2);
M = size(obstaclePos, 2);
elementPos = [spheroPos obstaclePos];

%add collision avoidance by calculating distances between elements (robots and obstacles)
d = distanceElements( elementPos, N, M );

%distance reference between all agents [N*N]
%matrix is symmetric, only triu is directly assigned, elements on main
%diag are 0
distanceRef = zeros(N);
for i = 1 : N
	for j = i+1 : N
		distanceRef(i, j) = norm(positionRef(:, j) - positionRef(:, i));
		distanceRef(j, i) = distanceRef(i, j);
	end
end

Phi = zeros (2, N);
Psi = zeros (2, N);
angle = zeros(1, N);
for i = 1:N
    k = i-1;	 %distance neighbor    
    %the first agents have neigbors with bigger indexes
    if k == 0
        k = N;
    end
    
    j = i-2;     %angle neighbor    
    if j == 0
        j = N;
    end
    if j == -1
        j = N-1;
    end
    %testing
    if j == 0
        j = N;
    end
    
    angle(i)  = anglePythagora(d(k,i), d(k,j), d(j,i));
    
    Phi(:, i) = angleErrorGradient( angle(i), angleRef(i),...
        elementPos(:,i), elementPos(:,k), elementPos(:,j)); 
    Psi(:, i) = distanceErrorGradient( d(k,i), distanceRef(k,i),...
        elementPos(:,i), elementPos(:,k));
end
%replace NaN elements in the gradients with 0. These result where there is
%no reference
Phi(isnan(Phi))=0;
Psi(isnan(Psi))=0;

phi = (angle - angleRef).^2;
%column vector with the distances to the neighbour with the previous index
dki = [d(N, 1); diag(d, 1)];
distanceRefki = [distanceRef(N, 1); diag(distanceRef, 1)];
psi = (dki - distanceRefki).^2;

%obstacle avoidance
dVadP  = avoidanceFunctionGradient(elementPos, N, M, d, r, R);

formationError = zeros(2, N);

if doOrientation
    %orientation control
    dVodP = orientationControl(positionRef, spheroPos );
    for i = 1:2
        formationError(:, i) = -k_v*dVodP(:, i) - k_o*dVadP(:, i);
    end;
else
    dVodP = zeros(2);
    for i = 1:2
        formationError(:, i) = k_v*Psi(:, i) - k_o*dVadP(:, i);
    end;
end;

for i = 3: N
    formationError(:, i) = k_a*Phi(:, i) + k_v*Psi(:, i) - k_o*dVadP(:, i);
end

end
