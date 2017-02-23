function [u, Phi, Psi, Va, dVadP, Vo, dVodP, phi, psi] =...
    formationController(spheroPos, obstaclePos, distanceRef, positionRef, angleRef, Vr, r, R,...
    formationGains, scaleMatrix)
%formationController is used to calculate the speed values according to the
%applied control law.
%INPUTS:
%   -spheroPos: current positions of the agents [2*N]
%   -obstaclePos: positions of obstacles [2*M]
%   -positionRef: position reference for agents [2*N]
%   -angleRef: angle referenc [1*N]
%   -r: detectiom radius
%   -R: avoidance radius
%   -formationGains[k_a, k_v, k_o]
%   -doOrientation :boolean, if true, than orientation control activated
%OUTPUT:
%   -formationControlOutput [2*N]
%   -d: distance between all objects [N*(N+M)]
%   -angle: angle between objects [1*N]
%   -Phi: angle error gradient [2*N]
%   -Psi: distance error gradient [2*N]
%   -dVadP: obstacle avoidance function gradient [2*N]
%   -dVodP: orietantion function gradient [2*2]
%   -phi: angle error [1*N]
%   -psi: distance error [1*N]


k_d = formationGains(1); % angle error gain
k_a = formationGains(2); % distance error gain
k_oa = formationGains(3); % obstacle error gain
k_o = formationGains(4);
k_r = formationGains(5);
%create a combined vector for the positions of robots and obstacles
N = size(spheroPos, 2);
M = size(obstaclePos, 2);
elementPos = [spheroPos obstaclePos];

%distance reference between all agents [N*N]
%matrix is symmetric, only triu is directly assigned, elements on main
%diag are 0

Phi = zeros (2, N);
Psi = zeros (2, N);
psi = zeros(1, N);
phi = zeros(1, N);
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
    
    [phi(i), Phi(:, i)] = angleErrorGradient(angleRef(i),...
        elementPos(:,i), elementPos(:,k), elementPos(:,j));
    
    [psi(i), Psi(:, i)] = distanceErrorGradient(distanceRef(i), elementPos(:,i), elementPos(:,k), scaleMatrix );
    
end
%replace NaN elements in the gradients with 0. These result where there is
%no reference
Phi(isnan(Phi))=0;
Psi(isnan(Psi))=0;
phi(isnan(phi))=0;
psi(isnan(psi))=0;
%obstacle avoidance
[Va, dVadP]  = avoidanceFunctionGradient(elementPos, N, M, r, R);

dVodP = zeros(2, N);
[Vo, dVodP(:, 1:2)] = orientationControl(positionRef, spheroPos);

u = k_d*Psi + k_a*Phi + k_oa*dVadP + k_o*dVodP + k_r*Vr.*ones(2,N);

end
