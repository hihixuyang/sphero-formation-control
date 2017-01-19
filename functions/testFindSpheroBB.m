load('frame16001200.mat');
N = 3;
threshold = 0.55;
%to avoid false recognitions
for i = 1:N
[centers, noDetections, bboxes, mask] = findSpheroBB(frame, threshold);
end;
%imshow(mask);

while noDetections < N
    
    threshold = threshold -.01; %makes it more sensitive
    disp('Attempting initial detection, searching for Spheros, decreasing threshold');
    tic
    [centers, noDetections, bboxes, mask] = findSpheroBB(frame, threshold);
    toc
    imshow(mask);
    %pause();
end
