load('frame16001200.mat');
N = 3;
threshold = 0.60;
%to avoid false recognitions
[centers, noDetections, bboxes, mask] = findSpheroBB(frame, threshold);

while noDetections < N
    imshow(mask);
    threshold = threshold -.01; %makes it more sensitive
    disp('Attempting initial detection, searching for Spheros, decreasing threshold');
    tic
    [centers, noDetections, bboxes, mask] = findSpheroBB(frame, threshold);
    toc
end
