function [centroid, detections, bbox, filteredImage] = findSpheroBB(frame, threshold)
%This function shall find all spheros in the given frame and return their
%positions(P) and their number(n), their bounding box (BB)and 
%a binar image(bw) obtained after tresholding. 
%The parameter frame should be an image to get reasonable results, and
%input parameter threshold should be [0..1], smaller means more sensitive
%Threshold = .45 works well usually

%%Setup constants
%%preprocess image
grayImage = rgb2gray(frame);
% figure(1);
% imshow(filteredImage)
% pause()
%% Hough Transformation
% tic
% 
% [accum, circen] = CircularHough_Grd(grayImage, [10 17], 20);
% % surf(accum, 'EdgeColor', 'none');
% % pause();
% % imshow(accum);%
% % circen
% num = size(circen);
% 
% %check wether some robots where found
% if(num(1) == 0)
%     n = 0;
%     P = 0;
%     disp('No robots found');
% else
%     n = num(1);
%     P = cell(n, 1);
%     %calculate center of robot
%     for i = 1:n
%         P{i,:} = circen(i,:);
%     end
% end
% toc

%% imfindcircles method - very slow
% tic
% [icenters, iradii] = imfindcircles(grayImage,[14 21],...
%     'ObjectPolarity', 'bright', 'Sensitivity', .95, 'EdgeThreshold',0.2)
% viscircles(icenters, iradii,'EdgeColor','g');
% toc
% % EdgeThreshold 1 = strict, 0 = less strict
% % Sensitivity 1 = more sensible, .85 default value

%% Standard Thresholding
binarImage = imbinarize(grayImage, threshold);
%imwrite(bw, 'bw.png')
% figure(2)
% imshow(bw1);
%pause();

%%find spheros
%Edge detection
filteredImage = bwareafilt(binarImage, [650 1000]); 
%min and max size for region to be detected
% figure(3)
%imshow(filteredImage);
%pause();

%Get robot positions if a robot was found
%[B,L] = bwboundaries(BW,'noholes');
properties = regionprops(filteredImage, 'BoundingBox', 'Centroid', 'Area');
sizeOfEdges = size(properties, 1); %number of detected sheros
%st.Area
%check wether some robots where found
if(sizeOfEdges == 0)
    detections = 0; centroid = 0; bbox = 0;
    disp('No robots found in frame');
else
    detections = sizeOfEdges(:,1);
    centroid = zeros(detections, 2); %preallocating
    bbox = zeros(detections, 4);
    if detections ~= size(centroid, 1) || detections ~= size(bbox, 1)
        disp('ERROR: inconsistent detection')
    end
    bbox = vertcat(properties(:).BoundingBox);
    centroid = vertcat(properties(:).Centroid);
    if size(bbox, 1) ~= size(centroid, 1)
        disp('Sizes of centroids and bboxes are different')
    end
end
