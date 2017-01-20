function [ matches ] = assignTracks(tracks, detectedCentroids)
%This function matches detected robots to the ones that are tracked.% It 
%just wraps up the function "assignDetectionsToTracks" with some utility 
%that is necessary for this case. 
 costOfNonAssignment = 20;
%assign detections to the Robots
nTracks = length(tracks);
nDetections = size(detectedCentroids, 1);

%compute the cost of each assignment and assign the detections to the
%most likely track (lowest cost)
cost = zeros(nTracks, nDetections);
for i = 1:nTracks
    cost(i, :) = distance(tracks(i).kalmanFilter, detectedCentroids);
end

%Solve the assignment problem. This is an actual predefined function
[matches, ~, ~] = assignDetectionsToTracks(cost, costOfNonAssignment);

end

