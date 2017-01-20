function [ tracks ] = createTracks( association, centroids, spheros )
%createTracks creates the struct >tracks> for further use in tracking
%   Creates the struct tracks with the association as ids and the already
%   determined properties of the found robots
    N = length(association);
    tracks = repmat(initializeTracks(), N); %??? seems like no reason to do this

    for i = 1:N
        centroid = centroids(association(i), :);
        % Create a Kalman filter object. Test the parameters if they grant
        % proper functioning
        kalmanFilter = configureKalmanFilter('ConstantAcceleration', ...
            centroid, [50, 5, 5], [100, 25, 25], 100);
        % Create a new track.
        newTrack = struct(...
            'id', i, ...
            'centroid', centroid, ...
            'kalmanFilter', kalmanFilter, ...
            'Sphero', spheros(i) );
        tracks(association(i)) = newTrack;
    end


    function tracks = initializeTracks()
    % create an empty array of tracks
        tracks = struct(...
            'id', {}, ...
            'centroid', {}, ...
            'kalmanFilter', {}, ...
            'Sphero', {} );
    end

end

