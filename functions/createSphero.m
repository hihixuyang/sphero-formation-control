function [ spheroStruct ] = createSphero( frame, currentCentroid, size)
%createSphero creates a Sphero struct array with the passed values. The
%whole struct lokks like:
%__________________________________________________________________________
%   * small extract of the image which contains the robot
%   * offset
%   * kalmanfilter for object tracking, initialized to the original
%       centroid
%   * current centroid
%   * last centroid
%   * heading angle in DEGREE
%   * velocity
%   * streak
%__________________________________________________________________________


currentCentroid;

[smallFrame, offset] = extractFromFrame(frame, currentCentroid, size);

kalmanFilter = configureKalmanFilter('ConstantAcceleration',... Motion model
                                        currentCentroid,... initial position
                                        [1 1 1]*1e5,... initial estimation Error
                                        [25 10 10],... Motionnoise
                                        25 ... Measurement Noise
                                        );

spheroStruct = struct('smallFrame', smallFrame,...
                        'offset', offset,...
                        'centroid', currentCentroid,...
                        'lastCentroid', currentCentroid,...
                        'heading', 0,...
                        'velocity', 0,...
                        'kalmannFilter', kalmanFilter,...
                        'streak', 0);

end

