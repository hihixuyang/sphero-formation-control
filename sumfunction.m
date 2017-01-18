function [ output ] = sumfunction( input )
%SUMFUNCTION Summary of this function goes here
%   Detailed explanation goes here
persistent sum test;
if isempty(test)
    test = 0;
end
test = test + MyConstants.INCREMENT;
if isempty(sum)
    sum = 0;
end
sum = sum +input;
output = sum;
end

