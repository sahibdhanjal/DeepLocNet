%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% [err,cdf] = getError(p1, p2)
% 
% err   : sum of euclidean distances between waypoints 1,2
% cdf   : cumulative sums of errors as a plot
% p1    : waypoints of path 1
% p2    : waypoints of path 2
% 
% returns the total deviation in the ground truth
% and the localized path
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [err,cdf] = getError(p1, p2)
    n = length(p1);
    cdf = zeros(n,1);
    for i=1:n
        cdf(i) = dist(p1(i,:), p2(i,:));
    end
    err = sum(cdf);
end