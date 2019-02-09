%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% [mu, sig] = fastMeanVar(samples)
% 
% mu        : mean of particles
% sig       : covariance of particles
% samples   : sample particles 
% 
% Assumes equally weighted particles
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [mu, sig] = fastMeanVar(samples)
    np = length(samples);
    
    mu = [0;0];
    totalWt = 0;
    
    % calculate weighted mean
    for i=1:np
        mu = mu + samples(i).w*samples(i).pose;
        totalWt = totalWt + samples(i).w;
    end
    mu = mu./totalWt;
    
    % calculate weighted covariance
    sig = zeros(2,2);
    for i=1:np
        x = samples(i).pose - mu;
        sig = sig + samples(i).w*(x*x');
    end
    sig = sig./totalWt;
    
end