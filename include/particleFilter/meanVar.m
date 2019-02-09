%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% [mu, sig] = meanVar(samples, numP)
% 
% mu        : mean of particles
% sig       : covariance of particles
% samples   : sample particles 
% numP      : number of particles
% 
% Assumes equally weighted particles
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [mu, sig] = meanVar(samples, numP)
    mu = mean(samples, 2);
    zeroMean = samples - repmat(mu,1,numP);
    sig = zeroMean*zeroMean' / numP;
end