%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% [APLocs, Sig]= getAPLocs(samples)
% 
% APLocs        : AP locations as calculated by Fast SLAM
% Sig           : Covariance of AP Locations
% samples       : given the particles, gets the AP locations
% 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [APLocs, Sig, IDs] = getAPLocs(samples)
    np = length(samples);
    maxWeight = -9999999;
    
    for i=1:np
       if samples(i).w>maxWeight
           maxWeight = samples(i).w;
           particleIdx = i;
       end
    end

    APLocs = samples(particleIdx).mapMu;
    Sig = samples(particleIdx).mapSigma;
    IDs = samples(particleIdx).mapID;
end