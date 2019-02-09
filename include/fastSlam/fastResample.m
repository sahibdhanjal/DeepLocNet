%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Resampling Algorithm
% new = resample(prev)
% 
% new   : resampled particles
% prev  : particles to be resampled
% 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function new = fastResample(prev)
    np = length(prev);
    
    % populate weights matrix
    W = zeros(np,1);
    for i=1:np
       W(i) = prev(i).w; 
    end
    
    % create array to store indices of particles to be replicated
    indexArray = ones(np,1);
    new = prev;
    
    % generate CDF
    Q = cumsum(W);
    t = rand(np + 1, 1);
    T = sort(t);
    T(np + 1) = 1.0;
    i = 1; j = 1;
    
    % low variance sampling
    while i<=np && j<=np
        if T(i) < Q(j)
            indexArray(i) = j;
            i = i+1;
        else
            j = j+1;
        end
    end
    
    % the reason for random jumping!!!
    % if the number of unique particles is 
    % less than a threshold, don't resample
    uniqueIdxs = length(unique(indexArray));
    if uniqueIdxs < 0.3*np
        new = prev;
    else
        % copy over resampled particles to new array
        % and set all particle weights to 1/np
        for i=1:np
           new(i).pose = prev(indexArray(i)).pose;
           new(i).w = 1/np;
           new(i).mapMu = prev(indexArray(i)).mapMu;
           new(i).mapSigma = prev(indexArray(i)).mapSigma;
           new(i).mapID = prev(indexArray(i)).mapID;
           new(i).hashMap = prev(indexArray(i)).hashMap;
        end
    end
end