%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Resampling Algorithm
% new = resample(prev, nump)
% 
% new   : resampled particles
% prev  : particles to be resampled
% 
% refer - http://cecas.clemson.edu/~ahoover/ece854/lecture-notes/lecture-pf.pdf
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function new = resample(prev, nump)
    
    % weights matrix
    W = prev(3,:);
    
    % create array to store indices of particles to be replicated
    indexArray = ones(nump,1);
    new = zeros(3,nump);
    
    % generate CDF
    Q = cumsum(W);
    t = rand(nump + 1, 1);
    T = sort(t);
    T(nump + 1) = 1.0;
    i = 1; j = 1;
    
    % low variance sampling
    while i<=nump && j<=nump
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
    if uniqueIdxs < 0.3*nump
        new = prev;
    else
        % copy over resampled particles to new array
        % and set all particle weights to 1/nump
        for i=1:nump
           new(1,i) = prev(1,indexArray(i));
           new(2,i) = prev(2,indexArray(i));
           new(3,i) = 1/nump;
        end
    end
end