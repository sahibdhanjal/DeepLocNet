%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% samples = startDistrib(np, start)
% 
% samples   : sample particles
% np        : number of particles
% start     : start position of the agent
% 
% Generates np particles with 
% x = start.x, y = start.y and weight = 1/np
% 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function samples = startDistrib(np, start)
    samples = [ start(1)*ones(1,np);
                start(2)*ones(1,np);
                1/np*ones(1,np)];
end