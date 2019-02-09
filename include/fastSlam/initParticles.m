%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% samples = initParticles(np, start)
% 
% samples   : sample particles
% np        : number of particles
% start     : start position of the agent
% 
% Generates np particles with 
% x = start.x, y = start.y and weight = 1/np
% 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function samples = initParticles(np, start)
    samples = cell(np,0);
    for i=1:np
        samples(i).pose = [start(1);start(2)];
        samples(i).w = 1/np;
        samples(i).mapMu = [];
        samples(i).mapID = [];
        samples(i).mapSigma = [];
        samples(i).hashMap = containers.Map('KeyType','uint32','ValueType','uint32');
    end
end