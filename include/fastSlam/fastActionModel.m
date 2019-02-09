%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Motion Model
% new = fastActionModel(prev, point, sv)
% 
% new   : new particles based on action model
% prev  : particles to be distributed
% point : waypoint in spherical coordinates
% sv    : sigma [x,y] (in pixels)
% 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function new = fastActionModel(prev, point, sv)
    np = length(prev);    
    new = cell(np,0);
    
    for i =1:np
       dx = point(1) - normrnd(0,sv(1)); 
       dy = point(2) - normrnd(0,sv(2));
       
       new(i).pose = [(prev(i).pose(1) + dx); (prev(i).pose(2) + dy)];
       new(i).w = prev(i).w;
       new(i).mapMu = prev(i).mapMu;
       new(i).mapSigma = prev(i).mapSigma;
       new(i).mapID = prev(i).mapID;
       new(i).hashMap = prev(i).hashMap;
    end
    
end