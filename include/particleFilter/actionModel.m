%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Motion Model
% new = actionModel(prev, point, su, numP)
% 
% new   : new particles based on action model
% prev  : particles to be distributed
% point : waypoint in spherical coordinates
% su    : sigma [r,theta] (in pixels, rads)
% numP  : number of particles
% 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function new = actionModel(prev, point, su, numP)
    new = zeros(3,numP);

    for i =1:numP
       dr = point(1) - normrnd(0,su(1));
       dt = point(2) - normrnd(0,su(2));
       
       dx = dr*cos(dt);
       dy = dr*sin(dt);
       
       new(1,i) = (prev(1,i) + dx);
       new(2,i) = (prev(2,i) + dy);
       new(3,i) = prev(3,i);
    end
end