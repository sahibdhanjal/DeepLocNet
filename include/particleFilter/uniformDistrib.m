% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% samples = uniformDistrib(np, map)
% 
% samples   : particles (x, y, weight)
% np        : number of particles
% map       : the map on which particles are to be generated
% 
% Generates uniform 2D distribution of particles on a given map
% 
% refer following link for point distribution algorithm:
% https://math.stackexchange.com/questions/1039482/how-to-evenly-space-a-number-of-points-in-a-rectangle
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [samples, numP] = uniformDistrib(np, map)
    samples = zeros(3,np);
    [w,h] = size(map);
    
    % calculate the number of particles in x and y direction
    nx = sqrt((w/h*np) +  (w-h)^2/(4*h*h)) - (w-h)/(2*h);
    nx = round(nx); ny = ceil(np/nx); numP = nx*ny;
    
    ctr = 1;
    for i = 1:nx+1
        for j = 1:ny+1
            % initial location of the sample
            samples(1,ctr) = round((i-1)*w/nx);
            samples(2,ctr) = round((j-1)*h/ny);

            % initial weight of the sample
            samples(3,ctr) = 1/numP;
            
            ctr = ctr + 1;
        end
    end
    
end