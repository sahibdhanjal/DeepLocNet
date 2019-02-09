%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Bresenham's algorithm to find if any point
% between 2 given points on grid is obstructed
% or is an obstacle
% 
% ret = obstruction(curr, prev, grid)
% ret : 1       => obstruction
% ret : 0       => no obstruction
% 
% curr  : first 2D point
% prev  : second 2D point
% grid  : the map in which obstruction 
%         needs to be found
% 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function ret = obstruction(curr, prev, grid)
    ret = 0;
    [x,y] = bresenham(curr(1), curr(2), prev(1), prev(2));
    numPoints = length(x);
    
    for i=1:numPoints
        if(grid(x(i), y(i))==0)
            ret = 1;
            break;
        end
    end
end