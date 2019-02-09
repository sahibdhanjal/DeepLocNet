%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% [point,minDist] = findClosest(rand, grid)
% 
% rand : a 2D point to find nearest neighbor
% grid : 2D grid in which neighbor is to be found
% 
% Finds the closest point to an input 
% point, that has been marked visited 
% in a grid. Takes a grid and point as 
% inputs and outputs the closest visited
% point and it's distance in the grid
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [point, minDist] = findClosest(rand, grid)
    grid = (grid>0);
    [row, col] = find(grid==1);
    numPoints = size(row,1);
    minDist = 9999999;
    
    for i = 1:numPoints
       dx = rand(1) - row(i);
       dy = rand(2) - col(i);
       dist = sqrt(dx^2 + dy^2);
       
       if dist<minDist
           point = [row(i), col(i)];
           minDist = dist;
       end     
    end
    
end