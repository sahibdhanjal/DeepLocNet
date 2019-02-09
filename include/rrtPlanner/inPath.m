%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% ret = inPath(curr, grid, boundary)
% 
% ret       : 0 or 1 based on if the point(+boundary) is
%             already in the grid
% 
% curr      : a 2D point to find if it is already in path
% grid      : 2D grid in which point is to be found
% boundary  : neighborhood boundary for finding inPath
%
% Finds if the given point (+ points in boundary) are
% already included in the Path. If not there, return 0
% else return 1
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function ret = inPath(curr, grid, boundary)
    
    nbrsX = [1, 1, -1, -1];
    nbrsY = [1, -1, 1, -1];
    
    [m,n] = size(grid);
    isValid = @(x) x(1)>=1 && x(1)<=m && x(2)>=1 && x(2)<=n;
    
    for i = 0:boundary
        for j = 1:4
            x = curr(1) + i*nbrsX(j);
            y = curr(2) + i*nbrsY(j);
            if isValid([x,y])
                if grid(x,y)==0
                    ret = 0;
                else
                    ret = 1;
                end
            end
        end
    end
    
end