%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% path = getPath(grid, ctr)
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
function path = getPath(points, goal)
    
    % print the points for debugging purposes
    % printPoints(points);
    
    if points(end).x==goal(1) && points(end).y == goal(2)
        last = goal;
    else
        % find the nearest point in the Tree if goal isn't found
        last = nearestToGoal(points, goal);
        fprintf("last point: (%d, %d)\n", last(1), last(2));
    end
    
    path = last;
    while ~isnan(last(1)) && ~isnan(last(2))
        parent = findParent(last, points);
        if ~isnan(parent(1)) && ~isnan(parent(2))
            path = [path; parent];
        end
        last = parent;
    end
   
    path = flipud(path);
    
end

% given a tree structure, find the point
% nearest to the goal in that tree
function pt = nearestToGoal(points, goal)
    minDist = 999999;
    for i = 1:length(points)
        d = dist([points(i).x, points(i).y], goal);
        if  d < minDist
            pt = [points(i).x, points(i).y];
            minDist = d;
        end
    end
end

% given a point and tree structure, find the parent
% of the given point in the tree
function pt = findParent(p, points)
    pt = [];
    for i = 1:length(points)
       if points(i).x == p(1) && points(i).y == p(2)
           pt = [points(i).px, points(i).py];
           break;
       end
    end
    
end

% function to print the RRT given the structure
function printPoints(pt)
    for i=1:length(pt)
        fprintf("child:(%d,%d)  |  parent:(%d,%d)\n", pt(i).x, pt(i).y, pt(i).px, pt(i).py);
    end
end