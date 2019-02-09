%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% path = calculatePath(start, goal, step, debflag, maxIter)
% 
% path      : the path found by RRT planner
% start     : start point on the map
% goal      : goal point on the map
% step      : step size for each step in RRT
% debflag   : to see debug msgs (default = 0)
% maxIter   : num iterations to run RRT
% 
% Runs RRT on a given map and returns the best random
% walk from the start point up to the goal (or the 
% point nearest to the goal if goal not found)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function path = calculatePath(start, goal, step, debflag, maxIter)
    global doPlot pLen map
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Initialize Parameters
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    stepSize        = step;                     % stepSize for each step in RRT
    goalBias        = 100;                      % heuristic to generate points nearby goal (chosen random here)
    goalFound       = 0;                        % flag to mark that goal was found
    randh           = 1;                        % changes heuristic to goalBias when 2
    hThresh         = 200;                      % threshold distance to change heuristic
    bound           = 10;                       % boundary to search for inPath
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Plot Map, Start and Goal
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    [m, n] = size(map);
    
    if doPlot
        hold on;
        plot(start(2), start(1), 'rs', 'MarkerSize',6,'MarkerFaceColor',[1,0,0]);
        plot(goal(2), goal(1), 'gs', 'MarkerSize',6,'MarkerFaceColor',[0,1,0]);
    end
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Function Definitions
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % check if the cell to move to is valid or not
    isValid = @(x) x(1)>=1 && x(1)<=m && x(2)>=1 && x(2)<=n && map(x(1), x(2))==1;
    % check if the cell is the goal position or not
    isGoal = @(x) x(1)==goal(1) && x(2)==goal(2);
    % check if position is near goal or not
    nearGoal = @(x) sqrt( (x(2)-goal(2))^2 + (x(1)-goal(1))^2 ) <= stepSize;
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % RRT Search Begins Here
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % assign ctr to valid point in pathGrid
    ctr = 1; pathGrid = zeros(size(map));
    pathGrid(start(1), start(2)) = ctr;
    
    % data structure from which the best path will be extracted
    pt.x = start(1); pt.y = start(2); pt.px = NaN; pt.py = NaN;
    points = [pt];
    
    if isValid(start) && isValid(goal)
        
        for numIter = 1:maxIter
            
            % if goal is already found, stop iterations
            if goalFound
                break
            end
            
            % init random point and find nearest neighbor
            if randh == 2
                randPoint(1,1) = (goal(1) + goalBias - 2*goalBias*rand()); randPoint(1,2) = (goal(2) + goalBias - 2*goalBias*rand());
            else 
                randPoint(1,1) = m*rand(); randPoint(1,2) = n*rand();
            end
            
            % find closest point
            [curr,~] = findClosest(randPoint, pathGrid);            
            ang = atan2((randPoint(2)-curr(2)),(randPoint(1)-curr(1)));

            % calc direction to point and make move of stepSize
            dx = round(stepSize*cos(ang)); dy = round(stepSize*sin(ang));        
            temp = [curr(1) + dx, curr(2) + dy];
           
            if debflag
                % the candidate points generated from the random algorithm
                plot(temp(2), temp(1), 'go');
                fprintf("(%d, %d) : [%d, %d, %d, %d]\n", temp(1), temp(2), isValid(temp), map(temp(1),temp(2)), inPath(temp,pathGrid)==0, obstruction(temp, prev, map)==0);
            end

            % check if the point is valid or not. If yes, add to grid
            if isValid(temp) && inPath(temp,pathGrid, bound)==0 && obstruction(temp, curr, map)==0                
                ctr = ctr + 1;
                
                if doPlot
                    plot([temp(2) curr(2)], [temp(1) curr(1)], 'b--',  'LineWidth', 0.3);
                    plot(temp(2), temp(1), 'c*');
                end
                
                pt.x = temp(1); pt.y = temp(2); pt.px = curr(1); pt.py = curr(2);
                points = [points; pt];
                
                % check if point is nearGoal or is the goal itself
                if isGoal(temp) || nearGoal(temp)
                    goalFound = 1;
                    pt.x = goal(1); pt.y = goal(2); pt.px = temp(1); pt.py = temp(2);
                    points = [points; pt];
                end
                
                curr = temp;
                pathGrid(curr(1), curr(2)) = ctr;
                if dist(curr, goal) < hThresh
                    randh = 2;
                end
            end

            pause(pLen);
            drawnow;
        end
    else
        fprintf("Either Start or Goal is not a valid point");
    end
    
    if goalFound
        disp("Found the goal");
    else
        disp("RRT Finished without finding goal");
    end
    
    disp("=============================================");

    path = getPath(points, goal);
    
    disp("Overlaying the best found Path");
    plot(path(:,2), path(:,1), 'k', 'LineWidth', 2);
    hold off;
    disp("=============================================");
    
    
end