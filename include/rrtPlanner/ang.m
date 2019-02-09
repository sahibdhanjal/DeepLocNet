%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% t = ang(x,y)
% t : angle in radians
% x : 1st 2D point
% y : 2st 2D point
% 
% given 2 points, returns the 
% angle between them
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function t = ang(x,y)
    t = atan2((y(2)-x(2)), (y(1)-x(1)));
end