%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% d = dist(x,y)
% d : distance 
% x : 1st 2D point
% y : 2st 2D point
% 
% 
% given 2 points, returns the 
% euclidean distance between them
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function d = dist(x,y)
    d = sqrt( (x(2)-y(2))^2 + (x(1)-y(1))^2 );
end