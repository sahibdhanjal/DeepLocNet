%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% rtPts = cartesian2Velocity(wpts)
% 
% rtPts     : velocities in cartesian coordinates
% wpts      : points in cartesian coordinates
% 
% converts cartesian points to velocities
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function rtPts = cartesian2Velocity(wpts)
    n = length(wpts);
    rtPts = zeros(n-1,2);
    
    for i=2:n
        dx = wpts(i,1) - wpts(i-1,1);
        dy = wpts(i,2) - wpts(i-1,2);
        rtPts(i-1,:) = [dx,dy];
    end
end