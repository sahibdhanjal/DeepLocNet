%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% rtPts = cartestian2Cylindrical(wpts)
% 
% rtPts     : points in spherical (or cylindrical coordinates)
% wpts      : points in cartesian coordinates
% 
% converts cartesian points to spherical coordinates
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function rtPts = cartestian2Cylindrical(wpts)
    n = length(wpts);
    rtPts = zeros(n-1,2);
    
    for i=2:n
        r = dist(wpts(i-1,:), wpts(i,:));
        t = ang(wpts(i-1,:), wpts(i,:));
        rtPts(i-1,:) = [r,t];
    end
end