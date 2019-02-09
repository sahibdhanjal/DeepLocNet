%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% [euclid, rssi, labels] = struct2Dist(dists)
% 
% euclid    : euclidean distances calculated
% rssi      : rssi distances calculated
% label     : LOS/NLOS => 0/1
% dists     : distance Map Struct from calculateDist
% 
% Takes distMap calculated from calculateDist
% as input and puts them into an array form 
% with labels for visualization
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [euclid, rssi, labels] = struct2Dist(dists)
    [nPts, nAP] = size(dists);
    
    euclid = zeros(nPts,nAP);
    rssi = zeros(nPts,nAP);
    labels = zeros(nPts,nAP);
    
    for i = 1:nPts
        for j = 1:nAP
            euclid(i,j) = dists(i,j).euclid;
            rssi(i,j) = dists(i,j).rssi;
            labels(i,j) = dists(i,j).label;
        end
    end
end