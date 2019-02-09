%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% distMap = calculateDist(wayPts, StrengthMap, Tx, freq, pathUnit)
% 
% distMap       : Map of having dimensions (numwayPts x 2*numAPs)
% rows are      : (rssi2Dist1,rssi2Dist2,.., actualDist1,actualDist2, ...)
% 
% wayPts        : calculated waypoints from random walk
% StrengthMap   : calculated RSSI Strength Map
% Tx            : actual location of each AP (rmmbr, xAP = yIMG)
% freq          : frequency of each AP (in Hz)
% pathUnit      : m/pixel ratio
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function distMap = calculateDist(wayPts, StrengthMap, Tx, freq, pathUnit)
    global numAPs map
    numPts = length(wayPts);

    for i = 1:numPts
        pt = [wayPts(i,1), wayPts(i,2)];
        for j = 1:numAPs
            tx = [Tx(j,1), Tx(j,2)];
            rssi = StrengthMap(pt(1), pt(2),j);
            val.rssi = RSSI2Dist(rssi, freq)/pathUnit;     % convert to pixels
            val.euclid = dist(tx, pt);                     % in pixels     
            val.label = obstruction(tx,pt,map);            % 0 - LOS, 1 - NLOS            
            distMap(i,j) = val;
        end
    end
end