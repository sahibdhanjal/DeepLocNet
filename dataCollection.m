clear; clc; close all;
addpath(genpath('./include/'));

global doPlot numAPs makeFig pLen mapName

% parameters for RSSI Generation
freq    = 2.412e9;                      % fequency range of router in Hz
res     = 20;                           % map resolution (higher the better, but make process slower)
model   = 2;                            % 1=Free-Space, 2=Motley-Keenan (COST231)
rand    = 1;                            % random router placement: 1 = true, 0 = false
debflag = 0;                            % debug flag

% parameters for random walk
maxIter = 1e4;                          % max iterations for RRT
step    = 20;                           % stepSize for random walk

% plot parameters
makeFig = 1;                            % global figure number
doPlot  = 0;                            % set to 1 if you want to make plot
pLen    = 0;                            % pause length for animation

% data collection parameters
nMaps   = 100;                           % number of times map to be selected
niter   = 100;                           % number of iterations for each map
maxAPs  = 15;                           % maximum number of APs allowed

%% Select Map
% input the map name in advance
if ispc
    filePath = string(pwd)+"\assets\";
elseif isunix
    filePath = string(pwd)+"/assets/";
end

mapNames = ["simple.bmp", "1room.bmp", "2room.bmp", "3room.bmp", "binb.bmp", "binbcuts.bmp", ...
            "floorplan0.bmp", "floorplan1.bmp", "floorplan2.bmp", "floorplan3.bmp", "nroom.bmp"];

startPts = [340, 20;
            270, 80;
            310, 50;
            270, 80;
            300, 50;
            340, 20;
            340, 90;
            20, 30;
            20, 280;
            340, 20;
            50, 50];

goalPts = [20, 440;
           40, 400;
           80, 350;
           300, 400;
           50, 400;
           50, 385;
           170, 420;
           100, 430;
           320, 380;
           220, 445;
           50, 400];

data = [];

for m = 1:nMaps
    idx = randi(length(mapNames));
    fileName = mapNames(idx);
    mapName = char(filePath+fileName);
    numAPs  = randi(maxAPs);
    start   = startPts(idx,:);
    goal    = goalPts(idx,:);
    
    %% Calculate Map
    [StrengthMap, Tx, floorPlanBW, pathUnit] = calculateMap(freq, res, rand, model);

    %% Data Collection
    for i = 1:niter
        clf; clear wayPts dists;
        wayPts = calculatePath(start, goal, step, floorPlanBW, debflag, maxIter);
        dists = calculateDist(wayPts, StrengthMap, Tx, freq, pathUnit, floorPlanBW);
        for j = 1:numAPs
            data = [data;dists(:,j)];
        end
    end
end

%% Convert to usable data and store in .mat file
[euclid, rssi, labels] = struct2Dist(data);
save('data.mat', 'euclid', 'rssi', 'labels');