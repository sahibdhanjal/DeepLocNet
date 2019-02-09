clear; clc; close all;
addpath(genpath('./include/'));

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Set preliminary parameters
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
global doPlot numAPs makeFig finalFig tempFig doGif pLen useClas hardClas map

% parameters for RSSI Generation
minAPs  = 3;                            % minimum number of APs required 
maxAPs  = 10;                           % maximum number of APs allowed
numAPs  = randi([minAPs maxAPs]);       % number of access points in the map
freq    = 2.412e9;                      % fequency range of router in Hz
res     = 30;                           % map resolution (higher the better, but make process slower)
model   = 2;                            % 1=Free-Space, 2=Motley-Keenan (COST231)
rand    = 1;                            % random router placement: 1 = true, 0 = false
debflag = 0;                            % debug flag

% parameters for random walk
maxIter = 1e8;                          % max iterations for RRT
start   = [336, 87];                    % start point for random walk
goal    = [173, 420];                   % goal point for random walk
step    = 20;                           % stepSize for random walk

% parameters for localization
numP    = 100;                          % number of particles for localization
sigU    = [rand()*20 , rand()*0.25];    % noise in motion in r(in pixels),theta(in rads)
sigZ    = randi([10 100],1,numAPs);     % noise in measurements (random between 10-100 in pixels)
senseR  = 15;                           % sensing range of agent (in m) (cannot detect all routers all the time)
useClas = 1;                            % flag for using or not using classification for LOS/NLOS
hardClas= 0;                            % hard classification (reject NLOS totally) or soft classification (weight with p)
slam    = 0;                            % run (1) Particle Filter Localization , (2) Fast SLAM (unknown prior AP map)

% plot parameters
makeFig = 1;                            % global figure number
doPlot  = 1;                            % set to 1 if you want to make plot
finalFig= 2;                            % gif made out of this fig
tempFig = 3;                            % figure for temporary plotting and debugging (set 3 if on)
doGif   = 1;                            % set to 1 if you want to make GIF
pLen    = 0.1;                          % pause length for animation

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Calculate the WiFi strength Map
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
[StrengthMap, Tx, map, pathUnit] = calculateMap(freq, res, rand, model);

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Calculate the Random Walk
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
wayPts = calculatePath(start, goal, step, debflag, maxIter);

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Calc distance and label data of each AP at each waypoint
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
dists = calculateDist(wayPts, StrengthMap, Tx, freq, pathUnit);

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Start Localization Based on Particle Filter
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
if slam
    [path,APLocs, ID] = localizeFast(numP, sigU, sigZ, dists, wayPts, Tx, senseR, pathUnit, start);
else
    path = localize(numP, sigU, sigZ, dists, wayPts, Tx, senseR, pathUnit, start);
end
[err, CDF] = getError(path, wayPts);

if useClas
    fprintf("The error in the ground truth and localization (with classifier) is %f\n",err);
else
    fprintf("The error in the ground truth and localization (without classifier) is %f\n",err);
end
disp("=============================================");

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Plotting
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

if doPlot
    h = figure(finalFig);
    imshow(map);
    hold on; axis on tight manual;
    
    if useClas
        filename = 'localization_with_Classifier.gif';
    else
        filename = 'localization_without_Classifier.gif';
    end
    
    title('Radio Inertial Localization');
    xlim([min(0, round(min(path(:,2)))-50), max(size(map, 2),max(path(:,2)))+50]);
    ylim([min(0, round(min(path(:,1)))-50), max(size(map, 1),max(path(:,1)))+50]);

    for AP = 1:numAPs
        str = "AP "+num2str(AP);
        text(Tx(AP,2),Tx(AP,1),str,'Color','Red','FontSize',8);
    end
    
    if slam
        for AP = 1:length(ID)
            str = "AP "+num2str(ID(AP));
            text(APLocs(AP,2),APLocs(AP,1),str,'Color','Black','FontSize',8);
        end
    end

    for i = 2:length(wayPts)
        plot( [wayPts(i,2) wayPts(i-1,2)], [wayPts(i,1) wayPts(i-1,1)], 'g-o', 'LineWidth', 1.5);
        plot( [path(i,2) path(i-1,2)], [path(i,1) path(i-1,1)], 'b--*', 'LineWidth', 1.5);

        drawnow;
        pause(0.3);

        if doGif
            % Capture the plot as an image 
            frame = getframe(h); 
            im = frame2im(frame); 
            [imind,cm] = rgb2ind(im,256); 

            % Write to the GIF File 
            if i == 2
                imwrite(imind,cm,filename,'gif', 'Loopcount',inf); 
            else
                imwrite(imind,cm,filename,'gif','WriteMode','append');
            end
        end

    end
    legend('Ground Truth', 'Localization');
    
    plotCDF(CDF);
end