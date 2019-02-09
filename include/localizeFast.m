%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% [path,APLocs] = localizeFast(nump, su, sz, dmap, wpts, tx, senseR, pathUnit, start)
% 
% path          : The localized path
% APLocs        : AP locations as calculated by Fast SLAM
% su            : noise in motion model (r,theta)
% sz            : noise in measurement model
% dmap          : distance map(in pixels) from calculateDist function
% wpts          : actual waypoints of ground truth
% tx            : locations of the transmitters
% senseR        : sensing range of agent in m
% pathUnit      : m/pixels ratio
% start         : start location 
%
% Fast Slam v1 implementation for localization using RSSI data
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [path,APLocs, ID] = localizeFast(nump, su, sz, dmap, wpts, tx, senseR, pathUnit, start)
    
    global doPlot tempFig map numAPs
    
    load trainedModel.mat stacked                   % load the trained autoencoder
    rtPts = cartesian2Velocity(wpts);               % convert cartesian points to set of velocities
    numObs = length(rtPts);                         % number of observations/ motions
    R = senseR/pathUnit;                            % sensing range of agent in pixels
    path = [start; zeros(numObs, 2)];               % initialize path vector
    samples = initParticles(nump, start);           % generate N particles at the start
    sv = [su(1)*cos(su(2)), su(1)*sin(su(2))];      % convert cylindrical to cartesian space
    
    disp("Starting FastSLAM v1");
    disp("=============================================");
    
    if doPlot && tempFig
        figure(tempFig);
        imshow(map);
        for AP = 1:numAPs
            str = "AP "+num2str(AP);
            text(tx(AP,2),tx(AP,1),str,'Color','Red','FontSize',10);
        end
    end
    
    
    for i=1:numObs
        % velocity action model
        samples = fastActionModel(samples, rtPts(i,:), sv);
        
        % measurement model
        samples = fastMeasureModel(samples, dmap(i,:), tx, sz, R, stacked);
        
        % resample particles if Neff above a threshold
        Neff = calcSum(samples);
        if Neff <= 0.33*nump
            samples = fastResample(samples);
        end
        
        % calculating the median of the particles for location
        [mXY, ~] = fastMeanVar(samples);

        path(i+1,:) = [mXY(1),mXY(2)];
        
        [APLocs,~,ID] = getAPLocs(samples);
        
        if doPlot && tempFig
            hold on; axis on;
            plot(wpts(1:i,2), wpts(1:i,1),'b-s');
            plot(path(1:i,2), path(1:i,1),'g--o');
            
            % get particles and display
            particles = getParticles(samples, nump);
            p = plot(particles(2,:), particles(1,:), 'c.', 'MarkerSize', 4);
            
            % display AP Locations
            ap = [];
            for AP = 1:length(ID)
                str = "AP "+num2str(ID(AP));
                ap = [ap;text(APLocs(AP,2),APLocs(AP,1),str,'Color','Black','FontSize',10)];
            end
            
            drawnow; pause(0.2); delete(p); 
            if i<numObs
                delete(ap);
            end
        end
        
    end
    
    disp("The localized path has been generated");
    disp("=============================================");
    
    % get AP Locations/ Variance/ IDs
    [APLocs, ~ , ID] = getAPLocs(samples);
    % Print list of routers that have been detected
    printMsg(ID);
        
end

function Neff = calcSum(samples)
    nP = length(samples);
    wghts = zeros(nP,1);
    for i=1:nP
        wghts(i) = samples(i).w;
    end
    s = sum(wghts);
    Neff = 1/sum((wghts./s).^2);
end

function printMsg(ID)
    disp("=============================================");    
    fprintf("The APs detected are: ");
    for i=1:length(ID)
       fprintf("%s ",num2str(ID(i)));
    end
    fprintf("\n");
    disp("=============================================");
end

function p = getParticles(s,np)
    p = zeros(2,np);
    for i=1:np
        p(1,i) = s(i).pose(1);
        p(2,i) = s(i).pose(2);
    end
end