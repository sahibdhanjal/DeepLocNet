%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% path = localize(nump, su, sz, dmap, wpts, tx, senseR, pathUnit, start)
% 
% path          : The localized path
% su            : noise in motion model (r,theta)
% sz            : noise in measurement model
% dmap          : distance map(in pixels) from calculateDist function
% wpts          : actual waypoints of ground truth
% tx            : locations of the transmitters
% senseR        : sensing range of agent in m
% pathUnit      : m/pixels ratio
% start         : start location 
%
% Particle Filter implementation for localization using RSSI data
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function path = localize(nump, su, sz, dmap, wpts, tx, senseR, pathUnit, start)
    global numAPs doPlot tempFig map
    
    % load the trained autoencoder
    load trainedModel.mat
    
    mm = 2;                                         % action model to use - (1)odometry, (2)veloctiy
    
    if mm == 1
        rtPts = cartestian2Cylindrical(wpts);       % convert cartesian points to set of (r,theta) points
    else
        rtPts = cartesian2Velocity(wpts);          % convert cartesian points to set of velocities
    end
    
    numObs = length(rtPts);                         % number of observations/ motions
    R = senseR/pathUnit;                            % sensing range of agent in pixels
    path = [start; zeros(numObs, 2)];               % initialize path vector
    samples = startDistrib(nump, start);            % generate N particles at the start

    
    disp("Starting the Particle Filter");
    disp("=============================================");
    
    if doPlot && tempFig
        figure(tempFig);
        imshow(map)
    end
    
    for i=1:numObs
        % generate measurement samples
        if mm == 1
            samples = actionModel(samples, rtPts(i,:), su, nump);
        else
            sv = [su(1)*cos(su(2)), su(1)*sin(su(2))];
            samples = velocityModel(samples, rtPts(i,:), sv, nump);
        end
        
        % weight generated samples
        samples = measureModel(samples, dmap(i,:), nump, numAPs, tx, sz, R, stacked);
        
        % resample particles
        samples = resample(samples, nump);
        
        % calculating the median of the particles for location
        [mXY, ~] = meanVar(samples(1:2,:), nump);
        path(i+1,:) = [mXY(1),mXY(2)];
        
        if doPlot && tempFig
            hold on; axis on;
            plot(wpts(1:i,2), wpts(1:i,1),'b-s');
            plot(path(1:i,2), path(1:i,1),'g--o');
            h = plot(samples(2,:), samples(1,:), 'r.', 'MarkerSize', 4);
            drawnow;
            pause(0.3);
            delete(h);
        end
    end
    
    disp("The localized path has been generated");
    disp("=============================================");
    
end