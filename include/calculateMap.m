%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% [StrengthMap, Tx, floorPlanBW, pathUnit] = calculateMap(f, res, r, model)
% 
% StrengthMap   : WiFi RSSI Map for each router
% Tx            : Transmitter (AP) locations
% floorPlanBW   : Rectified Image (boolean) of input map
% pathUnit      : m/pixel ratio
% 
% f             : frequency for each router (in Hz)
% res           : resolution of the structure for constructing RSSI Map
% r             : flag to check if you want to place APs randomly or not
% model         : use (1)FreeSpace or (2)COST231 Model for Propagation
% 
% Runs the desired signal propagation model and outputs the RSSI Strength
% map for each AP, along with their locations and the image of the map
% 
% 
% Code adapted from Salaheddin Hosseinzadeh and modified for multi-router
% scenario for 2D propagation.
% Refer - https://superuser.com/questions/298290/wireless-router-signal-strength-stats
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [StrengthMap, Tx, floorPlanBW, pathUnit] = calculateMap(f, res, r, model)
    global doPlot numAPs makeFig mapName
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Initialize Base Parameters
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    lightVel                = 3e8;                          % Light velocity (m/s)
    freq                    = f;                            % frequency of WiFi in Hz
    attendB                 = 0;                            % 1 if attenuation in dB, other wise linear
    pathLossModel           = model;                        % 1=Free-Space, 2=Motley-Keenan (COST231)
    demoMode                = 0;                            % Showes wall identification and other params
    TxPower                 = 17.5;                         % Transmission Power dBm or dB
    antennaLoss             = 1.5;                          % Antenna Loss dB
    TxAntennaGain           = 0 + antennaLoss ;             % Gain of Transmitting antenna
    RxAntennaGain           = TxAntennaGain;                % Gain of Receiving antenna
    meshNode.vert.num       = res;                          % Resolution of structure in y (more is better)
    meshNode.horz.num       = res;                          % Resolution of structure in x (more is better)
    randomPlace             = r;                            % random placement of routers or not
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Model and Wall Parameters
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Free Space Path Loss Model Parameters
    d0FSPL                  = 1;      % Reference Distance (meters) for Free Space Path Loss model
    % Multi-Wall Model Parameters
    d0Cost231               = 1;      % Multi-wall model reference distance
    % Wall Detection Parameters
    thetaRes = 0.1;                   % Resolution of the Hough Transform Space (don't make it smaller than 0.1)
    minWallLength = 15;               % Minimum length of the walls in pixel
    fillGap = 5;                      % Gap between walls

    % Wall Attenuation Coefficient Assignment dB
    % Manually Assign Attenuation factors to each particular wall based on it's
    % intensity dynamic range (change plot mode in autoWallDetection to 1 to
    % see the intesity of each wall.
    wallAt = zeros(255,1);
    wallAt(200:end) = 6;
    wallAt(254) = 5.5;
    wallAt(253) = 6;
    wallAt(252) = 6;
    wallAt(255) = round(sum(wallAt)./sum(wallAt>0));  % This is for intersecting walls, just leave it as it is

    %% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Read Image Input from User
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    if isempty(mapName)
        try
            [fileName,filePath] = uigetfile('*.*');
        catch
        end
        floorPlan = imread([filePath,fileName]);
    else
        floorPlan = imread(mapName);
        pathUnit = 0.0625;
    end
    
    floorPlanBW = ~im2bw(floorPlan);
    [imR,imC,~] = size(floorPlan);

    % Adjust the image if required: complements 
    % the image if not in correct form so that the 
    % structure will be in Black in case it's not
    try
        [co,bi] = imhist(floorPlanBW);

        if bi(1) == 0 && co(1) > co(2)
            floorPlanBW = ~floorPlanBW;
        end
    catch
    end

    %% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Image Dilation/Calibration and Wall Selection
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    originalFloorPlan = floorPlanBW;
    floorPlanBW = ~imdilate(~floorPlanBW,strel('disk',2));
    
    figure(makeFig);
    imshow(floorPlanBW,'InitialMagnification',100);
    title('Floor Plan');
    disp("=============================================");
    
    if isempty(mapName)
        disp('Select two points from walls to calibrate the plan!');

        % Getting 2 points from image for calibration
        while 1
            try
                [r,c] = ginput(1);
                R(1,1)= round(r);
                C(1,1) = round(c);
            catch
            end
            if ~floorPlanBW(C(1,1),R(1,1))
                fprintf("First Point: (%d,%d)\n", C(1,1), R(1,1));
                break
            end
        end
        while 1
            [r,c] = ginput(1);
            R(2,1) = round(r);
            C(2,1) = round(c);
            if ~floorPlanBW(C(2,1),R(2,1))
                fprintf("Second Point: (%d,%d)\n", C(2,1), R(2,1));
                break
            end
        end
        disp("=============================================");
        % Finding shortest path betweent the two points
        calibPath = shortestPath(imcomplement(floorPlanBW),R,C);
        % Calibrates pixel per meter as per the user path specified
        pathPixels = sum(sum(calibPath));
        pathLength = input('Length of calibration path in meters: ');
        % pathUnit (in meter/pixel)
        pathUnit = pathLength./pathPixels;
        disp("=============================================");
    else
        pathUnit = 0.0625;
    end

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Meshing the Floor Plan
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    floorMesh = zeros(size(floorPlanBW));
    mesh.vert.spacing = pathUnit .* size(floorPlanBW,1) ./ meshNode.vert.num;
    mesh.horz.spacing = pathUnit .* size(floorPlanBW,2) ./ meshNode.horz.num;
    floorMesh(floor(linspace(1,size(floorPlanBW,1),meshNode.vert.num)),...
        floor(linspace(1,size(floorPlanBW,2),meshNode.horz.num))) = 1;


    %% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Put the Transmitter in a Location
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    title('Transmitter Locations');

    % store locations of transmitters
    if randomPlace
        col = round(imC*rand(numAPs,1));
        row = round(imR*rand(numAPs,1));
        Tx = [row, col]; 
    else
        Tx = zeros(numAPs, 2);
    end

    for AP = 1:numAPs
        if ~randomPlace
            try
                Tx(AP,:) = ginput(1);
            catch
            end
        end

        fprintf("AP%d Location: (%d,%d)\n", AP, int32(Tx(AP,1)), int32(Tx(AP,2)));
        str = "AP "+num2str(AP);
        text(Tx(AP,2),Tx(AP,1),str,'Color','Red','FontSize',8);
    end

    disp("=============================================");


    %% Calculating mesh points distance from Tx
    [Rxr,Rxc] = find(floorMesh == 1); % finding the nodes

    lossdB = zeros(size(Rxr,1),numAPs);

    if pathLossModel == 2
        % Detecting all the walls Generates floorPlanGray where different wall are index coded in the gray image.    
        [floorPlanGray,~] = autoWallDetection(~originalFloorPlan,wallAt,demoMode,thetaRes,minWallLength,fillGap);

        % LOS & Walls Determination
        % Thining the floor plan. Only one pixel per wall should intersect with LOS  
        thinFloorPlanBW = ~ originalFloorPlan;
        thinFloorPlanBW = bwmorph(thinFloorPlanBW,'thin','inf');
        thinFloorPlanBW = bwmorph(thinFloorPlanBW,'diag');
        losC = cell(size(Rxr,1),1);
        losR = cell(size(Rxr,1),1);
        losTemp = zeros(size(thinFloorPlanBW));
        wallsType = cell(size(Rxr,1),1); % pre-defining
        numWalls = zeros(size(Rxr,1),1);
    end

    fprintf("Wave Propagation Model calulated for AP:");
    for AP = 1:numAPs
        Txr = Tx(AP,1);
        Txc = Tx(AP,2);

        dRxTxr = Rxr - Txr; % distance in terms of pixels
        dRxTxc = Rxc - Txc; % distance in terms of pixels 
        nodeDistance = sqrt(dRxTxr.^2 + dRxTxc.^2) * pathUnit; % distance in terms of meters

        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % Indoor Propagation Models
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % FREE SPACE PATH LOSS 
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        if pathLossModel == 1
            lossdB(:,AP) = 20 * log10(4*pi.*nodeDistance.*freq./lightVel) .* (nodeDistance > d0FSPL) + (nodeDistance < d0FSPL) .* 20 * log10(4*pi.*d0FSPL.*freq./lightVel);
            % loss = (4*pi.*nodeDistance*freq./lightVel).^2;
            lossdB(:,AP) = TxPower - lossdB(:,AP) + RxAntennaGain + TxAntennaGain;
        end

        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % COST231 MODEL
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

        if pathLossModel == 2
            for i = 1:numel(Rxr)
                [losC{i},losR{i}] = bresenham(Txc,Txr,Rxc(i),Rxr(i)); %LOS between Tx & Rx
                for j = 1:numel(losC{i}(:))
                    losTemp(losR{i}(j),losC{i}(j)) = 1; % temporary line of sight image
                end
                [wallsLable,numWalls(i)] = bwlabel(losTemp .* thinFloorPlanBW,8); % find intersection of LOS and walls
                wallsLable = bwmorph(wallsLable,'shrink','inf');
                wallsType{i} = unique(double(floorPlanGray).*wallsLable);  % type of the walls (grayscale) between each Tx to Rx

                % calculating the total wall attenuation for each beam
                wallLoss = 0;
                for k = 2:numel(wallsType{i})
                    wallLoss =  wallAt(wallsType{i}(k)) + wallLoss;
                end
                % Calculating the signal strength
                lossdB(i,AP) = ((20 * log10(4*pi.*nodeDistance(i).*freq./lightVel) .* (nodeDistance(i) > d0Cost231)) + ((nodeDistance(i) < d0Cost231) .* 20 * log10(4*pi.*d0Cost231.*freq./lightVel)) ) ...
                    + abs(wallLoss);
                lossdB(i,AP) = TxPower - lossdB(i,AP) + RxAntennaGain + TxAntennaGain;
                losTemp = zeros(size(thinFloorPlanBW)); % clears the LOS image
            end
        end
        fprintf(" %d",AP);
    end
    fprintf("\n");
    disp("=============================================");

    %% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Calculating Strengths of each router throughout
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    StrengthMap = zeros(imR, imC, numAPs);

    for AP = 1:numAPs
        % WiFi signal at each point on grid
        smallFSPLImage = (reshape(lossdB(:,AP),meshNode.vert.num, meshNode.horz.num));
        % bicubic interpolation for overlay over image
        StrengthMap(:,:,AP) = (imresize(smallFSPLImage,[imR,imC],'method','cubic'));
    end
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Visualizing Distribution of cumulation of all routers
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    if doPlot
        InterpolatedMap = mean(StrengthMap, 3);
        % Normalizing to 0-1 for color map application
        FSPLFullImage = mat2gray(InterpolatedMap);
        z = imoverlay(FSPLFullImage,~originalFloorPlan,[0,0,0]);
        imshow(rgb2gray(z));
        colormap(gca,'bone');
        caxis([-100 300]);
        for i = 1:7
            colorbarLabels(i) = min(lossdB(:,AP)) + i .* ((max(lossdB(:,AP))-min(lossdB(:,AP)))./7);
        end
        colorbar('YTickLabel',num2str(int32(colorbarLabels')));
    end
    
    for AP = 1:numAPs
        str = "AP "+num2str(AP);
        text(Tx(AP,2),Tx(AP,1),str,'Color','Red','FontSize',8);
        title('Radio-Inertial Localization');
    end
end