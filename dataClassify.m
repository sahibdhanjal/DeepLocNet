clear; clc; close all;
addpath(genpath('./include/'));

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Load Data and Preprocess
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
load('./data/trainLarge.mat');
X = [euclid, rssi]; Y = labels;
encode = 1; fineTuning = 1;
scale = 0.15; setRatio = 1.1; trainTestRatio = 0.7;

% scale data - as in use only some part of the dataset
idx = randperm(length(Y)); n = round(scale*length(Y));
X = X(idx(1:n),:); Y = Y(idx(1:n),:);

% Preprocess data and one hot encode
[X,Y] = PreProcess(X, Y, setRatio);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Define Autoencoder Parameters
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
params.numLayers = 4; params.useGPU = true;
params.hiddenSize = [10, 20, 50, 100];
params.L2 = [0.004, 0.002, 0.001, 0.001];
params.maxEpochs = [1000, 1000, 1000, 1000];
params.SparsityReg = [4, 4, 4, 4];
params.SparsityPro = [0.15, 0.15, 0.15, 0.1];
params.scaleData = [false, false, false, false];
params.softNetEpochs = 1000; 

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Find Autoencoded Representation or classify directly
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
if encode
    %% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Train Stacked AutoEncoder
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    % divide train and test set
    n = length(X); nTrain = round(trainTestRatio*n); idx = randperm(n);
    trainX = X(:, idx(1:nTrain)); trainY = Y(:, idx(1:nTrain));
    testX = X(:, idx(nTrain+1:end)); testY = Y(:, idx(nTrain+1:end));
    
    % train autoencoder with given architecture
    stacked = AutoEncode(trainX, trainY, params);
    view(stacked)
    
    % fine tune weights of network - not always necessary
    if fineTuning
        trainX = convert2Arr(trainX);
        stacked = train(stacked,trainX,trainY,'UseGPU','yes');
    end

    % convert test set from cell to array for testing and use NN
    testX = convert2Arr(testX);
    y = stacked(testX);

    % calculate confusion and plot confusion matrix
    errClassify = confusion(testY,y);
    fprintf('Percentage Correct Classification : %f%%\n', 100*(1-errClassify));
    plotconfusion(testY,y);    

else
    %% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Classify between LOS/NLOS
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    [classifier,errClassify] = Classify(X, Y, trainTestRatio);
    fprintf('Percentage Correct Classification : %f%%\n', 100*(1-errClassify));
end