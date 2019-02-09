%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% [model,err] = Classify(X, Y, ratio)
% 
% model         : fitted model to the training data
% err           : testing MSE
% X             : data
% Y             : labels (LOS = 0 | NLOS = 1)
% ratio         : train/test ratio
% 
% Neural Net to Classify between LOS/NLOS
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [model,err] = Classify(X, Y, ratio)    
    X = X'; Y = Y';

    % divide test and train set
    n = length(X); nTrain = round(ratio*n); idx = randperm(n);

    trainX = X(:, idx(1:nTrain)); trainY = Y(:, idx(1:nTrain)); 
    testX = X(:, idx(nTrain+1:end)); testY = Y(:, idx(nTrain+1:end));

    model = patternnet([100 100 100]);

    [model,tr] = train(model,trainX,trainY);
    nntraintool

    plotperform(tr)

    testLab = model(testX);
        
    [err,~] = confusion(testY,testLab);

    % fprintf('Percentage Correct Classification   : %f%%\n', 100*(1-err));
    % fprintf('Percentage Incorrect Classification : %f%%\n', 100*err);
    
end