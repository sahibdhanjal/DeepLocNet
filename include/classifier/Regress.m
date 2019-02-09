%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% [model,MSE_train, MSE_test] = Regress(X, ratio)
% 
% model         : fitted model to the training data
% MSE_train     : training MSE
% MSE_test      : testing MSE
% X             : data ({euclid, rssi})
% ratio         : train/test ratio
% 
% Regresses function to fit 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [model,MSE_train, MSE_test] = Regress(X, ratio)

    n = length(X); nTrain = round(ratio*n);
    idx = randperm(n);

    % divide test and train set
    XTrain = X(idx(1:nTrain), :); XTest = X(idx(nTrain+1:end), :);
    model = fit(XTrain(:,1), XTrain(:,2),'exp1');
    
    % calculate MSE on train and test data
    rssiTrain = model.a*exp(model.b*XTrain(:,1));
    rssiTest = model.a*exp(model.b*XTest(:,1));
    
    MSE_train = mse(rssiTrain-XTrain(:,2));
    MSE_test = mse(rssiTest-XTest(:,2));

end