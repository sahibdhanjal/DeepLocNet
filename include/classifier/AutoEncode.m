%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% [enc, MSE] = AutoEncode(X, Y, params)
% 
% enc       : encoded representation of the data
% MSE       : mean square error in encoding
% X         : [euclid, rssi]
% Y         : labels
% params    : Network Parameters
% 
% Autoencodes the data giving a better representation
% of the function and returns the encoder with MSE
% 
% refer : 
% https://www.mathworks.com/help/deeplearning/examples/train-stacked-autoencoders-for-image-classification.html
% https://www.mathworks.com/help/deeplearning/ug/construct-deep-network-using-autoencoders.html
% https://www.mathworks.com/help/releases/R2015a/nnet/examples/training-a-deep-neural-network-for-digit-classification.html
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [stackednet] = AutoEncode(X, Y, params)
    
    autoencoders = cell(params.numLayers,0);
    train = X;
    
    for i = 1:params.numLayers
        encoder = trainAutoencoder( train, params.hiddenSize(i), ...
                                    'MaxEpochs', params.maxEpochs(i), ...
                                    'L2WeightRegularization', params.L2(i), ...
                                    'SparsityRegularization', params.SparsityReg(i), ...
                                    'SparsityProportion', params.SparsityPro(i), ...
                                    'ScaleData', params.scaleData(i), ...
                                    'UseGPU', params.useGPU);
        train = encode(encoder, train);
        autoencoders{i} = encoder;
    end

    softnet = trainSoftmaxLayer(train,Y,'MaxEpochs',params.softNetEpochs);
    stackednet = stack(autoencoders{:,:},softnet);
    
end