%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Yenc = oneHotEncode(Y, numC)
% 
% Yenc  : one hot encoded labels
% Y     : original labels
% numC  : number of classes in labels
% 
% One hot encodes the labels for use with softmax
% Refer - https://machinelearningmastery.com/why-one-hot-encode-data-in-machine-learning/
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function Yenc = oneHotEncode(Y, numC)
    n = length(Y);
    Yenc = zeros(numC,n);
    
    for i=1:n
        if Y(i)==0
            Yenc(1,i) = 1;
        else
            Yenc(2,i) = 1;
        end
    end
 
end