%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% [Xout, Yout] = PreProcess(X, Y, setRatio)
% 
% Xout      : X output (put into cell form and ratio of NLOS/LOS <= setRatio )
% Yout      : Y output (one hot encoded)
% 
% X         : input data (with ratio of NLOS/LOS >setRatio)
% Y         : input labels in array
% setRatio  : desired ratio of NLOS/ LOS
% 
% takes the input data which has a much larger proportion of NLOS 
% values and selects few indices out of NLOS such that ratio of 
% NLOS:LOS is setRatio. It also converts the features to a cell array
% and the labels to one hot encoded labels
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [Xout, Yout] = PreProcess(X, Y, setRatio)
    % find LOS data
    idx = find(Y==0);
    nNLOS = sum(Y); nLOS = length(Y) - nNLOS;
    
    % find NLOS data
    NLOS = find(Y==1);
    msize = numel(NLOS);
    idx = [idx;NLOS(randperm(msize, round(nLOS*setRatio)))];
    
    Xout = formCell(X(idx,:)); Yout = oneHotEncode(Y(idx,:), 2);
end