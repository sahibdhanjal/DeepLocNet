%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Y = function getLabel(X, enc)
% 
% X     : Input(euclid; rssi)
% Y     : Output Label
% enc   : Trained Encoder Model
% Loads trained model and returns label
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function Y = getLabel(X, enc)
    out = enc(X);
    
    if out(1)>out(2)    % LOS
        Y = 0;
    else                % NLOS
        Y = 1;
    end
end