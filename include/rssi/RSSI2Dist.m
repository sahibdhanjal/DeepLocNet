%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% d = RSSI2Dist(rssi, freq)
% 
% d     : distance (in m)
% rssi 	: rssi (in dB)
% freq 	: router frequency (in Hz)
% 
% Calculates the distance of the AP
% based on the RSSI Signal based on
% the Free Space Path Loss Model
% refer : https://en.wikipedia.org/wiki/Free-space_path_loss
% refer : https://stackoverflow.com/questions/11217674/how-to-calculate-distance-from-wifi-router-using-signal-strength
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function d = RSSI2Dist(rssi, freq)
    fMHz = freq/1e6;                                    % convert freq to Mhz
    exp = (47.55 - 20*log10(fMHz) + abs(rssi))/20;      % exponent
    d = 10^exp;                                         % distance in m
end