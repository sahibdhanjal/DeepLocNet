%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% plots CDF given an array
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function plotCDF(X)

    xaxis = cumsum(X);
    yaxis = cumsum(X)./sum(X);
    
    figure(4);
    plot(xaxis, yaxis,'bx');
    title("Localization Performance");
    xlabel("Localization Error");
    ylabel("CDF Fraction");
    
end