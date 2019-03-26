clas = 1;

fsize = 18; % font size
set(groot, 'defaultAxesTickLabelInterpreter','latex'); 
set(groot, 'defaultLegendInterpreter','latex');

figure(1);
axlim = [0, 104, 0, 39];

if clas== 1
    load 'trialFSNC.mat'
elseif clas==2
    load 'trialFSHC.mat'
elseif clas==3
    load 'trialFSSC.mat'
end

imagesc([axlim(1) axlim(2)], [axlim(3) axlim(4)], map);
hold on; grid on; colormap gray;

% Plot waypoints and localized path
plot(waypts(:,2), waypts(:,1),'*-b','LineWidth',2)
plot(path(:,2), path(:,1), 'o-c','LineWidth',2)

% Plot ground truth map
plot(Tx(:,2), Tx(:,1),'xk','MarkerSize',10,'LineWidth',2);
for AP = 1:length(Tx)
    str = "     AP "+num2str(AP);
    text(double(Tx(AP,2)),double(Tx(AP,1)),str,'Color','Black','FontSize',10);
end

plot(TX(:,2), TX(:,1),'xr','MarkerSize',10,'LineWidth',2);
for AP = 1:length(TX)
    str = "     AP "+num2str(TX_ids(AP)+1);
    text(double(TX(AP,2)),double(TX(AP,1)),str,'Color','Red','FontSize',10);
end

legend("Ground Truth", "Localized Path", "AP Ground Truth", "AP Localized");

% Plot Start and Goal
start = [20,80];
goal = [18,15];
plot(start(2), start(1), 'gs', 'MarkerSize', 6,'LineWidth',6, 'HandleVisibility','off');
plot(goal(2), goal(1), 'rs', 'MarkerSize', 6,'LineWidth',6, 'HandleVisibility','off');

hold off;

set(gca,'fontsize',fsize)
set(gca,'TickLabelInterpreter','latex')
figuresize(104,39,'cm')

if clas == 1
    title("Fast SLAM with no Classifier");
elseif clas ==2
    title("Fast SLAM with Hard Classifier");
elseif clas ==3
    title("Fast SLAM with Soft Classifier");
end






