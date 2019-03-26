%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Mat file description:

% mseFSHC - list of all MSE recorded for the 10 trials of FS with Hard
% Classification

% errFSHC - waypoint wise error during one run of FS with Hard Classification
% errHC - waypoint wise error during one run of PF with Hard Classification

% map - map used for the experiment
% HC - Hard Classification
% SC - Soft Classification
% NC - No Classification
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clear; clc; close all;

load('3D plots.mat')

msePFNC = [2.2675, 1.0241, 1.8646, 1.5354, 4.7697, 4.2071, 1.6249, 4.5943, 3.5894, 3.0217];
msePFHC = [0.7014, 1.0186, 0.1293, 2.0492, 1.0858, 1.6362, 0.9964, 0.5928, 0.5690, 2.1116];
msePFSC = [0.4544, 2.2340, 0.4420, 2.4165, 1.3354, 0.5163, 0.9210, 0.9091, 0.7274, 1.6054];

mseFSNC = [7.2209, 10.2093, 14.8408, 9.8155, 6.9555, 16.9083, 10.7880, 13.6921, 7.0909, 7.4747];
mseFSHC = [0.7735, 0.0268, 1.0312, 0.1049, 2.1406, 0.6963, 0.7288, 1.7161, 0.9648, 0.0746];
mseFSSC = [0.4594, 0.67338, 0.4088, 0.6214, 1.1404, 2.3662, 0.0965, 1.0398, 1.4076, 0.5387];



% plot boxplots
figure(1); hold on;
boxplot([msePFNC', msePFHC', msePFSC', mseFSNC', mseFSHC', mseFSSC'], 'Labels', {'PF-NC', 'PF-HC', 'PF-SC', 'FS-NC', 'FS-HC', 'FS-SC'}, 'Whisker', 1, 'Colors','k')
title("RMSE for different experiments in 10 trials");
hold off;

% plot errors in PF
figure(2); hold on;
plot(errNC, 'b--x');
plot(errHC, 'r-o');
plot( errSC, 'g-s');
title("Error per Waypoint using Particle Filters");
xlabel("waypoint in path");
ylabel("error in map units");
legend("No Classification", "Hard Classification", "Soft Classification");
hold off;

% plot errors in FS
figure(3); hold on;
plot(errFSNC, 'b--x');
plot(errFSHC, 'r-o');
plot( errFSSC, 'g-s');
title("Error per Waypoint using Fast SLAM");
xlabel("waypoint in path");
ylabel("error in map units");
legend("No Classification", "Hard Classification", "Soft Classification");
hold off;

