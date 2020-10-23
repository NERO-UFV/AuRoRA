%% Analysing data from experiment using drone.

%% Cleaning Variables:
close all;
clear all;
clc;

%% Reading Data:
data = load('Harrison_quat_optitrack.mat');
dlmwrite('quaternions_optitrack.txt',data.quat_hist);
dlmwrite('time_optitrack.txt',data.t_hist);
dlmwrite('coordenates_optitrack.txt',data.x_hist);
% dlmwrite('desired_optitrack.txt',data.xd_hist);