%% SCRIPT_DisplayData
%   Read data from Optitrack and draw ardrone
%   ORIGINAL SCRIPT BY
%   M. Kutzer 10Mar2016, USNA

% Clear workspace, close all figures, clear command window
clear
close all
clc
% try
%     fclose(instrfindall);
% end
%% Rotina para buscar pasta raiz
PastaAtual = pwd;
PastaRaiz = 'AuRoRA 2018';
cd(PastaAtual(1:(strfind(PastaAtual,PastaRaiz)+numel(PastaRaiz)-1)))
addpath(genpath(pwd))

%% Create OptiTrack object and initialize
obj = OptiTrack;
obj.Initialize;

%% Create Robot
A = ArDrone;

%% Window configuration
fig = figure(1);
axis([-2 2 -2 2])
A.mCADplot
drawnow

% Time variable
tp = tic;

%% Display data
while true
    %     A.rGetSensorData; % gets robot sensor data (just in case)
    
    % Get current rigid body information
    rb = obj.RigidBody;
    A = getOptData(rb,A);    
  
    % Desenha os robôs
    if toc(tp) > 1/30
        tp = tic;
        A.mCADplot;
        
        axis([-3 3 -3 3 0 3])
        hold on
        grid on
        drawnow
    end
end