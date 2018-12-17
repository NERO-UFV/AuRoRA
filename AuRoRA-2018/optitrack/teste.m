%% SCRIPT_DisplayData
%   Read data from Optitrack and draw correspondent robot
%   ORIGINAL SCRIPT BY
%   M. Kutzer 10Mar2016, USNA

% Clear workspace, close all figures, clear command window
clear all
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
OPT = OptiTrack;
OPT.Initialize;   % motive streaming in loopback
% obj.Initialize('192.168.0.1');


%% Create Pioneer
P = Pioneer3DX;

%% Window configuration
fig = figure(1);
axis([-2 2 -2 2])
% pause(1)

% Time variable
tp = tic;

%% Display data
while true
    rb = OPT.RigidBody;
    
    % Assign optitrack data to robot
    P = getOptData(rb,P);
    
    
    % Desenha os robôs
    if toc(tp) > .1
        
        tp = tic;
        
        P = getOptData(OPT,P);
        try
            P.mCADdel
            delete(fig);
            
        catch
        end
        P.mCADplot2D('r')
        %                             P.mCADplot(1,'b')
        %             plot(P.pPos.Xc(1),P.pPos.Xc(2),'x','LineWidth',2)
        axis([-2 2 -2 2])
        hold on
        grid on
        drawnow
    end
    
end


