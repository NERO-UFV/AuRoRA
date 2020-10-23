%% SCRIPT_DisplayData
%   Read data from Optitrack, draw correspondent robot and send it
%   to network
%

% Clear workspace, close all figures, clear command window
clear all
close all
clc
% close all opened connections
try
    fclose(instrfindall);
catch
    disp('All connections already closed.');
end

%% Rotina para buscar pasta raiz
PastaAtual = pwd;
PastaRaiz = 'AuRoRA 2018';
cd(PastaAtual(1:(strfind(PastaAtual,PastaRaiz)+numel(PastaRaiz)-1)))
addpath(genpath(pwd))

%% Classes initialization
% OptiTrack 
OPT = OptiTrack;
OPT.Initialize;
% obj.Initialize('192.168.0.120','Unicast');

% Robots
P = Pioneer3DX;
A = ArDrone(2);

% Data share network
Rede = NetDataShare;

%% Get optitrack ID
% detect rigid body ID from optitrack
rb = OPT.RigidBody;         % read optitrack data
idP = getID(OPT,P);         % pioneer ID on optitrack
idA = getID(OPT,A);         % drone ID on optitrack

A = getOptData(rb(idA),A);  % get ardrone data
P = getOptData(rb(idP),P);  % get pioneer data


%% Window configuration
fig = figure(1);
axis([-2 2 -2 2])
pause(2)



%% Variables
data = [];
tp = tic;
tsimu = tic;
%% Loop read/display data
while true
      % Send data to network
%         Rede.mSendMsg(P);
        
    if toc(tsimu)>1/30
        tsimu = tic;
        
        % Get current rigid body information
        rb = OPT.RigidBody;
        A = getOptData(rb(idA),A);  % get ardrone data
        P = getOptData(rb(idP),P);  % get pioneer data


        % Send data to network
        Rede.mSendMsg(P);

      P.pPos.X'
      data = [data; P.pPos.X'];
      
    end
    
    
    % Desenha o robô
    if toc(tp) > .1
        tp = tic;

            P.mCADdel
            %             delete(fig);

        P.mCADplot2D('b')
%         A.mCADplot;
        %         P.mCADplot(1,'b')
        %         plot(P.pPos.Xc(1),P.pPos.Xc(2),'x','LineWidth',2)
        axis([-3 3 -3 3])
        hold on
        grid on
        drawnow
    end
    
end

figure;
plot(data(:,1),data(:,2))
grid on