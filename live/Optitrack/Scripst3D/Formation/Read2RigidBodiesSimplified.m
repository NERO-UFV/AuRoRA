%% GET DATA FROM TWO RIGID BODIES (FORMATION DRONE-PIONEER)
% Teste de leitura de dados de 2 corpos rígidos pelo optitrack
% O Pioneer deve ser configurado no optitrack como ID1 e o Ardrone como ID2
% Além disso, o nome do corpo rígido deve começar com a letra da respectiva
% classe: 
%    Classe         Nomes sugeridos
%    Pioneer3DX      P1 , Pioneer1       
%    Ardrone         A1 , Ardrone1 
%                       * O número servirá para identificar posteriormente
%                       no caso de mais de um robô do mesmo tipo (a ser feito por algum calouro)

close all
clear
clc

try
    fclose(instrfindall);
catch
end

% Search root directory
PastaAtual = pwd;
PastaRaiz = 'AuRoRA 2018';
cd(PastaAtual(1:(strfind(PastaAtual,PastaRaiz)+numel(PastaRaiz)-1)))
addpath(genpath(pwd))

%% Open file to save data
NomeArq = datestr(now,30);
cd('DataFiles')
cd('Log_Optitrack')
Arq = fopen(['teste_drone_' NomeArq '.txt'],'w');
cd(PastaAtual)

%% Initialize classes
% Create OptiTrack object and initialize
OPT = OptiTrack;
OPT.Initialize;

%OPT.Initialize('192.168.0.1');
% Robot initialization
P = Pioneer3DX;
A = ArDrone(2);

%% Window definition
figure(1)
axis([-3 3 -3 3 0 3])
grid on
% A.mCADplot;
% P.mCADplot(1,'k');
drawnow

%% Variables Initialization

tp = tic;
%% Simulation loop
while true
    
    %  Get current rigid body information from optitrack
    rb = OPT.RigidBody;
    if rb(1).isTracked
        P = getOptData(rb(1),P);
    end
    if rb(2).isTracked
        A = getOptData(rb(2),A);
    end
       
    % Draw robot
    if toc(tp) > 0.3
        try
            P.mCADdel
            delete(fig);
        catch
        end
        tp = tic;
        A.mCADplot;
        P.mCADplot(1,'k');
        drawnow
    end
    
end

