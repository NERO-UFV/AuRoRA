%% RECEBE COMANDOS PELA REDE E ENVIA DADOS DOS SENSORES
% Este programa apenas recebe comandos enviados de outro pc e os repassa ao
% pioneer. Além disso, envia para rede dados dos sensores do robô (u e w)

clear
close all
clc
% Fecha todas possíveis conexões abertas
try
    fclose(instrfindall);
catch
end

%% Rotina para buscar pasta raiz
PastaAtual = pwd;
PastaRaiz = 'AuRoRA 2018';
cd(PastaAtual(1:(strfind(PastaAtual,PastaRaiz)+numel(PastaRaiz)-1)))
addpath(genpath(pwd))

%% Classes initialization
% Robot

serverID = 1; % Robô Servidor
slaveID = 2; % Robô Escravo

P = Pioneer3DX(slaveID);   % the ID must be diferent of the pioneer in the server machine to avoid conflict on NetDataShare
P.pPar.Ts = 0.030;

% Start ROS Network
r = ROSNetwork;
r.InitROS('/master')

pause
%% Connect with robot or simulator MobileSim
% P.rConnect;

%% Network communication check
% Only enter the loop after exchange data with server machine
r.InitROS('/P1','192.168.0.159');
r.SubscriberROS(r.node,'P1/vel');
[u , v] = r.ReceiveROS('robot1/vel');

%% Variables initialization
% Time variables
tsim = 120;   % maximum simulation time
t    = tic;   % simulation current time
tc   = tic;   % sample time to send data to robot
tr   = tic;   % timer to reset pioneer pose

% Simulation data
data = [];    % save simulation data

%% Simulation
try
while true % toc(t) < tsim
    
    if toc(tc) > P.pPar.Ts
        tc = tic;
        
        % Read sensors data
%         P.rGetSensorData;
        data = [ data; P.pSC.U(1:2)'];
               
        % Read Network
        P.pSC.Ud(1:2) = r.ReceiveROS('P1/vel');
        
        % Send commands to pioneer
%         P.rSendControlSignals;
    end
    
end
catch ME
    disp('Emergency')
    P.pSC.Ud = [0 ; 0];
%     P.rSendControlSignals;
end

%%  Stopping Pioneer
% Zera velocidades do robô
% P.pSC.Ud = [0 ; 0];
% P.rSendControlSignals;


