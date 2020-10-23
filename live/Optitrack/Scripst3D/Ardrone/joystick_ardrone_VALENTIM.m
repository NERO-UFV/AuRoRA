%% Testar modelo dinâmico do ArDrone em controle de posição
% Inicialização
close all; clear all; clc;

try
    fclose(instrfindall);
catch
end

% Rotina para buscar pasta raiz
PastaAtual = pwd;
PastaRaiz = 'AuRoRA 2018';
cd(PastaAtual(1:(strfind(PastaAtual,PastaRaiz)+numel(PastaRaiz)-1)))
addpath(genpath(pwd))


%% Initialize classes
% Create OptiTrack object and initialize

% Connect Joystick
J = JoyControl;

% Robot initialization
A = ArDrone(40);
A.rConnect;

disp('Fim da estabilização!');

%% Robot initial pose

tc = tic;
A.rTakeOff;
% A.rTakeOff;

while true
    
    if toc(tc) > 1/30
        tc = tic;
        
        A = J.mControl(A);           % joystick command (priority)
        A.rSendControlSignals;
    end
end


% Land drone
if A.pFlag.Connected == 1
    A.rLand;
    %     A.rDisconnect;
end

