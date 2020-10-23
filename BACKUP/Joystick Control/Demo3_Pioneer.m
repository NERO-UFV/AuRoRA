clear
close all
clc

try
    fclose(instrfindall)
end

%% Rotina para buscar pasta raiz
PastaAtual = pwd;
PastaRaiz = 'AuRoRA 2018';
cd(PastaAtual(1:(strfind(PastaAtual,PastaRaiz)+numel(PastaRaiz)-1)))
addpath(genpath(pwd))


P = Pioneer3DX;
P.rConnect;

J = JoyControl;


tmax = 120; % Tempo Simulação em segundos
t = tic;
tc = tic;
tp = tic;

while true %toc(t) < tmax
    if toc(tc) > 0.1
        tc = tic;
        % Obter
        P.rGetSensorData;
        
        
       
        % Controlador Joystick
        P = J.mControl(P);
        
        P.rSendControlSignals
        
    end
end

robot.pSC.Ud = [0; 0];
P.rSendControlSignals
        
