% Demo 1 - Conectar ao ArDrone

close all
clear
clc
try
    fclose(instrfindall);
end

% % Rotina para buscar pasta raiz
% PastaAtual = pwd;
% PastaRaiz = 'AuRoRA 2018';
% cd(PastaAtual(1:(strfind(PastaAtual,PastaRaiz)+numel(PastaRaiz)-1)))
% addpath(genpath(pwd))

% figure(1)
% drawnow

A = ArDrone;  % ID referente ao ip do drone
A.pPar.ip = '192.168.1.61';
A.rConnect;
% A.rGetSensorCalibration;

% Conectar Joystick
J = JoyControl;

X = [];
pac_tot = 0;
pac_rec = 0;

tmax = 180; % Tempo Simulação em segundos
t = tic;
tc = tic;
tp = tic;

A.rGetSensorData


while toc(t) < tmax
    if toc(tc) > 1/30
        tc = tic;

        A = J.mControl(A);
        A.rSendControlSignals;


        A.pSC.Ud = zeros(4,1);

    end
end



A.rDisconnect

