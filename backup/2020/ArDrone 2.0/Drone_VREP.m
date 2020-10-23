%% Codigo para a versão teste de comunicacao entre V-Rep e Matlab

clear 
close all
clc

% Rotina para buscar pasta raiz
PastaAtual = pwd;
PastaRaiz = 'AuRoRA 2018';
cd(PastaAtual(1:(strfind(PastaAtual,PastaRaiz)+numel(PastaRaiz)-1)))
addpath(genpath(pwd))



%%

A = ArDrone; % criando a classe ArDrone

%% Conectar a plataforma V-Rep
A.vConnect;

A.vHandle('Quadricopter');
% A.vSetPose([0 0 0 pi]);
%% Coletando informaçoes do Drone
t = tic;
tc = tic;
tmax = 40;

a= 2;
erro = A.vrep.simxSetIntegerSignal(A.clientID,'comunication', a, A.vrep.simx_opmode_oneshot);
erro = A.vrep.simxSetFloatSignal(A.clientID,'transfer1',0,A.vrep.simx_opmode_oneshot);
erro = A.vrep.simxSetFloatSignal(A.clientID,'transfer2',0,A.vrep.simx_opmode_oneshot);
erro = A.vrep.simxSetFloatSignal(A.clientID,'transfer3',0,A.vrep.simx_opmode_oneshot);
erro = A.vrep.simxSetFloatSignal(A.clientID,'transfer4',0,A.vrep.simx_opmode_oneshot);


dados = [];
while toc(t)<15
    
    if toc(tc) > 0.1
        tc = tic;
%        
        A.pPos.Xd(1) = -2.8;
        A.pPos.Xd(2) = 1.525;
        A.pPos.Xd(3) = 0.5;
        A.pPos.Xd(6) = 0;
%         
        % Controlador
        A.vGetSensorData
        A = cUnderActuatedController(A);
        % Joystick: Sobrepõe controlador
        %A = J.mControl(A);
        A.rSendControlSignals;
        
        %% gambiarra 
        for i=1:4
        A.pPar.W(i)= A.pPar.W(i)/32;
        end
% 
        a= 2;
        erro = A.vrep.simxSetIntegerSignal(A.clientID,'comunication', a, A.vrep.simx_opmode_oneshot);
        erro = A.vrep.simxSetFloatSignal(A.clientID,'transfer1',A.pPar.W(1),A.vrep.simx_opmode_oneshot);
        erro = A.vrep.simxSetFloatSignal(A.clientID,'transfer2',A.pPar.W(2),A.vrep.simx_opmode_oneshot);
        erro = A.vrep.simxSetFloatSignal(A.clientID,'transfer3',A.pPar.W(3),A.vrep.simx_opmode_oneshot);
        erro = A.vrep.simxSetFloatSignal(A.clientID,'transfer4',A.pPar.W(4),A.vrep.simx_opmode_oneshot);

    end
    %A.vGetSensorData;
    %A.pPos.X
%     [erro, signalValue(1)] = A.vrep.simxGetFloatSignal(A.clientID,'transfer1',A.vrep.simx_opmode_buffer);
%     [erro, signalValue(2)] = A.vrep.simxGetFloatSignal(A.clientID,'transfer2',A.vrep.simx_opmode_buffer);
%     [erro, signalValue(3)] = A.vrep.simxGetFloatSignal(A.clientID,'transfer3',A.vrep.simx_opmode_buffer);
%     [erro, signalValue(4)] = A.vrep.simxGetFloatSignal(A.clientID,'transfer4',A.vrep.simx_opmode_buffer);
%    
%     dados = [dados; signalValue(1) signalValue(2) signalValue(3) signalValue(4) toc(t)];
end
% [erro, signalValue(1)] = A.vrep.simxGetFloatSignal(A.clientID,'transfer1',A.vrep.simx_opmode_streaming);
% [erro, signalValue(2)] = A.vrep.simxGetFloatSignal(A.clientID,'transfer2',A.vrep.simx_opmode_streaming);
% [erro, signalValue(3)] = A.vrep.simxGetFloatSignal(A.clientID,'transfer3',A.vrep.simx_opmode_streaming);
% [erro, signalValue(4)] = A.vrep.simxGetFloatSignal(A.clientID,'transfer4',A.vrep.simx_opmode_streaming);
% 
% [erro, signalValue(1)] = A.vrep.simxGetFloatSignal(A.clientID,'transfer1',A.vrep.simx_opmode_buffer);
% [erro, signalValue(2)] = A.vrep.simxGetFloatSignal(A.clientID,'transfer2',A.vrep.simx_opmode_buffer);
% [erro, signalValue(3)] = A.vrep.simxGetFloatSignal(A.clientID,'transfer3',A.vrep.simx_opmode_buffer);
% [erro, signalValue(4)] = A.vrep.simxGetFloatSignal(A.clientID,'transfer4',A.vrep.simx_opmode_buffer);
% % 
% signal_matlab=3;
% erro = A.vrep.simxSetIntegerSignal(A.clientID,'tranfer', signal_matlab, A.vrep.simx_opmode_oneshot);


%% Desconectar com V-Rep
% A.vDisconnect

