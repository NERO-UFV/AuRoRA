% Positioning task

clear
close all
clc

try
    fclose(instrfindall);
catch
end

%% Rotina para buscar pasta raiz
PastaAtual = pwd;
PastaRaiz = 'AuRoRA';
cd(PastaAtual(1:(strfind(PastaAtual,PastaRaiz)+numel(PastaRaiz)-1)))
addpath(genpath(pwd))

%% Definindo o Robô
P = Pioneer3DX;
P.pPar.a = 0;
P.pPar.alpha = 0*pi/3;
% P.rConnect; % Descomentar para realiazação de experimento
P.rSetPose
% P.rSetPose([0 0 0 0]);


%% Definindo a Figura que irá rodar a simulação
figure(1)
hold on
grid on
title('Simulação');
axis([-1 3 -1 3])
P.mCADplot2D('r')   % Visualização 2D
drawnow

% Tempo de esperar para início do experimento/simulação
dados = [];
pause(1)

%% Tempo Real
tmax = 60;
tc = tic;
tp = tic;
t = tic;

%% Inicio da simulação
while (toc(t) <= tmax)
    % Sub-rotina de controle
    if toc(tc) > 0.1
        % Inicio da realimentação
        tc = tic;
        
        P.pPos.Xda = P.pPos.Xd;
        if toc(t) > 40
            P.pPos.Xd(1:2) = [0; 2];
            P.pPar.a = 1;
            P.rSetPose
        elseif toc(t) > 20
            P.pPos.Xd(1:2) = [2; 1];
            P.pPar.a = 0;
            P.rSetPose
        else
            P.pPos.Xd(1:2) = [1; 2];
        end
        % Pegando os dados do robo
        P.rGetSensorData;
        P = fKinematicControllerExtended(P);
        
        % Armazenar dados da simulação
        dados = [dados; [P.pPos.Xd' P.pPos.X' P.pSC.Ud' P.pSC.U' toc(t)]];

        % Enviar sinais de controle para o robô
        P.rSendControlSignals;        
        
        % Sub-rotina para plotar
        if toc(tp) > 0.1
            tp = tic;
            % Plot da simulação
            P.mCADdel
            % P.mCADplot(1,'r') % Visualização 3D
            P.mCADplot2D('r')   % Visualização 2D
            drawnow
        end
        
    end
end

P.pSC.Ud([1 2]) = 0;
P.rSendControlSignals;

