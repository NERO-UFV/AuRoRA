% Demo 1 - Conectar ao ArDrone

close all
clear
clc
try
    fclose(instrfindall);
end

% Rotina para buscar pasta raiz
PastaAtual = pwd;
PastaRaiz = 'AuRoRA 2018';
cd(PastaAtual(1:(strfind(PastaAtual,PastaRaiz)+numel(PastaRaiz)-1)))
addpath(genpath(pwd))

% figure(1)
% drawnow

A = ArDrone;
A.rConnect;
% A.rGetSensorCalibration;
A.rTakeOff;
pause(3)
% Conectar Joystick
J = JoyControl;

X = [];
pac_tot = 0;
pac_rec = 0;

tmax = 120; % Tempo Simulação em segundos
t = tic;
tc = tic;
tp = tic;

XX = [];

while toc(t) < tmax
    if toc(tc) > 1/30
        tc = tic;
        % Obter dados de voo
        % A.rGetStatusRawData
        A.rGetSensorData
        
        pac_tot = pac_tot + 1;
        
        if sum(A.pCom.cRawData) > 1
            pac_rec = pac_rec + 1;
            % disp('ok')
            Y = [A.pCom.cRawData' toc(t)];
            X = [X; Y];
        end
        
        % Controlar Drone
        % Controlador Joystick
        A = J.mControl(A);
        A.rSendControlSignals;
%         A.rCommand
        
        XX = [XX [A.pPos.Xd; A.pPos.X; A.pSC.Ud; toc(t)]];

        A.pSC.Ud = zeros(4,1);
        
        if toc(tp) > 0.1
            tp = tic;
            try
                delete(h1)
                delete(h2)
                delete(h3)
            end
            subplot(311),h1 = plot(XX(end,:),XX([19],:));
            if toc(t) < 30
                axis([0 30 -1 1])
            else
                axis([toc(t)-30 toc(t) -1 1])
            end
            
            subplot(312),h2 = plot(XX(end,:),XX([20],:));
            if toc(t) < 30
                axis([0 30 -1 1])
            else
                axis([toc(t)-30 toc(t) -1 1])
            end
            
             subplot(313),h3 = plot(XX(end,:),XX([21],:));
            if toc(t) < 30
                axis([0 30 -1 1])
            else
                axis([toc(t)-30 toc(t) -1 1])
            end
            
            drawnow
        end
        
        %         display(A.pSC.Ud')
        %         disp('-------------')
    end
end

% Aterrissar/Desconectar
if A.pCom.cStatus(32)==1   % Caso tenha estourado o tempo e esteja voando
    A.rLand
end

A.rDisconnect
display([pac_tot pac_rec])

