% Guiar drone virtual usando joystick
% Testar modelo dinâmico

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

A = ArDrone;
A.rConnect
 A.rGetSensorData
% Conectar Joystick
J = JoyControl;

tmax = 30; % Tempo Simulação em segundos
X = zeros(1,19); % Dados correntes da simulação

figure(1)
axis([-1 1 -1 1 -1 1])
grid on
A.mCADplot
drawnow
% pause

% =========================================================================
t = tic;
tc = tic;
tp = tic;


XX = [];
raw = [];
while toc(t) < tmax
    if toc(tc) > 1/30
        tc = tic;
        
         % Controlador Joystick
        A = J.mControl(A);
                        
        % Controlador
        A.rGetSensorData
        A.rSendControlSignals;
        
        A.pPos.X([4 5 6])
        
        XX = [XX [A.pPos.Xd; A.pPos.X; toc(t)]];
        raw = [raw A.pCom.cRawData'];
    end
    
    if toc(tp) > 0.05
        tp = tic;
        A.mCADplot;
        drawnow
    end
    
end

A.rLand;
A.rDisconnect

figure
subplot(311),plot(XX(end,:),XX([4 16],:)'*180/pi)
legend('\phi_{Des}','\phi_{Atu}')
grid
subplot(312),plot(XX(end,:),XX([5 17],:)'*180/pi)
legend('\theta_{Des}','\theta_{Atu}')
grid
subplot(313),plot(XX(end,:),XX([6 18],:)'*180/pi)
legend('\psi_{Des}','\psi_{Atu}')
grid

% figure;
% plot(raw(6,:))

% figure
% plot(XX([1,13],:)',XX([2,14],:)')
% axis([-1.5 1.5 -1.5 1.5])
% axis equal


