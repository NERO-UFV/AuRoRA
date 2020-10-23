% Testar modelo dinâmico em uma trajetória

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

% Conectar Joystick
J = JoyControl;

A = ArDrone;
% A.rConnect;
% A.rTakeOff;
% A.rGetSensorCalibration;

Path = cPathFollowingReference;

tmax = 60; % Tempo Simulação em segundos
X = zeros(1,19); % Dados correntes da simulação

figure(1)
plot3(Path(1,:),Path(2,:),Path(3,:),'--r')
hold
axis([-3 3 -3 3 0 3])
grid on
A.mCADplot
drawnow
pause(2)
disp('Start............')

% =========================================================================
t = tic;
tc = tic;
tp = tic;

XX = [];
TT = [];
while toc(t) < tmax
    if toc(tc) > 1/30
        TT = [TT toc(tc)];
        tt = toc(t);
        
        A = cPathFollowing(A,Path);
        display(A.pPos.Xd')
                        
        % Controlador
        A.rGetSensorData
        A = cUnderActuatedController(A);        
        % A = J.mControl(A);
        A.rSendControlSignals;
        
        XX = [XX [A.pPos.Xd; A.pPos.X; tt]];
        
    end
    if toc(tp) > 0.3
        tp = tic;
        A.mCADplot;
        try 
            delete(h)
        end
        h = plot3(A.pPos.Xd(1),A.pPos.Xd(2),A.pPos.Xd(3),'or','MarkerSize',15);
        drawnow
    end
    
end
 
% A.rLand;
% A.rDisconnect;


figure
subplot(211),plot(XX(end,:),XX([4 16],:)'*180/pi)
legend('\phi_{Des}','\phi_{Atu}')
grid
subplot(212),plot(XX(end,:),XX([5 17],:)'*180/pi)
legend('\theta_{Des}','\theta_{Atu}')
grid

figure
plot(XX([1,13],:)',XX([2,14],:)')
% axis([-1.5 1.5 -1.5 1.5])
axis equal

figure
subplot(211),plot(XX(end,:),XX([1 13],:)')
legend('x_{Des}','x_{Atu}')
grid
subplot(212),plot(XX(end,:),XX([2 14],:)')
legend('y_{Des}','y_{Atu}')
grid

% figure
% subplot(211),plot(XX(end,:),XX(19,:)')
% legend('x_{Des}','x_{Atu}')
% grid
% subplot(212),plot(XX(end,:),[XX(13,:); [0 diff(XX(13,:))]*30])
% legend('y_{Des}','y_{Atu}')
% grid

