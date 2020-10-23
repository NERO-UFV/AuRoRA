% Guiar drone virtual usando joystick
% Testar modelo din�mico

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

Ar = ArDrone;   % Real

% Conectar Joystick
J = JoyControl;

% Arquivo Texto
% ArqTxt = fopen('DadosExp.txt');


% =========================================================================
f1 = figure('Name','Simula��o Posicionamento ArDrone','NumberTitle','off');
% f1.Position = [435 2 930 682];
f1.Position = [1367 11 831 634]; % Segunda tela em Sete Lagoas!
figure(f1);

title('Task: Position Control')
xlabel({'Eixo $$x$$', '[m]'},'FontSize',12,'FontWeight','bold','interpreter','latex');
ylabel({'Eixo $$y$$', '[m]'},'FontSize',12,'FontWeight','bold','interpreter','latex');
zlabel({'Eixo $$z$$', '[m]'},'FontSize',12,'FontWeight','bold','interpreter','latex');
axis equal
axis([-3 3 -3 3 0 3])
view(3)
view(50,10)
grid on
hold on
% grid minor

Ar.mCADplot
drawnow
pause(1)
disp('Start..........')

% =========================================================================
% Iniciar eta de controle
% Decolar
tmax = 40; % Tempo Simula��o em segundos
% X = zeros(1,19); % Dados correntes da simula��o

t = tic;
tc = tic;
tp = tic;


XX = [];

while toc(t) < tmax
    if toc(tc) > 1/30
        tc = tic;
%         if toc(t) > 3*tmax/4
%             A.pPos.Xd(1) = 0;
%             A.pPos.Xd(2) = 0;
%             A.pPos.Xd(3) = 1;
%             A.pPos.Xd(6) = 0;
%         elseif toc(t) > 2*tmax/4
%             A.pPos.Xd(1) = 0;
%             A.pPos.Xd(2) = 0;
%             A.pPos.Xd(3) = 1;
%             A.pPos.Xd(6) = 0;
%         elseif toc(t) > tmax/4
%             Ar.pPos.Xd(1) = 2;
%             Ar.pPos.Xd(2) = -1;
%             Ar.pPos.Xd(3) = 2;
%             Ar.pPos.Xd(6) = 0;
%         else
            Ar.pPos.Xd(1) = -2;
            Ar.pPos.Xd(2) = -1;
            Ar.pPos.Xd(3) = 2;
            Ar.pPos.Xd(6) = 0;
%         end
        
        % Controlador
        Ar.rGetSensorData
        Ar = cUnderActuatedController(Ar);
        % Joystick: Sobrep�e controlador
        Ar = J.mControl(Ar);
        Ar.rSendControlSignals;

        XX = [XX; [Ar.pPos.Xd' Ar.pPos.X' Ar.pSC.Ud' toc(t)]];        
    end
    if toc(tp) > 0.3
        tp = tic;
        Ar.mCADplot;
        drawnow
%         view(30,30)
    end
    
end


figure

subplot(311),plot(XX(:,end),XX(:,[1 13]))
legend({'$x_{d}$','$x_{Atu}$'},'FontSize',12,'interpreter','latex')
grid on
axis([0 40 -2.5 0.5])
title({'Drone'; 'Position'},'FontSize',14,'FontWeight','bold','Interpreter','latex')
xlabel('$time$ [sec]','FontSize',12,'FontWeight','bold','interpreter','Latex')
ylabel('$x$ [m]','FontSize',12,'FontWeight','bold','interpreter','Latex')

subplot(312),plot(XX(:,end),XX(:,[2 14]))
legend({'$y_{d}$','$y_{Atu}$'},'FontSize',12,'interpreter','latex')
grid on
axis([0 40 -1.5 0.5])
xlabel('$time$ [sec]','FontSize',12,'FontWeight','bold','interpreter','Latex')
ylabel('$y$ [m]','FontSize',12,'FontWeight','bold','interpreter','Latex')

subplot(313),plot(XX(:,end),XX(:,[3 15]))
legend({'$z_{d}$','$z_{Atu}$'},'FontSize',12,'interpreter','latex')
grid on
axis([0 40 -0.5 2.5])
xlabel('$time$ [sec]','FontSize',12,'FontWeight','bold','interpreter','Latex')
ylabel('$z$ [m]','FontSize',12,'FontWeight','bold','interpreter','Latex')



% subplot(211),plot(XX(end,:),XX([4 16],:)'*180/pi)
% legend('\phi_{Des}','\phi_{Atu}')
% grid
% subplot(212),plot(XX(end,:),XX([5 17],:)'*180/pi)
% legend('\theta_{Des}','\theta_{Atu}')
% grid
% 
% figure
% subplot(211),plot(XX(end,:),XX([3 15],:)')
% legend('z_{Des}','z_{Atu}')
% grid
% subplot(212),plot(XX(end,:),XX([6 18],:)'*180/pi)
% legend('\psi_{Des}','\psi_{Atu}')
% grid
% 
% figure
% subplot(211),plot(XX(end,:),XX(25,:))
% legend('\phi_{Des}')
% grid
% subplot(212),plot(XX(end,:),XX(26,:))
% legend('\theta_{Des}')
% grid
% 
% figure
% subplot(211),plot(XX(end,:),XX([1 13],:)')
% legend('x_{Des}','x_{Atu}')
% grid
% subplot(212),plot(XX(end,:),XX([2 14],:)')
% legend('y_{Des}','y_{Atu}')
% grid
% 
% figure
% subplot(311),plot(XX(end,:),XX([7 19],:)')
% legend('dx_{Des}','dx_{Atu}')
% grid
% subplot(312),plot(XX(end,:),XX([8 20],:)')
% legend('dy_{Des}','dy_{Atu}')
% grid
% subplot(313),plot(XX(end,:),XX([9 21],:)')
% axis([0 tmax -1 1])
% legend('dz_{Des}','dz_{Atu}')
% grid
% 

