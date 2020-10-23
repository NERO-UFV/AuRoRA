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

A = ArDrone; % Atrasado
% Ar = ArDrone; % Real

gains = [0.5 3 0.6 3 2 15;
         10 3 8 3 1 5];

% Conectar Joystick
J = JoyControl;

% =========================================================================
figure(1)
axis([-3 3 -3 3 0 3])
grid on
A.mCADplot
drawnow
pause(1)
disp('Start..........')

% =========================================================================
% Iniciar eta de controle
% Decolar
tmax = 40; % Tempo Simulação em segundos
X = zeros(1,19); % Dados correntes da simulação

t = tic;
tc = tic;
tp = tic;


XX = [A.pPos.Xd' A.pPos.X' A.pSC.Ud' toc(t)]; % Teste drone atrasado
kk = 1;

A.pPos.Xd(1) = 2;
A.pPos.Xd(2) = 2;
A.pPos.Xd(3) = 2.5;
A.pPos.Xd(6) = 0;

while toc(t) < tmax
    if toc(tc) > 1/30
        tc = tic;
        
%         if toc(t) > 3*tmax/4
%             A.pPos.Xd(1) = 0;
%             A.pPos.Xd(2) = 0;
%             A.pPos.Xd(3) = 1;
%             A.pPos.Xd(6) = 0;
%         elseif toc(t) > 2*tmax/4
%             A.pPos.Xd(1) = 1;
%             A.pPos.Xd(2) = 0;
%             A.pPos.Xd(3) = 1;
%             A.pPos.Xd(6) = 0;
%         elseif toc(t) > tmax/4
%             A.pPos.Xd(1) = 0;
%             A.pPos.Xd(2) = 0;
%             A.pPos.Xd(3) = 1;
%             A.pPos.Xd(6) = 0;
%         else

%         end
        
%         Ar.pPos.Xd = A.pPos.Xd;
        
        % Controlador
        A.rGetSensorData
%         Ar.rGetSensorData
        
        % Obter informação com atraso
        % A  informação da posição do robô está com atraso
        % Informação a cada 30ms
        % Atraso máximo de 1s
        idAtraso  = 20; %randi(30);
        
        if kk > idAtraso
            posAtraso = XX(kk-idAtraso,13:24)';
            A = cUnderActuatedControllerAtraso(A,posAtraso);        
        end
        
%         Ar = cUnderActuatedControllerMexido(Ar,gains);
%         A = cUnderActuatedController(A);
        
        % Joystick: Sobrepõe controlador
        A = J.mControl(A);
%         Ar = J.mControl(Ar);
        
        A.rSendControlSignals;
%         Ar.rSendControlSignals;

        

        XX = [XX; [A.pPos.Xd' A.pPos.X' A.pSC.Ud' toc(t)]];      
        kk = kk + 1;
        
    end
%     if toc(tp) > inf
    if toc(tp) > 0.30
        tp = tic;
        tic
        A.mCADplot;
        hold on
        plot3(XX(:,1),XX(:,2),XX(:,3),'r.')
        plot3(XX(:,13),XX(:,14),XX(:,15),'b-')
        drawnow
        toc
        view(30,30)
    end
    
end
%%
figure

subplot(311),plot(XX(:,end),XX(:,[1 13]))
legend({'$x_{d}$','$x_{delayed}$'},'FontSize',12,'interpreter','latex')
grid on
axis([0 40 -2.5 0.5])
title({'\textbf{Drone}'; 'Position'},'FontSize',14,'FontWeight','bold','Interpreter','latex')
xlabel('$time$ [sec]','FontSize',12,'FontWeight','bold','interpreter','Latex')
ylabel('$x$ [m]','FontSize',12,'FontWeight','bold','interpreter','Latex')

subplot(312),plot(XX(:,end),XX(:,[2 14]))
legend({'$y_{d}$','$y_{delayed}$'},'FontSize',12,'interpreter','latex')
grid on
axis([0 40 -1.5 0.5])
xlabel('$time$ [sec]','FontSize',12,'FontWeight','bold','interpreter','Latex')
ylabel('$y$ [m]','FontSize',12,'FontWeight','bold','interpreter','Latex')

subplot(313),plot(XX(:,end),XX(:,[3 15]))
legend({'$z_{d}$','$z_{delayed}$'},'FontSize',12,'interpreter','latex')
grid on
axis([0 40 -0.5 2.5])
xlabel('$time$ [sec]','FontSize',12,'FontWeight','bold','interpreter','Latex')
ylabel('$z$ [m]','FontSize',12,'FontWeight','bold','interpreter','Latex')


% figure
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


