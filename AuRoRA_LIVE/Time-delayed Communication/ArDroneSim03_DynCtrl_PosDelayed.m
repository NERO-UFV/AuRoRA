%%%% Time-deleyad Multi-robot formation Control %%%%


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

Ar = ArDrone;   % Real
A = ArDrone;    % Atrasado

% Conectar Joystick
J = JoyControl;

% Arquivo Texto
% ArqTxt = fopen('DadosExp.txt');


% =========================================================================
f1 = figure('Name','Simulação Posicionamento ArDrone','NumberTitle','off');
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

% Ar = vSetPose(Ar,[0; 0; 0.1586; 0]); % Erro

Ar.mCADplot
A.mCADplot;

% Time-delayed Drone Appearance
A.mCADcolor([0;.4470; 0.5410]);

% A.pCAD.i3D.FaceColor = 'none'; 
% A.pCAD.i3D.EdgeColor = [0 0.4470 0.7410]; % 'interp' ou 'flat' % [0 0.4470 0.7410]; [0.4660 0.6740 0.1880]
A.pCAD.i3D.FaceAlpha = 1;
% A.pCAD.i3D.EdgeAlpha = 0.025;
% A.pCAD.i3D.LineWidth = 1;
%

drawnow
pause(1)
disp('Start..........')

% =========================================================================
% Iniciar eta de controle
% Decolar
tmax = 40; % Tempo Simulação em segundos
% X = zeros(1,19); % Dados correntes da simulação

t = tic;
tc = tic;
tp = tic;

XX = [A.pPos.Xd' A.pPos.X' A.pSC.Ud' Ar.pPos.Xd' Ar.pPos.X' Ar.pSC.Ud' toc(t)];
% XX = [A.pPos.Xd' A.pPos.X' A.pSC.Ud' toc(t)]; % Teste drone atrasado
% XX = [];
kk = 1;

xx = [];
yy = [];
zz = [];

xx1 = [];
yy1 = [];
zz1 = [];

xx = [xx Ar.pPos.X(1)];
yy = [yy Ar.pPos.X(2)];
zz = [zz Ar.pPos.X(3)];

% Teste drone atrasado: {
xx1 = [xx1 A.pPos.X(1)];
yy1 = [yy1 A.pPos.X(2)];
zz1 = [zz1 A.pPos.X(3)];
%                       }

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
            A.pPos.Xd(1) = -2;
            A.pPos.Xd(2) = -1;
            A.pPos.Xd(3) = 2;
            A.pPos.Xd(6) = 0;
%         end
        
%         Ar.pPos.Xd = A.pPos.Xd;
        Ar.pPos.Xd(1:3,1) = A.pPos.Xd(1:3,1) - 0.5*ones(3,1);
        
        % Controlador
        A.rGetSensorData
        Ar.rGetSensorData
        
        %- Obter informação com atraso
        % A  informação da posição do robô está com atraso
        % Informação a cada 30ms
        % Atraso máximo de 1s
        idAtraso  = 10; %randi(30);
%         idAtraso  = randi(30);

%         tic
        if kk > idAtraso
            posAtraso = XX(kk-idAtraso,13:24)';
            A = cUnderActuatedControllerDelayed(A,posAtraso);        
        end
%         toc
        
%         A = cUnderActuatedController(A);
        Ar = cUnderActuatedController(Ar);
        
        % Joystick: Sobrepõe controlador
        A = J.mControl(A);
        Ar = J.mControl(Ar);

        A.rSendControlSignals;
        Ar.rSendControlSignals;

%         XX = [XX; [Ar.pPos.Xd' Ar.pPos.X' Ar.pSC.Ud' toc(t)]]; 
        
        % Teste drone atrasado: {
%         XX = [XX; [A.pPos.Xd' A.pPos.X' A.pSC.Ud' toc(t)]]; 
        %                       }

        XX = [XX; [A.pPos.Xd' A.pPos.X' A.pSC.Ud' Ar.pPos.Xd' Ar.pPos.X' Ar.pSC.Ud' toc(t)]];      
        kk = kk + 1;

        xx = [xx Ar.pPos.X(1)];
        yy = [yy Ar.pPos.X(2)];
        zz = [zz Ar.pPos.X(3)];
        
        % Teste drone atrasado: {
        xx1 = [xx1 A.pPos.X(1)];
        yy1 = [yy1 A.pPos.X(2)];
        zz1 = [zz1 A.pPos.X(3)];
        %                       }

    end
   
%     if toc(tp) > inf
    if toc(tp) > 0.3
        tp = tic;
%         tic
        A.mCADplot;
        Ar.mCADplot;
        plot3(xx,yy,zz,'-r')
        plot3(xx1,yy1,zz1,'-b')
        hold on            
        drawnow
%         toc
%         view(30,30)
    end
    
end

%----- Teste Drone Atrasado:
% figure
% 
% subplot(311),plot(XX(:,end),XX(:,[1 13]))
% legend({'$x_{d}$','$x_{delayed}$'},'FontSize',12,'interpreter','latex')
% grid on
% axis([0 40 -2.5 0.5])
% title({'\textbf{Drone}'; 'Position'},'FontSize',14,'FontWeight','bold','Interpreter','latex')
% xlabel('$time$ [sec]','FontSize',12,'FontWeight','bold','interpreter','Latex')
% ylabel('$x$ [m]','FontSize',12,'FontWeight','bold','interpreter','Latex')
% 
% subplot(312),plot(XX(:,end),XX(:,[2 14]))
% legend({'$y_{d}$','$y_{delayed}$'},'FontSize',12,'interpreter','latex')
% grid on
% axis([0 40 -1.5 0.5])
% xlabel('$time$ [sec]','FontSize',12,'FontWeight','bold','interpreter','Latex')
% ylabel('$y$ [m]','FontSize',12,'FontWeight','bold','interpreter','Latex')
% 
% subplot(313),plot(XX(:,end),XX(:,[3 15]))
% legend({'$z_{d}$','$z_{delayed}$'},'FontSize',12,'interpreter','latex')
% grid on
% axis([0 40 -0.5 2.5])
% xlabel('$time$ [sec]','FontSize',12,'FontWeight','bold','interpreter','Latex')
% ylabel('$z$ [m]','FontSize',12,'FontWeight','bold','interpreter','Latex')
%-----

figure

subplot(311),plot(XX(:,end),XX(:,[1 13 41]))
legend({'$x_{d}$','$x_{delayed}$','$x_{Actu}$'},'FontSize',12,'interpreter','latex')
grid on
axis([0 40 -2.5 0.5])
title({'\textbf{Drone}'; 'Position'},'FontSize',14,'FontWeight','bold','Interpreter','latex')
xlabel('$time$ [sec]','FontSize',12,'FontWeight','bold','interpreter','Latex')
ylabel('$x$ [m]','FontSize',12,'FontWeight','bold','interpreter','Latex')

subplot(312),plot(XX(:,end),XX(:,[2 14 42]))
legend({'$y_{d}$','$y_{delayed}$','$y_{Actu}$'},'FontSize',12,'interpreter','latex')
grid on
axis([0 40 -1.5 0.5])
xlabel('$time$ [sec]','FontSize',12,'FontWeight','bold','interpreter','Latex')
ylabel('$y$ [m]','FontSize',12,'FontWeight','bold','interpreter','Latex')

subplot(313),plot(XX(:,end),XX(:,[3 15 43]))
legend({'$z_{d}$','$z_{delayed}$','$z_{Actu}$'},'FontSize',12,'interpreter','latex')
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

