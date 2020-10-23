%%%% Time-deleyad Multi-robot formation Control %%%%

% Testar erros involvidos na simulação do modelo

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

% Time-delayed Drone Appearance
A.mCADcolor([0;.4470; 0.5410]);
A.pCAD.i3D.FaceAlpha = 1;
A.mCADplot;

% Drone não atrasado:
Ar.mCADplot;

drawnow
pause(1)
disp('Start..........')

% =========================================================================
% Iniciar eta de controle - Decolar:
tmax = 40; % Tempo Simulação em segundos
t = tic;
tc = tic;
tp = tic;

% =========================================================================
% Inicialização dos Índices de Desempenho (drone atrasado):
IAE = 0;
ITAE = 0;
IASC = 0;

IAE_Ar = 0;
ITAE_Ar = 0;
IASC_Ar = 0;

% =========================================================================
% Dados robôs:
XX = [Ar.pPos.Xd' Ar.pPos.X' Ar.pSC.Ud' A.pPos.Xd' A.pPos.X' A.pSC.Ud' IAE_Ar ITAE_Ar IASC_Ar IAE ITAE IASC toc(t)];
kk = 1;

xx = [A.pPos.X(1) Ar.pPos.X(1)];
yy = [A.pPos.X(2) Ar.pPos.X(2)];
zz = [A.pPos.X(3) Ar.pPos.X(3)]; 

% Parâmetros para integração numérica
dt = 30e-3;

ddX(1:3,1) = 0;
 dX(1:3,1) = 0;
  X(1:3,1) = A.pPos.X(1:3,1);

A.pPos.Xd(1) = -2;
A.pPos.Xd(2) = -1;
A.pPos.Xd(3) = 2;
A.pPos.Xd(7:9,1) = zeros(3,1);

ddXAr(1:3,1) = 0;
 dXAr(1:3,1) = 0;
  XAr(1:3,1) = A.pPos.X(1:3,1);

Ar.pPos.Xd(1:3,1) = A.pPos.Xd(1:3,1);
Ar.pPos.Xd(7:9,1) = zeros(3,1);

% =========================================================================
%- Obter informação com atraso
% A  informação da posição do robô está com atraso
% Informação a cada 30ms
% Atraso máximo de 1s
idAtraso  = 10; %randi(30);

% acc_max = 0.85; % m/s^2 (Tarefa de segmento de trajetória)
acc_max = 0; % Tarefa de posicionamento
       

while toc(t) < tmax
    if toc(tc) > 1/30
        % Inicio da realimentação:
        tc = tic;   % timer para controle

        % Controlador:
        acc_des = acc_max;
        K1 = diag([0.5 0.6 2]);
        K2 = diag([3 3 3]);
        K3 = sqrt(4*K1);
        K4 = sqrt(4*K1*K2)/K3;
        
        % -------------------------- Drone não Atrasado
        Ar.pPos.Xtil = Ar.pPos.Xd - Ar.pPos.X;
        pos_tilAr = Ar.pPos.Xtil(1:3,1);                
        vel_tilAr = Ar.pPos.Xtil(7:9,1);

        ddXAr = acc_des + K1*tanh(K2*vel_tilAr) + K3*tanh(K4*pos_tilAr);                                                     

        % Pegando os dados do robo:
        dXAr = dXAr + ddXAr*dt; % Velocidade atual
        XAr = XAr + dXAr*dt;    % Posição atual

        Ar.pPos.X(1:3,1) = XAr;
        Ar.pPos.X(7:9,1) = dXAr;
        
        % -------------------------- Drone Atrasado
        if kk > idAtraso
            A.pPos.X = XX(kk-idAtraso,41:52)';
        end

        A.pPos.Xtil = A.pPos.Xd - A.pPos.X;
        pos_til = A.pPos.Xtil(1:3,1);                
        vel_til = A.pPos.Xtil(7:9,1);

        ddX = acc_des + K1*tanh(K2*vel_til) + K3*tanh(K4*pos_til);                                                     

        % Pegando os dados do robo:
        dX = dX + ddX*dt; % Velocidade atual
        X = X + dX*dt;    % Posição atual

        A.pPos.X(1:3,1) = X;
        A.pPos.X(7:9,1) = dX;
        % -------------------------- 
        % Índices de desempenho (Atrasado & Não-atrasado)
        IAE = IAE + (abs(A.pPos.Xtil(1)) + abs(A.pPos.Xtil(2)) + abs(A.pPos.Xtil(3)))*dt;
        ITAE = ITAE + (abs(A.pPos.Xtil(1)) + abs(A.pPos.Xtil(2)) + abs(A.pPos.Xtil(3)))*toc(t)*dt;
        IASC = IASC + abs(abs(ddX(1)) + abs(ddX(2)) + abs(ddX(3)))*dt;
        
        IAE_Ar = IAE_Ar + (abs(Ar.pPos.Xtil(1)) + abs(Ar.pPos.Xtil(2)) + abs(Ar.pPos.Xtil(3)))*dt;
        ITAE_Ar = ITAE_Ar + (abs(Ar.pPos.Xtil(1)) + abs(Ar.pPos.Xtil(2)) + abs(Ar.pPos.Xtil(3)))*toc(t)*dt;
        IASC_Ar = IASC_Ar + (abs(ddXAr(1)) + abs(ddXAr(2)) + abs(ddXAr(3)))*dt;
        
        
        % Histórico de dados:
        XX = [XX; [Ar.pPos.Xd' Ar.pPos.X' Ar.pSC.Ud' A.pPos.Xd' A.pPos.X' A.pSC.Ud' IAE_Ar ITAE_Ar IASC_Ar IAE ITAE IASC toc(t)]];      
        kk = kk + 1;

        % Rota feita pelos drones {
        xx = [xx; A.pPos.X(1) Ar.pPos.X(1)];
        yy = [yy; A.pPos.X(2) Ar.pPos.X(2)];
        zz = [zz; A.pPos.X(3) Ar.pPos.X(3)];
        %                       }

    end
   
    if toc(tp) > 0.3
        tp = tic;
        A.mCADplot;
        Ar.mCADplot;
        plot3(xx(:,1),yy(:,1),zz(:,1),'-b',xx(:,2),yy(:,2),zz(:,2),'--r')
        drawnow
        hold on            
    end
    
end

% A.pSC.Ud([1 2 3 4]) = 0;
% A.rSendControlSignals;

%%----- Posição:
figure()

subplot(311),plot(XX(:,end),XX(:,[1 13 41]), 'LineWidth', 1)
legend({'$x_{d}$','$x_{Actu}$','$x_{delayed}$'},'FontSize',13,'interpreter','latex','Position',[0.83 0.87 0.091 0.092])
grid on
axis([0 tmax round(min(XX(:,41))-1) round(max(XX(:,41))+1)])
title({'\textbf{Drone}'; 'Position'},'FontSize',14,'FontWeight','bold','Interpreter','latex')
% xlabel('$t$ [$s$]','FontSize',14,'FontWeight','bold','interpreter','Latex')
ylabel('$x$ [$m$]','FontSize',13,'FontWeight','bold','interpreter','Latex')

subplot(312),plot(XX(:,end),XX(:,[2 14 42]), 'LineWidth', 1)
legend({'$y_{d}$','$y_{Actu}$','$y_{delayed}$'},'FontSize',13,'interpreter','latex','Position', [0.84 0.569 0.071 0.0905])
grid on
axis([0 tmax round(min(XX(:,42))-1) round(max(XX(:,42))+1)])
% xlabel('$t$ [$s$]','FontSize',12,'FontWeight','bold','interpreter','Latex')
ylabel('$y$ [$m$]','FontSize',13,'FontWeight','bold','interpreter','Latex')

subplot(313),plot(XX(:,end),XX(:,[3 15 43]), 'LineWidth', 1)
legend({'$z_{d}$','$z_{Actu}$','$z_{delayed}$'},'FontSize',13,'interpreter','latex','Position',[0.840 0.278 0.070 0.090])
grid on
axis([0 tmax round(min(XX(:,43))-1) round(max(XX(:,43))+0.5)])
xlabel('$t$ [$s$]','FontSize',13,'FontWeight','bold','interpreter','Latex')
ylabel('$z$ [$m$]','FontSize',13,'FontWeight','bold','interpreter','Latex')
%-----

%%----- Índices de desempenho:
figure()

subplot(311),plot(XX(:,end),XX(:,[end-6 end-3]), 'LineWidth', 1)
lgd = legend({'$Drone_{Actu}$','$Drone_{delayed}$'},'FontSize',12,'interpreter','latex','Location','northwest');
title(lgd,'IAE','FontWeight','bold');
grid on
axis([0 tmax round(min(XX(:,end-3))-5) round(max(XX(:,end-3))+5)])
title({'\textbf{Indices}'; 'de Desempenho'},'FontSize',14,'FontWeight','bold','Interpreter','latex')
% xlabel('$t$ [s]','FontSize',12,'FontWeight','bold','interpreter','Latex')
ylabel('IAE [$m$]','FontSize',13,'FontWeight','bold','interpreter','Latex')

subplot(312),plot(XX(:,end),XX(:,[end-5 end-2]), 'LineWidth', 1)
lgd1 = legend({'$Drone_{Actu}$','$Drone_{delayed}$'},'FontSize',12,'interpreter','latex','Location','northwest');
title(lgd1,'ITAE','FontWeight','bold');
grid on
axis([0 tmax round(min(XX(:,end-2))-5) round(max(XX(:,end-2))+5)])
% xlabel('$t$ [s]','FontSize',12,'FontWeight','bold','interpreter','Latex')
ylabel('ITAE [$m$]','FontSize',13,'FontWeight','bold','interpreter','Latex')

subplot(313),plot(XX(:,end),XX(:,[end-4 end-1]), 'LineWidth', 1)
lgd2 = legend({'$Drone_{Actu}$','$Drone_{delayed}$'},'FontSize',12,'interpreter','latex','Location','northwest');
title(lgd2,'IASC','FontWeight','bold');
grid on
axis([0 tmax round(min(XX(:,end-1))-5) round(max(XX(:,end-1))+5)])
xlabel('$t$ [$s$]','FontSize',13,'FontWeight','bold','interpreter','Latex')
ylabel('IASC [$m/s^2$]','FontSize',13,'FontWeight','bold','interpreter','Latex')
%-----


% plot(XX(:,end),XX(:,end-4))
% grid on
% axis([0 40 -0.5 2.5])
% title({'\textbf{Sinal de Controle}'; 'Position'},'FontSize',14,'FontWeight','bold','Interpreter','latex')
% xlabel('$time$ [sec]','FontSize',12,'FontWeight','bold','interpreter','Latex')
% ylabel('$a$ [m/s^2]','FontSize',12,'FontWeight','bold','interpreter','Latex')