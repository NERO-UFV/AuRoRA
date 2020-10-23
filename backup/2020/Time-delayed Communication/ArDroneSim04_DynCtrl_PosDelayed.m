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
% Iniciar eta de controle - Decolar
tmax = 40; % Tempo Simulação em segundos
t = tic;
tc = tic;
tp = tic;

XX = [A.pPos.Xd' A.pPos.X' A.pSC.Ud' Ar.pPos.Xd' Ar.pPos.X' Ar.pSC.Ud' toc(t)];
% XX = [A.pPos.Xd' A.pPos.X' A.pSC.Ud' toc(t)]; % Teste drone atrasado
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

% Ar.pPos.Xd(1:3,1) = A.pPos.Xd(1:3,1) - 0.5*ones(3,1);
Ar.pPos.Xd(1:3,1) = A.pPos.Xd(1:3,1);
Ar.pPos.Xd(7:9,1) = zeros(3,1);


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
            A.pPos.X = XX(kk-idAtraso,13:24)';
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
        
        % Histórico de dados:
        XX = [XX; [A.pPos.Xd' A.pPos.X' A.pSC.Ud' Ar.pPos.Xd' Ar.pPos.X' Ar.pSC.Ud' toc(t)]];      
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

%%----- Teste Drone Atrasado:
figure

subplot(311),plot(XX(:,end),XX(:,[1 13 41]), 'LineWidth', 1)
legend({'$x_{d}$','$x_{delayed}$','$x_{Actu}$'},'FontSize',12,'interpreter','latex')
grid on
axis([0 tmax -2.5 0.5])
title({'\textbf{Drone}'; 'Position'},'FontSize',14,'FontWeight','bold','Interpreter','latex')
xlabel('$time$ [sec]','FontSize',12,'FontWeight','bold','interpreter','Latex')
ylabel('$x$ [m]','FontSize',12,'FontWeight','bold','interpreter','Latex')

subplot(312),plot(XX(:,end),XX(:,[2 14 42]), 'LineWidth', 1)
legend({'$y_{d}$','$y_{delayed}$','$y_{Actu}$'},'FontSize',12,'interpreter','latex')
grid on
axis([0 tmax -1.5 0.5])
xlabel('$time$ [sec]','FontSize',12,'FontWeight','bold','interpreter','Latex')
ylabel('$y$ [m]','FontSize',12,'FontWeight','bold','interpreter','Latex')

subplot(313),plot(XX(:,end),XX(:,[3 15 43]), 'LineWidth', 1)
legend({'$z_{d}$','$z_{delayed}$','$z_{Actu}$'},'FontSize',12,'interpreter','latex')
grid on
axis([0 tmax -0.5 2.5])
xlabel('$time$ [sec]','FontSize',12,'FontWeight','bold','interpreter','Latex')
ylabel('$z$ [m]','FontSize',12,'FontWeight','bold','interpreter','Latex')
%-----

% plot(XX(:,end),XX(:,end-1))
% grid on
% axis([0 40 -0.5 2.5])
% title({'\textbf{Aceleração Desejada}'; 'Position'},'FontSize',14,'FontWeight','bold','Interpreter','latex')
% xlabel('$time$ [sec]','FontSize',12,'FontWeight','bold','interpreter','Latex')
% ylabel('$a$ [m/s^2]','FontSize',12,'FontWeight','bold','interpreter','Latex')