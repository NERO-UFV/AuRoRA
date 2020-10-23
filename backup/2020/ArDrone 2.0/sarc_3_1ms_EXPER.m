%% Comandos Iniciais

clear all; close all; clc;

disp('Início da simulação....')

plt = 0;

% Plotagem em tempo real
T = 0.01;

tmax = 180; % Tempo total da simulação
% T_plot = tmax - 0.5;
T_plot = 1;
t_aprend = 15; % usei 30 no trab

%% Parâmetros iniciais

% Ganhos Dinâmicos
Ku = diag([4.72 6.23 2.65 2.38]);

Kv = diag([0.28 0.53 2.58 1.52]);

Ks = diag([1.00 1.00 1.00 1.00]);

Ky = diag([3.10 3.10 3.10 3.10]);

Kd = diag([1.00 1.00 1.00 1.00]);

GAMA = diag([1  1  1  1  1  1  1  1]);
THETA = [1;  1;  1;  1;  1;  1;  1;  1]; %Funciona
% % % % THETA = [0;  0;  0;  0;  0;  0;  0;  0];
THETA_est = [0;  0;  0;  0;  0;  0;  0;  0];

dXr_max = diag([1.5 0.9375 0.2 0.2094]);

% UAV 1
dX_max = diag([1.60 1.10 0.35 0.25]);

Kc = dX_max - dXr_max;
Kc = 7*Kc;

X = [0; 0; 0; 0];
dX = [0; 0; 0; 0];
ddX = [0; 0; 0; 0];

Xb = [0; 0; 0; 0];
dXb = [0; 0; 0; 0];
ddXb = [0; 0; 0; 0];

U = [0; 0; 0; 0];
Uc_ant = [0; 0; 0; 0];
H = [];
H_THETA_est = [];

figure(1);
fig(1)= plot3(0,0,0,'ko','LineWidth',5);
axis([-4 4 -4 4 0 2]);
xlabel('Eixo x [m]','FontSize',12,'FontWeight','bold');
ylabel('Eixo y [m]','FontSize',12,'FontWeight','bold');
zlabel('Eixo z [m]','FontSize',12,'FontWeight','bold');
hold on;
grid on;
fig(2) = plot3(0,0,0,'r--','LineWidth',2);
fig(3) = plot3(0,0,0,'b','LineWidth',2);
legend([fig(1),fig(2),fig(3)],'Drone','X_{d}','X','Location','northwest');
drawnow;

% pause(10);

A = ArDrone(30);

figure(2)
axis([-4 4 -4 4 -0 4]);
xlabel('Eixo X [m]','FontSize',12,'FontWeight','bold');
ylabel('Eixo Y [m]','FontSize',12,'FontWeight','bold');
zlabel('Eixo Z [m]','FontSize',12,'FontWeight','bold');

A.mCADplot;
hold on;
grid on;
fig(1) = plot3(0,0,0,'r--','LineWidth',2);
fig(2) = plot3(0,0,0,'b','LineWidth',2);
legend([fig(1),fig(2)],'X_{d}','X','Location','northwest');
drawnow;
pause(1);

t = tic;
ta = tic;
tplot = tic;

%% Laço de Simulação



while toc(t) < tmax
    if toc(ta) > T
        ta = tic;
        
        % Referência de Trajetória
% % % %         p = 1.75;
% % % %         w = 1.00;
        
                p = 0.75;
        w = 1.00;
        
%         p = 3.75;
%         w = 0.3;
        
        Xr = [p*sin(w*toc(t))+0.1;
            1.25*p*cos(0.5*w*toc(t))-0.2;
            1+0.5*sin(w*toc(t));
            pi/6*sin(w*toc(t))];
        
        dXr = [w*p*cos(w*toc(t));
            -0.5*w*1.25*p*sin(0.5*w*toc(t));
            w*0.5*cos(w*toc(t));
            w*pi/6*cos(w*toc(t))];
        
        F = [ cos(Xb(4))   -sin(Xb(4))   0    0;
            sin(Xb(4))      cos(Xb(4))   0    0;
            0                 0        1    0;
            0                 0        0    1];
        
        % Modelo Dinâmico
        ddX = F*Ku*U - Kv*dX; % Global
        dX = dX + ddX*T;
        X = X + dX*T;
        
        ddXb = Ku*U - Kv*dXb; % Drone
        dXb = dXb + ddXb*T;
        Xb = Xb + dXb*T;
        
        Xtil = Xr - X; % Global
        
        % Controlador Cinemático
        Uc = F\(dXr + Kc*tanh(Ky*Xtil));
        dUc = (Uc - Uc_ant)/T;
        Uc_ant = Uc;
        
        % Controlador Adaptativo
        g11 = dUc(1) + Kd(1)*(Uc(1)-dXb(1));
        g15 = dXb(1);
        g22 = dUc(2) + Kd(2)*(Uc(2)-dXb(2));
        g26 = dXb(2);
        g33 = dUc(3) + Kd(3)*(Uc(3)-dXb(3));
        g37 = dXb(3);
        g44 = dUc(4) + Kd(4)*(Uc(4)-dXb(4));
        g48 = dXb(4);
        
        G = [ g11   0   0   0   g15   0   0   0 ;
            0   g22   0   0   0   g26   0   0  ;
            0   0   g33   0   0   0   g37   0  ;
            0   0   0   g44   0   0   0   g48 ];
        
        D = diag([THETA(1) THETA(2) THETA(3) THETA(4)]);
        SIGMA = dUc + Kd*(Uc - dXb);
        NETA = diag([dXb(1) dXb(2) dXb(3) dXb(4)])*...
            [THETA(5); THETA(6); THETA(7); THETA(8)];
        
        if toc(t) < t_aprend
            dTHETA_est = 0;
        else
            dTHETA_est = GAMA\G'*(Uc - dXb);
        end
        
        THETA_est = THETA_est + dTHETA_est*T;
        THETA_error = THETA_est - THETA;
        
        Ud = D*SIGMA + NETA + G*THETA_error;
        U = Ud;
              
        
        H = [H; [Xr' dXr' X' dX' Xtil' U' toc(t) THETA_est']];
        
% % % %         A.pSC.Ud = U;
% % % %         A.rSendControlSignals;
% % % %         
% % % %         H = [H; [Xr' A.pPos.X' toc(t)]];
% % % %       
% % % %         if toc(tplot) > T_plot
% % % %             tplot = tic;
% % % %             try %#ok<TRYNC>
% % % %                 delete (fig);
% % % %             end
% % % %             A.mCADplot
% % % %             fig(1) = plot3(H(:,1),H(:,2),H(:,3),'r--','LineWidth',2);
% % % %             fig(2) = plot3(H(:,5),H(:,6),H(:,7),'b','LineWidth',2);
% % % %             drawnow;
% % % %         end
        
        if toc(tplot) > T_plot
            tplot = tic;
            try %#ok<TRYNC>
                delete (fig);
            end
            fig(1)= plot3(X(1),X(2),X(3),'ko','LineWidth',5);
            fig(2) = plot3(H(:,1),H(:,2),H(:,3),'r--','LineWidth',2);
            fig(3) = plot3(H(:,9),H(:,10),H(:,11),'b','LineWidth',2);
            drawnow;
        end
    end
end

%% Plotagem dos Resultados

if plt == 1
    %% Posição durante o experimento
    figure(2);
    subplot(4,1,1);
    plot(H(:,25),H(:,1),'r--','LineWidth',2);
    hold on;
    plot(H(:,25),H(:,9),'b','LineWidth',2);
    ylabel('x [m]','FontSize',12,'FontWeight','bold');
    grid on;
    
    subplot(4,1,2);
    plot(H(:,25),H(:,2),'r--','LineWidth',2);
    hold on;
    plot(H(:,25),H(:,10),'b','LineWidth',2);
    ylabel('y [m]','FontSize',12,'FontWeight','bold');
    grid on;
    
    subplot(4,1,3);
    plot(H(:,25),H(:,3),'r--','LineWidth',2);
    hold on;
    plot(H(:,25),H(:,11),'b','LineWidth',2);
    ylabel('z [m]','FontSize',12,'FontWeight','bold');
    grid on;
    
    subplot(4,1,4);
    plot(H(:,25),H(:,4),'r--','LineWidth',2);
    hold on;
    plot(H(:,25),H(:,12),'b','LineWidth',2);
    ylabel('psi [rad]','FontSize',12,'FontWeight','bold');
    xlabel('Tempo [s]','FontSize',12,'FontWeight','bold');
    grid on;
    
    %% Erros lineares em x,y,z
    figure(3);
    subplot(2,1,1);
    plot(H(:,25),(H(:,1)-H(:,9)),'k','LineWidth',2);ylabel('Erros lineares em x,y,z','FontSize',12,'FontWeight','bold');
    hold on;
    grid on;
    plot(H(:,25),(H(:,2)-H(:,10)),'r','LineWidth',2);
    plot(H(:,25),(H(:,3)-H(:,11)),'b','LineWidth',2);
    legend('x_{error}','y_{error}','z_{error}');
    axis([0 tmax -6 5]);
    xlabel('Tempo [s]','FontSize',12,'FontWeight','bold');
    ylabel('Erros lineares em x,y,z','FontSize',12,'FontWeight','bold');
    
    subplot(2,1,2);
    plot(H(:,25),(H(:,4)-H(:,12)),'k','LineWidth',2);
    hold on;
    grid on;
    legend('psi_{error}');
    axis([0 tmax -1 1]);
    xlabel('Tempo [s]','FontSize',12,'FontWeight','bold');
    ylabel('Erro angular psi','FontSize',12,'FontWeight','bold');
    
    %% Velocidade linear dx
    
    figure(4);
    subplot(4,1,1);
    plot(H(:,25),H(:,5),'r--','LineWidth',2);
    hold on;
    plot(H(:,25),H(:,13),'b','LineWidth',2);
    ylabel('dx [m/s]','FontSize',12,'FontWeight','bold');
    grid on;
    
    subplot(4,1,2);
    plot(H(:,25),H(:,6),'r--','LineWidth',2);
    hold on;
    plot(H(:,25),H(:,14),'b','LineWidth',2);
    ylabel('dy [m/s]','FontSize',12,'FontWeight','bold');
    grid on;
    
    subplot(4,1,3);
    plot(H(:,25),H(:,7),'r--','LineWidth',2);
    hold on;
    plot(H(:,25),H(:,15),'b','LineWidth',2);
    ylabel('dz [m/s]','FontSize',12,'FontWeight','bold');
    grid on;
    
    subplot(4,1,4);
    plot(H(:,25),H(:,8),'r--','LineWidth',2);
    hold on;
    plot(H(:,25),H(:,16),'b','LineWidth',2);
    ylabel('dpsi [rad/s]','FontSize',12,'FontWeight','bold');
    xlabel('Tempo [s]','FontSize',12,'FontWeight','bold');
    grid on;

    
    %% THETA
    
    figure(8);
    plot(H(:,25),H(:,26),'LineWidth',2);
    hold on;
    grid on;
    plot(H(:,25),H(:,27),'LineWidth',2);
    plot(H(:,25),H(:,28),'LineWidth',2);
    plot(H(:,25),H(:,29),'LineWidth',2);
    plot(H(:,25),H(:,30),'LineWidth',2);
    plot(H(:,25),H(:,31),'LineWidth',2);
    plot(H(:,25),H(:,32),'LineWidth',2);
    plot(H(:,25),H(:,33),'LineWidth',2);
    legend('THETA_{1}','THETA_{2}','THETA_{3}','THETA_{4}',...
        'THETA_{5}','THETA_{6}','THETA_{7}','THETA_{8}','Location','northwest');
    xlabel('Tempo [s]','FontSize',12,'FontWeight','bold');
    axis([0 tmax -.5 1.5]);
    
    
end
disp(' ');
disp('Fim da simulação.');

